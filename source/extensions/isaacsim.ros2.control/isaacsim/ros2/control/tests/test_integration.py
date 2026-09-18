# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Full-run integration tests: a real in-memory articulation, a real in-process
ControllerManager (no mocks), physics stepping, and ROS 2 assertions.

Unlike test_manager / test_og_node (which mock the backend), these exercise the
whole pipeline: URDF synthesis -> backend setup_cm -> controller_manager ->
IsaacSimSystem read/write against the active physics backend. They require the packaged ROS 2
runtime and run only under the Kit test runner.
"""

from __future__ import annotations

import json
import os
import tempfile

import omni
import omni.graph.core as og
import omni.kit.app
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
from usdrt import Sdf as UsdrtSdf

# Shared controller config: a joint_state_broadcaster plus position, velocity, and
# effort joint_trajectory_controllers (one per command interface on joint1).
_CONTROLLER_YAML = """
controller_manager:
  ros__parameters:
    update_rate: 60
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    position_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    velocity_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    effort_controller:
      type: joint_trajectory_controller/JointTrajectoryController

position_controller:
  ros__parameters:
    joints:
      - joint1
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

velocity_controller:
  ros__parameters:
    joints:
      - joint1
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity
    gains:
      joint1:
        p: 20.0
        i: 0.0
        d: 0.0

effort_controller:
  ros__parameters:
    joints:
      - joint1
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    gains:
      joint1:
        p: 60.0
        i: 0.0
        d: 10.0
"""

_JOINT = "joint1"

# Force-torque broadcaster on a sensor bound to link1 via the overlay URDF below.
_FT_CONTROLLER_YAML = """
controller_manager:
  ros__parameters:
    update_rate: 60
    ft_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

ft_broadcaster:
  ros__parameters:
    sensor_name: ft_sensor
    frame_id: link1
"""

# Overlay declares an FT <sensor> (force.*/torque.* interfaces + required link).
# The <ros2_control> name need not match; _splice_sensor_overlay falls back to the
# first synthesized block.
_FT_OVERLAY_URDF = """<robot name="arm">
  <ros2_control name="arm">
    <sensor name="ft_sensor">
      <state_interface name="force.x"/>
      <state_interface name="force.y"/>
      <state_interface name="force.z"/>
      <state_interface name="torque.x"/>
      <state_interface name="torque.y"/>
      <state_interface name="torque.z"/>
      <param name="link">link1</param>
    </sensor>
  </ros2_control>
</robot>
"""

_BAD_FT_LINK_OVERLAY_URDF = """<robot name="arm">
  <ros2_control name="arm">
    <sensor name="ft_sensor">
      <state_interface name="force.x"/>
      <state_interface name="force.y"/>
      <state_interface name="force.z"/>
      <state_interface name="torque.x"/>
      <state_interface name="torque.y"/>
      <state_interface name="torque.z"/>
      <param name="link">missing_link</param>
    </sensor>
  </ros2_control>
</robot>
"""

# IMU broadcaster on a sensor bound to an IsaacImuSensor prim named "imu_sensor".
_IMU_CONTROLLER_YAML = """
controller_manager:
  ros__parameters:
    update_rate: 60
    imu_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

imu_broadcaster:
  ros__parameters:
    sensor_name: imu_sensor
    frame_id: link1
"""

# Overlay declares an IMU <sensor> with the 10 interfaces IMUSensorBroadcaster needs.
# No <param name="prim_path">: IsaacSimSystem resolves it by name to the IsaacImuSensor prim.
_IMU_OVERLAY_URDF = """<robot name="arm">
  <ros2_control name="arm">
    <sensor name="imu_sensor">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>
  </ros2_control>
</robot>
"""


def _build_arm(stage, root: str = "/World/Arm") -> None:
    """Fixed-base 1-DOF arm: base (fixed to world) -> revolute position drive -> link1.

    Explicit mass/inertia so physics backends simulate it without collision geometry."""
    UsdGeom.Xform.Define(stage, "/World")
    arm = UsdGeom.Xform.Define(stage, root).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(arm)

    base = UsdGeom.Xform.Define(stage, f"{root}/base").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(base)
    UsdPhysics.MassAPI.Apply(base).CreateMassAttr(1.0)

    # Fixed joint to world makes the articulation fixed-base.
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{root}/base_fixed")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])

    link1 = UsdGeom.Xform.Define(stage, f"{root}/link1").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(link1)
    mass1 = UsdPhysics.MassAPI.Apply(link1)
    mass1.CreateMassAttr(1.0)
    mass1.CreateDiagonalInertiaAttr(Gf.Vec3f(0.1, 0.1, 0.1))

    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/{_JOINT}")
    joint.CreateBody0Rel().SetTargets([base.GetPath()])
    joint.CreateBody1Rel().SetTargets([link1.GetPath()])
    joint.CreateAxisAttr("Z")
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(2000.0)  # stiffness>0 -> classified as a position command
    drive.CreateDampingAttr(200.0)
    drive.CreateTargetPositionAttr(0.0)


def _enable_gpu_physics(stage, scene_path: str = "/World/PhysicsScene") -> None:
    """Request PhysX GPU dynamics and GPU broadphase for the test scene."""
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableGPUDynamicsAttr(True)
    physx_scene.CreateBroadphaseTypeAttr("GPU")


class TestRos2ControlFullRun(ROS2TestCase):
    async def setUp(self):
        await super().setUp()
        self._stage = omni.usd.get_context().get_stage()
        _build_arm(self._stage)
        self._yaml_path = self._write_yaml(_CONTROLLER_YAML)
        self._clock_graph = None

        import rclpy

        self._rclpy = rclpy
        self._node = self.create_node("rc_integration_test")
        self._spin = lambda: rclpy.spin_once(self._node, timeout_sec=0)

    async def tearDown(self):
        try:
            from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend

            backend.teardown_all()
        finally:
            await super().tearDown()

    def _write_yaml(self, text: str) -> str:
        fd, path = tempfile.mkstemp(suffix=".yaml")
        os.close(fd)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        self.addCleanup(os.remove, path)
        return path

    def _write_overlay(self, text: str) -> str:
        fd, path = tempfile.mkstemp(suffix=".urdf")
        os.close(fd)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        self.addCleanup(os.remove, path)
        return path

    def _normalize_namespace(self, ns: str = "") -> str:
        return ns.strip("/")

    def _topic(self, ns: str, name: str) -> str:
        ns = self._normalize_namespace(ns)
        name = name.lstrip("/")
        return f"/{ns}/{name}" if ns else f"/{name}"

    def _controller_manager(self, ns: str = "") -> str:
        return self._topic(ns, "controller_manager")

    def _ensure_clock_graph(self) -> None:
        if self._clock_graph is not None:
            return
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/World/ROS2Clock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            },
        )
        self._clock_graph = graph

    async def _wait_backend_ready(self) -> None:
        from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend

        ready = await self.simulate_until_condition(
            lambda: backend.is_ready(), max_frames=600, per_frame_callback=self._spin
        )
        self.assertTrue(ready, "ros2.control backend never became ready")

    async def _play_and_setup(self, prim_path: str = "/World/Arm", **kwargs) -> int:
        """Initialize physics, configure the CM while paused, then resume simulation."""
        from isaacsim.ros2.control import Ros2ControlManager

        if kwargs.get("use_sim_time", True):
            self._ensure_clock_graph()
        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await self._wait_backend_ready()
        # Tensor articulation view construction may copy backend state; doing that
        # while simulation is actively running can log a physics error and fail
        # the extension test wrapper even though controller setup succeeds.
        self._timeline.pause()
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        try:
            rc = Ros2ControlManager.setup(prim_path, self._yaml_path, **kwargs)
            if rc == 0:
                await self._wait_controller_manager_services(kwargs.get("namespace", ""))
            return rc
        finally:
            self._timeline.play()

    async def _wait_controller_manager_services(self, ns: str = "") -> None:
        from controller_manager_msgs.srv import ListControllers

        cm = self._controller_manager(ns)
        client = self._node.create_client(ListControllers, f"{cm}/list_controllers")
        self.addCleanup(self._node.destroy_client, client)
        ready = await self.simulate_until_condition(
            lambda: client.service_is_ready(), max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(ready, f"controller_manager services for {cm!r} not available")

    async def _call_service(self, srv_type, srv_name, request, max_frames: int = 900):
        client = self._node.create_client(srv_type, srv_name)
        self.addCleanup(self._node.destroy_client, client)
        ready = await self.simulate_until_condition(
            lambda: client.service_is_ready(), max_frames=max_frames, per_frame_callback=self._spin
        )
        self.assertTrue(ready, f"service {srv_name!r} not available")
        future = client.call_async(request)
        done = await self.simulate_until_condition(
            lambda: future.done(), max_frames=max_frames, per_frame_callback=self._spin
        )
        self.assertTrue(done, f"service {srv_name!r} did not respond")
        return future.result()

    async def _activate(self, controllers: list[str], ns: str = "") -> None:
        """Load, configure, and activate each controller through the CM services."""
        from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController

        cm = self._controller_manager(ns)
        for name in controllers:
            res = await self._call_service(LoadController, f"{cm}/load_controller", LoadController.Request(name=name))
            self.assertTrue(res.ok, f"load_controller({name}) failed")
            res = await self._call_service(
                ConfigureController, f"{cm}/configure_controller", ConfigureController.Request(name=name)
            )
            self.assertTrue(res.ok, f"configure_controller({name}) failed")

        req = SwitchController.Request(activate_controllers=controllers, strictness=SwitchController.Request.STRICT)
        res = await self._call_service(SwitchController, f"{cm}/switch_controller", req)
        self.assertTrue(res.ok, f"switch_controller(activate={controllers}) failed")

    async def _load_configure(self, controllers: list[str], ns: str = "") -> None:
        """Load + configure (but do not activate) each controller."""
        from controller_manager_msgs.srv import ConfigureController, LoadController

        cm = self._controller_manager(ns)
        for name in controllers:
            res = await self._call_service(LoadController, f"{cm}/load_controller", LoadController.Request(name=name))
            self.assertTrue(res.ok, f"load_controller({name}) failed")
            res = await self._call_service(
                ConfigureController, f"{cm}/configure_controller", ConfigureController.Request(name=name)
            )
            self.assertTrue(res.ok, f"configure_controller({name}) failed")

    async def _switch_controllers(self, activate: list[str], deactivate: list[str], ns: str = "") -> None:
        from controller_manager_msgs.srv import SwitchController

        req = SwitchController.Request(
            activate_controllers=activate,
            deactivate_controllers=deactivate,
            strictness=SwitchController.Request.STRICT,
        )
        res = await self._call_service(SwitchController, f"{self._controller_manager(ns)}/switch_controller", req)
        self.assertTrue(res.ok, f"switch_controller(activate={activate}, deactivate={deactivate}) failed")

    async def _publish_trajectory_once(self, publisher, message, max_frames: int = 900) -> None:
        """Wait for controller discovery, then publish one trajectory without restarting it."""
        matched = await self.simulate_until_condition(
            lambda: publisher.get_subscription_count() > 0,
            max_frames=max_frames,
            per_frame_callback=self._spin,
        )
        self.assertTrue(matched, "trajectory publisher did not match a controller subscriber")
        publisher.publish(message)
        self._spin()

    def _latched_sub(self, msg_type, topic, callback):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        return self.create_subscription(self._node, msg_type, topic, callback, qos)

    async def _wait_robot_description(self, ns: str = "") -> str:
        from std_msgs.msg import String

        topic = self._topic(ns, "robot_description")
        received = {}
        self._latched_sub(String, topic, lambda m: received.update(urdf=m.data))
        got = await self.simulate_until_condition(
            lambda: "urdf" in received, max_frames=600, per_frame_callback=self._spin
        )
        self.assertTrue(got, f"no latched {topic}")
        return received["urdf"]

    async def _wait_joint_states(self, ns: str = "", min_count: int = 3):
        from sensor_msgs.msg import JointState

        msgs = []
        self.create_subscription(self._node, JointState, self._topic(ns, "joint_states"), lambda m: msgs.append(m))
        got = await self.simulate_until_condition(
            lambda: len(msgs) >= min_count, max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(got, f"{self._topic(ns, 'joint_states')} did not publish")
        return msgs

    async def _setup_many_while_paused(self, setups: list[tuple[str, str, str]]) -> None:
        from isaacsim.ros2.control import Ros2ControlManager

        self._ensure_clock_graph()
        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await self._wait_backend_ready()
        self._timeline.pause()
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        try:
            for prim_path, ns, yaml_path in setups:
                rc = Ros2ControlManager.setup(prim_path, yaml_path, namespace=ns)
                self.assertEqual(rc, 0, f"setup_cm should succeed for {prim_path} ns={ns!r}")
            for _, ns, _ in setups:
                await self._wait_controller_manager_services(ns)
        finally:
            self._timeline.play()

    def _create_ogn_manager_graph(self, prim_path: str, ns: str):
        graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2ControlManager", "isaacsim.ros2.control.ROS2ControlManager"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ROS2ControlManager.inputs:targetPrim", [UsdrtSdf.Path(prim_path)]),
                    ("ROS2ControlManager.inputs:controllerConfig", self._yaml_path),
                    ("ROS2ControlManager.inputs:namespace", ns),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2ControlManager.inputs:execIn"),
                ],
            },
        )
        node = og.Controller.node("/ActionGraph/ROS2ControlManager")
        self.assertTrue(node.is_valid(), "ROS2ControlManager OGN node was not created")
        self.assertIsNotNone(og.Controller.attribute("/ActionGraph/ROS2ControlManager.inputs:controllerConfig"))
        return graph

    async def _send_follow_joint_trajectory_goal(self, target: float, ns: str = ""):
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectoryPoint

        action_name = self._topic(ns, "position_controller/follow_joint_trajectory")
        action_client = ActionClient(self._node, FollowJointTrajectory, action_name)
        self.addCleanup(action_client.destroy)
        ready = await self.simulate_until_condition(
            lambda: action_client.server_is_ready(), max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(ready, f"action server {action_name!r} not available")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]

        goal_future = action_client.send_goal_async(goal)
        sent = await self.simulate_until_condition(
            lambda: goal_future.done(), max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(sent, f"action goal to {action_name!r} did not receive a response")
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted, f"action goal to {action_name!r} was rejected")

        result_future = goal_handle.get_result_async()
        finished = await self.simulate_until_condition(
            lambda: result_future.done(), max_frames=1800, per_frame_callback=self._spin
        )
        self.assertTrue(finished, f"action goal to {action_name!r} did not finish")
        response = result_future.result()
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        return response.result

    # ---- tests ---------------------------------------------------------------

    async def test_bringup_publishes_robot_description(self):
        rc = await self._play_and_setup()
        self.assertEqual(rc, 0, "setup_cm should succeed for a valid articulation")

        urdf = await self._wait_robot_description()
        self.assertIn("isaacsim_ros2_control/IsaacSimSystem", urdf)
        self.assertIn(_JOINT, urdf)
        self.assertIn("<ros2_control", urdf)

    async def test_ogn_bringup_honors_namespace_and_exports_interfaces(self):
        from controller_manager_msgs.srv import ListHardwareInterfaces

        ns = "arm1"
        graph = self._create_ogn_manager_graph("/World/Arm", ns)
        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        await self._wait_backend_ready()
        self._timeline.pause()
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(graph)
        await self._wait_controller_manager_services(ns)

        urdf = await self._wait_robot_description(ns)
        self.assertIn("isaacsim_ros2_control/IsaacSimSystem", urdf)
        self.assertIn(_JOINT, urdf)
        self.assertIn("<ros2_control", urdf)

        res = await self._call_service(
            ListHardwareInterfaces,
            f"{self._controller_manager(ns)}/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
        )
        command_names = {iface.name for iface in res.command_interfaces}
        state_names = {iface.name for iface in res.state_interfaces}
        # Driven joints export the full command-interface superset.
        for kind in ("position", "velocity", "effort"):
            self.assertIn(f"{_JOINT}/{kind}", command_names)
            self.assertIn(f"{_JOINT}/{kind}", state_names)

    async def test_two_live_managers_have_isolated_namespaces(self):
        _build_arm(self._stage, "/World/ArmA")
        _build_arm(self._stage, "/World/ArmB")
        yaml_path = self._write_yaml("""
/arm_a/controller_manager:
  ros__parameters:
    update_rate: 60
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
/arm_b/controller_manager:
  ros__parameters:
    update_rate: 60
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
""")
        setups = [("/World/ArmA", "arm_a", yaml_path), ("/World/ArmB", "/arm_b", yaml_path)]

        await self._setup_many_while_paused(setups)

        for _, ns, _ in setups:
            urdf = await self._wait_robot_description(ns)
            self.assertIn("isaacsim_ros2_control/IsaacSimSystem", urdf)
            self.assertIn(_JOINT, urdf)
            await self._activate(["joint_state_broadcaster"], ns=ns)

        msgs_a = await self._wait_joint_states("arm_a")
        msgs_b = await self._wait_joint_states("/arm_b")
        self.assertIn(_JOINT, list(msgs_a[-1].name))
        self.assertIn(_JOINT, list(msgs_b[-1].name))

    async def test_joint_state_broadcaster_publishes_states(self):
        from sensor_msgs.msg import JointState

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)
        await self._activate(["joint_state_broadcaster"])

        msgs = []
        self.create_subscription(self._node, JointState, "/joint_states", lambda m: msgs.append(m))
        got = await self.simulate_until_condition(lambda: len(msgs) >= 3, max_frames=900, per_frame_callback=self._spin)
        self.assertTrue(got, "joint_state_broadcaster did not publish")
        last = msgs[-1]
        self.assertIn(_JOINT, list(last.name))
        idx = list(last.name).index(_JOINT)
        # State read from the active physics backend must be finite (exercises read() + DOF mapping).
        self.assertTrue(abs(last.position[idx]) < 1e6)

    async def test_position_command_drives_joint(self):
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)
        await self._activate(["joint_state_broadcaster", "position_controller"])

        latest = {}

        def on_state(m):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)
        cmd_pub = self.create_publisher(self._node, JointTrajectory, "/position_controller/joint_trajectory")

        target = 0.5
        msg = JointTrajectory()
        msg.joint_names = [_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200_000_000
        msg.points = [point]

        # Repeated publication restarts trajectory interpolation.
        await self._publish_trajectory_once(cmd_pub, msg)
        converged = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - target) < 0.1,
            max_frames=1200,
            per_frame_callback=self._spin,
        )
        self.assertTrue(converged, f"joint did not reach {target} (last={latest.get('pos')})")

    async def test_follow_joint_trajectory_action_reaches_target(self):
        from sensor_msgs.msg import JointState

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)
        await self._activate(["joint_state_broadcaster", "position_controller"])

        latest = {}

        def on_state(m: JointState):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)
        target = 0.4
        await self._send_follow_joint_trajectory_goal(target)

        converged = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - target) < 0.1,
            max_frames=600,
            per_frame_callback=self._spin,
        )
        self.assertTrue(converged, f"joint did not reach {target} after action goal (last={latest.get('pos')})")

    async def test_bridge_clock_advances_and_stop_allows_resetup(self):
        from rosgraph_msgs.msg import Clock

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)

        stamps = []
        self.create_subscription(
            self._node, Clock, "/clock", lambda m: stamps.append(m.clock.sec + m.clock.nanosec * 1e-9)
        )
        got = await self.simulate_until_condition(
            lambda: len(stamps) >= 5 and stamps[-1] > stamps[0], max_frames=600, per_frame_callback=self._spin
        )
        self.assertTrue(got, f"/clock did not advance (stamps={stamps[:5]})")

        # Stop tears the CM down (timeline subscription -> destroyAll); a fresh
        # setup must then succeed rather than hit the already-registered error.
        self._timeline.stop()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        rc2 = await self._play_and_setup()
        self.assertEqual(rc2, 0, "re-setup after Stop should succeed (registry cleared)")

    async def test_update_rate_decimates_against_physics_and_tracks_sim_time(self):
        """Controller updates are rate-limited by physics steps and stamped in sim time."""
        from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        update_rate = 20.0
        self._yaml_path = self._write_yaml(f"""
controller_manager:
  ros__parameters:
    update_rate: {int(update_rate)}
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
""")
        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)
        await self._activate(["joint_state_broadcaster"])

        # Skip startup transients before measuring update cadence.
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        clocks = []
        stamps = []
        self.create_subscription(
            self._node,
            Clock,
            "/clock",
            lambda m: clocks.append(m.clock.sec + m.clock.nanosec * 1e-9),
        )
        self.create_subscription(
            self._node,
            JointState,
            "/joint_states",
            lambda m: stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
        )

        backend.reset_profiling()
        backend.set_profiling_enabled(True)
        try:
            for _ in range(180):
                await omni.kit.app.get_app().next_update_async()
                self._spin()
        finally:
            backend.set_profiling_enabled(False)

        prof = json.loads(backend.get_profiling_json())
        steps = int(prof["physics_step_calls"])
        updated = int(prof["updated_instances"])
        self.assertGreater(steps, 0, "profiling captured no physics steps; the CM is not ticking")
        self.assertGreater(updated, 0, "profiling captured no controller manager updates")

        # No more than one profiled instance update per physics step, and no
        # accidental 1:1 update when update_rate is below the physics rate.
        self.assertLessEqual(
            updated, steps, f"updated_instances {updated} exceeds physics_step_calls {steps}: CM is self-driving"
        )
        self.assertLess(updated, steps, f"updated_instances {updated} did not decimate physics steps {steps}")

        # Stop publishers and drain both subscription queues before comparing their
        # latest timestamps. A single `spin_once()` processes only one ready callback,
        # so the higher-rate `/clock` queue can otherwise lag `/joint_states` on CI.
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        idle_spins = 0
        for _ in range(400):
            message_counts = (len(clocks), len(stamps))
            self._rclpy.spin_once(self._node, timeout_sec=0.01)
            if message_counts == (len(clocks), len(stamps)):
                idle_spins += 1
                if idle_spins >= 20:
                    break
            else:
                idle_spins = 0
        self.assertGreaterEqual(idle_spins, 20, "ROS subscription queues did not become idle")

        # Sim-time stamps: /joint_states timestamps should advance at update_rate, not
        # at wall-clock or per-frame cadence.
        self.assertGreaterEqual(len(clocks), 3, "no /clock stamps captured")
        self.assertGreaterEqual(len(stamps), 3, "no /joint_states stamps captured")
        clock_min = min(clocks)
        clock_max = max(clocks)
        clock_span = clock_max - clock_min
        stamp_span = stamps[-1] - stamps[0]
        self.assertGreater(clock_span, 0.0, "/clock stamps did not advance")
        self.assertGreater(stamp_span, 0.0, "/joint_states stamps did not advance")
        self.assertGreaterEqual(stamps[0], clock_min - 0.1, "/joint_states stamps are not on the /clock timeline")
        self.assertLessEqual(stamps[-1], clock_max + 0.1, "/joint_states stamps are not on the /clock timeline")
        observed_hz = (len(stamps) - 1) / stamp_span
        self.assertLessEqual(
            abs(observed_hz - update_rate),
            max(2.0, 0.1 * update_rate),
            f"/joint_states cadence {observed_hz:.1f}Hz drifts from update_rate {update_rate}Hz",
        )
        expected = round(clock_span * update_rate)
        self.assertLessEqual(
            abs(updated - expected),
            max(3, round(0.1 * expected)),
            f"cadence off: updated={updated}, expected~={expected} from /clock span {clock_span:.3f}s",
        )

    async def test_requested_gpu_configuration_reads_state_and_drives_joint(self):
        """Exercise the requested PhysX GPU-dynamics configuration: read() must
        yield finite joint states and write() must apply the position command."""
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        # CUDA availability does not prove that PhysX avoided CPU fallback.
        try:
            import warp as wp
        except Exception as exc:
            self.skipTest(f"cannot prove CUDA availability: warp import failed: {exc}")

        if not wp.is_cuda_available() or wp.get_cuda_device_count() == 0:
            self.skipTest("no CUDA device available; the GPU physics pipeline test requires a GPU")
        try:
            wp.get_device("cuda:0")
        except Exception as exc:
            self.skipTest(f"cannot prove CUDA availability: cuda:0 unavailable: {exc}")

        _enable_gpu_physics(self._stage)
        rc = await self._play_and_setup()
        self.assertEqual(rc, 0, "setup_cm should succeed with GPU dynamics requested")
        await self._activate(["joint_state_broadcaster", "position_controller"])

        latest = {}

        def on_state(m):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)
        cmd_pub = self.create_publisher(self._node, JointTrajectory, "/position_controller/joint_trajectory")

        target = 0.5
        msg = JointTrajectory()
        msg.joint_names = [_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200_000_000
        msg.points = [point]

        got_state = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"]) < 1e6, max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(got_state, "no finite joint state read with GPU dynamics requested")

        await self._publish_trajectory_once(cmd_pub, msg)
        converged = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - target) < 0.1,
            max_frames=1200,
            per_frame_callback=self._spin,
        )
        self.assertTrue(
            converged, f"joint did not reach {target} with GPU dynamics requested (last={latest.get('pos')})"
        )

    async def test_command_mode_switch_position_to_velocity_and_back(self):
        """A joint imported as a position drive is velocity-controlled after a runtime
        controller switch, then switched back to position. The joint exports
        position+velocity+effort (superset), so no re-import is needed."""
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)

        latest = {}

        def on_state(m):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)

        def traj_to(topic, target):
            pub = self.create_publisher(self._node, JointTrajectory, topic)
            msg = JointTrajectory()
            msg.joint_names = [_JOINT]
            pt = JointTrajectoryPoint()
            pt.positions = [target]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = 300_000_000
            msg.points = [pt]
            return pub, msg

        # Hold a position first (position command interface), so the start pose is defined.
        await self._activate(["joint_state_broadcaster", "position_controller"])
        pos_pub, pos_msg = traj_to("/position_controller/joint_trajectory", 0.2)
        await self._publish_trajectory_once(pos_pub, pos_msg)
        held = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.2) < 0.1,
            max_frames=1200,
            per_frame_callback=self._spin,
        )
        self.assertTrue(held, f"joint did not reach the 0.2 start pose (last={latest.get('pos')})")

        # Switch the same joint to the velocity command interface and drive it to a
        # new target. The pre-step boundary zeroes the drive stiffness, so the leftover
        # 0.2 position target no longer holds the joint.
        await self._load_configure(["velocity_controller"])
        await self._switch_controllers(activate=["velocity_controller"], deactivate=["position_controller"])

        vel_pub, vel_msg = traj_to("/velocity_controller/joint_trajectory", 0.6)
        vel_msg.points[0].time_from_start.sec = 1
        vel_msg.points[0].time_from_start.nanosec = 0
        await self._publish_trajectory_once(vel_pub, vel_msg)
        moved = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.6) < 0.1,
            max_frames=1800,
            per_frame_callback=self._spin,
        )
        self.assertTrue(
            moved, f"joint did not reach 0.6 via the velocity interface after mode switch (last={latest.get('pos')})"
        )

        # Switch back to position control and drive to a third target.
        await self._switch_controllers(activate=["position_controller"], deactivate=["velocity_controller"])
        pos_pub, pos_msg = traj_to("/position_controller/joint_trajectory", 0.3)
        await self._publish_trajectory_once(pos_pub, pos_msg)
        back = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.3) < 0.1,
            max_frames=1800,
            per_frame_callback=self._spin,
        )
        self.assertTrue(back, f"joint did not reach 0.3 after switching back to position (last={latest.get('pos')})")

    async def test_effort_mode_restores_position_gains_after_reconfigure(self):
        """The arm is a stiff position drive (stiffness 2000, damping 200). Effort
        control would be impossible if those gains stayed live; the pre-step boundary
        zeroes them on the switch. Reconfiguration must restore the original gains."""
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)

        latest = {}

        def on_state(m):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)

        # Before: the position interface holds the stiff drive at a target.
        await self._activate(["joint_state_broadcaster", "position_controller"])
        pos_pub = self.create_publisher(self._node, JointTrajectory, "/position_controller/joint_trajectory")
        hold = JointTrajectory()
        hold.joint_names = [_JOINT]
        hold_pt = JointTrajectoryPoint()
        hold_pt.positions = [0.2]
        hold_pt.time_from_start.nanosec = 300_000_000
        hold.points = [hold_pt]
        await self._publish_trajectory_once(pos_pub, hold)
        held = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.2) < 0.1,
            max_frames=1200,
            per_frame_callback=self._spin,
        )
        self.assertTrue(held, f"position interface did not hold 0.2 (last={latest.get('pos')})")

        # After: switch to effort; the drive gains are zeroed so torque can move it.
        await self._load_configure(["effort_controller"])
        await self._switch_controllers(activate=["effort_controller"], deactivate=["position_controller"])
        eff_pub = self.create_publisher(self._node, JointTrajectory, "/effort_controller/joint_trajectory")
        msg = JointTrajectory()
        msg.joint_names = [_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [0.5]
        pt.time_from_start.sec = 1
        msg.points = [pt]
        await self._publish_trajectory_once(eff_pub, msg)
        moved = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.5) < 0.15,
            max_frames=1800,
            per_frame_callback=self._spin,
        )
        self.assertTrue(moved, f"effort command did not move the stiff joint to 0.5 (last={latest.get('pos')})")

        self._timeline.stop()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)
        await self._activate(["joint_state_broadcaster", "position_controller"])

        restore_pub = self.create_publisher(self._node, JointTrajectory, "/position_controller/joint_trajectory")
        restore = JointTrajectory()
        restore.joint_names = [_JOINT]
        restore_pt = JointTrajectoryPoint()
        restore_pt.positions = [0.1]
        restore_pt.time_from_start.nanosec = 300_000_000
        restore.points = [restore_pt]
        await self._publish_trajectory_once(restore_pub, restore)
        restored = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.1) < 0.1,
            max_frames=1800,
            per_frame_callback=self._spin,
        )
        self.assertTrue(restored, f"position gains were not restored after reconfigure (last={latest.get('pos')})")

    async def test_dual_command_interface_claim_is_rejected(self):
        """Activating two command interfaces on one joint (position + velocity) must
        fail: prepare_command_mode_switch allows only one interface per joint."""
        from controller_manager_msgs.srv import SwitchController

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0)

        await self._load_configure(["position_controller", "velocity_controller"])
        req = SwitchController.Request(
            activate_controllers=["position_controller", "velocity_controller"],
            strictness=SwitchController.Request.STRICT,
        )
        res = await self._call_service(SwitchController, f"{self._controller_manager()}/switch_controller", req)
        self.assertFalse(res.ok, "activating two command interfaces on one joint should be rejected")

    async def test_invalid_force_torque_sensor_link_fails_setup_and_allows_retry(self):
        """A bad FT sensor link should fail hardware configure, and the failed setup
        must not leave a registered manager that blocks a later valid setup."""
        bad_overlay = self._write_overlay(_BAD_FT_LINK_OVERLAY_URDF)

        with self.assertRaisesRegex(RuntimeError, "ControllerManager init failed"):
            await self._play_and_setup(urdf_path=bad_overlay)

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        rc = await self._play_and_setup()
        self.assertEqual(rc, 0, "valid setup after failed configure should succeed")
        await self._activate(["joint_state_broadcaster"])
        msgs = await self._wait_joint_states()
        self.assertIn(_JOINT, list(msgs[-1].name))

    async def test_ros2_control_on_active_physics_engine(self):
        """Engine-agnostic bring-up: ros2_control reads joint state and drives a
        position command on whichever physics engine is active. The unnamed test
        block runs this under PhysX; the 'newton' block runs it under Newton."""
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        try:
            from isaacsim.core.simulation_manager import SimulationManager

            engine = SimulationManager.get_active_physics_engine()
        except Exception:
            engine = "physx"
        rc = await self._play_and_setup()
        self.assertEqual(rc, 0, f"setup_cm should succeed on the {engine} engine")
        await self._activate(["joint_state_broadcaster", "position_controller"])

        latest = {}

        def on_state(m):
            if _JOINT in m.name:
                latest["pos"] = m.position[list(m.name).index(_JOINT)]

        self.create_subscription(self._node, JointState, "/joint_states", on_state)

        # read() yields finite joint state on this engine.
        got = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"]) < 1e6, max_frames=900, per_frame_callback=self._spin
        )
        self.assertTrue(got, f"no finite joint state on the {engine} engine")

        # write() applies a position command on this engine.
        cmd_pub = self.create_publisher(self._node, JointTrajectory, "/position_controller/joint_trajectory")
        msg = JointTrajectory()
        msg.joint_names = [_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [0.4]
        pt.time_from_start.nanosec = 300_000_000
        msg.points = [pt]
        await self._publish_trajectory_once(cmd_pub, msg)
        converged = await self.simulate_until_condition(
            lambda: "pos" in latest and abs(latest["pos"] - 0.4) < 0.15,
            max_frames=1500,
            per_frame_callback=self._spin,
        )
        self.assertTrue(
            converged, f"position command did not drive the joint on the {engine} engine (last={latest.get('pos')})"
        )

    async def test_force_torque_sensor_exports_available_interfaces(self):
        """An FT <sensor> with a valid <param name="link"> is classified as
        force-torque, its link resolved via findLinkIndex, and its 6 force.*/torque.*
        state interfaces exported and made available through the real CM. Reaching
        'available' proves on_configure (including link resolution) succeeded; a bad
        or missing link would error out and the interfaces would stay unavailable."""
        from controller_manager_msgs.srv import ListHardwareInterfaces

        self._yaml_path = self._write_yaml(_FT_CONTROLLER_YAML)
        overlay = self._write_overlay(_FT_OVERLAY_URDF)
        rc = await self._play_and_setup(urdf_path=overlay)
        self.assertEqual(rc, 0, "setup_cm should succeed with an FT sensor overlay")

        res = await self._call_service(
            ListHardwareInterfaces,
            f"{self._controller_manager()}/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
        )
        available = {i.name for i in res.state_interfaces if i.is_available}
        for comp in ("force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"):
            self.assertIn(f"ft_sensor/{comp}", available, f"ft_sensor/{comp} not exported/available")

    async def test_force_torque_sensor_broadcaster_publishes_wrench(self):
        """End-to-end: ForceTorqueSensorBroadcaster publishes a finite WrenchStamped
        whose force carries link1's gravity load, proving read() fills the interfaces
        from getLinkIncomingJointForce."""
        import math

        from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
        from geometry_msgs.msg import WrenchStamped

        self._yaml_path = self._write_yaml(_FT_CONTROLLER_YAML)
        overlay = self._write_overlay(_FT_OVERLAY_URDF)
        rc = await self._play_and_setup(urdf_path=overlay)
        self.assertEqual(rc, 0, "setup_cm should succeed with an FT sensor overlay")

        cm = self._controller_manager()
        loaded = await self._call_service(
            LoadController, f"{cm}/load_controller", LoadController.Request(name="ft_broadcaster")
        )
        self.assertTrue(loaded.ok, "load_controller(ft_broadcaster) failed")
        cfg = await self._call_service(
            ConfigureController, f"{cm}/configure_controller", ConfigureController.Request(name="ft_broadcaster")
        )
        self.assertTrue(cfg.ok, "configure_controller(ft_broadcaster) failed")
        sw = await self._call_service(
            SwitchController,
            f"{cm}/switch_controller",
            SwitchController.Request(
                activate_controllers=["ft_broadcaster"], strictness=SwitchController.Request.STRICT
            ),
        )
        self.assertTrue(sw.ok, "switch_controller(ft_broadcaster) failed")

        msgs = []
        self.create_subscription(self._node, WrenchStamped, "/ft_broadcaster/wrench", lambda m: msgs.append(m))
        got = await self.simulate_until_condition(lambda: len(msgs) >= 3, max_frames=900, per_frame_callback=self._spin)
        self.assertTrue(got, "ForceTorqueSensorBroadcaster did not publish /ft_broadcaster/wrench")

        f = msgs[-1].wrench.force
        t = msgs[-1].wrench.torque
        for c in (f.x, f.y, f.z, t.x, t.y, t.z):
            self.assertTrue(math.isfinite(c), f"non-finite wrench component (wrench={msgs[-1].wrench})")
        # link1 (1 kg) hangs off joint1, so the joint reaction must carry its weight:
        # a non-trivial force proves read() filled the interfaces from the physics
        # wrench rather than leaving the 0.0 export default.
        force_mag = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.assertGreater(force_mag, 1.0, f"FT force magnitude {force_mag} too small; wrench not reading the load")
        # COM sits at the joint origin (no lever arm), so the reaction is essentially pure
        # force; a torque on the order of the force would mean the force/torque slices are mixed.
        torque_mag = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
        self.assertLess(
            torque_mag,
            0.25 * force_mag,
            f"FT torque {torque_mag} too large vs force {force_mag}; slices may be transposed",
        )

    def _add_imu_prim(self, name: str = "imu_sensor", link: str = "link1") -> None:
        """Author an IsaacImuSensor prim under the arm so on_configure resolves it by name.

        Sets the stage to meters (the Isaac robotics convention) so the IMU reports
        linear_acceleration in m/s^2 per REP-145. IsaacImuSensor reports in stage units/s^2,
        so on the default centimeter stage it would read ~100x (cm/s^2)."""
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        self._stage.DefinePrim(f"/World/Arm/{link}/{name}", "IsaacImuSensor")

    async def test_imu_sensor_exports_available_interfaces(self):
        """An IMU <sensor> resolving to an IsaacImuSensor prim is classified as IMU and its
        10 orientation/angular_velocity/linear_acceleration state interfaces are exported
        and made available through the real CM. Reaching 'available' proves on_configure
        (by-name prim resolution + IsaacImuSensor type check) succeeded."""
        from controller_manager_msgs.srv import ListHardwareInterfaces

        self._add_imu_prim()
        self._yaml_path = self._write_yaml(_IMU_CONTROLLER_YAML)
        overlay = self._write_overlay(_IMU_OVERLAY_URDF)
        rc = await self._play_and_setup(urdf_path=overlay)
        self.assertEqual(rc, 0, "setup_cm should succeed with an IMU sensor overlay")

        res = await self._call_service(
            ListHardwareInterfaces,
            f"{self._controller_manager()}/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
        )
        available = {i.name for i in res.state_interfaces if i.is_available}
        for comp in (
            "orientation.x",
            "orientation.y",
            "orientation.z",
            "orientation.w",
            "angular_velocity.x",
            "angular_velocity.y",
            "angular_velocity.z",
            "linear_acceleration.x",
            "linear_acceleration.y",
            "linear_acceleration.z",
        ):
            self.assertIn(f"imu_sensor/{comp}", available, f"imu_sensor/{comp} not exported/available")

    async def test_imu_sensor_broadcaster_publishes_imu(self):
        """End-to-end: IMUSensorBroadcaster publishes a finite sensor_msgs/Imu whose
        linear_acceleration magnitude is ~g for a stationary sensor, proving read() fills
        the interfaces from getSensorReading with the REP-145 gravity convention
        (readGravity=true)."""
        import math

        from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
        from sensor_msgs.msg import Imu

        self._add_imu_prim()
        self._yaml_path = self._write_yaml(_IMU_CONTROLLER_YAML)
        overlay = self._write_overlay(_IMU_OVERLAY_URDF)
        rc = await self._play_and_setup(urdf_path=overlay)
        self.assertEqual(rc, 0, "setup_cm should succeed with an IMU sensor overlay")

        cm = self._controller_manager()
        loaded = await self._call_service(
            LoadController, f"{cm}/load_controller", LoadController.Request(name="imu_broadcaster")
        )
        self.assertTrue(loaded.ok, "load_controller(imu_broadcaster) failed")
        cfg = await self._call_service(
            ConfigureController, f"{cm}/configure_controller", ConfigureController.Request(name="imu_broadcaster")
        )
        self.assertTrue(cfg.ok, "configure_controller(imu_broadcaster) failed")
        sw = await self._call_service(
            SwitchController,
            f"{cm}/switch_controller",
            SwitchController.Request(
                activate_controllers=["imu_broadcaster"], strictness=SwitchController.Request.STRICT
            ),
        )
        self.assertTrue(sw.ok, "switch_controller(imu_broadcaster) failed")

        msgs = []
        self.create_subscription(self._node, Imu, "/imu_broadcaster/imu", lambda m: msgs.append(m))
        got = await self.simulate_until_condition(lambda: len(msgs) >= 3, max_frames=900, per_frame_callback=self._spin)
        self.assertTrue(got, "IMUSensorBroadcaster did not publish /imu_broadcaster/imu")

        a = msgs[-1].linear_acceleration
        for c in (a.x, a.y, a.z):
            self.assertTrue(math.isfinite(c), f"non-finite linear_acceleration ({a})")
        # REP-145: a stationary IMU reports proper acceleration including the gravity
        # reaction (~9.81 m/s^2), independent of orientation. readGravity=false would give
        # ~0, so the magnitude bracket guards the gravity convention.
        accel_mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.assertGreater(accel_mag, 9.0, f"IMU |linear_acceleration|={accel_mag}; gravity reaction missing")
        # Upper bound tolerates minor settling transients on the held arm while still
        # catching gross errors (e.g. a cm/s^2 stage-unit reading would be ~981).
        self.assertLess(accel_mag, 11.0, f"IMU |linear_acceleration|={accel_mag}; unexpectedly large")
