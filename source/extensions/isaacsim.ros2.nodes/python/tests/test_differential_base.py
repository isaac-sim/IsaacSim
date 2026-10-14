# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify ROS 2 differential base command and odometry graphs.

Covers Carter, Nova Carter, and scratch-built differential-drive setups.
"""

import time
from copy import deepcopy
from typing import Any

import carb
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from pxr import Gf

from .common import (
    add_carter,
    add_carter_ros,
    add_nova_carter_ros,
    get_qos_profile,
    set_rotate,
    set_translate,
    simulate_async,
)

_NOVA_CARTER_ROS_ROOT = "/nova_carter_ros2_sensors"
_NOVA_CARTER_BASE_LINK_PATH = _NOVA_CARTER_ROS_ROOT + "/chassis_link/base_link"
_NOVA_CARTER_ODOMETRY_GRAPH_PATH = _NOVA_CARTER_ROS_ROOT + "/transform_tree_odometry"
_NOVA_CARTER_DRIVE_GRAPH_PATH = _NOVA_CARTER_ROS_ROOT + "/differential_drive"


class TestRos2DifferentialBase(ROS2TestCase):
    """Verify differential-drive command, TF, and odometry ROS 2 graphs."""

    async def setUp(self) -> None:
        """Create a fresh stage and ROS 2 node for differential-drive graph tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        SimulationManager.enable_fabric(enable=False)

        # Initialize class members
        self._trans = None
        self._world_to_odom = None
        self._odom_to_base_link = None
        self._odom_data = None

        # Initialize velocity tracking members
        self._command_start_time = None
        self._target_velocity = 0.0
        self._target_velocity_reach_time = None
        self._velocity_check_func = None

        # Create ROS2 node for this test

        self.node = self.create_node("isaac_sim_test_diff_drive")

    async def tearDown(self) -> None:
        """Clear differential-drive callback state and the ROS 2 test node."""
        self.node = None

        # Reset class members
        self._trans = None
        self._world_to_odom = None
        self._odom_to_base_link = None
        self._odom_data = None

        # Reset velocity tracking members
        self._command_start_time = None
        self._target_velocity = 0.0
        self._target_velocity_reach_time = None
        self._velocity_check_func = None

        await super().tearDown()

    def spin(self) -> None:
        """Process one ready ROS 2 callback without blocking the simulation loop."""
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=0.0)

    async def wait_for_cmd_vel_subscriber(self, publisher: Any) -> None:
        """Wait for the graph-side cmd_vel subscriber before publishing one-shot commands."""
        await self.wait_for_subscribers_on_topic(publisher, timeout_sec=10.0, per_frame_callback=self.spin)

    async def simulate_for_duration(self, duration: float) -> None:
        """Advance until the requested simulation duration has elapsed."""
        start_time = SimulationManager.get_simulation_time()
        await self.simulate_until_condition(
            lambda: SimulationManager.get_simulation_time() - start_time >= duration,
            max_frames=max(240, int(duration * 240)),
            per_frame_callback=self.spin,
            failure_message=f"Simulation did not advance by {duration} seconds",
        )

    def tf_callback(self, data: Any) -> None:
        """Handle tf callback.

        Args:
            data: Transform tree message.
        """
        for transform in data.transforms:
            if transform.header.frame_id == "world" and transform.child_frame_id == "odom":
                self._world_to_odom = transform
            elif transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                self._odom_to_base_link = transform
        self._trans = data.transforms[-1]

    def odom_callback(self, data: Any) -> None:
        """Handle odom callback.

        Args:
            data: Odometry message.
        """
        self._odom_data = data.pose.pose
        if self._command_start_time is not None and self._velocity_check_func is not None:
            self._velocity_check_func(data)

    def move_cmd_msg(self, x: Any, y: Any, z: Any, ax: Any, ay: Any, az: Any) -> Any:
        """Create a move cmd msg message.

        Args:
            x: Linear x command.
            y: Linear y command.
            z: Linear z command.
            ax: Angular x command.
            ay: Angular y command.
            az: Angular z command.

        Returns:
            Twist command message.
        """
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = ax
        msg.angular.y = ay
        msg.angular.z = az
        return msg

    async def test_carter_differential_base(self) -> None:
        """Test carter differential base."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Odometry node not yet supported by Newton backend")
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_carter_ros(self._assets_root_path)
        stage = omni.usd.get_context().get_stage()

        graph_path = "/Carter/ActionGraph"

        # Publish the standard `world` -> `odom` and `odom` -> `base_link` transforms without adding an `odom` USD prim.
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("ComputeOdometryTF", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("PublishWorldToOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                        ("PublishOdomToBaseLink", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ComputeOdometryTF.inputs:chassisPrim", [usdrt.Sdf.Path("/Carter")]),
                        ("PublishWorldToOdom.inputs:topicName", "tf_test"),
                        ("PublishWorldToOdom.inputs:parentFrameId", "world"),
                        ("PublishWorldToOdom.inputs:childFrameId", "odom"),
                        ("PublishOdomToBaseLink.inputs:topicName", "tf_test"),
                        ("PublishOdomToBaseLink.inputs:parentFrameId", "odom"),
                        ("PublishOdomToBaseLink.inputs:childFrameId", "base_link"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "ComputeOdometryTF.inputs:execIn"),
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishWorldToOdom.inputs:execIn"),
                        ("ComputeOdometryTF.outputs:execOut", "PublishOdomToBaseLink.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishWorldToOdom.inputs:timeStamp",
                        ),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishOdomToBaseLink.inputs:timeStamp",
                        ),
                        ("ComputeOdometryTF.outputs:position", "PublishOdomToBaseLink.inputs:translation"),
                        ("ComputeOdometryTF.outputs:orientation", "PublishOdomToBaseLink.inputs:rotation"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # move carter off origin
        carter_prim = stage.GetPrimAtPath("/Carter")
        new_translate = Gf.Vec3d(1.00, -3.00, 0.25)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)
        await omni.kit.app.get_app().next_update_async()

        tf_sub = self.create_subscription(self.node, TFMessage, "tf_test", self.tf_callback, get_qos_profile())
        odom_sub = self.create_subscription(self.node, Odometry, "odom", self.odom_callback, get_qos_profile())
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Wait for one second of simulated physics to settle, then wait for subscriber data.
        # App updates are not guaranteed to map one-to-one to physics steps.
        await self.simulate_for_duration(1.0)
        await self.simulate_until_condition(
            lambda: self._world_to_odom is not None
            and self._odom_to_base_link is not None
            and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # Check 0: `world` -> `odom` is identity and `odom` -> `base_link` matches odometry.
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_world_to_odom = [0, 0, 0, 0, 0, 0, 1]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, -0.23, 0, 0, 0, 1]
        self._trans = self._world_to_odom
        self.check_pose(expected_world_to_odom, expected_odom, tolerance=1)
        self._trans = self._odom_to_base_link
        self.check_pose(expected_odom, None, tolerance=1)

        # straight forward for 3s at 0.1 m/s → ~0.3m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_for_duration(3.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 1: location using default param
        print(self._trans.transform, self._odom_data)
        # [tx, ty, tz, rx, ry, rz, rw]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.3, 0, -0.23, 0, 0, 0, 1]
        self._trans = self._world_to_odom
        self.check_pose(expected_world_to_odom, expected_odom, tolerance=1)
        self._trans = self._odom_to_base_link
        self.check_pose(expected_odom, None, tolerance=1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.spin()
        self._trans = None
        self._world_to_odom = None
        self._odom_to_base_link = None
        self._odom_data = None

        # change wheel rotation and wheel base
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelRadius"), 0.1)
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelDistance"), 0.5)

        self._timeline.play()

        # Let one second of simulated physics settle before reading subscriber data.
        await self.simulate_for_duration(1.0)
        await self.simulate_until_condition(
            lambda: self._world_to_odom is not None
            and self._odom_to_base_link is not None
            and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # Check 3: `world` -> `odom` is identity and `odom` -> `base_link` matches odometry.
        # [tx, ty, tz, rx, ry, rz, rw]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, -0.23, 0, 0, 0, 1]
        self._trans = self._world_to_odom
        self.check_pose(expected_world_to_odom, expected_odom, tolerance=1)
        self._trans = self._odom_to_base_link
        self.check_pose(expected_odom, None, tolerance=1)

        # straight forward for 3s at 0.1 m/s (odometry units, new wheel params) → ~0.7m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_for_duration(3.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        # check 4: location after change radius
        # wait for all data to be received by subscribers
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )

        # [tx, ty, tz, rx, ry, rz, rw]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.7, 0, -0.23, 0, 0, 0, 1]
        self._trans = self._world_to_odom
        self.check_pose(expected_world_to_odom, expected_odom, tolerance=1)
        self._trans = self._odom_to_base_link
        self.check_pose(expected_odom, None, tolerance=1)

        self._timeline.stop()
        self.spin()

    # add carter and ROS topic from scratch
    async def test_differential_base_scratch(self) -> None:
        """Test differential base scratch."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Odometry node not yet supported by Newton backend")
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        await add_carter(self._assets_root_path)

        odom_sub = self.create_subscription(self.node, Odometry, "odom", self.odom_callback, 10)
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        graph_path = "/ActionGraph"
        graph_id, created_nodes = self.add_differential_drive(graph_path, "/Carter")
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Let one second of simulated physics settle before checking data.
        await self.simulate_for_duration(1.0)
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # check 0: is carter initially stationary
        # No transform expected in this test, only check odometry
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(None, expected_odom, tolerance=1)

        # rotate for 2s at angular_z=0.2 rad/s → orientation.z ~0.40
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_for_duration(2.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_for_duration(1.0)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0.40, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.916, 1)

        # change wheel rotation and wheel base
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

        graph_id, created_nodes = self.add_differential_drive(graph_path, "/Carter")
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelDistance").set(0.8)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Let one second of simulated physics settle before issuing the rotation command.
        await self.simulate_for_duration(1.0)

        # rotate back for 2s at angular_z=-0.2 rad/s (wider wheelbase → faster rotation) → orientation.z ~-0.61
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, -0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        angular_velocity = og.Controller.attribute(graph_path + "/subscribeTwist.outputs:angularVelocity")
        deadline = time.monotonic() + 10.0
        last_publish = time.monotonic()
        while time.monotonic() < deadline:
            await omni.kit.app.get_app().next_update_async()
            self.spin()
            received_velocity = angular_velocity.get()
            if received_velocity is not None and abs(received_velocity[2] - move_cmd.angular.z) < 1e-3:
                break
            if time.monotonic() - last_publish >= 0.1:
                cmd_vel_pub.publish(move_cmd)
                last_publish = time.monotonic()
        else:
            self.fail("Recreated differential drive graph did not receive reverse cmd_vel")
        await self.simulate_for_duration(2.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_for_duration(1.0)

        # check 3: location after change radius
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, -0.61, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.79, 1)

        self._timeline.stop()
        self.spin()

    async def test_nova_carter_differential_base(self) -> None:
        """Test nova carter differential base."""
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_nova_carter_ros(self._assets_root_path, enable_sensors=False)
        stage = omni.usd.get_context().get_stage()

        graph_path = _NOVA_CARTER_ODOMETRY_GRAPH_PATH
        drive_graph_path = _NOVA_CARTER_DRIVE_GRAPH_PATH
        # Publish the world pose of the existing `base_link` prim as `world` -> `base_link`.
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("ReadBaseLinkWorldPose", "isaacsim.core.nodes.IsaacReadWorldPose"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "tf_test"),
                        ("PublishTF.inputs:parentFrameId", "world"),
                        ("PublishTF.inputs:childFrameId", "base_link"),
                        ("ReadBaseLinkWorldPose.inputs:prim", [usdrt.Sdf.Path(_NOVA_CARTER_BASE_LINK_PATH)]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                        ("ReadBaseLinkWorldPose.outputs:translation", "PublishTF.inputs:translation"),
                        ("ReadBaseLinkWorldPose.outputs:orientation", "PublishTF.inputs:rotation"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()
        # move carter off origin
        carter_prim = stage.GetPrimAtPath(_NOVA_CARTER_ROS_ROOT)
        new_translate = Gf.Vec3d(1.00, -3.00, 0.0)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)
        await omni.kit.app.get_app().next_update_async()

        self.create_subscription(self.node, TFMessage, "tf_test", self.tf_callback, get_qos_profile())
        self.create_subscription(self.node, Odometry, "chassis/odom", self.odom_callback, get_qos_profile())
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # wait for physics to settle and for all data to be received by subscribers
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=240,
            per_frame_callback=self.spin,
        )
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # check 0: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        # straight forward for 3s at 0.1 m/s → ~0.3m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_for_duration(3.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 1: location using default param
        carb.log_info(str(self._trans.transform))
        carb.log_info(str(self._odom_data))
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.22, -2.78, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.3, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.spin()

        # change wheel rotation and wheel base
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelRadius"), 0.1
        )
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelDistance"), 0.5
        )

        self._timeline.play()
        await self.simulate_for_duration(1.0)
        # Drain callbacks from before the stop, then wait for the reset pose from the replayed graph.
        self._trans = None
        self._odom_data = None
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            await omni.kit.app.get_app().next_update_async()
            self.spin()
            if self._trans is not None and self._odom_data is not None and abs(self._odom_data.position.x) <= 0.1:
                break
        else:
            last_position = None if self._odom_data is None else self._odom_data.position.x
            self.fail(f"Nova Carter odometry did not reset after timeline restart (last x={last_position})")

        # check 3: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # straight forward for 3s at 0.1 m/s (odometry units, new wheel params) → ~0.43m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_for_duration(3.0)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 4: location after change radius
        carb.log_info(str(self._trans.transform))
        carb.log_info(str(self._odom_data))
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.30, -2.69, 0, 0, 0, 0.3815, 0.9244]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.43, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        self._timeline.stop()
        self.spin()

    async def test_nova_carter_angular_velocity(self):
        """Test Nova Carter robot's response time to angular velocity commands.

        This test guards against the regression where Nova Carter took too long to
        reach a commanded angular velocity. It publishes a 0.1 rad/s angular velocity
        command and measures the simulation time until odometry first reports 90% of
        that target.

        The test fails if the robot needs more than 20 seconds to reach 90% of target.
        """
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        # Set up velocity tracking members
        self._target_velocity_reach_time = None

        def check_angular_velocity(data):
            # Latch the first crossing: callbacks keep arriving while the robot spins
            # down, and re-recording would report the last crossing instead.
            if self._target_velocity_reach_time is not None:
                return
            angular_z = data.twist.twist.angular.z
            if abs(angular_z) >= abs(self._target_velocity) * 0.9:
                self._target_velocity_reach_time = SimulationManager.get_simulation_time() - self._command_start_time
                print(f"Reached 90% target angular velocity in {self._target_velocity_reach_time} seconds")

        self._velocity_check_func = check_angular_velocity

        # Load Nova Carter robot
        await add_nova_carter_ros(self._assets_root_path, enable_sensors=False)

        # Set up ROS2 subscribers and publishers
        odom_sub = self.create_subscription(self.node, Odometry, "chassis/odom", self.odom_callback, get_qos_profile())
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        # Start simulation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, self.spin)

        # Wait for initial odometry data
        await self.simulate_until_condition(
            lambda: self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # Verify initial state - robot should be stationary
        self.assertIsNotNone(self._odom_data, "Initial odometry data should be available")

        # Send angular velocity command (replicating the reported issue)
        self._target_velocity = 0.1
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, self._target_velocity)

        # Wait for the graph-side subscriber so the one-shot command is not dropped.
        # This runs before the start time is recorded so the wait is not measured.
        await self.wait_for_cmd_vel_subscriber(cmd_vel_pub)

        # Record command send time
        self._command_start_time = SimulationManager.get_simulation_time()
        cmd_vel_pub.publish(move_cmd)

        carb.log_info(f"Sent angular velocity command: {self._target_velocity} rad/s")

        # Watch for up to 30 seconds of simulation time after the command. This window is
        # deliberately longer than the 20 second acceptance threshold asserted below, so a
        # slow run reports the time it actually took instead of "never reached target".
        max_monitor_duration = 30.0

        while SimulationManager.get_simulation_time() - self._command_start_time < max_monitor_duration:
            await simulate_async(0.1, 60, self.spin)

            # Check if we've reached 90% of target velocity
            if self._target_velocity_reach_time is not None:
                break

        # Stop the robot
        stop_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(stop_cmd)
        await simulate_async(1, 60, self.spin)

        # Validate response time
        self.assertIsNotNone(
            self._target_velocity_reach_time,
            f"Robot should have reached 90% of target velocity of {self._target_velocity} within test duration",
        )

        # The main assertion: robot should reach 90% of target velocity within 20 seconds.
        # Measured at ~15.5s once the test waits for the graph-side cmd_vel subscriber,
        # so this leaves margin for slower machines rather than tracking the observed value.
        max_acceptable_target_velocity_reach_time = 20.0  # seconds
        self.assertLess(
            self._target_velocity_reach_time,
            max_acceptable_target_velocity_reach_time,
            f"Robot took {self._target_velocity_reach_time:.2f}s to reach 90% target velocity of {self._target_velocity}, "
            f"but should reach it within {max_acceptable_target_velocity_reach_time}s",
        )

        print(f"SUCCESS: Robot reached 90% target velocity in {self._target_velocity_reach_time:.2f} seconds")

        self._timeline.stop()
        self.spin()

    def check_pose(self, expected_trans: Any, expected_odom: Any, tolerance: Any = 1, delta: Any = None) -> None:
        """Verify robot pose against expected transform and odometry values.

        Args:
            expected_trans: Expected [tx, ty, tz, rx, ry, rz, rw] or None to skip.
            expected_odom: Expected [px, py, pz, ox, oy, oz, ow] or None to skip.
            tolerance: Decimal places for assertAlmostEqual (used when delta is None).
            delta: Absolute tolerance for assertAlmostEqual (overrides tolerance).
        """

        def _assert_close(actual: Any, expected: Any, msg: Any = "") -> None:
            if delta is not None:
                self.assertAlmostEqual(actual, expected, delta=delta, msg=msg)
            else:
                self.assertAlmostEqual(actual, expected, tolerance, msg=msg)

        if expected_trans is not None:
            self.assertIsNotNone(self._trans)
            if len(expected_trans) >= 3:
                _assert_close(self._trans.transform.translation.x, expected_trans[0])
                _assert_close(self._trans.transform.translation.y, expected_trans[1])
                _assert_close(self._trans.transform.translation.z, expected_trans[2])
            if len(expected_trans) >= 7:
                _assert_close(self._trans.transform.rotation.x, expected_trans[3])
                _assert_close(self._trans.transform.rotation.y, expected_trans[4])
                _assert_close(self._trans.transform.rotation.z, expected_trans[5])
                _assert_close(self._trans.transform.rotation.w, expected_trans[6])

        if expected_odom is not None:
            self.assertIsNotNone(self._odom_data)
            odom_data = deepcopy(self._odom_data)
            if len(expected_odom) >= 3:
                _assert_close(odom_data.position.x, expected_odom[0])
                _assert_close(odom_data.position.y, expected_odom[1])
                _assert_close(odom_data.position.z, expected_odom[2])
            if len(expected_odom) >= 7:
                _assert_close(odom_data.orientation.x, expected_odom[3])
                _assert_close(odom_data.orientation.y, expected_odom[4])
                _assert_close(odom_data.orientation.z, expected_odom[5])
                _assert_close(odom_data.orientation.w, expected_odom[6])

    def add_differential_drive(self, graph_path: Any, robot_path: Any) -> Any:
        """Add differential drive to the test scene.

        Args:
            graph_path: OmniGraph path to create.
            robot_path: Robot prim path to control.

        Returns:
            Created graph and nodes.
        """
        try:
            keys = og.Controller.Keys
            graph, nodes, _, _ = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Odometry publisher
                        ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        ("publishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                        # Added nodes used for Twist subscriber, differential drive
                        ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("diffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                        ("artController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                        ("computeOdom.outputs:execOut", "publishOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                        ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                        ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                        ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                        ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                        ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                        ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),
                        ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                        ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                        ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                        ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                        ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                        ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                        ("diffController.outputs:velocityCommand", "artController.inputs:velocityCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("diffController.inputs:wheelRadius", 0.24),
                        ("diffController.inputs:wheelDistance", 0.5),
                        ("artController.inputs:jointNames", ["left_wheel", "right_wheel"]),
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
                        ("artController.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
                    ],
                },
            )
        except Exception as e:
            print(e)

        return graph, nodes
