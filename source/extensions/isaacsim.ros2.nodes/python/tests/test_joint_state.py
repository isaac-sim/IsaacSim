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

"""Tests for ROS 2 joint state publisher OmniGraph node."""

import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from numpy import pi as PI

from .common import (
    SIMPLE_ARTICULATION_3J_REVERSED_JOINTS,
    fix_reversed_joints,
    get_qos_profile,
    set_joint_drive_parameters,
)


class TestRos2JointStatePublisher(ROS2TestCase):
    """Test suite for ros2 joint state publisher."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        result, error = await stage_utils.open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()

        fix_reversed_joints(SIMPLE_ARTICULATION_3J_REVERSED_JOINTS)

        # ROS-ify asset by adding a joint state publisher
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "PublishJointState.inputs:targetPrim",
                            [usdrt.Sdf.Path("/Articulation")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        pass

    async def test_joint_state_position_publisher(self):
        """Test joint state position publisher."""
        import rclpy
        from sensor_msgs.msg import JointState

        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.position = data.position
            self.js_ros.velocity = data.velocity
            self.js_ros.effort = data.effort

        node = self.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = self.create_subscription(node, JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.01)

        default_position = [-80 * PI / 180.0, 0.4, 30 * PI / 180.0]

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        art_handle = Articulation("/Articulation")
        art_handle.set_dof_position_targets(default_position)

        await self.simulate_until_condition(
            lambda: len(self.js_ros.position) > 0
            and all(abs(self.js_ros.position[i] - default_position[i]) < 1e-3 for i in range(len(default_position))),
            max_frames=120,
            per_frame_callback=spin,
        )
        received_position = self.js_ros.position

        print("\n received_position", received_position)

        self.assertAlmostEqual(received_position[0], default_position[0], delta=1e-3)
        self.assertAlmostEqual(received_position[1], default_position[1], delta=1e-3)
        self.assertAlmostEqual(received_position[2], default_position[2], delta=1e-3)

        self._timeline.stop()
        spin()

        node.destroy_subscription(js_sub)
        node.destroy_node()

    async def test_joint_state_velocity_publisher(self):
        """Test joint state velocity publisher."""
        import rclpy
        from sensor_msgs.msg import JointState

        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.velocity = data.velocity

        node = self.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = self.create_subscription(node, JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.01)

        test_velocities = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        art_handle = Articulation("/Articulation")
        art_handle.set_dof_gains(stiffnesses=[0.0, 0.0, 0.0], dampings=[1e4, 1e4, 1e4])
        art_handle.set_dof_velocity_targets(test_velocities)

        # Wait for the joints to actually converge to the commanded velocities, not
        # just for any velocity sample to arrive. The prismatic joint reproducibly
        # overshoots its target on the very first published sample because the drive
        # is configured *after* timeline.play() + one update, so the first physics
        # step runs with the original (zero-damping) drive before the new gains take
        # effect. Mirror the convergence-based condition used by the sibling
        # test_joint_state_position_publisher.
        await self.simulate_until_condition(
            lambda: len(self.js_ros.velocity) > 0
            and all(abs(self.js_ros.velocity[i] - test_velocities[i]) < 1e-3 for i in range(len(test_velocities))),
            max_frames=120,
            per_frame_callback=spin,
        )
        received_velocity = self.js_ros.velocity

        print("received_velocity", received_velocity)

        self.assertAlmostEqual(received_velocity[0], test_velocities[0], delta=1e-3)
        self.assertAlmostEqual(received_velocity[1], test_velocities[1], delta=1e-3)
        self.assertAlmostEqual(received_velocity[2], test_velocities[2], delta=1e-3)
        self._timeline.stop()
        spin()

        node.destroy_subscription(js_sub)
        node.destroy_node()


class TestRos2JointStatePublisherFromSensor(ROS2TestCase):
    """Test ROS2 Publish Joint State using the new path: Isaac Read Joint State outputs connected (no targetPrim)."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("IsaacReadJointState sensor node requires PhysX backend")
        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        result, error = await stage_utils.open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)
        self._stage = omni.usd.get_context().get_stage()

        fix_reversed_joints(SIMPLE_ARTICULATION_3J_REVERSED_JOINTS)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ReadJointState.inputs:prim", [usdrt.Sdf.Path("/Articulation")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                        ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                        ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                        ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                        ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                        ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                        ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                        ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                        ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    async def test_joint_state_position_publisher_from_sensor(self):
        """Publish joint state from Isaac Read Joint State outputs; verify positions on joint_states topic."""
        import rclpy
        from sensor_msgs.msg import JointState

        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.position = data.position
            self.js_ros.velocity = data.velocity
            self.js_ros.effort = data.effort

        node = self.create_node("isaac_sim_test_joint_state_pub_sensor")
        js_sub = self.create_subscription(node, JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.01)

        default_position = [-80 * PI / 180.0, 0.4, 30 * PI / 180.0]

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        art_handle = Articulation("/Articulation")
        art_handle.set_dof_position_targets(default_position)
        print("\n commanded position", default_position)

        await self.simulate_until_condition(
            lambda: len(self.js_ros.position) > 0
            and all(abs(self.js_ros.position[i] - default_position[i]) < 1e-3 for i in range(len(default_position))),
            max_frames=120,
            per_frame_callback=spin,
        )
        received_position = self.js_ros.position

        print("\n received_position", received_position)

        self.assertAlmostEqual(received_position[0], default_position[0], delta=1e-3)
        self.assertAlmostEqual(received_position[1], default_position[1], delta=1e-3)
        self.assertAlmostEqual(received_position[2], default_position[2], delta=1e-3)

        self._timeline.stop()
        spin()
        node.destroy_subscription(js_sub)
        node.destroy_node()

    async def test_joint_state_velocity_publisher_from_sensor(self):
        """Publish joint state from Isaac Read Joint State; verify velocities on joint_states topic."""
        import rclpy
        from sensor_msgs.msg import JointState

        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.velocity = data.velocity

        node = self.create_node("isaac_sim_test_joint_state_pub_sensor_vel")
        js_sub = self.create_subscription(node, JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.01)

        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]
        joint_types = ["angular", "linear", "angular"]
        test_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", test_velocities[i], joint_stiffness, joint_damping
            )

        self._timeline.play()
        await self.simulate_until_condition(
            lambda: len(self.js_ros.velocity) > 0, max_frames=60, per_frame_callback=spin
        )
        received_velocity = self.js_ros.velocity

        comp_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        self.assertAlmostEqual(received_velocity[0], comp_velocity[0], delta=1e-3)
        self.assertAlmostEqual(received_velocity[1], comp_velocity[1], delta=1e-3)
        self.assertAlmostEqual(received_velocity[2], comp_velocity[2], delta=1e-3)

        self._timeline.stop()
        spin()
        node.destroy_subscription(js_sub)
        node.destroy_node()


class TestRos2JointStateSubscriber(ROS2TestCase):
    """Test suite for ros2 joint state subscriber."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        result, error = await stage_utils.open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()

        fix_reversed_joints(SIMPLE_ARTICULATION_3J_REVERSED_JOINTS)

        # setup the graph
        try:
            test_graph, new_nodes, _, _ = og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation")]),
                    ],
                },
            )

            self.subscriber_node = new_nodes[1]

        except Exception as e:
            print(e)

        pass

    async def test_joint_state_subscriber_node(self):
        """Test if the joint state subscriber node is able to receive the joint state commands."""
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = self.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = self.create_publisher(ros2_node, JointState, "joint_command", 10)

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        js_position.velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        js_position.effort = [0.4, -0.2, 0.3]

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=30)

        # publish value
        ros2_publisher.publish(js_position)
        await self.simulate_until_condition(
            lambda: len(og.Controller.attribute("outputs:jointNames", self.subscriber_node).get()) > 0,
            max_frames=60,
        )

        # get the value from the subscriber node
        joint_names = og.Controller.attribute("outputs:jointNames", self.subscriber_node).get()
        positions_received = og.Controller.attribute("outputs:positionCommand", self.subscriber_node).get()
        velocities_received = og.Controller.attribute("outputs:velocityCommand", self.subscriber_node).get()
        efforts_received = og.Controller.attribute("outputs:effortCommand", self.subscriber_node).get()

        self.assertAlmostEqual(positions_received[0], js_position.position[0], delta=1e-3)
        self.assertAlmostEqual(positions_received[1], js_position.position[1], delta=1e-3)
        self.assertAlmostEqual(positions_received[2], js_position.position[2], delta=1e-3)
        self.assertAlmostEqual(velocities_received[0], js_position.velocity[0], delta=1e-3)
        self.assertAlmostEqual(velocities_received[1], js_position.velocity[1], delta=1e-3)
        self.assertAlmostEqual(velocities_received[2], js_position.velocity[2], delta=1e-3)
        self.assertAlmostEqual(efforts_received[0], js_position.effort[0], delta=1e-3)
        self.assertAlmostEqual(efforts_received[1], js_position.effort[1], delta=1e-3)
        self.assertAlmostEqual(efforts_received[2], js_position.effort[2], delta=1e-3)

    async def test_joint_state_subscriber(self):
        """Test if the joint state subscriber is able to move the robot as expected."""
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = self.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = self.create_publisher(ros2_node, JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=30)

        art_handle = Articulation("/Articulation")

        def reset_robot():
            art_handle.set_dof_positions([0.0, 0.0, 0.0])
            post_reset = art_handle.get_dof_positions().numpy().flatten()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # wait for joints to reach commanded position
        await self.simulate_until_condition(
            lambda: all(
                abs(art_handle.get_dof_positions().numpy().flatten()[i] - test_position[i]) < 0.001
                for i in range(len(test_position))
            ),
            max_frames=180,
        )

        joint_command_received = art_handle.get_dof_positions().numpy().flatten()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        art_handle.set_dof_gains(stiffnesses=[0.0, 0.0, 0.0], dampings=[1e4, 1e4, 1e4])
        art_handle.set_dof_velocity_targets([5.0, 0.1, -2.5])

        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # wait for all joints to reach commanded velocity (coupling forces from joints 0/2
        # decelerating from their high initial targets can briefly push joint 1 above target)
        await self.simulate_until_condition(
            lambda: all(
                abs(art_handle.get_dof_velocities().numpy().flatten()[i] - test_velocity[i]) < 0.01
                for i in range(len(test_velocity))
            ),
            max_frames=240,
        )

        joint_command_received = art_handle.get_dof_velocities().numpy().flatten()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        art_handle.set_dof_gains(stiffnesses=[0.0, 1e5, 0.0], dampings=[1e4, 1e4, 1e4])
        art_handle.set_dof_position_targets([0.0, 0.2, 0.0])

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # wait for prismatic joint to reach commanded position and fully settle (velocity → 0)
        await self.simulate_until_condition(
            lambda: abs(art_handle.get_dof_positions().numpy().flatten()[1] - 0.4) < 0.001
            and abs(art_handle.get_dof_velocities().numpy().flatten()[1]) < 0.01,
            max_frames=180,
        )

        joint_position_received = art_handle.get_dof_positions().numpy().flatten()
        joint_velocity_received = art_handle.get_dof_velocities().numpy().flatten()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=2e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)

        ros2_node.destroy_publisher(ros2_publisher)
        ros2_node.destroy_node()

    async def test_joint_state_subscriber_with_names(self):
        """Test if the joint state subscriber is able to move the robot as expected."""
        # add the connection between joint names from subscriber and the controller
        graph_handle = og.get_graph_by_path("/ActionGraph")
        og.Controller.connect(
            "/ActionGraph/SubscribeJointState.outputs:jointNames",
            "/ActionGraph/ArticulationController.inputs:jointNames",
        )
        await og.Controller.evaluate(graph_handle)

        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = self.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = self.create_publisher(ros2_node, JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=30)

        art_handle = Articulation("/Articulation")

        def reset_robot():
            art_handle.set_dof_positions([0.0, 0.0, 0.0])
            post_reset = art_handle.get_dof_positions().numpy().flatten()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # wait for joints to reach commanded position
        await self.simulate_until_condition(
            lambda: all(
                abs(art_handle.get_dof_positions().numpy().flatten()[i] - test_position[i]) < 0.001
                for i in range(len(test_position))
            ),
            max_frames=180,
        )

        joint_command_received = art_handle.get_dof_positions().numpy().flatten()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        art_handle.set_dof_gains(stiffnesses=[0.0, 0.0, 0.0], dampings=[1e4, 1e4, 1e4])
        art_handle.set_dof_velocity_targets([5.0, 0.1, -2.5])

        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # wait for all joints to reach commanded velocity
        await self.simulate_until_condition(
            lambda: all(
                abs(art_handle.get_dof_velocities().numpy().flatten()[i] - test_velocity[i]) < 0.01
                for i in range(len(test_velocity))
            ),
            max_frames=240,
        )

        joint_command_received = art_handle.get_dof_velocities().numpy().flatten()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        art_handle.set_dof_gains(stiffnesses=[0.0, 1e5, 0.0], dampings=[1e4, 1e4, 1e4])
        art_handle.set_dof_position_targets([0.0, 0.2, 0.0])

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # wait for prismatic joint to reach commanded position and fully settle (velocity → 0)
        await self.simulate_until_condition(
            lambda: abs(art_handle.get_dof_positions().numpy().flatten()[1] - 0.4) < 0.001
            and abs(art_handle.get_dof_velocities().numpy().flatten()[1]) < 0.01,
            max_frames=180,
        )

        joint_position_received = art_handle.get_dof_positions().numpy().flatten()
        joint_velocity_received = art_handle.get_dof_velocities().numpy().flatten()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=2e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)

        ros2_node.destroy_publisher(ros2_publisher)
        ros2_node.destroy_node()

    async def test_joint_state_subscriber_with_name_override(self):
        """Test that JointNameResolver correctly maps overridden joint names to prim names."""
        from pxr import Sdf
        from sensor_msgs.msg import JointState

        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]
        override_names = ["center_joint", "slider_joint", "distal_joint"]

        for joint_path, override_name in zip(joint_paths, override_names):
            prim = self._stage.GetPrimAtPath(joint_path)
            attr = prim.GetAttribute("isaac:nameOverride")
            if not attr:
                attr = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
            attr.Set(override_name)

        graph_handle = og.get_graph_by_path("/ActionGraph")
        og.Controller.edit(
            graph_handle,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("JointNameResolver", "isaacsim.core.nodes.IsaacJointNameResolver"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointNameResolver.inputs:robotPath", "/Articulation"),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "/ActionGraph/SubscribeJointState.outputs:execOut",
                        "/ActionGraph/JointNameResolver.inputs:execIn",
                    ),
                    (
                        "/ActionGraph/SubscribeJointState.outputs:jointNames",
                        "/ActionGraph/JointNameResolver.inputs:jointNames",
                    ),
                    (
                        "/ActionGraph/JointNameResolver.outputs:execOut",
                        "/ActionGraph/ArticulationController.inputs:execIn",
                    ),
                    (
                        "/ActionGraph/JointNameResolver.outputs:jointNames",
                        "/ActionGraph/ArticulationController.inputs:jointNames",
                    ),
                ],
                og.Controller.Keys.DISCONNECT: [
                    ("/ActionGraph/OnPlaybackTick.outputs:tick", "/ActionGraph/ArticulationController.inputs:execIn"),
                ],
            },
        )
        await og.Controller.evaluate(graph_handle)

        ros2_node = self.create_node("isaac_sim_test_joint_state_name_override")
        ros2_publisher = self.create_publisher(ros2_node, JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]

        js = JointState()
        js.name = override_names
        js.position = test_position

        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=30)

        art_handle = Articulation("/Articulation")
        await self.simulate_until_condition(lambda: False, max_frames=30)

        ros2_publisher.publish(js)
        await self.simulate_until_condition(
            lambda: all(
                abs(art_handle.get_dof_positions().numpy().flatten()[i] - test_position[i]) < 0.001
                for i in range(len(test_position))
            ),
            max_frames=180,
        )

        joint_positions = art_handle.get_dof_positions().numpy().flatten()
        self.assertAlmostEqual(joint_positions[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_positions[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_positions[2], test_position[2], delta=1e-3)

        self._timeline.stop()
        ros2_node.destroy_publisher(ros2_publisher)
        ros2_node.destroy_node()
