# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from numpy import pi as PI

from .common import ROS2TestCase, get_qos_profile, set_joint_drive_parameters


class TestRos2JointStatePublisher(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await omni.usd.get_context().new_stage_async()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        await omni.kit.app.get_app().next_update_async()
        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()
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

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    async def test_joint_state_position_publisher(self):
        import rclpy
        from sensor_msgs.msg import JointState

        # setup ROS listener of the joint_state topic
        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.position = data.position
            self.js_ros.velocity = data.velocity
            self.js_ros.effort = data.effort

        node = rclpy.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = node.create_subscription(JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        default_position = [-80 * PI / 180.0, 0.4, 30 * PI / 180.0]

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)
        received_position = self.js_ros.position

        print("\n received_position", received_position)

        self.assertAlmostEqual(received_position[0], default_position[0], delta=1e-3)
        self.assertAlmostEqual(received_position[1], default_position[1], delta=1e-3)
        self.assertAlmostEqual(received_position[2], default_position[2], delta=1e-3)

        self._timeline.stop()
        spin()

        pass

    async def test_joint_state_velocity_publisher(self):
        import rclpy
        from sensor_msgs.msg import JointState

        # setup ROS listener of the joint_state topic
        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.velocity = data.velocity

        node = rclpy.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = node.create_subscription(JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

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

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", test_velocities[i], joint_stiffness, joint_damping
            )

        # tick and get ros sub
        # # set the stiffness and damping parameters accordingly for position control
        self._timeline.play()

        await simulate_async(2, 60, spin)
        received_velocity = self.js_ros.velocity

        comp_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]

        # print("test_velocities_radian", comp_velocity)

        self.assertAlmostEqual(received_velocity[0], comp_velocity[0], delta=1e-3)
        self.assertAlmostEqual(received_velocity[1], comp_velocity[1], delta=1e-3)
        self.assertAlmostEqual(received_velocity[2], comp_velocity[2], delta=1e-3)
        self._timeline.stop()
        spin()


class TestRos2JointStateSubscriber(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        await omni.kit.app.get_app().next_update_async()
        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()
        # ROS-ify asset by adding a joint state publisher
        # setup the graph
        try:
            (test_graph, new_nodes, _, _) = og.Controller.edit(
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

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    async def test_joint_state_subscriber_node(self):
        """
        test if the joint state subscriber node is able to receive the joint state commands
        """
        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        js_position.velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        js_position.effort = [0.4, -0.2, 0.3]

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5)

        # publish value
        ros2_publisher.publish(js_position)
        await simulate_async(0.5)

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
        """
        test if the joint state subscriber is able to move the robot as expected
        """
        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await simulate_async(0.5)

        # get the articulation review for the asset
        art_handle = SingleArticulation("/Articulation")
        art_handle.initialize()
        await simulate_async(0.5)

        def reset_robot():
            # reset the robot to 0s
            art_handle.set_joint_positions([0, 0, 0])
            # give it a second to move
            post_reset = art_handle.get_joint_positions()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_positions()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        # change joint drives for velocity drive test
        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]

        joint_types = ["angular", "linear", "angular"]
        target_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", target_velocities[i], joint_stiffness, joint_damping
            )

        # test velocity drive
        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_velocities()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        # change prismatic joint back to postion drive
        set_joint_drive_parameters(joint_paths[1], joint_types[1], "position", 0.2, 1e5, 1e4)

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # give it a second to move
        await simulate_async(2)

        joint_position_received = art_handle.get_joint_positions()
        joint_velocity_received = art_handle.get_joint_velocities()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=1e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)

    async def test_joint_state_subscriber_with_names(self):
        """
        test if the joint state subscriber is able to move the robot as expected
        """

        # add the connection between joint names from subscriber and the controller
        graph_handle = og.get_graph_by_path("/ActionGraph")
        og.Controller.connect(
            "/ActionGraph/SubscribeJointState.outputs:jointNames",
            "/ActionGraph/ArticulationController.inputs:jointNames",
        )
        await og.Controller.evaluate(graph_handle)

        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await simulate_async(0.5)

        # get the articulation review for the asset
        art_handle = SingleArticulation("/Articulation")
        art_handle.initialize()
        await simulate_async(0.5)

        def reset_robot():
            # reset the robot to 0s
            art_handle.set_joint_positions([0, 0, 0])
            # give it a second to move
            post_reset = art_handle.get_joint_positions()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_positions()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        # change joint drives for velocity drive test
        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]

        joint_types = ["angular", "linear", "angular"]
        target_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", target_velocities[i], joint_stiffness, joint_damping
            )

        # test velocity drive
        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_velocities()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        # change prismatic joint back to postion drive
        set_joint_drive_parameters(joint_paths[1], joint_types[1], "position", 0.2, 1e5, 1e4)

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # give it a second to move
        await simulate_async(2)

        joint_position_received = art_handle.get_joint_positions()
        joint_velocity_received = art_handle.get_joint_velocities()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=1e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)
