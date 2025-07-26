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

from copy import deepcopy

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from re import A, I

import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.utils.physics import simulate_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom

from .common import (
    ROS2TestCase,
    add_carter,
    add_carter_ros,
    add_nova_carter_ros,
    get_qos_profile,
    set_rotate,
    set_translate,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2DifferentialBase(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):

        await super().tearDown()

    async def test_carter_differential_base(self):

        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_carter_ros()
        stage = omni.usd.get_context().get_stage()

        meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

        # add an odom prim to carter
        odom_prim = stage.DefinePrim("/Carter/chassis_link/odom", "Xform")

        graph_path = "/Carter/ActionGraph"

        # add an tf publisher for world->odom
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "tf_test"),
                        ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/Carter/chassis_link/odom")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
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

        self._odom_data = None
        self.trans = None

        node = rclpy.create_node("isaac_sim_test_diff_drive")

        def tf_callback(data: TFMessage):
            self.trans = data.transforms[-1]

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        tf_sub = node.create_subscription(TFMessage, "tf_test", tf_callback, get_qos_profile())
        odom_sub = node.create_subscription(Odometry, "odom", odom_callback, get_qos_profile())
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        def check_odom_initial_pose():
            odom_data = deepcopy(self._odom_data)

            self.assertIsNotNone(self._odom_data)
            self.assertAlmostEqual(self.trans.transform.translation.x, 1.0 * meters_per_unit, 2)
            self.assertAlmostEqual(self.trans.transform.translation.y, -3.0 * meters_per_unit, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.z, 0.38268, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9238, 2)
            self.assertAlmostEqual(odom_data.position.x, 0, 1)
            self.assertAlmostEqual(odom_data.position.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # check 0: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(round(self.trans.transform.translation.x, 2), 1.21 * meters_per_unit, 2)
        self.assertAlmostEqual(round(self.trans.transform.translation.y, 2), -2.79 * meters_per_unit, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.z, 0.38268, 1)
        self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9238, 1)
        self.assertAlmostEqual(round(odom_data.position.x, 1), 0.3, 1)
        self.assertAlmostEqual(round(odom_data.position.y, 1), 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self.trans = None

        # change wheel rotation and wheel base
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelRadius"), 0.1)
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelDistance"), 0.5)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await simulate_async(1, 60, spin)

        # check 3: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 4: location after change radius
        odom_data = deepcopy(self._odom_data)

        self.assertAlmostEqual(round(self.trans.transform.translation.x, 2), 1.51 * meters_per_unit, 2)
        self.assertAlmostEqual(round(self.trans.transform.translation.y, 2), -2.49 * meters_per_unit, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.z, 0.3846, 1)
        self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9230, 1)
        self.assertAlmostEqual(round(odom_data.position.x, 1), 0.7, 1)
        self.assertAlmostEqual(round(odom_data.position.y, 1), 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()
        spin()
        pass

    # add carter and ROS topic from scratch
    async def test_differential_base_scratch(self):

        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        await add_carter()

        self._odom_data = None

        node = rclpy.create_node("isaac_sim_test_diff_drive_manual")

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        odom_sub = node.create_subscription(Odometry, "odom", odom_callback, 10)
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        graph_path = "/ActionGraph"
        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # check 0: is carter initially stationary
        odom_data = deepcopy(self._odom_data)
        self.assertIsNotNone(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # rotate
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0.40, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.916, 1)

        # change wheel rotation and wheel base
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelDistance").set(0.8)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # rotate back
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, -0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 3: location after change radius
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, -0.61, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.79, 1)

        self._timeline.stop()
        spin()
        pass

    async def test_nova_carter_differential_base(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_nova_carter_ros()
        stage = omni.usd.get_context().get_stage()

        meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

        graph_path = "/nova_carter_ros2_sensors/transform_tree_odometry"
        drive_graph_path = "/nova_carter_ros2_sensors/differential_drive"
        # add an tf publisher for world->base_link
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [usdrt.Sdf.Path("/nova_carter_ros2_sensors/chassis_link/base_link")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # move carter off origin
        carter_prim = stage.GetPrimAtPath("/nova_carter_ros2_sensors")
        new_translate = Gf.Vec3d(1.00, -3.00, 0.0)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)

        self._odom_data = None
        self.trans = None

        node = rclpy.create_node("isaac_sim_test_diff_drive")

        def tf_callback(data: TFMessage):
            self.trans = data.transforms[-1]

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        tf_sub = node.create_subscription(TFMessage, "tf_test", tf_callback, get_qos_profile())
        odom_sub = node.create_subscription(Odometry, "chassis/odom", odom_callback, get_qos_profile())
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        def check_odom_initial_pose():
            odom_data = deepcopy(self._odom_data)

            self.assertIsNotNone(self._odom_data)
            self.assertAlmostEqual(self.trans.transform.translation.x, 1.0 * meters_per_unit, 2)
            self.assertAlmostEqual(self.trans.transform.translation.y, -3.0 * meters_per_unit, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.z, 0.38268, 2)
            self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9238, 2)
            self.assertAlmostEqual(odom_data.position.x, 0, 1)
            self.assertAlmostEqual(odom_data.position.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # check 0: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)

        carb.log_info(str(self.trans.transform))
        carb.log_info(str(odom_data))
        self.assertAlmostEqual(self.trans.transform.translation.x, 1.22 * meters_per_unit, 2)
        self.assertAlmostEqual(self.trans.transform.translation.y, -2.78 * meters_per_unit, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.z, 0.38268, 1)
        self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9238, 1)
        self.assertAlmostEqual(odom_data.position.x, 0.3, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self.trans = None

        # change wheel rotation and wheel base
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelRadius"), 0.1
        )
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelDistance"), 0.5
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await simulate_async(1, 60, spin)

        # check 3: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 4: location after change radius
        odom_data = deepcopy(self._odom_data)
        carb.log_info(str(self.trans.transform))
        carb.log_info(str(odom_data))
        self.assertAlmostEqual(self.trans.transform.translation.x, 1.30 * meters_per_unit, delta=0.01)
        self.assertAlmostEqual(self.trans.transform.translation.y, -2.69 * meters_per_unit, delta=0.01)
        self.assertAlmostEqual(self.trans.transform.rotation.x, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.y, 0, 2)
        self.assertAlmostEqual(self.trans.transform.rotation.z, 0.3815, 1)
        self.assertAlmostEqual(self.trans.transform.rotation.w, 0.9244, 1)
        self.assertAlmostEqual(odom_data.position.x, 0.43, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()
        spin()
        pass

    # add carter and ROS topic from scratch
    async def test_differential_base_scratch(self):

        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        await add_carter()

        self._odom_data = None

        node = rclpy.create_node("isaac_sim_test_diff_drive_manual")

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        odom_sub = node.create_subscription(Odometry, "odom", odom_callback, 10)
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        graph_path = "/ActionGraph"
        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3, 60, spin)

        # check 0: is carter initially stationary
        odom_data = deepcopy(self._odom_data)
        self.assertIsNotNone(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # rotate
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0.40, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.916, 1)

        # change wheel rotation and wheel base
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelDistance").set(0.8)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # rotate back
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, -0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # check 3: location after change radius
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, -0.61, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.79, 1)

        self._timeline.stop()
        spin()
        pass

    def add_differential_drive(self, graph_path):

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
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
                        ("OnPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
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
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path("/carter")]),
                        ("artController.inputs:targetPrim", [usdrt.Sdf.Path("/carter")]),
                    ],
                },
            )
        except Exception as e:
            print(e)

        return graph, nodes
