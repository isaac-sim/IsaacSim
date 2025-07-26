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
import carb.tokens
import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.core.api import World
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.storage.native import get_assets_root_path_async


def get_qos_profile():
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestLeatherback(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Samples/ROS2/Robots/leatherback_ROS.usd"
        (result, error) = await open_stage_async(self.usd_path)

        # Make sure the stage loaded
        self.assertTrue(result)
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()

        rclpy.init()

        pass

    # After running each test
    async def tearDown(self):
        import rclpy

        self.my_world.stop()
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

        rclpy.shutdown()

        pass

    async def test_drive_forward(self):
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped

        # Create a node to publish ackermann messages
        node = rclpy.create_node("isaac_sim_test_leatherback")
        ros_topic = "/ackermann_cmd"
        test_pub = node.create_publisher(AckermannDriveStamped, ros_topic, 1)

        # Start Simulation and wait a second for it to settle
        self.my_world.play()
        await simulate_async(1)

        # Drive the robot
        for i in range(60):

            msg = AckermannDriveStamped()
            msg.drive.speed = 0.2
            test_pub.publish(msg)

            await omni.kit.app.get_app().next_update_async()

        # Let the robot come to a halt
        for i in range(60):

            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            test_pub.publish(msg)

            await omni.kit.app.get_app().next_update_async()

        target = np.array([0.17790918, -0.00121224, 0.0291148])
        x, r = get_world_pose("/Leatherback/Rigid_Bodies/Chassis")

        delta = np.linalg.norm(x - target)
        self.assertAlmostEqual(delta, 0, delta=0.01, msg=f"delta: {delta}, target: {target}, actual: {x}")

        pass

    async def test_cameras(self):

        import rclpy
        from sensor_msgs.msg import Image

        # Create a node to publish ackermann messages
        node = rclpy.create_node("isaac_sim_test_leatherback")

        self._rgb = None
        self._depth = None

        def rgb_callback(data):
            self._rgb = data

        def depth_callback(data):
            self._depth = data

        ros_rgb_topic = "/rgb"
        ros_depth_topic = "/depth"

        rgb_sub = node.create_subscription(Image, ros_rgb_topic, rgb_callback, get_qos_profile())
        depth_sub = node.create_subscription(Image, ros_depth_topic, depth_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        # Start Simulation and wait
        self.my_world.play()

        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._rgb is None:
                await simulate_async(1, callback=spin)

        depth_data = np.frombuffer(self._depth.data, dtype=np.float32)
        rgb_data = np.frombuffer(self._rgb.data, dtype=np.uint8)
        rgb_data = np.reshape(rgb_data, (self._rgb.height, self._rgb.width, 3))

        # Verify encoding and dimensions
        self.assertEqual(self._rgb.width, 1280)
        self.assertEqual(self._rgb.height, 720)
        self.assertEqual(self._rgb.encoding, "rgb8")
        self.assertEqual(self._depth.width, 1280)
        self.assertEqual(self._depth.height, 720)
        self.assertEqual(self._depth.encoding, "32FC1")

        # Look for a specific gray pixel in the bottom of the RGB image
        rgb_diff = np.array([203, 203, 203]) - rgb_data[self._rgb.height - 1, self._rgb.width // 2]
        self.assertTrue(np.linalg.norm(rgb_diff) < 10)

        # The first pixel corresponds to the sky, it should have an infinite depth
        # The last pixel is on the ground, it should be around 0.602
        self.assertTrue(np.isinf(depth_data[0]))
        self.assertTrue(np.abs(depth_data[-1] - 0.602) < 0.1)

        node.destroy_node()
        pass
