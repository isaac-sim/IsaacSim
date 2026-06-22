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

"""Tests for the Leatherback Ackermann-steering robot with ROS2 integration."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
from isaacsim.core.experimental.utils.stage import open_stage_async
from isaacsim.core.experimental.utils.xform import get_world_pose
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase


def get_qos_profile() -> object:
    """Get ROS2 QoS profile for sensor data subscription.

    Returns:
        QoSProfile configured for best-effort reliability with keep-last history.
    """
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class TestLeatherback(ROS2TestCase):
    """Tests for the Leatherback Ackermann-steering robot with ROS2 integration."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with Leatherback robot and ROS2."""
        await super().setUp()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.usd_path = self._assets_root_path + "/Isaac/Samples/ROS2/Robots/leatherback_ROS.usd"
        result, error = await open_stage_async(self.usd_path)

        # Make sure the stage loaded
        self.assertTrue(result, error)

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_drive_forward(self) -> None:
        """Test that Leatherback drives forward via ROS2 Ackermann commands."""
        from ackermann_msgs.msg import AckermannDriveStamped

        # Create a node to publish ackermann messages
        node = self.create_node("isaac_sim_test_leatherback")
        ros_topic = "/ackermann_cmd"
        test_pub = self.create_publisher(node, AckermannDriveStamped, ros_topic, 1)

        # Start Simulation and wait a second for it to settle
        self._timeline.play()
        await self.wait_for_subscribers_on_topic(test_pub)
        await self.simulate_until_condition(lambda: False, max_frames=60)

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
        # get_world_pose returns tuple of wp.arrays: (position, orientation)
        pos_arr, _ = get_world_pose("/Leatherback/Rigid_Bodies/Chassis")
        x = pos_arr.numpy()

        delta = np.linalg.norm(x - target)
        self.assertAlmostEqual(delta, 0, delta=0.01, msg=f"delta: {delta}, target: {target}, actual: {x}")

    async def test_cameras(self) -> None:
        """Test that RGB and depth cameras publish valid data via ROS2."""
        import rclpy
        from sensor_msgs.msg import Image

        # Create a node to subscribe to camera messages.
        node = self.create_node("isaac_sim_test_leatherback")

        self._rgb = None
        self._depth = None

        def rgb_callback(data: object) -> None:
            self._rgb = data

        def depth_callback(data: object) -> None:
            self._depth = data

        ros_rgb_topic = "/rgb"
        ros_depth_topic = "/depth"

        self.create_subscription(node, Image, ros_rgb_topic, rgb_callback, get_qos_profile())
        self.create_subscription(node, Image, ros_depth_topic, depth_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Start Simulation and wait
        self._timeline.play()
        # Multitick rendering can delay ROS 2 camera publisher discovery and first-real-frame
        # under warm-cache runs; extend wait + drain extra frames.
        await self.wait_for_publishers_on_topic(node, ros_rgb_topic, timeout_sec=30.0, per_frame_callback=spin)
        await self.wait_for_publishers_on_topic(node, ros_depth_topic, timeout_sec=30.0, per_frame_callback=spin)

        condition_met = await self.simulate_until_condition(
            lambda: self._rgb is not None and self._depth is not None,
            max_frames=600,
            per_frame_callback=spin,
        )
        # Drain additional frames so the camera sensor has ticked past any sentinel/zero
        # first-frame buffer republished by the writer under multitick.
        if condition_met:
            # Drain additional frames so the camera sensor has ticked past any sentinel/zero
            # first-frame buffer republished by the writer under multitick.
            await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)
        self.assertTrue(condition_met, "No RGB/depth data received from Leatherback cameras within timeout")
        self.assertIsNotNone(self._rgb, "No RGB data received from Leatherback camera")
        self.assertIsNotNone(self._depth, "No depth data received from Leatherback camera")

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
        self.assertTrue(
            np.linalg.norm(rgb_diff) < 15,
            msg=f"rgb_diff: {rgb_diff}, np.linalg.norm(rgb_diff): {np.linalg.norm(rgb_diff)}",
        )

        # The first pixel corresponds to the sky, it should have an infinite depth
        # The last pixel is on the ground, it should be around 0.602
        self.assertTrue(np.isinf(depth_data[0]))
        self.assertTrue(np.abs(depth_data[-1] - 0.602) < 0.1)
