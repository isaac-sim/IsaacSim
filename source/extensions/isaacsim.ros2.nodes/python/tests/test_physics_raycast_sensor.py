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

"""Verify physics raycast sensor ROS 2 publishers.

Covers point cloud and laser scan output for 2D, 3D, full-scan, intensity, and
depth-accuracy cases.
"""

import omni.graph.core as og
import omni.kit
import usdrt.Sdf
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from sensor_msgs_py.point_cloud2 import read_points

from .common import add_cube, create_raycast_lidar_sensor, get_qos_profile

HORIZONTAL_FOV = 360.0
HORIZONTAL_RESOLUTION = 0.4
NUM_RAYS = int(HORIZONTAL_FOV / HORIZONTAL_RESOLUTION)


class TestRos2PhysicsRaycastSensor(ROS2TestCase):
    """Verify raycast sensor point cloud and LaserScan ROS 2 publishers."""

    # -- Point Cloud tests --------------------------------------------------

    async def test_point_cloud_3d(self) -> None:
        """Test 3D point cloud from a multi-elevation raycast sensor."""
        import rclpy
        from sensor_msgs.msg import PointCloud2

        result, error = await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        self.assertTrue(result, f"Failed to load stage: {error}")
        await omni.kit.app.get_app().next_update_async()

        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=360.0,
            h_resolution=1.0,
            v_fov=30.0,
            v_count=8,
            min_range=0.4,
            max_range=100.0,
            translations=[[0.0, -0.5, 0.5]],
        )

        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                        ("PublishPCL.inputs:frameId", "my_custom_lidar_frame"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadRaycast.outputs:beamEndPoints", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2) -> None:
            self._point_cloud_data = data

        node = self.create_node("point_cloud_tester")
        self.create_subscription(node, PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "point_cloud", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._point_cloud_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.header.frame_id, "my_custom_lidar_frame")
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        points = read_points(self._point_cloud_data)
        self.assertGreater(len(list(points)), 0)

        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

    async def test_point_cloud_2d(self) -> None:
        """Test 2D point cloud from a single-elevation raycast sensor."""
        import rclpy
        from sensor_msgs.msg import PointCloud2

        result, error = await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        self.assertTrue(result, f"Failed to load stage: {error}")
        await omni.kit.app.get_app().next_update_async()

        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=360.0,
            h_resolution=1.0,
            min_range=0.4,
            max_range=100.0,
            translations=[[0.0, -0.5, 0.5]],
        )

        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadRaycast.outputs:beamEndPoints", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2) -> None:
            self._point_cloud_data = data

        node = self.create_node("flat_point_cloud_tester")
        self.create_subscription(node, PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "point_cloud", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._point_cloud_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )

        points = read_points(self._point_cloud_data)
        self.assertGreater(len(list(points)), 0)

        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

    async def test_point_cloud_full_scan(self) -> None:
        """Test full-scan point cloud with 900 horizontal rays."""
        import rclpy
        from sensor_msgs.msg import PointCloud2

        result, error = await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        self.assertTrue(result, f"Failed to load stage: {error}")
        await omni.kit.app.get_app().next_update_async()

        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=HORIZONTAL_FOV,
            h_resolution=HORIZONTAL_RESOLUTION,
            min_range=0.4,
            max_range=100.0,
            translations=[[0.0, -0.5, 0.5]],
        )

        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadRaycast.outputs:beamEndPoints", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2) -> None:
            self._point_cloud_data = data

        node = self.create_node("depth_point_cloud_tester")
        self.create_subscription(node, PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "point_cloud", per_frame_callback=spin)

        condition_met = await self.simulate_until_condition(
            lambda: self._point_cloud_data is not None, max_frames=120, per_frame_callback=spin
        )
        self.assertTrue(condition_met, "Failed to receive point cloud data within timeout")

        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )

        points = read_points(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        point_tuples = list(zip(points["x"], points["y"], points["z"]))
        self.assertEqual(len(set(point_tuples)), NUM_RAYS)

        self._timeline.stop()
        spin()

    # -- Laser Scan tests ---------------------------------------------------

    async def test_laser_scan(self) -> None:
        """Test laser scan publishing from a physics raycast sensor."""
        import rclpy
        from sensor_msgs.msg import LaserScan

        stage_utils.add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd", path="/World"
        )
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=HORIZONTAL_FOV,
            h_resolution=HORIZONTAL_RESOLUTION,
            min_range=0.4,
            max_range=100.0,
            translations=[[0.0, -0.5, 0.5]],
        )
        await omni.kit.app.get_app().next_update_async()

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("LaserScanPublisher", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                        ("LaserScanPublisher.inputs:timeStamp", 1.0),
                        ("LaserScanPublisher.inputs:horizontalFov", HORIZONTAL_FOV),
                        ("LaserScanPublisher.inputs:horizontalResolution", HORIZONTAL_RESOLUTION),
                        ("LaserScanPublisher.inputs:numCols", NUM_RAYS),
                        ("LaserScanPublisher.inputs:numRows", 1),
                        ("LaserScanPublisher.inputs:depthRange", [0.4, 100.0]),
                        ("LaserScanPublisher.inputs:rotationRate", 0.0),
                        ("LaserScanPublisher.inputs:azimuthRange", [-180.0, 180.0]),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "LaserScanPublisher.inputs:execIn"),
                        ("ReadRaycast.outputs:depths", "LaserScanPublisher.inputs:linearDepthData"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._lidar_data = None
        self._lidar_callback_count = 0

        def lidar_callback(data: LaserScan) -> None:
            self._lidar_callback_count += 1
            self._lidar_data = data
            self.assertGreater(data.angle_max, data.angle_min)
            self.assertEqual(len(data.ranges), NUM_RAYS)
            self.assertEqual(len(data.intensities), NUM_RAYS)

        node = self.create_node("lidar_tester")
        self.create_subscription(node, LaserScan, "scan", lidar_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        # 0.0 Hz rotation — time_increment should be 0
        self._lidar_callback_count = 0
        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "scan", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._lidar_callback_count > 0, max_frames=30, per_frame_callback=spin
        )
        self.assertGreater(self._lidar_callback_count, 0)
        self.assertEqual(self._lidar_data.time_increment, 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # 21.0 Hz rotation — time_increment should be positive
        og.Controller.set(og.Controller.attribute("/ActionGraph/LaserScanPublisher.inputs:rotationRate"), 21.0)

        await omni.kit.app.get_app().next_update_async()
        spin()
        self._lidar_callback_count = 0
        self._lidar_data = None
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "scan", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._lidar_callback_count > 0, max_frames=30, per_frame_callback=spin
        )
        self.assertGreater(self._lidar_callback_count, 0)
        self.assertGreater(self._lidar_data.time_increment, 0.0)

        prev_time_increment = self._lidar_data.time_increment

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # 201.0 Hz rotation — time_increment should be smaller than at 21 Hz
        og.Controller.set(og.Controller.attribute("/ActionGraph/LaserScanPublisher.inputs:rotationRate"), 201.0)

        await omni.kit.app.get_app().next_update_async()
        spin()
        self._lidar_callback_count = 0
        self._lidar_data = None
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "scan", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._lidar_callback_count > 0, max_frames=30, per_frame_callback=spin
        )
        self.assertGreater(self._lidar_callback_count, 0)
        self.assertGreater(prev_time_increment, self._lidar_data.time_increment)

        self._timeline.stop()
        spin()

    async def test_laser_scan_synthesized_intensities(self) -> None:
        """Verify synthesized intensities are binary hit/miss when intensitiesData is unconnected."""
        import rclpy
        from sensor_msgs.msg import LaserScan

        result, error = await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        self.assertTrue(result, f"Failed to load stage: {error}")
        await omni.kit.app.get_app().next_update_async()

        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=HORIZONTAL_FOV,
            h_resolution=HORIZONTAL_RESOLUTION,
            min_range=0.4,
            max_range=100.0,
            translations=[[0.0, -0.5, 0.5]],
        )
        await omni.kit.app.get_app().next_update_async()

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("LaserScanPublisher", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                        ("LaserScanPublisher.inputs:horizontalFov", HORIZONTAL_FOV),
                        ("LaserScanPublisher.inputs:horizontalResolution", HORIZONTAL_RESOLUTION),
                        ("LaserScanPublisher.inputs:numCols", NUM_RAYS),
                        ("LaserScanPublisher.inputs:numRows", 1),
                        ("LaserScanPublisher.inputs:depthRange", [0.4, 100.0]),
                        ("LaserScanPublisher.inputs:rotationRate", 0.0),
                        ("LaserScanPublisher.inputs:azimuthRange", [-180.0, 180.0]),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "LaserScanPublisher.inputs:execIn"),
                        ("ReadRaycast.outputs:depths", "LaserScanPublisher.inputs:linearDepthData"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._lidar_data = None

        def lidar_callback(data: LaserScan) -> None:
            self._lidar_data = data

        node = self.create_node("intensity_tester")
        self.create_subscription(node, LaserScan, "scan", lidar_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "scan", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._lidar_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.assertIsNotNone(self._lidar_data)
        self.assertEqual(len(self._lidar_data.intensities), NUM_RAYS)

        has_hit = False
        for i, intensity in enumerate(self._lidar_data.intensities):
            if self._lidar_data.ranges[i] < self._lidar_data.range_max:
                self.assertEqual(intensity, 255.0, f"Hit ray {i} should have intensity 255")
                has_hit = True
            else:
                self.assertEqual(intensity, 0.0, f"Miss ray {i} should have intensity 0")

        self.assertTrue(has_hit, "Expected at least one ray to hit geometry in simple_room")

        self.assertGreater(self._lidar_data.range_min, 0.0)
        self.assertGreater(self._lidar_data.range_max, self._lidar_data.range_min)

        self._timeline.stop()
        spin()

    async def test_point_cloud_depth_accuracy(self) -> None:
        """Verify point cloud positions are within expected range for a known scene."""
        import rclpy
        from sensor_msgs.msg import PointCloud2

        result, error = await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        self.assertTrue(result, f"Failed to load stage: {error}")
        await omni.kit.app.get_app().next_update_async()

        min_range = 0.4
        max_range = 100.0
        sensor_path = create_raycast_lidar_sensor(
            path="/World/Lidar",
            h_fov=360.0,
            h_resolution=1.0,
            min_range=min_range,
            max_range=max_range,
            translations=[[0.0, 0.0, 0.5]],
        )

        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadRaycast.inputs:execIn"),
                        ("ReadRaycast.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadRaycast.outputs:beamEndPoints", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2) -> None:
            self._point_cloud_data = data

        node = self.create_node("depth_accuracy_tester")
        self.create_subscription(node, PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await self.wait_for_publishers_on_topic(node, "point_cloud", per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._point_cloud_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.assertIsNotNone(self._point_cloud_data)

        import numpy as np

        points = read_points(self._point_cloud_data)
        xs = np.array(list(points["x"]))
        ys = np.array(list(points["y"]))
        zs = np.array(list(points["z"]))
        distances = np.sqrt(xs**2 + ys**2 + (zs - 0.5) ** 2)

        self.assertTrue(np.all(distances >= min_range - 0.01), "All points should be at least min_range from sensor")
        self.assertTrue(np.all(distances <= max_range + 0.01), "All points should be within max_range of sensor")

        self._timeline.stop()
        spin()
