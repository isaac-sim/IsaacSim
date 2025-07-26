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


import time

import carb.tokens
import numpy as np
import omni
import omni.graph.core as og
import omni.kit
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.utils.physics import simulate_async
from omni.syntheticdata import sensors
from pxr import UsdGeom, UsdPhysics

from .common import ROS2TestCase, fields_to_dtype, get_qos_profile


def add_cube(stage, path, scale, offset, physics=False):
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)
    cubeGeom.CreateSizeAttr(1.0)
    cubeGeom.AddTranslateOp().Set(offset)
    cubeGeom.AddScaleOp().Set(scale)
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    return cubePrim


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestROS2RTXSensor(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await omni.usd.get_context().new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._sensor_rate = 120
        self._stage = omni.usd.get_context().get_stage()

        self._sensor = None
        self._render_product = None
        self._render_product_path = None

        pass

    async def tearDown(self):
        await super().tearDown()

    async def _build_lidar_scene(self, sensor_config):
        _, self._sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar", path="/sim_lidar", config=sensor_config
        )
        self.assertEqual(self._sensor.GetTypeName(), "OmniLidar")
        self._sensor_prim_path = self._sensor.GetPath()

        # Create a render product for the sensor
        self._render_product = rep.create.render_product(
            self._sensor_prim_path, resolution=(128, 128), render_vars=["GenericModelOutput", "RtxSensorMetadata"]
        )
        self._render_product_path = self._render_product.path

        # Add cubes to the scene
        cube_prim = add_cube(self._stage, "/World/cube_1", (1, 20, 1), (5, 0, 0), physics=False)
        cube_prim = add_cube(self._stage, "/World/cube_2", (1, 20, 1), (-5, 0, 0), physics=False)
        cube_prim = add_cube(self._stage, "/World/cube_3", (20, 1, 1), (0, 5, 0), physics=False)
        cube_prim = add_cube(self._stage, "/World/cube_4", (20, 1, 1), (0, -5, 0), physics=False)
        cube_prim = add_cube(self._stage, "/World/cube_5", (1, 1, 1), (0, -4, 0), physics=False)

        return

    async def _test_rtx_lidar_point_cloud(
        self, sensor_config="Example_Rotary", enable_full_scan=False, use_system_time=False
    ):

        await self._build_lidar_scene(sensor_config=sensor_config)
        self.assertIsNotNone(self._sensor)
        self.assertIsNotNone(self._render_product)
        self.assertIsNotNone(self._render_product_path)

        # Create OmniGraph for sensor
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PCLPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PCLPublish.inputs:renderProductPath", self._render_product_path),
                        ("PCLPublish.inputs:topicName", "point_cloud"),
                        ("PCLPublish.inputs:type", "point_cloud"),
                        ("PCLPublish.inputs:resetSimulationTimeOnStop", True),
                        ("PCLPublish.inputs:fullScan", enable_full_scan),
                        ("PCLPublish.inputs:useSystemTime", use_system_time),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PCLPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # enable debug rendering for test purposes
        # rv = "RtxLidar"
        # writer = rep.writers.get(rv + "DebugDrawPointCloud")
        # writer.attach([self._render_product_path])

        import rclpy

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import LaserScan, PointCloud2

        self._pcl_data = None
        node = rclpy.create_node("rtx_lidar_tester")

        def pcl_callback(data):
            self._pcl_data = data

        pcl_sub = node.create_subscription(PointCloud2, "point_cloud", pcl_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._system_time = time.time()

        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._pcl_data is None:
                await simulate_async(1, callback=spin)
        self.assertIsNotNone(self._pcl_data)

        return

    async def _test_rtx_lidar_laser_scan(self, sensor_config="SICK_picoScan150", use_system_time=False):

        await self._build_lidar_scene(sensor_config=sensor_config)
        self.assertIsNotNone(self._sensor)
        self.assertIsNotNone(self._render_product)
        self.assertIsNotNone(self._render_product_path)

        # Create OmniGraph for sensor
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("FlatScanPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("FlatScanPublish.inputs:renderProductPath", self._render_product_path),
                        ("FlatScanPublish.inputs:topicName", "laser_scan"),
                        ("FlatScanPublish.inputs:type", "laser_scan"),
                        ("FlatScanPublish.inputs:resetSimulationTimeOnStop", True),
                        ("FlatScanPublish.inputs:useSystemTime", use_system_time),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "FlatScanPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # # enable debug rendering for test purposes
        # rv = "RtxLidar"
        # writer = rep.writers.get(rv + "DebugDrawPointCloud")
        # writer.attach([self._render_product_path])

        import rclpy

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import LaserScan, PointCloud2

        self._scan_data = None
        node = rclpy.create_node("rtx_lidar_tester")

        def scan_callback(data):
            self._scan_data = data

        scan_sub = node.create_subscription(LaserScan, "laser_scan", scan_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._system_time = time.time()

        await simulate_async(1, callback=spin)
        self.assertIsNotNone(self._scan_data)

        node.destroy_node()

        return

    async def test_rtx_lidar_point_cloud_full_scan_simulation_time(self):
        """Test publishing PointCloud2 from RTX Lidar with full scan enabled and using simulation time."""
        await self._test_rtx_lidar_point_cloud(enable_full_scan=True, use_system_time=False)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, 1)
        self.assertLess(self._pcl_data.header.stamp.sec, self._system_time / 2.0)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

    async def test_rtx_lidar_point_cloud_simulation_time(self):
        """Test publishing PointCloud2 from RTX Lidar with full scan disabled and using simulation time."""
        await self._test_rtx_lidar_point_cloud(enable_full_scan=False, use_system_time=False)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, 1)
        self.assertLess(self._pcl_data.header.stamp.sec, self._system_time / 2.0)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

    async def test_rtx_lidar_point_cloud_full_scan_system_time(self):
        """Test publishing PointCloud2 from RTX Lidar with full scan enabled and using system time."""
        await self._test_rtx_lidar_point_cloud(enable_full_scan=True, use_system_time=True)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, self._system_time)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

    async def test_rtx_lidar_point_cloud_system_time(self):
        """Test publishing PointCloud2 from RTX Lidar with full scan disabled and using system time."""
        await self._test_rtx_lidar_point_cloud(enable_full_scan=False, use_system_time=True)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, self._system_time)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

    async def test_rtx_lidar_laser_scan_simulation_time(self):
        """Test publishing LaserScan from RTX Lidar with full scan disabled and using simulation time."""
        await self._test_rtx_lidar_laser_scan(use_system_time=False)

        self.assertGreaterEqual(self._scan_data.header.stamp.sec, 1)
        self.assertLess(self._scan_data.header.stamp.sec, self._system_time / 2.0)

        # check laser scan
        self.assertGreater(len(self._scan_data.ranges), 10)

        for r in self._scan_data.ranges:
            if r < 0.0:
                continue  # invalid scan
            self.assertGreater(r, 3.49)  # this is the distance to the inset cube
            self.assertLess(r, 6.37)  # should be less than sqrt(4.5&2+4.5^2) ~ 6.3639

    async def test_rtx_lidar_laser_scan_system_time(self):
        """Test publishing LaserScan from RTX Lidar with full scan disabled and using system time."""
        await self._test_rtx_lidar_laser_scan(use_system_time=True)

        self.assertGreaterEqual(self._scan_data.header.stamp.sec, self._system_time)

        # check laser scan
        self.assertGreater(len(self._scan_data.ranges), 10)

        for r in self._scan_data.ranges:
            if r < 0.0:
                continue  # invalid scan
            self.assertGreater(r, 3.49)  # this is the distance to the inset cube
            self.assertLess(r, 6.38)  # should be less than sqrt(4.5&2+4.5^2) ~ 6.3639
