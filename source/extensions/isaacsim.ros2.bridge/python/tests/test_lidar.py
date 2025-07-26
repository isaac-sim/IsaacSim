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

import copy

import carb
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import ROS2TestCase, add_carter_ros, add_cube, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2Lidar(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await omni.usd.get_context().new_stage_async()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    # TODO: Carter V1 uses RTX lidar now
    # async def test_lidar(self):
    #     import rclpy
    #     from sensor_msgs.msg import LaserScan

    #     await add_carter_ros()
    #     await add_cube("/cube", 0.75, (2.00, 0, 0.75))

    #     self._lidar_data = None
    #     self._lidar_data_prev = None

    #     def lidar_callback(data: LaserScan):
    #         self._lidar_data = data

    #     node = rclpy.create_node("lidar_tester")
    #     subscriber = node.create_subscription(LaserScan, "scan", lidar_callback, get_qos_profile())

    #     def standard_checks():
    #         self.assertIsNotNone(self._lidar_data)
    #         self.assertGreater(self._lidar_data.angle_max, self._lidar_data.angle_min)
    #         self.assertEqual(self._lidar_data.intensities[0], 0.0)
    #         self.assertEqual(len(self._lidar_data.intensities), 900)
    #         self.assertEqual(self._lidar_data.intensities[450], 255.0)

    #     omni.kit.commands.execute(
    #         "ChangeProperty", prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"), value=0.0, prev=None
    #     )

    #     def spin():
    #         rclpy.spin_once(node, timeout_sec=0.1)

    #     # 0.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await simulate_async(1, 60, spin)

    #     standard_checks()
    #     self.assertEqual(self._lidar_data.time_increment, 0)

    #     self._timeline.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     self._lidar_data_prev = copy.deepcopy(self._lidar_data)
    #     self._lidar_data = None

    #     omni.kit.commands.execute(
    #         "ChangeProperty",
    #         prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
    #         value=121.0,
    #         prev=None,
    #     )

    #     await omni.kit.app.get_app().next_update_async()
    #     # 121.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await simulate_async(1, 60, spin)
    #     spin()

    #     standard_checks()

    #     self.assertGreater(self._lidar_data.header.stamp.sec, self._lidar_data_prev.header.stamp.sec)
    #     self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
    #     self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)
    #     self.assertGreater(self._lidar_data.time_increment, 0.0)

    #     self._timeline.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     self._lidar_data_prev = copy.deepcopy(self._lidar_data)
    #     self._lidar_data = None

    #     omni.kit.commands.execute(
    #         "ChangeProperty",
    #         prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
    #         value=201.0,
    #         prev=None,
    #     )

    #     # 201.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()

    #     await simulate_async(1, 60, spin)

    #     standard_checks()

    #     self.assertGreater(self._lidar_data.header.stamp.sec, self._lidar_data_prev.header.stamp.sec)
    #     self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
    #     self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)

    #     self.assertGreater(self._lidar_data_prev.time_increment, self._lidar_data.time_increment)

    #     self._timeline.stop()
    #     spin()

    #     pass

    async def test_lidar_buffer(self):
        # Test Lidar buffer with replicator activated
        import omni.graph.core as og
        import rclpy
        from sensor_msgs.msg import LaserScan

        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )

        # Make sure the stage loaded
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=True,
        )
        lidarPath = str(lidar.GetPath())
        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, -0.5, 0.5))

        # Setup a camera_helper to activate replicator to test PhysX Lidar buffer
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("LidarLaserScanNode", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                        ("LaserScanPublisher", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                        ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("LidarLaserScanNode.inputs:lidarPrim", [usdrt.Sdf.Path(lidarPath)]),
                        ("CameraHelper.inputs:renderProductPath", render_product_path),
                        ("LaserScanPublisher.inputs:timeStamp", 1.0),  # Use a non zero timestamp for test
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "LidarLaserScanNode.inputs:execIn"),
                        ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("LidarLaserScanNode.outputs:execOut", "LaserScanPublisher.inputs:execIn"),
                        ("LidarLaserScanNode.outputs:azimuthRange", "LaserScanPublisher.inputs:azimuthRange"),
                        ("LidarLaserScanNode.outputs:depthRange", "LaserScanPublisher.inputs:depthRange"),
                        ("LidarLaserScanNode.outputs:horizontalFov", "LaserScanPublisher.inputs:horizontalFov"),
                        (
                            "LidarLaserScanNode.outputs:horizontalResolution",
                            "LaserScanPublisher.inputs:horizontalResolution",
                        ),
                        ("LidarLaserScanNode.outputs:intensitiesData", "LaserScanPublisher.inputs:intensitiesData"),
                        ("LidarLaserScanNode.outputs:linearDepthData", "LaserScanPublisher.inputs:linearDepthData"),
                        ("LidarLaserScanNode.outputs:numCols", "LaserScanPublisher.inputs:numCols"),
                        ("LidarLaserScanNode.outputs:numRows", "LaserScanPublisher.inputs:numRows"),
                        ("LidarLaserScanNode.outputs:rotationRate", "LaserScanPublisher.inputs:rotationRate"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._lidar_data = None
        self._lidar_data_prev = None

        def lidar_callback(data: LaserScan):
            self._lidar_data = data

        node = rclpy.create_node("lidar_tester")
        subscriber = node.create_subscription(LaserScan, "scan", lidar_callback, get_qos_profile())

        def standard_checks():
            self.assertIsNotNone(self._lidar_data)
            self.assertGreater(self._lidar_data.angle_max, self._lidar_data.angle_min)
            self.assertEqual(len(self._lidar_data.intensities), 900)

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path(lidarPath + ".rotation_rate"), value=0.0, prev=None
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        # 0.0 Hz Lidar rotation
        self._timeline.play()
        await simulate_async(0.5, 60, spin)

        standard_checks()
        self.assertEqual(self._lidar_data.time_increment, 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        # 21.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=21.0,
            prev=None,
        )

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5, 60, spin)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)
        self.assertGreater(self._lidar_data.time_increment, 0.0)

        self.assertEqual(len(self._lidar_data.ranges), len(self._lidar_data_prev.ranges))
        self.assertEqual(self._lidar_data.ranges, self._lidar_data_prev.ranges)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        # 201.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=201.0,
            prev=None,
        )

        self._timeline.play()
        await simulate_async(0.5, 60, spin)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)

        self.assertGreater(self._lidar_data_prev.time_increment, self._lidar_data.time_increment)

        self.assertEqual(len(self._lidar_data.ranges), len(self._lidar_data_prev.ranges))
        self.assertEqual(self._lidar_data.ranges, self._lidar_data_prev.ranges)

        self._timeline.stop()
        spin()
        pass
