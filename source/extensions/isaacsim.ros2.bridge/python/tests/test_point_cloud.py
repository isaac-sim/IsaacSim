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
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
import usdrt.Sdf
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import ROS2TestCase, add_carter, add_carter_ros, add_cube, fields_to_dtype, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2PointCloud(ROS2TestCase):
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

    async def test_3D_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path("/carter/chassis_link/carter_lidar")])
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Enable highLod for Lidar
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/carter/chassis_link/carter_lidar.highLod"), value=True, prev=None
        )

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("point_cloud_tester")
        lidar_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # If 3D point cloud (highLOD enabled)
        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
        arr = np.frombuffer(self._point_cloud_data.data, ff)

        self.assertAlmostEqual(arr[100][0], -45.083733, delta=0.01)
        self.assertAlmostEqual(arr[100][1], -7.949485, delta=0.01)
        self.assertAlmostEqual(arr[100][2], -0.7990794, delta=0.01)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_flat_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path("/carter/chassis_link/carter_lidar")])
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("flat_point_cloud_tester")
        lidar_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # If flat point cloud (highLOD disabled)
        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )

        ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
        arr = np.frombuffer(self._point_cloud_data.data, ff)

        self.assertAlmostEqual(arr[50][0], 1.257611, delta=0.01)
        self.assertAlmostEqual(arr[50][1], 0.149961, delta=0.01)
        self.assertAlmostEqual(arr[50][2], -0.000000, delta=0.01)

        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_depth_to_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter_ros()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        graph_path = "/Carter/ROS_Cameras"

        # Disabling left camera rgb image publisher
        og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled").set(False)

        # Add Point Cloud publisher in ROS Camera
        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                graph_path,
                {
                    keys.CREATE_NODES: [("depthToPCL", "isaacsim.ros2.bridge.ROS2CameraHelper")],
                    keys.CONNECT: [
                        (graph_path + "/isaac_create_render_product_left.outputs:execOut", "depthToPCL.inputs:execIn"),
                        (graph_path + "/camera_frameId_left.inputs:value", "depthToPCL.inputs:frameId"),
                        (
                            graph_path + "/isaac_create_render_product_left.outputs:renderProductPath",
                            "depthToPCL.inputs:renderProductPath",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("depthToPCL.inputs:topicName", "/point_cloud_left"),
                        ("depthToPCL.inputs:type", "depth_pcl"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Enable left camera pipeline
        og.Controller.set(
            og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled"), True
        )

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("depth_point_cloud_tester")
        camera_sub = node.create_subscription(PointCloud2, "point_cloud_left", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        self.assertIsNotNone(self._point_cloud_data)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        self.assertEqual(self._point_cloud_data.data[526327], 190)
        self.assertEqual(self._point_cloud_data.data[712187], 63)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_flat_point_cloud_buffer(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )

        # Make sure the stage loaded
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        HORIZONTAL_FOV = 360.0

        HORIZONTAL_RESOLUTION = 0.4

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=HORIZONTAL_FOV,
            vertical_fov=30.0,
            horizontal_resolution=HORIZONTAL_RESOLUTION,
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

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                        ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path(lidarPath)]),
                        ("CameraHelper.inputs:renderProductPath", render_product_path),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("depth_point_cloud_tester")
        camera_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        def standard_checks():
            # If flat point cloud (highLOD disabled)
            self.assertIsNotNone(self._point_cloud_data)
            self.assertEqual(self._point_cloud_data.height, 1)
            self.assertGreater(self._point_cloud_data.width, 1)
            self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
            self.assertEqual(
                self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
            )

            ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
            arr = np.frombuffer(self._point_cloud_data.data, ff)

            self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
            self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
            self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

            def fix_data(arr):
                dat = []
                for x in arr:
                    dat.append((x[0], x[1], x[2]))
                return dat

            arr = fix_data(arr)

            # Check to see if number of points matches number of beams in full scan
            self.assertEqual(len(set(arr)), HORIZONTAL_FOV / HORIZONTAL_RESOLUTION)

        # Check with lidar 0.0 rotation rate
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=0.0,
            prev=None,
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # 21.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=21.0,
            prev=None,
        )

        self._timeline.play()
        await simulate_async(2.0, 60, spin)

        standard_checks()
        self._timeline.stop()
        spin()
        pass
