# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
import math

import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import omni.kit.test
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.semantics import add_labels
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport


class TestCameraSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self.my_world.scene.add_default_ground_plane()
        self.cube_2 = self.my_world.scene.add(
            DynamicCuboid(
                prim_path="/new_cube_2",
                name="cube_1",
                position=np.array([5.0, 3, 1.0]),
                scale=np.array([0.6, 0.5, 0.2]),
                size=1.0,
                color=np.array([255, 0, 0]),
            )
        )

        self.cube_3 = self.my_world.scene.add(
            DynamicCuboid(
                prim_path="/new_cube_3",
                name="cube_2",
                position=np.array([-5, 1, 3.0]),
                scale=np.array([0.1, 0.1, 0.1]),
                size=1.0,
                color=np.array([0, 0, 255]),
                linear_velocity=np.array([0, 0, 0.4]),
            )
        )
        self.xform = self.my_world.scene.add(
            SingleXFormPrim(
                prim_path="/World/rig",
                name="rig",
                position=np.array([5.0, 0.0, 5.0]),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, -90, 0]), degrees=True),
            )
        )
        self.camera = self.my_world.scene.add(
            Camera(
                prim_path="/World/rig/camera",
                name="camera",
                position=np.array([0.0, 0.0, 25.0]),
                frequency=20,
                resolution=(256, 256),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
            )
        )
        add_labels(self.cube_2.prim, labels=["cube"], instance_name="class")
        add_labels(self.cube_3.prim, labels=["cube"], instance_name="class")
        await update_stage_async()
        await update_stage_async()
        await self.my_world.reset_async()
        await update_stage_async()
        await update_stage_async()
        return

    # After running each test
    async def tearDown(self):
        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()
        self.camera = None
        self.viewport_camera = None
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_world_poses(self):
        position, orientation = self.camera.get_world_pose()
        self.assertTrue(np.allclose(position, [0, 0, 25], atol=1e-05))
        self.assertTrue(
            np.allclose(orientation, rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), atol=1e-05)
        )
        translation, orientation = self.camera.get_local_pose()
        self.assertTrue(np.allclose(translation, [20, 0, 5], atol=1e-05))
        self.assertTrue(
            np.allclose(orientation, rot_utils.euler_angles_to_quats(np.array([0, 180, 0]), degrees=True), atol=1e-05)
        )
        self.camera.set_local_pose(
            translation=[0, 0, 25], orientation=rot_utils.euler_angles_to_quats(np.array([0, 180, 0]), degrees=True)
        )
        return

    async def test_local_poses(self):
        return

    async def test_projection(self):
        await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 100)
        points_2d = self.camera.get_image_coords_from_world_points(
            np.array([self.cube_3.get_world_pose()[0], self.cube_2.get_world_pose()[0]])
        )
        # visual inspection golden values
        self.assertTrue(np.allclose(points_2d[0], [103.265594, 250.42622], atol=0.05), f"points_2d[0]: {points_2d[0]}")
        self.assertTrue(np.allclose(points_2d[1], [54.414757, 5.2186475], atol=0.05), f"points_2d[1]: {points_2d[1]}")
        points_3d = self.camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        self.assertTrue(np.allclose(points_3d[0], [-5, 1, 0.06], atol=0.05), f"points_3d[0]: {points_3d[0]}")
        self.assertTrue(np.allclose(points_3d[1], [5, 3, 0.1], atol=0.05), f"points_3d[1]: {points_3d[1]}")
        return

    async def test_data_acquisition(self):
        await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 1)
        self.camera.resume()
        for annotator in self.camera.supported_annotators:
            getattr(self.camera, "add_{}_to_frame".format(annotator))()
            # frequency is set to 20, rendering rate is set to 120, so do 6 updates to make sure always have a frame
            await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 10)
            data = self.camera.get_current_frame()
            self.assertTrue(len(data[annotator]) > 0, f"{annotator}")
            if isinstance(data[annotator], dict) and "data" in data[annotator]:
                self.assertTrue(len(data[annotator]["data"]) > 0, f"check for data in {annotator}")
            getattr(self.camera, "remove_{}_from_frame".format(annotator))()
            await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 1)
            data = self.camera.get_current_frame()
            self.assertTrue(annotator not in data.keys(), f"{annotator}")
        return

    async def test_focal_length(self):
        """Test setting and getting focal length."""
        self.camera.set_focal_length(5.0)
        self.assertAlmostEqual(self.camera.get_focal_length(), 5.0)

    async def test_focus_distance(self):
        """Test setting and getting focus distance."""
        self.camera.set_focus_distance(0.01)
        self.assertAlmostEqual(self.camera.get_focus_distance(), 0.01, delta=0.005)

    async def test_lens_aperture(self):
        """Test setting and getting lens aperture."""
        self.camera.set_lens_aperture(0.01)
        self.assertAlmostEqual(self.camera.get_lens_aperture(), 0.01, delta=0.005)

    async def test_horizontal_aperture(self):
        """Test setting and getting horizontal aperture."""
        self.camera.set_horizontal_aperture(1.2)
        self.assertAlmostEqual(self.camera.get_horizontal_aperture(), 1.2, delta=0.1)

    async def test_vertical_aperture(self):
        """Test setting and getting vertical aperture."""
        self.camera.set_vertical_aperture(1.2)
        self.assertAlmostEqual(self.camera.get_vertical_aperture(), 1.2, delta=0.1)

    async def test_clipping_range(self):
        """Test setting and getting clipping range."""
        self.camera.set_clipping_range(1.0, 1.0e5)
        clipping_range = self.camera.get_clipping_range()
        self.assertAlmostEqual(clipping_range[0], 1.0, delta=0.1)
        self.assertAlmostEqual(clipping_range[1], 1.0e5, delta=0.1)

    async def test_projection_type(self):
        """Test setting and getting projection type."""
        self.camera.set_projection_type("fisheyeOrthographic")
        self.assertTrue(self.camera.get_projection_type() == "fisheyeOrthographic")

    async def test_stereo_role(self):
        """Test setting and getting stereo role."""
        self.camera.set_stereo_role("left")
        self.assertTrue(self.camera.get_stereo_role() == "left")

    async def test_shutter_properties(self):
        """Test setting and getting shutter properties."""
        self.camera.set_shutter_properties(delay_open=2.0, delay_close=3.0)
        delay_open, delay_close = self.camera.get_shutter_properties()
        self.assertAlmostEqual(delay_open, 2.0, delta=0.1)
        self.assertAlmostEqual(delay_close, 3.0, delta=0.1)

    async def test_resolution(self):
        """Test setting and getting resolution."""
        self.camera.set_resolution((300, 300))
        resolution = self.camera.get_resolution()
        self.assertEqual(resolution[0], 300)
        self.assertEqual(resolution[1], 300)

    async def test_aspect_ratio_and_fov(self):
        """Test getting aspect ratio and FOV values."""
        aspect_ratio = self.camera.get_aspect_ratio()
        self.assertTrue(isinstance(aspect_ratio, float))

        horizontal_fov = self.camera.get_horizontal_fov()
        self.assertTrue(isinstance(horizontal_fov, float))

        vertical_fov = self.camera.get_vertical_fov()
        self.assertTrue(isinstance(vertical_fov, float))

    async def test_viewport_camera(self):
        viewport_api = get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()

        self.viewport_camera = Camera(
            prim_path="/World/rig/viewport_camera",
            name="viewport_camera",
            position=np.array([0.0, 0.0, 25.0]),
            resolution=(256, 256),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
            render_product_path=render_product_path,
        )
        self.viewport_camera.initialize()
        self.viewport_camera.add_distance_to_image_plane_to_frame()
        self.viewport_camera.add_pointcloud_to_frame()

        await omni.syntheticdata.sensors.next_render_simulation_async(self.viewport_camera.get_render_product_path(), 1)
        self.assertEqual(self.viewport_camera.get_rgba().size, 256 * 256 * 4)
        self.assertEqual(self.viewport_camera.get_rgb().size, 256 * 256 * 3)
        self.assertEqual(self.viewport_camera.get_depth().size, 256 * 256 * 1)
        # self.assertEqual(self.viewport_camera.get_pointcloud().size, 256 * 256 * 3)
        self.assertEqual(self.viewport_camera.get_render_product_path(), render_product_path)

    async def test_get_current_frame(self):
        current_frame_1 = self.camera.get_current_frame()
        current_frame_2 = self.camera.get_current_frame()

        # Make sure that the two frames refer to the same object
        self.assertIs(current_frame_1, current_frame_2)

        current_frame_3 = self.camera.get_current_frame()
        current_frame_4 = self.camera.get_current_frame(clone=True)

        # Make sure that the two frames refer to different objects
        self.assertIsNot(current_frame_3, current_frame_4)

    async def test_annotators_data(self):
        # Add all annotators to the camera
        self.camera.add_normals_to_frame()
        self.camera.add_motion_vectors_to_frame()
        self.camera.add_occlusion_to_frame()
        self.camera.add_distance_to_image_plane_to_frame()
        self.camera.add_distance_to_camera_to_frame()
        self.camera.add_bounding_box_2d_tight_to_frame()
        self.camera.add_bounding_box_2d_loose_to_frame()
        self.camera.add_bounding_box_3d_to_frame()
        self.camera.add_semantic_segmentation_to_frame()
        self.camera.add_instance_id_segmentation_to_frame()
        self.camera.add_instance_segmentation_to_frame()
        self.camera.add_pointcloud_to_frame()

        # Wait for the data to be available in the frame
        await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 10)
        current_frame = self.camera.get_current_frame()

        # Check the annotators data, shape, type and dtype
        normals_data = current_frame.get("normals")
        self.assertIsNotNone(normals_data)
        self.assertTrue(normals_data.shape == (256, 256, 4))
        self.assertTrue(isinstance(normals_data, np.ndarray))
        self.assertTrue(normals_data.dtype == np.float32)

        motion_vectors_data = current_frame.get("motion_vectors")
        self.assertIsNotNone(motion_vectors_data)
        self.assertTrue(motion_vectors_data.shape == (256, 256, 4))
        self.assertTrue(isinstance(motion_vectors_data, np.ndarray))
        self.assertTrue(motion_vectors_data.dtype == np.float32)

        occlusion_data = current_frame.get("occlusion")
        self.assertIsNotNone(occlusion_data)
        self.assertTrue(occlusion_data.shape == (4,))
        self.assertTrue(isinstance(occlusion_data, np.ndarray))
        self.assertTrue(
            occlusion_data.dtype
            == np.dtype([("instanceId", np.uint32), ("semanticId", np.uint32), ("occlusionRatio", np.float32)])
        )

        distance_to_image_plane_data = current_frame.get("distance_to_image_plane")
        self.assertIsNotNone(distance_to_image_plane_data)
        self.assertTrue(distance_to_image_plane_data.shape == (256, 256))
        self.assertTrue(isinstance(distance_to_image_plane_data, np.ndarray))
        self.assertTrue(distance_to_image_plane_data.dtype == np.float32)

        distance_to_camera_data = current_frame.get("distance_to_camera")
        self.assertIsNotNone(distance_to_camera_data)
        self.assertTrue(distance_to_camera_data.shape == (256, 256))
        self.assertTrue(isinstance(distance_to_camera_data, np.ndarray))
        self.assertTrue(distance_to_camera_data.dtype == np.float32)

        bounding_box_2d_tight_data = current_frame.get("bounding_box_2d_tight")
        self.assertIsNotNone(bounding_box_2d_tight_data)
        self.assertTrue(bounding_box_2d_tight_data["data"].shape == (1,))
        self.assertTrue(isinstance(bounding_box_2d_tight_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_2d_tight_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.int32),
                    ("y_min", np.int32),
                    ("x_max", np.int32),
                    ("y_max", np.int32),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        bounding_box_2d_loose_data = current_frame.get("bounding_box_2d_loose")
        self.assertIsNotNone(bounding_box_2d_loose_data)
        self.assertTrue(bounding_box_2d_loose_data["data"].shape == (1,))
        self.assertTrue(isinstance(bounding_box_2d_loose_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_2d_loose_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.int32),
                    ("y_min", np.int32),
                    ("x_max", np.int32),
                    ("y_max", np.int32),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        bounding_box_3d_data = current_frame.get("bounding_box_3d")
        self.assertIsNotNone(bounding_box_3d_data)
        self.assertTrue(bounding_box_3d_data["data"].shape == (1,))
        self.assertTrue(isinstance(bounding_box_3d_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_3d_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.float32),
                    ("y_min", np.float32),
                    ("z_min", np.float32),
                    ("x_max", np.float32),
                    ("y_max", np.float32),
                    ("z_max", np.float32),
                    ("transform", np.float32, (4, 4)),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        semantic_segmentation_data = current_frame.get("semantic_segmentation")
        self.assertIsNotNone(semantic_segmentation_data)
        self.assertTrue(semantic_segmentation_data["data"].shape == (256, 256))
        self.assertTrue(isinstance(semantic_segmentation_data["data"], np.ndarray))
        self.assertTrue(semantic_segmentation_data["data"].dtype == np.uint32)

        instance_id_segmentation_data = current_frame.get("instance_id_segmentation")
        self.assertIsNotNone(instance_id_segmentation_data)
        self.assertTrue(instance_id_segmentation_data["data"].shape == (256, 256))
        self.assertTrue(isinstance(instance_id_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_id_segmentation_data["data"].dtype == np.uint32)

        instance_segmentation_data = current_frame.get("instance_segmentation")
        self.assertIsNotNone(instance_segmentation_data)
        self.assertTrue(instance_segmentation_data["data"].shape == (256, 256))
        self.assertTrue(isinstance(instance_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_segmentation_data["data"].dtype == np.uint32)

        # pointcloud_data = current_frame.get("pointcloud")
        # self.assertIsNotNone(pointcloud_data)
        # self.assertTrue(pointcloud_data["data"].shape == (65536, 3))
        # self.assertTrue(isinstance(pointcloud_data["data"], np.ndarray))
        # self.assertTrue(pointcloud_data["data"].dtype == np.float32)

    async def test_annotators_data_with_init_params(self):
        # Add all annotators to the camera with dummy and known init_params entries
        self.camera.add_bounding_box_2d_tight_to_frame(init_params={"semanticTypes": ["dummy"]})
        self.camera.add_bounding_box_2d_loose_to_frame(init_params={"semanticTypes": ["dummy"]})
        self.camera.add_bounding_box_3d_to_frame(init_params={"semanticTypes": ["dummy"]})
        self.camera.add_semantic_segmentation_to_frame(init_params={"colorize": True})
        self.camera.add_instance_id_segmentation_to_frame(init_params={"colorize": True})
        self.camera.add_instance_segmentation_to_frame(init_params={"colorize": True})

        # Get the current frame
        await omni.syntheticdata.sensors.next_render_simulation_async(self.camera.get_render_product_path(), 10)
        current_frame = self.camera.get_current_frame()

        bounding_box_2d_tight_data = current_frame.get("bounding_box_2d_tight")
        self.assertIsNotNone(bounding_box_2d_tight_data)
        self.assertTrue(bounding_box_2d_tight_data["data"].shape == (0,))  # No data since semanticTypes is dummy
        self.assertTrue(isinstance(bounding_box_2d_tight_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_2d_tight_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.int32),
                    ("y_min", np.int32),
                    ("x_max", np.int32),
                    ("y_max", np.int32),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        bounding_box_2d_loose_data = current_frame.get("bounding_box_2d_loose")
        self.assertIsNotNone(bounding_box_2d_loose_data)
        self.assertTrue(bounding_box_2d_loose_data["data"].shape == (0,))  # No data since semanticTypes is dummy
        self.assertTrue(isinstance(bounding_box_2d_loose_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_2d_loose_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.int32),
                    ("y_min", np.int32),
                    ("x_max", np.int32),
                    ("y_max", np.int32),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        bounding_box_3d_data = current_frame.get("bounding_box_3d")
        self.assertIsNotNone(bounding_box_3d_data)
        self.assertTrue(bounding_box_3d_data["data"].shape == (0,))  # No data since semanticTypes is dummy
        self.assertTrue(isinstance(bounding_box_3d_data["data"], np.ndarray))
        self.assertTrue(
            bounding_box_3d_data["data"].dtype
            == np.dtype(
                [
                    ("semanticId", np.uint32),
                    ("x_min", np.float32),
                    ("y_min", np.float32),
                    ("z_min", np.float32),
                    ("x_max", np.float32),
                    ("y_max", np.float32),
                    ("z_max", np.float32),
                    ("transform", np.float32, (4, 4)),
                    ("occlusionRatio", np.float32),
                ]
            )
        )

        semantic_segmentation_data = current_frame.get("semantic_segmentation")
        self.assertIsNotNone(semantic_segmentation_data)
        self.assertTrue(
            semantic_segmentation_data["data"].shape == (256, 256, 4)
        )  # Colorize is True, so 4 uint8 channels
        self.assertTrue(isinstance(semantic_segmentation_data["data"], np.ndarray))
        self.assertTrue(semantic_segmentation_data["data"].dtype == np.uint8)

        instance_id_segmentation_data = current_frame.get("instance_id_segmentation")
        self.assertIsNotNone(instance_id_segmentation_data)
        self.assertTrue(instance_id_segmentation_data["data"].shape == (256, 256, 4))
        self.assertTrue(isinstance(instance_id_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_id_segmentation_data["data"].dtype == np.uint8)

        instance_segmentation_data = current_frame.get("instance_segmentation")
        self.assertIsNotNone(instance_segmentation_data)
        self.assertTrue(instance_segmentation_data["data"].shape == (256, 256, 4))
        self.assertTrue(isinstance(instance_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_segmentation_data["data"].dtype == np.uint8)

    async def test_ftheta_properties_full(self):
        """Test F-theta lens distortion model with full coefficients."""
        self.camera.set_ftheta_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(24, 25),
            max_fov=560,
            distortion_coefficients=[1, 2, 3, 4, 5],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_ftheta_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [24, 25], atol=2).all())
        self.assertAlmostEqual(max_fov, 560, delta=2)
        self.assertTrue(np.isclose(coeffs, [1, 2, 3, 4, 5]).all())

    async def test_kannala_brandt_k3_properties_full(self):
        """Test Kannala-Brandt K3 lens distortion model with full coefficients."""
        self.camera.set_kannala_brandt_k3_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.05, 0.01, -0.003, -0.0005],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_kannala_brandt_k3_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(np.isclose(coeffs, [0.05, 0.01, -0.003, -0.0005], atol=0.00001).all())

    async def test_rad_tan_thin_prism_properties_full(self):
        """Test Radial-Tangential Thin Prism lens distortion model with full coefficients."""
        self.camera.set_rad_tan_thin_prism_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_rad_tan_thin_prism_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(
            np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06], atol=0.00001).all()
        )

    async def test_lut_properties_full(self):
        """Test LUT lens distortion model with full parameters."""
        self.camera.set_lut_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            ray_enter_direction_texture="path/to/enter.png",
            ray_exit_position_texture="path/to/exit.png",
        )
        nominal_height, nominal_width, optical_center, enter_tex, exit_tex = self.camera.get_lut_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertEqual(enter_tex, "path/to/enter.png")
        self.assertEqual(exit_tex, "path/to/exit.png")

    async def test_ftheta_properties_partial(self):
        """Test F-theta lens distortion model with partial coefficients."""
        self.camera.set_ftheta_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(24, 25),
            max_fov=560,
            distortion_coefficients=[1, 2],  # Only providing k0, k1
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_ftheta_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [24, 25], atol=2).all())
        self.assertAlmostEqual(max_fov, 560, delta=2)
        self.assertTrue(np.isclose(coeffs, [1, 2, 0, 0, 0]).all())  # k2, k3, k4 should default to 0

    async def test_kannala_brandt_k3_properties_partial(self):
        """Test Kannala-Brandt K3 lens distortion model with partial coefficients."""
        self.camera.set_kannala_brandt_k3_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.05, 0.01],  # Only providing k0, k1
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_kannala_brandt_k3_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(np.isclose(coeffs, [0.05, 0.01, 0, 0], atol=0.00001).all())  # k2, k3 should default to 0

    async def test_rad_tan_thin_prism_properties_partial(self):
        """Test Radial-Tangential Thin Prism lens distortion model with partial coefficients."""
        self.camera.set_rad_tan_thin_prism_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.1, 0.2, 0.3],  # Only providing k0, k1, k2
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = self.camera.get_rad_tan_thin_prism_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        # Remaining coefficients should be schema defaults
        expected_coeffs = [0.1, 0.2, 0.3] + [0.0, 0.0, 0.0, -0.00037, -0.00074, -0.00058, -0.00022, 0.00019, -0.0002]
        self.assertTrue(np.isclose(coeffs, expected_coeffs, atol=0.00001).all())

    async def test_lut_properties_partial(self):
        """Test LUT lens distortion model with partial parameters."""
        # First set full parameters to verify they remain unchanged when not provided
        self.camera.set_lut_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            ray_enter_direction_texture="path/to/enter.png",
            ray_exit_position_texture="path/to/exit.png",
        )

        # Test with partial parameters
        self.camera.set_lut_properties(nominal_height=360, nominal_width=180, optical_center=(75, 131))
        nominal_height, nominal_width, optical_center, enter_tex, exit_tex = self.camera.get_lut_properties()
        self.assertAlmostEqual(nominal_height, 360, delta=2)
        self.assertAlmostEqual(nominal_width, 180, delta=2)
        self.assertTrue(np.isclose(optical_center, [75, 131], atol=2).all())
        # Textures should remain unchanged when not provided
        self.assertEqual(enter_tex, "path/to/enter.png")
        self.assertEqual(exit_tex, "path/to/exit.png")

    async def test_opencv_pinhole_properties_full(self):
        """Test OpenCV pinhole lens distortion model with full coefficients."""
        self.camera.set_opencv_pinhole_properties(
            cx=350.5,
            cy=250.5,
            fx=450.0,
            fy=450.0,
            pinhole=[
                0.1,
                0.2,
                0.3,
                0.4,
                0.5,
                0.6,
                0.7,
                0.8,
                0.9,
                1.0,
                1.1,
                1.2,
            ],  # k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4
        )
        cx, cy, fx, fy, coeffs = self.camera.get_opencv_pinhole_properties()
        self.assertAlmostEqual(cx, 350.5, delta=0.1)
        self.assertAlmostEqual(cy, 250.5, delta=0.1)
        self.assertAlmostEqual(fx, 450.0, delta=0.1)
        self.assertAlmostEqual(fy, 450.0, delta=0.1)
        self.assertTrue(
            np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2], atol=0.001).all()
        )

    async def test_opencv_pinhole_properties_partial(self):
        """Test OpenCV pinhole lens distortion model with partial parameters."""
        # Only set cx and fx
        self.camera.set_opencv_pinhole_properties(cx=375.5, fx=475.0)
        cx, cy, fx, fy, coeffs = self.camera.get_opencv_pinhole_properties()
        self.assertAlmostEqual(cx, 375.5, delta=0.1)
        # Other parameters should retain their default values
        self.assertAlmostEqual(cy, 512.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(fx, 475.0, delta=0.1)
        self.assertAlmostEqual(fy, 800.0, delta=1.0)  # Default from schema
        # All distortion coefficients should be 0 by default
        self.assertTrue(np.isclose(coeffs, [0] * 12, atol=0.001).all())

    async def test_opencv_fisheye_properties_full(self):
        """Test OpenCV fisheye lens distortion model with full coefficients."""
        self.camera.set_opencv_fisheye_properties(
            cx=350.5, cy=250.5, fx=450.0, fy=450.0, fisheye=[0.1, 0.2, 0.3, 0.4]  # k1, k2, k3, k4
        )
        cx, cy, fx, fy, coeffs = self.camera.get_opencv_fisheye_properties()
        self.assertAlmostEqual(cx, 350.5, delta=0.1)
        self.assertAlmostEqual(cy, 250.5, delta=0.1)
        self.assertAlmostEqual(fx, 450.0, delta=0.1)
        self.assertAlmostEqual(fy, 450.0, delta=0.1)
        self.assertTrue(np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4], atol=0.001).all())

    async def test_opencv_fisheye_properties_partial(self):
        """Test OpenCV fisheye lens distortion model with partial parameters."""
        # Only set cy and fy
        self.camera.set_opencv_fisheye_properties(cy=275.5, fy=485.0)
        cx, cy, fx, fy, coeffs = self.camera.get_opencv_fisheye_properties()
        # cx and fx should retain their default values
        self.assertAlmostEqual(cx, 1024.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(cy, 275.5, delta=0.1)
        self.assertAlmostEqual(fx, 900.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(fy, 485.0, delta=0.1)
        # k1 is 0.00245 by default, and others are 0
        self.assertTrue(np.isclose(coeffs, [0.00245, 0, 0, 0], atol=0.001).all())

    async def test_lens_distortion_model_handling(self):
        """Test how set_lens_distortion_model handles different model names."""
        # Test with model name "pinhole" - special case that removes distortion schemas
        self.camera.set_lens_distortion_model("pinhole")
        # After setting to pinhole, the property is removed, so get_lens_distortion_model returns "pinhole"
        self.assertEqual(self.camera.get_lens_distortion_model(), "pinhole")
        # Verify no lens distortion API is applied
        self.assertFalse(any(api.startswith("OmniLensDistortion") for api in self.camera.prim.GetAppliedSchemas()))

        # Test with a valid model (must use full API name)
        self.camera.set_lens_distortion_model("OmniLensDistortionFthetaAPI")
        self.assertEqual(self.camera.get_lens_distortion_model(), "ftheta")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionFthetaAPI", self.camera.prim.GetAppliedSchemas())

        # Test with an invalid model name - should log a warning but not raise an exception
        # The lens distortion model should stay as "ftheta"
        self.camera.set_lens_distortion_model("invalid_model")
        self.assertEqual(self.camera.get_lens_distortion_model(), "ftheta")
        # API should still be ftheta
        self.assertIn("OmniLensDistortionFthetaAPI", self.camera.prim.GetAppliedSchemas())

        # Test with another valid API name
        self.camera.set_lens_distortion_model("OmniLensDistortionKannalaBrandtK3API")
        self.assertEqual(self.camera.get_lens_distortion_model(), "kannalaBrandtK3")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionKannalaBrandtK3API", self.camera.prim.GetAppliedSchemas())

        # Test remaining valid API names
        self.camera.set_lens_distortion_model("OmniLensDistortionRadTanThinPrismAPI")
        self.assertEqual(self.camera.get_lens_distortion_model(), "radTanThinPrism")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionRadTanThinPrismAPI", self.camera.prim.GetAppliedSchemas())

        self.camera.set_lens_distortion_model("OmniLensDistortionLutAPI")
        self.assertEqual(self.camera.get_lens_distortion_model(), "lut")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionLutAPI", self.camera.prim.GetAppliedSchemas())

        self.camera.set_lens_distortion_model("OmniLensDistortionOpenCvFisheyeAPI")
        self.assertEqual(self.camera.get_lens_distortion_model(), "opencvFisheye")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionOpenCvFisheyeAPI", self.camera.prim.GetAppliedSchemas())

        self.camera.set_lens_distortion_model("OmniLensDistortionOpenCvPinholeAPI")
        self.assertEqual(self.camera.get_lens_distortion_model(), "opencvPinhole")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionOpenCvPinholeAPI", self.camera.prim.GetAppliedSchemas())

    async def test_fisheye_polynomial_properties(self):
        """Test setting and getting fisheye polynomial properties."""
        self.camera.set_projection_type("fisheyePolynomial")
        self.camera.set_fisheye_polynomial_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            polynomial=[0.1, 0.2, 0.3, 0.4, 0.5],
        )

        nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, poly = (
            self.camera.get_fisheye_polynomial_properties()
        )

        self.assertAlmostEqual(nominal_width, 1920, delta=1)
        self.assertAlmostEqual(nominal_height, 1080, delta=1)
        self.assertAlmostEqual(optical_centre_x, 960, delta=1)
        self.assertAlmostEqual(optical_centre_y, 540, delta=1)
        self.assertAlmostEqual(max_fov, 180, delta=1)
        self.assertTrue(np.isclose(poly, [0.1, 0.2, 0.3, 0.4, 0.5], atol=0.001).all())

    async def test_rational_polynomial_properties(self):
        """Test setting rational polynomial properties."""
        # set_rational_polynomial_properties is a wrapper for set_opencv_pinhole_properties
        self.camera.set_rational_polynomial_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            distortion_model=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10, 0.11, 0.12],
        )

        # Verify properties were set correctly by accessing the OpenCV pinhole properties
        cx, cy, fx, fy, pinhole = self.camera.get_opencv_pinhole_properties()

        self.assertAlmostEqual(cx, 960)
        self.assertAlmostEqual(cy, 540)
        self.assertAlmostEqual(
            fx, 1920 * self.camera.get_focal_length() / self.camera.get_horizontal_aperture(), delta=1
        )
        self.assertAlmostEqual(fy, 1080 * self.camera.get_focal_length() / self.camera.get_vertical_aperture(), delta=1)
        self.assertTrue(np.isclose(pinhole, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10, 0.11, 0.12]).all())

    async def test_kannala_brandt_properties(self):
        """Test setting Kannala-Brandt properties."""
        self.camera.set_kannala_brandt_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            distortion_model=[0.1, 0.2, 0.3, 0.4],
        )

        # Verify properties were set correctly by accessing the fisheye polynomial properties
        cx, cy, fx, fy, fisheye = self.camera.get_opencv_fisheye_properties()

        self.assertAlmostEqual(cx, 960)
        self.assertAlmostEqual(cy, 540)
        self.assertAlmostEqual(
            fx, 1920 * self.camera.get_focal_length() / self.camera.get_horizontal_aperture(), delta=1
        )
        self.assertAlmostEqual(fy, 1080 * self.camera.get_focal_length() / self.camera.get_vertical_aperture(), delta=1)
        self.assertTrue(np.isclose(fisheye, [0.1, 0.2, 0.3, 0.4]).all())

    async def test_projection_mode(self):
        """Test setting and getting projection mode."""
        self.camera.set_projection_mode("perspective")
        self.assertEqual(self.camera.get_projection_mode(), "perspective")

        self.camera.set_projection_mode("orthographic")
        self.assertEqual(self.camera.get_projection_mode(), "orthographic")

    async def test_intrinsics_matrix(self):
        """Test getting camera intrinsics matrix."""
        self.camera.set_resolution((640, 480))
        self.camera.set_horizontal_aperture(1.0)
        self.camera.set_vertical_aperture(0.75)
        self.camera.set_focal_length(10.0)

        # Get the intrinsics matrix
        intrinsics = self.camera.get_intrinsics_matrix()

        # Check shape
        self.assertEqual(intrinsics.shape, (3, 3))

        # Focal length and principal point should be properly encoded in the matrix
        # For 640x480 with horizontal aperture 1.0, focal_x ≈ 6400
        # For 480x480 with vertical aperture 0.75, focal_y ≈ 6400
        # Principal point should be approximately at (320, 240)
        self.assertAlmostEqual(intrinsics[0, 0], 6400, delta=1280)  # Allow 20% tolerance
        self.assertAlmostEqual(intrinsics[1, 1], 6400, delta=1280)
        self.assertAlmostEqual(intrinsics[0, 2], 320, delta=64)
        self.assertAlmostEqual(intrinsics[1, 2], 240, delta=48)

    async def test_camera_points_from_image_coords(self):
        """Test converting image coordinates to camera points."""
        # Set up a simple test case
        self.camera.set_resolution((100, 100))

        # Center pixel at a depth of 10 units
        points_2d = np.array([[50, 50]])
        depth = np.array([10.0])

        # Get camera points
        camera_points = self.camera.get_camera_points_from_image_coords(points_2d, depth)

        # For the center pixel, we expect a point along the Z axis (in camera coords)
        # with a distance equal to the depth
        self.assertEqual(camera_points.shape, (1, 3))
        self.assertAlmostEqual(camera_points[0, 2], 10.0, delta=0.1)  # Z coordinate should be depth
        self.assertAlmostEqual(camera_points[0, 0], 0.0, delta=0.5)  # X should be near 0 (with tolerance)
        self.assertAlmostEqual(camera_points[0, 1], 0.0, delta=0.5)  # Y should be near 0 (with tolerance)

    async def test_world_points_from_image_coords(self):
        """Test converting image coordinates to world points."""
        # Set a predictable camera pose
        self.camera.set_world_pose(
            position=[0.0, 0.0, 0.0], orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True)
        )

        # Set up a simple test case
        self.camera.set_resolution((100, 100))

        # Center pixel at a depth of 10 units
        points_2d = np.array([[50, 50]])
        depth = np.array([10.0])

        # Get world points
        world_points = self.camera.get_world_points_from_image_coords(points_2d, depth)

        # For the center pixel with the camera at origin and no rotation,
        # we expect a point along the X axis (in world coords) with a distance equal to the depth
        self.assertEqual(world_points.shape, (1, 3))
        self.assertAlmostEqual(world_points[0, 0], 10.0, delta=0.1)  # X coordinate should be depth
        self.assertAlmostEqual(world_points[0, 1], 0.0, delta=0.5)  # Y should be near 0 (with tolerance)
        self.assertAlmostEqual(world_points[0, 2], 0.0, delta=0.5)  # Z should be near 0 (with tolerance)
