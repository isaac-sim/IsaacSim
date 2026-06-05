# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for camera sensor functionality."""

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.utils.semantics import add_labels
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.camera.camera import R_U_TRANSFORM
from omni.kit.viewport.utility import get_active_viewport


class TestCameraSensor(omni.kit.test.AsyncTestCase):
    """Tests for the Camera sensor class."""

    CAMERA_RESOLUTION = (256, 256)
    CAMERA_FREQUENCY = 20  # Hz
    NUM_WARMUP_FRAMES = 10  # Frame dict data availability warmup frames, depends on the camera frequency

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def _create_test_environment(self) -> tuple:
        """Create the test environment with ground plane, cubes, and camera.

        Returns:
            None.

        """
        await create_new_stage_async()

        # Create ground plane and dome light
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=100.0, templates=None)

        # Add cubes for testing (static positions)
        cube_2 = Cube(
            "/new_cube_2",
            sizes=1.0,
            positions=np.array([5.0, 3, 1.0]),
            scales=np.array([0.6, 0.5, 0.2]),
        )
        cube_2_material = OmniPbrMaterial("/World/Materials/cube_2")
        cube_2_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
        cube_2.apply_visual_materials(cube_2_material)

        cube_3 = Cube(
            "/new_cube_3",
            sizes=1.0,
            positions=np.array([-5, 1, 3.0]),
            scales=np.array([0.1, 0.1, 0.1]),
        )
        cube_3_material = OmniPbrMaterial("/World/Materials/cube_3")
        cube_3_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
        cube_3.apply_visual_materials(cube_3_material)

        xform = SingleXFormPrim(
            prim_path="/World/rig",
            name="rig",
            position=np.array([5.0, 0.0, 5.0]),
            orientation=euler_angles_to_quaternion(np.array([0, -90, 0]), degrees=True, extrinsic=False).numpy(),
        )

        camera = Camera(
            prim_path="/World/rig/camera",
            name="camera",
            position=np.array([0.0, 0.0, 25.0]),
            frequency=self.CAMERA_FREQUENCY,
            resolution=self.CAMERA_RESOLUTION,
            orientation=euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy(),
        )

        add_labels(cube_2.prims[0], labels=["cube"], taxonomy="class")
        add_labels(cube_3.prims[0], labels=["cube"], taxonomy="class")

        await omni.kit.app.get_app().next_update_async()
        return camera, cube_2, cube_3, xform

    async def test_world_poses(self) -> None:
        """Test getting and setting world and local poses."""
        camera, _, _, _ = await self._create_test_environment()
        position, orientation = camera.get_world_pose()
        self.assertTrue(np.allclose(position, [0, 0, 25], atol=1e-05))
        self.assertTrue(
            np.allclose(
                orientation,
                euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy(),
                atol=1e-05,
            )
        )
        translation, orientation = camera.get_local_pose()
        self.assertTrue(np.allclose(translation, [20, 0, 5], atol=1e-05))
        self.assertTrue(
            np.allclose(
                orientation,
                euler_angles_to_quaternion(np.array([0, 180, 0]), degrees=True, extrinsic=False).numpy(),
                atol=1e-05,
            )
        )
        camera.set_local_pose(
            translation=[0, 0, 25],
            orientation=euler_angles_to_quaternion(np.array([0, 180, 0]), degrees=True, extrinsic=False).numpy(),
        )

    async def test_projection(self) -> None:
        """Test projecting world points to image coordinates and back."""
        camera, cube_2, cube_3, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()

        # Test projection from world points to image coordinates and back
        cube_3_positions, _ = cube_3.get_world_poses()
        cube_2_positions, _ = cube_2.get_world_poses()
        points_2d = camera.get_image_coords_from_world_points(
            np.array([cube_3_positions.numpy()[0], cube_2_positions.numpy()[0]])
        )
        points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))

        # Expected golden values
        expected_points_2d_0 = [100.23487, 266.82565]
        expected_points_2d_1 = [51.645905, 0.7432048]
        expected_points_3d_0 = [-5.668, 1.134, 0.06]
        expected_points_3d_1 = [5.187, 3.113, 0.1]

        self.assertTrue(np.allclose(points_2d[0], expected_points_2d_0, atol=0.05), f"points_2d[0]: {points_2d[0]}")
        self.assertTrue(np.allclose(points_2d[1], expected_points_2d_1, atol=0.05), f"points_2d[1]: {points_2d[1]}")
        self.assertTrue(np.allclose(points_3d[0], expected_points_3d_0, atol=0.05), f"points_3d[0]: {points_3d[0]}")
        self.assertTrue(np.allclose(points_3d[1], expected_points_3d_1, atol=0.05), f"points_3d[1]: {points_3d[1]}")

    async def test_data_acquisition(self) -> None:
        """Test adding/removing annotators and acquiring frame data."""
        camera, _, _, _ = await self._create_test_environment()
        timeline = omni.timeline.get_timeline_interface()
        # Timeline must be playing to initialize the camera
        timeline.play()
        timeline.commit()
        camera.initialize()

        for annotator in camera.supported_annotators:
            # Add annotator to the frame
            getattr(camera, f"add_{annotator}_to_frame")()
            for _ in range(self.NUM_WARMUP_FRAMES):
                await omni.kit.app.get_app().next_update_async()

            # Make sure the annotator data is in the frame dict
            data = camera.get_current_frame()
            self.assertIsNotNone(data.get(annotator), f"{annotator} data is None")
            self.assertTrue(len(data[annotator]) > 0, f"{annotator}")
            if isinstance(data[annotator], dict) and "data" in data[annotator]:
                self.assertTrue(len(data[annotator]["data"]) > 0, f"check for data in {annotator}")

            # Remove annotator from the frame
            getattr(camera, f"remove_{annotator}_from_frame")()
            for _ in range(self.NUM_WARMUP_FRAMES):
                await omni.kit.app.get_app().next_update_async()

            # Make sure the annotator data is no longer in the frame dict
            data = camera.get_current_frame()
            self.assertTrue(annotator not in data, f"{annotator}")

    async def test_camera_properties(self) -> None:
        """Test setting and getting camera USD prim properties."""
        camera, _, _, _ = await self._create_test_environment()

        # Focal length
        camera.set_focal_length(5.0)
        self.assertAlmostEqual(camera.get_focal_length(), 5.0)

        # Focus distance
        camera.set_focus_distance(0.01)
        self.assertAlmostEqual(camera.get_focus_distance(), 0.01, delta=0.005)

        # Lens aperture
        camera.set_lens_aperture(0.01)
        self.assertAlmostEqual(camera.get_lens_aperture(), 0.01, delta=0.005)

        # Horizontal aperture
        camera.set_horizontal_aperture(1.2)
        self.assertAlmostEqual(camera.get_horizontal_aperture(), 1.2, delta=0.1)

        # Vertical aperture
        camera.set_vertical_aperture(1.2)
        self.assertAlmostEqual(camera.get_vertical_aperture(), 1.2, delta=0.1)

        # Clipping range
        camera.set_clipping_range(1.0, 1.0e5)
        clipping_range = camera.get_clipping_range()
        self.assertAlmostEqual(clipping_range[0], 1.0, delta=0.1)
        self.assertAlmostEqual(clipping_range[1], 1.0e5, delta=0.1)

        # Projection type
        camera.set_projection_type("fisheyeOrthographic")
        self.assertEqual(camera.get_projection_type(), "fisheyeOrthographic")

        # Stereo role
        camera.set_stereo_role("left")
        self.assertEqual(camera.get_stereo_role(), "left")

        # Shutter properties
        camera.set_shutter_properties(delay_open=2.0, delay_close=3.0)
        delay_open, delay_close = camera.get_shutter_properties()
        self.assertAlmostEqual(delay_open, 2.0, delta=0.1)
        self.assertAlmostEqual(delay_close, 3.0, delta=0.1)

        # Aspect ratio and FOV (read-only derived values)
        self.assertIsInstance(camera.get_aspect_ratio(), float)
        self.assertIsInstance(camera.get_horizontal_fov(), float)
        self.assertIsInstance(camera.get_vertical_fov(), float)

        # Resolution requires timeline and camera.initialize()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        camera.set_resolution((400, 300))
        resolution = camera.get_resolution()
        self.assertEqual(resolution[0], 400)
        self.assertEqual(resolution[1], 300)

    async def test_viewport_camera(self) -> None:
        """Test camera attached to an existing viewport render product."""
        await self._create_test_environment()
        viewport_api = get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()

        viewport_camera = Camera(
            prim_path="/World/rig/viewport_camera",
            name="viewport_camera",
            position=np.array([0.0, 0.0, 25.0]),
            resolution=self.CAMERA_RESOLUTION,
            orientation=euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy(),
            render_product_path=render_product_path,
        )

        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        viewport_camera.initialize()
        viewport_camera.add_distance_to_image_plane_to_frame()
        viewport_camera.add_pointcloud_to_frame()

        # Warmup frames for the data to be available
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        width, height = self.CAMERA_RESOLUTION
        expected_pixels = width * height
        self.assertEqual(viewport_camera.get_rgba().size, expected_pixels * 4)
        self.assertEqual(viewport_camera.get_rgb().size, expected_pixels * 3)
        self.assertEqual(viewport_camera.get_depth().size, expected_pixels * 1)
        self.assertEqual(viewport_camera.get_render_product_path(), render_product_path)

    async def test_get_current_frame(self) -> None:
        """Test that get_current_frame returns consistent/cloned frames."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()

        # Warmup frames for the data to be available
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
        current_frame_1 = camera.get_current_frame()
        current_frame_2 = camera.get_current_frame()

        # Make sure that the two frames refer to the same object
        self.assertIs(current_frame_1, current_frame_2)

        current_frame_3 = camera.get_current_frame()
        current_frame_4 = camera.get_current_frame(clone=True)

        # Make sure that the two frames refer to different objects
        self.assertIsNot(current_frame_3, current_frame_4)

    async def test_annotators_data(self) -> None:
        """Test annotator data shapes, types, and dtypes for all annotators."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()

        # Add all annotators to the camera
        camera.add_normals_to_frame()
        camera.add_motion_vectors_to_frame()
        camera.add_occlusion_to_frame()
        camera.add_distance_to_image_plane_to_frame()
        camera.add_distance_to_camera_to_frame()
        camera.add_bounding_box_2d_tight_to_frame()
        camera.add_bounding_box_2d_loose_to_frame()
        camera.add_bounding_box_3d_to_frame()
        camera.add_semantic_segmentation_to_frame()
        camera.add_instance_id_segmentation_to_frame()
        camera.add_instance_segmentation_to_frame()
        camera.add_pointcloud_to_frame()

        # Warmup frames for the data to be available
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        # Access the current frame dict
        current_frame = camera.get_current_frame()
        width, height = self.CAMERA_RESOLUTION

        # Check the annotators data, shape, type and dtype
        normals_data = current_frame.get("normals")
        self.assertIsNotNone(normals_data)
        self.assertTrue(normals_data.shape == (width, height, 4))
        self.assertTrue(isinstance(normals_data, np.ndarray))
        self.assertTrue(normals_data.dtype == np.float32)

        motion_vectors_data = current_frame.get("motion_vectors")
        self.assertIsNotNone(motion_vectors_data)
        self.assertTrue(motion_vectors_data.shape == (width, height, 4))
        self.assertTrue(isinstance(motion_vectors_data, np.ndarray))
        self.assertTrue(motion_vectors_data.dtype == np.float32)

        occlusion_data = current_frame.get("occlusion")
        self.assertIsNotNone(occlusion_data)
        expected_occlusion_entries = 3  # 2 cubes + plane
        self.assertTrue(occlusion_data.shape == (expected_occlusion_entries,))
        self.assertTrue(isinstance(occlusion_data, np.ndarray))
        self.assertTrue(
            occlusion_data.dtype
            == np.dtype([("instanceId", np.uint32), ("semanticId", np.uint32), ("occlusionRatio", np.float32)])
        )

        distance_to_image_plane_data = current_frame.get("distance_to_image_plane")
        self.assertIsNotNone(distance_to_image_plane_data)
        self.assertTrue(distance_to_image_plane_data.shape == (width, height))
        self.assertTrue(isinstance(distance_to_image_plane_data, np.ndarray))
        self.assertTrue(distance_to_image_plane_data.dtype == np.float32)

        distance_to_camera_data = current_frame.get("distance_to_camera")
        self.assertIsNotNone(distance_to_camera_data)
        self.assertTrue(distance_to_camera_data.shape == (width, height))
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
        self.assertTrue(semantic_segmentation_data["data"].shape == (width, height))
        self.assertTrue(isinstance(semantic_segmentation_data["data"], np.ndarray))
        self.assertTrue(semantic_segmentation_data["data"].dtype == np.uint32)

        instance_id_segmentation_data = current_frame.get("instance_id_segmentation")
        self.assertIsNotNone(instance_id_segmentation_data)
        self.assertTrue(instance_id_segmentation_data["data"].shape == (width, height))
        self.assertTrue(isinstance(instance_id_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_id_segmentation_data["data"].dtype == np.uint32)

        instance_segmentation_data = current_frame.get("instance_segmentation")
        self.assertIsNotNone(instance_segmentation_data)
        self.assertTrue(instance_segmentation_data["data"].shape == (width, height))
        self.assertTrue(isinstance(instance_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_segmentation_data["data"].dtype == np.uint32)

    async def test_annotators_data_with_init_params(self) -> None:
        """Test annotator data with custom init_params like colorize and semanticTypes."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()

        # Add all annotators to the camera with dummy and known init_params entries
        camera.add_bounding_box_2d_tight_to_frame(init_params={"semanticTypes": ["dummy"]})
        camera.add_bounding_box_2d_loose_to_frame(init_params={"semanticTypes": ["dummy"]})
        camera.add_bounding_box_3d_to_frame(init_params={"semanticTypes": ["dummy"]})
        camera.add_semantic_segmentation_to_frame(init_params={"colorize": True})
        camera.add_instance_id_segmentation_to_frame(init_params={"colorize": True})
        camera.add_instance_segmentation_to_frame(init_params={"colorize": True})

        # Warmup frames for the data to be available
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        # Access the current frame dict
        current_frame = camera.get_current_frame()
        width, height = self.CAMERA_RESOLUTION

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
            semantic_segmentation_data["data"].shape == (width, height, 4)
        )  # Colorize is True, so 4 uint8 channels
        self.assertTrue(isinstance(semantic_segmentation_data["data"], np.ndarray))
        self.assertTrue(semantic_segmentation_data["data"].dtype == np.uint8)

        instance_id_segmentation_data = current_frame.get("instance_id_segmentation")
        self.assertIsNotNone(instance_id_segmentation_data)
        self.assertTrue(instance_id_segmentation_data["data"].shape == (width, height, 4))
        self.assertTrue(isinstance(instance_id_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_id_segmentation_data["data"].dtype == np.uint8)

        instance_segmentation_data = current_frame.get("instance_segmentation")
        self.assertIsNotNone(instance_segmentation_data)
        self.assertTrue(instance_segmentation_data["data"].shape == (width, height, 4))
        self.assertTrue(isinstance(instance_segmentation_data["data"], np.ndarray))
        self.assertTrue(instance_segmentation_data["data"].dtype == np.uint8)

    async def test_ftheta_properties_full(self) -> None:
        """Test F-theta lens distortion model with full coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_ftheta_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(24, 25),
            max_fov=560,
            distortion_coefficients=[1, 2, 3, 4, 5],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_ftheta_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [24, 25], atol=2).all())
        self.assertAlmostEqual(max_fov, 560, delta=2)
        self.assertTrue(np.isclose(coeffs, [1, 2, 3, 4, 5]).all())

    async def test_kannala_brandt_k3_properties_full(self) -> None:
        """Test Kannala-Brandt K3 lens distortion model with full coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_kannala_brandt_k3_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.05, 0.01, -0.003, -0.0005],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_kannala_brandt_k3_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(np.isclose(coeffs, [0.05, 0.01, -0.003, -0.0005], atol=0.00001).all())

    async def test_rad_tan_thin_prism_properties_full(self) -> None:
        """Test Radial-Tangential Thin Prism lens distortion model with full coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_rad_tan_thin_prism_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_rad_tan_thin_prism_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(
            np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06], atol=0.00001).all()
        )

    async def test_lut_properties_full(self) -> None:
        """Test LUT lens distortion model with full parameters."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_lut_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            ray_enter_direction_texture="path/to/enter.png",
            ray_exit_position_texture="path/to/exit.png",
        )
        nominal_height, nominal_width, optical_center, enter_tex, exit_tex = camera.get_lut_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertEqual(enter_tex, "path/to/enter.png")
        self.assertEqual(exit_tex, "path/to/exit.png")

    async def test_ftheta_properties_partial(self) -> None:
        """Test F-theta lens distortion model with partial coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_ftheta_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(24, 25),
            max_fov=560,
            distortion_coefficients=[1, 2],  # Only providing k0, k1
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_ftheta_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [24, 25], atol=2).all())
        self.assertAlmostEqual(max_fov, 560, delta=2)
        self.assertTrue(np.isclose(coeffs, [1, 2, 0, 0, 0]).all())  # k2, k3, k4 should default to 0

    async def test_kannala_brandt_k3_properties_partial(self) -> None:
        """Test Kannala-Brandt K3 lens distortion model with partial coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_kannala_brandt_k3_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.05, 0.01],  # Only providing k0, k1
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_kannala_brandt_k3_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        self.assertTrue(np.isclose(coeffs, [0.05, 0.01, 0, 0], atol=0.00001).all())  # k2, k3 should default to 0

    async def test_rad_tan_thin_prism_properties_partial(self) -> None:
        """Test Radial-Tangential Thin Prism lens distortion model with partial coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_rad_tan_thin_prism_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            max_fov=180,
            distortion_coefficients=[0.1, 0.2, 0.3],  # Only providing k0, k1, k2
        )
        nominal_height, nominal_width, optical_center, max_fov, coeffs = camera.get_rad_tan_thin_prism_properties()
        self.assertAlmostEqual(nominal_height, 240, delta=2)
        self.assertAlmostEqual(nominal_width, 120, delta=2)
        self.assertTrue(np.isclose(optical_center, [65, 121], atol=2).all())
        self.assertAlmostEqual(max_fov, 180, delta=2)
        # Remaining coefficients should be schema defaults
        expected_coeffs = [0.1, 0.2, 0.3] + [0.0, 0.0, 0.0, -0.00037, -0.00074, -0.00058, -0.00022, 0.00019, -0.0002]
        self.assertTrue(np.isclose(coeffs, expected_coeffs, atol=0.00001).all())

    async def test_lut_properties_partial(self) -> None:
        """Test LUT lens distortion model with partial parameters."""
        camera, _, _, _ = await self._create_test_environment()

        # First set full parameters to verify they remain unchanged when not provided
        camera.set_lut_properties(
            nominal_height=240,
            nominal_width=120,
            optical_center=(65, 121),
            ray_enter_direction_texture="path/to/enter.png",
            ray_exit_position_texture="path/to/exit.png",
        )

        # Test with partial parameters
        camera.set_lut_properties(nominal_height=360, nominal_width=180, optical_center=(75, 131))
        nominal_height, nominal_width, optical_center, enter_tex, exit_tex = camera.get_lut_properties()
        self.assertAlmostEqual(nominal_height, 360, delta=2)
        self.assertAlmostEqual(nominal_width, 180, delta=2)
        self.assertTrue(np.isclose(optical_center, [75, 131], atol=2).all())
        # Textures should remain unchanged when not provided
        self.assertEqual(enter_tex, "path/to/enter.png")
        self.assertEqual(exit_tex, "path/to/exit.png")

    async def test_opencv_pinhole_properties_full(self) -> None:
        """Test OpenCV pinhole lens distortion model with full coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_opencv_pinhole_properties(
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
        cx, cy, fx, fy, coeffs = camera.get_opencv_pinhole_properties()
        self.assertAlmostEqual(cx, 350.5, delta=0.1)
        self.assertAlmostEqual(cy, 250.5, delta=0.1)
        self.assertAlmostEqual(fx, 450.0, delta=0.1)
        self.assertAlmostEqual(fy, 450.0, delta=0.1)
        self.assertTrue(
            np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2], atol=0.001).all()
        )

        image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        self.assertEqual(image_size, self.CAMERA_RESOLUTION)

    async def test_opencv_pinhole_properties_partial(self) -> None:
        """Test OpenCV pinhole lens distortion model with partial parameters."""
        camera, _, _, _ = await self._create_test_environment()
        # Only set cx and fx
        camera.set_opencv_pinhole_properties(cx=375.5, fx=475.0)
        cx, cy, fx, fy, coeffs = camera.get_opencv_pinhole_properties()
        self.assertAlmostEqual(cx, 375.5, delta=0.1)
        # Other parameters should retain their default values
        self.assertAlmostEqual(cy, 512.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(fx, 475.0, delta=0.1)
        self.assertAlmostEqual(fy, 800.0, delta=1.0)  # Default from schema
        # All distortion coefficients should be 0 by default
        self.assertTrue(np.isclose(coeffs, [0] * 12, atol=0.001).all())

        image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        self.assertEqual(image_size, self.CAMERA_RESOLUTION)

    async def test_opencv_fisheye_properties_full(self) -> None:
        """Test OpenCV fisheye lens distortion model with full coefficients."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_opencv_fisheye_properties(
            cx=350.5, cy=250.5, fx=450.0, fy=450.0, fisheye=[0.1, 0.2, 0.3, 0.4]  # k1, k2, k3, k4
        )
        cx, cy, fx, fy, coeffs = camera.get_opencv_fisheye_properties()
        self.assertAlmostEqual(cx, 350.5, delta=0.1)
        self.assertAlmostEqual(cy, 250.5, delta=0.1)
        self.assertAlmostEqual(fx, 450.0, delta=0.1)
        self.assertAlmostEqual(fy, 450.0, delta=0.1)
        self.assertTrue(np.isclose(coeffs, [0.1, 0.2, 0.3, 0.4], atol=0.001).all())

        image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvFisheye:imageSize").Get()
        self.assertEqual(image_size, self.CAMERA_RESOLUTION)

    async def test_opencv_fisheye_properties_partial(self) -> None:
        """Test OpenCV fisheye lens distortion model with partial parameters."""
        camera, _, _, _ = await self._create_test_environment()
        # Only set cy and fy
        camera.set_opencv_fisheye_properties(cy=275.5, fy=485.0)
        cx, cy, fx, fy, coeffs = camera.get_opencv_fisheye_properties()
        # cx and fx should retain their default values
        self.assertAlmostEqual(cx, 1024.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(cy, 275.5, delta=0.1)
        self.assertAlmostEqual(fx, 900.0, delta=1.0)  # Default from schema
        self.assertAlmostEqual(fy, 485.0, delta=0.1)
        # k1 is 0.00245 by default, and others are 0
        self.assertTrue(np.isclose(coeffs, [0.00245, 0, 0, 0], atol=0.001).all())

        image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvFisheye:imageSize").Get()
        self.assertEqual(image_size, self.CAMERA_RESOLUTION)

    async def test_lens_distortion_model_handling(self) -> None:
        """Test how set_lens_distortion_model handles different model names."""
        camera, _, _, _ = await self._create_test_environment()

        # Test with model name "pinhole" - special case that removes distortion schemas
        camera.set_lens_distortion_model("pinhole")
        # After setting to pinhole, the property is removed, so get_lens_distortion_model returns "pinhole"
        self.assertEqual(camera.get_lens_distortion_model(), "pinhole")
        # Verify no lens distortion API is applied
        self.assertFalse(any(api.startswith("OmniLensDistortion") for api in camera.prim.GetAppliedSchemas()))

        # Test with a valid model (must use full API name)
        camera.set_lens_distortion_model("OmniLensDistortionFthetaAPI")
        self.assertEqual(camera.get_lens_distortion_model(), "ftheta")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionFthetaAPI", camera.prim.GetAppliedSchemas())

        # Test with an invalid model name - should log a warning but not raise an exception
        # The lens distortion model should stay as "ftheta"
        camera.set_lens_distortion_model("invalid_model")
        self.assertEqual(camera.get_lens_distortion_model(), "ftheta")
        # API should still be ftheta
        self.assertIn("OmniLensDistortionFthetaAPI", camera.prim.GetAppliedSchemas())

        # Test with another valid API name
        camera.set_lens_distortion_model("OmniLensDistortionKannalaBrandtK3API")
        self.assertEqual(camera.get_lens_distortion_model(), "kannalaBrandtK3")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionKannalaBrandtK3API", camera.prim.GetAppliedSchemas())

        # Test remaining valid API names
        camera.set_lens_distortion_model("OmniLensDistortionRadTanThinPrismAPI")
        self.assertEqual(camera.get_lens_distortion_model(), "radTanThinPrism")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionRadTanThinPrismAPI", camera.prim.GetAppliedSchemas())

        camera.set_lens_distortion_model("OmniLensDistortionLutAPI")
        self.assertEqual(camera.get_lens_distortion_model(), "lut")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionLutAPI", camera.prim.GetAppliedSchemas())

        camera.set_lens_distortion_model("OmniLensDistortionOpenCvFisheyeAPI")
        self.assertEqual(camera.get_lens_distortion_model(), "opencvFisheye")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionOpenCvFisheyeAPI", camera.prim.GetAppliedSchemas())

        camera.set_lens_distortion_model("OmniLensDistortionOpenCvPinholeAPI")
        self.assertEqual(camera.get_lens_distortion_model(), "opencvPinhole")
        # Verify correct API is applied
        self.assertIn("OmniLensDistortionOpenCvPinholeAPI", camera.prim.GetAppliedSchemas())

    async def test_opencv_non_square_resolution(self) -> None:
        """Test OpenCV distortion models with non-square resolution."""
        await self._create_test_environment()

        # Define initial resolution (16:9 aspect ratio)
        resolution = (1280, 720)

        # Create a new camera with non-square resolution
        non_square_camera = Camera(
            prim_path="/World/non_square_camera",
            position=np.array([0.0, 0.0, 10.0]),
            resolution=resolution,
        )

        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        non_square_camera.initialize()

        # Test OpenCV pinhole with non-square resolution
        non_square_camera.set_opencv_pinhole_properties(
            cx=resolution[0] / 2.0,  # Half of width
            cy=resolution[1] / 2.0,  # Half of height
            fx=1000.0,
            fy=1000.0,
            pinhole=[0.1, 0.2, 0.01, 0.02],
        )

        # Verify imageSize is correctly set for non-square resolution
        image_size = non_square_camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        self.assertEqual(image_size, resolution)

        # Test OpenCV fisheye with non-square resolution
        non_square_camera.set_opencv_fisheye_properties(
            cx=resolution[0] / 2.0,
            cy=resolution[1] / 2.0,
            fx=800.0,
            fy=800.0,
            fisheye=[0.05, 0.01, -0.003, -0.0005],
        )

        # Verify imageSize is correctly set for non-square resolution
        image_size = non_square_camera.prim.GetAttribute("omni:lensdistortion:opencvFisheye:imageSize").Get()
        self.assertEqual(image_size, resolution)

        # Test with portrait orientation
        resolution = (1080, 1920)
        non_square_camera.set_resolution(resolution)

        # Set pinhole properties again with new resolution
        non_square_camera.set_opencv_pinhole_properties(
            cx=resolution[0] / 2.0,  # Half of width
            cy=resolution[1] / 2.0,  # Half of height
            fx=1000.0,
            fy=1000.0,
        )

        # Verify imageSize is correctly updated
        image_size = non_square_camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        self.assertEqual(image_size, resolution)

    async def test_fisheye_polynomial_properties(self) -> None:
        """Test setting and getting fisheye polynomial properties."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_projection_type("fisheyePolynomial")
        camera.set_fisheye_polynomial_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            polynomial=[0.1, 0.2, 0.3, 0.4, 0.5],
        )

        nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, poly = (
            camera.get_fisheye_polynomial_properties()
        )

        self.assertAlmostEqual(nominal_width, 1920, delta=1)
        self.assertAlmostEqual(nominal_height, 1080, delta=1)
        self.assertAlmostEqual(optical_centre_x, 960, delta=1)
        self.assertAlmostEqual(optical_centre_y, 540, delta=1)
        self.assertAlmostEqual(max_fov, 180, delta=1)
        self.assertTrue(np.isclose(poly, [0.1, 0.2, 0.3, 0.4, 0.5], atol=0.001).all())

    async def test_rational_polynomial_properties(self) -> None:
        """Test setting rational polynomial properties."""
        camera, _, _, _ = await self._create_test_environment()
        # set_rational_polynomial_properties is a wrapper for set_opencv_pinhole_properties
        camera.set_rational_polynomial_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            distortion_model=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10, 0.11, 0.12],
        )

        # Verify properties were set correctly by accessing the OpenCV pinhole properties
        cx, cy, fx, fy, pinhole = camera.get_opencv_pinhole_properties()

        self.assertAlmostEqual(cx, 960)
        self.assertAlmostEqual(cy, 540)
        self.assertAlmostEqual(fx, 1920 * camera.get_focal_length() / camera.get_horizontal_aperture(), delta=1)
        self.assertAlmostEqual(fy, 1080 * camera.get_focal_length() / camera.get_vertical_aperture(), delta=1)
        self.assertTrue(np.isclose(pinhole, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10, 0.11, 0.12]).all())

    async def test_kannala_brandt_properties(self) -> None:
        """Test setting Kannala-Brandt properties."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_kannala_brandt_properties(
            nominal_width=1920,
            nominal_height=1080,
            optical_centre_x=960,
            optical_centre_y=540,
            max_fov=180,
            distortion_model=[0.1, 0.2, 0.3, 0.4],
        )

        # Verify properties were set correctly by accessing the fisheye polynomial properties
        cx, cy, fx, fy, fisheye = camera.get_opencv_fisheye_properties()

        self.assertAlmostEqual(cx, 960)
        self.assertAlmostEqual(cy, 540)
        self.assertAlmostEqual(fx, 1920 * camera.get_focal_length() / camera.get_horizontal_aperture(), delta=1)
        self.assertAlmostEqual(fy, 1080 * camera.get_focal_length() / camera.get_vertical_aperture(), delta=1)
        self.assertTrue(np.isclose(fisheye, [0.1, 0.2, 0.3, 0.4]).all())

    async def test_projection_mode(self) -> None:
        """Test setting and getting projection mode."""
        camera, _, _, _ = await self._create_test_environment()
        camera.set_projection_mode("perspective")
        self.assertEqual(camera.get_projection_mode(), "perspective")

        camera.set_projection_mode("orthographic")
        self.assertEqual(camera.get_projection_mode(), "orthographic")

    async def test_view_matrix_ros(self) -> None:
        """Test getting camera view matrix in ROS convention."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        camera.set_resolution((640, 480))
        camera.set_horizontal_aperture(1.0)
        camera.set_vertical_aperture(0.75)
        camera.set_focal_length(10.0)

        # Check 1: The camera's world position should map to the origin in camera (ROS) frame
        # Get the view matrix
        view_matrix = camera.get_view_matrix_ros(device="cpu")
        self.assertEqual(view_matrix.shape, (4, 4))
        cam_pos_world, _ = camera.get_world_pose()
        cam_pos_world_h = np.array([cam_pos_world[0], cam_pos_world[1], cam_pos_world[2], 1.0], dtype=np.float32)
        cam_pos_in_cam = view_matrix @ cam_pos_world_h
        self.assertTrue(np.allclose(cam_pos_in_cam, np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32), atol=1e-5))

        # Check 2: from USD camera convention to ROS camera convention
        # R_U_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        # Identity pose -> view matrix = R_U_TRANSFORM
        camera.set_world_pose(position=[0.0, 0.0, 0.0], orientation=[1.0, 0.0, 0.0, 0.0], camera_axes="usd")
        await omni.kit.app.get_app().next_update_async()
        view_matrix_identity = camera.get_view_matrix_ros(device="cpu")
        self.assertTrue(np.allclose(view_matrix_identity, R_U_TRANSFORM.astype(np.float32), atol=1e-5))

        # Translation only -> view matrix = R_U_TRANSFORM @ T(-t). T(-t): inverse(world_from_cam)
        camera.set_world_pose(position=[2.0, -1.0, 5.0], orientation=[1.0, 0.0, 0.0, 0.0], camera_axes="usd")
        await omni.kit.app.get_app().next_update_async()
        view_matrix_translation_only = camera.get_view_matrix_ros(device="cpu")
        T_minus = np.array([[1, 0, 0, -2.0], [0, 1, 0, 1.0], [0, 0, 1, -5.0], [0, 0, 0, 1.0]], dtype=np.float32)
        self.assertTrue(
            np.allclose(view_matrix_translation_only, R_U_TRANSFORM.astype(np.float32) @ T_minus, atol=1e-5)
        )

        # Rotation only -> view matrix = R_U_TRANSFORM @ R.T
        # Rotate 90 degrees about +z axis
        R = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        camera.set_world_pose(
            position=[0.0, 0.0, 0.0],
            orientation=euler_angles_to_quaternion(np.array([0, 0, 90]), degrees=True, extrinsic=False).numpy(),
            camera_axes="usd",
        )
        await omni.kit.app.get_app().next_update_async()
        view_matrix_rotation_only = camera.get_view_matrix_ros(device="cpu")
        self.assertTrue(np.allclose(view_matrix_rotation_only, R_U_TRANSFORM.astype(np.float32) @ R.T, atol=1e-5))

        # Rotation and translation -> view matrix = R_U_TRANSFORM @ R.T @ T(-t)
        camera.set_world_pose(
            position=[2.0, -1.0, 5.0],
            orientation=euler_angles_to_quaternion(np.array([0, 0, 90]), degrees=True, extrinsic=False).numpy(),
            camera_axes="usd",
        )
        await omni.kit.app.get_app().next_update_async()
        view_matrix_rotation_and_translation = camera.get_view_matrix_ros(device="cpu")
        self.assertTrue(
            np.allclose(
                view_matrix_rotation_and_translation, R_U_TRANSFORM.astype(np.float32) @ R.T @ T_minus, atol=1e-5
            )
        )

        # Check 3: For any pixel and depth, the camera-space point from intrinsics-only
        #              must match the camera-space point obtained by transforming the corresponding
        #              world-space point with the view matrix.
        # Pick two pixels and depths well inside the image.
        pixels = np.array([[320.0, 240.0], [100.0, 80.0]], dtype=np.float32)
        depths = np.array([3.0, 7.5], dtype=np.float32)
        view_matrix = camera.get_view_matrix_ros(device="cpu")

        # Camera points via intrinsics-only
        cam_points_from_intrinsics = camera.get_camera_points_from_image_coords(pixels, depths)
        self.assertEqual(cam_points_from_intrinsics.shape, (2, 3))

        # Corresponding world points from the same pixels and depths
        world_points = camera.get_world_points_from_image_coords(pixels, depths)
        self.assertEqual(world_points.shape, (2, 3))

        # Transform world points using the view matrix
        world_points_h = np.concatenate([world_points.astype(np.float32), np.ones((2, 1), dtype=np.float32)], axis=1)
        cam_points_from_view = (view_matrix @ world_points_h.T).T[:, :3]

        # Both methods must agree
        self.assertTrue(np.allclose(cam_points_from_view, cam_points_from_intrinsics, atol=1e-4))

    async def test_intrinsics_matrix(self) -> None:
        """Test getting camera intrinsics matrix."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        camera.set_resolution((640, 480))
        camera.set_horizontal_aperture(1.0)
        camera.set_vertical_aperture(0.75)
        camera.set_focal_length(10.0)

        # Get the intrinsics matrix
        intrinsics = camera.get_intrinsics_matrix()

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

    async def test_camera_points_from_image_coords(self) -> None:
        """Test converting image coordinates to camera points."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        # Set up a simple test case
        camera.set_resolution((100, 100))

        # Center pixel at a depth of 10 units
        points_2d = np.array([[50, 50]])
        depth = np.array([10.0])

        # Get camera points
        camera_points = camera.get_camera_points_from_image_coords(points_2d, depth)

        # For the center pixel, we expect a point along the Z axis (in camera coords)
        # with a distance equal to the depth
        self.assertEqual(camera_points.shape, (1, 3))
        self.assertAlmostEqual(camera_points[0, 2], 10.0, delta=0.1)  # Z coordinate should be depth
        self.assertAlmostEqual(camera_points[0, 0], 0.0, delta=0.5)  # X should be near 0 (with tolerance)
        self.assertAlmostEqual(camera_points[0, 1], 0.0, delta=0.5)  # Y should be near 0 (with tolerance)

    async def test_world_points_from_image_coords(self) -> None:
        """Test converting image coordinates to world points."""
        camera, _, _, _ = await self._create_test_environment()
        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()

        # Set a predictable camera pose
        camera.set_world_pose(
            position=[0.0, 0.0, 0.0],
            orientation=euler_angles_to_quaternion(np.array([0, 0, 0]), degrees=True, extrinsic=False).numpy(),
        )

        # Set up a simple test case
        camera.set_resolution((100, 100))

        # Center pixel at a depth of 10 units
        points_2d = np.array([[50, 50]])
        depth = np.array([10.0])

        # Get world points
        world_points = camera.get_world_points_from_image_coords(points_2d, depth)

        # For the center pixel with the camera at origin and no rotation,
        # we expect a point along the X axis (in world coords) with a distance equal to the depth
        self.assertEqual(world_points.shape, (1, 3))
        self.assertAlmostEqual(world_points[0, 0], 10.0, delta=0.1)  # X coordinate should be depth
        self.assertAlmostEqual(world_points[0, 1], 0.0, delta=0.5)  # Y should be near 0 (with tolerance)
        self.assertAlmostEqual(world_points[0, 2], 0.0, delta=0.5)  # Z should be near 0 (with tolerance)
