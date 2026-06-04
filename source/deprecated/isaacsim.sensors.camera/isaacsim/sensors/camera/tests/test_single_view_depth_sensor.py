# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for single-view depth sensor functionality."""

import os

import carb
import cv2
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cone, Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import SingleViewDepthSensor
from isaacsim.storage.native import get_assets_root_path_async
from isaacsim.test.utils.image_comparison import compare_images_within_tolerances


class TestSingleViewDepthSensor(omni.kit.test.AsyncTestCase):
    """Test suite for SingleViewDepthSensor functionality."""

    CAMERA_RESOLUTION = (1920, 1080)
    CAMERA_FREQUENCY = 20  # Hz
    NUM_WARMUP_FRAMES = 10  # Frame dict data availability warmup frames
    IMG_MEAN_TOLERANCE = 5.0  # Mean absolute difference tolerance for image comparison
    SUPPORTED_ANNOTATORS = [
        "DepthSensorDistance",
        "DepthSensorPointCloudPosition",
        "DepthSensorPointCloudColor",
        "DepthSensorImager",
    ]
    GOLDEN_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "single_view_depth_sensor")

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        self.test_dir = carb.tokens.get_tokens_interface().resolve("${temp}/test_camera_view_sensor")

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def _create_test_environment(self) -> tuple:
        """Create test environment with ground plane, cubes, cone, and depth camera.

        Returns:
            None.

        """
        await create_new_stage_async()

        # Create a plane and dome light
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=100.0, templates=None)

        cube_1 = Cube(
            "/cube_1",
            sizes=1.0,
            positions=np.array([0.25, 0.25, 0.25]),
            scales=np.array([0.5, 0.5, 0.5]),
        )
        cube_1_material = OmniPbrMaterial("/World/Materials/cube_1")
        cube_1_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
        cube_1.apply_visual_materials(cube_1_material)

        cube_2 = Cube(
            "/cube_2",
            sizes=1.0,
            positions=np.array([-1.0, -1.0, 0.25]),
            scales=np.array([1.0, 1.0, 1.0]),
        )
        cube_2_material = OmniPbrMaterial("/World/Materials/cube_2")
        cube_2_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
        cube_2.apply_visual_materials(cube_2_material)

        cone = Cone(
            "/cone",
            radii=0.5,
            heights=1.0,
            positions=np.array([-0.1, -0.3, 0.2]),
            scales=np.array([1.0, 1.0, 1.0]),
        )
        cone_material = OmniPbrMaterial("/World/Materials/cone")
        cone_material.set_input_values("diffuse_color_constant", [0.0, 1.0, 0.0])
        cone.apply_visual_materials(cone_material)

        camera = SingleViewDepthSensor(
            prim_path="/World/camera",
            name="depth_camera",
            position=np.array([3.0, 0.0, 0.6]),
            orientation=euler_angles_to_quaternion(np.array([0, 0, 180]), degrees=True, extrinsic=False).numpy(),
            frequency=self.CAMERA_FREQUENCY,
            resolution=self.CAMERA_RESOLUTION,
        )

        return camera, cube_1, cube_2, cone

    async def test_getter_setter_methods(self) -> None:
        """Test getter and setter methods before and after initialization."""
        camera, _, _, _ = await self._create_test_environment()

        # Before initialization, getters should return None
        self.assertIsNone(camera.get_baseline_mm())
        self.assertIsNone(camera.get_confidence_threshold())
        self.assertIsNone(camera.get_enabled())
        self.assertIsNone(camera.get_focal_length_pixel())
        self.assertIsNone(camera.get_max_disparity_pixel())
        self.assertIsNone(camera.get_max_distance())
        self.assertIsNone(camera.get_min_distance())
        self.assertIsNone(camera.get_noise_downscale_factor_pixel())
        self.assertIsNone(camera.get_noise_mean())
        self.assertIsNone(camera.get_noise_sigma())
        self.assertIsNone(camera.get_rgb_depth_output_mode())
        self.assertIsNone(camera.get_sensor_size_pixel())
        self.assertIsNone(camera.get_show_distance())

        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await omni.kit.app.get_app().next_update_async()

        # After initialization, getters should return default values
        self.assertAlmostEqual(camera.get_baseline_mm(), 55.0)
        self.assertAlmostEqual(camera.get_confidence_threshold(), 0.70)
        self.assertTrue(camera.get_enabled())
        self.assertAlmostEqual(camera.get_focal_length_pixel(), 897.0)
        self.assertAlmostEqual(camera.get_max_disparity_pixel(), 110.0)
        self.assertAlmostEqual(camera.get_max_distance(), 10000000.0)
        self.assertAlmostEqual(camera.get_min_distance(), 0.5)
        self.assertAlmostEqual(camera.get_noise_downscale_factor_pixel(), 1.0)
        self.assertAlmostEqual(camera.get_noise_mean(), 0.25)
        self.assertAlmostEqual(camera.get_noise_sigma(), 0.25)
        self.assertTrue(camera.get_outlier_removal_enabled())
        self.assertEqual(camera.get_rgb_depth_output_mode(), 0)
        self.assertEqual(camera.get_sensor_size_pixel(), 1280)
        self.assertFalse(camera.get_show_distance())

        # Test setters with new values
        camera.set_baseline_mm(60.0)
        self.assertAlmostEqual(camera.get_baseline_mm(), 60.0)

        camera.set_confidence_threshold(0.8)
        self.assertAlmostEqual(camera.get_confidence_threshold(), 0.8)

        camera.set_enabled(False)
        self.assertFalse(camera.get_enabled())

        camera.set_focal_length_pixel(900.0)
        self.assertAlmostEqual(camera.get_focal_length_pixel(), 900.0)

        camera.set_max_disparity_pixel(120.0)
        self.assertAlmostEqual(camera.get_max_disparity_pixel(), 120.0)

        camera.set_max_distance(9000000.0)
        self.assertAlmostEqual(camera.get_max_distance(), 9000000.0)

        camera.set_min_distance(1.0)
        self.assertAlmostEqual(camera.get_min_distance(), 1.0)

        camera.set_noise_downscale_factor_pixel(2.0)
        self.assertAlmostEqual(camera.get_noise_downscale_factor_pixel(), 2.0)

        camera.set_noise_mean(0.5)
        self.assertAlmostEqual(camera.get_noise_mean(), 0.5)

        camera.set_noise_sigma(0.5)
        self.assertAlmostEqual(camera.get_noise_sigma(), 0.5)

        camera.set_outlier_removal_enabled(False)
        self.assertFalse(camera.get_outlier_removal_enabled())

        camera.set_rgb_depth_output_mode(1)
        self.assertEqual(camera.get_rgb_depth_output_mode(), 1)

        camera.set_sensor_size_pixel(1920)
        self.assertEqual(camera.get_sensor_size_pixel(), 1920)

        camera.set_show_distance(True)
        self.assertTrue(camera.get_show_distance())

    async def test_annotator_methods(self) -> None:
        """Test attaching and detaching annotators."""
        camera, _, _, _ = await self._create_test_environment()

        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await omni.kit.app.get_app().next_update_async()

        # Test attaching annotators
        for annotator in self.SUPPORTED_ANNOTATORS:
            camera.attach_annotator(annotator)
            self.assertIn(annotator, camera._custom_annotators)
            camera.detach_annotator(annotator)
            self.assertNotIn(annotator, camera._custom_annotators)

        # Test attaching multiple annotators
        for annotator in self.SUPPORTED_ANNOTATORS:
            camera.attach_annotator(annotator)

        # Check if all annotators are in the camera's custom annotators
        for annotator in self.SUPPORTED_ANNOTATORS:
            self.assertIn(annotator, camera._custom_annotators)

        # Detach each annotator individually
        for annotator in self.SUPPORTED_ANNOTATORS:
            camera.detach_annotator(annotator)
            self.assertNotIn(annotator, camera._custom_annotators)

        # Test with invalid annotator name
        with self.assertRaises(rep.annotators.AnnotatorRegistryError):
            camera.attach_annotator("InvalidAnnotator")

    async def test_depth_sensor_distance_annotator(self) -> None:
        """Test the DepthSensorDistance annotator output against golden image."""
        camera, _, _, _ = await self._create_test_environment()

        # Initialize the black grid scene for the background
        assets_root_path = await get_assets_root_path_async()
        omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to="/World/black_grid",
            asset_path=assets_root_path + "/Isaac/Environments/Grid/gridroom_black.usd",
            instanceable=False,
        )

        # Timeline must be playing for the SDG pipeline to initialize the camera
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize(attach_rgb_annotator=False)

        camera.set_focal_length(1.814756)
        camera.set_focus_distance(400.0)
        camera.set_baseline_mm(55)
        camera.set_focal_length_pixel(891.0)
        camera.set_sensor_size_pixel(1280.0)
        camera.set_max_disparity_pixel(110.0)
        camera.set_confidence_threshold(0.99)
        camera.set_noise_mean(0.5)
        camera.set_noise_sigma(1.0)
        camera.set_noise_downscale_factor_pixel(1.0)
        camera.set_min_distance(0.5)
        camera.set_max_distance(9999.9)

        camera.attach_annotator("DepthSensorDistance")

        # Warmup frames for the data to be available
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        await omni.syntheticdata.sensors.next_render_simulation_async(camera.get_render_product_path(), 10)
        image = camera.get_current_frame()["DepthSensorDistance"].astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Save test image for comparison
        test_img_filename = "depth_sensor_distance_annotator.png"
        test_img_path = os.path.join(self.test_dir, test_img_filename)
        os.makedirs(self.test_dir, exist_ok=True)
        cv2.imwrite(test_img_path, image)

        # Compare with golden image
        golden_img_path = os.path.join(self.GOLDEN_DIR, test_img_filename)
        result = compare_images_within_tolerances(
            golden_img_path,
            test_img_path,
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.IMG_MEAN_TOLERANCE,
        )
        self.assertTrue(result["passed"], f"Image comparison failed: {result['metrics']}")
