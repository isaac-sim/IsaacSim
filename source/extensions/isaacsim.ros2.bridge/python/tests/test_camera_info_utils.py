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
import omni.kit.test
import omni.usd
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.ros2.bridge.impl.camera_info_utils import compute_relative_pose, read_camera_info
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf

from .common import ROS2TestCase


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCameraInfoUtils(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()

        # Load a scene
        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await open_stage_async(self._assets_root_path + scene_path)
        self._stage = omni.usd.get_context().get_stage()

        # Create a camera using the Camera API
        camera_path = "/test_camera"
        self._camera = Camera(
            prim_path=camera_path,
            name="test_camera",
            resolution=(1280, 720),
            position=np.array([0, 0, 100]),
            orientation=np.array([1, 0, 0, 0]),
        )
        self._camera.initialize(attach_rgb_annotator=False)

        # Store the render product path from the camera
        self._render_product_path = self._camera._render_product_path
        self.assertIsNotNone(self._render_product_path, "Failed to create render product")

        # Get the camera prim
        self._camera_prim = self._stage.GetPrimAtPath(camera_path)
        self.assertIsNotNone(self._camera_prim, f"Failed to get camera prim at {camera_path}")

        # Play the timeline to ensure the render product is active
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self):

        # Clean up the camera
        if hasattr(self, "_camera"):
            self._camera = None

        await super().tearDown()

    async def test_read_camera_info_pinhole(self):
        """Test reading camera info for a pinhole camera with no distortion"""
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, "Failed to read camera info")
        self.assertIsNotNone(camera_prim, "Failed to get camera prim")

        # Check resolution
        width, height = self._camera.get_resolution()
        self.assertEqual(camera_info.width, width)
        self.assertEqual(camera_info.height, height)

        # Check intrinsics matrix (k) - should have correct structure
        self.assertEqual(len(camera_info.k), 9)
        # Check expected matrix structure: fx, 0, cx; 0, fy, cy; 0, 0, 1
        self.assertAlmostEqual(camera_info.k[0], camera_info.p[0])  # fx
        self.assertAlmostEqual(camera_info.k[2], camera_info.p[2])  # cx
        self.assertAlmostEqual(camera_info.k[4], camera_info.p[5])  # fy
        self.assertAlmostEqual(camera_info.k[5], camera_info.p[6])  # cy
        self.assertEqual(camera_info.k[1], 0.0)
        self.assertEqual(camera_info.k[3], 0.0)
        self.assertEqual(camera_info.k[6], 0.0)
        self.assertEqual(camera_info.k[7], 0.0)
        self.assertEqual(camera_info.k[8], 1.0)

        # Check rectification matrix - should be identity for pinhole camera
        expected_identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.assertTrue(
            all(actual == expected for actual, expected in zip(camera_info.r, expected_identity)),
            "Rectification matrix should be identity",
        )

        # Check projection matrix (p)
        self.assertEqual(len(camera_info.p), 12)
        # Check that cx and cy are approximately half the image dimensions
        self.assertAlmostEqual(camera_info.p[2], width * 0.5, delta=1.0)
        self.assertAlmostEqual(camera_info.p[6], height * 0.5, delta=1.0)

    async def test_read_camera_info_fisheye_unset_distortion(self):
        """Test reading camera info for a fisheye camera with unset distortion attributes"""
        self._camera.set_projection_type("OmniLensDistortionFthetaAPI")

        self._camera.set_focal_length(24.0)
        self._camera.set_horizontal_aperture(36.0)

        await omni.kit.app.get_app().next_update_async()

        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, "Failed to read camera info")
        self.assertIsNotNone(camera_prim, "Failed to get camera prim")

        # Check distortion model - should default to plumb_bob
        self.assertEqual(camera_info.distortion_model, "plumb_bob")

        # Check distortion coefficients - should be all zeros
        self.assertEqual(len(camera_info.d), 5)  # plumb_bob uses 5 coefficients
        self.assertTrue(all(coef == 0.0 for coef in camera_info.d), "All distortion coefficients should be zero")

        # Check resolution
        width, height = self._camera.get_resolution()
        self.assertEqual(camera_info.width, width)
        self.assertEqual(camera_info.height, height)

    async def test_read_camera_info_fisheye_plumb_bob_distortion(self):
        """Test reading camera info for a fisheye camera with plumb_bob distortion model"""
        # Set the camera to be a fisheye camera
        self._camera.set_projection_type("OmniLensDistortionFthetaAPI")
        self._camera.set_focal_length(24.0)
        self._camera.set_horizontal_aperture(36.0)

        # Set up distortion model and coefficients
        distortion_model = "plumb_bob"
        coefficients = [0.1, 0.2, 0.01, 0.02, 0.003]
        expected_ros_model = "plumb_bob"
        expected_coef_count = 5

        # Apply the distortion model to the camera
        self._camera.prim.CreateAttribute("physicalDistortionModel", Sdf.ValueTypeNames.Token).Set(distortion_model)
        self._camera.prim.CreateAttribute("physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray).Set(
            coefficients
        )

        await omni.kit.app.get_app().next_update_async()

        # Call the function under test
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, f"Failed to read camera info for {distortion_model}")
        self.assertIsNotNone(camera_prim, f"Failed to get camera prim for {distortion_model}")

        # Check distortion model
        self.assertEqual(camera_info.distortion_model, expected_ros_model)

        # Check distortion coefficients
        self.assertEqual(len(camera_info.d), expected_coef_count)

        # Expected behavior: When using fisheye projection with plumb_bob model,
        # the distortion coefficients should be zeros
        self.assertTrue(
            all(coef == 0.0 for coef in camera_info.d),
            "All distortion coefficients should be zero for fisheye with plumb_bob model",
        )

    async def test_read_camera_info_fisheye_rational_polynomial_distortion(self):
        """Test reading camera info for a fisheye camera with rational_polynomial distortion model"""
        # Set the camera to be a fisheye camera
        self._camera.set_projection_type("OmniLensDistortionFthetaAPI")
        self._camera.set_focal_length(24.0)
        self._camera.set_horizontal_aperture(36.0)

        # Set up distortion model and coefficients
        distortion_model = "rational_polynomial"
        coefficients = [0.1, 0.2, 0.01, 0.02, 0.003, 0.004, 0.0005, 0.0006]
        expected_ros_model = "rational_polynomial"
        expected_coef_count = 8

        # Apply the distortion model to the camera
        self._camera.prim.CreateAttribute("physicalDistortionModel", Sdf.ValueTypeNames.Token).Set(distortion_model)
        self._camera.prim.CreateAttribute("physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray).Set(
            coefficients
        )

        await omni.kit.app.get_app().next_update_async()

        # Call the function under test
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, f"Failed to read camera info for {distortion_model}")
        self.assertIsNotNone(camera_prim, f"Failed to get camera prim for {distortion_model}")

        # Check distortion model
        self.assertEqual(camera_info.distortion_model, expected_ros_model)

        # Check distortion coefficients
        self.assertEqual(len(camera_info.d), expected_coef_count)

        # Check that the coefficients match what we set
        for j, (expected, actual) in enumerate(zip(coefficients, camera_info.d)):
            self.assertAlmostEqual(actual, expected, delta=1e-5, msg=f"Coefficient {j} mismatch for {distortion_model}")

    async def test_read_camera_info_fisheye_equidistant_distortion(self):
        """Test reading camera info for a fisheye camera with equidistant distortion model"""
        # Set the camera to be a fisheye camera
        self._camera.set_projection_type("OmniLensDistortionFthetaAPI")
        self._camera.set_focal_length(24.0)
        self._camera.set_horizontal_aperture(36.0)

        # Set up distortion model and coefficients
        distortion_model = "equidistant"
        coefficients = [0.1, 0.2, 0.01, 0.02]
        expected_ros_model = "equidistant"
        expected_coef_count = 4

        # Apply the distortion model to the camera
        self._camera.prim.CreateAttribute("physicalDistortionModel", Sdf.ValueTypeNames.Token).Set(distortion_model)
        self._camera.prim.CreateAttribute("physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray).Set(
            coefficients
        )

        await omni.kit.app.get_app().next_update_async()

        # Call the function under test
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, f"Failed to read camera info for {distortion_model}")
        self.assertIsNotNone(camera_prim, f"Failed to get camera prim for {distortion_model}")

        # Check distortion model
        self.assertEqual(camera_info.distortion_model, expected_ros_model)

        # Check distortion coefficients
        self.assertEqual(len(camera_info.d), expected_coef_count)

        # Check that the coefficients match what we set
        for j, (expected, actual) in enumerate(zip(coefficients, camera_info.d)):
            self.assertAlmostEqual(actual, expected, delta=1e-5, msg=f"Coefficient {j} mismatch for {distortion_model}")

    async def test_read_camera_info_fisheye_unsupported_distortion(self):
        """Test reading camera info for a fisheye camera with an unsupported distortion model"""
        # Set the camera to be a fisheye camera
        self._camera.set_projection_type("OmniLensDistortionFthetaAPI")
        self._camera.set_focal_length(24.0)
        self._camera.set_horizontal_aperture(36.0)

        # Set up distortion model and coefficients
        distortion_model = "unsupported_model"
        coefficients = [0.1, 0.2, 0.01, 0.02, 0.003]
        expected_ros_model = "plumb_bob"  # Should default to plumb_bob for unsupported models
        expected_coef_count = 5

        # Apply the distortion model to the camera
        self._camera.prim.CreateAttribute("physicalDistortionModel", Sdf.ValueTypeNames.Token).Set(distortion_model)
        self._camera.prim.CreateAttribute("physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray).Set(
            coefficients
        )

        await omni.kit.app.get_app().next_update_async()

        # Call the function under test
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, f"Failed to read camera info for {distortion_model}")
        self.assertIsNotNone(camera_prim, f"Failed to get camera prim for {distortion_model}")

        # Check distortion model
        self.assertEqual(camera_info.distortion_model, expected_ros_model)

        # Check distortion coefficients
        self.assertEqual(len(camera_info.d), expected_coef_count)

        # For unsupported model, we expect zeros
        self.assertTrue(
            all(coef == 0.0 for coef in camera_info.d), "All coefficients should be zero for unsupported model"
        )

    async def test_read_camera_info_opencv_pinhole_distortion(self):
        """Test reading camera info with OpenCV pinhole distortion model (5, 8, and 12 parameters)"""
        # Get current camera dimensions for calculating center and focal length
        width, height = self._camera.get_resolution()
        cx, cy = width / 2, height / 2

        # Calculate focal lengths based on current camera properties
        horizontal_aperture = self._camera.get_horizontal_aperture()
        vertical_aperture = self._camera.get_vertical_aperture()
        focal_length = self._camera.get_focal_length()

        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture

        # Test 1: Standard 5 parameters (k1, k2, p1, p2, k3) for plumb_bob model
        k1, k2 = 0.1, 0.05
        p1, p2 = 0.01, 0.02
        k3 = 0.003

        # Create a list with 5 distortion parameters (others will be None/0)
        pinhole_params_5 = [k1, k2, p1, p2, k3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Use the Camera API to set the parameters
        self._camera.set_opencv_pinhole_properties(cx, cy, fx, fy, pinhole_params_5)

        await omni.kit.app.get_app().next_update_async()

        camera_info_5, camera_prim_5 = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info_5, "Failed to read camera info for 5 parameters")
        self.assertIsNotNone(camera_prim_5, "Failed to get camera prim for 5 parameters")

        # Check distortion model - should be plumb_bob for 5 parameters
        self.assertEqual(camera_info_5.distortion_model, "plumb_bob")

        # Check coefficients for plumb_bob (k1, k2, p1, p2, k3)
        expected_coeffs_5 = [k1, k2, p1, p2, k3]
        self.assertEqual(len(camera_info_5.d), len(expected_coeffs_5))

        for i, (expected, actual) in enumerate(zip(expected_coeffs_5, camera_info_5.d)):
            self.assertAlmostEqual(
                actual, expected, delta=1e-5, msg=f"Coefficient {i} mismatch for opencvPinhole with 5 parameters"
            )

        # Test 2: All 12 parameters (k1-k6, p1, p2, s1-s4) for rational_polynomial model
        k4, k5, k6 = 0.0015, 0.0008, 0.0004
        s1, s2, s3, s4 = 0.0003, 0.0002, 0.0001, 0.00005

        # Create a list with all 12 distortion parameters
        pinhole_params_12 = [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]

        # Use the Camera API to set the parameters
        self._camera.set_opencv_pinhole_properties(cx, cy, fx, fy, pinhole_params_12)

        await omni.kit.app.get_app().next_update_async()

        camera_info_12, camera_prim_12 = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info_12, "Failed to read camera info for 12 parameters")
        self.assertIsNotNone(camera_prim_12, "Failed to get camera prim for 12 parameters")

        # Check that the model is still rational_polynomial for 12 params
        self.assertEqual(camera_info_12.distortion_model, "rational_polynomial")

        # ROS doesn't use s1-s4 in rational_polynomial model, so expect 8 coefficients
        self.assertEqual(len(camera_info_12.d), 12)

        # The first 8 coefficients should be the same as before
        for i, (expected, actual) in enumerate(zip(pinhole_params_12, camera_info_12.d)):
            self.assertAlmostEqual(
                actual, expected, delta=1e-5, msg=f"Coefficient {i} mismatch for opencvPinhole with 12 parameters"
            )

    async def test_read_camera_info_opencv_fisheye_distortion(self):
        """Test reading camera info with OpenCV fisheye distortion model"""
        # Get current camera dimensions for calculating center and focal length
        width, height = self._camera.get_resolution()
        cx, cy = width / 2, height / 2

        # Calculate focal lengths based on current camera properties
        horizontal_aperture = self._camera.get_horizontal_aperture()
        vertical_aperture = self._camera.get_vertical_aperture()
        focal_length = self._camera.get_focal_length()

        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture

        # Set lens distortion parameters for opencv fisheye model
        k1, k2, k3, k4 = 0.1, 0.05, 0.01, 0.002

        # Use the Camera API to set the parameters
        fisheye_params = [k1, k2, k3, k4]
        self._camera.set_opencv_fisheye_properties(cx, cy, fx, fy, fisheye_params)

        await omni.kit.app.get_app().next_update_async()

        camera_info, camera_prim = read_camera_info(self._render_product_path)

        # Verify the camera info
        self.assertIsNotNone(camera_info, "Failed to read camera info")
        self.assertIsNotNone(camera_prim, "Failed to get camera prim")

        # Check distortion model - should be equidistant for OpenCV fisheye model
        self.assertEqual(camera_info.distortion_model, "equidistant")

        # Check that the distortion coefficients were set correctly
        expected_coeffs = [k1, k2, k3, k4]
        self.assertEqual(len(camera_info.d), len(expected_coeffs))

        for i, (expected, actual) in enumerate(zip(expected_coeffs, camera_info.d)):
            self.assertAlmostEqual(actual, expected, delta=1e-5, msg=f"Coefficient {i} mismatch for opencv fisheye")

    async def test_compute_relative_pose(self):
        """Test computing relative pose between two camera prims"""
        # Create two cameras with known relative pose
        left_camera = Camera(
            prim_path="/left_camera",
            name="left_camera",
            resolution=(2048, 1024),
            position=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )

        right_camera = Camera(
            prim_path="/right_camera",
            name="right_camera",
            resolution=(2048, 1024),
            position=np.array([0.1, 0, 0]),  # 10cm baseline in X direction
            orientation=np.array([1, 0, 0, 0]),
        )

        await omni.kit.app.get_app().next_update_async()

        translation, orientation = compute_relative_pose(
            left_camera_prim=self._stage.GetPrimAtPath("/left_camera"),
            right_camera_prim=self._stage.GetPrimAtPath("/right_camera"),
        )

        # Verify the results
        self.assertIsNotNone(translation)
        self.assertIsNotNone(orientation)

        # Check that translation is as expected (0.0, 0.0, 0.1)
        # Note translation is in camera frame (default orientation is +90x, -90y, hence +Z offset)
        self.assertAlmostEqual(translation[0], 0.0, delta=1e-5)
        self.assertAlmostEqual(translation[1], 0.0, delta=1e-5)
        self.assertAlmostEqual(translation[2], 0.1, delta=1e-5)

        # Orientation should be identity (no rotation)
        expected_orientation = np.eye(3)
        np.testing.assert_array_almost_equal(orientation, expected_orientation, decimal=5)
