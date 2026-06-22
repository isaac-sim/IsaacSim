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

"""Verifies the deprecated LidarRtx wrapper for RTX lidar prim construction and runtime access. Covers render product paths, annotator and writer attachment, frame reads during timeline playback, and behavior after detach."""

import asyncio
from typing import Any

import numpy as np
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.rtx import LidarRtx
from pxr import Gf, UsdGeom


class TestLidarRtx(omni.kit.test.AsyncTestCase):
    """Test class for LidarRtx functionality."""

    # Class constants
    ALLOWED_ANNOTATORS = [
        "IsaacComputeRTXLidarFlatScan",
        "IsaacExtractRTXSensorPointCloudNoAccumulator",
        "IsaacCreateRTXLidarScanBuffer",
        "GenericModelOutput",
        "StableIdMap",
    ]
    """List of annotator names that are supported by the LidarRtx sensor for data processing and extraction."""

    EXPECTED_FRAME_KEYS = [
        "rendering_time",
        "rendering_frame",
        "linear_depth_data",
        "intensities_data",
        "azimuth_range",
        "horizontal_resolution",
    ]
    """List of expected keys that should be present in the frame data dictionary returned by get_current_frame."""

    async def setUp(self) -> None:
        """Set up the test environment with a new stage and world."""
        await create_new_stage_async()
        self.world = World(stage_units_in_meters=1.0)
        await self.world.initialize_simulation_context_async()
        await update_stage_async()

        # Get stage reference
        self.stage = omni.usd.get_context().get_stage()

        # Create an OmniLidar prim and apply OmniSensorGenericLidarCoreAPI schema
        self.lidar_prim_path = "/World/valid_lidar"
        lidar_prim = self.stage.DefinePrim(self.lidar_prim_path, "OmniLidar")
        lidar_prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")

        # Verify our prim was created with the right type and API
        self.assertEqual(lidar_prim.GetTypeName(), "OmniLidar")
        self.assertTrue(lidar_prim.HasAPI("OmniSensorGenericLidarCoreAPI"))

        # Create a Camera prim for testing constructor with Camera
        self.camera_prim_path = "/World/test_camera"
        camera_prim = self.stage.DefinePrim(self.camera_prim_path, "Camera")

        # Create an Xform prim for testing constructor with invalid prim type
        self.xform_prim_path = "/World/test_xform"
        xform_prim = self.stage.DefinePrim(self.xform_prim_path, "Xform")

        # Create an OmniLidar prim without the required API schema
        self.invalid_lidar_prim_path = "/World/invalid_lidar"
        invalid_lidar_prim = self.stage.DefinePrim(self.invalid_lidar_prim_path, "OmniLidar")

        await self.world.reset_async()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Clean up after tests."""
        self.world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def advance_frames(self, render_product_path: Any, num_frames: int = 1) -> None:
        """Helper method to advance frames.

        Args:
            render_product_path: Path of render product for the sensor.
            num_frames: Number of frames to advance.
        """
        for _ in range(num_frames):
            await omni.kit.app.get_app().next_update_async()

    def verify_annotators_added(self, lidar: Any, annotator_names: Any) -> None:
        """Helper method to verify annotators are correctly added.

        Args:
            lidar: LidarRtx instance to check.
            annotator_names: List of annotator names that should be present.
        """
        annotators = lidar.get_annotators()
        for name in annotator_names:
            self.assertIn(name, annotators)

    def attach_all_annotators(self, lidar: Any) -> None:
        """Helper method to attach all annotators to a lidar.

        Args:
            lidar: LidarRtx instance to attach annotators to.
        """
        for annotator_name in self.ALLOWED_ANNOTATORS:
            lidar.attach_annotator(annotator_name)
        self.verify_annotators_added(lidar, self.ALLOWED_ANNOTATORS)

    async def test_constructor_with_valid_lidar_prim(self) -> None:
        """Test that constructor works with valid OmniLidar prim path."""
        # This should not raise an exception
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="valid_lidar_instance")
        self.assertIsNotNone(lidar)
        self.assertEqual(lidar.prim_path, self.lidar_prim_path)

        # Verify accumulateOutputs is set to True by the constructor
        sensor_prim = self.stage.GetPrimAtPath(self.lidar_prim_path)
        attr = sensor_prim.GetAttribute("omni:sensor:Core:accumulateOutputs")
        if attr.IsValid():
            self.assertTrue(attr.Get(), "Expected accumulateOutputs to be True after LidarRtx construction.")

    async def test_constructor_with_camera_prim(self) -> None:
        """Test constructor with Camera prim."""
        # Test position and orientation in constructor
        test_position = np.array([1.0, 2.0, 3.0])
        test_orientation = np.array([0.0, 0.0, 0.707, 0.707])  # 90 degrees around Z

        lidar = LidarRtx(
            prim_path=self.camera_prim_path,
            name="camera_lidar_instance",
            position=test_position,
            orientation=test_orientation,
        )

        # Get the actual position and orientation
        camera_prim = self.stage.GetPrimAtPath(self.camera_prim_path)
        xform = UsdGeom.Xformable(camera_prim)

        # Get world transform matrix
        world_transform = xform.ComputeLocalToWorldTransform(0)

        # Extract position from transform
        actual_position = world_transform.ExtractTranslation()

        # Extract rotation from transform (as quaternion)
        rotation = world_transform.ExtractRotationQuat()

        # Verify position matches what was set
        np.testing.assert_array_almost_equal(
            np.array([actual_position[0], actual_position[1], actual_position[2]]), test_position, decimal=5
        )

        # Verify orientation matches what was set (allowing for quaternion sign differences)
        actual_quat = np.array(
            [rotation.GetReal(), rotation.GetImaginary()[0], rotation.GetImaginary()[1], rotation.GetImaginary()[2]]
        )

        self.assertTrue(
            np.allclose(actual_quat, test_orientation, atol=1e-3)
            or np.allclose(-actual_quat, test_orientation, atol=1e-3)
        )

    async def test_constructor_with_xform_prim(self) -> None:
        """Test constructor with Xform prim (should raise Exception)."""
        with self.assertRaises(Exception):
            lidar = LidarRtx(prim_path=self.xform_prim_path, name="xform_lidar_instance")

    async def test_constructor_with_invalid_lidar_prim(self) -> None:
        """Test constructor with OmniLidar prim without required API schema (should raise Exception)."""
        with self.assertRaises(Exception):
            lidar = LidarRtx(prim_path=self.invalid_lidar_prim_path, name="invalid_lidar_instance")

    async def test_constructor_preserves_existing_position(self) -> None:
        """Test that constructor preserves existing prim position and orientation when not specified."""
        # Set the lidar prim to position (5, 0, 0) and orientation (45 degrees around Z)
        target_position = np.array([5.0, 0.0, 0.0])
        target_orientation = np.array([0.0, 0.0, 0.383, 0.924])  # 45 degrees around Z

        lidar_prim = self.stage.GetPrimAtPath(self.lidar_prim_path)
        xform = UsdGeom.Xformable(lidar_prim)

        # Set the translation
        xform.AddTranslateOp().Set(Gf.Vec3d(target_position[0], target_position[1], target_position[2]))

        # Set the orientation
        target_quat = Gf.Quatf(
            target_orientation[3], target_orientation[0], target_orientation[1], target_orientation[2]
        )
        xform.AddOrientOp().Set(target_quat)

        # Construct LidarRtx object without specifying position or orientation
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="position_preserve_test")

        # Verify the prim is still at position (5, 0, 0)
        world_transform = xform.ComputeLocalToWorldTransform(0)
        actual_position = world_transform.ExtractTranslation()

        np.testing.assert_array_almost_equal(
            np.array([actual_position[0], actual_position[1], actual_position[2]]),
            target_position,
            decimal=5,
            err_msg=f"Actual position {actual_position} does not match target position {target_position}",
        )

        # Verify the prim orientation is preserved
        rotation = world_transform.ExtractRotationQuat()
        actual_quat = np.array(
            [rotation.GetImaginary()[0], rotation.GetImaginary()[1], rotation.GetImaginary()[2], rotation.GetReal()]
        )

        # Verify orientation matches what was set (allowing for quaternion sign differences)
        self.assertTrue(
            np.allclose(actual_quat, target_orientation, atol=1e-3)
            or np.allclose(-actual_quat, target_orientation, atol=1e-3),
            f"Actual quaternion {actual_quat} does not match target quaternion {target_orientation}",
        )

    async def test_get_render_product_path(self) -> None:
        """Test get_render_product_path returns a valid path with correct prim type."""
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="render_path_test")

        # Get the render product path
        render_product_path = lidar.get_render_product_path()

        # Verify it's not None and is a string
        self.assertIsNotNone(render_product_path)
        self.assertIsInstance(render_product_path, str)

        # Verify the prim at this path exists and is the right type
        render_product_prim = self.stage.GetPrimAtPath(render_product_path)
        self.assertTrue(render_product_prim.IsValid())
        self.assertEqual(render_product_prim.GetTypeName(), "RenderProduct")

    async def test_annotator_methods(self) -> None:
        """Test attach_annotator, detach_annotator, get_annotators, and detach_all_annotators."""
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="annotator_test")

        # Test each annotator individually
        for annotator_name in self.ALLOWED_ANNOTATORS:
            # Initially, get_annotators should return an empty dictionary for this annotator
            annotators = lidar.get_annotators()
            self.assertNotIn(annotator_name, annotators)

            # Attach the annotator
            lidar.attach_annotator(annotator_name)

            # Verify it was added with get_annotators
            annotators = lidar.get_annotators()
            self.assertIn(annotator_name, annotators)

            # Detach the annotator
            lidar.detach_annotator(annotator_name)

            # Verify it was removed with get_annotators
            annotators = lidar.get_annotators()
            self.assertNotIn(annotator_name, annotators)

        # Now add all annotators at once
        self.attach_all_annotators(lidar)

        # Detach all annotators
        lidar.detach_all_annotators()

        # Verify all were removed
        annotators = lidar.get_annotators()
        self.assertEqual(len(annotators), 0)

    async def test_annotator_kwarg_initialization(self) -> None:
        """Test that annotators can be initialized with kwargs."""
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="kwarg_test")

        # Test IsaacCreateRTXLidarScanBuffer with specific kwargs
        scan_buffer_kwargs = {
            "outputIntensity": True,
            "outputDistance": True,
            "outputObjectId": False,
            "outputVelocity": True,
            "outputAzimuth": True,
            "outputElevation": False,
            "outputHitNormal": True,
            "outputTimestamp": True,
            "outputEmitterId": False,
            "outputMaterialId": True,
        }

        # Attach annotator with kwargs - this tests that kwargs are properly passed to initialize()
        lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer", **scan_buffer_kwargs)

        # Test individual kwargs as well
        lidar.detach_annotator("IsaacCreateRTXLidarScanBuffer")
        lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer", outputIntensity=True, outputDistance=True)

        # Verify all annotators are attached
        annotators = lidar.get_annotators()
        self.assertIn("IsaacCreateRTXLidarScanBuffer", annotators)

    async def test_writer_methods(self) -> None:
        """Test get_writers, attach_writer, detach_writer, and detach_all_writers."""
        lidar = LidarRtx(prim_path=self.lidar_prim_path, name="writer_test")

        # Use RtxLidarDebugDrawPointCloud as the test writer
        writer_name = "RtxLidarDebugDrawPointCloud"

        # Initially, get_writers should return an empty dictionary
        writers = lidar.get_writers()
        self.assertNotIn(writer_name, writers)

        # Attach the writer
        lidar.attach_writer(writer_name)

        # Verify it was added with get_writers
        writers = lidar.get_writers()
        self.assertIn(writer_name, writers)

        # Detach the writer
        lidar.detach_writer(writer_name)

        # Verify it was removed with get_writers
        writers = lidar.get_writers()
        self.assertNotIn(writer_name, writers)

        # Add the writer again
        lidar.attach_writer(writer_name)

        # Verify it was added
        writers = lidar.get_writers()
        self.assertIn(writer_name, writers)

        # Detach all writers
        lidar.detach_all_writers()

        # Verify all were removed
        writers = lidar.get_writers()
        self.assertEqual(len(writers), 0)

        # Test visualization methods
        lidar.enable_visualization()

        # Let the stage update a few times
        for _ in range(3):
            await update_stage_async()

        lidar.disable_visualization()

        # Let the stage update a few times
        for _ in range(3):
            await update_stage_async()

    async def test_timeline_and_get_current_frame(self) -> None:
        """Test timeline events and get_current_frame method."""
        # Create a LidarRtx object with config_file_name set to "Example_Rotary"
        lidar = LidarRtx(
            prim_path="/World/timeline_test_lidar", name="timeline_test_lidar", config_file_name="Example_Rotary"
        )

        # Call initialize
        lidar.initialize()

        # Attach all possible annotators
        self.attach_all_annotators(lidar)

        # Advance the timeline by 3 frames
        self._timeline.play()
        await self.advance_frames(lidar.get_render_product_path(), 3)

        # Store the return value of get_current_frame
        current_frame = lidar.get_current_frame()

        # Confirm current_frame is a dict
        self.assertIsInstance(current_frame, dict)

        # Confirm current_frame contains expected keys
        for key in self.EXPECTED_FRAME_KEYS:
            self.assertIn(key, current_frame, f"Missing expected key: {key}")

        # Confirm current_frame contains keys for each annotator
        for annotator_name in self.ALLOWED_ANNOTATORS:
            self.assertIn(annotator_name, current_frame, f"Missing annotator data: {annotator_name}")

        # Test pause functionality
        lidar.pause()
        self.assertTrue(lidar.is_paused())

        # Render 3 more frames while paused
        await self.advance_frames(lidar.get_render_product_path(), 3)

        # Confirm still paused
        self.assertTrue(lidar.is_paused())

        # Test resume functionality
        lidar.resume()
        self.assertFalse(lidar.is_paused())

        # Render 3 more frames
        await self.advance_frames(lidar.get_render_product_path(), 3)

        # Test timeline pause (should pause the lidar)
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(lidar.is_paused())

        # Test timeline play (should resume the lidar)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(lidar.is_paused())

        # Render 10 frames
        await self.advance_frames(lidar.get_render_product_path(), 10)

        # Get the updated frame after all rendering
        current_frame = lidar.get_current_frame()

        # Verify annotator and frame data
        # TOOD - re-enable in later release.
        # Pausing and resuming the timeline results in empty GMO buffers since the clock the lidar renderer uses
        # increases monotonically even while the animation timeline is paused. This means GMO timestamps will fall
        # out-of-sync when handed to LidarPointAccumulator, so a full scan is never accumulated.
        # self.verify_frame_data(lidar, current_frame)

    def verify_frame_data(self, lidar: Any, current_frame: Any) -> None:
        """Helper method to verify frame data.

        Args:
            lidar: LidarRtx instance to check.
            current_frame: The current frame dictionary to verify.
        """
        # Verify each annotator value is a non-empty dictionary
        for annotator_name in self.ALLOWED_ANNOTATORS:
            self.assertIn(annotator_name, current_frame, f"Missing annotator data: {annotator_name}")
            self.assertIsInstance(current_frame[annotator_name], dict, f"{annotator_name} value is not a dictionary")
            self.assertNotEqual(len(current_frame[annotator_name]), 0, f"{annotator_name} dictionary is empty")

        # Check that expected keys contain meaningful data
        for key in self.EXPECTED_FRAME_KEYS:
            if key != "rendering_time" and key != "rendering_frame":
                self.assertIsNotNone(current_frame[key], f"{key} has None value")
                if isinstance(current_frame[key], dict):
                    self.assertNotEqual(len(current_frame[key]), 0, f"{key} dictionary is empty")
                elif isinstance(current_frame[key], (list, np.ndarray)):
                    self.assertNotEqual(len(current_frame[key]), 0, f"{key} list/array is empty")

        # Confirm data consistency between current_frame and annotator data
        # self.verify_point_cloud_data(current_frame)
        self.verify_flat_scan_data(current_frame)

        # Test the deprecated getter methods to ensure they're not returning None
        self.verify_getter_methods(lidar, current_frame)

    def verify_point_cloud_data(self, current_frame: Any) -> None:
        """Helper method to verify point cloud data.

        Args:
            current_frame: The current frame dictionary to verify.
        """
        # Point cloud data check
        np.testing.assert_array_equal(
            current_frame["point_cloud_data"],
            current_frame["IsaacExtractRTXSensorPointCloud"]["data"],
            "point_cloud_data does not match IsaacExtractRTXSensorPointCloud data",
        )

    def verify_flat_scan_data(self, current_frame: Any) -> None:
        """Helper method to verify flat scan data.

        Args:
            current_frame: The current frame dictionary to verify.
        """
        np.testing.assert_array_equal(
            current_frame["linear_depth_data"],
            current_frame["IsaacComputeRTXLidarFlatScan"]["linearDepthData"],
            "linear_depth_data does not match IsaacComputeRTXLidarFlatScan linearDepthData",
        )

        np.testing.assert_array_equal(
            current_frame["intensities_data"],
            current_frame["IsaacComputeRTXLidarFlatScan"]["intensitiesData"],
            "intensities_data does not match IsaacComputeRTXLidarFlatScan intensitiesData",
        )

        self.assertEqual(
            current_frame["azimuth_range"],
            current_frame["IsaacComputeRTXLidarFlatScan"]["azimuthRange"],
            "azimuth_range does not match IsaacComputeRTXLidarFlatScan azimuthRange",
        )

        self.assertEqual(
            current_frame["horizontal_resolution"],
            current_frame["IsaacComputeRTXLidarFlatScan"]["horizontalResolution"],
            "horizontal_resolution does not match IsaacComputeRTXLidarFlatScan horizontalResolution",
        )

    def verify_getter_methods(self, lidar: Any, current_frame: Any) -> None:
        """Helper method to verify getter methods.

        Args:
            lidar: LidarRtx instance to check.
            current_frame: The current frame dictionary to verify.
        """
        # Test get_horizontal_resolution
        horizontal_resolution = lidar.get_horizontal_resolution()
        self.assertIsNotNone(horizontal_resolution)
        self.assertEqual(
            horizontal_resolution,
            current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalResolution"),
        )

        # Test get_horizontal_fov
        horizontal_fov = lidar.get_horizontal_fov()
        self.assertIsNotNone(horizontal_fov)
        self.assertEqual(horizontal_fov, current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalFov"))

        # Test get_num_rows
        num_rows = lidar.get_num_rows()
        self.assertIsNotNone(num_rows)
        self.assertEqual(num_rows, current_frame["IsaacComputeRTXLidarFlatScan"].get("numRows"))

        # Test get_num_cols
        num_cols = lidar.get_num_cols()
        self.assertIsNotNone(num_cols)
        self.assertEqual(num_cols, current_frame["IsaacComputeRTXLidarFlatScan"].get("numCols"))

        # Test get_rotation_frequency
        rotation_frequency = lidar.get_rotation_frequency()
        self.assertIsNotNone(rotation_frequency)
        self.assertEqual(rotation_frequency, current_frame["IsaacComputeRTXLidarFlatScan"].get("rotationRate"))

        # Test get_depth_range
        depth_range = lidar.get_depth_range()
        self.assertIsNotNone(depth_range)
        self.assertEqual(depth_range, current_frame["IsaacComputeRTXLidarFlatScan"].get("depthRange"))

        # Test get_azimuth_range
        azimuth_range = lidar.get_azimuth_range()
        self.assertIsNotNone(azimuth_range)
        self.assertEqual(azimuth_range, current_frame["IsaacComputeRTXLidarFlatScan"].get("azimuthRange"))

    async def test_getter_methods_after_detach(self) -> None:
        """Test getter methods after detaching annotator."""
        # Create a LidarRtx object
        lidar = LidarRtx(
            prim_path="/World/annotator_detach_test_lidar",
            name="annotator_detach_test",
            config_file_name="Example_Rotary",
        )
        lidar.initialize()

        # Add the required annotator
        lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")

        # Run the timeline to populate data
        self._timeline.play()
        await self.advance_frames(lidar.get_render_product_path(), 3)

        # Verify getters return values
        self.assertIsNotNone(lidar.get_horizontal_resolution())
        self.assertIsNotNone(lidar.get_horizontal_fov())
        self.assertIsNotNone(lidar.get_num_rows())
        self.assertIsNotNone(lidar.get_num_cols())
        self.assertIsNotNone(lidar.get_rotation_frequency())
        self.assertIsNotNone(lidar.get_depth_range())
        self.assertIsNotNone(lidar.get_azimuth_range())

        # Detach the annotator
        lidar.detach_annotator("IsaacComputeRTXLidarFlatScan")

        # Advance frames
        await self.advance_frames(lidar.get_render_product_path(), 3)

        # Verify all getters now return None
        self.assertIsNone(lidar.get_horizontal_resolution())
        self.assertIsNone(lidar.get_horizontal_fov())
        self.assertIsNone(lidar.get_num_rows())
        self.assertIsNone(lidar.get_num_cols())
        self.assertIsNone(lidar.get_rotation_frequency())
        self.assertIsNone(lidar.get_depth_range())
        self.assertIsNone(lidar.get_azimuth_range())
