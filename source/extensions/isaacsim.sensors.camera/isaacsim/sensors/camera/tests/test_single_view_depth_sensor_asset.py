# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import omni.kit.test
import omni.usd
from isaacsim.core.utils.stage import create_new_stage_async, is_stage_loading, update_stage_async
from isaacsim.sensors.camera import SingleViewDepthSensor, SingleViewDepthSensorAsset
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom


class TestSingleViewDepthSensorAsset(omni.kit.test.AsyncTestCase):
    """Test suite for SingleViewDepthSensorAsset class."""

    async def setUp(self):
        """Set up test environment by creating a new stage."""
        await create_new_stage_async()
        await update_stage_async()
        while is_stage_loading():
            await update_stage_async()
        self.assets_root_path = get_assets_root_path()
        return

    async def test_valid_asset_path_initialization_and_depth_sensor_access(self):
        """Test valid asset path initialization and depth sensor access."""
        # Create SingleViewDepthSensorAsset with valid asset path
        depth_sensor_asset = SingleViewDepthSensorAsset(
            prim_path="/sensor", asset_path=f"{self.assets_root_path}/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        )

        # Test get_child_depth_sensor with valid camera_prim_path
        valid_camera_prim_path = "/sensor/RSD455/Camera_Pseudo_Depth"
        depth_sensor = depth_sensor_asset.get_child_depth_sensor(valid_camera_prim_path)

        # Confirm it returns a SingleViewDepthSensor object
        self.assertIsInstance(depth_sensor, SingleViewDepthSensor)
        self.assertEqual(depth_sensor.prim_path, valid_camera_prim_path)

        # Test get_child_depth_sensor with invalid camera_prim_path raises KeyError
        invalid_camera_prim_path = "/foo"
        with self.assertRaises(KeyError):
            depth_sensor_asset.get_child_depth_sensor(invalid_camera_prim_path)

        # Initialize the depth sensor asset
        depth_sensor_asset.initialize()

        # Confirm the SingleViewDepthSensor object at the valid camera_prim_path is enabled
        depth_sensor = depth_sensor_asset.get_child_depth_sensor(valid_camera_prim_path)
        self.assertTrue(depth_sensor.get_enabled())

    async def test_add_template_render_product_static_method(self):
        """Test the add_template_render_product static method."""
        stage = omni.usd.get_context().get_stage()

        # Create a Camera on the stage at prim path "/camera"
        camera_prim = UsdGeom.Camera.Define(stage, "/camera")

        # Create an Xform on the stage at prim path "/template"
        template_prim = UsdGeom.Xform.Define(stage, "/template")

        # Test with invalid camera_prim_path "/foo" - should return invalid prim
        invalid_prim = SingleViewDepthSensorAsset.add_template_render_product(
            parent_prim_path="/template", camera_prim_path="/foo"
        )
        self.assertFalse(invalid_prim.IsValid())

        # Test with parent_prim_path "/template" and camera_prim_path "/template" - should return invalid prim
        # (because "/template" is an Xform, not a Camera)
        invalid_prim_2 = SingleViewDepthSensorAsset.add_template_render_product(
            parent_prim_path="/template", camera_prim_path="/template"
        )
        self.assertFalse(invalid_prim_2.IsValid())

        # Test with valid parameters - should create a valid RenderProduct prim
        valid_prim = SingleViewDepthSensorAsset.add_template_render_product(
            parent_prim_path="/template", camera_prim_path="/camera"
        )
        self.assertTrue(valid_prim.IsValid())
        self.assertEqual(valid_prim.GetTypeName(), "RenderProduct")

        # Confirm the prim has the "OmniSensorDepthSensorSingleViewAPI" API applied
        self.assertTrue(valid_prim.HasAPI("OmniSensorDepthSensorSingleViewAPI"))

        # Confirm it has "/camera" as a target in its "camera" Relationship
        camera_rel = valid_prim.GetRelationship("camera")
        self.assertTrue(camera_rel.IsValid())
        targets = camera_rel.GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(str(targets[0]), "/camera")

        # Verify the render product was created at the expected path
        expected_path = "/template/camera_render_product"
        self.assertEqual(str(valid_prim.GetPath()), expected_path)
