# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test the grasping manager functionality."""

import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdGeom

from .common import check_grasp_pose_generation_dependencies

DEFAULT_SAMPLER_CONFIG = {
    "sampler_type": "antipodal",
    "num_candidates": 10,
    "num_orientations": 1,
    "gripper_maximum_aperture": 0.2,
    "gripper_standoff_fingertips": 0.2,
    "gripper_approach_direction": (0, 0, 1),
    "grasp_align_axis": (0, 1, 0),
    "orientation_sample_axis": (0, 1, 0),
    "lateral_sigma": 0.0,
    "random_seed": 12,
    "verbose": True,
}

OBJECT_ASSET_URL = "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"


class TestGraspingManager(omni.kit.test.AsyncTestCase):
    """Test grasp pose generation with the grasping manager."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_grasp_pose_generation_cube(self) -> None:
        """Test grasp pose generation on a simple cube primitive."""
        if not check_grasp_pose_generation_dependencies():
            print("Warning: Skipping test because grasp pose generation dependencies are not installed.")
            return

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()

        object_path = "/World/ObjectAsset"
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXformCommand", prim_type="Cube", prim_path=object_path)
        await omni.kit.app.get_app().next_update_async()
        object_prim = stage.GetPrimAtPath(object_path)
        if not object_prim.HasAttribute("xformOp:scale"):
            UsdGeom.Xformable(object_prim).AddScaleOp()
        object_prim.GetAttribute("xformOp:scale").Set((0.1, 0.1, 0.1))

        grasping_manager = GraspingManager()
        grasping_manager.set_object_prim_path(object_path)
        self.assertEqual(grasping_manager.get_object_prim_path(), object_path)

        success_generation = grasping_manager.generate_grasp_poses(config=DEFAULT_SAMPLER_CONFIG)
        self.assertTrue(success_generation)
        self.assertTrue(len(grasping_manager.grasp_locations) > 0)

    async def test_grasp_pose_generation_assets(self) -> None:
        """Test grasp pose generation on a referenced USD asset."""
        if not check_grasp_pose_generation_dependencies():
            print("Warning: Skipping test because grasp pose generation dependencies are not installed.")
            return

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()

        assets_root_path = await get_assets_root_path_async()
        asset_url = assets_root_path + OBJECT_ASSET_URL
        asset_path = "/World/ObjectAsset"
        object_prim = stage.DefinePrim(asset_path, "Xform")
        object_prim.GetReferences().AddReference(asset_url)

        grasping_manager = GraspingManager()
        grasping_manager.set_object_prim_path(asset_path)
        self.assertEqual(grasping_manager.get_object_prim_path(), asset_path)

        success_generation = grasping_manager.generate_grasp_poses(config=DEFAULT_SAMPLER_CONFIG)
        self.assertTrue(success_generation)
        self.assertTrue(len(grasping_manager.grasp_locations) > 0)
