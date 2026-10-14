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

import os
import tempfile

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import omni.usd
import yaml
from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdGeom, UsdPhysics

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
        """Create a clean stage."""
        await app_utils.update_app_async()
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Close the stage and wait for pending asset loads."""
        stage_utils.close_stage()
        await app_utils.update_app_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()

    async def test_grasp_pose_generation_cube(self) -> None:
        """Test grasp pose generation on a simple cube primitive."""
        if not check_grasp_pose_generation_dependencies():
            print("Warning: Skipping test because grasp pose generation dependencies are not installed.")
            return

        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()
        stage = stage_utils.get_current_stage()

        object_path = "/World/ObjectAsset"
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXformCommand", prim_type="Cube", prim_path=object_path)
        await app_utils.update_app_async()
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

    async def test_load_config_malformed_input(self) -> None:
        """Test that malformed phase entries and unknown components are reported instead of raising."""
        stage = stage_utils.get_current_stage()
        gripper_path = "/World/Gripper"
        joint_path = f"{gripper_path}/Joint"
        UsdGeom.Xform.Define(stage, gripper_path)
        UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        await app_utils.update_app_async()

        def load_config(config: dict, components: list[str]) -> tuple[GraspingManager, dict[str, str]]:
            grasping_manager = GraspingManager()
            grasping_manager.set_gripper(gripper_path)
            with tempfile.TemporaryDirectory() as temp_dir:
                config_path = os.path.join(temp_dir, "config.yaml")
                with open(config_path, "w") as file:
                    yaml.dump(config, file)
                return grasping_manager, grasping_manager.load_config(config_path, components=components)

        # Missing required `name` key is skipped, remaining valid phases are loaded
        manager, status = load_config(
            {"grasp_phases": [{"simulation_steps": 8}, {"name": "close", "joint_drive_targets": {joint_path: 1.0}}]},
            ["phases"],
        )
        self.assertEqual(status["phases"], "Success (some entries skipped due to errors)")
        self.assertEqual(manager.get_grasp_phase_names(), ["close"])

        # Only malformed phase entries fail the component
        manager, status = load_config({"grasp_phases": [{"joint_drive_targets": {}}, 42]}, ["phases"])
        self.assertTrue(status["phases"].startswith("Failed"))
        self.assertEqual(manager.grasp_phases, [])

        # Unknown phase keys are ignored
        manager, status = load_config({"grasp_phases": [{"name": "close", "bogus_key": 42}]}, ["phases"])
        self.assertEqual(status["phases"], "Success")
        self.assertEqual(manager.get_grasp_phase_names(), ["close"])

        # Non-dictionary joint targets are skipped
        manager, status = load_config({"grasp_phases": [{"name": "close", "joint_drive_targets": 5}]}, ["phases"])
        self.assertTrue(status["phases"].startswith("Failed"))

        # Unknown component names are reported as failed, valid ones still load
        manager, status = load_config({"gripper_path": gripper_path}, ["grip", "gripper"])
        self.assertEqual(status, {"grip": "Failed (Unknown component)", "gripper": "Success"})

    async def test_grasp_pose_generation_assets(self) -> None:
        """Test grasp pose generation on a referenced USD asset."""
        if not check_grasp_pose_generation_dependencies():
            print("Warning: Skipping test because grasp pose generation dependencies are not installed.")
            return

        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

        assets_root_path = await get_assets_root_path_async()
        asset_url = assets_root_path + OBJECT_ASSET_URL
        asset_path = "/World/ObjectAsset"
        object_prim = stage_utils.define_prim(asset_path, type_name="Xform")
        object_prim.GetReferences().AddReference(asset_url)

        grasping_manager = GraspingManager()
        grasping_manager.set_object_prim_path(asset_path)
        self.assertEqual(grasping_manager.get_object_prim_path(), asset_path)

        success_generation = grasping_manager.generate_grasp_poses(config=DEFAULT_SAMPLER_CONFIG)
        self.assertTrue(success_generation)
        self.assertTrue(len(grasping_manager.grasp_locations) > 0)
