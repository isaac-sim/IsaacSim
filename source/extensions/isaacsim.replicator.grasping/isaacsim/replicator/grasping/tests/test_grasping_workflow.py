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

"""Test the grasping workflow end-to-end."""

from __future__ import annotations

import os

import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils.app import get_extension_path
from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from isaacsim.storage.native import get_assets_root_path_async

from .common import check_grasp_pose_generation_dependencies


class TestGraspingWorkflow(omni.kit.test.AsyncTestCase):
    """Test the grasping workflow with configuration and evaluation."""

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

    async def test_grasping_workflow_example(self) -> None:
        """Test the full grasping workflow with config loading and pose evaluation."""

        async def run_example_async(
            stage_path: str | None = None,
            config_path: str | None = None,
            sampler_config: dict | None = None,
            physics_scene_path: str | None = None,
            output_dir: str | None = None,
            gripper_path: str | None = None,
            object_prim_path: str | None = None,
        ) -> None:
            assets_root_path = await get_assets_root_path_async()
            print(f"Assets root path: {assets_root_path}")
            stage_url = assets_root_path + stage_path
            print(f"Opening stage: {stage_url}")
            await omni.usd.get_context().open_stage_async(stage_url)
            stage = omni.usd.get_context().get_stage()

            grasping_manager = GraspingManager()

            if config_path is not None:
                load_status = grasping_manager.load_config(config_path)
                print(f"Config load status: {load_status}")

            # Make sure the object to grasp is set (either from the config file or from the argument)
            if not grasping_manager.get_object_prim_path() and object_prim_path:
                grasping_manager.object_path = object_prim_path

            if not grasping_manager.get_object_prim_path():
                print("Warning: Object to grasp is not set (missing in config and argument). Aborting.")
                return

            # Make sure the gripper is set (either from the config file or from the argument)
            if not grasping_manager.gripper_path and gripper_path:
                grasping_manager.gripper_path = gripper_path

            if not grasping_manager.gripper_path:
                print("Warning: Gripper path is not set (missing in config and argument). Aborting.")
                return

            # If there are already grasp poses in the configuration, don't generate new ones
            if grasping_manager.grasp_locations:
                print(
                    f"Found {len(grasping_manager.grasp_locations)} grasp poses in the configuration file. No new poses will be generated."
                )
            else:
                print("No grasp poses found in configuration, generating new ones...")

                # Determine Sampler Configuration
                if not (grasping_manager.sampler_config and grasping_manager.sampler_config.get("sampler_type")):
                    if sampler_config:
                        grasping_manager.sampler_config = sampler_config.copy()
                    else:
                        print(
                            "Warning: Sampler configuration is missing or invalid (not in config file and not provided as argument). Aborting pose generation."
                        )
                        return

                # Generate the grasp poses
                success_generation = grasping_manager.generate_grasp_poses()
                if not success_generation or not grasping_manager.grasp_locations:
                    print("Failed to generate grasp poses or no poses were generated.")
                    return
                print(f"Generated {len(grasping_manager.grasp_locations)} new grasp poses.")

            # Store the initial gripper pose to be able to restore it after the evaluation
            grasping_manager.store_initial_gripper_pose()

            print("Evaluating grasp poses...")
            poses_to_evaluate = grasping_manager.get_grasp_poses(in_world_frame=True)
            if not poses_to_evaluate:
                print("No poses available to evaluate..")
                return

            # Determine Output Path
            if not output_dir:
                print("Warning: Output path is not defined data will not be saved.")

            # Set the output path and overwrite flag
            grasping_manager.set_results_output_dir(output_dir)
            grasping_manager.set_overwrite_results_output(True)

            # Determine Physics Scene Path
            physics_scene_path_for_eval = None
            if physics_scene_path and stage.GetPrimAtPath(physics_scene_path):
                physics_scene_path_for_eval = physics_scene_path
            print(f"Physics scene path for evaluation: {physics_scene_path_for_eval}")

            await grasping_manager.evaluate_grasp_poses(
                grasp_poses=poses_to_evaluate,
                render=True,
                physics_scene_path=physics_scene_path_for_eval,
                simulate_using_timeline=False,
            )

            print("Grasping workflow example finished.")
            grasping_manager.clear()

        stage_path = "/Isaac/Samples/Replicator/Stage/sdg_grasping_xarm.usd"

        ext_path = get_extension_path("isaacsim.replicator.grasping")
        config_path = os.path.join(ext_path, "data/gripper_configs/xarm_antipodal_soup_can.yaml")
        output_dir = os.path.join(os.getcwd(), "xarm_antipodal")

        await run_example_async(stage_path=stage_path, config_path=config_path, output_dir=output_dir)

        if not check_grasp_pose_generation_dependencies():
            print("Warning: Skipping test because grasp pose generation dependencies are not installed.")
            return

        # Check that the expected files are present in the output directory
        expected_num_files = 4
        self.assertEqual(len(os.listdir(output_dir)), expected_num_files)
