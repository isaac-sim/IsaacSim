# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for external dependencies and asset conversion functionality."""

import os

import carb
import omni.kit.app
import omni.kit.test


class TestExternalDependencies(omni.kit.test.AsyncTestCase):
    """Tests for external dependencies and asset conversion."""

    async def setUp(self):
        """Set up test environment with new stage."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self.ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = self.ext_manager.get_enabled_extension_id("isaacsim.test.collection")
        self._extension_path = self.ext_manager.get_extension_path(ext_id)

    async def tearDown(self):
        """Clean up test environment."""
        await omni.kit.app.get_app().next_update_async()

    async def test_asset_converter(self):
        """Test that OBJ files can be converted to USD format."""
        import omni.kit.asset_converter

        def progress_callback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        # converter_context.ignore_materials = False
        # converter_context.ignore_animation = False
        # converter_context.ignore_cameras = True
        # converter_context.single_mesh = True
        # converter_context.smooth_normals = True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        # converter_context.use_meter_as_world_unit = True
        # converter_context.create_world_as_default_root_prim = False
        instance = omni.kit.asset_converter.get_instance()

        input_obj = os.path.abspath(self._extension_path + "/data/tests/test_mtl/test_mtl.obj")
        output_usd = os.path.abspath(self._extension_path + "/data/tests/test_mtl/test_mtl.usd")
        task = instance.create_converter_task(input_obj, output_usd, progress_callback, converter_context)
        success = await task.wait_until_finished()
        if not success:
            carb.log_error(f"{task.get_status()}, {task.get_error_message()}")
        print("converting done")
        self.assertTrue(os.path.isfile(output_usd))
