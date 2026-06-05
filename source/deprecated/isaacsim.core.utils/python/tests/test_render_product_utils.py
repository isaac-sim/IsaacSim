# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for render product utility functions."""

import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.utils.render_product import (
    add_aov,
    get_camera_prim_path,
    get_resolution,
    set_camera_prim_path,
    set_resolution,
)


class TestRenderProduct(omni.kit.test.AsyncTestCase):
    """Test cases for RenderProduct."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_hydra_texture(self) -> None:
        """Test hydra texture."""
        await omni.kit.app.get_app().next_update_async()
        hydra_texture = rep.create.render_product("/OmniverseKit_Persp", [512, 512], name="Isaac")
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_camera_prim_path(hydra_texture.path), "/OmniverseKit_Persp")
        add_aov(hydra_texture.path, "RtxSensorCpu")
        await omni.kit.app.get_app().next_update_async()
        get_camera_prim_path(hydra_texture.path)
        set_camera_prim_path(hydra_texture.path, "/OmniverseKit_Top")
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_camera_prim_path(hydra_texture.path), "/OmniverseKit_Top")
        self.assertEqual(get_resolution(hydra_texture.path), (512, 512))
        await omni.kit.app.get_app().next_update_async()
        set_resolution(hydra_texture.path, (1024, 1024))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_resolution(hydra_texture.path), (1024, 1024))
        hydra_texture = None
        await omni.kit.app.get_app().next_update_async()
