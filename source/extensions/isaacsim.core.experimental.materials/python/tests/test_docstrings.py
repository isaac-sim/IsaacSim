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

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.test.docstring
from isaacsim.core.experimental.materials import (
    OmniGlassMaterial,
    OmniPbrMaterial,
    PhysicsMaterial,
    PreviewSurfaceMaterial,
    RigidBodyMaterial,
    SurfaceDeformableMaterial,
    VisualMaterial,
    VolumeDeformableMaterial,
)
from isaacsim.core.simulation_manager import SimulationManager


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim(f"/World", "Xform")
        # configure simulation
        SimulationManager.set_physics_sim_device("cpu")

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_physics_material_rigid_body_docstrings(self):
        await self.assertDocTests(RigidBodyMaterial)
        await self.assertDocTests(PhysicsMaterial)

    async def test_physics_material_surface_deformable_docstrings(self):
        await self.assertDocTests(SurfaceDeformableMaterial)
        await self.assertDocTests(PhysicsMaterial)

    async def test_physics_material_volume_deformable_docstrings(self):
        await self.assertDocTests(VolumeDeformableMaterial)
        await self.assertDocTests(PhysicsMaterial)

    # --------------------------------------------------------------------

    async def test_visual_material_omni_glass_docstrings(self):
        await self.assertDocTests(OmniGlassMaterial)
        await self.assertDocTests(VisualMaterial)

    async def test_visual_material_omni_pbr_docstrings(self):
        await self.assertDocTests(OmniPbrMaterial)
        await self.assertDocTests(VisualMaterial)

    async def test_visual_material_preview_surface_docstrings(self):
        await self.assertDocTests(PreviewSurfaceMaterial)
        await self.assertDocTests(VisualMaterial)
