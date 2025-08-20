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
from isaacsim.core.experimental.prims import Articulation, DeformablePrim, GeomPrim, Prim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async

from .test_deformable_prim import _define_tetmesh


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

    async def test_prim_docstrings(self):
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
        # test case
        await self.assertDocTests(Prim)

    async def test_xform_prim_docstrings(self):
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
        # test case
        await self.assertDocTests(XformPrim)

    async def test_geom_prim_docstrings(self):
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
            stage_utils.define_prim(f"/World/prim_{i}/Cube", "Cube")
        # test case
        await self.assertDocTests(GeomPrim)

    async def test_rigid_prim_docstrings(self):
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
            stage_utils.define_prim(f"/World/prim_{i}/Cube", "Cube")
        # test case
        await self.assertDocTests(RigidPrim)

    async def test_articulation_docstrings(self):
        # get assets root path
        assets_root_path = await get_assets_root_path_async()
        # define prims
        for i in range(3):
            stage_utils.add_reference_to_stage(
                f"{assets_root_path}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
                path=f"/World/prim_{i}",
                variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
            )
        # test case
        await self.assertDocTests(Articulation, stop_on_failure=False)

    async def test_deformable_prim_docstrings(self):
        # define prims
        for i in range(3):
            _define_tetmesh(stage_utils.get_current_stage(), f"/World/prim_{i}")
        # test case
        SimulationManager.set_physics_sim_device("cuda")  # deformable prims are only supported on GPU
        await self.assertDocTests(DeformablePrim)
        SimulationManager.set_physics_sim_device("cpu")
