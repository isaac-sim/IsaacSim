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

"""Verifies public prim wrapper classes have complete API docstring coverage. Covers Prim, XformPrim, GeomPrim, RigidPrim, Articulation, and DeformablePrim exports."""

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.test.docstring
from isaacsim.core.experimental.prims import Articulation, DeformablePrim, GeomPrim, Prim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async

from . import common
from .test_deformable_prim import _define_tetmesh


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Test extension docstrings."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim(f"/World", "Xform")
        # configure simulation
        SimulationManager.set_physics_sim_device("cpu")

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_prim_docstrings(self) -> None:
        """Test prim docstrings."""
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
        # test case
        await self.assertDocTests(Prim)

    async def test_xform_prim_docstrings(self) -> None:
        """Test xform prim docstrings."""
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
        # test case
        await self.assertDocTests(XformPrim)

    async def test_geom_prim_docstrings(self) -> None:
        """Test geom prim docstrings."""
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
            stage_utils.define_prim(f"/World/prim_{i}/Cube", "Cube")
        # test case
        await self.assertDocTests(GeomPrim)

    @common.requires_engines(supported_engines=["physx"])
    async def test_rigid_prim_docstrings(self) -> None:
        """Test rigid prim docstrings."""
        # define prims
        for i in range(3):
            stage_utils.define_prim(f"/World/prim_{i}", "Xform")
            stage_utils.define_prim(f"/World/prim_{i}/Cube", "Cube")
        # test case
        await self.assertDocTests(RigidPrim)

    @common.requires_engines(supported_engines=["physx"])
    async def test_articulation_docstrings(self) -> None:
        """Test articulation docstrings."""
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

    @common.requires_engines(supported_engines=["physx"])
    async def test_deformable_prim_docstrings(self) -> None:
        """Test deformable prim docstrings."""
        # define prims
        for i in range(3):
            _define_tetmesh(stage_utils.get_current_stage(), f"/World/prim_{i}")
        # test case
        SimulationManager.set_physics_sim_device("cuda")  # deformable prims are only supported on GPU
        await self.assertDocTests(DeformablePrim)
        SimulationManager.set_physics_sim_device("cpu")
