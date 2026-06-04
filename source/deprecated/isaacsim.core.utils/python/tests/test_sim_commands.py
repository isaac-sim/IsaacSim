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

"""Tests for simulation command utilities."""

import asyncio
import gc

import carb

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.storage.native import get_assets_root_path_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestIsaacSimCommands(omni.kit.test.AsyncTestCase):
    """Test cases for IsaacSimCommands."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = omni.usd.get_context().get_stage()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        self._stage = None
        self._timeline = None
        gc.collect()

    async def test_spawn_command(self) -> None:
        """Test spawn command."""
        articulation_usd = self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        static_usd = self._assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        physics_usd = self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"

        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=(100, 100, 0)
        )
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=static_usd, prim_path="/klt", translation=(-100, 100, 0)
        )
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=physics_usd, prim_path="/block", translation=(100, -100, 0)
        )
        # we need to wait several frames for spawns to load
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(omni.usd.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka")).ExtractTranslation()),
            (100, 100, 0),
        )
        self.assertTupleEqual(
            tuple(omni.usd.get_world_transform_matrix(self._stage.GetPrimAtPath("/klt")).ExtractTranslation()),
            (-100, 100, 0),
        )
        self.assertTupleEqual(
            tuple(omni.usd.get_world_transform_matrix(self._stage.GetPrimAtPath("/block")).ExtractTranslation()),
            (100, -100, 0),
        )

    async def test_teleport_command(self) -> None:
        """Test teleport command."""
        articulation_usd = self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=None, rotation=None
        )
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("IsaacSimTeleportPrim", prim_path="/franka", translation=(-100, -100, 0))
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(omni.usd.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka")).ExtractTranslation()),
            (-100, -100, 0),
        )

    async def test_scale(self) -> None:
        """Test scale."""
        from pxr import Gf

        articulation_usd = self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=None, rotation=None
        )
        await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(self._stage.GetPrimAtPath("/franka"))
        omni.kit.commands.execute("IsaacSimScalePrim", prim_path="/franka", scale=(1.5, 1.5, 1.5))
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(Gf.Transform(omni.usd.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka"))).GetScale()),
            (1.5, 1.5, 1.5),
        )

    async def test_destroy_command(self) -> None:
        """Test destroy command."""
        articulation_usd = self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        omni.kit.commands.execute("IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka")
        [await omni.kit.app.get_app().next_update_async() for _ in range(10)]
        self.assertIsNotNone(self._stage.GetPrimAtPath("/franka"))
        omni.kit.commands.execute("IsaacSimDestroyPrim", prim_path="/franka")
        [await omni.kit.app.get_app().next_update_async() for _ in range(10)]
        self.assertFalse(self._stage.GetPrimAtPath("/franka"))
