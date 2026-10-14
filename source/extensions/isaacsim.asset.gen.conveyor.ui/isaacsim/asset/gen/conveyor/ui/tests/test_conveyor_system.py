"""Tests for conveyor system track lifecycle handling."""

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import tempfile
from pathlib import Path
from types import SimpleNamespace

import omni.kit.test
from isaacsim.asset.gen.conveyor.ui.impl.conveyor_builder.conveyor_system import (
    ConveyorBuilder,
)
from pxr import Usd, UsdGeom


class TestConveyorSystem(omni.kit.test.AsyncTestCase):
    """Test conveyor system track creation and removal."""

    async def setUp(self) -> None:
        """Create a minimal conveyor asset used by each test."""
        self._temp_dir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self._asset_path = str(Path(self._temp_dir.name) / "track.usda")
        asset_stage = Usd.Stage.CreateNew(self._asset_path)
        asset_root = UsdGeom.Xform.Define(asset_stage, "/Track").GetPrim()
        asset_stage.SetDefaultPrim(asset_root)
        UsdGeom.SetStageMetersPerUnit(asset_stage, 1.0)
        asset_stage.GetRootLayer().Save()
        self._track = SimpleNamespace(base_usd=self._asset_path, conveyor_nodes={})
        self._selector = SimpleNamespace(asset_path=f"{self._temp_dir.name}/")

    async def tearDown(self) -> None:
        """Remove the temporary conveyor asset."""
        self._temp_dir.cleanup()

    async def test_add_track_with_default_parent(self) -> None:
        """Add a track using the documented default parent."""
        stage = Usd.Stage.CreateInMemory()
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(world)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        path = ConveyorBuilder(stage, self._selector).add_track(self._track)

        self.assertEqual(path, "/World/ConveyorTrack")
        self.assertTrue(stage.GetPrimAtPath(path).IsValid())

    async def test_add_track_chains_at_root_without_default_prim(self) -> None:
        """Chain root-level tracks on a stage without a default prim."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        builder = ConveyorBuilder(stage, self._selector)

        first_path = builder.add_track(self._track, parent="/")
        second_path = builder.add_track(self._track, parent=first_path)

        self.assertNotIn("//", second_path)
        self.assertTrue(stage.GetPrimAtPath(second_path).IsValid())
        self.assertEqual(stage.GetPrimAtPath(second_path).GetParent(), stage.GetPseudoRoot())

    async def test_remove_track_allows_unregistered_stage_track(self) -> None:
        """Allow a caller to delete a conveyor loaded outside the current builder session."""
        stage = Usd.Stage.CreateInMemory()
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(world)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        path = ConveyorBuilder(stage, self._selector).add_track(self._track)

        reopened_builder = ConveyorBuilder(stage, self._selector)
        self.assertEqual(reopened_builder.remove_track(path), "")
        self.assertTrue(stage.RemovePrim(path))
        self.assertFalse(stage.GetPrimAtPath(path).IsValid())
