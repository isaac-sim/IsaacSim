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

"""Tests for the deprecated-OmniGraph-node migration on stage open."""

import os
import tempfile

import omni.kit.app
import omni.kit.test
import omni.usd
from pxr import Sdf, Usd


class TestDeprecationManagerOmniGraphMigration(omni.kit.test.AsyncTestCase):
    """Validate deprecated OmniGraph node:type migration on stage open."""

    async def setUp(self) -> None:
        self._tmp_dir = tempfile.TemporaryDirectory()

    async def tearDown(self) -> None:
        self._tmp_dir.cleanup()

    async def test_overlapping_deprecated_entry_does_not_abort_migration(self) -> None:
        """A node matching two config entries must not block other nodes from migrating.

        `omni.isaac.wheeled_robots.AckermannController` has no automatic migration (its I/O differs from
        `isaacsim.robot.wheeled_robots.AckermannController`) but its config entry overlaps the broader
        `omni.isaac.wheeled_robots` entry.
        """
        stage_path = os.path.join(self._tmp_dir.name, "deprecated_nodes.usda")
        stage = Usd.Stage.CreateNew(stage_path)
        stage.DefinePrim("/World", "Xform")
        stage.DefinePrim("/World/ActionGraph", "Xform")

        ackermann = stage.DefinePrim("/World/ActionGraph/a_ackermann", "Scope")
        ackermann.CreateAttribute("node:type", Sdf.ValueTypeNames.Token, custom=True).Set(
            "omni.isaac.wheeled_robots.AckermannController"
        )
        # Traversal order matters: this prim must be visited after `a_ackermann` to prove the sweep continues.
        later = stage.DefinePrim("/World/ActionGraph/z_later", "Scope")
        later.CreateAttribute("node:type", Sdf.ValueTypeNames.Token, custom=True).Set(
            "omni.isaac.core_nodes.IsaacReadSimulationTime"
        )
        stage.GetRootLayer().Save()

        result, error = await omni.usd.get_context().open_stage_async(stage_path)
        self.assertTrue(result, error)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        opened_stage = omni.usd.get_context().get_stage()
        self.assertEqual(
            opened_stage.GetPrimAtPath("/World/ActionGraph/a_ackermann").GetAttribute("node:type").Get(),
            "omni.isaac.wheeled_robots.AckermannController",
        )
        self.assertEqual(
            opened_stage.GetPrimAtPath("/World/ActionGraph/z_later").GetAttribute("node:type").Get(),
            "isaacsim.core.nodes.IsaacReadSimulationTime",
        )
