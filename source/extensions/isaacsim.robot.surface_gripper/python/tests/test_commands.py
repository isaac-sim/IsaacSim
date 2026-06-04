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

"""Tests for the deprecated CreateSurfaceGripper command."""

import omni.kit.commands
import omni.kit.test


class TestCreateSurfaceGripperCommand(omni.kit.test.AsyncTestCase):
    """Test the deprecated CreateSurfaceGripper omni.kit.commands command."""

    async def test_create_surface_gripper_command(self) -> None:
        """Test creating a surface gripper via the deprecated command."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World"
        success, gripper_prim = omni.kit.commands.execute(
            "CreateSurfaceGripper",
            prim_path=prim_path,
        )
        self.assertTrue(success)
        self.assertIsNotNone(gripper_prim)
        self.assertTrue(gripper_prim.IsValid())
        self.assertEqual(str(gripper_prim.GetPath()), prim_path + "/SurfaceGripper")

    async def test_create_surface_gripper_command_undo(self) -> None:
        """Test that the command's undo removes the created prim."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        prim_path = "/World"
        omni.kit.commands.execute(
            "CreateSurfaceGripper",
            prim_path=prim_path,
        )
        gripper_path = prim_path + "/SurfaceGripper"
        self.assertTrue(stage.GetPrimAtPath(gripper_path).IsValid())
        omni.kit.undo.undo()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(stage.GetPrimAtPath(gripper_path).IsValid())
