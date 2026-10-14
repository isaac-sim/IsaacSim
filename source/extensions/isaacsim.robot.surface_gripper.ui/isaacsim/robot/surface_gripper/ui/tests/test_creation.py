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

"""Exercise the registered Surface Gripper creation action and undo stack."""

import omni.kit.actions.core
import omni.kit.test
import omni.kit.undo
import omni.usd
from pxr import UsdGeom


class TestSurfaceGripperCreation(omni.kit.test.AsyncTestCase):
    """Menu creation supports pseudo-root, default prim, selection and undo."""

    async def tearDown(self) -> None:
        """Close the stage after each test, including failed assertions."""
        await omni.usd.get_context().close_stage_async()

    async def test_creation_parents_and_undo(self) -> None:
        """Invoke the same registered action used by the Create menu."""
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        stage.ClearDefaultPrim()
        selection = omni.usd.get_context().get_selection()
        selection.set_selected_prim_paths([], False)
        action = omni.kit.actions.core.get_action_registry().get_action(
            "isaacsim.robot.surface_gripper.ui", "isaac_create_surface_gripper"
        )
        self.assertIsNotNone(action)
        action.execute()
        self.assertTrue(stage.GetPrimAtPath("/SurfaceGripper"))
        action.execute()
        children = list(stage.GetPseudoRoot().GetChildren())
        grippers = [prim for prim in children if prim.GetName().startswith("SurfaceGripper")]
        self.assertEqual(len(grippers), 2)
        parent = UsdGeom.Xform.Define(stage, "/Default").GetPrim()
        stage.SetDefaultPrim(parent)
        action.execute()
        self.assertTrue(stage.GetPrimAtPath("/Default/SurfaceGripper"))
        UsdGeom.Xform.Define(stage, "/Selected")
        selection.set_selected_prim_paths(["/Selected"], False)
        action.execute()
        self.assertTrue(stage.GetPrimAtPath("/Selected/SurfaceGripper"))
        omni.kit.undo.undo()
        self.assertFalse(stage.GetPrimAtPath("/Selected/SurfaceGripper"))
        omni.kit.undo.redo()
        self.assertTrue(stage.GetPrimAtPath("/Selected/SurfaceGripper"))
        self.assertFalse(stage.GetPrimAtPath("/Selected/SurfaceGripper/SurfaceGripper"))
