# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for VisualCuesManager drop-line lifecycle."""

from __future__ import annotations

from unittest.mock import patch

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.experimental.objects import Cube
from isaacsim.replicator.teleop import TeleopManager, VisualCuesManager
from pxr import UsdGeom, UsdShade


class TestVisualCuesManager(omni.kit.test.AsyncTestCase):
    """Verify visual cue manager pose resolution and USD lifecycle."""

    async def setUp(self) -> None:
        """Create a fresh stage and visual-cue managers."""
        await app_utils.update_app_async()
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()
        self._manager = VisualCuesManager()
        self._teleop_manager = TeleopManager()

    async def tearDown(self) -> None:
        """Remove cue state and close the test stage."""
        self._manager.hide_all()
        self._teleop_manager.destroy()
        stage_utils.close_stage()
        await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

    async def test_auto_link_pose_provider(self) -> None:
        """Pose provider supplies auto-linked controller input positions."""
        self._manager.set_pose_provider(self._teleop_manager.get_input_world_position)
        self._teleop_manager._left_input_world_pos = (1.0, 2.0, 3.0)  # noqa: SLF001
        pos, status = self._manager._resolve_world_position("left")  # noqa: SLF001
        self.assertEqual(pos, (1.0, 2.0, 3.0))
        self.assertIn("Auto", status)

    async def test_override_prim_path(self) -> None:
        """Manual prim override replaces controller input for cue placement."""
        Cube("/World/visual_cue_test_cube", sizes=0.1, positions=[0.5, -0.25, 1.2])
        await app_utils.update_app_async()

        self._manager.set_override_prim_path("left", "/World/visual_cue_test_cube")
        pos, status = self._manager._resolve_world_position("left")  # noqa: SLF001
        self.assertIsNotNone(pos)
        assert pos is not None
        self.assertAlmostEqual(pos[0], 0.5, places=3)
        self.assertAlmostEqual(pos[2], 1.2, places=3)
        self.assertIn("Override", status)

    async def test_show_and_hide_side(self) -> None:
        """Show creates session-layer cue prims; hide removes them."""
        Cube("/World/visual_cue_anchor", sizes=0.1, positions=[0.0, 0.0, 1.0])
        await app_utils.update_app_async()

        self._manager.set_reference_z(0.0)
        self._manager.set_override_prim_path("left", "/World/visual_cue_anchor")
        ok, _message = self._manager.show_side("left")
        self.assertTrue(ok)
        await app_utils.update_app_async()

        stage = stage_utils.get_current_stage()
        left_side = VisualCuesManager.SIDE_PATHS["left"]
        cylinder_path = f"{left_side}/{VisualCuesManager.CYLINDER_CHILD}"
        material_path = f"{left_side}/{VisualCuesManager.MATERIAL_CHILD}"
        cylinder = stage.GetPrimAtPath(cylinder_path)
        self.assertTrue(cylinder.IsValid())
        self.assertTrue(cylinder.IsA(UsdGeom.Cylinder))
        self.assertEqual(
            UsdGeom.PrimvarsAPI(cylinder).GetPrimvar("doNotCastShadows").Get(),
            True,
        )
        self.assertNotEqual(
            UsdGeom.Imageable(cylinder).GetPurposeAttr().Get(),
            UsdGeom.Tokens.guide,
        )
        material = stage.GetPrimAtPath(material_path)
        self.assertTrue(material.IsValid())
        shader = UsdShade.Shader(stage.GetPrimAtPath(f"{material_path}/PreviewSurface"))
        self.assertEqual(shader.GetInput("opacity").Get(), 0.5)
        self.assertIsNotNone(shader.GetInput("emissiveColor").Get())

        self._manager.hide_side("left")
        self.assertFalse(self._manager.is_side_active("left"))

    async def test_visibility_never_authors_to_root_layer(self) -> None:
        """Missing and below-floor inputs must remain isolated to the anonymous layer."""
        stage = stage_utils.get_current_stage()
        root = stage.GetRootLayer()

        ok, _message = self._manager.show_side("left")
        self.assertTrue(ok)
        self.assertIsNone(root.GetPrimAtPath(VisualCuesManager.CUES_SCOPE))

        Cube("/World/below_visual_cue_floor", sizes=0.1, positions=[0.0, 0.0, -1.0])
        await app_utils.update_app_async()
        self._manager.set_override_prim_path("left", "/World/below_visual_cue_floor")
        self._manager._update_side_geometry("left")  # noqa: SLF001
        self.assertIsNone(root.GetPrimAtPath(VisualCuesManager.CUES_SCOPE))

        self._manager.hide_all()
        self.assertIsNone(root.GetPrimAtPath(VisualCuesManager.CUES_SCOPE))

    async def test_stage_replacement_invalidates_stale_layer(self) -> None:
        """An update after stage replacement must not target the previous session layer."""
        Cube("/World/visual_cue_anchor", sizes=0.1, positions=[0.0, 0.0, 1.0])
        await app_utils.update_app_async()
        self._manager.set_override_prim_path("right", "/World/visual_cue_anchor")
        ok, _message = self._manager.show_side("right")
        self.assertTrue(ok)
        stale_layer = self._manager._layer  # noqa: SLF001
        self.assertIsNotNone(stale_layer)

        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()
        self._manager._on_update(None)  # noqa: SLF001

        self.assertIsNone(self._manager._layer)  # noqa: SLF001
        self.assertFalse(self._manager.is_side_active("right"))
        assert stale_layer is not None
        current_stage = stage_utils.get_current_stage()
        self.assertNotIn(
            stale_layer.identifier,
            [layer.identifier for layer in current_stage.GetLayerStack(includeSessionLayers=True)],
        )

    async def test_update_invalidates_state_when_stage_becomes_unavailable(self) -> None:
        """An update during stage replacement must tolerate a temporarily unavailable stage."""
        Cube("/World/visual_cue_anchor", sizes=0.1, positions=[0.0, 0.0, 1.0])
        await app_utils.update_app_async()
        self._manager.set_override_prim_path("right", "/World/visual_cue_anchor")
        ok, _message = self._manager.show_side("right")
        self.assertTrue(ok)
        self.assertIsNotNone(self._manager._layer)  # noqa: SLF001
        self.assertIsNotNone(self._manager._update_subscription)  # noqa: SLF001
        self.assertTrue(self._manager.is_side_active("right"))

        with patch.object(stage_utils, "get_current_stage", side_effect=ValueError("No stage found")):
            self._manager._on_update(None)  # noqa: SLF001

        self.assertIsNone(self._manager._layer)  # noqa: SLF001
        self.assertFalse(self._manager.is_side_active("right"))
        self.assertIsNone(self._manager._update_subscription)  # noqa: SLF001
