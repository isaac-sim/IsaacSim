# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the Teleop UI IK panel covering PINK QP solver dropdown availability filtering.

The IK panel must only expose PINK QP solver backends whose Python dependency
imports successfully on the current install. Listing an unavailable backend would
let the user select a solver that fails at runtime with a confusing import error.
These tests open the live Teleop window and assert that:

* every solver shown in the dropdown reports ``available == True``,
* every solver hidden from the dropdown reports ``available == False`` and is
  surfaced in the combo tooltip, and
* the panel handles the all-unavailable edge case without raising.
"""

from __future__ import annotations

import os
import tempfile
from unittest.mock import patch

import omni.ui as ui
from isaacsim.replicator.teleop.ui.teleop_ui_extension import TeleopUIExtension
from isaacsim.test.utils import MenuUITestCase

WINDOW_TITLE = TeleopUIExtension.WINDOW_NAME
MENU_PATH = f"{TeleopUIExtension.MENU_GROUP}/{TeleopUIExtension.WINDOW_NAME}"


class TestTeleopUIIKPinkQPFilter(MenuUITestCase):
    """Validate that the IK panel hides PINK QP solver backends that cannot be imported."""

    async def _open_window(self, tmp_dir: str):
        await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
        window = ui.Workspace.get_window(WINDOW_TITLE)
        self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
        window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
        window.visible = True
        window.focus()
        await self.wait_n_frames(5)
        return window

    async def test_dropdown_lists_only_available_pink_qp_solvers(self):
        """The exposed list must match each solver's reported availability."""
        window = None
        with tempfile.TemporaryDirectory(prefix="teleop_ui_ik_") as tmp_dir:
            try:
                window = await self._open_window(tmp_dir)
                ik_panel = window._ik_panel
                ik_controller = window._ik_controller

                shown = list(ik_panel._pink_qp_solvers)
                hidden = dict(ik_panel._pink_qp_unavailable)

                all_solvers = set(ik_controller.get_pink_qp_solver_names())
                self.assertEqual(
                    set(shown) | set(hidden),
                    all_solvers,
                    "Every supported solver must appear in either the shown or hidden list",
                )
                self.assertTrue(
                    set(shown).isdisjoint(hidden),
                    "A solver cannot be both shown and hidden",
                )

                for solver_name in shown:
                    available, reason = ik_controller.get_pink_qp_solver_availability(solver_name)
                    self.assertTrue(
                        available,
                        f"Shown solver {solver_name!r} reports unavailable: {reason}",
                    )

                for solver_name, reason in hidden.items():
                    available, _ = ik_controller.get_pink_qp_solver_availability(solver_name)
                    self.assertFalse(
                        available,
                        f"Hidden solver {solver_name!r} reports available; should not be hidden",
                    )
                    self.assertTrue(reason, f"Hidden solver {solver_name!r} must include a reason")
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_combo_tooltip_summarises_hidden_solvers(self):
        """When solvers are filtered out, the combo tooltip must enumerate them."""
        window = None
        with tempfile.TemporaryDirectory(prefix="teleop_ui_ik_") as tmp_dir:
            try:
                window = await self._open_window(tmp_dir)
                ik_panel = window._ik_panel
                if not ik_panel._pink_qp_unavailable:
                    self.skipTest("All PINK QP solvers are importable on this install; nothing to verify")

                expected = ", ".join(sorted(ik_panel._pink_qp_unavailable))
                for side in ("left", "right"):
                    combo = ik_panel._widgets[side].get("pink_qp_solver")
                    if combo is None:
                        continue
                    tooltip = combo.tooltip or ""
                    self.assertIn(
                        "Hidden",
                        tooltip,
                        f"{side} QP combo tooltip should mention hidden solvers",
                    )
                    self.assertIn(
                        expected,
                        tooltip,
                        f"{side} QP combo tooltip should list every unavailable solver",
                    )
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_panel_survives_when_all_pink_qp_solvers_unavailable(self):
        """If no QP backend imports, the panel must not raise on init or callbacks."""
        window = None
        with tempfile.TemporaryDirectory(prefix="teleop_ui_ik_") as tmp_dir:
            try:
                from isaacsim.replicator.teleop.controllers.robot_ik import (
                    RobotIKController,
                )

                with patch.object(
                    RobotIKController,
                    "get_pink_qp_solver_availability",
                    staticmethod(lambda _: (False, "forced unavailable for test")),
                ):
                    window = await self._open_window(tmp_dir)
                    ik_panel = window._ik_panel

                    self.assertEqual(
                        ik_panel._pink_qp_solvers,
                        [],
                        "All solvers should be filtered out when forced unavailable",
                    )
                    self.assertGreater(
                        len(ik_panel._pink_qp_unavailable),
                        0,
                        "Hidden solvers must be recorded with reasons",
                    )

                    ik_panel._on_pink_qp_solver_changed("left", 0)
                    ik_panel._on_pink_qp_solver_changed("right", 0)
                    await self.wait_n_frames(1)
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()
