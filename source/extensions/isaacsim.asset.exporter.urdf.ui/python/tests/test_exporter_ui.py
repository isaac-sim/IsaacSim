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

"""Verifies the URDF exporter UI registration and dialog behavior. Covers option widget defaults, value updates, package name sanitization, and delegating export of a selected robot."""

from __future__ import annotations

import gc
import os
import tempfile

import omni.kit.app
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.asset.exporter.urdf.ui.impl import extension as urdf_ui_ext
from isaacsim.asset.exporter.urdf.ui.impl.option_widget import OptionWidget
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.storage.native import get_assets_root_path
from isaacsim.test.utils import MenuUITestCase


class TestExporterUI(MenuUITestCase):
    """Test URDF exporter UI menu integration, dialog lifecycle, and widget interactions."""

    async def setUp(self) -> None:
        """Prepare the Exporter UI test fixture."""
        await super().setUp()
        self._tmpdir = tempfile.mkdtemp(prefix="urdf_export_ui_test_")

    async def tearDown(self) -> None:
        """Clean up the Exporter UI test fixture."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = None
        gc.collect()
        await super().tearDown()

    # ------------------------------------------------------------------
    # Extension lifecycle
    # ------------------------------------------------------------------

    async def test_extension_instance(self) -> None:
        """Verify the extension singleton is available after startup."""
        instance = urdf_ui_ext.get_instance()
        self.assertIsNotNone(instance)
        self.assertIsInstance(instance, urdf_ui_ext.Extension)

    async def test_file_menu_item_registered(self) -> None:
        """Verify 'URDF Exporter' appears in the File menu."""
        menu_widget = await self.menu_click_with_retry("File/URDF Exporter", window_name="Export As ...")
        self.assertIsNotNone(menu_widget, "Export dialog did not open from File menu")

    # ------------------------------------------------------------------
    # Export dialog open / close
    # ------------------------------------------------------------------

    async def test_show_export_dialog(self) -> None:
        """Opening the dialog via the menu should create the Export As window."""
        await self.menu_click_with_retry("File/URDF Exporter", window_name="Export As ...")
        await self.wait_n_frames(5)

        export_window = ui_test.find("Export As ...")
        self.assertIsNotNone(export_window, "Export As ... window not found")

    async def test_hide_export_dialog(self) -> None:
        """Calling _hide_dialog should close the export window."""
        instance = urdf_ui_ext.get_instance()
        instance._show_dialog()
        await self.wait_n_frames(5)

        instance._hide_dialog()
        await self.wait_n_frames(5)

        export_window = ui_test.find("Export As ...")
        self.assertIsNone(export_window, "Export window should be hidden after _hide_dialog")

    # ------------------------------------------------------------------
    # OptionWidget defaults and property accessors
    # ------------------------------------------------------------------

    async def test_option_widget_defaults(self) -> None:
        """Verify OptionWidget initial property values."""
        w = OptionWidget()
        self.assertEqual(w.mesh_dir_name, "meshes")
        self.assertEqual(w.mesh_path_prefix, "./")
        self.assertIsNone(w.root_prim_path)
        self.assertFalse(w.visualize_collision_meshes)
        self.assertEqual(w.package_name, "")
        self.assertTrue(w.use_physx_inertia)

    async def test_option_widget_mesh_dir_fallback(self) -> None:
        """mesh_dir_name should fall back to 'meshes' when internal value is empty."""
        w = OptionWidget()
        w._mesh_dir = ""
        self.assertEqual(w.mesh_dir_name, "meshes")
        w._mesh_dir = None
        self.assertEqual(w.mesh_dir_name, "meshes")

    async def test_option_widget_value_change(self) -> None:
        """_on_value_changed should update the corresponding private attribute."""
        w = OptionWidget()
        w._on_value_changed("root", "/World/robot")
        self.assertEqual(w.root_prim_path, "/World/robot")

        w._on_value_changed("mesh_dir", "custom_meshes")
        self.assertEqual(w.mesh_dir_name, "custom_meshes")

        w._on_value_changed("visualize_collision_meshes", True)
        self.assertTrue(w.visualize_collision_meshes)

    async def test_option_widget_cleanup(self) -> None:
        """cleanup() should reset mutable state."""
        w = OptionWidget()
        w._on_value_changed("root", "/World/robot")
        w._on_value_changed("mesh_dir", "custom")
        w._on_value_changed("visualize_collision_meshes", True)

        w.cleanup()
        self.assertEqual(w.mesh_dir_name, "meshes")
        self.assertEqual(w.mesh_path_prefix, "")
        self.assertIsNone(w.root_prim_path)
        self.assertFalse(w.visualize_collision_meshes)

    # ------------------------------------------------------------------
    # Export options panel rendering
    # ------------------------------------------------------------------

    async def test_export_options_panel_renders(self) -> None:
        """Export options panel should contain expected widgets after dialog opens."""
        await self.menu_click_with_retry("File/URDF Exporter", window_name="Export As ...")
        await self.wait_n_frames(10)

        mesh_folder = ui_test.find("Export As ...//Frame/**/Label[*].text=='Mesh Folder Name'")
        root_prim = ui_test.find("Export As ...//Frame/**/Label[*].text=='Root Prim Path'")
        vis_collision = ui_test.find("Export As ...//Frame/**/Label[*].text=='Visualize Collisions'")

        self.assertIsNotNone(mesh_folder, "Mesh Folder Name label not found")
        self.assertIsNotNone(root_prim, "Root Prim Path label not found")
        self.assertIsNotNone(vis_collision, "Visualize Collisions label not found")

    # ------------------------------------------------------------------
    # Package name sanitization (logic in UrdfExporterDelegate._do_export)
    # ------------------------------------------------------------------

    async def test_package_name_sanitization(self) -> None:
        """Verify package:// prefix sanitization follows expected rules."""
        import re

        cases = [
            ("My-Robot", "my_robot"),
            ("  UPPER  ", "upper"),
            ("a", "a_pkg"),
            ("robot 2.0!!", "robot_2_0"),
            ("___test___", "test"),
            ("a-b--c", "a_b_c"),
        ]
        for raw, expected in cases:
            sanitized = re.sub(r"[^a-z0-9_]", "_", raw.lower())
            sanitized = re.sub(r"_+", "_", sanitized).strip("_")
            if len(sanitized) < 2:
                sanitized += "_pkg"
            self.assertEqual(
                sanitized,
                expected,
                f"Sanitization of {raw!r}: got {sanitized!r}, expected {expected!r}",
            )

    # ------------------------------------------------------------------
    # End-to-end export through the delegate (headless, no UI interaction)
    # ------------------------------------------------------------------

    async def test_delegate_export_ur10e(self) -> None:
        """UrdfExporterDelegate._do_export should produce a valid URDF for the UR10e robot."""
        assets_root = get_assets_root_path()
        if not assets_root:
            self.skipTest("Assets root not available")
            return

        robot_usd = f"{assets_root}/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        await stage_utils.open_stage_async(robot_usd)
        stage = stage_utils.get_current_stage()
        if stage is None:
            self.skipTest("Could not open UR10e asset")
            return

        await self.wait_n_frames(5)

        delegate = urdf_ui_ext.UrdfExporterDelegate()
        delegate._option_widget._use_physx_inertia = False

        result = delegate._do_export(self._tmpdir, "ur10e")

        self.assertTrue(result, "Export returned False")

        urdf_path = os.path.join(self._tmpdir, "ur10e.urdf")
        self.assertTrue(os.path.exists(urdf_path), f"URDF file not created at {urdf_path}")
        self.assertGreater(os.path.getsize(urdf_path), 0, "URDF file is empty")

        delegate.cleanup()
