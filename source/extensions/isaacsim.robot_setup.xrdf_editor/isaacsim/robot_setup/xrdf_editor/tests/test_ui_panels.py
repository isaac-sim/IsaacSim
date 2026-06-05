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

"""End-to-end UI integration tests for the XRDF editor.

Each test:

1. Loads the UR10e USD on a fresh stage,
2. Plays the timeline long enough for physics to initialise the articulation,
3. Instantiates the real :class:`Extension`, opens its window (which builds the
   panels and subscribes to stage events),
4. Drives the panels through the same callbacks the real UI uses, and
5. Asserts on the resulting domain state, USD stage, or exported file.

The tests exercise the full wiring between :class:`EditorState`, the four
interactive panels, and the orchestrator. They are deliberately written against
the panel callbacks (``_on_*``) rather than synthetic mouse events because that
gives us the cleanest assertion seam while still covering the real model/widget
plumbing.
"""

from __future__ import annotations

import asyncio
import os
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
import yaml
from isaacsim.robot_setup.xrdf_editor import articulation_discovery
from isaacsim.robot_setup.xrdf_editor.constants import (
    COLLISION_KEY_V1,
    COLLISION_KEY_V2,
    DEFAULT_ACCELERATION_LIMIT,
    DEFAULT_JERK_LIMIT,
    XRDF_FORMAT,
    XRDF_VERSION_1,
    XRDF_VERSION_2,
)
from isaacsim.robot_setup.xrdf_editor.extension import Extension
from isaacsim.robot_setup.xrdf_editor.yaml_utils import safe_load_yaml
from isaacsim.storage.native import get_assets_root_path
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

_ROBOT_USD = "Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"


def _articulation_base_path(stage: Usd.Stage) -> str | None:
    """Return the first articulation base path on ``stage`` (or ``None``).

    Uses :func:`articulation_discovery.find_all_articulation_base_paths` so the
    returned path matches what ``SelectionPanel`` / ``EditorState`` see (the
    maximal subtree containing the robot, e.g. ``/ur10e``), rather than the
    path of the prim carrying the ``UsdPhysics.ArticulationRootAPI`` (which on
    UR10e is ``/ur10e/root_joint``).
    """
    paths = articulation_discovery.find_all_articulation_base_paths(stage)
    return paths[0] if paths else None


def _disable_instanceable(stage: Usd.Stage) -> None:
    """Recursively turn off ``instanceable=True`` on every prim of ``stage``.

    UR10e's USD references sub-assets with ``instanceable=True``, but
    :func:`sphere_generation.find_link_meshes` treats instanceable meshes as
    opaque (it can't read their points) and records empty mesh lists for the
    enclosing link. The tests need real per-link mesh inventories to exercise
    the sphere editor and exporter, so we disable instancing first.

    A single pass over the stage isn't enough because hiding the children of
    an instanceable prim only becomes visible after the parent is flipped, so
    we loop until no more instanceable prims remain.
    """
    while True:
        changed = False
        for prim in stage.Traverse():
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                changed = True
        if not changed:
            return


class TestXrdfEditorUIPanels(omni.kit.test.AsyncTestCase):
    """Tier-A UI integration tests covering the cross-panel control flow."""

    # ------------------------------------------------------------------
    # Shared fixture
    # ------------------------------------------------------------------
    async def setUp(self) -> None:
        self._timeline = omni.timeline.get_timeline_interface()
        self._ext: Extension | None = None
        self._articulation_path: str | None = None

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        assets_root = get_assets_root_path()
        if assets_root is None:
            self.skipTest("Could not access asset server to open UR10e")
            return
        full_path = os.path.join(assets_root + "/", _ROBOT_USD)
        await stage_utils.open_stage_async(full_path)

        stage = stage_utils.get_current_stage()
        _disable_instanceable(stage)
        await omni.kit.app.get_app().next_update_async()
        self._articulation_path = _articulation_base_path(stage)
        self.assertIsNotNone(self._articulation_path, "UR10e USD should contain an articulation root")

        # Play the timeline so physics can initialise the articulation view.
        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # Instantiate the real extension and open its window — this builds every
        # panel and subscribes to stage events.
        self._ext = Extension()
        self._ext.on_startup("isaacsim.robot_setup.xrdf_editor")
        self._ext._window.visible = True
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        if self._ext is not None:
            try:
                self._ext._window.visible = False
                self._ext.on_shutdown()
            except Exception:
                # Cleanup is best-effort: don't mask the underlying test failure.
                pass
            self._ext = None

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)

        if self._timeline and self._timeline.is_playing():
            self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Helpers that drive the panels
    # ------------------------------------------------------------------
    async def _select_ur10e(self) -> None:
        """Trigger 'Select Articulation' selection of the UR10e robot."""
        sel = self._ext.ui_builder._selection_panel
        sel.refresh_articulations()
        await omni.kit.app.get_app().next_update_async()
        self.assertIn(
            self._articulation_path,
            sel.articulation_list,
            "UR10e articulation path should appear in the Selection Panel dropdown",
        )
        idx = sel.articulation_list.index(self._articulation_path)
        sel._articulation_model.get_item_value_model().set_value(idx)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    async def _select_link(self, link_subpath: str) -> None:
        """Trigger 'Select Link' selection in the SelectionPanel by subpath."""
        sel = self._ext.ui_builder._selection_panel
        keys = list(self._ext.ui_builder.state.link_to_meshes.keys())
        self.assertIn(link_subpath, keys, f"Link {link_subpath} not in link_to_meshes")
        idx = keys.index(link_subpath)
        sel._link_model.get_item_value_model().set_value(idx)
        await omni.kit.app.get_app().next_update_async()

    def _first_link_with_meshes(self) -> str:
        """Return the first link subpath that has at least one mesh."""
        for link, meshes in self._ext.ui_builder.state.link_to_meshes.items():
            if meshes:
                return link
        self.fail("UR10e should have at least one link with a non-instanced mesh")
        return ""  # unreachable; satisfies static analysers

    # ------------------------------------------------------------------
    # Test 1 — Articulation selection cascade
    # ------------------------------------------------------------------
    async def test_articulation_selection_populates_state_and_panels(self) -> None:
        """Selecting UR10e from the dropdown should populate state and rebuild panels."""
        await self._select_ur10e()
        state = self._ext.ui_builder.state

        self.assertIsNotNone(state.articulation, "EditorState.articulation must be set")
        self.assertEqual(state.articulation_base_path, self._articulation_path)
        self.assertGreater(state.num_dof, 0, "UR10e should expose at least one DOF")
        self.assertEqual(len(state.dof_names), state.num_dof)
        self.assertGreater(len(state.link_to_meshes), 0, "UR10e should expose at least one link with meshes")

        # JointPropertiesPanel was rebuilt with one frame per DOF.
        joint_panel = self._ext.ui_builder._joint_properties_panel
        self.assertEqual(len(joint_panel._joint_frames), state.num_dof)

        # SphereEditor + Tools panels are enabled and visible.
        self.assertTrue(self._ext.ui_builder._sphere_editor_panel._frame.enabled)
        self.assertTrue(self._ext.ui_builder._editor_tools_panel._tools_frame.enabled)

    # ------------------------------------------------------------------
    # Test 2 — Link selection cascade
    # ------------------------------------------------------------------
    async def test_link_selection_propagates_to_sphere_and_tools_panels(self) -> None:
        """Selecting a link in the SelectionPanel updates the sphere editor and tools panel."""
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        first_link = self._first_link_with_meshes()
        await self._select_link(first_link)

        sphere_panel = self._ext.ui_builder._sphere_editor_panel
        tools_panel = self._ext.ui_builder._editor_tools_panel

        self.assertEqual(sphere_panel.get_selected_link_name(), first_link)
        self.assertEqual(sphere_panel.get_selected_link_path(), state.articulation_base_path + first_link)

        # Mesh dropdown was repopulated for this link.
        self.assertIsNotNone(sphere_panel._mesh_model)
        mesh_count = len(state.link_to_meshes[first_link])
        self.assertEqual(
            len(list(sphere_panel._mesh_model.get_item_children(None))),
            mesh_count,
            "Mesh dropdown should contain one entry per mesh under the link",
        )

        # EditorToolsPanel tracks the previously-selected link for visibility toggling.
        self.assertEqual(tools_panel._prev_link, first_link)

    # ------------------------------------------------------------------
    # Test 3 — Sphere editor add → connect → scale → clear
    # ------------------------------------------------------------------
    async def test_sphere_editor_add_connect_scale_clear_round_trip(self) -> None:
        """Drive the full Add / Connect / Scale / Clear sequence and assert on stage state."""
        await self._select_ur10e()
        first_link = self._first_link_with_meshes()
        await self._select_link(first_link)

        state = self._ext.ui_builder.state
        sphere_panel = self._ext.ui_builder._sphere_editor_panel
        link_path = state.articulation_base_path + first_link

        # --- Add two spheres at distinct positions ---
        sphere_panel._add_sphere_radius.set_value(0.05)
        sphere_panel._add_sphere_translation_x.set_value(0.0)
        sphere_panel._add_sphere_translation_y.set_value(0.0)
        sphere_panel._add_sphere_translation_z.set_value(0.0)
        sphere_panel._on_add_sphere()

        sphere_panel._add_sphere_translation_z.set_value(0.1)
        sphere_panel._on_add_sphere()

        names = state.collision_sphere_editor.get_sphere_names_by_link(link_path)
        self.assertEqual(len(names), 2, f"Expected 2 spheres after two Add ops, got {len(names)}: {names}")

        # --- Connect: add 2 interpolated spheres between the originals ---
        # Ensure both connect-sphere comboboxes have their selection wired up.
        sphere_panel._connect_sphere_0_model.get_item_value_model().set_value(0)
        sphere_panel._on_collision_sphere_select_0()
        sphere_panel._connect_sphere_num.set_value(2)
        sphere_panel._on_connect_spheres()

        names = state.collision_sphere_editor.get_sphere_names_by_link(link_path)
        self.assertEqual(
            len(names),
            4,
            f"Expected 2 originals + 2 interpolated after Connect, got {len(names)}: {names}",
        )

        # --- Scale: factor 2 should leave the sphere count unchanged but double radii ---
        def _radius(sphere_path: str) -> float:
            return float(state.collision_sphere_editor.path_2_spheres[sphere_path].get_radii().numpy()[0])

        radii_before = sorted(_radius(link_path + name) for name in names)
        sphere_panel._scale_factor_field.set_value(2.0)
        sphere_panel._on_scale_spheres()
        names_after_scale = state.collision_sphere_editor.get_sphere_names_by_link(link_path)
        self.assertEqual(len(names_after_scale), 4, "Scale must not add or remove spheres")
        radii_after = sorted(_radius(link_path + name) for name in names_after_scale)
        np.testing.assert_allclose(radii_after, np.array(radii_before) * 2.0, atol=1e-4)

        # --- Clear the link's spheres ---
        sphere_panel._on_clear_link_spheres()
        names_after_clear = state.collision_sphere_editor.get_sphere_names_by_link(link_path)
        self.assertEqual(
            len(names_after_clear),
            0,
            f"Expected no spheres after Clear, got {names_after_clear}",
        )

    # ------------------------------------------------------------------
    # Test 4 — Joint properties → physics step → articulation positions
    # ------------------------------------------------------------------
    async def test_joint_position_change_applied_on_physics_step(self) -> None:
        """Mark a joint active, set a position, and verify a physics tick moves the robot.

        The test setUp plays the timeline *before* opening the window, which is
        the exact ordering that previously left ``Extension._physics_subscription``
        unset (the ``SIMULATION_START_PLAY`` event had already fired). Relying
        on the real subscription here doubles as a regression test for that
        bootstrap bug: if ``_on_window`` ever stops bootstrapping the
        subscription, the articulation will not move and this test fails.
        """
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        joint_panel = self._ext.ui_builder._joint_properties_panel

        target_idx = 0
        target_pos = 0.25  # within the wide joint limits of UR10e's shoulder_pan_joint
        joint_panel._on_update_active_joints(target_idx, "Active Joint")
        self.assertTrue(state.active_joints[target_idx])

        joint_panel._on_set_joint_position(target_idx, target_pos)
        self.assertAlmostEqual(state.joint_positions[target_idx], target_pos, places=4)
        self.assertTrue(
            joint_panel._set_joint_positions_on_step,
            "Position change should queue an articulation update on the next physics tick",
        )

        # Let the real physics-step subscription fire. The orchestrator's
        # ``on_physics_step`` consumes the queued flag and writes to the
        # articulation; no manual poke into the callback.
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        self.assertFalse(
            joint_panel._set_joint_positions_on_step,
            "Real physics-step subscription must consume the pending-position flag",
        )

        positions = state.articulation.get_dof_positions().numpy()[0]
        self.assertAlmostEqual(
            float(positions[target_idx]),
            target_pos,
            places=2,
            msg="Articulation DOF should have been moved to the queued position",
        )
        velocities = state.articulation.get_dof_velocities().numpy()[0]
        self.assertAlmostEqual(float(velocities[target_idx]), 0.0, places=2)

    # ------------------------------------------------------------------
    # Test 5 — Hide-link / hide-robot visibility toggles
    # ------------------------------------------------------------------
    async def test_visibility_toggles_modify_mesh_visibility(self) -> None:
        """Hide / show buttons should round-trip the USD visibility attribute on the affected meshes."""
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        first_link = self._first_link_with_meshes()
        await self._select_link(first_link)

        tools_panel = self._ext.ui_builder._editor_tools_panel
        link_path = state.articulation_base_path + first_link
        mesh_paths = [link_path + m for m in state.link_to_meshes[first_link]]
        self.assertGreater(len(mesh_paths), 0)

        stage = stage_utils.get_current_stage()
        sample_prim = stage.GetPrimAtPath(mesh_paths[0])
        imageable = UsdGeom.Imageable(sample_prim)
        original_vis = imageable.GetVisibilityAttr().Get()

        # --- Hide the selected link ---
        tools_panel._on_toggle_link_visible()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(tools_panel._hiding_link)
        self.assertNotEqual(
            imageable.GetVisibilityAttr().Get(),
            original_vis,
            "Toggling link visibility should change the USD visibility attribute on its meshes",
        )

        # --- Show the link again — visibility round-trips ---
        tools_panel._on_toggle_link_visible()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(tools_panel._hiding_link)
        self.assertEqual(
            imageable.GetVisibilityAttr().Get(),
            original_vis,
            "Toggling link visibility a second time should restore the original visibility",
        )

        # --- Hide the rest of the robot (everything except the selected link) ---
        other_link = None
        for link in state.link_to_meshes:
            if link != first_link and state.link_to_meshes[link]:
                other_link = link
                break
        if other_link is None:
            self.skipTest("UR10e should have a second link with meshes to exercise Hide Robot")
            return
        other_mesh_path = state.articulation_base_path + other_link + state.link_to_meshes[other_link][0]
        other_prim = stage.GetPrimAtPath(other_mesh_path)
        other_imageable = UsdGeom.Imageable(other_prim)
        other_vis_before = other_imageable.GetVisibilityAttr().Get()

        tools_panel._on_toggle_robot_visible()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(tools_panel._hiding_robot)
        self.assertNotEqual(
            other_imageable.GetVisibilityAttr().Get(),
            other_vis_before,
            "Hide Robot should toggle visibility on links other than the selected one",
        )

        # Toggle back to restore.
        tools_panel._on_toggle_robot_visible()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(tools_panel._hiding_robot)
        self.assertEqual(other_imageable.GetVisibilityAttr().Get(), other_vis_before)

    # ------------------------------------------------------------------
    # Test 6 — Export XRDF reflects current UI state
    # ------------------------------------------------------------------
    async def test_export_xrdf_captures_current_ui_state(self) -> None:
        """Add spheres via the editor, mark joints active, then export and re-parse the file."""
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        first_link = self._first_link_with_meshes()
        await self._select_link(first_link)

        sphere_panel = self._ext.ui_builder._sphere_editor_panel
        joint_panel = self._ext.ui_builder._joint_properties_panel
        tools_panel = self._ext.ui_builder._editor_tools_panel

        # Add two spheres via the panel
        sphere_panel._add_sphere_radius.set_value(0.05)
        sphere_panel._add_sphere_translation_z.set_value(0.0)
        sphere_panel._on_add_sphere()
        sphere_panel._add_sphere_translation_z.set_value(0.1)
        sphere_panel._on_add_sphere()

        # Mark up to 4 joints active via the joint panel
        active_count = min(4, state.num_dof)
        for i in range(active_count):
            joint_panel._on_update_active_joints(i, "Active Joint")
        # Default limits (do not write zero or the cspace block looks degenerate)
        for i in range(active_count):
            state.acceleration_limits[i] = DEFAULT_ACCELERATION_LIMIT
            state.jerk_limits[i] = DEFAULT_JERK_LIMIT

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmpdir:
            path = os.path.join(tmpdir, "robot.xrdf")

            # Set output path; this should enable the Export button and hide the merge checkbox
            # (since the file doesn't exist yet).
            tools_panel._xrdf_output_file.set_value(path)
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(
                tools_panel._xrdf_export_btn.enabled,
                "Export button should enable once a valid XRDF path is set",
            )

            tools_panel._xrdf_version_dropdown.set_selection_by_index(1)  # 2.0
            tools_panel._on_export_xrdf()
            self.assertTrue(os.path.exists(path), "Export should produce the XRDF file at the chosen path")

            parsed = safe_load_yaml(path)
            self.assertEqual(parsed["format"], XRDF_FORMAT)
            self.assertEqual(parsed["format_version"], float(XRDF_VERSION_2))
            self.assertEqual(
                len(parsed["cspace"]["joint_names"]),
                active_count,
                "cspace should contain exactly the joints we marked active",
            )
            self.assertIn("world_collision", parsed, "Version 2.0 should use the 'world_collision' key")
            self.assertNotIn("collision", parsed, "Version 2.0 should not retain the v1 'collision' key")

            geometry_group = parsed["world_collision"]["geometry"]
            spheres_by_link = parsed["geometry"][geometry_group].get("spheres", {}) or {}
            total_spheres = sum(len(v) for v in spheres_by_link.values())
            self.assertEqual(
                total_spheres,
                2,
                f"Exported file should contain the two spheres we added; got {total_spheres}: {spheres_by_link}",
            )

    # ------------------------------------------------------------------
    # Shared helpers for end-to-end import tests
    # ------------------------------------------------------------------
    @staticmethod
    def _link_subpath_to_key(link_subpath: str) -> str:
        """Convert ``/base_link`` style subpaths into XRDF/Lula sphere-dict keys."""
        return link_subpath.lstrip("/")

    def _spheres_under(self, link_full_path: str) -> list[tuple[np.ndarray, float]]:
        """Return ``[(center, radius), ...]`` for every sphere under ``link_full_path``."""
        editor = self._ext.ui_builder.state.collision_sphere_editor
        prefix = link_full_path + "/"
        results: list[tuple[np.ndarray, float]] = []
        for sphere_path, sphere in editor.path_2_spheres.items():
            if not sphere_path.startswith(prefix):
                continue
            center = sphere.get_local_poses()[0].numpy()[0]
            radius = float(sphere.get_radii().numpy()[0])
            results.append((np.asarray(center, dtype=float), radius))
        results.sort(key=lambda item: float(item[1]))
        return results

    async def _import_xrdf_with_spheres_and_assert(self, format_version: float, collision_key: str) -> None:
        """Drive the XRDF Import button for a real on-disk file and verify state + spheres."""
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        tools_panel = self._ext.ui_builder._editor_tools_panel
        joint_panel = self._ext.ui_builder._joint_properties_panel

        first_link = self._first_link_with_meshes()
        link_key = self._link_subpath_to_key(first_link)
        link_full_path = state.articulation_base_path + first_link

        active_count = min(3, state.num_dof)
        active_names = list(state.dof_names[:active_count])
        positions = {name: 0.1 * (i + 1) for i, name in enumerate(state.dof_names)}
        expected_spheres = [
            {"center": [0.0, 0.0, 0.05], "radius": 0.05},
            {"center": [0.0, 0.0, 0.10], "radius": 0.04},
            {"center": [0.0, 0.0, 0.15], "radius": 0.03},
        ]

        content = {
            "format": XRDF_FORMAT,
            "format_version": float(format_version),
            "default_joint_positions": positions,
            "cspace": {
                "joint_names": active_names,
                "acceleration_limits": [5.0] * active_count,
                "jerk_limits": [500.0] * active_count,
            },
            collision_key: {"geometry": "test_group"},
            "geometry": {
                "test_group": {
                    "spheres": {
                        link_key: expected_spheres,
                    }
                }
            },
            "self_collision": {
                "geometry": "test_group",
                "ignore": {link_key: []},
            },
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.xrdf")
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            tools_panel._xrdf_input_file.set_value(path)
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(
                tools_panel._xrdf_import_btn.enabled,
                "Import button should enable for a valid XRDF path",
            )

            tools_panel._on_import_xrdf()
            await omni.kit.app.get_app().next_update_async()

        for i, name in enumerate(state.dof_names):
            expected_active = name in active_names
            self.assertEqual(
                bool(state.active_joints[i]),
                expected_active,
                f"active_joints[{i}] (={name}) should be {expected_active}",
            )
        for name, expected_pos in positions.items():
            idx = state.dof_names.index(name)
            self.assertAlmostEqual(
                state.joint_positions[idx],
                expected_pos,
                places=3,
                msg=f"default_joint_positions[{name}] did not round-trip",
            )
        for i in range(active_count):
            idx = state.dof_names.index(active_names[i])
            self.assertAlmostEqual(state.acceleration_limits[idx], 5.0, places=3)
            self.assertAlmostEqual(state.jerk_limits[idx], 500.0, places=3)

        loaded = self._spheres_under(link_full_path)
        self.assertEqual(
            len(loaded),
            len(expected_spheres),
            f"Expected {len(expected_spheres)} spheres under {link_full_path}, got {len(loaded)}",
        )
        for (center, radius), expected in zip(
            loaded,
            sorted(expected_spheres, key=lambda s: s["radius"]),
        ):
            np.testing.assert_allclose(center, expected["center"], atol=1e-4)
            self.assertAlmostEqual(radius, expected["radius"], places=4)

        # The joint panel must be rebuilt with one frame per DOF after an import.
        self.assertEqual(
            len(joint_panel._joint_frames),
            state.num_dof,
            "Joint panel should still have one frame per DOF after rebuild",
        )

    # ------------------------------------------------------------------
    # Test 7 — Invalid XRDF output path disables the XRDF export button
    # ------------------------------------------------------------------
    async def test_invalid_xrdf_output_path_disables_only_xrdf_export(self) -> None:
        """Regression: typing an invalid XRDF path must disable the XRDF Export button.

        Previously this handler disabled the Lula ``Save`` button by mistake,
        leaving the XRDF Export button enabled even though the path was not a
        ``.xrdf``/``.yaml`` file, and disabling the unrelated Lula button.
        """
        await self._select_ur10e()
        tools_panel = self._ext.ui_builder._editor_tools_panel

        # Put the Lula export button into a known-enabled state so we can detect
        # accidental writes to it from the XRDF handler.
        tools_panel._robot_description_export_btn.enabled = True

        with tempfile.TemporaryDirectory() as tmpdir:
            valid_path = os.path.join(tmpdir, "robot.xrdf")
            invalid_path = os.path.join(tmpdir, "robot.urdf")

            tools_panel._xrdf_output_file.set_value(valid_path)
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(
                tools_panel._xrdf_export_btn.enabled,
                "Valid XRDF path should enable the XRDF Export button",
            )

            tools_panel._xrdf_output_file.set_value(invalid_path)
            await omni.kit.app.get_app().next_update_async()

            self.assertFalse(
                tools_panel._xrdf_export_btn.enabled,
                "Invalid XRDF path should disable the XRDF Export button",
            )
            self.assertFalse(
                tools_panel._xrdf_merge_cb.visible,
                "Invalid XRDF path should hide the merge checkbox",
            )
            self.assertFalse(
                tools_panel._xrdf_merge_cb.get_value(),
                "Invalid XRDF path should clear the merge checkbox value",
            )
            self.assertTrue(
                tools_panel._robot_description_export_btn.enabled,
                "Invalid XRDF path must not affect the Lula Save button",
            )

    # ------------------------------------------------------------------
    # Test 8 — XRDF v1 import populates state and spheres
    # ------------------------------------------------------------------
    async def test_import_xrdf_v1_populates_state_and_spheres(self) -> None:
        """Importing a v1 XRDF file with spheres must restore state arrays and stage spheres."""
        await self._import_xrdf_with_spheres_and_assert(XRDF_VERSION_1, COLLISION_KEY_V1)

    # ------------------------------------------------------------------
    # Test 9 — XRDF v2 import populates state and spheres
    # ------------------------------------------------------------------
    async def test_import_xrdf_v2_populates_state_and_spheres(self) -> None:
        """Importing a v2 XRDF file with spheres must restore state arrays and stage spheres."""
        await self._import_xrdf_with_spheres_and_assert(XRDF_VERSION_2, COLLISION_KEY_V2)

    # ------------------------------------------------------------------
    # Test 10 — Lula YAML import populates state and spheres
    # ------------------------------------------------------------------
    async def test_import_lula_yaml_populates_state_and_spheres(self) -> None:
        """Importing a Lula robot-description YAML restores state arrays and stage spheres.

        Covers the Lula counterpart of the XRDF import tests: the file uses
        ``cspace`` (list), ``default_q``, ``acceleration_limits``,
        ``jerk_limits``, ``cspace_to_urdf_rules`` (for fixed joints), and
        ``collision_spheres``. The import goes through the same callback the
        real UI invokes.
        """
        await self._select_ur10e()
        state = self._ext.ui_builder.state
        tools_panel = self._ext.ui_builder._editor_tools_panel

        first_link = self._first_link_with_meshes()
        link_key = self._link_subpath_to_key(first_link)
        link_full_path = state.articulation_base_path + first_link

        active_count = min(3, state.num_dof)
        active_names = list(state.dof_names[:active_count])
        fixed_names = list(state.dof_names[active_count:])
        active_positions = [0.1 * (i + 1) for i in range(active_count)]
        fixed_positions = {name: 0.02 * (i + 1) for i, name in enumerate(fixed_names)}
        expected_spheres = [
            {"center": [0.0, 0.0, 0.05], "radius": 0.05},
            {"center": [0.0, 0.0, 0.10], "radius": 0.04},
        ]

        content = {
            "api_version": 1.0,
            "cspace": active_names,
            "default_q": active_positions,
            "acceleration_limits": [5.0] * active_count,
            "jerk_limits": [500.0] * active_count,
            "cspace_to_urdf_rules": [
                {"name": name, "rule": "fixed", "value": fixed_positions[name]} for name in fixed_names
            ],
            "collision_spheres": [{link_key: expected_spheres}],
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "robot.yaml")
            with open(path, "w") as f:
                yaml.safe_dump(content, f)

            tools_panel._lula_input_file.set_value(path)
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(
                tools_panel._robot_description_import_btn.enabled,
                "Import button should enable for a valid Lula YAML path",
            )

            tools_panel._on_import_lula()
            await omni.kit.app.get_app().next_update_async()

        for i, name in enumerate(state.dof_names):
            expected_active = name in active_names
            self.assertEqual(
                bool(state.active_joints[i]),
                expected_active,
                f"active_joints[{i}] (={name}) should be {expected_active}",
            )

        for i, name in enumerate(active_names):
            idx = state.dof_names.index(name)
            self.assertAlmostEqual(state.joint_positions[idx], active_positions[i], places=3)
            self.assertAlmostEqual(state.acceleration_limits[idx], 5.0, places=3)
            self.assertAlmostEqual(state.jerk_limits[idx], 500.0, places=3)

        for name, expected_pos in fixed_positions.items():
            idx = state.dof_names.index(name)
            self.assertAlmostEqual(
                state.joint_positions[idx],
                expected_pos,
                places=3,
                msg=f"Fixed joint {name} should land at its cspace_to_urdf_rules value",
            )

        loaded = self._spheres_under(link_full_path)
        self.assertEqual(
            len(loaded),
            len(expected_spheres),
            f"Expected {len(expected_spheres)} spheres under {link_full_path}, got {len(loaded)}",
        )
        for (center, radius), expected in zip(
            loaded,
            sorted(expected_spheres, key=lambda s: s["radius"]),
        ):
            np.testing.assert_allclose(center, expected["center"], atol=1e-4)
            self.assertAlmostEqual(radius, expected["radius"], places=4)


class TestMimicJointBehavior(omni.kit.test.AsyncTestCase):
    """End-to-end tests for robots that include a mimic-follower joint.

    Differs from :class:`TestXrdfEditorUIPanels` in that the setUp **mutates
    the UR10e stage before play** to apply ``PhysxSchema.PhysxMimicJointAPI``
    to one of its joints. This produces a real physics-simulated articulation
    with an actual mimic constraint, so the test class can verify two things
    at once:

    * The editor correctly identifies the mimic follower via
      :func:`articulation_discovery.find_mimic_joint_names` and renders its
      panel frame as a read-only label (no ``FloatField``).
    * Updating an *unrelated* (non-mimic) joint's position through the panel
      callback still propagates through the physics-step subscriber to move
      the real articulation — the user's stated concern that "if I have a
      robot with a mimic joint, then updating the joint value in the panel
      makes the robot move".
    """

    # `wrist_3_joint` follows `wrist_2_joint` with a 1:1 gear ratio. Both are
    # revolute joints on UR10e and live inside the same articulation subtree,
    # so the mimic constraint is well-formed.
    _MIMIC_FOLLOWER_NAME = "wrist_3_joint"
    _MIMIC_DRIVER_NAME = "wrist_2_joint"

    async def setUp(self) -> None:
        self._timeline = omni.timeline.get_timeline_interface()
        self._ext: Extension | None = None
        self._articulation_path: str | None = None

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        assets_root = get_assets_root_path()
        if assets_root is None:
            self.skipTest("Could not access asset server to open UR10e")
            return
        full_path = os.path.join(assets_root + "/", _ROBOT_USD)
        await stage_utils.open_stage_async(full_path)

        stage = stage_utils.get_current_stage()
        _disable_instanceable(stage)
        await omni.kit.app.get_app().next_update_async()
        self._articulation_path = _articulation_base_path(stage)
        self.assertIsNotNone(self._articulation_path, "UR10e USD should contain an articulation root")

        # Apply the mimic relationship BEFORE play so the PhysX articulation
        # cooks with the constraint in place. Doing it post-play would not
        # rewire the underlying articulation view.
        follower_prim, driver_prim = self._find_joints_by_name(stage, self._articulation_path)
        self.assertIsNotNone(follower_prim, f"UR10e should expose a joint named {self._MIMIC_FOLLOWER_NAME!r}")
        self.assertIsNotNone(driver_prim, f"UR10e should expose a joint named {self._MIMIC_DRIVER_NAME!r}")

        mimic_api = PhysxSchema.PhysxMimicJointAPI.Apply(follower_prim, "rotZ")
        mimic_api.GetReferenceJointRel().AddTarget(driver_prim.GetPath())
        mimic_api.GetGearingAttr().Set(1.0)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        self._ext = Extension()
        self._ext.on_startup("isaacsim.robot_setup.xrdf_editor")
        self._ext._window.visible = True
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        if self._ext is not None:
            try:
                self._ext._window.visible = False
                self._ext.on_shutdown()
            except Exception:
                pass
            self._ext = None

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)

        if self._timeline and self._timeline.is_playing():
            self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Stage helpers
    # ------------------------------------------------------------------
    def _find_joints_by_name(
        self, stage: Usd.Stage, articulation_base_path: str
    ) -> tuple[Usd.Prim | None, Usd.Prim | None]:
        """Find ``(follower, driver)`` joint prims inside the articulation by name."""
        follower: Usd.Prim | None = None
        driver: Usd.Prim | None = None
        base_prim = stage.GetPrimAtPath(articulation_base_path)
        for prim in Usd.PrimRange(base_prim):
            if not UsdPhysics.Joint(prim):
                continue
            name = prim.GetName()
            if name == self._MIMIC_FOLLOWER_NAME:
                follower = prim
            elif name == self._MIMIC_DRIVER_NAME:
                driver = prim
        return follower, driver

    async def _select_articulation(self) -> None:
        """Pick the (now-mimicked) UR10e articulation in the SelectionPanel."""
        sel = self._ext.ui_builder._selection_panel
        sel.refresh_articulations()
        await omni.kit.app.get_app().next_update_async()
        idx = sel.articulation_list.index(self._articulation_path)
        sel._articulation_model.get_item_value_model().set_value(idx)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------
    async def test_mimic_joint_detected_and_marked_inactive(self) -> None:
        """Selecting the mimicked UR10e must populate ``state.mimic_joint_names``.

        The mimic follower must also be forced into the *inactive* bucket so
        it is never written into the XRDF ``cspace`` block (regardless of any
        prior active toggle the user may have made).
        """
        await self._select_articulation()
        state = self._ext.ui_builder.state

        self.assertIn(
            self._MIMIC_FOLLOWER_NAME,
            state.mimic_joint_names,
            f"{self._MIMIC_FOLLOWER_NAME!r} should be detected as a mimic follower",
        )
        self.assertNotIn(
            self._MIMIC_DRIVER_NAME,
            state.mimic_joint_names,
            f"{self._MIMIC_DRIVER_NAME!r} is the driver, not a follower",
        )

        mimic_idx = state.dof_names.index(self._MIMIC_FOLLOWER_NAME)
        self.assertFalse(
            bool(state.active_joints[mimic_idx]),
            "Mimic followers must never be marked active (they cannot be added to cspace)",
        )

    async def test_mimic_joint_panel_frame_is_read_only(self) -> None:
        """The joint-properties panel must render mimic followers as a read-only label.

        We verify this indirectly by inspecting the frame's child widget tree:
        a normal joint frame contains at least one ``FloatField`` (Joint
        Position); a mimic frame should contain ``Label`` widgets only.
        """
        await self._select_articulation()
        state = self._ext.ui_builder.state
        joint_panel = self._ext.ui_builder._joint_properties_panel

        mimic_idx = state.dof_names.index(self._MIMIC_FOLLOWER_NAME)
        self.assertLess(
            mimic_idx,
            len(joint_panel._joint_frames),
            "Joint frame list should cover every DOF in the articulation",
        )

        mimic_frame = joint_panel._joint_frames[mimic_idx]

        # Walk the frame's widget tree and detect any FloatField-style input
        # widget. A read-only mimic frame contains only labels.
        def _has_float_field(widget) -> bool:
            cls_name = type(widget).__name__
            if cls_name in {"FloatField", "FloatDrag", "FloatSlider"}:
                return True
            for child in getattr(widget, "_children", []) or []:
                if _has_float_field(child):
                    return True
            return False

        self.assertFalse(
            _has_float_field(mimic_frame),
            "Mimic-joint frame must not expose any FloatField widget",
        )

    async def test_non_mimic_joint_change_moves_robot_through_physics_step(self) -> None:
        """Updating a non-mimic joint position via the panel must move the robot.

        Regression for the user-reported concern: "if I have a robot with a
        mimic joint, then updating the joint value in the panel makes the
        robot move (I think this functionality might be broken right now)".

        Exercises the full pipeline through the **real** physics-step
        subscription (no synthetic ``_on_physics_step`` call), which doubles
        as a regression check for the window-opened-after-play subscription
        bootstrap fix in :py:meth:`Extension._on_window`:

        1. :meth:`JointPropertiesPanel._on_set_joint_position` updates state
           and arms the per-tick flag.
        2. The orchestrator's physics-step subscription fires
           :meth:`UIBuilder.on_physics_step`, which calls
           ``Articulation.set_dof_positions(state.joint_positions)`` with the
           full DOF array — including the mimic follower's position.
        3. The articulation's actual DOF positions reflect the change at the
           target index after a few sim frames.
        """
        await self._select_articulation()
        state = self._ext.ui_builder.state
        joint_panel = self._ext.ui_builder._joint_properties_panel

        # Pick a joint that is neither the mimic follower nor its driver to
        # isolate the change from any mimic-constraint snap-back.
        target_name = "shoulder_pan_joint"
        self.assertIn(target_name, state.dof_names)
        target_idx = state.dof_names.index(target_name)
        target_pos = 0.25  # well within UR10e shoulder_pan_joint's limits

        joint_panel._on_update_active_joints(target_idx, "Active Joint")
        self.assertTrue(state.active_joints[target_idx])

        joint_panel._on_set_joint_position(target_idx, target_pos)
        self.assertAlmostEqual(state.joint_positions[target_idx], target_pos, places=4)
        self.assertTrue(
            joint_panel._set_joint_positions_on_step,
            "Position change must queue an articulation update on the next physics tick",
        )

        # Let the real physics-step subscription fire and write to the
        # articulation; if the subscription was never bootstrapped (the bug
        # this test guards against) the flag stays armed and the position
        # never propagates to the articulation.
        for _ in range(8):
            await omni.kit.app.get_app().next_update_async()

        self.assertFalse(
            joint_panel._set_joint_positions_on_step,
            "Real physics-step subscription must consume the pending-position flag",
        )

        positions = state.articulation.get_dof_positions().numpy()[0]
        self.assertAlmostEqual(
            float(positions[target_idx]),
            target_pos,
            places=2,
            msg=(
                f"Articulation DOF {target_name!r} should have been moved to {target_pos}; "
                f"the panel-to-physics flow appears broken for robots with mimic joints."
            ),
        )
