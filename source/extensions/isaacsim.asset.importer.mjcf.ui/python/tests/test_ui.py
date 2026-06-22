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

"""Verifies the MJCF importer UI workflow for loading assets and applying import options. Covers UI selection state, parity with direct importer configuration, and per-file option isolation in multi-select imports."""

import asyncio
import gc
import os
import shutil
import tempfile

import carb

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.ui_test as ui_test
from isaacsim.asset.importer.mjcf import MJCFImporter
from isaacsim.asset.importer.mjcf.ui.impl import extension as mjcf_ui_extension
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.test.utils import MenuUITestCase, usd_utils
from pxr import Sdf


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestImporterUI(MenuUITestCase):
    """Test MJCF importer UI interactions.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...
    """

    # Before running each test
    async def setUp(self) -> None:
        """Prepare the UI test fixture.

        Example:

        .. code-block:: python

            >>> import omni.usd
            >>> omni.usd.get_context()  # doctest: +SKIP
        """
        await super().setUp()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf.ui")
        mjcf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._mjcf_extension_path = ext_manager.get_extension_path(mjcf_ext_id)
        self._mjcf_data_dir = os.path.normpath(os.path.join(self._mjcf_extension_path, "data", "mjcf"))
        self._mjcf_path = os.path.normpath(os.path.join(self._mjcf_data_dir, "nv_ant.xml"))
        self._tmpdir = tempfile.mkdtemp(prefix="mjcf_ui_test_")
        # Redirect tempfile's default directory to self._tmpdir so any
        # mkdtemp() calls made during the test (e.g. the MJCF importer's
        # private scratch dir in non-debug mode) are rooted inside
        # self._tmpdir and cleaned up deterministically in tearDown.
        self._prev_tempdir = tempfile.tempdir
        tempfile.tempdir = self._tmpdir

    # After running each test
    async def tearDown(self) -> None:
        """Wait for stage loading to complete after the test.

        Example:

        .. code-block:: python

            >>> import asyncio
            >>> asyncio.sleep(0)  # doctest: +SKIP
        """
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            carb.log_info("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        self._stage = None
        gc.collect()
        tempfile.tempdir = self._prev_tempdir
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def find_content_file(self, window_name: str, filename: str) -> object | None:
        """Find a file row in the picker tree view by label text.

        Args:
            window_name: UI test window name prefix for the search scope.
            filename: Label text identifying the desired file row.

        Returns:
            The matching label widget, or ``None`` if not found.
        """
        carb.log_info(f"Finding file {filename} in window {window_name}")
        for widget in ui_test.find_all(f"{window_name}//Frame/**/TreeView[*]"):
            for file_widget in widget.find_all("**/Label[*]"):
                if file_widget.widget.text == filename:
                    carb.log_info(f"Found file {filename} in window {window_name}")
                    return file_widget
        return None

    async def test_import_ant_from_ui(self, delete_output_on_success: bool = True) -> None:
        """Import the ant asset via the UI and validate output.

        Args:
            delete_output_on_success: When True, remove generated output after a passing run.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig
            >>> MJCFImporterConfig()
            <...>
        """
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )

        await dir_widget.input(self._mjcf_path)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        file = await self.find_widget_with_retry("**/Label[*].text=='nv_ant.xml'", parent=grid_view)
        await file.click()
        await ui_test.human_delay()

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        stage = stage_utils.get_current_stage()

        # check if object is there
        prim = stage.GetPrimAtPath("/ant")
        prim.GetVariantSet("Physics").SetVariantSelection("mujoco")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints and links exist
        front_left_leg_joint = stage.GetPrimAtPath("/ant/Geometry/torso/front_left_leg/hip_1")
        self.assertNotEqual(front_left_leg_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(front_left_leg_joint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertAlmostEqual(front_left_leg_joint.GetAttribute("physics:upperLimit").Get(), 40)
        self.assertAlmostEqual(front_left_leg_joint.GetAttribute("physics:lowerLimit").Get(), -40)

        # note: mass api is not automatically applied to the leg, so we expect None
        front_left_leg = stage.GetPrimAtPath("/ant/Geometry/torso/front_left_leg")
        self.assertAlmostEqual(front_left_leg.GetAttribute("physics:diagonalInertia").Get(), None)
        self.assertAlmostEqual(front_left_leg.GetAttribute("physics:mass").Get(), None)

        actuator_1 = stage.GetPrimAtPath("/ant/Physics/Actuator_1")
        self.assertNotEqual(actuator_1.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(actuator_1.GetTypeName(), "MjcActuator")
        self.assertAlmostEqual(actuator_1.GetAttribute("mjc:gear").Get(), [15, 0, 0, 0, 0, 0])

        if delete_output_on_success:
            output_path = os.path.normpath(os.path.join(os.path.dirname(self._mjcf_path), "nv_ant"))
            stage = None
            gc.collect()
            try:
                shutil.rmtree(output_path)
            except OSError as e:
                carb.log_warn(f"Warning: unable to delete {output_path} : {e.strerror}")

    async def test_mjcf_ui_selections(self) -> None:
        """Update UI settings and validate importer config values.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig
            >>> MJCFImporterConfig()
            <...>
        """
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )

        await dir_widget.input(self._mjcf_path)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        file = await self.find_widget_with_retry("**/Label[*].text=='nv_ant.xml'", parent=grid_view)
        await file.click()
        await ui_test.human_delay()

        output_path_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='mjcf_output_path'"
        )
        output_path_field.model.set_value(os.path.join(self._extension_path, "data", "mjcf", "temp/"))

        # As option widgets grow, lower controls get pushed below the file
        # picker's scrolling viewport. Always scroll the target into view
        # before clicking so the click coordinate lands on the intended
        # widget rather than the Import button at the bottom.
        collision_from_visuals_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='mjcf_collision_from_visuals'"
        )
        await self.scroll_to_widget(collision_from_visuals_btn)
        await collision_from_visuals_btn.click()
        await ui_test.human_delay()

        collision_type_dropdown = await self.find_widget_with_retry(
            "Select File//Frame/**/ComboBox[*].identifier=='mjcf_collision_type'"
        )
        collision_type_dropdown.model.get_item_value_model(None, 0).set_value(2)
        await ui_test.human_delay()

        allow_self_collision_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='mjcf_allow_self_collision'"
        )
        await self.scroll_to_widget(allow_self_collision_btn)
        await allow_self_collision_btn.click()
        await ui_test.human_delay()

        base_type_dropdown = await self.find_widget_with_retry(
            "Select File//Frame/**/ComboBox[*].identifier=='mjcf_base_type'"
        )
        base_type_dropdown.model.get_item_value_model(None, 0).set_value(1)
        await ui_test.human_delay()

        import_scene_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='mjcf_import_scene'"
        )
        await self.scroll_to_widget(import_scene_btn)
        await import_scene_btn.click()
        await ui_test.human_delay()

        merge_mesh_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='mjcf_merge_mesh'"
        )
        await self.scroll_to_widget(merge_mesh_btn)
        await merge_mesh_btn.click()
        await ui_test.human_delay()

        debug_mode_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='mjcf_debug_mode'"
        )
        await self.scroll_to_widget(debug_mode_btn)
        await debug_mode_btn.click()
        await ui_test.human_delay()

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        extension = mjcf_ui_extension.get_instance()
        config = extension._get_config()
        self.assertEqual(
            os.path.normpath(config.mjcf_path.lower()),
            os.path.normpath(self._mjcf_path.lower()),
        )
        self.assertEqual(
            os.path.normpath(config.usd_path.lower()),
            os.path.normpath(os.path.join(self._extension_path, "data", "mjcf", "temp")).lower(),
        )
        self.assertEqual(config.import_scene, False)
        self.assertEqual(config.merge_mesh, True)
        self.assertEqual(config.debug_mode, True)
        self.assertEqual(config.collision_from_visuals, True)
        self.assertEqual(config.collision_type, "Bounding Sphere")
        self.assertEqual(config.allow_self_collision, True)
        self.assertEqual(config.fix_base, True)

        await omni.kit.app.get_app().next_update_async()
        output_path = os.path.normpath(os.path.join(self._extension_path, "data", "mjcf", "temp"))
        gc.collect()
        try:
            shutil.rmtree(output_path)
        except OSError as e:
            carb.log_warn(f"Warning: unable to delete {output_path} : {e.strerror}")

    async def test_mjcf_ui_match_mjcf_importer(self) -> None:
        """Test mjcf ui match mjcf importer."""
        # UI workflow
        await self.test_import_ant_from_ui(delete_output_on_success=False)

        ui_mjcf_path = os.path.normpath(os.path.join(os.path.dirname(self._mjcf_path), "nv_ant", "nv_ant.usda"))
        # MJCF importer workflow
        config = mjcf_ui_extension.get_instance()._get_config()
        config.mjcf_path = self._mjcf_path
        config.usd_path = os.path.normpath(os.path.join(os.path.dirname(self._mjcf_path), "temp"))
        mjcf_importer = MJCFImporter(config)
        mjcf_importer_output_path = os.path.normpath(mjcf_importer.import_mjcf())

        carb.log_info(f"ui_mjcf_path: {ui_mjcf_path}")
        carb.log_info(f"mjcf_importer_output_path: {mjcf_importer_output_path}")

        result = await usd_utils.compare_usd_files([ui_mjcf_path, mjcf_importer_output_path])
        self.assertTrue(result, "USD comparison failed")

        try:
            shutil.rmtree(os.path.normpath(os.path.dirname(ui_mjcf_path)))
            shutil.rmtree(os.path.normpath(os.path.dirname(mjcf_importer_output_path)))
        except OSError as e:
            carb.log_warn(f"Warning: unable to delete {os.path.normpath(os.path.dirname(ui_mjcf_path))} : {e.strerror}")
            carb.log_warn(
                f"Warning: unable to delete {os.path.normpath(os.path.dirname(mjcf_importer_output_path))} : {e.strerror}"
            )

    async def test_multiselect_per_file_state_isolation(self) -> None:
        """Multi-select nv_ant.xml and nv_humanoid.xml and import with UI settings.

        Opens File > Import, navigates to the data/mjcf folder, then
        programmatically multi-selects both files via the file-browser
        TreeView ``selection`` property.  Each file's option panel is
        configured via UI widgets, then the Import button is clicked.
        The test verifies the config from the last imported file matches
        the UI-driven settings.
        """
        extension = mjcf_ui_extension.get_instance()
        self.assertIsNotNone(extension)

        path_ant = os.path.join(self._mjcf_data_dir, "nv_ant.xml")
        path_humanoid = os.path.join(self._mjcf_data_dir, "nv_humanoid.xml")

        # ----------------------------------------------------------------
        # Open File > Import and navigate to the mjcf data folder
        # ----------------------------------------------------------------
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )
        await dir_widget.input(path_ant)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        # ----------------------------------------------------------------
        # Multi-select both files via TreeView.selection
        # ----------------------------------------------------------------
        item_ant = None
        item_humanoid = None
        target_tv = None

        for tv_wrapper in ui_test.find_all("Select File//Frame/**/TreeView[*]"):
            tv = tv_wrapper.widget
            if tv.model is None:
                continue
            items = tv.model.get_item_children(None)
            for item in items:
                if item.name == "nv_ant.xml":
                    item_ant = item
                elif item.name == "nv_humanoid.xml":
                    item_humanoid = item
            if item_ant and item_humanoid:
                target_tv = tv
                break

        self.assertIsNotNone(target_tv, "Could not find TreeView with target file items")
        self.assertIsNotNone(item_ant, "nv_ant.xml not found in file model")
        self.assertIsNotNone(item_humanoid, "nv_humanoid.xml not found in file model")

        target_tv.selection = [item_ant, item_humanoid]
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        # Use DIFFERENT values per panel so any state bleed is caught below.
        async def find_nth(query: str, index: int = 0, max_frames: int = 100) -> object:
            for _ in range(max_frames):
                widgets = ui_test.find_all(query)
                if len(widgets) > index:
                    return widgets[index]
                await omni.kit.app.get_app().next_update_async()
            raise TimeoutError(
                f"Expected at least {index + 1} widget(s) for '{query}' after {max_frames} frames, found {len(widgets)}"
            )

        # -- Panel 0: nv_ant.xml --
        (await find_nth("Select File//Frame/**/StringField[*].identifier=='mjcf_output_path'", 0)).model.set_value(
            self._tmpdir
        )
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_allow_self_collision'", 0)
        ).model.set_value(False)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_merge_mesh'", 0)).model.set_value(False)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_debug_mode'", 0)).model.set_value(False)
        await ui_test.human_delay()

        # -- Panel 1: nv_humanoid.xml --
        (await find_nth("Select File//Frame/**/StringField[*].identifier=='mjcf_output_path'", 1)).model.set_value(
            self._tmpdir
        )
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_collision_from_visuals'", 1)
        ).model.set_value(True)
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/ComboBox[*].identifier=='mjcf_collision_type'", 1)
        ).model.get_item_value_model(None, 0).set_value(2)
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_allow_self_collision'", 1)
        ).model.set_value(True)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_import_scene'", 1)).model.set_value(False)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_merge_mesh'", 1)).model.set_value(True)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='mjcf_debug_mode'", 1)).model.set_value(True)
        await ui_test.human_delay()

        # Snapshot state before Import (``_start_import`` pops it).
        ant_key = os.path.normcase(os.path.normpath(path_ant))
        humanoid_key = os.path.normcase(os.path.normpath(path_humanoid))
        ant_state = extension._per_file_state.get(ant_key)
        humanoid_state = extension._per_file_state.get(humanoid_key)
        self.assertIsNotNone(ant_state, "nv_ant.xml has no per-file state")
        self.assertIsNotNone(humanoid_state, "nv_humanoid.xml has no per-file state")
        self.assertIsNot(ant_state, humanoid_state, "Per-file states must be distinct instances")
        self.assertIsNot(ant_state.config, humanoid_state.config, "Per-file configs must be distinct instances")
        self.assertIsNot(ant_state.models, humanoid_state.models, "Per-file models dicts must be distinct instances")
        self.assertIsNot(
            ant_state.option_builder,
            humanoid_state.option_builder,
            "Per-file OptionWidgets must be distinct instances",
        )

        # Panel 0 values — must not leak from panel 1.
        self.assertEqual(ant_state.config.merge_mesh, False)
        self.assertEqual(ant_state.config.debug_mode, False)
        self.assertEqual(ant_state.config.collision_from_visuals, False)
        self.assertEqual(ant_state.config.allow_self_collision, False)

        # Panel 1 values — must not be overwritten by panel 0.
        self.assertEqual(humanoid_state.config.import_scene, False)
        self.assertEqual(humanoid_state.config.merge_mesh, True)
        self.assertEqual(humanoid_state.config.debug_mode, True)
        self.assertEqual(humanoid_state.config.collision_from_visuals, True)
        self.assertEqual(humanoid_state.config.collision_type, "Bounding Sphere")
        self.assertEqual(humanoid_state.config.allow_self_collision, True)

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # ``_last_config`` is the last imported file; import order is unspecified.
        config = extension._get_config()
        self.assertIsNotNone(config)
        ant_signature = (False, False, False, False)
        humanoid_signature = (True, True, True, True)
        actual_signature = (
            config.merge_mesh,
            config.debug_mode,
            config.collision_from_visuals,
            config.allow_self_collision,
        )
        self.assertIn(
            actual_signature,
            (ant_signature, humanoid_signature),
            "Last imported config must match one panel's settings exactly",
        )

        # Per-file states should be consumed (popped) after import
        self.assertNotIn(ant_key, extension._per_file_state)
        self.assertNotIn(humanoid_key, extension._per_file_state)

        await omni.kit.app.get_app().next_update_async()
