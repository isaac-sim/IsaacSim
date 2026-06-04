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

"""UI tests for the URDF importer extension."""

import asyncio
import gc
import os
import shutil
import tempfile

import carb
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.ui_test as ui_test
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.asset.importer.urdf.ui.impl import extension as urdf_ui_extension
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.test.utils import MenuUITestCase, usd_utils
from pxr import Sdf


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestImporterUI(MenuUITestCase):
    """Test URDF importer UI interactions.

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
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf.ui")
        urdf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._urdf_extension_path = ext_manager.get_extension_path(urdf_ext_id)
        self._urdf_path = os.path.normpath(
            os.path.join(self._urdf_extension_path, "data", "urdf", "robots", "ur10", "urdf", "ur10.urdf")
        )
        self._tests_dir = os.path.normpath(os.path.join(self._urdf_extension_path, "data", "urdf", "tests"))
        self._tmpdir = tempfile.mkdtemp(prefix="urdf_ui_test_")
        # Redirect tempfile's default directory to self._tmpdir so any
        # mkdtemp() calls made during the test (e.g. the URDF importer's
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

    async def test_import_ur10_from_ui(self, delete_output_on_success: bool = True) -> None:
        """Import the UR10 asset via the UI and validate output.

        Args:
            delete_output_on_success: When True, remove generated output after a passing run.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporterConfig
            >>> URDFImporterConfig()
            <...>
        """
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )

        await dir_widget.input(self._urdf_path)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        file = await self.find_widget_with_retry("**/Label[*].text=='ur10.urdf'", parent=grid_view)
        await file.click()
        await ui_test.human_delay()

        output_path_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='urdf_output_path'"
        )
        output_path_field.model.set_value(self._tmpdir)

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        self._stage = stage_utils.get_current_stage()

        # check if object is there
        prim = self._stage.GetPrimAtPath("/ur10")
        prim.GetVariantSet("Physics").SetVariantSelection("physx")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints and links exist
        shoulder_pan_joint = self._stage.GetPrimAtPath("/ur10/Physics/shoulder_pan_joint")
        self.assertNotEqual(shoulder_pan_joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(shoulder_pan_joint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertAlmostEqual(shoulder_pan_joint.GetAttribute("physics:upperLimit").Get(), 360.0, delta=1e-2)
        self.assertAlmostEqual(shoulder_pan_joint.GetAttribute("physics:lowerLimit").Get(), -360.0, delta=1e-2)

        shoulder_link = self._stage.GetPrimAtPath("/ur10/Geometry/base_link/shoulder_link")
        self.assertAlmostEqual(
            shoulder_link.GetAttribute("physics:diagonalInertia").Get()[0], 0.03147431257693659, delta=1e-2
        )
        self.assertAlmostEqual(shoulder_link.GetAttribute("physics:mass").Get(), 7.778, delta=1e-2)

        await omni.kit.app.get_app().next_update_async()

    async def test_urdf_ui_selections(self) -> None:
        """Update UI settings and validate importer config values.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporterConfig
            >>> URDFImporterConfig()
            <...>
        """
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )

        await dir_widget.input(self._urdf_path)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        file = await self.find_widget_with_retry("**/Label[*].text=='ur10.urdf'", parent=grid_view)
        await file.click()
        await ui_test.human_delay()

        output_path_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='urdf_output_path'"
        )
        output_path_field.model.set_value(self._tmpdir)

        ros_package_table_name_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='ros_package_table_name_field_0'"
        )
        ros_package_table_name_field.model.set_value("test_package")
        await ui_test.human_delay()

        ros_package_table_path_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='ros_package_table_path_field_0'"
        )
        ros_package_table_path_field.model.set_value("test_path")
        await ui_test.human_delay()

        # As ROS package rows and option widgets get added, lower controls
        # are pushed below the file picker's scrolling viewport. Always
        # scroll the target into view before clicking so the click
        # coordinate lands on the intended widget instead of bleeding into
        # whatever sits at the bottom of the panel (e.g. the Import button).
        add_ros_package_row_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/Button[*].identifier=='urdf_add_ros_package_row'"
        )
        await self.scroll_to_widget(add_ros_package_row_btn)
        await add_ros_package_row_btn.click()
        await ui_test.human_delay()

        ros_package_table_name_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='ros_package_table_name_field_1'"
        )
        ros_package_table_name_field.model.set_value("test_package_row_1")
        await ui_test.human_delay()

        ros_package_table_path_field = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='ros_package_table_path_field_1'"
        )
        ros_package_table_path_field.model.set_value("test_path_row_1")
        await ui_test.human_delay()

        collision_from_visuals_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='urdf_collision_from_visuals'"
        )
        await self.scroll_to_widget(collision_from_visuals_btn)
        await collision_from_visuals_btn.click()
        await ui_test.human_delay()

        collision_type_dropdown = await self.find_widget_with_retry(
            "Select File//Frame/**/ComboBox[*].identifier=='urdf_collision_type'"
        )
        collision_type_dropdown.model.get_item_value_model(None, 0).set_value(2)
        await ui_test.human_delay()

        allow_self_collision_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='urdf_allow_self_collision'"
        )
        await self.scroll_to_widget(allow_self_collision_btn)
        await allow_self_collision_btn.click()
        await ui_test.human_delay()

        base_type_dropdown = await self.find_widget_with_retry(
            "Select File//Frame/**/ComboBox[*].identifier=='urdf_base_type'"
        )
        base_type_dropdown.model.get_item_value_model(None, 0).set_value(1)
        await ui_test.human_delay()

        merge_mesh_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='urdf_merge_mesh'"
        )
        await self.scroll_to_widget(merge_mesh_btn)
        await merge_mesh_btn.click()
        await ui_test.human_delay()

        debug_mode_btn = await self.find_widget_with_retry(
            "Select File//Frame/**/CheckBox[*].identifier=='urdf_debug_mode'"
        )
        await self.scroll_to_widget(debug_mode_btn)
        await debug_mode_btn.click()
        await ui_test.human_delay()

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        extension = urdf_ui_extension.get_instance()
        config = extension._get_config()
        self.assertEqual(os.path.normpath(config.urdf_path.lower()), os.path.normpath(self._urdf_path.lower()))
        self.assertEqual(
            os.path.normpath(config.usd_path.lower()),
            os.path.normpath(self._tmpdir).lower(),
        )
        self.assertEqual(config.merge_mesh, True)
        self.assertEqual(config.debug_mode, True)
        self.assertEqual(config.collision_from_visuals, True)
        self.assertEqual(config.collision_type, "Bounding Sphere")
        self.assertEqual(config.allow_self_collision, True)
        self.assertEqual(config.fix_base, True)
        self.assertEqual(
            config.ros_package_paths,
            [{"name": "test_package", "path": "test_path"}, {"name": "test_package_row_1", "path": "test_path_row_1"}],
        )

        await omni.kit.app.get_app().next_update_async()

    async def test_urdf_ui_match_urdf_importer(self) -> None:
        """Test urdf ui match urdf importer."""

        # UI workflow (output goes to self._tmpdir via test_import_ur10_from_ui)
        await self.test_import_ur10_from_ui()

        ui_ur10_path = os.path.normpath(os.path.join(self._tmpdir, "ur10", "ur10.usda"))

        # URDF importer workflow into a separate subdirectory
        importer_output_dir = os.path.join(self._tmpdir, "importer_output")
        os.makedirs(importer_output_dir, exist_ok=True)
        config = URDFImporterConfig()
        config.urdf_path = self._urdf_path
        config.usd_path = importer_output_dir
        urdf_importer = URDFImporter(config)
        urdf_importer_output_path = os.path.normpath(urdf_importer.import_urdf())
        print(f"urdf_importer_output_path: {urdf_importer_output_path}")
        print(f"ui_ur10_path: {ui_ur10_path}")
        result = await usd_utils.compare_usd_files([ui_ur10_path, urdf_importer_output_path])
        self.assertTrue(result, "USD comparison failed")

        await omni.kit.app.get_app().next_update_async()

    async def test_multiselect_per_file_state_isolation(self) -> None:
        """Verify per-file settings and ROS package tables don't cross-contaminate.

        Opens File > Import, navigates to the data/urdf/tests folder,
        then programmatically multi-selects test_basic.urdf and
        test_advanced.urdf via the file-browser TreeView ``selection``
        property.  The framework creates two independent option panels
        (one per file).  Each file is given different configs and ROS
        package entries via its per-file state, then the Import button
        is clicked.  The test asserts each file's ``_get_config`` matches
        only its own settings — especially the ROS package table.
        """
        extension = urdf_ui_extension.get_instance()
        self.assertIsNotNone(extension)

        path_basic = os.path.join(self._tests_dir, "test_basic.urdf")
        path_advanced = os.path.join(self._tests_dir, "test_advanced.urdf")

        # ----------------------------------------------------------------
        # Open File > Import and navigate to the tests folder
        # ----------------------------------------------------------------
        await self.menu_click_with_retry("File/Import")

        dir_widget = await self.find_widget_with_retry(
            "Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'"
        )
        await dir_widget.input(path_basic)
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        grid_view = await self.find_widget_with_retry(
            "Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'"
        )
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        # ----------------------------------------------------------------
        # Multi-select both files by setting the TreeView selection
        # property directly.  The file-browser table view (backed by a
        # ui.TreeView) shares the same listview model as the grid view.
        # Setting its ``selection`` triggers selection_changed_fn which
        # propagates through the asset-importer callback chain.
        # ----------------------------------------------------------------
        item_basic = None
        item_advanced = None
        target_tv = None

        for tv_wrapper in ui_test.find_all("Select File//Frame/**/TreeView[*]"):
            tv = tv_wrapper.widget
            if tv.model is None:
                continue
            items = tv.model.get_item_children(None)
            for item in items:
                if item.name == "test_basic.urdf":
                    item_basic = item
                elif item.name == "test_advanced.urdf":
                    item_advanced = item
            if item_basic and item_advanced:
                target_tv = tv
                break

        self.assertIsNotNone(target_tv, "Could not find TreeView with target file items")
        self.assertIsNotNone(item_basic, "test_basic.urdf not found in file model")
        self.assertIsNotNone(item_advanced, "test_advanced.urdf not found in file model")

        target_tv.selection = [item_basic, item_advanced]
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        # ----------------------------------------------------------------
        # Both files are now selected.  The asset importer creates one
        # option panel per file, so every widget identifier appears
        # twice.  Use find_all and pick by index to target the correct
        # file's panel (0 = first file, 1 = second file).
        # ----------------------------------------------------------------
        async def find_nth(query: str, index: int = 0, max_frames: int = 100) -> object:
            for _ in range(max_frames):
                widgets = ui_test.find_all(query)
                if len(widgets) > index:
                    return widgets[index]
                await omni.kit.app.get_app().next_update_async()
            raise TimeoutError(
                f"Expected at least {index + 1} widget(s) for '{query}' after {max_frames} frames, found {len(widgets)}"
            )

        # Use DIFFERENT values per panel so any state bleed is caught below.

        # -- Panel 0: test_basic.urdf --
        (await find_nth("Select File//Frame/**/StringField[*].identifier=='urdf_output_path'", 0)).model.set_value(
            self._tmpdir
        )
        (
            await find_nth("Select File//Frame/**/StringField[*].identifier=='ros_package_table_name_field_0'", 0)
        ).model.set_value("basic_package")
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/StringField[*].identifier=='ros_package_table_path_field_0'", 0)
        ).model.set_value("basic_path")
        await ui_test.human_delay()
        # collision_from_visuals stays False on panel 0
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_allow_self_collision'", 0)
        ).model.set_value(False)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_merge_mesh'", 0)).model.set_value(False)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_debug_mode'", 0)).model.set_value(False)
        await ui_test.human_delay()

        # -- Panel 1: test_advanced.urdf --
        (await find_nth("Select File//Frame/**/StringField[*].identifier=='urdf_output_path'", 1)).model.set_value(
            self._tmpdir
        )
        (
            await find_nth("Select File//Frame/**/StringField[*].identifier=='ros_package_table_name_field_0'", 1)
        ).model.set_value("advanced_package")
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/StringField[*].identifier=='ros_package_table_path_field_0'", 1)
        ).model.set_value("advanced_path")
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_collision_from_visuals'", 1)
        ).model.set_value(True)
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/ComboBox[*].identifier=='urdf_collision_type'", 1)
        ).model.get_item_value_model(None, 0).set_value(2)
        await ui_test.human_delay()
        (
            await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_allow_self_collision'", 1)
        ).model.set_value(True)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_merge_mesh'", 1)).model.set_value(True)
        await ui_test.human_delay()
        (await find_nth("Select File//Frame/**/CheckBox[*].identifier=='urdf_debug_mode'", 1)).model.set_value(True)
        await ui_test.human_delay()

        # Snapshot state before Import (``_start_import`` pops it).
        basic_key = os.path.normcase(os.path.normpath(path_basic))
        advanced_key = os.path.normcase(os.path.normpath(path_advanced))
        basic_state = extension._per_file_state.get(basic_key)
        advanced_state = extension._per_file_state.get(advanced_key)
        self.assertIsNotNone(basic_state, "test_basic.urdf has no per-file state")
        self.assertIsNotNone(advanced_state, "test_advanced.urdf has no per-file state")
        self.assertIsNot(basic_state, advanced_state, "Per-file states must be distinct instances")
        self.assertIsNot(basic_state.config, advanced_state.config, "Per-file configs must be distinct instances")
        self.assertIsNot(basic_state.models, advanced_state.models, "Per-file models dicts must be distinct instances")
        self.assertIsNot(
            basic_state.option_builder,
            advanced_state.option_builder,
            "Per-file OptionWidgets must be distinct instances",
        )

        # Panel 0 values — must not leak from panel 1.
        self.assertEqual(basic_state.config.merge_mesh, False)
        self.assertEqual(basic_state.config.debug_mode, False)
        self.assertEqual(basic_state.config.collision_from_visuals, False)
        self.assertEqual(basic_state.config.allow_self_collision, False)
        self.assertEqual(
            basic_state.option_builder.get_ros_package_map(),
            [{"name": "basic_package", "path": "basic_path"}],
        )

        # Panel 1 values — must not be overwritten by panel 0.
        self.assertEqual(advanced_state.config.merge_mesh, True)
        self.assertEqual(advanced_state.config.debug_mode, True)
        self.assertEqual(advanced_state.config.collision_from_visuals, True)
        self.assertEqual(advanced_state.config.collision_type, "Bounding Sphere")
        self.assertEqual(advanced_state.config.allow_self_collision, True)
        self.assertEqual(
            advanced_state.option_builder.get_ros_package_map(),
            [{"name": "advanced_package", "path": "advanced_path"}],
        )

        import_button = await self.find_widget_with_retry("Select File//Frame/VStack[0]/HStack[2]/Button[0]")
        await import_button.click()
        await ui_test.human_delay()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # ``_last_config`` is the last imported file; import order is unspecified.
        config = extension._get_config()
        self.assertIsNotNone(config)
        basic_signature = (False, False, False, False)
        advanced_signature = (True, True, True, True)
        actual_signature = (
            config.merge_mesh,
            config.debug_mode,
            config.collision_from_visuals,
            config.allow_self_collision,
        )
        self.assertIn(
            actual_signature,
            (basic_signature, advanced_signature),
            "Last imported config must match one panel's settings exactly",
        )

        # Per-file states should be consumed (popped) after import
        self.assertNotIn(basic_key, extension._per_file_state)
        self.assertNotIn(advanced_key, extension._per_file_state)

        await omni.kit.app.get_app().next_update_async()
