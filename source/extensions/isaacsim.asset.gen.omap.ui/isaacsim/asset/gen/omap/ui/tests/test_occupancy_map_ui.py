"""Tests for the occupancy map generator UI extension."""

# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import os
import tempfile

import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.asset.gen.omap.ui.extension import ROS_FREE_THRESHOLD, ROS_OCCUPIED_THRESHOLD, OccupancyMapWindow


class _StringModel:
    """Minimal stand-in for a ui.StringField model.

    Args:
        value: Initial string value.
    """

    def __init__(self, value: str = "") -> None:
        self.as_string = value

    def set_value(self, value: str) -> None:
        """Update the stored string value.

        Args:
            value: New value assigned to ``as_string``.
        """
        self.as_string = value


class _ItemValueModel:
    """Minimal stand-in for a ComboBox item value model.

    Args:
        index: Integer index exposed as ``as_int``.
    """

    def __init__(self, index: int) -> None:
        self.as_int = index


class _DropdownModel:
    """Minimal stand-in for a dropdown (ComboBox) model.

    Args:
        index: Initial selected index.
    """

    def __init__(self, index: int = 0) -> None:
        self._index = index

    def get_item_value_model(self) -> _ItemValueModel:
        """Return a value model for the current selection index.

        Returns:
            An ``_ItemValueModel`` wrapping ``_index``.
        """
        return _ItemValueModel(self._index)


class TestOccupancyMapUI(omni.kit.test.AsyncTestCase):
    """Test suite for the Occupancy Map UI extension.

    Tests the UI functionality including menu loading and basic interaction with
    the occupancy map generation interface.
    """

    async def setUp(self) -> None:
        """Sets up the test environment.

        Preloads materials and waits for UI to stabilize before running tests.
        """
        await ui_test.human_delay()

    async def tearDown(self) -> None:
        """Cleans up after each test."""

    async def test_loading(self) -> None:
        """Tests that the Occupancy Map UI can be loaded from the menu.

        Creates a new stage and navigates through the Tools > Robotics > Occupancy Map
        menu to verify the extension loads correctly.
        """
        await omni.usd.get_context().new_stage_async()
        menu_widget = ui_test.get_menubar()
        await menu_widget.find_menu("Tools").click()
        await menu_widget.find_menu("Robotics").click()
        await menu_widget.find_menu("Occupancy Map").click()

    async def test_save_yaml_no_data(self) -> None:
        """Tests that save_yaml handles the case where no YAML has been generated yet.

        Verifies that calling save_yaml before any map generation completes
        gracefully without writing any file.
        """
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                window.save_yaml("test_map", tmpdir)
                self.assertFalse(os.path.exists(os.path.join(tmpdir, "test_map.yaml")))
        finally:
            window.destroy()

    async def test_save_yaml_writes_file(self) -> None:
        """Tests that save_yaml writes the YAML content to the specified file.

        Verifies that the file is created at the correct path and contains
        the expected YAML text.
        """
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            expected_yaml = "image: test_map.png\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196"
            window._ros_yaml_text = expected_yaml

            with tempfile.TemporaryDirectory() as tmpdir:
                window.save_yaml("test_map", tmpdir)
                save_path = os.path.join(tmpdir, "test_map.yaml")
                self.assertTrue(os.path.exists(save_path))
                with open(save_path) as f:
                    self.assertEqual(f.read(), expected_yaml)
        finally:
            window.destroy()

    async def test_save_yaml_appends_extension(self) -> None:
        """Tests that save_yaml automatically appends the .yaml extension when omitted.

        Verifies that a filename without an extension gets .yaml appended.
        """
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            window._ros_yaml_text = "image: map.png\nresolution: 0.05"

            with tempfile.TemporaryDirectory() as tmpdir:
                window.save_yaml("my_map", tmpdir)
                self.assertTrue(os.path.exists(os.path.join(tmpdir, "my_map.yaml")))
                self.assertFalse(os.path.exists(os.path.join(tmpdir, "my_map")))
        finally:
            window.destroy()

    async def test_save_yaml_preserves_existing_extension(self) -> None:
        """Tests that save_yaml does not double-append .yaml when the extension is already present.

        Verifies that filenames already ending in .yaml or .yml are not modified.
        """
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            window._ros_yaml_text = "image: map.png\nresolution: 0.05"

            with tempfile.TemporaryDirectory() as tmpdir:
                window.save_yaml("my_map.yaml", tmpdir)
                self.assertTrue(os.path.exists(os.path.join(tmpdir, "my_map.yaml")))
                self.assertFalse(os.path.exists(os.path.join(tmpdir, "my_map.yaml.yaml")))

                window.save_yaml("my_map.yml", tmpdir)
                self.assertTrue(os.path.exists(os.path.join(tmpdir, "my_map.yml")))
        finally:
            window.destroy()

    def _setup_update_yaml_models(self, window: OccupancyMapWindow, stem: str, config_type_index: int = 0) -> None:
        """Injects the minimal model mocks needed for _update_yaml into a window instance.

        Args:
            window: The OccupancyMapWindow instance to configure.
            stem: The image filename stem to set.
            config_type_index: Index for the config type dropdown.
        """
        window._map_bottom_left = [1.0, 2.0]
        window._map_scale = 0.05
        window._map_scale_to_meters = 1.0
        window._models["image_name"] = _StringModel(stem)
        window._models["config_type"] = _DropdownModel(config_type_index)
        window._models["config_data"] = _StringModel()

    async def test_update_yaml_no_map_data(self) -> None:
        """Tests that _update_yaml warns and does nothing when no map has been generated yet."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            # _map_bottom_left is None by default — should return early without crash or YAML change
            window._update_yaml()
            self.assertIsNone(window._ros_yaml_text)
        finally:
            window.destroy()

    async def test_update_yaml_empty_stem(self) -> None:
        """Tests that _update_yaml warns and does nothing when the image filename field is empty."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            self._setup_update_yaml_models(window, stem="")
            original_yaml = "image: original.png\nresolution: 0.05"
            window._ros_yaml_text = original_yaml

            window._update_yaml()

            self.assertEqual(window._ros_yaml_text, original_yaml)
            self.assertEqual(window._models["config_data"].as_string, "")
        finally:
            window.destroy()

    async def test_update_yaml_appends_png(self) -> None:
        """Tests that _update_yaml always appends .png to the stem in the YAML image field."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            self._setup_update_yaml_models(window, stem="my_map")

            window._update_yaml()

            self.assertIn("image: my_map.png", window._ros_yaml_text)
        finally:
            window.destroy()

    async def test_update_yaml_content(self) -> None:
        """Tests that _update_yaml builds the full correct ROS YAML content."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            self._setup_update_yaml_models(window, stem="warehouse")

            window._update_yaml()

            yaml = window._ros_yaml_text
            self.assertIn("image: warehouse.png", yaml)
            self.assertIn(f"resolution: {float(0.05 / 1.0)}", yaml)
            self.assertIn(f"origin: [{float(1.0 / 1.0)}, {float(2.0 / 1.0)}, 0.0000]", yaml)
            self.assertIn("negate: 0", yaml)
            self.assertIn(f"occupied_thresh: {ROS_OCCUPIED_THRESHOLD}", yaml)
            self.assertIn(f"free_thresh: {ROS_FREE_THRESHOLD}", yaml)
        finally:
            window.destroy()

    async def test_update_yaml_updates_config_data_in_ros_mode(self) -> None:
        """Tests that _update_yaml updates the config_data field when Coordinate Type is ROS YAML (index 0)."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            self._setup_update_yaml_models(window, stem="my_map", config_type_index=0)

            window._update_yaml()

            self.assertIn("image: my_map.png", window._models["config_data"].as_string)
        finally:
            window.destroy()

    async def test_update_yaml_skips_config_data_in_coordinates_mode(self) -> None:
        """Tests that _update_yaml does not touch config_data when Coordinate Type is Coordinates (index 1)."""
        await omni.usd.get_context().new_stage_async()
        window = OccupancyMapWindow()
        try:
            self._setup_update_yaml_models(window, stem="my_map", config_type_index=1)

            window._update_yaml()

            # config_data should remain untouched (empty string from mock)
            self.assertEqual(window._models["config_data"].as_string, "")
            # but _ros_yaml_text should still be updated
            self.assertIn("image: my_map.png", window._ros_yaml_text)
        finally:
            window.destroy()
