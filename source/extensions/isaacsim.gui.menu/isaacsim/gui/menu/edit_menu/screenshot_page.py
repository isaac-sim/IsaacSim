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

"""Preferences page for screenshot capture settings."""

import os

import carb.settings
import omni.ui as ui
from omni.kit.window.preferences import PERSISTENT_SETTINGS_PREFIX, PreferenceBuilder, SettingType, show_file_importer

__all__ = ["ScreenshotPreferences"]


class ScreenshotPreferences(PreferenceBuilder):
    """Preferences page for configuring screenshot capture."""

    def __init__(self) -> None:
        super().__init__("Capture Screenshot")

        self._settings = carb.settings.get_settings()
        self._settings_widget: ui.Widget | None = None

        # setup captureFrame paths
        template_path = self._settings.get(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/path")
        if template_path is None or template_path == "./":
            template_path = self._settings.get("/app/captureFrame/path")
            if template_path[-1] != "/":
                template_path += "/"
            self._settings.set(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/path", template_path)

        # 3D viewport
        self._settings.set_default_bool(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/viewport", True)

        # ansel defaults
        if self._is_ansel_enabled():  # pragma: no cover
            ansel_quality_path = PERSISTENT_SETTINGS_PREFIX + "/exts/omni.ansel/quality"
            self._settings.set_default_string(ansel_quality_path, "Medium")
            ansel_super_resolution_size_path = PERSISTENT_SETTINGS_PREFIX + "/exts/omni.ansel/superResolution/size"
            self._settings.set_default_string(ansel_super_resolution_size_path, "2x")

        # create captureFrame directory
        original_umask = os.umask(0o777)
        if not os.path.isdir(template_path):
            try:
                os.makedirs(template_path)
            except OSError:
                carb.log_error(f"Failed to create directory {template_path}")
        os.umask(original_umask)

    def build(self) -> None:
        """Build the screenshot preferences UI.

        Example:
            .. code-block:: python

                prefs = ScreenshotPreferences()
                prefs.build()
        """
        # The path widget. It's not standard because it has the button browse. Since the main layout has two columns,
        # we need to create another layout and put it to the main one.
        with ui.VStack(height=0):
            with self.add_frame("Capture Screenshot"):
                with ui.VStack():
                    self._settings_widget = self.create_setting_widget(
                        "Path to save screenshots",
                        PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/path",
                        SettingType.STRING,
                        clicked_fn=self._on_browse_button_fn,
                    )

                    self.create_setting_widget(
                        "Capture only the 3D viewport",
                        PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/viewport",
                        SettingType.BOOL,
                    )

                    # Show Ansel super resolution configuration, only when Ansel enabled
                    self._create_ansel_super_resolution_settings()

    def _add_ansel_settings(self) -> None:  # pragma: no cover
        """Add Ansel quality settings when Ansel is enabled."""
        # check if Ansel enabled. If not, do not show Ansel settings
        if not self._is_ansel_enabled():
            return

        self.create_setting_widget_combo(
            "Quality", PERSISTENT_SETTINGS_PREFIX + "/exts/omni.ansel/quality", ["Low", "Medium", "High"]
        )

    def _create_ansel_super_resolution_settings(self) -> None:  # pragma: no cover
        """Add Ansel super resolution settings when available."""
        # check if Ansel enabled. If not, do not show Ansel settings
        if not self._is_ansel_enabled():
            return

        self.spacer()

        with self.add_frame("Super Resolution"):
            with ui.VStack():
                self.create_setting_widget_combo(
                    "Size",
                    PERSISTENT_SETTINGS_PREFIX + "/exts/omni.ansel/superResolution/size",
                    ["2x", "4x", "8x", "16x", "32x"],
                )
                self._add_ansel_settings()

    def _is_ansel_enabled(self) -> bool:
        """Check whether Ansel support is enabled.

        Returns:
            True when Ansel settings are available.
        """
        return self._settings.get("/exts/omni.ansel/enable")

    def _on_browse_button_fn(self, owner: ui.Widget) -> None:
        """Handle Browse button clicks for selecting a directory.

        Args:
            owner: The widget that triggered the action.
        """
        navigate_to = self._settings.get(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/path")
        show_file_importer(
            "Select Screenshot Directory",
            click_apply_fn=self._on_dir_pick,
            filename_url=navigate_to,
            show_only_folders=True,
        )

    def _on_dir_pick(self, real_path: str) -> None:
        """Handle directory selection from the file importer.

        Args:
            real_path: Selected directory path.
        """
        if self._settings_widget is None:
            return
        directory = self.cleanup_slashes(real_path, is_directory=True)
        self._settings.set(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/path", directory)
        self._settings_widget.model.set_value(directory)
