"""Conveyor builder preferences page and settings management."""

# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
from collections.abc import Callable
from pathlib import Path
from typing import Any

import carb
import omni
import omni.ui as ui
from isaacsim.storage.native import nucleus
from omni.kit.window.preferences import PreferenceBuilder

SETTINGS_PATH = "/persistent/exts/isaacsim.asset.gen.conveyor.ui.settings"

ASSETS_LOCATION = f"{SETTINGS_PATH}/assets_location"
CONFIG_LOCATION = f"{SETTINGS_PATH}/config_location"


def singleton(class_: type) -> object:
    """A singleton decorator that ensures only one instance of a class is created.

    Args:
        class_: The class to be converted to a singleton.

    Returns:
        A function that returns the singleton instance of the class.
    """
    instances = {}

    def getinstance(*args: Any, **kwargs: Any) -> Any:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


def create_filepicker(title: str, click_apply_fn: Callable = None, error_fn: Callable = None) -> None:
    """Creates a file picker dialog for selecting files or directories.

    Args:
        title: The title displayed in the file picker dialog.
        click_apply_fn: Callback function executed when a file is selected.
        error_fn: Callback function executed when an error occurs.
    """
    from omni.kit.window.filepicker import FilePickerDialog

    async def on_click_handler(filename: str, dirname: str, dialog: FilePickerDialog, click_fn: Callable) -> None:
        dirname = dirname.strip()
        if filename and dirname and not dirname.endswith("/"):
            dirname += "/"
        fullpath = f"{dirname}{filename}"
        if click_fn:
            click_fn(fullpath)
        dialog.hide()

    dialog = FilePickerDialog(
        title,
        allow_multi_selection=False,
        apply_button_label="Select",
        click_apply_handler=lambda filename, dirname: asyncio.ensure_future(
            on_click_handler(filename, dirname, dialog, click_apply_fn)
        ),
        click_cancel_handler=lambda filename, dirname: dialog.hide(),
        error_handler=error_fn,
    )


@singleton
class ConveyorBuilderPreferences(PreferenceBuilder):
    """Preferences builder for the Conveyor Builder extension.

    This class creates a preference panel that allows users to configure settings for the Conveyor Builder extension.
    It provides UI controls for setting the conveyor assets location and conveyor configuration file paths.
    The class is implemented as a singleton to ensure only one instance exists throughout the application lifecycle.

    The preference panel includes:
    - Conveyor Assets Location: Directory path where conveyor assets are stored
    - Conveyor Config: File path to the JSON configuration file containing track type definitions

    Both settings include browse buttons for easy file/directory selection and reset buttons to restore default values.
    The assets location defaults to the Isaac/Props/Conveyors/ directory in the assets root path, while the config
    file defaults to the track_types.json file included with the extension.
    """

    def __init__(self) -> None:
        super().__init__("Conveyor Builder")
        self._settings = carb.settings.get_settings()

    def reset_config_default(self) -> None:
        """Resets the conveyor configuration file path to the default track_types.json location."""
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.gen.conveyor.ui")
        extension_path = ext_manager.get_extension_path(ext_id)
        cfg = Path(extension_path).joinpath("data").joinpath("track_types.json")
        self._settings.set(CONFIG_LOCATION, str(cfg))

    def reset_assets_default(self) -> None:
        """Resets the conveyor assets location to the default Isaac conveyor assets directory."""
        timeout = carb.settings.get_settings().get("/persistent/isaac/asset_root/timeout")
        carb.settings.get_settings().set("/persistent/isaac/asset_root/timeout", 1.0)
        path = nucleus.get_assets_root_path()
        if timeout:
            carb.settings.get_settings().set("/persistent/isaac/asset_root/timeout", timeout)
        self._settings.set(ASSETS_LOCATION, f"{path}/Isaac/Props/Conveyors/")

    def cleanup_slashes(self, path: str, is_directory: bool = False) -> str:
        """Makes path/slashes uniform.

        Args:
            path: path
            is_directory: is path a directory, so final slash can be added

        Returns:
            path
        """
        # path = path.replace(":/", "://", 1)
        if is_directory:
            if path[-1] != "/":
                path += "/"
        return path.replace("\\", "/")

    def build(self) -> None:
        """Builds the preferences UI with conveyor assets location and config file settings."""
        with ui.VStack(height=0):
            with self.add_frame("General"):
                with ui.VStack(height=0, spacing=5):
                    with ui.HStack(height=24, spacing=4):
                        ui.Label("Conveyor Assets Location", width=290)
                        widget = ui.StringField(height=20)
                        widget.model.set_value(self.assets_location)
                        widget.model.add_end_edit_fn(
                            lambda a, w=widget: self._on_file_pick(a.get_value_as_string(), w, ASSETS_LOCATION)
                        )

                        def reset() -> None:
                            self.reset_assets_default()
                            widget.model.set_value(self.assets_location)

                        ui.Button(
                            style={"image_url": "resources/icons/folder.png"},
                            clicked_fn=lambda p=self.cleanup_slashes(
                                self.assets_location
                            ), w=widget: self._on_browse_button_fn(p, w, ASSETS_LOCATION),
                            width=24,
                        )

                        def reset_asset(w: Any) -> None:
                            self.reset_assets_default()
                            w.model.set_value(self.assets_location)

                        ui.Button(
                            "Reset to Default",
                            clicked_fn=lambda a=widget: reset_asset(a),
                            width=24,
                        )

                    with ui.HStack(height=24, spacing=4):
                        ui.Label("Conveyor Config", width=290)
                        widget = ui.StringField(height=20)
                        widget.model.set_value(self.config_file)
                        widget.model.add_end_edit_fn(
                            lambda a, w=widget: self._on_file_pick(a.get_value_as_string(), w, CONFIG_LOCATION)
                        )
                        ui.Button(
                            style={"image_url": "resources/icons/folder.png"},
                            clicked_fn=lambda p=self.cleanup_slashes(
                                self.config_file
                            ), w=widget: self._on_browse_button_fn(p, w, CONFIG_LOCATION),
                            width=24,
                        )

                        def reset_cfg(w: Any) -> None:
                            self.reset_config_default()
                            w.model.set_value(self.config_file)

                        ui.Button(
                            "Reset to Default",
                            clicked_fn=lambda a=widget: reset_cfg(a),
                            width=24,
                        )
        ui.Spacer(height=ui.Fraction(1))

    def _on_browse_button_fn(self, path: str, widget: object, setting: str) -> None:
        """Called when the user picks the Browse button.

        Args:
            path: The current path value.
            widget: The UI widget associated with the browse button.
            setting: The setting key to update when a path is selected.
        """
        file_pick = create_filepicker(
            title="Select Directory" if setting == ASSETS_LOCATION else "Select Config File",
            click_apply_fn=lambda p=self.cleanup_slashes(path), w=widget: self._on_file_pick(
                p, widget=w, setting=setting
            ),
        )
        # file_pick.show(path)

    def _on_file_pick(self, full_path: str, widget: object, setting: str) -> None:
        """Called when the user accepts directory in the Select Directory dialog.

        Args:
            full_path: The selected file or directory path.
            widget: The UI widget to update with the new path.
            setting: The setting key to store the selected path.
        """
        directory = self.cleanup_slashes(full_path, not full_path.endswith(".json"))
        self._settings.set(setting, directory)
        widget.model.set_value(directory)

    @property
    def assets_location(self) -> str:
        """Location path for conveyor assets.

        Returns:
            The conveyor assets directory path.
        """
        if self._settings.get(ASSETS_LOCATION) is None:
            self.reset_assets_default()

        return self._settings.get(ASSETS_LOCATION)

    @property
    def config_file(self) -> str:
        """Path to the conveyor configuration file.

        Returns:
            The conveyor configuration file path.
        """
        if self._settings.get(CONFIG_LOCATION) is None:
            self.reset_config_default()

        return self._settings.get(CONFIG_LOCATION)
