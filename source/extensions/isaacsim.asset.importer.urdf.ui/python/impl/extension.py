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

"""URDF importer UI extension and delegate integration."""

import copy
import gc
import os
from collections import namedtuple
from pathlib import Path

import carb
import omni.ext
import omni.kit.app
import omni.kit.tool.asset_importer as ai
import omni.ui as ui
import omni.usd
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.utils import stage as stage_utils
from omni.kit.helper.file_utils import asset_types
from omni.kit.notification_manager import NotificationStatus, post_notification

from .option_widget import OptionWidget

_extension_instance = None


def is_urdf_file(path: str) -> bool:
    """Check whether a path points to a URDF file.

    Args:
        path: Path to check.

    Returns:
        True if the path has a URDF extension, otherwise False.

    """
    _, ext = os.path.splitext(path.lower())
    return ext in [".urdf", ".URDF"]


def _normalize_path_key(path: str) -> str:
    """Return a canonical ``_per_file_state`` key (case-insensitive on Windows).

    Args:
        path: File path to normalize.

    Returns:
        Normalized path key for per-file state lookup.
    """
    return os.path.normcase(os.path.normpath(path))


class _FileImportState:
    """Per-file state created by each ``build_options`` call.

    Holds an independent config, UI model dict, and OptionWidget so that
    multiple selected files in the asset-importer dialog each get their
    own settings panel and ROS package table.

    Args:
        config: URDF importer configuration for this file.
        models: UI value models keyed by option name.
        option_builder: Widget that builds the settings panel.
    """

    __slots__ = ("config", "models", "option_builder")

    def __init__(
        self,
        config: URDFImporterConfig,
        models: dict[str, ui.AbstractValueModel],
        option_builder: OptionWidget,
    ) -> None:
        self.config = config
        self.models = models
        self.option_builder = option_builder


class Extension(omni.ext.IExt):
    """UI Extension for the URDF Importer.

    Provides the user interface for importing URDF files into USD.
    Depends on isaacsim.asset.importer.urdf for core import functionality.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.

        """
        self._usd_context = omni.usd.get_context()
        self._extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        self._per_file_state: dict[str, _FileImportState] = {}
        self._last_config: URDFImporterConfig | None = None

        self._delegate = UrdfImporterDelegate(
            "Urdf Importer",
            [r"(.*\.urdf$)|(.*\.URDF$)"],
            ["Urdf Files (*.urdf, *.URDF)"],
        )

        self._delegate.set_importer(self)
        ai.register_importer(self._delegate)

        global _extension_instance
        _extension_instance = self

    def build_new_options(self, paths: list[str] | None = None) -> None:
        """Build one independent ``OptionWidget`, "ROS package scanning" per selected URDF file.

        Args:
            paths: URDF file paths selected in the file picker.
        """
        urdf_paths = [p for p in (paths or []) if is_urdf_file(p)]

        if not urdf_paths:
            config = URDFImporterConfig()
            self._init_config_defaults(config)
            models: dict[str, ui.AbstractValueModel] = {}
            option_builder = OptionWidget(models, config)
            option_builder.build_options()
            return

        from .package_scanner import scan_urdf_packages

        for path in urdf_paths:
            config = URDFImporterConfig()
            self._init_config_defaults(config)
            models: dict[str, ui.AbstractValueModel] = {}
            option_builder = OptionWidget(models, config)
            option_builder.build_options()
            packages = scan_urdf_packages(path)
            if packages:
                option_builder.populate_packages(packages)

            self._per_file_state[_normalize_path_key(path)] = _FileImportState(config, models, option_builder)

    @staticmethod
    def _init_config_defaults(config: URDFImporterConfig) -> None:
        """Set default values on a config instance.

        Args:
            config: Configuration instance to initialise.

        """
        config.usd_path = None
        config.merge_mesh = False
        config.debug_mode = False
        config.collision_from_visuals = False
        config.collision_type = "Convex Hull"
        config.allow_self_collision = False
        config.ros_package_paths = []

    def reset_config(self) -> None:
        """Clear all per-file import state."""
        self._per_file_state.clear()

    async def _start_import(self, path: str | None = None, **kargs: object) -> str | None:
        """Start the URDF import process.

        Args:
            path: Path to the URDF file to import.
            **kargs: Additional keyword arguments.

        Returns:
            Path to the imported prim in the USD stage, or None if import failed.

        """
        if not path:
            carb.log_error("URDF Importer: No path provided")
            return None

        state = self._per_file_state.pop(_normalize_path_key(path), None)
        if state:
            config = state.config
            models = state.models
            option_builder = state.option_builder
        else:
            config = URDFImporterConfig()
            self._init_config_defaults(config)
            models = {}
            option_builder = None

        export_folder = models["dst_path"].get_value_as_string() if models.get("dst_path") else ""
        if export_folder == "Same as Imported Model(Default)":
            export_folder = ""

        if export_folder:
            if not os.path.isdir(export_folder):
                export_folder = os.path.dirname(export_folder)
        else:
            export_folder = os.path.dirname(path)

        if option_builder:
            config.ros_package_paths = option_builder.get_ros_package_map()

        config.urdf_path = path
        config.usd_path = export_folder

        stage_utils.create_new_stage()
        importer = URDFImporter(config)

        output_path = importer.import_urdf()
        if not output_path:
            carb.log_error(f"Failed to import URDF at path: {path}")
            return None

        result, _ = stage_utils.open_stage(output_path)
        self._last_config = copy.deepcopy(config)
        if not result:
            carb.log_error(f"Failed to open stage at path: {output_path}")
            return None

        return output_path

    def _get_config(self) -> URDFImporterConfig | None:
        """Get the most recently used importer configuration.

        Returns:
            Configuration instance from the last successful import, or None.

        """
        return self._last_config

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        if hasattr(self, "_delegate"):
            self._delegate.destroy()
            ai.remove_importer(self._delegate)

        self._per_file_state.clear()
        global _extension_instance
        _extension_instance = None
        gc.collect()


class UrdfImporterDelegate(ai.AbstractImporterDelegate):
    """Delegate implementation for the asset importer integration.

    Construct with display name, file-type filters, and descriptions; registers
    URDF asset type icons with the asset browser.

    Args:
        name: Display name for the importer.
        filters: Regex filters for supported file types.
        descriptions: Descriptions for supported file types.

    """

    def __init__(self, name: str, filters: list[str], descriptions: list[str]) -> None:
        super().__init__()
        self._name = name
        self._filters = filters
        self._descriptions = descriptions
        self._importer: Extension | None = None
        # register the urdf icon to asset types
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        icon_path = Path(ext_path).joinpath("icons").absolute()
        AssetTypeDef = namedtuple("AssetTypeDef", "glyph thumbnail matching_exts")
        known_asset_types = asset_types.known_asset_types()
        known_asset_types["urdf"] = AssetTypeDef(
            f"{icon_path}/icoFileURDF.png",
            f"{icon_path}/icoFileURDF.png",
            [".urdf", ".URDF"],
        )

    def set_importer(self, importer: Extension) -> None:
        """Set the importer instance used for conversions.

        Args:
            importer: Extension instance that performs imports.

        """
        self._importer = importer

    def show_destination_frame(self) -> bool:
        """Return whether the destination frame should be shown.

        Returns:
            False to hide the destination frame.

        """
        return False

    def destroy(self) -> None:
        """Release references held by the delegate."""
        self._importer = None

    def _on_import_complete(self, file_paths: list[str]) -> None:
        """Handle import completion callbacks.

        Args:
            file_paths: List of imported file paths.

        """

    @property
    def name(self) -> str:
        """Get the display name for the importer.

        Returns:
            Human-readable importer label.
        """
        return self._name

    @property
    def filter_regexes(self) -> list[str]:
        """Get the filter regex patterns supported by the importer.

        Returns:
            File-type filter patterns for the asset import dialog.
        """
        return self._filters

    @property
    def filter_descriptions(self) -> list[str]:
        """Get the filter descriptions shown in the UI.

        Returns:
            User-facing descriptions parallel to ``filter_regexes``.
        """
        return self._descriptions

    def build_options(self, paths: list[str]) -> None:
        """Build options for the provided asset paths.

        Args:
            paths: Paths selected for import.

        """
        if self._importer is not None:
            self._importer.build_new_options(paths)
        else:
            carb.log_error("URDF Importer: Importer not initialized, cannot build options")

    def supports_usd_stage_cache(self) -> bool:
        """Report whether USD stage cache is supported.

        Returns:
            False since the importer does not support stage caching.

        """
        return False

    async def convert_assets(self, paths: list[str], **kargs: object) -> dict | None:
        """Convert selected URDF assets to USD.

        Args:
            paths: Paths selected for import.
            **kargs: Additional keyword arguments for import.

        Returns:
            An empty result dictionary on success, or None on failure.

        """
        if not paths:
            post_notification(
                "No file selected",
                "Please select a file to import",
                NotificationStatus.ERROR,
            )
            return None
        if self._importer is None:
            carb.log_warn("URDF Importer: Importer not initialized, cannot import assets")
            return None
        for path in paths:
            await self._importer._start_import(path=path, **kargs)
        return {}


def get_instance() -> Extension | None:
    """Return the active URDF UI extension instance.

    Returns:
        Extension instance if initialized, otherwise None.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.urdf.ui.impl import extension
        >>> extension.get_instance()  # doctest: +SKIP

    """
    return _extension_instance
