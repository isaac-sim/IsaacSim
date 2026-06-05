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

"""Detail view implementation for the Isaac Sim content browser."""

import os
from collections import namedtuple
from datetime import datetime
from pathlib import Path

import omni.client
import omni.kit.app
import omni.ui as ui
from omni.kit.async_engine import run_coroutine
from omni.kit.helper.file_utils import asset_types
from omni.kit.window.filepicker import DetailFrameController
from pxr import Usd

EXTENSION_FOLDER_PATH = Path(omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__))
ICON_PATH = EXTENSION_FOLDER_PATH.joinpath("icons")
USD_FILETYPES = (".usd", ".usda", ".usdc", ".usdz")
IMPORTER_FILETYPES = (".urdf", ".mjcf")
TXT_FILETYPES = (".json", ".yaml", ".yml", ".pt", ".txt", ".log")


class ExtendedFileInfo(DetailFrameController):
    """Extended file info shown in detail view."""

    MockListEntry = namedtuple("MockListEntry", "relative_path modified_time created_by modified_by size")
    _empty_list_entry = MockListEntry("File info", datetime.now(), "", "", 0)
    """Default MockListEntry used when no file information is available or when file stat operation fails."""

    def __init__(self) -> None:
        super().__init__(
            build_fn=self._build_ui_impl,
            selection_changed_fn=self._on_selection_changed_impl,
            destroy_fn=self._destroy_impl,
        )
        self._widget = None
        self._current_url = ""
        self._resolve_subscription = None
        self._isaac_content_frame = None

    def build_header(self, collapsed: bool, title: str) -> None:
        """Builds the header. It is used to show the header of the file info.

        Args:
            collapsed: True if the header is collapsed.
            title: Title of the header to be shown.
        """
        with ui.HStack(style_type_name_override="DetailFrame.Header"):
            if collapsed:
                ui.ImageWithProvider(f"{ICON_PATH}/arrow_right.svg", width=20, height=20)
            else:
                ui.ImageWithProvider(f"{ICON_PATH}/arrow_down.svg", width=20, height=20)
            icon = asset_types.get_icon(title)
            if icon is not None:
                ui.ImageWithProvider(icon, width=18, style_type_name_override="DetailFrame.Header.Icon")
                ui.Spacer(width=4)
            ui.Label(
                "File Information", elided_text=True, tooltip=title, style_type_name_override="DetailFrame.Header.Label"
            )

    def _build_ui_impl(self, selected: list[str] | None = None) -> None:
        """Builds the UI implementation for the file info widget.

        Args:
            selected: List of selected file paths.
        """
        if selected is None:
            selected = []
        self._widget = ui.Frame()
        run_coroutine(self._build_ui_async(selected))

    async def _build_ui_async(self, selected: list[str] | None = None) -> None:
        """Builds the UI asynchronously, retrieving file information and displaying file details.

        Args:
            selected: List of selected file paths.
        """
        if selected is None:
            selected = []
        entry = None
        if len(selected) == 0:
            self._frame.title = "No files selected"
        elif len(selected) > 1:
            self._frame.title = "Multiple files selected"
        else:
            result, entry = await omni.client.stat_async(selected[-1])
            if result == omni.client.Result.OK and entry:
                self._frame.title = entry.relative_path or os.path.basename(selected[-1])
                if self._current_url != selected[-1]:
                    self._current_url = selected[-1]
                    self._resolve_subscription = omni.client.resolve_subscribe_with_callback(
                        self._current_url,
                        [self._current_url],
                        None,
                        lambda result, event, entry, url: self._on_file_change_event(result, entry),
                    )
            else:
                self._frame.title = os.path.basename(selected[-1])
                entry = None

            # check for variants
            variant_set_names = []
            if entry:
                if entry.size < 50000 and entry.relative_path.endswith(USD_FILETYPES):
                    stage = Usd.Stage.Open(self._current_url, load=Usd.Stage.LoadNone)
                    if stage.HasDefaultPrim():
                        vset = stage.GetDefaultPrim().GetVariantSets()
                        variant_set_names = vset.GetNames()

            entry = entry or self._empty_list_entry
            with self._widget:
                with ui.ZStack():
                    ui.Rectangle()
                    with ui.VStack():
                        ui.Rectangle(height=2, style_type_name_override="DetailFrame.Separator")
                        with ui.VStack(style_type_name_override="DetailFrame.Body"):
                            self._isaac_content_frame = ui.Frame()
                            with self._isaac_content_frame:
                                if variant_set_names:
                                    with ui.VStack():
                                        ui.Label("Variant Options", font_size=10)
                                        for name in variant_set_names:
                                            ui.Spacer(height=8)
                                        ui.Label(name, style_type_name_override="DetailFrame.LineItem.left_aligned")
                                        variant_set = vset.GetVariantSet(name)
                                        variants = variant_set.GetVariantNames()
                                        for variant in variants:
                                            with ui.HStack():
                                                ui.Spacer(width=10)
                                                ui.Label(
                                                    variant,
                                                    style_type_name_override="DetailFrame.LineItem.left_aligned",
                                                )
                                        ui.Spacer(height=8)

                                elif entry.relative_path.endswith(IMPORTER_FILETYPES):
                                    ui.Label("URDF file, use File->Import to convert to USD")
                                elif entry.relative_path.endswith(TXT_FILETYPES):
                                    ui.Label("Text file, Right Click and select Download to download file")
                                else:
                                    ui.Label("No Additional Information")

    def _on_file_change_event(self, result: omni.client.Result, entry: omni.client.ListEntry) -> None:
        """Handles file change events from the file resolver subscription.

        Args:
            result: The result of the file operation.
            entry: The file list entry information.
        """
        if result == omni.client.Result.OK and self._current_url:
            # isaac sim content info don't need to be updated on every single file change. only when clicked on.
            pass

    def _on_selection_changed_impl(self, selected: list[str] | None = None) -> None:
        """Handles selection change events by rebuilding the UI with the new selection.

        Args:
            selected: List of selected file paths.
        """
        if selected is None:
            selected = []
        self._build_ui_impl(selected)

    def _destroy_impl(self, _: object) -> None:
        """Cleans up resources when the widget is destroyed."""
        if self._widget:
            self._widget.destroy()
        self._widget = None
        self._resolve_subscription = None
        if self._isaac_content_frame:
            self._isaac_content_frame.destroy()
        self._isaac_content_frame = None
