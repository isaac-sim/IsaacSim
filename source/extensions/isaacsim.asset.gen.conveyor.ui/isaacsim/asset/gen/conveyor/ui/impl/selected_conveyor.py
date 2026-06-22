"""Widget for displaying and interacting with a selected conveyor track card."""

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

from __future__ import annotations

from typing import Any

import omni.ui as ui

from .conveyor_builder.conveyor_track import ConveyorTrack, Ramp, Type


class SelectedConveyorWidget:
    """Widget displaying the currently selected conveyor track with action buttons.

    Args:
        **kwargs: Keyword arguments including `name`, `track_list`, `selected_track`,
            `add_track_fn`, `selection_changed_fn`, `remove_track_fn`,
            `anchor_changed_fn`, and `flip_fn`.
    """

    def __init__(self, **kwargs: Any) -> None:
        self._frame = ui.Frame(**kwargs, width=128, height=128)
        self.name = kwargs.get("name", "")
        self.style = kwargs
        self.input_boxes = []
        self.output_boxes = []
        self.track_list = kwargs.get("track_list", [])
        self.selected_conveyor = kwargs.get("selected_track", self.track_list[0] if self.track_list else None)
        self.on_add_track_callback = kwargs.get("add_track_fn", None)
        self.on_selection_changed_callback = kwargs.get("selection_changed_fn", None)
        self.on_remove_track_callback = kwargs.get("remove_track_fn", None)
        self.on_anchor_changed_callback = kwargs.get("anchor_changed_fn", None)
        self.on_flip_callback = kwargs.get("flip_fn", None)
        self.current_anchor = 0
        self.available_anchors = []
        # self.build_ui()

    def on_flip(self, track: ConveyorTrack) -> None:
        """Invoke the flip callback for the given track.

        Args:
            track: Track whose orientation should be flipped.
        """
        if self.on_flip_callback:
            self.on_flip_callback(track)

    def on_add_track(self, track: ConveyorTrack) -> None:
        """Invoke the add-track callback for the given track.

        Args:
            track: Track the user chose to add.
        """
        if self.on_add_track_callback:
            self.on_add_track_callback(track)

    def on_selection_changed(self, track: ConveyorTrack) -> None:
        """Invoke the selection-changed callback for the given track.

        Args:
            track: Newly selected track, or ``None`` if cleared.
        """
        if self.on_selection_changed_callback:
            self.on_selection_changed_callback(track)

    def on_remove_track(self, track: ConveyorTrack) -> None:
        """Invoke the remove-track callback for the given track.

        Args:
            track: Track the user chose to remove.
        """
        if self.on_remove_track_callback:
            self.on_remove_track_callback(track)

    def update_list(self, new_list: list[ConveyorTrack], new_selection: int = 0, direction: int = 1) -> None:
        """Update the track list and select the specified track index.

        Args:
            new_list: Replacement list of selectable tracks.
            new_selection: Index into ``new_list`` to show as selected.
            direction: Passed through when resolving anchors for the selection.
        """
        self.track_list = new_list
        self.update_selection(self.track_list[new_selection], direction=direction)

    def update_selection(
        self, new_selection: ConveyorTrack | None, available_anchors: list[str] | None = None, direction: int = 1
    ) -> None:
        """Update the selected conveyor and available anchors, then rebuild UI.

        Args:
            new_selection: Track to display, or ``None`` to clear.
            available_anchors: Explicit anchor list; if omitted, derived from ``new_selection``.
            direction: Used when computing default anchors from the selected track.
        """
        if self.selected_conveyor != new_selection or self.available_anchors != available_anchors:
            self.selected_conveyor = new_selection
            # if new_selection:
            #     self.current_anchor = 0 if self.name != "current" else min(1, len(self.selected_conveyor.anchors) - 1)
            # else:
            self.current_anchor = 0
            if available_anchors:
                self.available_anchors = available_anchors
            else:
                if self.selected_conveyor:
                    self.available_anchors = self.selected_conveyor.get_anchors(direction)
                else:
                    self.available_anchors = []
            # Workaround for First item - If track type is flat and both anchors are free, use anchor 1.
            if self.name == "current":
                if len(self.available_anchors) > 1 and self.selected_conveyor.type == Type.STRAIGHT:
                    self.current_anchor = 1
                if len(self.available_anchors) > 2 and self.selected_conveyor.type in [
                    Type.Y_MERGE,
                    Type.T_MERGE,
                    Type.FORK_MERGE,
                ]:
                    self.current_anchor = 1
            self.on_selection_changed(new_selection)
        # print("available anchors", available_anchors, self.available_anchors)
        self.build_ui()

    def anchor_changed(self, new_anchor: int) -> None:
        """Handle anchor button selection change.

        Args:
            new_anchor: Index of the anchor button that was pressed.
        """
        self.current_anchor = new_anchor
        for i in range(len(self.anchor_btns)):
            self.anchor_btns[i].selected = new_anchor == i
        self.on_anchor_changed(self.available_anchors[new_anchor])

    def on_anchor_changed(self, new_anchor: str) -> None:
        """Invoke the anchor-changed callback.

        Args:
            new_anchor: Anchor name now selected.
        """
        if self.on_anchor_changed_callback:
            self.on_anchor_changed_callback(new_anchor)

    def get_current_anchor(self) -> str:
        """Get the name of the currently selected anchor point.

        Returns:
            Anchor name at ``current_anchor``, or empty string if no conveyor is selected.
        """
        if self.selected_conveyor:
            return self.available_anchors[self.current_anchor]
        return ""

    def get_next_anchor(self, direction: int) -> str:
        """Get the anchor point name for the given direction.

        Args:
            direction: Sign or mode used when resolving anchor ordering on the track.

        Returns:
            Anchor name at ``current_anchor`` for ``direction``, or empty string if none.
        """
        if self.selected_conveyor:
            return self.selected_conveyor.get_anchors(direction)[self.current_anchor]
        return ""

    def build_ui(self) -> None:
        """Build the conveyor card UI with thumbnail, anchor buttons, and actions."""
        with self._frame:
            with ui.ZStack(width=144, height=144, alignment=ui.Alignment.CENTER):
                self.style["alignment"] = ui.Alignment.CENTER
                ui.Rectangle(
                    **self.style, style_type_name_override="ConveyorCard", selected=bool(self.selected_conveyor)
                )
                if self.selected_conveyor:
                    with ui.ZStack():
                        # ui.Rectangle(
                        #     style_type_name_override="ConveyorCardBackground",
                        #     selected=bool(self.selected_conveyor) and self.name == "current",
                        #     width=100,
                        #     height=100,
                        # )
                        with ui.HStack():
                            ui.ImageWithProvider(
                                self.selected_conveyor.get_thumb(),
                                **self.style,
                                width=ui.Pixel(144),
                                height=ui.Pixel(144),
                                style={"margin": 15},
                            )
                        with ui.VStack(height=ui.Pixel(144)):
                            ui.Spacer(height=ui.Pixel(94))
                            with ui.ZStack():
                                with ui.HStack():
                                    ui.Spacer()

                                    if self.name == "current":
                                        ui.Button(
                                            name="delete",
                                            width=50,
                                            height=50,
                                            style={"border_radius": 6},
                                            clicked_fn=lambda: self.on_remove_track(self.selected_conveyor),
                                        )
                                    else:
                                        # ui.Spacer()
                                        ui.Button(
                                            name="add_box",
                                            width=50,
                                            height=50,
                                            style={"border_radius": 6},
                                            clicked_fn=lambda: self.on_add_track(self.selected_conveyor),
                                        )

                                    ui.Spacer()
                                if self.name != "current" and self.selected_conveyor.type in [
                                    Type.Y_MERGE,
                                    Type.T_MERGE,
                                ]:

                                    with ui.VStack():
                                        ui.Spacer(height=ui.Pixel(15))
                                        with ui.HStack():
                                            ui.Spacer()
                                            ui.Button(
                                                name="flip",
                                                width=35,
                                                height=35,
                                                style={"border_radius": 6},
                                                clicked_fn=lambda: self.on_flip(self.selected_conveyor),
                                            )

                    with ui.HStack(height=ui.Pixel(30), width=ui.Fraction(1)):
                        ui.Spacer(height=ui.Pixel(30))
                        if self.track_list and len(self.track_list) > 1:
                            for track in self.track_list:
                                ui.Button(
                                    image_url=str(track.get_thumb()),
                                    width=30,
                                    height=30,
                                    style={"border_radius": 6},
                                    style_type_name_override="CardButton",
                                    selected=track == self.selected_conveyor,
                                    clicked_fn=lambda t=track: self.update_selection(t),
                                )
                        else:
                            ui.Label(self.name, style={"color": 0x44FFFFFF})
                        ui.Spacer(height=ui.Pixel(30))
                    # with ui.HStack(height=ui.Pixel(30)):
                    #     if self.name == "current":
                    #         ui.Button(name="delete", width=30)
                    #     else:
                    # ui.Button(name="rotate_left", width=30)
                    # ui.Button(name="rotate_right", width=30)
                    # ui.Spacer()
                    # ui.Button(name="add_box", width=30)
                    self.anchor_btns = []
                    anchors = list(self.available_anchors)
                    if self.selected_conveyor.ramp != Ramp.FLAT:
                        anchors = [anchors[0]]
                    if len(anchors) > 1:
                        with ui.Placer(
                            offset_x=5 if self.name != "current" else 125,
                            offset_y=30 if self.track_list and len(self.track_list) > 1 else 0,
                            width=1,
                        ):
                            with ui.VStack(
                                height=ui.Pixel(100 if self.track_list and len(self.track_list) > 1 else 130)
                            ):
                                ui.Spacer()
                                for i in range(len(self.available_anchors)):
                                    self.anchor_btns.append(
                                        ui.Button(
                                            name="square",
                                            selected=i == self.current_anchor,
                                            width=15,
                                            height=15,
                                            clicked_fn=lambda a=i: self.anchor_changed(a),
                                        )
                                    )
                                    if i + 1 < len(self.available_anchors):
                                        ui.Spacer(height=15)
                                ui.Spacer()
                    # with ui.Placer(offset_x=5, offset_y=100):
                    #     ui.Button(name="square", selected=False, width=15, height=15)
                else:
                    ui.Label(
                        "Select a track to connect or add a start",
                        word_wrap=True,
                        alignment=ui.Alignment.CENTER,
                        style={"color": 0x88FFFFDD},
                    )
