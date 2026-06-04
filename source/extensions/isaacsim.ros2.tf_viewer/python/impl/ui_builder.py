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

"""Provides UI management for the ROS 2 TF viewer extension."""

from collections.abc import Callable

import isaacsim.gui.components.ui_utils as ui_utils
import omni.kit.menu.utils
import omni.ui as ui

from .viewport_scene import ViewportScene


class UIBuilder:
    """Manage extension UI.

    Args:
        menu_path: The menu path where the window menu item is placed.
        window_title: The title of the TF Viewer window.
        viewport_scene: The viewport scene used for rendering TF frames.
        on_visibility_changed_callback: Callback invoked when window visibility changes.
        on_reset_callback: Callback invoked when the reset button is clicked.
    """

    def __init__(
        self,
        menu_path: str,
        window_title: str,
        viewport_scene: ViewportScene,
        on_visibility_changed_callback: Callable[[bool], None],
        on_reset_callback: Callable[[], None],
    ) -> None:
        self._window_title = window_title
        self._viewport_scene = viewport_scene
        self._on_visibility_changed_callback = on_visibility_changed_callback
        self._on_reset = on_reset_callback

        self._root_frame = "World"
        self._update_frequency = 10

        ui.Workspace.set_show_window_fn(self._window_title, self.show_window)
        self._menu_helper = omni.kit.menu.utils.MenuHelperExtension()
        self._menu_helper.menu_startup(self._window_title, self._window_title, menu_path)
        self._window = None

    @property
    def root_frame(self) -> str:
        """Return the root frame name."""
        return self._root_frame

    @property
    def update_frequency(self) -> int:
        """Return the update frequency."""
        return self._update_frequency

    def show_window(self, value: bool) -> None:
        """Show or hide the TF Viewer window."""
        self._build_ui()
        self._window.visible = value

    def _on_visibility_changed(self, visible: bool) -> None:
        self._on_visibility_changed_callback(visible)
        self._menu_helper.menu_refresh()

    def _on_update_frequency_changed(self, model: object) -> None:
        frequency = model.as_int
        if frequency < 1:
            frequency = 1
        self._update_frequency = frequency

    def _on_frame_changed(self, frame_type: str, model: object, item: object) -> None:
        try:
            selected_index = model.get_item_value_model().get_value_as_int()
            value = model.get_item_value_model(model.get_item_children()[selected_index]).get_value_as_string()
        except Exception:
            return
        if frame_type == "root":
            self._root_frame = value

    def _build_ui(self) -> None:
        if not self._window:
            label_width = 120
            self._window = ui.Window(title=self._window_title, visible=False, width=375, height=270)
            with self._window.frame:
                with ui.VStack(spacing=5, height=0):
                    self._view = ui.CollapsableFrame(
                        title="TF Viewer",
                        height=0,
                        collapsed=False,
                        style=ui_utils.get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    with self._view:
                        with ui.VStack(spacing=5, height=0):

                            # root frame
                            with ui.HStack():
                                items = {"World", "world", "map"}
                                tooltip = "Frame on which to compute the transformations"
                                ui.Label(
                                    "Root Frame:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                self._ui_root_frame_combo_box = ui.ComboBox(
                                    1, *items, name="", width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER
                                ).model
                                self._ui_root_frame_combo_box.add_item_changed_fn(
                                    lambda m, i: self._on_frame_changed("root", m, i)
                                )
                                ui_utils.add_line_rect_flourish(False)

                            # frames
                            with ui.HStack():
                                tooltip = "Whether the frames (markers) are displayed"
                                ui.Label(
                                    "Show Frames:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                # show frames
                                self._ui_show_frames_checkbox = ui.SimpleBoolModel()
                                ui_utils.SimpleCheckBox(
                                    checked=True,
                                    model=self._ui_show_frames_checkbox,
                                    on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_frames_show(
                                        checked
                                    ),
                                )
                                # frame color
                                ui.Spacer(width=2)
                                self._ui_show_frames_color = ui.ColorWidget(
                                    1.0, 1.0, 1.0, 1.0, width=0, tooltip="Marker color"
                                ).model
                                for i, child in enumerate(self._ui_show_frames_color.get_item_children()):
                                    item_model = self._ui_show_frames_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(
                                        lambda m, i=i: self._viewport_scene.manipulator.set_frames_color(
                                            i, m.get_value_as_float()
                                        )
                                    )
                                # frame size
                                ui.Spacer(width=12)
                                self._ui_show_frames_size = ui.FloatDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=0,
                                    max=1,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Marker size (relative)",
                                ).model
                                self._ui_show_frames_size.add_value_changed_fn(
                                    lambda m: self._viewport_scene.manipulator.set_frames_size(m.as_float)
                                )
                                self._ui_show_frames_size.set_value(0.25)
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish()

                            # names
                            with ui.HStack():
                                tooltip = "Whether the frames' names are displayed"
                                ui.Label(
                                    "Show Names:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                # show names
                                self._ui_show_names_checkbox = ui.SimpleBoolModel()
                                ui_utils.SimpleCheckBox(
                                    checked=True,
                                    model=self._ui_show_names_checkbox,
                                    on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_names_show(
                                        checked
                                    ),
                                )
                                # text color
                                ui.Spacer(width=2)
                                self._ui_show_names_color = ui.ColorWidget(
                                    1.0, 1.0, 0.0, 1.0, width=0, tooltip="Text color"
                                ).model
                                for i, child in enumerate(self._ui_show_names_color.get_item_children()):
                                    item_model = self._ui_show_names_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(
                                        lambda m, i=i: self._viewport_scene.manipulator.set_names_color(
                                            i, m.get_value_as_float()
                                        )
                                    )
                                # text size
                                ui.Spacer(width=12)
                                self._ui_show_names_size = ui.FloatDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=0,
                                    max=1,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Text size (relative)",
                                ).model
                                self._ui_show_names_size.add_value_changed_fn(
                                    lambda m: self._viewport_scene.manipulator.set_names_size(m.as_float)
                                )
                                self._ui_show_names_size.set_value(0.4)
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish()

                            # axes
                            with ui.HStack():
                                tooltip = "Whether the frames's axes are displayed (RGB -> XYZ axes)"
                                ui.Label(
                                    "Show Axes:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                # show axes
                                self._ui_show_axes_checkbox = ui.SimpleBoolModel()
                                ui_utils.SimpleCheckBox(
                                    checked=True,
                                    model=self._ui_show_axes_checkbox,
                                    on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_axes_show(
                                        checked
                                    ),
                                )
                                # axes length
                                ui.Spacer(width=2)
                                self._ui_show_axes_length = ui.FloatDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=0,
                                    max=1,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Axis length (in meters)",
                                ).model
                                self._ui_show_axes_length.add_value_changed_fn(
                                    lambda m: self._viewport_scene.manipulator.set_axes_length(m.as_float)
                                )
                                self._ui_show_axes_length.set_value(0.1)
                                # axes thickness
                                ui.Spacer(width=12)
                                self._ui_show_axes_thickness = ui.FloatDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=0,
                                    max=1,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Axis thickness (relative)",
                                ).model
                                self._ui_show_axes_thickness.add_value_changed_fn(
                                    lambda m: self._viewport_scene.manipulator.set_axes_thickness(m.as_float)
                                )
                                self._ui_show_axes_thickness.set_value(0.2)
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish()

                            # arrows
                            with ui.HStack():
                                tooltip = (
                                    "Whether to show the connection between the child frames and the parent frames"
                                )
                                ui.Label(
                                    "Show Arrows:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                # show arrows
                                self._ui_show_arrows_checkbox = ui.SimpleBoolModel()
                                ui_utils.SimpleCheckBox(
                                    checked=True,
                                    model=self._ui_show_arrows_checkbox,
                                    on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_arrows_show(
                                        checked
                                    ),
                                )
                                # line color
                                ui.Spacer(width=2)
                                self._ui_show_arrows_color = ui.ColorWidget(
                                    0.0, 1.0, 1.0, 1.0, width=0, tooltip="Line color"
                                ).model
                                for i, child in enumerate(self._ui_show_arrows_color.get_item_children()):
                                    item_model = self._ui_show_arrows_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(
                                        lambda m, i=i: self._viewport_scene.manipulator.set_arrows_color(
                                            i, m.get_value_as_float()
                                        )
                                    )
                                # line thickness
                                ui.Spacer(width=12)
                                self._ui_show_arrows_thickness = ui.FloatDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=0,
                                    max=1,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Line thickness (relative)",
                                ).model
                                self._ui_show_arrows_thickness.add_value_changed_fn(
                                    lambda m: self._viewport_scene.manipulator.set_arrows_thickness(m.as_float)
                                )
                                self._ui_show_arrows_thickness.set_value(0.1)
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish()

                            # update frequency
                            with ui.HStack():
                                tooltip = "Frame transformation update frequency (Hz). Higher frequency may reduce simulation performance"
                                ui.Label(
                                    "Update Frequency:",
                                    width=label_width,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip=tooltip,
                                )
                                self._ui_update_interval = ui.IntDrag(
                                    height=ui_utils.LABEL_HEIGHT,
                                    min=1,
                                    max=60,
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Frequency (Hz)",
                                ).model
                                self._ui_update_interval.add_value_changed_fn(self._on_update_frequency_changed)
                                self._ui_update_interval.set_value(self.update_frequency)
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish()

                            # reset tf
                            with ui.HStack():
                                btn = ui.Button(
                                    "Reset",
                                    width=80,
                                    clicked_fn=self._on_reset,
                                    style=ui_utils.get_style(),
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    tooltip="Reset transformation tree",
                                )
                                ui.Spacer(width=5)
                                ui_utils.add_line_rect_flourish(True)

            # window event
            self._window.set_visibility_changed_fn(self._on_visibility_changed)

    def update(self, frames: set[str]) -> None:
        """Update the UI with the current TF frames."""
        # TODO: update only if different
        frames = sorted(frames)
        root_frame = self._root_frame
        # clean combo boxes
        for item in self._ui_root_frame_combo_box.get_item_children():
            self._ui_root_frame_combo_box.remove_item(item)
        # fill combo boxes
        for frame in frames:
            self._ui_root_frame_combo_box.append_child_item(None, ui.SimpleStringModel(frame))
        # set active frames
        self._ui_root_frame_combo_box.get_item_value_model().set_value(frames.index(root_frame))
        self._root_frame = root_frame

    def shutdown(self) -> None:
        """Clean up menu item."""
        ui.Workspace.set_show_window_fn(self._window_title, None)
        if self._window:
            self._window.destroy()
            self._window = None
