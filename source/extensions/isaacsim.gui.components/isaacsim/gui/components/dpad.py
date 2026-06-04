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

"""Directional pad (D-Pad) UI widget for controlling movement in Isaac Sim."""

from collections.abc import Callable
from pathlib import Path

import omni.appwindow
import omni.ext
import omni.kit
import omni.ui as ui
import omni.usd

ICON_FOLDER_PATH = Path(f"{omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)}/data")


class Dpad:
    """D-Pad Controller Class.

    Args:
        name: Window Title. Defaults to "D-Pad Controller".
        clicked_fn_up: Sets the function that will be called when when the button is activated
            (i.e., pressed down then released while the mouse cursor is inside the button). Defaults to None.
        mouse_pressed_fn_up: Sets the function that will be called when the user presses the
            mouse button inside the widget. The function should be like this:
            void onMousePressed(float x, float y, int32_t button, carb::input::KeyboardModifierFlags modifier)
            Where 'button' is the number of the mouse button pressed. 'modifier' is the flag for the keyboard modifier key.
            Defaults to None.
        mouse_released_fn_up: Sets the function that will be called when the user releases the
            mouse button if this button was pressed inside the widget. The function should be like this:
            void onMouseReleased(float x, float y, int32_t button, carb::input::KeyboardModifierFlags modifier)
            Defaults to None.
        clicked_fn_down: On Click Function. Defaults to None.
        mouse_pressed_fn_down: On Mouse Pressed Function. Defaults to None.
        mouse_released_fn_down: On Mouse Released Function. Defaults to None.
        clicked_fn_left: On Click Function. Defaults to None.
        mouse_pressed_fn_left: On Mouse Pressed Function. Defaults to None.
        mouse_released_fn_left: On Mouse Released Function. Defaults to None.
        clicked_fn_right: On Click Function. Defaults to None.
        mouse_pressed_fn_right: On Mouse Pressed Function. Defaults to None.
        mouse_released_fn_right: On Mouse Released Function. Defaults to None.
        clicked_fn_cw: On Click Function. Defaults to None.
        mouse_pressed_fn_cw: On Mouse Pressed Function. Defaults to None.
        mouse_released_fn_cw: On Mouse Released Function. Defaults to None.
        clicked_fn_ccw: On Click Function. Defaults to None.
        mouse_pressed_fn_ccw: On Mouse Pressed Function. Defaults to None.
        mouse_released_fn_ccw: On Mouse Released Function. Defaults to None.
        width: Window Width. Defaults to 210.
        height: Window Height. Defaults to 210.
    """

    def __init__(
        self,
        name: str = "D-Pad Controller",
        clicked_fn_up: Callable = None,
        mouse_pressed_fn_up: Callable = None,
        mouse_released_fn_up: Callable = None,
        clicked_fn_down: Callable = None,
        mouse_pressed_fn_down: Callable = None,
        mouse_released_fn_down: Callable = None,
        clicked_fn_left: Callable = None,
        mouse_pressed_fn_left: Callable = None,
        mouse_released_fn_left: Callable = None,
        clicked_fn_right: Callable = None,
        mouse_pressed_fn_right: Callable = None,
        mouse_released_fn_right: Callable = None,
        clicked_fn_cw: Callable = None,
        mouse_pressed_fn_cw: Callable = None,
        mouse_released_fn_cw: Callable = None,
        clicked_fn_ccw: Callable = None,
        mouse_pressed_fn_ccw: Callable = None,
        mouse_released_fn_ccw: Callable = None,
        width: int = 210,
        height: int = 210,
    ) -> None:
        self.name = name

        self.clicked_fn_up = clicked_fn_up
        self.mouse_pressed_fn_up = mouse_pressed_fn_up
        self.mouse_released_fn_up = mouse_released_fn_up

        self.clicked_fn_down = clicked_fn_down
        self.mouse_pressed_fn_down = mouse_pressed_fn_down
        self.mouse_released_fn_down = mouse_released_fn_down

        self.clicked_fn_left = clicked_fn_left
        self.mouse_pressed_fn_left = mouse_pressed_fn_left
        self.mouse_released_fn_left = mouse_released_fn_left

        self.clicked_fn_right = clicked_fn_right
        self.mouse_pressed_fn_right = mouse_pressed_fn_right
        self.mouse_released_fn_right = mouse_released_fn_right

        self.clicked_fn_cw = clicked_fn_cw
        self.mouse_pressed_fn_cw = mouse_pressed_fn_cw
        self.mouse_released_fn_cw = mouse_released_fn_cw

        self.clicked_fn_ccw = clicked_fn_ccw
        self.mouse_pressed_fn_ccw = mouse_pressed_fn_ccw
        self.mouse_released_fn_ccw = mouse_released_fn_ccw

        self.width = width
        self.height = height
        self._build_ui()
        return

    def _build_ui(self) -> None:
        self._window = ui.Window(
            title=self.name,
            width=self.width,
            height=self.height,
            visible=True,
            dockPreference=ui.DockPreference.LEFT,
            auto_resize=True,
        )
        with self._window.frame:
            padding = 6
            h_spacing = self.width / 3.0 - padding
            with ui.VStack(spacing=0, height=self.height, width=self.width):
                with ui.HStack():
                    if self.clicked_fn_ccw or self.mouse_pressed_fn_ccw or self.mouse_released_fn_ccw:
                        ui.Button(
                            clicked_fn=self.clicked_fn_ccw,
                            mouse_pressed_fn=self.mouse_pressed_fn_ccw,
                            mouse_released_fn=self.mouse_released_fn_ccw,
                            image_width=h_spacing,
                            image_height=h_spacing,
                            image_url=f"{ICON_FOLDER_PATH}/arrow_counterclockwise.svg",
                            alignment=ui.Alignment.LEFT_CENTER,
                        )
                    else:
                        ui.Spacer()
                    ui.Button(
                        clicked_fn=self.clicked_fn_up,
                        mouse_pressed_fn=self.mouse_pressed_fn_up,
                        mouse_released_fn=self.mouse_released_fn_up,
                        image_width=h_spacing,
                        image_height=h_spacing,
                        image_url=f"{ICON_FOLDER_PATH}/arrow_up.svg",
                        alignment=ui.Alignment.CENTER,
                    )
                    if self.clicked_fn_cw or self.mouse_pressed_fn_cw or self.mouse_released_fn_cw:
                        ui.Button(
                            clicked_fn=self.clicked_fn_cw,
                            mouse_pressed_fn=self.mouse_pressed_fn_cw,
                            mouse_released_fn=self.mouse_released_fn_cw,
                            image_width=h_spacing,
                            image_height=h_spacing,
                            image_url=f"{ICON_FOLDER_PATH}/arrow_clockwise.svg",
                            alignment=ui.Alignment.RIGHT_CENTER,
                        )
                    else:
                        ui.Spacer()
                with ui.HStack(spacing=0):
                    ui.Button(
                        clicked_fn=self.clicked_fn_left,
                        mouse_pressed_fn=self.mouse_pressed_fn_left,
                        mouse_released_fn=self.mouse_released_fn_left,
                        image_width=h_spacing,
                        image_height=h_spacing,
                        image_url=f"{ICON_FOLDER_PATH}/arrow_left.svg",
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer()
                    ui.Button(
                        clicked_fn=self.clicked_fn_right,
                        mouse_pressed_fn=self.mouse_pressed_fn_right,
                        mouse_released_fn=self.mouse_released_fn_right,
                        image_width=h_spacing,
                        image_height=h_spacing,
                        image_url=f"{ICON_FOLDER_PATH}/arrow_right.svg",
                        alignment=ui.Alignment.RIGHT_CENTER,
                    )
                with ui.HStack():
                    ui.Spacer()
                    ui.Button(
                        clicked_fn=self.clicked_fn_down,
                        mouse_pressed_fn=self.mouse_pressed_fn_down,
                        mouse_released_fn=self.mouse_released_fn_down,
                        image_width=h_spacing,
                        image_height=h_spacing,
                        image_url=f"{ICON_FOLDER_PATH}/arrow_down.svg",
                        alignment=ui.Alignment.CENTER_BOTTOM,
                    )
                    ui.Spacer()
        return

    def shutdown(self) -> None:
        """Clean up the D-Pad window and release resources."""
        self._window = None
        self.name = None
        self.width = None
        self.height = None
        return
