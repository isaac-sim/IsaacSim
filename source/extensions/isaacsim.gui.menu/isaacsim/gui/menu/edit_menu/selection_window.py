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

"""Selection set naming UI for the Edit menu."""

__all__ = ["SelectionSetWindow"]

from collections.abc import Callable

import omni.ui as ui


class SelectionSetWindow:
    """Collect a selection set name and invoke a callback.

    Args:
        callback: Function to call with the new selection set name.
    """

    def __init__(self, callback: Callable[[str], None]) -> None:
        self._callback: Callable[[str], None] | None = callback
        window = ui.Window(
            "Selection Set Name",
            width=300,
            height=110,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_MODAL,
        )

        with window.frame:

            def on_create(widget: ui.StringField) -> None:
                """Handle creation of a new selection set.

                Args:
                    widget: String field containing the set name.
                """
                if self._callback is not None:
                    self._callback(widget.model.as_string)
                window.visible = False

            def on_cancel() -> None:
                """Dismiss the selection set window."""
                window.visible = False

            with ui.VStack(
                height=0,
                spacing=5,
                name="top_level_stack",
                style={"VStack::top_level_stack": {"margin": 5}, "Button": {"margin": 0}},
            ):
                ui.Label("New selection set name:")
                widget = ui.StringField()
                ui.Spacer(width=5, height=5)
                with ui.HStack(spacing=5):
                    create_button = ui.Button("Create", enabled=False, clicked_fn=lambda w=widget: on_create(w))
                    ui.Button("Cancel", clicked_fn=on_cancel)

                def window_pressed_key(key_index: int, key_flags: int, key_down: bool) -> None:
                    """Handle key presses for the dialog.

                    Args:
                        key_index: Keyboard key code.
                        key_flags: Modifier flags.
                        key_down: Whether the key was pressed.
                    """
                    import carb.input

                    create_button.enabled = len(widget.model.as_string) > 0
                    key_mod = key_flags & ~ui.Widget.FLAG_WANT_CAPTURE_KEYBOARD
                    if (
                        create_button.enabled
                        and carb.input.KeyboardInput(key_index)
                        in [carb.input.KeyboardInput.ENTER, carb.input.KeyboardInput.NUMPAD_ENTER]
                        and key_mod == 0
                        and key_down
                    ):
                        on_create(widget)

                widget.focus_keyboard()
                window.set_key_pressed_fn(window_pressed_key)

        self._window: ui.Window = window
        self._widget: ui.StringField = widget

    def shutdown(self) -> None:
        """Release UI resources for the selection set dialog.

        Example:
            .. code-block:: python

                window = SelectionSetWindow(lambda name: None)
                window.shutdown()
        """
        self._callback = None
        del self._window

    def show(self) -> None:
        """Show the dialog and reset the input field.

        Example:
            .. code-block:: python

                window = SelectionSetWindow(lambda name: None)
                window.show()
        """
        self._window.visible = True
        self._widget.model.set_value("")
        self._widget.focus_keyboard()
