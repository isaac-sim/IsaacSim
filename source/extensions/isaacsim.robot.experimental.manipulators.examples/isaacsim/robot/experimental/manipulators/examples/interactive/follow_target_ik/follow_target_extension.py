# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UR10 follow target interactive example extension."""

from __future__ import annotations

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import (
    btn_builder,
    dropdown_builder,
    get_style,
    setup_ui_headers,
    state_btn_builder,
)
from isaacsim.robot.experimental.manipulators.examples.interactive.follow_target_ik.follow_target import (
    UR10FollowTargetInteractive,
)


class UR10FollowTargetExtension(omni.ext.IExt):
    """Extension for UR10 follow target interactive example."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the UR10 follow target extension.

        Sets up the extension properties, creates the UI handle, and registers the example
        with the browser instance for display in the examples collection.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Follow Target (IK)"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Follow Target (IK)",
            "overview": "This sample demonstrates a UR10 robot following a target cube using inverse kinematics.",
            "sample": UR10FollowTargetInteractive(),
        }

        ui_handle = UR10FollowTargetUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Clean up the extension on shutdown.

        Deregisters the example from the browser instance to remove it from the examples
        collection.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class UR10FollowTargetUI(BaseSampleUITemplate):
    """UI for the UR10 follow target interactive example.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    # Class constant for IK methods to avoid duplication
    IK_METHODS = ["damped-least-squares", "pseudoinverse", "transpose", "singular-value-decomposition"]
    """Available inverse kinematics methods for the UR10 robot arm."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.task_ui_elements = {}

    def build_ui(self) -> None:
        """Override to control the exact order of UI elements."""
        self._main_stack = ui.VStack(spacing=5, height=0)
        with self._main_stack:
            # 1. Overview (from base template)
            setup_ui_headers(
                self._ext_id, self._file_path, self._title, self._doc_link, self._overview, info_collapsed=False
            )

            # 2. Instructions (right after Overview)
            with ui.CollapsableFrame(
                title="Instructions",
                width=ui.Fraction(1),
                height=0,
                visible=True,
                collapsed=True,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_instructions_ui()

            # 3. World Controls (Load/Reset buttons)
            self._controls_frame = ui.CollapsableFrame(
                title="World Controls",
                width=ui.Fraction(1),
                height=0,
                collapsed=False,
                style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )

            # 4. Extra frames container
            self._extra_stacks = ui.VStack(margin=5, spacing=5, height=0)

        # Build World Controls content
        with self._controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                dict = {
                    "label": "Load World",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Load World and Task",
                    "on_clicked_fn": self._on_load_world,
                }
                self._buttons["Load World"] = btn_builder(**dict)
                self._buttons["Load World"].enabled = True
                dict = {
                    "label": "Reset",
                    "type": "button",
                    "text": "Reset",
                    "tooltip": "Reset robot and environment",
                    "on_clicked_fn": self._on_reset,
                }
                self._buttons["Reset"] = btn_builder(**dict)
                self._buttons["Reset"].enabled = False

        # Build extra frames (Task Control and Robot Status)
        self.build_extra_frames()

    def build_extra_frames(self) -> None:
        """Build additional UI frames for task control."""
        extra_stacks = self.get_extra_frames_handle()

        with extra_stacks:
            # Task Control Frame
            with ui.CollapsableFrame(
                title="Task Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_task_controls_ui()

            # Robot Status Frame
            with ui.CollapsableFrame(
                title="Robot Status",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_status_ui()

    def build_instructions_ui(self) -> None:
        """Build the instructions UI elements."""
        with ui.VStack(spacing=5):
            with ui.VStack(spacing=3):
                ui.Label("How to use:", word_wrap=True, style={"font_size": 14, "color": 0xFFFFFFFF})
                ui.Separator(height=3)

                ui.Label("Getting Started:", word_wrap=True, style={"font_size": 12, "color": 0xFFAAAAAA})
                ui.Label("1. Click 'LOAD' to load the scene", word_wrap=True)
                ui.Label("2. Click 'START' to begin target following", word_wrap=True)

                ui.Separator(height=5)
                ui.Label("Controlling the Robot:", word_wrap=True, style={"font_size": 12, "color": 0xFFAAAAAA})
                ui.Label("3. Move the red target cube in viewport to control robot", word_wrap=True)
                ui.Label("4. Watch the UR10 robot follow the target using IK", word_wrap=True)
                ui.Label("5. Click 'STOP' to pause target following", word_wrap=True)
                ui.Label("6. Click 'START' again to resume following", word_wrap=True)

                ui.Separator(height=5)
                ui.Label("More Features:", word_wrap=True, style={"font_size": 12, "color": 0xFFAAAAAA})
                ui.Label("7. Try different IK methods from dropdown", word_wrap=True)
                ui.Label("8. Monitor robot status in 'Robot Status' panel", word_wrap=True)
                ui.Label("9. Use 'UPDATE' to refresh position data", word_wrap=True)
                ui.Label("10. Use 'RESET' to return to initial state", word_wrap=True)

    def build_task_controls_ui(self) -> None:
        """Build the task control UI elements."""
        with ui.VStack(spacing=5):
            # Start/Stop Follow Target Button
            start_stop_dict = {
                "label": "Follow Target",
                "type": "button",
                "a_text": "START",
                "b_text": "STOP",
                "tooltip": "Start or stop following the target cube",
                "on_clicked_fn": self._on_follow_target_button_event,
            }
            self.task_ui_elements["Follow Target"] = state_btn_builder(**start_stop_dict)
            self.task_ui_elements["Follow Target"].enabled = False

            # IK Method Selection
            ui.Separator(height=5)
            with ui.HStack():
                ui.Label("IK Method:", width=80)
                ik_dropdown_dict = {
                    "label": "IK Method",
                    "type": "dropdown",
                    "default_val": 0,
                    "items": self.IK_METHODS,
                    "tooltip": "Select inverse kinematics method",
                    "on_clicked_fn": self._on_ik_method_change,
                }
                self.task_ui_elements["IK Method"] = dropdown_builder(**ik_dropdown_dict)

            # Status display
            ui.Separator(height=5)
            with ui.HStack():
                ui.Label("Status:", width=60)
                self.task_ui_elements["Status"] = ui.Label("Ready", width=200)

    def build_status_ui(self) -> None:
        """Build the status display UI elements."""
        with ui.VStack(spacing=3):
            # Target Position
            with ui.HStack():
                ui.Label("Target Pos:", width=80)
                self.task_ui_elements["Target Position"] = ui.Label("N/A", width=150)

            # End Effector Position
            with ui.HStack():
                ui.Label("EE Pos:", width=80)
                self.task_ui_elements["EE Position"] = ui.Label("N/A", width=150)

            # Distance to Target
            with ui.HStack():
                ui.Label("Distance:", width=80)
                self.task_ui_elements["Distance"] = ui.Label("N/A", width=100)

            # Target Reached Status
            with ui.HStack():
                ui.Label("Target Reached:", width=80)
                self.task_ui_elements["Target Reached"] = ui.Label("N/A", width=50)

            # Update Status Button
            update_dict = {
                "label": "Update Status",
                "type": "button",
                "text": "Update",
                "tooltip": "Refresh robot status display",
                "on_clicked_fn": self._on_update_status,
            }
            self.task_ui_elements["Update Status"] = btn_builder(**update_dict)

    def _on_follow_target_button_event(self, val: bool) -> None:
        """Handle follow target button toggle event.

        Args:
            val: True if START pressed, False if STOP pressed.
        """
        if val:  # START pressed
            asyncio.ensure_future(self.sample.start_following_async())
            self._update_status("Following target...")
        else:  # STOP pressed
            asyncio.ensure_future(self.sample.stop_following_async())
            self._update_status("Stopped")

    def _on_ik_method_change(self, selection: str | int) -> None:
        """Handle IK method selection change.

        Args:
            selection: The selected IK method name or index.
        """
        # Handle both string (method name) and integer (index) selection
        if isinstance(selection, str):
            selected_method = selection
        else:
            selected_method = self.IK_METHODS[selection]

        self.sample.set_ik_method(selected_method)
        print(f"IK method changed to: {selected_method}")

    def _on_update_status(self) -> None:
        """Handle update status button click."""
        self._refresh_status_display()

    def _refresh_status_display(self) -> None:
        """Refresh the status display with current robot information."""
        try:
            status = self.sample.get_controller_status()

            if "error" not in status:
                # Update position displays
                target_pos = status["target_position"]
                ee_pos = status["end_effector_position"]

                self.task_ui_elements["Target Position"].text = (
                    f"[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]"
                )
                self.task_ui_elements["EE Position"].text = f"[{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]"
                self.task_ui_elements["Distance"].text = f"{status['distance_to_target']:.3f}m"
                self.task_ui_elements["Target Reached"].text = "Yes" if status["target_reached"] else "No"
            else:
                self.task_ui_elements["Target Position"].text = "Error"
                self.task_ui_elements["EE Position"].text = "Error"
                self.task_ui_elements["Distance"].text = "N/A"
                self.task_ui_elements["Target Reached"].text = "N/A"

        except Exception as e:
            print(f"Error updating status: {e}")

    def _update_status(self, message: str) -> None:
        """Update the status label.

        Args:
            message: The status message to display.
        """
        if "Status" in self.task_ui_elements:
            self.task_ui_elements["Status"].text = message
            print(f"Status: {message}")

    def _set_task_buttons_enabled(self, enabled: bool) -> None:
        """Enable/disable task control buttons.

        Args:
            enabled: Whether to enable the task control buttons.
        """
        buttons_to_control = ["Follow Target", "Update Status"]
        for button_name in buttons_to_control:
            if button_name in self.task_ui_elements:
                self.task_ui_elements[button_name].enabled = enabled

    def post_reset_button_event(self) -> None:
        """Called after the reset button is pressed."""
        self._set_task_buttons_enabled(True)
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"
        self._update_status("Ready")

    def post_load_button_event(self) -> None:
        """Called after the load button is pressed."""
        self._set_task_buttons_enabled(True)
        self._update_status("Scene loaded - ready to start")

    def post_clear_button_event(self) -> None:
        """Called after the clear button is pressed."""
        self._set_task_buttons_enabled(False)
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"
        self._update_status("Scene cleared")
