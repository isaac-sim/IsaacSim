# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension module for the RoboFactory interactive example that provides UI components and extension lifecycle management."""

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder
from isaacsim.robot.experimental.manipulators.examples.interactive.robo_factory import RoboFactory


class RoboFactoryExtension(omni.ext.IExt):
    """Extension for RoboFactory interactive example."""

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is starting up.

        Initializes the RoboFactory extension by setting up the UI template and registering
        the example with the browser.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "RoboFactory"
        self.category = "Multi-Robot"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "RoboFactory",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_multiple_robots.html",
            "overview": "This Example shows how to run multiple tasks in the same scene.\n\nPress 'LOAD' to load the scene, \npress 'START STACKING' to start stacking.\n\nPress the 'Open in IDE' button to view the source code.",
            "sample": RoboFactory(),
        }

        ui_handle = RoboFactoryUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Called when the extension is shutting down.

        Cleans up by deregistering the RoboFactory example from the browser.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class RoboFactoryUI(BaseSampleUITemplate):
    """UI for the RoboFactory interactive example.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.task_ui_elements = {}

    def build_extra_frames(self) -> None:
        """Build additional UI frames for task control."""
        extra_stacks = self.get_extra_frames_handle()

        with extra_stacks:
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

    def _on_start_stacking_button_event(self) -> None:
        """Handle start stacking button click."""
        asyncio.ensure_future(self.sample._on_start_stacking_event_async())
        self.task_ui_elements["Start Stacking"].enabled = False

    def post_reset_button_event(self) -> None:
        """Called after the reset button is pressed."""
        self.task_ui_elements["Start Stacking"].enabled = True

    def post_load_button_event(self) -> None:
        """Called after the load button is pressed."""
        self.task_ui_elements["Start Stacking"].enabled = True

    def post_clear_button_event(self) -> None:
        """Called after the clear button is pressed."""
        self.task_ui_elements["Start Stacking"].enabled = False

    def build_task_controls_ui(self) -> None:
        """Build the task control UI elements."""
        with ui.VStack(spacing=5):
            start_stacking_dict = {
                "label": "Start Stacking",
                "type": "button",
                "text": "Start Stacking",
                "tooltip": "Start Stacking",
                "on_clicked_fn": self._on_start_stacking_button_event,
            }

            self.task_ui_elements["Start Stacking"] = btn_builder(**start_stacking_dict)
            self.task_ui_elements["Start Stacking"].enabled = False
