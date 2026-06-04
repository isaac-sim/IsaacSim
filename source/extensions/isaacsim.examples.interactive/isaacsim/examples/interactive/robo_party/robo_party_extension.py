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

"""Extension for running the RoboParty interactive multi-robot example with UI controls and browser integration."""

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.robo_party import RoboParty
from isaacsim.gui.components.ui_utils import btn_builder


class RoboPartyExtension(omni.ext.IExt):
    """Extension for the RoboParty interactive example.

    This extension demonstrates how to run multiple robots in the same scene by registering the RoboParty sample with
    the Isaac Sim examples browser. It creates a UI interface that allows users to load a scene with multiple robots
    and start coordinated robot movements through a "Start Party" button.

    The extension integrates with the examples browser system to provide an interactive multi-robot demonstration
    under the "Multi-Robot" category.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the RoboParty extension.

        Sets up the extension by configuring UI parameters, creating the RoboPartyUI instance,
        and registering the example with the browser instance.

        Args:
            ext_id: The extension ID.
        """
        self.example_name = "RoboParty"
        self.category = "Multi-Robot"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "RoboParty",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_multiple_robots.html",
            "overview": "This Example shows how to run multiple tasks in the same scene.\n\nPress 'LOAD' to load the scene, \npress 'START PARTY' to start moving the robots ",
            "sample": RoboParty(),
        }

        ui_handle = RoboPartyUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Clean up the RoboParty extension.

        Deregisters the example from the browser instance.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class RoboPartyUI(BaseSampleUITemplate):
    """A user interface class for the RoboParty interactive example.

    This class provides a specialized UI for the RoboParty example, which demonstrates how to run multiple tasks in the same scene with multiple robots. It extends the BaseSampleUITemplate to include task control functionality specific to managing robot party scenarios.

    The UI includes:
    - Standard sample controls inherited from the base template (load, reset, clear)
    - A collapsible "Task Control" frame with party-specific controls
    - A "Start Party" button that initiates robot movement across multiple tasks
    - Automatic button state management based on sample lifecycle events

    The interface handles the coordination between UI state and the underlying RoboParty sample, ensuring that controls are enabled/disabled appropriately during different phases of the example execution.

    Args:
        *args: Variable length argument list passed to the parent BaseSampleUITemplate.
        **kwargs: Additional keyword arguments passed to the parent BaseSampleUITemplate.
            Common kwargs include ext_id, file_path, title, doc_link, overview, and sample.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Builds the additional UI frames for the RoboParty example.

        Creates a collapsible task control frame containing the party control elements.
        """
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Task Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                # style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_task_controls_ui()

    def _on_start_party_button_event(self) -> None:
        """Handles the Start Party button click event.

        Starts the robot party asynchronously and disables the Start Party button.
        """
        asyncio.ensure_future(self.sample._on_start_party_event_async())
        self.task_ui_elements["Start Party"].enabled = False
        return

    def post_reset_button_event(self) -> None:
        """Handles post-reset operations for the UI.

        Re-enables the Start Party button after a reset operation.
        """
        self.task_ui_elements["Start Party"].enabled = True
        return

    def post_load_button_event(self) -> None:
        """Handles post-load operations for the UI.

        Enables the Start Party button after the scene has been loaded.
        """
        self.task_ui_elements["Start Party"].enabled = True
        return

    def post_clear_button_event(self) -> None:
        """Handles post-clear operations for the UI.

        Disables the Start Party button after the scene has been cleared.
        """
        self.task_ui_elements["Start Party"].enabled = False
        return

    def build_task_controls_ui(self) -> None:
        """Builds the task control UI elements.

        Creates the Start Party button and configures its initial state as disabled.
        """
        with ui.VStack(spacing=5):

            dict = {
                "label": "Start Party",
                "type": "button",
                "text": "Start Party",
                "tooltip": "Start Party",
                "on_clicked_fn": self._on_start_party_button_event,
            }

            self.task_ui_elements["Start Party"] = btn_builder(**dict)
            self.task_ui_elements["Start Party"].enabled = False
