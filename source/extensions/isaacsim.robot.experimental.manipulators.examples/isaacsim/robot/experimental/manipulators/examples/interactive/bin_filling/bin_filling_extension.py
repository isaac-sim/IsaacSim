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

"""Interactive bin filling extension: UR10 robot picking a bin and filling it with cubes in Isaac Sim."""

from __future__ import annotations

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder
from isaacsim.robot.experimental.manipulators.examples.interactive.bin_filling import BinFilling


class BinFillingExtension(omni.ext.IExt):
    """Extension for the Bin Filling interactive example.

    Registers the bin filling example with the examples browser and provides an
    interactive demonstration of a UR10 robot picking up a bin and holding it
    under a pipe while cubes drop into it.
    """

    def on_startup(self, ext_id: str) -> None:
        """Register the bin filling example with the examples browser.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Bin Filling"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Bin Filling",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/examples.html",
            "overview": (
                "This example shows how to do bin filling using a UR10 robot in Isaac Sim.\n\n"
                "The robot picks up a bin with its suction gripper and holds it under a pipe "
                "while cubes drop from above.\n\n"
                "Press the 'Open in IDE' button to view the source code."
            ),
            "sample": BinFilling(),
        }

        ui_handle = BinFillingUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Deregister the bin filling example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class BinFillingUI(BaseSampleUITemplate):
    """User interface for the bin filling example.

    Provides a "Start Bin Filling" button that triggers the pick-and-place state
    machine with cube spawning.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Build the task control UI frame with the start button."""
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}

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

    def _on_fill_bin_button_event(self) -> None:
        """Trigger the asynchronous bin filling operation and disable the start button."""
        asyncio.ensure_future(self.sample.on_fill_bin_event_async())
        self.task_ui_elements["Start Bin Filling"].enabled = False

    def post_reset_button_event(self) -> None:
        """Re-enable the start button after a reset."""
        self.task_ui_elements["Start Bin Filling"].enabled = True

    def post_load_button_event(self) -> None:
        """Re-enable the start button after a load."""
        self.task_ui_elements["Start Bin Filling"].enabled = True

    def post_clear_button_event(self) -> None:
        """Disable the start button after a clear."""
        self.task_ui_elements["Start Bin Filling"].enabled = False

    def build_task_controls_ui(self) -> None:
        """Build the start button for the bin filling task."""
        with ui.VStack(spacing=5):
            self.task_ui_elements["Start Bin Filling"] = btn_builder(
                label="Start Bin Filling",
                type="button",
                text="Start Bin Filling",
                tooltip="Start Bin Filling",
                on_clicked_fn=self._on_fill_bin_button_event,
            )
            self.task_ui_elements["Start Bin Filling"].enabled = False
