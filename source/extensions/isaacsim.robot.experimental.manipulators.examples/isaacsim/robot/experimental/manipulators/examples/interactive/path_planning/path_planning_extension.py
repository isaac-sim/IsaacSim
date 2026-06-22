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

"""Path planning demonstration extension for interactive robot manipulation examples in Isaac Sim."""

from __future__ import annotations

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder
from isaacsim.robot.experimental.manipulators.examples.interactive.path_planning import PathPlanning


class PathPlanningExtension(omni.ext.IExt):
    """Extension that provides an interactive path planning demonstration for robot manipulation.

    This extension registers a path planning example in the Isaac Sim examples browser, showing
    how to plan collision-free paths using a Franka robot with cuMotion's graph-based motion
    planner (RRT variants). Users can add and remove wall obstacles dynamically to create
    challenging planning scenarios.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the Path Planning extension and register with the examples browser.

        Args:
            ext_id: The extension identifier string.
        """
        self.example_name = "Path Planning"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Path Planning Task",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/advanced_robot_motion.html",
            "overview": (
                "This Example shows how to plan a collision-free path through an environment with "
                "the Franka robot using cuMotion's graph-based motion planner.\n\n"
                "Press the 'Open in IDE' button to view the source code."
            ),
            "sample": PathPlanning(),
        }

        ui_handle = PathPlanningUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Deregister the example from the browser on shutdown."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class PathPlanningUI(BaseSampleUITemplate):
    """User interface for the Path Planning example.

    Provides task control buttons for planning to target, adding walls, and removing walls.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Build the Task Control frame with plan/wall buttons."""
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

    def _on_plan_to_target_button_event(self) -> None:
        """Initiate asynchronous path planning and execution to the target position."""
        asyncio.ensure_future(self.sample._on_plan_to_target_event_async())

    def _on_add_wall_button_event(self) -> None:
        """Add a wall obstacle, then disable Add and enable Remove."""
        self.sample._on_add_wall_event()
        self.task_ui_elements["Add Wall"].enabled = False
        self.task_ui_elements["Remove Wall"].enabled = True

    def _on_remove_wall_button_event(self) -> None:
        """Remove the wall, then disable Remove and enable Add."""
        self.sample._on_remove_wall_event()
        self.task_ui_elements["Remove Wall"].enabled = False
        self.task_ui_elements["Add Wall"].enabled = True

    def post_reset_button_event(self) -> None:
        """Re-enable task controls after reset."""
        self.task_ui_elements["Plan To Target"].enabled = True
        self.task_ui_elements["Remove Wall"].enabled = False
        self.task_ui_elements["Add Wall"].enabled = True

    def post_load_button_event(self) -> None:
        """Enable task controls after scene load."""
        self.task_ui_elements["Plan To Target"].enabled = True
        self.task_ui_elements["Add Wall"].enabled = True

    def post_clear_button_event(self) -> None:
        """Disable all task controls after scene clear."""
        self.task_ui_elements["Plan To Target"].enabled = False
        self.task_ui_elements["Remove Wall"].enabled = False
        self.task_ui_elements["Add Wall"].enabled = False

    def build_task_controls_ui(self) -> None:
        """Build buttons for planning, adding walls, and removing walls."""
        with ui.VStack(spacing=5):
            self.task_ui_elements["Plan To Target"] = btn_builder(
                label="Plan To Target",
                type="button",
                text="Plan To Target",
                tooltip="Plan a collision-free path and move to target",
                on_clicked_fn=self._on_plan_to_target_button_event,
            )
            self.task_ui_elements["Plan To Target"].enabled = False

            self.task_ui_elements["Add Wall"] = btn_builder(
                label="Add Wall",
                type="button",
                text="ADD",
                tooltip="Add a wall obstacle",
                on_clicked_fn=self._on_add_wall_button_event,
            )
            self.task_ui_elements["Add Wall"].enabled = False

            self.task_ui_elements["Remove Wall"] = btn_builder(
                label="Remove Wall",
                type="button",
                text="REMOVE",
                tooltip="Remove the last added wall",
                on_clicked_fn=self._on_remove_wall_button_event,
            )
            self.task_ui_elements["Remove Wall"].enabled = False
