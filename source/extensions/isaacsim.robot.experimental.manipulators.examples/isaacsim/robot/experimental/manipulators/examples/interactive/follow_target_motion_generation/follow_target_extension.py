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

"""Interactive Follow Target example extension: UR10 robot following a target cube with cuMotion RMPflow."""

from __future__ import annotations

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, state_btn_builder
from isaacsim.robot.experimental.manipulators.examples.interactive.follow_target_motion_generation import FollowTarget


class FollowTargetExtension(omni.ext.IExt):
    """Extension for the Follow Target manipulation example.

    Provides an interactive demonstration of a UR10 robot following a target cube using
    cuMotion RMPflow for real-time motion planning with obstacle avoidance.
    """

    def on_startup(self, ext_id: str) -> None:
        """Register the Follow Target example with the examples browser.

        Args:
            ext_id: Extension identifier provided by Kit.
        """
        self.example_name = "Follow Target (cuMotion)"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Follow Target (cuMotion)",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/examples.html",
            "overview": (
                "This Example shows how to follow a target using a UR10 robot with cuMotion RMPflow.\n\n"
                "Click 'Load' to load the scene, and 'Start' to begin following.\n\n"
                "To move the target, select 'TargetCube' in the Stage tree, then drag it around.\n\n"
                "You can add multiple obstacles. 'Removing' them will remove obstacles in reverse order."
            ),
            "sample": FollowTarget(),
        }

        ui_handle = FollowTargetUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Deregister the Follow Target example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class FollowTargetUI(BaseSampleUITemplate):
    """UI for the Follow Target example.

    Provides task control buttons (start/stop following, add/remove obstacles).

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Build the task-control frame for the Follow Target UI."""
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

    def _on_follow_target_button_event(self, val: bool) -> None:
        asyncio.ensure_future(self.sample._on_follow_target_event_async(val))

    def _on_add_obstacle_button_event(self) -> None:
        self.sample._on_add_obstacle_event()
        self.task_ui_elements["Remove Obstacle"].enabled = True

    def _on_remove_obstacle_button_event(self) -> None:
        self.sample._on_remove_obstacle_event()
        if not self.sample.obstacles_exist():
            self.task_ui_elements["Remove Obstacle"].enabled = False

    def post_reset_button_event(self) -> None:
        """Reset task-control button states after reset."""
        self.task_ui_elements["Follow Target"].enabled = True
        self.task_ui_elements["Remove Obstacle"].enabled = False
        self.task_ui_elements["Add Obstacle"].enabled = True
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"

    def post_load_button_event(self) -> None:
        """Enable task controls after scene load."""
        self.task_ui_elements["Follow Target"].enabled = True
        self.task_ui_elements["Add Obstacle"].enabled = True

    def post_clear_button_event(self) -> None:
        """Disable task controls after scene clear."""
        self.task_ui_elements["Follow Target"].enabled = False
        self.task_ui_elements["Remove Obstacle"].enabled = False
        self.task_ui_elements["Add Obstacle"].enabled = False
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"

    def build_task_controls_ui(self) -> None:
        """Build controls for following a target and managing obstacles."""
        with ui.VStack(spacing=5):
            self.task_ui_elements["Follow Target"] = state_btn_builder(
                label="Follow Target",
                type="button",
                a_text="START",
                b_text="STOP",
                tooltip="Follow Target",
                on_clicked_fn=self._on_follow_target_button_event,
            )
            self.task_ui_elements["Follow Target"].enabled = False

            self.task_ui_elements["Add Obstacle"] = btn_builder(
                label="Add Obstacle",
                type="button",
                text="ADD",
                tooltip="Add Obstacle",
                on_clicked_fn=self._on_add_obstacle_button_event,
            )
            self.task_ui_elements["Add Obstacle"].enabled = False

            self.task_ui_elements["Remove Obstacle"] = btn_builder(
                label="Remove Obstacle",
                type="button",
                text="REMOVE",
                tooltip="Remove Obstacle",
                on_clicked_fn=self._on_remove_obstacle_button_event,
            )
            self.task_ui_elements["Remove Obstacle"].enabled = False
