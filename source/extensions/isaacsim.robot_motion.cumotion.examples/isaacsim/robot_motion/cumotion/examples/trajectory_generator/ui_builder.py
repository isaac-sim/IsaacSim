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

"""User interface builder for the UR10 trajectory generator example.

Strictly a view: builds widgets, forwards button clicks to the
:class:`UR10TrajectoryGeneratorExample` scenario, and resets widget
state on USD stage changes.  All scene-loading and trajectory state
live on the scenario.
"""

import asyncio
from typing import Any

import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame, StateButton
from isaacsim.gui.components.style import get_style
from omni.kit.async_engine import run_coroutine

from .scenario import UR10TrajectoryGeneratorExample


class UIBuilder:
    """Builds and drives the UR10 trajectory generator UI.

    Owns only widgets and the asyncio task for scene loading.  The
    :class:`UR10TrajectoryGeneratorExample` scenario owns all
    scene/trajectory state.

    The UI has two sections:
      - **World Controls** - Load the scene and reset the scenario.
      - **Run Scenario** - Choose one of three trajectory styles and play.
    """

    def __init__(self) -> None:
        self.frames: list[Any] = []
        self.wrapped_ui_elements: list[Any] = []
        self._timeline = omni.timeline.get_timeline_interface()
        self._load_task: asyncio.Task | None = None
        self._scenario: UR10TrajectoryGeneratorExample | None = UR10TrajectoryGeneratorExample()

    # ------------------------------------------------------------- lifecycle

    def cleanup(self) -> None:
        """Tear down the UI on extension shutdown or window close.

        Cancels any in-flight load task, cleans up wrapped widgets, and
        drops scenario references so the closing UsdStage can be fully
        released (avoids the ``Unexpected reference count of 2`` warning).
        """
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
            self._load_task = None
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()
        self.wrapped_ui_elements.clear()
        if self._scenario is not None:
            self._scenario.cleanup()

    def on_stage_changed(self, event: Any) -> None:
        """Reset widget state in response to a USD stage event.

        Skipped while a load is in flight (the load itself triggers stage
        events; resetting state mid-load would clobber it).
        """
        if self._load_task is not None and not self._load_task.done():
            return
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = UR10TrajectoryGeneratorExample()
        self._reset_widgets()

    def on_timeline_event(self, event: Any) -> None:
        """Reset all trajectory state buttons on timeline state changes."""
        for btn in (self._cspace_trajectory_btn, self._taskspace_trajectory_btn, self._hybrid_trajectory_btn):
            btn.reset()
            btn.enabled = False

    # -------------------------------------------------------------- build UI

    def build_ui(self) -> None:
        """Construct the widget tree."""
        with CollapsableFrame("World Controls", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = Button(
                    "Load Button",
                    "LOAD",
                    tooltip="Load the scene and initialize the scenario",
                    on_click_fn=self._on_load_btn,
                )
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = Button(
                    "Reset Button",
                    "RESET",
                    tooltip="Reset the scenario",
                    on_click_fn=self._on_reset_btn,
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        with CollapsableFrame("Run Scenario", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._cspace_trajectory_btn = StateButton(
                    "Run CSpace Trajectory",
                    "CSPACE TRAJECTORY",
                    "STOP",
                    on_a_click_fn=self._on_run_cspace,
                    on_b_click_fn=self._on_stop,
                    physics_callback_fn=self._update_scenario,
                )
                self._cspace_trajectory_btn.enabled = False

                self._taskspace_trajectory_btn = StateButton(
                    "Run TaskSpace Trajectory",
                    "TASKSPACE TRAJECTORY",
                    "STOP",
                    on_a_click_fn=self._on_run_taskspace,
                    on_b_click_fn=self._on_stop,
                    physics_callback_fn=self._update_scenario,
                )
                self._taskspace_trajectory_btn.enabled = False

                self._hybrid_trajectory_btn = StateButton(
                    "Run Hybrid Trajectory",
                    "HYBRID TRAJECTORY",
                    "STOP",
                    on_a_click_fn=self._on_run_hybrid,
                    on_b_click_fn=self._on_stop,
                    physics_callback_fn=self._update_scenario,
                )
                self._hybrid_trajectory_btn.enabled = False

                self.wrapped_ui_elements.append(self._cspace_trajectory_btn)
                self.wrapped_ui_elements.append(self._taskspace_trajectory_btn)
                self.wrapped_ui_elements.append(self._hybrid_trajectory_btn)

    # ------------------------------------------------------------ handlers

    def _on_load_btn(self) -> None:
        """Handle LOAD click - cancel any prior load, drop the old scenario, kick off a new one."""
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = UR10TrajectoryGeneratorExample()
        self._reset_widgets()
        self._load_task = run_coroutine(self._load_scene_async())

    async def _load_scene_async(self) -> None:
        """Load the scene then enable run/reset controls."""
        await self._scenario.load()
        self._cspace_trajectory_btn.enabled = True
        self._taskspace_trajectory_btn.enabled = True
        self._hybrid_trajectory_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_reset_btn(self) -> None:
        """Handle RESET click - stop the timeline and clear the active trajectory."""
        self._timeline.stop()
        self._scenario.reset()
        for btn in (self._cspace_trajectory_btn, self._taskspace_trajectory_btn, self._hybrid_trajectory_btn):
            btn.reset()
            btn.enabled = True

    def _on_run_cspace(self) -> None:
        """Play the timeline and set up the cspace trajectory."""
        self._timeline.play()
        self._scenario.reset()
        self._taskspace_trajectory_btn.reset()
        self._hybrid_trajectory_btn.reset()
        self._scenario.setup_cspace_trajectory()

    def _on_run_taskspace(self) -> None:
        """Play the timeline and set up the taskspace trajectory."""
        self._timeline.play()
        self._scenario.reset()
        self._cspace_trajectory_btn.reset()
        self._hybrid_trajectory_btn.reset()
        self._scenario.setup_taskspace_trajectory()

    def _on_run_hybrid(self) -> None:
        """Play the timeline and set up the hybrid trajectory."""
        self._timeline.play()
        self._scenario.reset()
        self._cspace_trajectory_btn.reset()
        self._taskspace_trajectory_btn.reset()
        self._scenario.setup_hybrid_trajectory()

    def _on_stop(self) -> None:
        """Pause the timeline when a STOP button is clicked."""
        self._timeline.pause()

    # ------------------------------------------------------------ per-tick

    def _update_scenario(self, step: float, *args: Any, **kwargs: Any) -> None:
        """Per-physics-step callback wired into the StateButtons."""
        if self._scenario is not None:
            self._scenario.step(step)

    # ------------------------------------------------------------- internals

    def _reset_widgets(self) -> None:
        """Disable trajectory/run/reset controls."""
        for btn in (self._cspace_trajectory_btn, self._taskspace_trajectory_btn, self._hybrid_trajectory_btn):
            btn.reset()
            btn.enabled = False
        self._reset_btn.enabled = False
