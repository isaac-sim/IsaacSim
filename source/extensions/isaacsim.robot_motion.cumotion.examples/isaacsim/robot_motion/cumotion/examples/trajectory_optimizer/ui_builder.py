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

"""User interface builder for the cuMotion trajectory optimizer example.

Strictly a view: builds widgets, forwards button clicks to the
:class:`FrankaTrajectoryOptimizerExample` scenario, and resets widget
state on USD stage changes.  All scene-loading, planning, and trajectory
state live on the scenario.
"""

import asyncio
from collections.abc import Callable
from typing import Any

import omni.kit.app
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame, StateButton
from isaacsim.gui.components.style import get_style
from omni.kit.async_engine import run_coroutine

from .scenario import FrankaTrajectoryOptimizerExample


class UIBuilder:
    """Builds and drives the cuMotion trajectory optimizer UI.

    Owns only widgets and the asyncio task for scene loading.  The
    :class:`FrankaTrajectoryOptimizerExample` scenario owns all
    scene/planner state.

    The UI has two sections:
      - **World Controls** - Load the scene, set joint targets, and
        plan to either a configuration-space or task-space target.
      - **Run Scenario** - Execute the planned trajectory.
    """

    def __init__(self) -> None:
        self.frames: list[Any] = []
        self.wrapped_ui_elements: list[Any] = []
        self._timeline = omni.timeline.get_timeline_interface()
        self._joint_slider_models: list[ui.AbstractValueModel] = []
        self._load_task: asyncio.Task | None = None
        self._scenario: FrankaTrajectoryOptimizerExample | None = FrankaTrajectoryOptimizerExample()

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
        events; resetting state mid-load would clobber it).  When the
        stage changes outside of a load - e.g. the user opens a different
        stage in the viewport - this drops the now-stale scenario state
        so the next LOAD click starts cleanly.
        """
        if self._load_task is not None and not self._load_task.done():
            return
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = FrankaTrajectoryOptimizerExample()
        self._reset_widgets()

    def on_timeline_event(self, event: Any) -> None:
        """Reset the Run/Stop button on timeline state changes."""
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False

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

                # Joint angle sliders for the C-space target are added in _create_joint_sliders
                # after the robot config has loaded.
                with CollapsableFrame("Configuration Target", collapsed=False):
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._joint_sliders_container = ui.VStack(style=get_style(), spacing=3, height=0)

                self._to_cspace_btn = Button(
                    "Plan to C-Space",
                    "TO CSPACE TARGET",
                    tooltip="Plan a trajectory to the joint configuration above",
                    on_click_fn=self._on_to_cspace_target_btn,
                )
                self._to_cspace_btn.enabled = False
                self.wrapped_ui_elements.append(self._to_cspace_btn)

                self._to_task_space_btn = Button(
                    "Plan to Task-Space",
                    "TO TASK-SPACE TARGET",
                    tooltip="Plan a trajectory to the target cube's world pose",
                    on_click_fn=self._on_to_task_space_target_btn,
                )
                self._to_task_space_btn.enabled = False
                self.wrapped_ui_elements.append(self._to_task_space_btn)

        with CollapsableFrame("Run Scenario"):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

    # ---------------------------------------------------------- LOAD handler

    def _on_load_btn(self) -> None:
        """Handle LOAD click.

        Cancels any previous in-flight load, drops the old scenario's prim
        references, and schedules a fresh two-phase load on the Kit asyncio
        loop.
        """
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = FrankaTrajectoryOptimizerExample()
        self._reset_widgets()
        self._load_task = run_coroutine(self._load_scene_async())

    async def _load_scene_async(self) -> None:
        """Two-phase scene load.

        Phase 1 (fast, local): create the stage and load the cuMotion
        robot config.  Returns as soon as the joint sliders can be built,
        so the UI is responsive even while the heavy USD load runs.

        Phase 2 (slow, network): pull the Franka USD, create target +
        obstacle, init physics.
        """
        await self._scenario.load_robot_config()
        self._create_joint_sliders()
        self._enable_planning_buttons()
        await self._scenario.load_assets()

    # ------------------------------------------------------- planning buttons

    def _on_to_cspace_target_btn(self) -> None:
        """Handle 'Plan to C-Space' click - read sliders, ask the scenario to plan."""
        self._timeline.pause()
        q_target = [m.get_value_as_float() for m in self._joint_slider_models]
        self._run_plan(lambda: self._scenario.plan_to_cspace_target(q_target=q_target))

    def _on_to_task_space_target_btn(self) -> None:
        """Handle 'Plan to Task-Space' click - ask the scenario to plan to the target cube's pose."""
        self._timeline.pause()
        self._run_plan(self._scenario.plan_to_task_space_target)

    def _run_plan(self, plan_fn: Callable[[], str | None]) -> None:
        """Run a planning function and update the Run button based on the result."""
        error_message = plan_fn()
        self._scenario_state_btn.reset()
        if error_message is not None:
            self._show_error_dialog(error_message)
            self._scenario_state_btn.enabled = False
        else:
            self._scenario_state_btn.enabled = True

    # ------------------------------------------------------------ per-tick

    def _update_scenario(self, step: float, *args: Any, **kwargs: Any) -> None:
        """Per-physics-step callback wired into the StateButton."""
        if self._scenario is not None:
            self._scenario.step(step)

    def _on_run_scenario_a_text(self) -> None:
        """Play the timeline when the Run Scenario StateButton is clicked with a_text "RUN"."""
        self._timeline.play()

    def _on_run_scenario_b_text(self) -> None:
        """Pause the timeline when the Run Scenario StateButton is clicked with b_text "STOP"."""
        self._timeline.pause()

    # ------------------------------------------------------------- internals

    def _reset_widgets(self) -> None:
        """Disable planning/run buttons and clear any joint sliders."""
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._to_cspace_btn.enabled = False
        self._to_task_space_btn.enabled = False
        self._joint_slider_models.clear()
        if hasattr(self, "_joint_sliders_container"):
            self._joint_sliders_container.clear()

    def _enable_planning_buttons(self) -> None:
        """Enable the plan buttons; called after the robot config loads."""
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False  # Enabled only after a successful plan.
        self._to_cspace_btn.enabled = True
        self._to_task_space_btn.enabled = True

    def _create_joint_sliders(self) -> None:
        """Build one labelled FloatSlider per controlled joint."""
        self._joint_slider_models.clear()
        self._joint_sliders_container.clear()

        if not self._scenario.is_robot_config_loaded():
            return

        joint_names = self._scenario.get_controlled_joint_names()
        lower_limits, upper_limits = self._scenario.get_joint_limits()
        default_target = self._scenario.get_default_target_configuration()

        with self._joint_sliders_container:
            for i, name in enumerate(joint_names):
                with ui.HStack(style=get_style(), spacing=5):
                    ui.Label(name, width=120, alignment=ui.Alignment.LEFT_CENTER)
                    field_model = ui.FloatField(name="Field", width=80, alignment=ui.Alignment.LEFT_CENTER).model
                    field_model.set_value(float(default_target[i]))
                    ui.FloatSlider(
                        width=ui.Fraction(1),
                        alignment=ui.Alignment.LEFT_CENTER,
                        min=float(lower_limits[i]),
                        max=float(upper_limits[i]),
                        step=0.01,
                        model=field_model,
                    )
                    self._joint_slider_models.append(field_model)

    def _show_error_dialog(self, message: str) -> None:
        """Show a modal error dialog with the given message."""
        dialog = ui.Window(
            "Planning Failed",
            width=500,
            height=0,
            visible=True,
            flags=(ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_MODAL | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS),
        )

        def _close_dialog() -> None:
            dialog.visible = False

            async def _destroy_next_frame() -> None:
                await omni.kit.app.get_app().next_update_async()
                dialog.destroy()

            asyncio.ensure_future(_destroy_next_frame())

        with dialog.frame:
            with ui.VStack(spacing=10):
                ui.Spacer(height=10)
                ui.Label(message, word_wrap=True, alignment=ui.Alignment.LEFT_TOP)
                ui.Spacer(height=10)
                ui.Button("OK", clicked_fn=_close_dialog, alignment=ui.Alignment.CENTER)
                ui.Spacer(height=10)
