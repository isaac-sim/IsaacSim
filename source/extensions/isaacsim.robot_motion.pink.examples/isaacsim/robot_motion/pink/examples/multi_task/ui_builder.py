# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI builder for the PINK Multi-Task IK example."""

from __future__ import annotations

import asyncio
from typing import Any

import omni.kit.app
import omni.timeline
import omni.ui as ui
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame, FloatField, StateButton
from isaacsim.gui.components.style import get_style
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdPhysics

from .scenario import FrankaMultiTaskExample

_GRID_ENVIRONMENT_PATH = "/Isaac/Environments/Grid/default_environment.usd"
_GRID_ENVIRONMENT_PRIM_PATH = "/World/gridEnvironment"


class UIBuilder:
    """UI builder for the PINK Multi-Task example.

    Provides controls for loading the scene, adjusting task cost weights, and
    running the multi-task IK demonstration.
    """

    def __init__(self) -> None:
        self.frames = []
        self.wrapped_ui_elements = []
        self._timeline = omni.timeline.get_timeline_interface()
        self._on_init()

    def on_menu_callback(self) -> None:
        """Handle menu callback when the UI is opened."""

    def on_timeline_event(self, event: Any) -> None:
        """Handle timeline events.

        Args:
            event: Timeline event.
        """
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float) -> None:
        """Handle physics step callbacks.

        Args:
            step: Size of physics step.
        """

    def on_stage_event(self, event: Any) -> None:
        """Handle stage events.

        Args:
            event: Stage event.
        """
        self._reset_extension()

    def cleanup(self) -> None:
        """Clean up UI elements."""
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self) -> None:
        """Build the extension UI with cost weight controls."""
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = Button(
                    "Load Button",
                    "LOAD",
                    tooltip="Load the scene and initialize the multi-task scenario",
                    on_click_fn=self._on_load_btn,
                )
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = Button(
                    "Reset Button", "RESET", tooltip="Reset the scenario", on_click_fn=self._on_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        task_weights_frame = CollapsableFrame("Task Weights", collapsed=False)

        with task_weights_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._position_cost_field = FloatField(
                    "Position Cost",
                    default_value=1.0,
                    tooltip="End-effector position tracking cost [cost/m].",
                )
                self.wrapped_ui_elements.append(self._position_cost_field)

                self._orientation_cost_field = FloatField(
                    "Orientation Cost",
                    default_value=1.0,
                    tooltip="End-effector orientation tracking cost [cost/rad].",
                )
                self.wrapped_ui_elements.append(self._orientation_cost_field)

                self._posture_cost_field = FloatField(
                    "Posture Cost",
                    default_value=1e-3,
                    tooltip="Joint posture regularization cost [cost/rad].",
                )
                self.wrapped_ui_elements.append(self._posture_cost_field)

                self._damping_cost_field = FloatField(
                    "Damping Cost",
                    default_value=1e-4,
                    tooltip="Joint velocity damping cost [cost*s/rad].",
                )
                self.wrapped_ui_elements.append(self._damping_cost_field)

                self._apply_weights_btn = Button(
                    "Apply Weights",
                    "APPLY",
                    tooltip="Apply updated task cost weights",
                    on_click_fn=self._on_apply_weights,
                )
                self._apply_weights_btn.enabled = False
                self.wrapped_ui_elements.append(self._apply_weights_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario")

        with run_scenario_frame:
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

    def _on_init(self) -> None:
        self._scenario = FrankaMultiTaskExample()

    def _on_load_btn(self) -> None:
        asyncio.ensure_future(self._load_scene_async())

    async def _load_scene_async(self) -> None:
        await stage_utils.create_new_stage_async(template="empty")

        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        stage_utils.define_prim("/World", "Xform")

        assets_root_path = await get_assets_root_path_async()
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + _GRID_ENVIRONMENT_PATH,
            path=_GRID_ENVIRONMENT_PRIM_PATH,
        )

        await self._scenario.load_example_assets()

        ViewportManager.set_camera_view(camera="/OmniverseKit_Persp", eye=[2, 1.5, 2], target=[0, 0, 0])

        stage = stage_utils.get_current_stage()
        physics_scene_path = "/World/PhysicsScene"
        if not stage.GetPrimAtPath(physics_scene_path).IsValid():
            UsdPhysics.Scene.Define(stage, physics_scene_path)
        await omni.kit.app.get_app().next_update_async()

        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._setup_scenario()

    def _setup_scenario(self) -> None:
        self._scenario.setup(
            position_cost=self._position_cost_field.get_value(),
            orientation_cost=self._orientation_cost_field.get_value(),
            posture_cost=self._posture_cost_field.get_value(),
            damping_cost=self._damping_cost_field.get_value(),
        )

        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True
        self._apply_weights_btn.enabled = True

    def _on_reset_btn(self) -> None:
        asyncio.ensure_future(self._reset_scene_async())

    async def _reset_scene_async(self) -> None:
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._scenario.reset(
            position_cost=self._position_cost_field.get_value(),
            orientation_cost=self._orientation_cost_field.get_value(),
            posture_cost=self._posture_cost_field.get_value(),
            damping_cost=self._damping_cost_field.get_value(),
        )
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _on_apply_weights(self) -> None:
        """Apply updated cost weights to the running scenario."""
        self._scenario.update_costs(
            position_cost=self._position_cost_field.get_value(),
            orientation_cost=self._orientation_cost_field.get_value(),
            posture_cost=self._posture_cost_field.get_value(),
            damping_cost=self._damping_cost_field.get_value(),
        )

    def _update_scenario(self, step: float, *args: Any, **kwargs: Any) -> None:
        if self._scenario._articulation is not None:
            if not self._scenario._articulation.is_physics_tensor_entity_valid():
                return
        self._scenario.update(step)

    def _on_run_scenario_a_text(self) -> None:
        self._timeline.play()

    def _on_run_scenario_b_text(self) -> None:
        self._timeline.pause()

    def _reset_extension(self) -> None:
        self._on_init()
        self._reset_ui()

    def _reset_ui(self) -> None:
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
        self._apply_weights_btn.enabled = False
