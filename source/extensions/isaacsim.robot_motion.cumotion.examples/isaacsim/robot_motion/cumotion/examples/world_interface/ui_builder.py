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

"""User interface builder for the CumotionWorldInterface example.

Strictly a view: builds widgets, forwards button clicks and combo
selections to the :class:`CumotionWorldInterfaceExample` scenario, and
resets widget state on USD stage changes.  All scene-loading and
world-binding state live on the scenario.
"""

import asyncio
from typing import Any

import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame, StateButton
from isaacsim.gui.components.style import get_style
from omni.kit.async_engine import run_coroutine

from .scenario import CumotionWorldInterfaceExample

_UPDATE_STYLE_ITEMS = ["synchronize", "synchronize_transforms", "synchronize_properties"]
_DEFAULT_UPDATE_STYLE_INDEX = 1  # synchronize_transforms - matches scenario default


class UIBuilder:
    """Builds and drives the CumotionWorldInterface example UI.

    Owns only widgets and the asyncio task for scene loading.  The
    :class:`CumotionWorldInterfaceExample` scenario owns all scene/world
    state.

    The UI has two sections:
      - **World Controls** - Load the scene and reset the scenario.
      - **Run Scenario** - Update style combo + start/stop the sync loop.
    """

    def __init__(self) -> None:
        self.frames: list[Any] = []
        self.wrapped_ui_elements: list[Any] = []
        self._timeline = omni.timeline.get_timeline_interface()
        self._load_task: asyncio.Task | None = None
        self._update_style_items = list(_UPDATE_STYLE_ITEMS)
        self._scenario: CumotionWorldInterfaceExample | None = CumotionWorldInterfaceExample()

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

        Args:
            event: USD stage event payload supplied by Kit.
        """
        if self._load_task is not None and not self._load_task.done():
            return
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = CumotionWorldInterfaceExample()
        self._reset_widgets()

    def on_timeline_event(self, event: Any) -> None:
        """Reset the Run/Stop button on timeline state changes.

        Args:
            event: Timeline event payload supplied by Kit.
        """
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

                self._reset_btn = Button(
                    "Reset Button",
                    "RESET",
                    tooltip="Reset the scenario",
                    on_click_fn=self._on_reset_btn,
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        with CollapsableFrame("Run Scenario"):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                with ui.HStack(style=get_style(), spacing=5):
                    ui.Label("Update Style:", width=120, alignment=ui.Alignment.LEFT_CENTER)
                    self._update_style_combo = ui.ComboBox(
                        _DEFAULT_UPDATE_STYLE_INDEX,
                        *self._update_style_items,
                        name="UpdateStyleComboBox",
                        width=ui.Fraction(1),
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    self._update_style_combo.model.add_item_changed_fn(self._on_update_style_changed)

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

    # ------------------------------------------------------------ handlers

    def _on_load_btn(self) -> None:
        """Handle LOAD click - cancel any prior load, drop the old scenario, kick off a new one."""
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        if self._scenario is not None:
            self._scenario.cleanup()
        self._scenario = CumotionWorldInterfaceExample()
        self._reset_widgets()
        self._load_task = run_coroutine(self._load_scene_async())

    async def _load_scene_async(self) -> None:
        """Load the scene then enable run/reset controls."""
        await self._scenario.load()
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_reset_btn(self) -> None:
        """Handle RESET click - stop the timeline and rebuild the world binding."""
        self._timeline.stop()
        self._scenario.reset()
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _on_update_style_changed(self, model: Any, _val: Any) -> None:
        """Push the new combo selection into the scenario.

        Args:
            model: Combo box model containing the selected update style index.
            _val: Unused value supplied by the item-changed callback.
        """
        selected_index = model.get_item_value_model().as_int
        self._scenario.set_update_style(self._update_style_items[selected_index])

    def _on_run_scenario_a_text(self) -> None:
        """Play the timeline when the Run Scenario StateButton is clicked with a_text "RUN"."""
        self._timeline.play()

    def _on_run_scenario_b_text(self) -> None:
        """Pause the timeline when the Run Scenario StateButton is clicked with b_text "STOP"."""
        self._timeline.pause()

    # ------------------------------------------------------------ per-tick

    def _update_scenario(self, step: float, *args: Any, **kwargs: Any) -> None:
        """Run the scenario's per-physics-step callback.

        Args:
            step: Physics time step in seconds.
            *args: Extra callback arguments supplied by the StateButton.
            **kwargs: Extra callback keyword arguments supplied by the StateButton.
        """
        if self._scenario is not None:
            self._scenario.step(step)

    # ------------------------------------------------------------- internals

    def _reset_widgets(self) -> None:
        """Disable run/reset controls."""
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
