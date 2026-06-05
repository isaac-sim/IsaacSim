# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides a base template class for creating interactive Isaac Sim example user interfaces.

.. deprecated::
    This module is deprecated. Use :mod:`isaacsim.examples.base.base_sample_experimental`
    and its corresponding UI template which use the new experimental APIs
    (``SimulationManager``, ``app_utils``, ``stage_utils``) instead of the legacy ``World`` API.
"""

from __future__ import annotations

import asyncio
import warnings
from abc import abstractmethod
from typing import Any

import carb.eventdispatcher
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers

from .base_sample import BaseSample


class BaseSampleUITemplate:
    """Base template class for creating interactive Isaac Sim example UIs.

    This class provides a standardized UI framework for Isaac Sim examples, including world controls,
    header information, and extensible frames for custom content. It manages the lifecycle of sample
    execution including loading, resetting, and cleanup operations.

    The template creates a default UI structure with:
    - Header section displaying extension information, title, documentation link, and overview
    - World Controls frame with Load World and Reset buttons
    - Extensible frames area for custom UI components
    - Event handling for stage and timeline events

    Subclasses must implement the abstract methods to define custom UI frames and handle button events
    specific to their example.

    Args:
        *args: Variable length argument list passed to the constructor.
        **kwargs: Keyword arguments for configuring the UI template. Supported keys include ext_id,
            file_path, title, doc_link, overview, and sample.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        warnings.warn(
            "BaseSampleUITemplate from isaacsim.examples.interactive.base_sample is deprecated. "
            "Use BaseSampleUITemplate from isaacsim.examples.base (base_sample_experimental) instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self._ext_id = kwargs.get("ext_id")
        self._file_path = kwargs.get("file_path", "")
        self._title = kwargs.get("title", "Isaac Sim Example")
        self._doc_link = kwargs.get("doc_link", "")
        self._overview = kwargs.get("overview", "")
        self._sample = kwargs.get("sample", BaseSample())

        self._buttons = {}
        self.extra_stacks = None
        self._stage_event_sub = None
        self._timeline_stop_callback_id = None

    @property
    def sample(self) -> BaseSample:
        """The sample instance associated with this UI template.

        Returns:
            The BaseSample instance.
        """
        return self._sample

    @sample.setter
    def sample(self, sample: Any) -> None:
        self._sample = sample

    def get_world(self) -> World:
        """Gets the current World instance.

        Returns:
            The active World instance.
        """
        return World.instance()

    def build_window(self) -> None:
        """Builds the main window for the sample UI template."""
        # separating out building the window and building the UI, so that example browser can build_ui but not the window
        # self._window = omni.ui.Window(
        #     self.example_name, width=350, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
        # )
        # with self._window.frame:
        #     self.build_ui()
        # return self._window

    def build_ui(self) -> None:
        """Builds the complete user interface by constructing the default frame and extra frames."""
        # separating out building default frame and extra frames, so examples can override the extra frames function

        self.build_default_frame()
        self.build_extra_frames()
        return

    def build_default_frame(self) -> None:
        """Builds the default UI frame containing world controls and basic buttons.

        Creates the main vertical stack with headers, a collapsible frame for world controls,
        and default Load World and Reset buttons.
        """
        self._main_stack = ui.VStack(spacing=5, height=0)
        with self._main_stack:
            setup_ui_headers(
                self._ext_id, self._file_path, self._title, self._doc_link, self._overview, info_collapsed=False
            )
            self._controls_frame = ui.CollapsableFrame(
                title="World Controls",
                width=ui.Fraction(1),
                height=0,
                collapsed=False,
                style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            self.extra_stacks = ui.VStack(margin=5, spacing=5, height=0)

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

        return

    def get_extra_frames_handle(self) -> ui.VStack:
        """Gets the handle to the extra frames container for adding custom UI elements.

        Returns:
            The VStack container for extra UI frames.
        """
        return self.extra_stacks

    @abstractmethod
    def build_extra_frames(self) -> None:
        """Builds additional custom frames for the sample UI.

        This abstract method must be implemented by subclasses to define sample-specific UI elements.
        """
        return

    def _on_load_world(self) -> None:
        """Handles the Load World button click event.

        Asynchronously loads the world, sets up event subscriptions for stage and timeline events,
        and updates button states.
        """

        async def _on_load_world_async() -> None:
            await self._sample.load_world_async()
            await omni.kit.app.get_app().next_update_async()

            # Subscribe to stage closed events using Events 2.0
            usd_context = omni.usd.get_context()
            self._stage_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self.on_stage_event,
                observer_name="base_sample_extension.on_stage_event",
            )

            # Subscribe to timeline stop events
            self._timeline_stop_callback_id = SimulationManager.register_callback(
                self._reset_on_stop_event, event=SimulationEvent.SIMULATION_STOPPED
            )

            self._enable_all_buttons(True)
            self._buttons["Load World"].enabled = False
            self.post_load_button_event()

        asyncio.ensure_future(_on_load_world_async())
        return

    def _on_reset(self) -> None:
        """Handles the Reset button click event.

        Asynchronously resets the sample and triggers post-reset button event handling.
        """

        async def _on_reset_async() -> None:
            await self._sample.reset_async()
            await omni.kit.app.get_app().next_update_async()
            self.post_reset_button_event()

        asyncio.ensure_future(_on_reset_async())
        return

    @abstractmethod
    def post_reset_button_event(self) -> None:
        """Handles actions to perform after the reset button is clicked.

        This abstract method must be implemented by subclasses to define sample-specific
        post-reset behavior.
        """
        return

    @abstractmethod
    def post_load_button_event(self) -> None:
        """Called after the Load World button is pressed and world loading is complete.

        This method is executed after the sample world is loaded and UI buttons are enabled.
        """
        return

    @abstractmethod
    def post_clear_button_event(self) -> None:
        """Called after the timeline is stopped and UI state is reset.

        This method is executed when the timeline stop event occurs and buttons are reconfigured.
        """
        return

    def _enable_all_buttons(self, flag: bool) -> None:
        """Enables or disables all UI buttons in the sample interface.

        Args:
            flag: Whether to enable or disable the buttons.
        """
        for btn_name, btn in self._buttons.items():
            if isinstance(btn, omni.ui._ui.Button):
                btn.enabled = flag
        return

    def on_shutdown(self) -> None:
        """Cleans up resources when the sample UI is being shut down.

        Resets event subscriptions, UI components, buttons, and sample references.
        """
        self._stage_event_sub = None
        if getattr(self, "_timeline_stop_callback_id", None) is not None:
            SimulationManager.deregister_callback(self._timeline_stop_callback_id)
            self._timeline_stop_callback_id = None
        self.extra_stacks = None
        self._buttons = {}
        self._sample = None
        return

    def on_stage_event(self, event: object) -> None:
        """Stage closed event callback.

        Note: With Events 2.0, this is called only for CLOSED events.

        Args:
            event: The stage event object containing event details.
        """
        if self._timeline_stop_callback_id is not None:
            SimulationManager.deregister_callback(self._timeline_stop_callback_id)
            self._timeline_stop_callback_id = None
        if World.instance() is not None:
            self._sample._world_cleanup()
            self._sample._world.clear_instance()
            if hasattr(self, "_buttons"):
                if self._buttons is not None:
                    self._enable_all_buttons(False)
                    self._buttons["Load World"].enabled = True
        return

    def _reset_on_stop_event(self, event: object) -> None:
        """Timeline stop event callback.

        Note: With Events 2.0, this is called only for STOP events.

        Args:
            event: The timeline event object containing event details.
        """
        world = World.instance()
        if world is not None:
            world.clear_physics_callbacks()

        self._buttons["Load World"].enabled = False
        self._buttons["Reset"].enabled = True
        self.post_clear_button_event()

        return
