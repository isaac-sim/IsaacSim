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

"""Defines a base template class for creating standardized Isaac Sim example user interfaces with common world controls and extensible UI framework."""

import asyncio
from abc import abstractmethod

import carb.eventdispatcher
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers


class BaseSampleUITemplate:
    """Base template class for creating Isaac Sim example user interfaces.

    Provides a standardized UI framework for Isaac Sim examples with common controls including world loading,
    resetting, and timeline management. The class creates a collapsible frame structure with default world
    controls and extensible areas for custom UI elements.

    The template automatically handles stage and timeline event subscriptions, button state management, and
    provides abstract methods for customizing behavior during load, reset, and clear operations.

    Args:
        *args: Variable length argument list passed to the base class.
        **kwargs: Additional keyword arguments for configuring the UI template.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        self._ext_id = kwargs.get("ext_id")
        self._file_path = kwargs.get("file_path", "")
        self._title = kwargs.get("title", "Isaac Sim Example")
        self._doc_link = kwargs.get("doc_link", "")
        self._overview = kwargs.get("overview", "")
        self._sample = kwargs.get("sample", BaseSample())

        self._buttons = {}
        self._extra_stacks = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage_event_subscription = None
        self._timeline_event_subscription = None

    @property
    def sample(self) -> BaseSample:
        """The base sample instance associated with this UI template.

        Returns:
            The BaseSample instance containing the sample logic and world setup.
        """
        return self._sample

    @sample.setter
    def sample(self, sample: BaseSample) -> None:
        self._sample = sample

    def build_ui(self) -> None:
        """Constructs the complete UI layout for the sample template.

        Creates both the default frame containing world controls and any additional frames specific to the sample.
        """
        # separating out building default frame and extra frames, so examples can override the extra frames function
        self.build_default_frame()
        self.build_extra_frames()

    def build_default_frame(self) -> None:
        """Builds the default UI frame containing standard world control buttons.

        Creates the main stack layout with headers, world controls frame, and Load World/Reset buttons.
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
            self._extra_stacks = ui.VStack(margin=5, spacing=5, height=0)

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

    def get_extra_frames_handle(self) -> object:
        """Retrieves the UI container for additional sample-specific frames.

        Returns:
            The VStack container where sample-specific UI elements can be added.
        """
        return self._extra_stacks

    @abstractmethod
    def build_extra_frames(self) -> None:
        """Builds additional UI frames specific to the sample implementation.

        This abstract method must be implemented by subclasses to add sample-specific UI elements.
        """

    def _on_load_world(self) -> None:
        """Handles the Load World button click event.

        Asynchronously loads the sample world, sets up event subscriptions, and updates button states.
        """

        async def _on_load_world_async() -> None:
            await self._sample.load_world_async()
            await omni.kit.app.get_app().next_update_async()

            # Subscribe to stage closed events using Events 2.0
            usd_context = omni.usd.get_context()
            self._stage_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self.on_stage_event,
                observer_name="base_sample_extension.on_stage_closed",
            )

            # Subscribe to timeline stop events using Events 2.0
            self._timeline_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._reset_on_stop_event,
                observer_name="base_sample_extension._reset_on_stop_event",
            )

            self._enable_all_buttons(True)
            self._buttons["Load World"].enabled = False
            self.post_load_button_event()

        asyncio.ensure_future(_on_load_world_async())

    def _on_reset(self) -> None:
        """Handles the Reset button click event.

        Asynchronously resets the sample and triggers post-reset event handling.
        """

        async def _on_reset_async() -> None:
            await self._sample.reset_async()
            await omni.kit.app.get_app().next_update_async()
            self.post_reset_button_event()

        asyncio.ensure_future(_on_reset_async())

    @abstractmethod
    def post_reset_button_event(self) -> None:
        """Handles actions to perform after the Reset button is clicked.

        This abstract method must be implemented by subclasses to define sample-specific reset behavior.
        """

    @abstractmethod
    def post_load_button_event(self) -> None:
        """Handles actions to perform after the Load World button is clicked.

        This abstract method must be implemented by subclasses to define sample-specific loading behavior.
        """

    @abstractmethod
    def post_clear_button_event(self) -> None:
        """Handles actions to perform after the timeline stop event clears the world.

        This abstract method must be implemented by subclasses to define sample-specific cleanup behavior.
        """

    def _enable_all_buttons(self, flag: bool) -> None:
        """Enables or disables all UI buttons in the widget.

        Args:
            flag: Whether to enable the buttons.
        """
        for btn_name, btn in self._buttons.items():
            if isinstance(btn, omni.ui._ui.Button):
                btn.enabled = flag

    def on_shutdown(self) -> None:
        """Cleans up resources and subscriptions when the widget is shut down."""
        # Clean up subscriptions
        self._stage_event_subscription = None
        self._timeline_event_subscription = None

        self._extra_stacks = None
        self._buttons = {}
        self._sample = None

    def on_stage_event(self, event: carb.eventdispatcher.Event) -> None:
        """Stage closed event callback.

        Note: With Events 2.0, this is called only for CLOSED events.

        Args:
            event: The stage event data.
        """
        self._sample._physics_cleanup()
        if hasattr(self, "_buttons"):
            if self._buttons is not None:
                self._enable_all_buttons(False)
                self._buttons["Load World"].enabled = True

    def _reset_on_stop_event(self, event: carb.eventdispatcher.Event) -> None:
        """Timeline stop event callback.

        Note: With Events 2.0, this is called only for STOP events.

        Args:
            event: The timeline event data.
        """
        self._buttons["Load World"].enabled = False
        self._buttons["Reset"].enabled = True
        self.post_clear_button_event()
