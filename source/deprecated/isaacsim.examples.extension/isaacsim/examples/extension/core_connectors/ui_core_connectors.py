# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI components that provide specialized buttons for loading and resetting simulation scenes in Isaac Sim."""

import asyncio
from collections.abc import Callable

import carb
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import update_stage_async
from isaacsim.gui.components.element_wrappers.ui_widget_wrappers import *


class LoadButton(UIWidgetWrapper):
    """Create a special type of UI button that connects to the isaacsim.core.api.World to enable convenient "Load" functionality.

    The World acts as a scene manager that simplifies user interaction with the simulator.
    This provides the user with certain guarantees at the time that their callback functions are
    called.

    The setup_scene_fn() is called with the guarantee that a World has been created.  In this function,
    the user is meant to add the assets they want to the USD stage.  These assets then must also be added to
    the World. World is a singleton class.  An example setup_scene_fn implementation would include:

        - world = World.instance() # Get the unique instance of the World
        - world.scene.add(usd_object) # Add the user-loaded usd object to the scene

    The setup_post_load() function is called with the guarantees that the World has been created, the
    setup_scene_fn() has already been called, all objects that the user added to the World have been properly
    initialized, and the timeline is paused at timestep 0.

    Args:
        label: Short descriptive text to the left of the LoadButton.
        text: Text on the LoadButton.
        tooltip: Text to appear when the mouse hovers over the LoadButton.
        setup_scene_fn: A function that will be called when the LoadButton is clicked.
            The user should use this function to add their assets to the USD stage and to add their assets
            to the World. This function should take 0 arguments.  The return value will not be used.
        setup_post_load_fn: A function that will be called when the LoadButton is clicked.
            The function is called with the guarantees that the World has been created, the
            setup_scene_fn() has already been called, all objects that the user added to the World have been properly
            initialized, and the timeline is paused at timestep 0.  This function should take 0 arguments.
            The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        text: str,
        tooltip: str = "",
        setup_scene_fn: Callable = None,
        setup_post_load_fn: Callable = None,
    ) -> None:
        self.setup_scene_fn = setup_scene_fn
        self.setup_post_load_fn = setup_post_load_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

        self._world_settings = {}

    @property
    def label(self) -> ui.Label:
        """Get the UI Label element that contains the descriptive text."""
        return self._label

    @property
    def button(self) -> ui.Button:
        """Get the UI Button element."""
        return self._button

    def set_setup_scene_fn(self, setup_scene_fn: Callable) -> None:
        """Set the setup_scene_fn that will be called when the LoadButton is clicked.

        The setup_scene_fn() is called with the guarantee that a World has been created.  In this function,
        the user is meant to add the assets they want to the USD stage.  These assets then must also be added to
        the World. World is a singleton class.  An example setup_scene_fn implementation would include:

        world = World.instance() # Get the unique instance of the World
        world.scene.add(usd_object) # Add the user-loaded usd object to the scene

        Args:
            setup_scene_fn: A function that will be called when the LoadButton is clicked.
                the user should use this function to add their assets to the USD stage and to add their assets
                to the World. This function should take 0 arguments.  The return value will not be used.
                Defaults to None.

        """
        self.setup_scene_fn = setup_scene_fn

    def set_setup_post_load_fn(self, setup_post_load_fn: Callable) -> None:
        """Set the setup_post_load_fn that will be called when the LoadButton is clicked.

        Args:
            setup_post_load_fn: A function that will be called when the LoadButton is clicked.
                The function is called with the guarantees that the World has been created, the
                setup_scene_fn() has already been called, all objects that the user added to the World have been properly
                initialized, and the timeline will be paused at timestep 0.  This function should take 0 arguments.
                The return value will not be used.  Defaults to None.
        """
        self.setup_post_load_fn = setup_post_load_fn

    def set_world_settings(self, **kwargs: object) -> None:
        """Pressing a Load Button will create a new instance of the isaacsim.core.api.World.

        The default settings will be used unless the user specifies new settings at runtime before the Load Button is clicked.

        The default settings will ensure that the physics and rendering timesteps are fixed at 1/60.0 seconds (see set_defaults argument).
        It is important to note that this will ensure that code is deterministic, but may not be executed in real time.
        I.e. physics and render dts will adjust automatically if the simulation is running too fast or slow.

        Args:
            **kwargs: Keyword arguments passed to the World constructor. Supported keys include:
                physics_dt: dt between physics steps. Defaults to None.
                rendering_dt: dt between rendering steps. Note: rendering means
                    rendering a frame of the current application and not
                    only rendering a frame to the viewports/ cameras. So UI
                    elements of Isaac Sim will be refreshed with this dt
                    as well if running non-headless. Defaults to None.
                stage_units_in_meters: The metric units of assets. This will affect gravity value..etc.
                    Defaults to None.
                physics_prim_path: specifies the prim path to create a PhysicsScene at,
                    only in the case where no PhysicsScene already defined.
                    Defaults to "/physicsScene".
                set_defaults: set to True to use the defaults settings. Defaults to True.
                backend: specifies the backend to be used (numpy or torch). Defaults to numpy.
                device: specifies the device to be used if running on the gpu with torch backend.
        """
        self._world_settings = kwargs

    def _on_clicked_fn_wrapper(self) -> None:
        """This function is called when the Load Button is Clicked."""
        # From an extension workflow, the stage and world need to be interacted with asynchronously

        async def _on_click_async() -> None:
            # Remove any previous World instance
            prev_world = World.instance()
            if prev_world is not None:
                prev_world.clear_all_callbacks()
                prev_world.clear_instance()
                prev_world = None
                # prev_world.clear()
            await update_stage_async()

            # Create a new World instance with user-defined settings.  See self.set_world_settings()
            world = World(**self._world_settings)

            # Call user function to put assets on the stage and add them to the World
            if self.setup_scene_fn is not None:
                self.setup_scene_fn()

            await world.initialize_simulation_context_async()

            await world.reset_async()
            await update_stage_async()
            await world.pause_async()

            # User assets are now initialized, and the timeline is playing at timestep 0
            if self.setup_post_load_fn is not None:
                self.setup_post_load_fn()

        asyncio.ensure_future(_on_click_async())

    def _create_ui_widget(self, label: str, text: str, tooltip: str) -> object:
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame


class ResetButton(UIWidgetWrapper):
    """Create a special type of UI button that connects to the isaacsim.core.api.World to perform a reset.

    If no World instance exists when this button will be clicked, this button will not create one.
    In this case, the button logs a warning and calls user callback functions with no guarantees
    on the Simulator State.

    Args:
        label: Short descriptive text to the left of the ResetButton.
        text: Text on the ResetButton.
        tooltip: Text to appear when the mouse hovers over the ResetButton.
        pre_reset_fn: A function that will be called before resetting the World.
            This function should take 0 arguments. The return value will not be used.
        post_reset_fn: A function that will be called after the World is reset.
            When this function is called, the timeline will be paused at timestep 0, and all
            USD assets added to the World will be properly initialized and placed at their default positions.
            This function should take no arguments. The return value will not be used.
    """

    def __init__(
        self, label: str, text: str, tooltip: str = "", pre_reset_fn: Callable = None, post_reset_fn: Callable = None
    ) -> None:
        self._pre_reset_fn = pre_reset_fn
        self._post_reset_fn = post_reset_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def button(self) -> ui.Button:
        """UI Button element.

        Returns:
            UI Button element.
        """
        return self._button

    def set_pre_reset_fn(self, pre_reset_fn: Callable) -> None:
        """Set the pre_reset_fn for when the ResetButton is clicked.

        Args:
            pre_reset_fn: A function that will be called before resetting the World.
                This function should take 0 arguments.  The return value will not be used.
        """
        self._pre_reset_fn = pre_reset_fn

    def set_post_reset_fn(self, post_reset_fn: Callable) -> None:
        """Set the post_reset_fn for when the ResetButton is clicked.

        Args:
            post_reset_fn: A function that will be called after the World is reset.
                When this function is called, the timeline will be paused at timestep 0, and all
                USD assets added to the World will be properly initialized and placed at their default locations.
                This function should take no arguments. The return value will not be used.
        """
        self._post_reset_fn = post_reset_fn

    def _on_clicked_fn_wrapper(self) -> None:
        """This function is called when the Reset Button is Clicked."""
        # From an extension workflow, the stage and world need to be interacted with asynchronously

        async def _on_click_async() -> None:
            # Call user function pre_reset
            if self._pre_reset_fn is not None:
                self._pre_reset_fn()

            world = World.instance()

            if world is None:
                carb.log_warn("Reset Button was used when there is no instance of World.")
            else:
                await world.reset_async()
                await update_stage_async()
                await world.pause_async()

            # User assets are initialized, and the timeline is playing at timestep 0
            if self._post_reset_fn is not None:
                self._post_reset_fn()

        asyncio.ensure_future(_on_click_async())

    def _create_ui_widget(self, label: str, text: str, tooltip: str) -> object:
        """Create the UI widget frame containing the label and button.

        Args:
            label: Short descriptive text to the left of the ResetButton.
            text: Text on the ResetButton.
            tooltip: Text to appear when the mouse hovers over the ResetButton.

        Returns:
            The containing frame with the UI elements.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame
