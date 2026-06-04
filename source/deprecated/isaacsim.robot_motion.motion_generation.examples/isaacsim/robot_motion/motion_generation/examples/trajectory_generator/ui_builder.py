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

"""UI builder module for the UR10 trajectory generation tutorial extension."""

from __future__ import annotations

import asyncio
from typing import Any

import carb
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import create_new_stage, get_current_stage, update_stage_async
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.gui.components.element_wrappers import Button, CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from pxr import Sdf, UsdLux

from .scenario import UR10TrajectoryGenerationExample


class UIBuilder:
    """UI builder for the UR10 trajectory generation tutorial extension.

    Provides an interactive interface for demonstrating robot motion generation concepts including configuration space
    trajectories, task space trajectories, and advanced trajectory planning. The UI enables users to load a UR10 robot
    scenario, reset the simulation, and execute different types of trajectory generation examples.

    The interface includes world control buttons for loading and resetting the scenario, and trajectory execution
    buttons for running configuration space, task space, and advanced trajectory demonstrations. Each trajectory type
    showcases different aspects of robot motion planning and control.

    The UI automatically handles timeline events, physics step callbacks, and stage events to maintain proper state
    management throughout the simulation lifecycle. It integrates with the Isaac Sim World system to manage scene
    objects and simulation parameters.
    """

    def __init__(self) -> None:
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self) -> None:
        """Callback for when the UI is opened from the toolbar.

        This is called directly after build_ui().
        """

    def on_timeline_event(self, event: object) -> None:
        """Callback for Timeline events (Play, Pause, Stop).

        Note: With Events 2.0, this is called only for STOP events from the extension.

        Args:
            event: Timeline event.
        """
        # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
        # For complete robustness, the user should resolve those edge cases here
        # In general, for extensions based off this template, there is no value to having the user click the play/stop
        # button instead of using the Load/Reset/Run buttons provided.
        self._cspace_trajectory_btn.reset()
        self._taskspace_trajectory_btn.reset()
        self._advanced_trajectory_btn.reset()
        self._cspace_trajectory_btn.enabled = False
        self._taskspace_trajectory_btn.enabled = False
        self._advanced_trajectory_btn.enabled = False

    def on_physics_step(self, step: float) -> None:
        """Callback for Physics Step.

        Physics steps only occur when the timeline is playing.

        Args:
            step: Size of physics step.
        """

    def on_stage_event(self, event: object) -> None:
        """Callback for Stage Events.

        Note: With Events 2.0, this is called only for OPENED events from the extension.

        Args:
            event: Stage event.
        """
        # If the user opens a new stage, the extension should completely reset
        self._reset_extension()

    def cleanup(self) -> None:
        """Called when the stage is closed or the extension is hot reloaded.

        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called.
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self) -> None:
        """Build a custom UI tool to run your extension.

        This function will be called any time the UI window is closed and reopened.
        """
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = Button("Load Button", "LOAD", on_click_fn=self._on_load_btn_clicked)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = Button("Reset Button", "RESET", on_click_fn=self._on_reset_btn_clicked)
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario", collapsed=False)

        with run_scenario_frame:
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

                self._advanced_trajectory_btn = StateButton(
                    "Run Advanced Trajectory",
                    "ADVANCED TRAJECTORY",
                    "STOP",
                    on_a_click_fn=self._on_run_advanced,
                    on_b_click_fn=self._on_stop,
                    physics_callback_fn=self._update_scenario,
                )
                self._advanced_trajectory_btn.enabled = False

                self.wrapped_ui_elements.append(self._cspace_trajectory_btn)
                self.wrapped_ui_elements.append(self._taskspace_trajectory_btn)
                self.wrapped_ui_elements.append(self._advanced_trajectory_btn)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self) -> None:
        """Initialize the extension state and scenario."""
        self._articulation = None
        self._cuboid = None
        self._scenario = UR10TrajectoryGenerationExample()

    def _add_light_to_stage(self) -> None:
        """A new stage does not have a light by default. This function creates a spherical light."""
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath().pathString)).set_world_pose([6.5, 0, 12])

    def _on_load_btn_clicked(self) -> None:
        asyncio.ensure_future(self._load_world_async())

    async def _load_world_async(self) -> None:
        prev_world = World.instance()
        if prev_world is not None:
            prev_world.clear_all_callbacks()
            prev_world.clear_instance()
        await update_stage_async()
        world = World(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
        self._setup_scene()
        await world.initialize_simulation_context_async()
        await world.reset_async()
        await update_stage_async()
        await world.pause_async()
        self._setup_scenario()

    def _on_reset_btn_clicked(self) -> None:
        asyncio.ensure_future(self._reset_world_async())

    async def _reset_world_async(self) -> None:
        world = World.instance()
        if world is None:
            carb.log_warn("Reset Button was used when there is no instance of World.")
        else:
            await world.reset_async()
            await update_stage_async()
            await world.pause_async()
        self._on_post_reset_btn()

    def _setup_scene(self) -> None:
        """This function is attached to the Load Button as the setup_scene_fn callback.

        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        create_new_stage()
        self._add_light_to_stage()
        set_camera_view(eye=[2.5, 2, 2.5], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        loaded_objects = self._scenario.load_example_assets()

        # Add user-loaded objects to the World
        world = World.instance()
        for loaded_object in loaded_objects:
            world.scene.add(loaded_object)

    def _setup_scenario(self) -> None:
        """This function is attached to the Load Button as the setup_post_load_fn callback.

        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.

        In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
        provided prim in a circle around the robot.
        """
        self._scenario.setup()

        # UI management
        self._cspace_trajectory_btn.enabled = True
        self._taskspace_trajectory_btn.enabled = True
        self._advanced_trajectory_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset_btn(self) -> None:
        """This function is attached to the Reset Button as the post_reset_fn callback.

        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        self._scenario.reset()
        self._cspace_trajectory_btn.reset()
        self._taskspace_trajectory_btn.reset()
        self._advanced_trajectory_btn.reset()
        self._cspace_trajectory_btn.enabled = True
        self._taskspace_trajectory_btn.enabled = True
        self._advanced_trajectory_btn.enabled = True

    def _update_scenario(self, step: float, *args: Any, **kwargs: Any) -> None:
        """This function is attached to the Run Scenario StateButton.

        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        Args:
            step: The dt of the current physics step
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        self._scenario.update(step)

    def _on_run_cspace(self) -> None:
        """Initiates the configuration space trajectory demonstration.

        Starts the timeline, resets the scenario, and sets up the configuration space trajectory.
        Disables other trajectory buttons to prevent conflicts.
        """
        self._timeline.play()
        self._scenario.reset()
        self._taskspace_trajectory_btn.reset()
        self._advanced_trajectory_btn.reset()
        self._scenario.setup_cspace_trajectory()

    def _on_run_taskspace(self) -> None:
        """Initiates the task space trajectory demonstration.

        Starts the timeline, resets the scenario, and sets up the task space trajectory.
        Disables other trajectory buttons to prevent conflicts.
        """
        self._timeline.play()
        self._scenario.reset()
        self._cspace_trajectory_btn.reset()
        self._advanced_trajectory_btn.reset()
        self._scenario.setup_taskspace_trajectory()

    def _on_run_advanced(self) -> None:
        """Initiates the advanced trajectory demonstration.

        Starts the timeline, resets the scenario, and sets up the advanced trajectory.
        Disables other trajectory buttons to prevent conflicts.
        """
        self._timeline.play()
        self._scenario.reset()
        self._cspace_trajectory_btn.reset()
        self._taskspace_trajectory_btn.reset()
        self._scenario.setup_advanced_trajectory()

    def _on_stop(self) -> None:
        """Stops the currently running trajectory demonstration.

        Pauses the timeline to halt the physics simulation.
        """
        self._timeline.pause()

    def _reset_extension(self) -> None:
        """This is called when the user opens a new stage from self.on_stage_event().

        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self) -> None:
        """Resets the UI elements to their initial state.

        Disables all trajectory buttons and the reset button, and resets their visual state.
        """
        self._cspace_trajectory_btn.reset()
        self._taskspace_trajectory_btn.reset()
        self._advanced_trajectory_btn.reset()
        self._cspace_trajectory_btn.enabled = False
        self._taskspace_trajectory_btn.enabled = False
        self._advanced_trajectory_btn.enabled = False
        self._reset_btn.enabled = False
