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

"""Extension providing interactive examples for integrating the Cortex framework with Franka robotic arm behaviors in Isaac Sim."""

import asyncio
import os

import omni
import omni.ext
import omni.ui as ui
from isaacsim.cortex.examples.base_sample import BaseSampleUITemplate
from isaacsim.cortex.examples.franka_cortex.franka_cortex import FrankaCortex
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, dropdown_builder, get_style


class FrankaCortexExtension(omni.ext.IExt):
    """Extension demonstrating Cortex framework integration with Franka robotic arm examples.

    This extension provides an interactive interface for exploring various Cortex behaviors with a Franka robot,
    including block stacking, state machines, decider networks, and interactive games. It showcases how to integrate
    the Cortex framework for behavior-driven robotics programming within Isaac Sim.

    The extension offers multiple pre-configured behaviors such as block stacking tasks, simple state machines,
    decider networks for complex decision-making, and interactive peck games. Users can switch between different
    behaviors at runtime and observe diagnostic information about the robot's decision-making process.

    The interface includes controls for loading worlds, resetting environments, starting tasks, and monitoring
    the robot's behavior execution through diagnostic panels that display decision stacks and real-time feedback.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Franka Cortex extension on startup.

        Registers the extension with the examples browser, sets up the UI template with
        configuration for Cortex-based Franka robot behaviors, and creates the main
        sample instance.

        Args:
            ext_id: Extension identifier used for UI initialization.
        """
        self.example_name = "Franka Cortex Examples"
        self.category = "Cortex"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Franka Cortex Examples",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html#isaac-sim-app-tutorial-cortex-4-franka-block-stacking",
            "overview": "This Example shows how to Use Cortex for multiple behaviors robot and Cortex behaviors in Isaac Sim.\\Open 'Link to Docs' to see more detailed instructions on how to run this example. \n\nPress the 'Open in IDE' button to view the source code.",
        }

        ui_handle = FrankaCortexUI(**ui_kwargs)

        ui_handle.sample = FrankaCortex(ui_handle.on_diagnostics)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Cleans up the extension on shutdown.

        Deregisters the extension from the examples browser to prevent memory leaks
        and ensure proper cleanup.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class FrankaCortexUI(BaseSampleUITemplate):
    """A user interface for the Franka Cortex examples in Isaac Sim.

    This class provides an interactive GUI for running various Cortex behavior examples with a Franka robot. It allows users to select different behaviors such as block stacking, state machines, decider networks, and peck games. The interface includes controls for loading worlds, starting behaviors, resetting the environment, and viewing diagnostic information.

    The UI features behavior selection dropdown, load/reset controls, task control buttons, and diagnostic panels that display decision stack information and diagnostic messages from the running behaviors.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        sample_behaviors_id = ext_manager.get_enabled_extension_id("isaacsim.cortex.behaviors")
        behavior_path = (
            omni.kit.app.get_app().get_extension_manager().get_extension_path(sample_behaviors_id)
            + "/isaacsim/cortex/behaviors/franka"
        )
        # example starter parameters
        self.behavior_map = {
            "Block Stacking": f"{behavior_path}/block_stacking_behavior.py",
            "Simple State Machine": f"{behavior_path}/simple/simple_state_machine.py",
            "Simple Decider Network": f"{behavior_path}/simple/simple_decider_network.py",
            "Peck State Machine": f"{behavior_path}/peck_state_machine.py",
            "Peck Decider Network": f"{behavior_path}/peck_decider_network.py",
            "Peck Game": f"{behavior_path}/peck_game.py",
        }
        self.selected_behavior = "Block Stacking"
        self.loaded = False

    def build_ui(self) -> None:
        """Builds the user interface for the Franka Cortex example.

        Creates the main UI elements including behavior selection dropdown, load world button,
        reset button, and additional control frames for task management and diagnostics.
        """
        # overwriting the baseSample's default frame
        self.task_ui_elements = {}
        self.build_default_frame()

        # modification to the control frame
        with self._controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.task_ui_elements["Selected Behavior"] = dropdown_builder(
                    "Selected Behavior",
                    items=[
                        "Block Stacking",
                        "Simple State Machine",
                        "Simple Decider Network",
                        "Peck State Machine",
                        "Peck Decider Network",
                        "Peck Game",
                    ],
                    on_clicked_fn=self.__on_selected_behavior_changed,
                )
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

        self.build_extra_frames()

    def build_extra_frames(self) -> None:
        """Builds additional UI frames for task control and diagnostics.

        Creates collapsible frames containing task control buttons and diagnostic displays
        for monitoring the Cortex behavior execution.
        """
        extra_stacks = self.get_extra_frames_handle()

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Task Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                # style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_task_controls_ui()

            with ui.CollapsableFrame(
                title="Diagnostic",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                # style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):

                self.build_diagnostic_ui()

    def _on_load_world(self) -> None:
        """Handles loading the world with the selected behavior.

        Sets the behavior for the sample based on the current selection and marks
        the world as loaded before calling the parent class load world functionality.
        """
        self._sample.behavior = self.get_behavior()
        self.loaded = True
        super()._on_load_world()

    def on_diagnostics(self, diagnostic: str, decision_stack: str) -> None:
        """Handles diagnostic updates from the Cortex framework.

        Updates the diagnostic and decision stack displays in the UI and controls
        the visibility of the diagnostics panel based on diagnostic content.

        Args:
            diagnostic: Diagnostic message from the Cortex framework.
            decision_stack: Current decision stack information from Cortex.
        """
        if diagnostic:
            self.diagostic_model.set_value(diagnostic)

        self.state_model.set_value(decision_stack)
        self.diagnostics_panel.visible = bool(diagnostic)

    def get_world(self) -> CortexWorld:
        """Current CortexWorld instance.

        Returns:
            The singleton instance of CortexWorld.
        """
        return CortexWorld.instance()

    def get_behavior(self) -> str:
        """File path of the currently selected behavior.

        Returns:
            The file path to the selected behavior script.
        """
        return self.behavior_map[self.selected_behavior]

    def _on_start_button_event(self) -> None:
        """Handles the start button click event.

        Starts the Cortex behavior execution asynchronously and disables the start button
        to prevent multiple simultaneous executions.
        """
        asyncio.ensure_future(self.sample.on_event_async())
        self.task_ui_elements["Start"].enabled = False
        return

    def post_reset_button_event(self) -> None:
        """Handles post-reset button event processing.

        Re-enables the start button after a reset operation to allow starting
        the behavior again.
        """
        self.task_ui_elements["Start"].enabled = True
        return

    def post_load_button_event(self) -> None:
        """Handles post-load button event processing.

        Re-enables the start button after the world has been loaded to allow
        starting the behavior execution.
        """
        self.task_ui_elements["Start"].enabled = True
        return

    def post_clear_button_event(self) -> None:
        """Handles post-clear button event processing.

        Disables the start button after clearing the world to prevent starting
        behaviors without a loaded environment.
        """
        self.task_ui_elements["Start"].enabled = False
        return

    def __on_selected_behavior_changed(self, selected_index: str) -> None:
        """Handles behavior selection changes in the dropdown.

        Updates the selected behavior and reloads it if a world is already loaded.
        Clears any existing diagnostics when switching behaviors.

        Args:
            selected_index: The name of the newly selected behavior.
        """
        self.selected_behavior = selected_index
        if self.loaded:
            asyncio.ensure_future(self._sample.load_behavior(self.get_behavior()))
            self.on_diagnostics("", "")

    def build_task_controls_ui(self) -> None:
        """Builds the task control UI elements.

        Creates a Start button for initiating the selected Cortex behavior.
        """
        with ui.VStack(spacing=5):
            dict = {
                "label": "Start",
                "type": "button",
                "text": "Start",
                "tooltip": "Start",
                "on_clicked_fn": self._on_start_button_event,
            }
            self.task_ui_elements["Start"] = btn_builder(**dict)
            self.task_ui_elements["Start"].enabled = False

    def build_diagnostic_ui(self) -> None:
        """Builds the diagnostic UI elements.

        Creates text fields for displaying the decision stack and diagnostic messages
        from the running Cortex behavior.
        """
        with ui.VStack(spacing=5):
            ui.Label("Decision Stack", height=20)
            self.state_model = ui.SimpleStringModel()
            ui.StringField(self.state_model, multiline=True, height=120)
            self.diagnostics_panel = ui.VStack(spacing=5)
            with self.diagnostics_panel:
                ui.Label("Diagnostic message", height=20)
                self.diagostic_model = ui.SimpleStringModel()
                ui.StringField(self.diagostic_model, multiline=True, height=200)
