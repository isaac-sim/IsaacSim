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

"""Extension providing an interactive UI for the UR10 Palletizing example using Cortex behaviors."""

import asyncio
import os

import omni.ext
import omni.ui as ui
from isaacsim.cortex.examples.base_sample import BaseSampleUITemplate
from isaacsim.cortex.examples.ur10_palletizing.ur10_palletizing import BinStacking
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, cb_builder, str_builder


class BinStackingExtension(omni.ext.IExt):
    """Extension providing an interactive UI for the UR10 Palletizing example using Cortex behaviors.

    This extension demonstrates robotic palletizing operations using a UR10 robot arm with Cortex framework
    behaviors. It creates an interactive window with task controls for starting palletizing operations and
    diagnostic tools for monitoring the robot's decision stack, bin selection, grasp status, and attachment
    state during execution.

    The extension integrates with the Isaac Sim examples browser and provides a complete UI interface for
    controlling and observing the palletizing workflow, including real-time feedback on the robot's current
    actions and status.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the UR10 Palletizing extension and registers it with the examples browser.

        Sets up the UI template, creates the BinStacking sample instance, and registers the example
        in the examples browser under the "Cortex" category.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "UR10 Palletizing"
        self.category = "Cortex"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "UR10 Palletizing",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_5_ur10_bin_stacking.html",
            "overview": "This Example shows how to do Palletizing using UR10 robot and Cortex behaviors in Isaac Sim.\n\nPress the 'Open in IDE' button to view the source code.",
        }

        ui_handle = BinStackingUI(**ui_kwargs)
        ui_handle.sample = BinStacking(ui_handle.on_diagnostics)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Cleans up the extension by deregistering the UR10 Palletizing example from the examples browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class BinStackingUI(BaseSampleUITemplate):
    """User interface for the UR10 Palletizing example.

    Provides interactive controls and real-time diagnostics for the bin stacking task using a UR10 robot
    and Cortex behaviors. The UI includes task control buttons to start palletizing operations and diagnostic
    panels that display the decision stack, selected bin information, grasp status, attachment state, and
    flip requirements.

    The interface automatically updates diagnostic information during task execution, showing the current
    bin being processed, the robot's grasp state, and whether objects need to be flipped. Task controls
    are enabled or disabled based on the current simulation state.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.decision_stack = ""

    def build_extra_frames(self) -> None:
        """Builds additional UI frames for task control and diagnostics.

        Creates two collapsible frames: Task Control and Diagnostic frames with their respective UI elements.
        """
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}

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

    def on_diagnostics(self, diagnostic: object, decision_stack: str) -> None:
        """Handles diagnostic updates from the bin stacking task.

        Updates the UI with current bin selection, decision stack, and various diagnostic states including grasp status and attachment information.

        Args:
            diagnostic: Diagnostic information containing bin details and current states.
            decision_stack: String representation of the current decision making stack.
        """
        if self.decision_stack != decision_stack:
            self.decision_stack = decision_stack
            if decision_stack:
                decision_stack = "\n".join(
                    [
                        "{0}{1}".format("  " * (i + 1) if i > 0 else "", element)
                        for i, element in enumerate(decision_stack.replace("]", "").split("["))
                    ]
                )
            self.state_model.set_value(decision_stack)
        if diagnostic.bin_name:
            self.selected_bin.set_value(str(diagnostic.bin_name))
            self.bin_base.set_value(str(diagnostic.bin_base.prim_path))
            self.grasp_reached.set_value(diagnostic.grasp_reached)
            self.is_attached.set_value(diagnostic.attached)
            self.needs_flip.set_value(diagnostic.needs_flip)
        else:
            self.selected_bin.set_value("No Bin Selected")
            self.bin_base.set_value("")
            self.grasp_reached.set_value(False)
            self.is_attached.set_value(False)
            self.needs_flip.set_value(False)

    def get_world(self) -> CortexWorld:
        """Get the Cortex World instance.

        Returns:
            The current CortexWorld singleton instance.
        """
        return CortexWorld.instance()

    def _on_start_button_event(self) -> None:
        """Handles the start button click event.

        Starts the palletizing task asynchronously and disables the start button to prevent multiple starts.
        """
        asyncio.ensure_future(self.sample.on_event_async())
        self.task_ui_elements["Start Palletizing"].enabled = False
        return

    def post_reset_button_event(self) -> None:
        """Handles the post-reset button event.

        Re-enables the start palletizing button after the scene has been reset.
        """
        self.task_ui_elements["Start Palletizing"].enabled = True
        return

    def post_load_button_event(self) -> None:
        """Handles the post-load button event.

        Re-enables the start palletizing button after the scene has been loaded.
        """
        self.task_ui_elements["Start Palletizing"].enabled = True
        return

    def post_clear_button_event(self) -> None:
        """Handles the post-clear button event.

        Disables the start palletizing button after the scene has been cleared.
        """
        self.task_ui_elements["Start Palletizing"].enabled = False
        return

    def build_task_controls_ui(self) -> None:
        """Builds the task controls UI elements.

        Creates the start palletizing button within a vertical stack layout. The button is initially disabled.
        """
        with ui.VStack(spacing=5):

            dict = {
                "label": "Start Palletizing",
                "type": "button",
                "text": "Start Palletizing",
                "tooltip": "Start Palletizing",
                "on_clicked_fn": self._on_start_button_event,
            }

            self.task_ui_elements["Start Palletizing"] = btn_builder(**dict)
            self.task_ui_elements["Start Palletizing"].enabled = False

    def build_diagnostic_ui(self) -> None:
        """Builds the diagnostic UI elements.

        Creates UI components to display decision stack, selected bin information, bin base path, and various boolean states like grasp reached, attachment status, and flip requirement.
        """
        with ui.VStack(spacing=5):
            ui.Label("Decision Stack", height=20)
            self.state_model = ui.SimpleStringModel()
            ui.StringField(self.state_model, multiline=True, height=120)
            self.selected_bin = str_builder("Selected Bin", "<No Bin Selected>", read_only=True)
            self.bin_base = str_builder("Bin Base", "", read_only=True)
            self.grasp_reached = cb_builder("Grasp Reached", False)
            self.is_attached = cb_builder("Is Attached", False)
            self.needs_flip = cb_builder("Needs Flip", False)
