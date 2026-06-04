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

"""Extension for replaying data from the Follow Target manipulation task with UI controls for loading and managing recorded robot trajectories and scene states."""

import asyncio
import os

import omni.ext
import omni.kit.app
import omni.ui as ui
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, str_builder
from isaacsim.robot.experimental.manipulators.examples.interactive.replay_follow_target import ReplayFollowTarget


class ReplayFollowTargetExtension(omni.ext.IExt):
    """Extension for demonstrating data logging replay functionality with the Follow Target task.

    This extension provides a UI interface for replaying previously recorded data from the Follow Target
    manipulation task. It demonstrates how to use Isaac Sim's data logging capabilities to capture and replay
    robot trajectories and scene states, enabling users to analyze and reproduce recorded behaviors.

    The extension integrates with the Isaac Sim examples browser and offers controls for loading data files
    and replaying either trajectory data or complete scene states. It serves as an educational tool for
    understanding advanced data logging techniques in robotics simulations.

    The extension is automatically registered in the "Manipulation" category of the examples browser when
    activated.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Replay Follow Target extension.

        Sets up the example configuration and registers it with the browser instance to make it available
        in the Isaac Sim examples interface.

        Args:
            ext_id: The extension identifier string.
        """
        self.example_name = "Replay Follow Target"
        self.category = "Manipulation"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Replay Follow Target Task",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_advanced_data_logging.html",
            "overview": "This Example shows how to use data logging to replay data collected\n\n from the follow target extension example.\n\n Press the 'Open in IDE' button to view the source code.",
            "sample": ReplayFollowTarget(),
        }

        ui_handle = ReplayFollowTargetUI(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self) -> None:
        """Cleans up the Replay Follow Target extension.

        Deregisters the example from the browser instance when the extension is shut down.
        """
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class ReplayFollowTargetUI(BaseSampleUITemplate):
    """UI template for the Replay Follow Target example.

    This class creates a user interface for replaying data collected from the follow target extension example.
    It provides controls for loading data files and replaying both trajectory and scene data through
    interactive UI elements.

    The UI includes a collapsible Task Control frame with:
    - Data file selection field for specifying the replay data source
    - Replay Trajectory button for replaying recorded trajectory data
    - Replay Scene button for replaying the complete scene data

    The interface manages button states based on the current sample state, enabling replay controls
    after loading and resetting, while disabling them during playback operations and when no sample
    is loaded.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

    def build_extra_frames(self) -> None:
        """Builds the additional UI frames for the replay follow target interface.

        Creates a collapsable frame containing task control elements including data logging UI components.
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
                self.build_data_logging_ui()

    def _on_replay_trajectory_button_event(self) -> None:
        """Handles the replay trajectory button click event.

        Starts asynchronous replay of trajectory data from the specified file and disables both replay buttons during execution.
        """
        asyncio.ensure_future(
            self.sample._on_replay_trajectory_event_async(self.task_ui_elements["Data File"].get_value_as_string())
        )
        self.task_ui_elements["Replay Trajectory"].enabled = False
        self.task_ui_elements["Replay Scene"].enabled = False
        return

    def _on_replay_scene_button_event(self) -> None:
        """Handles the replay scene button click event.

        Starts asynchronous replay of scene data from the specified file and disables both replay buttons during execution.
        """
        asyncio.ensure_future(
            self.sample._on_replay_scene_event_async(self.task_ui_elements["Data File"].get_value_as_string())
        )
        self.task_ui_elements["Replay Trajectory"].enabled = False
        self.task_ui_elements["Replay Scene"].enabled = False
        return

    def post_reset_button_event(self) -> None:
        """Handles post-reset button event actions.

        Re-enables both replay trajectory and replay scene buttons after a reset operation.
        """
        self.task_ui_elements["Replay Trajectory"].enabled = True
        self.task_ui_elements["Replay Scene"].enabled = True
        return

    def post_load_button_event(self) -> None:
        """Handles post-load button event actions.

        Re-enables both replay trajectory and replay scene buttons after a load operation.
        """
        self.task_ui_elements["Replay Trajectory"].enabled = True
        self.task_ui_elements["Replay Scene"].enabled = True
        return

    def post_clear_button_event(self) -> None:
        """Handles post-clear button event actions.

        Disables both replay trajectory and replay scene buttons after a clear operation.
        """
        self.task_ui_elements["Replay Trajectory"].enabled = False
        self.task_ui_elements["Replay Scene"].enabled = False
        return

    def build_data_logging_ui(self) -> None:
        """Builds the data logging user interface components.

        Creates UI elements including a data file input field and replay buttons for trajectory and scene replay functionality. Both replay buttons are initially disabled.
        """
        with ui.VStack(spacing=5):
            mgr = omni.kit.app.get_app().get_extension_manager()
            ext_id = mgr.get_enabled_extension_id("isaacsim.robot.experimental.manipulators.examples")
            ext_root = mgr.get_extension_path(ext_id)
            example_data_file = os.path.join(ext_root, "data", "example_data_file.json")
            dict = {
                "label": "Data File",
                "type": "stringfield",
                "default_val": example_data_file,
                "tooltip": "Data File",
                "on_clicked_fn": None,
                "use_folder_picker": False,
                "read_only": False,
            }
            self.task_ui_elements["Data File"] = str_builder(**dict)
            dict = {
                "label": "Replay Trajectory",
                "type": "button",
                "text": "Replay Trajectory",
                "tooltip": "Replay Trajectory",
                "on_clicked_fn": self._on_replay_trajectory_button_event,
            }

            self.task_ui_elements["Replay Trajectory"] = btn_builder(**dict)
            self.task_ui_elements["Replay Trajectory"].enabled = False
            dict = {
                "label": "Replay Scene",
                "type": "button",
                "text": "Replay Scene",
                "tooltip": "Replay Scene",
                "on_clicked_fn": self._on_replay_scene_button_event,
            }

            self.task_ui_elements["Replay Scene"] = btn_builder(**dict)
            self.task_ui_elements["Replay Scene"].enabled = False
        return
