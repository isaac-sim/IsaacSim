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

"""Interactive tutorial extension for getting started with robots in Isaac Sim, demonstrating manipulator and mobile robot operations."""

import os

import carb.eventdispatcher
import omni.ext
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.examples.base.base_sample_extension_experimental import BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.getting_started.start_with_robot import GettingStartedRobot
from isaacsim.gui.components.ui_utils import btn_builder


class GettingStartedRobotExtension(omni.ext.IExt):
    """Extension that provides an interactive tutorial for getting started with robots in Isaac Sim.

    This extension implements the "Part II: Robot" tutorial from the Getting Started documentation series.
    It creates a UI that guides users through adding manipulators and mobile robots to a scene, controlling
    them, and monitoring their states. The tutorial covers working with Franka Panda manipulators and
    Nova Carter mobile robots, demonstrating basic robot operations like positioning, movement, and state
    querying.

    The extension registers itself with the examples browser under the "Tutorials" category and provides
    a comprehensive interactive learning experience for robot fundamentals in Isaac Sim.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the Getting Started Robot extension.

        Sets up the UI components, creates the sample instance, and registers the example
        with the examples browser.

        Args:
            ext_id: The extension identifier.
        """
        self.example_name = "Part II: Robot"
        self.category = "Tutorials"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Getting Started with a Robot",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim_robot.html",
            "overview": "This Example follows the 'Getting Started' tutorials from the documentation\n\n 'Reset' Button is disabled. to Restart, click on the thumbnail in the browser instead.\n\n Press the 'Open in IDE' button to view the source code.",
            "sample": GettingStartedRobot(),
        }

        self.ui_handle = GettingStartedRobotUI(**ui_kwargs)

        # register the example with examples browser
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=self.ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Clean up and shutdown the Getting Started Robot extension.

        Performs UI cleanup and deregisters the example from the examples browser.
        """
        self.ui_handle.on_shutdown()
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)


class GettingStartedRobotUI(BaseSampleUITemplate):
    """UI interface for the Getting Started with a Robot tutorial.

    Provides an interactive UI that guides users through adding and controlling robots in Isaac Sim.
    Users can add a Franka Panda manipulator and a Nova Carter mobile robot to the scene,
    control their movements, and print joint states. The interface includes buttons for adding
    robots, moving them, and monitoring their states with timeline event handling.

    The UI automatically manages button states based on timeline events, disabling movement
    controls when the timeline is stopped and enabling them during playback. It also handles
    robot initialization and provides feedback through button text updates.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = None

    def build_ui(self) -> None:
        """Overwriting the build_ui function to add timeline callbacks that only registers when the tutorial is clicked on and UI is built."""
        self.arm_handle = None
        self.car_handle = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._timeline_play_callback_fn,
            observer_name="isaacsim.examples.interactive.getting_started.GettingStartedRobot._timeline_play_callback",
        )
        self._event_timer_callback_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._timeline_stop_callback_fn,
            observer_name="isaacsim.examples.interactive.getting_started.GettingStartedRobot._timeline_stop_callback",
        )
        super().build_ui()

    def build_extra_frames(self) -> None:
        """Builds the extra frames section containing the Getting Started with a Robot collapsible frame."""
        extra_stacks = self.get_extra_frames_handle()
        self.task_ui_elements = {}
        with extra_stacks:
            with ui.CollapsableFrame(
                title="Getting Started with a Robot",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                # style=get_style(),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_getting_started_ui()

    def build_getting_started_ui(self) -> None:
        """Builds the UI elements for the Getting Started with a Robot tutorial including buttons for adding and controlling arm and vehicle robots."""
        with ui.VStack(spacing=5):

            dict = {
                "label": "Add A Manipulator",
                "type": "button",
                "text": "Add Arm",
                "tooltip": "Add a manipulator (Franka Panda) to scene.",
                "on_clicked_fn": self._add_arm,
            }
            self.task_ui_elements["Add Arm"] = btn_builder(**dict)
            # self.task_ui_elements["Add Arm"].enabled= False

            dict = {
                "label": "Add A Mobile Robot",
                "type": "button",
                "text": "Add Vehicle",
                "tooltip": "Add a mobile robot to scene",
                "on_clicked_fn": self._add_vehicle,
            }
            self.task_ui_elements["Add Vehicle"] = btn_builder(**dict)
            # self.task_ui_elements["Add Vehicle"].enabled = False

            dict = {
                "label": "Move Arm",
                "type": "button",
                "text": "Move Arm",
                "tooltip": "Move the manipulator",
                "on_clicked_fn": self._move_arm,
            }
            self.task_ui_elements["Move Arm"] = btn_builder(**dict)
            self.task_ui_elements["Move Arm"].enabled = False

            dict = {
                "label": "Move Vehicle",
                "type": "button",
                "text": "Move Vehicle",
                "tooltip": "Move the mobile robot",
                "on_clicked_fn": self._move_vehicle,
            }
            self.task_ui_elements["Move Vehicle"] = btn_builder(**dict)
            self.task_ui_elements["Move Vehicle"].enabled = False

            dict = {
                "label": "Print State",
                "type": "button",
                "text": "Print Joint State",
                "tooltip": "Print the state of the robot",
                "on_clicked_fn": self._print_state,
            }
            self.task_ui_elements["Print Joint State"] = btn_builder(**dict)
            self.task_ui_elements["Print Joint State"].enabled = False

    def _add_arm(self) -> None:
        """Adds a Franka Panda manipulator to the scene at the specified position and creates an articulation handle for it."""
        import carb
        from isaacsim.core.experimental.prims import Articulation, XformPrim
        from isaacsim.core.experimental.utils import stage as stage_utils
        from isaacsim.storage.native import get_assets_root_path

        if self._timeline.is_playing():
            print("Timeline is playing. Stop the timeline to add robot to avoid collision")
            self._timeline.stop()
        else:

            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
            usd_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
            prim_path = "/World/Arm"

            # Add robot using experimental stage utils
            stage_utils.add_reference_to_stage(
                usd_path=usd_path,
                path=prim_path,
            )

            # Set position at USD level before creating articulation
            arm_xform = XformPrim(prim_path)
            arm_xform.set_world_poses(positions=[[0.0, -1.0, 0.0]])

            self.arm_handle = Articulation(prim_path)
            self.sample.arm_handle = self.arm_handle

            self.task_ui_elements["Move Arm"].text = "PRESS PLAY"
            self.task_ui_elements["Add Arm"].enabled = False

    def _add_vehicle(self) -> None:
        """Adds a Nova Carter mobile robot to the scene and creates an articulation handle for it."""
        import carb
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.experimental.utils import stage as stage_utils
        from isaacsim.storage.native import get_assets_root_path

        if self._timeline.is_playing():
            print("Timeline is playing. Stop the timeline to add robot to avoid collision")
            self._timeline.stop()
        else:
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
            usd_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
            prim_path = "/World/Car"

            # Add vehicle using experimental stage utils
            stage_utils.add_reference_to_stage(
                usd_path=usd_path,
                path=prim_path,
            )

            # add vehicle articulation
            self.car_handle = Articulation(prim_path)
            self.sample.car_handle = self.car_handle

            self.task_ui_elements["Move Vehicle"].text = "PRESS PLAY"
            self.task_ui_elements["Add Vehicle"].enabled = False

    def _move_arm(self) -> None:
        """Toggles the arm between a moved position and reset position by setting DOF positions."""
        if self.task_ui_elements["Move Arm"].text.upper() == "MOVE ARM":
            # move the arm
            self.arm_handle.set_dof_positions([[-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04]])

            # toggle btn
            self.task_ui_elements["Move Arm"].text = "RESET ARM"

        elif self.task_ui_elements["Move Arm"].text.upper() == "RESET ARM":
            # reset the arm to default position
            self.arm_handle.set_dof_positions([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            # toggle btn
            self.task_ui_elements["Move Arm"].text = "MOVE ARM"
        else:
            pass

    def _move_vehicle(self) -> None:
        """Toggles the vehicle between moving and stopped states by setting DOF velocities."""
        if self.task_ui_elements["Move Vehicle"].text.upper() == "MOVE VEHICLE":
            # move the vehicle
            self.car_handle.set_dof_velocities([[2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]])
            # toggle btn
            self.task_ui_elements["Move Vehicle"].text = "STOP VEHICLE"

        elif self.task_ui_elements["Move Vehicle"].text.upper() == "STOP VEHICLE":
            # stop the vehicle
            self.car_handle.set_dof_velocities([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

            # toggle btn
            self.task_ui_elements["Move Vehicle"].text = "MOVE VEHICLE"
        else:
            pass

    def _print_state(self) -> None:
        """Toggles the robot joint state printing functionality and updates the button text accordingly."""
        self.sample.print_state = not self.sample.print_state  # toggle print state
        if self.sample.print_state:
            self.task_ui_elements["Print Joint State"].text = "STOP PRINTING"
        else:
            self.task_ui_elements["Print Joint State"].text = "PRINT JOINT STATE"

    def _timeline_stop_callback_fn(self, event: object) -> None:
        """Timeline stop event callback - reset buttons when pressed STOP.

        Args:
            event: The timeline stop event.
        """
        if self.car_handle is not None:
            self.task_ui_elements["Move Vehicle"].enabled = False
            self.task_ui_elements["Move Vehicle"].text = "PRESS PLAY"
        if self.arm_handle is not None:
            self.task_ui_elements["Move Arm"].enabled = False
            self.task_ui_elements["Move Arm"].text = "PRESS PLAY"
        self.sample.print_state = False
        self.task_ui_elements["Print Joint State"].enabled = False
        self.task_ui_elements["Print Joint State"].text = "PRESS PLAY"

    def _timeline_play_callback_fn(self, event: object) -> None:
        """Timeline play event callback - enable buttons when pressed PLAY.

        Args:
            event: The timeline play event.
        """
        if self.car_handle is not None:
            self.task_ui_elements["Move Vehicle"].enabled = True
            self.task_ui_elements["Move Vehicle"].text = "MOVE VEHICLE"
        if self.arm_handle is not None:
            self.task_ui_elements["Move Arm"].enabled = True
            self.task_ui_elements["Move Arm"].text = "MOVE ARM"
        if self.car_handle or self.arm_handle:
            self.task_ui_elements["Print Joint State"].enabled = True
            if self.sample.print_state:
                self.task_ui_elements["Print Joint State"].text = "STOP PRINTING"
            else:
                self.task_ui_elements["Print Joint State"].text = "PRINT JOINT STATE"

    def on_shutdown(self) -> None:
        """Clean up on shutdown."""
        self._event_timer_callback_play = None
        self._event_timer_callback_stop = None
        super().on_shutdown()
