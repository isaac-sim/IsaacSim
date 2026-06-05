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

"""Interactive example demonstrating surface gripper simulation and control in Isaac Sim."""

import asyncio
import os

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni
import omni.ext
import omni.kit.app
import omni.physics.tensors as physics
import omni.ui as ui
import usd.schema.isaac.robot_schema as robot_schema
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import (
    add_separator,
    btn_builder,
    get_style,
    setup_ui_headers,
    state_btn_builder,
)
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH

EXTENSION_NAME = "Surface Gripper"


class Extension(omni.ext.IExt):
    """Interactive example demonstrating surface gripper simulation in Isaac Sim.

    This extension provides a complete interactive example for working with surface grippers (suction-cup grippers)
    in Isaac Sim. It demonstrates how to create, configure, and control a surface gripper that can attach to objects
    through simulated suction by creating joints between the gripper and target objects when they are in close proximity.

    The extension creates a user interface that allows users to:
    - Load a pre-configured scene with a gantry system containing a surface gripper and objects to manipulate
    - Control the gripper state (open/close) interactively
    - Monitor which objects are currently gripped by the surface gripper
    - Visualize the gripper behavior in real-time during simulation

    The surface gripper is implemented using USD prims with specific schema definitions and is managed through
    the SurfaceGripperManager interface. The gripper behavior is controlled by configurable parameters such as
    maximum grip distance, force limits, and retry intervals.

    Key features:
    - Interactive UI for gripper control and monitoring
    - Real-time status updates showing gripped objects
    - Configurable gripper parameters (grip distance, force limits)
    - Integration with Isaac Sim's physics simulation
    - Example scene with gantry system and pickable objects

    The extension serves as both a functional tool for testing surface gripper behavior and an educational
    resource demonstrating best practices for implementing custom gripper systems in Isaac Sim.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        self._ext_id = ext_id

        # Loads interfaces
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = None
        self._window = None
        self._models = {}

        # register the example with examples browser
        get_browser_instance().register_example(
            name=EXTENSION_NAME, execute_entrypoint=self.build_window, ui_hook=self.build_ui, category="Manipulation"
        )

        self.surface_gripper = None
        self._stage_id = -1
        self._physics_callback_id = None

    def build_window(self) -> None:
        """Build the extension window (no-op, UI is built in `build_ui`)."""

    def build_ui(self) -> None:
        """Build the surface gripper example UI panel."""
        self._usd_context = omni.usd.get_context()
        if self._usd_context is not None:
            self._stage_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_update_ui,
                observer_name="isaacsim.examples.interactive.surface_gripper.Extension._on_update_ui",
            )

        with ui.VStack(spacing=5, height=0):
            title = "Surface Gripper Example"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/ext_isaacsim_robot_surface_gripper.html"

            overview = "This Example shows how to simulate a suction-cup gripper in Isaac Sim. \n"
            overview += "It simulates suction by creating a Joint between two bodies when the parent and child bodies are close at the gripper's point of contact.\n"
            overview += "The gripper is defined as a USD prim and its behavior is managed by the SurfaceGripperManager. You can interact with it by calling the manager to Open/Close, and get its status. \n"
            overview += "Additional information can be directly extracted from the USD prim.\n"
            overview += "\n\nPress the 'Open in IDE' button to view the source code."

            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview, info_collapsed=False)

            frame = ui.CollapsableFrame(
                title="Command Panel",
                height=0,
                collapsed=False,
                style=get_style(),
                style_type_name_override="CollapsableFrame",
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            with frame:
                with ui.VStack(style=get_style(), spacing=5):
                    with ui.Frame(height=LABEL_HEIGHT):
                        args = {
                            "label": "Load Scene",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Load a gripper into the Scene",
                            "on_clicked_fn": self._on_create_scenario_button_clicked,
                        }
                        self._models["create_button"] = btn_builder(**args)
                    with ui.Frame(height=LABEL_HEIGHT):
                        args = {
                            "label": "Gripper State",
                            "type": "button",
                            "a_text": "Close",
                            "b_text": "Open",
                            "tooltip": "Open and Close the Gripper",
                            "on_clicked_fn": self._on_toggle_gripper_button_clicked,
                        }
                        self._models["toggle_button"] = state_btn_builder(**args)
                    with ui.HStack():
                        ui.Label("Gripped Objects", width=LABEL_WIDTH, height=LABEL_HEIGHT * 5)
                        self._models["gripped_objects"] = ui.SimpleStringModel()
                        ui.StringField(self._models["gripped_objects"])
                    add_separator()

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
        self._stage_event_sub = None
        self._window = None
        get_browser_instance().deregister_example(name=EXTENSION_NAME, category="Manipulation")

    def _on_update_ui(self, widget: object) -> None:
        self._models["create_button"].enabled = app_utils.is_playing()
        self._models["toggle_button"].enabled = app_utils.is_playing()
        # If the scene has been reloaded, reset UI to create Scenario
        if self._usd_context.get_stage_id() != self._stage_id:
            self._models["create_button"].enabled = True
            # self._models["create_button"].text = "Create Scenario"
            self._models["create_button"].set_tooltip(
                "Creates a new scenario with a gantry containing a surface gripper and some cubes to pick up."
            )
            self._models["create_button"].set_clicked_fn(self._on_create_scenario_button_clicked)
            self._stage_id = -1

    def _toggle_gripper_button_ui(self) -> None:
        # Checks if the surface gripper has been created
        status = self.gripper_interface.get_gripper_status(self.gripper_prim_path)
        if status == surface_gripper.GripperStatus.Open:
            self._models["toggle_button"].text = "OPEN"
        else:
            self._models["toggle_button"].text = "CLOSED"

    def _on_simulation_step(self, step: float, context: object) -> None:
        # Checks if the simulation is playing, and if the stage has been loaded
        if app_utils.is_playing() and self._stage_id != -1:
            self._toggle_gripper_button_ui()
            objects = self.gripper_interface.get_gripped_objects(self.gripper_prim_path)
            self._models["gripped_objects"].set_value("\n".join(objects))

    def _on_reset_scenario_button_clicked(self) -> None:
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
        self._on_create_scenario_button_clicked()

    async def _create_scenario(self, task: asyncio.Task[object]) -> None:
        done, pending = await asyncio.wait({task})
        if task in done:
            # Repurpose button to reset Scene
            self._models["create_button"].text = "Reset Scene"
            self._models["create_button"].set_tooltip("Resets scenario")

            # Get Handle for stage and stage ID to check if stage was reloaded
            self._stage = self._usd_context.get_stage()
            self._stage_id = self._usd_context.get_stage_id()
            app_utils.stop()
            self._models["create_button"].set_clicked_fn(self._on_reset_scenario_button_clicked)

            self.gripper_prim_path = "/World/SurfaceGripper"
            self.gripper_interface = surface_gripper.acquire_surface_gripper_interface()
            self.gripper_interface.set_write_to_usd(True)

            # Create the Surface Gripper Prim
            # This prim could be already defined in the stage,
            # but creating it in code instead to demonstrate how to do it.
            # Once it is created it can be saved and this doesn't need to be redone
            robot_schema.CreateSurfaceGripper(self._stage, self.gripper_prim_path)
            gripper_prim = self._stage.GetPrimAtPath(self.gripper_prim_path)
            attachment_points_rel = gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name)

            # Select the joints to the gripper
            # The joints should be D6 joints defined in the usd file.
            # All joint attributes can be defined as desired, except for:
            # Joint Should be enabled
            # Joint Type should be D6
            # All Joint Parents should be the same Rigid body
            # Exclude from Articulation must be checked
            # No Break force/Torque should be set
            # Joint drives can be used to derive the desired joint bounce/stretch behavior
            # Enable/Disable the joint DoFs and limits as desired.

            gripper_joints = [
                p.GetPath() for p in self._stage.GetPrimAtPath("/World/Surface_Gripper_Joints").GetChildren()
            ]
            attachment_points_rel.SetTargets(gripper_joints)

            # Define the distance the joint can grasp, and at what distance from the origin of the joints it will settle
            gripper_prim.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(0.011)
            # Define the Override Break limits
            gripper_prim.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(0.005)
            gripper_prim.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(5)

            # How long the gripper will try to close if it is open
            gripper_prim.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(1.0)

            # Select the gripper on the stage, and set the camera view to look at the machine
            selection = omni.usd.get_context().get_selection()
            selection.set_selected_prim_paths([self.gripper_prim_path], False)

            self.gripper_start_pose = physics.Transform([0, 0, 1.301], [0, 0, 0, 1])
            ViewportManager.set_camera_view(
                eye=[2.00, 2.00, 2.00], target=list(self.gripper_start_pose.p), camera="/OmniverseKit_Persp"
            )

            self._physics_callback_id = SimulationManager.register_callback(
                self._on_simulation_step, event=SimulationEvent.PHYSICS_POST_STEP
            )
            app_utils.play()

    def _on_create_scenario_button_clicked(self) -> None:
        # wait for new stage before creating scenario
        # Load the gantry USD scene
        async def load_gantry_scene() -> None:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_path = ext_manager.get_extension_path(self._ext_id)
            usd_path = os.path.join(ext_path, "data", "SurfaceGripper_gantry.usda")
            await omni.usd.get_context().new_stage_async()
            stage = omni.usd.get_context().get_stage()
            stage.DefinePrim("/World", "Xform").GetReferences().AddReference(usd_path)
            stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        task = asyncio.ensure_future(load_gantry_scene())
        asyncio.ensure_future(self._create_scenario(task))

    def _on_toggle_gripper_button_clicked(self, val: bool = False) -> None:
        """Toggles the surface gripper between open and closed states.

        When the timeline is playing, checks the current gripper status and switches it to the opposite state.
        If the gripper is open, it will be closed. If the gripper is closed, it will be opened.

        Args:
            val: Boolean value passed from the UI button click event.
        """
        if app_utils.is_playing():
            status = self.gripper_interface.get_gripper_status(self.gripper_prim_path)
            if status == surface_gripper.GripperStatus.Open:
                self.gripper_interface.close_gripper(self.gripper_prim_path)
            else:
                self.gripper_interface.open_gripper(self.gripper_prim_path)
