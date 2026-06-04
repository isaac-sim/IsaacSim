# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides a UI window for interactive robotic grasp generation and evaluation in Isaac Sim."""

import asyncio

import carb
import isaacsim.replicator.grasping.grasping_utils as grasping_utils
import isaacsim.replicator.grasping.ui.grasping_ui_utils as grasping_ui_utils
import omni.stageupdate
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.grasping import GraspingManager
from pxr import Gf, Sdf

GLYPHS = {
    "plus": ui.get_custom_glyph_code("${glyphs}/plus.svg"),
    "sync": ui.get_custom_glyph_code("${glyphs}/sync.svg"),
    "activate": ui.get_custom_glyph_code("${glyphs}/check_solid.svg"),
    "deactivate": ui.get_custom_glyph_code("${glyphs}/edit_deactivate.svg"),
    "arrow_up": ui.get_custom_glyph_code("${glyphs}/arrow_up.svg"),
    "arrow_down": ui.get_custom_glyph_code("${glyphs}/arrow_down.svg"),
    "delete": ui.get_custom_glyph_code("${glyphs}/menu_delete.svg"),
    "play": ui.get_custom_glyph_code("${glyphs}/audio_play.svg"),
    "reset": ui.get_custom_glyph_code("${glyphs}/restore.svg"),
}


class GraspingWindow(ui.Window):
    """A UI window for interactive robotic grasp generation and evaluation.

    Provides a comprehensive interface for configuring grippers, objects, grasp poses, and simulation
    parameters. Supports generating antipodal grasp poses using surface sampling, visualizing grasp
    candidates, simulating grasp execution through customizable phases, and evaluating grasp success
    metrics. The window integrates with USD stages and physics scenes to enable realistic grasp
    simulation workflows.

    The interface is organized into collapsible sections covering gripper configuration (joint
    pregrasp states and grasp phases), object selection and pose sampling parameters, visualization
    tools for grasp poses and object meshes, simulation settings (physics scenes, rendering options),
    workflow automation for batch grasp evaluation, and configuration management for saving and
    loading complete setups.

    Args:
        title: Window title displayed in the UI.
    """

    def __init__(self, title: str) -> None:
        super().__init__(title, dockPreference=ui.DockPreference.MAIN)
        self.deferred_dock_in("Property", ui.DockPolicy.DO_NOTHING)

        # UI window visibility changed listener
        self.set_visibility_changed_fn(self._visibility_changed_fn)
        self._visibility_changed_listener = None

        # Grasping manager, orchestrates the grasping workflow
        self._grasping_manager = GraspingManager()

        # Subscribe to open/close stage
        self._stage_update = omni.stageupdate.get_stage_update_interface()
        self._stage_subscription = self._stage_update.create_stage_update_node(
            display_name=self.title, on_attach_fn=self._on_stage_attach, on_detach_fn=self._on_stage_detach
        )

        self._physics_scene_path = ""
        self._isolate_grasp_simulation = False
        self._render_simulation = True
        self._simulate_using_timeline = False

        # UI
        # Keeps track of the collapsed state of the collapsable frames
        self._collapsed_states = {}

        # Joints available in the gripper, visible in the UI, and UI settings to show/hide them
        self._joint_ui_data = []
        self._joint_filter_mode_int = 0  # 0: Drive (Non-Mimic), 1: All

        # Name for adding a new grasp phase
        self._new_grasp_phase_name = ""

        # Config path to load or save the config
        self._config_path = ""

        # Visualization settings
        self._draw_poses_in_world_frame = True
        self._draw_trimesh_world_frame = True

        # Initialize default grasp phases (open, close)
        self._create_default_grasp_phases()

        # Used when when iterating over grasp poses
        self._current_grasp_pose_idx = 0

        # Grasp evaluation workflow state
        self._num_grasps_to_evaluate = -1
        self._current_evaluated_grasp_idx = 0
        self._is_workflow_running = False

        # Configurable fields for export/import
        # Maps config key (used in save/load) to UI label
        self._config_fields_labels = {
            "gripper": "Gripper Path",
            "pregrasp": "Joint Pregrasp States",
            "phases": "Grasp Phases",
            "object": "Object Path",
            "sampler": "Pose Sampler Parameters",
            "poses": "Grasp Poses",
        }
        # Holds the boolean state (checked/unchecked) for each config field key
        self._config_fields = dict.fromkeys(self._config_fields_labels, True)

        # Overwrite config file if it exists
        self._overwrite_config = False

        # Overwrite results output files if they exist
        self._overwrite_results = False

        # UI containers for partial rebuilds
        self._simulation_frame_container = None
        self._gripper_frame_container = None
        self._object_frame_container = None
        self._workflow_frame_container = None
        self._config_frame_container = None
        self._ui_initialized = False

        # Build the UI
        self._build_window_ui()
        asyncio.ensure_future(self._build_window_ui_async())

    def destroy(self) -> None:
        """Clean up all resources and destroy the grasping window.

        Clears all internal state, unsubscribes from stage events, and destroys all UI components.
        """
        self.set_visibility_changed_listener(None)
        self._clear()
        self._stage_update = None
        self._stage_subscription = None
        self._grasping_manager = None
        self._simulation_frame_container = None
        self._gripper_frame_container = None
        self._object_frame_container = None
        self._workflow_frame_container = None
        self._config_frame_container = None
        self._ui_initialized = False
        super().destroy()

    def _clear(self) -> None:
        """Clear the grasping manager state."""
        if self._grasping_manager:
            self._grasping_manager.clear()

    def _reset(self) -> None:
        """Reset the window to its initial state.

        Clears all data, resets UI state variables, creates default grasp phases, and rebuilds the UI.
        """
        self._clear()
        self._clear_gripper_joints()
        self._joint_ui_data = []
        self._joint_filter_mode_int = 0  # Reset to default
        self._collapsed_states = {}
        self._new_grasp_phase_name = ""
        self._config_path = ""

        self._create_default_grasp_phases()
        asyncio.ensure_future(self._build_window_ui_async())

    def _visibility_changed_fn(self, visible: bool) -> None:
        """Handle window visibility changes.

        Args:
            visible: Whether the window is now visible.
        """
        if self._visibility_changed_listener:
            self._visibility_changed_listener(visible)

    def set_visibility_changed_listener(self, listener: callable) -> None:
        """Set a callback function to be notified of window visibility changes.

        Args:
            listener: Callback function that will be called when window visibility changes.
        """
        self._visibility_changed_listener = listener

    def _on_stage_attach(self, stage_id: int, meters_per_unit: float) -> None:
        """Handle stage attachment events by rebuilding the UI.

        Args:
            stage_id: Identifier of the attached stage.
            meters_per_unit: The stage's meters per unit conversion factor.
        """
        asyncio.ensure_future(self._build_window_ui_async())

    def _on_stage_detach(self) -> None:
        """Handle stage detachment events by resetting the window state."""
        self._reset()

    # ==============================================================================
    # Main UI Building Methods
    # ==============================================================================
    def _build_window_ui(self) -> None:
        """Build the main window UI structure with container frames.

        Creates the scrolling frame and container stacks for different UI sections.

        Returns:
            None.
        """
        if self._ui_initialized:
            return

        with self.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5):
                    self._simulation_frame_container = ui.VStack()
                    self._gripper_frame_container = ui.VStack()
                    self._object_frame_container = ui.VStack()
                    self._workflow_frame_container = ui.VStack()
                    self._config_frame_container = ui.VStack()
        self._ui_initialized = True

    async def _build_window_ui_async(self) -> None:
        """Build all UI frames asynchronously.

        Rebuild all major UI sections including simulation, gripper, object, workflow, and config frames.
        """
        await asyncio.gather(
            self._rebuild_simulation_frame_async(),
            self._rebuild_gripper_frame_async(),
            self._rebuild_object_frame_async(),
            self._rebuild_workflow_frame_async(),
            self._rebuild_config_frame_async(),
        )

    async def _rebuild_simulation_frame_async(self) -> None:
        """Rebuild the simulation frame UI section."""
        if self._simulation_frame_container:
            self._simulation_frame_container.clear()
            with self._simulation_frame_container:
                self._build_simulation_frame()

    async def _rebuild_gripper_frame_async(self) -> None:
        """Rebuild the gripper configuration frame asynchronously.

        Clears and rebuilds the gripper frame container with updated gripper settings, joint controls, and grasp phase configurations.
        """
        if self._gripper_frame_container:
            self._gripper_frame_container.clear()
            with self._gripper_frame_container:
                self._build_gripper_frame()

    async def _rebuild_object_frame_async(self) -> None:
        """Rebuild the object configuration frame asynchronously.

        Clears and rebuilds the object frame container with updated object settings, grasp sampler parameters, and pose visualization controls.
        """
        if self._object_frame_container:
            self._object_frame_container.clear()
            with self._object_frame_container:
                self._build_object_frame()

    async def _rebuild_workflow_frame_async(self) -> None:
        """Rebuild the workflow evaluation frame asynchronously.

        Clears and rebuilds the workflow frame container with updated grasp evaluation controls and status information.
        """
        if self._workflow_frame_container:
            self._workflow_frame_container.clear()
            with self._workflow_frame_container:
                self._build_workflow_frame()

    async def _rebuild_config_frame_async(self) -> None:
        """Rebuild the configuration management frame asynchronously.

        Clears and rebuilds the config frame container with updated save/load controls and configuration field selections.
        """
        if self._config_frame_container:
            self._config_frame_container.clear()
            with self._config_frame_container:
                self._build_config_frame()

    def _rebuild_ui_if_joints_changed(self) -> None:
        """Rebuild UI frames if the gripper joints configuration has changed.

        Clears existing joint data, reloads joints from the gripper, synchronizes phase joint configurations with UI selections, and triggers asynchronous rebuilds of the gripper and workflow frames if joints were present before or after the reload.
        """
        had_joints = bool(self._joint_ui_data)
        self._clear_gripper_joints()
        self._load_gripper_joints()

        # Update grasp phases to include the new gripper joints
        self._sync_phase_joints_with_ui_selection()

        if had_joints or self._joint_ui_data:
            asyncio.ensure_future(
                asyncio.gather(self._rebuild_gripper_frame_async(), self._rebuild_workflow_frame_async())
            )

    # ==============================================================================
    # Gripper UI Building and Logic
    # ==============================================================================
    def _build_gripper_frame(self) -> None:
        """Build the gripper configuration frame UI.

        Creates UI controls for gripper path selection, joint filtering and configuration, and grasp phase management within a collapsible frame.
        """
        frame_name = "Gripper"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        gripper_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed, style=get_style())
        with gripper_frame:
            gripper_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label("Path:", tooltip="Path in stage to the gripper base frame")
                    gripper_base_path = ui.StringField()
                    gripper_base_path.model.set_value(self._grasping_manager.gripper_path)
                    gripper_base_path.model.add_value_changed_fn(self._on_gripper_base_path_changed)
                    ui.Button(
                        f"{GLYPHS['plus']}",
                        width=30,
                        clicked_fn=lambda: self._on_set_field_from_selection(gripper_base_path.model),
                        tooltip="Add path of the selected prim in stage",
                    )
                self._build_joints_frame()
                self._build_grasp_phases_frame()

    def _build_joints_frame(self) -> None:
        """Build the joints configuration frame UI.

        Creates UI controls for filtering joint visibility, configuring individual joint settings including inclusion in grasp phases, and setting pregrasp positions.
        """
        frame_name = "Joints"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        joints_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with joints_frame:
            joints_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                joint_filter_collection = ui.RadioCollection()

                with ui.HStack(height=0, spacing=5):
                    ui.Spacer(width=15)
                    ui.Label("Show Joints:", tooltip="Select which joints to display and include in phases.")

                    # Create RadioButtons and associate them with the collection and values directly
                    rb_drive_non_mimic = ui.RadioButton(
                        text="Drive (Non-Mimic)",
                        radio_collection=joint_filter_collection,
                        value=0,
                        tooltip="Show only drive joints that are not mimic joints. These are used for grasping.",
                    )

                    rb_all = ui.RadioButton(
                        text="All",
                        radio_collection=joint_filter_collection,
                        value=1,
                        tooltip="Show all joints, regardless of their drive or mimic status.",
                    )

                    # Set initial selection state using the collection's model
                    joint_filter_collection.model.set_value(self._joint_filter_mode_int)

                    # Define callback function (takes the collection's model as argument)
                    def on_joint_filter_mode_changed(model: object) -> None:  # model is the RadioCollection's model
                        """Callback when the joint filter radio button selection changes.

                        Args:
                            model: The RadioCollection model with the new selection value.
                        """
                        new_mode_int = model.as_int  # Get value from the collection's model
                        carb.log_info(
                            f"Joint filter mode changed. New item value: {new_mode_int}, current internal mode: {self._joint_filter_mode_int}"
                        )

                        if self._joint_filter_mode_int != new_mode_int:
                            self._joint_filter_mode_int = new_mode_int
                            self._rebuild_ui_if_joints_changed()
                        else:
                            carb.log_info(
                                f"Joint filter mode re-confirmed to: {new_mode_int}. No change in value, UI rebuild not re-triggered from here."
                            )

                    # Register callback to the collection's model
                    joint_filter_collection.model.add_value_changed_fn(on_joint_filter_mode_changed)

                with ui.VStack(spacing=5):
                    for joint_data in self._joint_ui_data:
                        if not joint_data["show_in_ui"]:
                            continue

                        # Use display name (relative to gripper) for UI
                        display_name = grasping_ui_utils.get_joint_display_name(
                            joint_data["path"], self._grasping_manager.gripper_path
                        )

                        with ui.CollapsableFrame(display_name, height=0, collapsed=True):
                            with ui.VStack(spacing=3):
                                # --- Include/Exclude ---
                                with ui.HStack(spacing=5):
                                    ui.Spacer(width=15)
                                    ui.Label(
                                        "Include/Exclude",
                                        width=200,
                                        tooltip="Include/exclude the joint in the grasp phases (non-drive or mimic joints cannot be included)",
                                    )
                                    include_joint = ui.CheckBox(
                                        width=10, height=0, enabled=joint_data["is_valid_grasp_joint"]
                                    )
                                    include_joint.model.set_value(joint_data["include"])
                                    include_joint.model.add_value_changed_fn(
                                        lambda model, jd=joint_data: self._on_include_joint_in_grasp_changed(model, jd)
                                    )
                                    ui.Button(
                                        "Highlight",
                                        clicked_fn=lambda path=joint_data[
                                            "path"
                                        ]: grasping_ui_utils.select_prim_in_stage(path),
                                        tooltip=f"Select the joint prim in stage.",
                                    )

                                # --- Joint Pregrasp State ---
                                with ui.HStack(spacing=5):
                                    ui.Spacer(width=15)
                                    ui.Label(
                                        "Pregrasp Position:",
                                        tooltip="Define the target joint position before the grasp simulation starts. Changing this value automatically updates the stored state.",
                                    )
                                    pregrasp_pos_field = ui.FloatField()
                                    pregrasp_pos = self._grasping_manager.joint_pregrasp_states.get(
                                        joint_data["path"], 0.0
                                    )
                                    pregrasp_pos_field.model.set_value(pregrasp_pos)
                                    pregrasp_pos_field.model.add_value_changed_fn(
                                        lambda model, path=joint_data["path"]: self._on_set_joint_pregrasp_state(
                                            model, path
                                        )
                                    )

    def _build_grasp_phases_frame(self) -> None:
        """Build the grasp phases configuration frame UI.

        Creates UI controls for managing grasp phases including phase-specific settings, adding new phases, and simulation controls for individual or all phases.
        """
        frame_name = "Grasp Phases"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        grasp_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with grasp_frame:
            grasp_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                for phase_name in self._grasping_manager.get_grasp_phase_names():
                    self._build_phase_frame(phase_name)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Label("Add New Phase:")
                    phase_name_field = ui.StringField()
                    phase_name_field.model.set_value("")
                    phase_name_field.model.add_value_changed_fn(self._on_new_phase_name_changed)
                    ui.Button(f"{GLYPHS['plus']}", width=30, clicked_fn=self._on_add_new_grasp_phase)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Button(
                        "Simulate All Grasp Phases",
                        clicked_fn=lambda: asyncio.ensure_future(self._on_simulate_all_grasp_phases_async()),
                    )
                    ui.Button("Reset Simulation", clicked_fn=self._on_reset_simulation)

    def _build_phase_frame(self, phase_name: str) -> None:
        """Build the UI frame for a specific grasp phase.

        Creates UI controls for phase management operations, simulation parameters, and joint target positions for the specified phase.

        Args:
            phase_name: Name of the grasp phase to build the frame for.

        Returns:
            None.
        """
        frame_name = f"Phase_{phase_name}"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        phase_frame = ui.CollapsableFrame(phase_name, height=0, collapsed=frame_collapsed)
        with phase_frame:
            phase_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                with ui.HStack():
                    ui.Spacer(width=15)
                    ui.Button(
                        f"{GLYPHS['arrow_up']}",
                        width=30,
                        clicked_fn=lambda p=phase_name: self._on_move_grasp_phase_up(p),
                        tooltip="Move this phase earlier in the sequence",
                    )
                    ui.Button(
                        f"{GLYPHS['arrow_down']}",
                        width=30,
                        clicked_fn=lambda p=phase_name: self._on_move_grasp_phase_down(p),
                        tooltip="Move this phase later in the sequence",
                    )
                    ui.Button(
                        f"{GLYPHS['delete']}",
                        width=30,
                        clicked_fn=lambda p=phase_name: self._on_delete_grasp_phase(p),
                        tooltip="Delete this phase",
                    )
                    ui.Button(
                        f"{GLYPHS['play']}",
                        width=30,
                        clicked_fn=lambda p=phase_name: asyncio.ensure_future(
                            self._on_simulate_single_grasp_phase_async(p)
                        ),
                        tooltip="Simulate this phase",
                    )
                    ui.Button(
                        f"{GLYPHS['reset']}",
                        width=30,
                        clicked_fn=self._on_reset_simulation,
                        tooltip="Reset simulation",
                    )

                # Get phase data from manager
                phase_data = self._grasping_manager.get_grasp_phase_by_name(phase_name)
                if not phase_data:
                    return

                with ui.HStack():
                    ui.Spacer(width=15)
                    ui.Label("Simulation Step Delta Time:")
                    simulation_step_dt = ui.FloatField()
                    simulation_step_dt.model.set_value(phase_data.simulation_step_dt)
                    simulation_step_dt.model.add_value_changed_fn(
                        lambda model, p=phase_name: self._on_simulation_step_dt_changed(model, p)
                    )
                with ui.HStack():
                    ui.Spacer(width=15)
                    ui.Label("Simulation Steps:")
                    simulation_steps = ui.IntField()
                    simulation_steps.model.set_value(phase_data.simulation_steps)
                    simulation_steps.model.add_value_changed_fn(
                        lambda model, p=phase_name: self._on_simulation_steps_changed(model, p)
                    )
                self._build_joints_target_positions_frame(phase_name)

    def _build_joints_target_positions_frame(self, phase_name: str) -> None:
        """Build the joint target positions frame UI for a specific grasp phase.

        Creates UI controls for setting target joint positions within the specified grasp phase, ensuring all included joints are represented and displaying joint-specific input fields.

        Args:
            phase_name: Name of the grasp phase to build joint target controls for.
        """
        frame_name = f"TargetPositions_{phase_name}"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        target_positions_frame = ui.CollapsableFrame("Target Positions", height=0, collapsed=frame_collapsed)
        with target_positions_frame:
            target_positions_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                phase = self._grasping_manager.get_grasp_phase_by_name(phase_name)
                if phase:
                    # Ensure all included joints are represented in the phase's targets
                    for joint_data in self._joint_ui_data:
                        if joint_data["include"] and joint_data["is_valid_grasp_joint"]:
                            absolute_path = joint_data["path"]
                            if not phase.has_joint(absolute_path):
                                # Add with default target 0.0 if missing
                                phase.add_joint(absolute_path)

                    # Display joint targets relevant to the current gripper
                    gripper_path_prefix = self._grasping_manager.gripper_path
                    joint_targets_to_display = {}
                    for absolute_path, target_position in phase.joint_drive_targets.items():
                        if absolute_path.startswith(gripper_path_prefix):
                            joint_targets_to_display[absolute_path] = target_position

                    # Display UI fields for these joints
                    for absolute_path, target_position in sorted(joint_targets_to_display.items()):
                        display_name = grasping_ui_utils.get_joint_display_name(absolute_path, gripper_path_prefix)

                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label(f"{display_name}:")
                            value_field = ui.FloatField()
                            value_field.model.set_value(target_position)

                            def on_position_changed(
                                model: object, p: object = phase, j_path: str = absolute_path
                            ) -> None:
                                # Update the target position in the specific phase object
                                if p:  # Check if phase object is still valid
                                    p.add_joint(j_path, model.as_float)

                            value_field.model.add_value_changed_fn(on_position_changed)

    def _create_default_grasp_phases(self) -> None:
        """Create default 'Open' and 'Close' grasp phases if no phases exist in the grasping manager."""
        if not self._grasping_manager.grasp_phases:
            self._grasping_manager.create_and_add_grasp_phase(name="Open")
            self._grasping_manager.create_and_add_grasp_phase(name="Close")

    def _load_gripper_joints(self) -> bool:
        """Load joint information from the current gripper and populates the UI joint data.

        Retrieves core joint information from the grasping manager's gripper prim and builds
        the internal `_joint_ui_data` structure with visibility and inclusion settings based on
        the current joint filter mode. Preserves previous UI selection states when available.

        Returns:
            True if joints were successfully loaded, False if gripper prim is not set or joint
            info could not be retrieved.
        """
        gripper_prim = self._grasping_manager.gripper_prim
        if not gripper_prim:
            if self._joint_ui_data:
                self._joint_ui_data = []
            return False

        # Preserve previous UI selection state
        previous_include_state = {joint["path"]: joint["include"] for joint in self._joint_ui_data}

        # Get core joint info from the manager - FETCH ONCE
        core_joint_info_list = grasping_utils.get_gripper_joints_info(self._grasping_manager.gripper_path)

        if core_joint_info_list is None:
            carb.log_warn(f"Failed to retrieve joint info from {self._grasping_manager.gripper_path}.")
            self._joint_ui_data = []
            return False

        new_ui_gripper_joints = []
        for core_info in core_joint_info_list:
            path = core_info["path"]
            is_mimic = core_info["is_mimic"]
            is_drive = core_info["is_drive"]
            can_be_included = core_info["is_valid_grasp_joint"]

            # Determine UI visibility based on the new filter mode
            if self._joint_filter_mode_int == 0:  # Drive (Non-Mimic)
                show_in_ui = is_drive and not is_mimic
            elif self._joint_filter_mode_int == 1:  # All
                show_in_ui = True
            else:  # Default/fallback
                carb.log_warn(
                    f"Unknown joint filter mode: {self._joint_filter_mode_int}. Defaulting to Drive (Non-Mimic)."
                )
                show_in_ui = is_drive and not is_mimic

            # Determine UI inclusion state based on capability and previous selection
            include_in_grasp = False
            if can_be_included:
                include_in_grasp = previous_include_state.get(path, True)  # Default to included

            # Build UI data structure (excluding 'prim')
            ui_joint_data = {
                "path": path,
                "is_mimic": is_mimic,
                "is_drive": is_drive,
                "is_valid_grasp_joint": can_be_included,
                "show_in_ui": show_in_ui,
                "include": include_in_grasp,
            }
            new_ui_gripper_joints.append(ui_joint_data)

        self._joint_ui_data = new_ui_gripper_joints

        # Populate manager's gripper pregrasp states, passing the already fetched info
        if self._joint_ui_data:
            self._grasping_manager.update_joint_pregrasp_states_from_current(joint_info_list=core_joint_info_list)

        return True

    def _clear_gripper_joints(self) -> None:
        """Clear all joint data from the UI, resetting the internal joint data list to empty."""
        self._joint_ui_data = []

    def _sync_phase_joints_with_ui_selection(self) -> None:
        """Synchronize the joints within each grasp phase based on the current UI selection.

        Ensures that each GraspPhase in the GraspingManager targets exactly the set
        of joints that are currently marked as 'includable' and 'include=True' in the
        UI's internal `_joint_ui_data` list and belong to the current gripper.

        - Adds joints to phases if they are selected for inclusion in the UI but missing from the phase.
        - Removes joints from phases if they are no longer selected for inclusion in the UI
          or if they do not belong to the currently loaded gripper.

        This is typically called after the gripper changes, UI visibility flags change,
        or a configuration is loaded, to ensure phase definitions match the UI state.

        Returns:
            None.
        """
        if not self._joint_ui_data:
            return

        # Get all joints that can be included in grasp phases with absolute paths
        includable_joints = [
            joint["path"] for joint in self._joint_ui_data if joint["is_valid_grasp_joint"] and joint["include"]
        ]

        # Update each phase to include the new joints
        for phase in self._grasping_manager.grasp_phases:
            # Add any missing joints
            for joint_path in includable_joints:
                if not phase.has_joint(joint_path):
                    phase.add_joint(joint_path)

            # Remove any joints that no longer exist for this gripper or are not selected
            for absolute_path in list(phase.joint_drive_targets.keys()):
                # Only check joints belonging to this gripper
                if absolute_path.startswith(self._grasping_manager.gripper_path):
                    if absolute_path not in includable_joints:
                        phase.remove_joint(absolute_path)

    def _update_ui_joint_selection_from_loaded_phases(self) -> None:
        """Update the UI's joint 'include' checkboxes based on loaded grasp phases.

        This function is primarily intended to be called after loading a configuration file.
        It examines the `joint_drive_targets` of the *first* grasp phase loaded from the
        config and sets the 'include' state of the corresponding joints in the UI's
        internal `_joint_ui_data` list. This makes the UI checkboxes reflect the
        joint selection saved in the configuration.

        Note:
            This method currently relies solely on the *first* grasp phase to determine
            which joints should be marked as included in the UI. It assumes the first
            phase in a saved configuration represents the intended set of active joints.
        """
        # Get joint paths from first grasp phase to determine which joints are included
        included_joint_paths = set()
        grasp_phases = self._grasping_manager.grasp_phases
        if grasp_phases:
            included_joint_paths = set(grasp_phases[0].joint_drive_targets.keys())

        # Update joint inclusion states and refresh UI
        core_joint_info = grasping_utils.get_gripper_joints_info(self._grasping_manager.gripper_path)
        for joint_data in core_joint_info:
            if joint_data["is_valid_grasp_joint"]:
                joint_data["include"] = joint_data["path"] in included_joint_paths

        self._rebuild_ui_if_joints_changed()

    # ==============================================================================
    # Object UI Building and Logic
    # ==============================================================================
    def _build_object_frame(self) -> None:
        """Build the UI frame for object configuration and management.

        Creates the object frame containing object path input, grasp pose sampler settings,
        grasp poses management, and trimesh visualization controls.
        """
        frame_name = "Object"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        object_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed, style=get_style())
        with object_frame:
            object_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label("Path:", tooltip="Path in stage to the object to grasp")
                    object_path = ui.StringField()
                    object_path.model.set_value(self._grasping_manager.get_object_prim_path() or "")
                    object_path.model.add_value_changed_fn(self._on_object_path_changed)
                    ui.Button(
                        f"{GLYPHS['plus']}",
                        width=30,
                        clicked_fn=lambda: self._on_set_field_from_selection(object_path.model),
                        tooltip="Add path of the selected prim in stage",
                    )

                # Build the sub-frames
                self._build_grasp_sampler_frame()
                self._build_grasp_poses_frame()
                self._build_trimesh_debug_draw_frame()

    def _build_grasp_poses_frame(self) -> None:
        """Build the UI frame for managing and visualizing grasp poses."""
        frame_name = "Grasp Poses"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        grasp_poses_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with grasp_poses_frame:
            grasp_poses_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                # Current number of loaded poses
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    loaded_poses_count = len(self._grasping_manager.grasp_locations)
                    ui.Label(f"Loaded Poses: {loaded_poses_count}")

                # Number of candidates
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Number of Candidates:",
                        tooltip="Target number of grasp candidates to attempt to sample. Influences the initial number of surface points sampled.",
                    )
                    num_candidates = ui.IntField()
                    num_candidates.model.set_value(self._grasping_manager.sampler_config["num_candidates"])
                    num_candidates.model.add_value_changed_fn(self._on_num_candidates_changed)

                # Random seed
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Random Seed:",
                        tooltip="Random seed for grasp pose generation. Set to -1 for no fixed seed (random each time).",
                    )
                    seed_field = ui.IntField()
                    seed_field.model.set_value(self._grasping_manager.sampler_config["random_seed"])
                    seed_field.model.add_value_changed_fn(self._on_random_seed_changed)

                # Button row for generating/clearing poses
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Button("Generate", clicked_fn=self._on_generate_grasp_poses)
                    ui.Button("Clear", clicked_fn=self._on_clear_grasp_poses)

                # Nested visualization frame
                self._build_grasp_pose_visualization_frame()

    def _build_grasp_sampler_frame(self) -> None:
        """Build the UI frame for grasp pose sampler configuration.

        Creates controls for sampler parameters including sampler type, number of orientations,
        gripper standoff distance, maximum aperture, axis configurations, lateral sigma,
        and verbose logging settings.
        """
        frame_name = "Grasp Pose Sampler"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        sampler_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with sampler_frame:
            sampler_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                # Sampler type
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Sampler Type:",
                        tooltip="Type of grasp sampling algorithm to use",
                    )
                    ui.Label(self._grasping_manager.sampler_config["sampler_type"], height=0)

                # Number of orientations
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Number of Orientations:",
                        tooltip="Number of different orientations to sample per valid grasp axis. Each orientation rotates around the grasp axis.",
                    )
                    num_orientations = ui.IntField()
                    num_orientations.model.set_value(self._grasping_manager.sampler_config["num_orientations"])
                    num_orientations.model.add_value_changed_fn(self._on_num_orientations_changed)

                # Gripper standoff fingertips
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Gripper Standoff Fingertips:",
                        tooltip="Distance from fingertip contact points to the gripper's wrist/origin along the approach direction in meters.",
                    )
                    standoff_field = ui.FloatField()
                    standoff_field.model.set_value(self._grasping_manager.sampler_config["gripper_standoff_fingertips"])
                    standoff_field.model.add_value_changed_fn(self._on_gripper_standoff_fingertips_changed)

                # Gripper maximum aperture
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Gripper Maximum Aperture:",
                        tooltip="Maximum width between gripper fingers in meters. Antipodal points with distance greater than this are rejected.",
                    )
                    gripper_max_aperture = ui.FloatField()
                    gripper_max_aperture.model.set_value(
                        self._grasping_manager.sampler_config["gripper_maximum_aperture"]
                    )
                    gripper_max_aperture.model.add_value_changed_fn(self._on_gripper_maximum_aperture_changed)

                # Grasp align axis
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Grasp Align Axis:",
                        tooltip="Direction perpendicular to the gripper's closing direction. Used to align the gripper with the object surface.",
                    )
                    align_axis_field = ui.StringField()
                    align_axis = self._grasping_manager.sampler_config["grasp_align_axis"]
                    align_axis_field.model.set_value(f"({align_axis[0]}, {align_axis[1]}, {align_axis[2]})")
                    align_axis_field.model.add_value_changed_fn(self._on_grasp_align_axis_changed)

                # Orientation sample axis
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Orientation Sample Axis:",
                        tooltip="Axis around which to sample different grasp orientations when num_orientations > 1.",
                    )
                    orientation_axis_field = ui.StringField()
                    orientation_axis = self._grasping_manager.sampler_config["orientation_sample_axis"]
                    orientation_axis_field.model.set_value(
                        f"({orientation_axis[0]}, {orientation_axis[1]}, {orientation_axis[2]})"
                    )
                    orientation_axis_field.model.add_value_changed_fn(self._on_orientation_axis_changed)

                # Gripper approach direction
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Gripper Approach Direction:",
                        tooltip="Direction along which the gripper approaches the object for grasping.",
                    )
                    approach_dir_field = ui.StringField()
                    approach_dir = self._grasping_manager.sampler_config["gripper_approach_direction"]
                    approach_dir_field.model.set_value(f"({approach_dir[0]}, {approach_dir[1]}, {approach_dir[2]})")
                    approach_dir_field.model.add_value_changed_fn(self._on_approach_direction_changed)

                # Lateral sigma
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Lateral Sigma:",
                        tooltip="Std deviation for random perturbation of grasp center point along the grasp axis (0=midpoint, >0=randomized offset).",
                    )
                    lateral_sigma = ui.FloatField()
                    lateral_sigma.model.set_value(self._grasping_manager.sampler_config["lateral_sigma"])
                    lateral_sigma.model.add_value_changed_fn(self._on_lateral_sigma_changed)

                # Verbose checkbox
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label("Verbose Logging:", width=200, tooltip="Enable detailed logging during grasp generation.")
                    verbose_check = ui.CheckBox(width=10, height=0)
                    verbose_check.model.set_value(self._grasping_manager.sampler_config.get("verbose", False))
                    verbose_check.model.add_value_changed_fn(self._on_sampler_verbose_changed)

    # ==============================================================================
    # Visualization UI Building and Logic
    # ==============================================================================
    def _build_grasp_pose_visualization_frame(self) -> None:
        """Build the UI frame for grasp pose visualization controls.

        Creates controls for visualizing grasp poses in world or local frame, drawing and clearing
        pose visualizations, and gripper visualization with pose navigation controls.
        """
        frame_name = "Pose Visualization"
        frame_collapsed = self._collapsed_states.get(frame_name, False)
        visualization_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with visualization_frame:
            visualization_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label("World Frame:", width=200)
                    vis_checkbox = ui.CheckBox(
                        width=10, height=0, tooltip="Visualize grasp poses in world or local frame"
                    )
                    vis_checkbox.model.set_value(self._draw_poses_in_world_frame)
                    vis_checkbox.model.add_value_changed_fn(self._on_visualize_frame_checkbox_changed)
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Button("Draw Poses", clicked_fn=self._on_visualize_grasp_poses)
                    ui.Button("Clear", clicked_fn=self._on_clear_visualized_grasp_poses)

        gripper_vis_frame_name = "Gripper Visualization"
        gripper_vis_frame_collapsed = self._collapsed_states.get(gripper_vis_frame_name, False)
        gripper_vis_frame = ui.CollapsableFrame(gripper_vis_frame_name, height=0, collapsed=gripper_vis_frame_collapsed)
        with gripper_vis_frame:
            gripper_vis_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(gripper_vis_frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    current_idx = self._current_grasp_pose_idx
                    total_poses = len(self._grasping_manager.grasp_locations)
                    ui.Label(
                        f"Selected Pose Index: {current_idx if total_poses else 0} / {total_poses - 1 if total_poses > 0 else 0}"
                    )
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    enabled = total_poses > 0
                    ui.Button("Previous", enabled=enabled, clicked_fn=self._on_prev_grasp_pose)
                    ui.Button("Next", enabled=enabled, clicked_fn=self._on_next_grasp_pose)
                    ui.Button("Reset", clicked_fn=self._on_reset_grasp_pose)

    def _build_trimesh_debug_draw_frame(self) -> None:
        """Build the UI frame for trimesh debug drawing controls.

        Creates controls for drawing and clearing trimesh visualizations of the object,
        with options to display in world or local coordinate frame.
        """
        trimesh_frame_name = "Trimesh"
        trimesh_frame_collapsed = self._collapsed_states.get(trimesh_frame_name, True)
        trimesh_frame = ui.CollapsableFrame(trimesh_frame_name, height=0, collapsed=trimesh_frame_collapsed)
        with trimesh_frame:
            trimesh_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(trimesh_frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label("World Frame", width=200, tooltip="Draw trimesh in world or local frame")
                    world_frame_checkbox = ui.CheckBox(width=10, height=0)
                    world_frame_checkbox.model.set_value(self._draw_trimesh_world_frame)

                    def on_world_frame_checkbox_changed(model: object) -> None:
                        self._draw_trimesh_world_frame = model.get_value_as_bool()

                    world_frame_checkbox.model.add_value_changed_fn(on_world_frame_checkbox_changed)
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Button("Draw Trimesh", clicked_fn=self._on_draw_trimesh)
                    ui.Button("Clear", clicked_fn=self._on_clear_trimesh)

    # ==============================================================================
    # Simulation UI Building and Logic
    # ==============================================================================
    def _build_simulation_frame(self) -> None:
        """Build the main Simulation settings frame."""
        frame_name = "Simulation"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        simulation_frame = ui.CollapsableFrame(frame_name, height=0, style=get_style(), collapsed=frame_collapsed)
        with simulation_frame:
            simulation_frame.set_collapsed_changed_fn(
                lambda collapsed: self._on_collapsed_changed(frame_name, collapsed)
            )
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Render / Update Kit",
                        width=200,
                        tooltip="If checked, render/update kit after each simulation step."
                        "If unchecked, the simulation will run in the background, rendering/updating after each simulation phase",
                    )
                    render_simulation = ui.CheckBox(width=10, height=0)
                    render_simulation.model.set_value(self._render_simulation)
                    render_simulation.model.add_value_changed_fn(self._on_render_simulation_changed)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)

                    ui.Label(
                        "Simulate Using Timeline:",
                        width=200,
                        tooltip=(
                            "Check to simulate by advancing the timeline instead of direct physics scene simulation steps.\\n\\n"
                            "If checked:\\n"
                            "- Simulation time follows the stage's 'Time Codes Per Second'.\\n"
                            "- Ignores individual phase 'Simulation Step Delta Time'.\\n"
                            "- Physics updates might not align perfectly with every frame if timeline FPS differs from physics FPS.\\n"
                            "- Simulation results can differ from direct physics simulation.\\n"
                            "- Uses the first UsdPhysics.Scene found in the stage, or creates a temporary one at runtime if none found.\\n"
                            "- The 'Isolated Physics Scene Path' setting is ignored.\\n"
                            "- The 'Isolate Simulation' setting is ignored."
                        ),
                    )
                    timeline_checkbox = ui.CheckBox(width=10, height=0)
                    timeline_checkbox.model.set_value(self._simulate_using_timeline)
                    timeline_checkbox.model.add_value_changed_fn(self._on_simulate_using_timeline_changed)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=10)
                    ui.Label(
                        "Isolated Physics Scene Path:",
                        tooltip=(
                            "Path to a specific UsdPhysics.Scene prim to use for isolated simulation. "
                            "This scene will be used ONLY when 'Isolate Simulation' is checked. "
                            "If left empty and isolation is enabled, a default scene will be used. "
                            "Ignored if 'Simulate Using Timeline' is checked."
                        ),
                    )
                    physics_path = ui.StringField(
                        tooltip=(
                            "Path to UsdPhysics.Scene for isolated simulation. Only used if 'Isolate Simulation' is checked."
                        )
                    )
                    physics_path.model.set_value(self._physics_scene_path)
                    physics_path.model.add_value_changed_fn(self._on_physics_scene_path_changed)
                    ui.Button(
                        f"{GLYPHS['plus']}",
                        width=30,
                        clicked_fn=lambda: self._on_set_field_from_selection(physics_path.model),
                        tooltip="Set the path to the currently selected prim in the stage.",
                    )

    # ==============================================================================
    # Workflow UI Building and Logic
    # ==============================================================================
    def _build_workflow_frame(self) -> None:
        """Build the workflow control frame for grasp evaluation."""
        frame_name = "Workflow"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        workflow_frame = ui.CollapsableFrame(frame_name, height=0, style=get_style(), collapsed=frame_collapsed)
        with workflow_frame:
            workflow_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Label("Number of Grasps Samples:", tooltip="Set to -1 to use all grasp samples.")
                    num_grasps_field = ui.IntField()
                    num_grasps_field.model.set_value(self._num_grasps_to_evaluate)
                    num_grasps_field.model.add_value_changed_fn(self._on_num_grasps_to_evaluate_changed)

                with ui.HStack(spacing=5):
                    ui.Label(
                        "Output Directory:",
                        tooltip="Directory path to save grasp evaluation results (e.g., `capture_N.yaml`).",
                    )
                    results_path_field = ui.StringField()
                    output_file_path = self._grasping_manager.get_results_output_dir() or ""
                    results_path_field.model.set_value(output_file_path)
                    results_path_field.model.add_value_changed_fn(self._on_results_output_dir_changed)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Label(
                        "Overwrite Results",
                        width=200,
                        tooltip="If checked, existing result files (capture_N.yaml) in the output path will be overwritten during the workflow."
                        " If unchecked, the workflow will start numbering from the next available index.",
                    )
                    overwrite_results_checkbox = ui.CheckBox(width=10, height=0)
                    overwrite_results_checkbox.model.set_value(self._overwrite_results)
                    overwrite_results_checkbox.model.add_value_changed_fn(self._on_overwrite_results_changed)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)

                    # Determine if Start button should be enabled
                    can_start_workflow = (
                        (not self._is_workflow_running)
                        and (len(self._grasping_manager.grasp_locations) > 0)
                        and bool(self._grasping_manager.gripper_path)
                        and bool(self._grasping_manager.get_object_prim_path())
                        and (self._grasping_manager.get_object_prim() is not None)
                    )

                    ui.Button(
                        "Start",
                        clicked_fn=lambda: asyncio.ensure_future(self._on_evaluate_grasp_poses_async()),
                        enabled=can_start_workflow,
                    )
                    ui.Button(
                        "Stop",
                        clicked_fn=lambda: asyncio.ensure_future(self._on_stop_workflow_clicked_async()),
                        enabled=self._is_workflow_running,
                    )

    # ==============================================================================
    # Configuration UI Building and Logic
    # ==============================================================================
    def _build_config_frame(self) -> None:
        """Build the configuration save/load frame."""
        frame_name = "Config"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        config_frame = ui.CollapsableFrame(frame_name, style=get_style(), height=0, collapsed=frame_collapsed)
        with config_frame:
            config_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Label("File Path:", width=80)
                    config_path = ui.StringField()
                    config_path.model.set_value(self._config_path)
                    config_path.model.add_value_changed_fn(self._on_config_path_changed)

                self._build_config_checkboxes()

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Label(
                        "Overwrite Existing File",
                        width=200,
                        tooltip="If checked, saving will overwrite the file if it already exists at the specified path.",
                    )
                    overwrite_checkbox = ui.CheckBox(width=10, height=0)
                    overwrite_checkbox.model.set_value(self._overwrite_config)
                    overwrite_checkbox.model.add_value_changed_fn(self._on_overwrite_config_changed)

                with ui.HStack(spacing=5):
                    ui.Spacer(width=15)
                    ui.Button("Load", clicked_fn=self._on_load_config)
                    ui.Button("Save", clicked_fn=self._on_save_config)

    def _build_config_checkboxes(self) -> None:
        """Build the configuration components selection checkboxes."""
        frame_name = "Config Includes"
        frame_collapsed = self._collapsed_states.get(frame_name, True)
        fields_frame = ui.CollapsableFrame(frame_name, height=0, collapsed=frame_collapsed)
        with fields_frame:
            fields_frame.set_collapsed_changed_fn(lambda collapsed: self._on_collapsed_changed(frame_name, collapsed))
            with ui.VStack(spacing=5):
                # Iterate through the defined config fields and their labels
                for field_key, label in self._config_fields_labels.items():
                    with ui.HStack(spacing=5):
                        ui.Spacer(width=10)
                        ui.Label(label, width=200)
                        checkbox = ui.CheckBox(width=10, height=0)
                        # Use the field_key (e.g., "gripper") to get the current state
                        checkbox.model.set_value(self._config_fields[field_key])
                        checkbox.model.add_value_changed_fn(
                            # Pass the field_key to the change handler
                            lambda model, f=field_key: self._on_config_field_changed(f, model)
                        )

    # ==============================================================================
    # Event Handlers
    # ==============================================================================

    # --- Gripper Event Handlers ---
    def _on_gripper_base_path_changed(self, model: object) -> None:
        """Handle changes to the gripper base path.

        Args:
            model: The UI model containing the gripper path value.
        """
        if self._grasping_manager.set_gripper(model.as_string):
            self._rebuild_ui_if_joints_changed()
        else:
            self._clear_gripper_joints()
            # Ensure UI (like start button) updates even if gripper set fails
            asyncio.ensure_future(
                asyncio.gather(self._rebuild_gripper_frame_async(), self._rebuild_workflow_frame_async())
            )

    def _on_include_joint_in_grasp_changed(self, model: object, joint_data: dict) -> None:
        """Handle changes to joint inclusion state in grasp phases.

        Args:
            model: The UI model containing the inclusion state.
            joint_data: Dictionary containing joint information and UI state.
        """
        if joint_data["is_valid_grasp_joint"]:
            new_state = model.get_value_as_bool()
            joint_data["include"] = new_state
            absolute_path = joint_data["path"]

            if not new_state:
                # Remove joint from all phases
                for phase in self._grasping_manager.grasp_phases:
                    phase.remove_joint(absolute_path)
            else:
                # Add joint to all phases with default target position
                for phase in self._grasping_manager.grasp_phases:
                    if not phase.has_joint(absolute_path):
                        phase.add_joint(absolute_path)

            asyncio.ensure_future(self._rebuild_gripper_frame_async())

    def _on_set_joint_pregrasp_state(self, model: object, joint_path: str) -> None:
        """Handle changes to joint pregrasp position values.

        Args:
            model: The UI model containing the position value.
            joint_path: Absolute path to the joint in the stage.
        """
        position_value = model.get_value_as_float()
        self._grasping_manager.joint_pregrasp_states[joint_path] = position_value
        grasping_utils.apply_joint_pregrasp_state(joint_path, position_value)

    def _on_move_grasp_phase_up(self, phase_name: str) -> None:
        """Move a grasp phase up in the sequence (earlier execution).

        Args:
            phase_name: Name of the phase to move up.
        """
        if grasping_ui_utils.move_grasp_phase_up(self._grasping_manager, phase_name):
            asyncio.ensure_future(self._rebuild_gripper_frame_async())

    def _on_move_grasp_phase_down(self, phase_name: str) -> None:
        """Move a grasp phase down in the sequence (later execution).

        Args:
            phase_name: Name of the phase to move down.
        """
        if grasping_ui_utils.move_grasp_phase_down(self._grasping_manager, phase_name):
            asyncio.ensure_future(self._rebuild_gripper_frame_async())

    def _on_delete_grasp_phase(self, phase_name: str) -> None:
        """Delete a grasp phase from the sequence.

        Args:
            phase_name: Name of the phase to delete.
        """
        if self._grasping_manager.remove_grasp_phase_by_name(phase_name):
            asyncio.ensure_future(self._rebuild_gripper_frame_async())
        else:
            carb.log_warn(f"Grasp phase '{phase_name}' not found.")

    def _on_new_phase_name_changed(self, model: object) -> None:
        """Handle changes to the new grasp phase name field.

        Updates the internal phase name string when the user types in the new phase name input field.

        Args:
            model: The UI model containing the new phase name text.
        """
        self._new_grasp_phase_name = model.as_string

    def _on_add_new_grasp_phase(self) -> None:
        """Handle adding a new grasp phase to the grasping manager.

        Creates a new grasp phase with the name specified in the phase name field and adds it to the manager.
        Validates that the name is not empty and does not already exist (case-insensitive).
        Rebuilds the gripper UI frame after successful addition.

        Returns:
            None.
        """
        if not self._new_grasp_phase_name:
            carb.log_warn("Please enter a valid grasp phase name.")
            return

        if self._grasping_manager.get_grasp_phase_by_name(self._new_grasp_phase_name, ignore_case=True):
            carb.log_warn(f"Grasp phase '{self._new_grasp_phase_name}' already exists (case-insensitive).")
            return

        # Create and add the new phase to the manager
        self._grasping_manager.create_and_add_grasp_phase(name=self._new_grasp_phase_name)
        self._new_grasp_phase_name = ""
        asyncio.ensure_future(self._rebuild_gripper_frame_async())

    # --- Object Event Handlers ---
    def _on_object_path_changed(self, model: object) -> None:
        """Handle changes to the object path field.

        Updates the grasping manager with the new object prim path and rebuilds the object and workflow UI frames.

        Args:
            model: The UI model containing the new object path string.
        """
        self._grasping_manager.set_object_prim_path(model.as_string)
        asyncio.ensure_future(asyncio.gather(self._rebuild_object_frame_async(), self._rebuild_workflow_frame_async()))

    def _on_num_candidates_changed(self, model: object) -> None:
        """Handle changes to the number of grasp candidates field.

        Updates the sampler configuration with the new target number of grasp candidates to sample.

        Args:
            model: The UI model containing the new number of candidates value.
        """
        self._grasping_manager.sampler_config["num_candidates"] = model.as_int

    def _on_num_orientations_changed(self, model: object) -> None:
        """Handle changes to the number of orientations field.

        Updates the sampler configuration with the new number of orientations to sample per valid grasp axis.

        Args:
            model: The UI model containing the new number of orientations value.
        """
        self._grasping_manager.sampler_config["num_orientations"] = model.as_int

    def _on_gripper_standoff_fingertips_changed(self, model: object) -> None:
        """Handle changes to the gripper standoff fingertips field.

        Updates the sampler configuration with the new distance from fingertip contact points to the gripper's
        wrist along the approach direction.

        Args:
            model: The UI model containing the new gripper standoff fingertips value in meters.
        """
        self._grasping_manager.sampler_config["gripper_standoff_fingertips"] = model.as_float

    def _on_lateral_sigma_changed(self, model: object) -> None:
        """Handle changes to the lateral sigma field.

        Updates the sampler configuration with the new standard deviation for random perturbation of grasp
        center point along the grasp axis.

        Args:
            model: The UI model containing the new lateral sigma value.
        """
        self._grasping_manager.sampler_config["lateral_sigma"] = model.as_float

    def _on_gripper_maximum_aperture_changed(self, model: object) -> None:
        """Handle changes to the gripper maximum aperture field.

        Updates the sampler configuration with the new maximum width between gripper fingers.
        Antipodal points with distance greater than this value are rejected during sampling.

        Args:
            model: The UI model containing the new gripper maximum aperture value in meters.
        """
        self._grasping_manager.sampler_config["gripper_maximum_aperture"] = model.as_float

    def _on_random_seed_changed(self, model: object) -> None:
        """Handle changes to the random seed field.

        Updates the sampler configuration with the new random seed for grasp pose generation.

        Args:
            model: The UI model containing the new random seed value.
        """
        self._grasping_manager.sampler_config["random_seed"] = model.as_int

    def _on_approach_direction_changed(self, model: object) -> None:
        """Handle changes to the gripper approach direction field.

        Parses the vector string and updates the sampler configuration with the new direction along which
        the gripper approaches the object for grasping.

        Args:
            model: The UI model containing the new approach direction as a vector string.
        """
        vector = grasping_ui_utils.parse_vector_string(model.as_string)
        if vector:
            self._grasping_manager.sampler_config["gripper_approach_direction"] = vector

    def _on_grasp_align_axis_changed(self, model: object) -> None:
        """Handle changes to the grasp align axis input field.

        Args:
            model: The UI model containing the new grasp align axis vector string.
        """
        vector = grasping_ui_utils.parse_vector_string(model.as_string)
        if vector:
            self._grasping_manager.sampler_config["grasp_align_axis"] = vector

    def _on_orientation_axis_changed(self, model: object) -> None:
        """Handle changes to the orientation sample axis input field.

        Args:
            model: The UI model containing the new orientation sample axis vector string.
        """
        vector = grasping_ui_utils.parse_vector_string(model.as_string)
        if vector:
            self._grasping_manager.sampler_config["orientation_sample_axis"] = vector

    def _on_sampler_verbose_changed(self, model: object) -> None:
        """Handle changes to the sampler verbose logging checkbox.

        Args:
            model: The UI model containing the new verbose logging state.
        """
        self._grasping_manager.sampler_config["verbose"] = model.get_value_as_bool()

    def _on_generate_grasp_poses(self) -> None:
        """Handle the generate grasp poses button click to create new grasp pose candidates."""
        grasping_ui_utils.clear_debug_draw()

        config = self._grasping_manager.sampler_config.copy()
        if config["random_seed"] == -1:
            config["random_seed"] = None

        self._grasping_manager.generate_grasp_poses(config)
        asyncio.ensure_future(asyncio.gather(self._rebuild_object_frame_async(), self._rebuild_workflow_frame_async()))

    def _on_clear_grasp_poses(self) -> None:
        """Clear the generated grasp poses from the manager."""
        if self._grasping_manager:
            self._grasping_manager.clear_grasp_poses()
            grasping_ui_utils.clear_debug_draw()
            asyncio.ensure_future(
                asyncio.gather(self._rebuild_object_frame_async(), self._rebuild_workflow_frame_async())
            )

    # --- Visualization Event Handlers ---
    def _on_visualize_grasp_poses(self) -> bool:
        """Handle the visualize grasp poses button click to draw pose axes in the viewport.

        Returns:
            True if poses were successfully visualized, False if no poses are available.
        """
        if self._draw_poses_in_world_frame:
            poses = self._grasping_manager.get_grasp_poses(in_world_frame=True)
        else:
            poses = self._grasping_manager.get_grasp_poses(in_world_frame=False)

        if not poses:
            carb.log_warn("No grasp poses available to visualize.")
            return False

        grasping_ui_utils.draw_grasp_samples_as_axes(poses, clear_existing=False)
        return True

    def _on_clear_visualized_grasp_poses(self) -> None:
        """Handle the clear visualized grasp poses button click to remove debug drawings from the viewport."""
        grasping_ui_utils.clear_debug_draw()

    def _on_visualize_frame_checkbox_changed(self, model: object) -> None:
        """Handle changes to the world frame visualization checkbox.

        Args:
            model: The UI model containing the new world frame visualization state.
        """
        self._draw_poses_in_world_frame = model.get_value_as_bool()

    def _on_draw_trimesh(self) -> None:
        """Handle the draw trimesh button click to visualize the object's mesh in the viewport."""
        object_prim = self._grasping_manager.get_object_prim()
        if object_prim:
            grasping_ui_utils.draw_trimesh(
                object_prim, world_frame=self._draw_trimesh_world_frame, clear_existing=False, verbose=True
            )
        else:
            carb.log_warn(f"Cannot draw trimesh: Object prim is not set.")

    def _on_clear_trimesh(self) -> None:
        """Handle the clear trimesh button click to remove trimesh debug drawings from the viewport."""
        grasping_ui_utils.clear_debug_draw()

    def _on_prev_grasp_pose(self) -> None:
        """Handle navigation to the previous grasp pose in the sequence.

        Moves the gripper to the previous grasp pose in the loaded sequence and updates the UI
        to reflect the new pose index.

        Returns:
            None.
        """
        gripper_path = self._grasping_manager.gripper_path
        initial_pose_set = self._grasping_manager.get_initial_gripper_pose() is not None
        total_poses = len(self._grasping_manager.grasp_locations)

        if not gripper_path or not initial_pose_set or total_poses == 0:
            carb.log_warn("Gripper not set, not moved from initial pose, or no poses loaded.")
            return

        self._current_grasp_pose_idx = (self._current_grasp_pose_idx - 1 + total_poses) % total_poses
        self._grasping_manager.move_gripper_to_grasp_pose(self._current_grasp_pose_idx, in_world_frame=True)
        asyncio.ensure_future(self._rebuild_object_frame_async())

    def _on_next_grasp_pose(self) -> None:
        """Handle navigation to the next grasp pose in the sequence.

        Moves the gripper to the next grasp pose in the loaded sequence and updates the UI
        to reflect the new pose index.

        Returns:
            None.
        """
        gripper_path = self._grasping_manager.gripper_path
        initial_pose_set = self._grasping_manager.get_initial_gripper_pose() is not None
        total_poses = len(self._grasping_manager.grasp_locations)

        if not gripper_path or not initial_pose_set or total_poses == 0:
            carb.log_warn("Gripper not set, not moved from initial pose, or no poses loaded.")
            return

        if not initial_pose_set:  # This check might be redundant if store is called elsewhere, but good for safety
            self._grasping_manager.store_initial_gripper_pose()

        self._current_grasp_pose_idx = (self._current_grasp_pose_idx + 1) % total_poses
        self._grasping_manager.move_gripper_to_grasp_pose(self._current_grasp_pose_idx, in_world_frame=True)
        asyncio.ensure_future(self._rebuild_object_frame_async())

    def _on_reset_grasp_pose(self) -> None:
        """Handle resetting the gripper to its initial pose.

        Resets the current grasp pose index to 0 and moves the gripper back to its stored
        initial position and orientation.
        """
        self._current_grasp_pose_idx = 0
        initial_pose = self._grasping_manager.get_initial_gripper_pose()
        if initial_pose is not None:
            self._grasping_manager.set_gripper_pose(initial_pose[0], initial_pose[1])
        else:  # Fallback: move to the origin if no default pose is available
            origin_location = Gf.Vec3d(0, 0, 0)
            origin_orientation = Gf.Quatd(1, 0, 0, 0)
            self._grasping_manager.set_gripper_pose(origin_location, origin_orientation)
        asyncio.ensure_future(self._rebuild_object_frame_async())

    # --- Simulation Event Handlers ---
    def _on_simulation_steps_changed(self, model: object, phase_name: str) -> None:
        """Handle changes to the simulation steps setting for a grasp phase.

        Args:
            model: UI model containing the new simulation steps value.
            phase_name: Name of the grasp phase to update.
        """
        phase_data = self._grasping_manager.get_grasp_phase_by_name(phase_name)
        if phase_data:
            phase_data.simulation_steps = model.as_int

    def _on_simulation_step_dt_changed(self, model: object, phase_name: str) -> None:
        """Handle changes to the simulation step delta time setting for a grasp phase.

        Args:
            model: UI model containing the new simulation step delta time value.
            phase_name: Name of the grasp phase to update.
        """
        phase_data = self._grasping_manager.get_grasp_phase_by_name(phase_name)
        if phase_data:
            phase_data.simulation_step_dt = model.as_float

    def _on_physics_scene_path_changed(self, model: object) -> None:
        """Handle changes to the isolated physics scene path setting.

        Updates the physics scene path and simulation isolation state, then clears
        any existing simulation state.

        Args:
            model: UI model containing the new physics scene path.
        """
        self._physics_scene_path = model.as_string
        self._isolate_grasp_simulation = bool(self._physics_scene_path.strip())
        self._grasping_manager.clear_simulation(simulate_using_timeline=False)

    def _on_render_simulation_changed(self, model: object) -> None:
        """Handle changes to the render simulation checkbox setting.

        Updates whether the simulation should render and update Kit after each simulation step.

        Args:
            model: UI model containing the new render simulation state.
        """
        self._render_simulation = model.get_value_as_bool()

    def _on_simulate_using_timeline_changed(self, model: object) -> None:
        """Handle changes to the simulate using timeline checkbox setting.

        Updates whether simulation should use timeline advancement instead of direct
        physics scene simulation steps.

        Args:
            model: UI model containing the new simulate using timeline state.
        """
        self._simulate_using_timeline = model.get_value_as_bool()

    async def _on_simulate_single_grasp_phase_async(self, phase_identifier: object) -> None:
        """Simulate a single grasp phase with the given phase identifier (index or name).

        Args:
            phase_identifier: Index or name of the grasp phase to simulate.
        """
        success = await self._grasping_manager.simulate_single_grasp_phase(
            phase_identifier,
            render=self._render_simulation,
            isolate_simulation=self._isolate_grasp_simulation,
            physics_scene_path=self._physics_scene_path,
            simulate_using_timeline=self._simulate_using_timeline,
        )
        if not success:
            carb.log_warn(f"Simulation of phase '{phase_identifier}' failed.")

        asyncio.ensure_future(self._rebuild_gripper_frame_async())

    def _on_reset_simulation(self) -> None:
        """Handle resetting the simulation state.

        Clears any existing simulation state and resets the physics scene.
        """
        self._grasping_manager.clear_simulation(simulate_using_timeline=self._simulate_using_timeline)

    async def _on_simulate_all_grasp_phases_async(self) -> None:
        """Handle the event when the user clicks to simulate all grasp phases.

        Executes all configured grasp phases sequentially using the grasping manager.
        This is an asynchronous operation that allows the user to see the complete grasping workflow
        from start to finish.
        """
        success = await self._grasping_manager.simulate_all_grasp_phases(
            render=self._render_simulation,
            isolate_simulation=self._isolate_grasp_simulation,
            physics_scene_path=self._physics_scene_path,
            simulate_using_timeline=self._simulate_using_timeline,
        )
        if not success:
            carb.log_warn("Simulation failed.")
        asyncio.ensure_future(self._rebuild_gripper_frame_async())

    # --- Workflow Event Handlers ---
    def _on_num_grasps_to_evaluate_changed(self, model: object) -> None:
        """Handle changes to the number of grasp poses to evaluate in the workflow.

        Updates the internal count that determines how many grasp poses will be processed
        during the evaluation workflow.

        Args:
            model: The UI model containing the new number of grasps to evaluate.
        """
        self._num_grasps_to_evaluate = model.as_int

    async def _on_evaluate_grasp_poses_async(self) -> None:
        """Handle the start of the grasp pose evaluation workflow.

        Initiates the asynchronous workflow to evaluate grasp poses by moving the gripper
        to each pose and simulating the grasping sequence. Updates the UI to reflect
        the workflow progress and handles completion or error states.

        Returns:
            None.
        """
        if self._is_workflow_running:
            carb.log_warn("Workflow is already running.")
            return

        self._is_workflow_running = True
        await self._rebuild_workflow_frame_async()

        if self._grasping_manager.get_initial_gripper_pose() is None:
            self._grasping_manager.store_initial_gripper_pose()

        if not self._grasping_manager.grasp_locations:
            carb.log_warn("No grasp poses available in manager to evaluate.")
            self._is_workflow_running = False
            self._current_evaluated_grasp_idx = 0
            self._on_reset_grasp_pose()
            await self._rebuild_workflow_frame_async()
            return

        total = len(self._grasping_manager.grasp_locations)
        num_to_evaluate = self._num_grasps_to_evaluate
        if num_to_evaluate == -1 or num_to_evaluate > total:
            num_to_evaluate = total
        self._current_evaluated_grasp_idx = 0

        poses_to_evaluate = [
            self._grasping_manager.get_grasp_pose_at_index(i, in_world_frame=True)
            for i in range(num_to_evaluate)
            if self._grasping_manager.get_grasp_pose_at_index(i, in_world_frame=True) is not None
        ]
        if len(poses_to_evaluate) != num_to_evaluate:
            carb.log_warn("Some grasp poses could not be retrieved for evaluation.")
            # Update num_to_evaluate if some poses were invalid, so the UI is consistent
            num_to_evaluate = len(poses_to_evaluate)

        if num_to_evaluate == 0:
            carb.log_warn("Number of grasps to evaluate is 0. No poses will be evaluated.")
            self._is_workflow_running = False
            self._current_evaluated_grasp_idx = 0
            self._on_reset_grasp_pose()
            await self._rebuild_workflow_frame_async()
            return

        scene_path_to_use = None
        if self._physics_scene_path:
            if not Sdf.Path.IsValidPathString(self._physics_scene_path):
                carb.log_warn(f"Custom physics scene path is not valid: {self._physics_scene_path}")
                await self._rebuild_workflow_frame_async()
                return
            scene_path_to_use = self._physics_scene_path

        async def progress_update_callback(evaluated_count: int) -> None:
            self._current_evaluated_grasp_idx = evaluated_count
            # Update current grasp pose index for visualization to follow along, if desired
            if evaluated_count > 0:
                self._current_grasp_pose_idx = evaluated_count - 1  # It's 0-indexed

        try:
            self._grasping_manager.set_overwrite_results_output(self._overwrite_results)

            await self._grasping_manager.evaluate_grasp_poses(
                grasp_poses=poses_to_evaluate,
                render=self._render_simulation,
                isolate_simulation=self._isolate_grasp_simulation,
                physics_scene_path=scene_path_to_use,
                simulate_using_timeline=self._simulate_using_timeline,
                progress_callback=progress_update_callback,
            )
            # If num_to_evaluate was 0, callback won't run, ensure idx is 0.
            if num_to_evaluate == 0:
                self._current_evaluated_grasp_idx = 0

            if num_to_evaluate > 0:
                # self._current_grasp_pose_idx is already set by the callback to the last evaluated index
                pass
            else:
                self._current_grasp_pose_idx = 0  # Reset if no poses were evaluated

        except Exception as e:
            carb.log_warn(f"Error during grasp poses evaluation: {e}")
            # _current_evaluated_grasp_idx will reflect the last successfully reported count by callback
        finally:
            # Ensure UI is consistent with the final state, especially if an error occurred
            # or if num_to_evaluate was 0 and no callbacks ran.
            self._is_workflow_running = False
            # Reset counters and gripper position if workflow stopped or completed
            self._current_evaluated_grasp_idx = 0
            self._on_reset_grasp_pose()  # Resets self._current_grasp_pose_idx and moves gripper
            await self._rebuild_workflow_frame_async()

    # --- Config Event Handlers ---
    def _on_config_path_changed(self, model: object) -> None:
        """Handle changes to the configuration file path.

        Updates the internal path used for saving and loading grasping configurations.

        Args:
            model: The UI model containing the new configuration file path.
        """
        self._config_path = model.get_value_as_string()

    def _on_results_output_dir_changed(self, model: object) -> None:
        """Handle changes to the results output directory path.

        Updates the directory where grasp evaluation results will be saved during
        the workflow execution.

        Args:
            model: The UI model containing the new results output directory path.
        """
        self._grasping_manager.set_results_output_dir(model.get_value_as_string())

    def _on_overwrite_config_changed(self, model: object) -> None:
        """Handle changes to the overwrite configuration setting.

        Updates whether existing configuration files should be overwritten when saving.

        Args:
            model: The UI model containing the overwrite configuration checkbox state.
        """
        self._overwrite_config = model.get_value_as_bool()

    def _on_config_field_changed(self, field: str, model: object) -> None:
        """Handle changes to individual configuration field inclusion settings.

        Updates which configuration components (gripper, poses, phases, etc.) should be
        included when saving or loading configuration files.

        Args:
            field: The configuration field key being changed.
            model: The UI model containing the checkbox state for the field.
        """
        self._config_fields[field] = model.get_value_as_bool()

    def _on_overwrite_results_changed(self, model: object) -> None:
        """Handle changes to the overwrite results setting.

        Updates whether existing result files should be overwritten during the
        grasp evaluation workflow.

        Args:
            model: The UI model containing the overwrite results checkbox state.
        """
        self._overwrite_results = model.get_value_as_bool()

    async def _on_stop_workflow_clicked_async(self) -> None:
        """Handle the stop workflow button click event.

        Requests the grasping manager to stop the currently running evaluation workflow.
        The workflow loop will handle final cleanup and UI updates.

        Returns:
            None.
        """
        if not self._is_workflow_running:
            carb.log_warn("Workflow is not running.")
            return

        self._grasping_manager.request_workflow_stop()
        # The main workflow loop in _on_evaluate_grasp_poses_async will handle UI updates and state resetting in its finally block.

    def _on_save_config(self) -> None:
        """Handle saving the current grasping configuration to file.

        Saves the selected configuration components (gripper, poses, phases, etc.)
        to the specified file path using the grasping manager.

        Returns:
            None.
        """
        file_path = self._config_path
        if not file_path:
            carb.log_warn("Cannot save configuration: File path is not set.")
            return

        components_to_save = [key for key, enabled in self._config_fields.items() if enabled]
        if not components_to_save:
            carb.log_warn("No configuration components selected to save.")
            return

        self._grasping_manager.save_config(
            file_path=file_path, components=components_to_save, overwrite=self._overwrite_config
        )

    def _on_load_config(self) -> None:
        """Load configuration from the specified file path.

        Attempts to load the selected configuration components from the file path specified in the UI.
        Applies loaded pregrasp joint states if the pregrasp component was successfully loaded.

        Returns:
            None.
        """
        file_path = self._config_path
        if not file_path:
            carb.log_warn("Cannot load configuration: File path is not set.")
            return

        components_to_load = [key for key, enabled in self._config_fields.items() if enabled]
        if not components_to_load:
            carb.log_warn("No configuration components selected to load.")
            return

        load_status = self._grasping_manager.load_config(file_path=file_path, components=components_to_load)

        status_log_lines = [f"Load configuration results from '{file_path}':"]
        successful_loads = []
        for component, status in load_status.items():
            status_log_lines.append(
                f"  - {self._config_fields_labels.get(component, component.capitalize())}: {status}"
            )
            if status.startswith("Success"):
                successful_loads.append(component)

        carb.log_info("\n".join(status_log_lines))

        if "pregrasp" in successful_loads and self._grasping_manager.joint_pregrasp_states:
            carb.log_info("Applying loaded pregrasp joint states...")
            grasping_utils.apply_joint_pregrasp_states(self._grasping_manager.joint_pregrasp_states)

        needs_gripper_rebuild = any(comp in successful_loads for comp in ["gripper", "phases", "pregrasp"])
        needs_object_rebuild = any(comp in successful_loads for comp in ["object", "poses", "sampler"])

        if needs_gripper_rebuild:
            # Rebuilds the joints UI to reload joint info and rebuild gripper/workflow frames.
            self._update_ui_joint_selection_from_loaded_phases()

        if needs_object_rebuild:
            if needs_gripper_rebuild:
                # Rebuild the object frame, workflow frame is already being rebuilt.
                asyncio.ensure_future(self._rebuild_object_frame_async())
            else:
                # Rebuild object and workflow frames.
                asyncio.ensure_future(
                    asyncio.gather(self._rebuild_object_frame_async(), self._rebuild_workflow_frame_async())
                )

    # --- General UI Event Handlers ---
    def _on_set_field_from_selection(self, model: object) -> None:
        """Set the given field to the path of the selected prim in the stage.

        Args:
            model: The UI field model to update with the selected prim path.
        """
        path = grasping_ui_utils.get_selected_prim_path()
        if path:
            model.set_value(path)

    def _on_collapsed_changed(self, key: str, collapsed: bool) -> None:
        """Keep track in a dict of the collapsed state of the frames.

        Args:
            key: The unique identifier for the collapsible frame.
            collapsed: Whether the frame is collapsed.
        """
        self._collapsed_states[key] = collapsed
