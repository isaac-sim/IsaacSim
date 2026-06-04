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

"""Manages grasp pose generation, simulation, and evaluation for robotic grasping workflows."""

import os
from dataclasses import asdict, dataclass, field

import carb
import isaacsim.replicator.grasping.grasping_utils as grasping_utils
import isaacsim.replicator.grasping.sampler_utils as sampler_utils
import isaacsim.replicator.grasping.transform_utils as transform_utils
import omni.usd
import yaml
from pxr import Gf, Sdf, Usd, UsdPhysics

DEFAULT_NUM_SIMULATION_STEPS = 32
DEFAULT_SIMULATION_STEP_DT = 1 / 60
DEFAULT_SAMPLER_CONFIG = {
    "sampler_type": "antipodal",
    "num_candidates": 100,
    "num_orientations": 1,
    "gripper_maximum_aperture": 0.08,
    "gripper_standoff_fingertips": 0.17,
    "gripper_approach_direction": (0, 0, 1),
    "grasp_align_axis": (0, 1, 0),
    "orientation_sample_axis": (0, 1, 0),
    "lateral_sigma": 0.0,
    "random_seed": -1,
    "verbose": False,
}


@dataclass
class GraspPhase:
    """Represents a phase in a grasping sequence with joint targets and simulation parameters.

    Args:
        name: Name of the grasp phase.
        joint_drive_targets: Target positions for the joints in the phase.
        simulation_steps: Number of simulation steps for the phase.
        simulation_step_dt: Simulation step dt for the phase.
    """

    name: str  # Name of the grasp phase
    joint_drive_targets: dict[str, float] = field(default_factory=dict)  # Target positions for the joints in the phase
    simulation_steps: int = DEFAULT_NUM_SIMULATION_STEPS  # Number of simulation steps for the phase
    """Number of simulation steps for the phase."""
    simulation_step_dt: float = DEFAULT_SIMULATION_STEP_DT  # Simulation step dt for the phase
    """Simulation step dt for the phase."""

    def add_joint(self, joint_path: str, target_position: float = 0) -> None:
        """Add a joint with its target position to this grasp phase.

        Args:
            joint_path: Path to the joint to add.
            target_position: Target position for the joint in the phase.
        """
        self.joint_drive_targets[joint_path] = target_position

    def remove_joint(self, joint_path: str) -> None:
        """Remove a joint from this grasp phase if it exists.

        Args:
            joint_path: Path to the joint to remove.
        """
        if joint_path in self.joint_drive_targets:
            del self.joint_drive_targets[joint_path]

    def has_joint(self, joint_path: str) -> bool:
        """Check if a joint exists in this grasp phase.

        Args:
            joint_path: Path to the joint to check.

        Returns:
            True if the joint exists in this phase, False otherwise.
        """
        return joint_path in self.joint_drive_targets

    def get_joint_target(self, joint_path: str) -> float:
        """Get the target position for a joint in this grasp phase.

        Args:
            joint_path: Path to the joint.

        Returns:
            The target position for the joint, or 0 if the joint is not found.
        """
        return self.joint_drive_targets.get(joint_path, 0)


class GraspingManager:
    """Manages grasp pose generation, simulation, and evaluation for robotic grasping workflows.

    The GraspingManager coordinates all aspects of grasp evaluation including gripper configuration, object
    selection, grasp pose sampling, physics simulation through multiple phases (open, close, lift, etc.),
    and results output. It supports both direct physics stepping and timeline-based simulation with optional
    scene isolation.

    Key capabilities include:
    - Grasp pose generation using configurable samplers (antipodal grasp sampling)
    - Multi-phase grasp simulation with customizable joint drive targets and timing
    - Physics simulation isolation for controlled testing environments
    - Configuration save/load for reproducible workflows
    - Structured results output in YAML format
    - Workflow management with stop controls and progress tracking

    The manager operates on USD prims for grippers and objects, automatically handling coordinate frame
    transformations between local object space and world coordinates. It maintains state for joint
    pregrasp positions, simulation parameters, and grasp evaluation results throughout the workflow.
    """

    def __init__(self) -> None:
        # List of the grasp poses to be tested
        self.grasp_locations: list[Gf.Vec3d] = []
        self.grasp_orientations: list[Gf.Quatd] = []

        # Contains joint drive targets and simulation parameters for each grasp phase (e.g. open, close, etc.)
        self.grasp_phases = []

        # Contains the pre grasp joint states of the gripper, the starting state of the joints before perfroming the simulation
        self.joint_pregrasp_states: dict[str, float] = {}

        # External force/torque applied to the object after the grasp phases to check stability
        self.object_simulation_phases = []

        # Gripper and object
        self._gripper_prim = None
        self._object_prim_path: str | None = None

        # Store initial gripper pose to restore after simulation or grasp pose testing
        self._initial_gripper_location: Gf.Vec3d | None = None
        self._initial_gripper_orientation: Gf.Quatd | None = None

        # Temporary physics scene created when a custom path is provided
        self._temp_grasping_physics_scene: UsdPhysics.Scene | None = None

        # Results output path
        self._results_output_dir: str | None = None
        self._write_frame_counter: int = 0  # Counter of the current frame being written
        self._overwrite_results_output: bool = False  # Flag to control result file overwriting

        # Store sampler configuration - initialized with defaults
        self.sampler_config = DEFAULT_SAMPLER_CONFIG.copy()

        # Workflow control
        self._workflow_stop_requested: bool = False
        self._workflow_printed_messages: set = set()
        self._first_write_failure_logged_this_workflow: bool = False

    def clear(self) -> None:
        """Clear all phases, poses, and reset any physics scene references."""
        self.grasp_phases = []
        self.grasp_locations = []
        self.grasp_orientations = []
        self.object_simulation_phases = []
        self.joint_pregrasp_states.clear()
        self._clear_all_simulation_aspects()
        self.clear_gripper()
        self.clear_object()
        self.sampler_config = DEFAULT_SAMPLER_CONFIG.copy()
        self._workflow_stop_requested = False
        self._workflow_printed_messages.clear()
        self._first_write_failure_logged_this_workflow = False

    def _clear_all_simulation_aspects(self) -> None:
        """Reset all simulation aspects, including direct physics, temporary scenes, and timeline."""
        stage = omni.usd.get_context().get_stage()

        # Clear direct physics simulation and temporary scene
        grasping_utils.reset_physics_simulation()
        if self._temp_grasping_physics_scene:
            scene_path = self._temp_grasping_physics_scene.GetPath()
            if stage and stage.GetPrimAtPath(scene_path):
                stage.RemovePrim(scene_path)
            self._temp_grasping_physics_scene = None

        # Clear timeline-based simulation
        grasping_utils.stop_timeline()

    def clear_simulation(self, simulate_using_timeline: bool) -> None:
        """Reset the physics simulation state, either for direct physics stepping or timeline-based simulation.

        If `simulate_using_timeline` is False, this also handles the cleanup of any temporary
        physics scene created by the grasping manager.

        Args:
            simulate_using_timeline: Whether to use timeline-based simulation.
        """
        stage = omni.usd.get_context().get_stage()
        if simulate_using_timeline:
            grasping_utils.stop_timeline()
        else:
            grasping_utils.reset_physics_simulation()
            if self._temp_grasping_physics_scene:
                scene_path = self._temp_grasping_physics_scene.GetPath()
                if stage and stage.GetPrimAtPath(scene_path):  # Check if prim exists before removal
                    stage.RemovePrim(scene_path)
            self._temp_grasping_physics_scene = None

    def clear_gripper(self) -> None:
        """Clear the gripper prim reference."""
        self._gripper_prim = None
        self.joint_pregrasp_states.clear()

    def clear_object(self) -> None:
        """Clear the object prim reference."""
        self._object_prim_path = None

    # --- Results ---
    def set_results_output_dir(self, dir_path: str | None) -> None:
        """Set the output directory for grasp results.

        Args:
            dir_path: Directory path for results output.
        """
        if dir_path and isinstance(dir_path, str) and dir_path.strip():
            expanded_path = os.path.expanduser(dir_path.strip())
            if not os.path.isabs(expanded_path):
                carb.log_warn(
                    f"Provided results output directory '{dir_path}' (expanded to '{expanded_path}') is not an absolute path. This might lead to unexpected behavior."
                )
            self._results_output_dir = expanded_path
            print(f"Results will be written to directory: {expanded_path}")
        else:
            self._results_output_dir = None
            print("Results writing disabled.")

    def get_results_output_dir(self) -> str | None:
        """Get the current results output directory.

        Returns:
            The current results output directory path.
        """
        return self._results_output_dir

    def set_overwrite_results_output(self, overwrite: bool) -> None:
        """Set whether to overwrite or find the next available index for result files.

        Args:
            overwrite: Whether to overwrite existing result files.
        """
        self._overwrite_results_output = overwrite

    # --- Gripper Management ---
    def set_gripper(self, gripper: str | Usd.Prim) -> bool:
        """Set the gripper prim by path (str) or prim (Usd.Prim).

        Args:
            gripper: Gripper prim path or Usd.Prim object.

        Returns:
            True if gripper was successfully set, False otherwise.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage or gripper is None:
            self._gripper_prim = None
            return False

        if isinstance(gripper, str):
            if not Sdf.Path.IsValidPathString(gripper):
                self._gripper_prim = None
                return False
            prim = stage.GetPrimAtPath(gripper)
            if prim and prim.IsValid():
                self._gripper_prim = prim
                self.joint_pregrasp_states.clear()
                return True
        elif isinstance(gripper, Usd.Prim):
            if gripper.IsValid():
                self._gripper_prim = gripper
                self.joint_pregrasp_states.clear()
                return True

        self._gripper_prim = None
        self.joint_pregrasp_states.clear()
        return False

    @property
    def gripper_path(self) -> str:
        """Path string of the currently set gripper prim.

        Returns:
            The gripper prim path as a string, or empty string if no gripper is set.
        """
        return str(self._gripper_prim.GetPath()) if self._gripper_prim else ""

    @property
    def gripper_prim(self) -> Usd.Prim | None:
        """Gripper prim reference.

        Returns:
            The gripper prim if set.
        """
        return self._gripper_prim

    def set_object_prim_path(self, path: str) -> None:
        """Set the object prim path for grasping operations.

        Args:
            path: Path to the object prim.
        """
        self._object_prim_path = path

    def get_object_prim_path(self) -> str | None:
        """Get the current object prim path.

        Returns:
            The object prim path if set.
        """
        return self._object_prim_path

    def get_object_prim(self) -> Usd.Prim | None:
        """Get the object prim for grasping operations.

        Returns:
            The object prim if the path is valid and the prim exists.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("Cannot get object prim: Stage is not set.")
            return None
        if self._object_prim_path is None:
            carb.log_warn("Cannot get object prim: Object path is not set.")
            return None
        if not Sdf.Path.IsValidPathString(self._object_prim_path):
            carb.log_warn(f"Cannot get object prim: Invalid object path '{self._object_prim_path}'.")
            return None
        return stage.GetPrimAtPath(self._object_prim_path)

    def create_and_add_grasp_phase(
        self,
        name: str,
        joint_drive_targets: dict[str, float] = None,
        simulation_steps: int = DEFAULT_NUM_SIMULATION_STEPS,
        simulation_step_dt: float = DEFAULT_SIMULATION_STEP_DT,
    ) -> GraspPhase:
        """Create a new grasp phase with the given parameters and add it to the manager.

        Args:
            name: Name of the grasp phase.
            joint_drive_targets: Target positions for the joints in the phase.
            simulation_steps: Number of simulation steps for the phase.
            simulation_step_dt: Simulation step dt for the phase.

        Returns:
            The created grasp phase.
        """
        joint_targets = joint_drive_targets or {}
        new_phase = GraspPhase(
            name=name,
            joint_drive_targets=joint_targets,
            simulation_steps=simulation_steps,
            simulation_step_dt=simulation_step_dt,
        )
        self.grasp_phases.append(new_phase)
        return new_phase

    def remove_grasp_phase_by_name(self, phase_name: str) -> bool:
        """Remove a grasp phase from the manager by name.

        Args:
            phase_name: Name of the grasp phase to remove.

        Returns:
            True if the phase was found and removed, False otherwise.
        """
        phase = self.get_grasp_phase_by_name(phase_name)
        if phase:
            self.grasp_phases.remove(phase)
            return True
        return False

    def get_grasp_phase_by_name(self, name: str, ignore_case: bool = True) -> GraspPhase | None:
        """Get a grasp phase by name.

        Args:
            name: Name of the grasp phase to retrieve.
            ignore_case: Whether to perform case-insensitive matching.

        Returns:
            The grasp phase if found, None otherwise.
        """
        for grasp_phase in self.grasp_phases:
            if ignore_case and grasp_phase.name.lower() == name.lower():
                return grasp_phase
            elif not ignore_case and grasp_phase.name == name:
                return grasp_phase
        return None

    def get_grasp_phase_by_index(self, index: int) -> GraspPhase | None:
        """Get a grasp phase by index.

        Args:
            index: Index of the grasp phase to retrieve.

        Returns:
            The grasp phase if the index is valid, None otherwise.
        """
        if 0 <= index < len(self.grasp_phases):
            return self.grasp_phases[index]
        return None

    def get_grasp_phases_as_dicts(self) -> list[dict]:
        """Get all grasp phases as a list of dictionaries.

        Returns:
            List of dictionaries representing the grasp phases.
        """
        return [asdict(phase) for phase in self.grasp_phases]

    def get_grasp_phase_names(self) -> list[str]:
        """Get a list of all grasp phase names.

        Returns:
            List of grasp phase names.
        """
        return [phase.name for phase in self.grasp_phases]

    # --- Configuration Saving/Loading ---
    def save_config(
        self,
        file_path: str,
        components: list[str] | None = None,
        overwrite: bool = False,
    ) -> None:
        """Save the current GraspingManager state to a YAML configuration file.

        Args:
            file_path: The path to save the configuration file.
            components: A list of strings specifying which logical components to save.
                        Valid short names: "gripper", "phases", "object", "pregrasp", "poses", "sampler".
                        If None, saves all components.
            overwrite: Whether to overwrite the file if it exists.
        """
        config = {}
        # Defines valid short names for configuration components.
        all_short_names = {"gripper", "phases", "object", "pregrasp", "poses", "sampler"}
        components_to_save_short_names = set(components) if components is not None else all_short_names

        if "gripper" in components_to_save_short_names and self.gripper_path:
            config["gripper_path"] = self.gripper_path

        if "phases" in components_to_save_short_names and self.grasp_phases:
            config["grasp_phases"] = self.get_grasp_phases_as_dicts()

        if "object" in components_to_save_short_names:
            object_path = self.get_object_prim_path()
            if object_path:
                config["object_path"] = object_path

        if "pregrasp" in components_to_save_short_names and self.joint_pregrasp_states:
            config["joint_pregrasp_states"] = self.joint_pregrasp_states.copy()

        if "poses" in components_to_save_short_names and self.grasp_locations:
            poses_data = []
            for loc, quat in zip(self.grasp_locations, self.grasp_orientations):
                poses_data.append(
                    {
                        "position": list(loc),
                        "orientation": {"w": quat.GetReal(), "xyz": list(quat.GetImaginary())},
                    }
                )
            config["grasp_poses"] = poses_data

        if "sampler" in components_to_save_short_names:
            config_to_save = self.sampler_config
            serializable_sampler_config = {}
            for key, value in config_to_save.items():
                if isinstance(value, tuple):
                    serializable_sampler_config[key] = list(value)
                else:
                    serializable_sampler_config[key] = value
            config["sampler_config"] = serializable_sampler_config

        if not config:
            carb.log_warn("Nothing specified or no valid data to save in configuration.")
            return

        grasping_utils.write_yaml_config(file_path, config, overwrite=overwrite)

    def load_config(self, file_path: str, components: list[str] | None = None) -> dict[str, str]:
        """Load GraspingManager state from a YAML configuration file.

        Args:
            file_path: The path to the configuration file.
            components: A list of strings specifying which logical components to load.
                        Valid short names: "gripper", "phases", "object", "pregrasp", "poses", "sampler".
                        If None, loads all available components from the file.

        Returns:
            A dictionary mapping each *requested short component name* to its loading status message.
        """
        config_data = grasping_utils.read_yaml_config(file_path)

        # Defines valid short names for configuration components.
        all_short_names = {"gripper", "phases", "object", "pregrasp", "poses", "sampler"}
        requested_short_names = set(components) if components is not None else all_short_names

        load_status = dict.fromkeys(all_short_names, "Not requested")
        for key in requested_short_names:
            load_status[key] = "Requested, pending"

        if config_data is None:
            carb.log_warn(f"Failed to read or parse configuration file: {file_path}")
            for key in requested_short_names:
                load_status[key] = "Failed (Could not read/parse file)"
            return {key: status for key, status in load_status.items() if key in requested_short_names}

        # Load gripper path
        if "gripper" in requested_short_names:
            if "gripper_path" in config_data:
                gripper_path_val = config_data["gripper_path"]
                if isinstance(gripper_path_val, str) and self.set_gripper(gripper_path_val):
                    print(f"Loaded gripper: {gripper_path_val}")
                    load_status["gripper"] = "Success"
                else:
                    msg = f"Gripper prim at path '{gripper_path_val}' from config not found or invalid."
                    carb.log_warn(msg)
                    load_status["gripper"] = f"Failed ({msg})"
                    # If gripper load fails, dependent components cannot be reliably loaded or validated.
                    for dep_key in ["phases", "pregrasp"]:
                        if dep_key in requested_short_names:
                            load_status[dep_key] = f"Skipped (Gripper load failed)"
                            requested_short_names.discard(dep_key)  # Avoid further processing
            else:
                load_status["gripper"] = "Not in file"

        # Load phases, validate against current gripper
        if "phases" in requested_short_names:
            if "grasp_phases" in config_data:
                phase_configs = config_data["grasp_phases"]
                if isinstance(phase_configs, list):
                    validated_phases = []
                    invalid_joint_paths = set()
                    current_gripper_path = self.gripper_path
                    if (
                        not current_gripper_path and load_status.get("gripper") == "Success"
                    ):  # Gripper was reported as loaded, but its path is unexpectedly empty.
                        carb.log_warn("Cannot validate phases: Gripper was reported as loaded, but its path is empty.")
                        load_status["phases"] = "Failed (Gripper path empty after load)"
                    elif not current_gripper_path:  # Gripper not loaded or failed to load
                        carb.log_warn("Cannot validate phases: No valid gripper is currently set.")
                        load_status["phases"] = "Skipped (No valid gripper)"
                    else:  # Gripper is set, proceed with validation
                        for phase_config in phase_configs:
                            if not isinstance(phase_config, dict):
                                carb.log_warn("Invalid phase format in config, skipping phase.")
                                continue
                            valid_phase = True
                            for joint_path_val in phase_config.get("joint_drive_targets", {}):
                                stage = omni.usd.get_context().get_stage()
                                joint_prim = stage.GetPrimAtPath(joint_path_val) if stage else None
                                if (
                                    not joint_prim
                                    or not joint_prim.IsValid()
                                    or not joint_prim.IsA(UsdPhysics.Joint)
                                    or not joint_path_val.startswith(current_gripper_path)
                                ):
                                    invalid_joint_paths.add(joint_path_val)
                                    valid_phase = False
                                    break
                            if valid_phase:
                                validated_phases.append(phase_config)

                        if invalid_joint_paths:
                            msg = f"Config references invalid/missing/non-joint/mismatched joint paths under gripper '{current_gripper_path}': {', '.join(sorted(invalid_joint_paths))}."
                            carb.log_warn(msg)
                            if validated_phases:
                                self.grasp_phases = []
                                for phase_dict in validated_phases:
                                    self.create_and_add_grasp_phase(**phase_dict)
                                print(
                                    f"Loaded {len(self.grasp_phases)} valid grasp phases (skipped some due to errors)."
                                )
                                load_status["phases"] = f"Success (with validation warnings: {msg})"
                            else:
                                self.grasp_phases = []
                                load_status["phases"] = f"Failed ({msg})"
                        elif validated_phases:
                            self.grasp_phases = []
                            for phase_dict in validated_phases:
                                self.create_and_add_grasp_phase(**phase_dict)
                            print(f"Loaded {len(self.grasp_phases)} grasp phases.")
                            load_status["phases"] = "Success"
                        else:
                            self.grasp_phases = []
                            load_status["phases"] = "Failed (No valid phase data found in list)"
                else:
                    load_status["phases"] = "Failed (Data for 'grasp_phases' is not a list)"
            else:
                load_status["phases"] = "Not in file"

        # Load initial joint states (pregrasp), validate against current gripper
        if "pregrasp" in requested_short_names:
            if "joint_pregrasp_states" in config_data:
                pregrasp_states_val = config_data["joint_pregrasp_states"]
                if isinstance(pregrasp_states_val, dict):
                    loaded_pregrasp_count = 0
                    current_gripper_path = self.gripper_path
                    if not current_gripper_path and load_status.get("gripper") == "Success":
                        carb.log_warn(
                            "Cannot validate pregrasp states: Gripper was reported as loaded, but its path is empty."
                        )
                        load_status["pregrasp"] = "Failed (Gripper path empty after load)"
                    elif not current_gripper_path:
                        carb.log_warn("Cannot validate pregrasp states: No valid gripper is currently set.")
                        load_status["pregrasp"] = "Skipped (No valid gripper)"
                    else:
                        self.joint_pregrasp_states.clear()
                        stage = omni.usd.get_context().get_stage()
                        for joint_path_val, position_val in pregrasp_states_val.items():
                            joint_prim = stage.GetPrimAtPath(joint_path_val) if stage else None
                            if (
                                joint_prim
                                and joint_prim.IsValid()
                                and joint_prim.IsA(UsdPhysics.Joint)
                                and joint_path_val.startswith(current_gripper_path)
                                and isinstance(position_val, (int, float))
                            ):
                                self.joint_pregrasp_states[joint_path_val] = float(position_val)
                                loaded_pregrasp_count += 1
                            else:
                                carb.log_warn(
                                    f"Skipping pregrasp state for invalid/missing/non-joint/mismatched/bad-value joint path: {joint_path_val}"
                                )
                        if loaded_pregrasp_count > 0:
                            print(f"Loaded {loaded_pregrasp_count} joint pregrasp states.")
                            load_status["pregrasp"] = "Success"
                        else:
                            print("No valid joint pregrasp states loaded for current gripper.")
                            load_status["pregrasp"] = "Failed (No valid joints found/validated for current gripper)"
                else:
                    load_status["pregrasp"] = "Failed (Data for 'joint_pregrasp_states' is not a dictionary)"
            else:
                load_status["pregrasp"] = "Not in file"

        # Load object path
        if "object" in requested_short_names:
            if "object_path" in config_data:
                object_path_val = config_data["object_path"]
                if object_path_val is not None and isinstance(object_path_val, str):
                    stage = omni.usd.get_context().get_stage()
                    obj_prim = stage.GetPrimAtPath(object_path_val) if stage else None
                    if obj_prim and obj_prim.IsValid():
                        self.set_object_prim_path(object_path_val)
                        print(f"Loaded object path: {object_path_val}")
                        load_status["object"] = "Success"
                    else:
                        msg = f"Object prim at path '{object_path_val}' from config not found or invalid."
                        carb.log_warn(msg)
                        load_status["object"] = f"Failed ({msg})"
                        self.clear_object()
                else:
                    load_status["object"] = "Failed (Invalid data format for 'object_path')"
                    self.clear_object()
            else:
                load_status["object"] = "Not in file"

        # Load grasp poses
        if "poses" in requested_short_names:
            if "grasp_poses" in config_data:
                grasp_poses_config_list = config_data["grasp_poses"]
                if isinstance(grasp_poses_config_list, list):
                    try:
                        loaded_locations = []
                        loaded_orientations = []
                        valid_poses_loaded = 0
                        for pose_entry in grasp_poses_config_list:
                            if not isinstance(pose_entry, dict):
                                carb.log_warn(f"Skipping invalid pose entry (not a dict): {pose_entry}")
                                continue

                            position_data = pose_entry.get("position")
                            orientation_data = pose_entry.get("orientation")

                            if not isinstance(position_data, list) or len(position_data) != 3:
                                carb.log_warn(f"Skipping pose with invalid position data: {position_data}")
                                continue
                            if (
                                not isinstance(orientation_data, dict)
                                or "w" not in orientation_data
                                or "xyz" not in orientation_data
                                or not isinstance(orientation_data["xyz"], list)
                                or len(orientation_data["xyz"]) != 3
                            ):
                                carb.log_warn(f"Skipping pose with invalid orientation data: {orientation_data}")
                                continue

                            try:
                                loc = Gf.Vec3d(*position_data)
                                quat_w = orientation_data["w"]
                                quat_xyz = orientation_data["xyz"]
                                quat = Gf.Quatd(quat_w, Gf.Vec3d(*quat_xyz))

                                loaded_locations.append(loc)
                                loaded_orientations.append(quat)
                                valid_poses_loaded += 1
                            except Exception as e:
                                carb.log_warn(f"Error converting pose data entry to Gf types: {pose_entry}. Error: {e}")
                                continue

                        if valid_poses_loaded > 0:
                            self.grasp_locations = loaded_locations
                            self.grasp_orientations = loaded_orientations
                            print(f"Loaded {valid_poses_loaded} grasp poses.")
                            load_status["poses"] = "Success"
                            if valid_poses_loaded < len(grasp_poses_config_list):
                                load_status["poses"] += " (some entries skipped due to errors)"
                        else:
                            carb.log_warn(
                                "No valid grasp pose entries found in 'grasp_poses' list. Skipping pose load."
                            )
                            load_status["poses"] = "Failed (No valid pose entries in list)"
                            self.clear_grasp_poses()

                    except Exception as e:
                        carb.log_warn(f"Error processing 'grasp_poses' list from config: {e}. Skipping pose load.")
                        load_status["poses"] = f"Failed (Error processing list: {e})"
                        self.clear_grasp_poses()
                else:
                    load_status["poses"] = "Failed (Data for 'grasp_poses' is not a list)"
                    self.clear_grasp_poses()
            else:
                load_status["poses"] = "Not in file"

        # Load Sampler Config
        if "sampler" in requested_short_names:
            if "sampler_config" in config_data:
                loaded_sampler_config = config_data["sampler_config"]
                if isinstance(loaded_sampler_config, dict):
                    sampler_updated = False
                    current_defaults = DEFAULT_SAMPLER_CONFIG
                    for key, value in loaded_sampler_config.items():
                        if key in self.sampler_config:
                            if isinstance(current_defaults.get(key), tuple) and isinstance(value, list):
                                try:
                                    self.sampler_config[key] = tuple(value)
                                    sampler_updated = True
                                except (TypeError, ValueError):
                                    carb.log_warn(
                                        f"Could not convert loaded list {value} to tuple for sampler config key '{key}'. Skipping."
                                    )
                            elif type(self.sampler_config[key]) == type(value):
                                self.sampler_config[key] = value
                                sampler_updated = True
                            elif isinstance(self.sampler_config[key], int) and isinstance(value, float):
                                self.sampler_config[key] = int(value)
                                sampler_updated = True
                            elif isinstance(self.sampler_config[key], float) and isinstance(value, int):
                                self.sampler_config[key] = float(value)
                                sampler_updated = True
                            else:
                                carb.log_warn(
                                    f"Type mismatch for sampler config key '{key}' (expected {type(self.sampler_config[key])}, got {type(value)}). Skipping."
                                )
                        else:
                            carb.log_warn(f"Ignoring unknown sampler config key '{key}' from file.")
                    if sampler_updated:
                        print("Loaded and updated sampler configuration.")
                        load_status["sampler"] = "Success"
                    else:
                        if load_status.get("sampler") == "Requested, pending":
                            load_status["sampler"] = "Failed (No valid/matching keys found in sampler_config)"
                else:
                    load_status["sampler"] = "Failed (Data for 'sampler_config' is not a dictionary)"
            else:
                load_status["sampler"] = "Not in file"

        # Filter load_status to only include initially requested short names
        final_status = {key: status for key, status in load_status.items() if key in requested_short_names}
        return final_status

    def request_workflow_stop(self) -> None:
        """Request the current grasp evaluation workflow to stop gracefully."""
        self._workflow_stop_requested = True
        # Use _log_once for consistency, ensuring it prints if not already part of a workflow log
        self._log_once("Workflow stop requested by user.", "print")

    def _log_once(self, message: str, level: str = "info") -> None:
        """Logs a message once per workflow run based on the message string.

        Args:
            message: The string message to log.
            level: "info", "warn", or "print".
        """
        if message not in self._workflow_printed_messages:
            if level == "info":
                carb.log_info(message)
            elif level == "warn":
                carb.log_info(message)
            elif level == "print":
                print(message)
            else:
                carb.log_warn(f"_log_once called with unknown level '{level}': {message}")
                print(message)  # Default to log_info for unknown level
            self._workflow_printed_messages.add(message)

    # --- Simulation ---
    async def simulate_all_grasp_phases(
        self,
        render: bool = True,
        physics_scene_path: str | None = None,
        isolate_simulation: bool = False,
        simulate_using_timeline: bool = False,
    ) -> bool:
        """Simulate all grasp phases.

        Args:
            render: Whether to render/update Kit for every simulation frame.
            physics_scene_path: Optional path to a specific UsdPhysics.Scene prim. If None or invalid,
                                the default physics scene is used.
            isolate_simulation: If True and a valid custom `physics_scene_path` is provided,
                              only the gripper and object prims will be added as owners to that scene.
                              *Ignored if simulate_using_timeline is True.*
            simulate_using_timeline: If True, use the main timeline for simulation instead of direct physics steps.

        Returns:
            True if simulation was successful, False otherwise
        """
        if not self.grasp_phases:
            carb.log_warn("No grasp phases defined.")
            return False

        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("Cannot simulate grasp phases: Stage is not available.")
            return False

        physics_scene = None  # Scene to be used for simulation call
        base_physics_scene = None  # User-provided scene
        isolated_rigid_body_prims = []  # Prims added if isolating
        is_isolated = False  # Flag if owners were added
        self._temp_grasping_physics_scene = None  # Clear any previous temp scene reference

        if simulate_using_timeline:
            self._log_once(
                "Simulating grasp phases using timeline (custom scene path and isolation ignored for timeline).",
                "print",
            )
            # Timeline simulation uses default scene logic implicitly
        elif physics_scene_path:
            # Try to get the user-specified scene
            base_physics_scene = grasping_utils.get_physics_scene(stage, physics_scene_path)
            if base_physics_scene:
                # Always duplicate if base scene is valid
                tmp_scene_path = f"{physics_scene_path}_tmp_grasping"
                self._log_once(
                    f"Attempting to duplicate scene '{base_physics_scene.GetPath()}' to '{tmp_scene_path}' for isolated/temp simulation.",
                    "print",
                )
                duplicated_prim = grasping_utils.duplicate_prim(
                    stage, str(base_physics_scene.GetPath()), tmp_scene_path
                )

                if duplicated_prim and duplicated_prim.IsA(UsdPhysics.Scene):
                    self._temp_grasping_physics_scene = UsdPhysics.Scene(duplicated_prim)
                    self._log_once(
                        f"Using temporary duplicated scene: {self._temp_grasping_physics_scene.GetPath()}", "print"
                    )
                    physics_scene = self._temp_grasping_physics_scene  # Simulate using the temp scene

                    # Check if isolation is needed ON the temp scene
                    if isolate_simulation:
                        self._log_once(
                            f"Isolating gripper/object within temporary scene: {physics_scene.GetPath()}", "print"
                        )
                        prims_to_isolate = []
                        if self.gripper_prim:
                            prims_to_isolate.append(self.gripper_prim)
                        if self._object_prim_path:
                            object_prim = self.get_object_prim()
                            if object_prim:
                                prims_to_isolate.append(object_prim)
                        # Call the utility function
                        isolated_rigid_body_prims = grasping_utils.isolate_prims_to_scene(
                            prims_to_isolate, physics_scene
                        )
                        if isolated_rigid_body_prims:
                            is_isolated = True
                        else:
                            self._log_once(
                                f"Failed to isolate prims to temporary scene {physics_scene.GetPath()}, simulation will run non-isolated in this temp scene.",
                                "warn",
                            )
                    else:
                        self._log_once(
                            f"Running non-isolated simulation in temporary scene: {physics_scene.GetPath()}", "print"
                        )
                else:
                    self._log_once(
                        f"Failed to duplicate scene '{base_physics_scene.GetPath()}' to '{tmp_scene_path}'. Falling back to default scene simulation.",
                        "warn",
                    )
                    physics_scene = None  # Fallback
                    self._temp_grasping_physics_scene = None
            else:
                self._log_once(
                    f"Could not get valid base UsdPhysics.Scene at '{physics_scene_path}'. Using default scene simulation.",
                    "warn",
                )
                physics_scene = None  # Fallback
        else:
            self._log_once("No custom physics scene path provided. Using default scene simulation.", "print")
            physics_scene = None  # Use default

        # --- Simulation Execution --- #
        try:
            for i, phase in enumerate(self.grasp_phases):
                phase_name = phase.name
                joint_drive_targets = phase.joint_drive_targets
                simulation_steps = phase.simulation_steps
                simulation_step_dt = phase.simulation_step_dt

                # Set joint targets before starting simulation for this phase
                for joint_path, target_position in joint_drive_targets.items():
                    joint_prim = stage.GetPrimAtPath(joint_path)
                    if not joint_prim.IsValid():
                        carb.log_warn(f"Joint at path '{joint_path}' is not valid.")
                        continue
                    grasping_utils.set_joint_drive_parameters(
                        joint_prim, target_value=target_position, target_type="position"
                    )

                # Choose simulation method
                if simulate_using_timeline:
                    await grasping_utils.advance_timeline_async(
                        num_frames=simulation_steps,
                        render=render,
                    )
                else:
                    # Direct physics simulation using the determined scene (could be temp or None)
                    await grasping_utils.simulate_physics_async(
                        num_frames=simulation_steps,
                        step_dt=simulation_step_dt,
                        physics_scene=physics_scene,
                        render=render,
                    )

            return True
        finally:
            # Cleanup temp scene if it was created and used
            if self._temp_grasping_physics_scene:
                scene_to_clear = self._temp_grasping_physics_scene
                scene_path = scene_to_clear.GetPath()
                self._log_once(f"Cleaning up temporary grasping physics scene: {scene_path}", "print")
                # Clear owners only if isolation was actually performed
                if is_isolated:
                    self._log_once(
                        f"Removing {len(isolated_rigid_body_prims)} owners from temporary scene {scene_path}.", "print"
                    )
                    grasping_utils.clear_isolated_simulation_owners(isolated_rigid_body_prims, scene_to_clear)
                # Delete the temporary scene prim
                if stage and stage.GetPrimAtPath(scene_path):
                    stage.RemovePrim(scene_path)
                    self._log_once(f"Removed temporary grasping physics scene: {scene_path}", "print")
                # Clear the reference
                self._temp_grasping_physics_scene = None

    async def simulate_single_grasp_phase(
        self,
        phase_identifier: str | int,
        render: bool = True,
        physics_scene_path: str | None = None,
        isolate_simulation: bool = False,
        simulate_using_timeline: bool = False,
    ) -> bool:
        """Simulate a single grasp phase by name or index.

        Args:
            phase_identifier: Name of the grasp phase to simulate (case-insensitive) or index (integer)
            render: Whether to render/update Kit for every simulation frame
            physics_scene_path: Optional path to a specific UsdPhysics.Scene prim. If None or invalid,
                                the default physics scene is used.
            isolate_simulation: If True and a valid custom `physics_scene_path` is provided,
                              only the gripper and object prims will be added as owners to that scene.
                              *Ignored if simulate_using_timeline is True.*
            simulate_using_timeline: If True, use the main timeline for simulation instead of direct physics steps.

        Returns:
            True if simulation was successful, False otherwise
        """
        phase = None
        if isinstance(phase_identifier, int):
            phase = self.get_grasp_phase_by_index(phase_identifier)
            if not phase:
                carb.log_warn(f"Grasp phase at index {phase_identifier} not found.")
                return False
        else:
            phase = self.get_grasp_phase_by_name(phase_identifier, ignore_case=True)
            if not phase:
                carb.log_warn(f"Grasp phase '{phase_identifier}' not found.")
                return False

        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("Cannot simulate single grasp phase: Stage is not available.")
            return False

        physics_scene = None  # Scene to be used for simulation call
        base_physics_scene = None  # User-provided scene
        isolated_rigid_body_prims = []  # Prims added if isolating
        is_isolated = False  # Flag if owners were added
        self._temp_grasping_physics_scene = None  # Clear any previous temp scene reference

        if simulate_using_timeline:
            print(f"Simulating phase '{phase.name}' using timeline (custom scene path ignored).")
            # Timeline simulation uses default scene logic implicitly
        elif physics_scene_path:
            # Try to get the user-specified scene
            base_physics_scene = grasping_utils.get_physics_scene(stage, physics_scene_path)
            if base_physics_scene:
                # Always duplicate if base scene is valid
                tmp_scene_path = f"{physics_scene_path}_tmp_grasping"
                print(f"Attempting to duplicate scene '{base_physics_scene.GetPath()}' to '{tmp_scene_path}'")
                duplicated_prim = grasping_utils.duplicate_prim(
                    stage, str(base_physics_scene.GetPath()), tmp_scene_path
                )

                if duplicated_prim and duplicated_prim.IsA(UsdPhysics.Scene):
                    self._temp_grasping_physics_scene = UsdPhysics.Scene(duplicated_prim)
                    print(f"Using temporary duplicated scene: {self._temp_grasping_physics_scene.GetPath()}")
                    physics_scene = self._temp_grasping_physics_scene  # Simulate using the temp scene

                    # Check if isolation is needed ON the temp scene
                    if isolate_simulation:
                        print(f"Isolating gripper/object within temporary scene: {physics_scene.GetPath()}")
                        prims_to_isolate = []
                        if self.gripper_prim:
                            prims_to_isolate.append(self.gripper_prim)
                        if self._object_prim_path:
                            object_prim = self.get_object_prim()
                            if object_prim:
                                prims_to_isolate.append(object_prim)
                        # Call the utility function
                        isolated_rigid_body_prims = grasping_utils.isolate_prims_to_scene(
                            prims_to_isolate, physics_scene
                        )
                        if isolated_rigid_body_prims:
                            is_isolated = True
                        else:
                            carb.log_warn(
                                f"Failed to isolate prims to temporary scene {physics_scene.GetPath()}, simulation will run non-isolated in this temp scene."
                            )
                    else:
                        print(f"Running non-isolated simulation in temporary scene: {physics_scene.GetPath()}")
                else:
                    carb.log_warn(
                        f"Failed to duplicate scene '{base_physics_scene.GetPath()}' to '{tmp_scene_path}'. Falling back to default scene simulation."
                    )
                    physics_scene = None  # Fallback
                    self._temp_grasping_physics_scene = None
            else:
                carb.log_warn(
                    f"Could not get valid base UsdPhysics.Scene at '{physics_scene_path}'. Using default scene simulation."
                )
                physics_scene = None  # Fallback
        else:
            print("No custom physics scene path provided. Using default scene simulation.")
            physics_scene = None  # Use default

        # --- Simulation Execution --- #
        try:
            joint_drive_targets = phase.joint_drive_targets
            simulation_steps = phase.simulation_steps
            simulation_step_dt = phase.simulation_step_dt

            # Set joint targets before simulation
            for joint_path, target_position in joint_drive_targets.items():
                joint_prim = stage.GetPrimAtPath(joint_path)
                if not joint_prim.IsValid():
                    carb.log_warn(f"Joint at path '{joint_path}' is not valid.")
                    continue
                grasping_utils.set_joint_drive_parameters(
                    joint_prim, target_value=target_position, target_type="position"
                )

            # Choose simulation method
            if simulate_using_timeline:
                await grasping_utils.advance_timeline_async(
                    num_frames=simulation_steps,
                    render=render,
                )
            else:
                # Direct physics simulation using the determined scene (could be temp or None)
                await grasping_utils.simulate_physics_async(
                    num_frames=simulation_steps,
                    step_dt=simulation_step_dt,
                    physics_scene=physics_scene,
                    render=render,
                )

            print(f"Single phase '{phase.name}' simulation completed.")
            return True
        finally:
            # Cleanup temp scene if it was created and used
            if self._temp_grasping_physics_scene:
                scene_to_clear = self._temp_grasping_physics_scene
                scene_path = scene_to_clear.GetPath()
                print(f"Cleaning up temporary grasping physics scene: {scene_path}")
                # Clear owners only if isolation was actually performed
                if is_isolated:
                    print(f"Removing {len(isolated_rigid_body_prims)} owners from temporary scene.")
                    grasping_utils.clear_isolated_simulation_owners(isolated_rigid_body_prims, scene_to_clear)
                # Delete the temporary scene prim
                if stage and stage.GetPrimAtPath(scene_path):
                    stage.RemovePrim(scene_path)
                    print(f"Removed temporary grasping physics scene: {scene_path}")
                # Clear the reference
                self._temp_grasping_physics_scene = None

    async def evaluate_grasp_poses(
        self,
        grasp_poses: list[tuple[Gf.Vec3d, Gf.Quatd]],
        render: bool = True,
        physics_scene_path: str | None = None,
        isolate_simulation: bool = False,
        simulate_using_timeline: bool = False,
        progress_callback: callable = None,
    ) -> None:
        """Evaluate a list of grasp poses by simulating the grasp phases for each.

        Args:
            grasp_poses: A list of tuples, where each tuple contains (location, orientation) for a grasp pose.
            render: Whether to render/update Kit for every simulation frame during simulation phases.
            physics_scene_path: Optional path to a specific UsdPhysics.Scene prim for simulation.
            isolate_simulation: Isolate gripper/object to the scene if `physics_scene_path` is used. Ignored if `simulate_using_timeline` is True.
            simulate_using_timeline: Use the main timeline for simulation.
            progress_callback: Optional async callable that takes the number of evaluated poses as an argument.
        """
        initial_pose = self.get_initial_gripper_pose()
        self._write_frame_counter = 0  # Reset counter for this workflow
        self._workflow_stop_requested = False  # Reset stop flag at the start of a new workflow
        self._workflow_printed_messages.clear()  # Clear printed messages for new workflow
        self._first_write_failure_logged_this_workflow = False  # Reset write failure log flag

        # Run the workflow for each grasp pose
        for idx, (world_location, world_orientation) in enumerate(grasp_poses):
            if self._workflow_stop_requested:
                print("Workflow stopped by request during evaluation loop.")
                break
            print(f"  Executing grasp {idx + 1}/{len(grasp_poses)}")
            await self.evaluate_grasp_pose(
                world_location,
                world_orientation,
                clear_simulation=True,
                render=render,
                isolate_simulation=isolate_simulation,
                physics_scene_path=physics_scene_path,
                simulate_using_timeline=simulate_using_timeline,
            )
            if progress_callback:
                await progress_callback(idx + 1)

        # Reset to initial pose after the loop finishes or is stopped
        if initial_pose:
            self.set_gripper_pose(initial_pose[0], initial_pose[1])
        self.clear_simulation(simulate_using_timeline=simulate_using_timeline)
        if self._workflow_stop_requested:
            print("Grasping workflow execution stopped by request.")
        else:
            print("Grasping workflow execution finished.")
        # Reset flag in case it was set during the final phase of a stopped workflow
        self._workflow_stop_requested = False

    async def evaluate_grasp_pose(
        self,
        location: Gf.Vec3d,
        orientation: Gf.Quatd,
        clear_simulation: bool = True,
        render: bool = True,
        physics_scene_path: str | None = None,
        isolate_simulation: bool = False,
        simulate_using_timeline: bool = False,
    ) -> None:
        """Evaluate a single grasp pose.

        Args:
            location: The location of the grasp pose.
            orientation: The orientation of the grasp pose.
            clear_simulation: Whether to clear the simulation before evaluation.
            render: Whether to render/update Kit for every simulation frame.
            physics_scene_path: Optional path to a specific UsdPhysics.Scene prim for simulation.
            isolate_simulation: Isolate gripper/object to the scene if `physics_scene_path` is used. Ignored if `simulate_using_timeline` is True.
            simulate_using_timeline: Use the main timeline for simulation.
        """
        if clear_simulation:
            self.clear_simulation(simulate_using_timeline=simulate_using_timeline)

        self.set_gripper_pose(location, orientation)

        await self.simulate_all_grasp_phases(
            render=render,
            physics_scene_path=physics_scene_path,
            isolate_simulation=isolate_simulation,
            simulate_using_timeline=simulate_using_timeline,
        )

        self.write_grasp_results(location=location, orientation=orientation)

    async def evaluate_grasp_pose_by_index(
        self,
        index: int,
        render: bool = True,
        physics_scene_path: str | None = None,
        isolate_simulation: bool = False,
        simulate_using_timeline: bool = False,
    ) -> None:
        """Evaluate a single grasp pose by index.

        Args:
            index: The index of the grasp pose to evaluate.
            render: Whether to render/update Kit for every simulation frame.
            physics_scene_path: Optional path to a specific UsdPhysics.Scene prim for simulation.
            isolate_simulation: Isolate gripper/object to the scene if `physics_scene_path` is used. Ignored if `simulate_using_timeline` is True.
            simulate_using_timeline: Use the main timeline for simulation.
        """
        if not self.grasp_locations or not self.grasp_orientations:
            carb.log_warn("No grasp poses available to execute.")
            return
        if index < 0 or index >= len(self.grasp_locations):
            carb.log_warn(f"Index {index} is out of bounds for grasp poses (0-{len(self.grasp_locations)-1}).")
            return

        pose = self.get_grasp_pose_at_index(index, in_world_frame=True)
        if pose is None:
            carb.log_warn(f"Could not get world pose for index {index}.")
            return
        location, orientation = pose

        # Pass isolate_simulation and physics_scene_path down
        await self.evaluate_grasp_pose(
            location,
            orientation,
            render=render,
            isolate_simulation=isolate_simulation,
            physics_scene_path=physics_scene_path,
            simulate_using_timeline=simulate_using_timeline,
        )

    # --- Results ---
    def write_grasp_results(self, location: Gf.Vec3d, orientation: Gf.Quatd) -> None:
        """Write the grasp results to the results output path.

        Args:
            location: The location of the evaluated grasp pose.
            orientation: The orientation of the evaluated grasp pose.
        """
        if not self._results_output_dir:
            self._log_once(
                "Results output directory is not set. Grasp results will not be written for this workflow.", "warn"
            )
            return

        if not self.gripper_path:
            carb.log_warn("Cannot write results: Gripper path is not set.")
            return

        joint_states = grasping_utils.get_gripper_joint_states(self.gripper_path)

        if joint_states is None:
            carb.log_warn(f"Could not retrieve joint states for {self.gripper_path}, skipping result writing.")
            return

        result_data = {
            "grasp_result": {
                "gripper_path": self.gripper_path,
                "object_path": self.get_object_prim_path(),
                "gripper_location": list(location),
                "gripper_orientation": {"w": orientation.GetReal(), "xyz": list(orientation.GetImaginary())},
                "joint_states": joint_states,
            }
        }

        try:
            # Ensure the base output directory exists
            output_dir = self._results_output_dir
            os.makedirs(output_dir, exist_ok=True)

            # Construct the filename using the counter
            output_filename = f"capture_{self._write_frame_counter}.yaml"
            full_output_path = os.path.join(output_dir, output_filename)

            # --- Overwrite Logic --- #
            # If not overwriting, find the next available index
            if not self._overwrite_results_output:
                current_index = self._write_frame_counter
                while os.path.exists(full_output_path):
                    current_index += 1
                    output_filename = f"capture_{current_index}.yaml"
                    full_output_path = os.path.join(output_dir, output_filename)
                self._write_frame_counter = current_index
            # If overwriting, full_output_path already uses self._write_frame_counter

            # Open the specific file for writing (overwrite if exists and overwrite=True, or use new index if overwrite=False)
            with open(full_output_path, "w") as f:
                yaml.dump(result_data, f, default_flow_style=False)
            print(f"Successfully wrote grasp results to {full_output_path}")
            self._write_frame_counter += 1  # Increment base counter for the next pose in this workflow run
        except Exception as e:
            carb.log_warn(f"Failed to write grasp results to {full_output_path}: {e}")

    # --- Utilities ---
    def set_gripper_pose(self, location: Gf.Vec3d, orientation: Gf.Quatd) -> None:
        """Set the internal gripper prim's pose to the given location and orientation.

        Args:
            location: The translation as a Gf.Vec3d.
            orientation: The orientation as a Gf.Quatd.
        """
        if not self._gripper_prim:
            carb.log_warn("No valid gripper prim set in GraspingManager.")
            return
        transform_utils.set_transform_attributes(self._gripper_prim, location=location, orientation=orientation)

    def move_gripper_to_grasp_pose(self, index: int, in_world_frame: bool = True) -> None:
        """Set the gripper pose to the grasp pose at the given index.

        Args:
            index: The index of the grasp pose to move to.
            in_world_frame: Whether to use world frame coordinates for the pose.
        """
        pose = self.get_grasp_pose_at_index(index, in_world_frame=in_world_frame)
        if pose:
            location, orientation = pose
            self.set_gripper_pose(location, orientation)
        else:
            carb.log_warn(
                f"Could not retrieve grasp pose at index {index} {'(world)' if in_world_frame else '(local)'}."
            )

    def store_initial_gripper_pose(self, location: Gf.Vec3d = None, orientation: Gf.Quatd = None) -> None:
        """Store the initial/default gripper pose. If not provided, use the current gripper prim pose if available.

        Args:
            location: The translation as a Gf.Vec3d.
            orientation: The orientation as a Gf.Quatd.
        """
        if location is not None and orientation is not None:
            self._initial_gripper_location = location
            self._initial_gripper_orientation = orientation
        elif self._gripper_prim is not None:
            try:
                current_loc, current_orient = transform_utils.get_prim_world_pose(self._gripper_prim)
                self._initial_gripper_location = current_loc
                self._initial_gripper_orientation = current_orient
            except Exception as e:
                carb.log_warn(
                    f"Could not get current world pose for gripper '{self.gripper_path}' to set initial pose: {e}"
                )
                self._initial_gripper_location = None
                self._initial_gripper_orientation = None
        else:
            self._initial_gripper_location = None
            self._initial_gripper_orientation = None

    def get_initial_gripper_pose(self) -> tuple[Gf.Vec3d, Gf.Quatd] | None:
        """The stored initial pose as (location, orientation), or None if not set.

        Returns:
            A tuple containing the initial gripper location and orientation, or None if not set.
        """
        if self._initial_gripper_location is None or self._initial_gripper_orientation is None:
            self.store_initial_gripper_pose()
            if self._initial_gripper_location is None or self._initial_gripper_orientation is None:
                return None
        return self._initial_gripper_location, self._initial_gripper_orientation

    # --- Grasp Pose Generation and Management ---
    def generate_grasp_poses(
        self,
        config: dict = None,
    ) -> bool:
        """Generate and store the local grasp poses for the current object_prim.

        Args:
            config: Dictionary containing grasp generation parameters. If None, uses the class's
                   sampler_config. Configuration includes:
                - sampler_type: String specifying which grasp sampler to use (default: "antipodal")
                - num_candidates: Target number of grasp candidates to attempt to sample.
                - gripper_maximum_aperture: Maximum width between gripper fingers in meters.
                - gripper_standoff_fingertips: Distance from fingertip contacts to gripper origin.
                - num_orientations: Number of orientations to sample per grasp axis.
                - orientation_sample_axis: Gripper's local axis for orientation sampling.
                - grasp_align_axis: Gripper's local axis to align with the physical grasp line.
                - gripper_approach_direction: Local approach direction vector for the gripper.
                - lateral_sigma: Std deviation for lateral perturbation along grasp axis.
                - random_seed: Seed for random number generation.
                - verbose: Enable detailed logging.

        Returns:
            True if grasp generation was successful, False otherwise.
        """
        object_prim = self.get_object_prim()
        if not object_prim:
            carb.log_warn(
                f"Cannot generate grasp poses: Object prim at path '{self._object_prim_path}' is not valid or not found."
            )
            return False

        mesh_prim = grasping_utils.find_first_mesh_in_hierarchy(object_prim)
        if not mesh_prim:
            carb.log_warn(f"Cannot generate grasp poses: No mesh found for '{self.get_object_prim_path()}'.")
            return False

        # Use provided config or the class's sampler_config
        sampler_config = self.sampler_config.copy() if config is None else config.copy()
        sampler_config["mesh_prim"] = mesh_prim

        # Convert random_seed: -1 to None for numpy compatibility
        if sampler_config.get("random_seed") == -1:
            sampler_config["random_seed"] = None

        # Select the appropriate sampler based on the config
        sampler_type = sampler_config.get("sampler_type", "antipodal")

        try:
            # Choose the sampling function based on sampler_type
            if sampler_type == "antipodal":
                locations, quaternions = sampler_utils.generate_grasp_poses(sampler_config)
            else:
                carb.log_warn(f"Unknown sampler type: {sampler_type}")
                return False

            if locations and quaternions:
                self.grasp_locations = locations
                self.grasp_orientations = quaternions
                return True
            else:
                self.clear_grasp_poses()
                carb.log_warn(f"Grasp pose generation failed for '{self._object_prim_path}'.")
                return False

        except Exception as e:
            self.clear_grasp_poses()
            warn_msg = f"Failed to generate grasp poses for '{self._object_prim_path}': {e}"
            if "spatialindex" in str(e).lower():
                warn_msg += "\n  'libspatialindex' is required for grasp sampling. Install via 'sudo apt install libspatialindex-dev' on Ubuntu."
            carb.log_warn(warn_msg)
            return False

    def clear_grasp_poses(self) -> None:
        """Clear the stored grasp locations and orientations."""
        self.grasp_locations = []
        self.grasp_orientations = []

    def get_grasp_poses(self, in_world_frame: bool = False) -> list[tuple[Gf.Vec3d, Gf.Quatd]]:
        """Get the stored grasp poses, optionally transforming them to the world frame.

        Args:
            in_world_frame: If True, transform poses to world frame using the object prim's
                            current transform.

        Returns:
            List of tuples, where each tuple is (location: Gf.Vec3d, orientation: Gf.Quatd).
            Returns an empty list if required data (object prim path, local poses) is missing or object prim is invalid.
        """
        if not self.grasp_locations or not self.grasp_orientations:
            return []

        if in_world_frame:
            if not self._object_prim_path:
                carb.log_warn("Cannot get world grasp poses: Object prim path is not set.")
                return []

            object_prim = self.get_object_prim()
            if not object_prim:
                carb.log_warn(
                    f"Cannot get world grasp poses: Object prim at path '{self._object_prim_path}' is not valid or not found."
                )
                return []

            try:
                world_poses = transform_utils.transform_local_poses_to_world(
                    list(zip(self.grasp_locations, self.grasp_orientations)), object_prim
                )
                return world_poses
            except Exception as e:
                carb.log_warn(f"Failed to transform local poses to world for '{self.object_path}': {e}")
                return []
        else:
            # Return local poses
            return list(zip(self.grasp_locations, self.grasp_orientations))

    def get_grasp_pose_at_index(self, index: int, in_world_frame: bool = False) -> tuple[Gf.Vec3d, Gf.Quatd] | None:
        """Retrieve a single grasp pose by its index, optionally transforming it to world frame.

        Args:
            index: The index of the grasp pose to retrieve.
            in_world_frame: If True, the pose is transformed to world coordinates using the
                            current object prim's transform.

        Returns:
            A tuple (location: Gf.Vec3d, orientation: Gf.Quatd) representing the grasp pose,
            or None if the index is out of bounds or transformation to world frame fails.
        """
        num_poses = len(self.grasp_locations)
        if not 0 <= index < num_poses:
            carb.log_warn(f"Index {index} is out of bounds for stored grasp poses (0-{num_poses-1}).")
            return None

        local_location = self.grasp_locations[index]
        local_orientation = self.grasp_orientations[index]

        if not in_world_frame:
            return local_location, local_orientation
        else:
            if not self._object_prim_path:
                carb.log_warn(f"Cannot transform pose at index {index} to world frame: Object prim path is not set.")
                return None

            object_prim = self.get_object_prim()
            if not object_prim:
                carb.log_warn(
                    f"Cannot transform pose at index {index} to world frame " f"for object '{self.object_path}': {e}"
                )
                return None

            try:
                world_pose = transform_utils.transform_local_pose_to_world(
                    pose=(local_location, local_orientation), prim=object_prim
                )
                return world_pose
            except Exception as e:
                carb.log_warn(
                    f"Failed to transform pose at index {index} to world frame " f"for object '{self.object_path}': {e}"
                )
                return None

    def update_joint_pregrasp_states_from_current(self, joint_info_list: list[dict] | None = None) -> None:
        """Update the internal joint_pregrasp_states by querying the current state of joints.

        Uses the currently set gripper path to find joints and populates the
        `joint_pregrasp_states` dictionary with their current positions.

        Args:
            joint_info_list: Optional list of joint info dictionaries (containing at least 'path').
                               If provided, paths are extracted from here. If None, info is fetched
                               using `grasping_utils.get_gripper_joints_info`.
        """
        if not self.gripper_path:
            carb.log_warn("Cannot update pregrasp states: Gripper path is not set.")
            self.joint_pregrasp_states = {}
            return

        local_joint_info_list = joint_info_list
        if local_joint_info_list is None:
            # Fetch info if not provided
            local_joint_info_list = grasping_utils.get_gripper_joints_info(self.gripper_path)
            if local_joint_info_list is None:  # Handle potential error from fetching
                carb.log_warn("Failed to get gripper joint info while updating pregrasp states.")
                self.joint_pregrasp_states = {}
                return

        # Extract paths from the (provided or fetched) list
        joint_paths = [info["path"] for info in local_joint_info_list]

        # Call the utility function to get the states
        self.joint_pregrasp_states = grasping_utils.populate_joint_pregrasp_states_from_current(joint_paths)
