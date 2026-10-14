# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Preflight validation for interactive SysID runs."""

from __future__ import annotations

import importlib.util
import math
import os
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .actuator_compatibility import (
    ACTUATOR_RUNTIME_AUTO,
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_IMPLICIT,
    ACTUATOR_RUNTIME_MIXED,
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    PHYSICS_BACKEND_AUTO,
    PHYSICS_BACKEND_NEWTON,
    PHYSICS_BACKEND_PHYSX,
    is_actuator_delay_parameter,
    resolve_simulation_compatibility,
)
from .articulation_utils import find_articulation_root
from .backend_capabilities import (
    RESIDUAL_CONTACT_FORCE,
    RESIDUAL_END_EFFECTOR_POSE,
    RESIDUAL_POSITION,
    RESIDUAL_TORQUE,
    RESIDUAL_VELOCITY,
    resolve_backend_capabilities,
)
from .errors import SysIdRuntimeUnavailableError
from .ingest.config_types import (
    TrajectoryIngestError,
    TrajectorySourceType,
    load_topic_mapping_file,
)
from .optimizer_config import OptimizerBackend
from .parameter_types import SysIdParameterType
from .run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    SIMULATION_ENGINE_NEWTON,
    SysIdRunSpec,
)
from .torque_semantics import EFFORT_SEMANTICS_LINK_SIDE, resolve_effort_semantics
from .training_logs import find_companion_mcap_path
from .trajectory_csv import TrajectoryDataset

SourceExistsFn = Callable[[TrajectorySourceType, str, str], bool]


@dataclass(frozen=True)
class SysIdPreflightIssue:
    """One run-readiness issue reported before SysID execution."""

    code: str
    message: str
    severity: str = "error"


@dataclass
class SysIdRunPreflight:
    """Aggregated preflight result used by the UI and controller."""

    issues: list[SysIdPreflightIssue] = field(default_factory=list)

    @property
    def ok(self) -> bool:  # noqa: D102
        return not any(issue.severity == "error" for issue in self.issues)

    @property
    def errors(self) -> list[SysIdPreflightIssue]:  # noqa: D102
        return [issue for issue in self.issues if issue.severity == "error"]

    @property
    def warnings(self) -> list[SysIdPreflightIssue]:  # noqa: D102
        return [issue for issue in self.issues if issue.severity != "error"]

    def add_error(self, code: str, message: str) -> None:  # noqa: D102
        self.issues.append(SysIdPreflightIssue(code=code, message=message, severity="error"))

    def add_warning(self, code: str, message: str) -> None:  # noqa: D102
        self.issues.append(SysIdPreflightIssue(code=code, message=message, severity="warning"))

    def summary_lines(self) -> list[str]:  # noqa: D102
        if not self.issues:
            return ["Preflight ready."]
        prefix = {"error": "ERROR", "warning": "WARN"}
        return [f"{prefix.get(issue.severity, issue.severity.upper())}: {issue.message}" for issue in self.issues]

    def raise_for_errors(self) -> None:  # noqa: D102
        if self.ok:
            return
        raise ValueError("SysID preflight failed: " + "; ".join(issue.message for issue in self.errors))


def preflight_sysid_run_spec(
    spec: SysIdRunSpec,
    *,
    stage=None,  # noqa: ANN001
    trajectory: TrajectoryDataset | None = None,
    timeline_playing: bool | None = None,
    source_exists_fn: SourceExistsFn | None = None,
    allow_empty_parameters: bool = False,
) -> SysIdRunPreflight:
    """Validate SysID run settings before building the optimizer and bridge.

    Args:
        spec: SysID run specification.
        stage: Optional USD stage used for robot and backend checks.
        trajectory: Optional loaded telemetry used for signal-shape checks.
        timeline_playing: Current timeline state, when known.
        source_exists_fn: Optional source-availability callback for UI or test environments.
        allow_empty_parameters: Permit an empty parameter list for staged configuration flows.

    Returns:
        Aggregated errors and warnings describing run readiness.
    """  # noqa: DOC107
    result = SysIdRunPreflight()
    _check_telemetry(spec, result, trajectory=trajectory, source_exists_fn=source_exists_fn)
    _check_stage_and_robot(spec, result, stage=stage, trajectory=trajectory)
    _check_parameters(spec, result, allow_empty=allow_empty_parameters)
    _check_solver(spec, result)
    _check_residuals(spec, result, trajectory=trajectory)
    _check_simulation(spec, result, stage=stage)
    _check_outputs(spec, result)
    if (
        timeline_playing is False
        and spec.simulation.engine == SIMULATION_ENGINE_ISAAC_SIM
        and not spec.simulation.offline_stepping
    ):
        result.add_error("timeline_not_playing", "Press Play on the timeline before running interactive SysID.")
    return result


def _check_telemetry(
    spec: SysIdRunSpec,
    result: SysIdRunPreflight,
    *,
    trajectory: TrajectoryDataset | None,
    source_exists_fn: SourceExistsFn | None,
) -> None:
    try:
        source_type = TrajectorySourceType(spec.telemetry.source_type)
    except ValueError:
        result.add_error("telemetry_source_type", f"Unsupported telemetry source type: {spec.telemetry.source_type}")
        return

    source_path = spec.telemetry.source_path.strip()
    if source_type == TrajectorySourceType.LEROBOT:
        if not source_path:
            result.add_error("telemetry_source_path", "A local LeRobot dataset directory is required.")
        try:
            pyarrow_available = importlib.util.find_spec("pyarrow") is not None
        except (ImportError, ValueError):
            pyarrow_available = False
        if not pyarrow_available:
            result.add_error(
                "lerobot_pyarrow_missing",
                "LeRobot ingestion requires PyArrow in the active Python environment.",
            )
        try:
            episode_index = int(spec.telemetry.lerobot_episode_index)
        except (TypeError, ValueError):
            result.add_error("lerobot_episode_index", "LeRobot episode index must be an integer.")
        else:
            if episode_index < 0:
                result.add_error("lerobot_episode_index", "LeRobot episode index must be non-negative.")
    elif not source_path:
        result.add_error("telemetry_source_path", "Telemetry source path is required.")

    mapping_path = spec.telemetry.mapping_path.strip()
    if source_type in (TrajectorySourceType.MCAP, TrajectorySourceType.ROS2_BAG) and not mapping_path:
        result.add_error(
            "mapping_path",
            "MCAP and ROS 2 bag telemetry require a topic mapping JSON/YAML file.",
        )
    if mapping_path and not Path(mapping_path).expanduser().exists():
        label = (
            "Topic mapping"
            if source_type in (TrajectorySourceType.MCAP, TrajectorySourceType.ROS2_BAG)
            else "Mapping config"
        )
        result.add_error("mapping_path", f"{label} does not exist: {mapping_path}")
    elif mapping_path and source_type in (TrajectorySourceType.MCAP, TrajectorySourceType.ROS2_BAG):
        try:
            load_topic_mapping_file(mapping_path)
        except ValueError as exc:
            result.add_error("mapping_invalid", str(exc))

    chunk_manifest_path = spec.telemetry.chunk_manifest_path.strip()
    if chunk_manifest_path and not Path(chunk_manifest_path).expanduser().exists():
        result.add_error("chunk_manifest_path", f"Chunk manifest path does not exist: {chunk_manifest_path}")

    if source_path and source_type in (
        TrajectorySourceType.CSV,
        TrajectorySourceType.MCAP,
        TrajectorySourceType.ROS2_BAG,
        TrajectorySourceType.LEROBOT,
    ):
        source = Path(source_path).expanduser()
        if source_exists_fn is not None:
            source_available = bool(source_exists_fn(source_type, source_path, mapping_path))
        else:
            source_available = source.exists()
        if not source_available:
            result.add_error("telemetry_source_missing", f"Telemetry source does not exist: {source_path}")
        elif source.exists():
            # A host callback answers only the existence question, so structural checks
            # still run whenever the recording is reachable on this filesystem.
            _check_local_source_layout(source_type, source, result)

    if trajectory is not None:
        times = np.asarray(trajectory.times, dtype=np.float64).reshape(-1)
        positions = np.asarray(trajectory.positions, dtype=np.float64)
        sample_count = int(times.shape[0])
        if sample_count < 2:
            result.add_error("telemetry_samples", "Telemetry must contain at least two samples.")
        positions_valid = positions.ndim == 2 and positions.shape[0] == sample_count
        if not positions_valid:
            result.add_error("telemetry_shape", "Telemetry positions must have shape (T, N) matching times.")
        elif positions.shape[1] < 1:
            result.add_error("telemetry_joints", "Telemetry must contain at least one joint.")

        expected_joint_shape = positions.shape if positions_valid else None
        joint_channels = {
            "velocity": trajectory.velocities,
            "command": trajectory.commands,
            "torque": trajectory.torques,
        }
        for name, values in joint_channels.items():
            if values is None:
                continue
            shape = np.asarray(values).shape
            if expected_joint_shape is None or shape != expected_joint_shape:
                result.add_error(
                    "telemetry_shape",
                    f"Telemetry {name} must have shape (T, N) matching positions.",
                )

        row_channels = {
            "end_effector_pose": trajectory.end_effector_poses,
            "contact_force": trajectory.contact_forces,
        }
        for name, values in row_channels.items():
            if values is None:
                continue
            shape = np.asarray(values).shape
            if len(shape) != 2 or shape[0] != sample_count:
                result.add_error(
                    "telemetry_shape",
                    f"Telemetry {name} must have shape (T, D) matching times.",
                )
        if trajectory.residual_sample_weights is not None:
            weight_shape = np.asarray(trajectory.residual_sample_weights).shape
            if weight_shape != (sample_count,):
                result.add_error(
                    "telemetry_shape",
                    "Telemetry residual sample weights must have shape (T,) matching times.",
                )

        if sample_count >= 2 and np.all(np.isfinite(times)) and np.any(np.diff(times) <= 0.0):
            result.add_error("telemetry_time_order", "Telemetry timestamps must be strictly increasing.")

        channels = {
            "time": times,
            "position": positions,
            "velocity": trajectory.velocities,
            "command": trajectory.commands,
            "torque": trajectory.torques,
            "end_effector_pose": trajectory.end_effector_poses,
            "contact_force": trajectory.contact_forces,
        }
        for name, values in channels.items():
            if values is not None and not np.all(np.isfinite(np.asarray(values, dtype=np.float64))):
                result.add_error("telemetry_nonfinite", f"Telemetry {name} channel contains NaN or Inf.")


def _check_local_source_layout(
    source_type: TrajectorySourceType,
    source: Path,
    result: SysIdRunPreflight,
) -> None:
    """Validate the on-disk layout of a telemetry source that exists locally.

    Args:
        source_type: Declared telemetry backend.
        source: Existing local telemetry path.
        result: Preflight result collecting layout errors.
    """
    if source_type == TrajectorySourceType.LEROBOT:
        _check_lerobot_layout(source, result)
    elif source_type in (TrajectorySourceType.MCAP, TrajectorySourceType.ROS2_BAG):
        _check_recording_layout(source_type, source, result)


def _check_lerobot_layout(source: Path, result: SysIdRunPreflight) -> None:
    """Require the dataset files that LeRobot ingestion reads.

    Args:
        source: Existing local LeRobot dataset root.
        result: Preflight result collecting layout errors.
    """
    if not source.is_dir():
        result.add_error("telemetry_source_path", f"LeRobot source must be a directory: {source}")
    elif not (source / "meta" / "info.json").is_file():
        result.add_error(
            "lerobot_info_missing",
            f"LeRobot info.json does not exist: {source / 'meta' / 'info.json'}",
        )
    elif not any((source / "data").glob("**/*.parquet")):
        result.add_error("lerobot_data_missing", f"No LeRobot parquet files found under: {source / 'data'}")


def _check_recording_layout(
    source_type: TrajectorySourceType,
    source: Path,
    result: SysIdRunPreflight,
) -> None:
    """Resolve a recording exactly as ingestion would, so surprises surface before the run.

    A run directory that holds several recordings, or none, passes a plain existence
    check and then fails minutes later inside the loader.

    Args:
        source_type: Either the MCAP or the ROS 2 bag backend.
        source: Existing local recording file or directory.
        result: Preflight result collecting resolution errors.
    """
    if source_type == TrajectorySourceType.MCAP:
        try:
            resolved = Path(find_companion_mcap_path(str(source))).expanduser()
        except ValueError as exc:
            result.add_error("telemetry_source_ambiguous", str(exc))
            return
        if not resolved.is_file():
            result.add_error("telemetry_source_missing", f"No MCAP recording was found under: {source}")
        return

    # Imported lazily so the portable package root stays free of ingestion machinery.
    from .ingest.ros2_bag import BAG_METADATA_FILE, resolve_ros2_bag_recording

    try:
        recording = resolve_ros2_bag_recording(source)
    except TrajectoryIngestError as exc:
        result.add_error("telemetry_source_ambiguous", str(exc))
        return
    if recording.is_dir() and not (
        any(recording.glob("*.mcap")) or any(recording.glob("*.db3")) or (recording / BAG_METADATA_FILE).is_file()
    ):
        result.add_error("telemetry_source_missing", f"No ROS 2 bag recording was found under: {recording}")


def _check_stage_and_robot(
    spec: SysIdRunSpec,
    result: SysIdRunPreflight,
    *,
    stage,  # noqa: ANN001
    trajectory: TrajectoryDataset | None,
) -> None:
    alignment = float(getattr(spec.telemetry, "command_alignment_seconds", 0.0))
    if not math.isfinite(alignment):
        result.add_error("command_alignment", "Telemetry command alignment must be finite.")
    robot_path = spec.simulation.robot_prim_path.strip()
    if not robot_path:
        result.add_error("robot_prim_path", "Robot prim path is required.")

    stage_path = spec.stage.input_path.strip()
    if stage is None:
        if not stage_path:
            # Without either source the robot prim and its articulation root are never
            # checked, so reporting the run as ready would be a false green.
            if robot_path:
                result.add_error(
                    "stage_input_path",
                    "No open stage was provided and stage.input_path is empty, so the robot articulation at "
                    f"'{robot_path}' cannot be validated. Open the stage or set stage.input_path.",
                )
        elif "://" not in stage_path and not Path(stage_path).expanduser().is_file():
            result.add_error("stage_input_path", f"USD stage does not exist: {stage_path}")
        return

    if not robot_path:
        return
    prim = stage.GetPrimAtPath(robot_path)
    if not prim or not prim.IsValid():
        result.add_error("robot_prim_path", f"Robot prim path is not valid: {robot_path}")
        return
    root = find_articulation_root(stage, robot_path)
    if root is None:
        result.add_error("articulation_root", f"No articulation root found under: {robot_path}")
        return


def _check_parameters(spec: SysIdRunSpec, result: SysIdRunPreflight, *, allow_empty: bool = False) -> None:
    selected = list(spec.parameters.selected)
    if not selected:
        if allow_empty:
            return
        result.add_error("parameters_selected", "Select at least one parameter to optimize.")
        return
    compatibility = resolve_simulation_compatibility(spec.simulation)
    capabilities = resolve_backend_capabilities(spec.simulation)
    for item in selected:
        label = item.param_type
        try:
            ptype = SysIdParameterType(item.param_type)
        except ValueError:
            result.add_error("parameter_type", f"Unsupported parameter type: {label}")
            continue
        explicit_actuators = compatibility.uses_explicit_actuators
        if capabilities is not None and ptype not in capabilities.parameter_types:
            if capabilities.key == "isaac_sim_newton" and ptype == SysIdParameterType.JOINT_FRICTION:
                result.add_error(
                    "isaac_sim_newton_joint_friction",
                    "Isaac-hosted Newton does not expose joint-friction properties through its tensor API. "
                    "Use the standalone Newton engine for friction identification, or select the PhysX backend.",
                )
            else:
                supported = ", ".join(sorted(value.value for value in capabilities.parameter_types))
                result.add_error(
                    "parameter_backend_unsupported",
                    f"{capabilities.label} does not support parameter '{ptype.value}'. Supported parameters: {supported}.",
                )
        if is_actuator_delay_parameter(ptype) and not compatibility.supports_delay:
            result.add_error(
                "command_delay_engine",
                "actuator_command_delay_seconds requires actuator_runtime='newton_explicit' or 'mixed' "
                "with a non-differentiable actuator pipeline.",
            )
        if ptype == SysIdParameterType.JOINT_INTEGRAL_GAIN:
            if not explicit_actuators or not compatibility.supports_pid:
                result.add_error(
                    "joint_integral_gain_engine",
                    "joint_integral_gain requires an explicit PD/PID actuator pipeline.",
                )
            elif (
                spec.simulation.engine == SIMULATION_ENGINE_NEWTON
                and spec.simulation.newton.actuator_source != "usd"
                and spec.simulation.newton.controller != "pid"
            ):
                result.add_error(
                    "joint_integral_gain_controller",
                    "joint_integral_gain requires simulation.newton.controller = 'pid'.",
                )
        if item.min >= item.max:
            result.add_error("parameter_bounds", f"{label}: min must be less than max.")
        if not (item.min <= item.initial <= item.max):
            result.add_error("parameter_initial", f"{label}: initial value must be inside its bounds.")


def _is_finite_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _check_solver(spec: SysIdRunSpec, result: SysIdRunPreflight) -> None:
    solver = spec.solver
    if (
        isinstance(solver.max_iterations, bool)
        or not isinstance(solver.max_iterations, int)
        or solver.max_iterations < 1
    ):
        result.add_error("solver_iterations", "Max iterations must be at least 1.")
    if (
        isinstance(solver.max_rollout_steps, bool)
        or not isinstance(solver.max_rollout_steps, int)
        or solver.max_rollout_steps < 2
    ):
        result.add_error("solver_rollout_steps", "Max rollout steps must be at least 2.")
    if not _is_finite_number(solver.epsilon) or solver.epsilon <= 0:
        result.add_error("solver_epsilon", "Finite-difference epsilon must be positive.")
    if not _is_finite_number(solver.damping_initial) or solver.damping_initial <= 0:
        result.add_error("solver_damping", "Initial damping must be positive.")
    if not _is_finite_number(solver.cma_sigma) or solver.cma_sigma <= 0:
        result.add_error("solver_cma_sigma", "CMA-ES sigma must be positive.")
    if solver.bo_initial_samples < 1:
        result.add_error("solver_bo_initial_samples", "BO initial samples must be at least 1.")
    if solver.bo_candidate_count < 1:
        result.add_error("solver_bo_candidate_count", "BO candidate count must be at least 1.")
    if solver.bo_batch_size < 1:
        result.add_error("solver_bo_batch_size", "BO batch size must be at least 1.")
    has_delay = any(is_actuator_delay_parameter(item.param_type) for item in spec.parameters.selected)
    if has_delay and solver.optimizer == OptimizerBackend.LEVENBERG_MARQUARDT.value:
        result.add_warning(
            "command_delay_lm_backend",
            "actuator_command_delay_seconds is quantized to actuator steps in Newton; "
            "prefer auto, CMA-ES, or Bayesian.",
        )


def _check_residuals(
    spec: SysIdRunSpec,
    result: SysIdRunPreflight,
    *,
    trajectory: TrajectoryDataset | None,
) -> None:
    residuals = spec.residuals
    channel_weights = {
        RESIDUAL_POSITION: ("position", residuals.position_weight),
        RESIDUAL_VELOCITY: ("velocity", residuals.velocity_weight),
        RESIDUAL_TORQUE: ("torque", residuals.torque_weight),
        RESIDUAL_END_EFFECTOR_POSE: ("end-effector pose", residuals.end_effector_pose_weight),
        RESIDUAL_CONTACT_FORCE: ("contact force", residuals.contact_force_weight),
    }
    for name, value in channel_weights.values():
        if not _is_finite_number(value) or value < 0:
            result.add_error("residual_weight", f"Residual weight for {name} must be finite and non-negative.")
    if not any(_is_finite_number(value) and value > 0 for _name, value in channel_weights.values()):
        result.add_error("residual_weight", "At least one residual channel weight must be positive.")
    capabilities = resolve_backend_capabilities(spec.simulation)
    if capabilities is not None:
        for channel, (name, value) in channel_weights.items():
            if _is_finite_number(value) and value > 0 and not capabilities.supports_residual(channel):
                supported = ", ".join(sorted(capabilities.residual_channels))
                result.add_error(
                    f"residual_backend_{channel}",
                    f"{capabilities.label} does not produce the simulated {name} channel required by this residual. "
                    f"Supported residual channels: {supported}.",
                )
    if trajectory is not None:
        if residuals.torque_weight > 0 and trajectory.torques is None:
            result.add_error("residual_torque", "Torque residual is enabled, but telemetry has no torque channel.")
        elif residuals.torque_weight > 0:
            semantics = resolve_effort_semantics(trajectory)
            if semantics != EFFORT_SEMANTICS_LINK_SIDE:
                result.add_error(
                    "residual_torque_semantics",
                    "Torque residual requires link-side joint torque telemetry "
                    f"(torque_semantics='link_side'); this dataset declares '{semantics}'. "
                    "External-torque estimates must use a contact/external-force objective instead.",
                )
        if residuals.end_effector_pose_weight > 0 and trajectory.end_effector_poses is None:
            result.add_error(
                "residual_end_effector",
                "End-effector pose residual is enabled, but telemetry has no end-effector pose channel.",
            )
        if residuals.contact_force_weight > 0 and trajectory.contact_forces is None:
            result.add_error(
                "residual_contact",
                "Contact-force residual is enabled, but telemetry has no contact-force channel.",
            )
        if residuals.sample_weights is not None and len(residuals.sample_weights) != int(trajectory.times.shape[0]):
            result.add_error("residual_sample_weights", "Sample weights must match the telemetry window exactly.")


def _check_simulation(spec: SysIdRunSpec, result: SysIdRunPreflight, *, stage) -> None:  # noqa: ANN001
    sim = spec.simulation
    if sim.engine not in (SIMULATION_ENGINE_ISAAC_SIM, SIMULATION_ENGINE_NEWTON):
        result.add_error("simulation_engine", f"Unsupported simulation.engine: {sim.engine}")
        return
    if sim.physics_backend not in (PHYSICS_BACKEND_AUTO, PHYSICS_BACKEND_PHYSX, PHYSICS_BACKEND_NEWTON):
        result.add_error("physics_backend", f"Unsupported simulation.physics_backend: {sim.physics_backend}")
        return
    if sim.actuator_runtime not in (
        ACTUATOR_RUNTIME_AUTO,
        ACTUATOR_RUNTIME_IMPLICIT,
        ACTUATOR_RUNTIME_EXPLICIT,
        ACTUATOR_RUNTIME_MIXED,
    ):
        result.add_error("actuator_runtime", f"Unsupported simulation.actuator_runtime: {sim.actuator_runtime}")
        return
    if sim.explicit_actuator_policy not in (
        EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
        EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    ):
        result.add_error(
            "explicit_actuator_policy",
            f"Unsupported simulation.explicit_actuator_policy: {sim.explicit_actuator_policy}",
        )
        return
    capabilities = resolve_backend_capabilities(sim)
    if capabilities is None:
        supported_solvers = ", ".join((NEWTON_SOLVER_MUJOCO, NEWTON_SOLVER_FEATHERSTONE_DIFF))
        result.add_error(
            "backend_capabilities",
            "No System Identification backend supports the requested simulation combination. "
            f"For standalone Newton, simulation.newton.solver must be one of: {supported_solvers}.",
        )
        return
    compatibility = resolve_simulation_compatibility(sim)
    if sim.engine == SIMULATION_ENGINE_NEWTON:
        if compatibility.physics_backend != PHYSICS_BACKEND_NEWTON:
            result.add_error(
                "physics_backend_engine",
                "Standalone simulation.engine='newton' requires physics_backend='newton' or 'auto'.",
            )
        if (
            str(sim.newton.solver) == NEWTON_SOLVER_FEATHERSTONE_DIFF
            and compatibility.actuator_runtime != ACTUATOR_RUNTIME_EXPLICIT
        ):
            result.add_error(
                "actuator_runtime_solver",
                "Differentiable Featherstone uses an explicit mapped-joint PD pipeline; "
                "select actuator_runtime='newton_explicit' or 'auto'.",
            )
        _check_newton_simulation(spec, result, stage=stage)
        return
    if compatibility.physics_backend == PHYSICS_BACKEND_NEWTON:
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            available = {name for name, _active in SimulationManager.get_available_physics_engines()}
        except Exception as exc:
            result.add_error(
                "isaac_sim_newton_unavailable",
                "Isaac Sim Newton physics was requested, but runtime availability could not be verified: "
                f"{exc}. Enable isaacsim.physics.newton and retry.",
            )
        else:
            if PHYSICS_BACKEND_NEWTON not in available:
                result.add_error(
                    "isaac_sim_newton_unavailable",
                    "Isaac Sim Newton physics was requested but is not registered; enable isaacsim.physics.newton.",
                )
    if sim.parallel_clones:
        source_env_path = sim.source_env_path.strip()
        env_paths_root = sim.env_paths_root.strip()
        if not source_env_path:
            result.add_error("source_env_path", "Clone source env is required when parallel clones are enabled.")
        if not env_paths_root:
            result.add_error("env_paths_root", "Clone env root is required when parallel clones are enabled.")
        if source_env_path and env_paths_root and source_env_path.rstrip("/") == env_paths_root.rstrip("/"):
            result.add_error("clone_paths", "Clone source env must be a specific env, not the clone env root.")
        param_count = len(spec.parameters.selected)
        if param_count > 4 and not sim.co_locate_clones:
            result.add_warning(
                "clone_grid_size",
                f"Spaced clone grid with {param_count + 1} envs is slow and can stress PhysX GPU memory.",
            )
        if sim.fabric_clones:
            result.add_warning(
                "fabric_clones",
                "Experimental Fabric clone authoring is enabled; viewport clone articulation motion may not reflect "
                "the direct PhysX tensor rollout state. Disable it if clone creation or cleanup fails.",
            )
    elif len(spec.parameters.selected) > 1:
        result.add_warning(
            "sequential_rollouts",
            "Parallel clones are disabled, so multi-parameter optimization will run sequential rollouts.",
        )


def _check_newton_simulation(spec: SysIdRunSpec, result: SysIdRunPreflight, *, stage) -> None:  # noqa: ANN001
    sim = spec.simulation
    cfg = sim.newton
    if cfg.actuator_source not in ("drive_defaults", "usd"):
        result.add_error(
            "newton_actuator_source",
            "Newton SysID supports simulation.newton.actuator_source values 'drive_defaults' and 'usd'.",
        )
    compatibility = resolve_simulation_compatibility(sim)
    if cfg.actuator_source == "usd" and compatibility.actuator_runtime == ACTUATOR_RUNTIME_IMPLICIT:
        result.add_error(
            "newton_actuator_source_runtime",
            "simulation.newton.actuator_source='usd' requires actuator_runtime='newton_explicit' or 'mixed'; "
            "implicit_drive uses the joint drive baseline.",
        )
    if cfg.controller not in ("pd", "pid"):
        result.add_error("newton_controller", "Newton SysID supports controller values 'pd' and 'pid'.")
    if cfg.effort_clamp not in ("max_effort", "dc_motor", "none"):
        result.add_error(
            "newton_effort_clamp",
            "Newton SysID supports effort_clamp values 'max_effort', 'dc_motor', and 'none'.",
        )
    if cfg.actuator_dt_mode != "physics_dt":
        result.add_error("newton_actuator_dt_mode", "Newton v1 SysID supports only actuator_dt_mode = 'physics_dt'.")
    if str(cfg.solver) == NEWTON_SOLVER_FEATHERSTONE_DIFF:
        if cfg.actuator_source != "drive_defaults":
            result.add_error(
                "newton_diff_actuator_source",
                "Differentiable Featherstone requires simulation.newton.actuator_source='drive_defaults'.",
            )
        if cfg.controller != "pd":
            result.add_error(
                "newton_diff_controller",
                "Differentiable Featherstone implements explicit PD control and requires "
                "simulation.newton.controller='pd'.",
            )
        if cfg.effort_clamp not in ("none", "max_effort"):
            result.add_error(
                "newton_diff_effort_clamp",
                "Differentiable Featherstone supports effort_clamp='none' or 'max_effort'.",
            )
    if cfg.actuator_source == "usd":
        _check_newton_usd_actuator_schemas(stage, result)
    try:
        from .newton_sysid_bridge import (
            newton_core_modules_available,
            newton_modules_available,
        )

        availability_check = (
            newton_core_modules_available
            if str(cfg.solver) == NEWTON_SOLVER_FEATHERSTONE_DIFF
            else newton_modules_available
        )
        available, reason = availability_check()
    except Exception as exc:
        available, reason = False, str(exc)
    if not available:
        result.add_error("newton_unavailable", f"Newton simulation.engine selected but Newton is unavailable: {reason}")


def _check_newton_usd_actuator_schemas(stage, result: SysIdRunPreflight) -> None:  # noqa: ANN001
    if stage is None:
        return
    try:
        from .newton_sysid_bridge import (
            _attr_asset_paths,
            _load_newton_modules,
            classify_newton_neural_prim,
        )
    except ModuleNotFoundError as exc:
        if exc.name != f"{__package__}.newton_sysid_bridge":
            raise
        modules = None
        _attr_asset_paths = None
        classify_newton_neural_prim = None
    else:
        try:
            modules = _load_newton_modules()
        except SysIdRuntimeUnavailableError:
            modules = None
    claimed_targets: dict[str, str] = {}
    for prim in stage.Traverse():
        try:
            applied = set(prim.GetAppliedSchemas())
        except Exception:
            applied = set()
        controller_schemas = applied & {
            "NewtonPDControlAPI",
            "NewtonPIDControlAPI",
            "NewtonNeuralControlAPI",
        }
        try:
            relation = prim.GetRelationship("newton:targets")
            targets = [str(path) for path in relation.GetTargets()] if relation else []
        except Exception:
            targets = []
        if controller_schemas or targets:
            if len(controller_schemas) != 1:
                result.add_error(
                    "newton_actuator_controller_schema",
                    f"Newton actuator {prim.GetPath()} must apply exactly one supported controller schema; "
                    f"got {sorted(controller_schemas) or ['none']}.",
                )
            if len(targets) != 1:
                result.add_error(
                    "newton_actuator_target",
                    f"Newton actuator {prim.GetPath()} must target exactly one joint; got {targets}.",
                )
            elif targets[0] in claimed_targets:
                result.add_error(
                    "newton_actuator_duplicate_target",
                    f"Newton actuators {claimed_targets[targets[0]]} and {prim.GetPath()} both target {targets[0]}.",
                )
            else:
                claimed_targets[targets[0]] = str(prim.GetPath())
            clamps = applied & {"NewtonMaxEffortClampingAPI", "NewtonDCMotorClampingAPI"}
            if len(clamps) > 1:
                result.add_error(
                    "newton_actuator_clamp_schema",
                    f"Newton actuator {prim.GetPath()} applies multiple effort-clamping schemas: {sorted(clamps)}.",
                )
        if "NewtonNeuralControlAPI" in applied:
            if _attr_asset_paths is None or classify_newton_neural_prim is None or modules is None:
                continue
            model_path, resolved_model_path = _attr_asset_paths(prim, "newton:modelPath")
            if not model_path:
                result.add_error(
                    "newton_neural_model_path",
                    f"Newton neural actuator {prim.GetPath()} is missing newton:modelPath.",
                )
                continue
            local_model_path = _resolve_newton_model_asset_path(prim, model_path, resolved_model_path)
            if local_model_path and not Path(local_model_path).expanduser().is_file():
                result.add_error(
                    "newton_neural_model_path",
                    f"Newton neural actuator model does not exist: {local_model_path}",
                )
                continue
            neural = classify_newton_neural_prim(modules.actuators, prim)
            if neural["unsupported_schemas"]:
                result.add_error(
                    "newton_unsupported_neural_actuator",
                    "Newton SysID supports only neural MLP/LSTM actuators in this pass: "
                    + ", ".join(neural["unsupported_schemas"]),
                )
        if "NewtonPositionBasedClampingAPI" in applied:
            result.add_error(
                "newton_unsupported_usd_actuator_schema",
                "Newton SysID v1 does not execute NewtonPositionBasedClampingAPI.",
            )


def _resolve_newton_model_asset_path(prim, model_path: str, resolved_model_path: str) -> str:  # noqa: ANN001
    if resolved_model_path:
        return resolved_model_path
    if not model_path or "://" in model_path:
        return ""
    path = Path(model_path).expanduser()
    if path.is_absolute():
        return str(path)
    try:
        stage = prim.GetStage()
        layer = stage.GetRootLayer()
        root = getattr(layer, "realPath", "") or getattr(layer, "identifier", "")
    except Exception:
        root = ""
    if not root or "://" in root:
        return ""
    return str((Path(root).expanduser().parent / path).resolve())


def _check_outputs(spec: SysIdRunSpec, result: SysIdRunPreflight) -> None:
    outputs = spec.outputs
    thresholds = {
        "validation_relative_change_threshold": outputs.validation_relative_change_threshold,
        "validation_absolute_change_threshold": outputs.validation_absolute_change_threshold,
    }
    for name, value in thresholds.items():
        if not _is_finite_number(value) or value < 0.0:
            result.add_error(name, f"{name} must be finite and non-negative.")
    if outputs.export_provenance_sidecar and outputs.provenance_path:
        _check_parent_writable(outputs.provenance_path, result, "provenance_path", "Provenance sidecar")
    if outputs.render_validation_animation:
        try:
            fps = int(outputs.validation_animation_fps)
        except (TypeError, ValueError):
            fps = 0
        try:
            max_frames = int(outputs.validation_animation_max_frames)
        except (TypeError, ValueError):
            max_frames = 0
        if fps < 1:
            result.add_error("validation_animation_fps", "Validation animation FPS must be >= 1.")
        if max_frames < 1:
            result.add_error("validation_animation_max_frames", "Validation animation max frames must be >= 1.")
        if outputs.validation_animation_path:
            _check_parent_writable(
                outputs.validation_animation_path,
                result,
                "validation_animation_path",
                "Validation animation",
            )


def _check_parent_writable(path: str, result: SysIdRunPreflight, code: str, label: str) -> None:
    if "://" in path:
        return
    parent = Path(path).expanduser().parent
    if not parent.exists():
        result.add_warning(code, f"{label} parent directory does not exist yet: {parent}")
        return
    if not os.access(parent, os.W_OK):
        result.add_error(code, f"{label} parent directory is not writable: {parent}")
