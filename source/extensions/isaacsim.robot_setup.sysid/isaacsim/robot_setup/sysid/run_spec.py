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

"""SysID run settings shared by the GUI, controller, and reports."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, TypeVar

from .actuator_compatibility import (
    ACTUATOR_RUNTIME_AUTO,
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    PHYSICS_BACKEND_AUTO,
)
from .ingest.config_types import TrajectoryLoadConfig, TrajectorySourceType
from .optimizer_config import OptimizerBackend, OptimizerBackendConfig
from .parameter_types import (
    GLOBAL_DOF_INDEX,
    GLOBAL_LINK_INDEX,
    ParameterCategory,
    SysIdParameterEntry,
    SysIdParameterType,
)
from .residual_config import ResidualWeightConfig
from .trajectory_resampling import SamplingConfig
from .trajectory_segments import TelemetryChunkRunSpec

SYSID_RUN_SPEC_SCHEMA_VERSION = 2
SIMULATION_ENGINE_ISAAC_SIM = "isaac_sim"
SIMULATION_ENGINE_NEWTON = "newton"
NEWTON_SOLVER_MUJOCO = "mujoco"
NEWTON_SOLVER_FEATHERSTONE_DIFF = "featherstone_diff"

_DataclassT = TypeVar("_DataclassT")


@dataclass
class StageRunSpec:
    """Current USD stage metadata for run reports."""

    input_path: str = ""


@dataclass
class TimeWindowRunSpec:
    """Inclusive telemetry time window."""

    start: float = 0.0
    end: float = 0.0


@dataclass
class TelemetryRunSpec:
    """Trajectory source and mapping settings."""

    source_type: str = TrajectorySourceType.CSV.value
    source_path: str = ""
    mapping_path: str = ""
    chunk_manifest_path: str = ""
    time_window: TimeWindowRunSpec = field(default_factory=TimeWindowRunSpec)
    chunks: list[TelemetryChunkRunSpec] = field(default_factory=list)
    lerobot_episode_index: int = 0
    # Automatic train/validation split (used when `chunks` is empty).
    auto_split: bool = False
    auto_split_train_fraction: float = 0.8
    auto_split_min_chunk_seconds: float = 1.0
    # Common-mode command-to-response alignment removed at load, uniform
    # across joints: command'(t) = command(t - delay). Corrects a data artifact
    # (trajectory-generator smoothing/lookahead), not per-joint actuator latency;
    # the telemetry quality report suggests this value from cross-correlation.
    command_alignment_seconds: float = 0.0

    def resolved_command_alignment_seconds(self) -> float:
        """Return the import-time command alignment.

        Returns:
            Alignment duration in seconds.
        """
        return float(self.command_alignment_seconds)

    def to_load_config(self) -> TrajectoryLoadConfig:
        """Build the trajectory loader configuration for this telemetry source.

        Returns:
            Loader configuration normalized to the selected telemetry source.
        """
        source_type = TrajectorySourceType(self.source_type)
        mapping_path = self.mapping_path or None
        if source_type == TrajectorySourceType.CSV:
            config = TrajectoryLoadConfig.csv(self.source_path, column_mapping_path=mapping_path)
        elif source_type == TrajectorySourceType.ROS2_BAG:
            config = TrajectoryLoadConfig.ros2_bag(self.source_path, topic_mapping_path=mapping_path)
        elif source_type == TrajectorySourceType.MCAP:
            config = TrajectoryLoadConfig.mcap(self.source_path, topic_mapping_path=mapping_path)
        else:
            config = TrajectoryLoadConfig.lerobot(
                self.source_path,
                episode_index=self.lerobot_episode_index,
            )
        config.command_alignment_seconds = self.resolved_command_alignment_seconds()
        return config


@dataclass
class NewtonSimulationRunSpec:
    """Newton actuator rollout settings."""

    device: str = "cpu"
    # ``drive_defaults`` permits drive-derived fallback configs while still
    # honoring authored actuator prims. ``usd`` is strict: every explicit DOF
    # must have one valid authored or promoted actuator prim.
    actuator_source: str = "drive_defaults"
    controller: str = "pd"
    effort_clamp: str = "max_effort"
    actuator_dt_mode: str = "physics_dt"
    # Rollout backend: "mujoco" (default, Newton SolverMuJoCo over replicated
    # worlds) or "featherstone_diff" (contact-free differentiable bridge for
    # gradient-based optimization). Simulator failures are never substituted.
    solver: str = NEWTON_SOLVER_MUJOCO
    # Symplectic-Euler substeps per command sample (featherstone_diff only).
    featherstone_substeps: int = 4
    # MuJoCo physics steps per command sample (mujoco only). MuJoCo integrates
    # drive stiffness explicitly; raise this when a stiff drive diverges at the
    # telemetry rate. Controls and explicit actuators still update per sample.
    mujoco_substeps: int = 1
    # Controller feedforward model applied to the mapped joints during
    # featherstone_diff and mujoco rollouts and included in the reported torque
    # channel:
    # "none" (raw PD, e.g. motor-side PD on legged robots), "gravity" (static
    # gravity compensation), or "inverse_dynamics" (full nominal inverse dynamics
    # at the measured trajectory — matches computed-torque/impedance controllers
    # such as the Franka's, whose measured torques are almost entirely
    # feedforward). The feedforward is computed from the BASELINE inertial model
    # (like the real controller's fixed internal model), so it does not depend on
    # the identified parameters.
    feedforward: str = "none"
    # Capture fixed-shape Newton rollouts into CUDA graphs and replay them on
    # later calls. Featherstone captures forward + backward tape execution;
    # MuJoCo captures reset + control scatter + solver steps + state gathering.
    # Effective only on CUDA devices; CPU paths are unchanged. Any capture
    # failure logs its reason and falls back to uncaptured rollouts.
    cuda_graph_capture: bool = True


@dataclass
class SimulationRunSpec:
    """Isaac Sim bridge settings."""

    engine: str = SIMULATION_ENGINE_ISAAC_SIM
    robot_prim_path: str = "/World/envs/env_0"
    source_env_path: str = "/World/envs/env_0"
    env_paths_root: str = "/World/envs/env"
    parallel_clones: bool = True
    co_locate_clones: bool = True
    fabric_clones: bool = False
    # Advance physics directly without rendering or pumping full Kit frames.
    # Set this to false to retain timeline-driven stepping for interactive debugging.
    offline_stepping: bool = True
    # Orthogonal execution axes. ``auto`` selects PhysX for Isaac Sim and
    # Newton for the standalone ``engine='newton'`` route.
    physics_backend: str = PHYSICS_BACKEND_AUTO
    # ``implicit_drive``, ``newton_explicit``, or ``mixed``. ``auto`` maps the
    # Isaac Sim runtime to implicit drives and standalone MuJoCo to mixed
    # per-DOF ownership.
    actuator_runtime: str = ACTUATOR_RUNTIME_AUTO
    # Missing explicit actuators can be rejected or temporarily promoted.
    explicit_actuator_policy: str = EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY
    newton: NewtonSimulationRunSpec = field(default_factory=NewtonSimulationRunSpec)


@dataclass
class ParameterSpaceRunSpec:
    """Flags that define the parameter registry generated for a robot."""

    include_basic: bool = True
    include_com_offsets: bool = False
    include_inertia_log_cholesky: bool = False
    include_joint_limit_scales: bool = False
    include_per_link_mass: bool = False
    include_command_delay: bool = False


@dataclass
class ParameterRunSpec:
    """One selected optimizer parameter and its bounds."""

    param_type: str
    dof_index: int = GLOBAL_DOF_INDEX
    link_index: int = GLOBAL_LINK_INDEX
    component_index: int = 0
    category: str = ""
    initial: float = 1.0
    min: float = 0.0
    max: float = 1.0

    @classmethod
    def from_entry(
        cls,
        entry: SysIdParameterEntry,
        *,
        initial: float,
        min_value: float,
        max_value: float,
    ) -> ParameterRunSpec:
        """Create a run-spec row from a registry parameter entry and bounds.

        Args:
            entry: Parameter addressing and category metadata.
            initial: Initial optimizer value.
            min_value: Inclusive lower bound.
            max_value: Inclusive upper bound.

        Returns:
            Serializable parameter row preserving entry addressing.
        """
        return cls(
            param_type=entry.param_type.value,
            dof_index=entry.dof_index,
            link_index=entry.link_index,
            component_index=entry.component_index,
            category=entry.resolved_category().value,
            initial=float(initial),
            min=float(min_value),
            max=float(max_value),
        )

    def to_entry(self) -> SysIdParameterEntry:
        """Convert this serialized parameter row back to a registry entry.

        Returns:
            Registry entry reconstructed from this serialized row.
        """
        category = ParameterCategory(self.category) if self.category else None
        return SysIdParameterEntry(
            param_type=SysIdParameterType(self.param_type),
            dof_index=int(self.dof_index),
            link_index=int(self.link_index),
            component_index=int(self.component_index),
            category=category,
        )


@dataclass
class ParametersRunSpec:
    """Parameter registry flags plus selected rows."""

    space: ParameterSpaceRunSpec = field(default_factory=ParameterSpaceRunSpec)
    selected: list[ParameterRunSpec] = field(default_factory=list)


@dataclass
class SolverRunSpec:
    """Optimizer and solver hyperparameters."""

    optimizer: str = OptimizerBackend.AUTO.value
    damping_initial: float = 1e-2
    epsilon: float = 1e-4
    max_iterations: int = 20
    max_rollout_steps: int = 300
    use_analytical_jacobian: bool = False
    use_analytical_presolve: bool = False
    cma_population_size: int | None = None
    cma_sigma: float = 0.3
    cma_batch_size: int | None = None
    cma_seed: int = 0
    bo_initial_samples: int = 5
    bo_candidate_count: int = 64
    bo_batch_size: int = 4
    bo_seed: int = 0
    gd_learning_rate: float = 0.05
    gd_num_restarts: int = 1
    gd_seed: int = 0

    def to_backend_config(self) -> OptimizerBackendConfig:
        """Convert solver fields into the optimizer backend configuration.

        Returns:
            Optimizer configuration populated from solver fields.
        """
        return OptimizerBackendConfig(
            backend=OptimizerBackend(self.optimizer),
            cma_population_size=self.cma_population_size,
            cma_sigma=float(self.cma_sigma),
            cma_batch_size=self.cma_batch_size,
            cma_seed=int(self.cma_seed),
            bo_initial_samples=int(self.bo_initial_samples),
            bo_candidate_count=int(self.bo_candidate_count),
            bo_batch_size=int(self.bo_batch_size),
            bo_seed=int(self.bo_seed),
            gd_learning_rate=float(self.gd_learning_rate),
            gd_num_restarts=int(self.gd_num_restarts),
            gd_seed=int(self.gd_seed),
            use_analytical_jacobian=bool(self.use_analytical_jacobian),
            use_analytical_presolve=bool(self.use_analytical_presolve),
        )


@dataclass
class SamplingRunSpec:
    """Trajectory resampling policy before physics rollout."""

    mode: str = "auto"
    target_dt_mode: str = "physics_dt_capped"
    command_interpolation: str = "zero_order_hold"
    measured_interpolation: str = "linear"
    auto_dt_ratio_threshold: float = 2.0
    auto_jitter_threshold: float = 0.10
    max_resampled_steps: int = 0

    def to_config(self) -> SamplingConfig:
        """Convert serialized sampling settings into a runtime config.

        Returns:
            Runtime sampling configuration.
        """
        return SamplingConfig.from_dict(asdict(self))


@dataclass
class ResidualsRunSpec:
    """Residual channel and sample weighting."""

    position_weight: float = 1.0
    velocity_weight: float = 1.0
    torque_weight: float = 0.0
    end_effector_pose_weight: float = 0.0
    contact_force_weight: float = 0.0
    sample_weights: list[float] | None = None
    end_effector_link_index: int = -1

    @classmethod
    def from_config(cls, config: ResidualWeightConfig) -> ResidualsRunSpec:
        """Create a serializable residual spec from a runtime config.

        Args:
            config: Runtime residual weighting configuration.

        Returns:
            Serializable residual configuration.
        """
        return cls(
            position_weight=float(config.position_weight),
            velocity_weight=float(config.velocity_weight),
            torque_weight=float(config.torque_weight),
            end_effector_pose_weight=float(config.end_effector_pose_weight),
            contact_force_weight=float(config.contact_force_weight),
            sample_weights=(config.sample_weights.tolist() if config.sample_weights is not None else None),
            end_effector_link_index=int(config.end_effector_link_index),
        )

    def to_config(self) -> ResidualWeightConfig:
        """Convert serialized residual settings into a runtime config.

        Returns:
            Runtime residual weighting configuration.
        """
        import numpy as np

        return ResidualWeightConfig(
            position_weight=float(self.position_weight),
            velocity_weight=float(self.velocity_weight),
            torque_weight=float(self.torque_weight),
            end_effector_pose_weight=float(self.end_effector_pose_weight),
            contact_force_weight=float(self.contact_force_weight),
            sample_weights=(
                np.asarray(self.sample_weights, dtype=np.float64) if self.sample_weights is not None else None
            ),
            end_effector_link_index=int(self.end_effector_link_index),
        )


@dataclass
class OutputsRunSpec:
    """Run artifacts produced by interactive execution."""

    apply_parameters_to_usd: bool = True
    write_usd_provenance: bool = True
    export_provenance_sidecar: bool = True
    provenance_path: str = ""
    remove_clones_after_run: bool = True
    render_validation_animation: bool = False
    validation_animation_path: str = ""
    validation_animation_fps: int = 24
    validation_animation_max_frames: int = 240
    validation_relative_change_threshold: float = 0.01
    validation_absolute_change_threshold: float = 1.0e-12
    # Post-solve per-parameter sensitivity ("value ± range where cost rises 10%").
    compute_parameter_confidence: bool = True
    parameter_confidence_rollout_budget: int = 2_000_000


@dataclass
class SysIdRunSpec:
    """Complete input contract for a SysID optimization run."""

    schema_version: int = SYSID_RUN_SPEC_SCHEMA_VERSION
    stage: StageRunSpec = field(default_factory=StageRunSpec)
    telemetry: TelemetryRunSpec = field(default_factory=TelemetryRunSpec)
    simulation: SimulationRunSpec = field(default_factory=SimulationRunSpec)
    parameters: ParametersRunSpec = field(default_factory=ParametersRunSpec)
    solver: SolverRunSpec = field(default_factory=SolverRunSpec)
    residuals: ResidualsRunSpec = field(default_factory=ResidualsRunSpec)
    outputs: OutputsRunSpec = field(default_factory=OutputsRunSpec)
    telemetry_quality: dict[str, Any] = field(default_factory=dict)
    sampling: SamplingRunSpec = field(default_factory=SamplingRunSpec)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON/YAML-ready representation of the complete run spec.

        Returns:
            Nested JSON-compatible mapping.
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> SysIdRunSpec:
        """Create a run spec from a JSON/YAML payload.

        Args:
            payload: Serialized input payload.

        Returns:
            Run specification with nested dataclasses.
        """
        if not isinstance(payload, dict):
            raise ValueError("SysID run spec must be a JSON/YAML object.")
        telemetry_payload = _mapping_field(payload, "telemetry")
        telemetry = _dataclass_from_dict(TelemetryRunSpec, telemetry_payload)
        tw = _mapping_field(telemetry_payload, "time_window")
        telemetry.time_window = _dataclass_from_dict(TimeWindowRunSpec, tw)
        chunks_payload = telemetry_payload.get("chunks", None)
        if chunks_payload is None:
            if telemetry.time_window.end > telemetry.time_window.start:
                telemetry.chunks = [
                    TelemetryChunkRunSpec(
                        name="Train 1",
                        role="train",
                        excitation="custom",
                        start=float(telemetry.time_window.start),
                        end=float(telemetry.time_window.end),
                        weight=1.0,
                    )
                ]
            else:
                telemetry.chunks = []
        elif not isinstance(chunks_payload, list):
            raise ValueError("telemetry.chunks must be a list when provided.")
        else:
            for index, item in enumerate(chunks_payload):
                if not isinstance(item, dict):
                    raise ValueError(f"telemetry.chunks[{index}] must be a mapping.")
            telemetry.chunks = [TelemetryChunkRunSpec.from_dict(item) for item in chunks_payload]
        params_payload = _mapping_field(payload, "parameters")
        selected_payload = params_payload.get("selected", [])
        if not isinstance(selected_payload, list):
            raise ValueError("parameters.selected must be a list when provided.")
        for index, item in enumerate(selected_payload):
            if not isinstance(item, dict):
                raise ValueError(f"parameters.selected[{index}] must be a mapping.")
        parameters = ParametersRunSpec(
            space=_dataclass_from_dict(ParameterSpaceRunSpec, _mapping_field(params_payload, "space")),
            selected=[_dataclass_from_dict(ParameterRunSpec, item) for item in selected_payload],
        )
        telemetry_quality = payload.get("telemetry_quality", {})
        if not isinstance(telemetry_quality, dict):
            raise ValueError("telemetry_quality must be a mapping when provided.")
        return cls(
            schema_version=int(payload.get("schema_version", SYSID_RUN_SPEC_SCHEMA_VERSION)),
            stage=_dataclass_from_dict(StageRunSpec, _mapping_field(payload, "stage")),
            telemetry=telemetry,
            simulation=_simulation_from_dict(_mapping_field(payload, "simulation")),
            parameters=parameters,
            solver=_dataclass_from_dict(SolverRunSpec, _mapping_field(payload, "solver")),
            residuals=_dataclass_from_dict(ResidualsRunSpec, _mapping_field(payload, "residuals")),
            outputs=_dataclass_from_dict(OutputsRunSpec, _mapping_field(payload, "outputs")),
            telemetry_quality=dict(telemetry_quality),
            sampling=_dataclass_from_dict(SamplingRunSpec, _mapping_field(payload, "sampling")),
        )

    def to_json(self) -> str:
        """Serialize the complete run spec to formatted JSON text.

        Returns:
            Indented canonical run-spec JSON.
        """
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_json(cls, text: str) -> SysIdRunSpec:
        """Parse a complete run spec from JSON text.

        Args:
            text: JSON object text containing a run specification.

        Returns:
            Parsed and normalized run specification.
        """
        payload = json.loads(text)
        return cls.from_dict(payload)


def _dataclass_from_dict(cls: type[_DataclassT], payload: dict[str, Any] | None) -> _DataclassT:
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        raise ValueError(f"{cls.__name__} payload must be a mapping.")
    allowed = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
    filtered = {key: value for key, value in payload.items() if key in allowed}
    return cls(**filtered)


def _mapping_field(payload: dict[str, Any], key: str) -> dict[str, Any]:
    value = payload.get(key, {})
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping when provided.")
    return value


def _simulation_from_dict(payload: dict[str, Any] | None) -> SimulationRunSpec:
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        raise ValueError("SimulationRunSpec payload must be a mapping.")
    sim = _dataclass_from_dict(SimulationRunSpec, payload)
    newton_payload = _mapping_field(payload, "newton")
    sim.newton = _dataclass_from_dict(NewtonSimulationRunSpec, newton_payload)
    return sim
