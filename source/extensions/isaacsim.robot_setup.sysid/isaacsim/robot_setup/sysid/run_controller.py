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

"""Shared SysID run preparation and execution helpers."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import nullcontext
from dataclasses import dataclass, replace
from importlib import metadata

import carb
import numpy as np
import torch

from .actuator_compatibility import (
    PHYSICS_BACKEND_NEWTON,
    resolve_simulation_compatibility,
)
from .analytical_presolve import (
    AnalyticalPresolveResult,
    build_fixed_base_context_from_usd,
    run_analytical_presolve,
)
from .articulation_utils import (
    collect_robot_link_and_joint_paths,
    order_joint_paths_for_trajectory,
)
from .ingest import load_trajectory
from .optimizer_base import OptimizerConfig
from .optimizer_config import (
    _BACKEND_TO_LABEL,
    OptimizerBackend,
    OptimizerBackendConfig,
    resolve_backend,
)
from .optimizer_factory import create_optimizer
from .parameter_apply import ParameterApplyState
from .parameter_space import ParameterSpace
from .parameter_types import SysIdParameterType
from .preflight import SysIdRunPreflight, preflight_sysid_run_spec
from .regressor import finite_difference_acceleration
from .run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    SIMULATION_ENGINE_NEWTON,
    SysIdRunSpec,
)
from .training_logs import attach_companion_training_details, phase_chunks_from_metadata
from .trajectory_csv import TrajectoryDataset
from .trajectory_resampling import ResamplingDiagnostic, resample_trajectory_for_rollout
from .trajectory_segments import (
    TrajectoryChunk,
    build_auto_chunks,
    build_trajectory_chunks,
    make_default_train_chunk,
    split_train_validation_chunks,
)

OptimizerFactory = Callable[[OptimizerBackend], object]
BridgeFactory = Callable[..., object]
_DEFAULT_PHYSICS_DT = 1.0 / 60.0
_SOLVER_SPECIFIC_USD_PARAMETER_TYPES = {
    SysIdParameterType.JOINT_FRICTION,
    SysIdParameterType.JOINT_ARMATURE,
}


@dataclass
class SysIdPreparedRun:
    """Objects needed to execute one SysID run."""

    raw_trajectory: TrajectoryDataset
    trajectory: TrajectoryDataset
    train_chunks: list[TrajectoryChunk]
    validation_chunks: list[TrajectoryChunk]
    parameter_space: ParameterSpace
    link_paths: list[str]
    joint_paths: list[str]
    param_entries: list
    optimizer: object
    bridge: object
    stage: object
    config: OptimizerConfig
    resolved_backend: OptimizerBackend
    backend_label: str
    preflight: SysIdRunPreflight
    analytical_presolve: AnalyticalPresolveResult | None = None
    resampling_diagnostics: list[ResamplingDiagnostic] | None = None
    simulation_engine: str = "isaac_sim"
    physics_backend: str = ""
    physics_solver: str = ""
    newton_config: dict | None = None
    check_report: object | None = None
    theta_authored_initial: torch.Tensor | None = None
    telemetry_quality: TelemetryQualityReport | None = None


class SysIdRunController:
    """Build and prepare SysID from GUI run settings.

    Args:
        spec: Constructor value for ``spec``.
        optimizer_factory: Constructor value for ``optimizer_factory``.
        bridge_factory: Constructor value for ``bridge_factory``.
    """

    def __init__(
        self,
        spec: SysIdRunSpec,
        *,
        optimizer_factory: OptimizerFactory = create_optimizer,
        bridge_factory: BridgeFactory | None = None,
    ) -> None:
        self.spec = spec
        self._optimizer_factory = optimizer_factory
        self._bridge_factory = bridge_factory

    def preflight(  # noqa: D102
        self,
        *,
        stage=None,  # noqa: ANN001
        trajectory: TrajectoryDataset | None = None,
        timeline_playing: bool | None = None,
        allow_empty_parameters: bool = False,
    ) -> SysIdRunPreflight:
        return preflight_sysid_run_spec(
            self.spec,
            stage=stage,
            trajectory=trajectory,
            timeline_playing=timeline_playing,
            allow_empty_parameters=allow_empty_parameters,
        )

    def load_trajectory(self) -> TrajectoryDataset:  # noqa: D102
        return load_run_trajectory(self.spec)

    def prepare(  # noqa: D102
        self,
        stage,  # noqa: ANN001
        *,
        trajectory: TrajectoryDataset | None = None,
        train_chunks: list[TrajectoryChunk] | None = None,
        validation_chunks: list[TrajectoryChunk] | None = None,
        timeline_playing: bool | None = None,
        allow_empty_parameters: bool = False,
    ) -> SysIdPreparedRun:
        raw_trajectory = trajectory or self.load_trajectory()
        if train_chunks is None or validation_chunks is None:
            train_chunks, validation_chunks = build_segmented_chunks(self.spec, raw_trajectory)
        if not train_chunks:
            raise ValueError("SysID run requires at least one training chunk.")
        train_chunks, validation_chunks, resampling_diagnostics = _resample_chunks_for_rollout(
            train_chunks,
            validation_chunks,
            physics_dt=_physics_dt(stage),
            sampling_config=self.spec.sampling.to_config(),
            max_rollout_steps=max(2, int(self.spec.solver.max_rollout_steps)),
        )
        active_trajectory = train_chunks[0].trajectory
        preflight = self.preflight(
            stage=stage,
            trajectory=raw_trajectory,
            timeline_playing=timeline_playing,
            allow_empty_parameters=allow_empty_parameters,
        )
        preflight.raise_for_errors()

        parameter_space, link_paths, joint_paths = build_parameter_space(self.spec, stage, active_trajectory)
        param_entries, theta_initial, theta_min, theta_max = optimizer_vectors(
            self.spec,
            allow_empty=allow_empty_parameters,
        )
        # Preserve the launch values before analytical presolve can replace the
        # solver seed.  Reports and untuned validation must never call a
        # presolved seed "untuned".
        theta_authored_initial = theta_initial.detach().cpu().clone()
        residual_cfg = self.spec.residuals.to_config()
        backend_cfg = self.spec.solver.to_backend_config()
        backend_cfg.differentiable_bridge = self._uses_differentiable_newton()
        _apply_multichunk_jacobian_policy(backend_cfg, chunk_count=len(train_chunks))
        resolved_backend = resolve_backend(backend_cfg, len(param_entries), residual_cfg, param_entries)
        backend_label = _BACKEND_TO_LABEL.get(resolved_backend, resolved_backend.value)
        analytical_presolve = None
        theta_fallback = None
        if backend_cfg.use_analytical_presolve:
            analytical_presolve = _apply_analytical_presolve(
                stage,
                parameter_space,
                link_paths,
                joint_paths,
                active_trajectory,
                train_chunks,
                param_entries,
                theta_initial,
                theta_min,
                theta_max,
                residual_cfg,
                resolved_backend,
                max_rollout_steps=max(2, int(self.spec.solver.max_rollout_steps)),
            )
            seed = analytical_presolve.theta_seed
            if not torch.equal(
                seed.detach().cpu().to(dtype=torch.float32),
                theta_initial.detach().cpu().to(dtype=torch.float32),
            ):
                theta_fallback = theta_initial
            theta_initial = seed

        optimizer = self._optimizer_factory(resolved_backend)
        optimizer.set_residual_config(residual_cfg)
        if hasattr(optimizer, "set_parameter_space"):
            optimizer.set_parameter_space(parameter_space)

        bridge = self._create_bridge(
            stage=stage,
            parameter_space=parameter_space,
            link_paths=link_paths,
            joint_paths=joint_paths,
            residual_cfg=residual_cfg,
        )
        bridge.set_trajectory(active_trajectory)
        validate = getattr(bridge, "validate_parameter_entries", None)
        if callable(validate):
            validate(param_entries)
        optimizer.set_bridge(bridge)

        config = OptimizerConfig(
            trajectory=active_trajectory,
            param_entries=param_entries,
            theta_initial=theta_initial,
            theta_min=theta_min,
            theta_max=theta_max,
            epsilon=float(self.spec.solver.epsilon),
            damping_initial=float(self.spec.solver.damping_initial),
            max_iterations=int(self.spec.solver.max_iterations),
            max_rollout_steps=max(2, int(self.spec.solver.max_rollout_steps)),
            use_analytical_jacobian=backend_cfg.use_analytical_jacobian,
            num_links=max(1, len(link_paths)),
            residual_weight_config=residual_cfg,
            backend_config=backend_cfg,
            training_segments=train_chunks,
            theta_fallback=theta_fallback,
            theta_authored_initial=theta_authored_initial,
        )

        newton_config = None
        compatibility = resolve_simulation_compatibility(self.spec.simulation)
        if compatibility.physics_backend == PHYSICS_BACKEND_NEWTON:
            newton_config = dict(self.spec.simulation.newton.__dict__)
            newton_config["dependency_versions"] = _newton_dependency_versions()
            newton_config["runtime"] = compatibility.runtime
            newton_config["physics_backend"] = compatibility.physics_backend
            newton_config["actuator_runtime"] = compatibility.actuator_runtime
            actuator_metadata = getattr(bridge, "actuator_metadata", None)
            if isinstance(actuator_metadata, dict):
                newton_config.update(actuator_metadata)

        check_report = None
        try:
            from .check_report import build_sysid_check_report

            check_report = build_sysid_check_report(
                self.spec,
                stage=stage,
                trajectory=raw_trajectory,
                train_chunks=train_chunks,
                validation_chunks=validation_chunks,
            )
            for line in check_report.summary_lines():
                if "[warning]" in line or "[error]" in line:
                    carb.log_warn(f"SysId check: {line}")
                else:
                    carb.log_info(f"SysId check: {line}")
        except Exception as exc:
            cleanup = getattr(bridge, "cleanup", None)
            if callable(cleanup):
                cleanup(remove_clones=True)
            raise RuntimeError(f"SysId pre-solve check report failed: {exc}") from exc

        return SysIdPreparedRun(
            raw_trajectory=raw_trajectory,
            trajectory=active_trajectory,
            train_chunks=train_chunks,
            validation_chunks=validation_chunks,
            parameter_space=parameter_space,
            link_paths=link_paths,
            joint_paths=joint_paths,
            param_entries=param_entries,
            optimizer=optimizer,
            bridge=bridge,
            stage=stage,
            config=config,
            resolved_backend=resolved_backend,
            backend_label=backend_label,
            preflight=preflight,
            analytical_presolve=analytical_presolve,
            resampling_diagnostics=resampling_diagnostics,
            simulation_engine=self.spec.simulation.engine,
            physics_backend=compatibility.physics_backend,
            physics_solver=compatibility.physics_solver,
            newton_config=newton_config,
            check_report=check_report,
            telemetry_quality=check_report.telemetry_quality_report,
            theta_authored_initial=theta_authored_initial,
        )

    def _uses_differentiable_newton(self) -> bool:
        """Check whether this run targets the contact-free differentiable Newton bridge.

        Returns:
            True when the configured engine and solver require the differentiable bridge.
        """
        return (
            self.spec.simulation.engine == SIMULATION_ENGINE_NEWTON
            and self.spec.simulation.newton.solver == NEWTON_SOLVER_FEATHERSTONE_DIFF
        )

    def _create_bridge(  # noqa: ANN202
        self,
        *,
        stage,  # noqa: ANN001
        parameter_space: ParameterSpace,
        link_paths: list[str],
        joint_paths: list[str],
        residual_cfg,  # noqa: ANN001
    ):
        compatibility = resolve_simulation_compatibility(self.spec.simulation)
        if self._bridge_factory is not None:
            return self._bridge_factory(
                stage=stage,
                parameter_space=parameter_space,
                link_paths=link_paths,
                joint_paths=joint_paths,
                simulation_engine=self.spec.simulation.engine,
                offline_stepping=self.spec.simulation.offline_stepping,
                physics_backend=compatibility.physics_backend,
                actuator_runtime=compatibility.actuator_runtime,
                explicit_actuator_policy=self.spec.simulation.explicit_actuator_policy,
                selected_dof_paths=joint_paths,
                newton_config=self.spec.simulation.newton,
                robot_prim_path=self.spec.simulation.robot_prim_path,
                source_env_path=self.spec.simulation.source_env_path,
                env_paths_root=self.spec.simulation.env_paths_root,
                use_parallel_clones=self.spec.simulation.parallel_clones,
                co_locate_clones=(
                    self.spec.simulation.co_locate_clones if self.spec.simulation.parallel_clones else True
                ),
                fabric_clones=(self.spec.simulation.fabric_clones if self.spec.simulation.parallel_clones else False),
                end_effector_link_index=residual_cfg.end_effector_link_index,
            )
        if self._uses_differentiable_newton():
            from .newton_diff_sysid_bridge import NewtonDifferentiableSysIdBridge

            return NewtonDifferentiableSysIdBridge(
                robot_prim_path=self.spec.simulation.robot_prim_path,
                stage_path=self.spec.stage.input_path,
                stage=stage,
                newton_config=self.spec.simulation.newton,
                joint_baselines=parameter_space.baseline.usd_joint_snapshots,
                joint_paths=joint_paths,
                link_paths=link_paths,
                baseline_link_inertia_lc=parameter_space.baseline.link_inertia_lc,
                physics_dt=_physics_dt(stage),
                num_joints=len(joint_paths),
                num_links=max(1, len(link_paths)),
                actuator_runtime=compatibility.actuator_runtime,
            )
        if self.spec.simulation.engine == SIMULATION_ENGINE_NEWTON:
            from .newton_sysid_bridge import NewtonSysIdBridge

            return NewtonSysIdBridge(
                robot_prim_path=self.spec.simulation.robot_prim_path,
                stage_path=self.spec.stage.input_path,
                stage=stage,
                newton_config=self.spec.simulation.newton,
                joint_baselines=parameter_space.baseline.usd_joint_snapshots,
                joint_paths=joint_paths,
                link_paths=link_paths,
                physics_dt=_physics_dt(stage),
                num_joints=len(joint_paths),
                actuator_runtime=compatibility.actuator_runtime,
                explicit_actuator_policy=self.spec.simulation.explicit_actuator_policy,
            )
        try:
            from .isaac_sim_sysid_bridge import IsaacSimSysIdBridge
        except ModuleNotFoundError as exc:
            from .errors import SysIdRuntimeUnavailableError
            from .runtime import kit_runtime_available

            missing = str(exc.name or "")
            runtime_roots = ("carb", "omni", "isaacsim.core", "isaacsim.gui")
            if not any(missing == root or missing.startswith(f"{root}.") for root in runtime_roots):
                raise
            if kit_runtime_available():
                raise
            raise SysIdRuntimeUnavailableError(
                "The Isaac Sim SysID bridge requires a bootstrapped Isaac Sim/Kit runtime."
            ) from exc
        return IsaacSimSysIdBridge(
            robot_prim_path=self.spec.simulation.robot_prim_path,
            source_env_path=self.spec.simulation.source_env_path,
            env_paths_root=self.spec.simulation.env_paths_root,
            use_parallel_clones=self.spec.simulation.parallel_clones,
            co_locate_clones=(self.spec.simulation.co_locate_clones if self.spec.simulation.parallel_clones else True),
            fabric_clones=(self.spec.simulation.fabric_clones if self.spec.simulation.parallel_clones else False),
            offline_stepping=self.spec.simulation.offline_stepping,
            end_effector_link_index=residual_cfg.end_effector_link_index,
            selected_dof_paths=joint_paths,
            physics_backend=compatibility.physics_backend,
            actuator_runtime=compatibility.actuator_runtime,
            explicit_actuator_policy=self.spec.simulation.explicit_actuator_policy,
        )


def _newton_dependency_versions() -> dict[str, str]:
    versions: dict[str, str] = {}
    for distribution in ("newton", "newton-usd-schemas", "mujoco", "mujoco-warp", "warp-lang"):
        try:
            versions[distribution] = metadata.version(distribution)
        except metadata.PackageNotFoundError:
            continue
    return versions


def load_run_trajectory(spec: SysIdRunSpec) -> TrajectoryDataset:  # noqa: D103
    trajectory = load_trajectory(spec.telemetry.to_load_config())
    manifest_path = spec.telemetry.chunk_manifest_path.strip()
    if manifest_path:
        trajectory = attach_companion_training_details(trajectory, manifest_path)
    return trajectory


def build_segmented_chunks(  # noqa: D103
    spec: SysIdRunSpec,
    trajectory: TrajectoryDataset,
) -> tuple[list[TrajectoryChunk], list[TrajectoryChunk]]:
    chunk_specs = list(spec.telemetry.chunks or [])
    if not chunk_specs and spec.telemetry.auto_split:
        chunk_specs = build_auto_chunks(
            trajectory,
            train_fraction=float(spec.telemetry.auto_split_train_fraction),
            min_chunk_seconds=float(spec.telemetry.auto_split_min_chunk_seconds),
            metadata_chunks=phase_chunks_from_metadata(trajectory) or None,
        )
    if not chunk_specs:
        chunk_specs = phase_chunks_from_metadata(trajectory)
    if not chunk_specs:
        window = spec.telemetry.time_window
        if window.end > window.start:
            chunk_specs = [make_default_train_chunk(trajectory, start=window.start, end=window.end)]
        else:
            chunk_specs = [make_default_train_chunk(trajectory)]
    chunks = build_trajectory_chunks(trajectory, chunk_specs, require_train=True, reject_overlaps=True)
    return split_train_validation_chunks(chunks)


def _resample_chunks_for_rollout(
    train_chunks: list[TrajectoryChunk],
    validation_chunks: list[TrajectoryChunk],
    *,
    physics_dt: float,
    sampling_config,  # noqa: ANN001
    max_rollout_steps: int,
) -> tuple[list[TrajectoryChunk], list[TrajectoryChunk], list[ResamplingDiagnostic]]:
    diagnostics: list[ResamplingDiagnostic] = []

    def _resample(chunk: TrajectoryChunk) -> TrajectoryChunk:
        result = resample_trajectory_for_rollout(
            chunk.trajectory,
            physics_dt=physics_dt,
            config=sampling_config,
            max_rollout_steps=max_rollout_steps,
        )
        diagnostics.append(result.diagnostic)
        return replace(
            chunk,
            trajectory=result.trajectory,
            sample_count=int(result.trajectory.times.shape[0]),
            duration_seconds=float(result.trajectory.times[-1] - result.trajectory.times[0]),
        )

    return [_resample(chunk) for chunk in train_chunks], [_resample(chunk) for chunk in validation_chunks], diagnostics


def _physics_dt(stage: object | None = None) -> float:
    """Resolve the rollout physics timestep, warning when it has to be assumed.

    The timestep sets the resampling rate and the actuator delay quantization,
    so an assumed value that disagrees with the scene skews identified delays
    and gains without any other symptom. Warn loudly rather than defaulting
    silently.

    Args:
        stage: Optional stage searched for an authored PhysX scene timestep.

    Returns:
        The resolved timestep in seconds, or the 60 Hz assumption.
    """
    try:
        from .runtime import get_physics_dt

        value = get_physics_dt()
        if value > 0.0:
            return value
    except Exception:
        pass
    if stage is not None:
        try:
            for prim in stage.Traverse():
                attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
                value = float(attr.Get()) if attr and attr.HasAuthoredValueOpinion() else 0.0
                if value > 0.0:
                    return 1.0 / value
        except Exception:
            pass
    carb.log_warn(
        f"SysId: no physics timestep available from the runtime or the stage; assuming {_DEFAULT_PHYSICS_DT:.6g} s "
        "(60 Hz). Author physxScene:timeStepsPerSecond so resampling and actuator delay quantization match the scene."
    )
    return _DEFAULT_PHYSICS_DT


def build_parameter_space(  # noqa: D103
    spec: SysIdRunSpec,
    stage,  # noqa: ANN001
    trajectory: TrajectoryDataset,
) -> tuple[ParameterSpace, list[str], list[str]]:
    flags = spec.parameters.space
    advanced = any(
        (
            flags.include_com_offsets,
            flags.include_inertia_log_cholesky,
            flags.include_joint_limit_scales,
            flags.include_per_link_mass,
            flags.include_command_delay,
        )
    )
    physics_backend = resolve_simulation_compatibility(spec.simulation).physics_backend
    link_paths, joint_paths = collect_robot_link_and_joint_paths(stage, spec.simulation.robot_prim_path)
    joint_paths = order_joint_paths_for_trajectory(joint_paths, trajectory)
    if not advanced:
        space = ParameterSpace.for_robot(trajectory.num_joints)
        space.read_usd_baselines(
            stage, link_paths, joint_paths[: trajectory.num_joints], physics_backend=physics_backend
        )
        return space, link_paths, joint_paths

    space = ParameterSpace.for_robot_extended(
        trajectory.num_joints,
        max(1, len(link_paths)),
        include_basic=flags.include_basic,
        include_com_offsets=flags.include_com_offsets,
        include_inertia_log_cholesky=flags.include_inertia_log_cholesky,
        include_joint_limit_scales=flags.include_joint_limit_scales,
        include_per_link_mass=flags.include_per_link_mass,
        include_command_delay=flags.include_command_delay,
    )
    space.read_usd_baselines(stage, link_paths, joint_paths[: trajectory.num_joints], physics_backend=physics_backend)
    return space, link_paths, joint_paths


def optimizer_vectors(  # noqa: D103
    spec: SysIdRunSpec,
    *,
    allow_empty: bool = False,
) -> tuple[list, torch.Tensor, torch.Tensor, torch.Tensor]:
    selected = spec.parameters.selected
    if not selected:
        if allow_empty:
            empty = torch.empty(0, dtype=torch.float32)
            return [], empty, empty.clone(), empty.clone()
        raise ValueError("Run spec must select at least one parameter.")
    entries = []
    initials = []
    mins = []
    maxs = []
    for item in selected:
        if item.min >= item.max:
            raise ValueError(f"{item.param_type}: min ({item.min}) must be less than max ({item.max}).")
        if not (item.min <= item.initial <= item.max):
            raise ValueError(f"{item.param_type}: initial ({item.initial}) must be within [{item.min}, {item.max}].")
        entries.append(item.to_entry())
        initials.append(float(item.initial))
        mins.append(float(item.min))
        maxs.append(float(item.max))
    return (
        entries,
        torch.tensor(initials, dtype=torch.float32),
        torch.tensor(mins, dtype=torch.float32),
        torch.tensor(maxs, dtype=torch.float32),
    )


def _apply_analytical_presolve(
    stage,  # noqa: ANN001
    parameter_space: ParameterSpace,
    link_paths: list[str],
    joint_paths: list[str],
    trajectory: TrajectoryDataset,
    train_chunks: list[TrajectoryChunk],
    param_entries: list,
    theta_initial: torch.Tensor,
    theta_min: torch.Tensor,
    theta_max: torch.Tensor,
    residual_cfg,  # noqa: ANN001
    resolved_backend: OptimizerBackend,
    *,
    max_rollout_steps: int,
) -> AnalyticalPresolveResult:
    if resolved_backend not in (
        OptimizerBackend.CMA_ES,
        OptimizerBackend.BAYESIAN,
        OptimizerBackend.GRADIENT_DESCENT,
    ):
        result = AnalyticalPresolveResult(
            theta_seed=theta_initial.detach().cpu().clone(),
            identifiable=[False for _ in param_entries],
            rank=0,
            condition=float("inf"),
            column_norms=[],
            warnings=["Analytical presolve applies only to CMA-ES, Bayesian, and gradient-descent optimizer backends."],
        )
        _log_analytical_presolve(result)
        return result

    presolve_trajectory, presolve_accelerations, presolve_steps = _presolve_training_inputs(
        train_chunks, trajectory, max_rollout_steps
    )

    dynamics_context = build_fixed_base_context_from_usd(
        stage,
        link_paths,
        joint_paths[: trajectory.num_joints],
        parameter_space.baseline,
    )
    if dynamics_context is None:
        carb.log_info("SysId analytical presolve: no fixed-base dynamics context; torque telemetry may still be used.")

    result = run_analytical_presolve(
        presolve_trajectory,
        param_entries,
        theta_initial,
        theta_min,
        theta_max,
        parameter_space.baseline,
        residual_config=residual_cfg,
        dynamics_context=dynamics_context,
        max_steps=presolve_steps,
        accelerations=presolve_accelerations,
    )
    _log_analytical_presolve(result)
    return result


def _presolve_training_inputs(
    train_chunks: list[TrajectoryChunk],
    trajectory: TrajectoryDataset,
    max_rollout_steps: int,
) -> tuple[TrajectoryDataset, np.ndarray | None, int]:
    """Stack training chunks into one presolve regression input.

    The presolve is a linear least-squares over per-sample regressor rows, so
    training chunks stack directly; accelerations are finite-differenced per
    chunk BEFORE concatenation (a finite difference across a chunk boundary
    differentiates through the inter-chunk discontinuity). Each chunk
    contributes up to ``max_rollout_steps`` samples. Without chunks the full
    trajectory passes through unchanged (accelerations left to the presolve).

    Args:
        train_chunks: Training chunks to stack for the regression.
        trajectory: Telemetry trajectory used by the operation.
        max_rollout_steps: Maximum samples retained from each chunk.

    Returns:
        Stacked trajectory, optional acceleration matrix, and retained step count.
    """
    if not train_chunks:
        return trajectory, None, max_rollout_steps

    times_parts: list[np.ndarray] = []
    q_parts: list[np.ndarray] = []
    qd_parts: list[np.ndarray] = []
    cmd_parts: list[np.ndarray] = []
    qdd_parts: list[np.ndarray] = []
    torque_parts: list[np.ndarray] = []
    torques_available = True
    for chunk in train_chunks:
        chunk_traj = chunk.trajectory
        steps = max(2, min(int(max_rollout_steps), int(chunk_traj.positions.shape[0])))
        times = np.asarray(chunk_traj.times[:steps], dtype=np.float64)
        qd = np.asarray(chunk_traj.velocities[:steps], dtype=np.float64)
        times_parts.append(times)
        q_parts.append(np.asarray(chunk_traj.positions[:steps], dtype=np.float64))
        qd_parts.append(qd)
        cmd_parts.append(np.asarray(chunk_traj.commands[:steps], dtype=np.float64))
        qdd_parts.append(finite_difference_acceleration(times, qd))
        torques = getattr(chunk_traj, "torques", None)
        if torques is None:
            torques_available = False
        else:
            torque_parts.append(np.asarray(torques[:steps], dtype=np.float64))

    stitched = TrajectoryDataset(
        times=np.concatenate(times_parts),
        positions=np.vstack(q_parts),
        velocities=np.vstack(qd_parts),
        commands=np.vstack(cmd_parts),
        metadata=getattr(trajectory, "metadata", None),
        torques=np.vstack(torque_parts) if torques_available and torque_parts else None,
    )
    accelerations = np.vstack(qdd_parts)
    return stitched, accelerations, int(stitched.positions.shape[0])


def _log_analytical_presolve(result: AnalyticalPresolveResult) -> None:
    carb.log_info(
        "SysId analytical presolve: "
        f"seeded={result.seeded_count}, rank={result.rank}, condition={result.condition:.6g}"
    )
    for warning in result.warnings:
        carb.log_warn(f"SysId analytical presolve: {warning}")


def _apply_multichunk_jacobian_policy(backend_cfg: OptimizerBackendConfig, *, chunk_count: int) -> None:
    """Disable the single-trajectory analytical Jacobian explicitly for multi-chunk training.

    Args:
        backend_cfg: Optimizer backend settings to update.
        chunk_count: Number of training chunks in the prepared run.
    """
    if chunk_count > 1 and backend_cfg.use_analytical_jacobian:
        carb.log_warn("SysId: analytical_jacobian disabled for multi-chunk training; falling back to numerical LM.")
        backend_cfg.use_analytical_jacobian = False


def write_optimized_parameters_to_usd(  # noqa: D103
    stage,  # noqa: ANN001
    space: ParameterSpace,
    config: OptimizerConfig,
    theta: list[float],
    num_links: int,
    bridge=None,  # noqa: ANN001
    *,
    physics_backend: str = "",
    neutral_write_layer=None,  # noqa: ANN001
    solver_write_layer=None,  # noqa: ANN001
) -> bool:
    refresh = getattr(bridge, "refresh_parameter_space_baselines", None)
    if callable(refresh):
        link_paths, _dof_paths = refresh(space, stage)
        num_links = max(num_links, len(link_paths))
    theta_row = torch.tensor(theta, dtype=torch.float32)
    usd_written = False
    apply_state = None
    claimed_types = set(getattr(bridge, "usd_parameter_types", set()) or set())
    claimed_dofs = dict(getattr(bridge, "usd_parameter_dof_claims", {}) or {})
    parameter_space_write_needed = _has_parameter_space_usd_writeback(
        config.param_entries,
        excluded_types=claimed_types,
    )
    have_snapshots = bool(space.baseline.usd_link_snapshots or space.baseline.usd_joint_snapshots)
    if parameter_space_write_needed and have_snapshots:
        apply_state = space.decode_theta_to_apply_state(
            theta_row,
            config.param_entries,
            config.trajectory.num_joints,
            num_links=max(1, num_links),
        )
        write_groups = [(config.param_entries, None)]
        if neutral_write_layer is not None or solver_write_layer is not None:
            neutral_entries = [
                entry
                for entry in config.param_entries
                if getattr(entry, "param_type", None) not in _SOLVER_SPECIFIC_USD_PARAMETER_TYPES
            ]
            solver_entries = [
                entry
                for entry in config.param_entries
                if getattr(entry, "param_type", None) in _SOLVER_SPECIFIC_USD_PARAMETER_TYPES
            ]
            neutral_target = neutral_write_layer if neutral_write_layer is not None else solver_write_layer
            solver_target = solver_write_layer if solver_write_layer is not None else neutral_write_layer
            write_groups = [
                (neutral_entries, neutral_target),
                (solver_entries, solver_target),
            ]
        for entries, target_layer in write_groups:
            if not _has_parameter_space_usd_writeback(entries, excluded_types=claimed_types):
                continue
            edit_context = nullcontext()
            if target_layer is not None:
                from pxr import Usd

                edit_context = Usd.EditContext(stage, target_layer)
            with edit_context:
                space.write_usd_from_baselines(
                    stage,
                    apply_state,
                    physics_backend=physics_backend,
                    skip_parameter_types=claimed_types,
                    skip_joint_parameter_dofs=claimed_dofs,
                    param_entries=entries,
                )
            usd_written = True
    bridge_write = getattr(bridge, "write_parameters_to_usd", None)
    if callable(bridge_write):
        bridge_groups = [(config.param_entries, theta_row, None)]
        bridge_claimed_types = claimed_types | set(claimed_dofs)
        if (neutral_write_layer is not None or solver_write_layer is not None) and bridge_claimed_types:
            neutral_indices = [
                index
                for index, entry in enumerate(config.param_entries)
                if getattr(entry, "param_type", None) in bridge_claimed_types
                and getattr(entry, "param_type", None) not in _SOLVER_SPECIFIC_USD_PARAMETER_TYPES
            ]
            solver_indices = [
                index
                for index, entry in enumerate(config.param_entries)
                if getattr(entry, "param_type", None) in bridge_claimed_types
                and getattr(entry, "param_type", None) in _SOLVER_SPECIFIC_USD_PARAMETER_TYPES
            ]
            neutral_target = neutral_write_layer if neutral_write_layer is not None else solver_write_layer
            solver_target = solver_write_layer if solver_write_layer is not None else neutral_write_layer
            bridge_groups = [
                (
                    [config.param_entries[index] for index in neutral_indices],
                    theta_row[neutral_indices],
                    neutral_target,
                ),
                ([config.param_entries[index] for index in solver_indices], theta_row[solver_indices], solver_target),
            ]
        bridge_written = True
        for entries, values, target_layer in bridge_groups:
            if not entries:
                continue
            edit_context = nullcontext()
            if target_layer is not None:
                from pxr import Usd

                edit_context = Usd.EditContext(stage, target_layer)
            with edit_context:
                bridge_written = bool(bridge_write(stage, entries, values)) and bridge_written
        if apply_state is not None:
            _verify_effective_joint_gain_writeback(
                stage,
                space,
                apply_state,
                config.param_entries,
                skipped_types=claimed_types,
                skipped_joint_dofs=claimed_dofs,
            )
        # A callable bridge can legitimately have nothing to persist for these
        # entries. Do not let that turn a missing parameter-space baseline into
        # a false successful write.
        parameter_space_satisfied = usd_written or not parameter_space_write_needed
        return parameter_space_satisfied and bridge_written
    if apply_state is not None:
        _verify_effective_joint_gain_writeback(
            stage,
            space,
            apply_state,
            config.param_entries,
            skipped_types=claimed_types,
            skipped_joint_dofs=claimed_dofs,
        )
    return usd_written


def _verify_effective_joint_gain_writeback(
    stage: object,
    space: ParameterSpace,
    apply_state: ParameterApplyState,
    param_entries: list,
    *,
    skipped_types: set[SysIdParameterType],
    skipped_joint_dofs: dict[SysIdParameterType, set[int]],
) -> None:
    """Verify that authored joint gains win USD composition.

    Args:
        stage: Composed stage containing the selected robot.
        space: Parameter space containing the writeback baselines.
        apply_state: Decoded accepted parameter values.
        param_entries: Selected parameter definitions.
        skipped_types: Gain types owned by a runtime-specific writer.
        skipped_joint_dofs: Gain DOFs owned by a runtime-specific writer.

    Raises:
        RuntimeError: If an authored gain is masked or cannot be composed.
    """
    from .usd_parameter_io import _drive_attr_to_effective

    gain_types = (SysIdParameterType.JOINT_STIFFNESS, SysIdParameterType.JOINT_DAMPING)
    selected_entries = {
        param_type: [entry for entry in param_entries if entry.param_type == param_type]
        for param_type in gain_types
        if param_type not in skipped_types
    }
    mismatches: list[str] = []
    for snapshot_index, baseline in enumerate(space.baseline.usd_joint_snapshots):
        if baseline is None:
            continue
        dof_index = baseline.dof_index if baseline.dof_index >= 0 else snapshot_index
        prim = stage.GetPrimAtPath(baseline.path)
        if prim is None or not prim.IsValid():
            mismatches.append(f"{baseline.path} is no longer present in the composed stage")
            continue
        drive_type = "angular" if baseline.is_angular else "linear"
        for param_type, entries in selected_entries.items():
            if dof_index in skipped_joint_dofs.get(param_type, set()):
                continue
            if not any(int(entry.dof_index) < 0 or int(entry.dof_index) == dof_index for entry in entries):
                continue
            if param_type == SysIdParameterType.JOINT_STIFFNESS:
                expected = float(baseline.stiffness) * float(apply_state.stiffness_scale[dof_index])
                attr = prim.GetAttribute(f"drive:{drive_type}:physics:stiffness")
                label = "stiffness"
            else:
                expected = float(baseline.damping) * float(apply_state.damping_scale[dof_index])
                attr = prim.GetAttribute(f"drive:{drive_type}:physics:damping")
                label = "damping"
            value = attr.Get() if attr is not None and attr.IsValid() else None
            actual = float("nan") if value is None else _drive_attr_to_effective(float(value), drive_type)
            tolerance = max(1e-6, abs(expected) * 1e-5)
            if np.isfinite(actual) and abs(actual - expected) <= tolerance:
                continue
            instance_note = " The joint is an instance proxy and requires a derived asset or de-instancing."
            is_instance_proxy = callable(getattr(prim, "IsInstanceProxy", None)) and prim.IsInstanceProxy()
            mismatches.append(
                f"{baseline.path} {label} expected {expected:.9g}, composed {actual:.9g}."
                + (instance_note if is_instance_proxy else " A stronger USD opinion may be masking the write.")
            )
    if mismatches:
        raise RuntimeError("USD joint-gain writeback was authored but is not effective: " + "; ".join(mismatches))


def _has_parameter_space_usd_writeback(
    param_entries: list,
    *,
    excluded_types: set[SysIdParameterType] | None = None,
) -> bool:
    writable = {
        SysIdParameterType.JOINT_FRICTION,
        SysIdParameterType.JOINT_STIFFNESS,
        SysIdParameterType.JOINT_DAMPING,
        SysIdParameterType.JOINT_ARMATURE,
        SysIdParameterType.LINK_MASS,
        SysIdParameterType.LINK_COM_OFFSET_X,
        SysIdParameterType.LINK_COM_OFFSET_Y,
        SysIdParameterType.LINK_COM_OFFSET_Z,
        SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
        SysIdParameterType.JOINT_LIMIT_UPPER_SCALE,
    }
    excluded = excluded_types or set()
    return any(
        getattr(entry, "param_type", None) in writable and getattr(entry, "param_type", None) not in excluded
        for entry in param_entries
    )
