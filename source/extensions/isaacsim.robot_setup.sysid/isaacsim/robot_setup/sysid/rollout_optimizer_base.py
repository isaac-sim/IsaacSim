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

"""Shared rollout evaluation for SysId optimizers."""

from __future__ import annotations

import asyncio
import hashlib
import math
import os
import tempfile
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import carb
import numpy as np
import torch

from .env_bridge import (
    NullSysIdEnvironmentBridge,
    SysIdEnvironmentBridge,
    SysIdEnvironmentBridgeError,
)
from .optimizer_base import OptimizerConfig
from .residual_config import ResidualWeightConfig, default_residual_weight_config
from .residual_engine import RolloutResidualInputs, WeightedResidualEngine
from .rollout_plot_data import SysIdRolloutPlotData
from .rollout_result import SysIdRolloutResult
from .runtime import yield_control
from .trajectory_csv import TrajectoryDataset
from .trajectory_resampling import combine_sample_weights
from .trajectory_segments import ChunkValidationMetric, TrajectoryChunk


@dataclass
class _TrajectoryTensorCache:
    key: tuple[object, ...]
    tensors: dict[str, torch.Tensor | None]


@dataclass
class _SampleWeightTensorCache:
    key: tuple[object, ...]
    tensor: torch.Tensor


@dataclass(frozen=True)
class _TrainingSegment:
    trajectory: TrajectoryDataset
    weight: float
    label: str


@dataclass
class ChunkValidationReplay:
    """Validation metric plus the rollout data needed for headless artifacts."""

    metric: ChunkValidationMetric
    rollout: SysIdRolloutPlotData | None = None


def _array_content_signature(
    values: np.ndarray | None,
    *,
    num_steps: int | None = None,
) -> tuple[object, ...] | None:
    """Return a stable signature that changes when an array is mutated in place.

    Args:
        values: Optional array whose shape, dtype, and contents identify cached data.
        num_steps: Optional leading-axis limit for rollout-relevant data.

    Returns:
        Shape, dtype, and content digest, or ``None`` for an absent channel.
    """
    if values is None:
        return None
    array = np.asarray(values)
    consumed = array if num_steps is None or array.ndim == 0 else array[: max(0, int(num_steps))]
    contiguous = np.ascontiguousarray(consumed)
    digest = hashlib.blake2b(contiguous.tobytes(order="C"), digest_size=8).digest()
    return (tuple(int(size) for size in consumed.shape), consumed.dtype.str, digest)


def _trajectory_content_signature(
    trajectory: TrajectoryDataset,
    *,
    num_steps: int | None = None,
) -> tuple[object, ...]:
    """Return the rollout-relevant content signature for a trajectory.

    Args:
        trajectory: Trajectory whose required and optional channels are fingerprinted.
        num_steps: Optional leading-axis limit for rollout-relevant data.

    Returns:
        Content signatures for every trajectory channel consumed by rollouts.
    """
    return tuple(
        _array_content_signature(getattr(trajectory, name, None), num_steps=num_steps)
        for name in (
            "times",
            "positions",
            "velocities",
            "commands",
            "torques",
            "end_effector_poses",
            "contact_forces",
            "residual_sample_weights",
        )
    )


def tensor_to_joint_series(tensor: torch.Tensor) -> list[list[float]]:
    """Convert a (time, joint) tensor into one time series per joint.

    Args:
        tensor: Time-major joint signal tensor.

    Returns:
        One Python time series per joint.
    """
    arr = tensor.detach().cpu().numpy()
    if arr.ndim == 1:
        return [arr.tolist()]
    if arr.ndim != 2:
        raise ValueError(f"Expected 1D or 2D tensor for plot data, got shape {arr.shape}")
    return [arr[:, joint_index].tolist() for joint_index in range(arr.shape[1])]


def _validate_rollout_shapes(
    rollout: SysIdRolloutResult,
    *,
    expected_envs: int,
    expected_steps: int,
    expected_trailing_shapes: dict[str, tuple[int, ...] | None] | None = None,
) -> None:
    """Require every returned signal to align with candidates and telemetry.

    Args:
        rollout: Batched simulator signals returned by the environment bridge.
        expected_envs: Number of evaluated parameter candidates.
        expected_steps: Number of requested rollout steps.
        expected_trailing_shapes: Optional required per-step dimensions by signal.
    """
    trailing_shapes = expected_trailing_shapes or {}
    for signal_name, signal in (
        ("positions", rollout.positions),
        ("velocities", rollout.velocities),
        ("torques", rollout.torques),
        ("end_effector_poses", rollout.end_effector_poses),
        ("contact_forces", rollout.contact_forces),
    ):
        if signal is None:
            continue
        if signal.ndim < 3 or int(signal.shape[0]) != expected_envs or int(signal.shape[1]) != expected_steps:
            raise SysIdEnvironmentBridgeError(
                f"Rollout {signal_name} must have leading shape "
                f"({expected_envs}, {expected_steps}), got {tuple(signal.shape)}."
            )
        expected_trailing = trailing_shapes.get(signal_name)
        if expected_trailing is not None and tuple(signal.shape[2:]) != tuple(expected_trailing):
            expected_shape = (expected_envs, expected_steps, *expected_trailing)
            raise SysIdEnvironmentBridgeError(
                f"Rollout {signal_name} must have shape {expected_shape}, got {tuple(signal.shape)}."
            )


def _expected_rollout_trailing_shapes(
    *,
    positions: torch.Tensor,
    velocities: torch.Tensor,
    torques: torch.Tensor | None,
    end_effector_poses: torch.Tensor | None,
    contact_forces: torch.Tensor | None,
) -> dict[str, tuple[int, ...] | None]:
    """Return expected per-step signal dimensions from measured telemetry.

    Args:
        positions: Measured joint positions.
        velocities: Measured joint velocities.
        torques: Optional measured joint torques.
        end_effector_poses: Optional measured end-effector poses.
        contact_forces: Optional measured contact forces.

    Returns:
        Expected trailing dimensions keyed by rollout signal name.
    """

    def trailing(value: torch.Tensor | None) -> tuple[int, ...] | None:
        return None if value is None else tuple(int(size) for size in value.shape[1:])

    return {
        "positions": trailing(positions),
        "velocities": trailing(velocities),
        "torques": trailing(torques),
        "end_effector_poses": trailing(end_effector_poses),
        "contact_forces": trailing(contact_forces),
    }


def _stack_optional_rollout_rows(
    rows: list[torch.Tensor],
    *,
    signal_name: str,
    expected_rows: int,
) -> torch.Tensor | None:
    """Stack an optional sequential signal only when every candidate returned it.

    Args:
        rows: Per-candidate signal tensors returned by sequential rollouts.
        signal_name: Signal name used in validation errors.
        expected_rows: Number of evaluated candidates.

    Returns:
        Batched signal tensor, or ``None`` when every rollout omitted the signal.
    """
    if not rows:
        return None
    if len(rows) != expected_rows:
        raise SysIdEnvironmentBridgeError(
            f"Sequential rollout returned {signal_name} for {len(rows)} of {expected_rows} candidates."
        )
    return torch.stack(rows, dim=0)


def make_rollout_plot_data(  # noqa: D103
    times: torch.Tensor,
    meas_pos: torch.Tensor,
    meas_vel: torch.Tensor,
    sim_pos: torch.Tensor,
    sim_vel: torch.Tensor,
    env_index: int = 0,
    *,
    meas_torque: torch.Tensor | None = None,
    sim_torque: torch.Tensor | None = None,
) -> SysIdRolloutPlotData:
    mp = meas_pos[env_index] if meas_pos.ndim == 3 else meas_pos
    mv = meas_vel[env_index] if meas_vel.ndim == 3 else meas_vel
    sp = sim_pos[env_index] if sim_pos.ndim == 3 else sim_pos
    sv = sim_vel[env_index] if sim_vel.ndim == 3 else sim_vel
    num_steps = int(mp.shape[0])
    t_list = times.detach().cpu().reshape(-1).tolist()[:num_steps]
    mt = meas_torque[env_index] if meas_torque is not None and meas_torque.ndim == 3 else meas_torque
    st = sim_torque[env_index] if sim_torque is not None and sim_torque.ndim == 3 else sim_torque
    return SysIdRolloutPlotData(
        times=t_list,
        measured_positions=tensor_to_joint_series(mp),
        simulated_positions=tensor_to_joint_series(sp),
        measured_velocities=tensor_to_joint_series(mv),
        simulated_velocities=tensor_to_joint_series(sv),
        measured_efforts=tensor_to_joint_series(mt[:num_steps]) if mt is not None else None,
        simulated_efforts=tensor_to_joint_series(st[:num_steps]) if st is not None else None,
    )


def _clone_joint_angles_debug_png_path(trajectory: TrajectoryDataset) -> Path:
    metadata = getattr(trajectory, "metadata", None)
    source_path = getattr(metadata, "source_path", "") if metadata is not None else ""
    if source_path:
        source = Path(str(source_path)).expanduser()
        directory = source if source.is_dir() else source.parent
        stem = source.stem if source.suffix else "trajectory"
    else:
        directory = Path(tempfile.gettempdir())
        stem = "trajectory"
    if not str(directory):
        directory = Path(tempfile.gettempdir())
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stem}_sysid_clone_joint_angles_latest.png"


def _clone_joint_efforts_debug_png_path(trajectory: TrajectoryDataset) -> Path:
    angle_path = _clone_joint_angles_debug_png_path(trajectory)
    return angle_path.with_name(angle_path.name.replace("joint_angles", "joint_efforts"))


def save_clone_joint_angles_debug_png(
    trajectory: TrajectoryDataset,
    times: torch.Tensor,
    sim_pos: torch.Tensor,
) -> None:
    """Write a temp PNG of clone joint angles for tensor sanity checks.

    Args:
        trajectory: Measured trajectory used for rollout comparison.
        times: Telemetry sample timestamps.
        sim_pos: Simulated joint-position batch.
    """
    if sim_pos.ndim != 3 or sim_pos.shape[0] <= 1:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        arr = sim_pos.detach().cpu().numpy()
        num_envs, num_steps, num_joints = arr.shape
        source_times = np.asarray(getattr(trajectory, "times", []), dtype=np.float64).reshape(-1)
        num_steps = min(num_steps, int(source_times.shape[0]))
        if num_steps < 1 or num_joints < 1:
            return
        t = source_times[:num_steps] - source_times[0]
        x_label = "time (s)"
        if not np.all(np.isfinite(t)) or (num_steps > 1 and float(np.ptp(t)) <= 0.0):
            t = np.arange(num_steps, dtype=np.float64)
            x_label = "sample index"
        sample_indices = np.arange(0, num_steps, 2, dtype=np.int64)
        env_indices = list(range(num_envs))
        output_path = _clone_joint_angles_debug_png_path(trajectory)
        fig_height = max(3.0, min(18.0, 1.8 * num_joints))
        fig, axes = plt.subplots(num_joints, 1, sharex=True, figsize=(11.0, fig_height), squeeze=False)
        axes_flat = axes.reshape(-1)
        for joint_i in range(num_joints):
            ax = axes_flat[joint_i]
            for env_i in env_indices:
                ax.plot(
                    t[sample_indices],
                    arr[env_i, sample_indices, joint_i],
                    linewidth=0.8,
                    label=f"env_{env_i}",
                )
            ax.set_ylabel(f"q{joint_i}")
            ax.grid(True, alpha=0.25)
        axes_flat[-1].set_xlabel(x_label)
        axes_flat[0].set_title("SysId clone joint angles (all clones, every other sample)")
        axes_flat[0].legend(loc="upper right", fontsize=7, ncol=min(5, max(1, len(env_indices))))
        fig.tight_layout()
        fig.savefig(output_path, dpi=140)
        plt.close(fig)
        carb.log_info(
            f"SysId: wrote clone joint angle debug plot -> {output_path} "
            f"(x={x_label}, span={float(np.ptp(t)):.6g}, samples={len(sample_indices)})"
        )
    except Exception as exc:
        carb.log_warn(f"SysId: failed to write clone joint angle debug plot: {exc}")


def save_clone_joint_efforts_debug_png(
    trajectory: TrajectoryDataset,
    times: torch.Tensor,
    sim_torque: torch.Tensor | None,
) -> None:
    """Write a temp PNG of clone joint efforts for tensor sanity checks.

    Args:
        trajectory: Measured trajectory used for rollout comparison.
        times: Telemetry sample timestamps.
        sim_torque: Optional simulated joint-torque batch.
    """
    if sim_torque is None or sim_torque.ndim != 3 or sim_torque.shape[0] <= 1:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        arr = sim_torque.detach().cpu().numpy()
        num_envs, num_steps, num_joints = arr.shape
        source_times = np.asarray(getattr(trajectory, "times", []), dtype=np.float64).reshape(-1)
        num_steps = min(num_steps, int(source_times.shape[0]))
        if num_steps < 1 or num_joints < 1:
            return
        t = source_times[:num_steps] - source_times[0]
        x_label = "time (s)"
        if not np.all(np.isfinite(t)) or (num_steps > 1 and float(np.ptp(t)) <= 0.0):
            t = np.arange(num_steps, dtype=np.float64)
            x_label = "sample index"
        sample_indices = np.arange(0, num_steps, 2, dtype=np.int64)
        env_indices = list(range(num_envs))
        output_path = _clone_joint_efforts_debug_png_path(trajectory)
        fig_height = max(3.0, min(18.0, 1.8 * num_joints))
        fig, axes = plt.subplots(num_joints, 1, sharex=True, figsize=(11.0, fig_height), squeeze=False)
        axes_flat = axes.reshape(-1)
        measured = np.asarray(trajectory.torques, dtype=np.float64) if trajectory.torques is not None else None
        if measured is not None and (
            measured.ndim != 2 or measured.shape[0] < num_steps or measured.shape[1] < num_joints
        ):
            measured = None
        for joint_i in range(num_joints):
            ax = axes_flat[joint_i]
            for env_i in env_indices:
                ax.plot(
                    t[sample_indices],
                    arr[env_i, sample_indices, joint_i],
                    linewidth=0.8,
                    label=f"env_{env_i}",
                )
            if measured is not None:
                ax.plot(
                    t[sample_indices],
                    measured[sample_indices, joint_i],
                    linewidth=1.2,
                    linestyle="--",
                    color="black",
                    label="recorded" if joint_i == 0 else None,
                )
            ax.set_ylabel(f"tau{joint_i}")
            ax.grid(True, alpha=0.25)
        axes_flat[-1].set_xlabel(x_label)
        axes_flat[0].set_title("SysId clone joint efforts (all clones, every other sample)")
        axes_flat[0].legend(loc="upper right", fontsize=7, ncol=min(5, max(1, len(env_indices) + 1)))
        fig.tight_layout()
        fig.savefig(output_path, dpi=140)
        plt.close(fig)
        carb.log_info(
            f"SysId: wrote clone joint effort debug plot -> {output_path} "
            f"(x={x_label}, span={float(np.ptp(t)):.6g}, samples={len(sample_indices)}, "
            f"recorded={'yes' if measured is not None else 'no'})"
        )
    except Exception as exc:
        carb.log_warn(f"SysId: failed to write clone joint effort debug plot: {exc}")


class RolloutOptimizerBase:
    """Bridge wiring and batched rollout cost evaluation shared by optimizer backends.

    Args:
        residual_engine: Optional residual engine; defaults to the standard weighted engine.
    """

    def __init__(self, residual_engine: Optional[WeightedResidualEngine] = None) -> None:
        self._bridge: SysIdEnvironmentBridge = NullSysIdEnvironmentBridge()
        self._cancel_requested = False
        self._residual_config = default_residual_weight_config()
        self._residual_engine = residual_engine or self._residual_config.build_engine()
        self._bridge_setup_key: tuple[object, ...] | None = None
        self._trajectory_tensor_cache: _TrajectoryTensorCache | None = None
        self._sample_weight_tensor_cache: _SampleWeightTensorCache | None = None
        self._clone_debug_plot_count = 0
        self._debug_rollout_plots = os.getenv("ISAACSIM_SYSID_DEBUG_ROLLOUT_PLOTS", "").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

    def set_debug_rollout_plots_enabled(self, enabled: bool) -> None:
        """Enable expensive clone diagnostic PNGs for troubleshooting only.

        Args:
            enabled: Whether debug rollout plots are enabled.
        """
        self._debug_rollout_plots = bool(enabled)

    def set_residual_config(self, config: ResidualWeightConfig) -> None:  # noqa: D102
        self._residual_config = config
        self._residual_engine = config.build_engine()
        self._sample_weight_tensor_cache = None

    def _engine_for_residual_config(self, config: ResidualWeightConfig) -> WeightedResidualEngine:
        """Return an engine for one evaluation without mutating optimizer defaults.

        Args:
            config: Residual configuration for the current rollout evaluation.

        Returns:
            Existing default engine or a temporary engine built from ``config``.
        """
        if config is self._residual_config:
            return self._residual_engine
        return config.build_engine()

    def set_bridge(self, bridge: SysIdEnvironmentBridge) -> None:  # noqa: D102
        if bridge is self._bridge:
            return
        self.cleanup_bridge()
        self._bridge = bridge
        self._bridge_setup_key = None
        self._trajectory_tensor_cache = None
        self._sample_weight_tensor_cache = None

    def get_bridge(self) -> SysIdEnvironmentBridge:  # noqa: D102
        return self._bridge

    def cleanup_bridge(self, remove_clones: bool = True) -> None:  # noqa: D102
        if hasattr(self._bridge, "cleanup"):
            try:
                self._bridge.cleanup(remove_clones=remove_clones)
            finally:
                self._bridge = NullSysIdEnvironmentBridge()
                self._bridge_setup_key = None
                self._trajectory_tensor_cache = None
                self._sample_weight_tensor_cache = None

    async def cleanup_bridge_async(self, remove_clones: bool = True) -> None:  # noqa: D102
        bridge = self._bridge
        cleanup_async = getattr(bridge, "cleanup_async", None)
        try:
            if callable(cleanup_async):
                await cleanup_async(remove_clones=remove_clones)
            elif hasattr(bridge, "cleanup"):
                bridge.cleanup(remove_clones=remove_clones)
        finally:
            if self._bridge is bridge:
                self._bridge = NullSysIdEnvironmentBridge()
            self._bridge_setup_key = None
            self._trajectory_tensor_cache = None
            self._sample_weight_tensor_cache = None

    def request_cancel(self) -> None:  # noqa: D102
        self._cancel_requested = True
        stop = getattr(self._bridge, "request_stop", None)
        if callable(stop):
            stop()

    def reset_cancel(self) -> None:  # noqa: D102
        self._cancel_requested = False

    def _raise_if_cancelled(self) -> None:
        """Propagate a graceful stop to the run-session state machine."""
        if self._cancel_requested:
            raise asyncio.CancelledError

    @staticmethod
    def _progress_for_phase(
        on_progress: Optional[Callable[[str, float], None]],
        phase_start: float,
        phase_end: float,
    ) -> Optional[Callable[[str, float], None]]:
        """Map nested rollout progress into a parent optimizer phase.

        Args:
            on_progress: Optional progress callback.
            phase_start: Progress fraction at the start of the phase.
            phase_end: Progress fraction at the end of the phase.

        Returns:
            Callback that maps local progress into the parent phase, or ``None``.
        """
        if on_progress is None:
            return None
        start = max(0.0, min(1.0, float(phase_start)))
        end = max(start, min(1.0, float(phase_end)))

        def report(message: str, fraction: float) -> None:
            local = max(0.0, min(1.0, float(fraction)))
            on_progress(message, start + (end - start) * local)

        return report

    @staticmethod
    def _optional_trajectory_tensor(
        array: np.ndarray | None,
        device: torch.device,
        num_steps: int,
    ) -> torch.Tensor | None:
        if array is None:
            return None
        return torch.tensor(np.asarray(array)[:num_steps], device=device, dtype=torch.float32)

    @staticmethod
    def trajectory_tensors(  # noqa: D102
        trajectory: TrajectoryDataset,
        device: torch.device,
        num_steps: int | None = None,
    ) -> dict[str, torch.Tensor | None]:
        # ROS bag/MCAP timestamps can be epoch seconds (~1e9). Converting those
        # directly to float32 loses sub-second spacing, so plot/debug payloads
        # carry times relative to the sliced trajectory start.
        step_count = len(trajectory.times) if num_steps is None else max(0, int(num_steps))
        relative_times = np.asarray(trajectory.times, dtype=np.float64).reshape(-1)[:step_count]
        if relative_times.shape[0] > 0 and np.all(np.isfinite(relative_times)):
            relative_times = relative_times - float(relative_times[0])
        times = torch.tensor(relative_times, device=device, dtype=torch.float32)
        positions = torch.tensor(np.asarray(trajectory.positions)[:step_count], device=device, dtype=torch.float32)
        velocities = torch.tensor(np.asarray(trajectory.velocities)[:step_count], device=device, dtype=torch.float32)
        commands = torch.tensor(np.asarray(trajectory.commands)[:step_count], device=device, dtype=torch.float32)
        return {
            "times": times,
            "positions": positions,
            "velocities": velocities,
            "commands": commands,
            "torques": RolloutOptimizerBase._optional_trajectory_tensor(trajectory.torques, device, step_count),
            "end_effector_poses": RolloutOptimizerBase._optional_trajectory_tensor(
                trajectory.end_effector_poses, device, step_count
            ),
            "contact_forces": RolloutOptimizerBase._optional_trajectory_tensor(
                trajectory.contact_forces, device, step_count
            ),
        }

    def _cached_trajectory_tensors(
        self,
        trajectory: TrajectoryDataset,
        device: torch.device,
        num_steps: int,
    ) -> dict[str, torch.Tensor | None]:
        key = (
            _trajectory_content_signature(trajectory, num_steps=num_steps),
            str(device),
            int(num_steps),
        )
        if self._trajectory_tensor_cache is not None and self._trajectory_tensor_cache.key == key:
            return self._trajectory_tensor_cache.tensors
        tensors = self.trajectory_tensors(trajectory, device, num_steps=num_steps)
        self._trajectory_tensor_cache = _TrajectoryTensorCache(key=key, tensors=tensors)
        return tensors

    def _cached_sample_weights(
        self,
        residual_cfg: ResidualWeightConfig,
        trajectory: TrajectoryDataset,
        device: torch.device,
        num_steps: int,
    ) -> torch.Tensor | None:
        trajectory_weights = getattr(trajectory, "residual_sample_weights", None)
        key = (
            id(residual_cfg),
            _array_content_signature(residual_cfg.sample_weights, num_steps=num_steps),
            _array_content_signature(trajectory_weights, num_steps=num_steps),
            str(device),
            int(num_steps),
        )
        if self._sample_weight_tensor_cache is not None and self._sample_weight_tensor_cache.key == key:
            return self._sample_weight_tensor_cache.tensor
        combined = combine_sample_weights(
            residual_cfg.sample_weights,
            trajectory_weights,
            num_steps=num_steps,
        )
        if combined is None:
            self._sample_weight_tensor_cache = None
            return None
        tensor = torch.tensor(
            combined[:num_steps],
            device=device,
            dtype=torch.float32,
        )
        self._sample_weight_tensor_cache = _SampleWeightTensorCache(key=key, tensor=tensor)
        return tensor

    def _ensure_bridge_setup(self, bridge: SysIdEnvironmentBridge, config: OptimizerConfig) -> None:
        # `set_trajectory` receives the untruncated trajectory and derives the
        # physics substep rate from the median dt over every sample, so the key
        # must cover the whole trajectory. Keying on the first
        # `max_rollout_steps` samples lets two chunks that share a prefix but
        # differ in their tails collide and replay at the wrong substep rate.
        setup_key = (
            id(bridge),
            _trajectory_content_signature(config.trajectory),
            tuple(config.param_entries),
        )
        if self._bridge_setup_key == setup_key:
            return
        bridge.set_trajectory(config.trajectory)
        validate = getattr(bridge, "validate_parameter_entries", None)
        if callable(validate):
            validate(config.param_entries)
        self._bridge_setup_key = setup_key

    async def compute_costs_for_theta_batch(
        self,
        config: OptimizerConfig,
        theta_batch: torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
        include_plot_data: bool = True,
    ) -> tuple[torch.Tensor, torch.Tensor, Optional[SysIdRolloutPlotData]]:
        """Run rollouts and return per-env scalar costs plus nominal plot data.

        Args:
            config: Optimizer and trajectory configuration.
            theta_batch: Candidate parameter rows to evaluate.
            on_progress: Optional progress callback.
            include_plot_data: Whether to construct nominal rollout plot data.

        Returns:
            Per-candidate costs, residual rows, and optional nominal rollout plot data.
        """
        training_segments = self._training_segments(config)
        if config.training_segments or config.training_chunks:
            return await self._compute_costs_for_segmented_training_batch(
                config,
                theta_batch,
                training_segments,
                on_progress=on_progress,
                include_plot_data=include_plot_data,
            )
        (
            residuals,
            cost_per_env,
            plot_data,
        ) = await self._compute_residuals_for_theta_batch(
            config,
            theta_batch,
            on_progress=on_progress,
            include_plot_data=include_plot_data,
        )
        return cost_per_env, residuals, plot_data

    @staticmethod
    def _training_segments(config: OptimizerConfig) -> list[_TrainingSegment]:
        segments = list(config.training_segments or [])
        if segments:
            total = len(segments)
            return [
                _TrainingSegment(
                    trajectory=chunk.trajectory,
                    weight=float(chunk.spec.weight),
                    label=_segment_label(chunk, idx, total),
                )
                for idx, chunk in enumerate(segments)
            ]

        chunks = list(config.training_chunks or [])
        if chunks:
            labels = list(config.training_chunk_labels or [])
            raw_weights = list(config.training_chunk_weights or [])
            result: list[_TrainingSegment] = []
            for idx, trajectory in enumerate(chunks):
                label = labels[idx] if idx < len(labels) and labels[idx] else f"chunk {idx + 1}/{len(chunks)}"
                weight = float(raw_weights[idx]) if idx < len(raw_weights) else 1.0
                result.append(_TrainingSegment(trajectory=trajectory, weight=weight, label=label))
            return result

        return [_TrainingSegment(trajectory=config.trajectory, weight=1.0, label="train")]

    @staticmethod
    def _normalized_training_weights(segments: list[_TrainingSegment]) -> list[float]:
        weights = [float(segment.weight) for segment in segments]
        for segment, weight in zip(segments, weights):
            if not math.isfinite(weight) or weight <= 0.0:
                raise ValueError(
                    f"Training segment weight for {segment.label!r} must be a finite positive number, got {weight!r}."
                )
        total = sum(weights)
        return [weight / total for weight in weights]

    async def _compute_costs_for_segmented_training_batch(
        self,
        config: OptimizerConfig,
        theta_batch: torch.Tensor,
        training_segments: list[_TrainingSegment],
        on_progress: Optional[Callable[[str, float], None]] = None,
        include_plot_data: bool = True,
    ) -> tuple[torch.Tensor, torch.Tensor, Optional[SysIdRolloutPlotData]]:
        residual_chunks: list[torch.Tensor] = []
        plot_data: Optional[SysIdRolloutPlotData] = None
        weights = self._normalized_training_weights(training_segments)
        normalize_residual_dimension = len(training_segments) > 1

        for idx, segment in enumerate(training_segments):
            start = idx / max(1, len(training_segments))
            end = (idx + 1) / max(1, len(training_segments))
            chunk_config = replace(
                config,
                trajectory=segment.trajectory,
                training_segments=None,
                training_chunks=None,
                training_chunk_weights=None,
                training_chunk_labels=None,
            )

            def chunk_progress(
                message: str,
                fraction: float,
                *,
                _start=start,  # noqa: ANN001
                _end=end,  # noqa: ANN001
                _label=segment.label,  # noqa: ANN001
            ) -> None:
                if on_progress is None:
                    return
                local = max(0.0, min(1.0, float(fraction)))
                on_progress(f"Train {_label}: {message}", _start + (_end - _start) * local)

            (
                residuals,
                _cost_per_env,
                chunk_plot,
            ) = await self._compute_residuals_for_theta_batch(
                chunk_config,
                theta_batch,
                on_progress=chunk_progress if on_progress is not None else None,
                include_plot_data=include_plot_data and plot_data is None,
            )
            residual_dim = max(1, int(residuals.shape[1]))
            denominator = residual_dim if normalize_residual_dimension else 1
            residual_chunks.append(residuals * float(weights[idx] / denominator) ** 0.5)
            if plot_data is None and chunk_plot is not None:
                plot_data = chunk_plot

        residuals = torch.cat(residual_chunks, dim=1)
        cost_per_env = 0.5 * (residuals * residuals).sum(dim=1)
        return cost_per_env, residuals, plot_data

    async def compute_segmented_costs_and_backward(
        self,
        config: OptimizerConfig,
        theta_batch: torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
        include_plot_data: bool = True,
    ) -> tuple[torch.Tensor, torch.Tensor, Optional[SysIdRolloutPlotData]]:
        """Accumulate segmented-training gradients before shared rollout state is reused.

        This path is required by bridges whose custom autograd nodes reference
        reusable simulator buffers. Each segment is backpropagated immediately,
        while detached per-candidate costs are retained for optimizer decisions.

        Returns:
            Detached per-candidate costs, a finite-candidate mask, and optional nominal rollout plot data.

        Args:
            config: Optimizer and trajectory configuration.
            theta_batch: Candidate parameter rows to evaluate.
            on_progress: Optional progress callback.
            include_plot_data: Whether to construct nominal rollout plot data.
        """
        training_segments = self._training_segments(config)
        weights = self._normalized_training_weights(training_segments)
        normalize_residual_dimension = len(training_segments) > 1
        detached_costs = torch.zeros(theta_batch.shape[0], device=theta_batch.device, dtype=theta_batch.dtype)
        finite_candidates = torch.ones(theta_batch.shape[0], device=theta_batch.device, dtype=torch.bool)
        plot_data: Optional[SysIdRolloutPlotData] = None

        for idx, segment in enumerate(training_segments):
            start = idx / max(1, len(training_segments))
            end = (idx + 1) / max(1, len(training_segments))
            chunk_config = replace(
                config,
                trajectory=segment.trajectory,
                training_segments=None,
                training_chunks=None,
                training_chunk_weights=None,
                training_chunk_labels=None,
            )

            def chunk_progress(
                message: str,
                fraction: float,
                *,
                _start=start,  # noqa: ANN001
                _end=end,  # noqa: ANN001
                _label=segment.label,  # noqa: ANN001
            ) -> None:
                if on_progress is None:
                    return
                local = max(0.0, min(1.0, float(fraction)))
                on_progress(f"Train {_label}: {message}", _start + (_end - _start) * local)

            (
                residuals,
                _cost_per_env,
                chunk_plot,
            ) = await self._compute_residuals_for_theta_batch(
                chunk_config,
                theta_batch,
                on_progress=chunk_progress if on_progress is not None else None,
                include_plot_data=include_plot_data and plot_data is None,
            )
            residual_dim = max(1, int(residuals.shape[1]))
            denominator = residual_dim if normalize_residual_dimension else 1
            segment_costs = 0.5 * (residuals * residuals).sum(dim=1) * float(weights[idx] / denominator)
            if not segment_costs.requires_grad:
                raise SysIdEnvironmentBridgeError(
                    "Segmented gradient accumulation requires a bridge whose rollout costs carry autograd gradients."
                )
            segment_finite = torch.isfinite(segment_costs.detach())
            finite_candidates &= segment_finite
            detached_costs += segment_costs.detach()

            # Always execute backward so the bridge can release the rollout
            # buffers even when every candidate in this segment is non-finite.
            safe_costs = torch.where(segment_finite, segment_costs, torch.zeros_like(segment_costs))
            safe_costs.sum().backward()
            if plot_data is None and chunk_plot is not None:
                plot_data = chunk_plot

        finite_candidates &= torch.isfinite(detached_costs)
        return detached_costs, finite_candidates, plot_data

    async def _compute_residuals_for_theta_batch(
        self,
        config: OptimizerConfig,
        theta_batch: torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
        include_plot_data: bool = True,
    ) -> tuple[torch.Tensor, torch.Tensor, Optional[SysIdRolloutPlotData]]:
        self._raise_if_cancelled()
        bridge = self._bridge
        num_envs = theta_batch.shape[0]
        if not callable(getattr(bridge, "set_trajectory", None)) or not callable(
            getattr(bridge, "run_rollout_async", None)
        ):
            raise SysIdEnvironmentBridgeError(
                "Environment bridge must implement SysIdEnvironmentBridge (run_rollout_async, set_trajectory)."
            )
        self._ensure_bridge_setup(bridge, config)

        num_steps = min(
            config.trajectory.commands.shape[0],
            max(1, config.max_rollout_steps),
        )
        traj = self._cached_trajectory_tensors(config.trajectory, bridge.device, num_steps)
        times = traj["times"]
        meas_pos = traj["positions"]
        meas_vel = traj["velocities"]
        commands = traj["commands"]
        meas_torque = traj["torques"]
        meas_ee = traj["end_effector_poses"]
        meas_contact = traj["contact_forces"]

        residual_cfg = config.residual_weight_config or self._residual_config
        residual_engine = self._engine_for_residual_config(residual_cfg)
        sample_weights = self._cached_sample_weights(residual_cfg, config.trajectory, bridge.device, num_steps)
        expected_trailing_shapes = _expected_rollout_trailing_shapes(
            positions=meas_pos,
            velocities=meas_vel,
            torques=meas_torque,
            end_effector_poses=meas_ee,
            contact_forces=meas_contact,
        )

        configure_signals = getattr(bridge, "configure_rollout_signals", None)
        if callable(configure_signals):
            configure_signals(
                torque=bool(float(residual_cfg.torque_weight) != 0.0 and meas_torque is not None),
                end_effector_pose=bool(float(residual_cfg.end_effector_pose_weight) != 0.0 and meas_ee is not None),
                contact_force=bool(float(residual_cfg.contact_force_weight) != 0.0 and meas_contact is not None),
            )

        use_batch = bool(
            getattr(bridge, "use_parallel_clones", False)
            or getattr(bridge, "supports_arbitrary_candidate_batch", False)
        )
        try:
            if use_batch and num_envs > 1:
                if on_progress:
                    on_progress(f"Running {num_envs} parallel rollouts...", 0.5)
                    await yield_control()
                    self._raise_if_cancelled()
                rollout = await bridge.run_rollout_async(
                    theta_batch.to(device=bridge.device, dtype=torch.float32),
                    config.param_entries,
                    commands,
                    num_steps,
                )
                _validate_rollout_shapes(
                    rollout,
                    expected_envs=num_envs,
                    expected_steps=num_steps,
                    expected_trailing_shapes=expected_trailing_shapes,
                )
                self._raise_if_cancelled()
            else:
                sim_rows_pos: list[torch.Tensor] = []
                sim_rows_vel: list[torch.Tensor] = []
                sim_rows_torque: list[torch.Tensor] = []
                sim_rows_ee: list[torch.Tensor] = []
                sim_rows_contact: list[torch.Tensor] = []
                for env_i in range(num_envs):
                    self._raise_if_cancelled()
                    if on_progress:
                        frac = (env_i + 0.5) / max(num_envs, 1)
                        label = "nominal" if env_i == 0 else f"candidate {env_i}"
                        on_progress(f"Rollout: {label} ({env_i + 1}/{num_envs})", frac)
                        self._raise_if_cancelled()
                    row = theta_batch[env_i : env_i + 1].to(device=bridge.device, dtype=torch.float32)
                    rollout_i = await bridge.run_rollout_async(row, config.param_entries, commands, num_steps)
                    _validate_rollout_shapes(
                        rollout_i,
                        expected_envs=1,
                        expected_steps=num_steps,
                        expected_trailing_shapes=expected_trailing_shapes,
                    )
                    self._raise_if_cancelled()
                    sim_rows_pos.append(rollout_i.positions[0])
                    sim_rows_vel.append(rollout_i.velocities[0])
                    if rollout_i.torques is not None:
                        sim_rows_torque.append(rollout_i.torques[0])
                    if rollout_i.end_effector_poses is not None:
                        sim_rows_ee.append(rollout_i.end_effector_poses[0])
                    if rollout_i.contact_forces is not None:
                        sim_rows_contact.append(rollout_i.contact_forces[0])
                    await yield_control()
                rollout = SysIdRolloutResult(
                    positions=torch.stack(sim_rows_pos, dim=0),
                    velocities=torch.stack(sim_rows_vel, dim=0),
                    torques=_stack_optional_rollout_rows(
                        sim_rows_torque,
                        signal_name="torques",
                        expected_rows=num_envs,
                    ),
                    end_effector_poses=_stack_optional_rollout_rows(
                        sim_rows_ee,
                        signal_name="end-effector poses",
                        expected_rows=num_envs,
                    ),
                    contact_forces=_stack_optional_rollout_rows(
                        sim_rows_contact,
                        signal_name="contact forces",
                        expected_rows=num_envs,
                    ),
                )
                _validate_rollout_shapes(
                    rollout,
                    expected_envs=num_envs,
                    expected_steps=num_steps,
                    expected_trailing_shapes=expected_trailing_shapes,
                )
            sim_pos = rollout.positions
            sim_vel = rollout.velocities
            sim_torque = rollout.torques
            sim_ee = rollout.end_effector_poses
            sim_contact = rollout.contact_forces
        except (RuntimeError, SysIdEnvironmentBridgeError) as exc:
            raise SysIdEnvironmentBridgeError(
                f"Physics rollout failed ({exc}). After CUDA error 700, fully restart Isaac Sim. "
                "Press Play before Run; use robot Xform path (not Geometry/base); shorten CSV window "
                "or lower max rollout steps; try disabling parallel clones if multi-env PhysX fails."
            ) from exc

        residuals, cost_per_env = residual_engine.compute(
            RolloutResidualInputs(
                meas_pos=meas_pos,
                meas_vel=meas_vel,
                sim_pos=sim_pos,
                sim_vel=sim_vel,
                meas_torque=meas_torque,
                sim_torque=sim_torque,
                meas_ee_pose=meas_ee,
                sim_ee_pose=sim_ee,
                meas_contact=meas_contact,
                sim_contact=sim_contact,
                sample_weights=sample_weights,
            )
        )
        if include_plot_data:
            if self._debug_rollout_plots:
                self._clone_debug_plot_count += 1
            if self._debug_rollout_plots and self._clone_debug_plot_count % 2 == 1:
                save_clone_joint_angles_debug_png(config.trajectory, times, sim_pos)
                save_clone_joint_efforts_debug_png(config.trajectory, times, sim_torque)
            plot_data = make_rollout_plot_data(
                times,
                meas_pos,
                meas_vel,
                sim_pos,
                sim_vel,
                0,
                meas_torque=meas_torque,
                sim_torque=sim_torque,
            )
        else:
            plot_data = None
        return residuals, cost_per_env, plot_data

    async def evaluate_validation_chunks(
        self,
        config: OptimizerConfig,
        chunks: list[TrajectoryChunk],
        theta: list[float] | torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> list[ChunkValidationMetric]:
        """Replay final theta on held-out chunks and return per-chunk metrics.

        Args:
            config: Optimizer and trajectory configuration.
            chunks: Held-out trajectory chunks to replay.
            theta: Final parameter vector to evaluate on every validation chunk.
            on_progress: Optional progress callback.

        Returns:
            One validation metric per chunk.
        """
        replays = await self.evaluate_validation_chunks_with_rollouts(
            config,
            chunks,
            theta,
            on_progress=on_progress,
        )
        return [replay.metric for replay in replays]

    async def evaluate_validation_chunks_with_rollouts(
        self,
        config: OptimizerConfig,
        chunks: list[TrajectoryChunk],
        theta: list[float] | torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> list[ChunkValidationReplay]:
        """Replay validation chunks and return metrics plus plot payloads.

        Args:
            config: Optimizer and trajectory configuration.
            chunks: Held-out trajectory chunks to replay.
            theta: Final parameter vector to evaluate on every validation chunk.
            on_progress: Optional progress callback.

        Returns:
            One validation metric and rollout payload per chunk.
        """
        self._raise_if_cancelled()
        replays: list[ChunkValidationReplay] = []
        total = max(1, len(chunks))
        for idx, chunk in enumerate(chunks):
            self._raise_if_cancelled()
            start = idx / total
            end = (idx + 1) / total
            label = chunk.spec.display_name(idx)

            def chunk_progress(
                message: str,
                fraction: float,
                *,
                _start: float = start,
                _end: float = end,
                _label: str = label,
            ) -> None:
                if on_progress is None:
                    return
                local = max(0.0, min(1.0, float(fraction)))
                on_progress(f"Validation {_label}: {message}", _start + (_end - _start) * local)

            replays.append(
                await self.evaluate_validation_chunk_with_rollout(
                    config,
                    chunk,
                    theta,
                    on_progress=chunk_progress if on_progress is not None else None,
                )
            )
            self._raise_if_cancelled()
        return replays

    async def evaluate_validation_parameter_sets_with_rollouts(
        self,
        config: OptimizerConfig,
        chunks: list[TrajectoryChunk],
        theta_values: list[list[float] | torch.Tensor],
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> list[list[ChunkValidationReplay]]:
        """Replay several parameter sets on the same validation chunks.

        Bridges with arbitrary candidate batching evaluate every parameter set
        in one simulation pass per chunk. Other bridges preserve the same
        result contract with serial passes.

        Args:
            config: Optimizer and trajectory configuration.
            chunks: Held-out trajectory chunks to replay.
            theta_values: Parameter vectors evaluated in result order.
            on_progress: Optional progress callback.

        Returns:
            Validation replays grouped by parameter-set index.
        """
        self._raise_if_cancelled()
        if not theta_values:
            return []
        supports_batch = bool(getattr(self._bridge, "supports_arbitrary_candidate_batch", False))
        if len(theta_values) < 2 or not supports_batch:
            grouped: list[list[ChunkValidationReplay]] = []
            total = max(1, len(theta_values))
            for index, theta in enumerate(theta_values):
                self._raise_if_cancelled()
                start = index / total
                end = (index + 1) / total
                grouped.append(
                    await self.evaluate_validation_chunks_with_rollouts(
                        config,
                        chunks,
                        theta,
                        on_progress=self._progress_for_phase(on_progress, start, end),
                    )
                )
                self._raise_if_cancelled()
            return grouped

        grouped = [[] for _ in theta_values]
        total = max(1, len(chunks))
        for chunk_index, chunk in enumerate(chunks):
            self._raise_if_cancelled()
            replays = await self._evaluate_validation_chunk_theta_batch(
                config,
                chunk,
                theta_values,
                on_progress=self._progress_for_phase(
                    on_progress,
                    chunk_index / total,
                    (chunk_index + 1) / total,
                ),
            )
            for parameter_index, replay in enumerate(replays):
                grouped[parameter_index].append(replay)
            self._raise_if_cancelled()
        return grouped

    async def evaluate_validation_chunk(
        self,
        config: OptimizerConfig,
        chunk: TrajectoryChunk,
        theta: list[float] | torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> ChunkValidationMetric:
        """Replay one validation chunk and compute normalized cost plus RMSE values.

        Args:
            config: Optimizer and trajectory configuration.
            chunk: Validation trajectory chunk to replay.
            theta: Final parameter vector to evaluate on the validation chunk.
            on_progress: Optional progress callback.

        Returns:
            Validation metric for ``chunk``.
        """
        return (
            await self.evaluate_validation_chunk_with_rollout(
                config,
                chunk,
                theta,
                on_progress=on_progress,
            )
        ).metric

    async def evaluate_validation_chunk_with_rollout(
        self,
        config: OptimizerConfig,
        chunk: TrajectoryChunk,
        theta: list[float] | torch.Tensor,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> ChunkValidationReplay:
        """Replay one validation chunk and return its metrics plus plot data.

        Args:
            config: Optimizer and trajectory configuration.
            chunk: Validation trajectory chunk to replay.
            theta: Final parameter vector to evaluate on the validation chunk.
            on_progress: Optional progress callback.

        Returns:
            Validation metric and rollout plot payload for ``chunk``.
        """
        replays = await self._evaluate_validation_chunk_theta_batch(
            config,
            chunk,
            [theta],
            on_progress=on_progress,
        )
        return replays[0]

    async def _evaluate_validation_chunk_theta_batch(
        self,
        config: OptimizerConfig,
        chunk: TrajectoryChunk,
        theta_values: list[list[float] | torch.Tensor],
        *,
        on_progress: Optional[Callable[[str, float], None]] = None,
    ) -> list[ChunkValidationReplay]:
        """Replay one validation chunk for a candidate batch.

        Args:
            config: Optimizer and trajectory configuration.
            chunk: Validation trajectory chunk to replay.
            theta_values: Parameter vectors evaluated in result order.
            on_progress: Optional progress callback.

        Returns:
            One validation replay per parameter vector.
        """
        self._raise_if_cancelled()
        if not theta_values:
            return []
        bridge = self._bridge
        if not callable(getattr(bridge, "set_trajectory", None)) or not callable(
            getattr(bridge, "run_rollout_async", None)
        ):
            raise SysIdEnvironmentBridgeError(
                "Environment bridge must implement SysIdEnvironmentBridge (run_rollout_async, set_trajectory)."
            )

        chunk_config = replace(
            config,
            trajectory=chunk.trajectory,
            training_segments=None,
            training_chunks=None,
            training_chunk_weights=None,
            training_chunk_labels=None,
        )
        self._ensure_bridge_setup(bridge, chunk_config)
        num_steps = min(
            chunk.trajectory.commands.shape[0],
            max(1, chunk_config.max_rollout_steps),
        )
        traj = self._cached_trajectory_tensors(chunk.trajectory, bridge.device, num_steps)
        commands = traj["commands"]
        if commands is None:
            raise SysIdEnvironmentBridgeError("Validation chunk is missing commands.")

        residual_cfg = chunk_config.residual_weight_config or self._residual_config
        residual_engine = self._engine_for_residual_config(residual_cfg)
        sample_weights = self._cached_sample_weights(residual_cfg, chunk.trajectory, bridge.device, num_steps)
        meas_pos = traj["positions"]
        meas_vel = traj["velocities"]
        meas_torque = traj["torques"]
        meas_ee = traj["end_effector_poses"]
        meas_contact = traj["contact_forces"]
        expected_trailing_shapes = _expected_rollout_trailing_shapes(
            positions=meas_pos,
            velocities=meas_vel,
            torques=meas_torque,
            end_effector_poses=meas_ee,
            contact_forces=meas_contact,
        )

        configure_signals = getattr(bridge, "configure_rollout_signals", None)
        if callable(configure_signals):
            configure_signals(
                torque=bool(float(residual_cfg.torque_weight) != 0.0 and meas_torque is not None),
                end_effector_pose=bool(float(residual_cfg.end_effector_pose_weight) != 0.0 and meas_ee is not None),
                contact_force=bool(float(residual_cfg.contact_force_weight) != 0.0 and meas_contact is not None),
            )

        if on_progress:
            on_progress("rollout", 0.25)
            await yield_control()
            self._raise_if_cancelled()

        theta_tensor = torch.stack(
            [torch.as_tensor(theta, device=bridge.device, dtype=torch.float32).reshape(-1) for theta in theta_values],
            dim=0,
        )
        rollout = await bridge.run_rollout_async(theta_tensor, chunk_config.param_entries, commands, num_steps)
        self._raise_if_cancelled()
        _validate_rollout_shapes(
            rollout,
            expected_envs=len(theta_values),
            expected_steps=num_steps,
            expected_trailing_shapes=expected_trailing_shapes,
        )

        sim_pos = rollout.positions
        sim_vel = rollout.velocities
        sim_torque = rollout.torques
        sim_ee = rollout.end_effector_poses
        sim_contact = rollout.contact_forces

        residuals, cost_per_env = residual_engine.compute(
            RolloutResidualInputs(
                meas_pos=meas_pos,
                meas_vel=meas_vel,
                sim_pos=sim_pos,
                sim_vel=sim_vel,
                meas_torque=meas_torque,
                sim_torque=sim_torque,
                meas_ee_pose=meas_ee,
                sim_ee_pose=sim_ee,
                meas_contact=meas_contact,
                sim_contact=sim_contact,
                sample_weights=sample_weights,
            )
        )
        residual_dim = max(1, int(residuals.shape[1]))

        if on_progress:
            on_progress("metrics", 0.9)
            await yield_control()
            self._raise_if_cancelled()

        replays: list[ChunkValidationReplay] = []
        costs = cost_per_env.detach().cpu().tolist()
        for env_index, cost_value in enumerate(costs):
            cost = float(cost_value)
            metric = ChunkValidationMetric(
                name=chunk.spec.name,
                role=chunk.spec.role,
                excitation=chunk.spec.excitation,
                start=float(chunk.spec.start),
                end=float(chunk.spec.end),
                sample_count=int(num_steps),
                cost=cost,
                normalized_cost=cost / residual_dim,
                position_rmse=_rmse(sim_pos[env_index, :num_steps], meas_pos[:num_steps]),
                velocity_rmse=_rmse(sim_vel[env_index, :num_steps], meas_vel[:num_steps]),
                torque_rmse=(
                    _rmse(sim_torque[env_index, :num_steps], meas_torque[:num_steps])
                    if sim_torque is not None and meas_torque is not None
                    else None
                ),
                end_effector_pose_rmse=(
                    _rmse(sim_ee[env_index, :num_steps], meas_ee[:num_steps])
                    if sim_ee is not None and meas_ee is not None
                    else None
                ),
                contact_force_rmse=(
                    _rmse(sim_contact[env_index, :num_steps], meas_contact[:num_steps])
                    if sim_contact is not None and meas_contact is not None
                    else None
                ),
            )
            rollout_plot = make_rollout_plot_data(
                traj["times"],
                meas_pos,
                meas_vel,
                sim_pos,
                sim_vel,
                env_index,
                meas_torque=meas_torque,
                sim_torque=sim_torque,
            )
            replays.append(ChunkValidationReplay(metric=metric, rollout=rollout_plot))
        return replays

    async def evaluate_nominal_cost(self, config: OptimizerConfig, theta: torch.Tensor) -> float:  # noqa: D102
        batch = theta.unsqueeze(0)
        cost, _, _ = await self.compute_costs_for_theta_batch(config, batch, include_plot_data=False)
        return float(cost[0].item())


def _rmse(sim: torch.Tensor, meas: torch.Tensor) -> float:
    diff = sim.to(dtype=torch.float32) - meas.to(device=sim.device, dtype=torch.float32)
    return float(torch.sqrt(torch.mean(diff * diff)).detach().cpu().item())


def _segment_label(chunk: TrajectoryChunk, index: int, total: int) -> str:
    excitation = chunk.spec.excitation.strip() or "custom"
    name = chunk.spec.name.strip()
    ordinal = f"{index + 1}/{max(1, total)}"
    if name and name != excitation:
        return f"{excitation} {ordinal} ({name})"
    return f"{excitation} {ordinal}"
