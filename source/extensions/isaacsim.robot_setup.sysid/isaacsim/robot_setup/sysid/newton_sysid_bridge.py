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

"""Newton actuator rollout bridge for SysID."""

from __future__ import annotations

import gc
import importlib
import inspect
import math
import time
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field, replace
from importlib import metadata
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import carb
import numpy as np
import torch

from .actuator_compatibility import (
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_IMPLICIT,
    ACTUATOR_RUNTIME_MIXED,
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    is_actuator_delay_parameter,
    resolve_actuator_dof_plan,
)
from .backend_capabilities import NEWTON_MUJOCO_CAPABILITIES
from .env_bridge import SysIdEnvironmentBridgeError
from .errors import SysIdRuntimeUnavailableError
from .feedforward import (
    FF_CACHE_MAX_ENTRIES,
    FeedforwardCacheEntry,
    compute_feedforward,
    feedforward_content_hash,
    filtered_acceleration,
)
from .inertia_param import inertia_matrix_to_log_cholesky
from .parameter_apply import decode_theta_to_apply_state
from .parameter_types import (
    GLOBAL_DOF_INDEX,
    GLOBAL_LINK_INDEX,
    SysIdParameterEntry,
    SysIdParameterType,
)
from .rollout_result import SysIdRolloutResult
from .run_spec import NEWTON_SOLVER_MUJOCO, NewtonSimulationRunSpec
from .trajectory_csv import TrajectoryDataset

if TYPE_CHECKING:
    from .usd_parameter_io import JointUsdSnapshot

NEWTON_ACTUATOR_SOURCE_DRIVE_DEFAULTS = "drive_defaults"
NEWTON_ACTUATOR_SOURCE_USD = "usd"
NEWTON_CONTROLLER_PD = "pd"
NEWTON_CONTROLLER_PID = "pid"
NEWTON_CONTROLLER_NEURAL_MLP = "neural_mlp"
NEWTON_CONTROLLER_NEURAL_LSTM = "neural_lstm"
NEWTON_CLAMP_MAX_EFFORT = "max_effort"
NEWTON_CLAMP_DC_MOTOR = "dc_motor"
NEWTON_CLAMP_NONE = "none"

_NEWTON_NEURAL_CONTROLLERS = {
    NEWTON_CONTROLLER_NEURAL_MLP,
    NEWTON_CONTROLLER_NEURAL_LSTM,
}

_NEWTON_SUPPORTED_PARAMETERS = NEWTON_MUJOCO_CAPABILITIES.parameter_types

_NEWTON_LINK_INERTIAL_PARAMETERS = {
    SysIdParameterType.LINK_MASS,
    SysIdParameterType.LINK_COM_OFFSET_X,
    SysIdParameterType.LINK_COM_OFFSET_Y,
    SysIdParameterType.LINK_COM_OFFSET_Z,
    SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
}

_MUJOCO_IO_KERNEL_CACHE: dict[int, tuple[Any, ...]] = {}
_INVALID_CANDIDATE_SENTINEL = 1.0e6
# Joint rates beyond this magnitude (rad/s or m/s, ~95,000 rpm) mark a diverged
# integration even while still finite. An explicit-stiffness blow-up exceeds it
# within a sample or two while MuJoCo keeps the state bounded, and runaway body
# poses can overrun MuJoCo's contact buffers before the state turns NaN.
_DIVERGED_RATE_LIMIT = 1.0e4
# Beyond this many MuJoCo substeps per sample, the divergence diagnostic points
# at the asset instead of recommending a substep count.
_PRACTICAL_MUJOCO_SUBSTEPS = 64


def _physical_inertia_is_valid(inertia: np.ndarray) -> bool:
    """Return whether an inertia is finite, SPD, and physically realizable.

    Log-Cholesky guarantees the first two properties. A rigid-body inertia also
    needs each principal moment to be no greater than the sum of the other two.

    Args:
        inertia: Value supplied for ``inertia``.

    Returns:
        Result produced by the operation.
    """
    matrix = np.asarray(inertia, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(matrix).all():
        return False
    if not np.allclose(matrix, matrix.T, rtol=1.0e-6, atol=1.0e-9):
        return False
    try:
        moments = np.linalg.eigvalsh(matrix)
    except np.linalg.LinAlgError:
        return False
    if not np.isfinite(moments).all() or float(moments[0]) <= 0.0:
        return False
    tolerance = max(1.0e-9, 1.0e-5 * float(moments[-1]))
    return bool(float(moments[-1]) <= float(moments[0] + moments[1]) + tolerance)


@dataclass
class _MujocoCaptureRecord:
    """One fixed-shape MuJoCo forward-rollout CUDA graph."""

    graph: Any
    replay_logged: bool = False


def _load_mujoco_io_kernels(wp) -> tuple[Any, ...]:  # noqa: ANN001
    """Build kernels that stage controls and gather mapped state on the Warp device.

    Args:
        wp: Value supplied for ``wp``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    cached = _MUJOCO_IO_KERNEL_CACHE.get(id(wp))
    if cached is not None:
        return cached

    @wp.kernel
    def scatter_position_target_row(  # noqa: ANN202
        target_pos_stage: wp.array2d(dtype=wp.float32),
        step: int,
        joint_target_pos: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        joint_target_pos[tid] = target_pos_stage[step, tid]

    @wp.kernel
    def scatter_dof_control_row(  # noqa: ANN202
        target_vel_stage: wp.array2d(dtype=wp.float32),
        feedforward_stage: wp.array2d(dtype=wp.float32),
        step: int,
        joint_target_vel: wp.array(dtype=wp.float32),
        joint_f: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        joint_target_vel[tid] = target_vel_stage[step, tid]
        joint_f[tid] = feedforward_stage[step, tid]

    @wp.kernel
    def gather_joint_state(  # noqa: ANN202
        joint_q: wp.array(dtype=wp.float32),
        joint_qd: wp.array(dtype=wp.float32),
        coord_index_map: wp.array(dtype=wp.int32),
        dof_index_map: wp.array(dtype=wp.int32),
        step: int,
        sim_q: wp.array2d(dtype=wp.float32),
        sim_qd: wp.array2d(dtype=wp.float32),
    ):
        tid = wp.tid()
        sim_q[step, tid] = joint_q[coord_index_map[tid]]
        sim_qd[step, tid] = joint_qd[dof_index_map[tid]]

    @wp.kernel
    def gather_actuator_inputs(  # noqa: ANN202
        joint_q: wp.array(dtype=wp.float32),
        joint_qd: wp.array(dtype=wp.float32),
        coord_index_map: wp.array(dtype=wp.int32),
        dof_index_map: wp.array(dtype=wp.int32),
        target_pos_index_map: wp.array(dtype=wp.int32),
        target_pos_stage: wp.array2d(dtype=wp.float32),
        target_vel_stage: wp.array2d(dtype=wp.float32),
        feedforward_stage: wp.array2d(dtype=wp.float32),
        step: int,
        compact_q: wp.array(dtype=wp.float32),
        compact_qd: wp.array(dtype=wp.float32),
        compact_target_pos: wp.array(dtype=wp.float32),
        compact_target_vel: wp.array(dtype=wp.float32),
        compact_feedforward: wp.array(dtype=wp.float32),
        compact_effort: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        dof = dof_index_map[tid]
        target_pos = target_pos_index_map[tid]
        compact_q[tid] = joint_q[coord_index_map[tid]]
        compact_qd[tid] = joint_qd[dof]
        compact_target_pos[tid] = target_pos_stage[step, target_pos]
        compact_target_vel[tid] = target_vel_stage[step, dof]
        compact_feedforward[tid] = feedforward_stage[step, dof]
        compact_effort[tid] = 0.0

    @wp.kernel
    def scatter_explicit_effort(  # noqa: ANN202
        compact_effort: wp.array(dtype=wp.float32),
        compact_indices: wp.array(dtype=wp.int32),
        model_dof_indices: wp.array(dtype=wp.int32),
        joint_f: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        joint_f[model_dof_indices[tid]] = compact_effort[compact_indices[tid]]

    @wp.kernel
    def gather_explicit_effort(  # noqa: ANN202
        compact_effort: wp.array(dtype=wp.float32),
        compact_indices: wp.array(dtype=wp.int32),
        step: int,
        effort_stage: wp.array2d(dtype=wp.float32),
    ):
        tid = wp.tid()
        effort_stage[step, tid] = compact_effort[compact_indices[tid]]

    @wp.kernel
    def copy_explicit_effort_row(  # noqa: ANN202
        effort_stage: wp.array2d(dtype=wp.float32),
        source_step: int,
        target_step: int,
    ):
        tid = wp.tid()
        effort_stage[target_step, tid] = effort_stage[source_step, tid]

    @wp.kernel
    def reset_diverged_steps(  # noqa: ANN202
        diverged_step: wp.array(dtype=wp.int32),
    ):
        diverged_step[wp.tid()] = -1

    @wp.kernel
    def quarantine_diverged_worlds(  # noqa: ANN202
        joint_q: wp.array(dtype=wp.float32),
        joint_qd: wp.array(dtype=wp.float32),
        initial_joint_q: wp.array(dtype=wp.float32),
        initial_joint_qd: wp.array(dtype=wp.float32),
        coords_per_world: int,
        dofs_per_world: int,
        step: int,
        rate_limit: float,
        diverged_step: wp.array(dtype=wp.int32),
    ):
        # Record the first diverged step of each world and put that world back
        # on its finite initial state, so the next MuJoCo step never runs
        # collision detection on NaN or runaway body poses. The world's
        # remaining samples are discarded by the caller.
        world = wp.tid()
        q_start = world * coords_per_world
        qd_start = world * dofs_per_world
        diverged = int(0)  # noqa: UP018 - Warp needs a typed local to mutate it inside a dynamic loop
        for i in range(coords_per_world):
            if not wp.isfinite(joint_q[q_start + i]):
                diverged = 1
        for i in range(dofs_per_world):
            value = joint_qd[qd_start + i]
            if not wp.isfinite(value) or wp.abs(value) > rate_limit:
                diverged = 1
        if diverged != 0:
            if diverged_step[world] < 0:
                diverged_step[world] = step
            for i in range(coords_per_world):
                joint_q[q_start + i] = initial_joint_q[q_start + i]
            for i in range(dofs_per_world):
                joint_qd[qd_start + i] = initial_joint_qd[qd_start + i]

    _MUJOCO_IO_KERNEL_CACHE[id(wp)] = (
        scatter_position_target_row,
        scatter_dof_control_row,
        gather_joint_state,
        gather_actuator_inputs,
        scatter_explicit_effort,
        gather_explicit_effort,
        copy_explicit_effort_row,
        reset_diverged_steps,
        quarantine_diverged_worlds,
    )
    return _MUJOCO_IO_KERNEL_CACHE[id(wp)]


def _record_mujoco_rollout_launches(bridge, ctx, num_steps: int, dt: float) -> None:  # noqa: ANN001
    """Issue an allocation-free, fixed-shape MuJoCo rollout launch sequence.

    Initial state, commands, feedforward, and model parameters live in stable
    device buffers populated before this function. Keeping reset inside the
    sequence makes graph replay idempotent even though MuJoCo alternates two
    state buffers while stepping.

    Args:
        bridge: Value supplied for ``bridge``.
        ctx: Value supplied for ``ctx``.
        num_steps: Value supplied for ``num_steps``.
        dt: Value supplied for ``dt``.
    """  # noqa: DOC107
    wp = bridge._modules.warp
    newton = bridge._modules.newton
    model, solver = ctx.model, ctx.solver
    state0, state1 = ctx.state0, ctx.state1
    control = ctx.control
    (
        scatter_position_target_row,
        scatter_dof_control_row,
        gather_joint_state,
        gather_actuator_inputs,
        scatter_explicit_effort,
        gather_explicit_effort,
        copy_explicit_effort_row,
        reset_diverged_steps,
        quarantine_diverged_worlds,
    ) = _load_mujoco_io_kernels(wp)
    mapped_width = int(ctx.world_count * ctx.num_dof)
    joint_target_q_count = int(control.joint_target_q.shape[0])
    joint_dof_count = int(model.joint_dof_count)
    # Controls and explicit actuators update once per telemetry sample; MuJoCo
    # integrates that interval in `substeps` equal physics steps.
    substeps = max(1, int(getattr(bridge, "_mujoco_substeps", 1)))
    sub_dt = float(dt) / substeps
    diverged_step = getattr(ctx, "diverged_step", None)
    if diverged_step is not None:
        wp.launch(reset_diverged_steps, dim=int(ctx.world_count), outputs=[diverged_step], device=model.device)

    # MuJoCo carries acceleration warm-start, actuator activation, applied-force,
    # and control buffers across steps. Clear those rollout-local values while
    # preserving the staged generalized coordinates copied immediately below.
    solver.reset(state0, flags=0)
    wp.copy(state0.joint_q, ctx.initial_joint_q_stage)
    wp.copy(state0.joint_qd, ctx.initial_joint_qd_stage)
    eval_fk = getattr(newton, "eval_fk", None)
    if callable(eval_fk):
        try:
            eval_fk(model, state0.joint_q, state0.joint_qd, state0)
        except Exception:
            # SolverMuJoCo resynchronizes body transforms from generalized
            # coordinates internally; retain that established fallback.
            pass
    wp.launch(
        gather_joint_state,
        dim=mapped_width,
        inputs=[
            state0.joint_q,
            state0.joint_qd,
            ctx.coord_map_wp,
            ctx.dof_map_wp,
            0,
        ],
        outputs=[ctx.sim_q_stage, ctx.sim_qd_stage],
        device=model.device,
    )
    runtime_actuators = list(getattr(ctx, "runtime_actuators", []))
    cur_states = list(getattr(ctx, "runtime_actuator_states", []))
    nxt_states = list(getattr(ctx, "runtime_next_actuator_states", []))
    for cur, nxt in zip(cur_states, nxt_states):
        if cur is not None:
            cur.reset()
        if nxt is not None:
            nxt.reset()
    for step in range(num_steps - 1):
        wp.launch(
            scatter_position_target_row,
            dim=joint_target_q_count,
            inputs=[
                ctx.target_pos_stage,
                step,
            ],
            outputs=[control.joint_target_q],
            device=model.device,
        )
        wp.launch(
            scatter_dof_control_row,
            dim=joint_dof_count,
            inputs=[
                ctx.target_vel_stage,
                ctx.feedforward_stage,
                step,
            ],
            outputs=[
                control.joint_target_qd,
                control.joint_f,
            ],
            device=model.device,
        )
        if runtime_actuators:
            wp.launch(
                gather_actuator_inputs,
                dim=mapped_width,
                inputs=[
                    state0.joint_q,
                    state0.joint_qd,
                    ctx.coord_map_wp,
                    ctx.dof_map_wp,
                    ctx.target_pos_map_wp,
                    ctx.target_pos_stage,
                    ctx.target_vel_stage,
                    ctx.feedforward_stage,
                    step,
                ],
                outputs=[
                    ctx.actuator_q,
                    ctx.actuator_qd,
                    ctx.actuator_target_pos,
                    ctx.actuator_target_vel,
                    ctx.actuator_feedforward,
                    ctx.actuator_effort,
                ],
                device=model.device,
            )
            sim_state = SimpleNamespace(joint_q=ctx.actuator_q, joint_qd=ctx.actuator_qd)
            sim_control = SimpleNamespace(
                joint_target_q=ctx.actuator_target_pos,
                joint_target_qd=ctx.actuator_target_vel,
                joint_control_feedforward=ctx.actuator_feedforward,
                joint_f=ctx.actuator_effort,
            )
            for actuator, cur, nxt in zip(runtime_actuators, cur_states, nxt_states):
                actuator.step(sim_state, sim_control, cur, nxt, dt=dt)
            cur_states, nxt_states = nxt_states, cur_states
            wp.launch(
                scatter_explicit_effort,
                dim=int(ctx.explicit_compact_indices.shape[0]),
                inputs=[
                    ctx.actuator_effort,
                    ctx.explicit_compact_indices,
                    ctx.explicit_model_dof_indices,
                ],
                outputs=[control.joint_f],
                device=model.device,
            )
            wp.launch(
                gather_explicit_effort,
                dim=int(ctx.explicit_compact_indices.shape[0]),
                inputs=[
                    ctx.actuator_effort,
                    ctx.explicit_compact_indices,
                    step,
                ],
                outputs=[ctx.sim_explicit_effort_stage],
                device=model.device,
            )
        for _substep in range(substeps):
            solver.step(state0, state1, control, None, sub_dt)
            state0, state1 = state1, state0
            if diverged_step is not None:
                wp.launch(
                    quarantine_diverged_worlds,
                    dim=int(ctx.world_count),
                    inputs=[
                        state0.joint_q,
                        state0.joint_qd,
                        ctx.initial_joint_q_stage,
                        ctx.initial_joint_qd_stage,
                        int(ctx.coords_per_world),
                        int(ctx.dofs_per_world),
                        step + 1,
                        _DIVERGED_RATE_LIMIT,
                    ],
                    outputs=[diverged_step],
                    device=model.device,
                )
        wp.launch(
            gather_joint_state,
            dim=mapped_width,
            inputs=[
                state0.joint_q,
                state0.joint_qd,
                ctx.coord_map_wp,
                ctx.dof_map_wp,
                step + 1,
            ],
            outputs=[ctx.sim_q_stage, ctx.sim_qd_stage],
            device=model.device,
        )
    if runtime_actuators and num_steps > 1:
        # A state sample exists at the terminal row but there is no additional
        # control interval. Repeat the last applied effort so the public torque
        # channel retains its established (time, state, effort) shape.
        wp.launch(
            copy_explicit_effort_row,
            dim=int(ctx.explicit_compact_indices.shape[0]),
            inputs=[ctx.sim_explicit_effort_stage, num_steps - 2, num_steps - 1],
            device=model.device,
        )


def _capture_mujoco_rollout_graph(
    bridge, ctx, num_steps: int, dt: float, capture_key: tuple  # noqa: ANN001
) -> _MujocoCaptureRecord | None:
    """Capture and prime one MuJoCo rollout graph, falling back on any failure.

    Args:
        bridge: Value supplied for ``bridge``.
        ctx: Value supplied for ``ctx``.
        num_steps: Value supplied for ``num_steps``.
        dt: Value supplied for ``dt``.
        capture_key: Value supplied for ``capture_key``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    wp = bridge._modules.warp
    device = ctx.model.device
    start = time.perf_counter()
    try:
        with wp.ScopedCapture(device) as capture:
            _record_mujoco_rollout_launches(bridge, ctx, num_steps, dt)
        record = _MujocoCaptureRecord(graph=capture.graph)
        # The graph begins with an initial-state reset, so priming is safe and
        # also keeps graph-instantiation failures inside the fallback handler.
        wp.capture_launch(record.graph)
        wp.synchronize_device(device)
    except Exception as exc:  # noqa: BLE001 - capture must always degrade safely
        ctx.capture_cache.clear()
        ctx.capture_failed = True
        carb.log_warn(
            "SysId: CUDA graph capture failed for the Newton MuJoCo rollout "
            f"(steps={num_steps}, worlds={ctx.world_count}): {exc}. Falling back to uncaptured rollouts "
            "for this model."
        )
        return None
    while len(ctx.capture_cache) >= 4:
        ctx.capture_cache.pop(next(iter(ctx.capture_cache)))
    ctx.capture_cache[capture_key] = record
    carb.log_info(
        f"SysId: captured Newton MuJoCo rollout into a CUDA graph in {time.perf_counter() - start:.1f} s "
        f"(steps={num_steps}, worlds={ctx.world_count})."
    )
    return record


@dataclass
class NewtonActuatorConfig:
    """Normalized Newton actuator settings for one joint DOF."""

    dof_index: int
    source: str = NEWTON_ACTUATOR_SOURCE_DRIVE_DEFAULTS
    controller_kind: str = NEWTON_CONTROLLER_PD
    clamp_kind: str = NEWTON_CLAMP_MAX_EFFORT
    stiffness: float = 1.0
    damping: float = 1.0
    integral_gain: float = 0.0
    integral_max: float = float("inf")
    const_effort: float = 0.0
    effort_limit: float = float("inf")
    max_motor_effort: float = float("inf")
    saturation_effort: float = float("inf")
    velocity_limit: float = float("inf")
    delay_steps: int = 0
    model_path: str = ""
    resolved_model_path: str = ""
    model_type: str = ""
    controller_class_name: str = ""
    controller_kwargs: dict[str, Any] = field(default_factory=dict)
    unsupported_schemas: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        payload = asdict(self)
        return payload


def _reported_net_joint_effort(
    actuator_effort: torch.Tensor,
    joint_velocity: torch.Tensor,
    friction: torch.Tensor,
    configs: list[NewtonActuatorConfig],
) -> torch.Tensor:
    """Apply friction and actuator saturation to a reported generalized effort.

    Newton's MuJoCo solver does not expose a portable per-DOF generalized-force
    readback. This mirrors the configured actuator pipeline so the rollout
    channel has the same net-effort meaning as the differentiable bridge.

    Args:
        actuator_effort: Value supplied for ``actuator_effort``.
        joint_velocity: Value supplied for ``joint_velocity``.
        friction: Value supplied for ``friction``.
        configs: Value supplied for ``configs``.

    Returns:
        Result produced by the operation.
    """
    # Keep the same portable torque convention as the differentiable bridge:
    # friction is part of the generalized effort injected into the plant, and
    # the configured actuator envelope limits that total effort.
    net_effort = actuator_effort - friction * torch.tanh(joint_velocity * 100.0)
    clamped_columns: list[torch.Tensor] = []
    for dof, cfg in enumerate(configs[: actuator_effort.shape[1]]):
        effort = net_effort[:, dof]
        if cfg.clamp_kind == NEWTON_CLAMP_MAX_EFFORT:
            limit = max(float(cfg.effort_limit), 0.0)
            effort = torch.clamp(effort, min=-limit, max=limit)
        elif cfg.clamp_kind == NEWTON_CLAMP_DC_MOTOR:
            max_motor = max(float(cfg.max_motor_effort), 0.0)
            saturation = max(float(cfg.saturation_effort), 0.0)
            velocity_limit = float(cfg.velocity_limit)
            if math.isfinite(velocity_limit) and velocity_limit > 0.0:
                velocity_ratio = joint_velocity[:, dof] / velocity_limit
                upper = torch.clamp(saturation * (1.0 - velocity_ratio), min=0.0, max=max_motor)
                lower = torch.clamp(saturation * (-1.0 - velocity_ratio), min=-max_motor, max=0.0)
                effort = torch.maximum(torch.minimum(effort, upper), lower)
            else:
                effort = torch.clamp(effort, min=-max_motor, max=max_motor)
        clamped_columns.append(effort)
    if len(clamped_columns) != actuator_effort.shape[1]:
        raise SysIdEnvironmentBridgeError(
            "Newton actuator configuration count does not match the reported torque DOFs."
        )
    return torch.stack(clamped_columns, dim=1)


@dataclass
class NewtonActuatorSpec:
    """Debuggable record of one configured Newton actuator."""

    dof_index: int
    controller: Any
    delay: Any
    clamping: Any
    stiffness: float
    damping: float
    effort_limit: float
    config: NewtonActuatorConfig
    actuator: Any = None
    actuator_state: Any = None
    next_actuator_state: Any = None

    @property
    def controller_kind(self) -> str:  # noqa: D102
        return self.config.controller_kind

    @property
    def clamp_kind(self) -> str:  # noqa: D102
        return self.config.clamp_kind

    @property
    def integral_gain(self) -> float:  # noqa: D102
        return self.config.integral_gain


@dataclass
class NewtonCoreModules:
    """Optional Newton modules shared by every Newton rollout backend."""

    newton: Any
    warp: Any


@dataclass
class NewtonModules:
    """Optional Newton modules required by the MuJoCo actuator bridge."""

    newton: Any
    warp: Any
    actuators: Any


def newton_modules_available() -> tuple[bool, str]:
    """Return whether the optional Newton/Warp actuator modules can be imported.

    Returns:
        Result produced by the operation.
    """
    try:
        _load_newton_modules()
    except SysIdRuntimeUnavailableError as exc:
        return False, str(exc)
    return newton_dependency_compatibility()


def newton_dependency_compatibility() -> tuple[bool, str]:
    """Validate the Newton 1.6 coupled package family used by this bridge.

    Returns:
        Result produced by the operation.
    """
    expected = {
        "newton": ("newton", "1.6."),
        "newton-usd-schemas": ("newton_usd_schemas", "0.5."),
        "mujoco": ("mujoco", "3.12."),
        "mujoco-warp": ("mujoco_warp", "3.12."),
    }
    mismatches = []
    for distribution, (module_name, prefix) in expected.items():
        try:
            installed = metadata.version(distribution)
        except metadata.PackageNotFoundError:
            # Kit's fast importer indexes bundled pip metadata by import-module
            # name and does not normalize PyPI hyphens to underscores.
            try:
                installed = metadata.version(module_name)
            except metadata.PackageNotFoundError:
                mismatches.append(f"{distribution} is missing")
                continue
        if not installed.startswith(prefix):
            mismatches.append(f"{distribution}=={installed} (expected {prefix}x)")
    if mismatches:
        return False, "Newton 1.6 dependency set mismatch: " + ", ".join(mismatches)
    return True, ""


def newton_core_modules_available() -> tuple[bool, str]:
    """Return whether the Newton and Warp core modules can be imported.

    Returns:
        Result produced by the operation.
    """
    try:
        _load_newton_core_modules()
    except SysIdRuntimeUnavailableError as exc:
        return False, str(exc)
    return True, ""


def delay_seconds_to_steps(delay_seconds: float, actuator_dt: float) -> int:
    """Quantize a continuous SysID delay value to Newton actuator timesteps.

    Args:
        delay_seconds: Non-negative command delay in seconds.
        actuator_dt: Positive actuator update period in seconds.

    Returns:
        Result produced by the operation.
    """
    if actuator_dt <= 0.0 or not math.isfinite(float(actuator_dt)):
        raise ValueError("actuator_dt must be finite and positive.")
    if not math.isfinite(float(delay_seconds)):
        raise ValueError("delay_seconds must be finite.")
    return max(0, int(round(max(float(delay_seconds), 0.0) / float(actuator_dt))))


def actuator_command_delay_seconds_vector(
    num_dof: int,
    param_entries: list[SysIdParameterEntry],
    theta_row: torch.Tensor | np.ndarray,
) -> np.ndarray:
    """Decode shared/per-DOF command delay entries from one theta row.

    Args:
        num_dof: Number of trajectory DOFs.
        param_entries: Metadata describing each theta value.
        theta_row: One candidate parameter row.

    Returns:
        Per-DOF delay values, with unselected DOFs resolved to zero.
    """
    if isinstance(theta_row, torch.Tensor):
        theta_values = theta_row.detach().to(device="cpu", dtype=torch.float32).reshape(-1).numpy()
    else:
        theta_values = np.asarray(theta_row, dtype=np.float32).reshape(-1)
    if theta_values.shape[0] != len(param_entries):
        raise ValueError(
            f"theta row has {theta_values.shape[0]} values but {len(param_entries)} parameter entries were given."
        )
    delay = np.full(num_dof, np.nan, dtype=np.float32)
    for index, entry in enumerate(param_entries):
        if not is_actuator_delay_parameter(entry.param_type):
            continue
        value = float(theta_values[index])
        if not math.isfinite(value):
            raise ValueError(f"Actuator command delay parameter at theta index {index} must be finite.")
        value = max(value, 0.0)
        dof_index = int(entry.dof_index)
        if dof_index == GLOBAL_DOF_INDEX:
            delay[~np.isfinite(delay)] = value
        elif 0 <= dof_index < num_dof:
            delay[dof_index] = value
        else:
            raise ValueError(
                f"{entry.param_type.value} dof_index {entry.dof_index} is outside [0, {num_dof}) "
                f"and is not GLOBAL_DOF_INDEX."
            )
    return np.where(np.isfinite(delay), delay, 0.0).astype(np.float32)


def _delayed_command_rows(
    command_rows: torch.Tensor,
    delay_seconds_rows: np.ndarray,
    actuator_dt: float,
) -> torch.Tensor:
    """Apply per-world, per-DOF delays to commands sent to the Newton solver.

    Args:
        command_rows: Shared command trajectory with shape ``(steps, dof)``.
        delay_seconds_rows: Per-candidate delays with shape ``(worlds, dof)``.
        actuator_dt: Positive actuator update period in seconds.

    Returns:
        Delayed commands with shape ``(worlds, steps, dof)``.
    """
    if command_rows.ndim != 2:
        raise ValueError("command_rows must have shape (steps, dof).")
    if not math.isfinite(float(actuator_dt)) or float(actuator_dt) <= 0.0:
        raise ValueError("actuator_dt must be finite and positive.")
    delays = np.asarray(delay_seconds_rows, dtype=np.float32)
    if delays.ndim != 2 or delays.shape[1] != int(command_rows.shape[1]):
        raise ValueError("delay_seconds_rows must have shape (worlds, dof).")
    if not np.isfinite(delays).all():
        raise ValueError("delay_seconds_rows must contain only finite values.")
    delay_steps = np.rint(np.maximum(delays, 0.0) / float(actuator_dt)).astype(np.int64)
    delay_steps_t = torch.as_tensor(delay_steps, device=command_rows.device, dtype=torch.long)
    sample_indices = torch.arange(command_rows.shape[0], device=command_rows.device, dtype=torch.long)
    source_indices = torch.clamp(sample_indices.view(1, -1, 1) - delay_steps_t.unsqueeze(1), min=0)
    source = command_rows.unsqueeze(0).expand(int(delays.shape[0]), -1, -1)
    return torch.gather(source, dim=1, index=source_indices)


def _commands_for_actuator_owners(
    command_rows: torch.Tensor,
    delay_seconds_rows: np.ndarray,
    actuator_dt: float,
    explicit_columns: tuple[int, ...],
) -> torch.Tensor:
    """Stage delayed implicit commands and pristine explicit-actuator commands.

    Explicit Newton actuators own their internal delay state. Their command
    columns therefore bypass the bridge-side shift so every DOF receives
    exactly one delay application.

    Args:
        command_rows: Command samples shared by all candidate worlds.
        delay_seconds_rows: Per-world actuator delay values.
        actuator_dt: Explicit actuator update interval.
        explicit_columns: Columns owned by explicit actuators.

    Returns:
        Per-world commands with delay applied only to implicit columns.
    """
    delayed = _delayed_command_rows(command_rows, delay_seconds_rows, actuator_dt)
    if explicit_columns:
        columns = list(explicit_columns)
        delayed[:, :, columns] = command_rows[:, columns].unsqueeze(0)
    return delayed


def _replace_invalid_rollout_worlds(
    positions: torch.Tensor,
    velocities: torch.Tensor,
    torques: torch.Tensor,
    invalid_candidates: np.ndarray,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Replace invalid or non-finite rollout worlds with the cost sentinel.

    Args:
        positions: Simulated joint positions.
        velocities: Simulated joint velocities.
        torques: Simulated joint torques.
        invalid_candidates: Candidate mask produced by parameter validation.

    Returns:
        Position, velocity, and torque tensors with invalid worlds replaced.
    """
    invalid = torch.as_tensor(invalid_candidates, device=positions.device, dtype=torch.bool)
    invalid = invalid | ~torch.isfinite(positions).all(dim=(1, 2))
    invalid = invalid | ~torch.isfinite(velocities).all(dim=(1, 2))
    invalid = invalid | ~torch.isfinite(torques).all(dim=(1, 2))
    invalid = invalid.view(-1, 1, 1)
    return (
        torch.where(invalid, torch.full_like(positions, _INVALID_CANDIDATE_SENTINEL), positions),
        torch.where(invalid, torch.full_like(velocities, _INVALID_CANDIDATE_SENTINEL), velocities),
        torch.where(invalid, torch.full_like(torques, _INVALID_CANDIDATE_SENTINEL), torques),
    )


class NewtonSysIdBridge:
    """Newton actuator bridge that conforms to :class:`SysIdEnvironmentBridge`.

    The bridge keeps the optimizer contract identical to the Isaac Sim bridge.
    Newton is imported only when selected, then per-DOF actuator configs are
    synthesized from USD drive defaults or parsed from Newton actuator prims.

    Args:
        robot_prim_path: Constructor value for ``robot_prim_path``.
        stage_path: Constructor value for ``stage_path``.
        stage: Constructor value for ``stage``.
        newton_config: Constructor value for ``newton_config``.
        joint_baselines: Constructor value for ``joint_baselines``.
        joint_paths: Constructor value for ``joint_paths``.
        link_paths: Constructor value for ``link_paths``.
        physics_dt: Constructor value for ``physics_dt``.
        num_joints: Constructor value for ``num_joints``.
        robot_builder: Constructor value for ``robot_builder``.
        actuator_runtime: Constructor value for ``actuator_runtime``.
        explicit_actuator_policy: Constructor value for ``explicit_actuator_policy``.
        **_kwargs: Constructor value for ``_kwargs``.
    """

    # SolverMuJoCo operates on a replicated model whose world count is keyed by
    # the candidate-row count.  Advertising that capability lets rollout-based
    # optimizers (especially CMA-ES) evaluate a population in one synchronized
    # solver pass instead of serializing identical one-world rollouts.
    supports_arbitrary_candidate_batch = True

    def __init__(
        self,
        *,
        robot_prim_path: str,
        stage_path: str = "",
        stage: Any = None,
        newton_config: NewtonSimulationRunSpec | None = None,
        joint_baselines: list[JointUsdSnapshot | None] | None = None,
        joint_paths: list[str] | None = None,
        link_paths: list[str] | None = None,
        physics_dt: float = 1.0 / 60.0,
        num_joints: int = 0,
        robot_builder: Any = None,
        actuator_runtime: str = ACTUATOR_RUNTIME_MIXED,
        explicit_actuator_policy: str = EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
        **_kwargs: Any,
    ) -> None:
        self.robot_prim_path = robot_prim_path
        self.stage_path = stage_path
        self.stage = stage
        self.newton_config = newton_config or NewtonSimulationRunSpec()
        self.actuator_runtime = str(actuator_runtime).lower()
        if self.actuator_runtime not in {
            ACTUATOR_RUNTIME_IMPLICIT,
            ACTUATOR_RUNTIME_EXPLICIT,
            ACTUATOR_RUNTIME_MIXED,
        }:
            raise SysIdEnvironmentBridgeError(
                f"NewtonSysIdBridge requires a resolved actuator runtime; got {actuator_runtime!r}."
            )
        self.explicit_actuator_policy = str(explicit_actuator_policy).lower()
        if self.explicit_actuator_policy not in {
            EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
            EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
        }:
            raise SysIdEnvironmentBridgeError(
                "NewtonSysIdBridge requires explicit_actuator_policy='authored_only' "
                f"or 'promote_selected'; got {explicit_actuator_policy!r}."
            )
        if str(self.newton_config.solver) != NEWTON_SOLVER_MUJOCO:
            raise SysIdEnvironmentBridgeError(
                f"NewtonSysIdBridge requires simulation.newton.solver='{NEWTON_SOLVER_MUJOCO}', "
                f"got {self.newton_config.solver!r}."
            )
        self._physics_dt = max(float(physics_dt), 1e-9)
        self._trajectory: TrajectoryDataset | None = None
        self._modules = _load_newton_modules()
        self._device = torch.device(self.newton_config.device or "cpu")
        self._joint_baselines = list(joint_baselines or [])
        self._joint_paths = [str(path) for path in (joint_paths or [])]
        self._link_paths = [str(path) for path in (link_paths or [])]
        self._num_joints = int(num_joints or len(self._joint_baselines) or len(self._joint_paths) or 0)
        mode = str(getattr(self.newton_config, "feedforward", "none") or "none").strip().lower()
        if mode not in ("none", "gravity", "inverse_dynamics"):
            raise SysIdEnvironmentBridgeError(
                f"Unknown simulation.newton.feedforward mode '{mode}'; "
                "expected 'none', 'gravity', or 'inverse_dynamics'."
            )
        self._feedforward_mode = mode
        self._mujoco_substeps = int(getattr(self.newton_config, "mujoco_substeps", 1))
        if self._mujoco_substeps < 1:
            raise SysIdEnvironmentBridgeError(
                f"simulation.newton.mujoco_substeps must be at least 1; got {self._mujoco_substeps}."
            )
        self._divergence_warnings: set[tuple[int, int]] = set()
        self._capture_enabled = bool(getattr(self.newton_config, "cuda_graph_capture", True))
        self._ff_cache: dict[int, FeedforwardCacheEntry] = {}
        self._actuator_configs: list[NewtonActuatorConfig] = []
        self._actuator_specs: list[NewtonActuatorSpec] = []
        self._actuator_promotions: list[Any] = []
        self._promoted_target_paths: set[str] = set()
        self._build_actuator_configs()
        self._validate_feedforward_actuator_compatibility()
        self._build_actuators()
        self._robot_builder = robot_builder
        self._mujoco_cache: dict[int, Any] = {}

    @property
    def device(self) -> torch.device:  # noqa: D102
        return self._device

    @property
    def actuator_specs(self) -> list[NewtonActuatorSpec]:  # noqa: D102
        return list(self._actuator_specs)

    @property
    def actuator_configs(self) -> list[NewtonActuatorConfig]:  # noqa: D102
        return list(self._actuator_configs)

    @property
    def usd_parameter_types(self) -> set[SysIdParameterType]:
        """Stateful actuator values are persisted only by this bridge."""
        return {
            SysIdParameterType.JOINT_INTEGRAL_GAIN,
            SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
        }

    @property
    def usd_parameter_dof_claims(self) -> dict[SysIdParameterType, set[int]]:
        """Gain writeback belongs to USD actuator prims only on their target DOFs."""
        actuator_dofs = {
            index for index, cfg in enumerate(self._actuator_configs) if cfg.source == NEWTON_ACTUATOR_SOURCE_USD
        }
        return {
            SysIdParameterType.JOINT_STIFFNESS: actuator_dofs,
            SysIdParameterType.JOINT_DAMPING: actuator_dofs,
        }

    @property
    def actuator_metadata(self) -> dict[str, Any]:  # noqa: D102
        configs = [cfg.to_dict() for cfg in self._actuator_configs]
        unsupported = sorted({name for cfg in self._actuator_configs for name in cfg.unsupported_schemas})
        unclamped = [
            cfg.dof_index
            for cfg in self._actuator_configs
            if cfg.clamp_kind != NEWTON_CLAMP_NONE and not math.isfinite(float(cfg.effort_limit))
        ]
        return {
            "actuator_source": self.newton_config.actuator_source,
            "actuator_runtime": self.actuator_runtime,
            "physics_dt": self._physics_dt,
            "mujoco_substeps": self._mujoco_substeps,
            "explicit_actuator_policy": self.explicit_actuator_policy,
            "temporary_promotions": [str(getattr(item, "prim_path", "")) for item in self._actuator_promotions],
            "controller": self.newton_config.controller,
            "effort_clamp": self.newton_config.effort_clamp,
            "feedforward": self._feedforward_mode,
            "cuda_graph_capture": self._capture_enabled,
            "unsupported_schemas": unsupported,
            "unclamped_dofs": unclamped,
            "actuators": configs,
        }

    def set_trajectory(self, trajectory: TrajectoryDataset) -> None:  # noqa: D102
        # Cheap and re-entrant: the optimizer re-sets the trajectory on every
        # training-chunk switch, every iteration. The feedforward is cached per
        # trajectory (`_ff_cache`), so alternating chunks do not recompute it.
        self._trajectory = trajectory
        if self._num_joints <= 0:
            self._num_joints = int(trajectory.num_joints)
        if len(self._actuator_configs) != self._num_joints:
            self._build_actuator_configs()
        self._validate_feedforward_actuator_compatibility()
        self._build_actuators()

    def validate_parameter_entries(self, entries: list[SysIdParameterEntry]) -> None:  # noqa: D102
        unsupported = sorted(
            {entry.param_type.value for entry in entries if entry.param_type not in _NEWTON_SUPPORTED_PARAMETERS}
        )
        if unsupported:
            raise SysIdEnvironmentBridgeError(
                "Newton MuJoCo SysID supports joint friction/stiffness/damping/integral_gain/armature, "
                "actuator_command_delay_seconds, and link mass/COM/inertia parameters; "
                f"unsupported: {unsupported}"
            )
        if self.actuator_runtime == ACTUATOR_RUNTIME_IMPLICIT:
            stateful = sorted(
                {
                    entry.param_type.value
                    for entry in entries
                    if entry.param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN
                    or is_actuator_delay_parameter(entry.param_type)
                }
            )
            if stateful:
                raise SysIdEnvironmentBridgeError(
                    "Implicit Newton drives cannot execute stateful actuator parameters: " + ", ".join(stateful)
                )

        self._ensure_stateful_actuator_targets(entries)
        self._ensure_config_count(max(self._num_joints, 0))
        if self.newton_config.actuator_source == NEWTON_ACTUATOR_SOURCE_USD:
            missing = [
                index
                for index in self._explicit_columns_for_rollout(entries, self._num_joints)
                if self._actuator_configs[index].source != NEWTON_ACTUATOR_SOURCE_USD
            ]
            if missing:
                raise SysIdEnvironmentBridgeError(
                    "simulation.newton.actuator_source='usd' requires one valid Newton actuator prim "
                    f"for every explicit DOF; missing trajectory DOFs: {missing}."
                )
        for entry in entries:
            dof = int(entry.dof_index)
            indices = range(self._num_joints) if dof < 0 else (dof,)
            for index in indices:
                if not 0 <= int(index) < len(self._actuator_configs):
                    continue
                cfg = self._actuator_configs[int(index)]
                if (
                    entry.param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN
                    and cfg.controller_kind != NEWTON_CONTROLLER_PID
                ):
                    raise SysIdEnvironmentBridgeError(
                        f"joint_integral_gain targets DOF {index}, whose resolved actuator controller is "
                        f"{cfg.controller_kind!r}, not 'pid'."
                    )
                if cfg.controller_kind in _NEWTON_NEURAL_CONTROLLERS and entry.param_type in {
                    SysIdParameterType.JOINT_STIFFNESS,
                    SysIdParameterType.JOINT_DAMPING,
                    SysIdParameterType.JOINT_INTEGRAL_GAIN,
                }:
                    raise SysIdEnvironmentBridgeError(
                        f"{entry.param_type.value} cannot mutate the neural actuator on DOF {index}."
                    )

    async def run_rollout_async(  # noqa: D102
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        commands: torch.Tensor,
        num_steps: int,
    ) -> SysIdRolloutResult:
        if self._trajectory is None:
            raise SysIdEnvironmentBridgeError("Newton bridge trajectory is not configured.")
        self.validate_parameter_entries(param_entries)
        rows = theta_per_env.to(device=self.device, dtype=torch.float32)
        command_rows = commands[:num_steps].to(device=self.device, dtype=torch.float32)
        positions, velocities, torques = self._run_mujoco_rollout(rows, param_entries, command_rows, num_steps)
        return SysIdRolloutResult(positions=positions, velocities=velocities, torques=torques)

    def cleanup(self, remove_clones: bool = True) -> None:
        """Release cached rollout state.

        Args:
            remove_clones: Ignored because Newton rollouts do not create USD clone environments.
        """
        _ = remove_clones
        self._trajectory = None
        for ctx in self._mujoco_cache.values():
            self._release_mujoco_context(ctx)
        self._mujoco_cache = {}
        self._ff_cache = {}
        self._robot_builder = None
        self._discard_actuator_promotions()

    def release_rollout_memory(self) -> None:
        """Release MuJoCo CUDA graphs and keep later validation rollouts plain."""
        self._capture_enabled = False
        for ctx in self._mujoco_cache.values():
            ctx.capture_cache.clear()
            ctx.capture_warm_keys.clear()

    def release_population_context(self, world_count: int) -> None:
        """Release one replicated population model and its captured graphs.

        Population-saturation benchmarks use this to isolate each population
        size instead of retaining every replicated model for the full sweep.
        Normal optimizers keep their fixed-size context cached as before.

        Args:
            world_count: Value supplied for ``world_count``.
        """
        ctx = self._mujoco_cache.pop(int(world_count), None)
        if ctx is None:
            return
        self._release_mujoco_context(ctx)

    def _release_mujoco_context(self, ctx: Any) -> None:
        """Synchronize and drop one context's graph, actuator, and device buffers.

        Args:
            ctx: Cached MuJoCo rollout context to release.
        """
        warp = getattr(getattr(self, "_modules", None), "warp", None)
        model = getattr(ctx, "model", None)
        synchronize = getattr(warp, "synchronize_device", None)
        if callable(synchronize) and model is not None:
            try:
                synchronize(model.device)
            except Exception as exc:
                carb.log_verbose(f"Could not synchronize Newton context before release: {exc}")
        ctx.capture_cache.clear()
        ctx.capture_warm_keys.clear()
        for name in ("runtime_actuators", "runtime_actuator_states", "runtime_next_actuator_states"):
            values = getattr(ctx, name, None)
            if hasattr(values, "clear"):
                values.clear()
        for name in (
            "solver",
            "state0",
            "state1",
            "control",
            "initial_joint_q_stage",
            "initial_joint_qd_stage",
            "target_pos_stage",
            "target_vel_stage",
            "feedforward_stage",
            "sim_q_stage",
            "sim_qd_stage",
            "sim_explicit_effort_stage",
            "diverged_step",
            "actuator_q",
            "actuator_qd",
            "actuator_target_pos",
            "actuator_target_vel",
            "actuator_feedforward",
            "actuator_effort",
            "explicit_compact_indices",
            "explicit_model_dof_indices",
            "model",
        ):
            if hasattr(ctx, name):
                setattr(ctx, name, None)
        gc.collect()

    def _capture_supported(self, ctx) -> bool:  # noqa: ANN001
        """Whether the MuJoCo context can use CUDA graph replay.

        Args:
            ctx: Value supplied for ``ctx``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC106, DOC107
        if not self._capture_enabled or ctx.capture_failed:
            return False
        try:
            device = self._modules.warp.get_device(ctx.model.device)
        except Exception:
            return False
        return bool(getattr(device, "is_cuda", False))

    # ------------------------------------------------------------------
    # Newton physics rollout (SolverMuJoCo over replicated worlds)
    # ------------------------------------------------------------------
    #
    # Trajectory DOFs are mapped to model joints the same way as in the
    # differentiable bridge (``_actuated_joint_maps``): joint paths are matched
    # against Newton joint labels, so the trajectory may cover a SUBSET of the
    # articulation's actuated joints (e.g. arm telemetry on a robot whose USD
    # also has gripper joints); unmatched actuated joints are simulated but held
    # at their default configuration.
    #
    # ``simulation.newton.feedforward`` (none | gravity | inverse_dynamics) is
    # honored the same way as in the differentiable bridge: the nominal-model
    # torque at the measured trajectory is injected per step per world through
    # ``Control.joint_f`` and included in the reported torque channel, and the
    # measured velocities become drive velocity TARGETS (mapped DOFs are built
    # as POSITION_VELOCITY actuators, so damping acts on the velocity error —
    # damping on absolute velocity would cancel the feedforward). A fixed base
    # is required. Model construction and stepping failures are surfaced.

    def _run_mujoco_rollout(self, rows, param_entries, command_rows, num_steps):  # noqa: ANN001, ANN202
        """Run the requested Newton MuJoCo model without simulator substitution.

        Args:
            rows: Value supplied for ``rows``.
            param_entries: Value supplied for ``param_entries``.
            command_rows: Value supplied for ``command_rows``.
            num_steps: Value supplied for ``num_steps``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC106, DOC107
        num_dof = int(command_rows.shape[1])
        world_count = int(rows.shape[0])
        if world_count < 1 or num_dof < 1:
            raise SysIdEnvironmentBridgeError("Newton MuJoCo rollout requires at least one world and one DOF.")
        ctx = self._get_replicated_model(world_count, num_dof)
        resolved_entries = self._remap_entries_for_newton(param_entries, ctx)
        self._ensure_context_actuators(ctx, resolved_entries, num_dof)
        wp = self._modules.warp
        # One synchronized transfer for the candidate batch replaces the prior
        # per-parameter ``Tensor.item()`` synchronizations in each world.
        rows_host = rows.detach().cpu().numpy().astype(np.float32, copy=False)
        dt = float(self._rollout_dt(num_steps))
        delay_seconds_rows = np.stack(
            [actuator_command_delay_seconds_vector(num_dof, param_entries, row) for row in rows_host],
            axis=0,
        )
        ke_tensor, kd_tensor, friction_tensor, invalid_candidates = self._apply_candidate_params(
            ctx, rows_host, resolved_entries, num_dof, world_count, dt
        )
        _base_k, _base_d, _base_ki, _base_fr, _base_arm, const_effort = self._baseline_drive_vectors(num_dof)
        commands_np = command_rows.detach().cpu().numpy().astype(np.float32, copy=False)
        self._ensure_feedforward(ctx, num_steps, commands_np)

        q0, dq0 = self._initial_state(num_dof)
        init_q = ctx.default_joint_q.copy()
        init_q[ctx.coord_map_np] = np.tile(q0.detach().cpu().numpy().astype(np.float32), world_count)
        init_qd = ctx.default_joint_qd.copy()
        init_qd[ctx.dof_map_np] = np.tile(dq0.detach().cpu().numpy().astype(np.float32), world_count)
        self._assign_wp(ctx.initial_joint_q_stage, init_q)
        self._assign_wp(ctx.initial_joint_qd_stage, init_qd)

        delayed_commands = _commands_for_actuator_owners(
            command_rows,
            delay_seconds_rows,
            dt,
            tuple(ctx.explicit_columns),
        )
        self._stage_mujoco_controls(ctx, delayed_commands, num_steps)
        mapped_width = world_count * num_dof
        # Keep the live staging views on the Warp model's native device.  The
        # bridge may expose CPU tensors while Newton selected CUDA internally;
        # moving these views early would snapshot only the initial row.
        sim_q_stage = wp.to_torch(ctx.sim_q_stage).to(dtype=torch.float32)
        sim_qd_stage = wp.to_torch(ctx.sim_qd_stage).to(dtype=torch.float32)
        sim_explicit_effort_stage = None
        if ctx.sim_explicit_effort_stage is not None:
            sim_explicit_effort_stage = wp.to_torch(ctx.sim_explicit_effort_stage).to(dtype=torch.float32)
        rollout_device = sim_q_stage.device
        delayed_rollout = delayed_commands.to(device=rollout_device, dtype=torch.float32)
        ke_rollout = ke_tensor.to(device=rollout_device, dtype=torch.float32)
        kd_rollout = kd_tensor.to(device=rollout_device, dtype=torch.float32)
        friction_rollout = friction_tensor.to(device=rollout_device, dtype=torch.float32)
        const_effort_rollout = const_effort.to(device=rollout_device, dtype=torch.float32)
        # Feedforward torque + velocity targets (theta-independent, world-0 data
        # tiled per world). Damping acts on the velocity ERROR: the mapped DOFs
        # were flipped to POSITION_VELOCITY actuators at solver build, so kd
        # tracks joint_target_qd instead of fighting the feedforward.
        ff_active = bool(ctx.ff_active and ctx.ff_np is not None and ctx.velocity_targets_np is not None)
        ff_torch = vel_target_torch = None
        if ff_active:
            ff_torch = torch.as_tensor(ctx.ff_np[:num_steps], device=rollout_device, dtype=torch.float32)
            vel_target_torch = torch.as_tensor(
                ctx.velocity_targets_np[:num_steps],
                device=rollout_device,
                dtype=torch.float32,
            )
        parameter_layout = tuple(
            (
                entry.param_type.value,
                int(getattr(entry, "dof_index", -1)),
                int(getattr(entry, "link_index", -1)),
            )
            for entry in param_entries
        )
        capture_key = (
            int(num_steps),
            float(dt),
            int(self._mujoco_substeps),
            parameter_layout,
            self._feedforward_mode,
            id(ctx.model),
            id(ctx.solver),
            id(ctx.state0),
            id(ctx.state1),
            id(ctx.control),
            id(ctx.initial_joint_q_stage),
            id(ctx.initial_joint_qd_stage),
            id(ctx.target_pos_stage),
            id(ctx.target_vel_stage),
            id(ctx.feedforward_stage),
            id(ctx.sim_q_stage),
            id(ctx.sim_qd_stage),
            id(ctx.sim_explicit_effort_stage),
            id(ctx.diverged_step),
            tuple(ctx.explicit_columns),
            *(id(actuator) for actuator in ctx.runtime_actuators),
            id(ctx.model.body_mass),
            id(ctx.model.body_com),
            id(ctx.model.body_inertia),
        )
        use_capture = num_steps >= 2 and self._capture_supported(ctx)
        record = ctx.capture_cache.get(capture_key) if use_capture else None
        if record is not None:
            wp.capture_launch(record.graph)
            if not record.replay_logged:
                record.replay_logged = True
                carb.log_info(
                    f"SysId: Newton MuJoCo CUDA graph replay active (steps={num_steps}, worlds={world_count})."
                )
        elif use_capture and capture_key in ctx.capture_warm_keys:
            record = _capture_mujoco_rollout_graph(self, ctx, num_steps, dt, capture_key)
            if record is None:
                _record_mujoco_rollout_launches(self, ctx, num_steps, dt)
        else:
            _record_mujoco_rollout_launches(self, ctx, num_steps, dt)
            if use_capture:
                ctx.capture_warm_keys.add(capture_key)

        positions = sim_q_stage[:num_steps, :mapped_width].reshape(num_steps, world_count, num_dof)
        velocities = sim_qd_stage[:num_steps, :mapped_width].reshape(num_steps, world_count, num_dof)
        # Reconstruct the reported torque channel in one vectorized pass after
        # physics. This keeps Torch launches out of CUDA capture while retaining
        # the exact pre-step PD/feedforward/friction/clamping convention.
        targets = delayed_rollout.permute(1, 0, 2)
        velocity_error = velocities
        if ff_active:
            velocity_error = velocity_error - vel_target_torch.unsqueeze(1)
        actuator_effort = (
            ke_rollout.unsqueeze(0) * (targets - positions)
            - kd_rollout.unsqueeze(0) * velocity_error
            + const_effort_rollout.view(1, 1, num_dof)
        )
        if ff_active:
            actuator_effort = actuator_effort + ff_torch.unsqueeze(1)
        friction_rows = friction_rollout.unsqueeze(0).expand(num_steps, -1, -1)
        torques = _reported_net_joint_effort(
            actuator_effort.reshape(-1, num_dof),
            velocities.reshape(-1, num_dof),
            friction_rows.reshape(-1, num_dof),
            self._actuator_configs,
        ).reshape(num_steps, world_count, num_dof)
        if sim_explicit_effort_stage is not None and num_steps > 1:
            explicit_columns = list(ctx.explicit_columns)
            actual_effort = sim_explicit_effort_stage[:num_steps, : world_count * len(explicit_columns)].reshape(
                num_steps, world_count, len(explicit_columns)
            )
            explicit_velocity = velocities[:, :, explicit_columns]
            explicit_friction = friction_rows[:, :, explicit_columns]
            # Actuator.step() has already applied delay, PID state, and its
            # configured clamp. Native joint friction remains a model force,
            # so subtract it once to preserve the bridge's net-effort channel.
            torques[:, :, explicit_columns] = actual_effort - explicit_friction * torch.tanh(explicit_velocity * 100.0)
        # The context buffers are reused on the next rollout; return owning
        # tensors with the bridge's public (world, time, dof) layout.
        positions = positions.permute(1, 0, 2).contiguous().clone().to(self.device)
        velocities = velocities.permute(1, 0, 2).contiguous().clone().to(self.device)
        torques = torques.permute(1, 0, 2).contiguous().to(self.device)
        if ctx.diverged_step is not None:
            diverged_step = np.asarray(ctx.diverged_step.numpy(), dtype=np.int64)[:world_count]
            if np.any(diverged_step >= 0):
                self._report_mujoco_divergence(
                    ctx,
                    diverged_step,
                    ke_tensor.detach().cpu().numpy(),
                    num_steps=num_steps,
                    dt=dt,
                )
                invalid_candidates = np.asarray(invalid_candidates, dtype=bool) | (diverged_step >= 0)
        return _replace_invalid_rollout_worlds(positions, velocities, torques, invalid_candidates)

    def _report_mujoco_divergence(
        self,
        ctx: Any,
        diverged_step: np.ndarray,
        ke_rows: np.ndarray,
        *,
        num_steps: int,
        dt: float,
    ) -> None:
        """Explain a diverged MuJoCo rollout, failing when no world stayed stable.

        Diverged worlds are scored as invalid candidates so an optimizer can
        still rank the rest of its population. When every world diverged there
        is nothing to rank, and a sentinel cost would read as a legitimately poor
        fit, so the rollout fails with the stability estimate instead.

        Args:
            ctx: MuJoCo rollout context that produced the rollout.
            diverged_step: First diverged step per world, or -1 for stable worlds.
            ke_rows: Candidate drive stiffness per world and trajectory DOF.
            num_steps: Rollout length in telemetry samples.
            dt: Telemetry sample interval of the rollout.

        Raises:
            SysIdEnvironmentBridgeError: If every world in the rollout diverged.
        """
        diverged = diverged_step >= 0
        world_count = int(diverged.shape[0])
        diverged_count = int(np.count_nonzero(diverged))
        first_step = int(diverged_step[diverged].min())
        message = (
            f"SysId: Newton MuJoCo rollout diverged in {diverged_count}/{world_count} world(s); the first "
            f"non-finite or runaway (>{_DIVERGED_RATE_LIMIT:.0e} per second) joint rate appeared at step "
            f"{first_step} of {num_steps} "
            f"(dt={dt * 1e3:.3g} ms, mujoco_substeps={self._mujoco_substeps})."
        )
        estimate = self._drive_stability_estimate(ctx, ke_rows[diverged], dt)
        if estimate is not None:
            message += " " + estimate
        message += (
            " Check that the asset's drive gains, link inertias, and joint armature are physically plausible "
            "(USD angular drive stiffness is per degree, so a value meant per radian is 57.3x too stiff)."
        )
        if diverged_count == world_count:
            raise SysIdEnvironmentBridgeError(
                message + " No candidate produced a stable rollout, so there is no cost to report."
            )
        warned = getattr(self, "_divergence_warnings", None)
        if warned is None:
            warned = self._divergence_warnings = set()
        key = (world_count, int(num_steps))
        if key not in warned:
            warned.add(key)
            carb.log_warn(message + " The diverged candidates were scored as invalid.")

    def _drive_stability_estimate(self, ctx: Any, ke_rows: np.ndarray, dt: float) -> str:
        """Describe the stiffest mapped drive against MuJoCo's explicit-stiffness step bound.

        MuJoCo integrates drive damping implicitly but drive stiffness
        explicitly, so a drive of stiffness ``k`` on inertia ``I`` needs a step
        ``h`` with ``h * sqrt(k / I)`` well below 2. The inertia is the child
        link about the joint axis plus armature, a lower bound on the
        articulated inertia, so the suggested substep count is conservative.

        Args:
            ctx: MuJoCo rollout context with the nominal model.
            ke_rows: Drive stiffness per diverged world and trajectory DOF.
            dt: Telemetry sample interval of the rollout.

        Returns:
            Diagnostic sentence, or an empty string when no estimate is available.
        """
        try:
            model = ctx.model
            joint_type = np.asarray(model.joint_type.numpy())
            joint_child = np.asarray(model.joint_child.numpy())
            joint_qd_start = np.asarray(model.joint_qd_start.numpy())
            joint_x_c = np.asarray(model.joint_X_c.numpy(), dtype=np.float64).reshape(-1, 7)
            joint_axis = np.asarray(model.joint_axis.numpy(), dtype=np.float64).reshape(-1, 3)
            prismatic = int(self._modules.newton.JointType.PRISMATIC)
            # Candidate armature is written to the model per world; the smallest
            # value over the batch keeps the suggested substep count conservative.
            armature = (
                np.asarray(model.joint_armature.numpy(), dtype=np.float64)
                .reshape(int(ctx.world_count), int(ctx.dofs_per_world))
                .min(axis=0)
            )
        except Exception as exc:  # noqa: BLE001 - diagnostics must not mask the divergence
            carb.log_verbose(f"SysId: MuJoCo stability estimate unavailable: {exc}")
            return ""
        explicit_columns = set(getattr(ctx, "explicit_columns", ()))
        worst: tuple[float, int, float, float, float] | None = None
        for column in range(min(int(ctx.num_dof), int(ke_rows.shape[1]))):
            stiffness = float(np.max(ke_rows[:, column]))
            if not math.isfinite(stiffness) or stiffness <= 0.0:
                continue
            dof = int(ctx.dof_map_np[column])
            joint = int(np.searchsorted(joint_qd_start, dof, side="right") - 1)
            body = int(joint_child[joint])
            if not 0 <= body < int(ctx.bodies_per_world):
                continue
            mass = float(ctx.baseline_body_mass_world0[body])
            if int(joint_type[joint]) == prismatic:
                inertia = mass
            else:
                rotation = joint_x_c[joint, 3:]
                axis = joint_axis[dof]
                # Rotate the joint-frame axis into the child body frame (xyzw quaternion).
                cross = 2.0 * np.cross(rotation[:3], axis)
                axis = axis + rotation[3] * cross + np.cross(rotation[:3], cross)
                axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
                lever = np.asarray(ctx.baseline_body_com_world0[body], dtype=np.float64) - joint_x_c[joint, :3]
                body_inertia = np.asarray(ctx.baseline_body_inertia_world0[body], dtype=np.float64)
                inertia = float(axis @ body_inertia @ axis) + mass * float(lever @ lever - float(lever @ axis) ** 2)
            inertia += max(float(armature[dof]), 0.0)
            if inertia <= 0.0:
                continue
            omega = math.sqrt(stiffness / inertia)
            step = dt if column in explicit_columns else dt / self._mujoco_substeps
            ratio = omega * step
            if worst is None or ratio > worst[0]:
                worst = (ratio, column, stiffness, inertia, omega)
        if worst is None:
            return ""
        ratio, column, stiffness, inertia, omega = worst
        name = self._joint_paths[column] if column < len(self._joint_paths) else f"DOF {column}"
        text = (
            f"Stiffest drive: trajectory DOF {column} ({name}) with stiffness {stiffness:.4g} on an estimated "
            f"inertia of {inertia:.3g} gives omega*h = {ratio:.2f}; explicit stiffness needs it well below 2."
        )
        if column in explicit_columns:
            return text + (
                " This DOF uses an explicit actuator that updates once per telemetry sample, so substeps do not "
                "stabilize it; lower its gains, raise its armature, or log telemetry at a higher rate."
            )
        recommended = max(self._mujoco_substeps + 1, int(math.ceil(omega * dt)))
        if recommended > _PRACTICAL_MUJOCO_SUBSTEPS:
            return text + (
                f" Stabilizing it would take about {recommended} simulation.newton.mujoco_substeps, so the asset "
                "itself needs correcting."
            )
        return text + f" Set simulation.newton.mujoco_substeps to at least {recommended}."

    def _ensure_feedforward(self, ctx, num_steps: int, commands_np: np.ndarray) -> None:  # noqa: ANN001
        """Load the per-step controller feedforward, computing and caching per trajectory.

        Mirrors the differentiable bridge: the feedforward is the NOMINAL model's
        torque at the measured trajectory (theta-independent — the real
        controller's internal model is fixed), applied through ``Control.joint_f``
        (MuJoCo ``qfrc_applied``), with the measured velocities as drive velocity
        targets so damping acts on the velocity error.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.
        """  # noqa: DOC107
        if self._feedforward_mode == "none":
            ctx.ff_active = False
            return
        self._validate_feedforward_actuator_compatibility()
        ctx.ff_active = False
        ctx.ff_np = None
        ctx.velocity_targets_np = None
        entry = self._feedforward_cache_entry(ctx, num_steps, commands_np)
        ctx.ff_np = entry.feedforward
        ctx.velocity_targets_np = entry.velocity_targets
        ctx.ff_active = True

    def _validate_feedforward_actuator_compatibility(self) -> None:
        """Reject feedforward whenever any authored actuator owns neural torque."""
        if self._feedforward_mode == "none":
            return
        neural_dofs = [
            int(config.dof_index)
            for config in self._actuator_configs
            if config.controller_kind in _NEWTON_NEURAL_CONTROLLERS
        ]
        if neural_dofs:
            raise SysIdEnvironmentBridgeError(
                f"simulation.newton.feedforward='{self._feedforward_mode}' cannot be combined with Newton "
                "neural actuator controllers. Neural controllers compute their own full torque signal; "
                "the nominal-model Newton-Euler feedforward would stack additively on top and corrupt "
                "the identified dynamics. Use simulation.newton.feedforward='none' for robots with "
                f"neural-controller DOFs (found {neural_dofs})."
            )

    def _feedforward_cache_entry(self, ctx: Any, num_steps: int, commands_np: np.ndarray) -> FeedforwardCacheEntry:
        """Return the cached feedforward for the active trajectory, computing on miss.

        The optimizer alternates ``set_trajectory`` over the training chunks every
        iteration; identity-plus-content keying makes that alternation a cache hit
        instead of a full nominal-model Newton-Euler recomputation per rollout.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        trajectory = self._trajectory
        key = id(trajectory)
        content_hash = feedforward_content_hash(trajectory)
        entry = self._ff_cache.get(key)
        if entry is not None and entry.content_hash == content_hash and entry.steps >= num_steps:
            self._ff_cache.pop(key)  # refresh LRU order
            self._ff_cache[key] = entry
            return entry
        entry = self._compute_feedforward_entry(ctx, num_steps, commands_np, key, content_hash)
        self._ff_cache.pop(key, None)
        while len(self._ff_cache) >= FF_CACHE_MAX_ENTRIES:
            self._ff_cache.pop(next(iter(self._ff_cache)))
        self._ff_cache[key] = entry
        return entry

    def _compute_feedforward_entry(
        self,
        ctx,  # noqa: ANN001
        num_steps: int,
        commands_np: np.ndarray,
        key: int,
        content_hash: int,
    ) -> FeedforwardCacheEntry:
        """Compute the feedforward for the active trajectory and mode.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.
            key: Value supplied for ``key``.
            content_hash: Value supplied for ``content_hash``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        # Feedforward modes model a fixed-base controller; a floating base has no
        # base-wrench telemetry to compensate against.
        newton = self._modules.newton
        joint_type = ctx.model.joint_type.numpy()[: ctx.joints_per_world]
        if any(int(jt) in (int(newton.JointType.FREE), int(newton.JointType.DISTANCE)) for jt in joint_type):
            error = SysIdEnvironmentBridgeError(
                f"simulation.newton.feedforward='{self._feedforward_mode}' requires a fixed-base robot; "
                "this articulation has a FREE/DISTANCE joint. Use feedforward='none' for floating-base robots."
            )
            error.feedforward_configuration = True
            raise error
        # Evaluate the feedforward at the measured trajectory — that is what the
        # real controller compensated at; fall back to the commanded targets when
        # telemetry positions are unavailable.
        positions = None
        trajectory = self._trajectory
        if trajectory is not None and int(trajectory.positions.shape[0]) >= num_steps:
            positions = np.asarray(trajectory.positions[:num_steps, : ctx.num_dof], dtype=np.float64)
        if positions is None:
            positions = np.asarray(commands_np[:num_steps, : ctx.num_dof], dtype=np.float64)
        velocities = None
        if trajectory is not None and int(trajectory.velocities.shape[0]) >= num_steps:
            velocities = np.asarray(trajectory.velocities[:num_steps, : ctx.num_dof], dtype=np.float64)
        if velocities is None:
            velocities = np.zeros_like(positions)
        expected_shape = (num_steps, int(ctx.num_dof))
        for name, values in (("positions", positions), ("velocity targets", velocities)):
            if values.shape != expected_shape or not np.isfinite(values).all():
                error = SysIdEnvironmentBridgeError(
                    f"Configured Newton '{self._feedforward_mode}' feedforward requires finite {name} "
                    f"with shape {expected_shape}; got {values.shape}."
                )
                error.feedforward_configuration = True
                raise error
        rates_for_ff = None
        accelerations = None
        if self._feedforward_mode == "inverse_dynamics":
            rates_for_ff = velocities
            times = None
            if trajectory is not None and int(np.asarray(trajectory.times).shape[0]) >= num_steps:
                times = np.asarray(trajectory.times[:num_steps], dtype=np.float64).reshape(-1)
            accelerations = filtered_acceleration(velocities, times)
        try:
            feedforward = compute_feedforward(
                newton,
                ctx.model,
                world_count=ctx.world_count,
                coords_per_world=ctx.coords_per_world,
                dofs_per_world=ctx.dofs_per_world,
                coord_map_world0=ctx.coord_map_np,
                dof_map_world0=ctx.dof_map_np,
                default_joint_q_world0=ctx.default_joint_q,
                baseline_mass=ctx.baseline_body_mass,
                baseline_com=ctx.baseline_body_com,
                baseline_inertia=ctx.baseline_body_inertia,
                q_steps=positions,
                qd_steps=rates_for_ff,
                qdd_steps=accelerations,
            )
        except SysIdEnvironmentBridgeError:
            raise
        except Exception as exc:
            error = SysIdEnvironmentBridgeError(
                f"Failed to compute configured Newton '{self._feedforward_mode}' feedforward: {exc}"
            )
            error.feedforward_configuration = True
            raise error from exc
        feedforward = np.asarray(feedforward, dtype=np.float64)
        if feedforward.shape != expected_shape:
            error = SysIdEnvironmentBridgeError(
                f"Configured Newton '{self._feedforward_mode}' feedforward returned shape "
                f"{feedforward.shape}; expected {expected_shape}."
            )
            error.feedforward_configuration = True
            raise error
        if not np.isfinite(feedforward).all() or not np.isfinite(velocities).all():
            error = SysIdEnvironmentBridgeError(
                f"Configured Newton '{self._feedforward_mode}' feedforward produced non-finite controller values."
            )
            error.feedforward_configuration = True
            raise error
        carb.log_warn(
            f"SysId: Newton MuJoCo bridge '{self._feedforward_mode}' feedforward active over "
            f"{num_steps} steps (|ff| max {float(np.abs(feedforward).max()):.2f} Nm)."
        )
        return FeedforwardCacheEntry(
            key=key,
            trajectory=trajectory,
            content_hash=content_hash,
            steps=num_steps,
            feedforward=feedforward.astype(np.float32),
            velocity_targets=velocities.astype(np.float32),
        )

    def _get_robot_builder(self):  # noqa: ANN202
        """Build (once) a single-world ModelBuilder for the robot articulation.

        Returns:
            Result produced by the operation.
        """
        if self._robot_builder is not None:
            return self._robot_builder
        builder_cls = getattr(self._modules.newton, "ModelBuilder", None)
        if builder_cls is None:
            return None
        source = self.stage if self.stage is not None else self.stage_path
        if not source:
            return None
        builder = builder_cls()
        add_usd = getattr(builder, "add_usd", None)
        if not callable(add_usd):
            return None
        add_usd(source, root_path=self.robot_prim_path or "/")
        self._robot_builder = builder
        return builder

    def _get_replicated_model(self, world_count: int, num_dof: int):  # noqa: ANN202
        """Replicate the robot into ``world_count`` worlds and build a MuJoCo solver (cached).

        Args:
            world_count: Value supplied for ``world_count``.
            num_dof: Value supplied for ``num_dof``.

        Returns:
            Result produced by the operation.
        """
        cached = self._mujoco_cache.get(world_count)
        if cached is not None:
            if int(cached.num_dof) != num_dof:
                raise SysIdEnvironmentBridgeError(
                    f"Trajectory has {num_dof} DOFs but the cached Newton model was mapped for {cached.num_dof}."
                )
            return cached
        # Structural failures raise (not return None) so the caller's handler
        # latches the genuine path off once, instead of rebuilding/re-warning
        # every rollout.
        robot = self._get_robot_builder()
        if robot is None:
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder.add_usd is unavailable for this robot.")
        builder_cls = getattr(self._modules.newton, "ModelBuilder", None)
        scene = builder_cls()
        replicate = getattr(scene, "replicate", None)
        if not callable(replicate):
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder.replicate is unavailable.")
        replicate(robot, world_count=world_count)
        model = scene.finalize()

        joint_count = int(model.joint_count)
        if joint_count < 1 or joint_count % world_count != 0:
            raise SysIdEnvironmentBridgeError(
                f"Newton model joint count {joint_count} is not divisible by world count {world_count}."
            )
        joints_per_world = joint_count // world_count
        body_count = int(model.body_count)
        if body_count < 1 or body_count % world_count != 0:
            raise SysIdEnvironmentBridgeError(
                f"Newton model body count {body_count} is not divisible by world count {world_count}."
            )
        bodies_per_world = body_count // world_count
        coords_per_world = int(model.joint_coord_count) // world_count
        dofs_per_world = int(model.joint_dof_count) // world_count
        coord0, dof0, target_pos0, actuated_world0 = self._actuated_joint_maps(model, joints_per_world, num_dof)
        world_index = np.repeat(np.arange(world_count, dtype=np.int64), num_dof)
        coord_map_np = (np.tile(coord0, world_count) + world_index * coords_per_world).astype(np.int64)
        dof_map_np = (np.tile(dof0, world_count) + world_index * dofs_per_world).astype(np.int64)
        target_pos_count = int(model.joint_target_q_start.numpy()[-1])
        if target_pos_count < 1 or target_pos_count % world_count != 0:
            raise SysIdEnvironmentBridgeError(
                f"Newton model position-target count {target_pos_count} is not divisible by world count {world_count}."
            )
        targets_per_world = target_pos_count // world_count
        target_pos_map_np = (np.tile(target_pos0, world_count) + world_index * targets_per_world).astype(np.int64)

        if self._feedforward_mode != "none":
            # Feedforward modes model a trajectory-tracking controller whose
            # damping acts on the velocity ERROR. MuJoCo actuators are built from
            # `joint_target_mode` at solver construction: POSITION folds kd into
            # the position actuator as damping on ABSOLUTE velocity (which would
            # fight and largely cancel the feedforward), while POSITION_VELOCITY
            # adds a velocity actuator tracking `joint_target_qd`. Flip the
            # mapped DOFs before building the solver; held joints keep their
            # imported mode. With zero velocity targets the two modes coincide,
            # so a later feedforward fallback needs no rebuild.
            target_mode = getattr(model, "joint_target_mode", None)
            if target_mode is None:
                raise SysIdEnvironmentBridgeError(
                    "Newton model.joint_target_mode is unavailable; feedforward modes need it "
                    "to build velocity-error damping actuators."
                )
            mode_np = target_mode.numpy().copy()
            mode_np[dof_map_np] = int(self._modules.newton.JointTargetMode.POSITION_VELOCITY)
            target_mode.assign(mode_np)

        solvers = importlib.import_module("newton.solvers")
        solver_cls = getattr(solvers, "SolverMuJoCo", None)
        if solver_cls is None:
            raise SysIdEnvironmentBridgeError("newton.solvers.SolverMuJoCo is unavailable.")
        solver = solver_cls(model)
        wp = self._modules.warp
        control = model.control()
        if control.joint_target_q is None:
            control.joint_target_q = wp.zeros(target_pos_count, dtype=wp.float32, device=model.device)
        if control.joint_target_qd is None:
            control.joint_target_qd = wp.zeros(int(model.joint_dof_count), dtype=wp.float32, device=model.device)

        # Hold-position targets: every actuated 1-DOF joint defaults to its import
        # configuration; the per-step command scatter overwrites the mapped subset.
        # Held joints also keep their imported drive gains. MuJoCo integrates
        # drive damping implicitly but drive stiffness explicitly, so a stiff
        # drive on a light link can still diverge at the telemetry step; the
        # rollout detects that per world and `mujoco_substeps` shortens the step.
        default_joint_q = model.joint_q.numpy().astype(np.float32).copy()
        default_joint_qd = model.joint_qd.numpy().astype(np.float32).copy()
        if model.joint_target_q is not None:
            default_joint_target = model.joint_target_q.numpy().astype(np.float32).copy()
        else:
            default_joint_target = np.zeros(target_pos_count, dtype=np.float32)
        for coord, _dof, target_pos in actuated_world0:
            for world in range(world_count):
                default_joint_target[target_pos + world * targets_per_world] = default_joint_q[
                    coord + world * coords_per_world
                ]

        def _default_array(name: str):  # noqa: ANN202
            arr = getattr(model, name, None)
            return arr.numpy().astype(np.float32).copy() if arr is not None else None

        baseline_body_mass = _default_array("body_mass")
        baseline_body_com = _default_array("body_com")
        baseline_body_inertia = _default_array("body_inertia")
        if baseline_body_mass is None or baseline_body_com is None or baseline_body_inertia is None:
            raise SysIdEnvironmentBridgeError("Newton model does not expose body mass, COM, and inertia arrays.")
        mass_world0 = baseline_body_mass[:bodies_per_world].copy()
        com_world0 = baseline_body_com[:bodies_per_world].reshape(bodies_per_world, 3).copy()
        inertia_world0 = baseline_body_inertia[:bodies_per_world].reshape(bodies_per_world, 3, 3).copy()
        inertia_lc_world0: dict[int, np.ndarray] = {}
        for body in range(bodies_per_world):
            try:
                inertia_lc_world0[body] = inertia_matrix_to_log_cholesky(inertia_world0[body])
            except ValueError:
                # Fixed or massless imported bodies may have a degenerate
                # inertia. They remain usable unless explicitly selected.
                continue

        link_to_body: dict[int, int] = {}
        labels = [str(label) for label in getattr(model, "body_label", [])][:bodies_per_world]
        for link_index, path in enumerate(self._link_paths):
            for body, label in enumerate(labels):
                if label == path or label.endswith(path) or path.endswith(label.rsplit("/", 1)[-1]):
                    link_to_body[link_index] = body
                    break

        ctx = SimpleNamespace(
            model=model,
            solver=solver,
            state0=model.state(),
            state1=model.state(),
            control=control,
            notify_flags=getattr(self._modules.newton, "ModelFlags", None),
            num_dof=num_dof,
            coords_per_world=coords_per_world,
            dofs_per_world=dofs_per_world,
            targets_per_world=targets_per_world,
            joints_per_world=joints_per_world,
            bodies_per_world=bodies_per_world,
            world_count=world_count,
            coord_map_np=coord_map_np,
            dof_map_np=dof_map_np,
            target_pos_map_np=target_pos_map_np,
            coord_map_wp=wp.array(coord_map_np.astype(np.int32), dtype=wp.int32, device=model.device),
            dof_map_wp=wp.array(dof_map_np.astype(np.int32), dtype=wp.int32, device=model.device),
            target_pos_map_wp=wp.array(target_pos_map_np.astype(np.int32), dtype=wp.int32, device=model.device),
            default_joint_q=default_joint_q,
            default_joint_qd=default_joint_qd,
            initial_joint_q_stage=wp.array(default_joint_q, dtype=wp.float32, device=model.device),
            initial_joint_qd_stage=wp.array(default_joint_qd, dtype=wp.float32, device=model.device),
            default_joint_target=default_joint_target,
            default_target_ke=_default_array("joint_target_ke"),
            default_target_kd=_default_array("joint_target_kd"),
            default_armature=_default_array("joint_armature"),
            default_friction=_default_array("joint_friction"),
            # Candidate plant inertia is restored from these immutable
            # baselines on every rollout. Controller feedforward intentionally
            # continues to use the same nominal arrays rather than theta.
            baseline_body_mass=baseline_body_mass,
            baseline_body_com=baseline_body_com,
            baseline_body_inertia=baseline_body_inertia,
            baseline_body_mass_world0=mass_world0,
            baseline_body_com_world0=com_world0,
            baseline_body_inertia_world0=inertia_world0,
            baseline_body_inertia_lc_world0=inertia_lc_world0,
            link_to_body=link_to_body,
            # Loaded from the bridge's per-trajectory feedforward cache
            # (`_ff_cache`) on every rollout; cheap references, no recompute.
            ff_np=None,
            velocity_targets_np=None,
            ff_active=False,
            control_ff_dirty=False,
            io_capacity=0,
            target_pos_stage=None,
            target_vel_stage=None,
            feedforward_stage=None,
            sim_q_stage=None,
            sim_qd_stage=None,
            sim_explicit_effort_stage=None,
            diverged_step=wp.full(world_count, -1, dtype=wp.int32, device=model.device),
            capture_cache={},
            capture_warm_keys=set(),
            capture_failed=False,
            explicit_columns=(),
            runtime_actuators=[],
            runtime_actuator_states=[],
            runtime_next_actuator_states=[],
            actuator_q=None,
            actuator_qd=None,
            actuator_target_pos=None,
            actuator_target_vel=None,
            actuator_feedforward=None,
            actuator_effort=None,
            explicit_compact_indices=None,
            explicit_model_dof_indices=None,
        )
        self._mujoco_cache[world_count] = ctx
        return ctx

    def _explicit_columns_for_rollout(self, param_entries, num_dof: int) -> tuple[int, ...]:  # noqa: ANN001
        """Select only DOFs that need application-side actuator behavior.

        Args:
            param_entries: Value supplied for ``param_entries``.
            num_dof: Value supplied for ``num_dof``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        authored = {
            index
            for index, cfg in enumerate(self._actuator_configs[:num_dof])
            if cfg.source == NEWTON_ACTUATOR_SOURCE_USD
            or cfg.controller_kind != NEWTON_CONTROLLER_PD
            or cfg.delay_steps > 0
        }
        plan = resolve_actuator_dof_plan(
            self.actuator_runtime,
            num_dof,
            authored_explicit_dofs=authored,
            parameter_entries=param_entries,
        )
        return tuple(item.dof_index for item in plan if item.explicit)

    def _ensure_context_actuators(self, ctx, param_entries, num_dof: int) -> None:  # noqa: ANN001
        """Build lower-level Newton actuators over compact mapped state arrays.

        Args:
            ctx: Value supplied for ``ctx``.
            param_entries: Value supplied for ``param_entries``.
            num_dof: Value supplied for ``num_dof``.
        """  # noqa: DOC107
        columns = self._explicit_columns_for_rollout(param_entries, num_dof)
        if tuple(ctx.explicit_columns) == columns:
            return
        ctx.capture_cache.clear()
        ctx.capture_warm_keys.clear()
        ctx.io_capacity = 0
        ctx.explicit_columns = columns
        ctx.runtime_actuators = []
        ctx.runtime_actuator_states = []
        ctx.runtime_next_actuator_states = []
        wp = self._modules.warp
        mapped_width = int(ctx.world_count * num_dof)
        device = ctx.model.device
        for name in (
            "actuator_q",
            "actuator_qd",
            "actuator_target_pos",
            "actuator_target_vel",
            "actuator_feedforward",
            "actuator_effort",
        ):
            setattr(ctx, name, wp.zeros(mapped_width, dtype=wp.float32, device=device))
        compact_indices = [world * num_dof + column for world in range(ctx.world_count) for column in columns]
        model_indices = [int(ctx.dof_map_np[index]) for index in compact_indices]
        ctx.explicit_compact_indices = wp.array(compact_indices, dtype=wp.int32, device=device)
        ctx.explicit_model_dof_indices = wp.array(model_indices, dtype=wp.int32, device=device)
        if not columns:
            return

        actuator_cls = getattr(self._modules.actuators, "Actuator", None)
        if actuator_cls is None:
            raise SysIdEnvironmentBridgeError("newton.actuators.Actuator is unavailable.")
        for column in columns:
            cfg = self._actuator_configs[column]

            def values(value: float) -> Any:
                return wp.array([float(value)] * ctx.world_count, dtype=wp.float32, device=device)

            if cfg.controller_kind in _NEWTON_NEURAL_CONTROLLERS:
                controller = self._construct_controller(self._modules.actuators, cfg, delay=None, clamping=None)
            elif cfg.controller_kind == NEWTON_CONTROLLER_PID:
                cls = _resolve_actuator_class(
                    self._modules.actuators,
                    ("DrivePID", "ControllerPID", "PIDController", "ControlPID"),
                )
                controller = _construct_optional(
                    cls,
                    kp=values(cfg.stiffness),
                    kd=values(cfg.damping),
                    ki=values(cfg.integral_gain),
                    integral_max=values(cfg.integral_max),
                    const_effort=values(cfg.const_effort),
                )
            else:
                cls = _resolve_actuator_class(
                    self._modules.actuators,
                    ("DrivePD", "ControllerPD", "PDController", "ControlPD"),
                )
                controller = _construct_optional(
                    cls,
                    kp=values(cfg.stiffness),
                    kd=values(cfg.damping),
                    const_effort=values(cfg.const_effort),
                )
            delay_cls = _resolve_actuator_class(self._modules.actuators, ("Delay",))
            delay = _construct_optional(
                delay_cls,
                delay_steps=wp.array([int(cfg.delay_steps)] * ctx.world_count, dtype=wp.int32, device=device),
                max_delay=max(1, self._actuator_max_delay_steps()),
            )
            clamping = []
            if cfg.clamp_kind == NEWTON_CLAMP_DC_MOTOR:
                cls = _resolve_actuator_class(
                    self._modules.actuators,
                    ("ClampingDCMotor", "DCMotorClamping", "ClampingDCMotorEffort"),
                )
                clamping.append(
                    _construct_optional(
                        cls,
                        max_motor_effort=values(cfg.max_motor_effort),
                        saturation_effort=values(cfg.saturation_effort),
                        velocity_limit=values(cfg.velocity_limit),
                    )
                )
            elif cfg.clamp_kind == NEWTON_CLAMP_MAX_EFFORT and math.isfinite(cfg.effort_limit):
                cls = _resolve_actuator_class(self._modules.actuators, ("ClampingMaxEffort", "MaxEffortClamping"))
                clamping.append(_construct_optional(cls, max_effort=values(cfg.effort_limit)))
            indices = wp.array(
                [world * num_dof + column for world in range(ctx.world_count)],
                dtype=wp.uint32,
                device=device,
            )
            actuator = actuator_cls(
                indices=indices,
                drive=controller,
                delay=delay,
                clamping=clamping or None,
                effort_indices=indices,
                control_target_pos_attr="joint_target_q",
                control_target_vel_attr="joint_target_qd",
                control_feedforward_attr="joint_control_feedforward",
            )
            state = actuator.state()
            ctx.runtime_actuators.append(actuator)
            ctx.runtime_actuator_states.append(state)
            ctx.runtime_next_actuator_states.append(actuator.state() if state is not None else None)

    def _ensure_mujoco_io_buffers(self, ctx, num_steps: int) -> None:  # noqa: ANN001
        """Allocate reusable device-side control staging and mapped-state buffers.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
        """  # noqa: DOC107
        if int(ctx.io_capacity) >= int(num_steps):
            return
        # Existing graphs retain the old staging-buffer addresses. Growing the
        # horizon is a structural change, so release them before reallocating.
        ctx.capture_cache.clear()
        ctx.capture_warm_keys.clear()
        wp = self._modules.warp
        capacity = int(num_steps)
        joint_target_q_count = int(ctx.control.joint_target_q.shape[0])
        joint_dof_count = int(ctx.model.joint_dof_count)
        mapped_width = int(ctx.world_count * ctx.num_dof)
        ctx.target_pos_stage = wp.zeros((capacity, joint_target_q_count), dtype=wp.float32, device=ctx.model.device)
        ctx.target_vel_stage = wp.zeros((capacity, joint_dof_count), dtype=wp.float32, device=ctx.model.device)
        ctx.feedforward_stage = wp.zeros((capacity, joint_dof_count), dtype=wp.float32, device=ctx.model.device)
        ctx.sim_q_stage = wp.zeros((capacity, mapped_width), dtype=wp.float32, device=ctx.model.device)
        ctx.sim_qd_stage = wp.zeros((capacity, mapped_width), dtype=wp.float32, device=ctx.model.device)
        ctx.sim_explicit_effort_stage = None
        if ctx.explicit_columns:
            ctx.sim_explicit_effort_stage = wp.zeros(
                (capacity, int(ctx.world_count * len(ctx.explicit_columns))),
                dtype=wp.float32,
                device=ctx.model.device,
            )
        ctx.io_capacity = capacity

    def _stage_mujoco_controls(self, ctx, delayed_commands: torch.Tensor, num_steps: int) -> None:  # noqa: ANN001
        """Upload all command, velocity-target, and feedforward rows once per rollout.

        Args:
            ctx: Value supplied for ``ctx``.
            delayed_commands: Value supplied for ``delayed_commands``.
            num_steps: Value supplied for ``num_steps``.
        """  # noqa: DOC107
        self._ensure_mujoco_io_buffers(ctx, num_steps)
        capacity = int(ctx.io_capacity)
        joint_dof_count = int(ctx.model.joint_dof_count)
        world_count = int(ctx.world_count)
        delayed_np = delayed_commands.detach().cpu().numpy().astype(np.float32, copy=False)

        target_pos = np.tile(ctx.default_joint_target, (capacity, 1)).astype(np.float32, copy=False)
        mapped_targets = delayed_np.transpose(1, 0, 2).reshape(num_steps, -1)
        target_pos[:num_steps, ctx.target_pos_map_np] = mapped_targets
        target_vel = np.zeros((capacity, joint_dof_count), dtype=np.float32)
        feedforward = np.zeros((capacity, joint_dof_count), dtype=np.float32)

        if ctx.ff_active and ctx.ff_np is not None and ctx.velocity_targets_np is not None:
            target_vel[:num_steps, ctx.dof_map_np] = np.tile(
                np.asarray(ctx.velocity_targets_np[:num_steps], dtype=np.float32),
                (1, world_count),
            )
            feedforward[:num_steps, ctx.dof_map_np] = np.tile(
                np.asarray(ctx.ff_np[:num_steps], dtype=np.float32),
                (1, world_count),
            )

        self._assign_wp_shaped(ctx.target_pos_stage, target_pos)
        self._assign_wp_shaped(ctx.target_vel_stage, target_vel)
        self._assign_wp_shaped(ctx.feedforward_stage, feedforward)

    @staticmethod
    def _assign_wp_shaped(arr, values) -> None:  # noqa: ANN001
        values_np = np.ascontiguousarray(values, dtype=np.float32)
        assign = getattr(arr, "assign", None)
        if not callable(assign):
            raise SysIdEnvironmentBridgeError("warp array does not support in-place assign.")
        assign(values_np)

    def _actuated_joint_maps(self, model, joints_per_world: int, num_dof: int):  # noqa: ANN001, ANN202
        """Map trajectory DOF order to world-0 joint coordinate/DOF indices.

        Same joint-path mapping as the differentiable bridge: the trajectory may
        cover a subset of the model's actuated joints; the unmatched joints are
        simulated but held at their default configuration. Returns coordinate,
        DOF, and position-target maps for selected joints plus all actuated
        1-DOF joints in world 0.

        Args:
            model: Value supplied for ``model``.
            joints_per_world: Value supplied for ``joints_per_world``.
            num_dof: Value supplied for ``num_dof``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        joint_q_start = model.joint_q_start.numpy()
        joint_qd_start = model.joint_qd_start.numpy()
        joint_target_q_start = model.joint_target_q_start.numpy()
        labels = [str(label) for label in getattr(model, "joint_label", [])]

        actuated: list[tuple[int, int, int, int]] = []  # joint, coordinate, DOF, target coordinate
        for joint in range(joints_per_world):
            coord_span = int(joint_q_start[joint + 1] - joint_q_start[joint])
            dof_span = int(joint_qd_start[joint + 1] - joint_qd_start[joint])
            if dof_span == 0:
                continue  # fixed joint
            if coord_span == 7 and dof_span == 6:
                continue  # floating base (free joint): tolerated, not actuated
            if coord_span == 1 and dof_span == 1:
                target_span = int(joint_target_q_start[joint + 1] - joint_target_q_start[joint])
                if target_span != 1:
                    raise SysIdEnvironmentBridgeError(
                        f"Joint {joint} has {target_span} position targets; expected one for a 1-DOF joint."
                    )
                actuated.append(
                    (
                        joint,
                        int(joint_q_start[joint]),
                        int(joint_qd_start[joint]),
                        int(joint_target_q_start[joint]),
                    )
                )
                continue
            raise SysIdEnvironmentBridgeError(
                f"Joint {joint} has {dof_span} DOFs; the Newton MuJoCo bridge maps only 1-DOF "
                "joints plus an optional floating base."
            )
        if len(actuated) < num_dof:
            raise SysIdEnvironmentBridgeError(
                f"Newton model exposes {len(actuated)} actuated joints per world but the trajectory "
                f"has {num_dof} DOFs (check robot import and joint ordering)."
            )

        # Prefer USD-path label matching to fix the DOF order; fall back to import order.
        ordered = actuated[:num_dof]
        matched: list[tuple[int, int, int, int]] = []
        if self._joint_paths and len(self._joint_paths) >= num_dof and len(labels) >= joints_per_world:
            for path in self._joint_paths[:num_dof]:
                found = None
                for joint, coord, dof, target_pos in actuated:
                    label = labels[joint]
                    if label == path or label.endswith(path) or path.endswith(label.rsplit("/", 1)[-1]):
                        found = (joint, coord, dof, target_pos)
                        break
                if found is None:
                    matched = []
                    break
                matched.append(found)
        if matched and len({item[0] for item in matched}) == num_dof:
            ordered = matched
        elif len(actuated) == num_dof:
            carb.log_warn(
                "SysId: Newton MuJoCo bridge could not match joint paths to Newton joint labels; "
                "assuming import order matches trajectory DOF order."
            )
        else:
            extra = len(actuated) - num_dof
            unmatched = [labels[j] if j < len(labels) else f"joint {j}" for j, _c, _d, _t in actuated[num_dof:]]
            carb.log_warn(
                f"SysId: Newton MuJoCo bridge: model has {extra} more actuated joints than the trajectory "
                f"and joint paths did not resolve; assuming the FIRST {num_dof} imported joints match the "
                f"trajectory order and holding the remainder ({', '.join(unmatched)}) at their default "
                "configuration. Pass matching joint paths if this ordering is wrong."
            )
        coord0 = np.asarray([item[1] for item in ordered], dtype=np.int64)
        dof0 = np.asarray([item[2] for item in ordered], dtype=np.int64)
        target_pos0 = np.asarray([item[3] for item in ordered], dtype=np.int64)
        actuated_world0 = [(int(c), int(d), int(t)) for _j, c, d, t in actuated]
        self._joint_mapping_summary = [
            f"tele[{i}]->{labels[j].rsplit('/', 1)[-1] if j < len(labels) else j}(dof {d})"
            for i, (j, _c, d, _t) in enumerate(ordered)
        ]
        return coord0, dof0, target_pos0, actuated_world0

    def _apply_candidate_params(
        self,
        ctx: Any,
        rows: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        num_dof: int,
        world_count: int,
        actuator_dt: float,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, np.ndarray]:
        """Write per-world actuator and rigid-body parameters into the model.

        Args:
            ctx: Value supplied for ``ctx``.
            rows: Value supplied for ``rows``.
            param_entries: Value supplied for ``param_entries``.
            num_dof: Value supplied for ``num_dof``.
            world_count: Value supplied for ``world_count``.
            actuator_dt: Value supplied for ``actuator_dt``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        ke_rows: list[torch.Tensor] = []
        kd_rows: list[torch.Tensor] = []
        fr_rows: list[torch.Tensor] = []
        arm_rows: list[torch.Tensor] = []
        states = []
        for row in rows:
            state = decode_theta_to_apply_state(
                num_dof=num_dof,
                num_links=ctx.bodies_per_world,
                param_entries=param_entries,
                theta_row=row,
                baseline_link_inertia_lc=ctx.baseline_body_inertia_lc_world0,
            )
            states.append(state)
            ke, kd, fr, arm = self._candidate_joint_vectors(state, num_dof)
            ke_rows.append(ke)
            kd_rows.append(kd)
            fr_rows.append(fr)
            arm_rows.append(arm)
        ke_tensor = torch.stack(ke_rows, dim=0)  # (world_count, num_dof)
        kd_tensor = torch.stack(kd_rows, dim=0)
        fr_tensor = torch.stack(fr_rows, dim=0)
        arm_tensor = torch.stack(arm_rows, dim=0)
        model = ctx.model

        # Explicit controllers receive the same candidates through their
        # stable Warp arrays; their native model gains remain zero so a DOF has
        # exactly one control owner.
        model_ke = ke_tensor.clone()
        model_kd = kd_tensor.clone()
        if ctx.explicit_columns:
            model_ke[:, list(ctx.explicit_columns)] = 0.0
            model_kd[:, list(ctx.explicit_columns)] = 0.0
            for runtime_index, column in enumerate(ctx.explicit_columns):
                actuator = ctx.runtime_actuators[runtime_index]
                actuator.drive.kp.assign(
                    np.ascontiguousarray(ke_tensor[:, column].detach().cpu().numpy(), dtype=np.float32)
                )
                actuator.drive.kd.assign(
                    np.ascontiguousarray(kd_tensor[:, column].detach().cpu().numpy(), dtype=np.float32)
                )
                controller_ki = getattr(actuator.drive, "ki", None)
                if controller_ki is not None:
                    ki_values = np.full(world_count, self._actuator_configs[column].integral_gain, dtype=np.float32)
                    for world, state in enumerate(states):
                        values = state.joint_integral_gain
                        if values is not None and np.isfinite(values[column]):
                            ki_values[world] = max(float(values[column]), 0.0)
                    controller_ki.assign(ki_values)
                delay_values = np.full(world_count, self._actuator_configs[column].delay_steps, dtype=np.int32)
                for world, state in enumerate(states):
                    values = state.actuator_command_delay_seconds
                    if values is not None and np.isfinite(values[column]):
                        delay_values[world] = delay_seconds_to_steps(values[column], actuator_dt)
                if np.any(delay_values > self._actuator_max_delay_steps()):
                    raise SysIdEnvironmentBridgeError(
                        f"Actuator delay exceeds allocated capacity ({self._actuator_max_delay_steps()} steps)."
                    )
                actuator.delay.delay_steps.assign(delay_values)

        # Scatter candidate values into the mapped DOFs; held joints keep the
        # model defaults captured at build time.
        def _scatter(default_full: np.ndarray | None, values: torch.Tensor) -> np.ndarray:
            if default_full is None:
                raise SysIdEnvironmentBridgeError("Newton model array is not allocated.")
            full = default_full.copy()
            full[ctx.dof_map_np] = values.reshape(-1).detach().cpu().numpy().astype(np.float32)
            return full

        self._assign_wp(
            getattr(model, "joint_target_ke", None),
            _scatter(ctx.default_target_ke, model_ke),
        )
        self._assign_wp(
            getattr(model, "joint_target_kd", None),
            _scatter(ctx.default_target_kd, model_kd),
        )
        if getattr(model, "joint_armature", None) is not None:
            self._assign_wp(model.joint_armature, _scatter(ctx.default_armature, arm_tensor))
        if getattr(model, "joint_friction", None) is not None:
            self._assign_wp(model.joint_friction, _scatter(ctx.default_friction, fr_tensor))

        has_link_inertial_params = any(entry.param_type in _NEWTON_LINK_INERTIAL_PARAMETERS for entry in param_entries)
        invalid_candidates = np.zeros(world_count, dtype=bool)
        if has_link_inertial_params:
            mass_rows: list[np.ndarray] = []
            com_rows: list[np.ndarray] = []
            inertia_rows: list[np.ndarray] = []
            for state in states:
                mass = ctx.baseline_body_mass_world0 * state.link_mass_scale
                com = ctx.baseline_body_com_world0 + state.link_com_delta
                inertia = ctx.baseline_body_inertia_world0.copy()
                for body, flat in state.link_inertia_flat.items():
                    inertia[int(body)] = np.asarray(flat, dtype=np.float32).reshape(3, 3)
                valid = (
                    np.isfinite(mass).all()
                    and np.all(np.asarray(mass) > 0.0)
                    and np.isfinite(com).all()
                    and all(_physical_inertia_is_valid(value) for value in inertia)
                )
                if not valid:
                    invalid_candidates[len(mass_rows)] = True
                    mass = ctx.baseline_body_mass_world0.copy()
                    com = ctx.baseline_body_com_world0.copy()
                    inertia = ctx.baseline_body_inertia_world0.copy()
                mass_rows.append(np.asarray(mass, dtype=np.float32))
                com_rows.append(np.asarray(com, dtype=np.float32))
                inertia_rows.append(np.asarray(inertia, dtype=np.float32))
            self._assign_wp(model.body_mass, np.stack(mass_rows, axis=0).reshape(-1))
            self._assign_wp_shaped(
                model.body_com,
                np.stack(com_rows, axis=0).reshape(-1, 3),
            )
            self._assign_wp_shaped(
                model.body_inertia,
                np.stack(inertia_rows, axis=0).reshape(-1, 3, 3),
            )

        if ctx.notify_flags is not None:
            notify = getattr(ctx.solver, "notify_model_changed", None)
            joint_flag = getattr(ctx.notify_flags, "JOINT_DOF_PROPERTIES", None)
            if joint_flag is None or not callable(notify):
                raise SysIdEnvironmentBridgeError(
                    "Newton SolverMuJoCo does not expose joint-property refresh notifications."
                )
            flags = int(joint_flag)
            if has_link_inertial_params:
                body_flag = getattr(ctx.notify_flags, "BODY_INERTIAL_PROPERTIES", None)
                if body_flag is None:
                    raise SysIdEnvironmentBridgeError(
                        "This Newton SolverMuJoCo build cannot refresh body inertial properties at runtime."
                    )
                flags |= int(body_flag)
            notify(flags)
        return ke_tensor, kd_tensor, fr_tensor, invalid_candidates

    def _candidate_joint_vectors(self, state, num_dof: int):  # noqa: ANN001, ANN202
        """Convert a decoded candidate state into absolute actuator vectors.

        Args:
            state: Value supplied for ``state``.
            num_dof: Value supplied for ``num_dof``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        base_k, base_d, _base_ig, base_fr, base_arm, _base_ce = self._baseline_drive_vectors(num_dof)
        ke = base_k * torch.as_tensor(state.stiffness_scale[:num_dof], device=self.device)
        kd = base_d * torch.as_tensor(state.damping_scale[:num_dof], device=self.device)
        fr = base_fr * torch.as_tensor(state.friction_scale[:num_dof], device=self.device)
        arm = base_arm.clone()
        if state.joint_armature is not None:
            vals = np.asarray(state.joint_armature[:num_dof], dtype=np.float32)
            finite = np.isfinite(vals)
            arm = torch.where(
                torch.as_tensor(finite, device=self.device),
                torch.as_tensor(np.maximum(vals, 0.0), device=self.device),
                arm,
            )
        return ke, kd, fr, arm

    def _remap_entries_for_newton(self, param_entries, ctx):  # noqa: ANN001, ANN202
        """Translate SysID link indices into replicated-model body indices.

        Args:
            param_entries: Value supplied for ``param_entries``.
            ctx: Value supplied for ``ctx``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC106, DOC107
        remapped: list[SysIdParameterEntry] = []
        for entry in param_entries:
            if entry.param_type not in _NEWTON_LINK_INERTIAL_PARAMETERS:
                remapped.append(entry)
                continue
            if entry.link_index == GLOBAL_LINK_INDEX:
                if entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
                    raise SysIdEnvironmentBridgeError(
                        "Global Link Inertia Log-Cholesky entries are ambiguous; select an explicit link."
                    )
                remapped.append(entry)
                continue
            body = ctx.link_to_body.get(int(entry.link_index))
            if body is None:
                path = (
                    self._link_paths[entry.link_index]
                    if 0 <= entry.link_index < len(self._link_paths)
                    else f"link {entry.link_index}"
                )
                raise SysIdEnvironmentBridgeError(
                    f"Cannot map SysID link '{path}' to a Newton body for parameter "
                    f"{entry.param_type.value}; pass matching link paths and check the robot import."
                )
            if (
                entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY
                and body not in ctx.baseline_body_inertia_lc_world0
            ):
                raise SysIdEnvironmentBridgeError(
                    f"Newton body {body} for link '{self._link_paths[entry.link_index]}' has a "
                    "degenerate baseline inertia and cannot use Log-Cholesky identification."
                )
            remapped.append(replace(entry, link_index=int(body)))
        return remapped

    def _assign_wp(self, arr, values):  # noqa: ANN001, ANN202
        """Assign a flat float buffer into an existing warp array in place.

        Args:
            arr: Value supplied for ``arr``.
            values: Value supplied for ``values``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC106, DOC107
        if arr is None:
            raise SysIdEnvironmentBridgeError("Newton model array is not allocated.")
        np_vals = np.ascontiguousarray(_to_numpy(values).reshape(-1), dtype=np.float32)
        assign = getattr(arr, "assign", None)
        if not callable(assign):
            raise SysIdEnvironmentBridgeError("warp array does not support in-place assign.")
        assign(np_vals)
        return arr

    def _read_wp(self, arr) -> torch.Tensor:  # noqa: ANN001
        """Read a warp array into a torch tensor on the bridge device.

        Always returns a CLONE: ``wp.to_torch`` aliases the underlying warp
        buffer, and the double-buffered ``state0``/``state1`` would otherwise be
        overwritten by a later ``solver.step``, corrupting earlier samples.

        Args:
            arr: Value supplied for ``arr``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC106, DOC107
        wp = self._modules.warp
        to_torch = getattr(wp, "to_torch", None)
        if callable(to_torch):
            return to_torch(arr).to(device=self.device, dtype=torch.float32).clone()
        return torch.as_tensor(np.asarray(arr.numpy(), dtype=np.float32), device=self.device).clone()

    def write_parameters_to_usd(
        self,
        stage,  # noqa: ANN001
        param_entries: list[SysIdParameterEntry],
        theta_row: torch.Tensor,
    ) -> bool:
        """Persist Newton actuator parameters that have a USD actuator target.

        Args:
            stage: USD stage used by the operation.
            param_entries: Value supplied for ``param_entries``.
            theta_row: Value supplied for ``theta_row``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        selected_actuator_entries = [
            entry for entry in param_entries if _requires_newton_actuator_writeback(self, entry)
        ]
        if not selected_actuator_entries:
            return True
        if stage is None:
            return False
        num_dof = max(1, self._num_joints)
        self._ensure_config_count(num_dof)
        state = decode_theta_to_apply_state(
            num_dof=num_dof,
            num_links=1,
            param_entries=param_entries,
            theta_row=theta_row,
        )
        all_written = True
        promotions = list(getattr(self, "_actuator_promotions", []))
        promotion_paths = {str(item.prim_path) for item in promotions}
        for entry in selected_actuator_entries:
            requested = int(entry.dof_index)
            indices = range(num_dof) if requested < 0 else (requested,)
            for dof_index in indices:
                if dof_index < 0 or dof_index >= len(self._joint_paths):
                    all_written = False
                    continue
                cfg = self._actuator_configs[dof_index]
                if getattr(cfg, "source", NEWTON_ACTUATOR_SOURCE_DRIVE_DEFAULTS) != NEWTON_ACTUATOR_SOURCE_USD:
                    # Stateful explicit parameters need an actuator prim as a
                    # durable writeback target. Never report provenance-only
                    # success when the accepted value cannot be persisted.
                    if entry.param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN or is_actuator_delay_parameter(
                        entry.param_type
                    ):
                        all_written = False
                    continue
                value = self._newton_writeback_value(entry.param_type, dof_index, state)
                if value is None or not np.isfinite(value):
                    all_written = False
                    continue
                attr_name = _newton_attr_for_parameter(entry.param_type)
                if not attr_name:
                    continue
                written = False
                target = self._joint_paths[dof_index]
                matching_prims = [prim for prim in stage.Traverse() if target in _target_paths(prim)]
                for prim in matching_prims:
                    edit_context = nullcontext()
                    if str(prim.GetPath()) in promotion_paths:
                        from pxr import Usd

                        edit_context = Usd.EditContext(stage, stage.GetSessionLayer())
                    with edit_context:
                        if is_actuator_delay_parameter(entry.param_type):
                            prim.AddAppliedSchema("NewtonActuatorDelayAPI")
                            _set_newton_int_attr(
                                prim,
                                attr_name,
                                delay_seconds_to_steps(float(value), self._physics_dt),
                            )
                        else:
                            _set_newton_float_attr(prim, attr_name, max(float(value), 0.0))
                    written = True
                all_written = all_written and written
        if all_written and promotions:
            from .explicit_pd_actuator_compat import (
                commit_session_actuator_promotions,
                discard_session_actuator_promotions,
            )

            all_written = bool(commit_session_actuator_promotions(stage, promotions))
            if all_written:
                discard_session_actuator_promotions(stage, promotions)
                self._actuator_promotions = []
        return all_written

    def _newton_writeback_value(
        self,
        param_type: SysIdParameterType,
        dof_index: int,
        state,  # noqa: ANN001
    ) -> float | None:
        if dof_index < 0 or dof_index >= self._num_joints:
            return None
        cfg = self._actuator_configs[dof_index] if dof_index < len(self._actuator_configs) else None
        if param_type == SysIdParameterType.JOINT_STIFFNESS:
            baseline = max(float(getattr(cfg, "stiffness", 1.0)), 1e-6)
            return baseline * float(state.stiffness_scale[dof_index])
        if param_type == SysIdParameterType.JOINT_DAMPING:
            baseline = max(float(getattr(cfg, "damping", 1.0)), 1e-6)
            return baseline * float(state.damping_scale[dof_index])
        if param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN and state.joint_integral_gain is not None:
            return float(state.joint_integral_gain[dof_index])
        if is_actuator_delay_parameter(param_type) and state.actuator_command_delay_seconds is not None:
            return float(state.actuator_command_delay_seconds[dof_index])
        return None

    def _initial_state(self, num_dof: int) -> tuple[torch.Tensor, torch.Tensor]:
        assert self._trajectory is not None
        q = torch.zeros(num_dof, dtype=torch.float32)
        dq = torch.zeros(num_dof, dtype=torch.float32)
        if self._trajectory.positions.shape[0] > 0:
            q = torch.as_tensor(self._trajectory.positions[0, :num_dof], dtype=torch.float32)
        if self._trajectory.velocities.shape[0] > 0:
            dq = torch.as_tensor(self._trajectory.velocities[0, :num_dof], dtype=torch.float32)
        return q, dq

    def _rollout_dt(self, num_steps: int) -> float:
        if self._trajectory is None or num_steps < 2:
            return self._physics_dt
        times = np.asarray(self._trajectory.times[:num_steps], dtype=np.float64).reshape(-1)
        if times.shape[0] < 2:
            return self._physics_dt
        diffs = np.diff(times)
        diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
        if diffs.size == 0:
            return self._physics_dt
        return float(np.median(diffs))

    def _baseline_drive_vectors(self, num_dof: int) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
    ]:
        self._ensure_config_count(num_dof)
        stiffness = np.ones(num_dof, dtype=np.float32)
        damping = np.ones(num_dof, dtype=np.float32)
        integral_gain = np.zeros(num_dof, dtype=np.float32)
        friction = np.zeros(num_dof, dtype=np.float32)
        armature = np.zeros(num_dof, dtype=np.float32)
        const_effort = np.zeros(num_dof, dtype=np.float32)
        for idx, cfg in enumerate(self._actuator_configs[:num_dof]):
            stiffness[idx] = max(float(cfg.stiffness), 1e-6)
            damping[idx] = max(float(cfg.damping), 1e-6)
            integral_gain[idx] = max(float(cfg.integral_gain), 0.0)
            const_effort[idx] = float(cfg.const_effort)
        for idx, joint in enumerate(self._joint_baselines[:num_dof]):
            if joint is None:
                continue
            friction[idx] = max(float(joint.friction or joint.dynamic_friction), 0.0)
            armature[idx] = max(float(joint.armature), 0.0)
        return (
            torch.as_tensor(stiffness, device=self.device),
            torch.as_tensor(damping, device=self.device),
            torch.as_tensor(integral_gain, device=self.device),
            torch.as_tensor(friction, device=self.device),
            torch.as_tensor(armature, device=self.device),
            torch.as_tensor(const_effort, device=self.device),
        )

    def _build_actuator_configs(self) -> None:
        count = max(self._num_joints, 0)
        configs = self._default_actuator_configs(count)
        actuator_runtime = getattr(self, "actuator_runtime", ACTUATOR_RUNTIME_MIXED)
        if actuator_runtime in {ACTUATOR_RUNTIME_EXPLICIT, ACTUATOR_RUNTIME_MIXED}:
            configs = self._overlay_usd_actuator_configs(configs)
        self._actuator_configs = configs

    def _ensure_stateful_actuator_targets(self, entries: list[SysIdParameterEntry]) -> None:
        """Promote selected stateful DOFs when durable actuator writeback is requested.

        Args:
            entries: Parameter entries handled by the operation.
        """
        if self.explicit_actuator_policy != EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED:
            return
        if self.stage is None or not self._joint_paths:
            return
        integral_dofs: set[int] = set()
        delay_dofs: set[int] = set()
        for entry in entries:
            is_integral = entry.param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN
            is_delay = is_actuator_delay_parameter(entry.param_type)
            if not (is_integral or is_delay):
                continue
            requested = int(entry.dof_index)
            indices = set(range(self._num_joints) if requested < 0 else (requested,))
            if is_integral:
                integral_dofs.update(indices)
            if is_delay:
                delay_dofs.update(indices)
        valid = set(range(min(self._num_joints, len(self._joint_paths))))
        integral_targets = {self._joint_paths[index] for index in integral_dofs & valid}
        delay_targets = {self._joint_paths[index] for index in delay_dofs & valid}
        targets = integral_targets | delay_targets
        missing = targets - self._promoted_target_paths
        if not missing:
            return
        from .articulation_utils import find_articulation_root
        from .explicit_pd_actuator_compat import (
            promote_selected_dofs_to_session_actuators,
        )

        root = find_articulation_root(self.stage, self.robot_prim_path) or self.robot_prim_path
        try:
            promotions = []
            groups = (
                (missing & integral_targets & delay_targets, NEWTON_CONTROLLER_PID, True),
                (missing & (integral_targets - delay_targets), NEWTON_CONTROLLER_PID, False),
                (missing & (delay_targets - integral_targets), NEWTON_CONTROLLER_PD, True),
            )
            for group_targets, controller_kind, include_delay in groups:
                if group_targets:
                    promotions.extend(
                        promote_selected_dofs_to_session_actuators(
                            self.stage,
                            root,
                            sorted(group_targets),
                            controller_kind=controller_kind,
                            include_delay=include_delay,
                        )
                    )
            self._actuator_promotions.extend(promotions)
            self._promoted_target_paths.update(targets)
            self._build_actuator_configs()
            self._build_actuators()
            for ctx in self._mujoco_cache.values():
                self._release_mujoco_context(ctx)
            self._mujoco_cache.clear()
        except Exception as exc:
            self._discard_actuator_promotions()
            raise SysIdEnvironmentBridgeError(f"Failed to promote selected Newton actuator DOFs: {exc}") from exc

    def _discard_actuator_promotions(self) -> None:
        promotions = list(getattr(self, "_actuator_promotions", []))
        stage = getattr(self, "stage", None)
        if stage is not None and promotions:
            from .explicit_pd_actuator_compat import discard_session_actuator_promotions

            discard_session_actuator_promotions(stage, promotions)
        self._actuator_promotions = []
        self._promoted_target_paths = set()

    def _default_actuator_configs(self, count: int) -> list[NewtonActuatorConfig]:
        configs: list[NewtonActuatorConfig] = []
        for dof_index in range(count):
            joint = self._joint_baselines[dof_index] if dof_index < len(self._joint_baselines) else None
            stiffness = max(float(getattr(joint, "stiffness", 1.0)), 1e-6)
            damping = max(float(getattr(joint, "damping", 1.0)), 1e-6)
            effort_limit = self._read_effort_limit(joint)
            configs.append(
                NewtonActuatorConfig(
                    dof_index=dof_index,
                    source=NEWTON_ACTUATOR_SOURCE_DRIVE_DEFAULTS,
                    controller_kind=self.newton_config.controller,
                    clamp_kind=self.newton_config.effort_clamp,
                    stiffness=stiffness,
                    damping=damping,
                    integral_gain=0.0,
                    effort_limit=effort_limit,
                    max_motor_effort=effort_limit,
                    saturation_effort=effort_limit,
                    velocity_limit=float("inf"),
                )
            )
        return configs

    def _overlay_usd_actuator_configs(self, configs: list[NewtonActuatorConfig]) -> list[NewtonActuatorConfig]:
        if self.stage is None or not self._joint_paths:
            return configs
        joint_to_dof = {path: idx for idx, path in enumerate(self._joint_paths)}
        claimed_targets: dict[str, str] = {}
        for prim in self.stage.Traverse():
            if not _looks_like_newton_actuator(prim):
                continue
            targets = _target_paths(prim)
            mapped_targets = [target for target in targets if target in joint_to_dof]
            if not mapped_targets:
                continue
            if len(targets) != 1:
                raise SysIdEnvironmentBridgeError(
                    f"Newton actuator '{prim.GetPath()}' must target exactly one joint; got {targets}."
                )
            target = mapped_targets[0]
            if target in claimed_targets:
                raise SysIdEnvironmentBridgeError(
                    f"Newton actuators '{claimed_targets[target]}' and '{prim.GetPath()}' both target '{target}'."
                )
            claimed_targets[target] = str(prim.GetPath())
            dof_index = joint_to_dof[target]
            if 0 <= dof_index < len(configs):
                configs[dof_index] = self._config_from_usd_prim(prim, configs[dof_index])
        return configs

    def _config_from_usd_prim(self, prim, baseline: NewtonActuatorConfig) -> NewtonActuatorConfig:  # noqa: ANN001
        applied = _applied_schemas(prim)
        controllers = applied & {
            "NewtonPDControlAPI",
            "NewtonPIDControlAPI",
            "NewtonNeuralControlAPI",
        }
        if len(controllers) != 1:
            raise SysIdEnvironmentBridgeError(
                f"Newton actuator '{prim.GetPath()}' must apply exactly one supported controller schema; "
                f"got {sorted(controllers) or ['none']}."
            )
        if {"NewtonMaxEffortClampingAPI", "NewtonDCMotorClampingAPI"}.issubset(applied):
            raise SysIdEnvironmentBridgeError(
                f"Newton actuator '{prim.GetPath()}' applies multiple effort-clamping schemas."
            )
        unsupported = []
        neural = None
        if "NewtonNeuralControlAPI" in applied:
            neural = classify_newton_neural_prim(self._modules.actuators, prim)
            unsupported.extend(neural["unsupported_schemas"])
            if not neural["controller_kind"]:
                details = ", ".join(neural["unsupported_schemas"]) or "unclassified neural controller"
                raise SysIdEnvironmentBridgeError(
                    f"Newton actuator '{prim.GetPath()}' has an unusable NewtonNeuralControlAPI: {details}."
                )
        if "NewtonPositionBasedClampingAPI" in applied:
            unsupported.append("NewtonPositionBasedClampingAPI")
        if neural is not None and neural["controller_kind"]:
            controller_kind = str(neural["controller_kind"])
        elif "NewtonPIDControlAPI" in applied:
            controller_kind = NEWTON_CONTROLLER_PID
        elif "NewtonPDControlAPI" in applied:
            controller_kind = NEWTON_CONTROLLER_PD
        else:
            controller_kind = self.newton_config.controller
        if "NewtonDCMotorClampingAPI" in applied:
            clamp_kind = NEWTON_CLAMP_DC_MOTOR
        elif "NewtonMaxEffortClampingAPI" in applied:
            clamp_kind = NEWTON_CLAMP_MAX_EFFORT
        else:
            clamp_kind = self.newton_config.effort_clamp
        config = NewtonActuatorConfig(
            dof_index=baseline.dof_index,
            source=NEWTON_ACTUATOR_SOURCE_USD,
            controller_kind=controller_kind,
            clamp_kind=clamp_kind,
            stiffness=_attr_float(prim, "newton:kp", baseline.stiffness),
            damping=_attr_float(prim, "newton:kd", baseline.damping),
            integral_gain=_attr_float(prim, "newton:ki", baseline.integral_gain),
            integral_max=_attr_float(prim, "newton:integralMax", baseline.integral_max),
            const_effort=_attr_float(prim, "newton:constEffort", baseline.const_effort),
            effort_limit=_attr_float(prim, "newton:maxEffort", baseline.effort_limit),
            max_motor_effort=_attr_float(prim, "newton:maxMotorEffort", baseline.max_motor_effort),
            saturation_effort=_attr_float(prim, "newton:saturationEffort", baseline.saturation_effort),
            velocity_limit=_attr_float(prim, "newton:velocityLimit", baseline.velocity_limit),
            delay_steps=max(0, _attr_int(prim, "newton:delaySteps", baseline.delay_steps)),
            model_path=str(neural.get("model_path", "")) if neural else "",
            resolved_model_path=str(neural.get("resolved_model_path", "")) if neural else "",
            model_type=str(neural.get("model_type", "")) if neural else "",
            controller_class_name=str(neural.get("controller_class_name", "")) if neural else "",
            controller_kwargs=dict(neural.get("controller_kwargs", {})) if neural else {},
            unsupported_schemas=unsupported,
        )
        self._validate_actuator_config(prim, config)
        return config

    @staticmethod
    def _validate_actuator_config(prim, config: NewtonActuatorConfig) -> None:  # noqa: ANN001
        for name, value in (
            ("newton:kp", config.stiffness),
            ("newton:kd", config.damping),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise SysIdEnvironmentBridgeError(f"{prim.GetPath()}.{name} must be finite and non-negative.")
        if config.controller_kind == NEWTON_CONTROLLER_PID:
            if not math.isfinite(float(config.integral_gain)) or float(config.integral_gain) < 0.0:
                raise SysIdEnvironmentBridgeError(f"{prim.GetPath()}.newton:ki must be finite and non-negative.")
            if math.isnan(float(config.integral_max)) or float(config.integral_max) < 0.0:
                raise SysIdEnvironmentBridgeError(f"{prim.GetPath()}.newton:integralMax must be non-negative.")
        if int(config.delay_steps) < 0:
            raise SysIdEnvironmentBridgeError(f"{prim.GetPath()}.newton:delaySteps must be non-negative.")
        clamp_values = (
            (
                ("newton:maxMotorEffort", config.max_motor_effort),
                ("newton:saturationEffort", config.saturation_effort),
                ("newton:velocityLimit", config.velocity_limit),
            )
            if config.clamp_kind == NEWTON_CLAMP_DC_MOTOR
            else (("newton:maxEffort", config.effort_limit),)
        )
        for name, value in clamp_values:
            if math.isnan(float(value)) or float(value) < 0.0:
                raise SysIdEnvironmentBridgeError(f"{prim.GetPath()}.{name} must be non-negative.")

    def _build_actuators(self) -> None:
        actuators = self._modules.actuators
        self._actuator_specs = []
        for cfg in self._actuator_configs:
            delay = self._construct_delay(actuators)
            clamping = self._construct_clamping(actuators, cfg)
            controller = self._construct_controller(actuators, cfg, delay=delay, clamping=clamping)
            actuator, actuator_state, next_actuator_state = self._construct_neural_actuator(cfg, controller, delay)
            self._actuator_specs.append(
                NewtonActuatorSpec(
                    dof_index=cfg.dof_index,
                    controller=controller,
                    delay=delay,
                    clamping=clamping,
                    stiffness=cfg.stiffness,
                    damping=cfg.damping,
                    effort_limit=cfg.effort_limit,
                    config=cfg,
                    actuator=actuator,
                    actuator_state=actuator_state,
                    next_actuator_state=next_actuator_state,
                )
            )

    def _construct_delay(self, actuators):  # noqa: ANN001, ANN202
        wp = self._modules.warp
        device = self.newton_config.device or "cpu"
        delay_cls = _resolve_actuator_class(actuators, ("Delay",))
        # Newton's Delay requires max_delay >= 1; before a trajectory is set the
        # estimated delay capacity is 0.
        return _construct_optional(
            delay_cls,
            delay_steps=_wp_array(wp, [0], "int32", device),
            max_delay=max(1, self._actuator_max_delay_steps()),
        )

    def _construct_controller(self, actuators, cfg: NewtonActuatorConfig, *, delay, clamping):  # noqa: ANN001, ANN202
        del delay, clamping
        kp = self._wp_float_array(cfg.stiffness)
        kd = self._wp_float_array(cfg.damping)
        const_effort = self._wp_float_array(cfg.const_effort)
        if cfg.controller_kind in _NEWTON_NEURAL_CONTROLLERS:
            controller_cls = self._resolve_neural_controller_class(actuators, cfg)
            kwargs = dict(cfg.controller_kwargs)
            model_path = cfg.resolved_model_path or cfg.model_path or kwargs.get("model_path", "")
            kwargs["model_path"] = str(model_path)
            return _construct_optional(controller_cls, **kwargs)
        if cfg.controller_kind == NEWTON_CONTROLLER_PID:
            controller_cls = _resolve_actuator_class(
                actuators,
                ("DrivePID", "ControllerPID", "PIDController", "ControlPID"),
            )
            return _construct_optional(
                controller_cls,
                kp=kp,
                kd=kd,
                ki=self._wp_float_array(cfg.integral_gain),
                integral_max=self._wp_float_array(cfg.integral_max),
                const_effort=const_effort,
            )
        controller_cls = _resolve_actuator_class(
            actuators,
            ("DrivePD", "ControllerPD", "PDController", "ControlPD"),
        )
        return _construct_optional(
            controller_cls,
            kp=kp,
            kd=kd,
            const_effort=const_effort,
        )

    def _resolve_neural_controller_class(self, actuators, cfg: NewtonActuatorConfig):  # noqa: ANN001, ANN202
        if cfg.controller_kind == NEWTON_CONTROLLER_NEURAL_MLP:
            names = ("DriveNeuralMLP", "ControllerNeuralMLP")
        elif cfg.controller_kind == NEWTON_CONTROLLER_NEURAL_LSTM:
            names = ("DriveNeuralLSTM", "ControllerNeuralLSTM")
        else:
            names = ()
        if cfg.controller_class_name:
            names = (cfg.controller_class_name, *names)
        return _resolve_actuator_class(actuators, names)

    def _construct_neural_actuator(self, cfg: NewtonActuatorConfig, controller, delay):  # noqa: ANN001, ANN202
        if cfg.controller_kind not in _NEWTON_NEURAL_CONTROLLERS:
            return None, None, None
        actuator_cls = getattr(self._modules.actuators, "Actuator", None)
        if actuator_cls is None:
            raise SysIdEnvironmentBridgeError("The configured Newton neural actuator class is unavailable.")
        wp = self._modules.warp
        device = self.newton_config.device or "cpu"
        actuator = actuator_cls(
            indices=_wp_array(wp, [0], "uint32", device),
            drive=controller,
            delay=delay,
            clamping=None,
            effort_indices=_wp_array(wp, [0], "uint32", device),
            control_feedforward_attr="joint_control_feedforward",
        )
        state_fn = getattr(actuator, "state", None)
        state = state_fn() if callable(state_fn) else None
        next_state = state_fn() if callable(state_fn) else None
        return actuator, state, next_state

    def _construct_clamping(self, actuators, cfg: NewtonActuatorConfig):  # noqa: ANN001, ANN202
        if cfg.clamp_kind == NEWTON_CLAMP_NONE:
            return None
        if cfg.clamp_kind == NEWTON_CLAMP_DC_MOTOR:
            clamp_cls = _resolve_actuator_class(
                actuators,
                ("ClampingDCMotor", "DCMotorClamping", "ClampingDCMotorEffort"),
            )
            return _construct_optional(
                clamp_cls,
                max_motor_effort=self._wp_float_array(cfg.max_motor_effort),
                saturation_effort=self._wp_float_array(cfg.saturation_effort),
                velocity_limit=self._wp_float_array(cfg.velocity_limit),
            )
        clamp_cls = _resolve_actuator_class(actuators, ("ClampingMaxEffort", "MaxEffortClamping"))
        return _construct_optional(clamp_cls, max_effort=self._wp_float_array(cfg.effort_limit))

    def _wp_float_array(self, value: float):  # noqa: ANN202
        return _wp_array(
            self._modules.warp,
            [float(value)],
            "float32",
            self.newton_config.device or "cpu",
        )

    def _actuator_max_delay_steps(self) -> int:
        if self._trajectory is None:
            return 0
        times = np.asarray(self._trajectory.times, dtype=np.float64).reshape(-1)
        if times.shape[0] < 2:
            return 0
        finite = times[np.isfinite(times)]
        if finite.shape[0] < 2:
            return 0
        dt = self._rollout_dt(int(times.shape[0]))
        if dt <= 0.0 or not math.isfinite(float(dt)):
            return max(0, int(times.shape[0]) - 1)
        duration_steps = int(math.ceil(max(float(finite[-1] - finite[0]), 0.0) / float(dt) - 1e-9))
        return max(0, int(times.shape[0]) - 1, duration_steps)

    def _ensure_config_count(self, num_dof: int) -> None:
        if len(self._actuator_configs) >= num_dof:
            return
        existing = list(self._actuator_configs)
        self._num_joints = num_dof
        self._actuator_configs = existing + self._default_actuator_configs(num_dof)[len(existing) :]

    @staticmethod
    def _read_effort_limit(joint: JointUsdSnapshot | None) -> float:
        if joint is not None and math.isfinite(float(joint.max_force)):
            return max(float(joint.max_force), 0.0)
        return float("inf")


def _looks_like_newton_actuator(prim) -> bool:  # noqa: ANN001
    try:
        if prim.GetTypeName() == "NewtonActuator":
            return True
    except Exception:
        pass
    return bool(_target_paths(prim))


def _target_paths(prim) -> list[str]:  # noqa: ANN001
    try:
        rel = prim.GetRelationship("newton:targets")
        if not rel:
            return []
        return [str(path) for path in rel.GetTargets()]
    except Exception:
        return []


def _applied_schemas(prim) -> set[str]:  # noqa: ANN001
    try:
        return {str(name) for name in prim.GetAppliedSchemas()}
    except Exception:
        return set()


def _attr_float(prim, name: str, default: float) -> float:  # noqa: ANN001
    try:
        attr = prim.GetAttribute(name)
        if not attr:
            return float(default)
        value = attr.Get()
    except Exception:
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _attr_int(prim, name: str, default: int) -> int:  # noqa: ANN001
    try:
        attr = prim.GetAttribute(name)
        if not attr:
            return int(default)
        value = attr.Get()
        return int(default if value is None else value)
    except (TypeError, ValueError, RuntimeError):
        return int(default)


def _uses_usd_actuator_writeback(bridge) -> bool:  # noqa: ANN001
    config = getattr(bridge, "newton_config", None)
    return str(getattr(config, "actuator_source", "")) == NEWTON_ACTUATOR_SOURCE_USD


def _requires_newton_actuator_writeback(bridge, entry: SysIdParameterEntry) -> bool:  # noqa: ANN001
    if entry.param_type == SysIdParameterType.JOINT_INTEGRAL_GAIN or is_actuator_delay_parameter(entry.param_type):
        return True
    if entry.param_type not in (
        SysIdParameterType.JOINT_STIFFNESS,
        SysIdParameterType.JOINT_DAMPING,
    ):
        return False
    if not _uses_usd_actuator_writeback(bridge):
        return False
    dof_index = int(entry.dof_index)
    configs = getattr(bridge, "_actuator_configs", [])
    if dof_index < 0:
        return any(str(getattr(config, "source", "")) == NEWTON_ACTUATOR_SOURCE_USD for config in configs)
    if dof_index >= len(configs):
        return False
    return str(getattr(configs[dof_index], "source", "")) == NEWTON_ACTUATOR_SOURCE_USD


def _newton_attr_for_parameter(param_type: SysIdParameterType) -> str:
    if is_actuator_delay_parameter(param_type):
        return "newton:delaySteps"
    return {
        SysIdParameterType.JOINT_STIFFNESS: "newton:kp",
        SysIdParameterType.JOINT_DAMPING: "newton:kd",
        SysIdParameterType.JOINT_INTEGRAL_GAIN: "newton:ki",
    }.get(param_type, "")


def _set_newton_float_attr(prim, name: str, value: float) -> None:  # noqa: ANN001
    from pxr import Sdf

    attr = prim.GetAttribute(name)
    if attr is None or not attr.IsValid():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float, custom=True)
    attr.Set(float(value))


def _set_newton_int_attr(prim, name: str, value: int) -> None:  # noqa: ANN001
    from pxr import Sdf

    attr = prim.GetAttribute(name)
    if attr is None or not attr.IsValid():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Int, custom=True)
    attr.Set(int(value))


def classify_newton_neural_prim(actuators, prim) -> dict[str, Any]:  # noqa: ANN001
    """Classify a NewtonNeuralControlAPI prim using Newton's own parser when available.

    Args:
        actuators: Value supplied for ``actuators``.
        prim: Value supplied for ``prim``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    model_path, resolved_model_path = _attr_asset_paths(prim, "newton:modelPath")
    result: dict[str, Any] = {
        "controller_kind": "",
        "model_path": model_path,
        "resolved_model_path": resolved_model_path,
        "model_type": "",
        "controller_class_name": "",
        "controller_kwargs": {},
        "unsupported_schemas": [],
    }
    if not model_path:
        result["unsupported_schemas"].append("NewtonNeuralControlAPI:missing_model_path")
        return result
    parse = getattr(actuators, "parse_actuator_prim", None)
    if not callable(parse):
        result["unsupported_schemas"].append("NewtonNeuralControlAPI:parser_unavailable")
        return result
    try:
        parsed = parse(prim)
    except Exception as exc:
        result["unsupported_schemas"].append(f"NewtonNeuralControlAPI:parse_error:{exc}")
        return result
    drive_class = getattr(parsed, "drive_class", None)
    if drive_class is None:
        drive_class = getattr(parsed, "controller_class", None)
    class_name = getattr(drive_class, "__name__", str(drive_class or ""))
    drive_kwargs = getattr(parsed, "drive_kwargs", None)
    if drive_kwargs is None:
        drive_kwargs = getattr(parsed, "controller_kwargs", {})
    kwargs = _stringify_kwargs(drive_kwargs or {})
    if kwargs.get("model_path"):
        result["model_path"] = str(kwargs["model_path"])
    kind = _neural_controller_kind_from_class_name(class_name)
    if not kind:
        label = class_name or kwargs.get("model_type") or kwargs.get("model_path") or "unknown"
        result["unsupported_schemas"].append(f"NewtonNeuralControlAPI:unsupported_model_type:{label}")
        result["controller_class_name"] = class_name
        result["controller_kwargs"] = kwargs
        return result
    result["controller_kind"] = kind
    result["model_type"] = "mlp" if kind == NEWTON_CONTROLLER_NEURAL_MLP else "lstm"
    result["controller_class_name"] = class_name
    result["controller_kwargs"] = kwargs
    return result


def _neural_controller_kind_from_class_name(class_name: str) -> str:
    if class_name in {"DriveNeuralMLP", "ControllerNeuralMLP"}:
        return NEWTON_CONTROLLER_NEURAL_MLP
    if class_name in {"DriveNeuralLSTM", "ControllerNeuralLSTM"}:
        return NEWTON_CONTROLLER_NEURAL_LSTM
    return ""


def _attr_asset_paths(prim, name: str) -> tuple[str, str]:  # noqa: ANN001
    try:
        attr = prim.GetAttribute(name)
        if not attr:
            return "", ""
        value = attr.Get()
    except Exception:
        return "", ""
    if value is None:
        return "", ""
    authored = getattr(value, "path", None)
    resolved = getattr(value, "resolvedPath", None)
    if authored is None:
        authored = str(value)
    return str(authored or ""), str(resolved or "")


def _stringify_kwargs(kwargs: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in dict(kwargs).items():
        if key == "model_path":
            out[key] = str(value)
        elif isinstance(value, (str, int, float, bool)) or value is None:
            out[key] = value
        else:
            out[key] = str(value)
    return out


def _wp_array(wp, values: list[float] | list[int], dtype_name: str, device: str):  # noqa: ANN001, ANN202
    dtype = getattr(wp, dtype_name, None)
    attempts = (
        {"dtype": dtype, "device": device},
        {"dtype": dtype},
        {},
    )
    last_error = None
    for kwargs in attempts:
        kwargs = {key: value for key, value in kwargs.items() if value is not None}
        try:
            return wp.array(values, **kwargs)
        except TypeError as exc:
            last_error = exc
            continue
    if last_error is not None:
        raise last_error
    return wp.array(values)


def _to_numpy(values) -> np.ndarray:  # noqa: ANN001
    """Convert a torch tensor / array-like to a float32 numpy array.

    Args:
        values: Value supplied for ``values``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    if isinstance(values, torch.Tensor):
        return values.detach().cpu().numpy().astype(np.float32)
    return np.asarray(values, dtype=np.float32)


def _load_newton_core_modules() -> NewtonCoreModules:
    try:
        newton = importlib.import_module("newton")
        warp = importlib.import_module("warp")
    except ModuleNotFoundError as exc:
        missing = str(exc.name or "")
        optional_roots = ("newton", "warp")
        if any(missing == root or missing.startswith(f"{root}.") for root in optional_roots):
            raise SysIdRuntimeUnavailableError(
                "Newton SysID requires the Newton and Warp packages delivered by isaacsim.pip.newton."
            ) from exc
        raise
    return NewtonCoreModules(newton=newton, warp=warp)


def _load_newton_modules() -> NewtonModules:
    core = _load_newton_core_modules()
    try:
        actuators = importlib.import_module("newton.actuators")
    except ModuleNotFoundError as exc:
        missing = str(exc.name or "")
        optional_roots = ("newton", "mujoco", "mujoco_warp")
        if any(missing == root or missing.startswith(f"{root}.") for root in optional_roots):
            raise SysIdRuntimeUnavailableError(
                "Newton MuJoCo SysID requires Newton 1.6 with built-in actuators, MuJoCo, and Warp as delivered "
                "by isaacsim.pip.newton."
            ) from exc
        raise
    return NewtonModules(newton=core.newton, warp=core.warp, actuators=actuators)


def _resolve_actuator_class(module, names: tuple[str, ...]):  # noqa: ANN001, ANN202
    for name in names:
        cls = getattr(module, name, None)
        if cls is not None:
            return cls
    return None


def _construct_optional(cls, **kwargs):  # noqa: ANN001, ANN003, ANN202
    if cls is None:
        raise SysIdEnvironmentBridgeError("The configured Newton actuator class is unavailable.")
    payload = _filter_constructor_kwargs(cls, {key: value for key, value in kwargs.items() if value is not None})
    return cls(**payload)


def _filter_constructor_kwargs(cls, kwargs: dict[str, Any]) -> dict[str, Any]:  # noqa: ANN001
    try:
        signature = inspect.signature(cls)
    except (TypeError, ValueError):
        return kwargs
    params = signature.parameters
    if any(param.kind == inspect.Parameter.VAR_KEYWORD for param in params.values()):
        return kwargs
    allowed = {
        name
        for name, param in params.items()
        if name != "self" and param.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    }
    return {key: value for key, value in kwargs.items() if key in allowed}
