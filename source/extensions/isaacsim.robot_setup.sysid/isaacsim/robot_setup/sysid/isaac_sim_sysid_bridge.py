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

"""Isaac Sim implementation of :class:`SysIdEnvironmentBridge` using cloned articulations."""

from __future__ import annotations

import asyncio
import contextlib
import os
import time
from pathlib import PurePosixPath
from typing import Any, Optional

import carb
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import numpy as np
import omni.kit.app
import omni.timeline
import torch
import warp as wp
from isaacsim.core.cloner import GridCloner
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdUtils

from . import runtime
from .actuator_compatibility import (
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_IMPLICIT,
    ACTUATOR_RUNTIME_MIXED,
    EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    PHYSICS_BACKEND_NEWTON,
    PHYSICS_BACKEND_PHYSX,
    is_actuator_delay_parameter,
)
from .analytical_presolve import build_fixed_base_context_from_usd
from .articulation_utils import find_articulation_root
from .backend_capabilities import (
    RESIDUAL_CONTACT_FORCE,
    RESIDUAL_END_EFFECTOR_POSE,
    RESIDUAL_TORQUE,
    get_backend_capabilities,
)
from .env_bridge import SysIdEnvironmentBridgeError
from .explicit_pd_actuator_compat import (
    ExplicitPdActuatorCompat,
    discard_session_actuator_promotions,
    promote_selected_dofs_to_session_actuators,
)
from .fabric_batch import (
    read_batched_contact_proxy,
    read_batched_joint_state,
    read_batched_link_pose,
)
from .inertia_param import inertia_matrix_to_log_cholesky
from .parameter_apply import decode_theta_to_apply_state
from .parameter_space import ParameterBaselineState
from .parameter_types import SysIdParameterEntry, SysIdParameterType
from .rollout_result import SysIdRolloutResult
from .trajectory_csv import TrajectoryDataset


def _build_cloned_articulation_regex(articulation_root: str, source_env_path: str, env_paths_root: str) -> str:
    """Build a regex matching articulation roots across env_0, env_1, ...

    Args:
        articulation_root: Value supplied for ``articulation_root``.
        source_env_path: Value supplied for ``source_env_path``.
        env_paths_root: Value supplied for ``env_paths_root``.

    Returns:
        Result produced by the operation.
    """
    root = PurePosixPath(articulation_root)
    source = PurePosixPath(source_env_path.rstrip("/"))
    env_root = PurePosixPath(env_paths_root.rstrip("/"))
    try:
        rel = root.relative_to(source)
    except ValueError as exc:
        raise SysIdEnvironmentBridgeError(
            f"Articulation root '{articulation_root}' must be inside clone source env "
            f"'{source_env_path}'. Set clone source env to the env Xform (e.g. "
            f"/World/envs/env_0), not /World or the robot link path. "
            f"Robot prim path was used to find the articulation root."
        ) from exc
    suffix = "" if rel == PurePosixPath(".") else "/" + rel.as_posix()
    env_parent = env_root.parent.as_posix()
    env_name = env_root.name
    pattern_prefix = f"{env_parent}/{env_name}_[0-9]+".replace(".", r"\.")
    return pattern_prefix + suffix.replace(".", r"\.")


def _warp_array_to_numpy(data) -> np.ndarray:  # noqa: ANN001
    """Copy warp GPU arrays to host numpy without routing through torch CUDA.

    Args:
        data: Value supplied for ``data``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    if isinstance(data, wp.array):
        return np.array(data.numpy(), dtype=np.float32, copy=True)
    if isinstance(data, torch.Tensor):
        return data.detach().cpu().numpy().astype(np.float32, copy=True)
    return np.asarray(data, dtype=np.float32)


def _warp_array_to_torch(data: Any, device: torch.device) -> torch.Tensor:
    """Return an owning tensor while keeping Warp GPU data on the device.

    Args:
        data: Warp, Torch, or array-compatible state.
        device: Device that owns the returned tensor.

    Returns:
        Owning state tensor on ``device``.
    """
    if isinstance(data, torch.Tensor):
        return data.detach().to(device=device, dtype=torch.float32).clone()
    if isinstance(data, wp.array):
        try:
            # Physics tensor buffers are mutable and reused, so retain an
            # owning device-side snapshot for the rollout sample.
            return wp.to_torch(data).to(device=device, dtype=torch.float32).clone()
        except (AttributeError, RuntimeError, TypeError):
            pass
    return torch.as_tensor(_warp_array_to_numpy(data), device=device, dtype=torch.float32)


def _joint_has_body_targets(prim: Usd.Prim) -> bool:
    targets = []
    for rel_name in ("physics:body0", "physics:body1"):
        rel = prim.GetRelationship(rel_name)
        if rel:
            targets.extend(rel.GetTargets())
    return bool(targets)


class IsaacSimSysIdBridge:
    """Vectorized SysId bridge backed by :class:`Articulation` and optional :class:`GridCloner`.
    By default, rollouts advance physics directly with ``SimulationManager.step()`` without
    rendering or pumping full Kit frames. The timeline-driven path remains available for
    interactive debugging by setting ``offline_stepping=False``.
    By default ``use_parallel_clones=False``: finite-difference perturbations run sequentially on
    ``env_0`` only (no GridCloner), which is more stable on GPU PhysX.

    Args:
        robot_prim_path: Constructor value for ``robot_prim_path``.
        source_env_path: Constructor value for ``source_env_path``.
        env_paths_root: Constructor value for ``env_paths_root``.
        clone_spacing: Constructor value for ``clone_spacing``.
        joint_dof_indices: Constructor value for ``joint_dof_indices``.
        use_parallel_clones: Constructor value for ``use_parallel_clones``.
        co_locate_clones: Constructor value for ``co_locate_clones``.
        fabric_clones: Constructor value for ``fabric_clones``.
        offline_stepping: Whether to step physics without pumping Kit frames.
        end_effector_link_index: Constructor value for ``end_effector_link_index``.
        max_actuator_delay_seconds: Maximum supported explicit actuator delay.
        selected_dof_paths: Optional ordered USD paths for selected degrees of freedom.
        explicit_dof_paths: Optional USD paths controlled by explicit actuators.
        physics_backend: Physics backend used for the rollout.
        actuator_runtime: Runtime ownership mode for actuator gains and commands.
        explicit_actuator_policy: Policy used to select explicit actuators.
    """  # noqa: D205

    def __init__(
        self,
        robot_prim_path: str,
        source_env_path: str = "/World/envs/env_0",
        env_paths_root: str = "/World/envs/env",
        clone_spacing: float = 2.0,
        joint_dof_indices: Optional[list[int]] = None,
        use_parallel_clones: bool = False,
        co_locate_clones: bool = True,
        fabric_clones: bool = False,
        offline_stepping: bool = True,
        end_effector_link_index: int = -1,
        max_actuator_delay_seconds: float = 0.0,
        selected_dof_paths: Optional[list[str]] = None,
        explicit_dof_paths: Optional[list[str]] = None,
        physics_backend: str = PHYSICS_BACKEND_PHYSX,
        actuator_runtime: str = ACTUATOR_RUNTIME_IMPLICIT,
        explicit_actuator_policy: str = EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
    ) -> None:
        self._robot_prim_path = robot_prim_path
        self._source_env_path = source_env_path
        self._env_paths_root = env_paths_root
        self._clone_spacing = clone_spacing
        self._joint_dof_indices = joint_dof_indices
        self.use_parallel_clones = use_parallel_clones
        self.co_locate_clones = co_locate_clones
        self.fabric_clones = fabric_clones
        self.offline_stepping = bool(offline_stepping)
        self.end_effector_link_index = end_effector_link_index
        self.physics_backend = str(physics_backend).lower()
        if self.physics_backend not in (PHYSICS_BACKEND_PHYSX, PHYSICS_BACKEND_NEWTON):
            raise SysIdEnvironmentBridgeError(f"Unknown Isaac Sim physics backend: {self.physics_backend!r}.")
        self._backend_capabilities = get_backend_capabilities(
            engine="isaac_sim",
            physics_backend=self.physics_backend,
        )
        if self._backend_capabilities is None:
            raise SysIdEnvironmentBridgeError(
                f"No SysID capabilities are defined for Isaac Sim backend {self.physics_backend!r}."
            )
        self.actuator_runtime = str(actuator_runtime).lower()
        if self.actuator_runtime not in (
            ACTUATOR_RUNTIME_IMPLICIT,
            ACTUATOR_RUNTIME_EXPLICIT,
            ACTUATOR_RUNTIME_MIXED,
        ):
            raise SysIdEnvironmentBridgeError(f"Unknown actuator runtime: {self.actuator_runtime!r}.")
        self.explicit_actuator_policy = str(explicit_actuator_policy).lower()
        if self.explicit_actuator_policy not in (
            EXPLICIT_ACTUATOR_POLICY_AUTHORED_ONLY,
            EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
        ):
            raise SysIdEnvironmentBridgeError(f"Unknown explicit actuator policy: {self.explicit_actuator_policy!r}.")
        self._previous_physics_backend = str(SimulationManager.get_active_physics_engine())
        if self._previous_physics_backend != self.physics_backend:
            if not SimulationManager.switch_physics_engine(self.physics_backend, verbose=True):
                raise SysIdEnvironmentBridgeError(
                    f"Unable to switch Isaac Sim physics from {self._previous_physics_backend!r} "
                    f"to {self.physics_backend!r}. Ensure isaacsim.physics.newton is enabled."
                )
        self._max_actuator_delay_seconds = max(float(max_actuator_delay_seconds), 0.0)
        self._selected_dof_paths = None if selected_dof_paths is None else [str(path) for path in selected_dof_paths]
        self._explicit_dof_paths = None if explicit_dof_paths is None else [str(path) for path in explicit_dof_paths]
        self._explicit_dof_columns: list[int] = []
        self._explicit_pd_compat: ExplicitPdActuatorCompat | None = None
        self._actuator_metadata_cache: dict = {
            "integration": self.actuator_runtime,
            "physics_backend": self.physics_backend,
        }
        self._cloned_for_count = 0
        self._trajectory: Optional[TrajectoryDataset] = None
        self._articulation: Optional[Articulation] = None
        self._articulation_paths_expr: Optional[str] = None
        self._articulation_root: Optional[str] = None
        self._num_envs = 0
        self._num_joints = 0
        # Friction baselines stay `None` when friction is unselected, so capture
        # completion needs its own sentinel rather than a `None` check.
        self._baselines_captured = False
        self._baseline_static_friction: Optional[np.ndarray] = None
        self._baseline_dynamic_friction: Optional[np.ndarray] = None
        self._baseline_stiffness: Optional[np.ndarray] = None
        self._baseline_damping: Optional[np.ndarray] = None
        self._baseline_armature: Optional[np.ndarray] = None
        self._baseline_link_masses: Optional[np.ndarray] = None
        self._baseline_link_com: Optional[np.ndarray] = None
        self._baseline_link_com_orientation: Optional[np.ndarray] = None
        self._baseline_link_inertia: Optional[np.ndarray] = None
        self._baseline_link_inertia_lc: Optional[dict[int, np.ndarray]] = None
        self._baseline_dof_lower: Optional[np.ndarray] = None
        self._baseline_dof_upper: Optional[np.ndarray] = None
        self._link_prim_paths: list[str] = []
        self._num_links = 0
        self._last_clone_co_locate: Optional[bool] = None
        self._physics_replication_registered = False
        self._created_clone_paths: list[str] = []
        self._logical_env_count: int = 1
        self._fabric_rebind_logged = False
        self._fabric_command_probe_logged = False
        self._fabric_state_probe_logged = False
        self._diagnostics_enabled = os.getenv("ISAACSIM_SYSID_DIAGNOSTICS", "").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        # Direct bridge users collect every trustworthy backend signal. The
        # rollout optimizer narrows these flags from the active residual config.
        self._collect_torque = self._backend_capabilities.supports_residual(RESIDUAL_TORQUE)
        self._collect_end_effector_pose = self._backend_capabilities.supports_residual(RESIDUAL_END_EFFECTOR_POSE)
        self._collect_contact_force = self._backend_capabilities.supports_residual(RESIDUAL_CONTACT_FORCE)
        self._physics_subscription = None
        self._rollout_active = False
        self._rollout_cancel_requested = False
        self._rollout_warmup_remaining: int = 0
        self._rollout_done: Optional[asyncio.Event] = None
        self._rollout_error: Optional[Exception] = None
        self._rollout_csv_index = 0
        self._rollout_substep = 0
        self._rollout_num_steps = 0
        self._rollout_commands: Optional[np.ndarray] = None
        self._rollout_positions: list[torch.Tensor] = []
        self._rollout_velocities: list[torch.Tensor] = []
        self._rollout_torques: list[torch.Tensor] = []
        self._rollout_ee_poses: list[torch.Tensor] = []
        self._rollout_contact_forces: list[torch.Tensor] = []
        self._device = torch.device("cpu")
        self._pending_param_entries: list[SysIdParameterEntry] = []

    @property
    def supports_arbitrary_candidate_batch(self) -> bool:
        """True when parallel clones evaluate arbitrary theta rows in one rollout."""
        return bool(self.use_parallel_clones)

    @property
    def device(self) -> torch.device:  # noqa: D102
        return self._device

    @property
    def usd_parameter_types(self) -> set[SysIdParameterType]:
        """Parameter types persisted by the explicit actuator adapter."""
        if not self._uses_explicit_actuators:
            return set()
        claimed = {
            SysIdParameterType.JOINT_INTEGRAL_GAIN,
            SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
        }
        if self.actuator_runtime == ACTUATOR_RUNTIME_EXPLICIT:
            claimed.update(
                {
                    SysIdParameterType.JOINT_STIFFNESS,
                    SysIdParameterType.JOINT_DAMPING,
                }
            )
        return claimed

    @property
    def usd_parameter_dof_claims(self) -> dict[SysIdParameterType, set[int]]:
        """Mixed-mode parameter ownership that cannot be represented by type alone."""
        if self.actuator_runtime != ACTUATOR_RUNTIME_MIXED:
            return {}
        columns = set(self._explicit_dof_columns)
        return {
            SysIdParameterType.JOINT_STIFFNESS: columns,
            SysIdParameterType.JOINT_DAMPING: columns,
        }

    @property
    def _uses_explicit_actuators(self) -> bool:
        return self.actuator_runtime in (ACTUATOR_RUNTIME_EXPLICIT, ACTUATOR_RUNTIME_MIXED)

    def _active_trajectory_dof_paths(self) -> list[str]:
        """Return only the USD joints represented by trajectory columns.

        Returns:
            Ordered USD joint paths matching the active telemetry columns.
        """
        paths = list(self._selected_dof_paths or self.source_parameter_paths()[1])
        if len(paths) < self._num_joints:
            raise SysIdEnvironmentBridgeError(
                f"The trajectory has {self._num_joints} DOFs but only {len(paths)} joint paths were resolved."
            )
        return paths[: self._num_joints]

    @property
    def actuator_metadata(self) -> dict:
        """Get metadata for the configured actuator runtime.

        Returns:
            Serializable actuator configuration metadata.
        """
        if self._explicit_pd_compat is not None:
            return self._explicit_pd_compat.metadata
        return dict(self._actuator_metadata_cache)

    def set_trajectory(self, trajectory: TrajectoryDataset) -> None:
        """Store the active trajectory for resets (initial pose / velocity).

        Args:
            trajectory: Telemetry trajectory used by the operation.
        """
        self._trajectory = trajectory
        self._num_joints = trajectory.num_joints
        if self._joint_dof_indices is None:
            self._joint_dof_indices = list(range(self._num_joints))
        elif len(self._joint_dof_indices) != self._num_joints:
            raise SysIdEnvironmentBridgeError(
                f"joint_dof_indices length ({len(self._joint_dof_indices)}) must match "
                f"trajectory joints ({self._num_joints})."
            )

    def validate_parameter_entries(self, param_entries: list[SysIdParameterEntry]) -> None:
        """Fail before rollout when selected parameters cannot be applied by this bridge.

        Args:
            param_entries: Value supplied for ``param_entries``.
        """
        self._pending_param_entries = list(param_entries)
        selected_types = {entry.param_type for entry in param_entries}
        unsupported = self._backend_capabilities.unsupported_parameter_types(param_entries)
        if unsupported:
            raise SysIdEnvironmentBridgeError(
                f"{self._backend_capabilities.label} does not support parameters: {', '.join(unsupported)}."
            )
        if any(is_actuator_delay_parameter(value) for value in selected_types) and not self._uses_explicit_actuators:
            raise SysIdEnvironmentBridgeError(
                "actuator_command_delay_seconds requires actuator_runtime='newton_explicit' or 'mixed' in Isaac Sim."
            )

        if SysIdParameterType.JOINT_INTEGRAL_GAIN in selected_types and not self._uses_explicit_actuators:
            raise SysIdEnvironmentBridgeError("joint_integral_gain requires an explicit Newton actuator runtime.")
        art = self._articulation
        checks = {
            SysIdParameterType.LINK_COM_OFFSET_X: "set_link_coms",
            SysIdParameterType.LINK_COM_OFFSET_Y: "set_link_coms",
            SysIdParameterType.LINK_COM_OFFSET_Z: "set_link_coms",
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY: "set_link_inertias",
            SysIdParameterType.JOINT_ARMATURE: "set_dof_armatures",
            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE: "set_dof_limits",
            SysIdParameterType.JOINT_LIMIT_UPPER_SCALE: "set_dof_limits",
        }
        if art is None:
            return
        missing = sorted(
            {
                method
                for entry in param_entries
                for ptype, method in checks.items()
                if entry.param_type == ptype and not hasattr(art, method)
            }
        )
        if missing:
            raise SysIdEnvironmentBridgeError(
                "Selected parameters require unsupported Articulation methods: " + ", ".join(missing)
            )

    def configure_rollout_signals(
        self,
        *,
        torque: bool,
        end_effector_pose: bool,
        contact_force: bool,
    ) -> None:
        """Select optional state channels collected at every trajectory sample.

        Args:
            torque: Whether to collect joint torque.
            end_effector_pose: Whether to collect the end-effector pose.
            contact_force: Whether to collect contact force.
        """
        requested = {
            RESIDUAL_TORQUE: bool(torque),
            RESIDUAL_END_EFFECTOR_POSE: bool(end_effector_pose),
            RESIDUAL_CONTACT_FORCE: bool(contact_force),
        }
        unsupported = sorted(
            channel
            for channel, enabled in requested.items()
            if enabled and not self._backend_capabilities.supports_residual(channel)
        )
        if unsupported:
            raise SysIdEnvironmentBridgeError(
                f"{self._backend_capabilities.label} does not produce rollout signals: {', '.join(unsupported)}."
            )
        self._collect_torque = requested[RESIDUAL_TORQUE]
        self._collect_end_effector_pose = requested[RESIDUAL_END_EFFECTOR_POSE]
        self._collect_contact_force = requested[RESIDUAL_CONTACT_FORCE]

    def _validate_pending_parameter_entries(self) -> None:
        if self._pending_param_entries:
            self.validate_parameter_entries(self._pending_param_entries)

    def get_analytical_dynamics_context(self):  # noqa: ANN201
        """Return a fixed-base analytical context when one has been configured.

        Returns:
            Result produced by the operation.
        """
        if self._baseline_link_masses is None:
            return None
        stage = runtime.get_current_stage()
        if stage is None:
            return None
        link_paths, dof_paths = self.source_parameter_paths()
        if not link_paths or not dof_paths:
            return None
        baseline = ParameterBaselineState()
        masses = self._source_baseline_row(self._baseline_link_masses)
        if masses is not None:
            for link_idx, value in enumerate(np.asarray(masses, dtype=np.float64).reshape(-1)):
                baseline.link_masses[link_idx] = float(max(value, 1e-9))
        if self._baseline_link_com is not None:
            for link_idx, value in enumerate(np.asarray(self._baseline_link_com, dtype=np.float64)):
                baseline.link_com[link_idx] = value.reshape(3)
        if self._baseline_link_inertia_lc is not None:
            baseline.link_inertia_lc.update(self._baseline_link_inertia_lc)
        return build_fixed_base_context_from_usd(stage, link_paths, dof_paths, baseline)

    def _ensure_physics_subscription(self) -> None:
        if self._physics_subscription is None:
            self._physics_subscription = runtime.register_physics_post_step_callback(self._on_physics_step)

    def _release_physics_subscription(self) -> None:
        """Drop the physics callback registered through ``SimulationManager``."""
        if self._physics_subscription is not None:
            try:
                runtime.deregister_physics_callback(self._physics_subscription)
            except Exception as exc:
                carb.log_warn(f"SysId: failed to deregister physics callback: {exc}")
            self._physics_subscription = None

    def _physics_env_count(self) -> int:
        """Number of clones stepped in PhysX (all FD envs when parallel clones are on).

        Returns:
            Result produced by the operation.
        """
        if not self.use_parallel_clones:
            return 1
        if self._cloned_for_count > 0:
            return self._cloned_for_count
        return max(1, self._logical_env_count)

    def _close_explicit_pd_compat(self) -> None:
        compat = self._explicit_pd_compat
        if compat is not None:
            self._actuator_metadata_cache = dict(compat.metadata)
            try:
                compat.close()
            except Exception as exc:
                carb.log_warn(f"SysId: failed to close explicit PD actuator adapter: {exc}")
            finally:
                self._explicit_pd_compat = None

    def _restore_physics_backend(self) -> None:
        """Return the process-wide Isaac Sim physics engine to its pre-run value."""
        previous = getattr(self, "_previous_physics_backend", "")
        if not previous or previous == self.physics_backend:
            return
        active = str(SimulationManager.get_active_physics_engine())
        if active == previous:
            self._previous_physics_backend = self.physics_backend
            return
        if SimulationManager.switch_physics_engine(previous, verbose=True):
            self._previous_physics_backend = self.physics_backend
        else:
            carb.log_warn(f"SysId: could not restore Isaac Sim physics backend from {active!r} to {previous!r}.")

    def _ensure_explicit_pd_compat(self) -> None:
        if not self._uses_explicit_actuators:
            return
        if self._articulation is None or not self._articulation_paths_expr or not self._articulation_root:
            raise SysIdEnvironmentBridgeError("Explicit PD actuators require a resolved articulation.")
        env_count = len(self._articulation)
        if self._explicit_pd_compat is not None and self._explicit_pd_compat.env_count == env_count:
            return
        self._close_explicit_pd_compat()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise SysIdEnvironmentBridgeError("No USD stage is open for explicit actuator discovery.")
        all_paths = self._active_trajectory_dof_paths()
        if self._explicit_dof_paths is not None:
            dof_paths = list(self._explicit_dof_paths)
        else:
            dof_paths = list(all_paths)
        if not dof_paths:
            return
        by_path = {path: index for index, path in enumerate(all_paths)}
        try:
            self._explicit_dof_columns = [by_path[path] for path in dof_paths]
        except KeyError as exc:
            raise SysIdEnvironmentBridgeError(
                f"Explicit actuator target {exc.args[0]!r} is not part of the trajectory DOF mapping."
            ) from exc
        explicit_indices = [self._joint_dof_indices[column] for column in self._explicit_dof_columns]
        selected_types = {entry.param_type for entry in self._pending_param_entries}
        promotions = []
        if self.explicit_actuator_policy == EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED:
            promotions = promote_selected_dofs_to_session_actuators(
                stage,
                self._articulation_root,
                dof_paths,
                controller_kind=("pid" if SysIdParameterType.JOINT_INTEGRAL_GAIN in selected_types else "pd"),
                include_delay=any(is_actuator_delay_parameter(value) for value in selected_types),
            )
        try:
            with self._articulation_backend_context():
                restore_stiffness, restore_damping = self._articulation.get_dof_gains(
                    indices=list(range(env_count)),
                    dof_indices=explicit_indices,
                )
            restore_stiffness = _warp_array_to_numpy(restore_stiffness)
            restore_damping = _warp_array_to_numpy(restore_damping)
            if restore_stiffness.ndim == 1:
                restore_stiffness = restore_stiffness.reshape(env_count, -1)
            if restore_damping.ndim == 1:
                restore_damping = restore_damping.reshape(env_count, -1)
            self._explicit_pd_compat = ExplicitPdActuatorCompat.create(
                stage=stage,
                articulation_root=self._articulation_root,
                articulation_paths=self._articulation_paths_expr,
                selected_dof_paths=dof_paths,
                env_count=env_count,
                physics_dt=runtime.get_physics_dt(),
                max_delay_seconds=self._max_actuator_delay_seconds,
                device=None,
                restore_stiffness=restore_stiffness,
                restore_damping=restore_damping,
                restore_dof_indices=explicit_indices,
                promotions=promotions,
            )
            integral_columns: set[int] = set()
            for entry in self._pending_param_entries:
                if entry.param_type != SysIdParameterType.JOINT_INTEGRAL_GAIN:
                    continue
                index = int(entry.dof_index)
                if index < 0:
                    integral_columns.update(range(self._num_joints))
                else:
                    integral_columns.add(index)
            for descriptor, column in zip(
                self._explicit_pd_compat.descriptors,
                self._explicit_dof_columns,
            ):
                if column in integral_columns and descriptor.controller_kind != "pid":
                    raise ValueError(
                        f"DOF '{descriptor.dof_name}' selects joint_integral_gain but its authored "
                        "explicit actuator is PD. Author a PID actuator or remove the integral parameter."
                    )
        except Exception as exc:
            compat_owned_promotions = self._explicit_pd_compat is not None
            self._close_explicit_pd_compat()
            if promotions and not compat_owned_promotions:
                discard_session_actuator_promotions(stage, promotions)
            raise SysIdEnvironmentBridgeError(f"Failed to configure explicit PD actuators: {exc}") from exc
        self._actuator_metadata_cache = dict(self._explicit_pd_compat.metadata)
        carb.log_info(
            f"SysId: explicit actuator adapter configured for {len(dof_paths)} of {self._num_joints} DOFs, "
            f"{env_count} environment(s), max delay {self._explicit_pd_compat.max_delay_steps} physics steps."
        )

    def _has_ready_articulation(self, required_count: int) -> bool:
        art = self._articulation
        if art is None:
            return False
        try:
            return len(art) >= required_count and art.is_physics_tensor_entity_valid()
        except Exception:
            return False

    async def _kit_updates_async(self, count: int = 8) -> None:
        """Yield Kit updates without blocking the asyncio event loop.

        Args:
            count: Value supplied for ``count``.
        """
        app = omni.kit.app.get_app()
        for _ in range(count):
            await app.next_update_async()

    def _kit_updates_sync(self, count: int = 4) -> None:
        """Pump a few Kit updates from synchronous cleanup call sites.

        Args:
            count: Value supplied for ``count``.
        """
        app_interface = omni.kit.app.get_app_interface()
        update = getattr(app_interface, "update", None)
        if not callable(update):
            return
        for _ in range(max(0, count)):
            try:
                update()
            except Exception:
                break

    def _flush_physics_changes(self) -> None:
        try:
            runtime.flush_physics_changes()
        except Exception:
            pass

    def _articulation_backend_context(self):  # noqa: ANN202
        """Force Fabric clone articulation operations through the PhysX tensor view.

        Returns:
            Result produced by the operation.
        """
        if self.fabric_clones and self.use_parallel_clones:
            return backend_utils.use_backend("tensor", raise_on_unsupported=True, raise_on_fallback=True)
        return contextlib.nullcontext()

    def _fabric_direct_articulation_view(self):  # noqa: ANN202
        if not self.fabric_clones or not self.use_parallel_clones or self._articulation is None:
            return None
        view = getattr(self._articulation, "_physics_articulation_view", None)
        if view is None:
            return None
        count = int(getattr(view, "count", 0))
        if count < self._physics_env_count():
            return None
        return view

    def _fabric_direct_read_dofs(self, field: str, *, indices: list[int], dof_indices: list[int]):  # noqa: ANN202
        view = self._fabric_direct_articulation_view()
        if view is None:
            return None
        getter = getattr(view, f"get_dof_{field}", None)
        if not callable(getter):
            return None
        try:
            data = getter()
        except (AttributeError, NotImplementedError, TypeError):
            return None
        env_indices = ops_utils.resolve_indices(indices, count=view.count, device=data.device)
        dof_indices_wp = ops_utils.resolve_indices(dof_indices, count=view.max_dofs, device=data.device)
        rows = data[env_indices, dof_indices_wp].contiguous()
        tensor = _warp_array_to_torch(rows, self._device)
        if tensor.ndim == 1:
            tensor = tensor.reshape(1, -1)
        return tensor

    def _fabric_direct_write_dofs(
        self,
        field: str,
        values,  # noqa: ANN001
        *,
        indices: list[int],
        dof_indices: list[int],
        also_target: bool = False,
    ) -> bool:
        view = self._fabric_direct_articulation_view()
        if view is None:
            return False
        getter = getattr(view, f"get_dof_{field}")
        setter = getattr(view, f"set_dof_{field}")
        data = getter()
        env_indices = ops_utils.resolve_indices(indices, count=view.count, device=data.device)
        dof_indices_wp = ops_utils.resolve_indices(dof_indices, count=view.max_dofs, device=data.device)
        values_wp = ops_utils.broadcast_to(
            values,
            shape=(env_indices.shape[0], dof_indices_wp.shape[0]),
            dtype=wp.float32,
            device=data.device,
        )
        wp.copy(data[env_indices, dof_indices_wp], values_wp)
        setter(data, env_indices)
        if also_target:
            target_field = {"positions": "position_targets", "velocities": "velocity_targets"}[field]
            target_getter = getattr(view, f"get_dof_{target_field}")
            target_setter = getattr(view, f"set_dof_{target_field}")
            target_data = target_getter()
            wp.copy(target_data[env_indices, dof_indices_wp], values_wp)
            target_setter(target_data, env_indices)
        return True

    def _log_fabric_direct_joint_rows(self, label: str, rows) -> None:  # noqa: ANN001
        if not self._diagnostics_enabled or not self.fabric_clones:
            return
        try:
            arr = _warp_array_to_numpy(rows)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[0] < 2:
                max_delta = 0.0
            else:
                max_delta = float(np.max(np.abs(arr[1:] - arr[0:1])))
            carb.log_info(
                f"SysId Fabric direct DOF probe ({label}): shape={arr.shape}, "
                f"max row delta vs env_0={max_delta:.6g}"
            )
        except Exception as exc:
            carb.log_warn(f"SysId Fabric direct DOF probe ({label}) failed: {exc}")

    def _log_fabric_direct_command_probe(self, cmd_targets, positions) -> None:  # noqa: ANN001
        if not self._diagnostics_enabled or self._fabric_command_probe_logged or not self.fabric_clones:
            return
        self._fabric_command_probe_logged = True
        try:
            target_np = _warp_array_to_numpy(cmd_targets)
            pos_np = _warp_array_to_numpy(positions)
            if target_np.ndim == 1:
                target_np = target_np.reshape(1, -1)
            if pos_np.ndim == 1:
                pos_np = pos_np.reshape(1, -1)
            target_row_delta = 0.0
            if target_np.shape[0] > 1:
                target_row_delta = float(np.max(np.abs(target_np[1:] - target_np[0:1])))
            target_pos_delta = (
                float(np.max(np.abs(target_np - pos_np))) if target_np.shape == pos_np.shape else float("nan")
            )
            carb.log_info(
                "SysId Fabric direct DOF probe (first command): "
                f"target_shape={target_np.shape}, max target row delta vs env_0={target_row_delta:.6g}, "
                f"max target-position delta={target_pos_delta:.6g}"
            )
        except Exception as exc:
            carb.log_warn(f"SysId Fabric direct DOF probe (first command) failed: {exc}")

    def _log_fabric_direct_state_probe(self, pos_t: torch.Tensor, vel_t: torch.Tensor) -> None:
        if not self._diagnostics_enabled or self._fabric_state_probe_logged or not self.fabric_clones:
            return
        self._fabric_state_probe_logged = True
        try:
            pos_np = pos_t.detach().cpu().numpy()
            vel_np = vel_t.detach().cpu().numpy()
            pos_row_delta = 0.0
            vel_max = float(np.max(np.abs(vel_np))) if vel_np.size else 0.0
            if pos_np.shape[0] > 1:
                pos_row_delta = float(np.max(np.abs(pos_np[1:] - pos_np[0:1])))
            carb.log_info(
                "SysId Fabric direct DOF probe (first sampled state): "
                f"pos_shape={pos_np.shape}, max pos row delta vs env_0={pos_row_delta:.6g}, "
                f"max |velocity|={vel_max:.6g}"
            )
        except Exception as exc:
            carb.log_warn(f"SysId Fabric direct DOF probe (first sampled state) failed: {exc}")

    def _ensure_gpu_dynamics_capacity(self, stage: Usd.Stage, num_envs: int) -> None:
        """Raise PhysX GPU pair capacities before cloning many articulated envs.

        Args:
            stage: USD stage used by the operation.
            num_envs: Value supplied for ``num_envs``.
        """
        scene_prim = None
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                scene_prim = prim
                break
        if scene_prim is None:
            return
        try:
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
            target = max(4096, int(num_envs) * 128)
            for get_name, create_name in (
                ("GetGpuFoundLostAggregatePairsCapacityAttr", "CreateGpuFoundLostAggregatePairsCapacityAttr"),
                ("GetGpuTotalAggregatePairsCapacityAttr", "CreateGpuTotalAggregatePairsCapacityAttr"),
            ):
                attr = getattr(physx_scene, get_name)()
                current = attr.Get() if attr and attr.HasAuthoredValue() else None
                if current is None or int(current) < target:
                    create = getattr(physx_scene, create_name)
                    create(target, True)
                    carb.log_info(
                        f"SysId: set {scene_prim.GetPath()}.{get_name[3:-4]} to {target} for {num_envs} envs."
                    )
        except Exception as exc:
            carb.log_warn(f"SysId: could not raise PhysX GPU dynamics capacities before cloning: {exc}")

    def _physics_replication_blocker(self, stage: Usd.Stage) -> str | None:
        """Return a reason to avoid PhysX replication for the current source env.

        Args:
            stage: USD stage used by the operation.

        Returns:
            Result produced by the operation.
        """
        source = stage.GetPrimAtPath(self._source_env_path)
        if not source or not source.IsValid():
            return None
        missing_body_rel_paths: list[str] = []
        for prim in Usd.PrimRange(source):
            try:
                is_joint = (
                    prim.IsA(UsdPhysics.Joint)
                    or prim.IsA(UsdPhysics.RevoluteJoint)
                    or prim.IsA(UsdPhysics.PrismaticJoint)
                )
            except Exception:
                is_joint = False
            if is_joint and not _joint_has_body_targets(prim):
                missing_body_rel_paths.append(str(prim.GetPath()))
        if missing_body_rel_paths:
            preview = ", ".join(missing_body_rel_paths[:3])
            extra = "" if len(missing_body_rel_paths) <= 3 else f", +{len(missing_body_rel_paths) - 3} more"
            return f"source joints missing body relationships: {preview}{extra}"
        return None

    def _warmup_physics_steps(self) -> int:
        """Physics steps to skip after reset/parameter writes (stabilizes multi-env GPU PhysX).

        Returns:
            Result produced by the operation.
        """
        return max(30, self._steps_per_csv_sample() * 2)

    def _record_rollout_sample(self) -> None:
        """Append the current sim state as measured-row ``_rollout_csv_index``.

        Row convention: recorded row ``k`` is the state at measured sample ``k``
        — row 0 is the reapplied initial state, and the CSV-sample interval
        driven by command row ``k`` produces row ``k + 1`` (the last command row
        never drives an interval).
        """
        pos, vel, torque, ee_pose, contact = self.get_extended_sim_state()
        self._rollout_positions.append(pos)
        self._rollout_velocities.append(vel)
        if torque is not None:
            self._rollout_torques.append(torque)
        if ee_pose is not None:
            self._rollout_ee_poses.append(ee_pose)
        if contact is not None:
            self._rollout_contact_forces.append(contact)
        self._rollout_csv_index += 1
        if self._rollout_csv_index >= self._rollout_num_steps:
            self._rollout_active = False
            self._rollout_done.set()
            return
        self._set_position_targets(self._rollout_commands[self._rollout_csv_index - 1])

    def _on_physics_step(self, _step_dt: float, _context) -> None:  # noqa: ANN001
        if not self._rollout_active or self._rollout_done is None:
            return
        try:
            if self._rollout_warmup_remaining > 0:
                self._rollout_warmup_remaining -= 1
                if self._rollout_warmup_remaining == 0:
                    self._reapply_trajectory_initial_state()
                    # The reapplied initial state IS measured row 0: record it
                    # before any command interval runs.
                    self._record_rollout_sample()
                return
            substeps = self._steps_per_csv_sample()
            self._rollout_substep += 1
            if self._rollout_substep < substeps:
                return
            self._rollout_substep = 0
            self._record_rollout_sample()
        except Exception as exc:
            self._rollout_error = exc
            self._articulation = None
            self._rollout_active = False
            self._rollout_done.set()

    def _ensure_timeline_playing(self) -> None:
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            raise SysIdEnvironmentBridgeError(
                "Press Play on the timeline before running SysId. Rollouts use the Kit physics loop "
                "not manual SimulationManager.step()."
            )
        self._initialize_physics("starting the rollout")

    async def _ensure_rollout_physics_ready_async(self) -> None:
        """Start physics automatically for offline stepping; require Play interactively."""
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            if not self.offline_stepping:
                self._ensure_timeline_playing()
            timeline.play()
            # Let SimulationManager create its physics and tensor views before
            # direct stepping. Any motion here is discarded by the later reset.
            await self._kit_updates_async(2)
        self._initialize_physics("starting the rollout")

    @staticmethod
    def _initialize_physics(context: str) -> None:
        """Initialize physics and preserve failures as bridge diagnostics.

        Args:
            context: Operation being attempted when initialization fails.
        """
        try:
            runtime.initialize_physics()
        except Exception as exc:
            raise SysIdEnvironmentBridgeError(f"Failed to initialize physics while {context}: {exc}") from exc

    def _resolve_articulation(self) -> Articulation:
        stage = runtime.get_current_stage()
        if stage is None:
            raise SysIdEnvironmentBridgeError("No USD stage is open.")
        root = find_articulation_root(stage, self._robot_prim_path)
        if root is None:
            raise SysIdEnvironmentBridgeError(
                f"No articulation root found under {self._robot_prim_path}. "
                "Set robot prim path to the robot Xform (e.g. .../so101_oliver_tunned), not .../Geometry/base."
            )
        self._articulation_root = root
        carb.log_info(f"SysId: articulation root '{root}'")
        if self.use_parallel_clones:
            source = PurePosixPath(self._source_env_path.rstrip("/"))
            env_root = PurePosixPath(self._env_paths_root.rstrip("/"))
            if source == env_root:
                raise SysIdEnvironmentBridgeError(
                    "Clone source env must be a specific environment (e.g. /World/envs/env_0), "
                    "not the clone env root (/World/envs/env)."
                )
            self._articulation_paths_expr = _build_cloned_articulation_regex(
                root, self._source_env_path, self._env_paths_root
            )
        else:
            self._articulation_paths_expr = root.replace(".", r"\.")
        carb.log_info(f"SysId articulation paths expr: {self._articulation_paths_expr}")
        articulation = Articulation(self._articulation_paths_expr)
        if len(articulation) < 1:
            raise SysIdEnvironmentBridgeError(
                f"No articulations matched '{self._articulation_paths_expr}'. " "Check robot path and cloning setup."
            )
        if not articulation.is_physics_tensor_entity_valid():
            self._ensure_timeline_playing()
        if not articulation.is_physics_tensor_entity_valid():
            raise SysIdEnvironmentBridgeError(
                "Physics tensor view is not valid. Press Play on the timeline and re-run SysId."
            )
        return articulation

    def _physics_replicate_root_path(self) -> str:
        """Prefix for PhysX replication (must end with ``_`` so env_1 not env1).

        Returns:
            Result produced by the operation.
        """
        root = self._env_paths_root.rstrip("/")
        return root if root.endswith("_") else f"{root}_"

    def _get_usdrt_stage(self, stage: Usd.Stage):  # noqa: ANN202
        """Attach to the Fabric stage that mirrors the active USD stage.

        Args:
            stage: USD stage used by the operation.

        Returns:
            Result produced by the operation.
        """
        import usdrt

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = cache.Insert(stage).ToLongInt()
        return usdrt.Usd.Stage.Attach(stage_id)

    def _stage_cache_id(self, stage: Usd.Stage) -> int:
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = cache.Insert(stage).ToLongInt()
        return stage_id

    def _fabric_prim_exists(self, stage: Usd.Stage, path: str) -> bool:
        try:
            prim = self._get_usdrt_stage(stage).GetPrimAtPath(path)
            return prim is not None and prim.IsValid()
        except Exception as exc:
            carb.log_warn(f"SysId: failed to query Fabric clone prim '{path}': {exc}")
            return False

    def _missing_clone_paths(self, stage: Usd.Stage, prim_paths: list[str]) -> list[str]:
        missing = []
        for path in prim_paths:
            if stage.GetPrimAtPath(path).IsValid():
                continue
            if self.fabric_clones and self._fabric_prim_exists(stage, path):
                continue
            missing.append(path)
        return missing

    def _remove_fabric_prim(self, stage: Usd.Stage, path: str) -> bool:
        try:
            import usdrt

            usdrt_stage = self._get_usdrt_stage(stage)
            prim = usdrt_stage.GetPrimAtPath(path)
            if prim is None or not prim.IsValid():
                return False
            prim_paths = []
            for child in usdrt.Usd.PrimRange(prim):
                child_path = child.GetPath()
                prim_paths.append(getattr(child_path, "pathString", str(child_path)))
            if not prim_paths:
                prim_paths = [path]
            removed = False
            for prim_path in sorted(prim_paths, key=lambda item: item.count("/"), reverse=True):
                try:
                    removed = bool(usdrt_stage.RemovePrim(prim_path)) or removed
                except Exception as child_exc:
                    carb.log_warn(f"SysId: failed to remove Fabric clone child prim '{prim_path}': {child_exc}")
            try:
                fabric_id = usdrt_stage.GetFabricId()
                hier = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
                    fabric_id, usdrt_stage.GetStageIdAsStageId()
                )
                hier.update_world_xforms()
            except Exception:
                pass
            return removed
        except Exception as exc:
            carb.log_warn(f"SysId: failed to remove Fabric clone prim '{path}': {exc}")
            return False

    def _fabric_paths_still_present(self, stage: Usd.Stage, paths: list[str]) -> list[str]:
        if not self.fabric_clones:
            return []
        present = []
        for path in paths:
            if self._fabric_prim_exists(stage, path):
                present.append(path)
        return present

    def _fabric_articulation_candidate_paths(self, count: int) -> list[str]:
        root = self._articulation_root or self._robot_prim_path
        return [self._path_for_env(root, env_i) for env_i in range(count)]

    def _create_physics_articulation_view(self, paths):  # noqa: ANN001, ANN202
        sim_view = runtime.get_physics_sim_view_warp()
        if sim_view is None or not sim_view.is_valid:
            return None, "physics simulation view is invalid"
        view = sim_view.create_articulation_view(paths)
        if view is None:
            return None, "create_articulation_view returned None"
        backend = getattr(view, "_backend", None)
        if backend is None:
            return None, "physics articulation view has no backend"
        if not view.check():
            return view, "physics articulation view check() failed"
        if not view.is_homogeneous:
            return view, "physics articulation view is not homogeneous"
        return view, "ok"

    def _physics_view_count(self, paths) -> tuple[int, str]:  # noqa: ANN001
        try:
            view, status = self._create_physics_articulation_view(paths)
            if view is None:
                return 0, status
            if status != "ok":
                return int(getattr(view, "count", 0)), status
            return int(getattr(view, "count", 0)), "ok"
        except Exception as exc:
            return 0, str(exc)

    def _log_fabric_articulation_probe(self, required_count: int, articulation: Articulation) -> None:
        if not self.fabric_clones:
            return
        wrapped_view = getattr(articulation, "_physics_articulation_view", None)
        wrapped_count = getattr(wrapped_view, "count", None)
        paths = self._fabric_articulation_candidate_paths(required_count)
        direct_count, direct_status = self._physics_view_count(paths)
        regex_count, regex_status = self._physics_view_count(self._articulation_paths_expr)
        carb.log_warn(
            "SysId Fabric articulation probe: "
            f"wrapper len={len(articulation)}, wrapper physics count={wrapped_count}, "
            f"direct path-list count={direct_count} ({direct_status}), "
            f"direct regex count={regex_count} ({regex_status}), "
            f"candidate paths={paths}"
        )

    def _refresh_articulation_metadata_from_physics_view(self, articulation: Articulation) -> None:
        view = getattr(articulation, "_physics_articulation_view", None)
        if view is None:
            return
        articulation._num_links = view.max_links
        articulation._link_names = list(view.shared_metatype.link_names)
        articulation._link_index_dict = dict(view.shared_metatype.link_indices)
        articulation._link_paths = view.link_paths
        articulation._num_joints = view.shared_metatype.joint_count
        articulation._joint_names = list(view.shared_metatype.joint_names)
        articulation._joint_index_dict = dict(view.shared_metatype.joint_indices)
        articulation._joint_types = list(view.shared_metatype.joint_types)
        articulation._num_dofs = view.max_dofs
        articulation._dof_names = list(view.shared_metatype.dof_names)
        articulation._dof_index_dict = dict(view.shared_metatype.dof_indices)
        articulation._dof_paths = view.dof_paths
        articulation._dof_types = list(view.shared_metatype.dof_types)
        articulation._num_shapes = view.max_shapes
        articulation._num_fixed_tendons = view.max_fixed_tendons
        articulation._physics_tensor_entity_initialized = True

    def _try_rebind_fabric_articulation_view(self, articulation: Articulation, required_count: int) -> bool:
        if not self.fabric_clones:
            return False
        paths = self._fabric_articulation_candidate_paths(required_count)
        try:
            view, status = self._create_physics_articulation_view(paths)
        except Exception as exc:
            carb.log_warn(f"SysId: Fabric articulation rebind failed while creating direct view: {exc}")
            return False
        count = int(getattr(view, "count", 0)) if view is not None else 0
        if view is None or status != "ok" or count < required_count:
            carb.log_warn(
                "SysId: Fabric articulation rebind skipped: "
                f"direct path-list count={count}, status={status}, need {required_count}."
            )
            return False
        stage = runtime.get_current_stage()
        if stage is None:
            return False
        articulation._raw_paths = list(paths)
        articulation._paths = list(paths)
        articulation._prims = [stage.GetPrimAtPath(path) for path in paths]
        articulation._physics_articulation_view = view
        self._refresh_articulation_metadata_from_physics_view(articulation)
        message = (
            "SysId: rebound Fabric articulation wrapper to direct PhysX tensor view "
            f"with {count} articulation(s). USD prim handles for Fabric clones remain invalid."
        )
        if self._fabric_rebind_logged:
            carb.log_info(message)
        else:
            carb.log_warn(message)
            self._fabric_rebind_logged = True
        return True

    async def _clone_environments_async(self, num_envs: int) -> None:
        """Clone env_0..env_{N-1}. Timeline is stopped during USD edits to avoid CUDA 700.

        Args:
            num_envs: Value supplied for ``num_envs``.
        """
        stage = runtime.get_current_stage()
        if stage is None:
            raise SysIdEnvironmentBridgeError("No USD stage is open.")
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        self._rollout_active = False
        if was_playing:
            carb.log_info("SysId: stopping timeline while cloning parallel environments...")
            timeline.stop()
            await self._kit_updates_async(4)

        env_parent = str(Sdf.Path(self._env_paths_root).GetParentPath())
        replicate_root = self._physics_replicate_root_path()
        for i in range(1, num_envs):
            bad_path = f"{env_parent}/env{i}"
            if stage.GetPrimAtPath(bad_path).IsValid():
                carb.log_warn(
                    f"SysId: misnamed prim '{bad_path}' (missing underscore). "
                    f"Delete it in the Stage tree; correct path is '{env_parent}/env_{i}'."
                )
        cloner = GridCloner(spacing=self._clone_spacing)
        cloner.define_base_env(self._source_env_path)
        prim_paths = cloner.generate_paths(self._env_paths_root, num_envs)
        if not stage.GetPrimAtPath(self._source_env_path).IsValid():
            raise SysIdEnvironmentBridgeError(
                f"Source environment '{self._source_env_path}' does not exist. "
                "Create env_0 or adjust source_env_path."
            )
        self._ensure_gpu_dynamics_capacity(stage, num_envs)
        replication_blocker = self._physics_replication_blocker(stage)
        use_physics_replication = self.co_locate_clones and replication_blocker is None
        if self.co_locate_clones and replication_blocker is not None:
            carb.log_warn(
                "SysId: co-located PhysX replication disabled for this robot because "
                f"{replication_blocker}. Falling back to full USD clones."
            )
        layout = (
            "co-located (fast)" if use_physics_replication else f"grid spacing {self._clone_spacing} m (full USD copy)"
        )
        fabric_mode = ", Fabric clone authoring" if self.fabric_clones else ""
        carb.log_info(f"SysId: cloning to {prim_paths} ({layout}{fabric_mode}, replicate root '{replicate_root}')")
        if (
            self._last_clone_co_locate is not None
            and self._last_clone_co_locate != use_physics_replication
            and self._cloned_for_count > 0
        ):
            carb.log_warn(
                "SysId: clone replication mode changed since last clone. Restart Isaac Sim or delete "
                f"stale env prims under '{self._env_paths_root}' before re-running."
            )
        cloner.clone(
            source_prim_path=self._source_env_path,
            prim_paths=prim_paths,
            replicate_physics=use_physics_replication,
            copy_from_source=not use_physics_replication,
            base_env_path=env_parent,
            root_path=replicate_root,
            enable_env_ids=use_physics_replication,
            clone_in_fabric=bool(self.fabric_clones),
        )
        self._last_clone_co_locate = use_physics_replication
        self._physics_replication_registered = bool(use_physics_replication)
        missing = self._missing_clone_paths(stage, prim_paths)
        if missing:
            raise SysIdEnvironmentBridgeError(
                f"GridCloner did not create: {missing}. Check clone env root "
                f"('{self._env_paths_root}') and source env ('{self._source_env_path}')."
            )
        self._flush_physics_changes()
        await self._kit_updates_async(12)
        self._initialize_physics("finalizing cloned environments")
        self._flush_physics_changes()
        carb.log_info(f"SysId: cloned {num_envs} environments under {self._env_paths_root}")
        self._created_clone_paths = list(prim_paths[1:])
        if was_playing:
            timeline.play()
            await self._kit_updates_async(8)
            self._initialize_physics("resuming the timeline after cloning")

    async def ensure_num_envs_async(self, num_envs: int) -> None:
        """Prepare articulation view (clone only when ``use_parallel_clones`` is True).

        Args:
            num_envs: Value supplied for ``num_envs``.
        """
        if num_envs < 1:
            raise SysIdEnvironmentBridgeError("num_envs must be >= 1.")
        self._logical_env_count = num_envs
        clone_target = num_envs if self.use_parallel_clones else 1
        if self.use_parallel_clones and self._cloned_for_count < clone_target:
            carb.log_info(f"SysId: cloning {clone_target} environments (env_0 .. env_{clone_target - 1})")
            self._close_explicit_pd_compat()
            await self._clone_environments_async(clone_target)
            self._cloned_for_count = clone_target
            self._articulation = None
            self._baselines_captured = False
            self._baseline_static_friction = None
            self._baseline_armature = None
        self._ensure_timeline_playing()
        required_match = clone_target if self.use_parallel_clones else 1
        if self._has_ready_articulation(required_match):
            self._ensure_explicit_pd_compat()
            self._validate_pending_parameter_entries()
            return
        articulation = self._resolve_articulation()
        current = len(articulation)
        if self.use_parallel_clones and current < required_match:
            if self._try_rebind_fabric_articulation_view(articulation, required_match):
                current = len(articulation)
        carb.log_info(f"SysId: articulation view matched {current} instance(s), need {required_match}")
        if self.use_parallel_clones and current < required_match:
            self._log_fabric_articulation_probe(required_match, articulation)
            raise SysIdEnvironmentBridgeError(
                f"Found {current} env(s) but need {required_match}. Remove stale clones under "
                f"'{self._env_paths_root}' (look for env_1 not env1) or restart Isaac Sim."
            )
        if len(articulation) < 1:
            raise SysIdEnvironmentBridgeError(f"No articulations matched '{self._articulation_paths_expr}'.")
        self._articulation = articulation
        if self.use_parallel_clones and not articulation.is_physics_tensor_entity_valid():
            raise SysIdEnvironmentBridgeError(
                "Parallel clone physics view is invalid after cloning. Restart Isaac Sim, delete stale "
                f"prims under '{self._env_paths_root}', disable parallel clones, or uncheck co-locate clones "
                "(SO101-style nested Geometry/base robots often need spaced grid copies)."
            )
        self._ensure_explicit_pd_compat()
        # Friction is only read from the backend when it is selected, so a bridge
        # reused with a newly selected friction parameter must capture again.
        needs_friction = SysIdParameterType.JOINT_FRICTION in {
            entry.param_type for entry in self._pending_param_entries
        }
        if not self._baselines_captured or (needs_friction and self._baseline_static_friction is None):
            self._capture_baselines()
        self._validate_pending_parameter_entries()

    def _capture_baselines(self) -> None:
        """Snapshot friction, PD gains, armatures, and link masses for optimization."""
        art = self._articulation
        n = self._physics_env_count()
        dof_idx = self._joint_dof_indices
        selected_types = {entry.param_type for entry in self._pending_param_entries}
        capture_friction = SysIdParameterType.JOINT_FRICTION in selected_types
        with self._articulation_backend_context():
            if capture_friction:
                try:
                    static_fric, dyn_fric, _ = art.get_dof_friction_properties(
                        indices=list(range(n)), dof_indices=dof_idx
                    )
                except Exception as exc:
                    raise SysIdEnvironmentBridgeError(
                        f"Physics backend {self.physics_backend!r} cannot read the selected joint-friction parameters: {exc}"
                    ) from exc
            else:
                # Leave friction unset rather than inventing a baseline: a
                # fabricated value would overwrite the authored USD friction
                # through `refresh_parameter_space_baselines`.
                static_fric = dyn_fric = None
            stiffnesses, dampings = art.get_dof_gains(indices=list(range(n)), dof_indices=dof_idx)
            try:
                armatures = art.get_dof_armatures(indices=list(range(n)), dof_indices=dof_idx)
            except Exception:
                armatures = None
            masses = art.get_link_masses(indices=list(range(n)))
            try:
                com_pos, com_orientation = art.get_link_coms(indices=[0])
            except Exception:
                com_pos = None
                com_orientation = None
            try:
                inertias = art.get_link_inertias(indices=[0])
            except Exception:
                inertias = None
            lower, upper = art.get_dof_limits(indices=list(range(n)), dof_indices=dof_idx)
        if static_fric is None or dyn_fric is None:
            self._baseline_dynamic_friction = None
            self._baseline_static_friction = None
        else:
            static_np = _warp_array_to_numpy(static_fric)
            dyn_np = _warp_array_to_numpy(dyn_fric)
            if static_np.ndim == 1:
                static_np = static_np.reshape(n, -1)
            if dyn_np.ndim == 1:
                dyn_np = dyn_np.reshape(n, -1)
            self._baseline_dynamic_friction = np.maximum(dyn_np, 1e-6)
            self._baseline_static_friction = np.maximum(static_np, self._baseline_dynamic_friction * 1.01)
        stiff_np = _warp_array_to_numpy(stiffnesses)
        damp_np = _warp_array_to_numpy(dampings)
        if stiff_np.ndim == 1:
            stiff_np = stiff_np.reshape(n, -1)
        if damp_np.ndim == 1:
            damp_np = damp_np.reshape(n, -1)
        if self._explicit_pd_compat is not None:
            # Only explicit DOFs have their implicit gains zeroed. Preserve the
            # drive baselines for all other joints in a mixed articulation.
            stiff_np[:, self._explicit_dof_columns] = self._explicit_pd_compat.baseline_kp
            damp_np[:, self._explicit_dof_columns] = self._explicit_pd_compat.baseline_kd
        self._baseline_stiffness = np.maximum(stiff_np, 1e-6)
        self._baseline_damping = np.maximum(damp_np, 1e-6)
        if armatures is not None:
            arm_np = _warp_array_to_numpy(armatures)
            if arm_np.ndim == 1:
                arm_np = arm_np.reshape(n, -1)
            self._baseline_armature = np.maximum(arm_np, 0.0)
        else:
            self._baseline_armature = np.zeros((n, len(dof_idx)), dtype=np.float32)
        mass_np = _warp_array_to_numpy(masses)
        if mass_np.ndim == 1:
            mass_np = mass_np.reshape(n, -1)
        self._baseline_link_masses = np.maximum(mass_np, 1e-6)
        self._num_links = self._baseline_link_masses.shape[1]

        if com_pos is not None:
            com_np = _warp_array_to_numpy(com_pos)
            if com_np.ndim == 1:
                com_np = com_np.reshape(1, -1, 3)
            com_np = com_np[0].astype(np.float32)
            if not np.all(np.isfinite(com_np)):
                raise SysIdEnvironmentBridgeError(
                    "Articulation returned non-finite baseline link COM values; refusing to optimize COM offsets."
                )
            self._baseline_link_com = com_np
            orientation_np = _warp_array_to_numpy(com_orientation)
            if orientation_np.ndim == 1:
                orientation_np = orientation_np.reshape(1, -1, 4)
            orientation_np = orientation_np[0].astype(np.float32)
            orientation_norm = np.linalg.norm(orientation_np, axis=-1, keepdims=True)
            if (
                orientation_np.shape != (self._num_links, 4)
                or not np.all(np.isfinite(orientation_np))
                or np.any(orientation_norm <= 1e-8)
            ):
                raise SysIdEnvironmentBridgeError(
                    "Articulation returned invalid baseline link COM orientations; " "refusing to optimize COM offsets."
                )
            self._baseline_link_com_orientation = orientation_np / orientation_norm
        else:
            self._baseline_link_com = np.zeros((self._num_links, 3), dtype=np.float32)
            self._baseline_link_com_orientation = np.tile(
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                (self._num_links, 1),
            )

        if inertias is not None:
            inertia_np = _warp_array_to_numpy(inertias)
            if inertia_np.ndim == 1:
                inertia_np = inertia_np.reshape(1, -1, 9)
            inertia_np = inertia_np[0].astype(np.float32)
            inertia_np = np.where(np.isfinite(inertia_np), inertia_np, 0.0).astype(np.float32)
            for link_idx in range(inertia_np.shape[0]):
                flat = inertia_np[link_idx].reshape(3, 3)
                sym = 0.5 * (flat + flat.T)
                if not np.all(np.isfinite(sym)):
                    sym = np.eye(3, dtype=np.float32)
                try:
                    eig = np.linalg.eigvalsh(sym.astype(np.float64))
                except np.linalg.LinAlgError:
                    eig = np.asarray([-1.0], dtype=np.float64)
                if np.any(eig <= 0.0):
                    sym = np.eye(3, dtype=np.float32)
                inertia_np[link_idx] = sym.reshape(9)
            self._baseline_link_inertia = inertia_np
            self._baseline_link_inertia_lc = {}
            for link_idx in range(self._num_links):
                flat = self._baseline_link_inertia[link_idx].reshape(3, 3)
                self._baseline_link_inertia_lc[link_idx] = inertia_matrix_to_log_cholesky(flat)
        else:
            self._baseline_link_inertia = None
            self._baseline_link_inertia_lc = {}

        lower_np = _warp_array_to_numpy(lower)
        upper_np = _warp_array_to_numpy(upper)
        if lower_np.ndim == 1:
            lower_np = lower_np.reshape(n, -1)
            upper_np = upper_np.reshape(n, -1)
        self._baseline_dof_lower = lower_np[0].astype(np.float32)
        self._baseline_dof_upper = upper_np[0].astype(np.float32)

        self._link_prim_paths = self._collect_link_prim_paths()
        self._baselines_captured = True

    def _source_articulation_index(self) -> int:
        art = self._articulation
        if art is None:
            return 0
        try:
            paths = [str(path) for path in getattr(art, "paths", [])]
        except Exception:
            paths = []
        source = self._source_env_path.rstrip("/")
        root = self._articulation_root or self._robot_prim_path
        for idx, path in enumerate(paths):
            if path == root or path == source or path.startswith(source + "/"):
                return idx
        return 0

    def _source_row(self, rows) -> list[str]:  # noqa: ANN001
        if not rows:
            return []
        if not isinstance(rows[0], (list, tuple)):
            return [str(path) for path in rows]
        idx = min(self._source_articulation_index(), len(rows) - 1)
        return [str(path) for path in rows[idx]]

    def _source_baseline_row(self, values: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values)
        if arr.ndim > 1:
            idx = min(self._source_articulation_index(), arr.shape[0] - 1)
            return arr[idx]
        return arr

    def source_parameter_paths(self) -> tuple[list[str], list[str]]:
        """Return link and DOF prim paths in the same source-articulation order used by rollouts.

        Returns:
            Result produced by the operation.
        """
        art = self._articulation
        if art is None:
            return list(self._link_prim_paths), []
        try:
            link_paths = self._source_row(art.link_paths)
        except Exception:
            link_paths = list(self._link_prim_paths)
        try:
            dof_paths_all = self._source_row(art.dof_paths)
        except Exception:
            dof_paths_all = []
        selected = self._joint_dof_indices or list(range(len(dof_paths_all)))
        dof_paths = [dof_paths_all[idx] for idx in selected if 0 <= idx < len(dof_paths_all)]
        return link_paths, dof_paths

    def refresh_parameter_space_baselines(self, space, stage: Usd.Stage) -> tuple[list[str], list[str]]:  # noqa: ANN001
        """Align USD writeback baselines with the live articulation baselines used during rollout.

        Args:
            space: Value supplied for ``space``.
            stage: USD stage used by the operation.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        link_paths, dof_paths = self.source_parameter_paths()
        if link_paths or dof_paths:
            space.read_usd_baselines(stage, link_paths, dof_paths, physics_backend=self.physics_backend)
        override = getattr(space, "override_joint_runtime_baselines", None)
        if callable(override):
            override(
                static_friction=self._source_baseline_row(self._baseline_static_friction),
                dynamic_friction=self._source_baseline_row(self._baseline_dynamic_friction),
                stiffness=self._source_baseline_row(self._baseline_stiffness),
                damping=self._source_baseline_row(self._baseline_damping),
                armature=self._source_baseline_row(self._baseline_armature),
                preserve_usd_drive_gains=(self._uses_explicit_actuators),
            )
        override_links = getattr(space, "override_link_runtime_baselines", None)
        if callable(override_links):
            override_links(
                masses=self._source_baseline_row(self._baseline_link_masses),
                coms=self._baseline_link_com,
                inertias=self._baseline_link_inertia,
            )
        return link_paths, dof_paths

    def write_parameters_to_usd(
        self,
        stage: Usd.Stage,
        param_entries: list[SysIdParameterEntry],
        theta_row: torch.Tensor,
    ) -> bool:
        """Persist explicit-PD parameters to Newton actuator schema attributes.

        Args:
            stage: USD stage that owns the actuator schemas.
            param_entries: Parameter definitions corresponding to ``theta_row``.
            theta_row: Parameter values to persist.

        Returns:
            Whether all applicable parameters were written successfully.
        """
        if not self._uses_explicit_actuators:
            return True
        selected_types = {entry.param_type for entry in param_entries}
        claimed = selected_types & (self.usd_parameter_types | set(self.usd_parameter_dof_claims))
        if not claimed:
            return True
        compat = self._explicit_pd_compat
        if compat is None or self._baseline_stiffness is None or self._baseline_damping is None:
            return False
        state = decode_theta_to_apply_state(
            num_dof=self._num_joints,
            num_links=max(1, self._num_links),
            param_entries=param_entries,
            theta_row=theta_row,
            baseline_link_inertia_lc=self._baseline_link_inertia_lc,
        )
        columns = self._explicit_dof_columns
        kp = kd = ki = delay_seconds = None
        if SysIdParameterType.JOINT_STIFFNESS in selected_types:
            kp = compat.baseline_kp[0] * state.stiffness_scale[columns]
        if SysIdParameterType.JOINT_DAMPING in selected_types:
            kd = compat.baseline_kd[0] * state.damping_scale[columns]
        if SysIdParameterType.JOINT_INTEGRAL_GAIN in selected_types and state.joint_integral_gain is not None:
            baseline_ki = compat.baseline_ki[0]
            values = state.joint_integral_gain[columns]
            ki = np.where(np.isfinite(values), values, baseline_ki)
        if any(is_actuator_delay_parameter(value) for value in selected_types):
            baseline_seconds = compat.baseline_delay_steps[0].astype(np.float32) * compat.physics_dt
            delay_seconds = (
                baseline_seconds
                if state.actuator_command_delay_seconds is None
                else np.where(
                    np.isfinite(state.actuator_command_delay_seconds[columns]),
                    state.actuator_command_delay_seconds[columns],
                    baseline_seconds,
                )
            )
        return compat.write_parameters_to_usd(
            stage,
            kp=kp,
            kd=kd,
            ki=ki,
            delay_seconds=delay_seconds,
        )

    def apply_accepted_parameters_to_live_articulation(
        self,
        theta_row: list[float] | torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> bool:
        """Apply and verify accepted parameters on an implicit-drive PhysX articulation.

        Args:
            theta_row: Accepted optimizer parameter values.
            param_entries: Parameter definitions corresponding to ``theta_row``.

        Returns:
            Whether the accepted values remain applied and were verified. Explicit
            actuator candidates return ``False`` because their temporary runtime
            manager is removed during run cleanup.

        Raises:
            SysIdEnvironmentBridgeError: If the live articulation cannot be updated or verified.
        """
        if self.physics_backend != PHYSICS_BACKEND_PHYSX or self._uses_explicit_actuators:
            return False
        self._apply_and_verify_live_parameters(theta_row, param_entries)
        return True

    def restore_parameters_on_live_articulation(
        self,
        theta_row: list[float] | torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> bool:
        """Restore parameters after a failed implicit-drive PhysX writeback.

        Args:
            theta_row: Authored parameter values captured before optimization.
            param_entries: Parameter definitions corresponding to ``theta_row``.

        Returns:
            Whether the authored values were restored and verified.
        """
        if self.physics_backend != PHYSICS_BACKEND_PHYSX or self._uses_explicit_actuators:
            return False
        self._apply_and_verify_live_parameters(theta_row, param_entries)
        return True

    def _apply_and_verify_live_parameters(
        self,
        theta_row: list[float] | torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> None:
        """Apply one parameter vector to the live articulation and verify gains.

        Args:
            theta_row: Parameter values to apply.
            param_entries: Parameter definitions corresponding to ``theta_row``.
        """
        if self._articulation is None:
            raise SysIdEnvironmentBridgeError("The live PhysX articulation is unavailable for accepted writeback.")
        theta = torch.as_tensor(theta_row, device=self._device, dtype=torch.float32).reshape(1, -1)
        self.apply_parameter_vector(theta, param_entries)
        self._flush_physics_changes()
        self._verify_live_parameters(theta[0], param_entries)

    def _verify_live_parameters(
        self,
        theta_row: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> None:
        """Verify every selected implicit PhysX parameter through live readback.

        Args:
            theta_row: Accepted optimizer parameter values.
            param_entries: Parameter definitions corresponding to ``theta_row``.

        Raises:
            SysIdEnvironmentBridgeError: If any live readback differs from the accepted values.
        """
        selected_types = {entry.param_type for entry in param_entries}
        if not selected_types:
            return
        state = decode_theta_to_apply_state(
            num_dof=len(self._joint_dof_indices),
            num_links=max(self._num_links, 1),
            param_entries=param_entries,
            theta_row=theta_row.detach().cpu().numpy(),
            baseline_link_inertia_lc=self._baseline_link_inertia_lc,
        )
        failures: list[str] = []

        def require_baseline(name: str, value: np.ndarray | None) -> np.ndarray | None:
            if value is None:
                failures.append(f"{name} baseline is unavailable")
            return value

        def source_row(value: object, expected: np.ndarray) -> np.ndarray:
            expected_array = np.asarray(expected)
            return _warp_array_to_numpy(value).reshape((-1, *expected_array.shape))[0]

        def compare(label: str, actual: float, expected: float) -> None:
            if not np.isclose(actual, expected, rtol=1e-5, atol=1e-6):
                failures.append(f"{label} expected {expected:.9g}, live {actual:.9g}")

        def selected_dofs(param_type: SysIdParameterType) -> set[int]:
            return {
                dof_index
                for entry in param_entries
                if entry.param_type == param_type
                for dof_index in (
                    range(len(self._joint_dof_indices)) if int(entry.dof_index) < 0 else (int(entry.dof_index),)
                )
            }

        def selected_links(param_types: set[SysIdParameterType]) -> set[int]:
            return {
                link_index
                for entry in param_entries
                if entry.param_type in param_types
                for link_index in (
                    range(max(self._num_links, 1)) if int(entry.link_index) < 0 else (int(entry.link_index),)
                )
            }

        art = self._articulation
        dof_indices = self._joint_dof_indices
        num_links = max(self._num_links, 1)
        try:
            with self._articulation_backend_context():
                if SysIdParameterType.JOINT_FRICTION in selected_types:
                    baseline_dynamic = require_baseline("joint friction", self._baseline_dynamic_friction)
                    baseline_static = require_baseline("joint friction", self._baseline_static_friction)
                    if baseline_dynamic is not None and baseline_static is not None:
                        static, dynamic, _viscous = art.get_dof_friction_properties(
                            indices=[0], dof_indices=dof_indices
                        )
                        expected_dynamic = baseline_dynamic[0] * state.friction_scale
                        expected_static = np.maximum(
                            baseline_static[0] * state.friction_scale,
                            expected_dynamic * 1.01,
                        )
                        actual_dynamic = source_row(dynamic, expected_dynamic)
                        actual_static = source_row(static, expected_static)
                        for index in sorted(selected_dofs(SysIdParameterType.JOINT_FRICTION)):
                            compare(f"joint_friction.dynamic[{index}]", actual_dynamic[index], expected_dynamic[index])
                            compare(f"joint_friction.static[{index}]", actual_static[index], expected_static[index])

                gain_types = selected_types & {
                    SysIdParameterType.JOINT_STIFFNESS,
                    SysIdParameterType.JOINT_DAMPING,
                }
                if gain_types:
                    baseline_stiffness = require_baseline("joint gain", self._baseline_stiffness)
                    baseline_damping = require_baseline("joint gain", self._baseline_damping)
                    if baseline_stiffness is not None and baseline_damping is not None:
                        stiffness, damping = art.get_dof_gains(indices=[0], dof_indices=dof_indices)
                        expected_stiffness = baseline_stiffness[0] * state.stiffness_scale
                        expected_damping = baseline_damping[0] * state.damping_scale
                        actual_stiffness = source_row(stiffness, expected_stiffness)
                        actual_damping = source_row(damping, expected_damping)
                        for param_type, actual_values, expected_values in (
                            (SysIdParameterType.JOINT_STIFFNESS, actual_stiffness, expected_stiffness),
                            (SysIdParameterType.JOINT_DAMPING, actual_damping, expected_damping),
                        ):
                            for index in sorted(selected_dofs(param_type)):
                                compare(f"{param_type.value}[{index}]", actual_values[index], expected_values[index])

                if SysIdParameterType.JOINT_ARMATURE in selected_types:
                    baseline_armature = require_baseline("joint armature", self._baseline_armature)
                    if baseline_armature is not None and state.joint_armature is not None:
                        expected_armature = baseline_armature[0].copy()
                        mask = np.isfinite(state.joint_armature)
                        expected_armature[mask] = np.maximum(state.joint_armature[mask], 0.0)
                        actual_armature = source_row(
                            art.get_dof_armatures(indices=[0], dof_indices=dof_indices),
                            expected_armature,
                        )
                        for index in sorted(selected_dofs(SysIdParameterType.JOINT_ARMATURE)):
                            compare(f"joint_armature[{index}]", actual_armature[index], expected_armature[index])

                if SysIdParameterType.LINK_MASS in selected_types:
                    baseline_masses = require_baseline("link mass", self._baseline_link_masses)
                    if baseline_masses is not None:
                        expected_masses = np.maximum(
                            baseline_masses[0, :num_links] * state.link_mass_scale[:num_links],
                            1e-6,
                        )
                        actual_masses = source_row(art.get_link_masses(indices=[0]), expected_masses)
                        for index in sorted(selected_links({SysIdParameterType.LINK_MASS})):
                            compare(f"link_mass[{index}]", actual_masses[index], expected_masses[index])

                com_types = selected_types & {
                    SysIdParameterType.LINK_COM_OFFSET_X,
                    SysIdParameterType.LINK_COM_OFFSET_Y,
                    SysIdParameterType.LINK_COM_OFFSET_Z,
                }
                if com_types:
                    baseline_com = require_baseline("link COM", self._baseline_link_com)
                    if baseline_com is not None:
                        expected_com = baseline_com[:num_links] + state.link_com_delta[:num_links]
                        actual_com, _orientations = art.get_link_coms(indices=[0])
                        actual_com = source_row(actual_com, expected_com)
                        axes = {
                            SysIdParameterType.LINK_COM_OFFSET_X: 0,
                            SysIdParameterType.LINK_COM_OFFSET_Y: 1,
                            SysIdParameterType.LINK_COM_OFFSET_Z: 2,
                        }
                        for param_type in com_types:
                            axis = axes[param_type]
                            for index in sorted(selected_links({param_type})):
                                compare(
                                    f"{param_type.value}[{index}]",
                                    actual_com[index, axis],
                                    expected_com[index, axis],
                                )

                if SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY in selected_types:
                    baseline_inertia = require_baseline("link inertia", self._baseline_link_inertia)
                    if baseline_inertia is not None:
                        expected_inertia = baseline_inertia[:num_links].copy()
                        for index, values in state.link_inertia_flat.items():
                            if 0 <= index < num_links:
                                expected_inertia[index] = values
                        actual_inertia = source_row(art.get_link_inertias(indices=[0]), expected_inertia)
                        for index in sorted(selected_links({SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY})):
                            for component, (actual, expected) in enumerate(
                                zip(actual_inertia[index].reshape(-1), expected_inertia[index].reshape(-1))
                            ):
                                compare(f"link_inertia[{index},{component}]", actual, expected)

                limit_types = selected_types & {
                    SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
                    SysIdParameterType.JOINT_LIMIT_UPPER_SCALE,
                }
                if limit_types:
                    baseline_lower = require_baseline("joint limit", self._baseline_dof_lower)
                    baseline_upper = require_baseline("joint limit", self._baseline_dof_upper)
                    if baseline_lower is not None and baseline_upper is not None:
                        expected_lower = baseline_lower * state.joint_limit_lower_scale
                        expected_upper = baseline_upper * state.joint_limit_upper_scale
                        lower, upper = art.get_dof_limits(indices=[0], dof_indices=dof_indices)
                        actual_lower = source_row(lower, expected_lower)
                        actual_upper = source_row(upper, expected_upper)
                        for param_type, actual_values, expected_values in (
                            (SysIdParameterType.JOINT_LIMIT_LOWER_SCALE, actual_lower, expected_lower),
                            (SysIdParameterType.JOINT_LIMIT_UPPER_SCALE, actual_upper, expected_upper),
                        ):
                            for index in sorted(selected_dofs(param_type)):
                                compare(f"{param_type.value}[{index}]", actual_values[index], expected_values[index])
        except SysIdEnvironmentBridgeError:
            raise
        except Exception as exc:
            raise SysIdEnvironmentBridgeError(f"Could not read back accepted PhysX parameters: {exc}") from exc
        if failures:
            raise SysIdEnvironmentBridgeError(
                "Accepted PhysX parameters did not survive live readback: " + "; ".join(failures)
            )

    def _collect_link_prim_paths(self) -> list[str]:
        stage = runtime.get_current_stage()
        if stage is None or not self._articulation_root:
            return []
        root_prim = stage.GetPrimAtPath(self._articulation_root)
        if not root_prim or not root_prim.IsValid():
            return []
        paths: list[str] = []
        stack = [root_prim]
        while stack:
            prim = stack.pop(0)
            try:
                if (
                    prim.HasAPI(UsdPhysics.RigidBodyAPI)
                    or prim.HasAPI(UsdPhysics.MassAPI)
                    or prim.HasAPI(PhysxSchema.PhysxCollisionAPI)
                ):
                    paths.append(str(prim.GetPath()))
            except Exception:
                pass
            stack.extend(list(prim.GetChildren()))
        return paths[: self._num_links]

    def _path_for_env(self, source_path: str, env_i: int) -> str:
        if env_i == 0 or not self.use_parallel_clones:
            return source_path
        source = self._source_env_path.rstrip("/")
        if not source_path.startswith(source):
            return source_path
        env_root = PurePosixPath(self._env_paths_root.rstrip("/"))
        env_path = f"{env_root.parent.as_posix()}/{env_root.name}_{env_i}"
        return env_path + source_path[len(source) :]

    def _reapply_trajectory_initial_state(self) -> None:
        """Write measured t=0 joint state to all clones (used after physics warmup)."""
        if self._trajectory is None or self._articulation is None:
            return
        art = self._articulation
        n = self._physics_env_count()
        dof_idx = self._joint_dof_indices
        q0 = np.asarray(self._trajectory.positions[0], dtype=np.float32)
        dq0 = np.asarray(self._trajectory.velocities[0], dtype=np.float32)
        if not np.all(np.isfinite(q0)) or not np.all(np.isfinite(dq0)):
            raise SysIdEnvironmentBridgeError("Trajectory start position/velocity contains NaN or Inf.")
        with self._articulation_backend_context():
            lower, upper = art.get_dof_limits(indices=list(range(n)), dof_indices=dof_idx)
        lower_np = _warp_array_to_numpy(lower)
        upper_np = _warp_array_to_numpy(upper)
        if lower_np.ndim == 1:
            lower_np = lower_np.reshape(n, -1)
            upper_np = upper_np.reshape(n, -1)
        q_clipped = np.clip(q0, lower_np[0], upper_np[0]).astype(np.float32)
        dq_clipped = np.clip(dq0, -50.0, 50.0).astype(np.float32)
        q_targets = np.tile(q_clipped, (n, 1))
        dq_targets = np.tile(dq_clipped, (n, 1))
        indices = list(range(n))
        direct_pos = self._fabric_direct_write_dofs(
            "positions", q_targets, indices=indices, dof_indices=dof_idx, also_target=True
        )
        direct_vel = self._fabric_direct_write_dofs(
            "velocities", dq_targets, indices=indices, dof_indices=dof_idx, also_target=True
        )
        if direct_pos and direct_vel and self._diagnostics_enabled:
            rows = self._fabric_direct_read_dofs("positions", indices=indices, dof_indices=dof_idx)
            self._log_fabric_direct_joint_rows("after trajectory init", rows)
        elif not (direct_pos and direct_vel):
            with self._articulation_backend_context():
                art.set_dof_positions(q_targets, indices=indices, dof_indices=dof_idx)
                art.set_dof_velocities(dq_targets, indices=indices, dof_indices=dof_idx)
                art.set_dof_position_targets(q_targets, indices=indices, dof_indices=dof_idx)
                art.set_dof_velocity_targets(dq_targets, indices=indices, dof_indices=dof_idx)
        self._flush_physics_changes()

    def reset_to_trajectory_start(self) -> None:
        """Reset joints to a stable initial state from the trajectory (clipped to limits)."""
        if self._trajectory is None:
            raise SysIdEnvironmentBridgeError("Call set_trajectory() before running optimization.")
        if self._articulation is None:
            raise SysIdEnvironmentBridgeError("Call ensure_num_envs() before reset.")
        art = self._articulation
        n = self._physics_env_count()
        try:
            with self._articulation_backend_context():
                art.reset_to_default_state(indices=list(range(n)))
        except Exception:
            pass
        self._reapply_trajectory_initial_state()
        if self._explicit_pd_compat is not None:
            self._explicit_pd_compat.reset()

    def apply_parameter_vector(
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> None:
        """Apply per-clone parameter scales and extended link/joint properties.

        Args:
            theta_per_env: Value supplied for ``theta_per_env``.
            param_entries: Value supplied for ``param_entries``.
        """
        if self._articulation is None:
            raise SysIdEnvironmentBridgeError("Call ensure_num_envs() before apply_parameter_vector.")
        art = self._articulation
        n_phys = self._physics_env_count()
        n_theta = theta_per_env.shape[0]
        if n_theta < n_phys:
            theta_apply = theta_per_env[0:1].expand(n_phys, -1).clone()
        else:
            theta_apply = theta_per_env
        n_dof = len(self._joint_dof_indices)
        dof_idx = self._joint_dof_indices
        num_links = max(self._num_links, 1)

        theta_host = theta_apply.detach().to(device="cpu", dtype=torch.float32).numpy()
        apply_states = [
            decode_theta_to_apply_state(
                num_dof=n_dof,
                num_links=num_links,
                param_entries=param_entries,
                theta_row=theta_host[env_i],
                baseline_link_inertia_lc=self._baseline_link_inertia_lc,
            )
            for env_i in range(n_phys)
        ]
        indices = list(range(n_phys))
        selected_types = {entry.param_type for entry in param_entries}
        gain_selected = bool(selected_types & {SysIdParameterType.JOINT_STIFFNESS, SysIdParameterType.JOINT_DAMPING})

        static = dynamic = None
        if SysIdParameterType.JOINT_FRICTION in selected_types:
            friction_scale = np.stack([state.friction_scale for state in apply_states], axis=0)
            dynamic = (self._baseline_dynamic_friction[:n_phys] * friction_scale).astype(np.float32)
            static = (self._baseline_static_friction[:n_phys] * friction_scale).astype(np.float32)
            static = np.maximum(static, dynamic * 1.01)

        stiffness_scale = None
        damping_scale = None
        if gain_selected or self._explicit_pd_compat is not None:
            stiffness_scale = np.stack([state.stiffness_scale for state in apply_states], axis=0)
            damping_scale = np.stack([state.damping_scale for state in apply_states], axis=0)
        if self._explicit_pd_compat is not None:
            if self._explicit_pd_compat.env_count != n_phys:
                raise SysIdEnvironmentBridgeError(
                    f"Explicit actuator manager has {self._explicit_pd_compat.env_count} environments, "
                    f"but the rollout is applying {n_phys}."
                )
            have_delay = any(state.actuator_command_delay_seconds is not None for state in apply_states)
            delay_rows = None
            if have_delay:
                delay_rows = np.stack(
                    [
                        (
                            state.actuator_command_delay_seconds
                            if state.actuator_command_delay_seconds is not None
                            else np.full(n_dof, np.nan, dtype=np.float32)
                        )
                        for state in apply_states
                    ],
                    axis=0,
                )
            try:
                columns = self._explicit_dof_columns
                integral_rows = None
                if any(state.joint_integral_gain is not None for state in apply_states):
                    integral_rows = np.stack(
                        [
                            (
                                np.where(
                                    np.isfinite(state.joint_integral_gain[columns]),
                                    state.joint_integral_gain[columns],
                                    self._explicit_pd_compat.baseline_ki[env_i],
                                )
                                if state.joint_integral_gain is not None
                                else self._explicit_pd_compat.baseline_ki[env_i]
                            )
                            for env_i, state in enumerate(apply_states)
                        ],
                        axis=0,
                    )
                self._explicit_pd_compat.apply_candidate_batch(
                    stiffness_scale=stiffness_scale[:, columns],
                    damping_scale=damping_scale[:, columns],
                    delay_seconds=(None if delay_rows is None else delay_rows[:, columns]),
                    integral_gain=integral_rows,
                )
            except Exception as exc:
                raise SysIdEnvironmentBridgeError(f"Failed to apply explicit PD actuator candidate: {exc}") from exc
        with self._articulation_backend_context():
            if static is not None and dynamic is not None:
                try:
                    art.set_dof_friction_properties(
                        static_frictions=static,
                        dynamic_frictions=dynamic,
                        indices=indices,
                        dof_indices=dof_idx,
                    )
                except Exception as exc:
                    raise SysIdEnvironmentBridgeError(
                        f"Physics backend {self.physics_backend!r} cannot apply the selected "
                        f"joint-friction parameters: {exc}"
                    ) from exc

            if self._explicit_pd_compat is None and gain_selected:
                art.set_dof_gains(
                    stiffnesses=(self._baseline_stiffness[:n_phys] * stiffness_scale).astype(np.float32),
                    dampings=(self._baseline_damping[:n_phys] * damping_scale).astype(np.float32),
                    indices=indices,
                    dof_indices=dof_idx,
                )
            elif gain_selected and len(self._explicit_dof_columns) < n_dof:
                explicit_columns = set(self._explicit_dof_columns)
                implicit_columns = [column for column in range(n_dof) if column not in explicit_columns]
                implicit_indices = [dof_idx[column] for column in implicit_columns]
                art.set_dof_gains(
                    stiffnesses=(
                        self._baseline_stiffness[:n_phys, implicit_columns] * stiffness_scale[:, implicit_columns]
                    ).astype(np.float32),
                    dampings=(
                        self._baseline_damping[:n_phys, implicit_columns] * damping_scale[:, implicit_columns]
                    ).astype(np.float32),
                    indices=indices,
                    dof_indices=implicit_indices,
                )

            if self._baseline_armature is not None and any(state.joint_armature is not None for state in apply_states):
                armatures = self._baseline_armature[:n_phys].astype(np.float32, copy=True)
                for env_i, state in enumerate(apply_states):
                    if state.joint_armature is None:
                        continue
                    values = np.asarray(state.joint_armature, dtype=np.float32)
                    mask = np.isfinite(values)
                    armatures[env_i, mask] = np.maximum(values[mask], 0.0)
                art.set_dof_armatures(
                    armatures=armatures.astype(np.float32),
                    indices=indices,
                    dof_indices=dof_idx,
                )

            if self._baseline_link_masses is not None and SysIdParameterType.LINK_MASS in selected_types:
                mass_scale = np.stack([state.link_mass_scale[:num_links] for state in apply_states], axis=0)
                masses = self._baseline_link_masses[:n_phys, :num_links] * mass_scale
                art.set_link_masses(np.maximum(masses, 1e-6).astype(np.float32), indices=indices)

            if self._baseline_link_com is not None and any(np.any(state.link_com_delta) for state in apply_states):
                com_delta = np.stack([state.link_com_delta[:num_links] for state in apply_states], axis=0)
                positions = np.tile(self._baseline_link_com[:num_links], (n_phys, 1, 1)) + com_delta
                orientations = np.tile(
                    self._baseline_link_com_orientation[:num_links],
                    (n_phys, 1, 1),
                )
                try:
                    # get_link_coms() and set_link_coms() both use wxyz. Pass
                    # the captured values back unchanged so only COM positions
                    # vary across candidates.
                    art.set_link_coms(
                        positions.astype(np.float32),
                        orientations.astype(np.float32),
                        indices=indices,
                    )
                except Exception as exc:
                    # A selected COM parameter that cannot be applied makes the
                    # rollout cost independent of it, so the optimizer would
                    # "converge" on a value never realized in physics. Fail loudly.
                    raise SysIdEnvironmentBridgeError(
                        f"Failed to apply optimized link COM offsets to physics: {exc}"
                    ) from exc

            if self._baseline_link_inertia is not None and any(state.link_inertia_flat for state in apply_states):
                inertia_tensor = np.tile(self._baseline_link_inertia[:num_links], (n_phys, 1, 1))
                for env_i, state in enumerate(apply_states):
                    for link_idx, flat in state.link_inertia_flat.items():
                        if 0 <= link_idx < num_links:
                            inertia_tensor[env_i, link_idx] = flat
                try:
                    art.set_link_inertias(inertia_tensor.astype(np.float32), indices=indices)
                except Exception as exc:
                    raise SysIdEnvironmentBridgeError(
                        f"Failed to apply optimized link inertias to physics: {exc}"
                    ) from exc

        if self._baseline_dof_lower is not None and any(
            state.joint_limit_lower_scale is not None for state in apply_states
        ):
            lower = np.stack(
                [
                    self._baseline_dof_lower
                    * (state.joint_limit_lower_scale if state.joint_limit_lower_scale is not None else 1.0)
                    for state in apply_states
                ],
                axis=0,
            )
            upper = np.stack(
                [
                    self._baseline_dof_upper
                    * (state.joint_limit_upper_scale if state.joint_limit_upper_scale is not None else 1.0)
                    for state in apply_states
                ],
                axis=0,
            )
            try:
                with self._articulation_backend_context():
                    art.set_dof_limits(
                        lower=lower.astype(np.float32),
                        upper=upper.astype(np.float32),
                        indices=indices,
                        dof_indices=dof_idx,
                    )
            except Exception as exc:
                raise SysIdEnvironmentBridgeError(f"Failed to apply optimized joint limits to physics: {exc}") from exc

        self._flush_physics_changes()

    def _steps_per_csv_sample(self) -> int:
        if self._trajectory is None or self._trajectory.times.shape[0] < 2:
            return 1
        dt_csv = float(np.median(np.diff(self._trajectory.times)))
        dt_physics = runtime.get_physics_dt()
        return max(1, int(round(dt_csv / dt_physics)))

    def _set_position_targets(self, cmd_row: np.ndarray) -> None:
        """Set joint position targets for all active envs from one CSV command row.

        Args:
            cmd_row: Value supplied for ``cmd_row``.
        """
        if self._articulation is None:
            raise SysIdEnvironmentBridgeError("Articulation is not initialized.")
        n = self._physics_env_count()
        dof_idx = self._joint_dof_indices
        cmd = np.asarray(cmd_row, dtype=np.float32).reshape(-1)
        if cmd.shape[0] != len(dof_idx):
            raise SysIdEnvironmentBridgeError(
                f"Command width {cmd.shape[0]} does not match {len(dof_idx)} trajectory DOFs."
            )
        cmds = np.tile(cmd, (n, 1))
        indices = list(range(n))
        if self._fabric_direct_write_dofs("position_targets", cmds, indices=indices, dof_indices=dof_idx):
            if self._diagnostics_enabled:
                targets = self._fabric_direct_read_dofs("position_targets", indices=indices, dof_indices=dof_idx)
                positions = self._fabric_direct_read_dofs("positions", indices=indices, dof_indices=dof_idx)
                self._log_fabric_direct_command_probe(targets, positions)
            return
        with self._articulation_backend_context():
            self._articulation.set_dof_position_targets(
                cmds,
                indices=indices,
                dof_indices=dof_idx,
            )

    def step_with_commands(self, joint_commands: torch.Tensor) -> None:
        """Set position targets only (physics advances via timeline + physics subscription).

        Args:
            joint_commands: Value supplied for ``joint_commands``.
        """
        cmds = joint_commands.detach().cpu().numpy().astype(np.float32)
        if cmds.ndim == 1:
            self._set_position_targets(cmds)
        else:
            if self._fabric_direct_write_dofs(
                "position_targets",
                cmds,
                indices=list(range(cmds.shape[0])),
                dof_indices=self._joint_dof_indices,
            ):
                return
            with self._articulation_backend_context():
                for env_i in range(cmds.shape[0]):
                    self._articulation.set_dof_position_targets(
                        cmds[env_i : env_i + 1],
                        indices=[env_i],
                        dof_indices=self._joint_dof_indices,
                    )

    def get_sim_state(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Return joint positions and velocities for active clones.

        Returns:
            Result produced by the operation.
        """
        pos, vel, _torque, _ee, _contact = self.get_extended_sim_state()
        return pos, vel

    def get_extended_sim_state(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor | None, torch.Tensor | None, torch.Tensor | None]:
        """Batched joint and optional link signals for all active clones.

        Returns:
            Result produced by the operation.
        """
        if self._articulation is None:
            raise SysIdEnvironmentBridgeError("Call ensure_num_envs() before get_extended_sim_state.")
        n_phys = self._physics_env_count()
        n_read = max(1, self._logical_env_count)
        dof_idx = self._joint_dof_indices
        art = self._articulation
        indices = list(range(n_phys))
        pos_t = self._fabric_direct_read_dofs("positions", indices=indices, dof_indices=dof_idx)
        vel_t = self._fabric_direct_read_dofs("velocities", indices=indices, dof_indices=dof_idx)
        effort_t = (
            self._fabric_direct_read_dofs("projected_joint_forces", indices=indices, dof_indices=dof_idx)
            if self._collect_torque
            else None
        )
        if pos_t is None or vel_t is None:
            with self._articulation_backend_context():
                pos_t, vel_t, effort_t = read_batched_joint_state(
                    art,
                    num_envs=n_phys,
                    dof_indices=dof_idx,
                    device=self._device,
                    include_effort=self._collect_torque,
                )
        elif effort_t is None:
            # Some Fabric tensor views expose position/velocity getters but no
            # projected-force getter. Preserve the direct state reads while
            # using the public articulation effort fallback.
            with self._articulation_backend_context():
                _fallback_pos, _fallback_vel, effort_t = read_batched_joint_state(
                    art,
                    num_envs=n_phys,
                    dof_indices=dof_idx,
                    device=self._device,
                )
        if pos_t.shape[0] > n_read:
            pos_t = pos_t[:n_read]
            vel_t = vel_t[:n_read]
            if effort_t is not None:
                effort_t = effort_t[:n_read]
        ee_link = self.end_effector_link_index
        if ee_link < 0 and self._num_links > 0:
            ee_link = self._num_links - 1
        ee_pose = None
        contact = None
        if self._collect_end_effector_pose or self._collect_contact_force:
            with self._articulation_backend_context():
                if self._collect_end_effector_pose:
                    ee_pose = read_batched_link_pose(
                        art,
                        num_envs=n_phys,
                        link_index=ee_link,
                        device=self._device,
                    )
                if self._collect_contact_force:
                    contact = read_batched_contact_proxy(
                        art,
                        num_envs=n_phys,
                        link_index=ee_link,
                        device=self._device,
                    )
        if ee_pose is not None and ee_pose.shape[0] > n_read:
            ee_pose = ee_pose[:n_read]
        if contact is not None and contact.shape[0] > n_read:
            contact = contact[:n_read]
        self._log_fabric_direct_state_probe(pos_t, vel_t)
        return pos_t, vel_t, effort_t, ee_pose, contact

    async def run_rollout_async(
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        commands: torch.Tensor,
        num_steps: int,
    ) -> SysIdRolloutResult:
        """Run a full trajectory rollout using direct or timeline-driven physics steps.

        Returns:
            :class:`SysIdRolloutResult` with shape ``(num_envs, T, *)`` tensors.

        Args:
            theta_per_env: Value supplied for ``theta_per_env``.
            param_entries: Value supplied for ``param_entries``.
            commands: Value supplied for ``commands``.
            num_steps: Value supplied for ``num_steps``.
        """
        if self._rollout_active:
            raise SysIdEnvironmentBridgeError("Another rollout is already in progress.")
        self._rollout_cancel_requested = False
        try:
            await self._ensure_rollout_physics_ready_async()
            await self.ensure_num_envs_async(theta_per_env.shape[0])
            self.apply_parameter_vector(theta_per_env.to(device=self._device, dtype=torch.float32), param_entries)
            self.reset_to_trajectory_start()
            cmd_np = commands.detach().cpu().numpy().astype(np.float32)
            if cmd_np.ndim == 1:
                cmd_np = cmd_np.reshape(1, -1)
            if cmd_np.shape[1] != self._num_joints:
                raise SysIdEnvironmentBridgeError(
                    f"Command columns {cmd_np.shape[1]} != trajectory joints {self._num_joints}."
                )
            self._rollout_commands = cmd_np[:num_steps]
            self._rollout_num_steps = num_steps
            self._rollout_csv_index = 0
            self._rollout_substep = 0
            self._rollout_warmup_remaining = self._warmup_physics_steps()
            self._rollout_positions = []
            self._rollout_velocities = []
            self._rollout_torques = []
            self._rollout_ee_poses = []
            self._rollout_contact_forces = []
            self._rollout_error = None
            self._fabric_command_probe_logged = False
            self._fabric_state_probe_logged = False
            self._rollout_done = asyncio.Event()
            self._set_position_targets(self._rollout_commands[0])
            self._flush_physics_changes()
            self._rollout_active = True
            if self.offline_stepping:
                await self._run_rollout_offline_async(num_steps, theta_per_env.shape[0])
            else:
                await self._kit_updates_async(2)
                self._ensure_physics_subscription()
                await self._wait_for_rollout_done_async(num_steps, theta_per_env.shape[0])
            if self._rollout_cancel_requested:
                raise asyncio.CancelledError()
            self._rollout_active = False
            if self._rollout_error is not None:
                self._articulation = None
                raise SysIdEnvironmentBridgeError(
                    f"Physics rollout failed: {self._rollout_error}"
                ) from self._rollout_error
            if len(self._rollout_positions) != num_steps:
                raise SysIdEnvironmentBridgeError(
                    f"Rollout ended with {len(self._rollout_positions)} samples, expected {num_steps}. "
                    + (
                        "Direct SimulationManager stepping ended early."
                        if self.offline_stepping
                        else "Keep the timeline on Play for the full rollout."
                    )
                )
            sim_pos = torch.stack(self._rollout_positions, dim=1)
            sim_vel = torch.stack(self._rollout_velocities, dim=1)
            sim_torque = torch.stack(self._rollout_torques, dim=1) if self._rollout_torques else None
            sim_ee = torch.stack(self._rollout_ee_poses, dim=1) if self._rollout_ee_poses else None
            sim_contact = torch.stack(self._rollout_contact_forces, dim=1) if self._rollout_contact_forces else None
            return SysIdRolloutResult(
                positions=sim_pos,
                velocities=sim_vel,
                torques=sim_torque,
                end_effector_poses=sim_ee,
                contact_forces=sim_contact,
            )
        finally:
            self._rollout_active = False
            self._rollout_warmup_remaining = 0
            self._rollout_commands = None
            self._release_physics_subscription()
            self._rollout_done = None

    async def _run_rollout_offline_async(self, num_steps: int, num_envs: int) -> None:
        """Collect one rollout without rendering or advancing the Kit frame loop.

        Args:
            num_steps: Number of trajectory samples to collect.
            num_envs: Number of parallel rollout environments.
        """
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            timeline.pause()
            # timeline.pause() is applied asynchronously; in the interactive Kit loop
            # is_playing() stays True until the next update is pumped. Let it settle
            # before deciding the pause failed, otherwise the offline rollout aborts.
            await self._kit_updates_async(1)
        if timeline.is_playing():
            raise SysIdEnvironmentBridgeError("Unable to pause the timeline before direct SimulationManager stepping.")

        start = time.perf_counter()
        last_log_time = start
        update_fabric = bool(self.fabric_clones)
        try:
            runtime.step_simulation(steps=self._rollout_warmup_remaining, update_fabric=update_fabric)
            self._rollout_warmup_remaining = 0
            self._reapply_trajectory_initial_state()
            self._record_rollout_sample()

            substeps = self._steps_per_csv_sample()
            while self._rollout_active:
                if self._rollout_cancel_requested:
                    raise asyncio.CancelledError()
                runtime.step_simulation(steps=substeps, update_fabric=update_fabric)
                self._record_rollout_sample()

                now = time.perf_counter()
                if now - last_log_time >= 2.0:
                    sample_index = int(self._rollout_csv_index)
                    elapsed = max(1e-9, now - start)
                    progress = sample_index / max(1, int(num_steps))
                    carb.log_info(
                        "SysId offline rollout: "
                        f"envs={int(num_envs)}, progress={progress:.3f} "
                        f"samples={sample_index}/{int(num_steps)}, "
                        f"avg_samples/s={sample_index / elapsed:.1f}"
                    )
                    last_log_time = now

                # Give cancellation and progress tasks a chance to run without
                # pumping a Kit frame (which would advance timeline physics).
                if self._rollout_csv_index % 32 == 0:
                    await asyncio.sleep(0)
        except Exception as exc:
            self._rollout_error = exc
            self._articulation = None
            self._rollout_active = False
            if self._rollout_done is not None and not self._rollout_done.is_set():
                self._rollout_done.set()

    async def _wait_for_rollout_done_async(self, num_steps: int, num_envs: int) -> None:
        """Pump Kit updates until the physics callback has collected the rollout.

        Args:
            num_steps: Value supplied for ``num_steps``.
            num_envs: Value supplied for ``num_envs``.
        """
        if self._rollout_done is None:
            return
        app = omni.kit.app.get_app()
        start = time.perf_counter()
        last_log_time = start
        last_sample_index = int(self._rollout_csv_index)
        while not self._rollout_done.is_set():
            await app.next_update_async()
            now = time.perf_counter()
            if now - last_log_time < 2.0:
                continue
            sample_index = int(self._rollout_csv_index)
            elapsed = max(1e-9, now - start)
            delta_time = max(1e-9, now - last_log_time)
            delta_samples = max(0, sample_index - last_sample_index)
            samples_per_sec = delta_samples / delta_time
            avg_samples_per_sec = sample_index / elapsed
            progress = sample_index / max(1, int(num_steps))
            carb.log_info(
                "SysId rollout: "
                f"envs={int(num_envs)}, progress={progress:.3f} "
                f"samples={sample_index}/{int(num_steps)}, "
                f"warmup_remaining={int(self._rollout_warmup_remaining)}, "
                f"samples/s={samples_per_sec:.1f}, avg_samples/s={avg_samples_per_sec:.1f}"
            )
            last_log_time = now
            last_sample_index = sample_index

    def request_stop(self) -> None:
        """Abort any in-flight rollout and release the step callback immediately."""
        self._rollout_cancel_requested = True
        self._rollout_active = False
        self._rollout_warmup_remaining = 0
        self._rollout_commands = None
        if self._rollout_done is not None and not self._rollout_done.is_set():
            try:
                self._rollout_done.set()
            except Exception:
                pass
        self._release_physics_subscription()

    def _reset_rollout_targets_to_initial_state(self) -> None:
        """Leave the source articulation at a stable target before teardown."""
        if self._trajectory is None or self._articulation is None:
            return
        valid = getattr(self._articulation, "is_physics_tensor_entity_valid", None)
        if callable(valid):
            try:
                if not valid():
                    return
            except Exception:
                return
        try:
            self._reapply_trajectory_initial_state()
        except Exception as exc:
            carb.log_warn(f"SysId: failed to reset rollout targets during cleanup: {exc}")

    def _unregister_physics_replication(self, stage: Usd.Stage) -> None:
        """Unregister the stage-level PhysX replicator before clone prims are removed.

        Args:
            stage: USD stage used by the operation.
        """
        should_unregister = bool(getattr(self, "_physics_replication_registered", False))
        if not should_unregister:
            return
        try:
            from omni.physx import get_physx_replicator_interface

            stage_id = self._stage_cache_id(stage)
            get_physx_replicator_interface().unregister_replicator(stage_id)
            carb.log_info("SysId: unregistered PhysX replication before clone cleanup.")
        except Exception as exc:
            carb.log_warn(f"SysId: failed to unregister PhysX replication during cleanup: {exc}")
        finally:
            self._physics_replication_registered = False

    def _clone_paths_to_remove(self) -> list[str]:
        if self._created_clone_paths:
            return [path for path in self._created_clone_paths if path != self._source_env_path]
        if self._cloned_for_count <= 1:
            return []
        env_root = PurePosixPath(self._env_paths_root.rstrip("/"))
        parent = env_root.parent.as_posix()
        return [f"{parent}/{env_root.name}_{i}" for i in range(1, self._cloned_for_count)]

    def _clear_clone_tracking(self) -> None:
        self._created_clone_paths = []
        self._cloned_for_count = 0
        self._logical_env_count = 1
        self._physics_replication_registered = False
        self._last_clone_co_locate = None
        self._fabric_rebind_logged = False
        self._fabric_command_probe_logged = False
        self._fabric_state_probe_logged = False

    def _remove_cloned_envs_sync(self) -> None:
        paths = self._clone_paths_to_remove()
        if not paths and not getattr(self, "_physics_replication_registered", False):
            return
        stage = runtime.get_current_stage()
        if stage is None:
            return
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        try:
            if self.fabric_clones:
                self._articulation = None
                self._fabric_rebind_logged = False
                self._fabric_command_probe_logged = False
                self._fabric_state_probe_logged = False
            if was_playing:
                if not self.fabric_clones:
                    self._reset_rollout_targets_to_initial_state()
                    self._flush_physics_changes()
                    self._kit_updates_sync(1)
                timeline.stop()
                self._kit_updates_sync(4)
            self._articulation = None
            self._flush_physics_changes()
            had_replication = bool(getattr(self, "_physics_replication_registered", False))
            self._unregister_physics_replication(stage)
            removed = []
            for path in reversed(paths):
                if stage.GetPrimAtPath(path).IsValid():
                    stage.RemovePrim(path)
                    removed.append(path)
                elif self.fabric_clones and self._remove_fabric_prim(stage, path):
                    removed.append(path)
            if removed or had_replication:
                self._flush_physics_changes()
                self._kit_updates_sync(8)
            leftovers = self._fabric_paths_still_present(stage, paths)
            if leftovers:
                carb.log_warn(f"SysId: Fabric clone cleanup left prims in usdrt stage: {leftovers}")
            if removed:
                carb.log_info(f"SysId: removed cloned environments: {list(reversed(removed))}")
        finally:
            self._clear_clone_tracking()
            if was_playing:
                try:
                    timeline.play()
                    self._kit_updates_sync(8)
                    runtime.initialize_physics()
                except Exception as exc:
                    carb.log_warn(f"SysId: failed to restore the timeline after clone cleanup: {exc}")

    async def _remove_cloned_envs_async(self) -> None:
        paths = self._clone_paths_to_remove()
        if not paths and not getattr(self, "_physics_replication_registered", False):
            return
        stage = runtime.get_current_stage()
        if stage is None:
            return
        timeline = omni.timeline.get_timeline_interface()
        was_playing = timeline.is_playing()
        try:
            if self.fabric_clones:
                self._articulation = None
                self._fabric_rebind_logged = False
                self._fabric_command_probe_logged = False
                self._fabric_state_probe_logged = False
            if was_playing:
                if not self.fabric_clones:
                    self._reset_rollout_targets_to_initial_state()
                    self._flush_physics_changes()
                    await self._kit_updates_async(1)
                timeline.stop()
                await self._kit_updates_async(4)
            self._articulation = None
            self._flush_physics_changes()
            had_replication = bool(getattr(self, "_physics_replication_registered", False))
            self._unregister_physics_replication(stage)
            removed = []
            for path in reversed(paths):
                if stage.GetPrimAtPath(path).IsValid():
                    stage.RemovePrim(path)
                    removed.append(path)
                elif self.fabric_clones and self._remove_fabric_prim(stage, path):
                    removed.append(path)
            if removed or had_replication:
                self._flush_physics_changes()
                await self._kit_updates_async(8)
            leftovers = self._fabric_paths_still_present(stage, paths)
            if leftovers:
                carb.log_warn(f"SysId: Fabric clone cleanup left prims in usdrt stage: {leftovers}")
            if removed:
                carb.log_info(f"SysId: removed cloned environments: {list(reversed(removed))}")
        finally:
            self._clear_clone_tracking()
            if was_playing:
                try:
                    timeline.play()
                    await self._kit_updates_async(8)
                    runtime.initialize_physics()
                except Exception as exc:
                    carb.log_warn(f"SysId: failed to restore the timeline after clone cleanup: {exc}")

    def cleanup(self, remove_clones: bool = True) -> None:
        """Drop physics handles and optionally remove generated clone environments.

        Args:
            remove_clones: Whether to remove generated simulation environments.
        """
        self.request_stop()
        self._close_explicit_pd_compat()
        if remove_clones:
            self._remove_cloned_envs_sync()
        else:
            self._articulation = None
        self._rollout_cancel_requested = False
        self._cloned_for_count = 0
        self._logical_env_count = 1
        self._created_clone_paths = []
        self._physics_replication_registered = False
        self._last_clone_co_locate = None
        self._fabric_rebind_logged = False
        self._fabric_command_probe_logged = False
        self._fabric_state_probe_logged = False
        self._restore_physics_backend()

    async def cleanup_async(self, remove_clones: bool = True) -> None:
        """Async teardown used after an optimization run completes or is cancelled.

        Args:
            remove_clones: Value supplied for ``remove_clones``.
        """
        self.request_stop()
        self._close_explicit_pd_compat()
        await self._kit_updates_async(2)
        if remove_clones:
            await self._remove_cloned_envs_async()
        else:
            self._articulation = None
        self._rollout_cancel_requested = False
        self._cloned_for_count = 0
        self._logical_env_count = 1
        self._created_clone_paths = []
        self._physics_replication_registered = False
        self._last_clone_co_locate = None
        self._fabric_rebind_logged = False
        self._fabric_command_probe_logged = False
        self._fabric_state_probe_logged = False
        self._restore_physics_backend()
