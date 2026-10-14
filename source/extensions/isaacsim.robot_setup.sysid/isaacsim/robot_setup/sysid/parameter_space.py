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
"""Canonical parameter registry for system identification."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING

import numpy as np
import torch

from .inertia_param import (
    inertia_matrix_to_log_cholesky,
    log_cholesky_to_inertia_matrix,
)
from .parameter_apply import (
    ParameterApplyState,
    decode_theta_to_apply_state,
    per_dof_scale_vectors,
)
from .parameter_types import (
    GLOBAL_DOF_INDEX,
    GLOBAL_LINK_INDEX,
    ParameterCategory,
    SysIdParameterEntry,
    SysIdParameterSpec,
    SysIdParameterType,
    build_dof_parameter_specs,
    build_extended_parameter_specs,
    entries_from_specs,
)

if TYPE_CHECKING:
    from pxr import Usd

    from .usd_parameter_io import JointUsdSnapshot, LinkUsdSnapshot

JOINT_FRICTION_BASELINE_FLOOR = 1e-6
JOINT_GAIN_BASELINE_FLOOR = 1e-6


def _runtime_baseline_vector(values) -> np.ndarray | None:  # noqa: ANN001
    if values is None:
        return None
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 0:
        return arr.reshape(1)
    if arr.ndim > 1:
        return arr[0]
    return arr


def _runtime_baseline_matrix(values, *, value_ndim: int = 1) -> np.ndarray | None:  # noqa: ANN001
    if values is None:
        return None
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim > value_ndim + 1:
        return arr[0]
    return arr


def _value_at(values: np.ndarray | None, index: int, default: float) -> float:
    if values is None or index >= values.shape[0]:
        return default
    return float(values[index])


@dataclass
class ParameterScaleState:
    """Per-DOF scale multipliers decoded from one optimizer theta row."""

    friction_scale: np.ndarray
    stiffness_scale: np.ndarray
    damping_scale: np.ndarray
    link_mass_scale: float


@dataclass
class ParameterBaselineState:
    """Captured simulation/USD baselines for extended parameter application."""

    link_inertia_lc: dict[int, np.ndarray] = field(default_factory=dict)
    link_com: dict[int, np.ndarray] = field(default_factory=dict)
    link_masses: dict[int, float] = field(default_factory=dict)
    joint_lower_limits: dict[int, float] = field(default_factory=dict)
    joint_upper_limits: dict[int, float] = field(default_factory=dict)
    joint_friction: dict[int, float] = field(default_factory=dict)
    joint_static_friction: dict[int, float] = field(default_factory=dict)
    joint_dynamic_friction: dict[int, float] = field(default_factory=dict)
    joint_viscous_friction: dict[int, float] = field(default_factory=dict)
    joint_stiffness: dict[int, float] = field(default_factory=dict)
    joint_damping: dict[int, float] = field(default_factory=dict)
    joint_armature: dict[int, float] = field(default_factory=dict)
    usd_link_snapshots: list[LinkUsdSnapshot] = field(default_factory=list)
    usd_joint_snapshots: list[JointUsdSnapshot | None] = field(default_factory=list)


class ParameterSpace:
    """Registry of tunable simulation parameters with SRD category tags and bounds.

    Args:
        specs: Tunable parameter definitions in registry order.
        num_joints: Optional articulated degree-of-freedom count.
        num_links: Optional articulated link count.
    """

    def __init__(
        self,
        specs: Sequence[SysIdParameterSpec],
        *,
        num_joints: int | None = None,
        num_links: int | None = None,
    ) -> None:
        if not specs:
            raise ValueError("ParameterSpace requires at least one parameter spec.")
        self._specs: tuple[SysIdParameterSpec, ...] = tuple(specs)
        self._num_joints = num_joints
        self._num_links = num_links
        self._baseline = ParameterBaselineState()

    @classmethod
    def for_robot(cls, num_joints: int) -> ParameterSpace:
        """Build the default per-joint parameter table for ``num_joints`` DOFs.

        Args:
            num_joints: Number of robot joints.

        Returns:
            Parameter space populated with the default per-joint parameters.
        """
        return cls(build_dof_parameter_specs(num_joints), num_joints=num_joints, num_links=1)

    @classmethod
    def for_robot_extended(
        cls,
        num_joints: int,
        num_links: int,
        **kwargs,  # noqa: ANN003
    ) -> ParameterSpace:
        """Build an extended parameter registry (COM, inertia, and limits).

        Args:
            num_joints: Number of robot joints.
            num_links: Number of articulated links.
            **kwargs: Optional parameter families and defaults forwarded to
                ``build_extended_parameter_specs``.

        Returns:
            Parameter space populated with joint, link, and limit parameters.
        """
        return cls(
            build_extended_parameter_specs(num_joints, num_links, **kwargs),
            num_joints=num_joints,
            num_links=num_links,
        )

    @property
    def specs(self) -> tuple[SysIdParameterSpec, ...]:  # noqa: D102
        return self._specs

    @property
    def baseline(self) -> ParameterBaselineState:  # noqa: D102
        return self._baseline

    def entries(self) -> list[SysIdParameterEntry]:  # noqa: D102
        return entries_from_specs(self._specs)

    def num_parameters(self) -> int:  # noqa: D102
        return len(self._specs)

    def specs_by_category(self, category: ParameterCategory) -> tuple[SysIdParameterSpec, ...]:  # noqa: D102
        return tuple(spec for spec in self._specs if spec.category == category)

    def entries_by_category(self, category: ParameterCategory) -> list[SysIdParameterEntry]:  # noqa: D102
        return [spec.entry for spec in self.specs_by_category(category)]

    def provenance_groups(
        self,
        theta: list[float] | np.ndarray,
    ) -> tuple[list[ParameterProvenanceEntry], list[ParameterProvenanceEntry], list[ParameterProvenanceEntry]]:
        """Split optimized theta into all/sysid/calibration provenance entries.

        Args:
            theta: Optimized parameter values in registry-entry order.

        Returns:
            All entries, SysID-category entries, and calibration-category entries.
        """
        from .provenance import split_parameters_by_category

        num_joints = self._num_joints or 1
        return split_parameters_by_category(self.entries(), theta, num_joints=num_joints)

    def index_of(self, entry: SysIdParameterEntry) -> int:  # noqa: D102
        for idx, spec in enumerate(self._specs):
            if spec.entry == entry:
                return idx
        raise KeyError(f"Parameter entry not registered: {entry!r}")

    def read_usd_baselines(
        self,
        stage: Usd.Stage,
        link_paths: list[str],
        dof_paths: list[str],
        *,
        physics_backend: str = "",
    ) -> None:
        """Capture USD link/joint snapshots and derive Log-Cholesky inertia baselines.

        Args:
            stage: USD stage containing the articulation.
            link_paths: Rigid-link prim paths in articulation order.
            dof_paths: Joint prim paths in degree-of-freedom order.
            physics_backend: Backend whose solver-specific joint properties are read.
        """
        from .usd_parameter_io import read_articulation_usd_snapshots

        links, joints = read_articulation_usd_snapshots(stage, link_paths, dof_paths, physics_backend=physics_backend)
        self._baseline.usd_link_snapshots = links
        self._baseline.usd_joint_snapshots = list(joints)

        for link_idx, snapshot in enumerate(links):
            self._baseline.link_com[link_idx] = snapshot.com_offset.copy()
            self._baseline.link_masses[link_idx] = snapshot.mass
            inertia = (
                snapshot.inertia_matrix if snapshot.inertia_matrix is not None else np.diag(snapshot.diagonal_inertia)
            )
            self._baseline.link_inertia_lc[link_idx] = inertia_matrix_to_log_cholesky(inertia)

        for dof_idx, joint in enumerate(joints):
            if joint is None:
                continue
            self._baseline.joint_lower_limits[dof_idx] = joint.lower_limit
            self._baseline.joint_upper_limits[dof_idx] = joint.upper_limit
            self._baseline.joint_friction[dof_idx] = joint.friction
            self._baseline.joint_static_friction[dof_idx] = joint.static_friction
            self._baseline.joint_dynamic_friction[dof_idx] = joint.dynamic_friction
            self._baseline.joint_viscous_friction[dof_idx] = joint.viscous_friction
            self._baseline.joint_stiffness[dof_idx] = joint.stiffness
            self._baseline.joint_damping[dof_idx] = joint.damping
            self._baseline.joint_armature[dof_idx] = joint.armature

    def override_link_runtime_baselines(
        self,
        *,
        masses=None,  # noqa: ANN001
        coms=None,  # noqa: ANN001
        inertias=None,  # noqa: ANN001
    ) -> None:
        """Replace USD link baselines with live articulation values captured for rollouts.

        Args:
            masses: Optional runtime link masses.
            coms: Optional runtime link center-of-mass positions.
            inertias: Optional runtime link inertia matrices.
        """  # noqa: DOC106, DOC107
        mass_vec = _runtime_baseline_vector(masses)
        com_mat = _runtime_baseline_matrix(coms, value_ndim=1)
        inertia_mat = _runtime_baseline_matrix(inertias, value_ndim=2)

        updated: list[LinkUsdSnapshot] = []
        for link_idx, link in enumerate(self._baseline.usd_link_snapshots):
            mass = max(_value_at(mass_vec, link_idx, link.mass), 1e-6)
            com = link.com_offset.copy()
            if com_mat is not None and link_idx < com_mat.shape[0]:
                candidate = np.asarray(com_mat[link_idx], dtype=np.float64).reshape(3)
                if np.all(np.isfinite(candidate)):
                    com = candidate
            diag = link.diagonal_inertia.copy()
            inertia_matrix = link.inertia_matrix
            if inertia_mat is not None and link_idx < inertia_mat.shape[0]:
                candidate = np.asarray(inertia_mat[link_idx], dtype=np.float64).reshape(3, 3)
                candidate = 0.5 * (candidate + candidate.T)
                if np.all(np.isfinite(candidate)):
                    try:
                        eig = np.linalg.eigvalsh(candidate)
                    except np.linalg.LinAlgError:
                        eig = np.asarray([-1.0], dtype=np.float64)
                    if np.all(eig > 0.0):
                        inertia_matrix = candidate
                        diag = np.array([candidate[0, 0], candidate[1, 1], candidate[2, 2]], dtype=np.float64)
                        self._baseline.link_inertia_lc[link_idx] = inertia_matrix_to_log_cholesky(candidate)
            updated_link = replace(
                link,
                mass=mass,
                com_offset=com,
                diagonal_inertia=diag,
                inertia_matrix=inertia_matrix,
            )
            updated.append(updated_link)
            self._baseline.link_masses[link_idx] = updated_link.mass
            self._baseline.link_com[link_idx] = updated_link.com_offset.copy()
        self._baseline.usd_link_snapshots = updated

    def override_joint_runtime_baselines(
        self,
        *,
        static_friction=None,  # noqa: ANN001
        dynamic_friction=None,  # noqa: ANN001
        stiffness=None,  # noqa: ANN001
        damping=None,  # noqa: ANN001
        armature=None,  # noqa: ANN001
        preserve_usd_drive_gains: bool = False,
    ) -> None:
        """Replace live joint baselines while optionally retaining authored drive gains.

        Explicit Newton actuators keep their PD gains outside ``UsdPhysics.DriveAPI``.
        For that path the runtime gain dictionaries must use controller ``kp``/``kd``,
        while the USD snapshots retain the original drive values so writeback does
        not accidentally author explicit gains into the implicit drive schema.

        Args:
            static_friction: Optional runtime static-friction values.
            dynamic_friction: Optional runtime dynamic-friction values.
            stiffness: Optional runtime joint-stiffness values.
            damping: Optional runtime joint damping values.
            armature: Optional runtime joint-armature values.
            preserve_usd_drive_gains: Keep authored USD gains while updating runtime gain baselines.
        """  # noqa: DOC107
        static_vec = _runtime_baseline_vector(static_friction)
        dynamic_vec = _runtime_baseline_vector(dynamic_friction)
        stiffness_vec = _runtime_baseline_vector(stiffness)
        damping_vec = _runtime_baseline_vector(damping)
        armature_vec = _runtime_baseline_vector(armature)

        updated: list[JointUsdSnapshot | None] = []
        for dof_idx, joint in enumerate(self._baseline.usd_joint_snapshots):
            if joint is None:
                updated.append(None)
                continue
            dynamic = _value_at(dynamic_vec, dof_idx, joint.dynamic_friction)
            friction = dynamic if dynamic_vec is not None else joint.friction
            if dynamic_vec is not None or static_vec is not None:
                dynamic = max(dynamic, JOINT_FRICTION_BASELINE_FLOOR)
                friction = max(friction if friction != 0.0 else dynamic, JOINT_FRICTION_BASELINE_FLOOR)
            static = _value_at(static_vec, dof_idx, joint.static_friction)
            if dynamic_vec is not None or static_vec is not None:
                static = max(static, dynamic * 1.01)

            runtime_stiffness = _value_at(stiffness_vec, dof_idx, joint.stiffness)
            runtime_damping = _value_at(damping_vec, dof_idx, joint.damping)
            if stiffness_vec is not None:
                runtime_stiffness = max(runtime_stiffness, JOINT_GAIN_BASELINE_FLOOR)
            if damping_vec is not None:
                runtime_damping = max(runtime_damping, JOINT_GAIN_BASELINE_FLOOR)
            joint_armature = _value_at(armature_vec, dof_idx, joint.armature)
            if armature_vec is not None:
                joint_armature = max(joint_armature, 0.0)

            updated_joint = replace(
                joint,
                friction=friction,
                static_friction=static,
                dynamic_friction=dynamic,
                stiffness=joint.stiffness if preserve_usd_drive_gains else runtime_stiffness,
                damping=joint.damping if preserve_usd_drive_gains else runtime_damping,
                armature=joint_armature,
            )
            updated.append(updated_joint)
            self._baseline.joint_friction[dof_idx] = updated_joint.friction
            self._baseline.joint_static_friction[dof_idx] = updated_joint.static_friction
            self._baseline.joint_dynamic_friction[dof_idx] = updated_joint.dynamic_friction
            self._baseline.joint_stiffness[dof_idx] = runtime_stiffness
            self._baseline.joint_damping[dof_idx] = runtime_damping
            self._baseline.joint_armature[dof_idx] = updated_joint.armature
        self._baseline.usd_joint_snapshots = updated

    def write_usd_from_baselines(
        self,
        stage: Usd.Stage,
        apply_state: ParameterApplyState,
        *,
        physics_backend: str = "",
        skip_parameter_types: set[SysIdParameterType] | None = None,
        skip_joint_parameter_dofs: dict[SysIdParameterType, set[int]] | None = None,
        param_entries: list[SysIdParameterEntry] | None = None,
    ) -> None:
        """Persist decoded apply state back to USD link/joint attributes.

        Args:
            stage: USD stage containing the articulation.
            apply_state: Decoded physical parameter values to persist.
            physics_backend: Selected backend that owns solver-specific properties.
            skip_parameter_types: Parameter types excluded from USD writeback.
            skip_joint_parameter_dofs: Per-parameter joint indices excluded from writeback.
            param_entries: Selected parameter entries in theta order.
        """
        from .usd_parameter_io import (
            LinkUsdSnapshot,
            write_joint_usd_snapshot,
            write_link_usd_snapshot,
        )

        skipped = skip_parameter_types or set()
        skipped_joint_dofs = skip_joint_parameter_dofs or {}
        active_entries = [entry for entry in (param_entries or []) if entry.param_type not in skipped]
        selective = param_entries is not None
        link_parameter_types = {
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        }
        joint_parameter_types = {
            SysIdParameterType.JOINT_FRICTION,
            SysIdParameterType.JOINT_STIFFNESS,
            SysIdParameterType.JOINT_DAMPING,
            SysIdParameterType.JOINT_ARMATURE,
            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
            SysIdParameterType.JOINT_LIMIT_UPPER_SCALE,
        }
        for link_idx, snapshot in enumerate(self._baseline.usd_link_snapshots):
            link_types = {
                entry.param_type
                for entry in active_entries
                if entry.param_type in link_parameter_types and entry.link_index in (GLOBAL_LINK_INDEX, link_idx)
            }
            if selective and not link_types:
                continue
            mass_scale = (
                float(apply_state.link_mass_scale[link_idx]) if link_idx < len(apply_state.link_mass_scale) else 1.0
            )
            com = snapshot.com_offset.copy()
            if link_idx < apply_state.link_com_delta.shape[0]:
                com += apply_state.link_com_delta[link_idx]
            diag = snapshot.diagonal_inertia.copy()
            inertia_matrix = snapshot.inertia_matrix
            if link_idx in apply_state.link_inertia_flat:
                flat = apply_state.link_inertia_flat[link_idx].reshape(3, 3)
                inertia_matrix = flat
                diag = np.array([flat[0, 0], flat[1, 1], flat[2, 2]], dtype=np.float64)
            write_link_usd_snapshot(
                stage,
                LinkUsdSnapshot(
                    path=snapshot.path,
                    mass=max(snapshot.mass * mass_scale, 1e-6),
                    com_offset=com,
                    diagonal_inertia=diag,
                    contact_offset=snapshot.contact_offset,
                    principal_axes_quat=snapshot.principal_axes_quat,
                    inertia_matrix=inertia_matrix,
                ),
                write_mass=(not selective or SysIdParameterType.LINK_MASS in link_types),
                write_com=(
                    not selective
                    or bool(
                        link_types
                        & {
                            SysIdParameterType.LINK_COM_OFFSET_X,
                            SysIdParameterType.LINK_COM_OFFSET_Y,
                            SysIdParameterType.LINK_COM_OFFSET_Z,
                        }
                    )
                ),
                write_inertia=(not selective or SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY in link_types),
                write_contact=False,
            )

        for snapshot_index, joint in enumerate(self._baseline.usd_joint_snapshots):
            if joint is None:
                continue
            dof_idx = joint.dof_index if joint.dof_index >= 0 else snapshot_index
            blocked_joint_types = {
                param_type for param_type, dof_indices in skipped_joint_dofs.items() if dof_idx in dof_indices
            }
            joint_types = {
                entry.param_type
                for entry in active_entries
                if entry.param_type in joint_parameter_types
                and entry.param_type not in blocked_joint_types
                and entry.dof_index in (GLOBAL_DOF_INDEX, dof_idx)
            }
            if selective and not joint_types:
                continue
            lower = joint.lower_limit
            upper = joint.upper_limit
            if apply_state.joint_limit_lower_scale is not None and dof_idx < len(apply_state.joint_limit_lower_scale):
                lower *= float(apply_state.joint_limit_lower_scale[dof_idx])
            if apply_state.joint_limit_upper_scale is not None and dof_idx < len(apply_state.joint_limit_upper_scale):
                upper *= float(apply_state.joint_limit_upper_scale[dof_idx])
            dynamic_friction = joint.dynamic_friction if joint.dynamic_friction != 0.0 else joint.friction
            dynamic_friction = max(dynamic_friction, JOINT_FRICTION_BASELINE_FLOOR)
            static_friction = joint.static_friction if joint.static_friction != 0.0 else dynamic_friction * 1.01
            static_friction = max(static_friction, dynamic_friction * 1.01)
            friction = joint.friction if joint.friction != 0.0 else dynamic_friction
            friction = max(friction, JOINT_FRICTION_BASELINE_FLOOR)
            viscous_friction = joint.viscous_friction
            stiffness = max(joint.stiffness, JOINT_GAIN_BASELINE_FLOOR)
            damping = max(joint.damping, JOINT_GAIN_BASELINE_FLOOR)
            armature = max(joint.armature, 0.0)
            if dof_idx < len(apply_state.friction_scale):
                friction_scale = float(apply_state.friction_scale[dof_idx])
                friction *= friction_scale
                static_friction *= friction_scale
                dynamic_friction *= friction_scale
                static_friction = max(static_friction, dynamic_friction * 1.01)
            if (
                SysIdParameterType.JOINT_STIFFNESS not in skipped
                and SysIdParameterType.JOINT_STIFFNESS not in blocked_joint_types
                and dof_idx < len(apply_state.stiffness_scale)
            ):
                stiffness *= float(apply_state.stiffness_scale[dof_idx])
            if (
                SysIdParameterType.JOINT_DAMPING not in skipped
                and SysIdParameterType.JOINT_DAMPING not in blocked_joint_types
                and dof_idx < len(apply_state.damping_scale)
            ):
                damping *= float(apply_state.damping_scale[dof_idx])
            if apply_state.joint_armature is not None and dof_idx < len(apply_state.joint_armature):
                armature_value = float(apply_state.joint_armature[dof_idx])
                if np.isfinite(armature_value):
                    armature = max(armature_value, 0.0)
            write_joint_usd_snapshot(
                stage,
                replace(
                    joint,
                    lower_limit=lower,
                    upper_limit=upper,
                    friction=max(friction, 0.0),
                    static_friction=max(static_friction, 0.0),
                    dynamic_friction=max(dynamic_friction, 0.0),
                    viscous_friction=max(viscous_friction, 0.0),
                    stiffness=max(stiffness, 0.0),
                    damping=max(damping, 0.0),
                    armature=max(armature, 0.0),
                    dof_index=dof_idx,
                ),
                physics_backend=physics_backend,
                write_stiffness=(
                    SysIdParameterType.JOINT_STIFFNESS not in skipped
                    and SysIdParameterType.JOINT_STIFFNESS not in blocked_joint_types
                    and (not selective or SysIdParameterType.JOINT_STIFFNESS in joint_types)
                ),
                write_damping=(
                    SysIdParameterType.JOINT_DAMPING not in skipped
                    and SysIdParameterType.JOINT_DAMPING not in blocked_joint_types
                    and (not selective or SysIdParameterType.JOINT_DAMPING in joint_types)
                ),
                write_limits=(
                    not selective
                    or bool(
                        joint_types
                        & {
                            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
                            SysIdParameterType.JOINT_LIMIT_UPPER_SCALE,
                        }
                    )
                ),
                write_friction=(not selective or SysIdParameterType.JOINT_FRICTION in joint_types),
                write_armature=(not selective or SysIdParameterType.JOINT_ARMATURE in joint_types),
            )

    def decode_theta_row(
        self,
        theta_row: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        num_dof: int,
        *,
        num_links: int | None = None,
    ) -> ParameterScaleState:
        """Map one optimizer theta row to legacy per-DOF scale vectors.

        Args:
            theta_row: One candidate parameter vector.
            param_entries: Selected parameter entries in theta order.
            num_dof: Number of articulated degrees of freedom.
            num_links: Number of articulated links.

        Returns:
            Legacy per-DOF friction, stiffness, damping, and link-mass scales.
        """
        del num_links
        friction, stiffness, damping, link_mass = per_dof_scale_vectors(num_dof, param_entries, theta_row)
        return ParameterScaleState(
            friction_scale=friction,
            stiffness_scale=stiffness,
            damping_scale=damping,
            link_mass_scale=link_mass,
        )

    def decode_theta_to_apply_state(
        self,
        theta_row: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        num_dof: int,
        *,
        num_links: int | None = None,
    ) -> ParameterApplyState:
        """Decode theta into full apply state using captured baselines.

        Args:
            theta_row: One candidate parameter vector.
            param_entries: Selected parameter entries in theta order.
            num_dof: Number of articulated degrees of freedom.
            num_links: Number of articulated links.

        Returns:
            Full parameter state ready to apply to the simulator.
        """
        links = num_links if num_links is not None else (self._num_links or 1)
        return decode_theta_to_apply_state(
            num_dof=num_dof,
            num_links=links,
            param_entries=param_entries,
            theta_row=theta_row,
            baseline_link_inertia_lc=self._baseline.link_inertia_lc,
        )

    def reparameterize_inertia_theta(
        self,
        theta: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> torch.Tensor:
        """Ensure Log-Cholesky inertia parameters decode to valid SPD matrices (REQ-15).

        Args:
            theta: Candidate parameter vector in ``param_entries`` order.
            param_entries: Selected parameter entries in theta order.

        Returns:
            Copy of ``theta`` with invalid inertia components restored to their baselines.
        """
        updated = theta.clone()
        inertia_indices_by_link: dict[int, list[int]] = {}
        for p_idx, entry in enumerate(param_entries):
            if entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
                inertia_indices_by_link.setdefault(entry.link_index, []).append(p_idx)

        for link_idx, parameter_indices in inertia_indices_by_link.items():
            baseline = self._baseline.link_inertia_lc.get(link_idx)
            if baseline is None:
                continue
            trial = baseline.copy()
            for idx in parameter_indices:
                entry = param_entries[idx]
                trial[entry.component_index] = float(updated[idx].item())
            try:
                with np.errstate(over="ignore", invalid="ignore"):
                    inertia = log_cholesky_to_inertia_matrix(trial)
                    eigenvalues = np.linalg.eigvalsh(inertia)
                scale = max(1.0, float(np.max(np.abs(eigenvalues))))
                valid = (
                    bool(np.all(np.isfinite(trial)))
                    and bool(np.all(np.isfinite(inertia)))
                    and bool(np.all(np.isfinite(eigenvalues)))
                    and float(np.min(eigenvalues)) > np.finfo(np.float64).eps * scale
                )
            except (ValueError, np.linalg.LinAlgError, OverflowError):
                valid = False
            if not valid:
                for idx in parameter_indices:
                    entry = param_entries[idx]
                    updated[idx] = updated.new_tensor(float(baseline[entry.component_index]))
        return updated

    def default_bounds_tensors(
        self,
        selected_entries: list[SysIdParameterEntry],
        *,
        device: torch.device | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return (initial, min, max) tensors aligned with ``selected_entries`` order.

        Args:
            selected_entries: Parameter entries requested in output tensor order.
            device: Device on which tensors are allocated.

        Returns:
            Initial, lower-bound, and upper-bound tensors in ``selected_entries`` order.
        """
        if not selected_entries:
            raise ValueError("Select at least one parameter.")
        entry_to_spec = {spec.entry: spec for spec in self._specs}
        initial: list[float] = []
        mins: list[float] = []
        maxs: list[float] = []
        for entry in selected_entries:
            spec = entry_to_spec.get(entry)
            if spec is None:
                raise KeyError(f"Selected entry not in parameter space: {entry!r}")
            if entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
                baseline = self._baseline.link_inertia_lc.get(entry.link_index)
                init_val = float(baseline[entry.component_index]) if baseline is not None else spec.default_initial
                initial.append(init_val)
            else:
                initial.append(spec.default_initial)
            mins.append(spec.default_min)
            maxs.append(spec.default_max)
        dev = device or torch.device("cpu")
        return (
            torch.tensor(initial, device=dev, dtype=torch.float32),
            torch.tensor(mins, device=dev, dtype=torch.float32),
            torch.tensor(maxs, device=dev, dtype=torch.float32),
        )
