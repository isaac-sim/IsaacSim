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

"""Parameter types and per-DOF entries for system identification optimization."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .inertia_param import LOG_CHOLESKY_DIM


class ParameterCategory(Enum):
    """SRD REQ-10: physically meaningful vs solver-specific parameters."""

    SYSID = "sysid"
    CALIBRATION = "calibration"


class SysIdParameterType(Enum):
    """Simulation parameters that can be optimized."""

    JOINT_FRICTION = "joint_friction"

    JOINT_STIFFNESS = "joint_stiffness"

    JOINT_DAMPING = "joint_damping"

    JOINT_INTEGRAL_GAIN = "joint_integral_gain"

    JOINT_ARMATURE = "joint_armature"

    ACTUATOR_COMMAND_DELAY_SECONDS = "actuator_command_delay_seconds"

    LINK_MASS = "link_mass"

    LINK_COM_OFFSET_X = "link_com_offset_x"

    LINK_COM_OFFSET_Y = "link_com_offset_y"

    LINK_COM_OFFSET_Z = "link_com_offset_z"

    LINK_INERTIA_LOG_CHOLESKY = "link_inertia_log_cholesky"

    JOINT_LIMIT_LOWER_SCALE = "joint_limit_lower_scale"

    JOINT_LIMIT_UPPER_SCALE = "joint_limit_upper_scale"


_PARAMETER_CATEGORY: dict[SysIdParameterType, ParameterCategory] = {
    SysIdParameterType.JOINT_FRICTION: ParameterCategory.SYSID,
    SysIdParameterType.JOINT_STIFFNESS: ParameterCategory.CALIBRATION,
    SysIdParameterType.JOINT_DAMPING: ParameterCategory.CALIBRATION,
    SysIdParameterType.JOINT_INTEGRAL_GAIN: ParameterCategory.CALIBRATION,
    SysIdParameterType.JOINT_ARMATURE: ParameterCategory.SYSID,
    SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS: ParameterCategory.CALIBRATION,
    SysIdParameterType.LINK_MASS: ParameterCategory.SYSID,
    SysIdParameterType.LINK_COM_OFFSET_X: ParameterCategory.SYSID,
    SysIdParameterType.LINK_COM_OFFSET_Y: ParameterCategory.SYSID,
    SysIdParameterType.LINK_COM_OFFSET_Z: ParameterCategory.SYSID,
    SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY: ParameterCategory.SYSID,
    SysIdParameterType.JOINT_LIMIT_LOWER_SCALE: ParameterCategory.CALIBRATION,
    SysIdParameterType.JOINT_LIMIT_UPPER_SCALE: ParameterCategory.CALIBRATION,
}


def default_category_for_type(param_type: SysIdParameterType) -> ParameterCategory:
    """Return the SRD category tag for a parameter type.

    Args:
        param_type: Parameter kind to classify.

    Returns:
        Default SysID or calibration category.
    """
    return _PARAMETER_CATEGORY[param_type]


# dof_index == -1 means a global (non per-DOF) entry, used for link mass scale.
# link_index == -1 means all links; component_index selects Log-Cholesky entries.

GLOBAL_DOF_INDEX = -1
GLOBAL_LINK_INDEX = -1


@dataclass(frozen=True)
class SysIdParameterEntry:
    """One optimizable scalar: a property on a specific DOF (or global link mass)."""

    param_type: SysIdParameterType

    dof_index: int

    link_index: int = GLOBAL_LINK_INDEX

    component_index: int = 0

    category: ParameterCategory | None = None

    def resolved_category(self) -> ParameterCategory:
        """Return explicit category or the default for ``param_type``.

        Returns:
            Explicit category when set, otherwise the type's default category.
        """
        if self.category is not None:
            return self.category
        return default_category_for_type(self.param_type)

    def display_name(self, num_joints: int) -> str:
        """Return a human-readable parameter label for UI tables.

        Args:
            num_joints: Joint count used to choose singular or indexed labels.

        Returns:
            Human-readable parameter name including its joint or link scope.
        """
        if self.param_type == SysIdParameterType.LINK_MASS:
            if self.link_index == GLOBAL_LINK_INDEX:
                return "Link mass scale (all links)"
            return f"Link mass scale (link {self.link_index + 1})"

        if self.param_type in (
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
        ):
            axis = self.param_type.value.rsplit("_", maxsplit=1)[-1].upper()
            link = "all links" if self.link_index == GLOBAL_LINK_INDEX else f"link {self.link_index + 1}"
            return f"COM offset {axis} ({link})"

        if self.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
            link = "all links" if self.link_index == GLOBAL_LINK_INDEX else f"link {self.link_index + 1}"
            return f"Inertia Log-Cholesky[{self.component_index}] ({link})"

        if self.param_type == SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS:
            label = "Actuator command delay"
            if self.dof_index == GLOBAL_DOF_INDEX:
                return f"{label} (all joints)"
            joint_label = f"Joint {self.dof_index + 1}" if num_joints > 1 else "Joint"
            return f"{joint_label} - {label}"

        joint_label = f"Joint {self.dof_index + 1}" if num_joints > 1 else "Joint"

        type_label = {
            SysIdParameterType.JOINT_FRICTION: "Friction",
            SysIdParameterType.JOINT_STIFFNESS: "Stiffness",
            SysIdParameterType.JOINT_DAMPING: "Damping",
            SysIdParameterType.JOINT_INTEGRAL_GAIN: "Integral gain",
            SysIdParameterType.JOINT_ARMATURE: "Armature",
            SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS: "Actuator command delay",
            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE: "Limit lower scale",
            SysIdParameterType.JOINT_LIMIT_UPPER_SCALE: "Limit upper scale",
        }[self.param_type]

        return f"{joint_label} - {type_label}"

    def short_type_label(self) -> str:
        """Compact label for grouped table rows (no joint prefix).

        Returns:
            Compact type label without a joint prefix.
        """
        if self.param_type == SysIdParameterType.LINK_MASS:
            if self.link_index == GLOBAL_LINK_INDEX:
                return "Link mass scale"
            return f"Link mass scale L{self.link_index + 1}"
        if self.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
            return f"Inertia LC[{self.component_index}]"
        if self.param_type == SysIdParameterType.LINK_COM_OFFSET_X:
            return "COM X"
        if self.param_type == SysIdParameterType.LINK_COM_OFFSET_Y:
            return "COM Y"
        if self.param_type == SysIdParameterType.LINK_COM_OFFSET_Z:
            return "COM Z"
        if self.param_type == SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS:
            return "Actuator delay"
        return {
            SysIdParameterType.JOINT_FRICTION: "Friction",
            SysIdParameterType.JOINT_STIFFNESS: "Stiffness",
            SysIdParameterType.JOINT_DAMPING: "Damping",
            SysIdParameterType.JOINT_INTEGRAL_GAIN: "Integral",
            SysIdParameterType.JOINT_ARMATURE: "Armature",
            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE: "Limit lower",
            SysIdParameterType.JOINT_LIMIT_UPPER_SCALE: "Limit upper",
        }[self.param_type]


@dataclass(frozen=True)
class SysIdParameterSpec:
    """UI and optimization metadata for a single parameter row."""

    entry: SysIdParameterEntry

    default_initial: float

    default_min: float

    default_max: float

    @property
    def param_type(self) -> SysIdParameterType:
        """Return the underlying parameter type."""
        return self.entry.param_type

    @property
    def dof_index(self) -> int:
        """Return the associated DOF index."""
        return self.entry.dof_index

    @property
    def category(self) -> ParameterCategory:
        """Return the resolved parameter category."""
        return self.entry.resolved_category()


_TYPE_DEFAULTS: dict[SysIdParameterType, tuple[float, float, float]] = {
    SysIdParameterType.JOINT_FRICTION: (1.0, 0.0, 10.0),
    SysIdParameterType.JOINT_STIFFNESS: (1.0, 0.01, 10.0),
    SysIdParameterType.JOINT_DAMPING: (1.0, 0.0, 100.0),
    SysIdParameterType.JOINT_INTEGRAL_GAIN: (0.0, 0.0, 100.0),
    SysIdParameterType.JOINT_ARMATURE: (0.01, 0.0, 1.0),
    SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS: (0.0, 0.0, 0.25),
    SysIdParameterType.LINK_MASS: (1.0, 0.1, 10.0),
    SysIdParameterType.LINK_COM_OFFSET_X: (0.0, -0.05, 0.05),
    SysIdParameterType.LINK_COM_OFFSET_Y: (0.0, -0.05, 0.05),
    SysIdParameterType.LINK_COM_OFFSET_Z: (0.0, -0.05, 0.05),
    SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY: (0.0, -3.0, 3.0),
    SysIdParameterType.JOINT_LIMIT_LOWER_SCALE: (1.0, 0.5, 1.5),
    SysIdParameterType.JOINT_LIMIT_UPPER_SCALE: (1.0, 0.5, 1.5),
}


def default_bounds_for_type(param_type: SysIdParameterType) -> tuple[float, float]:
    """Return the range a parameter type is solved within by default.

    A run's :class:`SysIdParameterSpec` rows may narrow these, so treat the
    result as the widest a candidate can reach absent an explicit override.

    Args:
        param_type: Parameter kind to look up.

    Returns:
        Minimum and maximum value.
    """
    _initial, minimum, maximum = _TYPE_DEFAULTS[param_type]
    return minimum, maximum


def build_dof_parameter_specs(num_joints: int) -> tuple[SysIdParameterSpec, ...]:
    """Build basic joint rows plus one global link-mass row.

    Args:
        num_joints: Number of joints N from the telemetry CSV.

    Returns:
        Tuple of ``5*N + 1`` parameter specs in stable order: friction, stiffness,
        damping, integral gain, and armature per joint, followed by global link mass.
    """
    if num_joints < 1:
        raise ValueError("num_joints must be at least 1.")

    specs: list[SysIdParameterSpec] = []
    joint_types = (
        SysIdParameterType.JOINT_FRICTION,
        SysIdParameterType.JOINT_STIFFNESS,
        SysIdParameterType.JOINT_DAMPING,
        SysIdParameterType.JOINT_INTEGRAL_GAIN,
        SysIdParameterType.JOINT_ARMATURE,
    )

    for dof_index in range(num_joints):
        for ptype in joint_types:
            initial, min_v, max_v = _TYPE_DEFAULTS[ptype]
            specs.append(
                SysIdParameterSpec(
                    entry=SysIdParameterEntry(
                        param_type=ptype,
                        dof_index=dof_index,
                        category=default_category_for_type(ptype),
                    ),
                    default_initial=initial,
                    default_min=min_v,
                    default_max=max_v,
                )
            )

    initial, min_v, max_v = _TYPE_DEFAULTS[SysIdParameterType.LINK_MASS]
    specs.append(
        SysIdParameterSpec(
            entry=SysIdParameterEntry(
                param_type=SysIdParameterType.LINK_MASS,
                dof_index=GLOBAL_DOF_INDEX,
                category=ParameterCategory.SYSID,
            ),
            default_initial=initial,
            default_min=min_v,
            default_max=max_v,
        )
    )

    return tuple(specs)


def build_extended_parameter_specs(
    num_joints: int,
    num_links: int,
    *,
    include_basic: bool = True,
    include_com_offsets: bool = False,
    include_inertia_log_cholesky: bool = False,
    include_joint_limit_scales: bool = False,
    include_per_link_mass: bool = False,
    include_command_delay: bool = False,
) -> tuple[SysIdParameterSpec, ...]:
    """Build an extended parameter registry for advanced SysID workflows.

    Args:
        num_joints: Number of articulation degrees of freedom.
        num_links: Number of rigid links available for link-scoped rows.
        include_basic: Include friction, drive, armature, and mass rows.
        include_com_offsets: Include per-link center-of-mass offsets.
        include_inertia_log_cholesky: Include six inertia coordinates per link.
        include_joint_limit_scales: Include lower and upper joint-limit scales.
        include_per_link_mass: Replace the global mass scale with per-link scales.
        include_command_delay: Include per-joint actuator-delay calibration rows.

    Returns:
        Parameter specifications in deterministic optimizer-column order.
    """
    if num_joints < 1 or num_links < 1:
        raise ValueError("num_joints and num_links must be at least 1.")

    specs: list[SysIdParameterSpec] = []
    if include_basic:
        specs.extend(build_dof_parameter_specs(num_joints))
        if include_per_link_mass:
            # Per-link mass rows cover every link and are applied after the
            # basic global row. Keeping both would make the global row a dead
            # optimizer variable because every element is overwritten later.
            specs = [spec for spec in specs if spec.param_type != SysIdParameterType.LINK_MASS]

    def _append(entry: SysIdParameterEntry) -> None:
        initial, min_v, max_v = _TYPE_DEFAULTS[entry.param_type]
        specs.append(
            SysIdParameterSpec(
                entry=entry,
                default_initial=initial,
                default_min=min_v,
                default_max=max_v,
            )
        )

    if include_per_link_mass:
        for link_index in range(num_links):
            _append(
                SysIdParameterEntry(
                    param_type=SysIdParameterType.LINK_MASS,
                    dof_index=GLOBAL_DOF_INDEX,
                    link_index=link_index,
                    category=ParameterCategory.SYSID,
                )
            )

    if include_command_delay:
        _append(
            SysIdParameterEntry(
                param_type=SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
                dof_index=GLOBAL_DOF_INDEX,
                category=ParameterCategory.CALIBRATION,
            )
        )

    if include_com_offsets:
        for link_index in range(num_links):
            for ptype in (
                SysIdParameterType.LINK_COM_OFFSET_X,
                SysIdParameterType.LINK_COM_OFFSET_Y,
                SysIdParameterType.LINK_COM_OFFSET_Z,
            ):
                _append(
                    SysIdParameterEntry(
                        param_type=ptype,
                        dof_index=GLOBAL_DOF_INDEX,
                        link_index=link_index,
                        category=ParameterCategory.SYSID,
                    )
                )

    if include_inertia_log_cholesky:
        for link_index in range(num_links):
            for component_index in range(LOG_CHOLESKY_DIM):
                _append(
                    SysIdParameterEntry(
                        param_type=SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
                        dof_index=GLOBAL_DOF_INDEX,
                        link_index=link_index,
                        component_index=component_index,
                        category=ParameterCategory.SYSID,
                    )
                )

    if include_joint_limit_scales:
        for dof_index in range(num_joints):
            for ptype in (
                SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
                SysIdParameterType.JOINT_LIMIT_UPPER_SCALE,
            ):
                _append(
                    SysIdParameterEntry(
                        param_type=ptype,
                        dof_index=dof_index,
                        category=ParameterCategory.CALIBRATION,
                    )
                )

    if not specs:
        raise ValueError("At least one parameter group must be enabled.")
    return tuple(specs)


def entries_from_specs(
    specs: tuple[SysIdParameterSpec, ...] | list[SysIdParameterSpec],
) -> list[SysIdParameterEntry]:
    """Extract entries in spec order.

    Args:
        specs: Parameter specifications to project.

    Returns:
        Entry metadata in the same order as ``specs``.
    """
    return [s.entry for s in specs]
