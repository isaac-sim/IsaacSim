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

"""Joint inspector window.

A standalone, multi-instance inspection tool for joints belonging to prims with
``IsaacRobotAPI`` applied. Distinct from the Robot Schema authoring widgets:
this is a runtime tuning surface, not a schema editor.

The window exposes a configurable per-joint table whose columns are picked from
a categorized menu (Joint Limits, Drives, Performance Envelope, Joint State).
Each cell is a free-form ``ui.FloatDrag`` bound to the underlying USD attribute;
cells whose backing API is missing on the joint are rendered empty to avoid
implying a bogus zero value. Multiple joint rows can be selected at once, and
editing one cell of a selected row applies the new value to the same column on
every selected row.

Public surface:

- :class:`JointInspectorWindow` -- a single inspector window bound to one robot.
- :class:`JointInspectorWindowManager` -- creates/tracks multiple windows so the
  user can compare several robots side by side; the menu opens or focuses the
  primary window, and an in-window button spawns additional inspectors.
"""

from __future__ import annotations

import asyncio
import fnmatch
from collections.abc import Callable
from typing import NamedTuple

import carb
import omni.kit.app
import omni.ui as ui
import omni.usd
from pxr import PhysxSchema, Tf, Usd, UsdPhysics
from usd.schema.isaac import robot_schema

from . import style
from .style import BG_HEADER as _BG_HEADER
from .style import BG_INPUT as _BG_INPUT
from .style import BG_PANEL as _BG_PANEL
from .style import INSPECTOR_HEADER_HEIGHT as _INSPECTOR_HEADER_HEIGHT
from .style import INSPECTOR_ROW_HEIGHT as _INSPECTOR_ROW_HEIGHT
from .style import TEXT_DIM as _TEXT_DIM
from .style import TEXT_PRIMARY as _TEXT_PRIMARY
from .style import TOOLTIP_STYLE as _TOOLTIP_STYLE

# Kit-shared glyph tokens (resolved by carb's icon path lookup).
_ICON_SEARCH = "${glyphs}/menu_search.svg"
_ICON_CLEAR = "${glyphs}/times_circle.svg"
_ICON_MENU = "${glyphs}/list.svg"
_ICON_REFRESH = "${glyphs}/menu_refresh.svg"
_ICON_PLUS = "${glyphs}/plus.svg"

# Shared NVIDIA Sans typography for every Joint Inspector widget. The font
# ships with Kit at ``kit/resources/fonts/`` and is resolved through carb's
# ``${kit}`` token. Sizes are deliberately +1 over the prior defaults to
# improve legibility (the categorized columns popup gets +2 because its dense
# checkbox rows looked blurry at the previous size).
_UI_FONT = "${kit}/resources/fonts/NVIDIASans_Md.ttf"
_FONT_SIZE_SMALL = 12  # was 11 (group headers in the columns popup)
_FONT_SIZE_BODY = 13  # was 12 (toolbar, table cells, headers, info text)
_FONT_SIZE_MENU = 14  # was 13 (column row labels in the popup)

WINDOW_TITLE = "Joint Inspector"
"""Title of the primary Joint Inspector window."""


# --- Stage / robot helpers ---
def _list_robot_prims(stage: Usd.Stage | None) -> list[Usd.Prim]:
    """Return all prims on ``stage`` that have ``IsaacRobotAPI`` applied."""
    if stage is None:
        return []
    return [p for p in stage.Traverse() if p.HasAPI(robot_schema.Classes.ROBOT_API.value)]


def _is_inspectable_joint(prim: Usd.Prim) -> bool:
    """Return True for movable physics joints worth listing in the inspector.

    Excludes non-joint prims and ``UsdPhysics.FixedJoint`` (zero DOF: nothing
    to inspect, and it would only contribute empty cells).
    """
    if not prim or not prim.IsValid():
        return False
    if not prim.IsA(UsdPhysics.Joint):
        return False
    if prim.IsA(UsdPhysics.FixedJoint):
        return False
    return True


def _joint_prims_for_robot(robot_prim: Usd.Prim) -> list[Usd.Prim]:
    """Return the inspectable joint prims belonging to ``robot_prim``.

    Resolves the ``isaac:robotJoints`` relationship targets through the prim's
    stage. Falls back to scanning descendants for ``IsaacJointAPI`` when the
    relationship is empty. Filters out invalid prims, non-joint prims and
    fixed joints (see :func:`_is_inspectable_joint`).
    """
    if not robot_prim or not robot_prim.IsValid():
        return []
    stage = robot_prim.GetStage()
    prims: list[Usd.Prim] = []
    seen: set[str] = set()
    joints_rel = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
    if joints_rel:
        for target in joints_rel.GetTargets():
            prim = stage.GetPrimAtPath(target)
            path_str = str(target)
            if path_str in seen or not _is_inspectable_joint(prim):
                continue
            prims.append(prim)
            seen.add(path_str)
    if not prims:
        for child in Usd.PrimRange(robot_prim):
            if child.HasAPI(robot_schema.Classes.JOINT_API.value):
                path_str = str(child.GetPath())
                if path_str in seen or not _is_inspectable_joint(child):
                    continue
                prims.append(child)
                seen.add(path_str)
    return prims


# --- Per-joint attribute resolvers ---
# Axis tokens used by the multi-apply schemas this inspector touches. The first
# six are D6 axes; ``angular``/``linear`` are the natural axes of revolute and
# prismatic joints respectively. ``""`` is the canonical empty token used when
# the joint is single-DOF and the catalogue collapses the axis dimension.
_D6_AXES: tuple[str, ...] = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")
_AXIS_PRIORITY: tuple[str, ...] = ("angular", "linear", *_D6_AXES)


def _natural_axis_for_joint(joint_prim: Usd.Prim) -> str | None:
    """Return ``"angular"`` for ``RevoluteJoint``, ``"linear"`` for ``PrismaticJoint``."""
    if not joint_prim or not joint_prim.IsValid():
        return None
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    return None


def _applied_api_instances(joint_prim: Usd.Prim, api_name: str) -> list[str]:
    """List multi-apply instance tokens of ``api_name`` on ``joint_prim``.

    For example, ``_applied_api_instances(joint, "PhysicsDriveAPI")`` returns
    ``["angular"]`` for a revolute drive joint, or ``["transX", "rotZ"]`` for a
    D6 joint that has DriveAPIs applied to those axes. Empty list when the API
    is not applied. Single-apply APIs return ``[""]`` when applied.
    """
    if not joint_prim or not joint_prim.IsValid():
        return []
    instances: list[str] = []
    prefix = f"{api_name}:"
    for schema in joint_prim.GetAppliedSchemas():
        if schema == api_name:
            instances.append("")
        elif schema.startswith(prefix):
            instances.append(schema[len(prefix) :])
    return instances


def _api_axes_for_joint(joint_prim: Usd.Prim, api_name: str) -> list[str]:
    """Return axes for a multi-apply API, sorted by ``_AXIS_PRIORITY``.

    Excludes the empty single-apply token so callers iterating over per-axis
    columns don't produce duplicate "no axis" entries.
    """
    instances = [a for a in _applied_api_instances(joint_prim, api_name) if a]
    return sorted(set(instances), key=lambda a: _AXIS_PRIORITY.index(a) if a in _AXIS_PRIORITY else 99)


def _attr_value(attr: Usd.Attribute | None, default: float = 0.0) -> float:
    if not attr:
        return default
    val = attr.Get()
    if val is None:
        return default
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _set_attr_value(attr: Usd.Attribute | None, value: float) -> None:
    if not attr:
        return
    try:
        attr.Set(float(value))
    except (TypeError, ValueError, Tf.ErrorException) as exc:
        carb.log_error(f"Failed to write {attr.GetPath()}: {exc}")


def _array_first_attr(attr: Usd.Attribute | None) -> float:
    """Return ``attr[0]`` as a float, or 0.0 if empty / not an array."""
    if not attr:
        return 0.0
    val = attr.Get()
    if val is None:
        return 0.0
    try:
        if hasattr(val, "__len__") and len(val) > 0:
            return float(val[0])
        return float(val)
    except (TypeError, ValueError):
        return 0.0


def _set_array_first(attr: Usd.Attribute | None, value: float) -> None:
    """Update ``attr[0]`` in place, leaving the rest of the array untouched."""
    if not attr:
        return
    try:
        current = attr.Get()
        if current is None:
            return
        new_val = list(current) if hasattr(current, "__iter__") else [float(current)]
        if not new_val:
            return
        new_val[0] = float(value)
        attr.Set(type(current)(new_val) if not isinstance(current, list) else new_val)
    except (TypeError, ValueError, Tf.ErrorException) as exc:
        carb.log_error(f"Failed to update first element of {attr.GetPath()}: {exc}")


# --- Per-API attribute resolvers ---
def _resolve_row_axis(joint_prim: Usd.Prim, axis: str, api_name: str) -> str | None:
    """Return the actual axis to read for a row whose column was collapsed.

    When the column has been coalesced to a single slot (``axis == ""``) the
    cell needs to pick the per-joint axis: prefer the one axis the joint
    authored under ``api_name`` (handles D6 joints with one DOF authored), and
    fall back to the joint's natural axis (``angular``/``linear``) so revolute
    + prismatic joints can share the column. Returns ``None`` if neither is
    available.
    """
    if axis:
        return axis
    authored = _api_axes_for_joint(joint_prim, api_name)
    if len(authored) == 1:
        return authored[0]
    natural = _natural_axis_for_joint(joint_prim)
    if natural and natural in authored:
        return natural
    return natural


def _drive_attr(joint_prim: Usd.Prim, axis: str, attr_name: str) -> Usd.Attribute | None:
    """Resolve a ``UsdPhysics.DriveAPI`` attribute for ``axis`` on ``joint_prim``.

    When ``axis`` is empty the row's column has been coalesced; pick this
    joint's authored axis (or its natural axis as a fallback).
    """
    actual_axis = _resolve_row_axis(joint_prim, axis, "PhysicsDriveAPI")
    if not actual_axis:
        return None
    drive = UsdPhysics.DriveAPI(joint_prim, actual_axis)
    if not drive:
        return None
    if attr_name == "targetPosition":
        return drive.GetTargetPositionAttr()
    if attr_name == "targetVelocity":
        return drive.GetTargetVelocityAttr()
    if attr_name == "stiffness":
        return drive.GetStiffnessAttr()
    if attr_name == "damping":
        return drive.GetDampingAttr()
    if attr_name == "maxForce":
        return drive.GetMaxForceAttr()
    return None


def _perf_envelope_attr(joint_prim: Usd.Prim, axis: str, attr_name: str) -> Usd.Attribute | None:
    """Resolve a ``PhysxDrivePerformanceEnvelopeAPI`` axis attribute."""
    actual_axis = _resolve_row_axis(joint_prim, axis, "PhysxDrivePerformanceEnvelopeAPI")
    if not actual_axis:
        return None
    if f"PhysxDrivePerformanceEnvelopeAPI:{actual_axis}" not in joint_prim.GetAppliedSchemas():
        return None
    attr = joint_prim.GetAttribute(f"physxDrivePerformanceEnvelope:{actual_axis}:{attr_name}")
    return attr if attr and attr.IsValid() else None


def _joint_state_attr(joint_prim: Usd.Prim, axis: str, attr_name: str) -> Usd.Attribute | None:
    """Resolve a ``PhysxSchema.JointStateAPI`` axis attribute."""
    actual_axis = _resolve_row_axis(joint_prim, axis, "PhysicsJointStateAPI")
    if not actual_axis:
        return None
    js = PhysxSchema.JointStateAPI(joint_prim, actual_axis)
    if not js:
        return None
    if attr_name == "position":
        return js.GetPositionAttr()
    if attr_name == "velocity":
        return js.GetVelocityAttr()
    return None


def _lower_limit_attr(joint_prim: Usd.Prim) -> Usd.Attribute | None:
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        return UsdPhysics.RevoluteJoint(joint_prim).GetLowerLimitAttr()
    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        return UsdPhysics.PrismaticJoint(joint_prim).GetLowerLimitAttr()
    return None


def _upper_limit_attr(joint_prim: Usd.Prim) -> Usd.Attribute | None:
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        return UsdPhysics.RevoluteJoint(joint_prim).GetUpperLimitAttr()
    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        return UsdPhysics.PrismaticJoint(joint_prim).GetUpperLimitAttr()
    return None


def _max_joint_velocity_attr(joint_prim: Usd.Prim) -> Usd.Attribute | None:
    if not joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        return None
    return PhysxSchema.PhysxJointAPI(joint_prim).GetMaxJointVelocityAttr()


def _mjc_joint_attr(joint_prim: Usd.Prim, attr_name: str) -> Usd.Attribute | None:
    """Resolve a ``MjcJointAPI`` attribute by name (single-apply schema)."""
    if "MjcJointAPI" not in joint_prim.GetAppliedSchemas():
        return None
    attr = joint_prim.GetAttribute(f"mjc:{attr_name}")
    return attr if attr and attr.IsValid() else None


# --- Column catalogue ---
# Backends drive the show/hide pill buttons in the columns popup. ``USD`` is
# always available (it's the Joint Limits group, which lives on
# ``UsdPhysics.RevoluteJoint``/``PrismaticJoint``).
BACKEND_USD = "USD"
BACKEND_PHYSX = "PhysX"
BACKEND_MUJOCO = "MuJoCo"


class _ColumnSpec(NamedTuple):
    """Static description of a single inspector column.

    Attributes:
        id: Stable identifier used in state and dict keys.
        label: Header label rendered in the table.
        group: Display group in the categorized column-picker menu.
        backend: Which simulation backend the API belongs to. Used for the
            "PhysX"/"MuJoCo" pill toggles in the columns popup. ``USD`` columns
            (joint limits) are always visible regardless of pill state.
        resolver: ``f(joint_prim, axis) -> Usd.Attribute | None``. ``axis`` is
            ``""`` for non-axed columns (USD limits, MjcJointAPI, etc.).
        api_test: ``f(joint_prim, axis) -> bool`` returning True when the column
            backs onto an applied API on this joint+axis. Drives the per-cell
            disable and the menu availability.
        axis_api: Multi-apply schema name when the column lives on a per-axis
            API (DriveAPI, JointStateAPI, PhysxDrivePerformanceEnvelopeAPI).
            ``None`` for single-apply / no-axis columns.
        default: Initial value used when the attribute is missing on the joint.
        step: Drag increment for ``ui.FloatDrag``.
        get_value: Optional override; defaults to ``_attr_value``. Used for
            array-typed MuJoCo attributes that surface only the dominant value.
        set_value: Optional override; defaults to ``_set_attr_value``. Mirror.
    """

    id: str
    label: str
    group: str
    backend: str
    resolver: Callable[[Usd.Prim, str], Usd.Attribute | None]
    api_test: Callable[[Usd.Prim, str], bool]
    axis_api: str | None = None
    default: float = 0.0
    step: float = 0.01
    get_value: Callable[[Usd.Attribute | None], float] = _attr_value
    set_value: Callable[[Usd.Attribute | None, float], None] = _set_attr_value


# -- Per-API availability predicates -------------------------------------


def _has_drive_axis(joint: Usd.Prim, axis: str) -> bool:
    if not axis:
        return bool(_api_axes_for_joint(joint, "PhysicsDriveAPI"))
    return f"PhysicsDriveAPI:{axis}" in joint.GetAppliedSchemas()


def _has_perf_envelope_axis(joint: Usd.Prim, axis: str) -> bool:
    if not axis:
        return bool(_api_axes_for_joint(joint, "PhysxDrivePerformanceEnvelopeAPI"))
    return f"PhysxDrivePerformanceEnvelopeAPI:{axis}" in joint.GetAppliedSchemas()


def _has_joint_state_axis(joint: Usd.Prim, axis: str) -> bool:
    if not axis:
        return bool(_api_axes_for_joint(joint, "PhysicsJointStateAPI"))
    return f"PhysicsJointStateAPI:{axis}" in joint.GetAppliedSchemas()


def _has_usd_limits(joint: Usd.Prim, axis: str) -> bool:
    return joint.IsA(UsdPhysics.RevoluteJoint) or joint.IsA(UsdPhysics.PrismaticJoint)


def _has_physx_joint_api(joint: Usd.Prim, axis: str) -> bool:
    return joint.HasAPI(PhysxSchema.PhysxJointAPI)


def _has_mjc_joint_api(joint: Usd.Prim, axis: str) -> bool:
    return "MjcJointAPI" in joint.GetAppliedSchemas()


# -- Column catalogue ---------------------------------------------------


def _drive_lambda(name: str) -> Callable[[Usd.Prim, str], Usd.Attribute | None]:
    return lambda j, axis: _drive_attr(j, axis, name)


def _perf_lambda(name: str) -> Callable[[Usd.Prim, str], Usd.Attribute | None]:
    return lambda j, axis: _perf_envelope_attr(j, axis, name)


def _state_lambda(name: str) -> Callable[[Usd.Prim, str], Usd.Attribute | None]:
    return lambda j, axis: _joint_state_attr(j, axis, name)


def _mjc_lambda(name: str) -> Callable[[Usd.Prim, str], Usd.Attribute | None]:
    return lambda j, _axis: _mjc_joint_attr(j, name)


_COLUMNS: list[_ColumnSpec] = [
    # Joint Limits (UsdPhysics + PhysxJointAPI) ---------------------
    _ColumnSpec(
        "limit_lower",
        "Position Min",
        "Joint Limits",
        BACKEND_USD,
        lambda j, _a: _lower_limit_attr(j),
        _has_usd_limits,
    ),
    _ColumnSpec(
        "limit_upper",
        "Position Max",
        "Joint Limits",
        BACKEND_USD,
        lambda j, _a: _upper_limit_attr(j),
        _has_usd_limits,
    ),
    _ColumnSpec(
        "max_joint_velocity",
        "Velocity Max",
        "Joint Limits",
        BACKEND_PHYSX,
        lambda j, _a: _max_joint_velocity_attr(j),
        _has_physx_joint_api,
        step=0.1,
    ),
    # Drives (UsdPhysics.DriveAPI, multi-apply per axis) -----------
    _ColumnSpec(
        "drive_max_force",
        "Max Force",
        "Drives",
        BACKEND_USD,
        _drive_lambda("maxForce"),
        _has_drive_axis,
        axis_api="PhysicsDriveAPI",
        step=1.0,
    ),
    _ColumnSpec(
        "drive_target_position",
        "Target Position",
        "Drives",
        BACKEND_USD,
        _drive_lambda("targetPosition"),
        _has_drive_axis,
        axis_api="PhysicsDriveAPI",
    ),
    _ColumnSpec(
        "drive_target_velocity",
        "Target Velocity",
        "Drives",
        BACKEND_USD,
        _drive_lambda("targetVelocity"),
        _has_drive_axis,
        axis_api="PhysicsDriveAPI",
    ),
    _ColumnSpec(
        "drive_stiffness",
        "Stiffness",
        "Drives",
        BACKEND_USD,
        _drive_lambda("stiffness"),
        _has_drive_axis,
        axis_api="PhysicsDriveAPI",
        step=1.0,
    ),
    _ColumnSpec(
        "drive_damping",
        "Damping",
        "Drives",
        BACKEND_USD,
        _drive_lambda("damping"),
        _has_drive_axis,
        axis_api="PhysicsDriveAPI",
        step=1.0,
    ),
    # Performance Envelope (PhysxDrivePerformanceEnvelopeAPI) ------
    _ColumnSpec(
        "perf_max_actuator_velocity",
        "Max Actuator Velocity",
        "Performance Envelope",
        BACKEND_PHYSX,
        _perf_lambda("maxActuatorVelocity"),
        _has_perf_envelope_axis,
        axis_api="PhysxDrivePerformanceEnvelopeAPI",
        step=0.1,
    ),
    _ColumnSpec(
        "perf_speed_effort_gradient",
        "Speed-Effort Gradient",
        "Performance Envelope",
        BACKEND_PHYSX,
        _perf_lambda("speedEffortGradient"),
        _has_perf_envelope_axis,
        axis_api="PhysxDrivePerformanceEnvelopeAPI",
    ),
    _ColumnSpec(
        "perf_velocity_resistance",
        "Velocity-Dependent Resistance",
        "Performance Envelope",
        BACKEND_PHYSX,
        _perf_lambda("velocityDependentResistance"),
        _has_perf_envelope_axis,
        axis_api="PhysxDrivePerformanceEnvelopeAPI",
    ),
    # Joint State (PhysxSchema.JointStateAPI) ----------------------
    _ColumnSpec(
        "state_position",
        "State Position",
        "Joint State",
        BACKEND_PHYSX,
        _state_lambda("position"),
        _has_joint_state_axis,
        axis_api="PhysicsJointStateAPI",
    ),
    _ColumnSpec(
        "state_velocity",
        "State Velocity",
        "Joint State",
        BACKEND_PHYSX,
        _state_lambda("velocity"),
        _has_joint_state_axis,
        axis_api="PhysicsJointStateAPI",
    ),
    # MuJoCo (MjcJointAPI, single-apply) ---------------------------
    _ColumnSpec(
        "mjc_armature",
        "Armature",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("armature"),
        _has_mjc_joint_api,
        step=0.01,
    ),
    _ColumnSpec(
        "mjc_damping", "Damping", "MuJoCo Joint", BACKEND_MUJOCO, _mjc_lambda("damping"), _has_mjc_joint_api, step=0.1
    ),
    _ColumnSpec(
        "mjc_stiffness",
        "Stiffness",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("stiffness"),
        _has_mjc_joint_api,
        step=1.0,
    ),
    _ColumnSpec(
        "mjc_frictionloss",
        "Friction Loss",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("frictionloss"),
        _has_mjc_joint_api,
        step=0.01,
    ),
    _ColumnSpec(
        "mjc_springref",
        "Spring Ref",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("springref"),
        _has_mjc_joint_api,
        step=0.01,
    ),
    _ColumnSpec(
        "mjc_solreflimit_timeconst",
        "SolRef Limit (timeconst)",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("solreflimit"),
        _has_mjc_joint_api,
        step=0.001,
        get_value=_array_first_attr,
        set_value=_set_array_first,
    ),
    _ColumnSpec(
        "mjc_solimplimit_dmin",
        "SolImp Limit (dmin)",
        "MuJoCo Joint",
        BACKEND_MUJOCO,
        _mjc_lambda("solimplimit"),
        _has_mjc_joint_api,
        step=0.01,
        get_value=_array_first_attr,
        set_value=_set_array_first,
    ),
]
"""Ordered catalogue of all inspector columns. Order drives both menu and table."""

_COLUMN_BY_ID: dict[str, _ColumnSpec] = {c.id: c for c in _COLUMNS}

_GROUPS: list[str] = list(dict.fromkeys(c.group for c in _COLUMNS))

_DEFAULT_VISIBLE_COLUMN_IDS: list[str] = [
    "limit_lower",
    "limit_upper",
    "drive_target_position",
    "drive_stiffness",
    "drive_damping",
    "state_position",
    "mjc_armature",
    "mjc_damping",
    "mjc_stiffness",
    "mjc_frictionloss",
]


# --- Per-axis catalogue resolution ---
def _column_axes_for_joints(col: _ColumnSpec, joints: list[Usd.Prim]) -> list[str]:
    """Return the per-axis suffixes to display for ``col`` across ``joints``.

    For axis-bound columns the column collapses to a single coalesced entry
    (``[""]``) whenever every joint authoring the API has at most one axis
    authored. Each row's cell then resolves its own axis at read time (see
    :func:`_resolve_row_axis`). The column only fans out into per-axis
    columns when *some* joint authors two or more axes under the API
    (in practice, multi-DOF ``D6Joint`` joints), in which case one column
    is rendered per distinct axis token across the whole joint set.

    For non-axis columns (no ``axis_api``): always returns ``[""]``.
    """
    if not col.axis_api:
        return [""]
    per_joint_axes = [_api_axes_for_joint(j, col.axis_api) for j in joints]
    distinct = sorted(
        {a for axes in per_joint_axes for a in axes},
        key=lambda a: _AXIS_PRIORITY.index(a) if a in _AXIS_PRIORITY else 99,
    )
    if not distinct:
        return []
    has_multi_dof_joint = any(len(axes) > 1 for axes in per_joint_axes)
    if not has_multi_dof_joint:
        return [""]
    return distinct


class _ResolvedColumn(NamedTuple):
    """A concrete column ready to render: a :class:`_ColumnSpec` + axis."""

    spec: _ColumnSpec
    axis: str

    @property
    def render_id(self) -> str:
        return f"{self.spec.id}::{self.axis}" if self.axis else self.spec.id

    @property
    def label(self) -> str:
        if not self.axis:
            return self.spec.label
        return f"{self.spec.label} [{self.axis}]"


def _resolve_columns(visible_ids: set[str], joints: list[Usd.Prim]) -> list[_ResolvedColumn]:
    """Expand the visible column ids into concrete per-axis :class:`_ResolvedColumn`s.

    Preserves the catalogue order, fans out per-axis columns when ``joints``
    require it, and silently drops columns whose API is not applied on any
    joint in the set.
    """
    out: list[_ResolvedColumn] = []
    for spec in _COLUMNS:
        if spec.id not in visible_ids:
            continue
        for axis in _column_axes_for_joints(spec, joints):
            out.append(_ResolvedColumn(spec, axis))
    return out


def _column_available_for(spec: _ColumnSpec, joints: list[Usd.Prim]) -> bool:
    """True if at least one joint backs ``spec`` (used by the columns menu)."""
    if not joints:
        return False
    if spec.axis_api:
        return any(_api_axes_for_joint(j, spec.axis_api) for j in joints)
    return any(spec.api_test(j, "") for j in joints)


# --- Categorized columns combo ---
class _ColumnsMenu:
    """Categorized checkbox popup that toggles which inspector columns are visible.

    Owns its popup window only; the host wires whatever widget triggers
    :meth:`toggle` (typically a hamburger image button). The popup positions
    itself relative to the supplied ``anchor_widget`` and keeps Kit's popup
    auto-dismiss semantics so clicking outside closes it.

    The popup header carries two pill buttons (PhysX / MuJoCo) that mirror the
    view's per-backend visibility set. Toggling a backend hides every column
    belonging to that backend across the table, but does not flip the
    per-column checkboxes themselves: when the backend pill is re-enabled the
    previously checked columns return.

    Args:
        columns: Ordered :class:`_ColumnSpec` list to render in the menu.
        active_ids: Mutable set of currently-active column ids; toggled in
            place so the host can read the result without subscribing.
        active_backends: Mutable set of currently-active backend names.
        on_changed_fn: Invoked after any item's checked state changes (or a
            backend toggles).
        availability_fn: ``f() -> set[str]`` returning the column ids backed by
            an applied API on at least one joint of the current robot. Items
            outside this set render disabled and ignore clicks.
    """

    _MIN_WIDTH = 280
    _BACKENDS_IN_HEADER: tuple[str, ...] = (BACKEND_PHYSX, BACKEND_MUJOCO)

    def __init__(
        self,
        columns: list[_ColumnSpec],
        active_ids: set[str],
        active_backends: set[str],
        on_changed_fn: Callable[[], None],
        availability_fn: Callable[[], set[str]] | None = None,
    ) -> None:
        self._columns = columns
        self._active = active_ids
        self._active_backends = active_backends
        self._on_changed = on_changed_fn
        self._availability_fn = availability_fn or (lambda: {c.id for c in columns})
        self._popup: ui.Window | None = None
        self._popup_frame: ui.Frame | None = None

    def toggle(self, anchor_widget: object) -> None:
        """Show or hide the popup, anchoring it under ``anchor_widget``."""
        if self._popup and self._popup.visible:
            self._popup.visible = False
            return
        if self._popup is None:
            self._build_window(anchor_widget)
        self._position_popup(anchor_widget)
        if self._popup_frame is not None:
            self._popup_frame.rebuild()
        self._popup.visible = True

    def _build_window(self, anchor_widget: object) -> None:
        item_h = 26
        header_h = 22
        backend_h = 26
        spacer_h = 6
        n_groups = len(_GROUPS)
        pop_h = 8 + backend_h + 6 + n_groups * (header_h + spacer_h) + len(self._columns) * item_h + 8
        pop_w = max(getattr(anchor_widget, "computed_width", 0) or 0, self._MIN_WIDTH)
        self._popup = ui.Window(
            "##joint_inspector_columns_popup",
            width=pop_w,
            height=pop_h,
            padding_x=0,
            padding_y=0,
            flags=(
                ui.WINDOW_FLAGS_POPUP
                | ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_MOVE
            ),
        )
        self._popup.frame.set_style({"Window": {"background_color": _BG_INPUT, "border_radius": 2}})
        with self._popup.frame:
            self._popup_frame = ui.Frame(build_fn=self._build_popup_contents)

    def _build_popup_contents(self) -> None:
        item_h = 26
        header_h = 22
        spacer_h = 6
        available = self._availability_fn()
        with ui.VStack(spacing=0):
            ui.Spacer(height=8)
            self._build_backend_header()
            ui.Spacer(height=6)
            for group in _GROUPS:
                with ui.HStack(height=header_h):
                    ui.Spacer(width=10)
                    ui.Label(
                        group.upper(),
                        height=header_h,
                        style={"color": _TEXT_DIM, "font": _UI_FONT, "font_size": _FONT_SIZE_SMALL},
                    )
                for col in [c for c in self._columns if c.group == group]:
                    is_available = col.id in available
                    # Items remain clickable even when their API isn't applied
                    # on the current robot or their backend pill is off, so the
                    # user can pre-select columns for a different robot. The
                    # label dims to signal "not visible right now"; a tooltip
                    # explains why.
                    label_color = _TEXT_PRIMARY if is_available else _TEXT_DIM
                    tooltip = (
                        f"{col.label}"
                        if is_available
                        else f"{col.label} (no joint on the current robot has the {col.backend} schema applied; "
                        "the column will appear when you switch to a robot that does)"
                    )
                    with ui.ZStack(height=item_h):
                        row_btn = ui.InvisibleButton(height=item_h, tooltip=tooltip, style=_TOOLTIP_STYLE)
                        row_btn.set_clicked_fn(lambda cid=col.id: self._on_item_clicked(cid))
                        with ui.HStack(height=item_h, spacing=0):
                            ui.Spacer(width=18)
                            chk = ui.CheckBox(width=16, height=16)
                            chk.model.set_value(col.id in self._active)
                            ui.Spacer(width=10)
                            ui.Label(
                                col.label,
                                height=item_h,
                                style={"color": label_color, "font": _UI_FONT, "font_size": _FONT_SIZE_MENU},
                            )
                            ui.Spacer(width=10)
                ui.Spacer(height=spacer_h)

    def _build_backend_header(self) -> None:
        with ui.HStack(height=26, spacing=6):
            ui.Spacer(width=10)
            ui.Label(
                "BACKENDS",
                width=80,
                height=26,
                style={"color": _TEXT_DIM, "font": _UI_FONT, "font_size": _FONT_SIZE_SMALL},
            )
            for backend in self._BACKENDS_IN_HEADER:
                self._build_backend_pill(backend)
            ui.Spacer()

    def _build_backend_pill(self, backend: str) -> None:
        active = backend in self._active_backends
        bg = style.SELECTED_BG if active else _BG_INPUT
        text_color = _TEXT_PRIMARY if active else _TEXT_DIM
        with ui.ZStack(width=70, height=22):
            ui.Rectangle(style={"background_color": bg, "border_radius": 11})
            with ui.HStack():
                ui.Spacer()
                ui.Label(
                    backend, height=22, style={"color": text_color, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY}
                )
                ui.Spacer()
            btn = ui.InvisibleButton(tooltip=f"Show/hide all {backend} columns", style=_TOOLTIP_STYLE)
            btn.set_clicked_fn(lambda b=backend: self._on_backend_toggle(b))

    def _position_popup(self, anchor_widget: object) -> None:
        if not self._popup or anchor_widget is None:
            return
        anchor_w = getattr(anchor_widget, "computed_width", 0) or 0
        anchor_h = getattr(anchor_widget, "computed_height", 0) or 0
        anchor_x = getattr(anchor_widget, "screen_position_x", 0) or 0
        anchor_y = getattr(anchor_widget, "screen_position_y", 0) or 0
        pop_w = max(anchor_w, self._MIN_WIDTH)
        self._popup.width = pop_w
        self._popup.position_x = anchor_x + anchor_w - pop_w
        self._popup.position_y = anchor_y + anchor_h

    def _on_item_clicked(self, col_id: str) -> None:
        if col_id in self._active:
            self._active.discard(col_id)
        else:
            self._active.add(col_id)
        if self._popup_frame is not None:
            self._popup_frame.rebuild()
        if self._on_changed:
            self._on_changed()

    def _on_backend_toggle(self, backend: str) -> None:
        if backend in self._active_backends:
            self._active_backends.discard(backend)
        else:
            self._active_backends.add(backend)
        if self._popup_frame is not None:
            self._popup_frame.rebuild()
        if self._on_changed:
            self._on_changed()

    def destroy(self) -> None:
        self._popup_frame = None
        if self._popup:
            self._popup.destroy()
            self._popup = None


# --- Inspector view ---
class _JointItem(ui.AbstractItem):
    """A single joint row in the inspector :class:`omni.ui.TreeView` model.

    Carries the joint :class:`Usd.Prim` plus lazily-built per-column
    ``SimpleFloatModel`` instances keyed by ``(spec.id, axis)``. Editing a cell
    fires ``on_value_changed`` so the view can propagate the new value across
    every row in the current selection.
    """

    def __init__(
        self,
        joint_prim: Usd.Prim,
        on_value_changed: Callable[[_JointItem, str, str, float], None],
    ) -> None:
        super().__init__()
        self.joint_prim = joint_prim
        self.path = str(joint_prim.GetPath())
        self.name = joint_prim.GetName()
        self.name_model = ui.SimpleStringModel(self.name)
        self._on_value_changed = on_value_changed
        self._cell_attrs: dict[tuple[str, str], Usd.Attribute | None] = {}
        self._cell_models: dict[tuple[str, str], ui.SimpleFloatModel] = {}

    def value_model(self, rcol: _ResolvedColumn) -> ui.SimpleFloatModel:
        """Return (lazily creating) the SimpleFloatModel for ``rcol`` on this joint."""
        key = (rcol.spec.id, rcol.axis)
        model = self._cell_models.get(key)
        if model is not None:
            return model
        attr = rcol.spec.resolver(self.joint_prim, rcol.axis)
        self._cell_attrs[key] = attr
        value = rcol.spec.get_value(attr) if attr else rcol.spec.default
        model = ui.SimpleFloatModel(value)
        self._cell_models[key] = model
        model.add_value_changed_fn(
            lambda m, sid=rcol.spec.id, ax=rcol.axis: self._on_value_changed(self, sid, ax, m.get_value_as_float())
        )
        return model

    def attr_for(self, spec_id: str, axis: str) -> Usd.Attribute | None:
        """Return the cached USD attribute for the column on this joint."""
        key = (spec_id, axis)
        if key not in self._cell_attrs:
            spec = _COLUMN_BY_ID.get(spec_id)
            self._cell_attrs[key] = spec.resolver(self.joint_prim, axis) if spec else None
        return self._cell_attrs[key]

    def has_api(self, rcol: _ResolvedColumn) -> bool:
        """Cheap availability check used to disable cells when the API is unapplied."""
        return rcol.spec.api_test(self.joint_prim, rcol.axis)

    def push_remote_value(self, spec_id: str, axis: str, value: float) -> None:
        """Update the cell model to reflect an external write without re-firing fan-out."""
        model = self._cell_models.get((spec_id, axis))
        if model is None:
            return
        if model.get_value_as_float() == value:
            return
        model.set_value(value)


class _JointTableModel(ui.AbstractItemModel):
    """TreeView model for the joint inspector table.

    Column 0 is always the joint name; columns ``1..len(active_columns)`` map to
    a list of :class:`_ResolvedColumn` instances supplied through
    :meth:`set_data`. The model owns the joint items and surfaces a callback
    when any cell value changes.
    """

    def __init__(self, on_value_changed: Callable[[_JointItem, str, str, float], None]) -> None:
        super().__init__()
        self._on_value_changed = on_value_changed
        self._items: list[_JointItem] = []
        self._active_columns: list[_ResolvedColumn] = []

    def set_data(self, joints: list[Usd.Prim], active_columns: list[_ResolvedColumn]) -> None:
        """Replace the row set and the visible-column list, then notify the view."""
        self._items = [_JointItem(j, self._on_value_changed) for j in joints]
        self._active_columns = list(active_columns)
        self._item_changed(None)

    def items(self) -> list[_JointItem]:
        return list(self._items)

    def active_columns(self) -> list[_ResolvedColumn]:
        return list(self._active_columns)

    def get_item_children(self, item: object | None = None) -> list[_JointItem]:
        return self._items if item is None else []

    def get_item_value_model_count(self, item: object | None) -> int:
        return 1 + len(self._active_columns)

    def get_item_value_model(self, item: _JointItem | None, column_id: int) -> ui.AbstractValueModel | None:
        """Return the value model for the given item and column index."""
        if item is None:
            return None
        if column_id == 0:
            return item.name_model
        col_idx = column_id - 1
        if 0 <= col_idx < len(self._active_columns):
            return item.value_model(self._active_columns[col_idx])
        return None


class _JointTableDelegate(ui.AbstractItemDelegate):
    """Delegate that paints the joint table cells for :class:`_JointTableModel`."""

    _DRAG_STYLE = {
        "FloatDrag": {
            "color": style.SLIDER_TEXT,
            "background_color": _BG_INPUT,
            "font": _UI_FONT,
            "font_size": _FONT_SIZE_BODY,
            "border_radius": 2,
        },
        "FloatDrag:disabled": {
            "color": _TEXT_DIM,
            "background_color": _BG_INPUT,
        },
    }

    def __init__(self, model: _JointTableModel) -> None:
        super().__init__()
        self._model = model

    def build_branch(
        self, model: object, item: object = None, column_id: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        if column_id == 0:
            ui.Spacer(width=4)

    def build_header(self, column_id: int = 0) -> None:
        with ui.ZStack(height=_INSPECTOR_HEADER_HEIGHT):
            ui.Rectangle(style={"background_color": _BG_HEADER})
            with ui.HStack():
                ui.Spacer(width=8)
                if column_id == 0:
                    ui.Label(
                        "Joint",
                        height=_INSPECTOR_HEADER_HEIGHT,
                        style={"color": _TEXT_DIM, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    )
                else:
                    columns = self._model.active_columns()
                    idx = column_id - 1
                    if 0 <= idx < len(columns):
                        rcol = columns[idx]
                        ui.Label(
                            rcol.label,
                            height=_INSPECTOR_HEADER_HEIGHT,
                            elided_text=True,
                            tooltip=f"{rcol.spec.group} / {rcol.label}",
                            style={
                                "color": _TEXT_DIM,
                                "font": _UI_FONT,
                                "font_size": _FONT_SIZE_BODY,
                                "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                            },
                        )
                ui.Spacer(width=8)

    def build_widget(
        self, model: object, item: _JointItem | None = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        if item is None:
            return
        if index == 0:
            with ui.HStack(height=_INSPECTOR_ROW_HEIGHT):
                ui.Spacer(width=4)
                ui.Label(
                    item.name,
                    height=_INSPECTOR_ROW_HEIGHT,
                    elided_text=True,
                    tooltip=item.path,
                    style={
                        "color": style.TEXT_JOINT_NAME,
                        "font": _UI_FONT,
                        "font_size": _FONT_SIZE_BODY,
                        "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                    },
                )
            return

        columns = self._model.active_columns()
        col_idx = index - 1
        if not (0 <= col_idx < len(columns)):
            return
        rcol = columns[col_idx]
        with ui.HStack(height=_INSPECTOR_ROW_HEIGHT):
            ui.Spacer(width=2)
            if item.has_api(rcol):
                ui.FloatDrag(item.value_model(rcol), step=rcol.spec.step, style=self._DRAG_STYLE)
            else:
                # No backing API on this joint -> paint an empty cell. Better
                # than a disabled ``0.0`` input that falsely implies the value
                # is meaningful.
                ui.Spacer()
            ui.Spacer(width=2)


class JointInspectorView:
    """Renders the joint inspector body (toolbar + :class:`omni.ui.TreeView`) for one robot.

    Mount inside any container with :meth:`build`; swap robots with
    :meth:`set_robot`. The view tracks its own column visibility, joint name
    filter, and multi-row selection (delegated to the underlying TreeView).
    """

    def __init__(self) -> None:
        self._robot_prim: Usd.Prim | None = None
        self._joint_prims: list[Usd.Prim] = []
        self._visible_columns: set[str] = set(_DEFAULT_VISIBLE_COLUMN_IDS)
        self._visible_backends: set[str] = {BACKEND_PHYSX, BACKEND_MUJOCO}
        self._filter_text: str = ""
        self._info_frame: ui.Frame | None = None
        self._table_frame: ui.Frame | None = None
        self._columns_menu: _ColumnsMenu | None = None
        self._columns_button: ui.Button | None = None
        self._filter_field: ui.StringField | None = None
        self._filter_placeholder: ui.HStack | None = None
        self._filter_clear_button: ui.HStack | None = None
        self._filter_focused: bool = False
        # Subscription handles kept alive so the `subscribe_begin_edit_fn` /
        # `subscribe_end_edit_fn` callbacks on the filter field's model are
        # not dropped by the omni.ui subscription GC.
        self._filter_begin_sub = None
        self._filter_end_sub = None
        self._table_model = _JointTableModel(self._on_cell_changed)
        self._table_delegate = _JointTableDelegate(self._table_model)
        self._tree_view: ui.TreeView | None = None

    # -- public API -------------------------------------------------

    def set_robot(self, robot_prim: Usd.Prim | None) -> None:
        """Bind the view to ``robot_prim`` and rebuild the table.

        ``self._visible_columns`` reflects the user's intent across all robots
        and is left untouched here. Columns whose backing API is not applied on
        any joint of the current robot are filtered out at render time only
        (see :meth:`_effective_visible_ids`), so switching back to a robot that
        does have those APIs restores the previously checked columns.
        """
        self._robot_prim = robot_prim if (robot_prim and robot_prim.IsValid()) else None
        self._joint_prims = _joint_prims_for_robot(self._robot_prim) if self._robot_prim else []
        self._rebuild_info()
        self._rebuild_table()

    def _available_column_ids(self) -> set[str]:
        """Set of column ids whose API is applied on at least one joint."""
        if not self._joint_prims:
            return set()
        return {c.id for c in _COLUMNS if _column_available_for(c, self._joint_prims)}

    def build(self) -> None:
        """Build the view UI inside the current ``ui`` container."""
        with ui.VStack(spacing=0):
            with ui.ZStack(height=0):
                ui.Rectangle(style={"background_color": _BG_PANEL})
                self._info_frame = ui.Frame(height=0, build_fn=self._build_info)
            ui.Spacer(height=6)
            self._build_toolbar()
            ui.Spacer(height=6)
            self._table_frame = ui.Frame(height=ui.Fraction(1), build_fn=self._build_joint_table)

    def destroy(self) -> None:
        """Release UI resources owned by this widget."""
        if self._columns_menu:
            self._columns_menu.destroy()
            self._columns_menu = None
        self._filter_begin_sub = None
        self._filter_end_sub = None
        self._columns_button = None
        self._filter_field = None
        self._filter_placeholder = None
        self._filter_clear_button = None
        self._tree_view = None
        self._info_frame = None
        self._table_frame = None

    # -- toolbar / info -------------------------------------------

    def _build_info(self) -> None:
        with ui.VStack(height=0, spacing=0):
            ui.Spacer(height=8)
            with ui.HStack(height=14):
                ui.Spacer(width=16)
                ui.Label(
                    "Click rows to select | Ctrl/Shift for multi-select | Edits propagate across selection",
                    style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                )
                ui.Label(
                    f"{len(self._joint_prims)} joints",
                    style={
                        "color": style.TEXT_HINT,
                        "font": _UI_FONT,
                        "font_size": _FONT_SIZE_BODY,
                        "alignment": ui.Alignment.RIGHT_CENTER,
                    },
                )
                ui.Spacer(width=16)
            ui.Spacer(height=8)

    _TOOLBAR_HEIGHT = 24
    _CLEAR_BUTTON_WIDTH = 24
    _COLUMNS_BUTTON_WIDTH = 28
    _SEARCH_ICON_GUTTER = 26  # search icon column on the left of the field

    def _build_toolbar(self) -> None:
        with ui.HStack(height=ui.Pixel(self._TOOLBAR_HEIGHT), spacing=6):
            ui.Spacer(width=4)
            # Filter "input": a single styled background rectangle behind a
            # row of [magnifier icon | StringField | clear button]. Each child
            # owns its own hit-rect so the field never steals the clear click.
            with ui.ZStack(width=ui.Fraction(1), height=ui.Pixel(self._TOOLBAR_HEIGHT)):
                ui.Rectangle(style={"background_color": _BG_INPUT, "border_radius": 2})
                with ui.HStack(spacing=0):
                    # Search-icon column.
                    with ui.ZStack(width=ui.Pixel(self._SEARCH_ICON_GUTTER)):
                        with ui.VStack():
                            ui.Spacer()
                            with ui.HStack():
                                ui.Spacer()
                                ui.Image(_ICON_SEARCH, width=14, height=14, style={"color": _TEXT_DIM})
                                ui.Spacer()
                            ui.Spacer()
                    # Editable field. The placeholder Label (`_filter_placeholder`)
                    # is rendered on top of the field via a ZStack only inside
                    # this slot so it appears centred over the text, not the icon.
                    with ui.ZStack():
                        self._filter_field = ui.StringField(
                            height=ui.Pixel(self._TOOLBAR_HEIGHT),
                            style={
                                "Field": {
                                    "background_color": 0x00000000,
                                    "color": _TEXT_PRIMARY,
                                    "padding": 4,
                                    "padding_height": 0,
                                    "padding_width": 4,
                                    "border_radius": 2,
                                    "font": _UI_FONT,
                                    "font_size": _FONT_SIZE_BODY,
                                }
                            },
                        )
                        self._filter_field.model.set_value(self._filter_text)
                        self._filter_field.model.add_value_changed_fn(self._on_filter_changed)
                        # Detect focus so the placeholder hides as soon as the
                        # caret enters the field. `subscribe_begin_edit_fn` and
                        # `subscribe_end_edit_fn` are the only focus-transition
                        # hooks exposed on `ui.AbstractValueModel`; their return
                        # values are subscription handles that must be retained
                        # or the callback is disconnected on GC.
                        self._filter_begin_sub = self._filter_field.model.subscribe_begin_edit_fn(
                            lambda _m: self._on_filter_focus_changed(True)
                        )
                        self._filter_end_sub = self._filter_field.model.subscribe_end_edit_fn(
                            lambda _m: self._on_filter_focus_changed(False)
                        )
                        # Placeholder text is overlaid only across the text
                        # column (no icon overlap). Hidden when text is non-
                        # empty or the field has focus.
                        self._filter_placeholder = ui.HStack(height=ui.Pixel(self._TOOLBAR_HEIGHT))
                        with self._filter_placeholder:
                            ui.Spacer(width=4)
                            with ui.VStack():
                                ui.Spacer()
                                ui.Label(
                                    "Joint Name",
                                    height=0,
                                    style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                                )
                                ui.Spacer()
                            ui.Spacer()
                    # Clear button as a real, focus-grabbing widget so it
                    # outranks the field's hit-test. Hidden when text is empty.
                    self._filter_clear_button = ui.HStack(
                        width=ui.Pixel(self._CLEAR_BUTTON_WIDTH),
                        height=ui.Pixel(self._TOOLBAR_HEIGHT),
                        visible=False,
                    )
                    with self._filter_clear_button:
                        ui.Button(
                            "",
                            width=ui.Pixel(self._CLEAR_BUTTON_WIDTH),
                            height=ui.Pixel(self._TOOLBAR_HEIGHT),
                            clicked_fn=self._on_clear_filter,
                            tooltip="Clear filter",
                            style={
                                "Button": {
                                    "background_color": 0x00000000,
                                    "border_radius": 2,
                                    "padding": 2,
                                },
                                "Button:hovered": {"background_color": style.BG_HEADER_HOVER},
                                "Button:pressed": {"background_color": style.BG_HEADER_HOVER},
                                "Button.Image": {
                                    "image_url": _ICON_CLEAR,
                                    "color": _TEXT_DIM,
                                    "alignment": ui.Alignment.CENTER,
                                },
                                "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                            },
                        )
            # Hamburger column-picker. Transparent background so it blends with
            # the toolbar; default Kit hover/pressed styling kicks in only on
            # interaction.
            self._columns_button = ui.Button(
                "",
                width=ui.Pixel(self._COLUMNS_BUTTON_WIDTH),
                height=ui.Pixel(self._TOOLBAR_HEIGHT),
                clicked_fn=self._open_columns_menu,
                tooltip="Choose displayed columns",
                style={
                    "Button": {
                        "background_color": 0x00000000,
                        "border_radius": 2,
                        "padding": 2,
                    },
                    "Button:hovered": {"background_color": style.BG_HEADER_HOVER},
                    "Button:pressed": {"background_color": style.BG_HEADER_HOVER},
                    "Button.Image": {
                        "image_url": _ICON_MENU,
                        "color": _TEXT_PRIMARY,
                        "alignment": ui.Alignment.CENTER,
                    },
                    "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                },
            )
            ui.Spacer(width=4)
        self._refresh_filter_overlays()

        if self._columns_menu is None:
            self._columns_menu = _ColumnsMenu(
                _COLUMNS,
                self._visible_columns,
                self._visible_backends,
                on_changed_fn=self._rebuild_table,
                availability_fn=self._available_column_ids,
            )

    def _refresh_filter_overlays(self) -> None:
        has_text = bool(self._filter_text)
        if self._filter_placeholder is not None:
            self._filter_placeholder.visible = not (has_text or self._filter_focused)
        if self._filter_clear_button is not None:
            self._filter_clear_button.visible = has_text

    def _on_filter_focus_changed(self, focused: bool) -> None:
        self._filter_focused = bool(focused)
        self._refresh_filter_overlays()

    def _open_columns_menu(self) -> None:
        if self._columns_menu is None or self._columns_button is None:
            return
        self._columns_menu.toggle(self._columns_button)

    def _on_filter_changed(self, model: ui.AbstractValueModel) -> None:
        self._filter_text = model.get_value_as_string()
        self._refresh_filter_overlays()
        self._rebuild_table()

    def _on_clear_filter(self) -> None:
        self._filter_text = ""
        if self._filter_field is not None:
            self._filter_field.model.set_value("")
        self._refresh_filter_overlays()
        self._rebuild_table()

    # -- table rebuilds -------------------------------------------

    def _rebuild_info(self) -> None:
        if self._info_frame:
            self._info_frame.rebuild()

    def _rebuild_table(self) -> None:
        # Recreate the TreeView so column count, headers, and widths refresh
        # cleanly when the visible column set changes.
        self._tree_view = None
        if self._table_frame:
            self._table_frame.rebuild()

    def _filtered_joints(self) -> list[Usd.Prim]:
        """Filter the joint list against the user-typed query.

        - Empty query: return all joints.
        - Query containing ``*`` or ``?``: treat as a glob. ``fnmatch`` is
          anchored to the full string, so to match a search-bar mental model
          (``hand*`` finds anything that *contains* ``hand``) the pattern is
          also matched in a substring-fenced form (``*pattern*``) when the
          user did not bracket it themselves. Both the joint short name and
          the full prim path are tested.
        - Otherwise: case-insensitive substring match on either.
        """
        text = self._filter_text.strip()
        if not text:
            return list(self._joint_prims)

        is_glob = "*" in text or "?" in text
        text_lower = text.lower()
        if is_glob:
            patterns = {text_lower}
            fenced = text_lower
            if not fenced.startswith("*"):
                fenced = "*" + fenced
            if not fenced.endswith("*"):
                fenced = fenced + "*"
            patterns.add(fenced)

        out: list[Usd.Prim] = []
        for joint in self._joint_prims:
            name = joint.GetName().lower()
            path = str(joint.GetPath()).lower()
            if is_glob:
                if any(fnmatch.fnmatchcase(name, p) or fnmatch.fnmatchcase(path, p) for p in patterns):
                    out.append(joint)
            elif text_lower in name or text_lower in path:
                out.append(joint)
        return out

    # -- table builder --------------------------------------------

    def _effective_visible_ids(self) -> set[str]:
        """Visible columns filtered through backend toggles AND robot availability.

        ``USD`` backend (joint limits + drives) is always allowed; PhysX/MuJoCo
        respect the popup pill toggles. The user's selection in
        ``self._visible_columns`` is *intent* and is preserved across robot
        switches; this method just hides columns whose backing API is not
        applied on any joint of the current robot, so flipping back to a
        compatible robot restores them.
        """
        available = self._available_column_ids()
        return {
            c.id
            for c in _COLUMNS
            if c.id in self._visible_columns
            and c.id in available
            and (c.backend == BACKEND_USD or c.backend in self._visible_backends)
        }

    def _build_joint_table(self) -> None:
        if not self._robot_prim or not self._joint_prims:
            with ui.VStack(height=0):
                ui.Spacer(height=12)
                with ui.HStack(height=20):
                    ui.Spacer(width=16)
                    ui.Label(
                        (
                            "Select a robot to display its joints."
                            if not self._robot_prim
                            else "This robot has no joints."
                        ),
                        style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    )
                ui.Spacer(height=12)
            return

        joints = self._filtered_joints()
        active = _resolve_columns(self._effective_visible_ids(), joints)

        if not active:
            # Disambiguate between "user hasn't picked any columns" and
            # "user-picked columns exist but none are applicable to this robot
            # (their schema isn't applied on any joint, or their backend pill
            # is off)". The user's selection in ``self._visible_columns`` is
            # preserved across robots regardless.
            available = self._available_column_ids()
            if not self._visible_columns:
                msg = "No columns selected. Use the Columns menu to pick fields to display."
            elif not (self._visible_columns & available):
                msg = (
                    "None of the selected columns apply to this robot. "
                    "Open the Columns menu to add ones backed by an applied schema."
                )
            else:
                msg = (
                    "Selected columns are hidden by the active backend toggles. "
                    "Re-enable PhysX or MuJoCo in the Columns menu."
                )
            with ui.VStack(height=0):
                ui.Spacer(height=12)
                with ui.HStack(height=20):
                    ui.Spacer(width=16)
                    ui.Label(msg, style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY})
                ui.Spacer(height=12)
            return

        if not joints:
            with ui.VStack(height=0):
                ui.Spacer(height=12)
                with ui.HStack(height=20):
                    ui.Spacer(width=16)
                    ui.Label(
                        "No joints match the current filter.",
                        style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    )
                ui.Spacer(height=12)
            return

        self._table_model.set_data(joints, active)
        column_widths = [ui.Fraction(2), *(ui.Fraction(1) for _ in active)]
        min_widths = [120, *(80 for _ in active)]
        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            style_type_name_override="TreeView",
        ):
            self._tree_view = ui.TreeView(
                self._table_model,
                delegate=self._table_delegate,
                root_visible=False,
                header_visible=True,
                columns_resizable=True,
                column_widths=column_widths,
                min_column_widths=min_widths,
                alignment=ui.Alignment.LEFT_TOP,
                style={
                    "TreeView": {"background_color": _BG_PANEL},
                    "TreeView.Item": {"color": style.TEXT_JOINT_NAME, "margin": 2},
                    "TreeView.Item:selected": {"background_color": style.SELECTED_BG},
                    "TreeView:selected": {"background_color": style.SELECTED_BG},
                },
            )

    # -- edit propagation -----------------------------------------

    def _selected_paths(self) -> set[str]:
        if not self._tree_view:
            return set()
        return {item.path for item in self._tree_view.selection if isinstance(item, _JointItem)}

    def _on_cell_changed(self, source: _JointItem, spec_id: str, axis: str, value: float) -> None:
        """Write ``value`` to the column's attribute on this joint, then propagate to selection.

        Uses the column's :class:`_ColumnSpec` setter so array-typed MuJoCo
        attributes (``solreflimit``, ``solimplimit``) update only their
        dominant element. The originating row is always written. If it is part
        of the current selection (more than one item selected), the same value
        is mirrored to every other selected row whose attribute exists.
        """
        spec = _COLUMN_BY_ID.get(spec_id)
        if spec is None:
            return
        primary_attr = source.attr_for(spec_id, axis)
        spec.set_value(primary_attr, value)

        selected = self._selected_paths()
        if source.path not in selected or len(selected) <= 1:
            return

        for item in self._table_model.items():
            if item.path == source.path or item.path not in selected:
                continue
            other_attr = item.attr_for(spec_id, axis)
            if other_attr is None:
                continue
            spec.set_value(other_attr, value)
            item.push_remote_value(spec_id, axis, value)


# --- Window ---
class JointInspectorWindow:
    """Standalone window hosting a robot picker and a :class:`JointInspectorView`.

    Multiple instances may exist concurrently; each window keeps its own robot
    selection. Pass ``on_new_window`` to wire the in-window
    "+ New Inspector" button to the manager.
    """

    def __init__(
        self,
        title: str = WINDOW_TITLE,
        on_new_window: Callable[[], None] | None = None,
        on_visibility_changed: Callable[[bool], None] | None = None,
        dock_target_title: str | None = None,
    ) -> None:
        self._title = title
        self._on_new_window = on_new_window
        self._on_visibility_changed = on_visibility_changed
        self._dock_target_title = dock_target_title
        self._view = JointInspectorView()
        self._robot_paths: list[str] = []
        self._selected_path: str | None = None
        self._search_text: str = ""
        self._dropdown_frame: ui.Frame | None = None
        self._popup: ui.Window | None = None
        self._search_field: ui.StringField | None = None
        self._results_frame: ui.Frame | None = None
        self._stage_event_sub = None
        self._dock_task: asyncio.Task | None = None

        self._window = ui.Window(
            title,
            width=820,
            height=560,
            visible=True,
        )
        self._window.set_visibility_changed_fn(self._on_window_visibility)

        self._build_ui()
        self._refresh_robot_list(select_first=True)
        self._subscribe_stage_events()
        self._schedule_dock()

    @property
    def title(self) -> str:
        """Window title as passed at construction time."""
        return self._title

    @property
    def visible(self) -> bool:
        """Whether the inspector window is currently visible."""
        return bool(self._window and self._window.visible)

    @visible.setter
    def visible(self, value: bool) -> None:
        if self._window:
            self._window.visible = bool(value)

    def focus(self) -> None:
        """Show the inspector window and bring it to the front."""
        if not self._window:
            return
        self._window.visible = True
        self._window.focus()

    def destroy(self) -> None:
        """Tear down the window and release subscriptions."""
        self._stage_event_sub = None
        if self._dock_task is not None and not self._dock_task.done():
            self._dock_task.cancel()
        self._dock_task = None
        if self._window:
            # Detach the visibility callback BEFORE hiding so the hide path in
            # `_on_window_visibility` does not fire with `self._popup` already
            # nulled and the external visibility listener does not see a spurious
            # hidden event during teardown.
            self._window.set_visibility_changed_fn(None)
        if self._popup:
            self._popup.destroy()
            self._popup = None
        if self._view:
            self._view.destroy()
        if self._window:
            self._window.visible = False
            self._window.destroy()
            self._window = None

    def _on_window_visibility(self, visible: bool) -> None:
        if not visible and self._popup:
            self._popup.visible = False
        if self._on_visibility_changed:
            self._on_visibility_changed(visible)

    def _subscribe_stage_events(self) -> None:
        usd_context = omni.usd.get_context()
        if not usd_context:
            return
        self._stage_event_sub = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name=f"isaacsim.gui.property.JointInspectorWindow.{id(self)}"
        )

    def _on_stage_event(self, event: object) -> None:
        et = getattr(event, "type", None)
        if et in (
            int(omni.usd.StageEventType.OPENED),
            int(omni.usd.StageEventType.CLOSED),
            int(omni.usd.StageEventType.ASSETS_LOADED),
        ):
            self._refresh_robot_list(select_first=self._selected_path is None)

    def _schedule_dock(self) -> None:
        """Defer docking by one frame so the window is registered before Workspace lookup.

        - When ``self._dock_target_title`` is None: dock the window into the
          left half of the viewport at 33% width, mirroring the convention
          used by Gain Tuner / Robot Poser / cuMotion examples.
        - When ``self._dock_target_title`` is set: tab the window onto that
          existing dock container (typically the primary inspector window) via
          ``DockPosition.SAME``.
        """

        async def _do_dock() -> None:
            app = omni.kit.app.get_app()
            await app.next_update_async()
            window = ui.Workspace.get_window(self._title)
            if not window:
                return
            if self._dock_target_title:
                target = ui.Workspace.get_window(self._dock_target_title)
                if target:
                    window.dock_in(target, ui.DockPosition.SAME)
            else:
                viewport = ui.Workspace.get_window("Viewport")
                if viewport:
                    window.dock_in(viewport, ui.DockPosition.LEFT, 0.33)
            await app.next_update_async()

        self._dock_task = asyncio.ensure_future(_do_dock())

    def _build_ui(self) -> None:
        with self._window.frame:
            with ui.VStack(spacing=0):
                ui.Spacer(height=8)
                with ui.HStack(height=24, spacing=8):
                    ui.Spacer(width=12)
                    ui.Label(
                        "Robot",
                        width=42,
                        style={"color": _TEXT_PRIMARY, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    )
                    self._dropdown_frame = ui.Frame(height=24, build_fn=self._build_dropdown)
                    ui.Button(
                        "",
                        width=24,
                        height=22,
                        clicked_fn=lambda: self._refresh_robot_list(select_first=False),
                        tooltip="Re-scan the stage for prims with IsaacRobotAPI",
                        image_url=_ICON_REFRESH,
                        style={
                            "Button": {
                                "background_color": 0x00000000,
                                "border_radius": 2,
                                "padding": 2,
                            },
                            "Button:hovered": {"background_color": style.BG_HEADER_HOVER},
                            "Button:pressed": {"background_color": style.BG_HEADER_HOVER},
                            "Button.Image": {
                                "image_url": _ICON_REFRESH,
                                "color": _TEXT_PRIMARY,
                                "alignment": ui.Alignment.CENTER,
                            },
                            "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                        },
                    )
                    ui.Button(
                        "New Inspector",
                        width=120,
                        height=22,
                        clicked_fn=self._on_new_inspector_clicked,
                        tooltip="Open another Joint Inspector window",
                        image_url=_ICON_PLUS,
                        image_width=12,
                        image_height=12,
                        style={
                            "Button": {
                                "background_color": style.BUTTON_BG,
                                "border_radius": 2,
                                "padding": 4,
                                "stack_direction": ui.Direction.LEFT_TO_RIGHT,
                            },
                            "Button.Label": {
                                "color": _TEXT_PRIMARY,
                                "font": _UI_FONT,
                                "font_size": _FONT_SIZE_BODY,
                            },
                            "Button.Image": {
                                "image_url": _ICON_PLUS,
                                "color": _TEXT_PRIMARY,
                            },
                            "Tooltip": _TOOLTIP_STYLE["Tooltip"],
                        },
                    )
                    ui.Spacer(width=12)
                ui.Spacer(height=8)
                with ui.ScrollingFrame(
                    height=ui.Fraction(1),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                ):
                    self._view.build()

    def _on_new_inspector_clicked(self) -> None:
        if self._on_new_window:
            self._on_new_window()

    def _build_dropdown(self) -> None:
        label_text = self._selected_path if self._selected_path else "<no robot selected>"
        with ui.ZStack(height=24):
            ui.Rectangle(style={"background_color": _BG_INPUT, "border_radius": 2})
            with ui.HStack(spacing=0):
                ui.Spacer(width=8)
                ui.Label(
                    label_text,
                    style={"color": _TEXT_PRIMARY, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    elided_text=True,
                )
                with ui.VStack(width=18):
                    ui.Spacer(height=ui.Fraction(1))
                    ui.Triangle(
                        width=10,
                        height=8,
                        alignment=ui.Alignment.CENTER_BOTTOM,
                        style={"background_color": _TEXT_PRIMARY, "border_width": 0},
                    )
                    ui.Spacer(height=ui.Fraction(1))
                ui.Spacer(width=8)
            btn = ui.InvisibleButton()
            btn.set_clicked_fn(self._toggle_picker)

    def _toggle_picker(self) -> None:
        if self._popup and self._popup.visible:
            self._popup.visible = False
            return
        if self._popup is None:
            self._build_picker_popup()
        self._position_picker_popup()
        if self._search_field:
            self._search_field.model.set_value(self._search_text)
        if self._results_frame:
            self._results_frame.rebuild()
        self._popup.visible = True

    def _build_picker_popup(self) -> None:
        max_visible_rows = 10
        list_h = max_visible_rows * 26 + 8
        pop_h = 8 + 28 + 8 + list_h + 8
        anchor_w = max(self._dropdown_frame.computed_width if self._dropdown_frame else 0, 320)
        self._popup = ui.Window(
            "##joint_inspector_robot_picker",
            width=anchor_w,
            height=pop_h,
            padding_x=0,
            padding_y=0,
            flags=(
                ui.WINDOW_FLAGS_POPUP
                | ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_MOVE
            ),
        )
        self._popup.frame.set_style({"Window": {"background_color": _BG_INPUT, "border_radius": 2}})
        with self._popup.frame:
            with ui.VStack(spacing=0):
                ui.Spacer(height=8)
                with ui.HStack(height=22):
                    ui.Spacer(width=8)
                    self._search_field = ui.StringField(height=20)
                    self._search_field.model.set_value(self._search_text)
                    self._search_field.model.add_value_changed_fn(self._on_search_changed)
                    ui.Spacer(width=8)
                ui.Spacer(height=8)
                with ui.ScrollingFrame(
                    height=list_h,
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                ):
                    self._results_frame = ui.Frame(build_fn=self._build_results)
                ui.Spacer(height=8)

    def _position_picker_popup(self) -> None:
        if not self._popup or not self._dropdown_frame:
            return
        anchor_w = max(self._dropdown_frame.computed_width, 320)
        self._popup.width = anchor_w
        self._popup.position_x = self._dropdown_frame.screen_position_x
        self._popup.position_y = self._dropdown_frame.screen_position_y + self._dropdown_frame.computed_height

    def _build_results(self) -> None:
        text = self._search_text.lower()
        matches = [p for p in self._robot_paths if not text or text in p.lower()]
        with ui.VStack(spacing=0):
            if not matches:
                ui.Spacer(height=8)
                with ui.HStack(height=20):
                    ui.Spacer(width=10)
                    ui.Label(
                        "No robots match." if self._robot_paths else "No prims with IsaacRobotAPI found.",
                        style={"color": style.TEXT_HINT, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                    )
                return
            for path in matches:
                with ui.ZStack(height=24):
                    is_selected = path == self._selected_path
                    ui.Rectangle(
                        style={
                            "background_color": style.SELECTED_BG if is_selected else _BG_INPUT,
                            "border_radius": 2,
                        }
                    )
                    with ui.HStack(spacing=0):
                        ui.Spacer(width=10)
                        ui.Label(
                            path,
                            style={"color": _TEXT_PRIMARY, "font": _UI_FONT, "font_size": _FONT_SIZE_BODY},
                            elided_text=True,
                        )
                        ui.Spacer(width=10)
                    btn = ui.InvisibleButton()
                    btn.set_clicked_fn(lambda p=path: self._on_pick(p))

    def _on_search_changed(self, model: ui.AbstractValueModel) -> None:
        self._search_text = model.get_value_as_string()
        if self._results_frame:
            self._results_frame.rebuild()

    def _on_pick(self, path: str) -> None:
        self._selected_path = path
        if self._popup:
            self._popup.visible = False
        if self._dropdown_frame:
            self._dropdown_frame.rebuild()
        self._apply_selection()

    def _refresh_robot_list(self, select_first: bool) -> None:
        stage = omni.usd.get_context().get_stage() if omni.usd.get_context() else None
        prims = _list_robot_prims(stage)
        self._robot_paths = [str(p.GetPath()) for p in prims]
        if self._selected_path not in self._robot_paths:
            self._selected_path = self._robot_paths[0] if (self._robot_paths and select_first) else None
        if self._dropdown_frame:
            self._dropdown_frame.rebuild()
        if self._results_frame:
            self._results_frame.rebuild()
        self._apply_selection()

    def _apply_selection(self) -> None:
        if not self._selected_path:
            self._view.set_robot(None)
            return
        stage = omni.usd.get_context().get_stage() if omni.usd.get_context() else None
        prim = stage.GetPrimAtPath(self._selected_path) if stage else None
        self._view.set_robot(prim)


class JointInspectorWindowManager:
    """Owns a primary :class:`JointInspectorWindow` and any spawned siblings.

    The menu entry calls :meth:`open_or_focus_primary` to toggle/focus the first
    window. Clicking ``+ New Inspector`` inside any window calls
    :meth:`spawn_window` to create another instance.
    """

    def __init__(self, on_primary_visibility_changed: Callable[[bool], None] | None = None) -> None:
        self._on_primary_visibility_changed = on_primary_visibility_changed
        self._primary: JointInspectorWindow | None = None
        self._extra: list[JointInspectorWindow] = []

    def open_or_focus_primary(self) -> None:
        """Show the primary inspector window, creating it if needed, or hide it if already visible."""
        if self._primary is None:
            self._primary = JointInspectorWindow(
                title=WINDOW_TITLE,
                on_new_window=self.spawn_window,
                on_visibility_changed=self._on_primary_visibility_changed,
            )
            return
        if self._primary.visible:
            self._primary.visible = False
        else:
            self._primary.focus()

    def spawn_window(self) -> JointInspectorWindow:
        """Create and track an additional inspector window with a unique title.

        The title is picked as the smallest integer suffix greater than 1 that
        is not currently assigned to a live window, so closing and respawning
        does not accumulate stale numbering. The primary window always owns
        the un-suffixed title.
        """
        used_titles = {w.title for w in self._extra if w.visible}
        if self._primary is not None and self._primary.visible:
            used_titles.add(self._primary.title)
        suffix = 2
        while f"{WINDOW_TITLE} {suffix}" in used_titles:
            suffix += 1
        title = f"{WINDOW_TITLE} {suffix}"
        win = JointInspectorWindow(
            title=title,
            on_new_window=self.spawn_window,
            on_visibility_changed=self._make_extra_reaper(),
            dock_target_title=WINDOW_TITLE,
        )
        self._extra.append(win)
        return win

    def is_primary_visible(self) -> bool:
        """Return True when the primary inspector window exists and is visible."""
        return bool(self._primary and self._primary.visible)

    def shutdown(self) -> None:
        """Destroy all owned inspector windows."""
        for win in self._extra:
            win.destroy()
        self._extra.clear()
        if self._primary:
            self._primary.destroy()
            self._primary = None

    def _make_extra_reaper(self) -> Callable[[bool], None]:
        """Return a visibility callback that drops the sender from `self._extra` on hide.

        The callback holds no direct reference to the window; it walks
        `self._extra` when invoked and removes any entries whose windows
        have been hidden or destroyed. This prevents the manager from
        retaining closed inspector windows for the lifetime of the extension.
        """

        def _reap(_visible: bool) -> None:
            self._extra[:] = [w for w in self._extra if w.visible]

        return _reap
