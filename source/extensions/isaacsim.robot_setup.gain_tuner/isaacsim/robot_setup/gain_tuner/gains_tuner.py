# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module for tuning joint gains on articulated robots through test signal generation and inertia analysis."""

from __future__ import annotations

import asyncio
import math
from collections.abc import Callable, Generator
from dataclasses import dataclass
from enum import IntEnum

import carb
import numpy as np
import omni
import omni.timeline as timeline
import pxr
import usd.schema.isaac.robot_schema as robot_schema
import usd.schema.isaac.robot_schema.utils as rs_utils
from isaacsim.core.experimental.prims import Articulation
from omni.physics.tensors import DofType
from pxr import Gf, PhysxSchema, Sdf, Tf, Usd, UsdPhysics

from .base import RobotTest, TestResult
from .gain_save_targets import build_mjc_gain_map
from .gain_sources import (
    GainSource,
    active_gain_source,
    build_actuator_gain_map,
    has_physics_drive,
    mjc_params_to_drive_gains,
    newton_mujoco_solver_active,
)

_SELF_COLLISION_ATTR = "physxArticulation:enabledSelfCollisions"
"""Cook-time PhysX property the "disable self collisions" test option overrides."""

_NEWTON_SELF_COLLISION_ATTR = "newton:selfCollisionEnabled"
"""Newton's equivalent, read at model build time and leading the resolver chain."""

_NEWTON_ARTICULATION_ROOT_API = "NewtonArticulationRootAPI"
"""Codeless schema defining `newton:selfCollisionEnabled`, applied by name."""


def _apply_newton_articulation_root_api(prim: pxr.Usd.Prim) -> bool:
    """Apply `NewtonArticulationRootAPI` to a prim.

    The schema is codeless, so it has no ``pxr`` Python class and is applied by
    name. Its registration comes from the ``omni.usd.schema.newton`` extension.

    Args:
        prim: The articulation root to apply the schema to.

    Returns:
        True when the schema was applied.
    """
    return prim.ApplyAPI(_NEWTON_ARTICULATION_ROOT_API)


class GainsTestMode(IntEnum):
    """Enumeration of available test signal modes for gain tuning."""

    SINUSOIDAL = 0
    """Test mode that generates sinusoidal signals for gain tuning analysis."""
    STEP = 1
    """Test mode that generates step response signals for gain tuning analysis."""
    USER_PROVIDED = 2
    """Test mode that allows custom user-defined signals for gain tuning analysis."""
    SNAP_TO_LIMITS = 3
    """Test mode that commands joints to their lower and upper limits to verify reachability."""
    STRESS_TEST = 4
    """Stress test that bombards joints with extreme random commands to surface solver instabilities."""
    DISCRETIZATION = 5
    """dt physics sweep that measures how position-control accuracy degrades as the physics timestep coarsens."""


class JointMode(IntEnum):
    """Enumeration of joint control modes."""

    POSITION = 0
    """Position control mode where joints are commanded to specific position targets."""
    VELOCITY = 1
    """Velocity control mode where joints are commanded to specific velocity targets."""
    NONE = 2
    """No control mode active for the joint."""


MAX_VELOCITY_AGREEMENT_REL_TOL = 1e-3
"""Relative tolerance for comparing a USD velocity limit against the engine's.

The engine reports angular limits in radians per second and the joint schemas
store degrees per second, so any comparison round-trips through a conversion in
float32-backed data.  A tolerance of 0.1% keeps that noise from being reported as
a disagreement while staying far tighter than any limit a user would author on
purpose.
"""


UNLIMITED_VELOCITY_THRESHOLD = 1.0e6
"""Engine velocity limit at or above which no practical limit is being enforced.

Neither backend reports an unlimited DOF as infinite.  Newton substitutes its
``ModelBuilder`` default of ``1e6`` for both an authored ``inf`` and an unauthored
limit (``newton.utils.import_usd`` maps ``inf`` to None and then to
``default_joint_cfg.velocity_limit``), and PhysX reports its own ceiling.  In the
tensor API's own units ``1e6`` is over nine million revolutions per minute, so any
limit that large is a stand-in for "unclamped" rather than a limit a user
authored.
"""


def max_velocity_agrees(usd_value: float | None, engine_value: float | None) -> bool:
    """Return whether a USD velocity limit matches the one the engine enforces.

    Args:
        usd_value: The limit the active backend's chain resolves, in schema units
            (degrees per second for rotational DOFs).  ``math.inf`` for a limit
            that is unlimited -- whether authored as ``inf`` or left unauthored,
            since the engine default for both is no clamp.  None only when the
            USD side could not be resolved at all, which is the case when the
            running solver, and therefore the resolver chain, is unknown.
        engine_value: The limit from
            :meth:`GainTuner.get_dof_effective_max_velocity`, in the same units,
            ``math.inf`` when the engine enforces no limit, or None when there is
            no engine truth yet (before the timeline plays).

    Returns:
        True when the two agree within :data:`MAX_VELOCITY_AGREEMENT_REL_TOL`, and
        when one side is genuinely unknowable.  An unauthored USD limit against a
        finite enforced one is a disagreement, not an absence\\: the field then
        reads as unlimited while the engine clamps the joint, which is exactly the
        case worth reporting.

    Example:

    .. code-block:: python

        >>> import math
        >>> from isaacsim.robot_setup.gain_tuner import max_velocity_agrees

        >>> max_velocity_agrees(180.0, 180.00001)
        True
        >>> max_velocity_agrees(180.0, 90.0)
        False
        >>> max_velocity_agrees(math.inf, 107.0)
        False
    """
    if usd_value is None or engine_value is None:
        return True
    if math.isnan(usd_value) or math.isnan(engine_value):
        return True
    if math.isinf(usd_value) or math.isinf(engine_value):
        return math.isinf(usd_value) and math.isinf(engine_value)
    return math.isclose(usd_value, engine_value, rel_tol=MAX_VELOCITY_AGREEMENT_REL_TOL, abs_tol=1e-9)


def _joint_mode_from_gains(kp: float | None, kd: float | None) -> JointMode:
    """Classify a control mode from a joint's effective proportional/derivative gains.

    A non-zero proportional gain (stiffness / Kp) means position control; a zero
    proportional gain with a non-zero derivative gain (damping / Kd) means
    velocity control; otherwise the joint is not driven.

    Args:
        kp: Effective proportional gain (stiffness / Kp), or None.
        kd: Effective derivative gain (damping / Kd), or None.

    Returns:
        The classified :class:`JointMode`.
    """
    if kp:
        return JointMode.POSITION
    if kd:
        return JointMode.VELOCITY
    return JointMode.NONE


def resolve_active_source_gains(
    tested_dof_indices: list[int],
    dof_joint_map: dict[int, "pxr.Usd.Prim"],
    actuator_map: dict[str, object],
    mjc_map: dict[str, object],
    *,
    mujoco_solver_active: bool = False,
    dof_drive_axis: dict[int, object] | None = None,
) -> dict[int, tuple[float, float]]:
    """Resolve effective (Kp, Kd) for DOFs whose active source is an actuator / mjc.

    A Newton actuator (backend-independent) or an active MuJoCo actuator drives its
    joint through gains that the physics tensor's DriveAPI stiffness/damping does
    **not** reflect (the actuator runtime zeros the DriveAPI gains).  For each
    tested DOF whose resolved active source (per
    :func:`isaacsim.robot_setup.gain_tuner.gain_sources.active_gain_source`) is a
    Newton actuator or MuJoCo-native gains, this returns the effective ``(kp, kd)``
    read from that source so the DOF can be classified and commanded correctly.

    Args:
        tested_dof_indices: DOF indices under test.
        dof_joint_map: Map of DOF index -> joint prim.
        actuator_map: Map of joint path -> ``ActuatorGains`` (Newton actuators).
        mjc_map: Map of joint path -> ``MjcGainSource`` (MuJoCo-native gains).
        mujoco_solver_active: Whether the live Newton solver is the MuJoCo solver.
        dof_drive_axis: Optional map of DOF index -> D6 drive-axis token.

    Returns:
        Dict mapping DOF index -> ``(kp, kd)`` for actuator- / mjc-active DOFs only
        (DriveAPI-driven DOFs are omitted and fall back to the tensor gains).
    """
    overrides: dict[int, tuple[float, float]] = {}
    dof_drive_axis = dof_drive_axis or {}
    for dof_idx in tested_dof_indices:
        joint = dof_joint_map.get(dof_idx)
        if joint is None:
            continue
        joint_path = joint.GetPath().pathString
        actuator = actuator_map.get(joint_path)
        mjc = mjc_map.get(joint_path)
        has_pd = has_physics_drive(joint, dof_drive_axis.get(dof_idx))
        active = active_gain_source(
            has_pd, actuator is not None, mjc is not None, mujoco_solver_active=mujoco_solver_active
        )
        if active == GainSource.ACTUATOR and actuator is not None:
            overrides[dof_idx] = (actuator.kp or 0.0, actuator.kd or 0.0)
        elif active == GainSource.MUJOCO and mjc is not None:
            gain_prm = mjc.gain_prm_attr.Get() if (mjc.gain_prm_attr and mjc.gain_prm_attr.IsValid()) else None
            bias_prm = mjc.bias_prm_attr.Get() if (mjc.bias_prm_attr and mjc.bias_prm_attr.IsValid()) else None
            overrides[dof_idx] = mjc_params_to_drive_gains(gain_prm, bias_prm)
    return overrides


def classify_joint_modes(
    tested_dof_indices: list[int],
    drive_stiffness: list[float] | np.ndarray,
    drive_damping: list[float] | np.ndarray,
    active_source_gains: dict[int, tuple[float, float]] | None = None,
) -> dict[int, JointMode]:
    """Classify each tested DOF into a :class:`JointMode`.

    By default the mode is read from the DriveAPI stiffness/damping (from the
    physics tensor's ``get_dof_gains``).  ``active_source_gains`` overrides those
    gains for DOFs whose active source is a Newton actuator or an active MuJoCo
    actuator, so such joints are classified POSITION (or VELOCITY when only a
    damping-like gain is present) and therefore get commanded during tests instead
    of being dropped as un-driven.

    Args:
        tested_dof_indices: DOF indices under test.
        drive_stiffness: Per-DOF DriveAPI stiffness (indexed by DOF index).
        drive_damping: Per-DOF DriveAPI damping (indexed by DOF index).
        active_source_gains: Optional map of DOF index -> ``(kp, kd)`` overriding
            the DriveAPI gains for actuator- / mjc-active DOFs (see
            :func:`resolve_active_source_gains`).

    Returns:
        Dict mapping each tested DOF index -> its :class:`JointMode`.
    """
    active_source_gains = active_source_gains or {}
    modes: dict[int, JointMode] = {}
    for dof_idx in tested_dof_indices:
        if dof_idx in active_source_gains:
            kp, kd = active_source_gains[dof_idx]
        else:
            kp = drive_stiffness[dof_idx]
            kd = drive_damping[dof_idx]
        modes[dof_idx] = _joint_mode_from_gains(kp, kd)
    return modes


@dataclass
class JointListEntry:
    """Data structure representing a joint entry in the gain tuner.

    Args:
        joint: The USD prim representing the joint.
        display_name: Human-readable name for UI display.
        dof_index: Index of this degree of freedom in the articulation.
        drive_axis: Axis token for D6 joints, None for single-axis joints.
    """

    joint: pxr.Usd.Prim
    display_name: str
    dof_index: int
    drive_axis: str


_D6_AXIS_TOKENS = [
    pxr.UsdPhysics.Tokens.transX,
    pxr.UsdPhysics.Tokens.transY,
    pxr.UsdPhysics.Tokens.transZ,
    pxr.UsdPhysics.Tokens.rotX,
    pxr.UsdPhysics.Tokens.rotY,
    pxr.UsdPhysics.Tokens.rotZ,
]

_AXIS_TOKEN_TO_VECTOR = {
    "X": Gf.Vec3f(1, 0, 0),
    "Y": Gf.Vec3f(0, 1, 0),
    "Z": Gf.Vec3f(0, 0, 1),
}


def _extract_d6_axis_token(dof_name: str) -> str | None:
    """Extract the D6 axis token from a DOF name if present.

    Args:
        dof_name: Name of the degree of freedom.

    Returns:
        The matching axis token, or None if no match found.
    """
    lower_name = dof_name.lower()
    for token in _D6_AXIS_TOKENS:
        if token.lower() in lower_name:
            return token
    return None


def _assign_d6_axis_token(joint_identifier: str, dof_name: str, usage_map: dict[str, set]) -> str:
    """Assign an unused D6 axis token for a joint.

    Args:
        joint_identifier: Unique identifier for the joint.
        dof_name: Name of the degree of freedom.
        usage_map: Map tracking which axes have been used per joint.

    Returns:
        The assigned axis token, or None if all axes are used.
    """
    token = _extract_d6_axis_token(dof_name)
    used_axes = usage_map.setdefault(joint_identifier, set())
    if token and token not in used_axes:
        used_axes.add(token)
        return token
    for candidate in _D6_AXIS_TOKENS:
        if candidate not in used_axes:
            used_axes.add(candidate)
            return candidate
    return None


def _d6_axis_has_unlocked_limit(joint_prim: pxr.Usd.Prim, axis_token: str) -> bool:
    """Check if a D6 joint axis has unlocked (non-fixed) limits.

    Args:
        joint_prim: The USD prim representing the joint.
        axis_token: The axis token to check.

    Returns:
        True if the axis has a valid range (lower < upper), False otherwise.
    """
    limit_api = pxr.UsdPhysics.LimitAPI.Get(joint_prim, axis_token)
    if not limit_api:
        return False
    lower_attr = limit_api.GetLowAttr()
    upper_attr = limit_api.GetHighAttr()
    if not lower_attr or not upper_attr:
        return False
    lower = lower_attr.Get()
    upper = upper_attr.Get()
    if lower is None or upper is None:
        return False
    try:
        return float(lower) < float(upper)
    except (TypeError, ValueError):
        return False


def _format_d6_display_name(joint_prim: pxr.Usd.Prim, dof_name: str, axis_token: str) -> str:
    """Format a display name for a D6 joint axis.

    Args:
        joint_prim: The USD prim representing the joint.
        dof_name: Name of the degree of freedom.
        axis_token: The axis token for this DOF.

    Returns:
        Formatted display name in the form "joint_name:axis".
    """
    suffix = axis_token if dof_name == joint_prim.GetName() else dof_name
    return f"{joint_prim.GetName()}:{suffix}"


def get_original_spec_for_drive_api(
    stage: pxr.Usd.Stage, joint_drive_path: str, drive_type: str
) -> pxr.Sdf.PropertySpec:
    """Get the original property spec where a drive attribute was authored.

    Args:
        stage: The USD stage.
        joint_drive_path: Path to the joint with the drive API.
        drive_type: The drive type (axis token).

    Returns:
        The original property spec, or None if not found.
    """
    drive_prim = stage.GetPrimAtPath(joint_drive_path)
    attr = pxr.UsdPhysics.DriveAPI(drive_prim, drive_type).GetStiffnessAttr()
    if attr:
        composition_stack = attr.GetPropertyStack()
        if composition_stack:
            return composition_stack[0]
    return None


def _gf_matrix3f_to_numpy(matrix: Gf.Matrix3f) -> np.ndarray:
    """Convert a Gf.Matrix3f to a numpy array.

    Args:
        matrix: The 3x3 Gf matrix.

    Returns:
        A 3x3 numpy array.
    """
    return np.array([[matrix[i][j] for j in range(3)] for i in range(3)])


def matrix_norm(matrix: Gf.Matrix3f) -> float:
    """Compute the Frobenius norm of a 3x3 matrix.

    Args:
        matrix: The 3x3 inertia tensor.

    Returns:
        The Frobenius norm of the matrix.
    """
    return np.linalg.norm(_gf_matrix3f_to_numpy(matrix))


def project_inertia_onto_axis(inertia_matrix: Gf.Matrix3f, axis: Gf.Vec3f) -> float:
    """Project an inertia tensor onto a specific axis to get the scalar moment of inertia.

    For a rotation about axis n, the effective moment of inertia is I_n = n^T * I * n.

    Args:
        inertia_matrix: The 3x3 inertia tensor.
        axis: Unit vector representing the rotation axis direction.

    Returns:
        Scalar moment of inertia about the specified axis.
    """
    axis_np = np.array([axis[0], axis[1], axis[2]])
    axis_norm = np.linalg.norm(axis_np)
    if axis_norm < 1e-9:
        carb.log_warn(
            f"GainTuner: degenerate joint rotation axis {axis_np.tolist()} (norm={axis_norm:.3e}); "
            "projected inertia is 0, so gain recommendations for affected joints may be invalid."
        )
        return 0.0
    axis_normalized = axis_np / axis_norm
    inertia_np = _gf_matrix3f_to_numpy(inertia_matrix)
    return float(axis_normalized @ inertia_np @ axis_normalized)


def _get_axis_attr_from_joint(joint_prim: pxr.Usd.Prim) -> pxr.Usd.Attribute:
    """Get the axis attribute from a revolute or prismatic joint.

    Args:
        joint_prim: The USD prim representing the joint.

    Returns:
        The axis attribute, or None if not a supported joint type.
    """
    if joint_prim.IsA(pxr.UsdPhysics.RevoluteJoint):
        return pxr.UsdPhysics.RevoluteJoint(joint_prim).GetAxisAttr()
    elif joint_prim.IsA(pxr.UsdPhysics.PrismaticJoint):
        return pxr.UsdPhysics.PrismaticJoint(joint_prim).GetAxisAttr()
    return None


def get_joint_axis_world_direction(joint_prim: pxr.Usd.Prim, joint_pose: Gf.Matrix4d) -> Gf.Vec3f:
    """Get the world direction of the joint rotation/translation axis.

    Args:
        joint_prim: The USD prim representing the joint.
        joint_pose: The joint's pose matrix in robot coordinates.

    Returns:
        Unit vector representing the joint axis direction in world coordinates.
    """
    local_axis = Gf.Vec3f(1, 0, 0)  # Default X axis

    axis_attr = _get_axis_attr_from_joint(joint_prim)
    if axis_attr:

        axis_token = axis_attr.Get()
        local_axis = _AXIS_TOKEN_TO_VECTOR.get(axis_token, local_axis)

    # Transform local axis to world coordinates using the rotation part of joint pose
    local_vec = Gf.Vec3d(local_axis[0], local_axis[1], local_axis[2])
    origin = Gf.Vec3d(0, 0, 0)
    transformed_origin = joint_pose.Transform(origin)
    transformed_point = joint_pose.Transform(local_vec)
    world_axis = transformed_point - transformed_origin
    return Gf.Vec3f(world_axis[0], world_axis[1], world_axis[2]).GetNormalized()


def _compute_outer_product_matrix(displacement: Gf.Vec3f) -> Gf.Matrix3f:
    """Compute the outer product matrix d * d^T for parallel axis theorem.

    Args:
        displacement: The displacement vector.

    Returns:
        The 3x3 outer product matrix.
    """
    d = displacement
    return Gf.Matrix3f(
        d[0] * d[0],
        d[0] * d[1],
        d[0] * d[2],
        d[1] * d[0],
        d[1] * d[1],
        d[1] * d[2],
        d[2] * d[0],
        d[2] * d[1],
        d[2] * d[2],
    )


def _compute_displacement_inertia_term(displacement: Gf.Vec3f, mass: float) -> Gf.Matrix3f:
    """Compute the displacement term for parallel axis theorem: m * (||d||^2 * I - d*d^T).

    Args:
        displacement: Vector from center of mass to new origin.
        mass: Mass of the object.

    Returns:
        The displacement contribution to the inertia tensor.
    """
    d_squared = displacement[0] ** 2 + displacement[1] ** 2 + displacement[2] ** 2
    identity = Gf.Matrix3f(1.0)
    outer_product = _compute_outer_product_matrix(displacement)

    displacement_term = Gf.Matrix3f(identity)
    displacement_term *= d_squared
    displacement_term -= outer_product
    displacement_term *= mass
    return displacement_term


def compute_parallel_axis_inertia(
    center_of_mass_inertia: Gf.Matrix3f, mass: float, displacement: Gf.Vec3f
) -> Gf.Matrix3f:
    """Apply the parallel axis theorem to translate an inertia tensor.

    Args:
        center_of_mass_inertia: Inertia tensor about center of mass.
        mass: Mass of the object.
        displacement: Vector from center of mass to new origin.

    Returns:
        Inertia tensor about the new origin.
    """
    displacement_term = _compute_displacement_inertia_term(displacement, mass)
    result = Gf.Matrix3f(center_of_mass_inertia)
    result += displacement_term
    return result


def find_articulation_root(stage: pxr.Usd.Stage, robot_path: str) -> str:
    """Find the articulation root prim for a robot.

    Args:
        stage: The USD stage.
        robot_path: Path to the robot prim.

    Returns:
        Path to the articulation root, or None if not found.
    """
    articulations = [
        prim for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path)) if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if articulations:
        return str(articulations[0].GetPath())
    if robot_path == "/":
        return None
    return find_articulation_root(stage, pxr.Sdf.Path(robot_path).GetParentPath())


def _schema_relationship_prims(
    stage: pxr.Usd.Stage,
    prim: pxr.Usd.Prim,
    relation: robot_schema.Relations,
    api: str,
    visited: set[str] | None = None,
) -> list[pxr.Usd.Prim]:
    """Return prims named by a robot-schema relationship.

    A target that carries ``api`` is included. A target that carries ``IsaacRobotAPI``
    is walked the same way, so a nested robot contributes the prims named by its own
    relationship.

    Args:
        stage: The USD stage containing the robot.
        prim: The robot prim whose relationship is read.
        relation: ``robotLinks`` or ``robotJoints``.
        api: Schema API a target must carry to be included.
        visited: Paths already walked, for cycles in the relationships.

    Returns:
        Prims in relationship order. Empty when the prim or the relationship is absent.
    """
    if not stage or not prim:
        return []
    if visited is None:
        visited = set()
    path = str(prim.GetPath())
    if path in visited:
        return []
    visited.add(path)

    if prim.HasAPI(api):
        return [prim]
    if not prim.HasAPI(robot_schema.Classes.ROBOT_API.value):
        return []
    relationship = prim.GetRelationship(relation.name)
    if not relationship:
        return []

    collected: list[pxr.Usd.Prim] = []
    for target in relationship.GetTargets():
        child = stage.GetPrimAtPath(target)
        if not child:
            continue
        if child.HasAPI(api):
            collected.append(child)
            continue
        if child.HasAPI(robot_schema.Classes.ROBOT_API.value):
            collected.extend(_schema_relationship_prims(stage, child, relation, api, visited))
    return collected


def _schema_link_tree(stage: pxr.Usd.Stage, robot_prim: pxr.Usd.Prim) -> rs_utils.RobotLinkNode | None:
    """Build the kinematic tree from the robot's schema links and joints.

    Args:
        stage: The USD stage containing the robot.
        robot_prim: The robot prim whose ``robotLinks`` and ``robotJoints`` are read.

    Returns:
        The tree rooted at the first schema link, or None when the robot names no links.
    """
    links = _schema_relationship_prims(
        stage, robot_prim, robot_schema.Relations.ROBOT_LINKS, robot_schema.Classes.LINK_API.value
    )
    joints = _schema_relationship_prims(
        stage, robot_prim, robot_schema.Relations.ROBOT_JOINTS, robot_schema.Classes.JOINT_API.value
    )
    if not links:
        return None

    root = rs_utils.RobotLinkNode(links[0])
    joints_per_body: list[dict[pxr.Sdf.Path, list[pxr.Usd.Prim]]] = [
        {link.GetPath(): [] for link in links} for _ in range(2)
    ]
    for joint in joints:
        for body_index in (0, 1):
            body = rs_utils.GetJointBodyRelationship(joint, body_index)
            if body and body in joints_per_body[body_index]:
                joints_per_body[body_index][body].append(joint)

    stack = [root]
    processed: list[pxr.Usd.Prim] = []
    while stack:
        current = stack.pop()
        current_path = current.prim.GetPath()
        for side in range(2):
            for joint in joints_per_body[side].get(current_path, []):
                if joint in processed:
                    continue
                processed.append(joint)
                other = rs_utils.GetJointBodyRelationship(joint, 1 - side)
                if other and (current.parent is None or current.parent.path != other):
                    child = rs_utils.RobotLinkNode(stage.GetPrimAtPath(other), current, joint)
                    current._joints.append(joint)
                    current.add_child(child)
                    stack.append(child)
    return root


class SinusoidalTest(RobotTest):
    """Registry adapter for the built-in sinusoidal test.

    Execution is handled by GainTuner's native code path.  This class
    exists so the test appears in the registry and can be discovered by
    name.  A full standalone ``run()`` implementation will be added when
    the test is extracted to ``isaacsim.robot_setup.validation_tests``.
    """

    name = "Sinusoidal"


class StepFunctionTest(RobotTest):
    """Registry adapter for the built-in step-function test.

    Execution is handled by GainTuner's native code path.  This class
    exists so the test appears in the registry and can be discovered by
    name.  A full standalone ``run()`` implementation will be added when
    the test is extracted to ``isaacsim.robot_setup.validation_tests``.
    """

    name = "Step Function"


class GainTuner:
    """Controller for tuning joint gains on articulated robots.

    This class provides functionality to:
    - Set up and configure robots for gain tuning.
    - Run sinusoidal or step response tests.
    - Compute inertia properties for gain estimation.
    - Record and analyze joint state data.

    Initializes the gain tuner with default values.
    """

    def __init__(self) -> None:
        self._timeline = timeline.get_timeline_interface()
        self._self_collision_override_stage: pxr.Usd.Stage | None = None
        self._self_collision_override_layer: pxr.Sdf.Layer | None = None
        # Robot paths already warned about for DOFs the robot schema does not list.
        # Deliberately survives reset() so a re-bind does not repeat the warning.
        self._warned_skipped_dofs: set[str] = set()
        self._test_duration = 5.0
        self._test_registry: dict[int, RobotTest] = {}
        self._active_test: RobotTest = None
        self._inertia_updated_callbacks: list[Callable[[], None]] = []
        self.reset()

    def reset(self) -> None:
        """Reset all internal state to initial values.

        The test registry is intentionally preserved across resets so that
        tests registered at startup remain available.
        """
        # Drop the override before the articulation goes, or a stage close or hot
        # reload mid-test leaves it applied and the state armed for a later
        # restore onto whatever robot is bound by then.
        self.restore_self_collision_override()
        self._initialized = False
        self._articulation = None
        self._articulation_root = None
        self._robot_prim_path = None
        self._robot = None
        self._joint_position_commands = []
        self._joint_velocity_commands = []
        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []
        self._joint_indices = None
        self._data_ready = False
        self._test_timestep = 0
        self._gains_test_generator = None
        self._joint_accumulated_inertia = {}
        self._joint_entries = []
        self._active_test = None
        self._test_result_metrics = {}
        self.step = 0

    def add_inertia_updated_callback(self, callback: Callable[[], None]) -> None:
        """Register a callback invoked after joint accumulated inertia is recomputed.

        Args:
            callback: Zero-argument callback to invoke after inertia recomputation.
        """
        if callback not in self._inertia_updated_callbacks:
            self._inertia_updated_callbacks.append(callback)

    def _notify_inertia_updated(self) -> None:
        for callback in self._inertia_updated_callbacks:
            try:
                callback()
            except Exception as exc:
                carb.log_warn(f"GainTuner: inertia-updated callback failed: {exc}")

    def stop_test(self) -> None:
        """Stop the current test and reset the articulation to default state.

        Does nothing to the robot when no articulation is bound, so the call stays
        safe before :meth:`setup` and after :meth:`reset`.
        """
        if self._active_test is not None:
            self._active_test.stop()
            self._active_test = None
        if self._articulation is None:
            return
        self._articulation.reset_to_default_state()

    # ======================== Test Registry ========================

    def register_test(self, mode_id: int, test: RobotTest) -> None:
        """Register a RobotTest under the given mode ID.

        Args:
            mode_id: Integer key (typically a GainsTestMode value).
            test: RobotTest instance to register.
        """
        self._test_registry[mode_id] = test

    def unregister_test(self, mode_id: int) -> None:
        """Remove a previously registered test.

        Args:
            mode_id: Integer key of the test to remove.
        """
        self._test_registry.pop(mode_id, None)

    def get_registered_tests(self) -> dict[int, RobotTest]:
        """Return a copy of the test registry.

        Returns:
            Dictionary mapping mode IDs to RobotTest instances.
        """
        return dict(self._test_registry)

    def on_reset(self) -> None:
        """Handle simulation reset event."""
        self._initialized = False
        if self._robot_prim_path:
            robot_path = self._robot_prim_path
            self.reset()
            self.setup(robot_path)
        else:
            self.setup(None)
        self.step = 0

    def setup(self, robot_path: str | None) -> None:
        """Configure the gain tuner for a specific robot.

        Args:
            robot_path: USD path to the robot prim, or ``None`` to skip binding.
        """
        if robot_path is None:
            return
        # Re-bind when the path is unchanged but the stage was rebuilt (prim at ``robot_path`` is new
        # USD, or articulation was cleared by :meth:`reset`).
        if robot_path == self._robot_prim_path and self._articulation is not None:
            stage = omni.usd.get_context().get_stage()
            if stage and self._robot and self._robot.IsValid():
                return

        stage = omni.usd.get_context().get_stage()
        self._robot_prim_path = robot_path
        self._robot = stage.GetPrimAtPath(robot_path)

        self._setup_robot_links(stage)
        self._setup_articulation(stage)
        self._setup_fixed_links(stage)
        self._setup_joint_entries(stage)

        self._joint_names = {i: self._joint_entries[i].joint.GetName() for i in self._joints}
        self._joint_map = {self._joint_names[i]: self._joints[i] for i in self._joints}
        self._all_joint_indices = self._articulation.get_dof_indices(self._joint_names.values()).list()

        self.initialize()

    def _setup_robot_links(self, stage: pxr.Usd.Stage) -> None:
        """Set up robot link mass properties.

        Args:
            stage: The USD stage.
        """
        self._robot_links = [
            link for link in pxr.Usd.PrimRange(self._robot) if link.HasAPI(robot_schema.Classes.LINK_API.value)
        ]

        async def update_link_mass() -> None:
            was_playing = self._timeline.is_playing()
            if not was_playing:
                self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            try:
                self.compute_joints_accumulated_inertia()
                self._notify_inertia_updated()
            except AssertionError:
                carb.log_warn(
                    "GainTuner: compute_joints_accumulated_inertia skipped in deferred link-mass update "
                    "(physics tensor not ready yet)."
                )
            if not was_playing:
                self._timeline.stop()

        asyncio.ensure_future(update_link_mass())
        self._robot_tree = _schema_link_tree(stage, self._robot)

    def _setup_articulation(self, stage: pxr.Usd.Stage) -> None:
        """Set up the articulation wrapper.

        Args:
            stage: The USD stage.
        """
        self._articulation_root = find_articulation_root(stage, self._robot_prim_path)
        self._articulation = Articulation(self._articulation_root)

    def _setup_fixed_links(self, stage: pxr.Usd.Stage) -> None:
        """Identify links that are fixed to the world.

        Args:
            stage: The USD stage.
        """
        robot_joints = _schema_relationship_prims(
            stage, self._robot, robot_schema.Relations.ROBOT_JOINTS, robot_schema.Classes.JOINT_API.value
        )

        # Find fixed joints with one body connected to world (None)
        fixed_joint_candidates = [
            joint
            for joint in robot_joints
            if pxr.UsdPhysics.FixedJoint(joint)
            and (
                rs_utils.GetJointBodyRelationship(joint, 0) is None
                or rs_utils.GetJointBodyRelationship(joint, 1) is None
            )
        ]

        # Filter to only enabled fixed joints
        fixed_joints = []
        for joint in fixed_joint_candidates:
            enabled_attr = pxr.UsdPhysics.Joint(joint).GetJointEnabledAttr()
            if enabled_attr and enabled_attr.Get():
                fixed_joints.append(joint)

        # Collect all links connected to fixed joints
        fixed_links = set()
        for joint in fixed_joints:
            for body_index in (0, 1):
                link = rs_utils.GetJointBodyRelationship(joint, body_index)
                if link is not None:
                    fixed_links.add(link)

        self._fixed_links = fixed_links
        self._robot_joints = robot_joints

    def _setup_joint_entries(self, stage: pxr.Usd.Stage) -> None:
        """Build the list of joint entries for the UI.

        Args:
            stage: The USD stage.
        """
        joints = [stage.GetPrimAtPath(self._articulation.dof_paths[0][i]) for i in range(self._articulation.num_dofs)]

        self._joints = {}
        self._joint_entries = []
        dof_offset = 0
        dof_index = 0
        skipped_joint_paths = []

        for joint in joints:
            if joint not in self._robot_joints:
                # `joints` is DOF-indexed, so the counter has to advance past a
                # dropped DOF as well. Skipping the increment leaves every later
                # entry reading its name and type from the wrong DOF.
                skipped_joint_paths.append(str(joint.GetPath()))
                dof_index += 1
                continue

            display_name = self._articulation.dof_names[dof_index]

            if joint.IsA(pxr.UsdPhysics.PrismaticJoint) or joint.IsA(pxr.UsdPhysics.RevoluteJoint):
                self._add_single_axis_joint_entry(joint, display_name, dof_index + dof_offset)
                dof_index += 1
                continue

            # Handle multi-axis (D6) joints
            dof_offset = self._add_multi_axis_joint_entries(joint, dof_index, dof_offset)
            dof_index += 1

        if skipped_joint_paths and self._robot_prim_path not in self._warned_skipped_dofs:
            # Warn once per robot: setup() re-runs on every robot re-bind, and the
            # deferred joint retry runs it twice for a single selection.
            self._warned_skipped_dofs.add(self._robot_prim_path)
            unique_paths = list(dict.fromkeys(skipped_joint_paths))
            carb.log_warn(
                f"GainTuner: {len(skipped_joint_paths)} of {len(joints)} articulation DOFs on "
                f"'{self._robot_prim_path}' have no joint in the robot schema and are missing from the "
                f"joint table: {unique_paths}. Re-run the robot schema population on this asset so the "
                "joints appear in the Gain Tuner."
            )

    def _add_single_axis_joint_entry(self, joint: pxr.Usd.Prim, display_name: str, dof_index: int) -> None:
        """Add a single-axis joint to the entries list.

        Args:
            joint: The joint prim.
            display_name: Display name for the joint.
            dof_index: The DOF index.
        """
        self._joints[dof_index] = joint
        self._joint_entries.append(
            JointListEntry(joint=joint, display_name=display_name, dof_index=dof_index, drive_axis=None)
        )

    def _add_multi_axis_joint_entries(self, joint: pxr.Usd.Prim, base_dof_index: int, dof_offset: int) -> int:
        """Add multi-axis (D6) joint entries.

        Args:
            joint: The joint prim.
            base_dof_index: Base DOF index for this joint.
            dof_offset: Current offset for multi-axis joints.

        Returns:
            Updated dof_offset.
        """
        for axis_index, drive_axis in enumerate(_D6_AXIS_TOKENS):
            display_name = self._articulation.dof_names[base_dof_index]

            if joint.IsA(pxr.UsdPhysics.Joint) and not joint.IsA(pxr.UsdPhysics.FixedJoint):
                carb.log_warn(
                    f"Joint: {joint.GetName()} is a multi-axis joint and Articulation does not "
                    "support it. The gains tuner will allow editing the gains but Running tests "
                    "and Charts may have unpredictable results"
                )
                if not _d6_axis_has_unlocked_limit(joint, drive_axis):
                    continue
                display_name = _format_d6_display_name(joint, display_name, drive_axis)

            current_dof_index = base_dof_index + dof_offset
            self._joints[current_dof_index] = joint
            self._joint_entries.append(
                JointListEntry(
                    joint=joint, display_name=display_name, dof_index=current_dof_index, drive_axis=drive_axis
                )
            )
            dof_offset += 1

        return dof_offset

    def get_dof_type(self, dof_index: int) -> int:
        """Get the type of a degree of freedom.

        Args:
            dof_index: Index of the DOF.

        Returns:
            The DOF type enumeration value.
        """
        return self._articulation.dof_types[dof_index]

    def get_dof_effective_max_velocity(self, dof_index: int) -> float | None:
        """Get the velocity limit the engine enforces on a DOF, in schema units.

        The velocity sweeps in :meth:`step_sinusoid` and :meth:`step_step` scale
        their commands by the engine's limit, not by the USD opinion the Advanced
        panel resolves.  Where the two disagree -- most often because the limit is
        authored on a schema the running backend's resolver chain does not read --
        the panel would report one limit while the sweep excited the joint at
        another.  This exposes the value the sweep actually uses so the panel can
        show it and say which one is in force.

        The tensor API reports angular limits in radians per second while both
        joint schemas store degrees per second, so rotational DOFs are converted
        to degrees here: the result is directly comparable with
        :func:`~isaacsim.robot_setup.gain_tuner.resolve_joint_param`.

        Args:
            dof_index: Index of the DOF.

        Returns:
            The enforced limit in the schemas' units (degrees per second for
            rotational DOFs, linear units per second otherwise),
            :data:`math.inf` when the engine enforces no practical limit (see
            :data:`UNLIMITED_VELOCITY_THRESHOLD`), or None when no articulation is
            bound or the physics view cannot be queried -- which is the case until
            the timeline has played.  None means "no engine truth", never
            "unlimited", so a caller can tell the two apart.

        Example:

        .. code-block:: python

            >>> gains_tuner.get_dof_effective_max_velocity(0)  # doctest: +NO_CHECK
            180.0
        """
        if self._articulation is None:
            return None
        try:
            max_velocity = float(self._articulation.get_dof_max_velocities(dof_indices=[dof_index]).numpy().item())
        except Exception:
            # The physics view is unavailable until the timeline plays, and the
            # USD fallback path needs a resolvable DOF prim; either way there is
            # no engine truth to report yet.
            return None
        if not math.isfinite(max_velocity) or abs(max_velocity) >= UNLIMITED_VELOCITY_THRESHOLD:
            return math.inf
        if self._articulation.dof_types[dof_index] == DofType.Rotation:
            max_velocity = math.degrees(max_velocity)
        return max_velocity

    def get_dof_engine_armature(self, dof_index: int) -> float | None:
        """Get the armature the running engine simulates a DOF with.

        A joint that authors no armature is still simulated with one -- Newton
        applies its ``ModelBuilder`` default rather than zero -- so the engine is
        the only source that can say what the value actually is.  Both backends
        store armature unscaled, so no unit conversion is applied.

        Args:
            dof_index: Index of the DOF.

        Returns:
            The armature in effect, or None when no articulation is bound, the
            physics view cannot be queried (the case until the timeline has
            played), or the backend does not report armature.

        Example:

        .. code-block:: python

            >>> gains_tuner.get_dof_engine_armature(0)  # doctest: +NO_CHECK
            0.1
        """
        if self._articulation is None:
            return None
        try:
            armature = float(self._articulation.get_dof_armatures(dof_indices=[dof_index]).numpy().item())
        except Exception:
            # No physics view before the first play, and not every backend
            # implements the armature tensor.
            return None
        return armature if math.isfinite(armature) else None

    def invalidate_physics_views(self) -> None:
        """Drop the state owned by the physics engine, keeping the robot selection.

        Switching the active physics engine invalidates the tensor views behind the
        bound articulation, but leaves this object's handle non-None, so the
        no-articulation guards do not catch it and the next ``get_dof_*`` call
        reaches into an invalidated view.  This releases the handle so the caller
        can re-acquire one against the new engine, and discards the recorded
        response, which belongs to the solver that produced it and must not be
        charted alongside samples from another.

        The robot prim path and joint entries are kept, so the caller can re-bind
        the same robot instead of making the user pick it again.

        Example:

        .. code-block:: python

            >>> gains_tuner.invalidate_physics_views()  # doctest: +NO_CHECK
        """
        self._articulation = None
        self._articulation_root = None
        self._initialized = False
        self._joint_position_commands = []
        self._joint_velocity_commands = []
        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []
        self._data_ready = False
        self._test_timestep = 0
        self._gains_test_generator = None
        self._active_test = None
        self._test_result_metrics = {}
        self.step = 0

    def __del__(self) -> None:
        """Clean up resources."""
        self._articulation = None

    def initialize(self) -> None:
        """Initialize the gain tuner with the current articulation state."""
        if self._articulation and self._timeline.is_playing():
            positions, orientations = self._articulation.get_world_poses()
            linear_velocities, angular_velocities = self._articulation.get_velocities()
            dof_positions = self._articulation.get_dof_positions()
            dof_velocities = self._articulation.get_dof_velocities()
            dof_efforts = self._articulation.get_dof_efforts()
            self._articulation.set_default_state(
                positions=positions,
                orientations=orientations,
                linear_velocities=linear_velocities,
                angular_velocities=angular_velocities,
                dof_positions=dof_positions,
                dof_velocities=dof_velocities,
                dof_efforts=dof_efforts,
            )
            self._articulation.reset_to_default_state()
            self._initialized = True

    @property
    def initialized(self) -> bool:
        """Whether the gain tuner has been initialized.

        Returns:
            True if initialized, False otherwise.
        """
        return self._initialized

    @initialized.setter
    def initialized(self, value: bool) -> None:
        """Set the initialized state.

        Args:
            value: True if initialized, False otherwise.
        """
        self._initialized = value

    @property
    def joint_range_maximum(self) -> float:
        """Joint range maximum.

        Returns:
            Joint range maximum.
        """
        return self._joint_range_maximum

    @joint_range_maximum.setter
    def joint_range_maximum(self, value: float) -> None:
        """Set the joint range maximum.

        Args:
            value: Joint range maximum.
        """
        self._joint_range_maximum = value

    @property
    def position_impulse(self) -> float:
        """Position impulse.

        Returns:
            Position impulse.
        """
        return self._position_impulse

    @position_impulse.setter
    def position_impulse(self, value: float) -> None:
        """Set the position impulse.

        Args:
            value: Position impulse.
        """
        self._position_impulse = value

    @property
    def velocity_impulse(self) -> float:
        """Velocity impulse.

        Returns:
            Velocity impulse.
        """
        return self._velocity_impulse

    @velocity_impulse.setter
    def velocity_impulse(self, value: float) -> None:
        """Set the velocity impulse.

        Args:
            value: Velocity impulse.
        """
        self._velocity_impulse = value

    @property
    def robot(self) -> pxr.Usd.Prim:
        """Robot prim.

        Returns:
            The robot prim.
        """
        return self._robot

    def _accumulate_link_inertia(
        self,
        links: list[pxr.Usd.Prim],
        joint_pose: Gf.Matrix4d,
        robot_transform: Gf.Matrix4d,
        is_prismatic: bool,
        link_path_to_index: dict[Sdf.Path, int],
        link_masses: np.ndarray,
        link_com_positions: np.ndarray,
        link_inertias: np.ndarray,
    ) -> tuple[float, Gf.Matrix3f, bool]:
        """Accumulate mass and inertia for a set of links.

        Args:
            links: List of link prims to accumulate.
            joint_pose: The joint's pose matrix.
            robot_transform: The robot's world transform.
            is_prismatic: True if the joint is prismatic.
            link_path_to_index: Mapping from link prim path to articulation link index.
            link_masses: Mass for each link, shape ``(L,)``.
            link_com_positions: Local-frame COM positions for each link, shape ``(L, 3)``.
            link_inertias: Full 3x3 inertia tensors (row-major) for each link, shape ``(L, 9)``.

        Returns:
            Tuple of (total_mass, accumulated_inertia, is_fixed).
            Returns (0, zero_matrix, True) if a fixed link is encountered.
        """
        total_mass = 0.0
        accumulated_inertia = pxr.Gf.Matrix3f().SetZero()

        for link in links:
            link_path = link.GetPath()

            if link_path in self._fixed_links:
                return (0.0, pxr.Gf.Matrix3f().SetZero(), True)

            link_index = link_path_to_index.get(link_path)
            if link_index is None:
                continue

            mass = float(link_masses[link_index])
            total_mass += mass

            if not is_prismatic:
                link_pose = omni.usd.get_world_transform_matrix(link)
                com_local = link_com_positions[link_index].astype(float)
                world_com = pxr.Gf.Vec3d(
                    (
                        link_pose
                        * pxr.Gf.Matrix4d().SetTranslate(pxr.Gf.Vec3d(*com_local))
                        * robot_transform.GetInverse()
                    ).ExtractTranslation()
                )
                displacement = world_com - joint_pose.ExtractTranslation()

                inertia_flat = link_inertias[link_index].astype(float)
                body_frame_inertia = pxr.Gf.Matrix3f(
                    inertia_flat[0],
                    inertia_flat[1],
                    inertia_flat[2],
                    inertia_flat[3],
                    inertia_flat[4],
                    inertia_flat[5],
                    inertia_flat[6],
                    inertia_flat[7],
                    inertia_flat[8],
                )
                transformed_inertia = compute_parallel_axis_inertia(body_frame_inertia, mass, displacement)
                accumulated_inertia += transformed_inertia

        return (total_mass, accumulated_inertia, False)

    def compute_joints_accumulated_inertia(self) -> None:
        """Compute the effective inertia for each joint about its motion axis.

        For revolute joints (rotational spring-damper):
            I * theta'' + D * theta' + K * theta = tau
            Effective inertia is the moment of inertia projected onto the rotation axis.

        For prismatic joints (linear spring-damper):
            m * x'' + D * x' + K * x = F
            Effective inertia is simply the accumulated mass (F = ma).

        The equivalent inertia for series connection of both sides:
            - If one side is fixed: I_eq = I_moving
            - If both sides move: I_eq = (I1 * I2) / (I1 + I2)

        Note:
            This computation is dependent on the initial position of the robot.
            Natural frequency will be heavily biased by the initial position.
        """
        if not self._robot or not self._articulation or self._robot_tree is None:
            return

        # Build link path -> link index mapping
        link_paths = self._articulation.link_paths[0]
        link_path_to_index = {Sdf.Path(path): i for i, path in enumerate(link_paths)}

        # Query all link mass properties from the Articulation tensor API
        link_masses = self._articulation.get_link_masses().numpy()[0]  # (L,)
        com_positions, _ = self._articulation.get_link_coms()
        link_com_positions = com_positions.numpy()[0]  # (L, 3)
        link_inertias = self._articulation.get_link_inertias().numpy()[0]  # (L, 9)

        robot_transform = omni.usd.get_world_transform_matrix(self._robot)
        joint_inertia = {}

        for joint in self._joints.values():
            backward_links, forward_links = rs_utils.GetLinksFromJoint(self._robot_tree, joint)
            joint_pose = rs_utils.GetJointPose(self._robot, joint)
            is_prismatic = joint.IsA(pxr.UsdPhysics.PrismaticJoint)
            joint_axis = get_joint_axis_world_direction(joint, joint_pose)

            # Accumulate inertia for backward links
            backward_result = self._accumulate_link_inertia(
                backward_links,
                joint_pose,
                robot_transform,
                is_prismatic,
                link_path_to_index,
                link_masses,
                link_com_positions,
                link_inertias,
            )
            backward_mass, backward_inertia, backward_fixed = backward_result

            # Accumulate inertia for forward links
            forward_result = self._accumulate_link_inertia(
                forward_links,
                joint_pose,
                robot_transform,
                is_prismatic,
                link_path_to_index,
                link_masses,
                link_com_positions,
                link_inertias,
            )
            forward_mass, forward_inertia, forward_fixed = forward_result

            # Compute effective inertia based on joint type
            if is_prismatic:
                inertia_forward = forward_mass
                inertia_backward = backward_mass
            else:
                inertia_forward = project_inertia_onto_axis(forward_inertia, joint_axis)
                inertia_backward = project_inertia_onto_axis(backward_inertia, joint_axis)

            # Compute equivalent inertia for spring-damper system
            equivalent_inertia = self._compute_equivalent_inertia(
                inertia_forward, inertia_backward, forward_fixed, backward_fixed
            )
            joint_inertia[joint] = equivalent_inertia

        self._joint_accumulated_inertia = joint_inertia

    def _compute_equivalent_inertia(
        self,
        inertia_forward: float,
        inertia_backward: float,
        forward_fixed: bool,
        backward_fixed: bool,
    ) -> float:
        """Compute the equivalent inertia for a spring-damper system.

        For series connection: I_eq = (I1 * I2) / (I1 + I2).
        If one side is fixed, only the moving side contributes.

        Args:
            inertia_forward: Inertia of the forward link chain.
            inertia_backward: Inertia of the backward link chain.
            forward_fixed: True if forward chain is fixed to world.
            backward_fixed: True if backward chain is fixed to world.

        Returns:
            The equivalent inertia value.
        """
        epsilon = 1e-9

        if backward_fixed or inertia_backward < epsilon:
            return inertia_forward
        if forward_fixed or inertia_forward < epsilon:
            return inertia_backward

        return (inertia_forward * inertia_backward) / (inertia_forward + inertia_backward)

    @staticmethod
    def _detach_override_layer(stage: pxr.Usd.Stage, layer: pxr.Sdf.Layer) -> None:
        """Detach a temporary override sublayer from the stage's session layer.

        Args:
            stage: The stage the sublayer was attached to.
            layer: The anonymous layer to detach.
        """
        session = stage.GetSessionLayer()
        if layer.identifier in session.subLayerPaths:
            session.subLayerPaths.remove(layer.identifier)

    @staticmethod
    def _disable_self_collision_attr(
        prim: pxr.Usd.Prim, attr_name: str, apply_api: Callable[[pxr.Usd.Prim], object]
    ) -> bool:
        """Author one self-collision property to False, applying its API schema if absent.

        Args:
            prim: The articulation root to author on.
            attr_name: Property to author.
            apply_api: Callable applying the API schema that defines `attr_name`.

        Returns:
            True when the property was authored, False when it could not be.
        """
        attr = prim.GetAttribute(attr_name)
        if not attr:
            # A robot carrying only the other engine's schema does not have the
            # property, and `GetAttribute` answers that with an invalid attribute
            # rather than raising, so a bare `Set` on it would silently do nothing.
            try:
                apply_api(prim)
            except Tf.ErrorException:
                # Applying an unregistered schema raises rather than returning
                # False, which is how a missing schema extension shows up. The
                # other engine's property still carries the override.
                return False
            attr = prim.GetAttribute(attr_name)
        if not attr:
            return False
        attr.Set(False)
        return True

    def apply_self_collision_override(self) -> str | None:
        """Disable articulation self-collisions for the duration of a test.

        Both engines read their property when the scene is cooked, so the caller
        is responsible for restarting the timeline after this returns.

        `physxArticulation:enabledSelfCollisions` and `newton:selfCollisionEnabled`
        are both authored, so the override holds whichever engine runs and does not
        depend on the Newton resolver falling back to the PhysX property.

        The opinions are authored on an anonymous sublayer of the **session layer**,
        which is stronger than the root layer and is never saved. The engine
        therefore composes the disabled value while the user's asset stays exactly
        as it shipped -- no API schema applied to it, no value authored, and no
        unsaved edits to prompt about. :meth:`restore_self_collision_override`
        detaches that sublayer, removing every trace of the override in one step.

        Calling this while an override is already active is a no-op, so a second
        run cannot mistake the override for the robot's own setting.

        Returns:
            None when the override is active, or a human-readable reason it could
            not be applied, for the caller to surface.

        Example:

        .. code-block:: python

            >>> reason = gains_tuner.apply_self_collision_override()  # doctest: +NO_CHECK
            >>> print(reason)  # doctest: +NO_CHECK
            None
            >>> gains_tuner.self_collision_override_active()  # doctest: +NO_CHECK
            True
        """
        if self._self_collision_override_layer is not None:
            return None
        if self._articulation is None:
            return "no robot is loaded"

        prim = self._articulation.prims[0]
        stage = prim.GetStage() if prim else None
        if stage is None:
            return "the robot is not on an open stage"

        override_layer = Sdf.Layer.CreateAnonymous("gain_tuner_self_collision.usda")
        session_layer = stage.GetSessionLayer()
        session_layer.subLayerPaths.insert(0, override_layer.identifier)

        # Anything that goes wrong from here on has to take the sublayer with it,
        # or the stage keeps composing a disabled value nothing knows how to undo.
        applied = False
        try:
            with Usd.EditContext(stage, override_layer):
                authored = [
                    attr_name
                    for attr_name, apply_api in (
                        (_SELF_COLLISION_ATTR, PhysxSchema.PhysxArticulationAPI.Apply),
                        (_NEWTON_SELF_COLLISION_ATTR, _apply_newton_articulation_root_api),
                    )
                    if self._disable_self_collision_attr(prim, attr_name, apply_api)
                ]

            if not authored:
                return (
                    f"neither '{_SELF_COLLISION_ATTR}' nor '{_NEWTON_SELF_COLLISION_ATTR}' "
                    "could be authored on the articulation root"
                )

            # A stronger opinion elsewhere in the session layer outranks this
            # sublayer, so authoring is not the same as taking effect. Read the
            # composed value back rather than reporting a success the engine
            # will not honour.
            still_enabled = [name for name in authored if prim.GetAttribute(name).Get()]
            if still_enabled:
                return f"a stronger opinion on the stage keeps {' and '.join(sorted(still_enabled))} enabled"

            applied = True
        finally:
            if not applied:
                self._detach_override_layer(stage, override_layer)

        self._self_collision_override_stage = stage
        self._self_collision_override_layer = override_layer
        return None

    def restore_self_collision_override(self) -> None:
        """Drop a self-collision override applied by :meth:`apply_self_collision_override`.

        Detaches the override from the stage it was applied to, not from whichever
        robot is bound now, so a robot swap, an engine switch, or deleting the
        robot mid-test cannot leave the override stranded on the stage. Safe to
        call when no override is active.

        Example:

        .. code-block:: python

            >>> gains_tuner.restore_self_collision_override()  # doctest: +NO_CHECK
            >>> gains_tuner.self_collision_override_active()  # doctest: +NO_CHECK
            False
        """
        stage = self._self_collision_override_stage
        layer = self._self_collision_override_layer
        self._self_collision_override_stage = None
        self._self_collision_override_layer = None

        if layer is None or stage is None:
            return
        try:
            self._detach_override_layer(stage, layer)
        except Tf.ErrorException:
            # The stage was closed, and the session layer went with it.
            return

    def self_collision_override_active(self) -> bool:
        """Return whether a self-collision override is currently applied.

        Returns:
            True when :meth:`apply_self_collision_override` succeeded and has not
            been restored yet.

        Example:

        .. code-block:: python

            >>> gains_tuner.self_collision_override_active()  # doctest: +NO_CHECK
            False
        """
        return self._self_collision_override_layer is not None

    def get_articulation(self) -> Articulation:
        """Get the articulation wrapper.

        Returns:
            The articulation object.
        """
        return self._articulation

    def get_articulation_root(self) -> str | None:
        """Get the resolved articulation root prim path.

        Returns:
            The articulation root path, or None if no robot is loaded. Useful for
            locating the robot's ``Actuators`` scope when reading actuator gains.
        """
        return self._articulation_root

    def get_all_joint_indices(self) -> list[int]:
        """Get all joint DOF indices.

        Returns:
            List of all joint indices.
        """
        return self._all_joint_indices

    def get_joint_accumulated_inertia(self, joint: object) -> float:
        """Get the effective inertia (or mass) about a joint's motion axis.

        The value is populated by :meth:`compute_joints_accumulated_inertia`,
        which requires the articulation's physics tensors to be available (the
        timeline must have played at least one step).  It is used as the
        equivalent inertia ``m_eq`` in the natural-frequency drive conversions.

        Args:
            joint: The joint prim to look up.

        Returns:
            The accumulated inertia (revolute, kg*m^2) or mass (prismatic, kg) for
            the joint, or 0.0 when it has not been computed yet.
        """
        return self._joint_accumulated_inertia.get(joint, 0.0)

    def get_joint_entries(self) -> list[JointListEntry]:
        """Get the list of joint entries.

        Returns:
            Copy of the joint entries list.
        """
        return list(self._joint_entries)

    def get_permanent_fixed_joint_indices(self) -> list[int]:
        """Get indices of permanently fixed joints.

        Returns:
            List of fixed joint indices.
        """
        return self._permanent_fixed_joint_indices

    def get_test_duration(self) -> float:
        """Get the test duration.

        Returns:
            Test duration in seconds.
        """
        return self._test_duration

    def is_data_ready(self) -> bool:
        """Check if test data is ready for analysis.

        Returns:
            True if data is ready, False otherwise.
        """
        return self._data_ready

    def get_test_result_metrics(self) -> dict[int, dict]:
        """Return per-joint metrics from the last registered test run.

        Returns:
            Dict mapping DOF index to a metrics dict, or empty dict if
            no registered test has been run.
        """
        return self._test_result_metrics

    def get_robot_prim_path(self) -> str | None:
        """Get the prim path of the robot passed to :meth:`setup`.

        Returns:
            The robot prim path, or None when no robot has been set up. This is
            the path the caller selected, which may differ from the resolved
            articulation root returned by :meth:`get_articulation_root`.
        """
        return self._robot_prim_path

    def snapshot_recorded_trajectory(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Snapshot the full recorded trajectory arrays from the last test run.

        Returns the complete (all-DOF) command and observed arrays, unlike
        :meth:`get_joint_states_from_gains_test`, which slices a single joint.
        Useful for orchestrators (such as the dt sweep) that must retain a level's
        trajectory before a subsequent run overwrites it, then restore it later via
        :meth:`ingest_sweep_results`.

        Returns:
            Tuple of (position_commands, velocity_commands, observed_positions,
            observed_velocities, timestamps) referencing the current arrays.
        """
        return (
            self._joint_position_commands,
            self._joint_velocity_commands,
            self._observed_joint_positions,
            self._observed_joint_velocities,
            self._command_times,
        )

    def ingest_sweep_results(
        self,
        metrics: dict[int, dict],
        trajectory: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None,
    ) -> None:
        """Install externally-aggregated sweep results as the tuner's current results.

        The dt physics sweep runs the single-level probe repeatedly from the UI
        layer, aggregates the per-level samples outside the tuner, and then hands
        the combined per-joint metrics back here so the charts and detail panels
        read them through the normal :meth:`get_test_result_metrics` /
        :meth:`get_joint_states_from_gains_test` API. Pass ``trajectory`` (as
        returned by :meth:`snapshot_recorded_trajectory`) to also restore a chosen
        level's recorded arrays for the Position / Effort charts.

        Args:
            metrics: Per-DOF aggregated metrics dict to expose as the test results.
            trajectory: Optional full trajectory arrays to restore; when provided,
                the recorded arrays are replaced and the data-ready flag is set so
                the plotting API reports the results as available.
        """
        self._test_result_metrics = metrics
        if trajectory is not None:
            (
                self._joint_position_commands,
                self._joint_velocity_commands,
                self._observed_joint_positions,
                self._observed_joint_velocities,
                self._command_times,
            ) = trajectory
            self._data_ready = True

    def clear_test_results(self) -> None:
        """Clear the last test's results so stale metrics are no longer reported.

        Resets the per-joint metrics and marks the recorded data as not ready, so
        :meth:`is_data_ready` returns False and results panels fall back to their
        empty state. Used when switching test modes, where the previous mode's
        metric set no longer applies.
        """
        self._test_result_metrics = {}
        self._data_ready = False

    def set_test_duration(self, duration: float) -> None:
        """Set the test duration.

        Args:
            duration: Test duration in seconds.
        """
        self._test_duration = duration

    # ======================== Run Gains Test ========================

    def _partition_joints_by_mode(self, sequence_index: int) -> tuple[list[int], list[int], list[int], list[int]]:
        """Partition joints into position and velocity control groups.

        Args:
            sequence_index: Index of the current test sequence.

        Returns:
            Tuple of (position_dof_indices, velocity_dof_indices,
                     position_param_indices, velocity_param_indices).
        """
        joint_indices = self.test_params["sequence"][sequence_index]["joint_indices"]

        position_dof_indices = [i for i in joint_indices if self.joint_modes[i] == JointMode.POSITION]
        velocity_dof_indices = [i for i in joint_indices if self.joint_modes[i] == JointMode.VELOCITY]
        position_param_indices = [i for i, j in enumerate(joint_indices) if self.joint_modes[j] == JointMode.POSITION]
        velocity_param_indices = [i for i, j in enumerate(joint_indices) if self.joint_modes[j] == JointMode.VELOCITY]

        return position_dof_indices, velocity_dof_indices, position_param_indices, velocity_param_indices

    def _get_sequence_params(self, sequence_index: int, param_indices: list[int], param_name: str) -> np.ndarray:
        """Extract parameter values for specific joint indices from a sequence.

        Args:
            sequence_index: Index of the test sequence.
            param_indices: Indices to extract.
            param_name: Name of the parameter in the sequence config.

        Returns:
            Numpy array of parameter values.
        """
        sequence = self.test_params["sequence"][sequence_index]
        return np.array([sequence[param_name][i] for i in param_indices])

    def sinusoidal_step(
        self, timestep: float, sequence_index: int
    ) -> tuple[list[int], np.ndarray, list[int], np.ndarray]:
        """Generate sinusoidal position and velocity commands.

        Args:
            timestep: Current simulation time.
            sequence_index: Index of the current test sequence.

        Returns:
            Tuple of (position_indices, position_commands,
                     velocity_indices, velocity_commands).
        """
        pos_dof_idx, vel_dof_idx, pos_param_idx, vel_param_idx = self._partition_joints_by_mode(sequence_index)

        # Position control - sinusoidal motion within joint limits
        lower_limits, upper_limits = [
            np.array(lim.list()) for lim in self._articulation.get_dof_limits(dof_indices=pos_dof_idx)
        ]
        amplitudes = self._get_sequence_params(sequence_index, pos_param_idx, "joint_amplitudes")
        offsets = self._get_sequence_params(sequence_index, pos_param_idx, "joint_offsets")
        periods = self._get_sequence_params(sequence_index, pos_param_idx, "joint_periods")
        phases = self._get_sequence_params(sequence_index, pos_param_idx, "joint_phases")

        joint_range = upper_limits - lower_limits
        center = (upper_limits + lower_limits) / 2
        position_command = np.clip(
            joint_range * amplitudes * np.sin(2 * np.pi * timestep / periods + phases) + center + offsets,
            lower_limits,
            upper_limits,
        )

        # Velocity control - sinusoidal velocity
        vel_periods = self._get_sequence_params(sequence_index, vel_param_idx, "joint_periods")
        vel_phases = self._get_sequence_params(sequence_index, vel_param_idx, "joint_phases")
        max_velocities = self._articulation.get_dof_max_velocities(dof_indices=vel_dof_idx).numpy()
        velocity_command = (
            max_velocities * 2 * np.pi / vel_periods * np.sin(2 * np.pi * timestep / vel_periods + vel_phases)
        )

        return pos_dof_idx, position_command, vel_dof_idx, velocity_command

    def step_step(self, timestep: float, sequence_index: int) -> tuple[list[int], np.ndarray, list[int], np.ndarray]:
        """Generate square wave position and velocity commands.

        Args:
            timestep: Current simulation time.
            sequence_index: Index of the current test sequence.

        Returns:
            Tuple of (position_indices, position_commands,
                     velocity_indices, velocity_commands).
        """
        pos_dof_idx, vel_dof_idx, pos_param_idx, vel_param_idx = self._partition_joints_by_mode(sequence_index)

        # Position control - square wave between step_min and step_max
        lower_limits, upper_limits = [
            np.array(lim.list()) for lim in self._articulation.get_dof_limits(dof_indices=pos_dof_idx)
        ]
        step_max = self._get_sequence_params(sequence_index, pos_param_idx, "joint_step_max")
        step_min = self._get_sequence_params(sequence_index, pos_param_idx, "joint_step_min")
        periods = self._get_sequence_params(sequence_index, pos_param_idx, "joint_periods")
        phases = self._get_sequence_params(sequence_index, pos_param_idx, "joint_phases")

        wave_phase = np.sin(2 * np.pi * timestep / periods + phases)
        position_command = np.where(wave_phase >= 0, step_max, step_min)
        position_command = np.clip(position_command, lower_limits, upper_limits)

        # Velocity control - square wave between -v_max and +v_max
        vel_periods = self._get_sequence_params(sequence_index, vel_param_idx, "joint_periods")
        vel_phases = self._get_sequence_params(sequence_index, vel_param_idx, "joint_phases")
        max_velocities = self._articulation.get_dof_max_velocities(dof_indices=vel_dof_idx).numpy()

        vel_wave_phase = np.sin(2 * np.pi * timestep / vel_periods + vel_phases)
        velocity_command = np.where(vel_wave_phase >= 0, max_velocities, -max_velocities)

        return pos_dof_idx, position_command, vel_dof_idx, velocity_command

    def initialize_gains_test(self, test_params: dict) -> None:
        """Initialize a gains test with the given parameters.

        Args:
            test_params: Dictionary containing test configuration including
                test_mode, joint_indices, test_duration, and sequence data.
        """
        self.test_params = test_params
        self._test_duration = test_params.get("test_duration", self._test_duration)
        self._test_result_metrics = {}
        indices = self.test_params["joint_indices"]
        stiffnesses, dampings = [gains.list() for gains in self._articulation.get_dof_gains()]

        # Joints driven by a Newton actuator (backend-independent) or by an active
        # MuJoCo actuator have their DriveAPI stiffness/damping zeroed by the
        # application-side actuator runtime, so ``get_dof_gains`` reports them as
        # un-driven.  Reclassify those DOFs from their actuator / mjc gains so they
        # are commanded (position/velocity) during every test type.
        active_source_gains = self._resolve_test_active_source_gains(indices)
        self.joint_modes = classify_joint_modes(indices, stiffnesses, dampings, active_source_gains)

        self._test_timestep = 0
        self._data_ready = False
        self._gains_test_generator = self._gains_test_generator_fn()

    def _resolve_test_active_source_gains(self, tested_dof_indices: list[int]) -> dict[int, tuple[float, float]]:
        """Resolve effective actuator / mjc gains for the tested DOFs (best-effort).

        Builds the Newton-actuator and MuJoCo-native gain maps from the live stage
        and returns, for each actuator- / mjc-active DOF, the effective ``(kp, kd)``
        used to classify its control mode.  Returns an empty map (DriveAPI-only
        classification) when no such sources are authored or the stage is missing.

        Args:
            tested_dof_indices: DOF indices under test.

        Returns:
            Dict mapping DOF index -> ``(kp, kd)`` for actuator- / mjc-active DOFs.
        """
        try:
            stage = omni.usd.get_context().get_stage()
        except Exception:
            stage = None
        if stage is None or not self._joints:
            return {}
        try:
            actuator_map = build_actuator_gain_map(stage, self._articulation_root) if self._articulation_root else {}
            mjc_map = build_mjc_gain_map(stage)
            if not actuator_map and not mjc_map:
                return {}
            dof_drive_axis = {entry.dof_index: entry.drive_axis for entry in self._joint_entries}
            return resolve_active_source_gains(
                list(tested_dof_indices),
                dict(self._joints),
                actuator_map,
                mjc_map,
                mujoco_solver_active=newton_mujoco_solver_active(),
                dof_drive_axis=dof_drive_axis,
            )
        except Exception as exc:
            carb.log_warn(f"GainTuner: failed to resolve actuator/mjc gains for test classification: {exc}")
            return {}

    def _compute_gains_test_dof_error_terms(self, joint_index: int) -> tuple[float, float]:
        """Compute RMSE error terms for a single DOF.

        Args:
            joint_index: Index of the joint DOF.

        Returns:
            Tuple of (position_rmse, velocity_rmse).
        """
        if joint_index in self._joint_indices:
            remapped_index = np.argmax(self._joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_position_commands[:, remapped_index]
                        - self._observed_joint_positions[:, joint_index]
                    )
                )
            )
            vel_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_velocity_commands[:, remapped_index]
                        - self._observed_joint_velocities[:, joint_index]
                    )
                )
            )
        else:
            remapped_index = np.argmax(self._fixed_joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(self._fixed_positions[remapped_index] - self._observed_joint_positions[:, joint_index])
                )
            )
            vel_rmse = np.sqrt(np.mean(np.square(self._observed_joint_velocities[:, joint_index])))
        return pos_rmse, vel_rmse

    def compute_gains_test_error_terms(self) -> tuple[np.ndarray, np.ndarray]:
        """Compute RMSE error terms for all DOFs.

        Returns:
            Tuple of (position_rmse_array, velocity_rmse_array).
        """
        pos_rmse_list = []
        vel_rmse_list = []
        for dof_index in range(self._articulation.num_dofs):
            pos_rmse, vel_rmse = self._compute_gains_test_dof_error_terms(dof_index)
            pos_rmse_list.append(pos_rmse)
            vel_rmse_list.append(vel_rmse)
        return np.array(pos_rmse_list), np.array(vel_rmse_list)

    def update_gains_test(self, step: float) -> bool:
        """Advance the gains test by one step.

        Args:
            step: Time step size.

        Returns:
            True if test is complete, False otherwise.
        """
        try:
            self.step = step
            if self._active_test is not None:
                self._active_test._step = step
            next(self._gains_test_generator)
            self._test_timestep += step
            return False
        except StopIteration:
            self._active_test = None
            self._v_max = None
            self._T = None
            return True

    def _gains_test_generator_fn(self) -> Generator:
        """Generator function that runs the gains test sequence.

        For built-in modes (SINUSOIDAL, STEP) the original code path is
        used unchanged.  For any other mode that has a registered
        :class:`RobotTest`, execution is delegated to the test's
        ``run()`` generator and results are translated back into the
        internal arrays used by the plotting pipeline.

        Yields:
            Control back to the simulation loop after each physics step.
        """
        if self._articulation is None:
            return
        if self.test_params is None:
            carb.log_error("Attempted to run gains test without first calling initialize_test()")
            return

        test_mode = self.test_params["test_mode"]

        registered_test = self._test_registry.get(int(test_mode))
        is_builtin = test_mode in (GainsTestMode.SINUSOIDAL, GainsTestMode.STEP, GainsTestMode.USER_PROVIDED)
        if registered_test is not None and not is_builtin:
            yield from self._run_registered_test(registered_test)
            return

        step_fn = self.sinusoidal_step if test_mode == GainsTestMode.SINUSOIDAL else self.step_step

        # Initialize data storage
        self._joint_position_commands = []
        self._joint_velocity_commands = []
        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []

        num_sequences = len(self.test_params["sequence"])

        for sequence_index in range(num_sequences):
            yield from self._run_test_sequence(sequence_index, step_fn)

        self._articulation.reset_to_default_state()
        self._finalize_test_data()

    def _run_registered_test(self, test: RobotTest) -> Generator:
        """Run a registered RobotTest and translate its results.

        Args:
            test: The RobotTest instance to run.

        Yields:
            Empty tuple to yield control to simulation.
        """
        self._active_test = test
        test.setup(
            self._articulation,
            self.test_params["joint_indices"],
            self.joint_modes,
            self.test_params,
        )
        gen = test.run()
        try:
            while True:
                test._step = self.step
                next(gen)
                yield ()
        except StopIteration as e:
            result = e.value
            self._active_test = None
            if isinstance(result, TestResult):
                self._apply_test_result(result)
            else:
                self._finalize_test_data()

    def _apply_test_result(self, result: TestResult) -> None:
        """Translate a TestResult into the internal arrays used by plotting.

        Args:
            result: Structured result from a RobotTest.
        """
        self._joint_position_commands = result.joint_position_commands
        self._joint_velocity_commands = result.joint_velocity_commands
        self._observed_joint_positions = result.observed_joint_positions
        self._observed_joint_velocities = result.observed_joint_velocities
        self._command_times = result.command_times
        self._test_result_metrics = result.joint_metrics
        self._data_ready = True

    def _run_test_sequence(self, sequence_index: int, step_fn: callable) -> Generator:
        """Run a single test sequence.

        Args:
            sequence_index: Index of the sequence to run.
            step_fn: Function to generate step commands.

        Yields:
            Empty tuple to yield control to simulation.
        """
        sequence_time = 0
        self._articulation.reset_to_default_state()

        pos_idx, pos_cmd, vel_idx, vel_cmd = step_fn(sequence_time, sequence_index)
        self._articulation.set_dof_position_targets(pos_cmd, dof_indices=pos_idx)
        self._articulation.set_dof_velocity_targets(vel_cmd, dof_indices=vel_idx)

        position_targets = np.copy(self._articulation.get_dof_position_targets().numpy()[0])
        velocity_targets = np.copy(self._articulation.get_dof_velocity_targets().numpy()[0])

        self._record_test_sample(position_targets, velocity_targets)

        yield ()

        while sequence_time < self._test_duration:
            sequence_time += self.step

            pos_idx, pos_cmd, vel_idx, vel_cmd = step_fn(sequence_time, sequence_index)
            self._articulation.set_dof_position_targets(pos_cmd, dof_indices=pos_idx)
            self._articulation.set_dof_velocity_targets(vel_cmd, dof_indices=vel_idx)

            position_targets[pos_idx] = pos_cmd
            velocity_targets[vel_idx] = vel_cmd

            self._joint_position_commands.append(np.copy(position_targets))
            self._joint_velocity_commands.append(np.copy(velocity_targets))
            self._command_times.append(self._test_timestep)

            yield ()

            self._observed_joint_positions.append(self._articulation.get_dof_positions().numpy()[0].copy())
            self._observed_joint_velocities.append(self._articulation.get_dof_velocities().numpy()[0].copy())

    def _record_test_sample(self, position_targets: np.ndarray, velocity_targets: np.ndarray) -> None:
        """Record a single test sample.

        Args:
            position_targets: Current position targets.
            velocity_targets: Current velocity targets.
        """
        self._joint_position_commands.append(np.copy(position_targets))
        self._joint_velocity_commands.append(np.copy(velocity_targets))
        self._observed_joint_positions.append(self._articulation.get_dof_positions().numpy()[0].copy())
        self._observed_joint_velocities.append(self._articulation.get_dof_velocities().numpy()[0].copy())
        self._command_times.append(self._test_timestep)

    def _finalize_test_data(self) -> None:
        """Convert test data lists to numpy arrays and mark data as ready."""
        self._joint_position_commands = np.array(self._joint_position_commands)
        self._joint_velocity_commands = np.array(self._joint_velocity_commands)
        self._observed_joint_positions = np.array(self._observed_joint_positions)
        self._observed_joint_velocities = np.array(self._observed_joint_velocities)
        self._command_times = np.array(self._command_times)
        self._data_ready = True

    # ======================== For Plotting ========================

    def get_joint_states_from_gains_test(
        self, joint_index: int
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get recorded joint states from the last gains test.

        Args:
            joint_index: Index of the joint to retrieve data for.

        Returns:
            Tuple of (position_commands, velocity_commands, observed_positions,
                     observed_velocities, timestamps). All values are None if
                     no data is available for the specified joint.
        """
        empty_result = (None, None, None, None, None)

        if len(self._observed_joint_positions) == 0:
            return empty_result

        if joint_index >= self._joint_position_commands.shape[1]:
            return empty_result

        return (
            self._joint_position_commands[:, joint_index],
            self._joint_velocity_commands[:, joint_index],
            self._observed_joint_positions[:, joint_index],
            self._observed_joint_velocities[:, joint_index],
            self._command_times,
        )
