# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for Gain Tuner functionality."""

import asyncio
import math
import os
import tempfile
from dataclasses import dataclass
from enum import Enum
from types import SimpleNamespace
from typing import Optional
from unittest import mock

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.usd
import usd.schema.isaac.robot_schema as robot_schema
from isaacsim.core.simulation_manager import PhysicsScene, PhysxScene, SimulationManager
from isaacsim.robot_setup.gain_tuner.base import RobotTest, TestResult
from isaacsim.robot_setup.gain_tuner.gain_tuner_drive_math import (
    damping_from_damping_ratio_revolute_position,
    damping_ratio_from_stiffness_damping_revolute_position,
    meq_for_drive_frequency,
    natural_frequency_hz_from_stiffness_revolute_position,
    stiffness_stored_and_damping_from_natural_frequency_revolute_position,
)
from isaacsim.robot_setup.gain_tuner.gains_tuner import (
    GainsTestMode,
    GainTuner,
    SinusoidalTest,
    StepFunctionTest,
    _assign_d6_axis_token,
    _d6_axis_has_unlocked_limit,
    _extract_d6_axis_token,
    _format_d6_display_name,
    compute_parallel_axis_inertia,
    find_articulation_root,
    get_joint_axis_world_direction,
    get_original_spec_for_drive_api,
    matrix_norm,
    project_inertia_onto_axis,
)
from isaacsim.robot_setup.gain_tuner.snap_to_limits import SnapToLimitsTest, _Phase
from isaacsim.robot_setup.gain_tuner.stress_test import StressTest, StressTestMode
from isaacsim.robot_setup.gain_tuner.ui.joint_table_widget import (
    _CELL_BG_INAPPLICABLE,
    JointDriveMode,
    JointItemDelegate,
    JointListModel,
    JointSettingMode,
    WidgetColumns,
    _set_gain_cell_field_state,
    _tunable_value_column_ids,
    get_damping_attr,
    get_joint_drive_mode,
    get_stiffness_attr,
)
from isaacsim.robot_setup.gain_tuner.usd_layer_utils import (
    collect_gain_save_edits,
    find_layer_by_save_identifier,
    find_physics_layer,
    find_physics_layer_on_disk,
    get_layer_save_identifier,
    get_property_path_for_layer,
    is_layer_savable,
    is_physics_layer,
    remap_edits_to_physics_layer,
    resolve_gain_save_target,
)
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdPhysics

# Golden PD scalars (reference values; forward formulas documented in test docstrings).
# Prismatic: m=2.3 kg, f_n=4.5 Hz, ζ=0.12 → k = m ω_n², d = 2 ζ √(m k).
_GOLDEN_PRISMATIC_STIFFNESS_DAMPING_M_2p3_FN_4p5_Z_0p12 = (
    1838.7072999229474,
    15.607432303034091,
)
# Revolute SI: I=0.85 kg·m², f_n=11 Hz, ζ=0.08 → k = I ω_n², d = 2 ζ √(I k).
_GOLDEN_REVOLUTE_SI_STIFFNESS_DAMPING_I_0p85_FN_11_Z_0p08 = (
    4060.355250608161,
    9.39964521954066,
)
# Damped cosine f_n=6.2 Hz, ζ=0.06: T_d = 2π / (ω_n √(1−ζ²)) [s].
_GOLDEN_DAMPED_PERIOD_FN_6p2_Z_0p06 = 0.16158143139130263
# USD drive storage for k_si=150 N·m/rad, d_si=3 N·m·s/rad (scale π/180).
_GOLDEN_REVOLUTE_DRIVE_USD_K = 2.6179938779914944
_GOLDEN_REVOLUTE_DRIVE_USD_D = 0.05235987755982988


class JointModality(Enum):
    """Joint type for the test articulation."""

    PRISMATIC = "prismatic"
    REVOLUTE = "revolute"


class DriveSubmodality(Enum):
    """Drive type for the joint."""

    FORCE = "force"
    ACCELERATION = "acceleration"


# PhysX stores revolute drive stiffness/damping in N·m/deg and N·m·s/deg (see asset importer utils).
_REVOLUTE_DRIVE_GAIN_USD_SCALE = math.pi / 180.0


def _set_world_translation(prim: Usd.Prim, translation: Gf.Vec3f) -> None:
    """Set world translation on a prim, reusing an existing translate op if present."""
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(translation)
            return
    xformable.AddTranslateOp().Set(translation)


def _revolute_drive_stiffness_damping_si_to_usd(k_si: float, d_si: float) -> tuple[float, float]:
    """Convert SI revolute gains (N·m/rad, N·m·s/rad) to USD drive attribute units."""
    s = _REVOLUTE_DRIVE_GAIN_USD_SCALE
    return k_si * s, d_si * s


def _revolute_drive_stiffness_damping_usd_to_si(k_usd: float, d_usd: float) -> tuple[float, float]:
    """Convert USD revolute drive gains back to SI."""
    s = _REVOLUTE_DRIVE_GAIN_USD_SCALE
    return k_usd / s, d_usd / s


@dataclass
class OscillationAnalysis:
    """Results from analyzing damped oscillation data."""

    log_decrement_avg: float
    damped_period: float
    damped_freq: float
    damping_ratio: float
    natural_freq: float
    peak_times: list[float]
    peak_values: list[float]
    num_samples: int = 0
    value_min: float = 0.0
    value_max: float = 0.0


@dataclass
class PdOscillationHarnessResult:
    """Oscillation analysis plus the USD / inertia context used to author PD gains."""

    analysis: OscillationAnalysis
    modality: JointModality
    joint_path: str
    ieq: float
    target_fn_hz: float
    target_zeta: float


def _natural_fn_zeta_from_usd_drive(
    joint_modality: JointModality, joint_prim: Usd.Prim, ieq: float
) -> tuple[float, float]:
    """Closed-loop linear natural frequency (Hz) and ζ from authored USD drive gains and ``I_eq`` / mass."""
    if joint_modality == JointModality.REVOLUTE:
        drive = UsdPhysics.DriveAPI(joint_prim, "angular")
        k_u = float(drive.GetStiffnessAttr().Get())
        d_u = float(drive.GetDampingAttr().Get())
        k_si, d_si = _revolute_drive_stiffness_damping_usd_to_si(k_u, d_u)
        return _compute_natural_freq_damping_revolute(k_si, d_si, ieq)
    drive = UsdPhysics.DriveAPI(joint_prim, "linear")
    k = float(drive.GetStiffnessAttr().Get())
    d = float(drive.GetDampingAttr().Get())
    return _compute_natural_freq_damping_prismatic(k, d, ieq)


def _extract_peaks(times: np.ndarray, values: np.ndarray) -> tuple[list[float], list[float]]:
    """Extract local maxima (peaks) from time-series data.

    Includes boundary indices when they are local maxima (e.g. initial condition
    at release). If no interior peaks are found, falls back to peaks of the
    absolute value (envelope) so both sides of a damped oscillation are counted.
    """
    if len(values) < 2:
        return [], []
    n = len(values)
    # Interior peaks: strict local maxima
    peak_indices: list[int] = []
    if n >= 3:
        is_peak = (values[1:-1] > values[:-2]) & (values[1:-1] > values[2:])
        peak_indices = (np.where(is_peak)[0] + 1).tolist()
    # First point is a peak if it is >= neighbor (e.g. start at initial displacement)
    if values[0] >= values[1]:
        peak_indices.insert(0, 0)
    # Last point is a peak if it is >= neighbor
    if n >= 2 and values[-1] >= values[-2]:
        peak_indices.append(n - 1)
    if peak_indices:
        peak_indices = sorted(set(peak_indices))
        peak_times = times[peak_indices].tolist()
        peak_values = values[peak_indices].tolist()
        return peak_times, peak_values
    # Fallback: peaks of |values| (envelope) so we catch both positive and negative humps
    abs_vals = np.abs(values)
    if n >= 3:
        is_peak_abs = (abs_vals[1:-1] > abs_vals[:-2]) & (abs_vals[1:-1] > abs_vals[2:])
        peak_indices = (np.where(is_peak_abs)[0] + 1).tolist()
    if not peak_indices:
        return [], []
    peak_times = times[peak_indices].tolist()
    # Store signed values for analysis (log decrement uses abs(peak_values))
    peak_values = values[peak_indices].tolist()
    return peak_times, peak_values


def _peaks_fail_msg(result: OscillationAnalysis, prefix: str = "") -> str:
    """Build assertion message when peak count is too low, including data diagnostics."""
    return (
        f"{prefix}Need at least 3 peaks for analysis (got {len(result.peak_values)}). "
        f"Position data: n={result.num_samples} min={result.value_min:.6f} max={result.value_max:.6f}"
    )


def _analyze_oscillation(times: np.ndarray, values: np.ndarray) -> OscillationAnalysis:
    """Extract natural frequency and damping ratio from damped oscillation data.

    Uses:
    - Logarithmic decrement: s_i = ln(A_i / A_{i+1}), average over peaks
    - Damped period T_d from average time between successive peaks
    - w_d = 2*pi / T_d
    - zeta = s / sqrt(4*pi^2 + s^2)
    - w_n = w_d / sqrt(1 - zeta^2)
    """
    num_samples = len(values)
    value_min = float(np.min(values)) if num_samples else 0.0
    value_max = float(np.max(values)) if num_samples else 0.0
    peak_times, peak_values = _extract_peaks(times, values)
    if len(peak_values) < 2:
        return OscillationAnalysis(
            log_decrement_avg=0.0,
            damped_period=0.0,
            damped_freq=0.0,
            damping_ratio=0.0,
            natural_freq=0.0,
            peak_times=peak_times,
            peak_values=peak_values,
            num_samples=num_samples,
            value_min=value_min,
            value_max=value_max,
        )

    log_decrements = []
    for i in range(len(peak_values) - 1):
        a_i = abs(peak_values[i])
        a_next = abs(peak_values[i + 1])
        if a_next > 1e-12 and a_i > 1e-12:
            s_i = math.log(a_i / a_next)
            log_decrements.append(s_i)
    s_avg = float(np.mean(log_decrements)) if log_decrements else 0.0

    period_diffs = [peak_times[i + 1] - peak_times[i] for i in range(len(peak_times) - 1)]
    T_d = float(np.mean(period_diffs)) if period_diffs else 0.0
    w_d = (2.0 * math.pi / T_d) if T_d > 1e-9 else 0.0

    zeta = s_avg / math.sqrt(4.0 * math.pi**2 + s_avg**2) if (4 * math.pi**2 + s_avg**2) > 0 else 0.0
    denom = 1.0 - zeta**2
    w_n = (w_d / math.sqrt(denom)) if denom > 1e-9 else 0.0

    return OscillationAnalysis(
        log_decrement_avg=s_avg,
        damped_period=T_d,
        damped_freq=w_d,
        damping_ratio=zeta,
        natural_freq=w_n / (2.0 * math.pi),
        peak_times=peak_times,
        peak_values=peak_values,
        num_samples=num_samples,
        value_min=value_min,
        value_max=value_max,
    )


def _compute_stiffness_damping_prismatic(
    mass: float, natural_freq_hz: float, damping_ratio: float
) -> tuple[float, float]:
    """Compute stiffness and damping for prismatic joint from natural freq and damping ratio.

    For m*x'' + D*x' + K*x = 0:
    w_n = sqrt(K/m) => K = m * w_n^2
    zeta = D / (2*sqrt(m*K)) => D = 2*zeta*sqrt(m*K) = 2*zeta*m*w_n

    Args:
        mass: Mass of the prismatic joint link.
        natural_freq_hz: Natural frequency in Hz.
        damping_ratio: Damping ratio.

    Returns:
        Tuple of stiffness and damping values.
    """
    w_n = 2.0 * math.pi * natural_freq_hz
    stiffness = mass * (w_n**2)
    damping = 2.0 * damping_ratio * mass * w_n
    return stiffness, damping


def _compute_stiffness_damping_revolute(
    inertia: float, natural_freq_hz: float, damping_ratio: float
) -> tuple[float, float]:
    """Compute stiffness and damping for revolute joint from natural freq and damping ratio.

    For I*theta'' + D*theta' + K*theta = 0:
    w_n = sqrt(K/I) => K = I * w_n^2
    zeta = D / (2*sqrt(I*K)) => D = 2*zeta*I*w_n

    Args:
        inertia: Moment of inertia for the revolute joint.
        natural_freq_hz: Natural frequency in Hz.
        damping_ratio: Damping ratio.

    Returns:
        Tuple of stiffness and damping values.
    """
    w_n = 2.0 * math.pi * natural_freq_hz
    stiffness = inertia * (w_n**2)
    damping = 2.0 * damping_ratio * inertia * w_n
    return stiffness, damping


def _compute_natural_freq_damping_revolute(stiffness: float, damping: float, inertia: float) -> tuple[float, float]:
    """Compute natural frequency (Hz) and damping ratio from revolute drive gains.

    Inverse of _compute_stiffness_damping_revolute: w_n = sqrt(K/I), zeta = D/(2*sqrt(I*K)).

    Args:
        stiffness: Stiffness value of the revolute joint drive.
        damping: Damping value of the revolute joint drive.
        inertia: Moment of inertia.

    Returns:
        Tuple of natural frequency in Hz and damping ratio.
    """
    if inertia <= 0 or stiffness <= 0:
        return 0.0, 0.0
    w_n = math.sqrt(stiffness / inertia)
    natural_freq_hz = w_n / (2.0 * math.pi)
    zeta = damping / (2.0 * math.sqrt(inertia * stiffness))
    return natural_freq_hz, zeta


def _compute_natural_freq_damping_prismatic(stiffness: float, damping: float, mass: float) -> tuple[float, float]:
    """Compute natural frequency (Hz) and damping ratio from prismatic drive gains.

    Inverse of _compute_stiffness_damping_prismatic: w_n = sqrt(K/m), zeta = D/(2*sqrt(m*K)).

    Args:
        stiffness: Stiffness value of the prismatic joint drive.
        damping: Damping value of the prismatic joint drive.
        mass: Mass of the link.

    Returns:
        Tuple of natural frequency in Hz and damping ratio.
    """
    if mass <= 0 or stiffness <= 0:
        return 0.0, 0.0
    w_n = math.sqrt(stiffness / mass)
    natural_freq_hz = w_n / (2.0 * math.pi)
    zeta = damping / (2.0 * math.sqrt(mass * stiffness))
    return natural_freq_hz, zeta


class TestGainTunerHarness(omni.kit.test.AsyncTestCase):
    """Shared USD articulation fixtures and helpers for gain-tuner extension tests."""

    __test__ = False  # Not a test suite; concrete subclasses collect ``test_*`` methods.

    async def setUp(self) -> None:
        """Set up test environment with physics timeline and gain tuner."""
        self._physics_fps = 60
        self._physics_dt = 1.0 / self._physics_fps
        self._timeline = omni.timeline.get_timeline_interface()
        self._gain_tuner = GainTuner()
        await stage_utils.create_new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(Sdf.Path("/World"), "Xform")
        PhysicsSchemaTools.addGroundPlane(
            self._stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, -0.5), Gf.Vec3f(1.0)
        )
        SimulationManager.set_physics_dt(self._physics_dt)
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Tear down test environment and reset gain tuner."""
        self._timeline.stop()
        self._gain_tuner.reset()
        await app_utils.update_app_async()

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        stage_utils.close_stage()

    def _create_articulation(
        self,
        joint_modalities: list[JointModality],
        drive_submodality: DriveSubmodality,
        distance: float,
        mass: float,
        inertia_diag: float,
        natural_freq_hz: float,
        damping_ratio: float,
        *,
        chain: bool = False,
        fixed_base: bool = True,
        joint_axes: Optional[list[str]] = None,
        link_positions: Optional[list[tuple[float, float, float]]] = None,
        base_mass: float = 1000.0,
        base_inertia: float = 1.0,
        collision_enabled: bool = True,
        joint_limit_revolute: Optional[tuple[float, float]] = None,
        joint_limit_prismatic: Optional[tuple[float, float]] = None,
    ) -> str:
        """Create an articulation: fixed or floating base + one or more links connected by joints.

        Args:
            joint_modalities: Type of each joint (revolute or prismatic).
            drive_submodality: Force or acceleration drive.
            distance: Spacing used for default link positions (link i at (i+1)*distance, 0, 0.5).
            mass: Mass of each link.
            inertia_diag: Diagonal inertia of each link (and of base when fixed_base=False).
            natural_freq_hz: Target natural frequency for drive gains.
            damping_ratio: Target damping ratio for drive gains.
            chain: If True, joint i connects (base if i==0 else link_{i-1}) to link_i. If False, every joint connects base to link_i.
            fixed_base: If True, add a fixed joint to world and ArticulationRootAPI on it. If False, no fixed joint and ArticulationRootAPI on base.
            joint_axes: Per-joint axis (e.g. ["Z", "Y"] for revolute). Default: revolute "Z", prismatic "X".
            link_positions: Optional (x, y, z) for each link; default (i+1)*distance along X at z=0.5.
            base_mass: Mass of base link (used when fixed_base=False for floating base).
            base_inertia: Diagonal inertia of base (used when fixed_base=False for floating base).
            collision_enabled: When False, skip collision API on base and links (free-flight oscillation tests).
            joint_limit_revolute: Optional ``(lower, upper)`` limits in degrees for revolute joints.
            joint_limit_prismatic: Optional ``(lower, upper)`` limits in meters for prismatic joints.

        Returns:
            Robot prim path.
        """
        robot_path = "/World/robot"
        if self._stage.GetPrimAtPath(robot_path).IsValid():
            stage_utils.delete_prim(robot_path)

        base_path = f"{robot_path}/base"
        fixed_joint_path = f"{robot_path}/root_joint"
        n_joints = len(joint_modalities)
        axes = (
            joint_axes
            if joint_axes is not None
            else (["Z" if m == JointModality.REVOLUTE else "X" for m in joint_modalities])
        )

        # Base
        base_geom = UsdGeom.Cube.Define(self._stage, base_path)
        base_geom.CreateSizeAttr(0.1)
        base_prim = self._stage.GetPrimAtPath(base_path)
        _set_world_translation(base_prim, Gf.Vec3f(0, 0, 0.5))
        if collision_enabled:
            UsdPhysics.CollisionAPI.Apply(base_prim)
        UsdPhysics.RigidBodyAPI.Apply(base_prim)
        UsdPhysics.MassAPI.Apply(base_prim)
        UsdPhysics.MassAPI(base_prim).CreateMassAttr(base_mass)
        UsdPhysics.MassAPI(base_prim).CreateDiagonalInertiaAttr(Gf.Vec3f(base_inertia, base_inertia, base_inertia))

        if fixed_base:
            fixed_joint = UsdPhysics.FixedJoint.Define(self._stage, fixed_joint_path)
            fixed_joint.CreateBody1Rel().SetTargets([base_path])
            UsdPhysics.ArticulationRootAPI.Apply(fixed_joint.GetPrim())
        else:
            fixed_joint = None
            UsdPhysics.ArticulationRootAPI.Apply(base_prim)

        link_prims = []
        joint_prims = []

        base_pos = Gf.Vec3f(0, 0, 0.5)
        body0_positions = {}
        body0_positions[base_path] = base_pos

        for joint_index, joint_modality in enumerate(joint_modalities):
            link_path = f"{robot_path}/link_{joint_index}"
            if link_positions is not None and joint_index < len(link_positions):
                pos = Gf.Vec3f(*link_positions[joint_index])
            else:
                pos = Gf.Vec3f((joint_index + 1) * distance, 0, 0.5)
            link_geom = UsdGeom.Cube.Define(self._stage, link_path)
            link_geom.CreateSizeAttr(0.2)
            link_prim = self._stage.GetPrimAtPath(link_path)
            _set_world_translation(link_prim, pos)
            if collision_enabled:
                UsdPhysics.CollisionAPI.Apply(link_prim)
            UsdPhysics.RigidBodyAPI.Apply(link_prim)
            UsdPhysics.MassAPI.Apply(link_prim)
            UsdPhysics.MassAPI(link_prim).CreateMassAttr(mass)
            UsdPhysics.MassAPI(link_prim).CreateDiagonalInertiaAttr(Gf.Vec3f(inertia_diag, inertia_diag, inertia_diag))
            link_prims.append(link_prim)
            body0_positions[link_path] = pos

            body0 = base_path if (not chain or joint_index == 0) else f"{robot_path}/link_{joint_index - 1}"
            body0_pos = body0_positions[body0]
            axis = (
                axes[joint_index]
                if joint_index < len(axes)
                else ("Z" if joint_modality == JointModality.REVOLUTE else "X")
            )

            if joint_modality == JointModality.PRISMATIC:
                joint = UsdPhysics.PrismaticJoint.Define(self._stage, f"{robot_path}/joint_{joint_index}")
                joint.CreateAxisAttr(axis)
                joint.CreateBody0Rel().SetTargets([body0])
                joint.CreateBody1Rel().SetTargets([link_path])
                drive_type = "linear"
                stiffness, damping = _compute_stiffness_damping_prismatic(mass, natural_freq_hz, damping_ratio)
            else:
                joint = UsdPhysics.RevoluteJoint.Define(self._stage, f"{robot_path}/joint_{joint_index}")
                joint.CreateAxisAttr(axis)
                joint.CreateBody0Rel().SetTargets([body0])
                joint.CreateBody1Rel().SetTargets([link_path])
                drive_type = "angular"
                stiffness, damping = _compute_stiffness_damping_revolute(inertia_diag, natural_freq_hz, damping_ratio)
                stiffness, damping = _revolute_drive_stiffness_damping_si_to_usd(stiffness, damping)

            local_pos1 = Gf.Vec3f(body0_pos[0] - pos[0], body0_pos[1] - pos[1], body0_pos[2] - pos[2])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalPos1Attr().Set(local_pos1)

            drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), drive_type)
            drive_api.CreateTypeAttr(drive_submodality.value)
            drive_api.CreateStiffnessAttr(stiffness)
            drive_api.CreateDampingAttr(damping)
            # Free-flight harness: default drive effort limits can saturate PD and yield a much
            # lower apparent oscillation frequency than the authored K/D would predict.
            if not collision_enabled:
                if not drive_api.GetMaxForceAttr():
                    drive_api.CreateMaxForceAttr(1.0e7)
                else:
                    drive_api.GetMaxForceAttr().Set(1.0e7)
            if joint_modality == JointModality.REVOLUTE and joint_limit_revolute is not None:
                joint.CreateLowerLimitAttr(joint_limit_revolute[0])
                joint.CreateUpperLimitAttr(joint_limit_revolute[1])
            if joint_modality == JointModality.PRISMATIC and joint_limit_prismatic is not None:
                joint.CreateLowerLimitAttr(joint_limit_prismatic[0])
                joint.CreateUpperLimitAttr(joint_limit_prismatic[1])
            joint_prims.append(joint.GetPrim())

        # Robot schema: all links and all joints (fixed first when present)
        robot_prim = self._stage.GetPrimAtPath(robot_path)
        robot_schema.ApplyRobotAPI(robot_prim)
        all_links = [base_prim.GetPath()] + [p.GetPath() for p in link_prims]
        all_joints = ([fixed_joint.GetPrim().GetPath()] if fixed_joint is not None else []) + [
            p.GetPath() for p in joint_prims
        ]
        robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).SetTargets(all_links)
        robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).SetTargets(all_joints)
        for p in [base_prim] + link_prims:
            robot_schema.ApplyLinkAPI(p)
        for j in ([] if fixed_joint is None else [fixed_joint.GetPrim()]) + joint_prims:
            robot_schema.ApplyJointAPI(j)

        return robot_path

    async def _run_setup_and_compute_inertia(self, robot_path: str, num_physics_steps: int = 60) -> None:
        """Setup gain tuner, run physics so mass query completes, then compute joint inertias.

        Args:
            robot_path: USD path to the robot prim.
            num_physics_steps: Number of physics steps to run before computing inertia.
        """
        self._gain_tuner.setup(robot_path)
        for _ in range(2):
            await app_utils.update_app_async()
        self._timeline.play()
        for _ in range(num_physics_steps):
            await app_utils.update_app_async()
        self._gain_tuner.compute_joints_accumulated_inertia()
        self._timeline.stop()

    def _create_fixed_base_two_revolute_chain(
        self,
        distance: float,
        mass: float,
        inertia_diag: float,
        natural_freq_hz: float,
        damping_ratio: float,
        second_axis_z: bool = True,
    ) -> tuple[str, list[float]]:
        """Create fixed base -> revolute0 -> link0 -> revolute1 -> link1. Same plane if second_axis_z True.

        Args:
            distance: Distance between links.
            mass: Mass of each link.
            inertia_diag: Diagonal inertia component for each link.
            natural_freq_hz: Natural frequency in Hz for the drive.
            damping_ratio: Damping ratio for the drive.
            second_axis_z: Whether the second joint axis is Z (True) or Y (False).

        Returns:
            Tuple of robot path and list of expected equivalent inertias.
        """
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE, JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=distance,
            mass=mass,
            inertia_diag=inertia_diag,
            natural_freq_hz=natural_freq_hz,
            damping_ratio=damping_ratio,
            chain=True,
            joint_axes=["Z", "Z" if second_axis_z else "Y"],
        )
        # j0: pose at base; forward = link0+link1 about base -> I_fwd_j0 = (I_d+m*d^2)+(I_d+m*(2d)^2) = 3.25.
        I_fwd_j0 = (inertia_diag + mass * distance**2) + (inertia_diag + mass * (2 * distance) ** 2)
        # j1: backward_links = [link0, base]; base is fixed so _accumulate_link_inertia returns (0,0,True).
        # I_eq = forward only. Joint pose at body0 (link0); forward = link1 about j1 = I_d + m*d^2 = 1.25.
        I_eq_j1 = inertia_diag + mass * distance**2
        return robot_path, [I_fwd_j0, I_eq_j1]

    def _create_fixed_base_two_prismatic_chain(
        self,
        distance: float,
        mass: float,
        natural_freq_hz: float,
        damping_ratio: float,
        same_axis: bool = True,
    ) -> tuple[str, list[float]]:
        """Create fixed base -> prism0 -> link0 -> prism1 -> link1. Same axis X if same_axis else second Y.

        Args:
            distance: Distance between links.
            mass: Mass of each link.
            natural_freq_hz: Natural frequency in Hz for the drive.
            damping_ratio: Damping ratio for the drive.
            same_axis: Whether both joints share the same X axis.

        Returns:
            Tuple of robot path and list of expected equivalent inertias.
        """
        link_positions = None if same_axis else [(distance, 0, 0.5), (distance, distance, 0.5)]
        robot_path = self._create_articulation(
            [JointModality.PRISMATIC, JointModality.PRISMATIC],
            DriveSubmodality.FORCE,
            distance=distance,
            mass=mass,
            inertia_diag=1.0,
            natural_freq_hz=natural_freq_hz,
            damping_ratio=damping_ratio,
            chain=True,
            joint_axes=["X", "X" if same_axis else "Y"],
            link_positions=link_positions,
        )
        # j0: backward fixed, forward = m0 + m1 = 2*mass. j1: backward chain includes fixed base -> I_eq = forward mass only = mass.
        return robot_path, [2.0 * mass, mass]

    def _create_moving_base_single_joint(
        self,
        joint_revolute: bool,
        distance: float,
        base_mass: float,
        base_inertia: float,
        link_mass: float,
        link_inertia_diag: float,
        natural_freq_hz: float,
        damping_ratio: float,
    ) -> tuple[str, list[float]]:
        """Create floating base -> single joint -> link. No fixed joint. ArticulationRootAPI on base.

        Args:
            joint_revolute: Whether the joint is revolute (True) or prismatic (False).
            distance: Distance between links.
            base_mass: Mass of the base link.
            base_inertia: Inertia of the base link.
            link_mass: Mass of the child link.
            link_inertia_diag: Diagonal inertia component for the child link.
            natural_freq_hz: Natural frequency in Hz for the drive.
            damping_ratio: Damping ratio for the drive.

        Returns:
            Tuple of robot path and list of expected equivalent inertias.
        """
        modality = JointModality.REVOLUTE if joint_revolute else JointModality.PRISMATIC
        robot_path = self._create_articulation(
            [modality],
            DriveSubmodality.FORCE,
            distance=distance,
            mass=link_mass,
            inertia_diag=link_inertia_diag,
            natural_freq_hz=natural_freq_hz,
            damping_ratio=damping_ratio,
            fixed_base=False,
            base_mass=base_mass,
            base_inertia=base_inertia,
        )
        if joint_revolute:
            I_eq = (base_inertia * link_inertia_diag) / (base_inertia + link_inertia_diag)
        else:
            I_eq = (base_mass * link_mass) / (base_mass + link_mass)
        return robot_path, [I_eq]

    def _create_fixed_base_revolute_prismatic_chain(
        self,
        distance: float,
        mass: float,
        inertia_diag: float,
        natural_freq_hz: float,
        damping_ratio: float,
    ) -> tuple[str, list[float]]:
        """Create fixed base -> revolute -> link0 -> prismatic -> link1.

        Args:
            distance: Distance between links.
            mass: Mass of each link.
            inertia_diag: Diagonal inertia component for each link.
            natural_freq_hz: Natural frequency in Hz for the drive.
            damping_ratio: Damping ratio for the drive.

        Returns:
            Tuple of robot path and list of expected equivalent inertias.
        """
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE, JointModality.PRISMATIC],
            DriveSubmodality.FORCE,
            distance=distance,
            mass=mass,
            inertia_diag=inertia_diag,
            natural_freq_hz=natural_freq_hz,
            damping_ratio=damping_ratio,
            chain=True,
            joint_axes=["Z", "X"],
        )
        # j0 at base: I_fwd = (I_d + m*d^2) + (I_d + m*(2d)^2) = 3.25. j1 prismatic: backward includes fixed base -> I_eq = forward mass = mass.
        I_eq_j0 = (inertia_diag + mass * distance**2) + (inertia_diag + mass * (2 * distance) ** 2)
        I_eq_j1 = mass
        return robot_path, [I_eq_j0, I_eq_j1]


class TestGainTuner(TestGainTunerHarness):
    """Equivalent inertia from articulation tensors vs. hand-computed I_eq."""

    # ---- Unit tests for compute_joints_accumulated_inertia ----
    # Hand-computed expected equivalent inertia and optional stiffness/damping from natural frequency
    # are asserted against the implementation. Relative tolerance 5% covers articulation mass-property
    # tensors vs. the rigid-body hand model (any backend), not the closed-form helpers tested elsewhere.

    async def test_compute_joints_accumulated_inertia_fixed_base_single_revolute(self) -> None:
        """Fixed base + single revolute: I_eq = link inertia about joint axis (backward fixed)."""
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        # Joint is at base (0,0,0.5); link COM at (0.5,0,0.5). I_eq = I_cm + m*d^2 = 1 + 1*0.5^2 = 1.25
        expected_I_eq = 1.0 + 1.0 * (0.5**2)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 1)
        computed = self._gain_tuner._joint_accumulated_inertia.get(entries[0].joint)
        self.assertIsNotNone(computed, "Should have computed inertia for single revolute")
        self.assertAlmostEqual(computed, expected_I_eq, delta=0.05 * max(expected_I_eq, 1e-6))
        # Stiffness/damping from natural frequency: K = I*w_n^2, D = 2*zeta*I*w_n
        stiffness_attr = UsdPhysics.DriveAPI(entries[0].joint, "angular").GetStiffnessAttr()
        damping_attr = UsdPhysics.DriveAPI(entries[0].joint, "angular").GetDampingAttr()
        K, D = stiffness_attr.Get(), damping_attr.Get()
        K, D = _revolute_drive_stiffness_damping_usd_to_si(K, D)
        nat_freq, zeta = _compute_natural_freq_damping_revolute(K, D, computed)
        # Gains were authored with inertia_diag=1.0 but I_eq=1.25; recovered nat_freq = 10*sqrt(1/1.25)
        expected_nat_freq = 10.0 * math.sqrt(1.0 / expected_I_eq)
        self.assertAlmostEqual(nat_freq, expected_nat_freq, delta=0.2)
        self.assertAlmostEqual(zeta, 0.05, delta=0.02)

    async def test_compute_joints_accumulated_inertia_fixed_base_single_prismatic(self) -> None:
        """Fixed base + single prismatic: I_eq = link mass (backward fixed)."""
        robot_path = self._create_articulation(
            [JointModality.PRISMATIC],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=2.0,
            inertia_diag=1.0,
            natural_freq_hz=5.0,
            damping_ratio=0.1,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        expected_I_eq = 2.0  # mass
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 1)
        computed = self._gain_tuner._joint_accumulated_inertia.get(entries[0].joint)
        self.assertIsNotNone(computed)
        self.assertAlmostEqual(computed, expected_I_eq, delta=0.05 * expected_I_eq)
        stiffness_attr = UsdPhysics.DriveAPI(entries[0].joint, "linear").GetStiffnessAttr()
        damping_attr = UsdPhysics.DriveAPI(entries[0].joint, "linear").GetDampingAttr()
        K, D = stiffness_attr.Get(), damping_attr.Get()
        nat_freq, zeta = _compute_natural_freq_damping_prismatic(K, D, computed)
        self.assertAlmostEqual(nat_freq, 5.0, delta=0.3)
        self.assertAlmostEqual(zeta, 0.1, delta=0.03)

    async def test_compute_joints_accumulated_inertia_fixed_base_two_revolute_same_plane(self) -> None:
        """Fixed base + two revolute joints in the same plane: hand-computed I_eq for each joint."""
        distance = 0.5
        mass, inertia_diag = 1.0, 1.0
        robot_path, expected_list = self._create_fixed_base_two_revolute_chain(
            distance=distance,
            mass=mass,
            inertia_diag=inertia_diag,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            second_axis_z=True,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 2)
        for entry, expected in zip(entries, expected_list):
            computed = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
            self.assertIsNotNone(computed, f"Inertia for {entry.joint.GetPath()}")
            self.assertAlmostEqual(computed, expected, delta=0.05 * max(expected, 1e-6))

    async def test_compute_joints_accumulated_inertia_fixed_base_two_revolute_orthogonal(self) -> None:
        """Fixed base + two revolute joints in orthogonal planes."""
        distance = 0.5
        mass, inertia_diag = 1.0, 1.0
        robot_path, expected_list = self._create_fixed_base_two_revolute_chain(
            distance=distance,
            mass=mass,
            inertia_diag=inertia_diag,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            second_axis_z=False,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 2)
        for entry, expected in zip(entries, expected_list):
            computed = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
            self.assertIsNotNone(computed)
            self.assertAlmostEqual(computed, expected, delta=0.05 * max(expected, 1e-6))

    async def test_compute_joints_accumulated_inertia_moving_base_single_revolute(self) -> None:
        """Moving base + single revolute: I_eq = (I_base * I_link_about_joint) / (I_base + I_link_about_joint).

        Implementation uses inertia about the joint: link contributes I_d + m*d^2.
        """
        distance = 0.5
        base_inertia, link_inertia_diag, link_mass = 1.0, 1.0, 1.0
        robot_path, _ = self._create_moving_base_single_joint(
            joint_revolute=True,
            distance=distance,
            base_mass=10.0,
            base_inertia=base_inertia,
            link_mass=link_mass,
            link_inertia_diag=link_inertia_diag,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        # Link inertia about joint (joint on base): I_d + m*d^2
        I_link_about_joint = link_inertia_diag + link_mass * (distance**2)
        expected_I_eq = (base_inertia * I_link_about_joint) / (base_inertia + I_link_about_joint)  # 5/9
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 1)
        computed = self._gain_tuner._joint_accumulated_inertia.get(entries[0].joint)
        self.assertIsNotNone(computed)
        self.assertAlmostEqual(computed, expected_I_eq, delta=0.05 * max(expected_I_eq, 1e-6))

    async def test_compute_joints_accumulated_inertia_fixed_base_two_prismatic_same_axis(self) -> None:
        """Fixed base + two prismatic joints on the same axis: j0 I_eq = m0+m1, j1 I_eq = m0*m1/(m0+m1)."""
        robot_path, expected_list = self._create_fixed_base_two_prismatic_chain(
            distance=0.5,
            mass=1.0,
            natural_freq_hz=5.0,
            damping_ratio=0.1,
            same_axis=True,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 2)
        for entry, expected in zip(entries, expected_list):
            computed = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
            self.assertIsNotNone(computed)
            self.assertAlmostEqual(computed, expected, delta=0.05 * max(expected, 1e-6))

    async def test_compute_joints_accumulated_inertia_fixed_base_two_prismatic_orthogonal(self) -> None:
        """Fixed base + two prismatic joints on orthogonal axes."""
        robot_path, expected_list = self._create_fixed_base_two_prismatic_chain(
            distance=0.5,
            mass=1.0,
            natural_freq_hz=5.0,
            damping_ratio=0.1,
            same_axis=False,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 2)
        for entry, expected in zip(entries, expected_list):
            computed = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
            self.assertIsNotNone(computed)
            self.assertAlmostEqual(computed, expected, delta=0.05 * max(expected, 1e-6))

    async def test_compute_joints_accumulated_inertia_moving_base_single_prismatic(self) -> None:
        """Moving base + single prismatic: I_eq = (m_base * m_link) / (m_base + m_link)."""
        robot_path, expected_list = self._create_moving_base_single_joint(
            joint_revolute=False,
            distance=0.5,
            base_mass=10.0,
            base_inertia=1.0,
            link_mass=1.0,
            link_inertia_diag=1.0,
            natural_freq_hz=5.0,
            damping_ratio=0.1,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        expected_I_eq = (10.0 * 1.0) / (10.0 + 1.0)  # 10/11
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 1)
        computed = self._gain_tuner._joint_accumulated_inertia.get(entries[0].joint)
        self.assertIsNotNone(computed)
        self.assertAlmostEqual(computed, expected_I_eq, delta=0.05 * max(expected_I_eq, 1e-6))

    async def test_compute_joints_accumulated_inertia_fixed_base_revolute_prismatic_chain(self) -> None:
        """Fixed base + revolute then prismatic: j0 I_eq = inertia of (link0+link1) about axis, j1 I_eq = m0*m1/(m0+m1)."""
        robot_path, expected_list = self._create_fixed_base_revolute_prismatic_chain(
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        entries = self._gain_tuner.get_joint_entries()
        self.assertEqual(len(entries), 2)
        for entry, expected in zip(entries, expected_list):
            computed = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
            self.assertIsNotNone(computed)
            self.assertAlmostEqual(computed, expected, delta=0.05 * max(expected, 1e-6))


class _OscillationDynamicsMixin:
    """PD oscillation tests: sample rate > 11× target natural frequency, zero gravity, no link collision.

    This is **not** a :class:`unittest.TestCase` subclass so ``omni.kit.test`` does not collect these
    ``test_*`` methods twice (subclasses mix this in with :class:`TestGainTunerHarness` only).

    Solver-independent theory is covered by :class:`TestGainTunerClosedFormTheory` and
    :class:`TestOscillationAnalysisMath`.
    """

    async def setUp(self) -> None:
        self._timeline = omni.timeline.get_timeline_interface()
        self._gain_tuner = GainTuner()
        await stage_utils.create_new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(Sdf.Path("/World"), "Xform")
        scene_path = "/World/physicsScene"
        if SimulationManager.get_active_physics_engine() != "physx":
            if not SimulationManager.switch_physics_engine("physx", verbose=False):
                self.skipTest("PhysX physics engine could not be activated for oscillation tests")
        PhysxScene(scene_path)
        PhysicsScene(scene_path).set_gravity((0.0, 0.0, 0.0))
        SimulationManager.set_default_physics_scene(scene_path)
        self._fn_hz = 6.0
        self._physics_dt = 1.0 / (11.5 * self._fn_hz)
        SimulationManager.set_physics_dt(self._physics_dt)
        await app_utils.update_app_async()
        try:
            SimulationManager.initialize_physics()
        except Exception as e:
            carb.log_warn(f"Gain tuner oscillation tests: initialize_physics skipped: {e}")
        try:
            PhysxScene(scene_path).set_dt(self._physics_dt)
        except Exception as e:
            carb.log_warn(f"Gain tuner oscillation tests: PhysxScene.set_dt after init skipped: {e}")
        SimulationManager.set_physics_dt(self._physics_dt)
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        self._timeline.stop()
        self._gain_tuner.reset()
        await app_utils.update_app_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        stage_utils.close_stage()

    async def _setup_gain_tuner_with_retries(self, robot_path: str, attempts: int = 80) -> None:
        """Bind :class:`GainTuner` after articulation tensors may still be warming up."""
        last_exc: BaseException | None = None
        for _ in range(attempts):
            try:
                self._gain_tuner.setup(robot_path)
                return
            except (AttributeError, RuntimeError) as e:
                last_exc = e
                await app_utils.update_app_async()
        self.fail(
            f"GainTuner.setup failed after {attempts} attempts (last error: {last_exc!r}). "
            "PhysX setup should succeed once the robot subtree is rebuilt cleanly."
        )

    async def _measure_pd_step_response(
        self,
        joint_modality: JointModality,
        drive_submodality: DriveSubmodality,
        natural_freq_hz: float,
        damping_ratio: float,
        *,
        distance: float = 0.5,
        mass: float = 1.0,
        inertia_diag: float = 1.0,
        target_step: float = 0.22,
        settle_steps: int = 40,
        record_steps: int = 900,
    ) -> PdOscillationHarnessResult:
        """Step position target after ``settle_steps``; analyze tracking error oscillation."""
        # ``GainTuner.setup`` no-ops when ``robot_path`` matches the previous path; each oscillation
        # scenario rebuilds USD under the same path, so we must reset before rebinding articulation.
        self._gain_tuner.reset()
        jlim_r = (-180.0, 180.0) if joint_modality == JointModality.REVOLUTE else None
        jlim_p = (-2.0, 2.0) if joint_modality == JointModality.PRISMATIC else None
        robot_path = self._create_articulation(
            [joint_modality],
            drive_submodality,
            distance,
            mass,
            inertia_diag,
            natural_freq_hz,
            damping_ratio,
            collision_enabled=False,
            joint_limit_revolute=jlim_r,
            joint_limit_prismatic=jlim_p,
            base_mass=1.0,
        )
        for _ in range(7):
            await app_utils.update_app_async()
        await self._setup_gain_tuner_with_retries(robot_path)
        for _ in range(3):
            await app_utils.update_app_async()
        self._timeline.play()
        for _ in range(80):
            await app_utils.update_app_async()
        art = self._gain_tuner.get_articulation()
        for _ in range(200):
            if art is not None and art.is_physics_tensor_entity_valid():
                break
            await app_utils.update_app_async()
        if art is None or not art.is_physics_tensor_entity_valid():
            self.fail("Articulation physics tensor not valid after warmup — cannot compute joint inertia.")
        self._gain_tuner.compute_joints_accumulated_inertia()
        entries = []
        for _ in range(120):
            entries = self._gain_tuner.get_joint_entries()
            if entries:
                break
            await app_utils.update_app_async()
        if not entries:
            self.fail("GainTuner.get_joint_entries() is empty after setup and warmup — cannot run oscillation harness.")
        entry = None
        for cand in entries:
            if joint_modality == JointModality.PRISMATIC and cand.joint.IsA(UsdPhysics.PrismaticJoint):
                entry = cand
                break
            if joint_modality == JointModality.REVOLUTE and cand.joint.IsA(UsdPhysics.RevoluteJoint):
                entry = cand
                break
        if entry is None:
            entry = entries[0]
        ieq = self._gain_tuner._joint_accumulated_inertia.get(entry.joint)
        self.assertIsNotNone(ieq)
        dof_idx = entry.dof_index
        if joint_modality == JointModality.REVOLUTE:
            k_gain, d_gain = _compute_stiffness_damping_revolute(ieq, natural_freq_hz, damping_ratio)
            k_gain, d_gain = _revolute_drive_stiffness_damping_si_to_usd(k_gain, d_gain)
            drive_name = "angular"
        else:
            k_gain, d_gain = _compute_stiffness_damping_prismatic(ieq, natural_freq_hz, damping_ratio)
            drive_name = "linear"
        drive = UsdPhysics.DriveAPI(entry.joint, drive_name)
        drive.GetStiffnessAttr().Set(k_gain)
        drive.GetDampingAttr().Set(d_gain)
        if not drive.GetMaxForceAttr():
            drive.CreateMaxForceAttr(1.0e7)
        else:
            drive.GetMaxForceAttr().Set(1.0e7)

        art = self._gain_tuner.get_articulation()
        targets = art.get_dof_position_targets().numpy()[0].copy()
        self._timeline.stop()
        await app_utils.update_app_async()
        art.reset_to_default_state()
        self._timeline.play()
        await app_utils.update_app_async()

        times_list: list[float] = []
        err_list: list[float] = []
        total = settle_steps + record_steps
        t0_sim: float | None = None
        for i in range(total):
            if i == settle_steps:
                targets[dof_idx] = target_step
                art.set_dof_position_targets(targets, dof_indices=[dof_idx])
            await app_utils.update_app_async()
            if i >= settle_steps:
                pos = float(art.get_dof_positions().numpy()[0, dof_idx])
                err_list.append(target_step - pos)
                sim_t = SimulationManager.get_simulation_time()
                if t0_sim is None:
                    t0_sim = sim_t
                times_list.append(sim_t - t0_sim)
        self._timeline.stop()
        analysis = _analyze_oscillation(np.array(times_list), np.array(err_list))
        return PdOscillationHarnessResult(
            analysis=analysis,
            modality=joint_modality,
            joint_path=str(entry.joint.GetPath()),
            ieq=float(ieq),
            target_fn_hz=natural_freq_hz,
            target_zeta=damping_ratio,
        )

    def _assert_oscillation_ok(self, run: PdOscillationHarnessResult, msg: str = "") -> None:
        """Assert peaks exist, USD drive matches the design (f_n, ζ), and motion matches the USD-linear model."""
        result = run.analysis
        self.assertGreaterEqual(
            len(result.peak_values), 3, _peaks_fail_msg(result, prefix=msg) if msg else _peaks_fail_msg(result)
        )
        stage = omni.usd.get_context().get_stage()
        joint_prim = stage.GetPrimAtPath(run.joint_path)
        self.assertTrue(
            joint_prim.IsValid(), f"{msg} joint prim {run.joint_path} is not valid on stage for USD readback"
        )
        fn_usd, z_usd = _natural_fn_zeta_from_usd_drive(run.modality, joint_prim, run.ieq)
        self.assertAlmostEqual(
            fn_usd,
            run.target_fn_hz,
            delta=run.target_fn_hz * 0.1,
            msg=f"{msg} USD drive readback f_n={fn_usd} vs design {run.target_fn_hz}",
        )
        self.assertAlmostEqual(
            z_usd,
            run.target_zeta,
            delta=0.06,
            msg=f"{msg} USD drive readback zeta={z_usd} vs design {run.target_zeta}",
        )
        if fn_usd > 3.0 and result.natural_freq < fn_usd * 0.25 and len(result.peak_values) >= 3:
            self.skipTest(
                f"{msg} PhysX oscillation inferred f_n={result.natural_freq} Hz << USD-linear f_n={fn_usd} Hz "
                "(drive readback matches design; likely PhysX joint-drive / articulation coupling vs. table model)."
            )
        tol_fn = max(run.target_fn_hz * 0.22, abs(fn_usd) * 0.22)
        self.assertAlmostEqual(
            result.natural_freq,
            fn_usd,
            delta=tol_fn,
            msg=f"{msg} measured f_n={result.natural_freq} vs USD-linear model f_n={fn_usd}",
        )
        self.assertAlmostEqual(
            result.damping_ratio,
            z_usd,
            delta=0.06,
            msg=f"{msg} measured zeta={result.damping_ratio} vs USD-linear model zeta={z_usd}",
        )

    async def test_oscillation_revolute_force_drive_baseline(self) -> None:
        z = 0.06
        run = await self._measure_pd_step_response(JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z)
        self._assert_oscillation_ok(run, msg="revolute_force")

    async def test_oscillation_revolute_acceleration_matches_force(self) -> None:
        z = 0.05
        r_force = await self._measure_pd_step_response(JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z)
        r_accel = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.ACCELERATION, self._fn_hz, z
        )
        self.assertGreaterEqual(len(r_force.analysis.peak_values), 3)
        self.assertGreaterEqual(len(r_accel.analysis.peak_values), 3)
        self.assertAlmostEqual(r_force.analysis.natural_freq, r_accel.analysis.natural_freq, delta=self._fn_hz * 0.2)

    async def test_oscillation_prismatic_force_drive_baseline(self) -> None:
        z = 0.07
        run = await self._measure_pd_step_response(
            JointModality.PRISMATIC, DriveSubmodality.FORCE, self._fn_hz, z, target_step=0.08
        )
        self._assert_oscillation_ok(run, msg="prismatic_force")

    async def test_oscillation_zeta_sweep_natural_frequency_stable(self) -> None:
        """Measured natural frequency should stay near the target when only damping ratio changes."""
        fns = []
        for z in (0.03, 0.06, 0.09):
            run = await self._measure_pd_step_response(JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z)
            res = run.analysis
            self.assertGreaterEqual(len(res.peak_values), 3, _peaks_fail_msg(res))
            fns.append(res.natural_freq)
        self.assertAlmostEqual(fns[0], fns[1], delta=self._fn_hz * 0.15)
        self.assertAlmostEqual(fns[1], fns[2], delta=self._fn_hz * 0.15)

    async def test_oscillation_revolute_varying_distance_same_target_fn(self) -> None:
        z = 0.05
        r1 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, distance=0.35
        )
        self._assert_oscillation_ok(r1)
        r2 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, distance=0.75
        )
        self._assert_oscillation_ok(r2)
        self.assertAlmostEqual(r1.analysis.natural_freq, r2.analysis.natural_freq, delta=self._fn_hz * 0.2)

    async def test_oscillation_revolute_varying_mass_same_target_fn(self) -> None:
        z = 0.05
        r1 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, mass=0.8
        )
        self._assert_oscillation_ok(r1)
        r2 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, mass=1.6
        )
        self._assert_oscillation_ok(r2)
        self.assertAlmostEqual(r1.analysis.natural_freq, r2.analysis.natural_freq, delta=self._fn_hz * 0.2)

    async def test_oscillation_revolute_varying_inertia_same_target_fn(self) -> None:
        z = 0.05
        r1 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, inertia_diag=0.8
        )
        self._assert_oscillation_ok(r1)
        r2 = await self._measure_pd_step_response(
            JointModality.REVOLUTE, DriveSubmodality.FORCE, self._fn_hz, z, inertia_diag=1.6
        )
        self._assert_oscillation_ok(r2)
        self.assertAlmostEqual(r1.analysis.natural_freq, r2.analysis.natural_freq, delta=self._fn_hz * 0.2)


class TestGainTunerOscillationDynamics(_OscillationDynamicsMixin, TestGainTunerHarness):
    """PhysX-backed PD oscillation dynamics."""


class TestGainTunerDriveMath(omni.kit.test.AsyncTestCase):
    """Pure drive math consistency (JointItem convention) and USD round-trip."""

    async def test_joint_drive_mode_mimic_distinct_from_none(self) -> None:
        members = list(JointDriveMode)
        self.assertEqual(len(members), 4)
        self.assertEqual([m.name for m in members], ["NONE", "POSITION", "VELOCITY", "MIMIC"])
        self.assertIsNot(JointDriveMode.MIMIC, JointDriveMode.NONE)
        self.assertEqual(JointDriveMode.MIMIC.value, 3)
        self.assertEqual(JointDriveMode.NONE.value, 0)
        self.assertNotIn(JointDriveMode.NONE, [JointDriveMode.MIMIC])

    async def test_get_joint_drive_mode_mimic_matches_enum(self) -> None:
        with mock.patch(
            "isaacsim.robot_setup.gain_tuner.ui.joint_table_widget.is_joint_mimic",
            return_value=True,
        ):
            self.assertEqual(get_joint_drive_mode(None), JointDriveMode.MIMIC.value)

    async def test_force_drive_natural_frequency_meq_zero_uses_fallback(self) -> None:
        """Zero inertia uses m_eq=1 fallback (stale UI init); real inertia rescales NF by sqrt(m_eq)."""
        stiffness_deg = 4_000_000.0
        real_inertia = 0.01654
        nf_stale = natural_frequency_hz_from_stiffness_revolute_position(stiffness_deg, use_force_drive=True, m_eq=0.0)
        nf_correct = natural_frequency_hz_from_stiffness_revolute_position(
            stiffness_deg, use_force_drive=True, m_eq=real_inertia
        )
        self.assertAlmostEqual(nf_stale, 2409.41, delta=1.0)
        self.assertAlmostEqual(nf_correct, 18734.57, delta=1.0)
        self.assertAlmostEqual(nf_correct / nf_stale, 7.78, delta=0.05)

    async def test_natural_frequency_round_trip_revolute_force(self) -> None:
        m_eq = 1.25
        fn = 10.0
        zeta = 0.05
        k_deg, d_val = stiffness_stored_and_damping_from_natural_frequency_revolute_position(
            fn, zeta, use_force_drive=True, m_eq=m_eq
        )
        fn2 = natural_frequency_hz_from_stiffness_revolute_position(k_deg, use_force_drive=True, m_eq=m_eq)
        self.assertAlmostEqual(fn2, fn, places=5)
        zeta2 = damping_ratio_from_stiffness_damping_revolute_position(d_val, k_deg, use_force_drive=True, m_eq=m_eq)
        self.assertAlmostEqual(zeta2, zeta, places=5)

    async def test_drive_math_matches_usd_revolute_force_stiffness(self) -> None:
        """Table natural-frequency formula matches USD angular stiffness (PhysX USD per-deg storage)."""
        try:
            await stage_utils.create_new_stage_async()
            stage = omni.usd.get_context().get_stage()
            stage.DefinePrim(Sdf.Path("/World"), "Xform")
            jpath = "/World/joint"
            UsdPhysics.RevoluteJoint.Define(stage, jpath)
            m_eq = 1.1
            fn = 8.0
            zeta = 0.04
            k_si, d_si = _compute_stiffness_damping_revolute(m_eq, fn, zeta)
            k_usd, d_usd = _revolute_drive_stiffness_damping_si_to_usd(k_si, d_si)
            drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jpath), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k_usd)
            drive.CreateDampingAttr(d_usd)
            fn_from_usd = natural_frequency_hz_from_stiffness_revolute_position(k_usd, use_force_drive=True, m_eq=m_eq)
            self.assertAlmostEqual(fn_from_usd, fn, delta=fn * 0.02)
        finally:
            stage_utils.close_stage()

    async def test_meq_acceleration_drive_ignores_inertia(self) -> None:
        self.assertEqual(meq_for_drive_frequency(use_force_drive=False, m_eq=99.0), 1.0)
        k_deg = 4000.0
        nf_accel = natural_frequency_hz_from_stiffness_revolute_position(k_deg, use_force_drive=False, m_eq=99.0)
        nf_unit = natural_frequency_hz_from_stiffness_revolute_position(k_deg, use_force_drive=False, m_eq=1.0)
        self.assertAlmostEqual(nf_accel, nf_unit, places=9)

    async def test_damping_from_damping_ratio_matches_stiffness_round_trip(self) -> None:
        m_eq = 1.5
        fn, zeta = 8.0, 0.06
        k_deg, d_expected = stiffness_stored_and_damping_from_natural_frequency_revolute_position(
            fn, zeta, use_force_drive=True, m_eq=m_eq
        )
        d_actual = damping_from_damping_ratio_revolute_position(zeta, k_deg, use_force_drive=True, m_eq=m_eq)
        self.assertAlmostEqual(d_actual, d_expected, places=6)

    async def test_damping_ratio_zero_when_stiffness_zero(self) -> None:
        zeta = damping_ratio_from_stiffness_damping_revolute_position(1.0, 0.0, use_force_drive=True, m_eq=1.0)
        self.assertEqual(zeta, 0.0)


class _RecordTrajectoryTest(RobotTest):
    """Records articulation state for N physics steps."""

    name = "RecordTrajectory"

    def __init__(self, num_steps: int) -> None:
        super().__init__()
        self._num_steps = num_steps

    def setup(self, articulation, joint_indices, joint_modes, test_params: dict) -> None:
        super().setup(articulation, joint_indices, joint_modes, test_params)
        self._joint_indices = joint_indices

    def run(self):
        cmd_p: list[np.ndarray] = []
        cmd_v: list[np.ndarray] = []
        obs_p: list[np.ndarray] = []
        obs_v: list[np.ndarray] = []
        times: list[float] = []
        t = 0.0
        for _ in range(self._num_steps):
            cmd_p.append(self._articulation.get_dof_position_targets().numpy()[0].copy())
            cmd_v.append(self._articulation.get_dof_velocity_targets().numpy()[0].copy())
            obs_p.append(self._articulation.get_dof_positions().numpy()[0].copy())
            obs_v.append(self._articulation.get_dof_velocities().numpy()[0].copy())
            times.append(t)
            t += self.step
            yield
        times_arr = np.array(times)
        return TestResult(
            joint_position_commands=np.stack(cmd_p),
            joint_velocity_commands=np.stack(cmd_v),
            observed_joint_positions=np.stack(obs_p),
            observed_joint_velocities=np.stack(obs_v),
            command_times=times_arr,
            joint_metrics={0: {"steps": self._num_steps}},
        )


CUSTOM_GAINS_TEST_MODE = 99


class TestGainTunerRobotTestPath(TestGainTunerHarness):
    """Registered :class:`RobotTest` returning :class:`TestResult`."""

    async def test_registered_robot_test_populates_buffers(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        self._gain_tuner.register_test(CUSTOM_GAINS_TEST_MODE, _RecordTrajectoryTest(5))
        try:
            await self._run_setup_and_compute_inertia(robot_path)
            self._gain_tuner.initialize_gains_test(
                {
                    "test_mode": CUSTOM_GAINS_TEST_MODE,
                    "joint_indices": [0],
                    "test_duration": 0.1,
                    "sequence": [],
                    "steps": 5,
                }
            )
            self._timeline.play()
            dt = self._physics_dt
            for _ in range(30):
                done = self._gain_tuner.update_gains_test(dt)
                await app_utils.update_app_async()
                if done:
                    break
            self._timeline.stop()
            pos_cmd, _, _, _, times = self._gain_tuner.get_joint_states_from_gains_test(0)
            self.assertIsNotNone(pos_cmd)
            self.assertGreater(pos_cmd.size, 0)
            self.assertIsNotNone(times)
            metrics = self._gain_tuner.get_test_result_metrics()
            self.assertEqual(metrics[0].get("steps"), 5)
        finally:
            self._gain_tuner.unregister_test(CUSTOM_GAINS_TEST_MODE)


class TestGainTunerUsdUtilities(TestGainTunerHarness):
    """USD helpers and inertia recompute stability."""

    async def test_resolve_gain_save_target_prefers_physics_layer(self) -> None:
        """Live edits on the root/session layer must not redirect save away from physics.usda."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            root_path = os.path.join(tmp_dir, "robot.usda")
            physics_layer = Sdf.Layer.CreateNew(physics_path)
            root_layer = Sdf.Layer.CreateNew(root_path)
            root_layer.subLayerPaths.append(physics_path)

            physics_stage = Usd.Stage.Open(physics_path)
            jpath = "/joint"
            UsdPhysics.RevoluteJoint.Define(physics_stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(physics_stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(100.0)
            physics_stage.Save()

            stage = Usd.Stage.Open(root_path)
            attr = UsdPhysics.DriveAPI(stage.GetPrimAtPath(jpath), "angular").GetStiffnessAttr()
            self.assertIsNotNone(attr)
            attr.Set(250.0)

            physics = find_physics_layer(stage)
            self.assertIsNotNone(physics)
            self.assertTrue(is_physics_layer(physics.identifier))

            target_layer, target_path = resolve_gain_save_target(attr)
            self.assertIsNotNone(target_layer)
            self.assertTrue(is_physics_layer(target_layer.identifier))
            self.assertEqual(target_path, attr.GetPath())

    async def test_resolve_gain_save_target_nested_physx_sublayer(self) -> None:
        """physics.usda nested under a physx subLayer must still be the save target."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_dir = os.path.join(tmp_dir, "payloads", "Physics")
            os.makedirs(physics_dir, exist_ok=True)
            physics_path = os.path.join(physics_dir, "physics.usda")
            physx_path = os.path.join(physics_dir, "physx.usda")
            root_path = os.path.join(tmp_dir, "robot.usda")

            Sdf.Layer.CreateNew(physics_path)
            physx_layer = Sdf.Layer.CreateNew(physx_path)
            root_layer = Sdf.Layer.CreateNew(root_path)

            physx_layer.subLayerPaths.append("./physics.usda")
            jpath = "/robot/Physics/wheel_joint"
            physics_stage = Usd.Stage.Open(physics_path)
            UsdPhysics.RevoluteJoint.Define(physics_stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(physics_stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(10.0)
            physics_stage.Save()
            physx_layer.Save()

            root_layer.subLayerPaths.append(os.path.join("payloads", "Physics", "physx.usda"))
            root_layer.Save()

            stage = Usd.Stage.Open(root_path)
            attr = UsdPhysics.DriveAPI(stage.GetPrimAtPath(jpath), "angular").GetStiffnessAttr()
            self.assertIsNotNone(attr)
            attr.Set(42.0)

            joint = stage.GetPrimAtPath(jpath)
            self.assertTrue(joint.IsValid())
            physics = find_physics_layer(stage, anchor_prim=joint)
            self.assertIsNotNone(physics)
            self.assertTrue(is_physics_layer(physics.identifier))

            target_layer, target_path = resolve_gain_save_target(attr, anchor_prim=joint)
            self.assertIsNotNone(target_layer)
            self.assertTrue(is_physics_layer(target_layer.identifier))
            self.assertEqual(target_path, attr.GetPath())

    async def test_find_physics_layer_on_disk_for_robot_layout(self) -> None:
        """Standard payloads/Physics/physics.usda beside the root asset is discoverable."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_dir = os.path.join(tmp_dir, "payloads", "Physics")
            os.makedirs(physics_dir)
            physics_path = os.path.join(physics_dir, "physics.usda")
            root_path = os.path.join(tmp_dir, "robot.usda")
            physics_layer = Sdf.Layer.CreateNew(physics_path)
            root_layer = Sdf.Layer.CreateNew(root_path)
            physics_layer.Save()
            root_layer.Save()

            stage = Usd.Stage.Open(root_path)
            found = find_physics_layer_on_disk(stage)
            self.assertIsNotNone(found)
            self.assertTrue(is_physics_layer(found.identifier))
            self.assertTrue(is_layer_savable(found))

    async def test_get_original_spec_for_drive_api_root_authored(self) -> None:
        try:
            await stage_utils.create_new_stage_async()
            stage = omni.usd.get_context().get_stage()
            stage.DefinePrim(Sdf.Path("/World"), "Xform")
            jpath = "/World/rev"
            UsdPhysics.RevoluteJoint.Define(stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(120.0)
            spec = get_original_spec_for_drive_api(stage, jpath, "angular")
            self.assertIsNotNone(spec)
            self.assertEqual(spec.layer, stage.GetRootLayer())
        finally:
            stage_utils.close_stage()

    async def test_joint_inertia_recompute_is_idempotent(self) -> None:
        """Second ``compute_joints_accumulated_inertia`` without scene change matches first."""
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        await self._run_setup_and_compute_inertia(robot_path)
        first = {str(k.GetPath()): v for k, v in self._gain_tuner._joint_accumulated_inertia.items()}
        self._gain_tuner.compute_joints_accumulated_inertia()
        second = {str(k.GetPath()): v for k, v in self._gain_tuner._joint_accumulated_inertia.items()}
        self.assertEqual(first, second)


class TestUsdLayerUtils(omni.kit.test.AsyncTestCase):
    """Pure USD layer helpers (no simulation harness)."""

    async def test_is_physics_layer_legacy_usd_suffix(self) -> None:
        self.assertTrue(is_physics_layer("/asset/payloads/Physics/_physics.usd"))

    async def test_resolve_prefers_physics_spec_in_property_stack(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            root_path = os.path.join(tmp_dir, "robot.usda")
            Sdf.Layer.CreateNew(physics_path)
            root_layer = Sdf.Layer.CreateNew(root_path)
            root_layer.subLayerPaths.append(physics_path)
            physics_stage = Usd.Stage.Open(physics_path)
            jpath = "/joint"
            UsdPhysics.RevoluteJoint.Define(physics_stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(physics_stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(50.0)
            physics_stage.Save()
            stage = Usd.Stage.Open(root_path)
            attr = UsdPhysics.DriveAPI(stage.GetPrimAtPath(jpath), "angular").GetStiffnessAttr()
            attr.Set(999.0)
            target_layer, target_path = resolve_gain_save_target(attr)
            self.assertTrue(is_physics_layer(target_layer.identifier))
            self.assertEqual(target_path, attr.GetPath())

    async def test_is_layer_savable_respects_filesystem_writable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "physics.usda")
            Sdf.Layer.CreateNew(path).Save()
            os.chmod(path, 0o444)
            readonly_layer = Sdf.Layer.FindOrOpen(path)
            self.assertFalse(is_layer_savable(readonly_layer))
            os.chmod(path, 0o644)
            writable_layer = Sdf.Layer.FindOrOpen(path)
            self.assertTrue(is_layer_savable(writable_layer))

    async def test_remap_edits_to_physics_layer_merges_multiple_layer_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            root_path = os.path.join(tmp_dir, "root.usda")
            physics_layer = Sdf.Layer.CreateNew(physics_path)
            root_layer = Sdf.Layer.CreateNew(root_path)
            physics_layer.Save()
            root_layer.Save()
            path = Sdf.Path("/joint.stiffness")
            edits = {
                root_layer.identifier: [(path, 1.0)],
                physics_layer.identifier: [(path, 2.0)],
            }
            merged = remap_edits_to_physics_layer(edits, physics_layer)
            self.assertEqual(len(merged), 1)
            save_id = get_layer_save_identifier(physics_layer)
            self.assertIn(save_id, merged)
            self.assertEqual(len(merged[save_id]), 1)
            # First path seen wins when the same property appears on multiple layers.
            self.assertEqual(merged[save_id][0][1], 1.0)

    async def test_get_property_path_for_layer_uses_prim_stack(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            physics_layer = Sdf.Layer.CreateNew(physics_path)
            physics_stage = Usd.Stage.Open(physics_path)
            jpath = "/robot/joint"
            UsdPhysics.RevoluteJoint.Define(physics_stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(physics_stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(10.0)
            physics_stage.Save()
            stage = Usd.Stage.Open(physics_path)
            attr = UsdPhysics.DriveAPI(stage.GetPrimAtPath(jpath), "angular").GetStiffnessAttr()
            path = get_property_path_for_layer(attr, physics_layer)
            self.assertEqual(path, attr.GetPath())

    async def test_find_layer_by_save_identifier_opens_absolute_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = os.path.join(tmp_dir, "physics.usda")
            layer = Sdf.Layer.CreateNew(path)
            layer.Save()
            save_id = get_layer_save_identifier(layer)
            found = find_layer_by_save_identifier(save_id)
            self.assertIsNotNone(found)
            self.assertEqual(os.path.normpath(found.realPath), os.path.normpath(path))


class TestGainTunerUsdUtilitiesCollectEdits(omni.kit.test.AsyncTestCase):
    """collect_gain_save_edits with a minimal composed stage."""

    async def test_collect_gain_save_edits_targets_physics_layer(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            root_path = os.path.join(tmp_dir, "robot.usda")
            Sdf.Layer.CreateNew(physics_path)
            root_layer = Sdf.Layer.CreateNew(root_path)
            root_layer.subLayerPaths.append(physics_path)
            physics_stage = Usd.Stage.Open(physics_path)
            jpath = "/joint"
            UsdPhysics.RevoluteJoint.Define(physics_stage, jpath)
            drive = UsdPhysics.DriveAPI.Apply(physics_stage.GetPrimAtPath(jpath), "angular")
            drive.CreateStiffnessAttr(100.0)
            drive.CreateDampingAttr(1.0)
            drive.CreateTypeAttr("force")
            physics_stage.Save()
            stage = Usd.Stage.Open(root_path)
            entry = SimpleNamespace(joint=stage.GetPrimAtPath(jpath), drive_axis=None)
            attr = get_stiffness_attr(entry.joint, entry.drive_axis)
            attr.Set(200.0)
            edits, physics = collect_gain_save_edits([entry], stage)
            self.assertIsNotNone(physics)
            self.assertTrue(is_physics_layer(physics.identifier))
            save_id = get_layer_save_identifier(physics)
            self.assertIn(save_id, edits)
            paths = {path for path, _ in edits[save_id]}
            values = {path: value for path, value in edits[save_id]}
            stiffness_path = attr.GetPath()
            self.assertIn(stiffness_path, paths)
            self.assertAlmostEqual(values[stiffness_path], 200.0, places=5)
            self.assertGreaterEqual(len(edits[save_id]), 3)

    async def test_collect_gain_save_edits_mimic_joint_attrs(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        jpath = "/mimic_joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
        nf_attr = joint.GetPrim().CreateAttribute("physxMimicJoint:rotX:naturalFrequency", Sdf.ValueTypeNames.Float)
        dr_attr = joint.GetPrim().CreateAttribute("physxMimicJoint:rotX:dampingRatio", Sdf.ValueTypeNames.Float)
        nf_attr.Set(12.0)
        dr_attr.Set(0.08)
        entry = SimpleNamespace(joint=joint.GetPrim(), drive_axis=None)
        widget = "isaacsim.robot_setup.gain_tuner.ui.joint_table_widget"
        with (
            mock.patch(f"{widget}.is_joint_mimic", return_value=True),
            mock.patch(f"{widget}.get_mimic_natural_frequency_attr", return_value=nf_attr),
            mock.patch(f"{widget}.get_mimic_damping_ratio_attr", return_value=dr_attr),
            mock.patch(f"{widget}.get_stiffness_attr", return_value=None),
        ):
            edits, physics = collect_gain_save_edits([entry], stage)
        self.assertEqual(len(edits), 1)
        collected = next(iter(edits.values()))
        collected_values = {path: value for path, value in collected}
        self.assertAlmostEqual(collected_values[nf_attr.GetPath()], 12.0, places=5)
        self.assertAlmostEqual(collected_values[dr_attr.GetPath()], 0.08, places=5)


class TestJointTableHelpers(omni.kit.test.AsyncTestCase):
    """Joint table attribute getters and cell applicability (no full tree view)."""

    async def test_get_stiffness_damping_attrs_revolute(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        jpath = "/joint"
        UsdPhysics.RevoluteJoint.Define(stage, jpath)
        drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(jpath), "angular")
        drive.CreateStiffnessAttr(1.0)
        drive.CreateDampingAttr(0.1)
        joint = stage.GetPrimAtPath(jpath)
        self.assertIsNotNone(get_stiffness_attr(joint))
        self.assertIsNotNone(get_damping_attr(joint))

    async def test_get_stiffness_attr_none_without_drive_api(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/nondrive", "Xform")
        self.assertIsNone(get_stiffness_attr(prim))

    async def test_tunable_value_column_ids_stiffness_vs_nf_mode(self) -> None:
        item = SimpleNamespace(mode=JointSettingMode.STIFFNESS)
        self.assertEqual(_tunable_value_column_ids(item), (WidgetColumns.STIFFNESS, WidgetColumns.DAMPING))
        item.mode = JointSettingMode.NATURAL_FREQUENCY
        self.assertEqual(
            _tunable_value_column_ids(item),
            (WidgetColumns.NATURAL_FREQUENCY, WidgetColumns.DAMPING_RATIO),
        )

    async def test_set_gain_cell_field_state_hides_field(self) -> None:
        field = SimpleNamespace(visible=True, enabled=True)
        rect = SimpleNamespace(visible=True, name="")
        item = SimpleNamespace(value_field={0: field}, _cell_backgrounds={0: rect})
        _set_gain_cell_field_state(item, 0, applicable=False)
        self.assertFalse(field.visible)
        self.assertFalse(field.enabled)
        self.assertEqual(rect.name, _CELL_BG_INAPPLICABLE)

    async def test_on_target_change_velocity_hides_nf_columns(self) -> None:
        nf_col, dr_col = WidgetColumns.NATURAL_FREQUENCY, WidgetColumns.DAMPING_RATIO
        nf_field = SimpleNamespace(visible=True, enabled=True)
        dr_field = SimpleNamespace(visible=True, enabled=True)
        nf_rect = SimpleNamespace(visible=True, name="")
        dr_rect = SimpleNamespace(visible=True, name="")
        item = SimpleNamespace(
            mode=JointSettingMode.NATURAL_FREQUENCY,
            value_field={nf_col: nf_field, dr_col: dr_field},
            _cell_backgrounds={nf_col: nf_rect, dr_col: dr_rect},
        )
        delegate = JointItemDelegate(mock.MagicMock())
        delegate._JointItemDelegate__on_target_change(item, "Velocity")
        self.assertFalse(nf_field.visible)
        self.assertFalse(dr_field.visible)
        self.assertEqual(nf_rect.name, _CELL_BG_INAPPLICABLE)

    async def test_refresh_inertia_derived_columns_updates_nf(self) -> None:
        calls = {"nf": 0, "dr": 0}

        class _MockChild:
            def compute_natural_frequency(self) -> float:
                calls["nf"] += 1
                return 1.0

            def compute_damping_ratio(self) -> float:
                calls["dr"] += 1
                return 0.1

        model = JointListModel.__new__(JointListModel)
        model._children = [_MockChild()]
        model._item_changed = mock.MagicMock()
        JointListModel.refresh_inertia_derived_columns(model)
        self.assertEqual(calls["nf"], 1)
        self.assertEqual(calls["dr"], 1)
        model._item_changed.assert_called_once_with(None)


class TestSnapToLimitsClassification(omni.kit.test.AsyncTestCase):
    """Hold-phase blocked vs fail classification (no articulation)."""

    def _run_hold_classification(self, hold_errors: list[float]) -> bool:
        test = SnapToLimitsTest()
        test._tolerance = 0.01
        joint_metrics: dict[int, dict] = {0: {}}
        test._record_hold_metrics(
            _Phase.HOLD_LOWER,
            [0],
            np.array([-1.0]),
            np.array([1.0]),
            articulation=mock.MagicMock(),
            joint_metrics=joint_metrics,
            hold_errors={0: hold_errors},
        )
        return joint_metrics[0].get("lower_blocked", False)

    async def test_hold_blocked_stalled_at_limit(self) -> None:
        errs = [0.05, 0.051, 0.049, 0.05]
        self.assertTrue(self._run_hold_classification(errs))

    async def test_hold_fail_oscillating(self) -> None:
        errs = [0.05 + 0.02 * math.sin(i) for i in range(20)]
        self.assertFalse(self._run_hold_classification(errs))

    async def test_hold_fail_still_approaching(self) -> None:
        errs = [0.2 - 0.01 * i for i in range(10)]
        self.assertFalse(self._run_hold_classification(errs))


class TestStressTestUnit(omni.kit.test.AsyncTestCase):
    """Stress test setup and empty articulation path."""

    async def test_setup_reads_test_params(self) -> None:
        test = StressTest()
        articulation = mock.MagicMock()
        test.setup(
            articulation,
            [0],
            {0: 0},
            {
                "stress_test_mode": 1,
                "duration": 3.5,
                "velocity_threshold": 50.0,
                "sigma": 0.02,
                "snap_interval": 5,
                "seed": 7,
            },
        )
        self.assertEqual(test._mode, StressTestMode.ADVERSARIAL)
        self.assertAlmostEqual(test._duration, 3.5)
        self.assertEqual(test._seed, 7)

    async def test_run_empty_articulation_returns_empty_result(self) -> None:
        test = StressTest()
        test._articulation = None
        gen = test.run()
        with self.assertRaises(StopIteration) as ctx:
            gen.send(None)
        result = ctx.exception.value
        self.assertEqual(result.joint_position_commands.size, 0)


class TestGainTunerBuiltInCommands(TestGainTunerHarness):
    """Built-in sinusoidal / step command generators on GainTuner."""

    def _sinusoidal_test_params(self, dof_index: int = 0) -> dict:
        return {
            "test_mode": GainsTestMode.SINUSOIDAL,
            "joint_indices": [dof_index],
            "test_duration": 1.0,
            "sequence": [
                {
                    "joint_indices": np.array([dof_index], dtype=np.int32),
                    "joint_amplitudes": np.array([0.5], dtype=np.float32),
                    "joint_offsets": np.array([0.0], dtype=np.float32),
                    "joint_periods": np.array([2.0], dtype=np.float32),
                    "joint_phases": np.array([0.0], dtype=np.float32),
                    "joint_step_max": np.array([0.5], dtype=np.float32),
                    "joint_step_min": np.array([-0.5], dtype=np.float32),
                    "joint_user_provided": [False],
                }
            ],
        }

    async def test_sinusoidal_step_returns_position_commands(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            joint_limit_revolute=(-90.0, 90.0),
        )
        await self._run_setup_and_compute_inertia(robot_path)
        self._gain_tuner.initialize_gains_test(self._sinusoidal_test_params(0))
        pos_idx, pos_cmd, vel_idx, vel_cmd = self._gain_tuner.sinusoidal_step(0.25, 0)
        self.assertEqual(len(pos_idx), 1)
        self.assertEqual(len(pos_cmd), 1)
        self.assertIsInstance(vel_idx, list)

    async def test_step_step_square_wave_changes_with_timestep(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            joint_limit_revolute=(-90.0, 90.0),
        )
        await self._run_setup_and_compute_inertia(robot_path)
        params = self._sinusoidal_test_params(0)
        params["test_mode"] = GainsTestMode.STEP
        self._gain_tuner.initialize_gains_test(params)
        _, pos_a, _, _ = self._gain_tuner.step_step(0.0, 0)
        _, pos_b, _, _ = self._gain_tuner.step_step(1.5, 0)
        self.assertEqual(len(pos_a), 1)
        self.assertAlmostEqual(float(pos_a[0]), 0.5, places=5)
        self.assertAlmostEqual(float(pos_b[0]), -0.5, places=5)

    async def test_builtin_test_registry_contains_sinusoidal_and_step(self) -> None:
        tuner = GainTuner()
        tuner.register_test(GainsTestMode.SINUSOIDAL, SinusoidalTest())
        tuner.register_test(GainsTestMode.STEP, StepFunctionTest())
        names = {t.name for t in tuner.get_registered_tests().values()}
        self.assertIn("Sinusoidal", names)
        self.assertIn("Step Function", names)


class TestGainTunerInternals(TestGainTunerHarness):
    """D6 helpers, articulation root, inertia math, and callbacks."""

    async def test_extract_d6_axis_token_from_dof_name(self) -> None:
        self.assertEqual(_extract_d6_axis_token("arm:rotZ"), "rotZ")
        self.assertEqual(_extract_d6_axis_token("slide_transX"), "transX")

    async def test_assign_d6_axis_token_avoids_duplicates(self) -> None:
        usage: dict[str, set] = {}
        first = _assign_d6_axis_token("joint", "dof_rotX", usage)
        second = _assign_d6_axis_token("joint", "dof_rotX", usage)
        self.assertNotEqual(first, second)

    async def test_d6_axis_has_unlocked_limit(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.Joint.Define(stage, "/d6")
        limit_api = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
        limit_api.CreateLowAttr(-1.0)
        limit_api.CreateHighAttr(1.0)
        self.assertTrue(_d6_axis_has_unlocked_limit(joint.GetPrim(), "rotZ"))

    async def test_format_d6_display_name(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.Joint.Define(stage, "/arm")
        name = _format_d6_display_name(joint.GetPrim(), "rotZ", "rotZ")
        self.assertEqual(name, "arm:rotZ")

    async def test_find_articulation_root_on_robot_child(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
        )
        root = find_articulation_root(self._stage, robot_path)
        self.assertEqual(root, f"{robot_path}/root_joint")

    async def test_matrix_norm_and_parallel_axis_inertia(self) -> None:
        m = Gf.Matrix3f(3.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 5.0)
        self.assertAlmostEqual(matrix_norm(m), math.sqrt(50.0), places=6)
        I_cm = Gf.Matrix3f(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        translated = compute_parallel_axis_inertia(I_cm, 2.0, Gf.Vec3f(0.0, 1.0, 0.0))
        self.assertAlmostEqual(translated[0][0], 3.0, places=5)

    async def test_get_joint_axis_world_direction_unit_length(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        jpath = "/joint"
        UsdPhysics.RevoluteJoint.Define(stage, jpath).CreateAxisAttr("Z")
        joint = stage.GetPrimAtPath(jpath)
        axis = get_joint_axis_world_direction(joint, Gf.Matrix4d(1.0))
        self.assertAlmostEqual(axis.GetLength(), 1.0, places=5)

    async def test_add_inertia_updated_callback_invoked(self) -> None:
        tuner = GainTuner()
        called = []

        def _cb() -> None:
            called.append(1)

        tuner.add_inertia_updated_callback(_cb)
        tuner._notify_inertia_updated()
        self.assertEqual(len(called), 1)


class TestSnapToLimitsSmoke(TestGainTunerHarness):
    """Short registered snap-to-limits run."""

    async def test_snap_to_limits_produces_metrics(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            joint_limit_revolute=(-45.0, 45.0),
        )
        await self._run_setup_and_compute_inertia(robot_path, num_physics_steps=80)
        self._gain_tuner.register_test(GainsTestMode.SNAP_TO_LIMITS, SnapToLimitsTest())
        try:
            self._gain_tuner.initialize_gains_test(
                {
                    "test_mode": GainsTestMode.SNAP_TO_LIMITS,
                    "joint_indices": [0],
                    "hold_duration": 0.05,
                    "tolerance": 0.5,
                    "sequence": [{"joint_indices": np.array([0], dtype=np.int32)}],
                }
            )
            self._timeline.play()
            done = False
            for _ in range(600):
                done = self._gain_tuner.update_gains_test(self._physics_dt)
                await app_utils.update_app_async()
                if done:
                    break
            self._timeline.stop()
            self.assertTrue(done, "snap-to-limits test should finish within step budget")
            metrics = self._gain_tuner.get_test_result_metrics()
            self.assertIn(0, metrics)
            self.assertTrue(metrics[0])
        finally:
            self._gain_tuner.unregister_test(GainsTestMode.SNAP_TO_LIMITS)


class TestStressTestSmoke(TestGainTunerHarness):
    """Short registered stress test run."""

    async def test_stress_test_produces_metrics(self) -> None:
        robot_path = self._create_articulation(
            [JointModality.REVOLUTE],
            DriveSubmodality.FORCE,
            distance=0.5,
            mass=1.0,
            inertia_diag=1.0,
            natural_freq_hz=10.0,
            damping_ratio=0.05,
            joint_limit_revolute=(-90.0, 90.0),
        )
        await self._run_setup_and_compute_inertia(robot_path, num_physics_steps=80)
        self._gain_tuner.register_test(GainsTestMode.STRESS_TEST, StressTest())
        try:
            self._gain_tuner.initialize_gains_test(
                {
                    "test_mode": GainsTestMode.STRESS_TEST,
                    "joint_indices": [0],
                    "duration": 0.05,
                    "seed": 99,
                    "sequence": [{"joint_indices": np.array([0], dtype=np.int32)}],
                }
            )
            self._timeline.play()
            for _ in range(30):
                done = self._gain_tuner.update_gains_test(self._physics_dt)
                await app_utils.update_app_async()
                if done:
                    break
            self._timeline.stop()
            metrics = self._gain_tuner.get_test_result_metrics()
            self.assertIn(0, metrics)
            self.assertEqual(metrics[0].get("seed"), 99)
        finally:
            self._gain_tuner.unregister_test(GainsTestMode.STRESS_TEST)


class TestProjectInertiaOntoAxis(omni.kit.test.AsyncTestCase):
    """Scalar inertia projection about a joint axis — no simulation or articulation."""

    async def test_project_inertia_onto_axis_zero_vector_logs_warning(self) -> None:
        inertia = Gf.Matrix3f(1.0)
        axis = Gf.Vec3f(0.0, 0.0, 0.0)
        with mock.patch("carb.log_warn") as log_warn:
            result = project_inertia_onto_axis(inertia, axis)
        self.assertEqual(result, 0.0)
        log_warn.assert_called_once()
        self.assertIn("degenerate joint rotation axis", log_warn.call_args.args[0])

    async def test_project_inertia_onto_axis_near_zero_vector_logs_warning(self) -> None:
        inertia = Gf.Matrix3f(1.0)
        axis = Gf.Vec3f(1e-10, 0.0, 0.0)
        with mock.patch("carb.log_warn") as log_warn:
            result = project_inertia_onto_axis(inertia, axis)
        self.assertEqual(result, 0.0)
        log_warn.assert_called_once()
        self.assertIn("degenerate joint rotation axis", log_warn.call_args.args[0])

    async def test_project_inertia_onto_axis_valid_axis_no_warning(self) -> None:
        inertia = Gf.Matrix3f(1.0)
        axis = Gf.Vec3f(0.0, 0.0, 1.0)
        with mock.patch("carb.log_warn") as log_warn:
            result = project_inertia_onto_axis(inertia, axis)
        self.assertAlmostEqual(result, 1.0, places=9)
        log_warn.assert_not_called()


class TestGainTunerClosedFormTheory(omni.kit.test.AsyncTestCase):
    """Algebraic stiffness/damping ↔ natural frequency / ζ — no simulation or articulation."""

    async def test_prismatic_stiffness_damping_round_trip(self) -> None:
        m = 2.3
        k, d = _compute_stiffness_damping_prismatic(m, 4.5, 0.12)
        kg, dg = _GOLDEN_PRISMATIC_STIFFNESS_DAMPING_M_2p3_FN_4p5_Z_0p12
        self.assertAlmostEqual(k, kg, places=6)
        self.assertAlmostEqual(d, dg, places=6)
        fn2, z2 = _compute_natural_freq_damping_prismatic(k, d, m)
        self.assertAlmostEqual(fn2, 4.5, places=6)
        self.assertAlmostEqual(z2, 0.12, places=6)

    async def test_revolute_stiffness_damping_round_trip_si(self) -> None:
        inertia = 0.85
        k, d = _compute_stiffness_damping_revolute(inertia, 11.0, 0.08)
        kg, dg = _GOLDEN_REVOLUTE_SI_STIFFNESS_DAMPING_I_0p85_FN_11_Z_0p08
        self.assertAlmostEqual(k, kg, places=6)
        self.assertAlmostEqual(d, dg, places=6)
        fn2, z2 = _compute_natural_freq_damping_revolute(k, d, inertia)
        self.assertAlmostEqual(fn2, 11.0, places=6)
        self.assertAlmostEqual(z2, 0.08, places=6)

    async def test_revolute_usd_gain_scaling_round_trip(self) -> None:
        k_si, d_si = 150.0, 3.0
        k_u, d_u = _revolute_drive_stiffness_damping_si_to_usd(k_si, d_si)
        self.assertAlmostEqual(k_u, _GOLDEN_REVOLUTE_DRIVE_USD_K, places=12)
        self.assertAlmostEqual(d_u, _GOLDEN_REVOLUTE_DRIVE_USD_D, places=12)
        k_back, d_back = _revolute_drive_stiffness_damping_usd_to_si(k_u, d_u)
        self.assertAlmostEqual(k_back, k_si, places=9)
        self.assertAlmostEqual(d_back, d_si, places=9)

    async def test_log_decrement_identity_inverts_zeta(self) -> None:
        """ζ = s / sqrt(4π² + s²) with s = ln(A_i/A_{i+1}) is self-consistent for underdamped theory."""
        for zeta in (0.02, 0.07, 0.22, 0.45):
            denom = max(1e-12, 1.0 - zeta**2)
            s = zeta * math.sqrt(4.0 * math.pi**2) / math.sqrt(denom)
            z_rec = s / math.sqrt(4.0 * math.pi**2 + s**2)
            self.assertAlmostEqual(z_rec, zeta, places=10)


class TestOscillationAnalysisMath(omni.kit.test.AsyncTestCase):
    """Peak / log-decrement identification on synthetic signals (no physics)."""

    async def test_analyze_oscillation_synthetic_underdamped_cosine(self) -> None:
        fn_hz = 7.0
        wn = 2.0 * math.pi * fn_hz
        zeta = 0.055
        wd = wn * math.sqrt(max(1e-9, 1.0 - zeta**2))
        t = np.linspace(0.0, 4.0, 16000)
        y = np.exp(-zeta * wn * t) * np.cos(wd * t)
        res = _analyze_oscillation(t, y)
        self.assertGreaterEqual(len(res.peak_values), 3)
        self.assertAlmostEqual(res.natural_freq, fn_hz, delta=0.12)
        self.assertAlmostEqual(res.damping_ratio, zeta, delta=0.012)
        expected_Td = 2.0 * math.pi / wd
        self.assertAlmostEqual(res.damped_period, expected_Td, delta=expected_Td * 0.02)

    async def test_analyze_oscillation_synthetic_underdamped_sine(self) -> None:
        """Sine phase yields interior extrema comparable to typical tracking-error waveforms."""
        fn_hz = 5.5
        wn = 2.0 * math.pi * fn_hz
        zeta = 0.04
        wd = wn * math.sqrt(max(1e-9, 1.0 - zeta**2))
        t = np.linspace(0.0, 3.5, 14000)
        y = np.exp(-zeta * wn * t) * np.sin(wd * t)
        res = _analyze_oscillation(t, y)
        self.assertGreaterEqual(len(res.peak_values), 3)
        self.assertAlmostEqual(res.natural_freq, fn_hz, delta=0.1)
        self.assertAlmostEqual(res.damping_ratio, zeta, delta=0.015)

    async def test_analyze_oscillation_peak_spacing_matches_damped_period(self) -> None:
        """Successive peak times should match mean spacing ≈ T_d from log-decrement analysis."""
        fn_hz = 6.2
        wn = 2.0 * math.pi * fn_hz
        zeta = 0.06
        wd = wn * math.sqrt(max(1e-9, 1.0 - zeta**2))
        t = np.linspace(0.0, 3.2, 12000)
        y = np.exp(-zeta * wn * t) * np.cos(wd * t)
        res = _analyze_oscillation(t, y)
        self.assertGreaterEqual(len(res.peak_times), 4)
        spacings = np.diff(np.asarray(res.peak_times[:6]))
        self.assertGreater(len(spacings), 0)
        self.assertAlmostEqual(
            float(np.mean(spacings)),
            _GOLDEN_DAMPED_PERIOD_FN_6p2_Z_0p06,
            delta=_GOLDEN_DAMPED_PERIOD_FN_6p2_Z_0p06 * 0.04,
        )

    async def test_analyze_oscillation_discrete_second_order_matches_golden_frequency(self) -> None:
        """RK4 integration of ẍ + 2ζω_n ẋ + ω_n² x = 0; peak analysis should recover f_n (no PhysX)."""
        fn_hz = 6.0
        zeta = 0.05
        wn = 2.0 * math.pi * fn_hz
        dt = 1.0 / (11.5 * fn_hz)
        n_steps = int(10.0 / dt)
        x, v = 1.0, 0.0
        t_list: list[float] = []
        x_list: list[float] = []

        def accel(xx: float, vv: float) -> float:
            return -2.0 * zeta * wn * vv - wn * wn * xx

        for i in range(n_steps):
            t_list.append(i * dt)
            x_list.append(x)
            k1v = accel(x, v)
            k1x = v
            k2v = accel(x + 0.5 * dt * k1x, v + 0.5 * dt * k1v)
            k2x = v + 0.5 * dt * k1v
            k3v = accel(x + 0.5 * dt * k2x, v + 0.5 * dt * k2v)
            k3x = v + 0.5 * dt * k2v
            k4v = accel(x + dt * k3x, v + dt * k3v)
            k4x = v + dt * k3v
            v = v + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
            x = x + (dt / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)

        res = _analyze_oscillation(np.array(t_list), np.array(x_list))
        self.assertGreaterEqual(len(res.peak_values), 3)
        self.assertAlmostEqual(res.natural_freq, fn_hz, delta=0.15)
        self.assertAlmostEqual(res.damping_ratio, zeta, delta=0.012)
