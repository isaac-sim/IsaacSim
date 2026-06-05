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

"""Teleop profile validation against USD stage state."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import omni.usd
from isaacsim.core.experimental.prims import Articulation
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .controllers.grasp import load_grasp_config
from .coordinate_utils import CoordinateSystem
from .markers_manager import MarkersManager
from .teleop_profiles import TeleopProfile
from .xr_anchor_manager import AnchorRotationMode

STAGE_STATE_NO_STAGE = "no_stage"
STAGE_STATE_LOADING = "loading"
STAGE_STATE_READY = "ready"

SEVERITY_ERROR = "error"
SEVERITY_WARNING = "warning"


@dataclass
class TeleopResolverIssue:
    """Structured issue surfaced by the teleop resolver."""

    source: str
    severity: str
    message: str


@dataclass
class TeleopResolutionReport:
    """Read-only readiness report for a teleop profile."""

    stage_state: str
    stage_message: str
    issues: list[TeleopResolverIssue] = field(default_factory=list)

    @property
    def error_count(self) -> int:
        """Return the number of errors in the report."""
        return sum(issue.severity == SEVERITY_ERROR for issue in self.issues)

    @property
    def warning_count(self) -> int:
        """Return the number of warnings in the report."""
        return sum(issue.severity == SEVERITY_WARNING for issue in self.issues)

    @property
    def ready(self) -> bool:
        """Return whether the current stage is ready and no errors remain."""
        return self.stage_state == STAGE_STATE_READY and self.error_count == 0


def resolve_teleop_profile(profile: TeleopProfile) -> TeleopResolutionReport:
    """Resolve a teleop profile against the current USD stage."""
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if stage is None:
        return TeleopResolutionReport(
            stage_state=STAGE_STATE_NO_STAGE,
            stage_message="No stage is open.",
        )

    _, _, remaining = usd_context.get_stage_loading_status()
    if remaining > 0:
        return TeleopResolutionReport(
            stage_state=STAGE_STATE_LOADING,
            stage_message="Stage is still loading.",
        )

    report = TeleopResolutionReport(
        stage_state=STAGE_STATE_READY,
        stage_message="Stage is ready.",
    )
    _validate_session_settings(profile, stage, report)
    _validate_floating_profile(profile, stage, report)
    _validate_ik_profile(profile, report)
    _validate_grasp_profile(profile, report)
    _validate_locomotion_profile(profile, stage, report)
    return report


def _validate_session_settings(profile: TeleopProfile, stage: Usd.Stage, report: TeleopResolutionReport) -> None:
    coordinate_system = profile.session.coordinate_system
    if coordinate_system not in {member.value for member in CoordinateSystem}:
        report.issues.append(
            TeleopResolverIssue(
                source="Session",
                severity=SEVERITY_ERROR,
                message=f"Unknown coordinate system '{coordinate_system}'.",
            )
        )

    rotation_mode = profile.session.anchor_rotation_mode
    if rotation_mode not in {member.value for member in AnchorRotationMode}:
        report.issues.append(
            TeleopResolverIssue(
                source="Session",
                severity=SEVERITY_ERROR,
                message=f"Unknown XR anchor rotation mode '{rotation_mode}'.",
            )
        )

    tracking_space_path = profile.session.tracking_space_path.strip()
    if not profile.session.tracking_space_enabled or not tracking_space_path:
        return

    if tracking_space_path.startswith(MarkersManager.MARKERS_SCOPE):
        report.issues.append(
            TeleopResolverIssue(
                source="Session Tracking Space",
                severity=SEVERITY_WARNING,
                message=(
                    f"Custom tracking space '{tracking_space_path}' targets Teleop markers. "
                    "Built-in tracking space will be used instead."
                ),
            )
        )
        return

    if not Sdf.Path.IsValidPathString(tracking_space_path):
        report.issues.append(
            TeleopResolverIssue(
                source="Session Tracking Space",
                severity=SEVERITY_WARNING,
                message=f"Invalid custom path '{tracking_space_path}'. Built-in tracking space will be used instead.",
            )
        )
        return

    path_report = TeleopResolutionReport(stage_state=report.stage_state, stage_message=report.stage_message)
    _validate_xformable_path(stage, source="Session Tracking Space", path=tracking_space_path, report=path_report)
    for issue in path_report.issues:
        report.issues.append(
            TeleopResolverIssue(
                source=issue.source,
                severity=SEVERITY_WARNING,
                message=f"{issue.message} Built-in tracking space will be used instead.",
            )
        )


def _validate_floating_profile(profile: TeleopProfile, stage: Usd.Stage, report: TeleopResolutionReport) -> None:
    for side_name, side in (("Left", profile.floating.left), ("Right", profile.floating.right)):
        prim_path = str(side.settings.get("prim_path", "")).strip()
        if not prim_path:
            if side.enabled:
                report.issues.append(
                    TeleopResolverIssue(
                        source=f"Floating {side_name}",
                        severity=SEVERITY_ERROR,
                        message="Controller is enabled but no rigid-body prim is configured.",
                    )
                )
            continue

        prim = _get_stage_prim(stage, prim_path, f"Floating {side_name}", report)
        if prim is None:
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            report.issues.append(
                TeleopResolverIssue(
                    source=f"Floating {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"'{prim_path}' must already have RigidBodyAPI.",
                )
            )
            continue

        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
        if kinematic_attr and bool(kinematic_attr.Get()):
            report.issues.append(
                TeleopResolverIssue(
                    source=f"Floating {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"'{prim_path}' is kinematic. Use a dynamic rigid body handle.",
                )
            )


def _validate_ik_profile(profile: TeleopProfile, report: TeleopResolutionReport) -> None:
    for side_name, side in (("Left", profile.ik.left), ("Right", profile.ik.right)):
        robot_path = str(side.settings.get("robot_path", "")).strip()
        if not robot_path:
            if side.enabled:
                report.issues.append(
                    TeleopResolverIssue(
                        source=f"IK {side_name}",
                        severity=SEVERITY_ERROR,
                        message="Controller is enabled but no articulation prim is configured.",
                    )
                )
            continue

        try:
            articulation_paths = Articulation.fetch_articulation_root_api_prim_paths(robot_path)
        except Exception as exc:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"IK {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"Prim not found: {exc}",
                )
            )
            continue

        articulation_path = articulation_paths[0] if articulation_paths else None
        if articulation_path is None:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"IK {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"No ArticulationRootAPI found at or under '{robot_path}'.",
                )
            )
            continue

        ee_link = str(side.settings.get("ee_link", "")).strip()
        if not ee_link:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"IK {side_name}",
                    severity=SEVERITY_WARNING,
                    message="Articulation is resolved but no EE link is selected yet.",
                )
            )
            continue

        try:
            robot = Articulation(articulation_path)
        except Exception as exc:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"IK {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"Invalid articulation: {exc}",
                )
            )
            continue

        if ee_link not in list(robot.link_names):
            report.issues.append(
                TeleopResolverIssue(
                    source=f"IK {side_name}",
                    severity=SEVERITY_ERROR,
                    message=f"EE link '{ee_link}' is not present on '{articulation_path}'.",
                )
            )


def _validate_grasp_profile(profile: TeleopProfile, report: TeleopResolutionReport) -> None:
    from .controllers.grasp import GraspController

    validator = GraspController()
    for side_name, side in (("Left", profile.grasp.left), ("Right", profile.grasp.right)):
        prim_path = side.prim_path.strip()
        has_config_path = bool(side.config_path.strip())
        if not prim_path:
            if side.enabled or has_config_path:
                report.issues.append(
                    TeleopResolverIssue(
                        source=f"Grasp {side_name}",
                        severity=SEVERITY_ERROR,
                        message="No grasp prim path is configured.",
                    )
                )
            continue

        validation = validator.validate_prim(prim_path)
        if not validation.is_valid:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"Grasp {side_name}",
                    severity=SEVERITY_ERROR,
                    message="; ".join(validation.errors) if validation.errors else "Invalid grasp prim.",
                )
            )

        if has_config_path:
            _, errors = load_grasp_config(side.config_path)
        else:
            errors = ["No grasp config is selected."]

        if errors:
            report.issues.append(
                TeleopResolverIssue(
                    source=f"Grasp {side_name}",
                    severity=SEVERITY_ERROR if side.enabled else SEVERITY_WARNING,
                    message="; ".join(errors),
                )
            )


def _validate_locomotion_profile(profile: TeleopProfile, stage: Usd.Stage, report: TeleopResolutionReport) -> None:
    prim_path = str(profile.locomotion.settings.get("prim_path", "")).strip()
    if not prim_path:
        if profile.locomotion.enabled:
            report.issues.append(
                TeleopResolverIssue(
                    source="Locomotion",
                    severity=SEVERITY_ERROR,
                    message="Controller is enabled but no base prim is configured.",
                )
            )
        return

    _validate_xformable_path(stage, "Locomotion", prim_path, report)


def _validate_xformable_path(stage: Usd.Stage, source: str, path: str, report: TeleopResolutionReport) -> None:
    if not path or not Sdf.Path.IsValidPathString(path):
        report.issues.append(
            TeleopResolverIssue(
                source=source,
                severity=SEVERITY_ERROR,
                message=f"Invalid prim path '{path}'.",
            )
        )
        return

    prim = _get_stage_prim(stage, path, source, report)
    if prim is None:
        return

    if not prim.IsA(UsdGeom.Xformable):
        report.issues.append(
            TeleopResolverIssue(
                source=source,
                severity=SEVERITY_ERROR,
                message=f"'{path}' is not Xformable.",
            )
        )


def _get_stage_prim(stage: Usd.Stage, path: str, source: str, report: TeleopResolutionReport) -> Any:
    if not Sdf.Path.IsValidPathString(path):
        report.issues.append(
            TeleopResolverIssue(
                source=source,
                severity=SEVERITY_ERROR,
                message=f"Invalid prim path '{path}'.",
            )
        )
        return None

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        report.issues.append(
            TeleopResolverIssue(
                source=source,
                severity=SEVERITY_ERROR,
                message=f"Prim not found: {path}",
            )
        )
        return None
    return prim
