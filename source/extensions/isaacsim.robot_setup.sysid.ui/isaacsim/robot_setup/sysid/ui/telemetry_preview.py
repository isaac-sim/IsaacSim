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

"""Viewport previews for no-physics telemetry pose samples."""

from __future__ import annotations

import contextlib
import re
from dataclasses import dataclass
from typing import Any

import carb
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from ..articulation_utils import find_articulation_root
from ..trajectory_csv import TrajectoryDataset
from ..usd_kinematics import (
    axis_vec,
    collect_usd_joint_edges,
    usd_joint_local_transform,
    usd_joint_motion_transform,
)

PREVIEW_ROOT_PATH = "/World/SysIdTelemetryPreview"
_PREVIEW_MARKER_RADIUS = 0.035
_PREVIEW_COLOR = Gf.Vec3f(0.2, 0.85, 0.95)
_PREVIEW_EE_COLOR = Gf.Vec3f(0.95, 0.72, 0.28)
_HIDDEN_SOURCE_PATH_ATTR = "sysid:preview:hiddenSourcePath"
_HIDDEN_SOURCE_HAD_VISIBILITY_ATTR = "sysid:preview:hiddenSourceHadVisibility"
_HIDDEN_SOURCE_VISIBILITY_ATTR = "sysid:preview:hiddenSourceVisibility"


def _preview_edit_context(stage: Any) -> Any:
    """Pin all preview authoring and teardown to the stage's session layer.

    Preview geometry and the source-prim visibility override are transient
    viewport decoration, not part of the asset. Authoring them on the session
    layer -- rather than whatever edit target happens to be active -- keeps them
    out of the saved asset and, crucially, makes teardown deterministic:
    :func:`clear_gt_preview` removes the preview namespace and restores
    visibility on the *same* layer they were authored to, independent of the
    user's current authoring layer.

    Without this pin, ``Usd.Stage.RemovePrim`` and ``visibility.Clear()`` (both
    edit-target scoped) act only on the currently active layer. On a
    multi-layer/variant asset the edit target commonly differs between
    preview-open and preview-close, so teardown targets the wrong layer, orphans
    ``/World/SysIdTelemetryPreview`` on the stage, and strands the source prim
    invisible. Pinning the session layer makes the behavior identical for a
    simple single-layer asset and a composed one.

    Args:
        stage: USD stage to operate on.

    Returns:
        A context manager that redirects authoring to the session layer, or a
        no-op context when no stage/session layer is available.
    """
    session = stage.GetSessionLayer() if stage is not None else None
    if session is None:
        return contextlib.nullcontext()
    return Usd.EditContext(stage, session)


@dataclass(frozen=True)
class GtLinkPose:
    """One ground-truth link pose extracted from telemetry."""

    name: str
    position: tuple[float, float, float]
    orientation: tuple[float, ...] | None = None
    body_path: str = ""


@dataclass(frozen=True)
class TelemetryPreviewResult:
    """Result of authoring preview geometry into the USD stage."""

    previewed: bool
    marker_count: int
    sample_index: int
    sample_time: float
    source: str
    message: str
    mesh_count: int = 0


@dataclass(frozen=True)
class _FkJointEdge:
    joint_path: str
    joint_index: int
    joint_type: str
    axis: str
    body0_path: str
    body1_path: str
    local0: Gf.Matrix4d
    local1: Gf.Matrix4d


@dataclass(frozen=True)
class _MeshPreviewBuildResult:
    meshed_indices: set[int]
    fallback_reasons: dict[int, str]


def clear_gt_preview(stage: Any, *, root_path: str = PREVIEW_ROOT_PATH) -> None:
    """Remove the telemetry preview namespace if it exists.

    Args:
        stage: USD stage to operate on.
        root_path: Root path value.
    """
    if stage is None:
        return
    with _preview_edit_context(stage):
        _restore_hidden_preview_source(stage, root_path)
        if stage.GetPrimAtPath(root_path).IsValid():
            stage.RemovePrim(root_path)


def preview_gt_link_positions(
    stage: Any,
    trajectory: TrajectoryDataset,
    sample_time: float,
    *,
    robot_path: str = "",
    root_path: str = PREVIEW_ROOT_PATH,
    hide_source_prim: bool = False,
) -> TelemetryPreviewResult:
    """Draw GT link/world-pose preview geometry for the nearest trajectory sample.

    This intentionally does not pose a USD articulation or step physics. It only
    authors transient mesh references or marker prims for link/world positions
    recorded in telemetry. All authoring is pinned to the session layer (see
    :func:`_preview_edit_context`) so it never lands in the saved asset and so
    :func:`clear_gt_preview` can tear it down deterministically.

    Args:
        stage: USD stage to operate on.
        trajectory: Trajectory value.
        sample_time: Sample time value.
        robot_path: Robot path value.
        root_path: Root path value.
        hide_source_prim: Hide source prim value.

    Returns:
        The resulting value.
    """
    if stage is None:
        return _result(False, 0, -1, 0.0, "", "Preview error: no USD stage is open.")
    with _preview_edit_context(stage):
        return _preview_gt_link_positions_impl(
            stage,
            trajectory,
            sample_time,
            robot_path=robot_path,
            root_path=root_path,
            hide_source_prim=hide_source_prim,
        )


def _preview_gt_link_positions_impl(
    stage: Any,
    trajectory: TrajectoryDataset,
    sample_time: float,
    *,
    robot_path: str = "",
    root_path: str = PREVIEW_ROOT_PATH,
    hide_source_prim: bool = False,
) -> TelemetryPreviewResult:
    """Author preview geometry for :func:`preview_gt_link_positions`.

    Runs inside the session-layer edit context established by the public entry
    point; all USD authoring here is therefore redirected to the session layer.

    Args:
        stage: USD stage to operate on.
        trajectory: Loaded telemetry trajectory.
        sample_time: Sample time value.
        robot_path: Robot path value.
        root_path: Root path value.
        hide_source_prim: Hide source prim value.

    Returns:
        The resulting value.
    """
    _restore_hidden_preview_source(stage, root_path)
    if trajectory is None or trajectory.times.shape[0] == 0:
        return _result(False, 0, -1, 0.0, "", "Preview error: load telemetry first.")

    sample_index = _nearest_sample_index(trajectory.times, sample_time)
    poses, source = _extract_gt_link_poses(trajectory, sample_index)
    unavailable_message = ""
    if not poses:
        poses, source, unavailable_message = _poses_from_joint_positions(stage, trajectory, sample_index, robot_path)
    elif _poses_need_orientation(poses):
        poses, orientation_source = _merge_fk_orientations_for_position_only_poses(
            stage,
            trajectory,
            sample_index,
            robot_path,
            poses,
        )
        if orientation_source:
            source = f"{source}+{orientation_source}"
    sample = float(trajectory.times[sample_index])

    if not poses:
        previous_mode, previous_reason = _preview_state(stage, root_path)
        cause = (
            unavailable_message
            or "Preview unavailable: this telemetry has joint positions but no GT link/world pose channel."
        )
        if previous_mode:
            _log_preview_mode_change(previous_mode, previous_reason, "unavailable", cause, 0, 0)
        clear_gt_preview(stage, root_path=root_path)
        return _result(
            False,
            0,
            sample_index,
            sample,
            source or "joint_positions",
            cause,
        )

    previous_mode, previous_reason = _preview_state(stage, root_path)
    root_prim = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    mesh_result = _define_mesh_preview(stage, root_path, poses, robot_path)
    meshed_indices = mesh_result.meshed_indices
    marker_indices = [index for index in range(len(poses)) if index not in meshed_indices]

    if meshed_indices:
        markers_prim = stage.GetPrimAtPath(f"{root_path}/markers")
        if markers_prim.IsValid() and not marker_indices:
            stage.RemovePrim(markers_prim.GetPath())
    if marker_indices:
        markers_path = f"{root_path}/markers"
        markers_prim = stage.GetPrimAtPath(markers_path)
        if markers_prim.IsValid():
            stage.RemovePrim(markers_prim.GetPath())
        UsdGeom.Xform.Define(stage, markers_path)
        for index in marker_indices:
            _define_marker(stage, markers_path, poses[index], index, source)
    else:
        markers_prim = stage.GetPrimAtPath(f"{root_path}/markers")
        if markers_prim.IsValid():
            stage.RemovePrim(markers_prim.GetPath())

    if not meshed_indices:
        meshes_prim = stage.GetPrimAtPath(f"{root_path}/meshes")
        if meshes_prim.IsValid():
            stage.RemovePrim(meshes_prim.GetPath())

    if meshed_indices and marker_indices:
        mode = "mixed"
        reason = (
            f"{len(meshed_indices)}/{len(poses)} link poses mapped to USD visual meshes; "
            f"marker fallback for {_fallback_reason_summary(poses, mesh_result.fallback_reasons, marker_indices)}"
        )
        message = (
            f"Previewed {len(meshed_indices)} USD mesh link(s) and {len(marker_indices)} marker(s) "
            f"from {source} at t={sample:.3f}s."
        )
    elif meshed_indices:
        mode = "mesh"
        reason = f"all {len(meshed_indices)} link poses mapped to visible USD visual geometry"
        message = f"Previewed {len(meshed_indices)} USD mesh link(s) from {source} at t={sample:.3f}s."
    else:
        mode = "markers"
        reason = _fallback_reason_summary(poses, mesh_result.fallback_reasons, marker_indices)
        message = f"Previewed {len(poses)} GT marker(s) from {source} at t={sample:.3f}s."

    _set_preview_state(root_prim, mode, reason)
    _log_preview_mode_change(previous_mode, previous_reason, mode, reason, len(meshed_indices), len(marker_indices))
    if hide_source_prim:
        _hide_preview_source_prim(stage, root_prim, robot_path, root_path)
    return _result(
        True,
        len(marker_indices),
        sample_index,
        sample,
        source,
        message,
        mesh_count=len(meshed_indices),
    )


def _result(
    previewed: bool,
    marker_count: int,
    sample_index: int,
    sample_time: float,
    source: str,
    message: str,
    *,
    mesh_count: int = 0,
) -> TelemetryPreviewResult:
    return TelemetryPreviewResult(
        previewed=previewed,
        marker_count=marker_count,
        sample_index=sample_index,
        sample_time=sample_time,
        source=source,
        message=message,
        mesh_count=mesh_count,
    )


def _nearest_sample_index(times: np.ndarray, sample_time: float) -> int:
    times = np.asarray(times, dtype=np.float64).reshape(-1)
    if times.shape[0] == 0:
        return -1
    return int(np.argmin(np.abs(times - float(sample_time))))


def _extract_gt_link_poses(trajectory: TrajectoryDataset, sample_index: int) -> tuple[list[GtLinkPose], str]:
    extra = trajectory.metadata.extra if trajectory.metadata is not None else {}
    for key in ("link_poses", "gt_link_poses"):
        poses = _poses_from_metadata(extra.get(key), sample_index, _link_names(extra), source=key)
        if poses:
            return poses, key

    for key in ("link_positions", "gt_link_positions"):
        poses = _poses_from_metadata(extra.get(key), sample_index, _link_names(extra), source=key)
        if poses:
            return poses, key

    ee_pose = trajectory.end_effector_poses
    if ee_pose is not None and sample_index >= 0 and sample_index < ee_pose.shape[0] and ee_pose.shape[1] >= 3:
        row = np.asarray(ee_pose[sample_index], dtype=np.float64)
        orientation = tuple(float(v) for v in row[3:]) if row.shape[0] > 3 else None
        return [GtLinkPose("end_effector", _vec3(row[:3]), orientation)], "end_effector_poses"

    return [], ""


def _poses_need_orientation(poses: list[GtLinkPose]) -> bool:
    return any(pose.orientation is None for pose in poses)


def _merge_fk_orientations_for_position_only_poses(
    stage: Usd.Stage,
    trajectory: TrajectoryDataset,
    sample_index: int,
    robot_path: str,
    poses: list[GtLinkPose],
) -> tuple[list[GtLinkPose], str]:
    if not robot_path or trajectory.positions is None:
        return poses, ""

    fk_poses, fk_source, _error = _poses_from_joint_positions(stage, trajectory, sample_index, robot_path)
    if not fk_poses:
        return poses, ""

    body_name_map, _body_map_reason = _robot_body_name_map(stage, robot_path)
    fk_by_body = {pose.body_path: pose for pose in fk_poses if pose.body_path}
    fk_by_name = {_link_key(pose.name): pose for pose in fk_poses}

    merged: list[GtLinkPose] = []
    used_fk = False
    for pose in poses:
        body_path = _resolve_pose_body_path(stage, pose, body_name_map)
        fk_pose = fk_by_body.get(body_path) or fk_by_name.get(_link_key(pose.name))
        if pose.orientation is None and fk_pose is not None and fk_pose.orientation is not None:
            merged.append(
                GtLinkPose(
                    name=pose.name,
                    position=pose.position,
                    orientation=fk_pose.orientation,
                    body_path=body_path or pose.body_path or fk_pose.body_path,
                )
            )
            used_fk = True
        elif body_path and not pose.body_path:
            merged.append(
                GtLinkPose(
                    name=pose.name,
                    position=pose.position,
                    orientation=pose.orientation,
                    body_path=body_path,
                )
            )
        else:
            merged.append(pose)

    return merged, "joint_positions_fk_orientation" if used_fk and fk_source else ""


def _poses_from_joint_positions(
    stage: Usd.Stage,
    trajectory: TrajectoryDataset,
    sample_index: int,
    robot_path: str,
) -> tuple[list[GtLinkPose], str, str]:
    if not robot_path:
        return (
            [],
            "joint_positions",
            "Preview unavailable: joint-only telemetry needs a Robot prim path.",
        )
    if sample_index < 0 or sample_index >= trajectory.positions.shape[0]:
        return [], "joint_positions", "Preview unavailable: sample index is outside telemetry."

    edges, error = _collect_fk_joint_edges(stage, robot_path)
    if error:
        return [], "joint_positions", error
    if not edges:
        return [], "joint_positions", "Preview unavailable: no revolute/prismatic joints found under Robot prim path."

    q = np.asarray(trajectory.positions[sample_index], dtype=np.float64).reshape(-1)
    if q.shape[0] == 0:
        return [], "joint_positions", "Preview unavailable: telemetry has no joint position columns."
    edges = edges[: q.shape[0]]

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_order = _fk_body_order(edges)
    initial_world = {
        body_path: Gf.Matrix4d(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body_path)))
        for body_path in body_order
        if stage.GetPrimAtPath(body_path).IsValid()
    }
    child_paths = {edge.body1_path for edge in edges}
    root_bodies = [path for path in body_order if path not in child_paths]
    computed = {path: initial_world[path] for path in root_bodies if path in initial_world}
    if not computed and body_order and body_order[0] in initial_world:
        computed[body_order[0]] = initial_world[body_order[0]]

    pending = list(edges)
    while pending:
        next_pending: list[_FkJointEdge] = []
        progressed = False
        for edge in pending:
            parent_world = computed.get(edge.body0_path)
            if parent_world is None:
                parent_world = initial_world.get(edge.body0_path)
            if parent_world is None:
                next_pending.append(edge)
                continue
            motion = _joint_motion_transform(edge, q[edge.joint_index])
            computed[edge.body1_path] = edge.local1.GetInverse() * motion * edge.local0 * parent_world
            progressed = True
        if not progressed:
            break
        pending = next_pending

    poses: list[GtLinkPose] = []
    for body_path in body_order:
        world = computed.get(body_path)
        if world is None:
            continue
        rot = world.ExtractRotationQuat()
        imag = rot.GetImaginary()
        poses.append(
            GtLinkPose(
                _path_name(body_path),
                _vec3(np.asarray(world.ExtractTranslation(), dtype=np.float64)),
                (float(rot.GetReal()), float(imag[0]), float(imag[1]), float(imag[2])),
                body_path,
            )
        )

    if not poses:
        return [], "joint_positions", "Preview unavailable: could not solve robot FK from the USD joint tree."
    return poses, "joint_positions_fk", ""


def _collect_fk_joint_edges(stage: Usd.Stage, robot_path: str) -> tuple[list[_FkJointEdge], str]:
    edges, message = collect_usd_joint_edges(stage, robot_path)
    if message:
        message = f"Preview unavailable: {message}"
    return edges, message


def _collect_joint_edges_from_prim(root_prim: Usd.Prim, *, body_root_path: str = "") -> list[_FkJointEdge]:
    edges: list[_FkJointEdge] = []
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        joint_type = ""
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint_type = "revolute"
            joint_schema = UsdPhysics.RevoluteJoint(prim)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            joint_type = "prismatic"
            joint_schema = UsdPhysics.PrismaticJoint(prim)
        else:
            continue

        base_joint = UsdPhysics.Joint(prim)
        body0 = base_joint.GetBody0Rel().GetTargets()
        body1 = base_joint.GetBody1Rel().GetTargets()
        if body0 and body1:
            body0_path = str(body0[0])
            body1_path = str(body1[0])
            if body_root_path and not (
                _path_is_under(body0_path, body_root_path) and _path_is_under(body1_path, body_root_path)
            ):
                continue
            axis = joint_schema.GetAxisAttr().Get() or "X"
            edges.append(
                _FkJointEdge(
                    joint_path=str(prim.GetPath()),
                    joint_index=len(edges),
                    joint_type=joint_type,
                    axis=str(axis).upper(),
                    body0_path=body0_path,
                    body1_path=body1_path,
                    local0=_joint_local_transform(base_joint, 0),
                    local1=_joint_local_transform(base_joint, 1),
                )
            )
    return edges


def _path_is_under(path: str, root_path: str) -> bool:
    path = str(path).rstrip("/")
    root_path = str(root_path).rstrip("/")
    return path == root_path or path.startswith(f"{root_path}/")


def _fk_body_order(edges: list[_FkJointEdge]) -> list[str]:
    body_order: list[str] = []
    for edge in edges:
        for path in (edge.body0_path, edge.body1_path):
            if path not in body_order:
                body_order.append(path)
    return body_order


def _joint_local_transform(joint: UsdPhysics.Joint, body_index: int) -> Gf.Matrix4d:
    return usd_joint_local_transform(joint, body_index)


def _joint_motion_transform(edge: _FkJointEdge, q_value: float) -> Gf.Matrix4d:
    return usd_joint_motion_transform(edge, q_value)


def _axis_vec(axis: str) -> Gf.Vec3d:
    return axis_vec(axis)


def _poses_from_metadata(
    raw: Any,
    sample_index: int,
    names: list[str],
    *,
    source: str,
) -> list[GtLinkPose]:
    if raw is None:
        return []
    try:
        arr = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError):
        return []
    if arr.ndim != 3 or sample_index < 0 or sample_index >= arr.shape[0] or arr.shape[2] < 3:
        return []

    poses: list[GtLinkPose] = []
    for link_index, row in enumerate(arr[sample_index]):
        name = names[link_index] if link_index < len(names) else f"link_{link_index}"
        orientation = tuple(float(v) for v in row[3:]) if row.shape[0] > 3 and source.endswith("poses") else None
        poses.append(GtLinkPose(name, _vec3(row[:3]), orientation))
    return poses


def _link_names(extra: dict[str, Any]) -> list[str]:
    raw = extra.get("link_names") or extra.get("gt_link_names")
    if not isinstance(raw, (list, tuple)):
        return []
    return [str(item) for item in raw]


def _vec3(values: np.ndarray) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _path_name(path: str) -> str:
    return path.rstrip("/").split("/")[-1] or "link"


def _preview_state(stage: Usd.Stage, root_path: str) -> tuple[str, str]:
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return "", ""
    mode_attr = root_prim.GetAttribute("sysid:preview:mode")
    reason_attr = root_prim.GetAttribute("sysid:preview:reason")
    mode = str(mode_attr.Get() or "") if mode_attr else ""
    reason = str(reason_attr.Get() or "") if reason_attr else ""
    return mode, reason


def _set_preview_state(root_prim: Usd.Prim, mode: str, reason: str) -> None:
    root_prim.CreateAttribute("sysid:preview:mode", Sdf.ValueTypeNames.String).Set(mode)
    root_prim.CreateAttribute("sysid:preview:reason", Sdf.ValueTypeNames.String).Set(reason)


def _restore_hidden_preview_source(stage: Usd.Stage, root_path: str) -> None:
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return

    path_attr = root_prim.GetAttribute(_HIDDEN_SOURCE_PATH_ATTR)
    source_path = str(path_attr.Get() or "") if path_attr else ""
    if not source_path:
        return

    source_prim = stage.GetPrimAtPath(source_path)
    if source_prim and source_prim.IsValid() and source_prim.IsA(UsdGeom.Imageable):
        imageable = UsdGeom.Imageable(source_prim)
        visibility_attr = imageable.GetVisibilityAttr()
        had_visibility_attr = root_prim.GetAttribute(_HIDDEN_SOURCE_HAD_VISIBILITY_ATTR)
        visibility_value_attr = root_prim.GetAttribute(_HIDDEN_SOURCE_VISIBILITY_ATTR)
        had_visibility = bool(had_visibility_attr.Get()) if had_visibility_attr else False
        previous_visibility = str(visibility_value_attr.Get() or "") if visibility_value_attr else ""
        if had_visibility:
            visibility_attr.Set(previous_visibility or UsdGeom.Tokens.inherited)
        else:
            visibility_attr.Clear()

    for attr_name in (
        _HIDDEN_SOURCE_PATH_ATTR,
        _HIDDEN_SOURCE_HAD_VISIBILITY_ATTR,
        _HIDDEN_SOURCE_VISIBILITY_ATTR,
    ):
        attr = root_prim.GetAttribute(attr_name)
        if attr:
            attr.Clear()


def _hide_preview_source_prim(stage: Usd.Stage, root_prim: Usd.Prim, robot_path: str, root_path: str) -> None:
    source_path = _preview_source_prim_path(stage, robot_path, root_path)
    if not source_path:
        return

    source_prim = stage.GetPrimAtPath(source_path)
    if not source_prim or not source_prim.IsValid() or not source_prim.IsA(UsdGeom.Imageable):
        return

    imageable = UsdGeom.Imageable(source_prim)
    visibility_attr = imageable.GetVisibilityAttr()
    had_visibility = bool(visibility_attr.HasAuthoredValueOpinion())
    previous_visibility = str(visibility_attr.Get() or "") if had_visibility else ""

    root_prim.CreateAttribute(_HIDDEN_SOURCE_PATH_ATTR, Sdf.ValueTypeNames.String).Set(source_path)
    root_prim.CreateAttribute(_HIDDEN_SOURCE_HAD_VISIBILITY_ATTR, Sdf.ValueTypeNames.Bool).Set(had_visibility)
    root_prim.CreateAttribute(_HIDDEN_SOURCE_VISIBILITY_ATTR, Sdf.ValueTypeNames.String).Set(previous_visibility)
    visibility_attr.Set(UsdGeom.Tokens.invisible)


def _preview_source_prim_path(stage: Usd.Stage, robot_path: str, root_path: str) -> str:
    if not robot_path:
        return ""

    candidates: list[str] = []
    try:
        robot_prim = stage.GetPrimAtPath(robot_path)
    except Exception:
        robot_prim = None
    if robot_prim and robot_prim.IsValid():
        candidates.append(str(robot_prim.GetPath()))

    articulation_root = find_articulation_root(stage, robot_path)
    if articulation_root and articulation_root not in candidates:
        candidates.append(articulation_root)

    for candidate in candidates:
        if _can_hide_preview_source_path(candidate, root_path):
            return candidate
    return ""


def _can_hide_preview_source_path(source_path: str, root_path: str) -> bool:
    source_path = str(source_path).rstrip("/")
    root_path = str(root_path).rstrip("/")
    if not source_path or source_path == "/":
        return False
    if source_path == root_path:
        return False
    if _path_is_under(root_path, source_path):
        return False
    if _path_is_under(source_path, root_path):
        return False
    return True


def _log_preview_mode_change(
    previous_mode: str,
    previous_reason: str,
    mode: str,
    reason: str,
    mesh_count: int,
    marker_count: int,
) -> None:
    if previous_mode == mode and previous_reason == reason:
        return
    transition = f"{previous_mode or 'none'} -> {mode}"
    counts = f"meshes={mesh_count}, markers={marker_count}"
    message = f"SysId preview: {transition} ({counts}). Cause: {reason}"
    if mode in ("markers", "mixed", "unavailable"):
        carb.log_warn(message)
    else:
        carb.log_info(message)


def _fallback_reason_summary(
    poses: list[GtLinkPose],
    fallback_reasons: dict[int, str],
    marker_indices: list[int],
) -> str:
    if not marker_indices:
        return "no marker fallback"
    parts: list[str] = []
    for index in marker_indices[:3]:
        pose_name = poses[index].name or f"link_{index}"
        reason = fallback_reasons.get(index, "mesh preview unavailable")
        parts.append(f"{pose_name}: {reason}")
    extra = len(marker_indices) - len(parts)
    if extra > 0:
        parts.append(f"+{extra} more")
    return "; ".join(parts)


def _define_mesh_preview(
    stage: Usd.Stage,
    root_path: str,
    poses: list[GtLinkPose],
    robot_path: str,
) -> _MeshPreviewBuildResult:
    fallback_reasons: dict[int, str] = {}
    if not robot_path:
        return _MeshPreviewBuildResult(
            set(),
            {index: "Robot prim path is empty" for index, _pose in enumerate(poses)},
        )

    body_name_map, body_map_reason = _robot_body_name_map(stage, robot_path)

    meshes_path = f"{root_path}/meshes"
    UsdGeom.Xform.Define(stage, meshes_path)
    used_link_paths: set[str] = set()
    meshed_indices: set[int] = set()

    for index, pose in enumerate(poses):
        body_path = _resolve_pose_body_path(stage, pose, body_name_map)
        if not body_path:
            fallback_reasons[index] = body_map_reason or f"no USD body prim matched pose '{pose.name}'"
            continue
        visual_paths = _visual_reference_roots(stage, body_path)
        if not visual_paths:
            fallback_reasons[index] = f"matched body {body_path} has no visible renderable visual geometry"
            continue

        link_path = f"{meshes_path}/{_safe_link_prim_name(pose.name, index)}"
        used_link_paths.add(link_path)
        link_prim = stage.GetPrimAtPath(link_path)
        if not _mesh_link_matches(link_prim, body_path, visual_paths):
            if link_prim.IsValid():
                stage.RemovePrim(link_prim.GetPath())
            link_prim = UsdGeom.Xform.Define(stage, link_path).GetPrim()
            link_prim.CreateAttribute("sysid:preview:name", Sdf.ValueTypeNames.String).Set(pose.name)
            link_prim.CreateAttribute("sysid:preview:bodyPath", Sdf.ValueTypeNames.String).Set(body_path)
            link_prim.CreateAttribute("sysid:preview:visualCount", Sdf.ValueTypeNames.Int).Set(len(visual_paths))
            for visual_index, visual_path in enumerate(visual_paths):
                _define_visual_reference(stage, link_path, visual_index, visual_path)

        _set_link_pose(stage.GetPrimAtPath(link_path), pose)
        meshed_indices.add(index)

    meshes_prim = stage.GetPrimAtPath(meshes_path)
    if meshes_prim.IsValid():
        for child in list(meshes_prim.GetChildren()):
            if str(child.GetPath()) not in used_link_paths:
                stage.RemovePrim(child.GetPath())

    return _MeshPreviewBuildResult(meshed_indices, fallback_reasons)


def _robot_body_name_map(stage: Usd.Stage, robot_path: str) -> tuple[dict[str, str], str]:
    root_path = find_articulation_root(stage, robot_path) or robot_path
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return {}, f"Robot prim path is invalid: {robot_path}"

    body_paths: list[str] = []
    stack = [root_prim]
    while stack:
        prim = stack.pop(0)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.MassAPI):
            body_paths.append(str(prim.GetPath()))
        stack.extend(list(prim.GetChildren()))
    if not body_paths:
        return {}, f"no rigid/mass body prims found under Robot prim path: {root_path}"

    body_map: dict[str, str] = {}
    for body_path in body_paths:
        for key in (_path_name(body_path), body_path):
            normalized = _link_key(key)
            if normalized and normalized not in body_map:
                body_map[normalized] = body_path
    return body_map, ""


def _resolve_pose_body_path(stage: Usd.Stage, pose: GtLinkPose, body_name_map: dict[str, str]) -> str:
    if pose.body_path and stage.GetPrimAtPath(pose.body_path).IsValid():
        return pose.body_path
    if pose.name and stage.GetPrimAtPath(pose.name).IsValid():
        return pose.name
    return body_name_map.get(_link_key(pose.name), "")


def _visual_reference_roots(stage: Usd.Stage, body_path: str) -> list[str]:
    """Collect the renderable visual geometry roots under a rigid-body prim.

    Args:
        stage: USD stage to operate on.
        body_path: Body path value.

    Returns:
        The resulting value.

    Classification precedence, most authoritative first:
      1. USD signals (``_is_render_excluded``): invisible or proxy/guide purpose -> skip.
      2. Physics topology (``_is_physics_body``): a nested rigid body owns its own
         visuals, so we never descend into it for the parent link.
      3. Name-based fallbacks (``_is_collision_like`` / ``_is_visual_group``) for assets
         that only separate visual from collision geometry by scope/prim naming.
    """
    body_prim = stage.GetPrimAtPath(body_path)
    if not body_prim or not body_prim.IsValid():
        return []

    roots: list[str] = []

    def visit(prim: Usd.Prim) -> None:
        """Handle visit.

        Args:
            prim: Prim value.
        """
        if prim != body_prim and _is_physics_body(prim):
            return
        if _is_render_excluded(prim) or _is_collision_like(prim):
            return
        if _is_renderable(prim):
            roots.append(str(prim.GetPath()))
            return

        if (
            prim != body_prim
            and _has_renderable_descendant(prim, root_body_prim=body_prim)
            and not _has_collision_descendant(prim, root_body_prim=body_prim)
        ):
            roots.append(str(prim.GetPath()))
            return

        if (
            prim != body_prim
            and _is_visual_group(prim)
            and not _has_collision_descendant(prim, root_body_prim=body_prim)
        ):
            roots.append(str(prim.GetPath()))
            return

        for child in prim.GetFilteredChildren(Usd.TraverseInstanceProxies()):
            visit(child)

    for child in body_prim.GetFilteredChildren(Usd.TraverseInstanceProxies()):
        visit(child)

    if (
        not roots
        and _is_renderable(body_prim)
        and not _is_render_excluded(body_prim)
        and not _is_collision_like(body_prim)
    ):
        roots.append(body_path)
    if not roots:
        roots.extend(_sibling_visual_reference_roots(body_prim))
    return _dedupe_paths(roots)


def _sibling_visual_reference_roots(body_prim: Usd.Prim) -> list[str]:
    parent = body_prim.GetParent()
    if not parent or not parent.IsValid():
        return []

    body_name = _path_name(str(body_prim.GetPath()))
    body_key = _link_key(body_name)
    if not body_key:
        return []

    roots: list[str] = []
    stack = list(parent.GetFilteredChildren(Usd.TraverseInstanceProxies()))
    while stack:
        prim = stack.pop(0)
        if prim == body_prim or _is_physics_body(prim):
            continue
        if _is_render_excluded(prim) or _is_collision_like(prim):
            continue

        if _visual_candidate_matches_body(prim, body_name, body_key):
            if _is_renderable(prim):
                roots.append(str(prim.GetPath()))
                continue
            if _has_renderable_descendant(prim) and not _has_collision_descendant(prim):
                roots.append(str(prim.GetPath()))
                continue

        stack.extend(list(prim.GetFilteredChildren(Usd.TraverseInstanceProxies())))
    return roots


def _is_visual_group(prim: Usd.Prim) -> bool:
    name = _link_key(_path_name(str(prim.GetPath())))
    return name in (
        "visual",
        "visuals",
        "visualmesh",
        "visualmeshes",
        "geometry",
        "geometries",
        "geom",
        "mesh",
        "meshes",
        "render",
        "renderable",
        "renderables",
    )


def _visual_candidate_matches_body(prim: Usd.Prim, body_name: str, body_key: str) -> bool:
    for token in str(prim.GetPath()).split("/"):
        if _raw_token_matches_body(token, body_name):
            return True
        token_key = _link_key(token)
        if _key_matches_visual_body_name(token_key, body_key):
            return True
    return False


def _raw_token_matches_body(token: str, body_name: str) -> bool:
    lowered = str(token).lower()
    body = str(body_name).lower()
    if not lowered or not body:
        return False

    start = 0
    while True:
        index = lowered.find(body, start)
        if index < 0:
            return False
        before_index = index - 1
        after_index = index + len(body)
        before_ok = before_index < 0 or not lowered[before_index].isalnum()
        after_ok = after_index >= len(lowered) or not lowered[after_index].isalnum()
        if before_ok and after_ok:
            return True
        start = index + 1


def _key_matches_visual_body_name(token_key: str, body_key: str) -> bool:
    if not token_key or not body_key:
        return False
    if token_key == body_key:
        return True

    visual_affixes = ("visual", "visuals", "vis", "mesh", "meshes", "geometry", "geom", "render", "renderable", "shape")
    if token_key.startswith(body_key):
        suffix = token_key[len(body_key) :]
        return any(suffix.startswith(affix) for affix in visual_affixes)
    if token_key.endswith(body_key):
        prefix = token_key[: -len(body_key)]
        return any(prefix.endswith(affix) for affix in visual_affixes)
    return False


def _dedupe_paths(paths: list[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for path in paths:
        if path in seen:
            continue
        seen.add(path)
        deduped.append(path)
    return deduped


def _has_renderable_descendant(prim: Usd.Prim, *, root_body_prim: Usd.Prim | None = None) -> bool:
    if root_body_prim is not None and prim != root_body_prim and _is_physics_body(prim):
        return False
    if _is_render_excluded(prim) or _is_collision_like(prim):
        return False
    if _is_renderable(prim):
        return True
    return any(
        _has_renderable_descendant(child, root_body_prim=root_body_prim)
        for child in prim.GetFilteredChildren(Usd.TraverseInstanceProxies())
    )


def _has_collision_descendant(prim: Usd.Prim, *, root_body_prim: Usd.Prim | None = None) -> bool:
    if root_body_prim is not None and prim != root_body_prim and _is_physics_body(prim):
        return False
    if _is_collision_like(prim):
        return True
    return any(
        _has_collision_descendant(child, root_body_prim=root_body_prim)
        for child in prim.GetFilteredChildren(Usd.TraverseInstanceProxies())
    )


def _is_renderable(prim: Usd.Prim) -> bool:
    return prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.Boundable)


def _is_visible(prim: Usd.Prim) -> bool:
    if not prim.IsA(UsdGeom.Imageable):
        return True
    imageable = UsdGeom.Imageable(prim)
    visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
    return visibility != UsdGeom.Tokens.invisible


_NON_RENDER_PURPOSES = (UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide)


def _imageable_purpose(prim: Usd.Prim) -> Any:
    if not prim.IsA(UsdGeom.Imageable):
        return UsdGeom.Tokens.default_
    return UsdGeom.Imageable(prim).ComputePurpose()


def _is_render_excluded(prim: Usd.Prim) -> bool:
    """USD-native "never rendered" test, independent of prim naming.

    Honours the same signals the viewport does: an invisible prim, or one whose
    resolved purpose is ``proxy``/``guide`` (the conventional tags for collision and
    helper geometry), is not part of the visible robot and is skipped. This is the
    authoritative path; ``_is_collision_like`` is only a name-based fallback for assets
    that separate collision by scope name without authoring purpose/visibility.

    Args:
        prim: Prim value.

    Returns:
        The resulting value.
    """
    if not _is_visible(prim):
        return True
    return _imageable_purpose(prim) in _NON_RENDER_PURPOSES


def _is_collision_like(prim: Usd.Prim) -> bool:
    lowered = str(prim.GetPath()).lower()
    return any(token in lowered for token in ("/collision", "/collisions", "/collider", "/colliders"))


def _is_physics_body(prim: Usd.Prim) -> bool:
    return prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.MassAPI)


def _mesh_link_matches(link_prim: Usd.Prim, body_path: str, visual_paths: list[str]) -> bool:
    if not link_prim or not link_prim.IsValid():
        return False
    body_attr = link_prim.GetAttribute("sysid:preview:bodyPath")
    visual_count_attr = link_prim.GetAttribute("sysid:preview:visualCount")
    if not body_attr or body_attr.Get() != body_path:
        return False
    if not visual_count_attr or int(visual_count_attr.Get() or 0) != len(visual_paths):
        return False
    for visual_index, visual_path in enumerate(visual_paths):
        child_path = _visual_reference_path(str(link_prim.GetPath()), visual_index, visual_path)
        child = link_prim.GetStage().GetPrimAtPath(child_path)
        attr = child.GetAttribute("sysid:preview:sourceVisualPath") if child.IsValid() else None
        if not attr or attr.Get() != visual_path:
            return False
    return True


def _define_visual_reference(stage: Usd.Stage, link_path: str, visual_index: int, visual_path: str) -> None:
    child_path = _visual_reference_path(link_path, visual_index, visual_path)
    prim = stage.OverridePrim(child_path)
    prim.GetReferences().AddInternalReference(Sdf.Path(visual_path))
    prim.CreateAttribute("sysid:preview:sourceVisualPath", Sdf.ValueTypeNames.String).Set(visual_path)


def _visual_reference_path(link_path: str, visual_index: int, visual_path: str) -> str:
    return f"{link_path}/visual_{visual_index:03d}_{_safe_link_prim_name(_path_name(visual_path), visual_index)}"


def _set_link_pose(link_prim: Usd.Prim, pose: GtLinkPose) -> None:
    if not link_prim or not link_prim.IsValid():
        return
    xform = UsdGeom.Xformable(link_prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(_pose_matrix(pose))


def _pose_matrix(pose: GtLinkPose) -> Gf.Matrix4d:
    matrix = Gf.Matrix4d(1.0)
    quat = _quat_from_orientation(pose.orientation)
    if quat is not None:
        matrix.SetRotateOnly(quat)
    matrix.SetTranslateOnly(Gf.Vec3d(*pose.position))
    return matrix


def _quat_from_orientation(orientation: tuple[float, ...] | None) -> Gf.Quatd | None:
    if orientation is None or len(orientation) < 4:
        return None
    values = [float(v) for v in orientation[:4]]
    quat = Gf.Quatd(values[0], values[1], values[2], values[3])
    return quat.GetNormalized()


def _link_key(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", str(name).lower())


def _define_marker(stage: Any, markers_path: str, pose: GtLinkPose, index: int, source: str) -> None:
    path = f"{markers_path}/{_safe_prim_name(pose.name, index)}"
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(_PREVIEW_MARKER_RADIUS)
    sphere.CreateDisplayColorAttr([_PREVIEW_EE_COLOR if source == "end_effector_poses" else _PREVIEW_COLOR])
    sphere.CreateDisplayOpacityAttr([0.85])
    xform = UsdGeom.Xformable(sphere.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pose.position))
    sphere.GetPrim().CreateAttribute("sysid:preview:name", Sdf.ValueTypeNames.String).Set(pose.name)
    sphere.GetPrim().CreateAttribute("sysid:preview:source", Sdf.ValueTypeNames.String).Set(source)


def _safe_prim_name(name: str, index: int) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", name.strip())
    cleaned = cleaned.strip("_") or f"link_{index}"
    if cleaned[0].isdigit():
        cleaned = f"link_{cleaned}"
    return f"marker_{index:03d}_{cleaned}"


def _safe_link_prim_name(name: str, index: int) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", name.strip())
    cleaned = cleaned.strip("_") or f"link_{index}"
    if cleaned[0].isdigit():
        cleaned = f"link_{cleaned}"
    return f"link_{index:03d}_{cleaned}"
