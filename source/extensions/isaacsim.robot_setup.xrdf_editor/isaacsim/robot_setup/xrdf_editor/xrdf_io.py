# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Pure read/write/merge/validate helpers for cuMotion XRDF files.

This module is deliberately UI-free so it can be unit-tested without an
``omni.ui`` / Kit context. All inputs are passed explicitly; no global state.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any

import carb
import numpy as np
import yaml

from .constants import (
    COLLISION_KEY_V1,
    COLLISION_KEY_V2,
    DEFAULT_GEOMETRY_GROUP_NAME,
    SUPPORTED_XRDF_VERSIONS,
    XRDF_FORMAT,
    XRDF_VERSION_1,
    XRDF_VERSION_2,
)
from .yaml_utils import safe_load_yaml


def is_xrdf_file(path: str) -> bool:
    """Return True if ``path`` has a YAML or XRDF extension."""
    _, ext = os.path.splitext(path.lower())
    return ext in (".yaml", ".yml", ".xrdf")


def on_filter_xrdf_item(item: object) -> bool:
    """File-browser filter showing XRDF files and non-Omniverse folders."""
    if not item:
        return False
    if item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_xrdf_file(item.path)


def collision_key_for_version(version: float) -> str:
    """Return the top-level collision key used by the given XRDF format version.

    Version 1.0 uses ``collision`` and version 2.0 uses ``world_collision``.
    """
    if version == XRDF_VERSION_1:
        return COLLISION_KEY_V1
    if version == XRDF_VERSION_2:
        return COLLISION_KEY_V2
    raise ValueError(f"Unsupported XRDF format version: {version}. Supported: {SUPPORTED_XRDF_VERSIONS}")


def is_valid_xrdf_file(path: str) -> bool:
    """Validate that ``path`` points to a parseable XRDF document.

    The file must exist, parse as YAML, contain ``format: xrdf``, and supply a
    ``format_version`` field. Unsupported versions emit a warning but still return
    True (so callers can attempt graceful degradation).

    Args:
        path: Path to the candidate file.

    Returns:
        True if ``path`` is a valid XRDF file.
    """
    warning_msg = f"XRDF file {path} is not a valid XRDF file for Merging. Save to a new XRDF file."
    if not os.path.isfile(path):
        carb.log_warn(warning_msg)
        return False
    with open(path, "r") as stream:
        try:
            parsed_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            carb.log_warn(warning_msg + f" {exc}")
            return False

    if not isinstance(parsed_file, dict):
        # Empty, list-rooted, or scalar-rooted YAML files are not valid XRDFs.
        carb.log_warn(warning_msg)
        return False

    if parsed_file.get("format") == XRDF_FORMAT and "format_version" in parsed_file:
        if parsed_file["format_version"] not in SUPPORTED_XRDF_VERSIONS:
            carb.log_warn(
                f"Attempting to read an XRDF file with format version {parsed_file['format_version']}. "
                f"Only versions {SUPPORTED_XRDF_VERSIONS} are supported."
            )
        return True

    carb.log_warn(warning_msg)
    return False


def merge_passthrough_dict(existing_path: str, articulation_frames: set[str]) -> dict[str, Any]:
    """Load the subset of an existing XRDF that should survive a merge-export.

    Returns a dict containing everything we want to preserve verbatim
    (``modifiers``, ``tool_frames``, ``self_collision.ignore`` lists, geometry
    groups, etc.) while stripping the fields the editor is about to overwrite
    (``default_joint_positions``, ``cspace``).

    Args:
        existing_path: Path to an XRDF file to merge with.
        articulation_frames: Link names belonging to the current articulation
            (without leading slash). Used for buffer-distance reconciliation.

    Returns:
        Dictionary to seed the export with. Empty dict if the file is invalid.
    """
    parsed_file: dict[str, Any] = {}
    if not is_valid_xrdf_file(existing_path):
        return parsed_file

    parsed_file = safe_load_yaml(existing_path)

    parsed_file.pop("default_joint_positions", None)
    parsed_file.pop("cspace", None)

    # Normalize to v2 internally; the writer reconciles down to the requested version.
    if COLLISION_KEY_V1 in parsed_file and COLLISION_KEY_V2 not in parsed_file:
        parsed_file[COLLISION_KEY_V2] = parsed_file.pop(COLLISION_KEY_V1)

    collision_key = COLLISION_KEY_V2

    if "self_collision" in parsed_file and collision_key in parsed_file and "geometry" in parsed_file[collision_key]:
        # self_collision may be present without a geometry pointer; mirror the
        # collision group so preserved fields (ignore, buffer_distance) keep a
        # consistent target and the writer emits a well-formed block.
        if "geometry" not in parsed_file["self_collision"]:
            parsed_file["self_collision"]["geometry"] = parsed_file[collision_key]["geometry"]

        if parsed_file["self_collision"]["geometry"] == parsed_file[collision_key]["geometry"]:
            # Since buffer distances in the world_collision group are going to be set to zero,
            # to keep the relative sphere sizes between world_collision and self_collision the
            # same, world_collision buffer distances are subtracted from self_collision buffer
            # distances.
            if "buffer_distance" in parsed_file[collision_key]:
                collision_buffer_distance = parsed_file[collision_key]["buffer_distance"]
                # Use setdefault so a freshly-created buffer_distance dict is
                # actually written back into self_collision; previously
                # ``.get(..., {})`` returned a throwaway empty dict and any
                # entries we appended were silently discarded.
                self_collision_buffer_distance = parsed_file["self_collision"].setdefault("buffer_distance", {})
                for k, v in collision_buffer_distance.items():
                    if k in articulation_frames:
                        if k in self_collision_buffer_distance:
                            self_collision_buffer_distance[k] -= v
                        else:
                            self_collision_buffer_distance[k] = -v
                        collision_buffer_distance[k] = 0
        else:
            # Update the geometry pointer in place so preserved fields like
            # `ignore` and `buffer_distance` survive the merge. Previously the
            # whole self_collision block was replaced, dropping those fields
            # even though this helper is supposed to carry them forward.
            parsed_file["self_collision"]["geometry"] = parsed_file[collision_key]["geometry"]

    # A merge source can pass is_valid_xrdf_file() while still being incomplete:
    # missing top-level geometry, missing the geometry group referenced by
    # collision_key.geometry, or missing self_collision entirely. Normalise all
    # of those cases here so build_xrdf_dict() does not have to defend against
    # broken merged state. The fully-from-scratch creation path lives in
    # build_xrdf_dict() and is exercised when this function returns {}.
    has_collision = collision_key in parsed_file and "geometry" in parsed_file[collision_key]
    geometry_dict = parsed_file.get("geometry")
    geometry_group_name = parsed_file[collision_key]["geometry"] if has_collision else None
    geometry_group_is_usable = (
        has_collision and isinstance(geometry_dict, dict) and isinstance(geometry_dict.get(geometry_group_name), dict)
    )

    if not geometry_group_is_usable:
        # No usable geometry to carry forward; drop the half-populated sections
        # so build_xrdf_dict() re-creates them from scratch.
        parsed_file.pop("geometry", None)
        parsed_file.pop(collision_key, None)
        parsed_file.pop("self_collision", None)
    else:
        for k in list(geometry_dict.keys()):
            if k != geometry_group_name:
                geometry_dict.pop(k, None)
            else:
                geometry_dict[k].pop("clone", None)

    return parsed_file


@dataclass
class XrdfWriteInputs:
    """All inputs required to serialise a robot configuration as an XRDF file.

    Attributes:
        path: Destination path for the XRDF file.
        format_version: XRDF format version. Must be one of
            :data:`~.constants.SUPPORTED_XRDF_VERSIONS`.
        articulation_base_path: Articulation root path on the stage (used as
            sphere-path prefix).
        dof_names: Names of every DOF in the articulation, in articulation order.
        active_joints_mask: Boolean mask over ``dof_names`` selecting active DOFs.
            Shape ``(num_dof,)``.
        joint_positions: Default position for every DOF. Shape ``(num_dof,)``.
        acceleration_limits: Per-DOF acceleration limits. Shape ``(num_dof,)``.
        jerk_limits: Per-DOF jerk limits. Shape ``(num_dof,)``.
        ordered_links: Links in articulation order from root to end-effector.
        ignore_dict: Self-collision ignore rules (see
            :func:`articulation_discovery.get_ignore_dict`).
        sphere_dict_writer: Callable that populates a ``{link_name: [{center, radius}]}``
            dict for the spheres currently authored on the stage. Receives the
            articulation base path and the destination dict.
        merge_existing: Optional path to an existing XRDF file to merge with.
            ``None`` disables merging.
        articulation_frames: Link names of the current articulation (without
            leading slash); only used when ``merge_existing`` is not ``None``.
        mimic_joint_names: Names of joints with ``PhysxSchema.PhysxMimicJointAPI``
            applied. Mimic followers are filtered out of both
            ``default_joint_positions`` and ``cspace.joint_names`` because
            cuMotion's ``load_robot_from_memory`` rejects independent defaults
            for mimic-controlled joints. Defaults to the empty set, in which
            case every entry of ``dof_names`` is treated as an ordinary DOF.
    """

    path: str
    format_version: float
    articulation_base_path: str
    dof_names: list[str]
    active_joints_mask: np.ndarray
    joint_positions: np.ndarray
    acceleration_limits: np.ndarray
    jerk_limits: np.ndarray
    ordered_links: list[str]
    ignore_dict: dict[str, list[str]]
    sphere_dict_writer: Any = None  # Callable[[str, dict], None]
    merge_existing: str | None = None
    articulation_frames: set[str] = field(default_factory=set)
    mimic_joint_names: set[str] = field(default_factory=set)


@dataclass
class XrdfReadResult:
    """Parsed payload of an XRDF file, projected onto a target articulation.

    Attributes:
        parsed_file: Raw parsed dictionary (suitable for
            :meth:`CollisionSphereEditor.load_xrdf_spheres`).
        active_joints_mask: Boolean mask over the supplied ``dof_names`` selecting
            joints listed in the file's ``cspace``. Shape ``(num_dof,)``.
        acceleration_limits: Per-DOF acceleration limits sourced from the file.
            Defaults to the supplied initial values for joints not in cspace.
        jerk_limits: Per-DOF jerk limits sourced from the file.
        joint_positions: Per-DOF default positions sourced from the file's
            ``default_joint_positions``.
    """

    parsed_file: dict[str, Any]
    active_joints_mask: np.ndarray
    acceleration_limits: np.ndarray
    jerk_limits: np.ndarray
    joint_positions: np.ndarray


def _normalise_version(parsed_file: dict, format_version: float) -> None:
    """Ensure ``parsed_file`` uses exactly the collision key for ``format_version``.

    Mutates the dict in place so only the version-appropriate top-level key
    (``collision`` for v1, ``world_collision`` for v2) is present.
    """
    if format_version == XRDF_VERSION_1:
        if COLLISION_KEY_V2 in parsed_file:
            parsed_file[COLLISION_KEY_V1] = parsed_file.pop(COLLISION_KEY_V2)
        parsed_file.pop(COLLISION_KEY_V2, None)
    elif format_version == XRDF_VERSION_2:
        if COLLISION_KEY_V1 in parsed_file:
            parsed_file[COLLISION_KEY_V2] = parsed_file.pop(COLLISION_KEY_V1)
        parsed_file.pop(COLLISION_KEY_V1, None)


def _validate_xrdf_version(format_version: float) -> float:
    """Coerce ``format_version`` to a supported value, warning on mismatch."""
    if format_version in SUPPORTED_XRDF_VERSIONS:
        return float(format_version)
    carb.log_warn(f"Invalid XRDF version {format_version}, defaulting to {XRDF_VERSION_2}")
    return XRDF_VERSION_2


def _write_yaml_item(f, item: Any, tabbing: str) -> None:
    """Write ``item`` to ``f`` using the XRDF-style formatting.

    The XRDF format expects ordered keys, terse inline numeric lists, and
    a per-section block style; that's what this writer produces.
    """
    if isinstance(item, dict):
        for k in list(item.keys()):
            f.write(f"{tabbing}{k}: ")
            tabbing = " " * len(tabbing)
            if isinstance(item[k], dict):
                f.write("\n")
            _write_yaml_item(f, item[k], tabbing + "  ")
    elif isinstance(item, (list, np.ndarray)):
        if len(item) == 0:
            f.write("[]\n")
            return
        if isinstance(item[0], dict):
            f.write("\n")
            for d in item:
                _write_yaml_item(f, d, tabbing + "- ")
        elif isinstance(item[0], str):
            f.write("\n")
            for val in item:
                f.write(tabbing + "- ")
                _write_yaml_item(f, val, "")
        else:
            f.write("[")
            for val in item[:-1]:
                f.write(f"{str(np.around(val, 4))}, ")
            f.write(f"{str(np.around(item[-1], 4))}]\n")
    else:
        if isinstance(item, str):
            f.write(f'"{item}"\n')
        else:
            f.write(f"{str(np.around(item, 4))}\n")


def build_xrdf_dict(inputs: XrdfWriteInputs) -> dict[str, Any]:
    """Build the dictionary that will be serialised as an XRDF file.

    Public for testing — most callers should use :func:`write_xrdf_file`.

    Args:
        inputs: See :class:`XrdfWriteInputs`.

    Returns:
        Dictionary with the XRDF payload (including ``format``, ``format_version``,
        ``default_joint_positions``, ``cspace``, ``geometry``, etc.).
    """
    format_version = _validate_xrdf_version(inputs.format_version)
    collision_key = collision_key_for_version(format_version)

    if inputs.merge_existing:
        parsed_file = merge_passthrough_dict(inputs.merge_existing, inputs.articulation_frames)
        _normalise_version(parsed_file, format_version)
    else:
        parsed_file = {}

    parsed_file["format"] = XRDF_FORMAT
    parsed_file["format_version"] = float(format_version)

    dof_names = np.array(inputs.dof_names)
    active_mask = inputs.active_joints_mask[: len(dof_names)]
    # Mimic followers are excluded from BOTH `default_joint_positions` and
    # `cspace`. cuMotion derives their position from the reference joint via
    # the URDF `<mimic>` element and raises if the XRDF tries to set one
    # independently.
    mimic_names = set(inputs.mimic_joint_names or set())
    non_mimic_mask = np.array([name not in mimic_names for name in inputs.dof_names], dtype=bool)
    cspace_mask = active_mask & non_mimic_mask
    acceleration_limits = inputs.acceleration_limits[: len(dof_names)][cspace_mask]
    jerk_limits = inputs.jerk_limits[: len(dof_names)][cspace_mask]

    default_joint_positions_dict: dict[str, float] = {}
    for i, dof_name in enumerate(dof_names):
        if str(dof_name) in mimic_names:
            continue
        default_joint_positions_dict[dof_name] = float(inputs.joint_positions[i])
    parsed_file["default_joint_positions"] = default_joint_positions_dict

    cspace_dict: dict[str, list] = {
        "joint_names": [],
        "acceleration_limits": [],
        "jerk_limits": [],
    }
    for i, dof_name in enumerate(dof_names[cspace_mask]):
        cspace_dict["joint_names"].append(str(dof_name))
        cspace_dict["acceleration_limits"].append(float(acceleration_limits[i]))
        cspace_dict["jerk_limits"].append(float(jerk_limits[i]))
    parsed_file["cspace"] = cspace_dict

    if "geometry" not in parsed_file or collision_key not in parsed_file:
        default_name = DEFAULT_GEOMETRY_GROUP_NAME
        parsed_file[collision_key] = {"geometry": default_name}
        parsed_file["geometry"] = {default_name: {"spheres": {}}}
        parsed_file["self_collision"] = {"geometry": default_name}

    # A merge source may carry collision + geometry but no self_collision; mirror the
    # collision geometry group so the ignore-dict block below has a place to live.
    if "self_collision" not in parsed_file:
        parsed_file["self_collision"] = {"geometry": parsed_file[collision_key]["geometry"]}

    if "ignore" not in parsed_file["self_collision"]:
        parsed_file["self_collision"]["ignore"] = inputs.ignore_dict

    geometry_group_name = parsed_file[collision_key]["geometry"]
    sphere_dict = parsed_file["geometry"][geometry_group_name].get("spheres")
    if sphere_dict is None:
        sphere_dict = {}
        parsed_file["geometry"][geometry_group_name]["spheres"] = sphere_dict
    for link in inputs.ordered_links:
        sphere_dict.pop(link, None)
    if inputs.sphere_dict_writer is not None:
        inputs.sphere_dict_writer(inputs.articulation_base_path, sphere_dict)

    # Final cleanup: ensure only the version-appropriate collision key survives.
    _normalise_version(parsed_file, format_version)

    return parsed_file


def write_xrdf_file(inputs: XrdfWriteInputs) -> None:
    """Build and serialise an XRDF document to disk.

    See :class:`XrdfWriteInputs` for the input bundle.
    """
    parsed_file = build_xrdf_dict(inputs)
    format_version = _validate_xrdf_version(inputs.format_version)
    collision_key = collision_key_for_version(format_version)

    key_order = [
        "format",
        "format_version",
        "modifiers",
        "default_joint_positions",
        "cspace",
        "tool_frames",
        collision_key,
        "self_collision",
        "geometry",
    ]

    with open(inputs.path, "w") as f:
        for key in key_order:
            if key in parsed_file:
                f.write(f"{key}: ")
                value = parsed_file[key]
                if isinstance(value, dict):
                    f.write("\n")
                _write_yaml_item(f, value, "  ")
                if key != key_order[-1]:
                    f.write("\n")


def read_xrdf_file(
    path: str,
    dof_names: list[str],
    *,
    default_acceleration_limit: float,
    default_jerk_limit: float,
    joint_limits_lower: np.ndarray | None = None,
    joint_limits_upper: np.ndarray | None = None,
) -> XrdfReadResult:
    """Parse an XRDF file and project its payload onto ``dof_names``.

    Args:
        path: Path to the XRDF file.
        dof_names: DOF names for the target articulation, in articulation order.
        default_acceleration_limit: Acceleration limit applied to DOFs not
            present in the file's cspace.
        default_jerk_limit: Jerk limit applied to DOFs not present in the file's cspace.
        joint_limits_lower: Per-DOF lower limits, used to clamp default positions.
            Pass ``None`` to skip clamping.
        joint_limits_upper: Per-DOF upper limits, used to clamp default positions.

    Returns:
        :class:`XrdfReadResult` populated with masks/values sized to ``dof_names``.

    Raises:
        ValueError: If the file's ``format`` is not ``xrdf`` or it is missing
            ``format_version``.
    """
    parsed_file = safe_load_yaml(path)

    if parsed_file.get("format") != XRDF_FORMAT:
        raise ValueError("XRDF file is expected to contain the line\nformat: xrdf\nbut this line is missing.")
    if "format_version" not in parsed_file:
        raise ValueError("XRDF file is expected to have a field:\nformat_version\nBut this field is missing.")
    if parsed_file["format_version"] not in SUPPORTED_XRDF_VERSIONS:
        carb.log_warn(
            f"Attempting to read an XRDF file with format version {parsed_file['format_version']}. "
            f"Only versions {SUPPORTED_XRDF_VERSIONS} are supported."
        )

    num_dof = len(dof_names)
    active_joints_mask = np.zeros(num_dof, dtype=bool)
    acceleration_limits = np.full(num_dof, default_acceleration_limit, dtype=float)
    jerk_limits = np.full(num_dof, default_jerk_limit, dtype=float)
    joint_positions = np.zeros(num_dof, dtype=float)

    cspace = parsed_file.get("cspace", {})
    cspace_joint_names = cspace.get("joint_names", [])
    file_acceleration_limits = cspace.get("acceleration_limits", [])
    file_jerk_limits = cspace.get("jerk_limits", [])

    default_q_map = parsed_file.get("default_joint_positions", {}) or {}

    cspace_arr = np.asarray(cspace_joint_names)
    in_mask = np.isin(cspace_arr, np.array(dof_names))
    if cspace_arr.size > 0 and not np.all(in_mask):
        carb.log_warn(
            "Some joints listed in the cspace of the provided XRDF file are not present in the robot Articulation:"
            f" {cspace_arr[~in_mask]}"
        )

    for i, joint in enumerate(cspace_joint_names):
        if joint not in dof_names:
            continue
        idx = dof_names.index(joint)
        active_joints_mask[idx] = True
        if i < len(file_acceleration_limits):
            acceleration_limits[idx] = file_acceleration_limits[i]
        if i < len(file_jerk_limits):
            jerk_limits[idx] = file_jerk_limits[i]

    for dof_name, position in default_q_map.items():
        if dof_name in dof_names:
            joint_positions[dof_names.index(dof_name)] = position
        else:
            carb.log_warn(
                f"Invalid DOF name [{dof_name}] specified in XRDF file 'default_joint_positions' "
                "field that could not be found in the currently selected Articulation."
            )

    if joint_limits_lower is not None and joint_limits_upper is not None:
        joint_positions = np.clip(joint_positions, joint_limits_lower, joint_limits_upper)

    return XrdfReadResult(
        parsed_file=parsed_file,
        active_joints_mask=active_joints_mask,
        acceleration_limits=acceleration_limits,
        jerk_limits=jerk_limits,
        joint_positions=joint_positions,
    )
