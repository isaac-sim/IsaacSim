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

"""Pure read/write helpers for Lula robot description YAML files."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any

import carb
import numpy as np

from .collision_sphere_editor import CollisionSphereEditor
from .yaml_utils import safe_load_yaml


def is_yaml_file(path: str) -> bool:
    """Return True if ``path`` has a ``.yaml`` or ``.yml`` extension."""
    _, ext = os.path.splitext(path.lower())
    return ext in (".yaml", ".yml")


def on_filter_item(item: object) -> bool:
    """File-browser filter showing YAML files and non-Omniverse folders."""
    if not item:
        return False
    if item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


@dataclass
class LulaWriteInputs:
    """Inputs for serialising a Lula robot description YAML file.

    Attributes:
        path: Destination path for the YAML file.
        articulation_base_path: Articulation root path on the stage.
        dof_names: All DOF names in articulation order.
        active_joints_mask: Boolean mask selecting active DOFs. Shape ``(num_dof,)``.
        joint_positions: Default/fixed joint positions per DOF.
        acceleration_limits: Per-DOF acceleration limits.
        jerk_limits: Per-DOF jerk limits.
        collision_sphere_editor: Editor instance whose ``save_spheres`` is used
            to write the collision-sphere section.
        mimic_joint_names: Names of joints with ``PhysxSchema.PhysxMimicJointAPI``
            applied. Mimic followers are filtered out of both the ``cspace``
            block and the ``cspace_to_urdf_rules`` block because Lula derives
            their position from the URDF ``<mimic>`` relationship at runtime
            and an explicit override here would conflict. Defaults to the
            empty set.
    """

    path: str
    articulation_base_path: str
    dof_names: list[str]
    active_joints_mask: np.ndarray
    joint_positions: np.ndarray
    acceleration_limits: np.ndarray
    jerk_limits: np.ndarray
    collision_sphere_editor: CollisionSphereEditor
    mimic_joint_names: set[str] = field(default_factory=set)


@dataclass
class LulaReadResult:
    """Parsed payload of a Lula robot description YAML file.

    Attributes:
        active_joints_mask: Boolean mask over the supplied ``dof_names`` selecting
            joints listed in the file's ``cspace``.
        acceleration_limits: Per-DOF acceleration limits.
        jerk_limits: Per-DOF jerk limits.
        joint_positions: Per-DOF positions (defaults for active, fixed values for inactive).
    """

    active_joints_mask: np.ndarray
    acceleration_limits: np.ndarray
    jerk_limits: np.ndarray
    joint_positions: np.ndarray


def write_lula_robot_description_file(inputs: LulaWriteInputs) -> None:
    """Serialise a Lula robot description YAML file.

    Args:
        inputs: See :class:`LulaWriteInputs`.

    Raises:
        ValueError: If no joints are marked active.
    """
    active_mask = inputs.active_joints_mask[: len(inputs.dof_names)]
    # Mimic followers are excluded from both `cspace` and `cspace_to_urdf_rules`
    # because Lula derives their position from the URDF `<mimic>` relationship
    # at runtime, and an explicit `fixed` rule here would conflict.
    mimic_names = set(inputs.mimic_joint_names or set())
    non_mimic_mask = np.array([name not in mimic_names for name in inputs.dof_names], dtype=bool)
    active_mask = active_mask & non_mimic_mask
    if np.sum(active_mask) == 0:
        raise ValueError(
            "There are no Active Joints in this robot description (Reference the Information Panel subsection: "
            "Command Panel).  This means that Lula will not control the robot at all.  Aborting Save Operation."
        )

    fixed_mask = (~active_mask) & non_mimic_mask
    dof_names = np.array(inputs.dof_names)
    acceleration_limits = inputs.acceleration_limits[: len(inputs.dof_names)]
    jerk_limits = inputs.jerk_limits[: len(inputs.dof_names)]
    joint_positions = inputs.joint_positions[: len(inputs.dof_names)]

    with open(inputs.path, "w") as f:
        f.write(
            "# The robot description defines the generalized coordinates and how to map those\n"
            "# to the underlying URDF dofs.\n\n"
            "api_version: 1.0\n\n"
            "# Defines the generalized coordinates. Each generalized coordinate is assumed\n"
            "# to have an entry in the URDF.\n"
            "# Lula will only use these joints to control the robot position.\n"
            "cspace:\n"
        )
        for name in dof_names[active_mask]:
            f.write(f"    - {name}\n")

        f.write("default_q: [\n")
        f.write("    ")
        for joint_pos in joint_positions[active_mask][:-1]:
            f.write(f"{str(np.around(joint_pos, 4))},")
        f.write(f"{str(np.around(joint_positions[active_mask][-1], 4))}\n")
        f.write("]\n\n")

        f.write("acceleration_limits: [\n")
        f.write("   ")
        for accel_limit in acceleration_limits[active_mask][:-1]:
            f.write(f"{str(np.around(accel_limit, 2))},")
        f.write(f"{str(np.around(acceleration_limits[active_mask][-1], 2))}\n")
        f.write("]\n\n")

        f.write("jerk_limits: [\n")
        f.write("   ")
        for jerk_limit in jerk_limits[active_mask][:-1]:
            f.write(f"{str(np.around(jerk_limit, 2))},")
        f.write(f"{str(np.around(jerk_limits[active_mask][-1], 2))}\n")
        f.write("]\n\n")

        f.write("# Most dimensions of the cspace have a direct corresponding element\n")
        f.write("# in the URDF. This list of rules defines how unspecified coordinates\n")
        f.write("# should be extracted or how values in the URDF should be overwritten.\n\n")

        f.write("cspace_to_urdf_rules:\n")
        for name, position in zip(dof_names[fixed_mask], joint_positions[fixed_mask]):
            f.write(f"    - {{name: {name}, rule: fixed, value: {str(np.around(position, 4))}}}\n")
        f.write("\n")

        f.write("# Lula uses collision spheres to define the robot geometry in order to avoid\n")
        f.write("# collisions with external obstacles.  If no spheres are specified, Lula will\n")
        f.write("# not be able to avoid obstacles.\n\n")

        inputs.collision_sphere_editor.save_spheres(inputs.articulation_base_path, f)


def read_lula_robot_description_file(
    path: str,
    dof_names: list[str],
    *,
    default_acceleration_limit: float,
    default_jerk_limit: float,
) -> LulaReadResult:
    """Parse a Lula robot description YAML file onto ``dof_names``.

    Args:
        path: Path to the YAML file.
        dof_names: DOF names for the target articulation, in articulation order.
        default_acceleration_limit: Default acceleration limit for joints without one.
        default_jerk_limit: Default jerk limit for joints without one.

    Returns:
        :class:`LulaReadResult` sized to ``dof_names``.
    """
    parsed_file: dict[str, Any] = safe_load_yaml(path)

    num_dof = len(dof_names)
    active_joints_mask = np.zeros(num_dof, dtype=bool)
    acceleration_limits = np.full(num_dof, default_acceleration_limit, dtype=float)
    jerk_limits = np.full(num_dof, default_jerk_limit, dtype=float)
    joint_positions = np.zeros(num_dof, dtype=float)

    cspace = parsed_file.get("cspace", [])
    default_q = parsed_file.get("default_q", [])
    file_acceleration_limits = parsed_file.get("acceleration_limits")
    file_jerk_limits = parsed_file.get("jerk_limits")

    cspace_arr = np.asarray(cspace)
    if cspace_arr.size > 0:
        in_mask = np.isin(cspace_arr, np.array(dof_names))
        if not np.all(in_mask):
            carb.log_warn(
                "Some joints listed in the cspace of the provided robot_description YAML file are not "
                f"present in the robot Articulation: {cspace_arr[~in_mask]}"
            )

    for i, joint in enumerate(cspace):
        if joint not in dof_names:
            continue
        idx = dof_names.index(joint)
        active_joints_mask[idx] = True
        if i < len(default_q):
            joint_positions[idx] = default_q[i]
        if file_acceleration_limits is not None and i < len(file_acceleration_limits):
            acceleration_limits[idx] = file_acceleration_limits[i]
        if file_jerk_limits is not None and i < len(file_jerk_limits):
            jerk_limits[idx] = file_jerk_limits[i]

    for item in parsed_file.get("cspace_to_urdf_rules", []) or []:
        if item.get("rule") != "fixed":
            continue
        joint_name = item["name"]
        if joint_name not in dof_names:
            carb.log_warn(
                f"Fixed joint specified for a joint that is not present in the robot Articulation: {joint_name}"
            )
            continue
        idx = dof_names.index(joint_name)
        active_joints_mask[idx] = False
        joint_positions[idx] = item["value"]

    return LulaReadResult(
        active_joints_mask=active_joints_mask,
        acceleration_limits=acceleration_limits,
        jerk_limits=jerk_limits,
        joint_positions=joint_positions,
    )
