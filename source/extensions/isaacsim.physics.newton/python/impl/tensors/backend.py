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

"""Backend classes for Newton tensor API."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import carb
import newton
import numpy as np
import omni.physics.tensors
import warp as wp

from .utils import find_matching_paths

if TYPE_CHECKING:
    from ..newton_stage import NewtonStage


UNRESOLVED_BODY = -2
"""Returned by ``_resolve_filter_path_to_body_index`` when a path cannot be
resolved to any body or world shape."""


def _resolve_filter_path_to_body_index(filter_path: str, model: object) -> int:
    """Resolve a filter prim path to a body index.

    Args:
        filter_path: The prim path to resolve.
        model: The Newton model containing body labels and shape data.

    Returns:
        body index (>=0), -1 for world body shapes (``shape_body == -1``),
        or :data:`UNRESOLVED_BODY` (-2) if the path cannot be resolved.
    """
    if not filter_path or model is None:
        return UNRESOLVED_BODY
    body_label = model.body_label  # type: ignore[attr-defined]
    if filter_path in body_label:
        return body_label.index(filter_path)
    candidates = [(i, path) for i, path in enumerate(body_label) if filter_path.startswith(path + "/")]
    if candidates:
        best = max(candidates, key=lambda x: len(x[1]))
        return best[0]
    shape_body = model.shape_body.numpy() if hasattr(model.shape_body, "numpy") else model.shape_body  # type: ignore[attr-defined]
    for i, label in enumerate(model.shape_label):  # type: ignore[attr-defined]
        if label == filter_path or filter_path.startswith(label + "/") or label.startswith(filter_path + "/"):
            if shape_body[i] == -1:
                return -1
    return UNRESOLVED_BODY


# Map Newton joint types to omni.physics.tensors.JointType enum
JointTypeDic = {
    newton.JointType.PRISMATIC: omni.physics.tensors.JointType.Prismatic,
    newton.JointType.REVOLUTE: omni.physics.tensors.JointType.Revolute,
    newton.JointType.BALL: omni.physics.tensors.JointType.Spherical,
    newton.JointType.FIXED: omni.physics.tensors.JointType.Fixed,
    newton.JointType.FREE: omni.physics.tensors.JointType.Invalid,
    newton.JointType.DISTANCE: omni.physics.tensors.JointType.Invalid,
    newton.JointType.D6: omni.physics.tensors.JointType.Invalid,
}


class ArticulationSet:
    """Set of articulations for tensor-based access.

    Args:
        newton_stage: The Newton stage instance.
        articulation_indices: Warp array of articulation indices.
        root_body_indices: Warp array of root body indices.
        dof_position_indices: Warp array of DOF position indices.
        dof_velocity_indices: Warp array of DOF velocity indices.
        dof_axis_indices: Warp array of DOF axis indices.
        joint_indices: Warp array of joint indices.
        shape_indices: Warp array of shape indices.
        link_indices: Warp array of link indices.
        meta_types: List of articulation metadata types.
        count: Number of articulations in this set.
        max_dofs: Maximum number of DOFs across all articulations.
    """

    def __init__(
        self,
        newton_stage: NewtonStage,
        articulation_indices: wp.array,
        root_body_indices: wp.array,
        dof_position_indices: wp.array,
        dof_velocity_indices: wp.array,
        dof_axis_indices: wp.array,
        joint_indices: wp.array,
        shape_indices: wp.array,
        link_indices: wp.array,
        meta_types: list[ArticulationMetaType],
        count: int,
        max_dofs: int,
    ) -> None:
        self.newton_stage = newton_stage
        self.articulation_indices = articulation_indices
        self.root_body_indices = root_body_indices
        self.dof_position_indices = dof_position_indices
        self.dof_velocity_indices = dof_velocity_indices
        self.dof_axis_indices = dof_axis_indices
        self.joint_indices = joint_indices
        self.shape_indices = shape_indices
        self.link_indices = link_indices
        self.meta_types = meta_types
        self.count = count
        self.max_dofs = max_dofs
        self.max_fixed_tendons = 0
        self.q_ik = self.model.joint_q
        self.qd_ik = self.model.joint_qd
        self.q_forces = wp.zeros(self.dof_axis_indices.shape[0], dtype=wp.float32)

    @property
    def model(self) -> Any:
        """The Newton model."""
        return self.newton_stage.model

    @property
    def shared_metatype(self) -> ArticulationMetaType:
        """Shared metatype for homogeneous articulations.

        Returns:
            The shared ArticulationMetaType instance.
        """
        return self.meta_types[0]

    @property
    def max_links(self) -> int:
        """Maximum number of links per articulation.

        Returns:
            The maximum link count.
        """
        return self.shared_metatype.link_count

    @property
    def link_paths(self) -> list[list[str]]:
        """Link paths for all articulations.

        Returns:
            List of link paths for each articulation.
        """
        link_paths = []
        for meta in self.meta_types:
            paths = meta.link_paths
            link_paths.append(paths)
        return link_paths

    @property
    def max_shapes(self) -> int:
        """Maximum number of shapes per articulation.

        Returns:
            The maximum shape count.
        """
        max_shapes = 0
        for i in range(self.count):
            max_shapes = max(max_shapes, len(self.meta_types[i].link_shapes))
        return max_shapes


class ArticulationMetaType:
    """Metadata type for an articulation structure.

    Args:
        link_paths: List of USD paths for links.
        link_shapes: List of shape indices for links.
        joint_paths: List of USD paths for joints.
        dof_paths: List of USD paths for DOFs.
        link_indices: Dictionary mapping link names to indices.
        joint_indices: Dictionary mapping joint names to indices.
        dof_indices: Dictionary mapping DOF names to indices.
        joint_types: List of joint types.
        joint_dof_offsets: List of DOF offsets for joints.
        joint_dof_counts: List of DOF counts for joints.
        dof_types: List of DOF types.
        fixed_base: Whether the articulation has a fixed base.
    """

    def __init__(
        self,
        link_paths: list[str],
        link_shapes: list[int],
        joint_paths: list[str],
        dof_paths: list[str],
        link_indices: dict[str, int],
        joint_indices: dict[str, int],
        dof_indices: dict[str, int],
        joint_types: list[omni.physics.tensors.JointType],
        joint_dof_offsets: list[int],
        joint_dof_counts: list[int],
        dof_types: list[omni.physics.tensors.DofType],
        fixed_base: bool,
    ) -> None:
        self.link_count = len(link_paths)
        self.joint_count = len(joint_paths)
        self.dof_count = len(dof_paths)
        self.link_paths = link_paths
        self.link_shapes = link_shapes
        self.joint_paths = joint_paths
        self.dof_paths = dof_paths
        self.link_indices = link_indices
        self.joint_indices = joint_indices
        self.dof_indices = dof_indices
        self.joint_types = joint_types
        self.joint_dof_offsets = joint_dof_offsets
        self.joint_dof_counts = joint_dof_counts
        self.dof_types = dof_types
        self.fixed_base = fixed_base

    @property
    def link_names(self) -> list[str]:
        """Link names derived from link paths by extracting the name after the last '/'.

        Returns:
            List of link names.
        """
        return [path.rsplit("/", 1)[-1] for path in self.link_paths]

    @property
    def joint_names(self) -> list[str]:
        """Joint names derived from joint paths by extracting the name after the last '/'.

        Returns:
            List of joint names.
        """
        return [path.rsplit("/", 1)[-1] for path in self.joint_paths]

    @property
    def dof_names(self) -> list[str]:
        """DOF names derived from DOF paths by extracting the name after the last '/'.

        Returns:
            List of DOF names.
        """
        return [path.rsplit("/", 1)[-1] for path in self.dof_paths]


class RigidBodySet:
    """Set of rigid bodies for tensor-based access.

    Args:
        newton_stage: The Newton stage instance.
        body_indices: Warp array of body indices.
        body_paths: List of USD paths for bodies.
        body_names: List of body names.
    """

    def __init__(
        self,
        newton_stage: NewtonStage,
        body_indices: wp.array,
        body_paths: list[str],
        body_names: list[str],
    ) -> None:
        self.newton_stage = newton_stage
        self.body_indices = body_indices
        self.body_paths = body_paths
        self.prim_paths = body_paths
        self.body_names = body_names
        self.count = len(body_names)

        body_indices_list = (
            self.body_indices.list() if hasattr(self.body_indices, "list") else self.body_indices.tolist()  # type: ignore[attr-defined]
        )
        self.max_shapes = 0
        for i in range(self.count):
            body_idx = body_indices_list[i]
            shape_ids = self.model.body_shapes[body_idx]
            self.max_shapes = max(self.max_shapes, len(shape_ids))

    @property
    def model(self) -> Any:
        """The Newton model."""
        return self.newton_stage.model


class RigidContactSet:
    """Set of rigid body contacts for tensor-based access.

    Args:
        newton_stage: The Newton stage instance.
        sensor_indices: Warp array of sensor body indices.
        sensor_paths: List of USD paths for sensors.
        sensor_names: List of sensor names.
        filter_indices: Warp array of filter body indices.
        filter_paths: List of lists of filter body paths.
        filter_names: List of lists of filter body names.
        max_filters: Maximum number of filters per sensor.
        body_sensor_map: Warp array mapping body indices to sensor indices,
            length ``body_count + 1`` (last element is the world body slot).
        world_body_idx: Index used for the world body (ground plane) in the
            sensor / filter mapping arrays.
        max_contact_data_count: Maximum contact data buffer size.
    """

    def __init__(
        self,
        newton_stage: NewtonStage,
        sensor_indices: wp.array,
        sensor_paths: list[str],
        sensor_names: list[str],
        filter_indices: wp.array,
        filter_paths: list[list[str]],
        filter_names: list[list[str]],
        max_filters: int,
        body_sensor_map: wp.array,
        world_body_idx: int,
        max_contact_data_count: int = 0,
    ) -> None:
        self.newton_stage = newton_stage
        self.sensor_indices = sensor_indices
        self.sensor_paths = sensor_paths
        self.sensor_names = sensor_names
        self.filter_indices = filter_indices
        self.filter_paths = filter_paths
        self.filter_names = filter_names
        self.sensor_count = len(sensor_names)
        self.filter_count = max_filters
        self.count = self.sensor_count
        self.body_sensor_map = body_sensor_map
        self.world_body_idx = world_body_idx
        self.max_contact_data_count = max_contact_data_count

    @property
    def model(self) -> Any:
        """The Newton model."""
        return self.newton_stage.model


class NewtonSimView:
    """Simulation view backend for Newton physics.

    Args:
        newton_stage: The Newton stage instance.
    """

    def __init__(self, newton_stage: NewtonStage) -> None:
        self.newton_stage = newton_stage
        self.is_valid_flag = True
        self.device = newton_stage.device
        if hasattr(self.device, "is_cpu") and self.device.is_cpu:
            self.device_ordinal = -1
        elif hasattr(self.device, "is_cuda") and self.device.is_cuda:
            self.device_ordinal = self.device.ordinal
        elif str(self.device) == "cpu":
            self.device_ordinal = -1
        elif "cuda" in str(self.device):
            self.device_ordinal = int(str(self.device).split(":")[-1])
        else:
            self.device_ordinal = -1

    @property
    def model(self) -> Any:
        """The Newton model."""
        return self.newton_stage.model

    def get_gravity(self, gravity: list[float]) -> bool:
        """Get the simulation gravity vector.

        Args:
            gravity: Output list to store gravity values.

        Returns:
            True if successful.
        """
        newton_gravity = self.newton_stage.model.gravity  # type: ignore[union-attr, attr-defined]
        gravity[0] = newton_gravity[0]
        gravity[1] = newton_gravity[1]
        gravity[2] = newton_gravity[2]
        return True

    def set_gravity(self, gravity: list[float]) -> None:
        """Set the simulation gravity vector.

        Args:
            gravity: Gravity vector as [x, y, z].
        """
        self.newton_stage.model.gravity = gravity  # type: ignore[union-attr, attr-defined]

    def update_articulations_kinematic(self) -> bool:
        """Update articulation kinematics.

        Returns:
            True if successful.
        """
        return True

    def initialize_kinematic_bodies(self) -> None:
        """Initialize kinematic bodies."""

    def invalidate(self) -> None:
        """Invalidate the simulation view."""
        self.is_valid_flag = False

    def is_valid(self) -> bool:
        """Check if the simulation view is valid.

        Returns:
            True if the simulation view is valid.
        """
        return self.is_valid_flag

    def create_rigid_contact_view(
        self,
        pattern: list[str],
        filter_patterns: list[list[str]] | None = None,
        max_contact_data_count: int = 0,
    ) -> RigidContactSet | None:
        """Create rigid contact view matching PhysX behavior.

        Args:
            pattern: List of sensor pattern strings.
            filter_patterns: List of lists of filter patterns for each sensor.
            max_contact_data_count: Maximum number of contact data points.

        Returns:
            RigidContactSet for the matching contacts, or None if no matches.
        """
        if filter_patterns is None:
            filter_patterns = []

        stage = self.newton_stage.usd_stage

        if isinstance(pattern, str):
            pattern = [pattern]

        if len(filter_patterns) == 0:
            filter_patterns = [[]] * len(pattern)
        elif len(filter_patterns) != len(pattern):
            carb.log_error(
                f"filter_patterns length ({len(filter_patterns)}) must match pattern length ({len(pattern)})"
            )
            return None

        all_sensor_paths = []
        all_sensor_indices = []
        all_sensor_names = []
        all_filter_paths_per_sensor = []
        all_filter_indices_per_sensor = []
        all_filter_names_per_sensor = []
        num_filters = 0

        for pattern_idx, sensor_pattern in enumerate(pattern):
            matched_sensor_paths = find_matching_paths(stage, sensor_pattern)
            if len(matched_sensor_paths) == 0:
                carb.log_warn(f"Sensor pattern '{sensor_pattern}' matched no paths")
                continue

            sensor_filter_patterns = filter_patterns[pattern_idx]
            num_filter_patterns_for_this_group = len(sensor_filter_patterns)

            filter_paths_per_pattern = []

            for filt_pat in sensor_filter_patterns:
                matched_filter_paths = find_matching_paths(stage, filt_pat)
                # Same as PhysX BaseSimulationView: single filter match → use for all sensors (e.g. ground plane).
                if len(matched_filter_paths) == 1:
                    filter_paths_per_pattern.append([matched_filter_paths[0]] * len(matched_sensor_paths))
                elif len(matched_filter_paths) == len(matched_sensor_paths):
                    filter_paths_per_pattern.append(matched_filter_paths)
                else:
                    carb.log_error(
                        f"Filter pattern '{filt_pat}' matched {len(matched_filter_paths)} objects, "
                        f"expected 1 or {len(matched_sensor_paths)}"
                    )
                    filter_paths_per_pattern.append([""] * len(matched_sensor_paths))

            if pattern_idx == 0:
                num_filters = num_filter_patterns_for_this_group
            elif num_filter_patterns_for_this_group != num_filters:
                carb.log_error(
                    f"Sensor pattern '{sensor_pattern}' has {num_filter_patterns_for_this_group} filters, "
                    f"but previous patterns had {num_filters} filters"
                )

            for sensor_local_idx, sensor_path in enumerate(matched_sensor_paths):
                if sensor_path not in self.model.body_label:
                    carb.log_warn(f"Sensor path '{sensor_path}' not in simulator")
                    continue

                body_idx = self.model.body_label.index(sensor_path)
                all_sensor_paths.append(sensor_path)
                all_sensor_indices.append(body_idx)
                all_sensor_names.append(
                    self.model.body_label[body_idx] if body_idx < len(self.model.body_label) else f"body_{body_idx}"
                )

                sensor_filter_paths = []
                sensor_filter_indices = []
                sensor_filter_names = []

                for filter_pattern_idx in range(num_filter_patterns_for_this_group):
                    filter_path = filter_paths_per_pattern[filter_pattern_idx][sensor_local_idx]
                    sensor_filter_paths.append(filter_path)

                    filter_body_idx = (
                        _resolve_filter_path_to_body_index(filter_path, self.model) if filter_path else UNRESOLVED_BODY
                    )
                    if filter_body_idx == -1:
                        sensor_filter_indices.append(self.model.body_count)
                        sensor_filter_names.append("world")
                    elif filter_body_idx >= 0:
                        sensor_filter_indices.append(filter_body_idx)
                        sensor_filter_names.append(self.model.body_label[filter_body_idx])
                    else:
                        sensor_filter_indices.append(-1)
                        sensor_filter_names.append("")

                all_filter_paths_per_sensor.append(sensor_filter_paths)
                all_filter_indices_per_sensor.append(sensor_filter_indices)
                all_filter_names_per_sensor.append(sensor_filter_names)

        num_sensors = len(all_sensor_paths)
        if num_sensors == 0:
            carb.log_error("No sensors matched")
            return None

        world_body_idx = self.model.body_count

        filter_indices = np.ones((num_sensors, num_filters), dtype=int) * -1
        # Extra slot at index body_count for the world body (shape_body = -1).
        body_sensor_map = np.ones((world_body_idx + 1), dtype=int) * -1

        for i in range(num_sensors):
            body_sensor_map[all_sensor_indices[i]] = i
            for j in range(len(all_filter_indices_per_sensor[i])):
                filter_indices[i, j] = all_filter_indices_per_sensor[i][j]

        sensor_indices = wp.array(all_sensor_indices, dtype=wp.int32, device=self.device)  # type: ignore[var-annotated]
        filter_indices = wp.array(filter_indices, dtype=wp.int32, device=self.device)  # type: ignore[assignment]
        body_sensor_map = wp.array(body_sensor_map, dtype=wp.int32, device=self.device)  # type: ignore[assignment]

        return RigidContactSet(
            self.newton_stage,
            sensor_indices,
            all_sensor_paths,
            all_sensor_names,
            filter_indices,  # type: ignore[arg-type]
            all_filter_paths_per_sensor,
            all_filter_names_per_sensor,
            num_filters,
            body_sensor_map,  # type: ignore[arg-type]
            world_body_idx,
            max_contact_data_count,
        )

    def create_rigid_body_view(self, pattern: str | list[str]) -> RigidBodySet:
        """Create a rigid body view for bodies matching the pattern.

        Args:
            pattern: Path pattern or list of patterns to match.

        Returns:
            RigidBodySet for the matching bodies.
        """
        if isinstance(pattern, str):
            pattern_list = [pattern]
        else:
            pattern_list = pattern

        body_indices = []
        body_names = []
        body_paths = []

        all_prim_paths = []  # type: ignore[var-annotated]
        for patt in pattern_list:
            current_size = len(all_prim_paths)
            prim_paths = find_matching_paths(self.newton_stage.usd_stage, patt)
            all_prim_paths.extend(prim_paths)
            if len(all_prim_paths) == current_size:
                carb.log_error(f"Pattern '{patt}' did not match any rigid bodies")

        prim_paths = all_prim_paths

        for body_path in prim_paths:
            try:
                body_idx = self.model.body_label.index(body_path)
                body_indices.append(body_idx)
                body_paths.append(body_path)
                body_names.append(self.model.body_label[body_idx])
            except ValueError:
                carb.log_warn(f"Rigid body path '{body_path}' not found in Newton model")

        body_indices = wp.array(body_indices, dtype=int, device=self.device)  # type: ignore[assignment]

        return RigidBodySet(self.newton_stage, body_indices, body_paths, body_names)  # type: ignore[arg-type]

    def create_articulation_view(self, pattern: str | list[str]) -> ArticulationSet:
        """Create an articulation view for articulations matching the pattern.

        Args:
            pattern: Path pattern or list of patterns to match.

        Returns:
            ArticulationSet for the matching articulations.

        Raises:
            RuntimeError: If the Newton model is not initialized.
        """
        if isinstance(pattern, str):
            pattern_list = [pattern]
        else:
            pattern_list = pattern

        if self.model is None:
            raise RuntimeError(
                "Newton model is not initialized. Make sure the simulation has been started and "
                "initialize_newton() has been called."
            )

        matched_arti_paths = []  # type: ignore[var-annotated]
        for p in pattern_list:
            current_size = len(matched_arti_paths)
            prim_paths = find_matching_paths(self.newton_stage.usd_stage, p)
            matched_arti_paths.extend(prim_paths)
            if len(matched_arti_paths) == current_size:
                carb.log_error(f"Pattern '{p}' did not match any articulations")

        arti_starts = self.model.articulation_start.numpy().tolist()
        joint_q_start = self.model.joint_q_start.numpy().tolist()
        joint_qd_start = self.model.joint_qd_start.numpy().tolist()
        joint_axis_start = joint_qd_start
        joint_dof_dim = self.model.joint_dof_dim.numpy()
        joint_axis = self.model.joint_axis.numpy().tolist()
        joint_label = self.model.joint_label
        joint_types = self.model.joint_type.numpy().tolist()
        view_indices = []
        root_indices = []
        dof_position_indices = []
        dof_velocity_indices = []
        dof_axis_indices = []
        joint_indices = []
        meta_types = []
        max_dofs = 0

        for arti_idx in range(self.model.articulation_count):
            arti_path = self.model.articulation_label[arti_idx]
            joint_start = arti_starts[arti_idx]
            root_body_idx = self.model.joint_child.numpy()[joint_start]

            body_names = []
            if arti_path in matched_arti_paths:
                view_indices.append(arti_idx)
                root_indices.append(root_body_idx)
                body_names.append(self.model.body_label[root_body_idx])
                joint_count = arti_starts[arti_idx + 1] - arti_starts[arti_idx]
                start_joint_index = arti_starts[arti_idx]
                joint_index = 0
                joint_dof_start = 0
                total_dof_count = 0
                articulation_dof_types = []
                articulation_dof_names = []  # type: ignore[var-annotated]
                articulation_dof_paths = []
                articulation_joint_dof_counts = []
                articulation_joint_types = []
                articulation_joint_indices = []
                articulation_g_joint_indices = []
                joint_dof_offsets = [0]
                is_fixed_base = False
                for j in range(joint_count):
                    global_joint_index = j + start_joint_index
                    joint_type = joint_types[global_joint_index]
                    if j == 0:
                        if joint_type != newton.JointType.FREE:
                            is_fixed_base = True
                        continue

                    if joint_type in [
                        newton.JointType.PRISMATIC,
                        newton.JointType.REVOLUTE,
                        newton.JointType.BALL,
                        newton.JointType.D6,
                    ]:
                        next_q_start = joint_q_start[global_joint_index + 1]
                        joint_dof_count = next_q_start - joint_q_start[global_joint_index]
                        joint_dof_start = joint_q_start[global_joint_index]

                        for c in range(joint_dof_count):
                            dof_position_indices.append(joint_dof_start + c)

                        next_qd_start = joint_qd_start[global_joint_index + 1]

                        joint_dof_vel_count = next_qd_start - joint_qd_start[global_joint_index]
                        joint_dof_vel_start = joint_qd_start[global_joint_index]

                        for c in range(joint_dof_vel_count):
                            dof_velocity_indices.append(joint_dof_vel_start + c)

                        joint_axis_s = joint_qd_start[global_joint_index]
                        joint_axis_count = int(
                            joint_dof_dim[global_joint_index][0] + joint_dof_dim[global_joint_index][1]
                        )
                        joint_path = joint_label[global_joint_index]
                        for c in range(joint_axis_count):
                            dof_axis_indices.append(joint_axis_s + c)
                            if joint_type == newton.JointType.BALL or joint_type == newton.JointType.D6:
                                if joint_axis[joint_axis_s + c] == [1.0, 0.0, 0.0]:
                                    articulation_dof_paths.append(joint_path + ":0")
                                    articulation_dof_types.append(omni.physics.tensors.DofType.Rotation)
                                if joint_axis[joint_axis_s + c] == [0.0, 1.0, 0.0]:
                                    articulation_dof_paths.append(joint_path + ":1")
                                    articulation_dof_types.append(omni.physics.tensors.DofType.Rotation)
                                if joint_axis[joint_axis_s + c] == [0.0, 0.0, 1.0]:
                                    articulation_dof_paths.append(joint_path + ":2")
                                    articulation_dof_types.append(omni.physics.tensors.DofType.Rotation)
                            else:
                                articulation_dof_paths.append(joint_path)
                                if joint_type == newton.JointType.PRISMATIC:
                                    articulation_dof_types.append(omni.physics.tensors.DofType.Translation)
                                else:
                                    articulation_dof_types.append(omni.physics.tensors.DofType.Rotation)

                        joint_indices.append(global_joint_index)

                        joint_dof_offsets.append(joint_dof_offsets[-1] + joint_dof_vel_count)
                        articulation_joint_dof_counts.append(joint_dof_count)
                        articulation_joint_types.append(joint_type)
                        articulation_joint_indices.append(joint_index)
                        articulation_g_joint_indices.append(global_joint_index)
                        joint_index += 1
                        total_dof_count += joint_dof_vel_count

                articulation_link_indices = []
                for j in range(joint_count):
                    global_joint_index = j + start_joint_index
                    child_body_idx = self.model.joint_child.numpy()[global_joint_index]
                    articulation_link_indices.append(child_body_idx)

                l_shapes = []
                for idx in articulation_link_indices:
                    shape_ids = self.model.body_shapes[idx]
                    l_shapes.extend(shape_ids)
                l_paths = [self.model.body_label[idx] for idx in articulation_link_indices]
                l_names = [path.rsplit("/", 1)[-1] for path in l_paths]
                l_indices = dict(zip(l_names, articulation_link_indices))
                j_paths = [self.model.joint_label[j] for j in articulation_g_joint_indices]
                j_names = [path.rsplit("/", 1)[-1] for path in j_paths]

                j_indices = dict(zip(j_names, articulation_joint_indices))

                j_types = [JointTypeDic[jt] for jt in articulation_joint_types]

                d_names = [path.rsplit("/", 1)[-1] for path in articulation_dof_paths]

                d_names = [path.rsplit("/", 1)[-1] for path in articulation_dof_paths]
                d_indices = dict(zip(d_names, range(total_dof_count)))

                meta_types.append(
                    ArticulationMetaType(
                        l_paths,
                        l_shapes,
                        j_paths,
                        articulation_dof_paths,
                        l_indices,
                        j_indices,
                        d_indices,
                        j_types,
                        joint_dof_offsets[:-1],
                        articulation_joint_dof_counts,
                        articulation_dof_types,
                        is_fixed_base,
                    )
                )

                max_dofs = max(max_dofs, total_dof_count)

        arti_indices = wp.array(view_indices, dtype=int, device=self.device)  # type: ignore[var-annotated]
        root_body_indices = wp.array(root_indices, dtype=int, device=self.device)  # type: ignore[var-annotated]
        dof_position_indices = wp.array(dof_position_indices, dtype=int, device=self.device)  # type: ignore[assignment]
        dof_velocity_indices = wp.array(dof_velocity_indices, dtype=int, device=self.device)  # type: ignore[assignment]
        dof_axis_indices = wp.array(dof_axis_indices, dtype=int, device=self.device)  # type: ignore[assignment]
        joint_indices = wp.array(joint_indices, dtype=int, device=self.device)  # type: ignore[assignment]

        max_shapes = 0
        max_links = 0
        shape_counts = []
        link_counts = []
        for meta in meta_types:
            max_shapes = max(max_shapes, len(meta.link_shapes))
            max_links = max(max_links, len(meta.link_names))
            shape_counts.append(len(meta.link_shapes))
            link_counts.append(len(meta.link_names))

        shape_indices = np.ones((len(view_indices), max_shapes), dtype=int) * -1
        link_indices = np.ones((len(view_indices), max_links), dtype=int) * -1

        for i, meta in enumerate(meta_types):
            for j in range(shape_counts[i]):
                shape_indices[i, j] = meta.link_shapes[j]

            for j in range(link_counts[i]):
                link_indices[i, j] = meta.link_indices[meta.link_names[j]]

        count = len(view_indices)
        shape_indices = wp.array(shape_indices, dtype=wp.int32, device=self.device)  # type: ignore[assignment]
        link_indices = wp.array(link_indices, dtype=wp.int32, device=self.device)  # type: ignore[assignment]

        return ArticulationSet(
            self.newton_stage,
            arti_indices,  # type: ignore[arg-type]
            root_body_indices,  # type: ignore[arg-type]
            dof_position_indices,  # type: ignore[arg-type]
            dof_velocity_indices,  # type: ignore[arg-type]
            dof_axis_indices,  # type: ignore[arg-type]
            joint_indices,  # type: ignore[arg-type]
            shape_indices,  # type: ignore[arg-type]
            link_indices,  # type: ignore[arg-type]
            meta_types,
            count,
            max_dofs,
        )

    def set_subspace_roots(self, pattern: str | list[str]) -> bool:
        """Set subspace roots for the simulation.

        Args:
            pattern: Path pattern for subspace roots.

        Returns:
            True if successful.
        """
        return True
