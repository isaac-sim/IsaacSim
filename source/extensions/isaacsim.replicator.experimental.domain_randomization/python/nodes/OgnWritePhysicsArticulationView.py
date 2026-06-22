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

"""Write randomized values into registered articulation physics views."""

from typing import Any

import carb
import numpy as np
import omni.graph.core as og
import warp as wp
from isaacsim.core.experimental.utils.ops import place
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.replicator.experimental.domain_randomization import ARTICULATION_ATTRIBUTES, TENDON_ATTRIBUTES
from isaacsim.replicator.experimental.domain_randomization import physics_view as physics

OPERATION_TYPES = ["direct", "additive", "scaling"]


def apply_randomization_operation(
    view_name: Any, operation: Any, attribute_name: Any, samples: Any, indices: Any, on_reset: Any
) -> Any:
    """Apply randomization operation for indexed values.

    Args:
        view_name: Name of the registered articulation view.
        operation: Operation to apply to the stored reset values.
        attribute_name: Name of the randomized attribute.
        samples: Sample values to apply.
        indices: Indices of the selected articulations.
        on_reset: Whether to return reset values without applying samples.

    Returns:
        Values to write for the selected articulation indices.
    """
    if on_reset:
        return physics._articulation_views_reset_values[view_name][attribute_name][indices]
    if operation == "additive":
        return physics._articulation_views_reset_values[view_name][attribute_name][indices] + samples
    elif operation == "scaling":
        return physics._articulation_views_reset_values[view_name][attribute_name][indices] * samples
    else:
        return samples


def apply_randomization_operation_full_tensor(
    view_name: Any, operation: Any, attribute_name: Any, samples: Any, indices: Any, on_reset: Any
) -> None:
    """Apply randomization operation for full tensor values.

    Args:
        view_name: Name of the registered articulation view.
        operation: Operation to apply to the stored reset values.
        attribute_name: Name of the randomized attribute.
        samples: Sample values to apply.
        indices: Indices of the selected articulations.
        on_reset: Whether to return reset values without applying samples.
    """
    if on_reset:
        return physics._articulation_views_reset_values[view_name][attribute_name]
    initial_values = np.copy(physics._articulation_views_reset_values[view_name][attribute_name])
    if operation == "additive":
        initial_values[indices] += samples
    elif operation == "scaling":
        initial_values[indices] *= samples
    else:
        initial_values[indices] = samples
    return initial_values


def modify_initial_values(view_name: Any, operation: Any, attribute_name: Any, samples: Any, indices: Any) -> Any:
    """Modify initial values based on operation type.

    Args:
        view_name: Name of the registered articulation view.
        operation: Operation to apply to the stored initial values.
        attribute_name: Name of the randomized attribute.
        samples: Sample values to store.
        indices: Indices of the selected articulations.

    Returns:
        None.
    """
    if operation == "additive":
        physics._articulation_views_reset_values[view_name][attribute_name][indices] = (
            physics._articulation_views_initial_values[view_name][attribute_name][indices] + samples
        )
    elif operation == "scaling":
        physics._articulation_views_reset_values[view_name][attribute_name][indices] = (
            physics._articulation_views_initial_values[view_name][attribute_name][indices] * samples
        )
    else:
        physics._articulation_views_reset_values[view_name][attribute_name][indices] = samples


def get_bucketed_values(
    view_name: Any,
    attribute_name: Any,
    samples: Any,
    distribution: Any,
    dist_param_1: Any,
    dist_param_2: Any,
    num_buckets: Any,
) -> Any:
    """Quantize material-property samples into distribution-derived buckets.

    Args:
        view_name: Name of the registered articulation view.
        attribute_name: Name of the randomized attribute.
        samples: Material property samples to quantize.
        distribution: Distribution name used to derive bucket bounds.
        dist_param_1: First distribution parameter.
        dist_param_2: Second distribution parameter.
        num_buckets: Number of buckets to quantize into.

    Returns:
        Copy of ``samples`` with values snapped to distribution-derived buckets.
    """
    new_samples = samples.copy()

    if distribution == "gaussian":
        lo = dist_param_1 - 2 * np.sqrt(dist_param_2)
        hi = dist_param_1 + 2 * np.sqrt(dist_param_2)
    elif distribution in ("uniform", "loguniform"):
        lo = dist_param_1
        hi = dist_param_2
    else:
        raise ValueError(f"Unsupported distribution for bucketing: {distribution!r}")

    dim = samples.shape[-1]
    lo = lo.reshape(-1, dim)[0]
    hi = hi.reshape(-1, dim)[0]
    for d in range(dim):
        buckets = np.array([(hi[d] - lo[d]) * i / num_buckets + lo[d] for i in range(num_buckets)])
        idx = np.clip(np.searchsorted(buckets, new_samples[..., d], side="right") - 1, 0, num_buckets - 1)
        new_samples[..., d] = buckets[idx]

    return new_samples


class OgnWritePhysicsArticulationView:
    """OmniGraph writer for registered ``Articulation`` view attributes."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Apply sampled values to selected articulation environments.

        The node expects a registered articulation view name, an attribute from
        ``ARTICULATION_ATTRIBUTES``, one of ``direct``, ``additive``, or
        ``scaling`` operations, sampled values, and selected environment
        indices. Empty indices keep ``execOut`` enabled but perform no write.
        On reset, the stored reset baseline is updated before values are
        restored or applied. Tendon attributes are staged together and written
        as fixed tendon properties when the articulation has fixed tendons.
        Invalid views, attributes, or operations log an error, disable
        ``execOut``, and return ``False``.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True when values are written, False when inputs are empty or invalid.
        """
        view_name = db.inputs.prims
        attribute_name = db.inputs.attribute
        operation = db.inputs.operation
        values = db.inputs.values

        distribution = db.inputs.distribution
        dist_param_1 = db.inputs.dist_param_1
        dist_param_2 = db.inputs.dist_param_2
        num_buckets = db.inputs.num_buckets

        if db.inputs.indices is None or len(db.inputs.indices) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False
        indices = np.array(db.inputs.indices)
        on_reset = db.inputs.on_reset

        try:
            view = physics.resolve_articulation_view(view_name)
            if view is None:
                raise ValueError(f"Expected a registered articulation_view, but instead received {view_name}")
            if attribute_name not in ARTICULATION_ATTRIBUTES:
                raise ValueError(
                    f"Expected an attribute in {ARTICULATION_ATTRIBUTES}, but instead received {attribute_name}"
                )
            if operation not in OPERATION_TYPES:
                raise ValueError(f"Expected an operation type in {OPERATION_TYPES}, but instead received {operation}")

            samples = np.array(values).reshape(len(indices), -1)
            physics_view = view._physics_articulation_view
        except Exception as error:
            db.log_error(f"WritePhysics Error: {error}")
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        if on_reset:
            modify_initial_values(view_name, operation, attribute_name, samples, indices)

        wp_indices = place(indices, dtype=wp.int32, device="cpu")

        if attribute_name == "stiffness":
            stiffnesses = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_stiffnesses(place(stiffnesses, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "damping":
            dampings = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_dampings(place(dampings, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "joint_friction":
            frictions = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_friction_coefficients(place(frictions, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "position":
            positions = apply_randomization_operation(view_name, operation, attribute_name, samples, indices, on_reset)
            view.set_world_poses(positions=positions, indices=indices)
        elif attribute_name == "orientation":
            rpys = apply_randomization_operation(view_name, operation, attribute_name, samples, indices, on_reset)
            orientations = euler_angles_to_quaternion(rpys, degrees=False, extrinsic=True).numpy()
            view.set_world_poses(orientations=orientations, indices=indices)
        elif attribute_name == "linear_velocity":
            linear_velocities = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            view.set_velocities(linear_velocities=linear_velocities, indices=indices)
        elif attribute_name == "angular_velocity":
            angular_velocities = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            view.set_velocities(angular_velocities=angular_velocities, indices=indices)
        elif attribute_name == "velocity":
            velocities = apply_randomization_operation(view_name, operation, attribute_name, samples, indices, on_reset)
            linear_vel = velocities[:, :3]
            angular_vel = velocities[:, 3:]
            view.set_velocities(linear_velocities=linear_vel, angular_velocities=angular_vel, indices=indices)
        elif attribute_name == "joint_positions":
            joint_positions = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            view.set_dof_positions(positions=joint_positions, indices=indices)
        elif attribute_name == "joint_velocities":
            joint_velocities = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            view.set_dof_velocities(velocities=joint_velocities, indices=indices)
        elif attribute_name == "lower_dof_limits":
            dof_limits_np = np.asarray(physics_view.get_dof_limits())
            dof_limits_np[..., 0] = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_limits(place(dof_limits_np, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "upper_dof_limits":
            dof_limits_np = np.asarray(physics_view.get_dof_limits())
            dof_limits_np[..., 1] = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_limits(place(dof_limits_np, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "max_efforts":
            max_efforts = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            view.set_dof_max_efforts(max_efforts, indices=indices)
        elif attribute_name == "joint_armatures":
            joint_armatures = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_armatures(place(joint_armatures, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "joint_max_velocities":
            joint_max_velocities = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_dof_max_velocities(place(joint_max_velocities, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "joint_efforts":
            view.set_dof_efforts(efforts=samples, indices=indices)
        elif attribute_name == "body_masses":
            body_masses = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_masses(place(body_masses, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "body_inertias":
            diagonal_inertias = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            inertia_matrices = np.zeros((len(view), physics_view.max_links, 9), dtype=np.float32)
            inertia_matrices[:, :, [0, 4, 8]] = diagonal_inertias.reshape(len(view), physics_view.max_links, 3)
            physics_view.set_inertias(place(inertia_matrices, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "material_properties":
            material_properties = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            ).reshape(len(view), physics_view.max_shapes, 3)
            if num_buckets is not None and num_buckets > 0:
                material_properties = get_bucketed_values(
                    view_name,
                    attribute_name,
                    material_properties,
                    distribution,
                    dist_param_1,
                    dist_param_2,
                    num_buckets,
                )
            physics_view.set_material_properties(place(material_properties, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "contact_offset":
            contact_offsets = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_contact_offsets(place(contact_offsets, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name == "rest_offset":
            rest_offsets = apply_randomization_operation_full_tensor(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics_view.set_rest_offsets(place(rest_offsets, dtype=wp.float32, device="cpu"), wp_indices)
        elif attribute_name in TENDON_ATTRIBUTES:
            if not physics._current_tendon_properties:
                carb.log_error(
                    f"Cannot randomize tendon attribute '{attribute_name}' for view '{view_name}': "
                    "articulation has no fixed tendons."
                )
                db.outputs.execOut = og.ExecutionAttributeState.ENABLED
                return True
            tendon_values = apply_randomization_operation(
                view_name, operation, attribute_name, samples, indices, on_reset
            )
            physics._current_tendon_properties[attribute_name][indices] = tendon_values

        if attribute_name in TENDON_ATTRIBUTES and physics._current_tendon_properties:
            current_tendon_limits = np.stack(
                (
                    physics._current_tendon_properties["tendon_lower_limits"],
                    physics._current_tendon_properties["tendon_upper_limits"],
                ),
                axis=-1,
            )
            physics_view.set_fixed_tendon_properties(
                physics._current_tendon_properties["tendon_stiffnesses"],
                physics._current_tendon_properties["tendon_dampings"],
                physics._current_tendon_properties["tendon_limit_stiffnesses"],
                current_tendon_limits,
                physics._current_tendon_properties["tendon_rest_lengths"],
                physics._current_tendon_properties["tendon_offsets"],
                indices,
            )

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
