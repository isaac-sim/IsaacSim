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
"""Warp kernels for Newton contact sensor operations.

These kernels process Newton's contact data (shape pairs, normals, forces) and
organize them by sensor/filter to match the PhysX tensor API conventions.

Newton stores ground plane / static shapes with shape_body = -1 (world body).
All kernels accept a ``world_body_idx`` parameter that maps body index -1 to
an extra slot at the end of the sensor / filter mapping arrays, so that
contacts between dynamic bodies and the world (ground plane) are reported.

Contact forces are pre-computed by Newton's solver and scaled by dt to produce
impulses; sign convention is aligned with PhysX (sensor receiving the direct
force gets positive, the other sensor gets negated). Contact points are
populated from MuJoCo world-space positions when Newton Contacts do not
provide rigid_contact_point0/1.

See:
- newton/_src/sensors/contact_sensor.py - ContactSensor using pre-computed forces
- newton/_src/solvers/mujoco/solver_mujoco.py - Where force/normal fields are added
"""

import warp as wp


@wp.kernel
def populate_contact_points_kernel(
    # inputs - from Newton's contacts
    contact_count: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    mj_contact_pos: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    # outputs
    contact_point0: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_point1: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
) -> None:
    """Populate body-local contact points from MuJoCo world-space contact positions.

    Used when Newton Contacts do not provide rigid_contact_point0/1; transforms
    mj_contact_pos into each body's local frame.

    Args:
        contact_count: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        mj_contact_pos: World-space contact position from MuJoCo.
        shape_body: Shape to body mapping.
        body_q: Body transforms (world to body).
        contact_point0: Output contact point on first shape (body-local).
        contact_point1: Output contact point on second shape (body-local).
    """
    tid = wp.tid()
    count = contact_count[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]
    if shape_a < 0 or shape_b < 0:
        return

    pos_world = mj_contact_pos[tid]
    body_a = shape_body[shape_a]
    body_b = shape_body[shape_b]

    # Transform world position to body-local frame; world body (body < 0) keeps world coords
    if body_a >= 0:
        contact_point0[tid] = wp.transform_point(wp.transform_inverse(body_q[body_a]), pos_world)
    else:
        contact_point0[tid] = pos_world

    if body_b >= 0:
        contact_point1[tid] = wp.transform_point(wp.transform_inverse(body_q[body_b]), pos_world)
    else:
        contact_point1[tid] = pos_world


@wp.kernel
def count_contacts_per_pair_kernel(
    # inputs - from Newton's contacts
    contact_count_total: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    body_filter_map: wp.array2d(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    # outputs
    contact_counts: wp.array2d(dtype=wp.uint32),  # type: ignore[valid-type]
) -> None:
    """Count how many contacts exist for each sensor-filter pair (for prefix scan).

    body_filter_map is indexed directly by body index (dimension body_count + 1),
    matching PhysX where the filter map spans all bodies in the scene.

    Args:
        contact_count_total: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        shape_body: Shape to body mapping.
        body_sensor_map: Body to sensor index mapping.
        body_filter_map: (sensor_idx, body_idx) to filter index (-1 if none).
        world_body_idx: Body index for world body (shape_body == -1).
        contact_counts: Output contact counts per sensor-filter pair (uint32).
    """
    tid = wp.tid()
    count = contact_count_total[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    # Get bodies for both shapes; remap world body (-1) to world_body_idx slot
    body_a = shape_body[shape_a]
    body_b = shape_body[shape_b]
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Increment count for appropriate sensor-filter pairs
    if sensor_a >= 0 and body_b >= 0 and body_b < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_a, body_b]
        if filter_idx >= 0:
            wp.atomic_add(contact_counts, sensor_a, filter_idx, wp.uint32(1))

    if sensor_b >= 0 and body_a >= 0 and body_a < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_b, body_a]
        if filter_idx >= 0:
            wp.atomic_add(contact_counts, sensor_b, filter_idx, wp.uint32(1))


@wp.kernel
def net_contact_forces_kernel(
    # inputs - from Newton's contacts (pre-computed by solver)
    contact_count: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_force: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    dt: float,
    # outputs
    net_forces: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Compute net contact force per sensor (force * dt) with PhysX sign convention.

    Newton stores force on shape0 (from shape1 toward shape0). Sensor for shape0
    gets the direct force (add); sensor for shape1 gets the reaction (subtract).

    Args:
        contact_count: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        contact_force: Contact force vector (world) from solver.
        shape_body: Shape to body mapping.
        body_sensor_map: Body to sensor index mapping.
        world_body_idx: Body index for world body (shape_body == -1).
        dt: Physics step size for force-to-impulse scaling.
        net_forces: Output net force per sensor (sensor_count, 3).
    """
    tid = wp.tid()
    count = contact_count[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    # Get bodies for both shapes; remap world body (-1) to world_body_idx slot
    body_a = shape_body[shape_a]
    body_b = shape_body[shape_b]
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Use pre-computed force from Newton's solver, scaled by dt
    f_total = contact_force[tid] * dt

    # sensor_a (shape0) receives the direct force: add
    # sensor_b (shape1) receives the reaction: subtract
    if sensor_a >= 0:
        wp.atomic_add(net_forces, sensor_a, 0, f_total[0])
        wp.atomic_add(net_forces, sensor_a, 1, f_total[1])
        wp.atomic_add(net_forces, sensor_a, 2, f_total[2])

    if sensor_b >= 0:
        wp.atomic_sub(net_forces, sensor_b, 0, f_total[0])
        wp.atomic_sub(net_forces, sensor_b, 1, f_total[1])
        wp.atomic_sub(net_forces, sensor_b, 2, f_total[2])


@wp.kernel
def contact_force_matrix_kernel(
    # inputs - from Newton's contacts (pre-computed by solver)
    contact_count: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_force: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    body_filter_map: wp.array2d(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    dt: float,
    filter_count: int,
    # outputs
    force_matrix: wp.array3d(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Compute per-filter contact force matrix (force * dt) with PhysX sign convention.

    Same sign rule as net_contact_forces_kernel, organized into a sensor x filter matrix.
    body_filter_map is indexed directly by body index (dimension body_count + 1).

    Args:
        contact_count: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        contact_force: Contact force vector (world) from solver.
        shape_body: Shape to body mapping.
        body_sensor_map: Body to sensor index mapping.
        body_filter_map: (sensor_idx, body_idx) to filter index (-1 if none).
        world_body_idx: Body index for world body (shape_body == -1).
        dt: Physics step size for force-to-impulse scaling.
        filter_count: Number of filters per sensor.
        force_matrix: Output force matrix (sensor_count, filter_count, 3).
    """
    tid = wp.tid()
    count = contact_count[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    # Get bodies for both shapes; remap world body (-1) to world_body_idx slot
    body_a = shape_body[shape_a]
    body_b = shape_body[shape_b]
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Use pre-computed force from Newton's solver, scaled by dt
    f_total = contact_force[tid] * dt

    # Add force to appropriate sensor-filter pair (with bounds checking)
    if sensor_a >= 0 and body_b >= 0 and body_b < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_a, body_b]
        if filter_idx >= 0:
            wp.atomic_add(force_matrix, sensor_a, filter_idx, 0, f_total[0])
            wp.atomic_add(force_matrix, sensor_a, filter_idx, 1, f_total[1])
            wp.atomic_add(force_matrix, sensor_a, filter_idx, 2, f_total[2])

    if sensor_b >= 0 and body_a >= 0 and body_a < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_b, body_a]
        if filter_idx >= 0:
            wp.atomic_sub(force_matrix, sensor_b, filter_idx, 0, f_total[0])
            wp.atomic_sub(force_matrix, sensor_b, filter_idx, 1, f_total[1])
            wp.atomic_sub(force_matrix, sensor_b, filter_idx, 2, f_total[2])


@wp.kernel
def contact_data_kernel(
    # inputs - from Newton's contacts (pre-computed by solver)
    contact_count: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_point0: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_point1: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_normal: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_force: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_thickness0: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    contact_thickness1: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    body_filter_map: wp.array2d(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    dt: float,
    max_contact_data_count: int,
    # outputs
    contact_forces: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_points: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_normals: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_separations: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_counts: wp.array2d(dtype=wp.uint32),  # type: ignore[valid-type]
    contact_start_indices: wp.array2d(dtype=wp.uint32),  # type: ignore[valid-type]
) -> None:
    """Gather detailed contact data per sensor-filter pair using pre-computed forces.

    Gathers geometric information (contact points, normals, separations) and force
    magnitudes per sensor-filter pair, matching PhysX contact data format.
    body_filter_map is indexed directly by body index (dimension body_count + 1).

    Args:
        contact_count: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        contact_point0: Contact point on first shape (body-local).
        contact_point1: Contact point on second shape (body-local).
        contact_normal: Contact normal (world).
        contact_force: Contact force vector (world) from solver.
        contact_thickness0: Thickness of first shape at contact.
        contact_thickness1: Thickness of second shape at contact.
        shape_body: Shape to body mapping.
        body_q: Body transforms.
        body_sensor_map: Body to sensor index mapping.
        body_filter_map: (sensor_idx, body_idx) to filter index (-1 if none).
        world_body_idx: Body index for world body (shape_body == -1).
        dt: Physics step size for force magnitude scaling.
        max_contact_data_count: Maximum contacts to store in output arrays.
        contact_forces: Output force magnitudes (N, 1).
        contact_points: Output contact points world (N, 3).
        contact_normals: Output contact normals (N, 3).
        contact_separations: Output penetration depth (N, 1).
        contact_counts: Contact counts per sensor-filter pair (uint32).
        contact_start_indices: Start index per sensor-filter pair (uint32).
    """
    tid = wp.tid()
    count = contact_count[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    raw_body_a = shape_body[shape_a]
    raw_body_b = shape_body[shape_b]
    body_a = raw_body_a
    body_b = raw_body_b
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Get pre-computed force from Newton's solver
    f_vec = contact_force[tid]
    n = contact_normal[tid]
    force_mag = wp.length(f_vec) * dt

    # Get contact geometry
    bx_a = contact_point0[tid]
    bx_b = contact_point1[tid]
    thickness_a = contact_thickness0[tid]
    thickness_b = contact_thickness1[tid]

    # Transform contact points to world space
    if raw_body_a >= 0:
        X_wb_a = body_q[raw_body_a]
        bx_a_world = wp.transform_point(X_wb_a, bx_a) - thickness_a * n
    else:
        bx_a_world = bx_a - thickness_a * n

    if raw_body_b >= 0:
        X_wb_b = body_q[raw_body_b]
        bx_b_world = wp.transform_point(X_wb_b, bx_b) + thickness_b * n
    else:
        bx_b_world = bx_b + thickness_b * n

    # Penetration depth and contact midpoint
    d = wp.dot(n, bx_a_world - bx_b_world)
    contact_point = (bx_a_world + bx_b_world) * 0.5

    # Store data for sensor A (with bounds checking)
    if sensor_a >= 0 and body_b >= 0 and body_b < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_a, body_b]
        if filter_idx >= 0:
            # Atomically increment count and get write index
            data_idx = wp.atomic_add(contact_counts, sensor_a, filter_idx, wp.uint32(1))
            start_idx = int(contact_start_indices[sensor_a, filter_idx])
            write_idx = start_idx + int(data_idx)

            if write_idx < max_contact_data_count:
                contact_forces[write_idx, 0] = -force_mag
                contact_points[write_idx, 0] = contact_point[0]
                contact_points[write_idx, 1] = contact_point[1]
                contact_points[write_idx, 2] = contact_point[2]
                contact_normals[write_idx, 0] = n[0]
                contact_normals[write_idx, 1] = n[1]
                contact_normals[write_idx, 2] = n[2]
                contact_separations[write_idx, 0] = d

    # Store data for sensor B (negated force and normal)
    if sensor_b >= 0 and body_a >= 0 and body_a < body_filter_map.shape[1]:
        filter_idx = body_filter_map[sensor_b, body_a]
        if filter_idx >= 0:
            data_idx = wp.atomic_add(contact_counts, sensor_b, filter_idx, wp.uint32(1))
            start_idx = int(contact_start_indices[sensor_b, filter_idx])
            write_idx = start_idx + int(data_idx)

            if write_idx < max_contact_data_count:
                contact_forces[write_idx, 0] = force_mag
                contact_points[write_idx, 0] = contact_point[0]
                contact_points[write_idx, 1] = contact_point[1]
                contact_points[write_idx, 2] = contact_point[2]
                contact_normals[write_idx, 0] = -n[0]
                contact_normals[write_idx, 1] = -n[1]
                contact_normals[write_idx, 2] = -n[2]
                contact_separations[write_idx, 0] = -d


@wp.kernel
def count_raw_contacts_per_sensor_kernel(
    # inputs - from Newton's contacts
    contact_count_total: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    # outputs
    contact_counts: wp.array(dtype=wp.uint32),  # type: ignore[valid-type]
) -> None:
    """Count contacts per sensor without filter matching (for raw contact data prefix scan).

    Unlike count_contacts_per_pair_kernel, this accumulates into a 1D array
    indexed only by sensor (no filter dimension).

    Args:
        contact_count_total: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        shape_body: Shape to body mapping.
        body_sensor_map: Body to sensor index mapping.
        world_body_idx: Body index for world body (shape_body == -1).
        contact_counts: Output contact count per sensor (1D uint32).
    """
    tid = wp.tid()
    count = contact_count_total[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    # Get bodies for both shapes; remap world body (-1) to world_body_idx slot
    body_a = shape_body[shape_a]
    body_b = shape_body[shape_b]
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Increment count for each sensor involved in this contact
    if sensor_a >= 0:
        wp.atomic_add(contact_counts, sensor_a, wp.uint32(1))

    if sensor_b >= 0:
        wp.atomic_add(contact_counts, sensor_b, wp.uint32(1))


@wp.kernel
def raw_contact_data_kernel(
    # inputs - from Newton's contacts (pre-computed by solver)
    contact_count: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape0: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_shape1: wp.array(dtype=int),  # type: ignore[valid-type]
    contact_point0: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_point1: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_normal: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_force: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    contact_thickness0: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    contact_thickness1: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # model data
    shape_body: wp.array(dtype=int),  # type: ignore[valid-type]
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # type: ignore[valid-type]
    world_body_idx: int,
    dt: float,
    max_contact_data_count: int,
    # outputs
    contact_forces: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_points: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_normals: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_separations: wp.array2d(dtype=wp.float32),  # type: ignore[valid-type]
    contact_counts: wp.array(dtype=wp.uint32),  # type: ignore[valid-type]
    contact_start_indices: wp.array(dtype=wp.uint32),  # type: ignore[valid-type]
    other_actor_ids: wp.array(dtype=wp.uint64),  # type: ignore[valid-type]
) -> None:
    """Gather raw contact data per sensor without filter matching.

    Unlike contact_data_kernel, this does not use body_filter_map. All contacts
    touching a sensor are stored, and the other body's index is recorded in
    other_actor_ids (as uint64 for PhysX API compatibility).

    Args:
        contact_count: Total number of contacts.
        contact_shape0: First shape index per contact.
        contact_shape1: Second shape index per contact.
        contact_point0: Contact point on first shape (body-local).
        contact_point1: Contact point on second shape (body-local).
        contact_normal: Contact normal (world).
        contact_force: Contact force vector (world) from solver.
        contact_thickness0: Thickness of first shape at contact.
        contact_thickness1: Thickness of second shape at contact.
        shape_body: Shape to body mapping.
        body_q: Body transforms.
        body_sensor_map: Body to sensor index mapping.
        world_body_idx: Body index for world body (shape_body == -1).
        dt: Physics step size for force magnitude scaling.
        max_contact_data_count: Maximum contacts to store in output arrays.
        contact_forces: Output force magnitudes (N, 1).
        contact_points: Output contact points world (N, 3).
        contact_normals: Output contact normals (N, 3).
        contact_separations: Output penetration depth (N, 1).
        contact_counts: Contact count per sensor (1D uint32).
        contact_start_indices: Start index per sensor (1D uint32).
        other_actor_ids: Output body index of the other body (uint64).
    """
    tid = wp.tid()
    count = contact_count[0]
    if tid >= count:
        return

    shape_a = contact_shape0[tid]
    shape_b = contact_shape1[tid]

    if shape_a == shape_b or shape_a < 0 or shape_b < 0:
        return

    raw_body_a = shape_body[shape_a]
    raw_body_b = shape_body[shape_b]
    body_a = raw_body_a
    body_b = raw_body_b
    if body_a < 0:
        body_a = world_body_idx
    if body_b < 0:
        body_b = world_body_idx

    # Check if either body is a sensor (with bounds checking)
    sensor_a = -1
    if body_a >= 0 and body_a < body_sensor_map.shape[0]:
        sensor_a = body_sensor_map[body_a]

    sensor_b = -1
    if body_b >= 0 and body_b < body_sensor_map.shape[0]:
        sensor_b = body_sensor_map[body_b]

    if sensor_a < 0 and sensor_b < 0:
        return

    # Get pre-computed force from Newton's solver
    f_vec = contact_force[tid]
    n = contact_normal[tid]
    force_mag = wp.length(f_vec) * dt

    # Get contact geometry
    bx_a = contact_point0[tid]
    bx_b = contact_point1[tid]
    thickness_a = contact_thickness0[tid]
    thickness_b = contact_thickness1[tid]

    # Transform contact points to world space
    if raw_body_a >= 0:
        X_wb_a = body_q[raw_body_a]
        bx_a_world = wp.transform_point(X_wb_a, bx_a) - thickness_a * n
    else:
        bx_a_world = bx_a - thickness_a * n

    if raw_body_b >= 0:
        X_wb_b = body_q[raw_body_b]
        bx_b_world = wp.transform_point(X_wb_b, bx_b) + thickness_b * n
    else:
        bx_b_world = bx_b + thickness_b * n

    # Penetration depth and contact midpoint
    d = wp.dot(n, bx_a_world - bx_b_world)
    contact_point = (bx_a_world + bx_b_world) * 0.5

    # Store data for sensor A; other body is body_b
    if sensor_a >= 0:
        data_idx = wp.atomic_add(contact_counts, sensor_a, wp.uint32(1))
        start_idx = int(contact_start_indices[sensor_a])
        write_idx = start_idx + int(data_idx)

        if write_idx < max_contact_data_count:
            contact_forces[write_idx, 0] = -force_mag
            contact_points[write_idx, 0] = contact_point[0]
            contact_points[write_idx, 1] = contact_point[1]
            contact_points[write_idx, 2] = contact_point[2]
            contact_normals[write_idx, 0] = n[0]
            contact_normals[write_idx, 1] = n[1]
            contact_normals[write_idx, 2] = n[2]
            contact_separations[write_idx, 0] = d
            other_actor_ids[write_idx] = wp.uint64(body_b)

    # Store data for sensor B (negated force and normal); other body is body_a
    if sensor_b >= 0:
        data_idx = wp.atomic_add(contact_counts, sensor_b, wp.uint32(1))
        start_idx = int(contact_start_indices[sensor_b])
        write_idx = start_idx + int(data_idx)

        if write_idx < max_contact_data_count:
            contact_forces[write_idx, 0] = force_mag
            contact_points[write_idx, 0] = contact_point[0]
            contact_points[write_idx, 1] = contact_point[1]
            contact_points[write_idx, 2] = contact_point[2]
            contact_normals[write_idx, 0] = -n[0]
            contact_normals[write_idx, 1] = -n[1]
            contact_normals[write_idx, 2] = -n[2]
            contact_separations[write_idx, 0] = -d
            other_actor_ids[write_idx] = wp.uint64(body_a)
