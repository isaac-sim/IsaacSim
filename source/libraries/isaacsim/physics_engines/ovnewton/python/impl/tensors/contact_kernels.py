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

"""Provide Warp kernels for Newton contact-sensor operations.

These kernels process Newton's contact data (shape pairs, normals, forces) and
organize them by sensor/filter to match the PhysX tensor API conventions.

Newton stores ground plane / static shapes with shape_body = -1 (world body).
All kernels accept a ``world_body_idx`` parameter that maps body index -1 to
an extra slot at the end of the sensor / filter mapping arrays, so that
contacts between dynamic bodies and the world (ground plane) are reported.

Contact forces are precomputed by Newton's solver and multiplied by a
caller-supplied scale. The Newton views use a scale of one because their public
results are forces rather than impulses. The sensor receiving the direct force
gets the positive value and the other sensor gets its negation. Contact points
can be reconstructed from MuJoCo world-space positions when Newton contacts do
not provide per-shape points.
"""

from __future__ import annotations

import warp as wp


@wp.kernel
def populate_contact_points_kernel(
    # inputs - from Newton's contacts
    contact_count: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    mj_contact_pos: wp.array(dtype=wp.vec3),  # world-space contact position from MuJoCo
    # model data
    shape_body: wp.array(dtype=int),
    body_q: wp.array(dtype=wp.transform),
    # outputs
    contact_point0: wp.array(dtype=wp.vec3),  # body-local contact point on first shape
    contact_point1: wp.array(dtype=wp.vec3),  # body-local contact point on second shape
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
        body_q: Body-to-world transforms.
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
    contact_count_total: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    # model data
    shape_body: wp.array(dtype=int),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index (-1 if not a sensor)
    body_filter_map: wp.array2d(dtype=int),  # maps (sensor_idx, body_idx) -> filter_idx (-1 if no filter)
    world_body_idx: int,
    # outputs
    contact_counts: wp.array2d(dtype=wp.uint32),  # per sensor-filter pair (uint32 to match PhysX API)
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
    contact_count: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    contact_force: wp.array(dtype=wp.vec3),  # pre-computed force vector from solver
    # model data
    shape_body: wp.array(dtype=int),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index
    world_body_idx: int,
    dt: float,
    # outputs
    net_forces: wp.array2d(dtype=wp.float32),  # shape: (sensor_count, 3)
) -> None:
    """Compute scaled net contact forces for each sensor.

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
        dt: Multiplicative scale applied to each contact force.
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

    # Use the precomputed force from Newton's solver with the requested scale.
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
    contact_count: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    contact_force: wp.array(dtype=wp.vec3),  # pre-computed force vector from solver
    # model data
    shape_body: wp.array(dtype=int),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index
    body_filter_map: wp.array2d(dtype=int),  # maps (sensor_idx, body_idx) -> filter_idx (-1 if no filter)
    world_body_idx: int,
    dt: float,
    filter_count: int,
    # outputs
    force_matrix: wp.array3d(dtype=wp.float32),  # shape: (sensor_count, filter_count, 3)
) -> None:
    """Compute scaled contact forces for each sensor-filter pair.

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
        dt: Multiplicative scale applied to each contact force.
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

    # Use the precomputed force from Newton's solver with the requested scale.
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
    contact_count: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    contact_point0: wp.array(dtype=wp.vec3),  # body-local contact point on first shape
    contact_point1: wp.array(dtype=wp.vec3),  # body-local contact point on second shape
    contact_normal: wp.array(dtype=wp.vec3),
    contact_force: wp.array(dtype=wp.vec3),  # pre-computed force vector from solver
    contact_thickness0: wp.array(dtype=wp.float32),
    contact_thickness1: wp.array(dtype=wp.float32),
    # model data
    shape_body: wp.array(dtype=int),
    body_q: wp.array(dtype=wp.transform),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index
    body_filter_map: wp.array2d(dtype=int),  # maps (sensor_idx, body_idx) -> filter_idx (-1 if no filter)
    world_body_idx: int,
    dt: float,
    max_contact_data_count: int,
    # outputs
    contact_forces: wp.array2d(dtype=wp.float32),  # force magnitudes (N, 1)
    contact_points: wp.array2d(dtype=wp.float32),  # contact points (N, 3)
    contact_normals: wp.array2d(dtype=wp.float32),  # contact normals (N, 3)
    contact_separations: wp.array2d(dtype=wp.float32),  # penetration depth (N, 1)
    contact_counts: wp.array2d(dtype=wp.uint32),  # per sensor-filter pair (uint32 to match PhysX API)
    contact_start_indices: wp.array2d(dtype=wp.uint32),  # start index per sensor-filter pair
) -> None:
    """Gather detailed contact data per sensor-filter pair using pre-computed forces.

    Gathers geometric information and signed force magnitudes per
    sensor-filter pair.
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
        dt: Multiplicative scale applied to each contact-force magnitude.
        max_contact_data_count: Maximum contacts to store in output arrays.
        contact_forces: Output signed force magnitudes.
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


@wp.kernel(enable_backward=False)
def gather_raw_contact_counts(
    source_counts: wp.array(dtype=wp.uint32),
    source_starts: wp.array(dtype=wp.uint32),
    indices: wp.array(dtype=wp.int32),
    record_capacity: int,
    selected_counts: wp.array(dtype=wp.int32),
) -> None:
    """Gather contact counts for selected sensors within the source capacity.

    Args:
        source_counts: Number of source records for each sensor.
        source_starts: Starting source-record offset for each sensor.
        indices: Source sensor index for each selected row.
        record_capacity: Number of valid records in the source buffers.
        selected_counts: Output count for each selected row.

    """
    row = wp.tid()
    source_sensor = indices[row]
    source_start = int(source_starts[source_sensor])
    available = int(source_counts[source_sensor])
    if source_start >= record_capacity:
        available = 0
    elif source_start + available > record_capacity:
        available = record_capacity - source_start
    selected_counts[row] = available


@wp.kernel(enable_backward=False)
def clamp_raw_contact_counts_to_capacity(
    selected_starts: wp.array(dtype=wp.int32),
    record_capacity: int,
    selected_counts: wp.array(dtype=wp.int32),
) -> None:
    """Clamp selected contact counts to the destination capacity.

    Args:
        selected_starts: Starting destination-record offset for each selected row.
        record_capacity: Number of records available in the destination buffers.
        selected_counts: In-place contact counts for the selected rows.

    """
    row = wp.tid()
    start = selected_starts[row]
    available = selected_counts[row]
    if start >= record_capacity:
        available = 0
    elif start + available > record_capacity:
        available = record_capacity - start
    selected_counts[row] = available


@wp.kernel(enable_backward=False)
def pack_raw_contact_data(
    source_forces: wp.array2d(dtype=wp.float32),
    source_points: wp.array2d(dtype=wp.float32),
    source_normals: wp.array2d(dtype=wp.float32),
    source_separations: wp.array2d(dtype=wp.float32),
    source_counts: wp.array(dtype=wp.uint32),
    source_starts: wp.array(dtype=wp.uint32),
    source_actor_ids: wp.array(dtype=wp.uint64),
    indices: wp.array(dtype=wp.int32),
    selected_starts: wp.array(dtype=wp.int32),
    max_contact_data_count: int,
    destination_forces: wp.array2d(dtype=wp.float32),
    destination_points: wp.array2d(dtype=wp.float32),
    destination_normals: wp.array2d(dtype=wp.float32),
    destination_separations: wp.array2d(dtype=wp.float32),
    destination_actor_ids: wp.array(dtype=wp.uint64),
) -> None:
    """Pack selected raw contact records into compact destination buffers.

    Args:
        source_forces: Source contact-force magnitudes.
        source_points: Source world-space contact points.
        source_normals: Source world-space contact normals.
        source_separations: Source contact separations.
        source_counts: Number of source records for each sensor.
        source_starts: Starting source-record offset for each sensor.
        source_actor_ids: Source identifier of the other actor in each contact.
        indices: Source sensor index for each selected row.
        selected_starts: Starting destination-record offset for each selected row.
        max_contact_data_count: Maximum number of valid records in either buffer set.
        destination_forces: Destination contact-force magnitudes.
        destination_points: Destination world-space contact points.
        destination_normals: Destination world-space contact normals.
        destination_separations: Destination contact separations.
        destination_actor_ids: Destination identifier of the other actor in each contact.

    """
    selected_row = wp.tid()
    source_sensor = indices[selected_row]
    source_start = int(source_starts[source_sensor])
    destination_start = selected_starts[selected_row]
    count = int(source_counts[source_sensor])

    for offset in range(count):
        source_row = source_start + offset
        destination_row = destination_start + offset
        if source_row < max_contact_data_count and destination_row < max_contact_data_count:
            destination_forces[destination_row, 0] = source_forces[source_row, 0]
            destination_points[destination_row, 0] = source_points[source_row, 0]
            destination_points[destination_row, 1] = source_points[source_row, 1]
            destination_points[destination_row, 2] = source_points[source_row, 2]
            destination_normals[destination_row, 0] = source_normals[source_row, 0]
            destination_normals[destination_row, 1] = source_normals[source_row, 1]
            destination_normals[destination_row, 2] = source_normals[source_row, 2]
            destination_separations[destination_row, 0] = source_separations[source_row, 0]
            destination_actor_ids[destination_row] = source_actor_ids[source_row]


@wp.kernel
def count_raw_contacts_per_sensor_kernel(
    # inputs - from Newton's contacts
    contact_count_total: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    # model data
    shape_body: wp.array(dtype=int),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index (-1 if not a sensor)
    world_body_idx: int,
    # outputs
    contact_counts: wp.array(dtype=wp.int32),  # per sensor scan workspace
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
        contact_counts: Output contact count per sensor (1D int32).

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
        wp.atomic_add(contact_counts, sensor_a, wp.int32(1))

    if sensor_b >= 0:
        wp.atomic_add(contact_counts, sensor_b, wp.int32(1))


@wp.kernel
def raw_contact_data_kernel(
    # inputs - from Newton's contacts (pre-computed by solver)
    contact_count: wp.array(dtype=int),
    contact_shape0: wp.array(dtype=int),
    contact_shape1: wp.array(dtype=int),
    contact_point0: wp.array(dtype=wp.vec3),  # body-local contact point on first shape
    contact_point1: wp.array(dtype=wp.vec3),  # body-local contact point on second shape
    contact_normal: wp.array(dtype=wp.vec3),
    contact_force: wp.array(dtype=wp.vec3),  # pre-computed force vector from solver
    contact_thickness0: wp.array(dtype=wp.float32),
    contact_thickness1: wp.array(dtype=wp.float32),
    # model data
    shape_body: wp.array(dtype=int),
    body_q: wp.array(dtype=wp.transform),
    # sensor mapping
    body_sensor_map: wp.array(dtype=int),  # maps body index -> sensor index
    world_body_idx: int,
    dt: float,
    max_contact_data_count: int,
    # outputs
    contact_forces: wp.array2d(dtype=wp.float32),  # force magnitudes (N, 1)
    contact_points: wp.array2d(dtype=wp.float32),  # contact points (N, 3)
    contact_normals: wp.array2d(dtype=wp.float32),  # contact normals (N, 3)
    contact_separations: wp.array2d(dtype=wp.float32),  # penetration depth (N, 1)
    contact_counts: wp.array(dtype=wp.uint32),  # per sensor (1D)
    contact_start_indices: wp.array(dtype=wp.int32),  # start index per sensor (1D)
    other_actor_ids: wp.array(dtype=wp.uint64),  # body index of the other body in the pair
) -> None:
    """Gather raw contact data per sensor without filter matching.

    All contacts touching a sensor are stored without filter matching, and each
    record identifies the other body.

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
        dt: Multiplicative scale applied to each contact-force magnitude.
        max_contact_data_count: Maximum contacts to store in output arrays.
        contact_forces: Output signed force magnitudes.
        contact_points: Output contact points world (N, 3).
        contact_normals: Output contact normals (N, 3).
        contact_separations: Output penetration depth (N, 1).
        contact_counts: Contact count per sensor (1D uint32).
        contact_start_indices: Start index per sensor (1D int32 workspace).
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

    # The raw entity-view convention orients normal and separation values from
    # each sensor toward the other body.
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
            contact_normals[write_idx, 0] = -n[0]
            contact_normals[write_idx, 1] = -n[1]
            contact_normals[write_idx, 2] = -n[2]
            contact_separations[write_idx, 0] = -d
            other_actor_ids[write_idx] = wp.uint64(body_b)

    # Store data for sensor B; other body is body_a
    if sensor_b >= 0:
        data_idx = wp.atomic_add(contact_counts, sensor_b, wp.uint32(1))
        start_idx = int(contact_start_indices[sensor_b])
        write_idx = start_idx + int(data_idx)

        if write_idx < max_contact_data_count:
            contact_forces[write_idx, 0] = force_mag
            contact_points[write_idx, 0] = contact_point[0]
            contact_points[write_idx, 1] = contact_point[1]
            contact_points[write_idx, 2] = contact_point[2]
            contact_normals[write_idx, 0] = n[0]
            contact_normals[write_idx, 1] = n[1]
            contact_normals[write_idx, 2] = n[2]
            contact_separations[write_idx, 0] = d
            other_actor_ids[write_idx] = wp.uint64(body_a)
