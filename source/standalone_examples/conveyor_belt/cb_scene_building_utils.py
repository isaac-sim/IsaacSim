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

"""Create USD mesh and rigid body helpers for the conveyor belt scene."""

import math

import warp as wp
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# not needed for the purpose of this sample
wp.config.enable_backward = False


CONVEYOR_BELT_TURN_SEGMENT_COUNT = 16
CONVEYOR_BELT_BORDER_SEGMENT_COUNT = 16


def _create_quarter_hollow_cylinder_mesh_data(
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    segment_count: int,
    close_at_front_and_back: bool = True,
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Generate a hollow cylinder mesh that is a quarter of a circle (90 degrees).

    The cylinder has an inner and outer radius (annular cross-section). It extends
    along the Z axis from -half_height to +half_height. The circular arc runs from
    0 to 90 degrees in the XY plane (first quadrant: x >= 0, y >= 0).

    Args:
        inner_radius: Radius of the inner circular edge (must be < outer_radius).
        outer_radius: Radius of the outer circular edge.
        half_height: Half-extent along the Z axis; cylinder runs from -half_height to
            +half_height (typically positive).
        segment_count: Number of segments along the curved arc (>= 1). Higher values
            give a smoother quarter-circle.
        close_at_front_and_back: Whether the faces at angle 0 and angle 90 respectively
            should get added or not

    Returns:
        A tuple (points, face_vertex_indices):
        - points: List of (x, y, z) vertex positions, suitable for
          UsdGeom.Mesh.GetPointsAttr().Set(Vt.Vec3fArray(points)).
        - face_vertex_indices: Flat list of vertex indices, three per triangle,
          suitable for UsdGeom.Mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices).
          Use face vertex counts of [3] * (len(face_vertex_indices) // 3).
    """
    if segment_count < 1:
        raise ValueError("segment_count must be >= 1")
    if inner_radius >= outer_radius:
        raise ValueError("inner_radius must be strictly less than outer_radius")

    n = segment_count + 1  # number of vertices per ring
    half_pi = math.pi / 2.0

    # Build vertices: for each angle index i in [0, segment_count], four vertices
    # (bottom inner, bottom outer, top inner, top outer). Order matches UsdGeom usage.
    points: list[tuple[float, float, float]] = []
    for i in range(n):
        t = (i / segment_count) * half_pi if segment_count > 0 else 0.0
        cos_t = math.cos(t)
        sin_t = math.sin(t)
        # Bottom ring (z = -half_height)
        points.append((inner_radius * cos_t, inner_radius * sin_t, -half_height))
        points.append((outer_radius * cos_t, outer_radius * sin_t, -half_height))
        # Top ring (z = +half_height)
        points.append((inner_radius * cos_t, inner_radius * sin_t, half_height))
        points.append((outer_radius * cos_t, outer_radius * sin_t, half_height))

    # Index helpers: for angle index i, vertex indices for the four rings
    def bi(i: int) -> int:
        return i * 4 + 0

    def bo(i: int) -> int:
        return i * 4 + 1

    def ti(i: int) -> int:
        return i * 4 + 2

    def to_(i: int) -> int:
        return i * 4 + 3

    face_vertex_indices: list[int] = []

    for i in range(segment_count):
        # Bottom cap (quarter annulus, normal -Z)
        face_vertex_indices.extend([bi(i), bi(i + 1), bo(i)])
        face_vertex_indices.extend([bi(i + 1), bo(i + 1), bo(i)])

        # Top cap (normal +Z)
        face_vertex_indices.extend([ti(i), to_(i), to_(i + 1)])
        face_vertex_indices.extend([ti(i), to_(i + 1), ti(i + 1)])

        # Inner curved wall (normal toward center)
        face_vertex_indices.extend([bi(i), ti(i), ti(i + 1)])
        face_vertex_indices.extend([bi(i), ti(i + 1), bi(i + 1)])

        # Outer curved wall (normal outward)
        face_vertex_indices.extend([bo(i), bo(i + 1), to_(i + 1)])
        face_vertex_indices.extend([bo(i), to_(i + 1), to_(i)])

    if close_at_front_and_back:
        # Side face at angle 0 (vertical edge, normal -Y)
        face_vertex_indices.extend([bi(0), bo(0), to_(0)])
        face_vertex_indices.extend([bi(0), to_(0), ti(0)])

        # Side face at angle 90° (vertical edge, normal +X)
        face_vertex_indices.extend([bo(segment_count), bi(segment_count), ti(segment_count)])
        face_vertex_indices.extend([bo(segment_count), ti(segment_count), to_(segment_count)])

    return (points, face_vertex_indices)


def _create_quarter_hollow_cylinder_mesh(
    stage: Usd.Stage,
    path: str,
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    segment_count: int,
) -> UsdGeom.Mesh:
    """Create a UsdGeom.Mesh prim with the quarter hollow cylinder geometry.

    Args:
        stage: Usd.Stage to create the mesh on.
        path: path for the new mesh prim.
        inner_radius: Radius of the inner circular edge.
        outer_radius: Radius of the outer circular edge.
        half_height: Half-extent along the Z axis.
        segment_count: Number of segments along the curved arc.

    Returns:
        UsdGeom.Mesh prim.
    """
    points, face_vertex_indices = _create_quarter_hollow_cylinder_mesh_data(
        inner_radius,
        outer_radius,
        half_height,
        segment_count,
    )

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(points))
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetFaceVertexCountsAttr().Set([3] * (len(face_vertex_indices) // 3))

    return mesh


def _create_conveyor_belt_front_or_back_mesh_data(
    half_width: float,
    half_height: float,
    belt_radius: float,
    segment_count: int,
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    index_offset: int = 0,
    border_vertex_index_list: tuple[int, int, int, int] | None = None,
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Generate triangle mesh data for the front or back rounded edge of a conveyor belt component.

    Produces a segmented quad strip that curves over the rounded belt radius at the top
    edge of the belt face.

    Args:
        half_width: Half-extent of the belt along the X axis.
        half_height: Half-extent of the belt along the Z axis.
        belt_radius: Radius of the rounded border at the top of the face.
        segment_count: Number of segments used to approximate the rounded curve.
        offset: Translation (x, y, z) applied to all generated vertices.
        rotation: Rotation quaternion (w, x, y, z) applied to all generated vertices.
        index_offset: Starting vertex index offset for the generated face indices.
        border_vertex_index_list: If provided, reuses existing vertex indices for the
            lower-left, lower-right, upper-left, and upper-right corners instead of
            creating new vertices.

    Returns:
        A tuple ``(vertices, face_vertex_indices)`` of new vertices and triangle indices.
    """
    #   _____   _
    #  /         | belt_radius
    # |         _|
    # |
    #  \_____
    #       |   _
    #       |    |
    #       |    |                        z
    #       |    | half_height            ^
    #       |    |                        |
    #       |   _|                   y <---

    offset_vec = wp.vec3(offset[0], offset[1], offset[2])
    rotation_quat = wp.quat(rotation[1], rotation[2], rotation[3], rotation[0])

    delta_angle = math.pi / segment_count

    upper_left = wp.vec3(-half_width, 0.0, half_height - (2.0 * belt_radius))
    upper_right = wp.vec3(half_width, 0.0, half_height - (2.0 * belt_radius))

    upper_left = wp.quat_rotate(rotation_quat, upper_left) + offset_vec
    upper_right = wp.quat_rotate(rotation_quat, upper_right) + offset_vec

    vertices = []

    face_vertex_indices = []

    if border_vertex_index_list is None:
        lower_left = wp.vec3(-half_width, 0.0, -half_height)
        lower_right = wp.vec3(half_width, 0.0, -half_height)

        lower_left = wp.quat_rotate(rotation_quat, lower_left) + offset_vec
        lower_right = wp.quat_rotate(rotation_quat, lower_right) + offset_vec

        vertices.append((lower_left[0], lower_left[1], lower_left[2]))
        vertices.append((lower_right[0], lower_right[1], lower_right[2]))

        lower_left_index = index_offset + 0
        lower_right_index = index_offset + 1
        upper_left_index = index_offset + 2
        upper_right_index = index_offset + 3
    else:
        lower_left_index = border_vertex_index_list[0]
        lower_right_index = border_vertex_index_list[1]
        upper_left_index = index_offset + 0
        upper_right_index = index_offset + 1

    vertices.append((upper_left[0], upper_left[1], upper_left[2]))
    vertices.append((upper_right[0], upper_right[1], upper_right[2]))

    face_vertex_indices.append(lower_left_index)
    face_vertex_indices.append(lower_right_index)
    face_vertex_indices.append(upper_right_index)
    # ---
    face_vertex_indices.append(lower_left_index)
    face_vertex_indices.append(upper_right_index)
    face_vertex_indices.append(upper_left_index)

    i = 1

    while i < segment_count:

        lower_left_index = upper_left_index
        lower_right_index = upper_right_index
        upper_left_index = upper_right_index + 1
        upper_right_index = upper_right_index + 2

        y = -math.sin(i * delta_angle) * belt_radius
        z = half_height - belt_radius - (math.cos(i * delta_angle) * belt_radius)

        upper_left = wp.vec3(-half_width, y, z)
        upper_right = wp.vec3(half_width, y, z)

        upper_left = wp.quat_rotate(rotation_quat, upper_left) + offset_vec
        upper_right = wp.quat_rotate(rotation_quat, upper_right) + offset_vec

        vertices.append((upper_left[0], upper_left[1], upper_left[2]))
        vertices.append((upper_right[0], upper_right[1], upper_right[2]))

        face_vertex_indices.append(lower_left_index)
        face_vertex_indices.append(lower_right_index)
        face_vertex_indices.append(upper_right_index)
        # ---
        face_vertex_indices.append(lower_left_index)
        face_vertex_indices.append(upper_right_index)
        face_vertex_indices.append(upper_left_index)

        i += 1

    lower_left_index = upper_left_index
    lower_right_index = upper_right_index

    if border_vertex_index_list is None:
        upper_left_index = upper_right_index + 1
        upper_right_index = upper_right_index + 2

        upper_left = wp.vec3(-half_width, 0.0, half_height)
        upper_right = wp.vec3(half_width, 0.0, half_height)

        upper_left = wp.quat_rotate(rotation_quat, upper_left) + offset_vec
        upper_right = wp.quat_rotate(rotation_quat, upper_right) + offset_vec

        vertices.append((upper_left[0], upper_left[1], upper_left[2]))
        vertices.append((upper_right[0], upper_right[1], upper_right[2]))
    else:
        upper_left_index = border_vertex_index_list[2]
        upper_right_index = border_vertex_index_list[3]

    face_vertex_indices.append(lower_left_index)
    face_vertex_indices.append(lower_right_index)
    face_vertex_indices.append(upper_right_index)
    # ---
    face_vertex_indices.append(lower_left_index)
    face_vertex_indices.append(upper_right_index)
    face_vertex_indices.append(upper_left_index)

    return (vertices, face_vertex_indices)


def _create_conveyor_belt_box_mesh_data(
    half_extent: tuple[float, float, float],
    border_radius: float,
    box_segment_count: int,
    border_segment_count: int,
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Generate triangle mesh data for the full surface of a straight conveyor belt box.

    Combines the top and side quads of the box body with rounded front and back edges.

    Args:
        half_extent: Half-extents (x, y, z) of the conveyor belt box.
        border_radius: Radius of the rounded border at the front and back edges.
        box_segment_count: Number of segments to divide the belt length (Y axis) into.
        border_segment_count: Number of segments used to approximate each rounded border.

    Returns:
        A tuple ``(vertices, face_vertex_indices)`` of the full mesh data.
    """
    offset_front = (0.0, -half_extent[1], 0.0)
    offset_back = (0.0, half_extent[1], 0.0)

    rotation_quat_back = (0.0, 0.0, 0.0, 1.0)

    vertices = []

    face_vertex_indices = []

    y = -half_extent[1]

    front_lower_left = (-half_extent[0], y, -half_extent[2])
    front_lower_left_index = 0
    vertices.append(front_lower_left)

    front_lower_right = (half_extent[0], y, -half_extent[2])
    front_lower_right_index = 1
    vertices.append(front_lower_right)

    front_upper_left = (-half_extent[0], y, half_extent[2])
    front_upper_left_index = 2
    vertices.append(front_upper_left)

    front_upper_right = (half_extent[0], y, half_extent[2])
    front_upper_right_index = 3
    vertices.append(front_upper_right)

    i = 0

    lower_left_index = front_lower_left_index
    lower_right_index = front_lower_right_index
    upper_left_index = front_upper_left_index
    upper_right_index = front_upper_right_index

    delta_y = (2.0 * half_extent[1]) / box_segment_count

    while i < box_segment_count:

        y += delta_y

        back_lower_left = (-half_extent[0], y, -half_extent[2])
        back_lower_left_index = lower_left_index + 4
        vertices.append(back_lower_left)

        back_lower_right = (half_extent[0], y, -half_extent[2])
        back_lower_right_index = lower_right_index + 4
        vertices.append(back_lower_right)

        back_upper_left = (-half_extent[0], y, half_extent[2])
        back_upper_left_index = upper_left_index + 4
        vertices.append(back_upper_left)

        back_upper_right = (half_extent[0], y, half_extent[2])
        back_upper_right_index = upper_right_index + 4
        vertices.append(back_upper_right)

        #
        # top
        #
        face_vertex_indices.append(upper_left_index)
        face_vertex_indices.append(upper_right_index)
        face_vertex_indices.append(back_upper_right_index)
        # ---
        face_vertex_indices.append(upper_left_index)
        face_vertex_indices.append(back_upper_right_index)
        face_vertex_indices.append(back_upper_left_index)

        #
        # left side
        #
        face_vertex_indices.append(back_lower_left_index)
        face_vertex_indices.append(lower_left_index)
        face_vertex_indices.append(upper_left_index)
        # ---
        face_vertex_indices.append(back_lower_left_index)
        face_vertex_indices.append(upper_left_index)
        face_vertex_indices.append(back_upper_left_index)

        #
        # right side
        #
        face_vertex_indices.append(lower_right_index)
        face_vertex_indices.append(back_lower_right_index)
        face_vertex_indices.append(back_upper_right_index)
        # ---
        face_vertex_indices.append(lower_right_index)
        face_vertex_indices.append(back_upper_right_index)
        face_vertex_indices.append(upper_right_index)

        lower_left_index = back_lower_left_index
        lower_right_index = back_lower_right_index
        upper_left_index = back_upper_left_index
        upper_right_index = back_upper_right_index

        i += 1

    #
    # front
    #
    tmp_vertices, tmp_face_vertex_indices = _create_conveyor_belt_front_or_back_mesh_data(
        half_extent[0],
        half_extent[2],
        border_radius,
        border_segment_count,
        offset_front,
        (1.0, 0.0, 0.0, 0.0),
        len(vertices),
        (front_lower_left_index, front_lower_right_index, front_upper_left_index, front_upper_right_index),
    )

    vertices.extend(tmp_vertices)
    face_vertex_indices.extend(tmp_face_vertex_indices)

    #
    # back
    #
    # note: the mesh gets rotated by 180 degrees, thus the provided indices for left and right vertices
    #       need to flip
    #
    tmp_vertices, tmp_face_vertex_indices = _create_conveyor_belt_front_or_back_mesh_data(
        half_extent[0],
        half_extent[2],
        border_radius,
        border_segment_count,
        offset_back,
        rotation_quat_back,
        len(vertices),
        (lower_right_index, lower_left_index, upper_right_index, upper_left_index),
    )

    vertices.extend(tmp_vertices)
    face_vertex_indices.extend(tmp_face_vertex_indices)

    return (vertices, face_vertex_indices)


def _create_conveyor_belt_turn_90_degree_mesh_data(
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    border_radius: float,
    turn_segment_count: int,
    border_segment_count: int,
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Generate triangle mesh data for a 90-degree conveyor belt turn section.

    Combines a quarter hollow-cylinder surface with rounded front and back connection edges.

    Args:
        inner_radius: Inner radius of the 90-degree turn.
        outer_radius: Outer radius of the 90-degree turn.
        half_height: Half-height of the turn section along the Z axis.
        border_radius: Radius of the rounded border at the connection edges.
        turn_segment_count: Number of arc segments for the curved surface.
        border_segment_count: Number of segments for each rounded border edge.

    Returns:
        A tuple ``(vertices, face_vertex_indices)`` of the full turn mesh data.
    """
    vertices, face_vertex_indices = _create_quarter_hollow_cylinder_mesh_data(
        inner_radius,
        outer_radius,
        half_height,
        turn_segment_count,
        False,
    )

    mid_radius = (inner_radius + outer_radius) * 0.5
    half_width = (outer_radius - inner_radius) * 0.5

    #
    # Adding the "front" where the conveyor belt will connect to other belts
    #
    offset_front = (mid_radius, 0.0, 0.0)
    front_lower_left_index = 0
    front_lower_right_index = 1
    front_upper_left_index = 2
    front_upper_right_index = 3

    tmp_vertices, tmp_face_vertex_indices = _create_conveyor_belt_front_or_back_mesh_data(
        half_width,
        half_height,
        border_radius,
        border_segment_count,
        offset_front,
        (1.0, 0.0, 0.0, 0.0),
        len(vertices),
        (front_lower_left_index, front_lower_right_index, front_upper_left_index, front_upper_right_index),
    )

    vertices.extend(tmp_vertices)
    face_vertex_indices.extend(tmp_face_vertex_indices)

    #
    # Adding the "back" where the conveyor belt will connect to other belts
    #
    offset_back = (0.0, mid_radius, 0.0)
    rotation_back = (math.cos(-math.pi * 0.25), 0.0, 0.0, math.sin(-math.pi * 0.25))
    back_start_index = 4 * border_segment_count
    back_lower_left_index = back_start_index + 1
    back_lower_right_index = back_start_index + 0
    back_upper_left_index = back_start_index + 3
    back_upper_right_index = back_start_index + 2

    tmp_vertices, tmp_face_vertex_indices = _create_conveyor_belt_front_or_back_mesh_data(
        half_width,
        half_height,
        border_radius,
        border_segment_count,
        offset_back,
        rotation_back,
        len(vertices),
        (back_lower_left_index, back_lower_right_index, back_upper_left_index, back_upper_right_index),
    )

    vertices.extend(tmp_vertices)
    face_vertex_indices.extend(tmp_face_vertex_indices)

    return (vertices, face_vertex_indices)


def _compute_box_inertia(
    mass: float,
    half_extent: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Compute the diagonal inertia tensor of a solid box.

    Args:
        mass: Mass of the box.
        half_extent: Half-extents of the box as a tuple (x, y, z).

    Returns:
        Tuple (Ix, Iy, Iz) of principal moments of inertia.
    """
    base_inertia_value = (1.0 / 12.0) * mass
    extent_x = 2.0 * half_extent[0]
    extent_y = 2.0 * half_extent[1]
    extent_z = 2.0 * half_extent[2]

    inertia_x = base_inertia_value * ((extent_y * extent_y) + (extent_z * extent_z))
    inertia_y = base_inertia_value * ((extent_x * extent_x) + (extent_z * extent_z))
    inertia_z = base_inertia_value * ((extent_x * extent_x) + (extent_y * extent_y))

    return (inertia_x, inertia_y, inertia_z)


def _set_physics_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    """Bind a USD physics material to a prim using weaker-than-descendants strength.

    Args:
        prim: The USD prim to bind the material to.
        material: The UsdShade material to bind.
    """
    bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
    bindingAPI.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


BODY_TYPE_DEFAULT = 0
BODY_TYPE_KINEMATIC = 1
BODY_TYPE_ARTICULATION_ROOT = 2
BODY_TYPE_ARTICULATION_LINK = 3


def _create_rigid_body(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    mass: float,
    inertia: tuple[float, float, float],
    type: int = BODY_TYPE_DEFAULT,
) -> Usd.Prim:
    """Create a USD Xform prim with RigidBodyAPI and MassAPI applied.

    Args:
        stage: USD stage on which to create the prim.
        path: USD prim path for the new rigid body.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        mass: Mass of the rigid body.
        inertia: Diagonal inertia tensor as a tuple (Ix, Iy, Iz).
        type: Body type constant (``BODY_TYPE_DEFAULT``, ``BODY_TYPE_KINEMATIC``, etc.).

    Returns:
        The created USD prim.
    """
    body_xform = UsdGeom.Xform.Define(stage, path)

    body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    body_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    body_prim = body_xform.GetPrim()

    # Make it a rigid body (dynamic object that responds to physics)
    body_api = UsdPhysics.RigidBodyAPI.Apply(body_prim)

    if type == BODY_TYPE_KINEMATIC:
        body_api.CreateKinematicEnabledAttr(True)
    elif type == BODY_TYPE_ARTICULATION_ROOT:
        UsdPhysics.ArticulationRootAPI.Apply(body_prim)

    mass_api = UsdPhysics.MassAPI.Apply(body_prim)

    mass_api.CreateMassAttr(mass)
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia[0], inertia[1], inertia[2]))

    return body_prim


def _set_contact_and_rest_offset(
    prim: Usd.Prim,
    contact_offset: float,
    rest_offset: float,
) -> None:
    """Apply PhysxCollisionAPI to ``prim`` and set its contact and rest offset values.

    Args:
        prim: The USD prim to configure.
        contact_offset: Distance at which contacts are detected (in scene units).
        rest_offset: Distance at which objects come to rest against each other (in scene units).
    """
    physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx_collision_api.CreateContactOffsetAttr(contact_offset)
    physx_collision_api.CreateRestOffsetAttr(rest_offset)


def _create_collision_box(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    half_extent: tuple[float, float, float],
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> Usd.Prim:
    """Create a UsdGeom.Cube collision prim with CollisionAPI and optional physics material.

    Args:
        stage: USD stage on which to create the prim.
        path: USD prim path for the collision box.
        position: Position as a tuple (x, y, z).
        orientation: Orientation as a quaternion tuple (w, x, y, z).
        half_extent: Half-extents of the box as a tuple (x, y, z).
        material: Optional physics material to bind to the prim. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        The created collision geometry prim.
    """
    cube_geom = UsdGeom.Cube.Define(stage, path)

    cube_geom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    cube_geom.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    cube_geom.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(half_extent[0], half_extent[1], half_extent[2])
    )

    cube_geom_prim = cube_geom.GetPrim()

    UsdPhysics.CollisionAPI.Apply(cube_geom_prim)

    _set_contact_and_rest_offset(cube_geom_prim, contact_offset, rest_offset)

    if material is not None:
        _set_physics_material(cube_geom_prim, material)

    return cube_geom_prim


def _create_collision_turn_90_degree(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> Usd.Prim:
    """Create a quarter hollow-cylinder mesh collision prim for a 90-degree conveyor belt turn.

    Args:
        stage: USD stage on which to create the prim.
        path: USD prim path for the collision mesh.
        position: Position as a tuple (x, y, z).
        orientation: Orientation as a quaternion tuple (w, x, y, z).
        inner_radius: Inner radius of the turn geometry.
        outer_radius: Outer radius of the turn geometry.
        half_height: Half-height of the turn geometry along the Z axis.
        material: Optional physics material to bind to the prim. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        The created collision geometry prim.
    """
    mesh_geom = _create_quarter_hollow_cylinder_mesh(
        stage,
        path,
        inner_radius,
        outer_radius,
        half_height,
        CONVEYOR_BELT_TURN_SEGMENT_COUNT,
    )

    mesh_geom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    mesh_geom.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    mesh_geom.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    mesh_geom_prim = mesh_geom.GetPrim()

    UsdPhysics.CollisionAPI.Apply(mesh_geom_prim)

    _set_contact_and_rest_offset(mesh_geom_prim, contact_offset, rest_offset)

    if material is not None:
        _set_physics_material(mesh_geom_prim, material)

    UsdPhysics.MeshCollisionAPI.Apply(mesh_geom_prim)

    return mesh_geom_prim


def create_rigid_body_box(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    half_extent: tuple[float, float, float],
    mass: float,
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
    type: int = BODY_TYPE_DEFAULT,
) -> tuple[Usd.Prim, Usd.Prim]:
    """Create a rigid body prim with a box collision geometry child.

    Args:
        stage: USD stage on which to create the prims.
        path: USD prim path for the rigid body Xform.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        half_extent: Half-extents of the collision box as a tuple (x, y, z).
        mass: Mass of the body.
        material: Optional physics material to bind to the collision geometry. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.
        type: Body type constant controlling kinematic/articulation behaviour.

    Returns:
        A tuple ``(body_prim, collision_geom_prim)``.
    """
    #
    # Creating the rigid body prim
    #

    inertia = _compute_box_inertia(mass, half_extent)

    body_prim = _create_rigid_body(stage, path, position, orientation, mass, inertia, type)

    #
    # Creating the collision box geometry
    #

    coll_geom_path = path + "/collision"

    cube_geom_prim = _create_collision_box(
        stage,
        coll_geom_path,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        half_extent,
        material,
        contact_offset,
        rest_offset,
    )

    gprim = UsdGeom.Gprim(cube_geom_prim)

    if (type == BODY_TYPE_ARTICULATION_ROOT) or (type == BODY_TYPE_ARTICULATION_LINK):
        gprim.CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.7, 1.0)])
    else:
        gprim.CreateDisplayColorAttr([Gf.Vec3f(0.7, 1.0, 0.7)])

    return (body_prim, cube_geom_prim)


def create_conveyor_belt_box(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    half_extent: tuple[float, float, float],
    border_margin_y: float,
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> tuple[Usd.Prim, Usd.Prim]:
    """Create a static conveyor belt box with a rounded-edge mesh collision surface.

    Args:
        stage: USD stage on which to create the prims.
        path: USD prim path for the conveyor belt Xform.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        half_extent: Half-extents of the belt box as a tuple (x, y, z).
        border_margin_y: Radius of the rounded border at the front and back edges of the belt.
        material: Optional physics material to bind to the collision mesh. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        A tuple ``(body_prim, collision_mesh_prim)``.
    """
    #
    # Creating the xform prim
    #

    body_xform = UsdGeom.Xform.Define(stage, path)

    body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    body_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    body_prim = body_xform.GetPrim()

    #
    # Creating the collision geometry
    #

    coll_geom_path = path + "/collision"

    max_segment_length_y = 0.5
    box_segment_count = math.ceil((2.0 * half_extent[1]) / max_segment_length_y)

    vertices, face_vertex_indices = _create_conveyor_belt_box_mesh_data(
        (half_extent[0], half_extent[1], half_extent[2]),
        border_margin_y,
        box_segment_count,
        CONVEYOR_BELT_BORDER_SEGMENT_COUNT,
    )

    mesh = UsdGeom.Mesh.Define(stage, coll_geom_path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(vertices))
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetFaceVertexCountsAttr().Set([3] * (len(face_vertex_indices) // 3))

    mesh_geom_prim = mesh.GetPrim()

    UsdPhysics.CollisionAPI.Apply(mesh_geom_prim)

    _set_contact_and_rest_offset(mesh_geom_prim, contact_offset, rest_offset)

    if material is not None:
        _set_physics_material(mesh_geom_prim, material)

    UsdPhysics.MeshCollisionAPI.Apply(mesh_geom_prim)

    return (body_prim, mesh_geom_prim)


def create_conveyor_belt_box_guard(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    half_extent: tuple[float, float, float],
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> tuple[Usd.Prim, Usd.Prim]:
    """Create a static guard rail for a straight conveyor belt using a box collision geometry.

    Args:
        stage: USD stage on which to create the prims.
        path: USD prim path for the guard Xform.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        half_extent: Half-extents of the guard box as a tuple (x, y, z).
        material: Optional physics material to bind to the collision geometry. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        A tuple ``(body_prim, collision_geom_prim)``.
    """
    #
    # Creating the xform prim
    #

    body_xform = UsdGeom.Xform.Define(stage, path)

    body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    body_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    body_prim = body_xform.GetPrim()

    #
    # Creating the collision geometry
    #

    coll_geom_path = path + "/collision"

    cube_geom_prim = _create_collision_box(
        stage,
        coll_geom_path,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        half_extent,
        material,
        contact_offset,
        rest_offset,
    )

    return (body_prim, cube_geom_prim)


def create_conveyor_belt_turn(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    border_margin_y: float,
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> tuple[Usd.Prim, Usd.Prim]:
    """Create a 90-degree conveyor belt turn section with a rounded-edge mesh collision surface.

    Args:
        stage: USD stage on which to create the prims.
        path: USD prim path for the turn Xform.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        inner_radius: Inner radius of the 90-degree arc.
        outer_radius: Outer radius of the 90-degree arc.
        half_height: Half-height of the turn section along the Z axis.
        border_margin_y: Radius of the rounded border at the connection edges.
        material: Optional physics material to bind to the collision mesh. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        A tuple ``(body_prim, collision_mesh_prim)``.
    """
    #
    # Creating the xform prim
    #

    body_xform = UsdGeom.Xform.Define(stage, path)

    body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    body_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    body_prim = body_xform.GetPrim()

    #
    # Creating the collision geometry
    #

    coll_geom_path = path + "/collision"

    vertices, face_vertex_indices = _create_conveyor_belt_turn_90_degree_mesh_data(
        inner_radius,
        outer_radius,
        half_height,
        border_margin_y,
        CONVEYOR_BELT_TURN_SEGMENT_COUNT,
        CONVEYOR_BELT_BORDER_SEGMENT_COUNT,
    )

    mesh = UsdGeom.Mesh.Define(stage, coll_geom_path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(vertices))
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetFaceVertexCountsAttr().Set([3] * (len(face_vertex_indices) // 3))

    mesh_geom_prim = mesh.GetPrim()

    UsdPhysics.CollisionAPI.Apply(mesh_geom_prim)

    _set_contact_and_rest_offset(mesh_geom_prim, contact_offset, rest_offset)

    if material is not None:
        _set_physics_material(mesh_geom_prim, material)

    UsdPhysics.MeshCollisionAPI.Apply(mesh_geom_prim)

    return (body_prim, mesh_geom_prim)


def create_conveyor_belt_turn_guard(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    inner_radius: float,
    outer_radius: float,
    half_height: float,
    material: UsdShade.Material | None = None,
    contact_offset: float = 0.02,
    rest_offset: float = 0.0,
) -> tuple[Usd.Prim, Usd.Prim]:
    """Create a static guard rail for a 90-degree conveyor belt turn using a mesh collision geometry.

    Args:
        stage: USD stage on which to create the prims.
        path: USD prim path for the guard Xform.
        position: World-space position as a tuple (x, y, z).
        orientation: World-space orientation as a quaternion tuple (w, x, y, z).
        inner_radius: Inner radius of the 90-degree arc guard.
        outer_radius: Outer radius of the 90-degree arc guard.
        half_height: Half-height of the guard geometry along the Z axis.
        material: Optional physics material to bind to the collision mesh. If ``None``, a default material will be used.
        contact_offset: Contact detection distance in scene units.
        rest_offset: Rest distance in scene units.

    Returns:
        A tuple ``(body_prim, collision_mesh_prim)``.
    """
    #
    # Creating the xform prim
    #

    body_xform = UsdGeom.Xform.Define(stage, path)

    body_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(position[0], position[1], position[2])
    )
    body_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(orientation[0], orientation[1], orientation[2], orientation[3])
    )
    body_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    body_prim = body_xform.GetPrim()

    #
    # Creating the collision geometry
    #

    coll_geom_path = path + "/collision"

    mesh_geom_prim = _create_collision_turn_90_degree(
        stage,
        coll_geom_path,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        inner_radius,
        outer_radius,
        half_height,
        material,
        contact_offset,
        rest_offset,
    )

    return (body_prim, mesh_geom_prim)
