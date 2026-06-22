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

"""Build the USD scene for the conveyor belt example."""

import math

import warp as wp
from cb_actuators import VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY, VELOCITY_FIELD_TYPE_PIVOT, VelocityFieldActuator
from cb_body_manager import BodyManager
from cb_conveyor_belt_manager import ConveyorBeltManager
from cb_material_pair_manager import MaterialPairManager
from cb_scene_building_utils import (
    BODY_TYPE_ARTICULATION_LINK,
    BODY_TYPE_ARTICULATION_ROOT,
    BODY_TYPE_DEFAULT,
    create_conveyor_belt_box,
    create_conveyor_belt_box_guard,
    create_conveyor_belt_turn,
    create_conveyor_belt_turn_guard,
    create_rigid_body_box,
)
from cb_visualizers import VelocityFieldVisualizer
from isaacsim.core.api.materials import PhysicsMaterial
from pxr import Gf, PhysxSchema, Usd, UsdPhysics

# not needed for the purpose of this sample
wp.config.enable_backward = False


def create_scene(
    stage: Usd.Stage,
    velocity_field_actuator: VelocityFieldActuator,
    conveyor_belt_manager: ConveyorBeltManager,
    body_manager: BodyManager,
    material_pair_manager: MaterialPairManager,
    velocity_field_visualizer: VelocityFieldVisualizer | None = None,
) -> None:
    """Create the main conveyor belt circuit scene with straight sections, 90-degree turns, and transported bodies.

    Args:
        stage: USD stage on which to build the scene.
        velocity_field_actuator: Actuator that owns the velocity field definitions.
        conveyor_belt_manager: Manager that tracks conveyor belt prims and their velocity field mappings.
        body_manager: Manager that tracks the dynamic bodies to be transported.
        material_pair_manager: Manager that holds the friction table for body/belt material pairs.
        velocity_field_visualizer: Optional visualizer for rendering velocity field markers;
            pass ``None`` to disable visualization.
    """
    # The material for the conveyor belt geometry.
    # The friction is set to zero and the friction combine mode to "min" since
    # friction will be handled by custom logic.
    conveyor_belt_material = PhysicsMaterial(
        "/World/Physics_Materials/conveyor_belt_material",
        static_friction=0.0,
        dynamic_friction=0.0,
        restitution=0.0,
    )
    physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(conveyor_belt_material.prim)
    physx_material_api.CreateFrictionCombineModeAttr("min")

    conveyor_belt_material_index = material_pair_manager.add_conveyor_belt_material_index()

    # The material for the conveyor belt guard geometry.
    conveyor_belt_guard_material = PhysicsMaterial(
        "/World/Physics_Materials/conveyor_belt_guard_material",
        static_friction=0.2,
        dynamic_friction=0.2,
        restitution=0.0,
    )

    # Materials for the dynamic objects being transported by conveyor belts.
    moveable_object_material0 = PhysicsMaterial(
        "/World/Physics_Materials/moveable_object_material0",
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.0,
    )
    moveable_object_material0_index = material_pair_manager.add_transported_body_material_index()

    moveable_object_material1 = PhysicsMaterial(
        "/World/Physics_Materials/moveable_object_material1",
        static_friction=0.9,
        dynamic_friction=0.9,
        restitution=0.0,
    )
    moveable_object_material1_index = material_pair_manager.add_transported_body_material_index()

    # Friction coefficients for body vs. conveyor belt material pairs
    material_pair_manager.set_material_pair_friction(
        moveable_object_material0_index,
        conveyor_belt_material_index,
        0.5,
    )

    material_pair_manager.set_material_pair_friction(
        moveable_object_material1_index,
        conveyor_belt_material_index,
        0.9,
    )

    # Using a rather large rest offset to smooth transitions when objects move
    # from one conveyor belt section to the next
    contact_offset = 0.0025
    rest_offset = 0.002
    rest_offset_doubled = 2.0 * rest_offset

    #
    # Conveyor belt objects
    #

    conveyor_belt_base_vel_magn = 2.0

    conveyor_belt_half_width = 0.5
    conveyor_belt_half_thickness = 0.1
    conveyor_belt_border_margin_y = conveyor_belt_half_thickness * 0.5

    conveyor_belt_guard_width = 0.1
    conveyor_belt_guard_half_thickness = conveyor_belt_half_thickness + 0.1

    multi_conveyor_belt_half_width = 0.5 * conveyor_belt_half_width
    multi_conveyor_belt_half_gap = 0.01

    conveyor_belt_contact_processing_threshold = 0.997  # in the order of 4 degrees difference

    visualizer_marker_delta = 0.4

    # ---

    conv_belt_path = "/World/conv_belt0"

    conv_belt0_position = (0.0, 0.0, 0.5)
    conv_belt0_orientation = (1.0, 0.0, 0.0, 0.0)
    conv_belt0_half_extent = (conveyor_belt_half_width, 5.0, conveyor_belt_half_thickness)

    conv_belt0_prim, conv_belt0_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt0_position,
        conv_belt0_orientation,
        conv_belt0_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    target_velocity = wp.vec3(0.0, -conveyor_belt_base_vel_magn, 0.0)

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt0_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt0_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt0_position[0],
                conv_belt0_position[1] + conv_belt0_half_extent[1],
                conv_belt0_position[2] + conv_belt0_half_extent[2],
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt1"

    conv_belt1_outer_radius = 3.0
    conv_belt1_inner_radius = conv_belt1_outer_radius - (conveyor_belt_half_width * 2.0)
    conv_belt1_mid_radius = (conv_belt1_outer_radius + conv_belt1_inner_radius) * 0.5

    conv_belt1_position = (
        conv_belt0_position[0] - conv_belt1_mid_radius,
        conv_belt0_position[1] - conv_belt0_half_extent[1] - (2.0 * conveyor_belt_border_margin_y),
        conv_belt0_position[2],
    )
    conv_belt1_orientation = (math.cos(-math.pi / 4.0), 0.0, 0.0, math.sin(-math.pi / 4.0))

    conv_belt1_prim, conv_belt1_geom_prim = create_conveyor_belt_turn(
        stage,
        conv_belt_path,
        conv_belt1_position,
        conv_belt1_orientation,
        conv_belt1_inner_radius,
        conv_belt1_outer_radius,
        conveyor_belt_half_thickness,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    pivot_point = wp.vec3(conv_belt1_position[0], conv_belt1_position[1], conv_belt1_position[2])
    angular_velocity = wp.vec3(0.0, 0.0, (-conveyor_belt_base_vel_magn / conv_belt1_mid_radius))

    velocity_field_id = velocity_field_actuator.add_pivot_velocity_field(
        pivot_point,
        angular_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt1_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_PIVOT,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    conv_belt_guard_path = "/World/conv_belt1_guard"

    create_conveyor_belt_turn_guard(
        stage,
        conv_belt_guard_path,
        conv_belt1_position,
        conv_belt1_orientation,
        conv_belt1_outer_radius,
        conv_belt1_outer_radius + conveyor_belt_guard_width,
        conveyor_belt_guard_half_thickness,
        conveyor_belt_guard_material.material,
        contact_offset,
        rest_offset,
    )

    if velocity_field_visualizer is not None:

        start_angle = 0.0
        angle_delta = wp.pi * 0.5

        velocity_field_visualizer.add_pivot_velocity_field(
            pivot_point + wp.vec3(0.0, 0.0, conveyor_belt_half_thickness),
            angular_velocity,
            wp.vec3(1.0, 0.0, 0.0),
            wp.vec3(0.0, -1.0, 0.0),
            conv_belt1_mid_radius,
            start_angle,
            start_angle + angle_delta,
            int((conv_belt1_mid_radius * angle_delta) / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt2"

    conv_belt2_outer_radius = conv_belt1_outer_radius
    conv_belt2_inner_radius = conv_belt1_inner_radius
    conv_belt2_mid_radius = conv_belt1_mid_radius

    conv_belt2_position = (
        conv_belt1_position[0] - (2.0 * conveyor_belt_border_margin_y),
        conv_belt1_position[1],
        conv_belt1_position[2],
    )
    conv_belt2_orientation = (math.cos(-math.pi / 2.0), 0.0, 0.0, math.sin(-math.pi / 2.0))

    conv_belt2_prim, conv_belt2_geom_prim = create_conveyor_belt_turn(
        stage,
        conv_belt_path,
        conv_belt2_position,
        conv_belt2_orientation,
        conv_belt2_inner_radius,
        conv_belt2_outer_radius,
        conveyor_belt_half_thickness,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    pivot_point = wp.vec3(conv_belt2_position[0], conv_belt2_position[1], conv_belt2_position[2])
    angular_velocity = wp.vec3(0.0, 0.0, (-conveyor_belt_base_vel_magn / conv_belt2_mid_radius))

    velocity_field_id = velocity_field_actuator.add_pivot_velocity_field(
        pivot_point,
        angular_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt2_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_PIVOT,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    conv_belt_guard_path = "/World/conv_belt2_guard"

    create_conveyor_belt_turn_guard(
        stage,
        conv_belt_guard_path,
        conv_belt2_position,
        conv_belt2_orientation,
        conv_belt2_outer_radius,
        conv_belt2_outer_radius + conveyor_belt_guard_width,
        conveyor_belt_guard_half_thickness,
        conveyor_belt_guard_material.material,
        contact_offset,
        rest_offset,
    )

    if velocity_field_visualizer is not None:

        start_angle = wp.pi * 0.5
        angle_delta = wp.pi * 0.5

        velocity_field_visualizer.add_pivot_velocity_field(
            pivot_point + wp.vec3(0.0, 0.0, conveyor_belt_half_thickness),
            angular_velocity,
            wp.vec3(1.0, 0.0, 0.0),
            wp.vec3(0.0, -1.0, 0.0),
            conv_belt1_mid_radius,
            start_angle,
            start_angle + angle_delta,
            int((conv_belt1_mid_radius * angle_delta) / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt3"

    conv_belt3_half_extent = (conveyor_belt_half_width, 2.0, conveyor_belt_half_thickness)

    conv_belt3_half_extent_y_with_margin = conv_belt3_half_extent[1] + conveyor_belt_border_margin_y

    extra_shift_z = 0.003

    conv_belt3_inclination_angle = 5.0 * math.pi / 180.0
    conv_belt3_sin = math.sin(conv_belt3_inclination_angle)
    conv_belt3_cos = math.cos(conv_belt3_inclination_angle)
    conv_belt3_z = conv_belt3_sin * conv_belt3_half_extent_y_with_margin - (conv_belt3_cos * extra_shift_z)
    conv_belt3_y = (conv_belt3_cos * conv_belt3_half_extent_y_with_margin) + (conv_belt3_sin * extra_shift_z)

    conv_belt3_position = (
        conv_belt2_position[0] - conv_belt2_mid_radius,
        conv_belt2_position[1] + conveyor_belt_border_margin_y + conv_belt3_y,
        conv_belt2_position[2] + conv_belt3_z,
    )
    conv_belt3_orientation = (
        math.cos(conv_belt3_inclination_angle / 2.0),
        math.sin(conv_belt3_inclination_angle / 2.0),
        0.0,
        0.0,
    )

    conv_belt3_prim, conv_belt3_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt3_position,
        conv_belt3_orientation,
        conv_belt3_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    target_velocity = wp.vec3(
        0.0, conveyor_belt_base_vel_magn * conv_belt3_cos, conveyor_belt_base_vel_magn * conv_belt3_sin
    )

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt3_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, -conv_belt3_sin, conv_belt3_cos),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    conv_belt_guard_path = "/World/conv_belt3_guard"

    conv_belt3_guard_half_extent = (0.1, 0.5, conveyor_belt_guard_half_thickness)

    conv_belt3_guard_position = (
        conv_belt3_position[0] - conv_belt3_half_extent[0],
        conv_belt3_position[1],
        conv_belt3_position[2],
    )
    rotX = Gf.Quatf(
        math.cos(conv_belt3_inclination_angle / 2.0), math.sin(conv_belt3_inclination_angle / 2.0), 0.0, 0.0
    )
    rotZ_angle_halve = 0.5 * (-15.0 * math.pi / 180.0)
    rotZ = Gf.Quatf(math.cos(rotZ_angle_halve), 0.0, 0.0, math.sin(rotZ_angle_halve))
    rot = rotX * rotZ
    conv_belt3_guard_orientation = (rot.GetReal(), rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2])

    create_conveyor_belt_box_guard(
        stage,
        conv_belt_guard_path,
        conv_belt3_guard_position,
        conv_belt3_guard_orientation,
        conv_belt3_guard_half_extent,
        conveyor_belt_guard_material.material,
        contact_offset,
        rest_offset,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt3_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt3_position[0],
                conv_belt3_position[1] - conv_belt3_y - (conv_belt3_sin * conv_belt3_half_extent[2]),
                conv_belt3_position[2] - conv_belt3_z + (conv_belt3_cos * conv_belt3_half_extent[2]),
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt4"

    conv_belt4_half_extent = (multi_conveyor_belt_half_width, 5.0, conveyor_belt_half_thickness)

    conv_belt4_position = (
        conv_belt3_position[0] + multi_conveyor_belt_half_width + multi_conveyor_belt_half_gap,
        conv_belt3_position[1] + conv_belt3_y + conveyor_belt_border_margin_y + conv_belt4_half_extent[1],
        conv_belt3_position[2] + conv_belt3_z,
    )
    conv_belt4_orientation = (1.0, 0.0, 0.0, 0.0)

    conv_belt4_prim, conv_belt4_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt4_position,
        conv_belt4_orientation,
        conv_belt4_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    target_velocity = wp.vec3(0.0, 2.0 * conveyor_belt_base_vel_magn, 0.0)

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt4_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt4_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt4_position[0],
                conv_belt4_position[1] - conv_belt4_half_extent[1],
                conv_belt4_position[2] + conv_belt4_half_extent[2],
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt5"

    conv_belt5_half_extent = conv_belt4_half_extent

    conv_belt5_position = (
        conv_belt4_position[0] - (2.0 * (multi_conveyor_belt_half_width + multi_conveyor_belt_half_gap)),
        conv_belt4_position[1],
        conv_belt4_position[2],
    )
    conv_belt5_orientation = (1.0, 0.0, 0.0, 0.0)

    conv_belt5_prim, conv_belt5_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt5_position,
        conv_belt5_orientation,
        conv_belt5_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    target_velocity = wp.vec3(0.0, conveyor_belt_base_vel_magn, 0.0)

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt5_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt5_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt5_position[0],
                conv_belt5_position[1] - conv_belt5_half_extent[1],
                conv_belt5_position[2] + conv_belt5_half_extent[2],
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt6"

    conv_belt_6_half_extent_y = 0.5 * (
        (conv_belt0_position[0] + conv_belt0_half_extent[0])
        - (conv_belt5_position[0] - conv_belt5_half_extent[0])
        - (conveyor_belt_border_margin_y + (2.0 * conv_belt0_half_extent[0]))
    )

    # note: the belt will be rotated by 90 degrees but the extents are defined in the non-rotated frame
    conv_belt6_half_extent = (conveyor_belt_half_width * 2.0, conv_belt_6_half_extent_y, conveyor_belt_half_thickness)

    conv_belt6_position = (
        conv_belt5_position[0] - conv_belt5_half_extent[0] + conv_belt6_half_extent[1],
        conv_belt5_position[1] + conv_belt5_half_extent[1] + conveyor_belt_border_margin_y + conv_belt6_half_extent[0],
        conv_belt5_position[2],
    )
    conv_belt6_orientation = (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))

    conv_belt6_prim, conv_belt6_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt6_position,
        conv_belt6_orientation,
        conv_belt6_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    target_velocity = wp.vec3(conveyor_belt_base_vel_magn, 0.0, 0.0)

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt6_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, 0.0, 1.0),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    conv_belt_guard_path = "/World/conv_belt6_guard"

    conv_belt6_guard_half_extent = (
        conv_belt6_half_extent[1],
        conveyor_belt_guard_width,
        conveyor_belt_guard_half_thickness,
    )

    conv_belt6_guard_position = (
        conv_belt6_position[0],
        conv_belt6_position[1] + conv_belt6_half_extent[0] + conv_belt6_guard_half_extent[1],
        conv_belt6_position[2],
    )
    conv_belt6_guard_orientation = (1.0, 0.0, 0.0, 0.0)

    create_conveyor_belt_box_guard(
        stage,
        conv_belt_guard_path,
        conv_belt6_guard_position,
        conv_belt6_guard_orientation,
        conv_belt6_guard_half_extent,
        conveyor_belt_guard_material.material,
        contact_offset,
        rest_offset,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt6_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt6_position[0] - conv_belt6_half_extent[1],
                conv_belt6_position[1],
                conv_belt6_position[2] + conv_belt6_half_extent[2],
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    # ---

    conv_belt_path = "/World/conv_belt7"

    conv_belt7_offset_y = (conv_belt6_position[1] + conv_belt6_half_extent[0]) - (
        conv_belt0_position[1] + conv_belt0_half_extent[1] + conveyor_belt_border_margin_y
    )
    conv_belt7_offset_z = conv_belt6_position[2] - conv_belt0_position[2]

    conv_belt7_inclination_angle = math.atan2(conv_belt7_offset_z, conv_belt7_offset_y)

    conv_belt_7_extent_y_with_margin = math.sqrt(
        (conv_belt7_offset_y * conv_belt7_offset_y) + (conv_belt7_offset_z * conv_belt7_offset_z)
    )

    conv_belt_7_half_extent_y = 0.5 * (conv_belt_7_extent_y_with_margin - (2 * conveyor_belt_border_margin_y))

    conv_belt7_half_extent = (conv_belt0_half_extent[0], conv_belt_7_half_extent_y, conveyor_belt_half_thickness)

    conv_belt7_position = (
        conv_belt0_position[0],
        conv_belt0_position[1]
        + conv_belt0_half_extent[1]
        + conveyor_belt_border_margin_y
        + (0.5 * conv_belt7_offset_y),
        0.5 * (conv_belt0_position[2] + conv_belt6_position[2]),
    )
    conv_belt7_orientation = (
        math.cos(conv_belt7_inclination_angle / 2.0),
        math.sin(conv_belt7_inclination_angle / 2.0),
        0.0,
        0.0,
    )

    conv_belt7_prim, conv_belt7_geom_prim = create_conveyor_belt_box(
        stage,
        conv_belt_path,
        conv_belt7_position,
        conv_belt7_orientation,
        conv_belt7_half_extent,
        conveyor_belt_border_margin_y,
        conveyor_belt_material.material,
        contact_offset,
        rest_offset,
    )

    conv_belt7_sin = math.sin(conv_belt7_inclination_angle)
    conv_belt7_cos = math.cos(conv_belt7_inclination_angle)

    target_velocity = wp.vec3(
        0.0, conv_belt7_cos * (-conveyor_belt_base_vel_magn), conv_belt7_sin * (-conveyor_belt_base_vel_magn)
    )

    velocity_field_id = velocity_field_actuator.add_constant_velocity_field(
        target_velocity,
    )

    conveyor_belt_manager.add_conveyor_belt(
        str(conv_belt7_geom_prim.GetPath()),
        VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
        velocity_field_id,
        wp.vec3(0.0, -conv_belt7_sin, conv_belt7_cos),
        conveyor_belt_contact_processing_threshold,
        conveyor_belt_material_index,
    )

    conv_belt_guard_path = "/World/conv_belt7_guard"

    conv_belt7_guard_half_extent = (0.1, 1.5, conveyor_belt_guard_half_thickness)

    conv_belt7_guard_position = (
        conv_belt7_position[0] + conv_belt7_half_extent[0],
        conv_belt7_position[1] + (0.2 * conv_belt7_offset_y),
        conv_belt7_position[2] + (0.2 * conv_belt7_offset_z),
    )
    rotX = Gf.Quatf(
        math.cos(conv_belt7_inclination_angle / 2.0), math.sin(conv_belt7_inclination_angle / 2.0), 0.0, 0.0
    )
    rotZ_angle_halve = 0.5 * (-5.0 * math.pi / 180.0)
    rotZ = Gf.Quatf(math.cos(rotZ_angle_halve), 0.0, 0.0, math.sin(rotZ_angle_halve))
    rot = rotX * rotZ
    conv_belt7_guard_orientation = (rot.GetReal(), rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2])

    create_conveyor_belt_box_guard(
        stage,
        conv_belt_guard_path,
        conv_belt7_guard_position,
        conv_belt7_guard_orientation,
        conv_belt7_guard_half_extent,
        conveyor_belt_guard_material.material,
        contact_offset,
        rest_offset,
    )

    if velocity_field_visualizer is not None:

        max_dist = 2.0 * conv_belt7_half_extent[1]

        velocity_field_visualizer.add_constant_velocity_field(
            target_velocity,
            wp.vec3(
                conv_belt7_position[0],
                conv_belt7_position[1] + (conv_belt7_offset_y * 0.5) - (conv_belt7_sin * conv_belt7_half_extent[2]),
                conv_belt7_position[2] + (conv_belt7_offset_z * 0.5) + (conv_belt7_cos * conv_belt7_half_extent[2]),
            ),
            max_dist,
            int(max_dist / visualizer_marker_delta),
        )

    #
    # Objects to move on the conveyor belts
    #

    create_moving_objects = True

    box_T0_half_extent = (0.2, 0.3, 0.2)
    box_T0_mass = 2.0

    box_T1_half_extent = (0.1, 0.1, 0.1)
    box_T1_mass = 0.5

    box_T2_half_extent = (0.25, 0.25, 0.1)
    box_T2_mass = 1.0

    # ---

    if create_moving_objects:

        body_path = "/World/body0_0"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt0_position[0],
                conv_belt0_position[1],
                conv_belt0_position[2] + conv_belt0_half_extent[2] + box_T0_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T0_half_extent,
            box_T0_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body0_1"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt0_position[0] - (2.0 * box_T1_half_extent[0]),
                conv_belt0_position[1] - 1.0,
                conv_belt0_position[2] + conv_belt0_half_extent[2] + box_T1_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T1_half_extent,
            box_T1_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body0_2"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt0_position[0] + 0.1,
                conv_belt0_position[1] - 2.0,
                conv_belt0_position[2] + conv_belt0_half_extent[2] + box_T0_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T0_half_extent,
            box_T0_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_ARTICULATION_ROOT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body0_3"

        body_manager.add_body(body_path, moveable_object_material1_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt0_position[0],
                conv_belt0_position[1] + (conv_belt0_half_extent[1] * 0.8),
                conv_belt0_position[2] + conv_belt0_half_extent[2] + box_T2_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T2_half_extent,
            box_T2_mass,
            moveable_object_material1.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body1_0"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt1_position[0] + (conv_belt1_mid_radius * 0.7),
                conv_belt1_position[1] - (conv_belt1_mid_radius * 0.7),
                conv_belt1_position[2] + conveyor_belt_half_thickness + box_T1_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T1_half_extent,
            box_T1_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body2_0"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt2_position[0] - (conv_belt2_mid_radius * 0.7),
                conv_belt2_position[1] - (conv_belt2_mid_radius * 0.7),
                conv_belt2_position[2] + conveyor_belt_half_thickness + box_T0_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T0_half_extent,
            box_T0_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body4_5_0"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt4_position[0] - conv_belt4_half_extent[0] - multi_conveyor_belt_half_gap,
                conv_belt4_position[1],
                conv_belt4_position[2] + conv_belt4_half_extent[2] + box_T0_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T0_half_extent,
            box_T0_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body4_5_1"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt4_position[0],
                conv_belt4_position[1] + (1.5 * box_T0_half_extent[1]),
                conv_belt4_position[2] + conv_belt4_half_extent[2] + box_T1_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T1_half_extent,
            box_T1_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body4_5_2"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt5_position[0],
                conv_belt5_position[1] - (1.5 * box_T0_half_extent[1]),
                conv_belt5_position[2] + conv_belt5_half_extent[2] + box_T1_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T1_half_extent,
            box_T1_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )

    # ---

    if create_moving_objects:

        body_path = "/World/body4_5_3"

        body_manager.add_body(body_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt4_position[0] - (conv_belt4_half_extent[0] * 0.7) - multi_conveyor_belt_half_gap,
                conv_belt4_position[1] - (9.0 * box_T0_half_extent[1]),
                conv_belt4_position[2] + conv_belt4_half_extent[2] + box_T0_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T0_half_extent,
            box_T0_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_ARTICULATION_ROOT,
        )

    # ---

    if create_moving_objects:
        link_half_extent = (conv_belt4_half_extent[0] * 0.5, 0.05, 0.05)
        link_mass = 1.0

        link_0_path = "/World/body4_5_4"

        body_manager.add_body(link_0_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            link_0_path,
            (
                conv_belt4_position[0] - conv_belt4_half_extent[0] - multi_conveyor_belt_half_gap + link_half_extent[0],
                conv_belt4_position[1] - conv_belt4_half_extent[1] + (link_half_extent[1] * 4.0),
                conv_belt4_position[2] + conv_belt4_half_extent[2] + link_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            link_half_extent,
            link_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_ARTICULATION_ROOT,
        )

        link_1_path = "/World/body4_5_5"

        body_manager.add_body(link_1_path, moveable_object_material0_index)

        create_rigid_body_box(
            stage,
            link_1_path,
            (
                conv_belt5_position[0] + conv_belt5_half_extent[0] + multi_conveyor_belt_half_gap - link_half_extent[0],
                conv_belt5_position[1] - conv_belt5_half_extent[1] + (link_half_extent[1] * 4.0),
                conv_belt5_position[2] + conv_belt5_half_extent[2] + link_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            link_half_extent,
            link_mass,
            moveable_object_material0.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_ARTICULATION_LINK,
        )

        joint_path = "/World/joint_4_5_4_vs_body4_5_5"

        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        joint.CreateAxisAttr("Z")

        joint.CreateLowerLimitAttr().Set(-45.0)
        joint.CreateUpperLimitAttr().Set(45.0)

        joint.GetBody0Rel().AddTarget(link_0_path)
        joint.GetBody1Rel().AddTarget(link_1_path)

        joint.CreateLocalPos0Attr(Gf.Vec3f(-link_half_extent[0], 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(link_half_extent[0], 0.0, 0.0))

        physx_joint_api = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        physx_joint_api.CreateJointFrictionAttr(0.0)
        physx_joint_api.CreateArmatureAttr(0.0)

    # ---

    if create_moving_objects:

        body_path = "/World/body4_5_6"

        body_manager.add_body(body_path, moveable_object_material1_index)

        create_rigid_body_box(
            stage,
            body_path,
            (
                conv_belt4_position[0] - conv_belt4_half_extent[0] - multi_conveyor_belt_half_gap,
                conv_belt4_position[1] + (conv_belt4_half_extent[1] * 0.7),
                conv_belt4_position[2] + conv_belt4_half_extent[2] + box_T2_half_extent[2] + rest_offset_doubled,
            ),
            (1.0, 0.0, 0.0, 0.0),
            box_T2_half_extent,
            box_T2_mass,
            moveable_object_material1.material,
            contact_offset,
            rest_offset,
            BODY_TYPE_DEFAULT,
        )
