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

import math

import warp as wp
from cb_actuators import (
    VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
    VELOCITY_FIELD_TYPE_PIVOT,
)
from isaacsim.util.debug_draw import _debug_draw

# not needed for the purpose of this sample
wp.config.enable_backward = False


class VelocityFieldVisualizer:
    """Class to inject and animate point markers that help visualize velocity field speeds and
    directions.
    """

    def __init__(
        self,
        velocity_startup_duration: float,
    ) -> None:
        """
        Args:
            velocity_startup_duration: Time (in seconds) until the velocity fields reach the defined
                velocity magnitudes. This basically defines a linear acceleration phase that can be
                used to smoothly get the velocity fields from zero to their actual target velocities.
        """

        self._velocity_startup_duration = velocity_startup_duration

        self.reset()

        self._debug_draw_interface = _debug_draw.acquire_debug_draw_interface()

        self._total_marker_count = 0

        self._marker_color = (1.0, 1.0, 0.0, 1.0)
        self._marker_size = 5

        self._marker_position_list = []
        self._marker_color_list = []
        self._marker_size_list = []

        #
        # constant velocity fields
        #
        self._constant_velocity_field_target_velocity_magnitude_list = []
        self._constant_velocity_field_start_position_list = []
        self._constant_velocity_field_direction_list = []
        self._constant_velocity_field_max_distance_list = []
        self._constant_velocity_field_marker_count_list = []
        self._constant_velocity_field_marker_start_index_list = []

        #
        # pivot velocity fields
        #
        self._pivot_velocity_field_pivot_point_list = []
        self._pivot_velocity_field_angular_velocity_magnitude_list = []
        self._pivot_velocity_field_rotation_axis_list = []
        self._pivot_velocity_field_tangential_plane_axis0_list = []
        self._pivot_velocity_field_tangential_plane_axis1_list = []
        self._pivot_velocity_field_start_position_list = []
        self._pivot_velocity_field_start_angle_list = []
        self._pivot_velocity_field_end_angle_list = []
        self._pivot_velocity_field_marker_count_list = []
        self._pivot_velocity_field_marker_start_index_list = []

    def reset(
        self,
    ) -> None:

        self._total_elapsed_time = 0.0

    def add_constant_velocity_field(
        self,
        target_velocity: wp.vec3,
        start_position: wp.vec3,
        max_distance: float,
        marker_count: int,
    ) -> None:
        """Register a constant-velocity field and place its debug markers along the velocity direction.

        Args:
            target_velocity: Constant target velocity vector for this field.
            start_position: World-space position where the first marker is placed.
            max_distance: Distance along the velocity direction after which markers wrap back to the start.
            marker_count: Number of debug markers to distribute across ``max_distance``.
        """

        target_vel_magn = wp.length(target_velocity)

        self._constant_velocity_field_target_velocity_magnitude_list.append(target_vel_magn)

        self._constant_velocity_field_start_position_list.append(start_position)

        direction = wp.normalize(target_velocity)

        self._constant_velocity_field_direction_list.append(direction)

        self._constant_velocity_field_max_distance_list.append(max_distance)

        self._constant_velocity_field_marker_count_list.append(marker_count)

        self._constant_velocity_field_marker_start_index_list.append(self._total_marker_count)

        delta = wp.float32(max_distance / marker_count)

        for i in range(marker_count):

            pos = start_position + (direction * (delta * wp.float32(i)))

            self._marker_position_list.append((pos[0], pos[1], pos[2]))

            self._marker_color_list.append(self._marker_color)

            self._marker_size_list.append(self._marker_size)

        self._total_marker_count += marker_count

    def add_pivot_velocity_field(
        self,
        pivot_point: wp.vec3,
        angular_velocity: wp.vec3,
        tangent_axis0: wp.vec3,
        tangent_axis1: wp.vec3,
        radius: float,
        start_angle: float,
        end_angle: float,
        marker_count: int,
    ) -> None:
        """Register a pivot (rotational) velocity field and place its debug markers along the arc.

        Args:
            pivot_point: World-space center of rotation.
            angular_velocity: Angular velocity vector (direction is rotation axis, magnitude is speed).
            tangent_axis0: First axis of the tangential plane (used to compute marker positions on the arc).
            tangent_axis1: Second axis of the tangential plane, orthogonal to ``tangent_axis0``.
            radius: Distance from ``pivot_point`` at which markers are placed.
            start_angle: Start angle of the arc in radians.
            end_angle: End angle of the arc in radians; markers wrap back to ``start_angle`` after this.
            marker_count: Number of debug markers distributed across the arc.
        """

        self._pivot_velocity_field_pivot_point_list.append(pivot_point)

        rotation_axis = wp.normalize(angular_velocity)

        self._pivot_velocity_field_rotation_axis_list.append(rotation_axis)

        target_vel_magn = wp.length(angular_velocity)

        self._pivot_velocity_field_angular_velocity_magnitude_list.append(target_vel_magn)

        self._pivot_velocity_field_tangential_plane_axis0_list.append(tangent_axis0)
        self._pivot_velocity_field_tangential_plane_axis1_list.append(tangent_axis1)

        self._pivot_velocity_field_start_angle_list.append(start_angle)
        self._pivot_velocity_field_end_angle_list.append(end_angle)

        self._pivot_velocity_field_marker_count_list.append(marker_count)

        self._pivot_velocity_field_marker_start_index_list.append(self._total_marker_count)

        delta_angle = wp.float32((end_angle - start_angle) / marker_count)

        rotation = wp.quat_from_axis_angle(rotation_axis, delta_angle)

        start_delta_position_0 = tangent_axis0 * (radius * wp.cos(start_angle))
        start_delta_position_1 = tangent_axis1 * (radius * wp.sin(start_angle))

        delta_position = start_delta_position_0 + start_delta_position_1

        self._pivot_velocity_field_start_position_list.append(pivot_point + delta_position)

        for i in range(marker_count):

            pos = pivot_point + delta_position

            self._marker_position_list.append((pos[0], pos[1], pos[2]))

            self._marker_color_list.append(self._marker_color)

            self._marker_size_list.append(self._marker_size)

            delta_position = wp.quat_rotate(rotation, delta_position)

        self._total_marker_count += marker_count

    def update(
        self,
        dt: float,
    ) -> None:
        """Advance all marker positions by one time step and redraw them via the debug-draw interface.

        Args:
            dt: Elapsed simulation time for the current step in seconds.
        """

        self._total_elapsed_time += dt

        if self._total_elapsed_time > self._velocity_startup_duration:
            global_velocity_scale = 1.0
        else:
            global_velocity_scale = self._total_elapsed_time / self._velocity_startup_duration

        for i in range(len(self._constant_velocity_field_target_velocity_magnitude_list)):

            target_vel_magn = self._constant_velocity_field_target_velocity_magnitude_list[i] * global_velocity_scale
            start_position = self._constant_velocity_field_start_position_list[i]
            direction = self._constant_velocity_field_direction_list[i]
            max_distance = self._constant_velocity_field_max_distance_list[i]
            marker_count = self._constant_velocity_field_marker_count_list[i]
            marker_start_index = self._constant_velocity_field_marker_start_index_list[i]

            delta = direction * (dt * target_vel_magn)

            index = marker_start_index
            end_index_plus_one = marker_start_index + marker_count

            while index < end_index_plus_one:

                old_pos_tmp = self._marker_position_list[index]
                old_pos = wp.vec3(old_pos_tmp[0], old_pos_tmp[1], old_pos_tmp[2])

                new_pos = old_pos + delta

                new_dist = wp.dot((new_pos - start_position), direction)

                if new_dist > max_distance:
                    new_pos = start_position

                self._marker_position_list[index] = (new_pos[0], new_pos[1], new_pos[2])

                index += 1

        for i in range(len(self._pivot_velocity_field_pivot_point_list)):

            pivot_point = self._pivot_velocity_field_pivot_point_list[i]
            angular_velocity_magn = (
                self._pivot_velocity_field_angular_velocity_magnitude_list[i] * global_velocity_scale
            )
            rotation_axis = self._pivot_velocity_field_rotation_axis_list[i]
            tangent_axis0 = self._pivot_velocity_field_tangential_plane_axis0_list[i]
            tangent_axis1 = self._pivot_velocity_field_tangential_plane_axis1_list[i]
            start_position = self._pivot_velocity_field_start_position_list[i]
            start_angle = self._pivot_velocity_field_start_angle_list[i]
            end_angle = self._pivot_velocity_field_end_angle_list[i]
            marker_count = self._pivot_velocity_field_marker_count_list[i]
            marker_start_index = self._pivot_velocity_field_marker_start_index_list[i]

            delta_angle = dt * angular_velocity_magn
            rotation = wp.quat_from_axis_angle(rotation_axis, delta_angle)

            index = marker_start_index
            end_index_plus_one = marker_start_index + marker_count

            while index < end_index_plus_one:

                old_pos_tmp = self._marker_position_list[index]
                old_pos = wp.vec3(old_pos_tmp[0], old_pos_tmp[1], old_pos_tmp[2])

                delta = old_pos - pivot_point

                delta_0 = wp.dot(delta, tangent_axis0)
                delta_1 = wp.dot(delta, tangent_axis1)

                angle = wp.atan2(delta_1, delta_0)

                if (angle < 0) and (angle < start_angle):
                    angle += 2.0 * wp.pi

                angle += delta_angle

                if angle < end_angle:
                    delta = wp.quat_rotate(rotation, delta)
                    new_pos = delta + pivot_point
                else:
                    new_pos = start_position

                self._marker_position_list[index] = (new_pos[0], new_pos[1], new_pos[2])

                index += 1

        self._debug_draw_interface.clear_points()

        self._debug_draw_interface.draw_points(
            self._marker_position_list,
            self._marker_color_list,
            self._marker_size_list,
        )
