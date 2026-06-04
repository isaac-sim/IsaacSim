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

"""Provides classes for representing and converting joint-space paths to minimal-time trajectories."""

from typing import Optional, Union

import numpy as np
import warp as wp
from isaacsim.core.experimental.utils.ops import place

from .trajectory import Trajectory
from .types import JointState, RobotState


class Path:
    """A path in joint-space represented as a series of waypoints connected linearly.

    Basic conversion to a trajectory is provided.

    Args:
        waypoints: The waypoints of the path in joint-space. Can be a list, numpy array, or warp array.

    Raises:
        ValueError: If waypoints is not a two-dimensional array.
    """

    def __init__(self, waypoints: Union[list, np.ndarray, wp.array]):
        # internally represent all waypoints as a warp array.
        warp_waypoints = place(waypoints)
        self._waypoints = warp_waypoints

        # the waypoints must be a two-dimensional array:
        if self._waypoints.ndim != 2:
            raise ValueError("Waypoints must be a two-dimensional array.")

        self._n_waypoints = self._waypoints.shape[0]

        # also store a numpy representation of the waypoints, such that waypoints can be
        # requested by index without having to copy from the GPU --> CPU on each call.
        self._waypoints_np = self._waypoints.numpy()

    def get_waypoints(self) -> wp.array:
        """Get the waypoints of the path.

        Returns:
            The waypoints of the path in joint-space as a warp array.
        """
        return self._waypoints

    def get_waypoints_count(self) -> int:
        """Get the number of waypoints in the path.

        Returns:
            The number of waypoints in the path.
        """
        return self._n_waypoints

    def get_waypoint_by_index(self, index: int) -> np.ndarray:
        """Get the waypoint at the given index.

        Note this returns a numpy array, not a warp array, as warp arrays are not indexable
        outside of warp functions/kernels.

        Args:
            index: The index of the waypoint.

        Returns:
            The waypoint at the given index as a numpy array.

        Raises:
            IndexError: If the waypoint index is out of range.
        """
        if (index < 0) or (index >= self._n_waypoints):
            raise IndexError("Waypoint index is out of range.")

        return self._waypoints_np[index]

    def to_minimal_time_joint_trajectory(
        self,
        max_velocities: Union[list, np.ndarray, wp.array],
        max_accelerations: Union[list, np.ndarray, wp.array],
        robot_joint_space: list[str],
        active_joints: list[str],
        waypoint_relative_difference_tolerance: float = 1e-6,
        waypoint_absolute_difference_tolerance: float = 1e-10,
    ) -> Trajectory:
        """Convert the path to a minimal-time trajectory.

        Args:
            max_velocities: The maximum joint velocities.
            max_accelerations: The maximum joint accelerations.
            robot_joint_space: The ordered list of joint names defining the joint space.
            active_joints: The active joints.
            waypoint_relative_difference_tolerance: Minimal relative difference required between waypoints.
            waypoint_absolute_difference_tolerance: Minimal absolute difference required between waypoints.

        Returns:
            The minimal-time trajectory.

        Raises:
            ValueError: If waypoints is not a two-dimensional array.
            ValueError: If max_velocities or max_accelerations is not a one-dimensional array.
            ValueError: If max_velocities length does not match the joint-dimensionality of waypoints.
            ValueError: If max_accelerations length does not match the joint-dimensionality of waypoints.
            ValueError: If active_joints length does not match the joint-dimensionality of waypoints.
            ValueError: If any max_velocities value is not strictly greater than 0.
            ValueError: If any max_accelerations value is not strictly greater than 0.
            ValueError: If tolerance values are less than or equal to 0.
            ValueError: If consecutive waypoints are equal within the allowed tolerances.
            ValueError: If there are not at least two waypoints.
        """
        return MinimalTimeJointTrajectory(
            path=self,
            max_velocities=place(max_velocities, dtype=self._waypoints.dtype, device=self._waypoints.device),
            max_accelerations=place(max_accelerations, dtype=self._waypoints.dtype, device=self._waypoints.device),
            robot_joint_space=robot_joint_space,
            active_joints=active_joints,
            waypoint_relative_difference_tolerance=waypoint_relative_difference_tolerance,
            waypoint_absolute_difference_tolerance=waypoint_absolute_difference_tolerance,
        )


class MinimalTimeJointTrajectory(Trajectory):
    """Converts a Path-object into a minimal-time trajectory.

    This class converts a discrete set of joint-space waypoints into a trajectory, given
    maximum joint velocities and accelerations. This class is intended to be constructed
    by the Path class (see Path.to_minimal_time_trajectory).

    Args:
        path: The Path object containing waypoints in joint-space.
        max_velocities: The maximum joint velocities.
        max_accelerations: The maximum joint accelerations.
        robot_joint_space: The ordered list of joint names defining the joint space.
        active_joints: The active joints.
        waypoint_relative_difference_tolerance: Minimal relative difference required between waypoints.
        waypoint_absolute_difference_tolerance: Minimal absolute difference required between waypoints.

    Raises:
        ValueError: If waypoints is not a two-dimensional array.
        ValueError: If max_velocities or max_accelerations is not a one-dimensional array.
        ValueError: If max_velocities length does not match the joint-dimensionality of waypoints.
        ValueError: If max_accelerations length does not match the joint-dimensionality of waypoints.
        ValueError: If active_joints length does not match the joint-dimensionality of waypoints.
        ValueError: If any max_velocities value is not strictly greater than 0.
        ValueError: If any max_accelerations value is not strictly greater than 0.
        ValueError: If tolerance values are less than or equal to 0.
        ValueError: If consecutive waypoints are equal within the allowed tolerances.
        ValueError: If there are not at least two waypoints.
    """

    def __init__(
        self,
        path: Path,
        max_velocities: Union[list, np.ndarray, wp.array],
        max_accelerations: Union[list, np.ndarray, wp.array],
        robot_joint_space: list[str],
        active_joints: list[str],
        waypoint_relative_difference_tolerance: float,
        waypoint_absolute_difference_tolerance: float,
    ):
        self._robot_joint_space = robot_joint_space
        self._active_joints = active_joints
        self._waypoints = path.get_waypoints()
        self._n_waypoints = path.get_waypoints_count()

        if self._waypoints.ndim != 2:
            raise ValueError("Waypoints must be a two-dimensional array.")

        if max_velocities.ndim != 1 or max_accelerations.ndim != 1:
            raise ValueError("Max velocity and max acceleration must be one-dimensional arrays.")

        n_joints = self._waypoints.shape[1]

        if len(max_velocities) != n_joints:
            raise ValueError("Max velocities must match the joint-dimensionality of the waypoints.")
        if len(max_accelerations) != n_joints:
            raise ValueError("Max accelerations must match the joint-dimensionality of the waypoints.")
        if len(active_joints) != n_joints:
            raise ValueError("Active joints must match the joint-dimensionality of the waypoints.")

        if isinstance(max_velocities, wp.array):
            max_velocities_np = max_velocities.numpy()
        else:
            max_velocities_np = np.array(max_velocities)

        if isinstance(max_accelerations, wp.array):
            max_accelerations_np = max_accelerations.numpy()
        else:
            max_accelerations_np = np.array(max_accelerations)

        if not (max_velocities_np > np.zeros_like(max_velocities_np)).all():
            raise ValueError("Max velocities must be strictly greater than 0.")

        if not (max_accelerations_np > np.zeros_like(max_accelerations_np)).all():
            raise ValueError("Max accelerations must be strictly greater than 0.")

        if self._n_waypoints <= 1:
            raise ValueError("Path must have at least 2 waypoints.")

        if (waypoint_absolute_difference_tolerance <= 0.0) or (waypoint_relative_difference_tolerance <= 0.0):
            raise ValueError(
                f"Neither waypoint_relative_difference_tolerance nor waypoint_absolute_difference_tolerance can be less than or equal to 0"
            )

        # verify that there are no duplicate waypoints:
        waypoints_np = self._waypoints.numpy()
        self._waypoints_np = waypoints_np
        for i in range(len(waypoints_np) - 1):
            if np.allclose(
                waypoints_np[i, :],
                waypoints_np[i + 1, :],
                rtol=waypoint_relative_difference_tolerance,
                atol=waypoint_absolute_difference_tolerance,
            ):
                raise ValueError(
                    f"Waypoint {i} and waypoint {i+1} are equal within allowed tolerances. Waypoints are:\n"
                    + f"Waypoint {i}: {waypoints_np[i,:]}\n"
                    + f"Waypoint {i+1}: {waypoints_np[i+1,:]}\n"
                    + f"If these waypoints are intentional, please reduce the tolerances. Tolerances must be greater than 0",
                )

        # some useful parameters to have:
        self._n_waypoints = self._waypoints.shape[0]
        self._n_segments = self._n_waypoints - 1
        self._n_joints = self._waypoints.shape[1]

        # this scales the velocity and acceleration available to each joint by segment,
        # ensuring that all joints take the same amount of time as the slowest joint!
        self._segment_durations, self._scaled_maximum_velocities, self._scaled_maximum_accelerations = (
            self._scale_velocity_and_acceleration_on_all_segments(self._waypoints, max_velocities, max_accelerations)
        )

        # compute the switching times between segments:
        segment_switching_times = np.zeros([self._n_waypoints])
        for i in range(1, self._n_waypoints):
            segment_switching_times[i] = self._segment_durations.numpy()[i - 1] + segment_switching_times[i - 1]

        self._segment_switching_times = place(
            segment_switching_times, dtype=self._waypoints.dtype, device=self._waypoints.device
        )
        self._duration = float(segment_switching_times[-1])

        # Used to search for segment index in non-warp code.
        self._segment_switching_times_np = segment_switching_times.copy()

    def _scale_maximum_velocity_and_acceleration_on_one_segment(
        self, joint_deltas: np.array, joint_v_max: np.array, joint_a_max: np.array
    ) -> tuple[float, np.array, np.array]:
        """Scale the maximum velocity and acceleration on one segment.

        This is used to make sure that all joints take the same amount of time as the slowest
        joint, while not altering the shape of the original path through space.

        Args:
            joint_deltas: The joint deltas on the segment.
            joint_v_max: The maximum joint velocities.
            joint_a_max: The maximum joint accelerations.

        Returns:
            A tuple containing the motion time, the scaled maximum velocities, and the scaled
            maximum accelerations.
        """
        # substitution variables p and q:
        velocity_constraints = np.zeros_like(joint_deltas)
        acceleration_constraints = np.zeros_like(joint_deltas)

        for i_joint in range(joint_deltas.size):
            velocity_constraints[i_joint] = 2 * np.abs(joint_deltas[i_joint]) / joint_v_max[i_joint]
            acceleration_constraints[i_joint] = 4 * np.abs(joint_deltas[i_joint]) / joint_a_max[i_joint]

        # find the limiting velocity and acceleration curves:
        velocity_constraint = np.max(velocity_constraints)
        acceleration_constraint = np.max(acceleration_constraints)

        if velocity_constraint < np.sqrt(acceleration_constraint):
            p = q = np.sqrt(acceleration_constraint)
        else:
            p = velocity_constraint
            q = acceleration_constraint / p

        # with p and q, we can solve for:
        #  a) the total motion time;
        #  b) the drifting ration; and
        #  c) the maximum velocity and acceleration for each joint.
        motion_time = (p + q) / 2

        joint_v_max_out = joint_v_max.copy()
        joint_a_max_out = joint_a_max.copy()

        for i_joint in range(joint_deltas.size):
            joint_v_max_out[i_joint] = 2 * np.abs(joint_deltas[i_joint]) / p
            joint_a_max_out[i_joint] = 4 * np.abs(joint_deltas[i_joint]) / (p * q)

        return (motion_time, joint_v_max_out, joint_a_max_out)

    def _scale_velocity_and_acceleration_on_all_segments(
        self, waypoints: wp.array, max_velocities: wp.array, max_accelerations: wp.array
    ) -> tuple[wp.array, wp.array, wp.array]:
        """Scale the maximum velocity and acceleration on all segments.

        This is used to make sure that all joints take the same amount of time as the slowest
        joint, while not altering the shape of the original path through space.

        Args:
            waypoints: The waypoints of the path in joint-space.
            max_velocities: The maximum joint velocities.
            max_accelerations: The maximum joint accelerations.

        Returns:
            A tuple containing the motion times, the scaled maximum velocities, and the scaled
            maximum accelerations for all segments.
        """
        # This function is only called once at instantiation. Therefore,
        # using numpy types for simplicity.
        waypoints = waypoints.numpy()
        max_velocities = max_velocities.numpy()
        max_accelerations = max_accelerations.numpy()

        n_waypoints = waypoints.shape[0]
        n_segments = n_waypoints - 1
        n_joints = waypoints.shape[1]
        scaled_maximum_velocities = np.zeros([n_segments, n_joints])
        scaled_maximum_accelerations = np.zeros([n_segments, n_joints])
        motion_times = np.zeros(
            [
                n_segments,
            ]
        )

        for i_seg in range(n_segments):
            joints_delta = waypoints[i_seg + 1] - waypoints[i_seg]
            motion_times[i_seg], scaled_maximum_velocities[i_seg, :], scaled_maximum_accelerations[i_seg, :] = (
                self._scale_maximum_velocity_and_acceleration_on_one_segment(
                    joints_delta, max_velocities, max_accelerations
                )
            )
        return (
            place(motion_times, dtype=self._waypoints.dtype, device=self._waypoints.device),
            place(scaled_maximum_velocities, dtype=self._waypoints.dtype, device=self._waypoints.device),
            place(scaled_maximum_accelerations, dtype=self._waypoints.dtype, device=self._waypoints.device),
        )

    @property
    def duration(self) -> float:
        """Get the duration of the trajectory.

        Returns:
            The duration of the trajectory in seconds.
        """
        return self._duration

    def get_active_joints(self) -> list[str]:
        """Get the active joints of the trajectory.

        Returns:
            The names of the active joints in the trajectory.
        """
        return self._active_joints

    def get_target_state(self, trajectory_time: float) -> Optional[RobotState]:
        """Get the target robot state at the given time.

        Args:
            trajectory_time: Time along the trajectory at which to sample the target state.

        Returns:
            Desired robot state at the given time, or None if a target cannot be computed.

        Example:

        .. code-block:: python

            >>> target = trajectory.get_target_state(trajectory_time)
            >>> if target is None:
            ...     raise RuntimeError("No target available")
        """
        # bound the trajectory input time.
        trajectory_time = np.clip(trajectory_time, 0.0, self.duration)

        # find which segment we are in:
        i_segment = np.clip(
            np.searchsorted(self._segment_switching_times_np, trajectory_time) - 1, 0, self._n_waypoints - 2
        )

        positions_out = wp.zeros([self._n_joints], dtype=self._waypoints.dtype, device=self._waypoints.device)
        velocities_out = wp.zeros([self._n_joints], dtype=self._waypoints.dtype, device=self._waypoints.device)

        warp_time = self._waypoints.dtype(float(trajectory_time))
        warp_i_segment = int(i_segment)

        wp.launch(
            _get_joint_targets_kernel,
            dim=self._n_joints,
            inputs=[
                self._waypoints,
                self._scaled_maximum_velocities,
                self._scaled_maximum_accelerations,
                self._segment_durations,
                self._segment_switching_times,
                warp_time,
                warp_i_segment,
            ],
            outputs=[
                positions_out,
                velocities_out,
            ],
            device=self._waypoints.device,
        )

        return RobotState(
            joints=JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._active_joints, positions_out),
                velocities=(self._active_joints, velocities_out),
                efforts=None,
            )
        )


@wp.func
def _compute_joint_position_velocity(
    t_since_segment_start: wp.Float,
    motion_segment_duration: wp.Float,
    s_delta: wp.Float,
    v_max: wp.Float,
    a_max: wp.Float,
) -> tuple[wp.Float, wp.Float]:
    """Compute a single joint's position and velocity at the given time.

    Args:
        t_since_segment_start: The time since the segment start.
        motion_segment_duration: The duration of the motion segment.
        s_delta: The delta of the motion segment (position change).
        v_max: The maximum velocity for this joint on this segment.
        a_max: The maximum acceleration for this joint on this segment.

    Returns:
        A tuple containing the joint position offset and velocity at the given time.
    """
    # If there is no motion, then there shouldn't
    # be any velocity or acceleration.
    zero = type(t_since_segment_start)(0.0)
    if s_delta == zero:
        return zero, zero

    # get the sign of the motion here, we will treat them all as positive
    # internally and just change the sign when we return.
    sign = type(t_since_segment_start)(1.0) if s_delta >= 0.0 else type(t_since_segment_start)(-1.0)
    s_delta = wp.abs(s_delta)
    # get some switching times internal to this set of waypoints.
    t_switch_no_drift = wp.sqrt(s_delta / a_max)
    trajectory_includes_drifting = a_max * t_switch_no_drift > v_max

    # compute the total trajectory time for this joint IF there is drifting
    # at max velocity in the middle of the trajectory:
    if trajectory_includes_drifting:
        pos, vel = _compute_joint_position_velocity_drifting(
            t_since_segment_start, motion_segment_duration, s_delta, v_max, a_max
        )
        return sign * pos, sign * vel

    # total trajectory time if there is no drifting:
    pos, vel = _compute_joint_position_velocity_no_drifting(
        t_since_segment_start, motion_segment_duration, s_delta, v_max, a_max
    )
    return sign * pos, sign * vel


@wp.func
def _compute_joint_position_velocity_drifting(
    t_since_segment_start: wp.Float,
    motion_segment_duration: wp.Float,
    s_delta: wp.Float,
    v_max: wp.Float,
    a_max: wp.Float,
) -> tuple[wp.Float, wp.Float]:
    """Compute joint position and velocity when there is a constant velocity (drifting) phase.

    This helper function handles the case where the trajectory includes a phase at maximum
    velocity between the acceleration and deceleration phases.

    Args:
        t_since_segment_start: The time since the segment start.
        motion_segment_duration: The duration of the motion segment.
        s_delta: The delta of the motion segment (position change).
        v_max: The maximum velocity for this joint on this segment.
        a_max: The maximum acceleration for this joint on this segment.

    Returns:
        A tuple containing the joint position offset and velocity at the given time.
    """
    t_switch = v_max / a_max
    two = type(t_since_segment_start)(2.0)
    half = type(t_since_segment_start)(0.5)

    drift_duration = motion_segment_duration - two * t_switch
    if t_since_segment_start < t_switch:
        return half * a_max * t_since_segment_start**two, a_max * t_since_segment_start

    if t_since_segment_start < (t_switch + drift_duration):
        t_since_switch = t_since_segment_start - t_switch
        return half * a_max * t_switch**two + v_max * t_since_switch, v_max

    t_since_switch = t_since_segment_start - (t_switch + drift_duration)
    return (
        half * a_max * t_switch**two
        + v_max * drift_duration
        + v_max * t_since_switch
        - half * a_max * t_since_switch**two,
        v_max - a_max * t_since_switch,
    )


@wp.func
def _compute_joint_position_velocity_no_drifting(
    t_since_segment_start: wp.Float,
    motion_segment_duration: wp.Float,
    s_delta: wp.Float,
    v_max: wp.Float,
    a_max: wp.Float,
) -> tuple[wp.Float, wp.Float]:
    """Compute joint position and velocity when there is no constant velocity phase.

    This helper function handles the case where the trajectory transitions directly from
    acceleration to deceleration without a constant velocity phase.

    Args:
        t_since_segment_start: The time since the segment start.
        motion_segment_duration: The duration of the motion segment.
        s_delta: The delta of the motion segment (position change).
        v_max: The maximum velocity for this joint on this segment.
        a_max: The maximum acceleration for this joint on this segment.

    Returns:
        A tuple containing the joint position offset and velocity at the given time.
    """
    two = type(t_since_segment_start)(2.0)
    half = type(t_since_segment_start)(0.5)

    t_switch = wp.sqrt(s_delta / a_max)
    if t_since_segment_start < t_switch:
        return half * a_max * t_since_segment_start**two, a_max * t_since_segment_start

    t_since_switch = t_since_segment_start - t_switch
    return (
        half * a_max * t_switch**two + a_max * t_switch * t_since_switch - half * a_max * t_since_switch**two,
        a_max * t_switch - a_max * t_since_switch,
    )


@wp.kernel
def _get_joint_targets_kernel(
    waypoints: wp.array(dtype=wp.Float, ndim=2),
    scaled_maximum_velocities: wp.array(dtype=wp.Float, ndim=2),
    scaled_maximum_accelerations: wp.array(dtype=wp.Float, ndim=2),
    segment_durations: wp.array(dtype=wp.Float),
    segment_switching_times: wp.array(dtype=wp.Float),
    trajectory_time: wp.Float,
    i_segment: int,
    out_desired_position: wp.array(dtype=wp.Float),
    out_desired_velocity: wp.array(dtype=wp.Float),
):
    """Warp kernel to compute joint targets for all joints in parallel.

    Args:
        waypoints: The waypoints of the path in joint-space (n_waypoints x n_joints).
        scaled_maximum_velocities: The scaled maximum velocities per segment per joint.
        scaled_maximum_accelerations: The scaled maximum accelerations per segment per joint.
        segment_durations: The duration of each segment.
        segment_switching_times: The cumulative time at which each segment starts.
        trajectory_time: The current time in the trajectory.
        i_segment: The index of the current segment.
        out_desired_position: Output array for desired joint positions.
        out_desired_velocity: Output array for desired joint velocities.
    """
    i_joint = wp.tid()
    p0 = waypoints[i_segment, i_joint]
    p1 = waypoints[i_segment + 1, i_joint]
    joint_motion = p1 - p0
    maximum_velocity = scaled_maximum_velocities[i_segment, i_joint]
    maximum_acceleration = scaled_maximum_accelerations[i_segment, i_joint]
    segment_duration = segment_durations[i_segment]
    segment_start_time = segment_switching_times[i_segment]

    t_since_segment_start = trajectory_time - segment_start_time

    pos, vel = _compute_joint_position_velocity(
        t_since_segment_start, segment_duration, joint_motion, maximum_velocity, maximum_acceleration
    )
    out_desired_position[i_joint] = pos + p0
    out_desired_velocity[i_joint] = vel
