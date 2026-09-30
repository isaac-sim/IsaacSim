# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Derive an IMU reading from the state of the rigid body the sensor is mounted on.

The implementation uses Warp types ordered ``xyzw``, like the physics buffers the state comes from.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field

import warp as wp

#: Identity rotation, ordered ``xyzw``.
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
ZERO_VECTOR = (0.0, 0.0, 0.0)


def average_quaternions(window: Sequence[Sequence[float]]) -> wp.quatd:
    """Average a window of quaternions, the one rotation builtin Warp lacks.

    Args:
        window: Quaternions, ordered ``xyzw``.

    Returns:
        The mean rotation, or the identity for an empty window.

    """
    if len(window) == 0:
        return wp.quatd(IDENTITY_QUATERNION)
    reference = wp.quatd(window[0])
    total = wp.quatd(0.0, 0.0, 0.0, 0.0)
    for entry in window:
        quaternion = wp.quatd(entry)
        # A quaternion and its negation are one rotation, so align signs or the window cancels.
        total = total + (quaternion if float(wp.dot(quaternion, reference)) >= 0.0 else -quaternion)
    # Warp normalizes a zero quaternion to the identity, so a cancelling window is safe.
    return wp.normalize(total)


@dataclass
class BodyState:
    """World-frame state of the rigid body a sensor is mounted on."""

    #: Orientation of the body, ordered ``xyzw``.
    orientation: wp.quatd = field(default_factory=lambda: wp.quatd(IDENTITY_QUATERNION))
    #: Linear velocity at the body origin.
    linear_velocity: wp.vec3d = field(default_factory=lambda: wp.vec3d(ZERO_VECTOR))
    #: Angular velocity of the body.
    angular_velocity: wp.vec3d = field(default_factory=lambda: wp.vec3d(ZERO_VECTOR))

    def __post_init__(self) -> None:
        """Convert every field, which also checks its component count."""
        self.orientation = wp.quatd(self.orientation)
        self.linear_velocity = wp.vec3d(self.linear_velocity)
        self.angular_velocity = wp.vec3d(self.angular_velocity)


@dataclass
class ImuReading:
    """One filtered reading, in the sensor frame unless a field says otherwise.

    Window means lag the newest sample by ``(filter_width - 1) / 2`` steps as the shipped sensor's do.
    """

    #: Simulation time of the newest sample, in seconds.
    time: float
    #: World orientation of the sensor, ordered ``xyzw``.
    orientation: wp.quatd
    #: Angular velocity of the sensor.
    angular_velocity: wp.vec3d
    #: Specific force, ``a - g``.
    linear_acceleration: wp.vec3d


@dataclass
class _Sample:
    """One derived sample, buffered for the rolling average."""

    time: float
    orientation: wp.quatd
    # World frame on purpose: see `ImuSensor.read()`.
    linear_velocity_world: wp.vec3d
    angular_velocity: wp.vec3d


class ImuSensor:
    """An IMU rigidly mounted on a body.

    The sensor buffers ``2 * filter_width`` samples so a reading can average one window and difference it against the
    window before.

    Args:
        gravity: World-frame gravitational acceleration, subtracted from every reading to give
            specific force. Required, since it follows the stage's linear unit and up axis:
            ``(0.0, 0.0, -9.80665)`` for a Z-up stage in meters, or zero for coordinate
            acceleration.
        mount_translation: Translation of the sensor in the body frame.
        mount_orientation: Orientation of the sensor in the body frame, ordered ``xyzw`` and
            normalized here.
        filter_width: Rolling average window applied to every filtered quantity.

    Raises:
        ValueError: If `filter_width` is less than one, or a component count is wrong.

    """

    def __init__(
        self,
        gravity: Sequence[float],
        *,
        mount_translation: Sequence[float] = ZERO_VECTOR,
        mount_orientation: Sequence[float] = IDENTITY_QUATERNION,
        filter_width: int = 1,
    ) -> None:
        if filter_width < 1:
            raise ValueError(f"`filter_width` must be at least 1, got {filter_width}.")
        self._gravity = wp.vec3d(gravity)
        self._mount_translation = wp.vec3d(mount_translation)
        self._mount_orientation = wp.normalize(wp.quatd(mount_orientation))
        self._filter_width = filter_width
        self._samples: deque[_Sample] = deque(maxlen=2 * filter_width)

    @property
    def sample_count(self) -> int:
        """Number of samples currently buffered."""
        return len(self._samples)

    def reset(self) -> None:
        """Discard every buffered sample.

        A caller resets the sensor when the body state jumps and a stale sample would contaminate the finite difference.
        """
        self._samples.clear()

    def sample(self, time: float, state: BodyState) -> None:
        """Derive one sample from a body state and buffer it.

        The sample may have any timestamp because `read()` skips differences over a zero or negative interval.

        Args:
            time: Simulation time of the state, in seconds. Must increase for the difference to
                mean anything.
            state: World-frame state of the body the sensor is mounted on.

        """
        body_orientation = wp.normalize(state.orientation)
        # The mount offset in the world, which is the lever arm the transport term needs.
        lever_arm = wp.quat_rotate(body_orientation, self._mount_translation)
        # wp.mul applies the right operand first, so this is the body rotation after the mount.
        orientation = wp.normalize(wp.mul(body_orientation, self._mount_orientation))
        # An offset point picks up the transport term w x r; angular velocity is the same everywhere.
        world_linear_velocity = state.linear_velocity + wp.cross(state.angular_velocity, lever_arm)

        self._samples.append(
            _Sample(
                time=time,
                orientation=orientation,
                linear_velocity_world=world_linear_velocity,
                angular_velocity=wp.quat_rotate_inv(orientation, state.angular_velocity),
            )
        )

    def read(self) -> ImuReading | None:
        """Produce a filtered reading from the buffered samples.

        Returns:
            The reading, or ``None`` until ``2 * filter_width`` samples have been buffered.

        """
        if len(self._samples) < 2 * self._filter_width:
            return None

        # Newest first, so an index and that index one window later are a difference apart.
        samples = list(reversed(self._samples))
        newest = samples[0]
        window = samples[: self._filter_width]

        angular_velocity = sum((entry.angular_velocity for entry in window), wp.vec3d(ZERO_VECTOR))
        angular_velocity = angular_velocity * wp.float64(1.0 / len(window))

        # Difference the world-frame velocity, never the sensor-frame one: differentiating in the
        # rotating frame subtracts the transport term back out.
        linear_acceleration = wp.vec3d(ZERO_VECTOR)
        usable = 0
        for index in range(self._filter_width):
            newer, older = samples[index], samples[index + self._filter_width]
            interval = newer.time - older.time
            if interval > 0.0:
                difference = newer.linear_velocity_world - older.linear_velocity_world
                linear_acceleration = linear_acceleration + difference * wp.float64(1.0 / interval)
                usable += 1
        # Average over the usable differences only, or a repeated timestamp biases this toward zero.
        if usable > 0:
            linear_acceleration = linear_acceleration * wp.float64(1.0 / usable)

        # An accelerometer measures specific force, a - g, still in the world frame; only the
        # reported result is rotated into the sensor frame.
        linear_acceleration = linear_acceleration - self._gravity

        return ImuReading(
            time=newest.time,
            # Newest first, so the newest sample sets the sign convention.
            orientation=average_quaternions([entry.orientation for entry in window]),
            angular_velocity=angular_velocity,
            linear_acceleration=wp.quat_rotate_inv(newest.orientation, linear_acceleration),
        )
