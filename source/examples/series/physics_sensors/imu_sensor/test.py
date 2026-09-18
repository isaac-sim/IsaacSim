# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for `imu`.

The tests check rotations against an independent matrix formulation rather than against `imu` itself.
"""

from __future__ import annotations

import math
import random
from typing import Any

import imu
import warp as wp
from imu import BodyState, ImuSensor

TOLERANCE = 1.0e-9
# Ordered xyzw, matching `imu`. Warp builtins accept Warp types only, so these are wp.quatd
# rather than tuples.
QUARTER_TURN_ABOUT_Z = wp.quatd(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
QUARTER_TURN_ABOUT_Y = wp.quatd(0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0))
QUARTER_TURN_ABOUT_X = wp.quatd(math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))


def _rotate_by_matrix(quaternion: Any, vector: Any) -> Any:
    """Rotate a vector with a rotation matrix, independently of `imu`.

    Args:
        quaternion: Components, ordered ``xyzw``.
        vector: Components, ordered ``xyz``.

    Returns:
        The rotated vector.
    """
    components = [float(value) for value in quaternion]
    length = math.sqrt(sum(value * value for value in components))
    x, y, z, w = (value / length for value in components)
    matrix = wp.mat33d(
        1 - 2 * (y * y + z * z),
        2 * (x * y - w * z),
        2 * (x * z + w * y),
        2 * (x * y + w * z),
        1 - 2 * (x * x + z * z),
        2 * (y * z - w * x),
        2 * (x * z - w * y),
        2 * (y * z + w * x),
        1 - 2 * (x * x + y * y),
    )
    return matrix * wp.vec3d([float(value) for value in vector])


def _conjugate(quaternion: Any) -> Any:
    """Conjugate an xyzw quaternion, independently of `imu`.

    Args:
        quaternion: Components, ordered ``xyzw``.

    Returns:
        The conjugate, ordered ``xyzw``.
    """
    x, y, z, w = (float(value) for value in quaternion)
    return (-x, -y, -z, w)


def _assert_close(actual: Any, expected: Any, tolerance: Any = TOLERANCE) -> None:
    """Assert two vectors agree component-wise.

    Args:
        actual: Value under test.
        expected: Value it should match.
        tolerance: Largest permitted difference per component.
    """
    left = [float(value) for value in actual]
    right = [float(value) for value in expected]
    assert len(left) == len(right), f"length {len(left)} != {len(right)}"
    worst = max(abs(a - b) for a, b in zip(left, right))
    assert worst <= tolerance, f"{left} != {right} (worst {worst})"


def _drive(sensor: Any, states: Any, time_step: Any = 1.0) -> Any:
    """Push states one time step apart and read.

    Args:
        sensor: Sensor to drive.
        states: Body states, oldest first.
        time_step: Interval between samples, in seconds.

    Returns:
        The reading after the last sample.
    """
    for step, state in enumerate(states):
        sensor.sample(step * time_step, state)
    return sensor.read()


def _random_quaternions(count: Any, seed: Any) -> Any:
    """Generate unit quaternions from a seeded generator.

    Args:
        count: Number of quaternions.
        seed: Seed for the generator.

    Returns:
        Unit quaternions, ordered ``xyzw``.
    """
    generator = random.Random(seed)
    return [wp.normalize(wp.quatd([generator.gauss(0.0, 1.0) for _ in range(4)])) for _ in range(count)]


# ----------------------------------------------------------------------------------------------
# Quaternion and vector arithmetic.
# ----------------------------------------------------------------------------------------------


def test_rotation_matches_an_independent_matrix_formulation() -> None:
    """Quaternion rotation agrees with the matrix form."""
    generator = random.Random(1)
    for quaternion in _random_quaternions(200, seed=0):
        vector = wp.vec3d([generator.gauss(0.0, 1.0) for _ in range(3)])
        _assert_close(wp.quat_rotate(quaternion, vector), _rotate_by_matrix(quaternion, vector), 1.0e-12)


def test_composition_applies_the_right_operand_first() -> None:
    """``wp.mul(a, b)`` rotates by ``b``, then ``a``."""
    probe = wp.vec3d(1.0, 2.0, 3.0)
    composed = wp.mul(QUARTER_TURN_ABOUT_Z, QUARTER_TURN_ABOUT_X)
    _assert_close(
        wp.quat_rotate(composed, probe),
        wp.quat_rotate(QUARTER_TURN_ABOUT_Z, wp.quat_rotate(QUARTER_TURN_ABOUT_X, probe)),
    )
    # Composition does not commute, so the reverse must differ.
    reversed_product = wp.mul(QUARTER_TURN_ABOUT_X, QUARTER_TURN_ABOUT_Z)
    difference = wp.quat_rotate(composed, probe) - wp.quat_rotate(reversed_product, probe)
    assert float(wp.length(difference)) > 1.0e-6


def test_inversion_undoes_a_rotation() -> None:
    """A rotation followed by its inverse is the identity."""
    generator = random.Random(3)
    for quaternion in _random_quaternions(100, seed=2):
        vector = wp.vec3d([generator.gauss(0.0, 1.0) for _ in range(3)])
        restored = wp.quat_rotate(wp.quat_inverse(quaternion), wp.quat_rotate(quaternion, vector))
        _assert_close(restored, vector, 1.0e-12)


def test_normalization_handles_the_degenerate_case() -> None:
    """Warp normalizes a zero quaternion to the identity rather than dividing by zero."""
    _assert_close(wp.normalize(wp.quatd(0.0, 0.0, 0.0, 0.0)), imu.IDENTITY_QUATERNION)
    _assert_close(wp.normalize(wp.quatd(0.0, 0.0, 0.0, 2.0)), imu.IDENTITY_QUATERNION)


def test_averaging_an_alternating_window_does_not_cancel() -> None:
    """Sign-aligning first stops an alternating window cancelling."""
    flipped = -QUARTER_TURN_ABOUT_Z
    window = [QUARTER_TURN_ABOUT_Z, flipped, QUARTER_TURN_ABOUT_Z, flipped]
    probe = wp.vec3d(1.0, 2.0, 3.0)
    _assert_close(
        wp.quat_rotate(imu.average_quaternions(window), probe),
        wp.quat_rotate(QUARTER_TURN_ABOUT_Z, probe),
    )


def test_averaging_an_empty_window_yields_identity() -> None:
    """An empty window returns the identity."""
    _assert_close(imu.average_quaternions([]), imu.IDENTITY_QUATERNION)


# ----------------------------------------------------------------------------------------------
# Sensor lifecycle and argument validation.
# ----------------------------------------------------------------------------------------------


def test_read_returns_none_until_the_window_is_full() -> None:
    """A reading is undefined until the window is full."""
    for filter_width in (1, 2, 5):
        sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=filter_width)
        for step in range(2 * filter_width):
            assert sensor.read() is None
            sensor.sample(step, BodyState())
        assert sensor.read() is not None


def test_the_buffer_saturates_instead_of_growing() -> None:
    """Only ``2 * filter_width`` samples are retained."""
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=2)
    for step in range(20):
        sensor.sample(step, BodyState())
    assert sensor.sample_count == 4


def test_reset_discards_buffered_samples() -> None:
    """Resetting makes readings undefined, so a discontinuity cannot leak into a difference."""
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=1)
    _drive(sensor, [BodyState(), BodyState()])
    assert sensor.read() is not None
    sensor.reset()
    assert sensor.sample_count == 0
    assert sensor.read() is None


def test_a_filter_width_below_one_is_rejected() -> None:
    """A window narrower than one sample has no meaning."""
    for filter_width in (0, -1):
        try:
            ImuSensor([0.0, 0.0, 0.0], filter_width=filter_width)
        except ValueError:
            continue
        raise AssertionError(f"filter_width={filter_width} was not rejected")


def test_gravity_is_required() -> None:
    """There is no default gravity; it depends on the stage."""
    try:
        ImuSensor(filter_width=1)
    except TypeError:
        return
    raise AssertionError("gravity was not required")


def test_wrong_component_counts_are_rejected() -> None:
    """Vectors take three components and quaternions take four."""
    cases = [
        {"gravity": (0.0, 0.0)},
        {"gravity": (0.0, 0.0, 0.0), "mount_translation": (0.0, 0.0, 0.0, 0.0)},
        {"gravity": (0.0, 0.0, 0.0), "mount_orientation": (0.0, 0.0, 1.0)},
    ]
    for case in cases:
        arguments = dict(case)
        gravity = arguments.pop("gravity")
        try:
            ImuSensor(gravity, **arguments)
        except ValueError:
            continue
        raise AssertionError(f"{case} was not rejected")


def test_a_degenerate_mount_orientation_falls_back_to_identity() -> None:
    """A zero mount quaternion normalizes to the identity, not NaNs."""
    sensor = ImuSensor([0.0, 0.0, 0.0], mount_orientation=[0.0, 0.0, 0.0, 0.0], filter_width=1)
    reading = _drive(sensor, [BodyState(), BodyState()])
    assert reading is not None
    _assert_close(reading.orientation, imu.IDENTITY_QUATERNION)


# ----------------------------------------------------------------------------------------------
# Pose composition and the lever arm.
# ----------------------------------------------------------------------------------------------


def test_the_sensor_orientation_composes_the_body_rotation_after_the_mount() -> None:
    """The sensor pose is body composed with mount, in that order."""
    mount = QUARTER_TURN_ABOUT_X
    body = QUARTER_TURN_ABOUT_Z
    sensor = ImuSensor([0.0, 0.0, 0.0], mount_orientation=mount, filter_width=1)
    state = BodyState(orientation=body)
    reading = _drive(sensor, [state, state])
    assert reading is not None

    probe = wp.vec3d(1.0, 2.0, 3.0)
    # These do not commute, so the wrong order differs.
    _assert_close(
        wp.quat_rotate(reading.orientation, probe),
        _rotate_by_matrix(body, _rotate_by_matrix(mount, probe)),
    )


# ----------------------------------------------------------------------------------------------
# Acceleration, filtering, and gravity.
# ----------------------------------------------------------------------------------------------


def test_constant_velocity_reads_zero_acceleration() -> None:
    """Coordinate acceleration is zero while the state does not change."""
    generator = random.Random(6)
    for orientation in _random_quaternions(25, seed=5):
        sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=2)
        state = BodyState(
            orientation=orientation, linear_velocity=wp.vec3d([generator.gauss(0.0, 1.0) for _ in range(3)])
        )
        reading = _drive(sensor, [state] * 4)
        assert reading is not None
        _assert_close(reading.linear_acceleration, [0.0, 0.0, 0.0])


def test_a_resting_body_reads_one_gravity_along_world_up() -> None:
    """An accelerometer at rest measures the reaction to gravity."""
    gravity = wp.vec3d(0.0, 0.0, -9.81)
    for orientation in _random_quaternions(25, seed=7):
        sensor = ImuSensor(gravity, filter_width=1)
        state = BodyState(orientation=orientation)
        reading = _drive(sensor, [state, state])
        assert reading is not None
        # Back in the world it must be -g, whatever the pose.
        world = _rotate_by_matrix(reading.orientation, reading.linear_acceleration)
        _assert_close(world, [-value for value in gravity], 1.0e-8)


def test_free_fall_reads_zero_proper_acceleration() -> None:
    """A body accelerating at g measures no specific force."""
    gravity = wp.vec3d(0.0, 0.0, -9.81)
    sensor = ImuSensor(gravity, filter_width=1)
    states = [BodyState(linear_velocity=[float(value) * step for value in gravity]) for step in range(2)]
    reading = _drive(sensor, states)
    assert reading is not None
    _assert_close(reading.linear_acceleration, [0.0, 0.0, 0.0], 1.0e-8)


def test_a_filter_width_of_one_is_the_raw_single_step_difference() -> None:
    """Unfiltered, the acceleration is the last two samples differenced."""
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=1)
    states = [BodyState(linear_velocity=[0.0, 0.0, 0.0]), BodyState(linear_velocity=[4.0, 0.0, 0.0])]
    reading = _drive(sensor, states, time_step=0.5)
    assert reading is not None
    _assert_close(reading.linear_acceleration, [8.0, 0.0, 0.0])


def test_a_constant_acceleration_survives_filtering() -> None:
    """Averaging equal differences returns that difference."""
    rate = 2.5
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=3)
    states = [BodyState(linear_velocity=[rate * step, 0.0, 0.0]) for step in range(6)]
    reading = _drive(sensor, states)
    assert reading is not None
    _assert_close(reading.linear_acceleration, [rate, 0.0, 0.0])


def test_a_sensor_circling_with_the_body_reports_centripetal_acceleration() -> None:
    """Require velocity to be differenced in the world frame.

    Differencing in the rotating frame cancels the transport term and reports zero instead of ``-w^2 r``.
    """
    rate = 2.0
    radius = 0.5
    time_step = 1.0e-4
    sensor = ImuSensor([0.0, 0.0, 0.0], mount_translation=[radius, 0.0, 0.0], filter_width=1)
    for step in range(2):
        angle = rate * step * time_step
        sensor.sample(
            step * time_step,
            BodyState(
                orientation=[0.0, 0.0, math.sin(0.5 * angle), math.cos(0.5 * angle)],
                angular_velocity=[0.0, 0.0, rate],
            ),
        )
    reading = sensor.read()
    assert reading is not None
    _assert_close(reading.linear_acceleration, [-rate * rate * radius, 0.0, 0.0], 1.0e-3)


def test_the_orientation_average_survives_an_alternating_window() -> None:
    """Sign flips between samples must not cancel the mean."""
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=2)
    flipped = -QUARTER_TURN_ABOUT_Z
    states = [BodyState(orientation=orientation) for orientation in (QUARTER_TURN_ABOUT_Z, flipped) * 2]
    reading = _drive(sensor, states)
    assert reading is not None
    probe = wp.vec3d(1.0, 2.0, 3.0)
    _assert_close(
        wp.quat_rotate(reading.orientation, probe),
        _rotate_by_matrix(QUARTER_TURN_ABOUT_Z, probe),
        1.0e-9,
    )


# ----------------------------------------------------------------------------------------------
# Timestamp handling.
# ----------------------------------------------------------------------------------------------


def test_repeated_timestamps_do_not_divide_by_zero() -> None:
    """A zero interval contributes nothing rather than an infinity."""
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=1)
    sensor.sample(1.0, BodyState())
    sensor.sample(1.0, BodyState(linear_velocity=[5.0, 0.0, 0.0]))
    reading = sensor.read()
    assert reading is not None
    _assert_close(reading.linear_acceleration, [0.0, 0.0, 0.0])


def test_a_stalled_timestamp_does_not_bias_the_average_toward_zero() -> None:
    """Leave unusable differences out of the mean instead of counting them as zero.

    A rewound final timestamp exposes this behavior by making one of the two pairs unusable.
    """
    rate = 3.0
    sensor = ImuSensor([0.0, 0.0, 0.0], filter_width=2)
    for time in (0.0, 1.0, 2.0, 1.0):
        sensor.sample(time, BodyState(linear_velocity=[rate * time, 0.0, 0.0]))
    reading = sensor.read()
    assert reading is not None
    _assert_close(reading.linear_acceleration, [rate, 0.0, 0.0])


def run_all() -> Any:
    """Run every test in this module and report the outcome.

    Returns:
        The number of failures.

    """
    tests = sorted((name, value) for name, value in globals().items() if name.startswith("test_") and callable(value))
    failures = []
    for name, test in tests:
        try:
            test()
        except Exception as error:  # noqa: BLE001 - report every failure kind.
            failures.append((name, error))
            print(f"  FAIL {name}: {error}")
    passed = len(tests) - len(failures)
    if failures:
        # A distinct line, so the example's stdout assertion cannot match a partial pass.
        print(f"Self-test: {passed}/{len(tests)} imu.py unit tests passed, {len(failures)} failed.")
    else:
        print(f"Self-test: all {len(tests)} imu.py unit tests passed.")
    return len(failures)


if __name__ == "__main__":
    raise SystemExit(1 if run_all() else 0)
