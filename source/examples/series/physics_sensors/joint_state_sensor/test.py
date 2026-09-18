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

"""Unit tests for `joint_state`.

Synthetic states expose mistakes that the deliberately simple scene in ``main.py`` cannot.
"""

from __future__ import annotations

from typing import Any

import warp as wp
from joint_state import ArticulationState, Dof, DofType, JointStateSensor

TOLERANCE = 1.0e-12

LIFT = Dof("lift", DofType.TRANSLATION)
SPIN = Dof("spin", DofType.ROTATION)
BOTH = [LIFT, SPIN]


def _assert_close(actual: Any, expected: Any, tolerance: Any = TOLERANCE) -> None:
    """Assert two sequences agree element-wise.

    Args:
        actual: Value under test.
        expected: Value it should match.
        tolerance: Largest permitted difference per element.
    """
    left = [float(value) for value in actual]
    right = [float(value) for value in expected]
    assert len(left) == len(right), f"length {len(left)} != {len(right)}"
    worst = max((abs(a - b) for a, b in zip(left, right)), default=0.0)
    assert worst <= tolerance, f"{left} != {right} (worst {worst})"


def _assert_raises(error_type: Any, call: Any, description: Any) -> None:
    """Assert a call raises a given error.

    Args:
        error_type: Expected exception type.
        call: Zero-argument callable under test.
        description: What was expected to be rejected, for the failure message.
    """
    try:
        call()
    except error_type:
        return
    raise AssertionError(f"{description} was not rejected")


def _array(values: Any) -> Any:
    """Build the per-DOF Warp array an entity's DOF getters would return.

    Args:
        values: Raw per-DOF values.

    Returns:
        The array, or ``None`` when given ``None``.
    """
    return None if values is None else wp.array(values, dtype=wp.float32, device="cpu")


def _state(positions: Any, velocities: Any, efforts: Any = None) -> Any:
    """Build a raw articulation state.

    Args:
        positions: Raw DOF positions.
        velocities: Raw DOF velocities.
        efforts: Raw measured efforts, or ``None`` when the backend reports none.

    Returns:
        The state.
    """
    return ArticulationState(positions=_array(positions), velocities=_array(velocities), efforts=_array(efforts))


def _drive(sensor: Any, states: Any, time_step: Any = 1.0) -> Any:
    """Read the last of a series of states, stamped one time step apart.

    Args:
        sensor: Sensor to read through.
        states: Raw articulation states, oldest first.
        time_step: Interval between states, in seconds.

    Returns:
        The reading of the last state.
    """
    return sensor.read((len(states) - 1) * time_step, states[-1])


def _sensor(dofs: Any = None) -> Any:
    """Build a sensor over both DOFs, or over the ones given.

    Args:
        dofs: DOFs to report. ``None`` reports both of the fixtures.

    Returns:
        The sensor.
    """
    return JointStateSensor(BOTH if dofs is None else dofs)


# ----------------------------------------------------------------------------------------------
# DOF metadata.
# ----------------------------------------------------------------------------------------------


def test_a_dof_type_the_module_does_not_define_is_rejected() -> None:
    """A DOF is a rotation or a translation, and nothing else."""
    for value in (2, -1, "rotation", None):
        _assert_raises(ValueError, lambda value=value: Dof("elbow", value), f"the DOF type {value!r}")


def test_a_dof_type_is_the_code_the_shipped_reading_carries() -> None:
    """`dof_types` is ``uint8`` on the C++ reading, so a code converts without a lookup table."""
    assert DofType(0) is DofType.ROTATION
    assert DofType(1) is DofType.TRANSLATION
    assert Dof("elbow", 0).type is DofType.ROTATION
    # The codes compare equal to their numbers, so a consumer can publish them as they are.
    assert DofType.TRANSLATION == 1


def test_a_dof_type_can_be_looked_up_by_the_name_usd_uses() -> None:
    """USD reports the same distinction as ``"rotation"`` and ``"translation"``."""
    assert DofType["ROTATION".upper()] is DofType.ROTATION
    assert DofType["translation".upper()] is DofType.TRANSLATION


def test_an_articulation_with_no_dofs_is_rejected() -> None:
    """There is no joint state to report for an articulation without DOFs."""
    _assert_raises(ValueError, lambda: _sensor([]), "an empty DOF list")


# ----------------------------------------------------------------------------------------------
# Sensor lifecycle and argument validation.
# ----------------------------------------------------------------------------------------------


def test_a_state_that_does_not_cover_every_dof_is_rejected() -> None:
    """A reading reports every DOF, so a state that covers fewer would misalign the names."""
    sensor = _sensor()
    _assert_raises(ValueError, lambda: sensor.read(0.0, _state([0.0], [0.0, 0.0])), "short positions")
    _assert_raises(ValueError, lambda: sensor.read(0.0, _state([0.0, 0.0], [0.0])), "short velocities")
    _assert_raises(ValueError, lambda: sensor.read(0.0, _state([0.0, 0.0], [0.0, 0.0], [0.0])), "short efforts")
    _assert_raises(ValueError, lambda: sensor.read(0.0, _state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])), "long arrays")


# ----------------------------------------------------------------------------------------------
# Efforts.
# ----------------------------------------------------------------------------------------------


def test_a_backend_without_efforts_reports_zeros() -> None:
    """A backend that measures no effort reports zeros, which is what the shipped sensor does."""
    reading = _drive(_sensor(), [_state([1.0, 1.0], [2.0, 2.0])])
    _assert_close(reading.efforts, [0.0, 0.0])
    # Nothing else is affected.
    _assert_close(reading.velocities, [2.0, 2.0])


def test_a_reading_carries_the_stage_linear_unit() -> None:
    """The shipped reading carries `stage_meters_per_unit`, and a meter stage makes it one."""
    reading = _drive(_sensor(), [_state([1.0, 1.0], [2.0, 2.0])])
    assert reading.stage_meters_per_unit == 1.0
    assert reading.is_valid is True


# ----------------------------------------------------------------------------------------------
# Reported values.
# ----------------------------------------------------------------------------------------------


def test_a_reading_comes_from_the_state_it_was_given() -> None:
    """The shipped sensor reports each step untouched, so nothing here is averaged."""
    states = [_state([100.0 * step, 0.0], [0.0, 10.0 * step]) for step in range(4)]
    reading = _drive(_sensor(), states)
    assert reading is not None
    _assert_close([reading.positions[0]], [300.0])
    _assert_close([reading.velocities[1]], [30.0])


def test_an_effort_comes_from_that_state_too() -> None:
    """Efforts are reported the same way velocities are."""
    states = [_state([0.0], [0.0], [4.0 * step]) for step in range(4)]
    reading = _drive(JointStateSensor([SPIN]), states)
    assert reading is not None
    _assert_close([reading.efforts[0]], [12.0])


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
        print(f"Self-test: {passed}/{len(tests)} joint_state.py unit tests passed, {len(failures)} failed.")
    else:
        print(f"Self-test: all {len(tests)} joint_state.py unit tests passed.")
    return len(failures)


if __name__ == "__main__":
    raise SystemExit(1 if run_all() else 0)
