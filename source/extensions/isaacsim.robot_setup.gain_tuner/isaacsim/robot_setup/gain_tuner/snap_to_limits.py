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

"""Snap-to-limits test: commands each joint to its lower limit, holds, then upper limit, holds, then returns home."""

from collections.abc import Generator
from enum import IntEnum

import numpy as np
from isaacsim.core.experimental.prims import Articulation

from .base import RobotTest, TestResult

_VELOCITY_THRESHOLD = 0.01
_APPROACH_TIMEOUT = 10.0
_SETTLE_WINDOW = 0.25


class _Phase(IntEnum):
    """Internal phases of the snap-to-limits test for a single sequence."""

    APPROACH_LOWER = 0
    HOLD_LOWER = 1
    APPROACH_UPPER = 2
    HOLD_UPPER = 3
    RETURN_HOME = 4
    HOLD_HOME = 5


_APPROACH_PHASES = {_Phase.APPROACH_LOWER, _Phase.APPROACH_UPPER, _Phase.RETURN_HOME}
_HOLD_PHASES = {_Phase.HOLD_LOWER, _Phase.HOLD_UPPER, _Phase.HOLD_HOME}


class SnapToLimitsTest(RobotTest):
    """Test that commands joints to their lower and upper limits.

    For each test sequence the joints are commanded to the lower limit.
    Per-joint tracking detects when each joint has stabilized
    (velocity below ``_VELOCITY_THRESHOLD`` for ``_SETTLE_WINDOW``).
    The approach phase advances as soon as every joint has
    stabilized or ``_APPROACH_TIMEOUT`` (10 s) expires — a blocked
    joint does **not** force the full timeout.

    The **blocked vs fail** distinction is made during the *hold*
    phase, not during approach.  During the hold, position errors
    are sampled every physics step.  A joint is classified as
    **blocked** only when *both* conditions hold: (1) the error
    did not decrease over the hold (first vs last sample) *and*
    (2) the error standard deviation is below tolerance (the joint
    is not oscillating).  A joint that shows no net progress but
    has high error variance is *oscillating* due to poor gains,
    not physically stuck — it is classified as **fail**.  A joint
    whose error did decrease is still approaching and is also
    **fail**.  This avoids misclassifying an oscillating or
    overdamped joint as blocked.

    It then holds for ``hold_duration`` seconds and records the
    mean/max position error over the hold.  The same cycle repeats
    for the upper limit and finally for the home (default) position.

    Each joint receives a three-state classification:
        - **pass**: both limits reached within tolerance.
        - **blocked**: a limit was not reached but the joint was
          stalled (velocity near zero) — typically caused by
          self-collisions or other physical constraints.
        - **fail**: a limit was not reached and the joint was still
          moving — indicating a gains or dynamics issue.

    The total test time is variable: fast robots settle quickly and the
    test finishes in seconds; slow or weak drives consume the full
    timeout per phase.

    Configurable via ``test_params``:
        hold_duration (float): Seconds to hold at each target **after**
            the joints have settled. Default 1.0.
        tolerance (float): Position error threshold (rad or m) used
            both for the settling check during approach and for the
            pass/fail determination at the end of each hold.
            Default 0.01.
        disable_velocity_limits (bool): When True, joint max velocity
            limits are set to a very large value for the test duration
            and restored afterward. This isolates whether failures are
            a gains issue vs. a velocity-limit issue. Default False.
    """

    name = "Snap to Limits"

    def __init__(self) -> None:
        super().__init__()
        self._hold_duration = 1.0
        self._tolerance = 0.01
        self._disable_velocity_limits = False
        self._original_max_velocities: np.ndarray | None = None
        self._joint_indices: list[int] = []
        self._joint_modes: dict[int, int] = {}
        self._test_params: dict = {}
        self._home_positions = None

    def setup(
        self, articulation: Articulation, joint_indices: list[int], joint_modes: dict[int, int], test_params: dict
    ) -> None:
        """Prepare the snap-to-limits test.

        Args:
            articulation: The Articulation wrapper for the robot under test.
            joint_indices: DOF indices being tested.
            joint_modes: Mapping of dof_index to JointMode value.
            test_params: Dict with optional keys listed in the class docstring.
        """
        super().setup(articulation, joint_indices, joint_modes, test_params)
        self._joint_indices = list(joint_indices)
        self._joint_modes = dict(joint_modes)
        self._test_params = test_params
        self._hold_duration = float(test_params.get("hold_duration", 1.0))
        self._tolerance = float(test_params.get("tolerance", 0.01))
        self._disable_velocity_limits = bool(test_params.get("disable_velocity_limits", False))
        self._original_max_velocities = None

    # ------------------------------------------------------------------

    def run(self) -> Generator[None, None, TestResult]:
        """Generator that executes the snap-to-limits test.

        Yields once per physics step.  On completion, returns a
        :class:`TestResult` with recorded trajectories and per-joint
        pass/fail metrics.
        """
        articulation = self._articulation
        if articulation is None:
            return TestResult(
                joint_position_commands=np.empty((0, 0)),
                joint_velocity_commands=np.empty((0, 0)),
                observed_joint_positions=np.empty((0, 0)),
                observed_joint_velocities=np.empty((0, 0)),
                command_times=np.empty(0),
            )

        self._disable_max_velocities(articulation)

        pos_cmd_list: list[np.ndarray] = []
        vel_cmd_list: list[np.ndarray] = []
        obs_pos_list: list[np.ndarray] = []
        obs_vel_list: list[np.ndarray] = []
        time_list: list[float] = []

        self._home_positions = articulation.get_dof_positions().numpy()[0].copy()

        lower_limits_all, upper_limits_all = [np.array(lim.list()) for lim in articulation.get_dof_limits()]

        sequences = self._test_params.get("sequence", [{}])
        total_time = 0.0

        joint_metrics: dict[int, dict] = {}
        for dof_idx in self._joint_indices:
            joint_metrics[dof_idx] = {
                "lower_limit": float(lower_limits_all[dof_idx]),
                "upper_limit": float(upper_limits_all[dof_idx]),
                "lower_position_error": float("inf"),
                "upper_position_error": float("inf"),
                "lower_max_error": float("inf"),
                "upper_max_error": float("inf"),
                "lower_settled": False,
                "upper_settled": False,
                "lower_reached": False,
                "upper_reached": False,
                "lower_blocked": False,
                "upper_blocked": False,
                "lower_settling_time": float("nan"),
                "upper_settling_time": float("nan"),
                "status": "fail",
            }

        for seq in sequences:
            seq_joint_indices = seq.get("joint_indices", np.array(self._joint_indices, dtype=np.int32))
            if len(seq_joint_indices) == 0:
                continue

            pos_dof_idx = [int(i) for i in seq_joint_indices if self._joint_modes.get(int(i), 2) == 0]
            vel_dof_idx = [int(i) for i in seq_joint_indices if self._joint_modes.get(int(i), 2) == 1]

            lower_limits = lower_limits_all[pos_dof_idx] if pos_dof_idx else np.array([])
            upper_limits = upper_limits_all[pos_dof_idx] if pos_dof_idx else np.array([])
            home_pos = self._home_positions[pos_dof_idx] if pos_dof_idx else np.array([])

            articulation.reset_to_default_state()

            phase_targets = {
                _Phase.APPROACH_LOWER: lower_limits,
                _Phase.HOLD_LOWER: lower_limits,
                _Phase.APPROACH_UPPER: upper_limits,
                _Phase.HOLD_UPPER: upper_limits,
                _Phase.RETURN_HOME: home_pos,
                _Phase.HOLD_HOME: home_pos,
            }

            current_phase = _Phase.APPROACH_LOWER
            phase_time = 0.0
            phase_start_time = total_time
            low_vel_times: dict[int, float] = dict.fromkeys(pos_dof_idx, 0.0)
            joint_done: dict[int, bool] = dict.fromkeys(pos_dof_idx, False)
            joint_at_target: dict[int, bool] = dict.fromkeys(pos_dof_idx, False)
            joint_settle_time: dict[int, float] = {idx: float("nan") for idx in pos_dof_idx}
            hold_errors: dict[int, list[float]] = {idx: [] for idx in pos_dof_idx}

            if pos_dof_idx:
                articulation.set_dof_position_targets(phase_targets[current_phase], dof_indices=pos_dof_idx)
            if vel_dof_idx:
                articulation.set_dof_velocity_targets(np.zeros(len(vel_dof_idx)), dof_indices=vel_dof_idx)

            position_targets = articulation.get_dof_position_targets().numpy()[0].copy()
            velocity_targets = articulation.get_dof_velocity_targets().numpy()[0].copy()

            self._record_sample(
                articulation,
                position_targets,
                velocity_targets,
                total_time,
                pos_cmd_list,
                vel_cmd_list,
                obs_pos_list,
                obs_vel_list,
                time_list,
            )
            yield

            while current_phase <= _Phase.HOLD_HOME:
                dt = self.step
                total_time += dt
                phase_time += dt

                advance = False
                approach_timed_out = False
                if current_phase in _APPROACH_PHASES:
                    target = phase_targets[current_phase]
                    self._update_per_joint_settle(
                        articulation,
                        pos_dof_idx,
                        target,
                        dt,
                        low_vel_times,
                        joint_done,
                        joint_at_target,
                        total_time,
                        phase_start_time,
                        joint_settle_time,
                    )
                    all_done = all(joint_done[idx] for idx in pos_dof_idx) if pos_dof_idx else True
                    approach_timed_out = not all_done and phase_time >= _APPROACH_TIMEOUT
                    advance = all_done or approach_timed_out
                else:
                    target = phase_targets[current_phase]
                    if pos_dof_idx:
                        observed = articulation.get_dof_positions().numpy()[0]
                        for i, dof_idx in enumerate(pos_dof_idx):
                            hold_errors[dof_idx].append(abs(float(observed[dof_idx] - target[i])))
                    advance = phase_time >= self._hold_duration

                if advance:
                    self._record_hold_metrics(
                        current_phase,
                        pos_dof_idx,
                        lower_limits,
                        upper_limits,
                        articulation,
                        joint_metrics,
                        hold_errors,
                        approach_timed_out,
                        joint_done,
                        joint_at_target,
                        joint_settle_time,
                    )
                    if current_phase >= _Phase.HOLD_HOME:
                        self._record_sample(
                            articulation,
                            position_targets,
                            velocity_targets,
                            total_time,
                            pos_cmd_list,
                            vel_cmd_list,
                            obs_pos_list,
                            obs_vel_list,
                            time_list,
                        )
                        break

                    current_phase = _Phase(current_phase + 1)
                    phase_time = 0.0
                    phase_start_time = total_time
                    for idx in pos_dof_idx:
                        low_vel_times[idx] = 0.0
                        joint_done[idx] = False
                        joint_at_target[idx] = False
                        joint_settle_time[idx] = float("nan")
                    for errs in hold_errors.values():
                        errs.clear()

                    if pos_dof_idx:
                        articulation.set_dof_position_targets(phase_targets[current_phase], dof_indices=pos_dof_idx)
                    if vel_dof_idx:
                        articulation.set_dof_velocity_targets(np.zeros(len(vel_dof_idx)), dof_indices=vel_dof_idx)

                target = phase_targets.get(current_phase)
                if pos_dof_idx and target is not None:
                    position_targets[pos_dof_idx] = target

                pos_cmd_list.append(position_targets.copy())
                vel_cmd_list.append(velocity_targets.copy())
                time_list.append(total_time)

                yield

                obs_pos_list.append(articulation.get_dof_positions().numpy()[0].copy())
                obs_vel_list.append(articulation.get_dof_velocities().numpy()[0].copy())

        articulation.reset_to_default_state()
        self._restore_velocity_limits()

        for m in joint_metrics.values():
            lower_ok = m["lower_reached"]
            upper_ok = m["upper_reached"]
            if lower_ok and upper_ok:
                m["status"] = "pass"
            elif (not lower_ok and not m["lower_blocked"]) or (not upper_ok and not m["upper_blocked"]):
                m["status"] = "fail"
            else:
                m["status"] = "blocked"

        return TestResult(
            joint_position_commands=np.array(pos_cmd_list),
            joint_velocity_commands=np.array(vel_cmd_list),
            observed_joint_positions=np.array(obs_pos_list),
            observed_joint_velocities=np.array(obs_vel_list),
            command_times=np.array(time_list),
            joint_metrics=joint_metrics,
        )

    def stop(self) -> None:
        """Cancel the test and restore velocity limits if overridden."""
        self._restore_velocity_limits()
        super().stop()

    # ------------------------------------------------------------------

    def _disable_max_velocities(self, articulation: Articulation) -> None:
        """Save current max velocities and set them to a very large value."""
        if not self._disable_velocity_limits:
            return
        self._original_max_velocities = articulation.get_dof_max_velocities().numpy()[0].copy()
        articulation.set_dof_max_velocities(np.full_like(self._original_max_velocities, 1e6))

    def _restore_velocity_limits(self) -> None:
        """Restore original max velocity values."""
        if self._original_max_velocities is None or self._articulation is None:
            return
        self._articulation.set_dof_max_velocities(self._original_max_velocities)
        self._original_max_velocities = None

    # ------------------------------------------------------------------

    @staticmethod
    def _record_sample(
        articulation: Articulation,
        position_targets: np.ndarray,
        velocity_targets: np.ndarray,
        time: float,
        pos_cmd_list: list,
        vel_cmd_list: list,
        obs_pos_list: list,
        obs_vel_list: list,
        time_list: list,
    ) -> None:
        """Append one data sample to the recording lists."""
        pos_cmd_list.append(position_targets.copy())
        vel_cmd_list.append(velocity_targets.copy())
        obs_pos_list.append(articulation.get_dof_positions().numpy()[0].copy())
        obs_vel_list.append(articulation.get_dof_velocities().numpy()[0].copy())
        time_list.append(time)

    def _update_per_joint_settle(
        self,
        articulation: Articulation,
        pos_dof_idx: list[int],
        target: np.ndarray,
        dt: float,
        low_vel_times: dict[int, float],
        joint_done: dict[int, bool],
        joint_at_target: dict[int, bool],
        total_time: float = 0.0,
        phase_start_time: float = 0.0,
        joint_settle_time: dict[int, float] | None = None,
    ) -> None:
        """Update per-joint stabilization tracking.

        Each joint independently accumulates time with velocity below
        ``_VELOCITY_THRESHOLD``.  Once a joint's low-velocity duration
        reaches ``_SETTLE_WINDOW`` it is latched as *done*.  At the
        latch instant, position is checked against ``tolerance`` to
        record whether the joint is at the target.

        This method only determines *when* a joint has stopped moving
        so the approach phase can advance.  The **blocked vs fail**
        classification is deferred to the hold phase, where a longer
        observation window reliably distinguishes a physically stuck
        joint from an extremely overdamped one still creeping.

        When ``joint_settle_time`` is provided, the elapsed time from
        ``phase_start_time`` to the latch instant is recorded for each
        joint that settles at the target.

        Args:
            articulation: The articulation being tested.
            pos_dof_idx: Position-mode DOF indices.
            target: Target positions for those DOFs.
            dt: Physics time step.
            low_vel_times: Per-joint continuous low-velocity time (mutated).
            joint_done: Per-joint latched "done" flag (mutated).
            joint_at_target: Per-joint "at target when latched" flag (mutated).
            total_time: Current simulation time.
            phase_start_time: Sim time when the current phase command was issued.
            joint_settle_time: Per-joint settling time in seconds (mutated).
        """
        if not pos_dof_idx:
            return
        positions = articulation.get_dof_positions().numpy()[0]
        velocities = articulation.get_dof_velocities().numpy()[0]
        for i, dof_idx in enumerate(pos_dof_idx):
            if joint_done[dof_idx]:
                continue
            if abs(velocities[dof_idx]) <= _VELOCITY_THRESHOLD:
                low_vel_times[dof_idx] += dt
                if low_vel_times[dof_idx] >= _SETTLE_WINDOW:
                    joint_done[dof_idx] = True
                    at_target = abs(positions[dof_idx] - target[i]) <= self._tolerance
                    joint_at_target[dof_idx] = at_target
                    if at_target and joint_settle_time is not None:
                        joint_settle_time[dof_idx] = total_time - phase_start_time
            else:
                low_vel_times[dof_idx] = 0.0

    def _record_hold_metrics(
        self,
        phase: _Phase,
        pos_dof_idx: list[int],
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
        articulation: Articulation,
        joint_metrics: dict[int, dict],
        hold_errors: dict[int, list[float]],
        approach_timed_out: bool = False,
        joint_done: dict[int, bool] | None = None,
        joint_at_target: dict[int, bool] | None = None,
        joint_settle_time: dict[int, float] | None = None,
    ) -> None:
        """Compute error statistics and blocked/fail classification.

        For **hold phases** the mean and max absolute errors are
        derived from every sample accumulated during the phase.  If
        a joint did not reach its target (mean error >= tolerance),
        two checks determine whether it is **blocked** or **fail**:
        (1) no net progress (first sample ≈ last sample), and
        (2) low error variance (not oscillating).  Only when both
        hold is the joint classified as **blocked** (physically
        stuck).  A joint with high error variance is oscillating
        due to poor gains and is classified as **fail**.

        For **approach phases**, records whether each joint reached
        the target at the moment it stabilized, plus settling time.

        Args:
            phase: The phase that just completed.
            pos_dof_idx: Position-mode DOF indices for this sequence.
            lower_limits: Lower limits for the position DOFs.
            upper_limits: Upper limits for the position DOFs.
            articulation: The articulation being tested.
            joint_metrics: Metrics dict to update in place.
            hold_errors: Per-DOF lists of absolute errors sampled every
                step during the hold phase.
            approach_timed_out: True if the approach phase ended because
                ``_APPROACH_TIMEOUT`` expired rather than settling.
            joint_done: Per-joint latched "stopped moving" flags.
            joint_at_target: Per-joint "was at target when latched" flags.
            joint_settle_time: Per-joint settling time in seconds.
        """
        if not pos_dof_idx:
            return

        if phase in (_Phase.HOLD_LOWER, _Phase.HOLD_UPPER):
            err_key = "lower_position_error" if phase == _Phase.HOLD_LOWER else "upper_position_error"
            max_key = "lower_max_error" if phase == _Phase.HOLD_LOWER else "upper_max_error"
            reached_key = "lower_reached" if phase == _Phase.HOLD_LOWER else "upper_reached"
            blocked_key = "lower_blocked" if phase == _Phase.HOLD_LOWER else "upper_blocked"

            for i, dof_idx in enumerate(pos_dof_idx):
                errs = hold_errors.get(dof_idx, [])
                mean_err = float(np.mean(errs)) if errs else float("inf")
                max_err = float(np.max(errs)) if errs else float("inf")
                joint_metrics[dof_idx][err_key] = mean_err
                joint_metrics[dof_idx][max_key] = max_err
                reached = mean_err < self._tolerance
                joint_metrics[dof_idx][reached_key] = reached
                if not reached and len(errs) >= 2:
                    progress = errs[0] - errs[-1]
                    no_progress = progress <= self._tolerance * 0.1
                    oscillating = float(np.std(errs)) > self._tolerance
                    joint_metrics[dof_idx][blocked_key] = no_progress and not oscillating
        elif phase in (_Phase.APPROACH_LOWER, _Phase.APPROACH_UPPER, _Phase.RETURN_HOME):
            if joint_at_target is None:
                joint_at_target = {}
            if joint_settle_time is None:
                joint_settle_time = {}

            settled_key = (
                "lower_settled"
                if phase == _Phase.APPROACH_LOWER
                else ("upper_settled" if phase == _Phase.APPROACH_UPPER else None)
            )
            settle_time_key = (
                "lower_settling_time"
                if phase == _Phase.APPROACH_LOWER
                else ("upper_settling_time" if phase == _Phase.APPROACH_UPPER else None)
            )

            for i, dof_idx in enumerate(pos_dof_idx):
                if settled_key:
                    joint_metrics[dof_idx][settled_key] = joint_at_target.get(dof_idx, False)
                if settle_time_key:
                    joint_metrics[dof_idx][settle_time_key] = joint_settle_time.get(dof_idx, float("nan"))
