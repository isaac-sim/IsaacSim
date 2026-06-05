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

"""Stress test: bombards joints with extreme random commands to surface PhysX solver instabilities."""

from collections.abc import Generator
from enum import IntEnum

import carb
import numpy as np
from isaacsim.core.experimental.prims import Articulation

from .base import RobotTest, TestResult


class StressTestMode(IntEnum):
    """Sub-modes for the stress test."""

    RANDOM_WALK = 0
    ADVERSARIAL = 1


class StressTest(RobotTest):
    """Stress test that surfaces PhysX solver instabilities.

    Two sub-modes are available:

    **Random Walk**: Every physics step, a Gaussian-distributed delta
    is added to each joint's current position target.  Commands are
    clamped to joint limits.  This simulates unconstrained neural-net
    exploration during early policy training.

    **Adversarial**: Every *N* physics steps, all active joints are
    simultaneously snapped to randomly chosen lower or upper limits
    (50/50 per joint).  This maximises worst-case solver load by
    driving extreme correlated configurations.

    Per-joint instability detection runs every step:

    - Velocity exceeding a configurable threshold -> UNSTABLE
    - NaN in position or velocity -> UNSTABLE
    - If neither occurs over the full duration -> STABLE

    The RNG seed is logged so destabilising runs can be reproduced.

    Configurable via ``test_params``:
        stress_test_mode (int): 0 = Random Walk, 1 = Adversarial.
            Default 0.
        duration (float): Sim-time seconds to run. Default 10.0.
        velocity_threshold (float): Abs velocity above which a joint
            is flagged UNSTABLE (rad/s or m/s). Default 100.0.
        sigma (float): Standard deviation of the per-step Gaussian
            delta in Random Walk mode, expressed as a fraction of
            each joint's range. Default 0.01 (1% of range).
        snap_interval (int): Physics steps between random snaps in
            Adversarial mode. Default 10.
        seed (int): RNG seed for reproducibility. Default 42.
    """

    name = "Stress Test"

    def __init__(self) -> None:
        super().__init__()
        self._mode = StressTestMode.RANDOM_WALK
        self._duration = 10.0
        self._velocity_threshold = 100.0
        self._sigma_frac = 0.01
        self._snap_interval = 10
        self._seed = 42
        self._rng: np.random.Generator | None = None
        self._joint_indices: list[int] = []
        self._joint_modes: dict[int, int] = {}
        self._test_params: dict = {}
        self._disable_velocity_limits = False
        self._original_max_velocities: np.ndarray | None = None

    def setup(
        self, articulation: Articulation, joint_indices: list[int], joint_modes: dict[int, int], test_params: dict
    ) -> None:
        """Prepare the stress test with the target articulation and parameters.

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
        self._mode = StressTestMode(int(test_params.get("stress_test_mode", 0)))
        self._duration = float(test_params.get("duration", 10.0))
        self._velocity_threshold = float(test_params.get("velocity_threshold", 100.0))
        self._sigma_frac = float(test_params.get("sigma", 0.01))
        self._snap_interval = max(1, int(test_params.get("snap_interval", 10)))
        self._seed = int(test_params.get("seed", 42))
        self._rng = np.random.default_rng(self._seed)
        self._disable_velocity_limits = bool(test_params.get("disable_velocity_limits", False))
        self._original_max_velocities = None

    # ------------------------------------------------------------------

    def run(self) -> Generator[None, None, TestResult]:
        """Generator that executes the stress test.

        Yields once per physics step. On completion, returns a
        :class:`TestResult` with recorded trajectories and per-joint
        stable/unstable metrics.
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

        pos_cmd_list: list[np.ndarray] = []
        vel_cmd_list: list[np.ndarray] = []
        obs_pos_list: list[np.ndarray] = []
        obs_vel_list: list[np.ndarray] = []
        time_list: list[float] = []

        lower_limits_all, upper_limits_all = [np.array(lim.list()) for lim in articulation.get_dof_limits()]

        self._disable_max_velocities(articulation)

        joint_metrics: dict[int, dict] = {}
        mode_str = "random_walk" if self._mode == StressTestMode.RANDOM_WALK else "adversarial"

        sequences = self._test_params.get("sequence", [{}])
        total_time = 0.0

        carb.log_info(
            f"Stress test: mode={mode_str}, duration={self._duration}s, "
            f"vel_threshold={self._velocity_threshold}, seed={self._seed}, "
            f"sequences={len(sequences)}"
        )

        for seq_idx, seq in enumerate(sequences):
            seq_joint_indices = seq.get("joint_indices", np.array(self._joint_indices, dtype=np.int32))
            if len(seq_joint_indices) == 0:
                continue

            pos_dof_idx = [int(i) for i in seq_joint_indices if self._joint_modes.get(int(i), 2) == 0]
            vel_dof_idx = [int(i) for i in seq_joint_indices if self._joint_modes.get(int(i), 2) == 1]

            if not pos_dof_idx:
                continue

            articulation.reset_to_default_state()

            lower_limits = lower_limits_all[pos_dof_idx]
            upper_limits = upper_limits_all[pos_dof_idx]
            joint_ranges = np.maximum(upper_limits - lower_limits, 1e-6)
            per_joint_sigma = joint_ranges * self._sigma_frac
            current_targets = articulation.get_dof_positions().numpy()[0][pos_dof_idx].copy()

            joint_unstable: dict[int, bool] = {}
            joint_max_vel: dict[int, float] = {}
            for dof_idx in pos_dof_idx:
                joint_unstable[dof_idx] = False
                joint_max_vel[dof_idx] = 0.0
                joint_metrics[dof_idx] = {
                    "status": "stable",
                    "trigger_time": float("nan"),
                    "trigger_velocity": float("nan"),
                    "trigger_command": float("nan"),
                    "max_velocity": 0.0,
                    "test_mode": mode_str,
                    "seed": self._seed,
                }

            if vel_dof_idx:
                articulation.set_dof_velocity_targets(np.zeros(len(vel_dof_idx)), dof_indices=vel_dof_idx)

            position_targets = articulation.get_dof_position_targets().numpy()[0].copy()
            velocity_targets = articulation.get_dof_velocity_targets().numpy()[0].copy()

            seq_time = 0.0
            step_counter = 0

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

            while seq_time < self._duration:
                dt = self.step
                total_time += dt
                seq_time += dt
                step_counter += 1

                if self._mode == StressTestMode.RANDOM_WALK:
                    deltas = self._rng.normal(0.0, per_joint_sigma)
                    current_targets += deltas
                    np.clip(current_targets, lower_limits, upper_limits, out=current_targets)
                    articulation.set_dof_position_targets(current_targets, dof_indices=pos_dof_idx)
                elif self._mode == StressTestMode.ADVERSARIAL:
                    if step_counter % self._snap_interval == 0:
                        choices = self._rng.random(size=len(pos_dof_idx))
                        current_targets = np.where(choices < 0.5, lower_limits, upper_limits)
                        articulation.set_dof_position_targets(current_targets, dof_indices=pos_dof_idx)

                position_targets[pos_dof_idx] = current_targets

                nan_break = False
                try:
                    positions = articulation.get_dof_positions().numpy()[0]
                    velocities = articulation.get_dof_velocities().numpy()[0]
                except Exception:
                    for dof_idx in pos_dof_idx:
                        if not joint_unstable[dof_idx]:
                            joint_unstable[dof_idx] = True
                            joint_metrics[dof_idx]["status"] = "unstable"
                            joint_metrics[dof_idx]["trigger_time"] = seq_time
                    nan_break = True

                if not nan_break:
                    for i, dof_idx in enumerate(pos_dof_idx):
                        if joint_unstable[dof_idx]:
                            continue
                        pos_val = float(positions[dof_idx])
                        vel_val = float(velocities[dof_idx])
                        abs_vel = abs(vel_val)

                        if abs_vel == abs_vel:
                            joint_max_vel[dof_idx] = max(joint_max_vel[dof_idx], abs_vel)

                        is_nan = (pos_val != pos_val) or (vel_val != vel_val)
                        is_over = abs_vel > self._velocity_threshold

                        if is_nan or is_over:
                            joint_unstable[dof_idx] = True
                            joint_metrics[dof_idx]["status"] = "unstable"
                            joint_metrics[dof_idx]["trigger_time"] = seq_time
                            joint_metrics[dof_idx]["trigger_velocity"] = vel_val
                            joint_metrics[dof_idx]["trigger_command"] = float(current_targets[i])

                for dof_idx in pos_dof_idx:
                    joint_metrics[dof_idx]["max_velocity"] = joint_max_vel[dof_idx]

                pos_cmd_list.append(position_targets.copy())
                vel_cmd_list.append(velocity_targets.copy())
                time_list.append(total_time)

                if not nan_break:
                    obs_pos_list.append(positions.copy())
                    obs_vel_list.append(velocities.copy())
                else:
                    obs_pos_list.append(np.full_like(position_targets, float("nan")))
                    obs_vel_list.append(np.full_like(velocity_targets, float("nan")))

                if nan_break or (pos_dof_idx and all(joint_unstable[idx] for idx in pos_dof_idx)):
                    break

                yield

        articulation.reset_to_default_state()
        self._restore_velocity_limits()

        return TestResult(
            joint_position_commands=np.array(pos_cmd_list) if pos_cmd_list else np.empty((0, 0)),
            joint_velocity_commands=np.array(vel_cmd_list) if vel_cmd_list else np.empty((0, 0)),
            observed_joint_positions=np.array(obs_pos_list) if obs_pos_list else np.empty((0, 0)),
            observed_joint_velocities=np.array(obs_vel_list) if obs_vel_list else np.empty((0, 0)),
            command_times=np.array(time_list) if time_list else np.empty(0),
            joint_metrics=joint_metrics,
        )

    def stop(self) -> None:
        """Cancel the test and restore velocity limits if overridden."""
        self._restore_velocity_limits()
        super().stop()

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
        pos_cmd_list.append(position_targets.copy())
        vel_cmd_list.append(velocity_targets.copy())
        obs_pos_list.append(articulation.get_dof_positions().numpy()[0].copy())
        obs_vel_list.append(articulation.get_dof_velocities().numpy()[0].copy())
        time_list.append(time)
