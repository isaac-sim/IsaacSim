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

"""Base class and result type for robot articulation tests used by the Gains Tuner."""

from collections.abc import Generator
from dataclasses import dataclass, field

import numpy as np
from isaacsim.core.experimental.prims import Articulation


@dataclass
class TestResult:
    """Structured result from a robot test run.

    Contains per-joint time-series data compatible with the Gains Tuner
    plotting pipeline, plus optional per-joint metrics for test-specific
    analysis (e.g. pass/fail, position error at hold).

    Args:
        joint_position_commands: Commanded positions, shape ``(T, num_dofs)``.
        joint_velocity_commands: Commanded velocities, shape ``(T, num_dofs)``.
        observed_joint_positions: Observed positions, shape ``(T, num_dofs)``.
        observed_joint_velocities: Observed velocities, shape ``(T, num_dofs)``.
        command_times: Timestamps for each sample, shape ``(T,)``.
        joint_metrics: Per-DOF metrics dict, keyed by DOF index.
    """

    joint_position_commands: np.ndarray
    joint_velocity_commands: np.ndarray
    observed_joint_positions: np.ndarray
    observed_joint_velocities: np.ndarray
    command_times: np.ndarray
    joint_metrics: dict[int, dict] = field(default_factory=dict)


class RobotTest:
    """Base class for robot articulation tests.

    Subclasses implement a generator-based test that yields once per
    physics step, compatible with the Gains Tuner's existing
    update-per-step architecture.

    The caller sets ``_step`` before each generator advance so the test
    can read the current physics time step.
    """

    name: str = "Unnamed Test"

    def __init__(self) -> None:
        self._articulation = None
        self._step = 0.0

    @property
    def step(self) -> float:
        """Current physics step size, set by the caller before each generator advance."""
        return self._step

    def setup(
        self, articulation: Articulation, joint_indices: list[int], joint_modes: dict[int, int], test_params: dict
    ) -> None:
        """Prepare the test with the target articulation and parameters.

        Called once before the test generator is created.

        Args:
            articulation: The Articulation wrapper for the robot under test.
            joint_indices: DOF indices being tested.
            joint_modes: Mapping of dof_index to JointMode value.
            test_params: Test-specific configuration dict.
        """
        self._articulation = articulation

    def run(self) -> Generator[None, None, "TestResult"]:
        """Generator that drives the test.

        Each ``yield`` returns control to the simulation loop for one
        physics step.  On completion the generator returns a
        :class:`TestResult` via ``return``.

        Returns:
            TestResult with all recorded data.
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Abort the test early and reset the articulation to its default state."""
        if self._articulation is not None:
            self._articulation.reset_to_default_state()
