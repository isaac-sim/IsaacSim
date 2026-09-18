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

"""Measurement-driven phases for one physical Franka pick-and-place transfer."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from transitions import EventData, Machine

# (source, destination, guard, timeout_seconds)
_GRAPH = (
    ("approach_pick", "descend_pick", "_motion_complete", 20.0),
    ("descend_pick", "grasp", "_motion_complete", 10.0),
    ("grasp", "lift", "_grasp_complete", 6.0),
    ("lift", "approach_place", "_lift_complete", 10.0),
    ("approach_place", "descend_place", "_motion_complete", 20.0),
    ("descend_place", "release", "_motion_complete", 10.0),
    ("release", "retreat", "_release_complete", 6.0),
    ("retreat", "settle", "_motion_complete", 10.0),
    ("settle", "done", "_settle_complete", 8.0),
)
_PHASE_TIMEOUTS = {source: timeout for source, _, _, timeout in _GRAPH}
_PLACE_PHASES = {"approach_place", "descend_place", "release", "retreat", "settle", "done"}
_CLOSED_PHASES = {"grasp", "lift", "approach_place", "descend_place"}
_CARRY_PHASES = {"lift", "approach_place", "descend_place"}
# Distances use meters, angles radians, durations seconds, and speeds SI units.
_APPROACH_CLEARANCE = 0.20
_LIFT_CLEARANCE = 0.30
_POSITION_TOLERANCE = 0.010
_LIFT_POSITION_TOLERANCE = 0.035
_ORIENTATION_TOLERANCE = 0.20
_STABILITY_DWELL = 0.12
_GRIPPER_DWELL = 0.25
_LIFT_RISE = 0.08
_GRASP_DISTANCE = 0.040
_CARRY_DISTANCE = 0.080
_CARRY_LOSS_DWELL = 0.25
_FINGER_MIN_GRASP = 0.005
_FINGER_MAX_GRASP = 0.035
_FINGER_OPEN_POSITION = 0.035
_SETTLE_POSITION_TOLERANCE = 0.035
_SETTLE_LINEAR_SPEED = 0.05
_SETTLE_ANGULAR_SPEED = 0.50


@dataclass(frozen=True, slots=True)
class GuardInput:
    """Measurements used to decide whether the current phase is complete."""

    position_error: float
    orientation_error: float
    finger_positions: tuple[float, float]
    object_height: float
    grasp_distance: float
    placement_error: float
    object_linear_speed: float
    object_angular_speed: float


class PickPlaceStateMachine:
    """Advance one transfer through named, bounded phases."""

    def __init__(self) -> None:
        self.phase = "approach_pick"
        self._phase_started = self._last_time = 0.0
        self._stable_since: float | None = None
        self._carry_lost_since: float | None = None
        self._pick_position = (0.0, 0.0, 0.0)
        self._place_position = (0.0, 0.0, 0.0)
        self._pick_height = 0.0
        self._transfer_index = 0
        self._machine = Machine(
            model=self,
            states=(*_PHASE_TIMEOUTS, "done"),
            initial="approach_pick",
            model_attribute="phase",
            auto_transitions=False,
            send_event=True,
        )
        for source, target, guard, _ in _GRAPH:
            self._machine.add_transition("tick", source, target, conditions=guard, after="_enter_phase")

    @property
    def target_position(self) -> tuple[float, float, float]:
        """Return the current world-space gripper target."""
        position = self._place_position if self.phase in _PLACE_PHASES else self._pick_position
        if self.phase.startswith("approach"):
            height = _APPROACH_CLEARANCE
        else:
            height = _LIFT_CLEARANCE if self.phase in {"lift", "retreat", "settle", "done"} else 0.0
        return position[0], position[1], position[2] + height

    @property
    def close_fingers(self) -> bool:
        """Return whether the current phase commands a closed gripper."""
        return self.phase in _CLOSED_PHASES

    @property
    def is_done(self) -> bool:
        """Return whether the transfer has completed."""
        return self.phase == "done"

    def reset(
        self,
        *,
        transfer_index: int,
        pick_position: Sequence[float],
        place_position: Sequence[float],
        time: float,
    ) -> None:
        """Reset the machine with freshly measured world-space goals.

        Args:
            transfer_index: Zero-based transfer number used in diagnostics.
            pick_position: Measured world-space center of the object to move.
            place_position: Desired world-space center of the placed object.
            time: Current simulation time in seconds.
        """
        self._transfer_index = transfer_index
        self._pick_position = tuple(float(value) for value in pick_position)
        self._place_position = tuple(float(value) for value in place_position)
        self._pick_height = self._pick_position[2]
        self._phase_started = self._last_time = float(time)
        self._stable_since = None
        self._carry_lost_since = None
        self._machine.set_state("approach_pick", model=self)

    def tick(self, status: GuardInput, time: float) -> bool:
        """Evaluate one bounded transition from one physics-step measurement.

        Args:
            status: Measurements used by the current phase guard.
            time: Current simulation time in seconds.

        Returns:
            Whether the machine changed phases.
        """
        if self.is_done:
            return False
        values = (
            time,
            status.position_error,
            status.orientation_error,
            *status.finger_positions,
            status.object_height,
            status.grasp_distance,
            status.placement_error,
            status.object_linear_speed,
            status.object_angular_speed,
        )
        if not all(math.isfinite(float(value)) for value in values) or time < self._last_time:
            raise RuntimeError(f"Transfer {self._transfer_index + 1} {self.phase}: invalid measurement.")
        elapsed = time - self._phase_started
        if elapsed >= _PHASE_TIMEOUTS[self.phase]:
            raise RuntimeError(f"Transfer {self._transfer_index + 1} {self.phase}: timed out at {elapsed:.2f} s.")
        if self.phase in _CARRY_PHASES and status.grasp_distance > _CARRY_DISTANCE:
            if self._carry_lost_since is None:
                self._carry_lost_since = time
            if time - self._carry_lost_since >= _CARRY_LOSS_DWELL:
                raise RuntimeError(f"Transfer {self._transfer_index + 1} {self.phase}: lost the carried cube.")
        else:
            self._carry_lost_since = None
        changed = bool(self._machine.events["tick"].trigger(self, status=status, time=float(time)))
        self._last_time = float(time)
        return changed

    def _motion_complete(self, event: EventData) -> bool:
        status, time = event.kwargs["status"], event.kwargs["time"]
        return self._stable(self._converged(status), time)

    def _grasp_complete(self, event: EventData) -> bool:
        status, time = event.kwargs["status"], event.kwargs["time"]
        good = (
            self._converged(status)
            and min(status.finger_positions) >= _FINGER_MIN_GRASP
            and max(status.finger_positions) <= _FINGER_MAX_GRASP
            and status.grasp_distance <= _GRASP_DISTANCE
            and time - self._phase_started >= _GRIPPER_DWELL
        )
        return self._stable(good, time)

    def _lift_complete(self, event: EventData) -> bool:
        status, time = event.kwargs["status"], event.kwargs["time"]
        good = (
            self._converged(status, _LIFT_POSITION_TOLERANCE)
            and status.object_height - self._pick_height >= _LIFT_RISE
            and status.grasp_distance <= _CARRY_DISTANCE
        )
        return self._stable(good, time)

    def _release_complete(self, event: EventData) -> bool:
        status, time = event.kwargs["status"], event.kwargs["time"]
        good = (
            self._converged(status)
            and min(status.finger_positions) >= _FINGER_OPEN_POSITION
            and time - self._phase_started >= _GRIPPER_DWELL
        )
        return self._stable(good, time)

    def _settle_complete(self, event: EventData) -> bool:
        status, time = event.kwargs["status"], event.kwargs["time"]
        settled = (
            status.placement_error <= _SETTLE_POSITION_TOLERANCE
            and status.object_linear_speed <= _SETTLE_LINEAR_SPEED
            and status.object_angular_speed <= _SETTLE_ANGULAR_SPEED
        )
        return self._stable(settled, time)

    @staticmethod
    def _converged(status: GuardInput, position_tolerance: float = _POSITION_TOLERANCE) -> bool:
        return status.position_error <= position_tolerance and status.orientation_error <= _ORIENTATION_TOLERANCE

    def _stable(self, good: bool, time: float) -> bool:
        if not good:
            self._stable_since = None
            return False
        if self._stable_since is None:
            self._stable_since = time
        return time - self._stable_since >= _STABILITY_DWELL

    def _enter_phase(self, event: EventData) -> None:
        self._phase_started = event.kwargs["time"]
        self._stable_since = None
