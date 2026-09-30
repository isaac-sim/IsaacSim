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

"""Run the state-machine example headlessly and validate physical boundary observations."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
from unittest.mock import patch

import main as example
import numpy as np

_ROBOT_PATH = "/World/Franka"
_BLOCK_PATHS = ("/World/Block0", "/World/Block1")
_TOOL_FRAME = "panda_hand"
_FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
_MOVING_BLOCK = 1
_SUPPORT_BLOCK = 0
_HAND_TO_GRASP_POSITION = np.asarray((0.0, 0.0, 0.1034), dtype=np.float64)
_EXPECTED_SUPPORT_POSITION = np.asarray((0.45, 0.32, 0.025), dtype=np.float64)
_EXPECTED_PLACE_POSITIONS = (
    np.asarray((0.45, 0.32, 0.075), dtype=np.float64),
    np.asarray((0.64, 0.34, 0.025), dtype=np.float64),
)
# Independent acceptance thresholds: distances use meters, angles radians, and speeds SI units.
_FINGER_CLOSED_TARGET = 1.0e-3
_FINGER_STOPPED_MIN = 0.005
_FINGER_STOPPED_MAX = 0.035
_FINGER_STOPPED_SPEED = 0.10
_FINGER_OPEN_POSITION = 0.035
_LIFT_RISE = 0.08
_CARRY_DISTANCE = 0.08
# One 60 Hz sample may straddle a command/measurement boundary; sustained separation is a failure.
_MAX_CARRY_SEPARATION_STEPS = 1
_PLACE_TOLERANCE = 0.05
_SUPPORT_TOLERANCE = 0.02
_SETTLE_LINEAR_SPEED = 0.10
_SETTLE_ANGULAR_SPEED = 0.25
_SETTLE_TILT = 0.20
_SETTLE_WINDOW_STEPS = 4


@dataclass(frozen=True, slots=True)
class _PhysicsStep:
    """Copied commands and measurements from one post-physics callback."""

    index: int
    position_targets: np.ndarray
    finger_positions: np.ndarray
    finger_velocities: np.ndarray
    grasp_object_distance: float
    block_positions: np.ndarray
    block_orientations: np.ndarray
    block_linear_velocities: np.ndarray
    block_angular_velocities: np.ndarray


@dataclass
class _Observations:
    frames: list[tuple[int, int, int, bytes]] = field(default_factory=list)
    steps: list[_PhysicsStep] = field(default_factory=list)
    callback_error: Exception | None = None
    finger_indices: tuple[int, int] | None = None
    dof_count: int = 0


def _rotate(vector: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    quaternion = orientation / np.linalg.norm(orientation)
    xyz = quaternion[1:]
    return vector + 2.0 * (quaternion[0] * np.cross(xyz, vector) + np.cross(xyz, np.cross(xyz, vector)))


def _tilt_error(orientation: np.ndarray) -> float:
    local_z = _rotate(np.asarray((0.0, 0.0, 1.0)), orientation)
    return float(np.arccos(np.clip(abs(local_z[2]), 0.0, 1.0)))


def _validate_transfer(
    transfer_index: int,
    closing_steps: list[_PhysicsStep],
    released_steps: list[_PhysicsStep],
) -> _PhysicsStep:
    """Validate one independently identified close/lift/open/place cycle.

    Args:
        transfer_index: Zero-based transfer number used in diagnostics and expected-position lookup.
        closing_steps: Measurements captured while the fingers were commanded closed.
        released_steps: Measurements captured after the fingers were commanded open.

    Returns:
        Last measurement in the trailing settled window.
    """
    stopped = [
        step
        for step in closing_steps
        if np.all(step.finger_positions > _FINGER_STOPPED_MIN)
        and np.all(step.finger_positions < _FINGER_STOPPED_MAX)
        and np.max(np.abs(step.finger_velocities)) <= _FINGER_STOPPED_SPEED
    ]
    if not stopped:
        raise RuntimeError(f"Transfer {transfer_index + 1} had no stopped-short, low-speed finger closure.")

    pick_height = float(closing_steps[0].block_positions[_MOVING_BLOCK, 2])
    lifted_index = next(
        (
            index
            for index, step in enumerate(closing_steps)
            if step.block_positions[_MOVING_BLOCK, 2] >= pick_height + _LIFT_RISE
            and step.grasp_object_distance <= _CARRY_DISTANCE
        ),
        None,
    )
    if lifted_index is None:
        raise RuntimeError(f"Transfer {transfer_index + 1} did not lift the cube while keeping grasp proximity.")

    reopened_index = next(
        (index for index, step in enumerate(released_steps) if np.min(step.finger_positions) >= _FINGER_OPEN_POSITION),
        None,
    )
    if reopened_index is None:
        raise RuntimeError(f"Transfer {transfer_index + 1} did not measurably reopen both fingers.")

    separation_run = 0
    for step in closing_steps[lifted_index:] + released_steps[: reopened_index + 1]:
        separation_run = separation_run + 1 if step.grasp_object_distance > _CARRY_DISTANCE else 0
        if separation_run > _MAX_CARRY_SEPARATION_STEPS:
            raise RuntimeError(f"Transfer {transfer_index + 1} lost sustained grasp proximity during carry.")

    expected = _EXPECTED_PLACE_POSITIONS[transfer_index]
    trailing_settle = released_steps[reopened_index:][-_SETTLE_WINDOW_STEPS:]
    if len(trailing_settle) < _SETTLE_WINDOW_STEPS:
        raise RuntimeError(f"Transfer {transfer_index + 1} did not retain a trailing settled window.")
    for step in trailing_settle:
        measurements = np.asarray(
            (
                np.linalg.norm(step.block_positions[_MOVING_BLOCK] - expected),
                np.linalg.norm(step.block_linear_velocities[_MOVING_BLOCK]),
                np.linalg.norm(step.block_angular_velocities[_MOVING_BLOCK]),
                _tilt_error(step.block_orientations[_MOVING_BLOCK]),
            )
        )
        limits = np.asarray((_PLACE_TOLERANCE, _SETTLE_LINEAR_SPEED, _SETTLE_ANGULAR_SPEED, _SETTLE_TILT))
        if not np.all(np.isfinite(measurements)) or not np.all(measurements <= limits):
            raise RuntimeError(
                f"Transfer {transfer_index + 1} did not end with a consecutive settled window at its destination."
            )
    return trailing_settle[-1]


def _validate_physical_behavior(observations: _Observations) -> None:
    """Validate two ordered, independently measured physical transfers.

    Args:
        observations: Commands and measurements captured during the example.
    """
    if not observations.steps or observations.finger_indices is None:
        raise RuntimeError("The test did not observe Franka physics steps.")
    commands = np.stack([step.position_targets[0] for step in observations.steps])
    if commands.shape[1] != observations.dof_count:
        raise RuntimeError("The backend position targets do not span the Franka DOF space.")
    finger_commands = commands[:, list(observations.finger_indices)]
    closed = np.max(np.abs(finger_commands), axis=1) <= _FINGER_CLOSED_TARGET
    opened = np.min(finger_commands, axis=1) >= _FINGER_OPEN_POSITION
    close_starts = np.flatnonzero(closed & ~np.concatenate(([False], closed[:-1])))
    if len(close_starts) != len(_EXPECTED_PLACE_POSITIONS):
        raise RuntimeError(f"Expected two close-command cycles, observed {len(close_starts)}.")

    settled_steps = []
    for transfer_index, close_start in enumerate(close_starts):
        stop = int(close_starts[transfer_index + 1]) if transfer_index + 1 < len(close_starts) else len(closed)
        reopen_candidates = np.flatnonzero(opened[int(close_start) + 1 : stop])
        if len(reopen_candidates) == 0:
            raise RuntimeError(f"Transfer {transfer_index + 1} had no ordered open command after closing.")
        reopen_start = int(close_start) + 1 + int(reopen_candidates[0])
        settled_steps.append(
            _validate_transfer(
                transfer_index,
                observations.steps[int(close_start) : reopen_start],
                observations.steps[reopen_start:stop],
            )
        )

    first_settle_index = settled_steps[0].index
    second_close_start = int(close_starts[1])
    support_motion = max(
        np.linalg.norm(step.block_positions[_SUPPORT_BLOCK] - _EXPECTED_SUPPORT_POSITION)
        for step in observations.steps[first_settle_index:second_close_start]
    )
    final_support_error = np.linalg.norm(settled_steps[-1].block_positions[_SUPPORT_BLOCK] - _EXPECTED_SUPPORT_POSITION)
    if support_motion > _SUPPORT_TOLERANCE or final_support_error > _SUPPORT_TOLERANCE:
        raise RuntimeError("The support cube was not stable through the stack/unstack sequence.")


def _validate_rendering(observations: _Observations) -> None:
    """Validate initial, intermediate, and final copied OVGL frames.

    Args:
        observations: Frames captured during the example.
    """
    if len(observations.frames) < 3:
        raise RuntimeError("OVGL did not render initial, intermediate, and final task frames.")
    dimensions = {(width, height) for width, height, _, _ in observations.frames}
    ordinals = [ordinal for _, _, ordinal, _ in observations.frames]
    if len(dimensions) != 1 or ordinals != sorted(ordinals) or len(set(ordinals)) != len(ordinals):
        raise RuntimeError("OVGL frame dimensions or stage ordinals changed unexpectedly.")
    initial, intermediate, final = (observations.frames[index][3] for index in (0, len(observations.frames) // 2, -1))
    if initial == intermediate or intermediate == final:
        raise RuntimeError("OVGL did not render changed pixels at task boundaries.")


def main() -> None:
    """Run the example with test-specific physics and rendering observers."""
    import isaacsim.core.experimental.utils.stage as compatibility_stage_utils
    import isaacsim.ovgl_viewport.debug as viewport_api
    from isaacsim.foundation.objects import Stage
    from isaacsim.physics.entities import ArticulationEntity, RigidBodyEntity
    from isaacsim.physics.manager import PhysicsEvent, PhysicsManager

    observations = _Observations()
    physics_manager = PhysicsManager.get_instance()
    articulation = blocks = None
    finger_indices = hand_link_index = None

    def observe_step(_: Any) -> None:
        nonlocal articulation, blocks, finger_indices, hand_link_index
        if observations.callback_error is not None:
            return
        try:
            if articulation is None:
                articulation = ArticulationEntity("ovphysx", _ROBOT_PATH)
                blocks = RigidBodyEntity("ovphysx", list(_BLOCK_PATHS))
                dof_names = list(articulation.dof_names)
                link_names = list(articulation.link_names)
                finger_indices = tuple(dof_names.index(name) for name in _FINGER_JOINTS)
                hand_link_index = link_names.index(_TOOL_FRAME)
                observations.finger_indices = finger_indices
                observations.dof_count = len(dof_names)

            positions = articulation.get_dof_positions().numpy().reshape(1, observations.dof_count)
            velocities = articulation.get_dof_velocities().numpy().reshape(1, observations.dof_count)
            position_targets = (
                articulation.get_data("dof-position-targets").numpy().copy().reshape(1, observations.dof_count)
            )
            link_transforms = articulation.get_data("link-transforms").numpy().reshape(1, -1, 7)
            hand = link_transforms[0, hand_link_index]
            hand_position = hand[:3].astype(np.float64, copy=True)
            hand_orientation = hand[[6, 3, 4, 5]]
            grasp_position = hand_position + _rotate(_HAND_TO_GRASP_POSITION, hand_orientation)
            block_transforms = blocks.get_data("transforms").numpy().reshape(len(_BLOCK_PATHS), 7)
            linear_velocities, angular_velocities = blocks.get_velocities()
            block_positions = block_transforms[:, :3].astype(np.float64, copy=True)
            observations.steps.append(
                _PhysicsStep(
                    index=len(observations.steps),
                    position_targets=position_targets,
                    finger_positions=positions[0, list(finger_indices)].astype(np.float64, copy=True),
                    finger_velocities=velocities[0, list(finger_indices)].astype(np.float64, copy=True),
                    grasp_object_distance=float(np.linalg.norm(grasp_position - block_positions[_MOVING_BLOCK])),
                    block_positions=block_positions,
                    block_orientations=block_transforms[:, [6, 3, 4, 5]].astype(np.float64, copy=True),
                    block_linear_velocities=linear_velocities.numpy().copy().reshape(-1, 3),
                    block_angular_velocities=angular_velocities.numpy().copy().reshape(-1, 3),
                )
            )
        except Exception as error:
            # OvPhysX callback dispatch swallows exceptions, so retain them for the test boundary.
            observations.callback_error = error

    real_render = viewport_api.Viewport.render

    def observe_render(viewport: Any) -> Any:
        frame = real_render(viewport)
        observations.frames.append((frame.width, frame.height, frame.stage_ordinal, bytes(frame.rgba)))
        return frame

    previous_compatibility_id = compatibility_stage_utils.get_default_stage_id()
    stale_stage = Stage("openusd").create_stage(make_default=False)
    stale_stage_id = stale_stage.get_stage_id()
    try:
        compatibility_stage_utils.set_default_stage_id(stale_stage_id)
        callback_id = physics_manager.register_callback(observe_step, PhysicsEvent.PHYSICS_POST_STEP)
        try:
            with patch.object(viewport_api.Viewport, "render", observe_render):
                example.main(visible=False)
        finally:
            physics_manager.deregister_callback(callback_id)

        if compatibility_stage_utils.get_default_stage_id() != stale_stage_id:
            raise RuntimeError("The example did not restore the caller's compatibility default stage ID.")

        if observations.callback_error is not None:
            raise RuntimeError("Post-physics observation failed.") from observations.callback_error
        _validate_physical_behavior(observations)
        _validate_rendering(observations)
    finally:
        try:
            compatibility_stage_utils.set_default_stage_id(previous_compatibility_id)
        finally:
            stale_stage.close_stage()

    print("Pick-and-place state-machine example passed.")
    print("Headless rendering smoke test passed.")


if __name__ == "__main__":
    main()
