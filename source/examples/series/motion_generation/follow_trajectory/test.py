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

"""Validate the repeating Franka trajectory and its headless rendering."""

from __future__ import annotations

from typing import Any
from unittest.mock import patch

import main as example
import numpy as np

_COMMAND_TOLERANCE = 1.0e-4
_MINIMUM_MEASURED_TRAVEL = 0.4
_RETURN_TOLERANCE = 0.2


class _Observations:
    """Test-specific observations collected from the example's articulation and viewport."""

    def __init__(self) -> None:
        self.dof_names: list[str] = []
        self.commanded_positions: list[np.ndarray] = []
        self.measured_positions: list[np.ndarray] = []
        self.frames: list[tuple[int, int, bytes]] = []


class _ObservedViewport:
    """Record owned frame data while forwarding to a real viewport.

    Args:
        viewport: Real viewport receiving forwarded calls.
        observations: Destination for observed state.
    """

    def __init__(self, viewport: Any, observations: _Observations) -> None:
        self._viewport = viewport
        self._observations = observations

    def __getattr__(self, name: str) -> Any:
        return getattr(self._viewport, name)

    def render(self) -> Any:
        """Render and retain the frame data needed after the viewport closes."""
        frame = self._viewport.render()
        self._observations.frames.append((frame.width, frame.height, bytes(frame.rgba)))
        return frame


class _ObservedArticulation:
    """Record Franka commands and measurements while forwarding to a real articulation.

    Args:
        articulation: Real articulation receiving forwarded calls.
        observations: Destination for observed state.
    """

    def __init__(self, articulation: Any, observations: _Observations) -> None:
        self._articulation = articulation
        self._observations = observations
        self._observations.dof_names = list(articulation.dof_names)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._articulation, name)

    def set_dof_position_targets(self, values: Any, *args: Any, **kwargs: Any) -> Any:
        """Record and forward one position-target command."""
        self._observations.commanded_positions.append(np.asarray(values).reshape(-1).copy())
        return self._articulation.set_dof_position_targets(values, *args, **kwargs)

    def get_dof_positions(self, *args: Any, **kwargs: Any) -> Any:
        """Record and return the measured joint positions."""
        positions = self._articulation.get_dof_positions(*args, **kwargs)
        self._observations.measured_positions.append(positions.numpy()[0].copy())
        return positions

    def record_dof_positions(self) -> None:
        """Record joint positions without routing a read through the example."""
        positions = self._articulation.get_dof_positions()
        self._observations.measured_positions.append(positions.numpy()[0].copy())


class _ObservedPhysicsManager:
    """Record articulation state after each published physics step.

    Args:
        manager: Real physics manager receiving forwarded calls.
        articulations: Articulations created by the example.
    """

    def __init__(self, manager: Any, articulations: list[_ObservedArticulation]) -> None:
        self._manager = manager
        self._articulations = articulations

    def __getattr__(self, name: str) -> Any:
        return getattr(self._manager, name)

    def publish_transforms_to_stage(self) -> bool:
        """Publish transforms and record the resulting articulation positions."""
        published = self._manager.publish_transforms_to_stage()
        if published:
            for articulation in self._articulations:
                articulation.record_dof_positions()
        return published


def _validate_trajectory(observations: _Observations) -> tuple[float, float]:
    """Validate one commanded and measured round trip.

    Args:
        observations: State collected during the example run.

    Returns:
        Maximum measured travel from pose A and final return error.
    """
    if not observations.commanded_positions or not observations.measured_positions:
        raise RuntimeError("The Franka trajectory produced no articulation observations.")

    arm_indices = [observations.dof_names.index(name) for name in example._ARM_JOINTS]
    pose_a = np.asarray(example._POSE_A)
    pose_b = np.asarray(example._POSE_B)
    commanded = np.asarray(observations.commanded_positions)[:, arm_indices]
    measured = np.asarray(observations.measured_positions)[:, arm_indices]

    pose_a_command_error = float(np.min(np.linalg.norm(commanded - pose_a, axis=1)))
    pose_b_command_error = float(np.min(np.linalg.norm(commanded - pose_b, axis=1)))
    if max(pose_a_command_error, pose_b_command_error) > _COMMAND_TOLERANCE:
        raise RuntimeError("The trajectory did not command both configured Franka poses.")

    maximum_travel = float(np.max(np.linalg.norm(measured - pose_a, axis=1)))
    if maximum_travel < _MINIMUM_MEASURED_TRAVEL:
        raise RuntimeError(f"The Franka moved only {maximum_travel:.3f} rad from pose A.")

    return_error = float(np.linalg.norm(measured[-1] - pose_a))
    if return_error > _RETURN_TOLERANCE:
        raise RuntimeError(f"The Franka ended {return_error:.3f} rad from pose A after one round trip.")
    return maximum_travel, return_error


def _validate_rendering(observations: _Observations) -> int:
    """Validate that OVGL rendered visible motion during the round trip.

    Args:
        observations: State collected during the example run.

    Returns:
        Largest number of pixels changed from the initial frame.
    """
    if len(observations.frames) < 3:
        raise RuntimeError("OVGL did not render the start, opposite pose, and end of the Franka trajectory.")
    initial_width, initial_height, initial_rgba = observations.frames[0]
    initial_pixels = np.frombuffer(initial_rgba, dtype=np.uint8).reshape(-1, 4)
    changed_pixel_counts = []
    for width, height, rgba in observations.frames[1:]:
        if (width, height) != (initial_width, initial_height):
            raise RuntimeError("OVGL changed the output layout during the Franka trajectory.")
        pixels = np.frombuffer(rgba, dtype=np.uint8).reshape(-1, 4)
        changed_pixel_counts.append(
            int(
                np.count_nonzero(
                    np.any(
                        np.abs(pixels.astype(np.int16) - initial_pixels.astype(np.int16)) > 4,
                        axis=1,
                    )
                )
            )
        )
    changed_pixels = max(changed_pixel_counts)
    minimum_changed_pixels = initial_width * initial_height // 200
    if changed_pixels < minimum_changed_pixels:
        raise RuntimeError(
            f"OVGL changed only {changed_pixels} pixels during the Franka trajectory; "
            f"expected at least {minimum_changed_pixels}."
        )
    return changed_pixels


def main() -> None:
    """Run one trajectory cycle headlessly and validate its observable behavior."""
    import isaacsim.ovgl_viewport.debug as viewport_api
    import isaacsim.physics.entities as physics_entities
    import isaacsim.physics.manager as physics_manager_api

    observations = _Observations()
    real_viewport = viewport_api.Viewport
    real_articulation = physics_entities.ArticulationEntity
    real_physics_manager = physics_manager_api.PhysicsManager
    observed_articulations: list[_ObservedArticulation] = []

    def observed_viewport(*args: Any, **kwargs: Any) -> _ObservedViewport:
        return _ObservedViewport(real_viewport(*args, **kwargs), observations)

    def observed_articulation(*args: Any, **kwargs: Any) -> _ObservedArticulation:
        articulation = _ObservedArticulation(real_articulation(*args, **kwargs), observations)
        observed_articulations.append(articulation)
        return articulation

    observed_manager = _ObservedPhysicsManager(real_physics_manager.get_instance(), observed_articulations)

    class ObservedPhysicsManager:
        """Provide the observed singleton through the example's normal API."""

        @staticmethod
        def get_instance() -> _ObservedPhysicsManager:
            return observed_manager

    with (
        patch.object(viewport_api, "Viewport", observed_viewport),
        patch.object(example, "ArticulationEntity", observed_articulation),
        patch.object(example, "PhysicsManager", ObservedPhysicsManager),
    ):
        example.main(visible=False)

    maximum_travel, return_error = _validate_trajectory(observations)
    changed_pixels = _validate_rendering(observations)
    print(f"Franka trajectory completed: maximum travel {maximum_travel:.3f} rad, return error {return_error:.3f} rad.")
    print("Headless Franka trajectory rendering test passed.")
    print(f"Changed pixels: {changed_pixels}.")


if __name__ == "__main__":
    main()
