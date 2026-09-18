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

"""Validate the falling-cube simulation and its headless rendering."""

from __future__ import annotations

import itertools
from typing import Any
from unittest.mock import patch

import main as example


class _ObservedViewport:
    """Record owned frame data while forwarding to a real viewport.

    Args:
        viewport: Real viewport receiving forwarded calls.
        frames: Destination for owned frame data.
    """

    def __init__(self, viewport: Any, frames: list[tuple[int, int, int, bytes]]) -> None:
        self._viewport = viewport
        self._frames = frames
        self._poll_count = 0

    def __enter__(self) -> _ObservedViewport:
        self._viewport.__enter__()
        return self

    def __exit__(self, *args: Any) -> None:
        self._viewport.__exit__(*args)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._viewport, name)

    def render(self) -> Any:
        frame = self._viewport.render()
        owned_frame = (frame.width, frame.height, frame.stage_ordinal, bytes(frame.rgba))
        if len(self._frames) < 2:
            self._frames.append(owned_frame)
        else:
            self._frames[-1] = owned_frame
        return frame

    def poll_events(self) -> bool:
        self._poll_count += 1
        simulation_polls = (
            example._SIMULATION_STEPS + example._MAX_STEPS_PER_FRAME - 1
        ) // example._MAX_STEPS_PER_FRAME
        if self._poll_count > simulation_polls + 4:
            return False
        return self._viewport.poll_events()


class _ObservedRigidBody:
    """Record cube heights while forwarding to a real rigid body.

    Args:
        rigid_body: Real rigid body receiving forwarded calls.
        heights: Destination for observed cube heights.
    """

    def __init__(self, rigid_body: Any, heights: list[float]) -> None:
        self._rigid_body = rigid_body
        self._heights = heights

    def __getattr__(self, name: str) -> Any:
        return getattr(self._rigid_body, name)

    def get_world_poses(self, *args: Any, **kwargs: Any) -> Any:
        poses = self._rigid_body.get_world_poses(*args, **kwargs)
        self._heights.append(float(poses[0].numpy()[0, 2]))
        return poses


def main() -> None:
    """Run the example headlessly and validate its observable behavior."""
    frames: list[tuple[int, int, int, bytes]] = []
    heights: list[float] = []
    real_viewport = example.Viewport
    real_rigid_body = example.RigidBodyEntity
    clock = itertools.count()

    def observed_viewport(*args: Any, **kwargs: Any) -> _ObservedViewport:
        kwargs["visible"] = False
        return _ObservedViewport(real_viewport(*args, **kwargs), frames)

    def observed_rigid_body(*args: Any, **kwargs: Any) -> _ObservedRigidBody:
        return _ObservedRigidBody(real_rigid_body(*args, **kwargs), heights)

    def monotonic_time() -> float:
        return next(clock) * example._MAX_FRAME_TIME

    with (
        patch.object(example, "Viewport", observed_viewport),
        patch.object(example, "RigidBodyEntity", observed_rigid_body),
        patch.object(example.time, "monotonic", monotonic_time),
    ):
        example.main()

    if len(heights) < 2 or heights[-1] >= heights[0]:
        raise RuntimeError("The simulated cube did not fall.")
    if len(frames) < 2 or frames[-1][2] <= frames[0][2] or frames[-1][3] == frames[0][3]:
        raise RuntimeError("OVGL did not render the falling cube's OVStage transform updates.")
    print("Headless rendering smoke test passed.")


if __name__ == "__main__":
    main()
