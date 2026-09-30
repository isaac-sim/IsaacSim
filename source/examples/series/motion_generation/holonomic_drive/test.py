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

"""Validate holonomic-drive control and headless rendering."""

from __future__ import annotations

from typing import Any
from unittest.mock import patch

import main as example
import numpy as np

_MINIMUM_DISTANCE = 0.05
_WHEEL_VELOCITY_TOLERANCE = 0.35


class _ObservedViewport:
    """Record owned frame data while forwarding to a real viewport.

    Args:
        viewport: Real viewport receiving forwarded calls.
        frames: Destination for captured frame ordinals and pixels.
    """

    def __init__(self, viewport: Any, frames: list[tuple[int, bytes]]) -> None:
        self._viewport = viewport
        self._frames = frames

    def __getattr__(self, name: str) -> Any:
        return getattr(self._viewport, name)

    def render(self) -> Any:
        """Render and retain frame data needed after the viewport closes.

        Returns:
            Frame returned by the real viewport.
        """
        frame = self._viewport.render()
        self._frames.append((frame.stage_ordinal, bytes(frame.rgba)))
        return frame


def main() -> None:
    """Run the example headlessly and validate its observable behavior."""
    import isaacsim.ovgl_viewport.debug as viewport_api

    frames: list[tuple[int, bytes]] = []
    results: list[tuple[np.ndarray, np.ndarray, float]] = []
    real_viewport = viewport_api.Viewport
    real_report_result = example._report_result

    def observed_viewport(*args: Any, **kwargs: Any) -> _ObservedViewport:
        return _ObservedViewport(real_viewport(*args, **kwargs), frames)

    def observed_report_result(
        robot: Any,
        initial_position: np.ndarray,
        wheel_indices: np.ndarray,
        commanded_wheel_velocities: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        measured_wheel_velocities, distance = real_report_result(
            robot,
            initial_position,
            wheel_indices,
            commanded_wheel_velocities,
        )
        results.append((measured_wheel_velocities, commanded_wheel_velocities, distance))
        return measured_wheel_velocities, distance

    with (
        patch.object(viewport_api, "Viewport", observed_viewport),
        patch.object(example, "_report_result", observed_report_result),
    ):
        example.main(visible=False)

    if len(results) != 1:
        raise RuntimeError("The holonomic-drive example did not report exactly one result.")
    measured_wheel_velocities, commanded_wheel_velocities, distance = results[0]
    if not np.allclose(
        measured_wheel_velocities,
        commanded_wheel_velocities,
        atol=_WHEEL_VELOCITY_TOLERANCE,
    ):
        raise RuntimeError("Kaya's measured wheel velocities did not track the holonomic-drive controller targets.")
    if distance < _MINIMUM_DISTANCE:
        raise RuntimeError("The holonomic-drive robot did not move.")
    if len(frames) < 2 or frames[-1][0] <= frames[0][0] or frames[-1][1] == frames[0][1]:
        raise RuntimeError("OVGL did not render the holonomic-drive transform updates.")
    print("Holonomic drive example passed.")
    print("Headless holonomic-drive rendering test passed.")


if __name__ == "__main__":
    main()
