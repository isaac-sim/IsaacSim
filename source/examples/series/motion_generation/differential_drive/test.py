# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate differential-drive control and headless rendering."""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import patch

import main as example

_MINIMUM_DISTANCE = 0.05
_MAXIMUM_VERTICAL_DISPLACEMENT = 0.05


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
    results: list[tuple[float, float, float, float]] = []
    real_viewport = viewport_api.Viewport
    real_report_result = example._report_result

    def observed_viewport(*args: Any, **kwargs: Any) -> _ObservedViewport:
        return _ObservedViewport(real_viewport(*args, **kwargs), frames)

    def observed_report_result(
        robot: Any,
        initial_position: Any,
        commanded_left: float,
        commanded_right: float,
    ) -> tuple[float, float]:
        result = real_report_result(robot, initial_position, commanded_left, commanded_right)
        results.append((*result, commanded_left, commanded_right))
        return result

    with (
        patch.object(viewport_api, "Viewport", observed_viewport),
        patch.object(example, "_report_result", observed_report_result),
    ):
        example.main(visible=False)

    if len(results) != 1:
        raise RuntimeError("The differential-drive example did not report exactly one result.")
    distance, vertical_displacement, actual_left, actual_right = results[0]
    expected_left = (2.0 * example.LINEAR_SPEED - example.YAW_RATE * example.WHEEL_BASE) / (2.0 * example.WHEEL_RADIUS)
    expected_right = (2.0 * example.LINEAR_SPEED + example.YAW_RATE * example.WHEEL_BASE) / (2.0 * example.WHEEL_RADIUS)
    if not (
        math.isclose(actual_left, expected_left, rel_tol=1e-5)
        and math.isclose(actual_right, expected_right, rel_tol=1e-5)
    ):
        raise RuntimeError(
            f"Unexpected wheel speeds [{actual_left}, {actual_right}]; "
            f"expected [{expected_left:.3f}, {expected_right:.3f}]."
        )
    if distance < _MINIMUM_DISTANCE:
        raise RuntimeError("The differential-drive robot did not move.")
    if vertical_displacement > _MAXIMUM_VERTICAL_DISPLACEMENT:
        raise RuntimeError("The differential-drive robot moved vertically instead of remaining on the ground.")
    if len(frames) < 2 or frames[-1][0] <= frames[0][0] or frames[-1][1] == frames[0][1]:
        raise RuntimeError("OVGL did not render the differential-drive transform updates.")
    print("Differential drive example passed.")
    print("Headless differential-drive rendering test passed.")


if __name__ == "__main__":
    main()
