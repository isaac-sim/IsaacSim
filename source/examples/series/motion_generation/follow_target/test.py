# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate follow-target control and headless rendering."""

from __future__ import annotations

from typing import Any
from unittest.mock import patch

import main as example

_POSITION_TOLERANCE = 0.08


class _ObservedViewport:
    """Record owned frame data while forwarding to a real viewport.

    Args:
        viewport: Real viewport receiving forwarded calls.
        frames: Destination for captured frame data.
    """

    def __init__(self, viewport: Any, frames: list[tuple[int, int, int, bytes]]) -> None:
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
        self._frames.append((frame.width, frame.height, frame.stage_ordinal, bytes(frame.rgba)))
        return frame


def main() -> None:
    """Run the example headlessly and validate its observable behavior."""
    import isaacsim.ovgl_viewport.debug as viewport_api

    frames: list[tuple[int, int, int, bytes]] = []
    target_errors: list[float] = []
    real_viewport = viewport_api.Viewport
    real_report_result = example._report_result

    def observed_viewport(*args: Any, **kwargs: Any) -> _ObservedViewport:
        return _ObservedViewport(real_viewport(*args, **kwargs), frames)

    def observed_report_result(*args: Any, **kwargs: Any) -> float:
        error = real_report_result(*args, **kwargs)
        target_errors.append(error)
        return error

    with (
        patch.object(viewport_api, "Viewport", observed_viewport),
        patch.object(example, "_report_result", observed_report_result),
    ):
        example.main(visible=False)

    if len(target_errors) != 1 or target_errors[0] > _POSITION_TOLERANCE:
        error = target_errors[0] if target_errors else float("inf")
        raise RuntimeError(
            f"Franka missed the RMPflow target by {error:.3f} m; " f"expected at most {_POSITION_TOLERANCE:.3f} m."
        )
    if len(frames) < 2:
        raise RuntimeError("OVGL did not render both ends of the RMPflow operation.")
    initial_width, initial_height, initial_ordinal, initial_rgba = frames[0]
    final_width, final_height, final_ordinal, final_rgba = frames[-1]
    if (final_width, final_height) != (initial_width, initial_height):
        raise RuntimeError("OVGL changed the output layout during the RMPflow operation.")
    if final_ordinal <= initial_ordinal or final_rgba == initial_rgba:
        raise RuntimeError("OVGL did not render the RMPflow transform updates.")
    print("Headless RMPflow rendering test passed.")


if __name__ == "__main__":
    main()
