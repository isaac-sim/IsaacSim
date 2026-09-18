# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Quit Kit after the active USD viewport renders a frame."""

from __future__ import annotations

import asyncio
from typing import Any

import carb.eventdispatcher
import omni.kit.app
import omni.usd

_new_frame_received = asyncio.Event()


def _on_new_frame(_event: Any) -> None:
    """Record delivery of the viewport's new-frame rendering event."""
    _new_frame_received.set()


_usd_context = omni.usd.get_context()
_new_frame_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=_usd_context.stage_rendering_event_name(omni.usd.StageRenderingEventType.NEW_FRAME, True),
    on_event=_on_new_frame,
    observer_name="warmup_shutdown.new_frame",
)


async def _quit_after_new_frame() -> None:
    """Request Kit shutdown after the viewport renders a frame."""
    await _new_frame_received.wait()
    print("Viewport rendered a frame; warmup is complete", flush=True)
    omni.kit.app.get_app().post_quit(0)


_warmup_task = asyncio.ensure_future(_quit_after_new_frame())
