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

"""Stage polling utilities for UI automation and testing.

Provides helpers to wait for specific prims or conditions on the USD stage
by polling with app updates. Useful after deferred button clicks that trigger
async scene loading.

Each polling function has a sync version (uses ``update_app``) and an async
version (uses ``await update_app_async``). Prefer the async versions when
running inside the python_server to avoid blocking other asyncio tasks.
"""

from __future__ import annotations

from collections.abc import Callable

import carb
import isaacsim.core.experimental.utils.app as app_utils

__all__ = [
    "poll_until",
    "poll_until_async",
    "wait_for_prim",
    "wait_for_prim_async",
    "wait_for_stage_prims",
    "wait_for_stage_prims_async",
]


def poll_until(
    check_fn: Callable[[], object],
    timeout_frames: int = 1800,
    poll_steps: int = 30,
    label: str = "",
) -> int | None:
    """Poll a check function until it returns truthy, stepping the app between polls.

    Calls ``app_utils.update_app(steps=poll_steps)`` between each check. Returns
    the number of frames elapsed when the check passes, or ``None`` on timeout.

    Args:
        check_fn: Callable returning a truthy value when the condition is met.
        timeout_frames: Maximum total frames before giving up.
        poll_steps: Number of ``update_app`` steps per poll iteration.
        label: Optional label for log messages.

    Returns:
        Number of elapsed frames on success, or ``None`` on timeout.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.stage_utils import poll_until
        >>> import omni.usd
        >>> stage = omni.usd.get_context().get_stage()
        >>> elapsed = poll_until(lambda: stage.GetPrimAtPath("/World/Robot").IsValid())
    """
    for i in range(timeout_frames // poll_steps):
        app_utils.update_app(steps=poll_steps)
        if check_fn():
            elapsed = (i + 1) * poll_steps
            if label:
                carb.log_info(f"{label} satisfied after {elapsed} frames")
            return elapsed
    if label:
        carb.log_warn(f"{label} not satisfied after {timeout_frames} frames")
    return None


async def poll_until_async(
    check_fn: Callable[[], object],
    timeout_frames: int = 1800,
    poll_steps: int = 30,
    label: str = "",
) -> int | None:
    """Async version of :func:`poll_until`.

    Uses ``await update_app_async`` instead of ``update_app`` to yield to the
    event loop between polls.

    Args:
        check_fn: Callable returning a truthy value when the condition is met.
        timeout_frames: Maximum total frames before giving up.
        poll_steps: Number of ``update_app_async`` steps per poll iteration.
        label: Optional label for log messages.

    Returns:
        Number of elapsed frames on success, or ``None`` on timeout.
    """
    for i in range(timeout_frames // poll_steps):
        await app_utils.update_app_async(steps=poll_steps)
        if check_fn():
            elapsed = (i + 1) * poll_steps
            if label:
                carb.log_info(f"{label} satisfied after {elapsed} frames")
            return elapsed
    if label:
        carb.log_warn(f"{label} not satisfied after {timeout_frames} frames")
    return None


def wait_for_prim(
    prim_path: str,
    timeout_frames: int = 1800,
    poll_steps: int = 30,
) -> bool:
    """Poll until a prim exists and is valid on the current stage.

    Useful after scheduling a deferred button click (e.g. LOAD) that triggers
    async scene loading. Call this in a **separate** python_server command so
    the event loop is free to process the async load.

    Args:
        prim_path: USD path to wait for (e.g. "/World/Franka").
        timeout_frames: Maximum frames before giving up.
        poll_steps: App update steps per poll iteration.

    Returns:
        ``True`` if the prim was found, ``False`` on timeout.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.stage_utils import wait_for_prim
        >>> found = wait_for_prim("/World/Franka", timeout_frames=3600)
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    def _check() -> bool:
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(prim_path)
        return prim is not None and prim.IsValid()

    elapsed = poll_until(_check, timeout_frames, poll_steps, label=prim_path)
    return elapsed is not None


async def wait_for_prim_async(
    prim_path: str,
    timeout_frames: int = 1800,
    poll_steps: int = 30,
) -> bool:
    """Async version of :func:`wait_for_prim`.

    Uses ``await update_app_async`` between polls so other asyncio tasks can run.

    Args:
        prim_path: USD path to wait for.
        timeout_frames: Maximum frames before giving up.
        poll_steps: App update steps per poll iteration.

    Returns:
        ``True`` if the prim was found, ``False`` on timeout.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    def _check() -> bool:
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(prim_path)
        return prim is not None and prim.IsValid()

    elapsed = await poll_until_async(_check, timeout_frames, poll_steps, label=prim_path)
    return elapsed is not None


def wait_for_stage_prims(
    min_prims: int = 20,
    timeout_frames: int = 1800,
    poll_steps: int = 30,
) -> bool:
    """Poll until the stage has at least a minimum number of prims.

    Generic load detection — useful when you don't know the exact prim path
    but know a loaded scene should have many prims.

    Args:
        min_prims: Minimum number of prims required.
        timeout_frames: Maximum frames before giving up.
        poll_steps: App update steps per poll iteration.

    Returns:
        ``True`` if the minimum was reached, ``False`` on timeout.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.stage_utils import wait_for_stage_prims
        >>> loaded = wait_for_stage_prims(min_prims=50)
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    def _check() -> bool:
        if stage is None:
            return False
        count = 0
        for _ in stage.Traverse():
            count += 1
            if count >= min_prims:
                return True
        return False

    elapsed = poll_until(_check, timeout_frames, poll_steps, label=f">={min_prims} prims")
    return elapsed is not None


async def wait_for_stage_prims_async(
    min_prims: int = 20,
    timeout_frames: int = 1800,
    poll_steps: int = 30,
) -> bool:
    """Async version of :func:`wait_for_stage_prims`.

    Args:
        min_prims: Minimum number of prims required.
        timeout_frames: Maximum frames before giving up.
        poll_steps: App update steps per poll iteration.

    Returns:
        ``True`` if the minimum was reached, ``False`` on timeout.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    def _check() -> bool:
        if stage is None:
            return False
        count = 0
        for _ in stage.Traverse():
            count += 1
            if count >= min_prims:
                return True
        return False

    elapsed = await poll_until_async(_check, timeout_frames, poll_steps, label=f">={min_prims} prims")
    return elapsed is not None
