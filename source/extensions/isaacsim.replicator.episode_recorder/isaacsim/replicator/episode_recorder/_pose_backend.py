# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Pose-batch backend selection for the recorder and replayer.

Both the recorder's sampling batch and the replayer's tier-ordered apply
batch route ``XformPrim`` world-pose I/O through one of three backends.
``"usd"`` (default) handles nested xforms correctly - parent writes are
visible to children within the same frame. ``"usdrt"`` / ``"fabric"`` can
lag a frame on nested articulations but win on flat scenes; both require
Fabric Scene Delegate (FSD). FSD is re-validated at every
:func:`pose_backend_ctx` entry so a mid-session toggle silently demotes
to ``"usd"`` instead of crashing the writer.
"""

from __future__ import annotations

from collections.abc import Generator
from contextlib import AbstractContextManager, contextmanager, nullcontext
from typing import Literal

import carb
import carb.settings

PoseBackend = Literal["usd", "usdrt", "fabric"]

_SUPPORTED_BACKENDS: tuple[PoseBackend, ...] = ("usd", "usdrt", "fabric")
_DEFAULT_BACKEND: PoseBackend = "usd"

# One-shot warning gate per demoted backend; cleared when FSD comes back on.
_demotion_warned: set[PoseBackend] = set()


def _fsd_enabled() -> bool:
    return carb.settings.get_settings().get_as_bool("/app/useFabricSceneDelegate")


def normalize_pose_backend(backend: str | None) -> PoseBackend:
    """Validate / normalize a pose-backend selector to one of the supported values.

    ``None`` and unknown names fall back to ``"usd"`` (the latter logs a
    warning so typos don't silently degrade a session). Non-USD backends
    with FSD off are also coerced to ``"usd"`` here; :func:`pose_backend_ctx`
    re-validates per tick so a mid-session FSD toggle is also handled.

    Args:
        backend: Pose backend selector to normalize.

    Returns:
        Normalized pose backend selector.
    """
    if backend is None:
        return _DEFAULT_BACKEND
    if backend not in _SUPPORTED_BACKENDS:
        carb.log_warn(
            f"[episode_recorder] pose_backend {backend!r} is not supported "
            f"(valid: {_SUPPORTED_BACKENDS}); falling back to {_DEFAULT_BACKEND!r}."
        )
        return _DEFAULT_BACKEND
    if backend != "usd" and not _fsd_enabled():
        carb.log_warn(
            f"[episode_recorder] pose_backend {backend!r} requires Fabric Scene Delegate "
            f"('/app/useFabricSceneDelegate'); falling back to {_DEFAULT_BACKEND!r}."
        )
        return _DEFAULT_BACKEND
    return backend


def pose_backend_ctx(backend: PoseBackend) -> AbstractContextManager[None]:
    """Return a context manager that activates ``backend`` for ``XformPrim`` ops.

    ``"usd"`` is a free :class:`contextlib.nullcontext`. ``"usdrt"`` /
    ``"fabric"`` delegate to ``use_backend`` and re-check FSD first;
    when FSD is off the backend silently demotes to ``"usd"`` with a
    one-shot warning so writes never crash on a missing fabric stage.

    Args:
        backend: Pose backend selector to normalize.

    Returns:
        Context manager for the selected pose backend.
    """
    if backend == "usd":
        return nullcontext()
    if not _fsd_enabled():
        if backend not in _demotion_warned:
            _demotion_warned.add(backend)
            carb.log_warn(
                f"[episode_recorder] pose_backend {backend!r} now requires FSD "
                f"('/app/useFabricSceneDelegate'); FSD is off, demoting to "
                f"{_DEFAULT_BACKEND!r} for this and subsequent ticks."
            )
        return nullcontext()
    # FSD is back on - clear the gate so a future toggle-off warns again.
    _demotion_warned.discard(backend)
    return _use_backend_ctx(backend)


@contextmanager
def _use_backend_ctx(backend: PoseBackend) -> Generator[None, None, None]:
    from isaacsim.core.experimental.utils.backend import use_backend

    with use_backend(backend):
        yield
