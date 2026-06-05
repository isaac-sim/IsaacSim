# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Process-wide hooks for attaching extra :class:`Recordable`\\ s to a session.

A session injector is any callable that receives a freshly created
:class:`EpisodeRecorder` (before :meth:`EpisodeRecorder.open_session` is invoked)
and may call :meth:`EpisodeRecorder.add` to append extra recordables.

This mechanism lets satellite extensions (e.g. ``isaacsim.replicator.teleop``)
contribute their own channels to sessions opened by a UI that has no direct
awareness of them — e.g. the standalone Episode Recorder window.

The recorder itself does *not* call these hooks; callers (typically the UI
panel) invoke :func:`apply_session_injectors` after the recorder is built and
before :meth:`open_session`. Scripted users who assemble recorders directly
remain unaffected unless they opt in.

Example:

.. code-block:: python

    from isaacsim.replicator.episode_recorder import (
        EpisodeRecorder,
        apply_session_injectors,
        register_session_injector,
    )

    def _add_teleop(recorder: EpisodeRecorder) -> None:
        recorder.add(TeleopHeadRecordable(group="teleop/head", teleop_manager=tm))

    handle = register_session_injector(_add_teleop)
    ...
    recorder = EpisodeRecorder(output_dir)
    apply_session_injectors(recorder)
    recorder.open_session()
    ...
    handle()  # unregister when teleop is destroyed
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import carb

if TYPE_CHECKING:
    from .recorder import EpisodeRecorder

SessionInjector = Callable[["EpisodeRecorder"], None]
"""Type alias for a session-injector callable."""


_injectors: list[SessionInjector] = []


def register_session_injector(fn: SessionInjector) -> Callable[[], None]:
    """Register ``fn`` as a session injector.

    Returns a zero-arg handle that removes the injector when called. The handle is
    idempotent: invoking it more than once is safe. Re-registering the same callable
    appends a second entry and therefore produces a second handle.
    """
    _injectors.append(fn)
    removed = False

    def _unregister() -> None:
        nonlocal removed
        if removed:
            return
        removed = True
        try:
            _injectors.remove(fn)
        except ValueError:
            pass

    return _unregister


def unregister_session_injector(fn: SessionInjector) -> bool:
    """Remove the first registration of ``fn``. Returns ``True`` if removed."""
    try:
        _injectors.remove(fn)
        return True
    except ValueError:
        return False


def registered_session_injectors() -> tuple[SessionInjector, ...]:
    """Snapshot of currently registered injectors (registration order)."""
    return tuple(_injectors)


def clear_session_injectors() -> None:
    """Remove all registered injectors. Intended for tests."""
    _injectors.clear()


def apply_session_injectors(recorder: "EpisodeRecorder") -> None:
    """Invoke every registered injector against ``recorder``.

    Injectors are called in registration order. Exceptions are logged but not
    re-raised so a single misbehaving injector cannot break session setup; the
    offending injector simply contributes no recordables.
    """
    for fn in list(_injectors):
        try:
            fn(recorder)
        except Exception as exc:  # noqa: BLE001 — injectors are third-party code.
            carb.log_warn(
                f"[isaacsim.replicator.episode_recorder] session injector "
                f"{getattr(fn, '__qualname__', fn)!r} raised: {exc}"
            )
