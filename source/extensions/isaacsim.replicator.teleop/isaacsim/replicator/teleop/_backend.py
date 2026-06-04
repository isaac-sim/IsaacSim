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

"""Shared XformPrim backend selection for all teleop components.

Provides a single source of truth for the USD / Fabric backend used by
markers, locomotion, tracking-space reads, and any future writers.
Components import ``get_teleop_backend`` instead of managing backend
selection independently.
"""

from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager
from typing import Literal

import carb.settings
from isaacsim.core.experimental.utils.backend import use_backend

_SUPPORTED_BACKENDS: tuple[str, ...] = ("usd", "usdrt", "fabric")

_override: str | None = None


def reset_teleop_backend() -> None:
    """Clear the backend override, reverting to the default (USD).

    Called from extension shutdown to prevent stale state across reloads.
    """
    global _override
    _override = None


def get_teleop_backend() -> str:
    """Return the active teleop write backend.

    If an explicit override has been set via :func:`set_teleop_backend`,
    that value is returned.  Otherwise defaults to ``"usd"``.
    """
    if _override is not None:
        return _override
    return "usd"


def set_teleop_backend(backend: Literal["usd", "usdrt", "fabric"] | None) -> None:
    """Override the default backend.

    Pass ``None`` to clear the override and return to the default (USD).
    ``"usdrt"`` and ``"fabric"`` require Fabric Scene Delegate (FSD);
    if FSD is not enabled the override is rejected and a warning is printed.
    """
    global _override
    if backend is None:
        _override = None
        print("[Teleop][Backend] Override cleared — using default (USD).")
        return
    if backend not in _SUPPORTED_BACKENDS:
        print(f"[Teleop][Backend] Unknown backend '{backend}'; supported: {_SUPPORTED_BACKENDS}")
        return
    if backend != "usd" and not carb.settings.get_settings().get_as_bool("/app/useFabricSceneDelegate"):
        print(f"[Teleop][Backend] '{backend}' requires FSD. Keeping current backend.")
        return
    _override = backend
    print(f"[Teleop][Backend] Override set to '{backend}'.")


@contextmanager
def teleop_backend_ctx() -> Generator[None, None, None]:
    """Context manager that activates the teleop backend for XformPrim ops."""
    backend = get_teleop_backend()
    if backend != "usd":
        with use_backend(backend):
            yield
    else:
        yield
