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

"""Registry mapping ``Recordable.TYPE_ID`` strings to :class:`Recordable` classes.

The registry is populated at import time by the ``@register_recordable`` decorator.
The replayer uses it to rehydrate recordables from manifest entries without any
caller-side wiring.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .base import Recordable

_REGISTRY: dict[str, type[Recordable]] = {}


def register_recordable(cls: type[Recordable]) -> type[Recordable]:
    """Class decorator: register ``cls`` under its :attr:`Recordable.TYPE_ID`.

    Idempotent re-registration with the identical class is a no-op (Kit's hot-reload
    during tests would otherwise raise). A *different* class trying to claim an
    already-used ``TYPE_ID`` raises :class:`RuntimeError`.

    Args:
        cls: Recordable class to register.

    Returns:
        register recordable result.
    """
    if not cls.TYPE_ID:
        raise ValueError(f"{cls.__name__} must define a non-empty TYPE_ID to be registered.")
    existing = _REGISTRY.get(cls.TYPE_ID)
    if existing is not None and existing is not cls:
        raise RuntimeError(
            f"Recordable TYPE_ID '{cls.TYPE_ID}' is already registered by "
            f"{existing.__module__}.{existing.__qualname__}; cannot re-register "
            f"{cls.__module__}.{cls.__qualname__}."
        )
    _REGISTRY[cls.TYPE_ID] = cls
    return cls


def get_registered(type_id: str) -> type[Recordable] | None:
    """Return the registered class for ``type_id`` or ``None`` if unknown.

    Args:
        type_id: Type id to use.

    Returns:
        get registered result.
    """
    return _REGISTRY.get(type_id)


def rehydrate(entry: Mapping[str, Any]) -> Recordable:
    """Reconstruct a :class:`Recordable` from a manifest entry.

    Args:
        entry: Manifest entry used to reconstruct the recordable.

    Returns:
        rehydrate result.
    """
    if "type" not in entry:
        raise KeyError("Manifest entry missing required 'type' field.")
    type_id = entry["type"]
    cls = _REGISTRY.get(type_id)
    if cls is None:
        raise KeyError(
            f"Unknown recordable type {type_id!r}. Is the extension that defines it loaded? "
            f"Registered types: {sorted(_REGISTRY)}."
        )
    return cls.from_manifest(entry)


def registered_types() -> list[str]:
    """Return the list of currently-registered ``TYPE_ID`` strings (sorted).

    Returns:
        registered types result.
    """
    return sorted(_REGISTRY)


def unregister_recordable(type_id: str) -> None:
    """Remove a registration. Primarily used by tests. Silently ignores unknown ids.

    Args:
        type_id: Type id to use.
    """
    _REGISTRY.pop(type_id, None)
