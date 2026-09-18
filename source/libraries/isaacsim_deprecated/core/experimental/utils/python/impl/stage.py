# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Kit-independent functions for working with USD/USDRT stages.

This is the Kit-free implementation. It resolves the process "current stage" without ``omni.usd`` by combining an
explicit thread-local override (see :func:`use_stage`) with a process-level default stage id resolved through
:class:`pxr.UsdUtils.StageCache`. Inside Kit, an adapter sets the default stage id; standalone callers set it once.
"""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Generator

from pxr import Sdf, Usd, UsdUtils

from . import backend as backend_utils

# Thread-local storage to handle nested contexts and concurrent access.
_context = threading.local()

# Process-level default stage id, resolved through the USD stage cache. This replaces the Kit ``omni.usd`` context as
# the source of the "current stage" when no thread-local stage override is active.
_default_stage_id: int | None = None


@contextlib.contextmanager
def use_stage(stage: Usd.Stage) -> Generator[None, None, None]:
    """Context manager that sets a thread-local stage instance.

    Args:
        stage: The stage to set in the context.

    Raises:
        AssertionError: If the stage is not a USD stage instance.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_in_memory = Usd.Stage.CreateInMemory()
        >>> with stage_utils.use_stage(stage_in_memory):
        ...    # operate on the specified stage
        ...    pass
    """
    assert isinstance(stage, Usd.Stage), f"Expected a USD stage instance, got {type(stage)}"
    previous = getattr(_context, "stage", None)
    _context.stage = stage
    try:
        yield
    finally:
        _context.stage = previous


def is_stage_set() -> bool:
    """Check if a stage is set in the context manager.

    Returns:
        Whether a stage is set in the context manager.
    """
    return getattr(_context, "stage", None) is not None


def set_default_stage(stage: Usd.Stage) -> int:
    """Register a stage as the process default and return its stage id.

    The stage is inserted into the USD stage cache if needed so it can be resolved later by id. This preserves the
    implicit "current stage" semantics without a Kit USD context.

    Args:
        stage: The stage to make the process default.

    Returns:
        The stage id of the registered stage.
    """
    global _default_stage_id
    _default_stage_id = get_stage_id(stage)
    return _default_stage_id


def set_default_stage_id(stage_id: int | None) -> None:
    """Set the process default stage id used to resolve the current stage.

    Args:
        stage_id: The stage id to use as the process default, or ``None`` to clear it.
    """
    global _default_stage_id
    _default_stage_id = stage_id


def get_default_stage_id() -> int | None:
    """Get the process default stage id used to resolve the current stage.

    Returns:
        The process default stage id, or ``None`` if it has not been set.
    """
    return _default_stage_id


def _resolve_default_stage() -> Usd.Stage | None:
    """Resolve the compatibility or foundation default stage from the USD stage cache.

    Returns:
        The resulting value.
    """
    stage_cache = UsdUtils.StageCache.Get()
    if _default_stage_id is not None:
        stage = stage_cache.Find(Usd.StageCache.Id.FromLongInt(_default_stage_id))
        if stage:
            return stage

    # Foundation owns the standalone process/thread stage state. Fall back to it
    # so compatibility APIs and migrated libraries observe the same active stage.
    from isaacsim.foundation.utils.stage import get_active_stage

    try:
        foundation_stage = get_active_stage()
    except RuntimeError:
        return None
    return stage_cache.Find(Usd.StageCache.Id.FromLongInt(foundation_stage.get_stage_id()))


def get_current_stage(*, backend: str | None = None) -> Usd.Stage:
    """Get the stage set in the context manager or the process default stage.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        backend: Backend to use to get the stage. If not ``None``, it has precedence over the current backend
            set via the :py:func:`~isaacsim.core.experimental.utils.impl.backend.use_backend` context manager.

    Returns:
        The current stage instance. For the ``usdrt``/``fabric`` backends, a USDRT stage attached to the same
        stage id is returned.

    Raises:
        ValueError: If the backend is not supported.
        ValueError: If there is no stage (set via context manager or as the process default).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.get_current_stage()  # doctest: +NO_CHECK
        Usd.Stage.Open(rootLayer=Sdf.Find('anon:...usd'), ...)
    """
    # Resolve the requested backend.
    if backend is None:
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
    elif backend not in ["usd", "usdrt", "fabric"]:
        raise ValueError(f"Invalid backend: {backend}")
    # Resolve the USD stage: thread-local override first, then the process default.
    stage = getattr(_context, "stage", None)
    if stage is None:
        stage = _resolve_default_stage()
    if stage is None:
        raise ValueError(
            "No stage found. Create a stage and register it with `set_default_stage`/`set_default_stage_id` "
            "or set one via the `use_stage` context manager."
        )
    # Resolve the USDRT/Fabric stage lazily so the pure-USD path does not require the Fabric runtime.
    if backend in ["usdrt", "fabric"]:
        import usdrt

        return usdrt.Usd.Stage.Attach(get_stage_id(stage))
    return stage


def get_stage_id(stage: Usd.Stage) -> int:
    """Get the stage ID of a USD stage.

    Backends: :guilabel:`usd`.

    Args:
        stage: The stage to get the ID of.

    Returns:
        The stage ID.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage = stage_utils.get_current_stage()  # doctest: +NO_CHECK
        >>> stage_utils.get_stage_id(stage)  # doctest: +NO_CHECK
        9223006
    """
    stage_cache = UsdUtils.StageCache.Get()
    stage_id = stage_cache.GetId(stage).ToLongInt()
    if stage_id < 0:
        stage_id = stage_cache.Insert(stage).ToLongInt()
    return stage_id


def define_prim(path: str, type_name: str = "Xform") -> Usd.Prim:
    """Attempt to define a prim of the specified type at the given path.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Common token values for ``type_name`` are:

    * ``"Camera"``, ``"Mesh"``, ``"PhysicsScene"``, ``"Scope"``, ``"Xform"``
    * Shapes (``"Capsule"``, ``"Cone"``, ``"Cube"``, ``"Cylinder"``, ``"Plane"``, ``"Sphere"``)
    * Lights (``"CylinderLight"``, ``"DiskLight"``, ``"DistantLight"``, ``"DomeLight"``,
      ``"RectLight"``, ``"SphereLight"``)

    Args:
        path: Absolute prim path.
        type_name: Token identifying the prim type.

    Raises:
        ValueError: If the path is not a valid or absolute path string.
        RuntimeError: If there is already a prim at the given path with a different type.

    Returns:
        Defined prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage_utils.define_prim("/World/Sphere", type_name="Sphere")  # doctest: +NO_CHECK
        Usd.Prim(</World/Sphere>)
    """
    if not Sdf.Path.IsValidPathString(path) or not Sdf.Path(path).IsAbsolutePath():
        raise ValueError(f"Prim path ({path}) is not a valid or absolute path string")
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        if prim.GetTypeName() != type_name:
            raise RuntimeError(f"A prim already exists at path ({path}) with type ({prim.GetTypeName()})")
        return prim
    return stage.DefinePrim(path, type_name)
