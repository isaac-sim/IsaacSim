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

"""Helpers to auto-discover recorder targets under a USD root.

The Record panel UX is designed around "point at a USD root and record everything of the right
type underneath it". These helpers walk the USD stage under a given root path and return the
``{name: prim_path}`` dictionaries consumed by :func:`build_teleop_recorder` or by explicit
``EpisodeRecorder.add(...)`` calls. Names are derived from the prim leaf-name and uniquified
with a numeric suffix on collision so they are safe to use as HDF5 group identifiers.

Functions:
    * :func:`discover_articulations_under` — prims carrying ``UsdPhysics.ArticulationRootAPI``.
    * :func:`discover_rigid_bodies_under` — prims carrying ``UsdPhysics.RigidBodyAPI``, optionally
      excluding those that are already covered by a discovered articulation.
    * :func:`discover_xforms_under` — ``UsdGeom.Xformable`` prims, with optional filtering.
    * :func:`discover_all_under` — convenience bundler that returns a
      ``(articulations, prims)`` tuple ready to hand to ``EpisodeRecorder``.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Iterable
from typing import Any

import carb

_SANITIZE_PATTERN = re.compile(r"[^0-9A-Za-z_]")


def sanitize_name(name: str) -> str:
    """Convert an arbitrary string into a safe HDF5 group name (``[A-Za-z0-9_]+``)."""
    clean = _SANITIZE_PATTERN.sub("_", name).strip("_")
    if not clean:
        clean = "prim"
    if clean[0].isdigit():
        clean = f"_{clean}"
    return clean


def assign_unique_name(base: str, used: set[str]) -> str:
    """Return a variant of ``base`` that is not present in ``used``. Adds a numeric suffix on collision.

    Mutates ``used`` in-place by adding the chosen name.
    """
    if base not in used:
        used.add(base)
        return base
    i = 1
    while f"{base}_{i}" in used:
        i += 1
    name = f"{base}_{i}"
    used.add(name)
    return name


def _iter_descendants(stage: Any, root_path: str, *, max_depth: int | None):
    """Yield ``Usd.Prim`` descendants of ``root_path`` (exclusive), honoring ``max_depth``.

    Traversal is depth-first with children pushed onto a stack; ``stack.pop()`` consumes the
    last pushed sibling first, so the visit order is the *reverse* of ``prim.GetChildren()``.
    This is stable and deterministic for a given stage layout — important because
    :func:`assign_unique_name` resolves collisions based on discovery order, so the first
    prim of a pair wins the clean name and the second gets the numeric suffix.
    """
    from pxr import Sdf

    if not root_path:
        raise ValueError("root_path must be a non-empty prim path.")
    root = stage.GetPrimAtPath(Sdf.Path(root_path))
    if not root.IsValid():
        raise ValueError(f"Root prim '{root_path}' not found on stage.")

    root_depth = Sdf.Path(root_path).pathElementCount
    stack = [root]
    while stack:
        prim = stack.pop()
        if prim == root:
            for child in prim.GetChildren():
                stack.append(child)
            continue
        depth = prim.GetPath().pathElementCount - root_depth
        if max_depth is not None and depth > max_depth:
            continue
        yield prim
        if max_depth is None or depth < max_depth:
            for child in prim.GetChildren():
                stack.append(child)


def _get_stage(stage: Any | None) -> Any:
    """Resolve a stage argument: use ``stage`` if given, else fetch the active ``omni.usd`` stage."""
    if stage is not None:
        return stage
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is loaded; cannot discover recorder targets.")
    return stage


def discover_articulations_under(
    root_path: str,
    *,
    stage: Any = None,
    max_depth: int | None = None,
    name_predicate: Callable[[str], bool] | None = None,
) -> dict[str, str]:
    """Walk ``root_path`` and return articulations keyed by a safe identifier.

    An articulation is any prim with ``UsdPhysics.ArticulationRootAPI`` applied. The returned
    mapping is safe to pass to ``EpisodeRecorder(articulations=...)``.

    Args:
        root_path: USD prim path whose descendants are scanned.
        stage: Optional explicit ``Usd.Stage``. Defaults to the active ``omni.usd`` stage.
        max_depth: Maximum traversal depth (``None`` = unlimited). Depth 1 = direct children only.
        name_predicate: Optional callable that receives the generated name and returns ``True`` to
            keep it. Useful for filtering by prefix (e.g. ``lambda n: n.startswith("Robot")``).

    Returns:
        ``{name: prim_path}`` dictionary. Empty if nothing matches.

    Example:

    .. code-block:: python

        arts = discover_articulations_under("/World")
        # {'Franka': '/World/Franka', 'Allegro': '/World/Allegro'}
    """
    from pxr import UsdPhysics

    stage = _get_stage(stage)
    results: dict[str, str] = {}
    used: set[str] = set()
    for prim in _iter_descendants(stage, root_path, max_depth=max_depth):
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        name = assign_unique_name(sanitize_name(prim.GetName()), used)
        if name_predicate is not None and not name_predicate(name):
            continue
        results[name] = str(prim.GetPath())
    return results


def discover_rigid_bodies_under(
    root_path: str,
    *,
    stage: Any = None,
    max_depth: int | None = None,
    exclude_articulation_descendants: bool = True,
    name_predicate: Callable[[str], bool] | None = None,
) -> dict[str, str]:
    """Walk ``root_path`` and return rigid-body prims keyed by a safe identifier.

    A rigid body is any prim with ``UsdPhysics.RigidBodyAPI`` applied.

    Args:
        root_path: USD prim path whose descendants are scanned.
        stage: Optional explicit ``Usd.Stage``. Defaults to the active ``omni.usd`` stage.
        max_depth: Maximum traversal depth (``None`` = unlimited).
        exclude_articulation_descendants: When ``True`` (default), rigid bodies that live underneath
            an articulation root are skipped — they are already captured via articulation channels.
        name_predicate: Optional callable filtering generated names.

    Returns:
        ``{name: prim_path}`` dictionary ready for ``build_teleop_recorder(rigid_bodies=...)``
        or for explicit :class:`RigidBodyRecordable` construction.
    """
    from pxr import UsdPhysics

    stage = _get_stage(stage)

    articulation_root_paths: list[str] = []
    if exclude_articulation_descendants:
        for prim in _iter_descendants(stage, root_path, max_depth=max_depth):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulation_root_paths.append(str(prim.GetPath()))

    def _is_under_articulation(path_str: str) -> bool:
        return any(path_str == r or path_str.startswith(r + "/") for r in articulation_root_paths)

    results: dict[str, str] = {}
    used: set[str] = set()
    for prim in _iter_descendants(stage, root_path, max_depth=max_depth):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        path_str = str(prim.GetPath())
        if exclude_articulation_descendants and _is_under_articulation(path_str):
            continue
        name = assign_unique_name(sanitize_name(prim.GetName()), used)
        if name_predicate is not None and not name_predicate(name):
            continue
        results[name] = path_str
    return results


def discover_xforms_under(
    root_path: str,
    *,
    stage: Any = None,
    max_depth: int | None = None,
    include_articulations: bool = False,
    include_rigid_bodies: bool = True,
    exclude_articulation_descendants: bool = False,
    name_predicate: Callable[[str], bool] | None = None,
    excluded_types: Iterable[str] = ("Scope", "Camera", "DomeLight", "DistantLight", "SphereLight"),
) -> dict[str, str]:
    """Walk ``root_path`` and return xformable prims keyed by a safe identifier.

    This is the broadest helper: it returns any ``UsdGeom.Xformable`` prim, optionally pruning
    articulation-root prims (usually tracked separately via
    :func:`discover_articulations_under`) and purely-visual prim types that don't carry physics.

    Args:
        root_path: USD prim path whose descendants are scanned.
        stage: Optional explicit ``Usd.Stage``.
        max_depth: Maximum traversal depth.
        include_articulations: If ``False`` (default), prims with ``ArticulationRootAPI`` are
            skipped.
        include_rigid_bodies: If ``False``, prims with ``RigidBodyAPI`` are skipped (useful for
            separating "just an Xform tracker" marker prims from physics bodies).
        exclude_articulation_descendants: If ``True``, skip any prim whose path is underneath an
            articulation root. Use this to get only "loose" xforms (trackers, markers) while
            letting the articulation channel own every link pose below its root. Defaults to
            ``False`` — so visual meshes under articulations *are* returned by default, matching
            the historical behavior.
        name_predicate: Optional callable filtering generated names.
        excluded_types: USD typeNames to skip (e.g. ``Scope``, ``Camera``). Pass an empty tuple to
            include everything.

    Returns:
        ``{name: prim_path}`` dictionary.
    """
    from pxr import UsdGeom, UsdPhysics

    stage = _get_stage(stage)
    excluded = set(excluded_types or ())
    results: dict[str, str] = {}
    used: set[str] = set()

    articulation_root_paths: list[str] = []
    if exclude_articulation_descendants:
        for prim in _iter_descendants(stage, root_path, max_depth=max_depth):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulation_root_paths.append(str(prim.GetPath()))

    def _is_under_articulation(path_str: str) -> bool:
        return any(path_str == r or path_str.startswith(r + "/") for r in articulation_root_paths)

    for prim in _iter_descendants(stage, root_path, max_depth=max_depth):
        if not prim.IsA(UsdGeom.Xformable):
            continue
        if prim.GetTypeName() in excluded:
            continue
        if not include_articulations and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        if not include_rigid_bodies and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        path_str = str(prim.GetPath())
        if exclude_articulation_descendants and _is_under_articulation(path_str):
            continue
        name = assign_unique_name(sanitize_name(prim.GetName()), used)
        if name_predicate is not None and not name_predicate(name):
            continue
        results[name] = path_str
    return results


def discover_all_under(
    root_path: str,
    *,
    stage: Any = None,
    max_depth: int | None = None,
    include_loose_xforms: bool = False,
) -> tuple[dict[str, str], dict[str, str]]:
    """Convenience bundler: return ``(articulations, prims)`` ready for recorder wiring.

    Args:
        root_path: USD prim path whose descendants are scanned.
        stage: Optional explicit ``Usd.Stage``.
        max_depth: Maximum traversal depth.
        include_loose_xforms: When ``True``, also include xform-only prims (no physics APIs) under
            the prims dictionary. Useful for tracking marker / anchor prims that aren't rigid bodies.

    Returns:
        ``(articulations, prims)`` where ``articulations`` comes from
        :func:`discover_articulations_under` and ``prims`` is the union of
        :func:`discover_rigid_bodies_under` (always) and loose Xforms (when requested).

    Names in the two returned dicts are kept mutually unique so callers can feed them into
    ``build_teleop_recorder(...)`` or convert them into explicit recordables without collisions.
    """
    arts = discover_articulations_under(root_path, stage=stage, max_depth=max_depth)
    used: set[str] = set(arts.keys())

    rigids = discover_rigid_bodies_under(
        root_path,
        stage=stage,
        max_depth=max_depth,
        exclude_articulation_descendants=True,
    )
    prims: dict[str, str] = {}
    for name, path in rigids.items():
        prims[assign_unique_name(name, used)] = path

    if include_loose_xforms:
        xforms = discover_xforms_under(
            root_path,
            stage=stage,
            max_depth=max_depth,
            include_articulations=False,
            include_rigid_bodies=False,
        )
        existing_paths = set(prims.values()) | set(arts.values())
        for name, path in xforms.items():
            if path in existing_paths:
                continue
            prims[assign_unique_name(name, used)] = path

    if not arts and not prims:
        carb.log_warn(
            f"[discover_all_under] Nothing recordable found under '{root_path}'. "
            "Add ArticulationRootAPI / RigidBodyAPI to the desired prims, "
            "or pass include_loose_xforms=True to track plain Xforms."
        )
    return arts, prims
