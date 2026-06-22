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

"""Manifest serialization for the HDF5 V2 recorder layout.

The manifest is the **contract** between recorder and replayer. It is stored as JSON
strings inside the ``/manifest`` group of every session file:

- ``/manifest/tracks`` — list of recordable entries (each carries ``type``, ``group``,
  and any plugin-specific binding fields).
- ``/manifest/session`` — user session metadata (task, scene, etc.).
- ``/manifest/sampling`` — :class:`SamplingConfig` fields.
- ``/manifest/coord_conventions`` — global conventions (quaternion order, units).

The top-level file attribute ``schema_version`` is bumped whenever any of the above
formats change incompatibly.
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from typing import Any

SCHEMA_VERSION = 2
MANIFEST_GROUP = "manifest"

_MANIFEST_DATASETS = ("tracks", "session", "sampling", "coord_conventions")


@dataclass
class SessionManifest:
    """In-memory representation of ``/manifest/*`` blobs."""

    tracks: list[dict[str, Any]] = field(default_factory=list)
    session: dict[str, Any] = field(default_factory=dict)
    sampling: dict[str, Any] = field(default_factory=dict)
    coord_conventions: dict[str, Any] = field(
        default_factory=lambda: {
            "quaternion_order": "wxyz",
            "position_units": "meters",
            "angle_units": "radians",
        }
    )

    def track_groups(self) -> list[str]:
        """Return HDF5 group paths of all tracks in insertion order.

        Returns:
            Track group paths in insertion order.
        """
        return [entry["group"] for entry in self.tracks]


def build_manifest(
    recordable_entries: Iterable[dict[str, Any]],
    *,
    sampling: dict[str, Any] | None = None,
    session_metadata: dict[str, Any] | None = None,
    coord_conventions: dict[str, Any] | None = None,
) -> SessionManifest:
    """Assemble a :class:`SessionManifest` from plugin-provided entries and config.

    Args:
        recordable_entries: Recordable manifest entries to include.
        sampling: Sampling metadata to write into the manifest.
        session_metadata: Session metadata to write into the manifest.
        coord_conventions: Coordinate convention metadata to write into the manifest.

    Returns:
        Constructed session manifest.
    """
    manifest = SessionManifest(
        tracks=list(recordable_entries),
        session=dict(session_metadata or {}),
        sampling=dict(sampling or {}),
    )
    if coord_conventions is not None:
        manifest.coord_conventions = dict(coord_conventions)
    _validate_manifest(manifest)
    return manifest


def _validate_manifest(manifest: SessionManifest) -> None:
    seen_groups: set[str] = set()
    for i, entry in enumerate(manifest.tracks):
        if "type" not in entry:
            raise ValueError(f"Manifest track entry {i} missing 'type'.")
        if "group" not in entry:
            raise ValueError(f"Manifest track entry {i} missing 'group'.")
        group = entry["group"]
        if group in seen_groups:
            raise ValueError(f"Manifest has duplicate group {group!r}.")
        seen_groups.add(group)


def write_manifest(h5_file: Any, manifest: SessionManifest) -> None:
    """Write manifest JSON blobs under ``/manifest/*`` and set ``schema_version``.

    Overwrites existing manifest datasets if present.

    Args:
        h5_file: Open HDF5 file handle.
        manifest: Session manifest to write.
    """
    _validate_manifest(manifest)
    h5_file.attrs["schema_version"] = SCHEMA_VERSION
    grp = h5_file.require_group(MANIFEST_GROUP)
    payloads = {
        "tracks": manifest.tracks,
        "session": manifest.session,
        "sampling": manifest.sampling,
        "coord_conventions": manifest.coord_conventions,
    }
    for name, payload in payloads.items():
        if name in grp:
            del grp[name]
        grp.create_dataset(name, data=json.dumps(payload, sort_keys=True, default=_json_default))


def read_manifest(h5_file: Any) -> SessionManifest:
    """Read manifest JSON blobs from ``/manifest/*``.

    Args:
        h5_file: Open HDF5 file handle.

    Returns:
        Session manifest read from the file.
    """
    version = int(h5_file.attrs.get("schema_version", 0))
    if version != SCHEMA_VERSION:
        raise ValueError(
            f"HDF5 session has schema_version={version}; this build expects {SCHEMA_VERSION}. "
            "Files produced by the pre-V2 EpisodeRecorder are not compatible."
        )
    if MANIFEST_GROUP not in h5_file:
        raise ValueError(f"HDF5 session is missing required '/{MANIFEST_GROUP}' group.")
    grp = h5_file[MANIFEST_GROUP]
    for name in _MANIFEST_DATASETS:
        if name not in grp:
            raise ValueError(f"HDF5 session manifest is missing '/{MANIFEST_GROUP}/{name}'.")

    def _load(name: str) -> Any:
        raw = grp[name][()]
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8")
        return json.loads(raw)

    return SessionManifest(
        tracks=_load("tracks"),
        session=_load("session"),
        sampling=_load("sampling"),
        coord_conventions=_load("coord_conventions"),
    )


def _json_default(obj: Any) -> Any:
    """Coerce common non-JSON types (NumPy, dataclasses, tuples) into JSON-safe ones.

    Args:
        obj: Obj to use.

    Returns:
        JSON-serializable representation of the object.
    """
    try:
        import numpy as np

        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.generic):
            return obj.item()
    except ImportError:
        pass
    if hasattr(obj, "__dataclass_fields__"):
        return asdict(obj)
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable.")
