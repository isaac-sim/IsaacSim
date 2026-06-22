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

"""Simulation state recorders and replayers.

This package houses the manifest-first HDF5 V2 recorder / replayer system:

- :class:`EpisodeRecorder` — orchestrator for sampling a list of :class:`Recordable`
  plugins into an HDF5 session file.
- :class:`EpisodeReplayer` — best-effort, kinematic, visuals-only replay of such
  sessions.
- :class:`Recordable` — the one plugin protocol; schema + sample + apply + manifest
  round-trip.
- Built-in recordables (``recordables/``) for sim time, articulations, xforms, rigid
  bodies, cameras, USD attributes.
"""

from __future__ import annotations

import importlib.util as _importlib_util
import os as _os
import pathlib as _pathlib
from typing import Any


def _preload_h5py_native_dlls() -> None:
    """Preload h5py's native DLLs on Windows so ``import h5py`` succeeds under Kit.

    The h5py Windows wheel ships ``hdf5.dll``, ``hdf5_hl.dll`` and ``z.dll`` inside
    the ``h5py`` package folder next to the ``.pyd`` extension modules. Under Kit's
    embedded Python, when ``_errors.pyd`` triggers transitive resolution of
    ``hdf5.dll``, the resolved DLL search path does not include ``pip_prebundle/h5py``
    and the import fails with ``DLL load failed while importing _errors: The
    specified module could not be found``.

    Locate ``h5py`` via :func:`importlib.util.find_spec` (which resolves through
    Kit's ``[[python.module]]`` registration and therefore finds the built
    ``pip_prebundle`` regardless of whether the extension is loaded from a source
    checkout via junctions / symlinks), then load the native DLLs explicitly with
    :class:`ctypes.WinDLL` in dependency order (``z`` -> ``hdf5`` -> ``hdf5_hl``).
    Once in the process module cache, subsequent import-table resolution inside
    ``_errors.pyd`` matches them by name and the h5py import succeeds.

    Safe to call multiple times; no-op on non-Windows platforms or when h5py is
    unavailable (e.g. in a source checkout without a built prebundle).
    """
    if _os.name != "nt":
        return
    try:
        spec = _importlib_util.find_spec("h5py")
    except (ImportError, ValueError):
        return
    if spec is None or not spec.origin:
        return
    h5py_pkg = _pathlib.Path(spec.origin).resolve().parent
    if not h5py_pkg.is_dir():
        return
    add_dll_directory = getattr(_os, "add_dll_directory", None)
    if add_dll_directory is not None:
        try:
            add_dll_directory(str(h5py_pkg))
        except OSError:
            pass
    import ctypes

    for dll_name in ("z.dll", "hdf5.dll", "hdf5_hl.dll"):
        dll_path = h5py_pkg / dll_name
        if not dll_path.is_file():
            continue
        try:
            ctypes.WinDLL(str(dll_path))
        except OSError:
            pass


_preload_h5py_native_dlls()


def require_h5py() -> Any:
    """Import ``h5py`` with a user-friendly error message if unavailable.

    Returns:
        Imported h5py module.
    """
    try:
        import h5py
    except ImportError as exc:
        raise ImportError(
            "EpisodeRecorder / EpisodeReplayer require h5py and its native DLLs. "
            f"Underlying error: {exc!r}. Install it via pip "
            "(e.g. `./_repo/python -m pip install h5py`) or ensure the extension's "
            "packaged `pip_prebundle/h5py` (with sibling ``hdf5.dll``, ``hdf5_hl.dll``, "
            "``z.dll`` on Windows) is available."
        ) from exc
    return h5py


from ._pose_backend import PoseBackend
from .base import ChannelDescriptor, Recordable, ReplayPolicy, SamplingConfig
from .commands import (
    EPISODE_BINDING_EVENT,
    EPISODE_CMD_EVENT,
    VALID_BINDING_ACTIONS,
    VALID_COMMANDS,
    dispatch_episode_binding,
    dispatch_episode_command,
)
from .manifest import SCHEMA_VERSION, SessionManifest, build_manifest, read_manifest, write_manifest
from .recordables import (
    ArticulationRecordable,
    AttributeRecordable,
    CameraRecordable,
    RigidBodyRecordable,
    SimTimeRecordable,
    XformRecordable,
)
from .recorder import EpisodeRecorder, SessionEvents
from .registry import get_registered, register_recordable, registered_types, rehydrate, unregister_recordable
from .replayer import EpisodeReplayer
from .session_injectors import (
    SessionInjector,
    apply_session_injectors,
    clear_session_injectors,
    register_session_injector,
    registered_session_injectors,
    unregister_session_injector,
)
from .stage_snapshot import STAGE_SNAPSHOT_BASENAME, export_stage_snapshot
from .storage import DEFAULT_BUFFER_FRAMES, SessionReader, SessionStorage
from .timeline_controller import TimelineDrivenEpisodeController

__all__ = [
    "ArticulationRecordable",
    "AttributeRecordable",
    "CameraRecordable",
    "ChannelDescriptor",
    "DEFAULT_BUFFER_FRAMES",
    "EPISODE_BINDING_EVENT",
    "EPISODE_CMD_EVENT",
    "EpisodeRecorder",
    "EpisodeReplayer",
    "PoseBackend",
    "Recordable",
    "ReplayPolicy",
    "RigidBodyRecordable",
    "STAGE_SNAPSHOT_BASENAME",
    "SCHEMA_VERSION",
    "SamplingConfig",
    "SessionEvents",
    "SessionInjector",
    "SessionManifest",
    "SessionReader",
    "SessionStorage",
    "SimTimeRecordable",
    "TimelineDrivenEpisodeController",
    "VALID_BINDING_ACTIONS",
    "VALID_COMMANDS",
    "XformRecordable",
    "apply_session_injectors",
    "build_manifest",
    "clear_session_injectors",
    "dispatch_episode_binding",
    "dispatch_episode_command",
    "export_stage_snapshot",
    "get_registered",
    "read_manifest",
    "register_recordable",
    "register_session_injector",
    "registered_session_injectors",
    "registered_types",
    "rehydrate",
    "require_h5py",
    "unregister_recordable",
    "unregister_session_injector",
    "write_manifest",
]
