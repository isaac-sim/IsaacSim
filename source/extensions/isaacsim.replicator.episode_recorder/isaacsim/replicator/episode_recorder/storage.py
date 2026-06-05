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

"""HDF5 V2 session storage: buffered per-episode dataset writes and per-frame reads.

Layout::

    <file>.hdf5
    ├── @schema_version = 2
    ├── @created_at     (ISO-8601 UTC)
    ├── @tool_version   (optional)
    │
    ├── manifest/       (JSON blobs; written via manifest.write_manifest)
    │
    └── episodes/
        ├── episode_00000/
        │   ├── @episode_index
        │   ├── @started_at
        │   ├── @ended_at
        │   ├── @num_frames
        │   ├── @success     (optional)
        │   ├── @user_metadata (optional JSON blob)
        │   │
        │   └── <recordable.group>/<channel>   (N, *channel.shape) datasets
        │
        └── episode_00001/ ...

All writes are buffered: every channel keeps an in-memory NumPy buffer of size
``buffer_frames``; when full, the buffer is appended to the resizable HDF5 dataset in
one contiguous write. ``flush`` / ``end_episode`` / ``close`` force a flush.
"""

from __future__ import annotations

import json
import warnings
from collections.abc import Mapping
from datetime import datetime, timezone
from typing import Any

import carb
import numpy as np

from .base import ChannelDescriptor
from .manifest import SCHEMA_VERSION, SessionManifest, read_manifest, write_manifest

DEFAULT_BUFFER_FRAMES = 1024
EPISODES_GROUP = "episodes"


def _require_h5py() -> Any:
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message=r"h5py is running against HDF5 .* when it was built against .*",
                category=UserWarning,
                module=r"h5py",
            )
            import h5py

        return h5py
    except ImportError as exc:
        raise ImportError(
            "HDF5 session storage requires h5py and its native DLLs. "
            f"Underlying error: {exc!r}. Install via pip "
            "(e.g. `./_repo/python -m pip install h5py`) or ensure the extension's "
            "packaged `pip_prebundle/h5py` (with sibling ``hdf5.dll``, ``hdf5_hl.dll``, "
            "``z.dll`` on Windows) is available."
        ) from exc


def _coerce_attr(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool, np.ndarray)):
        return value
    return json.dumps(value, default=str)


def _h5_string_dtype() -> Any:
    h5py = _require_h5py()
    return h5py.string_dtype(encoding="utf-8")


class SessionStorage:
    """Write-side HDF5 session. One instance per session; holds one open file.

    Public API:
        - :meth:`open` — create the file and set root attrs.
        - :meth:`write_manifest` — write ``/manifest/*`` (call once, after ``open``).
        - :meth:`begin_episode` — create an episode group + per-channel datasets.
        - :meth:`append_frame` — add one frame for one recordable group (buffered).
        - :meth:`end_episode` — flush + trim to real size + write episode attrs.
        - :meth:`flush` — flush all buffers without ending the episode.
        - :meth:`close` — flush + close the file.

    Thread-safety: buffered append / flush / end are **not** internally synchronized.
    Callers (e.g. :class:`EpisodeRecorder`) must serialize access with their own lock.
    """

    def __init__(self, h5_path: str, *, buffer_frames: int = DEFAULT_BUFFER_FRAMES) -> None:
        if buffer_frames < 1:
            raise ValueError(f"buffer_frames must be >= 1, got {buffer_frames}.")
        self._h5_path = h5_path
        self._buffer_frames = int(buffer_frames)
        self._h5: Any = None
        self._episode_group: Any = None
        self._episode_index: int = 0
        self._num_episodes: int = 0
        # Per-(group, channel) resizable datasets.
        self._datasets: dict[tuple[str, str], Any] = {}
        # Resizable datasets grouped in channel order for faster flushes.
        self._datasets_by_group: dict[str, tuple[Any, ...]] = {}
        # Per-(group, channel) in-memory buffers.
        self._buffers: dict[tuple[str, str], np.ndarray] = {}
        # Ordered channel names per recordable group.
        self._channels_by_group: dict[str, tuple[str, ...]] = {}
        # Current buffer fill count per recordable group (all channels in a group share it).
        self._buffer_count: dict[str, int] = {}
        # Per-recordable-group row count actually written (buffer + on-disk).
        self._frames_written: dict[str, int] = {}
        # Pre-bound per-group tick plan: parallel tuples of (channels, buffers, expected shapes).
        # Populated by :meth:`begin_episode`; consumed by :meth:`append_frame` to avoid per-tick
        # dict lookups and schema validation.
        self._group_plans: dict[str, tuple[tuple[str, ...], tuple[np.ndarray, ...], tuple[tuple[int, ...], ...]]] = {}
        # Episode-level frame counter (matches the driver's tick count).
        self._episode_frames: int = 0
        self._episode_started_at: str | None = None

    # --- lifecycle -----------------------------------------------------

    @property
    def path(self) -> str:
        return self._h5_path

    @property
    def is_open(self) -> bool:
        return self._h5 is not None

    @property
    def current_episode_frames(self) -> int:
        return self._episode_frames

    @property
    def num_episodes_finalized(self) -> int:
        return self._num_episodes

    def open(self) -> None:
        """Create the session HDF5 file in write mode and set root attrs."""
        if self._h5 is not None:
            raise RuntimeError(f"SessionStorage already open: {self._h5_path}")
        h5py = _require_h5py()
        self._h5 = h5py.File(self._h5_path, "w")
        self._h5.attrs["schema_version"] = SCHEMA_VERSION
        self._h5.attrs["created_at"] = datetime.now(timezone.utc).isoformat()
        self._h5.require_group(EPISODES_GROUP)

    def write_manifest(self, manifest: SessionManifest) -> None:
        self._require_open()
        write_manifest(self._h5, manifest)
        self._h5.flush()

    def set_root_attr(self, key: str, value: Any) -> None:
        """Write a top-level HDF5 attribute (e.g. ``tool_version``, ``session_id``)."""
        self._require_open()
        self._h5.attrs[key] = _coerce_attr(value)

    def close(self) -> None:
        """Flush and close. Idempotent."""
        if self._h5 is None:
            return
        if self._episode_group is not None:
            self.end_episode(success=None)
        try:
            self._h5.attrs["num_episodes"] = self._num_episodes
            self._h5.flush()
            self._h5.close()
        finally:
            self._h5 = None
            self._datasets.clear()
            self._datasets_by_group.clear()
            self._buffers.clear()
            self._channels_by_group.clear()
            self._buffer_count.clear()
            self._frames_written.clear()
            self._group_plans.clear()

    # --- episodes ------------------------------------------------------

    def begin_episode(
        self,
        channel_schemas: Mapping[str, Mapping[str, ChannelDescriptor]],
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> int:
        """Create an episode group and preallocate resizable datasets for every channel.

        Args:
            channel_schemas: ``{recordable_group: {channel_name: ChannelDescriptor}}``.
            metadata: Optional dict written as JSON on the episode group.

        Returns:
            Zero-based episode index.
        """
        self._require_open()
        if self._episode_group is not None:
            raise RuntimeError("An episode is already active; end_episode() first.")

        episode_name = f"episode_{self._episode_index:05d}"
        grp = self._h5[EPISODES_GROUP].create_group(episode_name)
        grp.attrs["episode_index"] = self._episode_index
        grp.attrs["started_at"] = datetime.now(timezone.utc).isoformat()
        if metadata:
            grp.attrs["user_metadata"] = json.dumps(dict(metadata), default=str)

        self._episode_group = grp
        self._episode_started_at = grp.attrs["started_at"]
        self._episode_frames = 0
        self._datasets.clear()
        self._datasets_by_group.clear()
        self._buffers.clear()
        self._channels_by_group.clear()
        self._buffer_count.clear()
        self._frames_written.clear()
        self._group_plans.clear()

        for rec_group, channels in channel_schemas.items():
            self._frames_written[rec_group] = 0
            self._buffer_count[rec_group] = 0
            chan_names = tuple(channels.keys())
            self._channels_by_group[rec_group] = chan_names
            target_grp = grp.require_group(rec_group)
            plan_datasets: list[Any] = []
            plan_buffers: list[np.ndarray] = []
            plan_shapes: list[tuple[int, ...]] = []
            for chan_name, desc in channels.items():
                ds_name = chan_name
                shape_full = (0,) + tuple(desc.shape)
                max_shape = (None,) + tuple(desc.shape)
                chunks = (self._buffer_frames,) + tuple(desc.shape) if desc.shape else (self._buffer_frames,)
                ds = target_grp.create_dataset(
                    ds_name,
                    shape=shape_full,
                    maxshape=max_shape,
                    chunks=chunks,
                    dtype=desc.dtype,
                )
                for attr_key, attr_value in {
                    "units": desc.units,
                    "space": desc.space,
                    "quaternion_order": desc.quaternion_order,
                }.items():
                    if attr_value is not None:
                        ds.attrs[attr_key] = attr_value
                for k, v in (desc.attrs or {}).items():
                    ds.attrs[k] = _coerce_attr(v)
                self._datasets[(rec_group, chan_name)] = ds
                plan_datasets.append(ds)
                buf_shape = (self._buffer_frames,) + tuple(desc.shape)
                buf = np.zeros(buf_shape, dtype=desc.dtype)
                self._buffers[(rec_group, chan_name)] = buf
                plan_buffers.append(buf)
                plan_shapes.append(tuple(desc.shape))
            self._datasets_by_group[rec_group] = tuple(plan_datasets)
            self._group_plans[rec_group] = (chan_names, tuple(plan_buffers), tuple(plan_shapes))

        return self._episode_index

    def append_frame(self, recordable_group: str, frame: Mapping[str, np.ndarray | float | int]) -> None:
        """Append one frame for one recordable. Flushes the group's buffer when full.

        Uses the per-episode tick plan pre-bound in :meth:`begin_episode` for the hot
        path: no per-channel dataset lookups, no set-based channel validation, no
        ``np.asarray`` allocation for value ingress. ``frame`` must provide every
        channel declared for ``recordable_group``; missing keys raise :class:`KeyError`
        naturally at the dict access, and shape mismatches raise at the buffer slice
        assignment. Extra keys are tolerated and ignored — they do not affect stored data.
        """
        plan = self._group_plans.get(recordable_group)
        if plan is None:
            if self._episode_group is None:
                raise RuntimeError("No active episode; call begin_episode() first.")
            raise KeyError(f"No channel schema for recordable group {recordable_group!r}.")
        channels, buffers, _shapes = plan
        idx = self._buffer_count[recordable_group]
        for i, chan_name in enumerate(channels):
            buffers[i][idx] = frame[chan_name]
        idx += 1
        self._buffer_count[recordable_group] = idx
        if idx >= self._buffer_frames:
            self._flush_group(recordable_group)

    def advance_episode_frame(self) -> None:
        """Bump the episode-level frame counter. Called once per tick after all recordables sampled."""
        self._episode_frames += 1

    def flush(self) -> None:
        """Flush all per-group buffers to disk and ``h5.flush()`` the file."""
        self._require_open()
        for rec_group in list(self._buffer_count):
            self._flush_group(rec_group)
        if self._h5 is not None:
            self._h5.flush()

    def _flush_group(self, rec_group: str) -> None:
        count = self._buffer_count.get(rec_group, 0)
        if count == 0:
            return
        plan = self._group_plans[rec_group]
        _channels, buffers, _shapes = plan
        datasets = self._datasets_by_group.get(rec_group, ())
        # Channel-order invariant: ``_datasets_by_group[rec_group]`` and the buffers tuple
        # in ``_group_plans[rec_group]`` are populated together in :meth:`begin_episode`
        # by iterating ``recordable.channels()`` once. The ``zip(datasets, buffers)``
        # below relies on that pairing - a future change that mutates either tuple
        # independently would silently misroute channel data, so this raises
        # unconditionally rather than via ``assert`` (which strips under ``python -O``).
        if len(datasets) != len(buffers):
            raise RuntimeError(
                f"SessionStorage channel-order invariant violated for group {rec_group!r}: "
                f"{len(datasets)} datasets vs {len(buffers)} buffers. "
                f"`_datasets_by_group` and `_group_plans` must be populated together."
            )
        existing = self._frames_written[rec_group]
        new_total = existing + count
        for ds, buf in zip(datasets, buffers):
            ds.resize((new_total,) + ds.shape[1:])
            ds[existing:new_total] = buf[:count]
        self._frames_written[rec_group] = new_total
        self._buffer_count[rec_group] = 0

    def end_episode(self, *, success: bool | None = None, metadata: Mapping[str, Any] | None = None) -> None:
        """Flush buffers, trim datasets to real row counts, and write episode attrs."""
        if self._h5 is None:
            return
        if self._episode_group is None:
            return
        for rec_group in list(self._buffer_count):
            self._flush_group(rec_group)
        grp = self._episode_group
        final_num_frames = min(self._frames_written.values(), default=self._episode_frames)
        if final_num_frames != self._episode_frames:
            per_group = ", ".join(f"{g}={n}" for g, n in sorted(self._frames_written.items()))
            carb.log_warn(
                "SessionStorage detected divergent per-group frame counts; "
                f"clamping episode length from {self._episode_frames} to {final_num_frames}. "
                f"Per-group frames written: {{{per_group}}}."
            )
        for ds in self._datasets.values():
            if ds.shape[0] > final_num_frames:
                ds.resize((final_num_frames,) + ds.shape[1:])
        grp.attrs["ended_at"] = datetime.now(timezone.utc).isoformat()
        grp.attrs["num_frames"] = final_num_frames
        if success is not None:
            grp.attrs["success"] = bool(success)
        if metadata:
            existing = grp.attrs.get("user_metadata")
            if existing is not None:
                try:
                    merged = {**json.loads(existing), **dict(metadata)}
                except Exception:
                    merged = dict(metadata)
            else:
                merged = dict(metadata)
            grp.attrs["user_metadata"] = json.dumps(merged, default=str)
        self._h5.flush()
        self._episode_group = None
        self._episode_index += 1
        self._num_episodes = self._episode_index
        self._h5.attrs["num_episodes"] = self._num_episodes
        self._episode_started_at = None
        self._episode_frames = 0
        self._datasets.clear()
        self._datasets_by_group.clear()
        self._buffers.clear()
        self._channels_by_group.clear()
        self._buffer_count.clear()
        self._frames_written.clear()
        self._group_plans.clear()

    # --- util ----------------------------------------------------------

    def _require_open(self) -> None:
        if self._h5 is None:
            raise RuntimeError(f"SessionStorage is not open: {self._h5_path}")


class SessionReader:
    """Read-side HDF5 session. Opened read-only; supports random frame access.

    Args:
        h5_path: Path to an existing HDF5 session file (schema_version 2).
    """

    def __init__(self, h5_path: str) -> None:
        h5py = _require_h5py()
        self._h5_path = h5_path
        self._h5 = h5py.File(h5_path, "r")
        try:
            self._manifest = read_manifest(self._h5)
        except Exception:
            self._h5.close()
            raise

    @property
    def path(self) -> str:
        return self._h5_path

    def manifest(self) -> SessionManifest:
        return self._manifest

    def list_episodes(self) -> list[str]:
        if EPISODES_GROUP not in self._h5:
            return []
        return sorted(self._h5[EPISODES_GROUP].keys())

    def normalize_episode(self, episode: int | str) -> str:
        episodes = self.list_episodes()
        if isinstance(episode, int):
            if episode < 0 or episode >= len(episodes):
                raise IndexError(f"Episode index {episode} out of range (have {len(episodes)}).")
            return episodes[episode]
        if episode in episodes:
            return episode
        raise KeyError(f"Unknown episode {episode!r}. Available: {episodes}.")

    def num_frames(self, episode: int | str) -> int:
        name = self.normalize_episode(episode)
        return int(self._h5[EPISODES_GROUP][name].attrs.get("num_frames", 0))

    def episode_attrs(self, episode: int | str) -> dict[str, Any]:
        name = self.normalize_episode(episode)
        attrs = dict(self._h5[EPISODES_GROUP][name].attrs)
        user_md = attrs.pop("user_metadata", None)
        if user_md is not None:
            try:
                attrs["user_metadata"] = json.loads(user_md)
            except Exception:
                attrs["user_metadata"] = user_md
        return attrs

    def read_frame(self, episode: int | str, recordable_group: str, frame_index: int) -> dict[str, np.ndarray]:
        """Read one frame of one recordable group into a channel-name-keyed dict."""
        name = self.normalize_episode(episode)
        ep = self._h5[EPISODES_GROUP][name]
        if recordable_group not in ep:
            raise KeyError(f"Episode {name} missing group {recordable_group!r}.")
        grp = ep[recordable_group]
        out: dict[str, np.ndarray] = {}
        n = int(ep.attrs.get("num_frames", 0))
        if frame_index < 0 or frame_index >= n:
            raise IndexError(f"frame_index {frame_index} out of range [0, {n}).")
        for chan in grp.keys():
            ds = grp[chan]
            out[chan] = np.asarray(ds[frame_index])
        return out

    def read_channel(self, episode: int | str, recordable_group: str, channel: str) -> np.ndarray:
        """Read the full time-series for one channel as a contiguous array."""
        name = self.normalize_episode(episode)
        ds = self._h5[EPISODES_GROUP][name][recordable_group][channel]
        return np.asarray(ds[:])

    def read_group_all_frames(self, episode: int | str, recordable_group: str) -> dict[str, np.ndarray]:
        """Read every channel of ``recordable_group`` as full time-series arrays.

        Each value has shape ``(num_frames, ...)``. Lets callers prefetch an entire
        episode into RAM once and index into the arrays per-frame instead of issuing
        thousands of small HDF5 slab reads during replay.
        """
        name = self.normalize_episode(episode)
        ep = self._h5[EPISODES_GROUP][name]
        if recordable_group not in ep:
            raise KeyError(f"Episode {name} missing group {recordable_group!r}.")
        grp = ep[recordable_group]
        return {chan: np.asarray(grp[chan][:]) for chan in grp.keys()}

    def close(self) -> None:
        if self._h5 is not None:
            self._h5.close()
            self._h5 = None

    def __enter__(self) -> SessionReader:
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()
