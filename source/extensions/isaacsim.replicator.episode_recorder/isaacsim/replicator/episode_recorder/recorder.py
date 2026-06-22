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

"""Manifest-first :class:`EpisodeRecorder` orchestrator.

Owns the HDF5 session file, the session / episode state machine, the sampling tick
subscription, and the per-session command bus. Holds a list of :class:`Recordable`
plugins; has no knowledge of articulations / xforms / cameras / teleop.

Threading:
    Sampling is expected to arrive on Kit's main thread (post-physics-step or app
    update). State transitions and storage writes are serialized with an internal
    :class:`threading.Lock` so external callers (e.g. UI buttons) may safely invoke
    :meth:`start_episode` / :meth:`end_episode` from threads other than the sampler.
"""

from __future__ import annotations

import enum
import os
import threading
import uuid
from collections.abc import Callable
from datetime import datetime, timezone
from typing import Any

import carb
import carb.eventdispatcher
import numpy as np

from ._pose_backend import PoseBackend, normalize_pose_backend, pose_backend_ctx
from .base import Recordable, SamplingConfig
from .commands import EPISODE_CMD_EVENT, VALID_COMMANDS
from .manifest import build_manifest
from .recordables._utils import to_numpy_f32
from .recordables.sim_time import SimTimeRecordable
from .stage_snapshot import STAGE_SNAPSHOT_BASENAME, export_stage_snapshot
from .storage import SessionStorage


class _State(enum.Enum):
    IDLE = "idle"
    SESSION_OPEN = "session_open"
    EPISODE_ACTIVE = "episode_active"
    CLOSED = "closed"


class SessionEvents:
    """Thin observer API for UI panels that need to reflect recorder state.

    Callers use :meth:`add_session_opened` / :meth:`add_episode_started` etc. and receive
    a cancel token. Callbacks are invoked synchronously from the recorder thread that
    triggered the transition.
    """

    def __init__(self) -> None:
        self._session_opened: list[Callable[[], None]] = []
        self._session_closed: list[Callable[[], None]] = []
        self._episode_started: list[Callable[[int], None]] = []
        self._episode_ended: list[Callable[[int, bool | None, int], None]] = []
        self._paused: list[Callable[[], None]] = []
        self._resumed: list[Callable[[], None]] = []

    def add_session_opened(self, cb: Callable[[], None]) -> Callable[[], None]:
        """Add the session opened callback.

        Args:
            cb: Cb to use.

        Returns:
            add session opened result.
        """
        return _subscribe(self._session_opened, cb)

    def add_session_closed(self, cb: Callable[[], None]) -> Callable[[], None]:
        """Add the session closed callback.

        Args:
            cb: Cb to use.

        Returns:
            add session closed result.
        """
        return _subscribe(self._session_closed, cb)

    def add_episode_started(self, cb: Callable[[int], None]) -> Callable[[], None]:
        """Add the episode started callback.

        Args:
            cb: Cb to use.

        Returns:
            add episode started result.
        """
        return _subscribe(self._episode_started, cb)

    def add_episode_ended(self, cb: Callable[[int, bool | None, int], None]) -> Callable[[], None]:
        """Add the episode ended callback.

        Args:
            cb: Cb to use.

        Returns:
            add episode ended result.
        """
        return _subscribe(self._episode_ended, cb)

    def add_paused(self, cb: Callable[[], None]) -> Callable[[], None]:
        """Add the paused callback.

        Args:
            cb: Cb to use.

        Returns:
            add paused result.
        """
        return _subscribe(self._paused, cb)

    def add_resumed(self, cb: Callable[[], None]) -> Callable[[], None]:
        """Add the resumed callback.

        Args:
            cb: Cb to use.

        Returns:
            add resumed result.
        """
        return _subscribe(self._resumed, cb)

    def _fire_session_opened(self) -> None:
        _fire(self._session_opened)

    def _fire_session_closed(self) -> None:
        _fire(self._session_closed)

    def _fire_episode_started(self, idx: int) -> None:
        _fire(self._episode_started, idx)

    def _fire_episode_ended(self, idx: int, success: bool | None, frames: int) -> None:
        _fire(self._episode_ended, idx, success, frames)

    def _fire_paused(self) -> None:
        _fire(self._paused)

    def _fire_resumed(self) -> None:
        _fire(self._resumed)


def _subscribe(bucket: list, cb: Callable) -> Callable[[], None]:
    bucket.append(cb)

    def unsubscribe() -> None:
        try:
            bucket.remove(cb)
        except ValueError:
            pass

    return unsubscribe


def _fire(bucket: list, *args: Any) -> None:
    for cb in list(bucket):
        try:
            cb(*args)
        except Exception as exc:
            carb.log_error(f"[SessionEvents] listener raised: {exc}")


class EpisodeRecorder:
    """Orchestrator for sampling :class:`Recordable` plugins into an HDF5 session.

    Lifecycle::

        IDLE -> SESSION_OPEN -> EPISODE_ACTIVE -> SESSION_OPEN -> CLOSED

    :meth:`add` is permitted only in ``IDLE`` (before :meth:`open_session`). After
    :meth:`open_session` the manifest is frozen.

    Args:
        output_dir: Directory for the session HDF5 file and any stage snapshot sidecar.
        file_prefix: Filename prefix; full name is ``{prefix}_{ISO-UTC-timestamp}.hdf5``.
        sampling: :class:`SamplingConfig` controlling the tick source and decimation.
        session_metadata: Extra session attributes persisted in the manifest.
        session_id: Opaque id used by :func:`dispatch_episode_command` filtering. Auto-
            generated UUID when omitted.
        buffer_frames: Per-channel buffer size before flushing to disk. Defaults to
            ``1024`` — with pose-only frames this keeps flushes infrequent (< 10 Hz at
            120 Hz sampling) so the main thread is not stalled writing HDF5 during a
            recording.
        auto_attach_sim_time: When ``True`` (default), :class:`SimTimeRecordable` is
            auto-added at :meth:`open_session`.
        link_stage_snapshot: When ``True`` (default), if an
            ``<output_dir>/stage_snapshot.usd`` already exists, link it in the root attrs.
        pose_backend: Backend used by the shared pose-batch read each tick. Defaults to
            ``"usd"`` for correctness with nested xform hierarchies; ``"usdrt"`` /
            ``"fabric"`` are accepted when Fabric Scene Delegate is enabled and trade
            correctness for speed on flat scenes.
    """

    def __init__(
        self,
        output_dir: str,
        *,
        file_prefix: str = "episode",
        sampling: SamplingConfig | None = None,
        session_metadata: dict[str, Any] | None = None,
        session_id: str | None = None,
        buffer_frames: int = 1024,
        auto_attach_sim_time: bool = True,
        link_stage_snapshot: bool = True,
        pose_backend: PoseBackend = "usd",
    ) -> None:
        self._output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self._file_prefix = file_prefix
        self._sampling = sampling or SamplingConfig()
        self._session_metadata = dict(session_metadata or {})
        self._session_id: str = session_id if session_id is not None else f"session_{uuid.uuid4().hex[:12]}"
        self._buffer_frames = int(buffer_frames)
        self._auto_attach_sim_time = bool(auto_attach_sim_time)
        self._link_stage_snapshot = bool(link_stage_snapshot)
        self._pose_backend: PoseBackend = normalize_pose_backend(pose_backend)

        self._state: _State = _State.IDLE
        self._lock = threading.RLock()
        self._recordables: list[Recordable] = []
        self._storage: SessionStorage | None = None
        self._stage: Any = None

        self._episode_index: int = 0
        self._episode_frames_this: int = 0
        self._paused: bool = False
        self._tick_counter: int = 0

        self._sim_callback_ids: list[int] = []
        self._app_update_sub: Any = None
        self._command_sub: Any = None

        self._events = SessionEvents()

        # Shared pose batch: one ``XformPrim`` covers every pose-batchable recordable's
        # prim paths. Each tick samples all world poses in a single Fabric call and
        # distributes slices to the participating recordables.
        self._pose_batch: Any = None
        # ``id(recordable) -> (start, end)`` — recordables absent from this map are
        # sampled via :meth:`Recordable.sample` each tick. Rebuilt at every
        # :meth:`open_session` and cleared on :meth:`close_session`.
        self._pose_batch_slot_by_id: dict[int, tuple[int, int]] = {}
        # Total number of pose-batched recordables, kept for the log line.
        self._pose_batch_rec_count: int = 0

    # ------------------------------------------------------------------ properties
    @property
    def session_id(self) -> str:
        """Run the session id operation."""
        return self._session_id

    @property
    def output_dir(self) -> str:
        """Run the output dir operation."""
        return self._output_dir

    @property
    def hdf5_path(self) -> str | None:
        """Run the hdf5 path operation."""
        return self._storage.path if self._storage is not None else None

    @property
    def is_session_open(self) -> bool:
        """Return whether session open."""
        return self._state in (_State.SESSION_OPEN, _State.EPISODE_ACTIVE)

    @property
    def is_recording(self) -> bool:
        """Return whether recording."""
        return self._state is _State.EPISODE_ACTIVE and not self._paused

    @property
    def is_paused(self) -> bool:
        """Return whether paused."""
        return self._state is _State.EPISODE_ACTIVE and self._paused

    @property
    def current_episode_frames(self) -> int:
        """Run the current episode frames operation."""
        return self._episode_frames_this if self._state is _State.EPISODE_ACTIVE else 0

    @property
    def events(self) -> SessionEvents:
        """Run the events operation."""
        return self._events

    @property
    def state(self) -> str:
        """Run the state operation."""
        return self._state.value

    @property
    def pose_backend(self) -> PoseBackend:
        """Active backend for the shared pose-batch read (``"usd"`` / ``"usdrt"`` / ``"fabric"``)."""
        return self._pose_backend

    # ------------------------------------------------------------------ plugin mgmt
    def add(self, recordable: Recordable) -> None:
        """Register a :class:`Recordable`. Must be called before :meth:`open_session`.

        Args:
            recordable: Recordable to use.
        """
        with self._lock:
            if self._state is not _State.IDLE:
                raise RuntimeError(
                    f"add() is only allowed before open_session(); current state is '{self._state.value}'."
                )
            for existing in self._recordables:
                if existing.group == recordable.group:
                    raise ValueError(
                        f"Duplicate recordable group {recordable.group!r}; another plugin already claims it."
                    )
            self._recordables.append(recordable)

    def recordables(self) -> list[Recordable]:
        """Return a snapshot of registered recordables (defensive copy).

        Returns:
            recordables result.
        """
        with self._lock:
            return list(self._recordables)

    # ------------------------------------------------------------------ session
    def open_session(self, output_path: str | None = None) -> str:
        """Create the HDF5 file, write the manifest, subscribe to sampling + commands.

        Args:
            output_path: Explicit file path (``.hdf5``). If omitted, a timestamped file
                is created inside ``output_dir``.

        Returns:
            Absolute path of the session HDF5 file.
        """
        with self._lock:
            if self._state is not _State.IDLE:
                raise RuntimeError(f"open_session() requires IDLE state; current state is '{self._state.value}'.")

            if self._auto_attach_sim_time and not any(r.TYPE_ID == "sim_time" for r in self._recordables):
                self._recordables.insert(0, SimTimeRecordable())

            if output_path is None:
                os.makedirs(self._output_dir, exist_ok=True)
                ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                output_path = os.path.join(self._output_dir, f"{self._file_prefix}_{ts}.hdf5")
                output_path = os.path.abspath(os.path.expanduser(output_path))
            else:
                output_path = os.path.abspath(os.path.expanduser(output_path))
                os.makedirs(os.path.dirname(output_path), exist_ok=True)

            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("open_session: no USD stage is currently loaded.")
            self._stage = stage

            storage = SessionStorage(output_path, buffer_frames=self._buffer_frames)
            storage.open()
            storage.set_root_attr("session_id", self._session_id)
            storage.set_root_attr("created_at", datetime.now(timezone.utc).isoformat())
            for key, value in self._session_metadata.items():
                storage.set_root_attr(key, value)

            if self._link_stage_snapshot:
                self._try_link_stage_snapshot(storage)

            opened: list[Recordable] = []
            for rec in list(self._recordables):
                try:
                    rec.on_session_open(stage)
                except Exception as exc:
                    for prior in opened:
                        try:
                            prior.on_session_close()
                        except Exception as close_exc:
                            carb.log_warn(
                                f"[EpisodeRecorder] on_session_close during rollback raised for "
                                f"{prior.group}: {close_exc}"
                            )
                    storage.close()
                    self._state = _State.IDLE
                    self._stage = None
                    raise RuntimeError(
                        f"[EpisodeRecorder] on_session_open failed for {rec.group} ({rec.TYPE_ID}): {exc}"
                    ) from exc
                opened.append(rec)

            manifest_entries = [rec.to_manifest() for rec in self._recordables]
            manifest = build_manifest(
                manifest_entries,
                sampling={"mode": self._sampling.mode, "decimation": self._sampling.decimation},
                session_metadata=self._session_metadata,
            )
            storage.write_manifest(manifest)

            self._storage = storage
            self._state = _State.SESSION_OPEN
            self._episode_index = 0

            self._build_pose_batch()

            self._subscribe_sampling()
            self._subscribe_command_bus()

            carb.log_info(f"[EpisodeRecorder] Session opened: {output_path} (session_id={self._session_id})")
            self._events._fire_session_opened()
            return output_path

    def close_session(self) -> None:
        """End the active episode (if any), unsubscribe, flush and close the HDF5 file.

        All teardown steps are guarded so a misbehaving recordable or storage flush
        still unsubscribes sampling/command bus and closes the HDF5 file.
        """
        with self._lock:
            if self._state is _State.IDLE or self._state is _State.CLOSED:
                self._state = _State.CLOSED
                return
            try:
                if self._state is _State.EPISODE_ACTIVE:
                    try:
                        self._end_episode_locked(success=None, metadata=None)
                    except Exception as exc:
                        carb.log_error(f"[EpisodeRecorder] end_episode during close raised: {exc}")
                self._unsubscribe_sampling()
                self._unsubscribe_command_bus()
                self._teardown_pose_batch()
                for rec in list(self._recordables):
                    try:
                        rec.on_session_close()
                    except Exception as exc:
                        carb.log_warn(f"[EpisodeRecorder] on_session_close raised for {rec.group}: {exc}")
                if self._storage is not None:
                    try:
                        self._storage.close()
                        carb.log_info(
                            f"[EpisodeRecorder] Session closed: {self._storage.path} "
                            f"({self._storage.num_episodes_finalized} episodes)"
                        )
                    except Exception as exc:
                        carb.log_error(f"[EpisodeRecorder] storage.close raised: {exc}")
                    self._storage = None
            finally:
                self._stage = None
                self._state = _State.CLOSED
                self._events._fire_session_closed()

    def destroy(self) -> None:
        """Alias for :meth:`close_session`. Safe to call multiple times."""
        with self._lock:
            if self._state is not _State.CLOSED:
                self.close_session()

    # ------------------------------------------------------------------ episodes
    def start_episode(self, metadata: dict[str, Any] | None = None) -> int:
        """Begin a new episode. If one is already active, it is auto-ended first.

        Args:
            metadata: Episode metadata to store.

        Returns:
            start episode result.
        """
        with self._lock:
            if self._state is _State.IDLE or self._state is _State.CLOSED:
                raise RuntimeError(f"start_episode() requires an open session; current state is '{self._state.value}'.")
            if self._state is _State.EPISODE_ACTIVE:
                carb.log_warn("[EpisodeRecorder] start_episode called with active episode; auto-ending current one.")
                self._end_episode_locked(success=None, metadata=None)

            assert self._storage is not None
            channel_schemas = {rec.group: rec.describe_channels() for rec in self._recordables}
            episode_index = self._storage.begin_episode(channel_schemas, metadata=metadata)
            for rec in list(self._recordables):
                try:
                    rec.on_episode_start()
                except Exception as exc:
                    carb.log_warn(f"[EpisodeRecorder] on_episode_start raised for {rec.group}: {exc}")
            self._episode_index = episode_index
            self._episode_frames_this = 0
            self._paused = False
            self._tick_counter = 0
            self._state = _State.EPISODE_ACTIVE
            carb.log_info(f"[EpisodeRecorder] Episode started: episode_{episode_index:05d}")
            self._events._fire_episode_started(episode_index)
            return episode_index

    def end_episode(self, *, success: bool | None = None, metadata: dict[str, Any] | None = None) -> None:
        """End the active episode and trim datasets to their real length.

        Args:
            success: Episode success flag to store.
            metadata: Episode metadata to store.
        """
        with self._lock:
            self._end_episode_locked(success=success, metadata=metadata)

    def _end_episode_locked(self, *, success: bool | None, metadata: dict[str, Any] | None) -> None:
        if self._state is not _State.EPISODE_ACTIVE:
            carb.log_warn("[EpisodeRecorder] end_episode called with no active episode.")
            return
        assert self._storage is not None
        for rec in list(self._recordables):
            try:
                rec.on_episode_end()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder] on_episode_end raised for {rec.group}: {exc}")
        frames = self._episode_frames_this
        idx = self._episode_index
        self._storage.end_episode(success=success, metadata=metadata)
        self._state = _State.SESSION_OPEN
        self._paused = False
        self._episode_frames_this = 0
        carb.log_info(f"[EpisodeRecorder] Episode ended: episode_{idx:05d} ({frames} frames)")
        self._events._fire_episode_ended(idx, success, frames)

    def pause(self) -> None:
        """Pause the operation."""
        with self._lock:
            if self._state is not _State.EPISODE_ACTIVE:
                carb.log_warn("[EpisodeRecorder] pause called with no active episode.")
                return
            if not self._paused:
                self._paused = True
                self._events._fire_paused()

    def resume(self) -> None:
        """Resume the operation."""
        with self._lock:
            if self._state is not _State.EPISODE_ACTIVE:
                carb.log_warn("[EpisodeRecorder] resume called with no active episode.")
                return
            if self._paused:
                self._paused = False
                self._events._fire_resumed()

    # ------------------------------------------------------------------ stage snapshot
    def export_stage_snapshot(
        self,
        output_dir: str | None = None,
        *,
        basename: str = STAGE_SNAPSHOT_BASENAME,
        flatten: bool = True,
        include_sidecar: bool = True,
    ) -> str:
        """Export the current USD stage as a scene-level snapshot next to the session.

        Args:
            output_dir: Output dir to use.
            basename: Basename to use.
            flatten: Flatten to use.
            include_sidecar: Include sidecar to use.

        Returns:
            export stage snapshot result.
        """
        target_dir = output_dir if output_dir is not None else self._output_dir
        return export_stage_snapshot(target_dir, basename=basename, flatten=flatten, include_sidecar=include_sidecar)

    def _try_link_stage_snapshot(self, storage: SessionStorage) -> None:
        usd_path = os.path.join(self._output_dir, f"{STAGE_SNAPSHOT_BASENAME}.usd")
        if os.path.isfile(usd_path):
            storage.set_root_attr("stage_snapshot", os.path.basename(usd_path))

    # ------------------------------------------------------------------ sampling
    def _subscribe_sampling(self) -> None:
        if self._sampling.mode == "physics_post_step":
            from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

            cb_id = SimulationManager.register_callback(
                self._on_physics_post_step, event=SimulationEvent.PHYSICS_POST_STEP
            )
            self._sim_callback_ids.append(cb_id)
        elif self._sampling.mode == "app_update":
            import omni.kit.app

            dispatcher = carb.eventdispatcher.get_eventdispatcher()
            self._app_update_sub = dispatcher.observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_app_update,
                observer_name=f"EpisodeRecorder.{self._session_id}",
            )

    def _unsubscribe_sampling(self) -> None:
        if self._sim_callback_ids:
            from isaacsim.core.simulation_manager import SimulationManager

            for cb_id in list(self._sim_callback_ids):
                try:
                    SimulationManager.deregister_callback(cb_id)
                except Exception:
                    pass
            self._sim_callback_ids.clear()
        if self._app_update_sub is not None:
            self._app_update_sub = None

    def _on_physics_post_step(self, step_dt: float, context: Any) -> None:
        self._tick()

    def _on_app_update(self, event: Any) -> None:
        self._tick()

    def _tick(self) -> None:
        with self._lock:
            if self._state is not _State.EPISODE_ACTIVE or self._paused:
                return

            do_sample = self._tick_counter % self._sampling.decimation == 0
            self._tick_counter += 1
            if not do_sample:
                return

            assert self._storage is not None

            pos_np: np.ndarray | None = None
            quat_np: np.ndarray | None = None
            if self._pose_batch is not None:
                try:
                    with pose_backend_ctx(self._pose_backend):
                        pos_wp, quat_wp = self._pose_batch.get_world_poses()
                    pos_np = to_numpy_f32(pos_wp)
                    quat_np = to_numpy_f32(quat_wp)
                except Exception as exc:
                    carb.log_warn(
                        f"[EpisodeRecorder] pose batch read failed ({exc}); falling back to "
                        "per-recordable sample() for this tick."
                    )
                    pos_np = None
                    quat_np = None

            slot_by_id = self._pose_batch_slot_by_id
            storage = self._storage

            frames = {}
            for rec in self._recordables:
                try:
                    slot = slot_by_id.get(id(rec)) if pos_np is not None else None
                    if slot is not None:
                        s, e = slot
                        frames[rec.group] = rec.consume_pose_batch(pos_np[s:e], quat_np[s:e])
                    else:
                        frames[rec.group] = rec.sample()
                except Exception as exc:
                    carb.log_error(f"[EpisodeRecorder] sample failed for {rec.group}: {exc}")
                    return

            for rec in self._recordables:
                try:
                    storage.append_frame(rec.group, frames[rec.group])
                except Exception as exc:
                    carb.log_error(f"[EpisodeRecorder] append_frame failed for {rec.group}: {exc}")
                    return
            storage.advance_episode_frame()
            self._episode_frames_this += 1

    # ------------------------------------------------------------------ pose batch
    def _build_pose_batch(self) -> None:
        """Concatenate every opt-in recordable's ``pose_paths()`` into one ``XformPrim``.

        Must be called after every recordable has been opened (so their internal
        wrappers have resolved and paths are stable). Recordables that return ``None``
        or an empty list from :meth:`Recordable.pose_paths` stay on the per-recordable
        ``sample()`` path.
        """
        self._pose_batch = None
        self._pose_batch_slot_by_id = {}
        self._pose_batch_rec_count = 0

        recs_with_paths: list[tuple[Recordable, list[str]]] = []
        all_paths: list[str] = []
        for rec in self._recordables:
            try:
                paths = rec.pose_paths()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder] pose_paths() raised for {rec.group}: {exc}")
                continue
            if not paths:
                continue
            recs_with_paths.append((rec, list(paths)))
            all_paths.extend(paths)

        if not all_paths:
            return

        try:
            from isaacsim.core.experimental.prims import XformPrim

            batch = XformPrim(all_paths)
        except Exception as exc:
            carb.log_warn(
                f"[EpisodeRecorder] failed to build shared pose batch over {len(all_paths)} prims "
                f"({exc}); reverting to per-recordable sampling."
            )
            return

        cursor = 0
        for rec, paths in recs_with_paths:
            start = cursor
            cursor += len(paths)
            self._pose_batch_slot_by_id[id(rec)] = (start, cursor)
        self._pose_batch = batch
        self._pose_batch_rec_count = len(recs_with_paths)
        carb.log_info(
            f"[EpisodeRecorder] Pose batch: {self._pose_batch_rec_count} recordables, "
            f"{cursor} prim{'s' if cursor != 1 else ''}, backend={self._pose_backend!r}."
        )

    def _teardown_pose_batch(self) -> None:
        self._pose_batch = None
        self._pose_batch_slot_by_id = {}
        self._pose_batch_rec_count = 0

    # ------------------------------------------------------------------ command bus
    def _subscribe_command_bus(self) -> None:
        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._command_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=self._on_command_event,
            observer_name=f"EpisodeRecorder.{self._session_id}",
        )

    def _unsubscribe_command_bus(self) -> None:
        if self._command_sub is not None:
            self._command_sub = None

    def _on_command_event(self, event: Any) -> None:
        payload = dict(event.payload) if hasattr(event, "payload") else dict(event)
        target_sid = payload.get("session_id")
        if target_sid is not None and target_sid != self._session_id:
            return
        command = payload.get("command")
        if command not in VALID_COMMANDS:
            carb.log_warn(f"[EpisodeRecorder] unknown command {command!r}; valid: {sorted(VALID_COMMANDS)}.")
            return
        try:
            if command == "start":
                if self.is_session_open:
                    self.start_episode(payload.get("metadata"))
            elif command == "end":
                if self._state is _State.EPISODE_ACTIVE:
                    self.end_episode(success=payload.get("success"), metadata=payload.get("metadata"))
            elif command == "toggle":
                if self._state is _State.EPISODE_ACTIVE:
                    self.end_episode(success=payload.get("success"), metadata=payload.get("metadata"))
                elif self.is_session_open:
                    self.start_episode(payload.get("metadata"))
            elif command == "pause":
                self.pause()
            elif command == "resume":
                self.resume()
            elif command == "open_session":
                if self._state is _State.IDLE:
                    self.open_session(payload.get("output_path"))
            elif command == "close_session":
                self.close_session()
        except Exception as exc:
            carb.log_error(f"[EpisodeRecorder] command {command!r} failed: {exc}")
