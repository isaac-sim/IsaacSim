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

"""Minimal :class:`EpisodeReplayer`.

Reads an HDF5 V2 session's manifest, rehydrates :class:`Recordable` plugins via the
registry, and applies frames back to the live stage into an **anonymous sublayer** so
replay never mutates the root stage and :meth:`stop_replay` can revert the stage in
one step by popping that sublayer.

Design points:

* **Single driver.** The one supported interactive path is
  :meth:`start_replay` / :meth:`stop_replay`. Each app update advances the cursor by
  one recorded frame and applies it; the Kit timeline is *seeked* to the recorded
  ``sim_time`` (never played) so stage-authored USD animations evaluate in sync and
  physics is never stepped. This avoids the physics-tensor view invalidation that
  playing the timeline during replay would otherwise cause.
* **Stop reverts the stage.** :meth:`stop_replay` pops the anonymous sublayer, so the
  prims return to whatever the root stage authored before replay started.
* **Best-effort.** Missing prims / malformed frames are logged and skipped. Pass
  ``policy=ReplayPolicy(strictness="strict")`` to raise instead.

Low-level helpers (:meth:`prepare_episode`, :meth:`apply_frame`,
:meth:`replay_episode`) are kept for offline / scripted loops.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

import carb
import carb.eventdispatcher
import numpy as np
from pxr import Sdf

from ._pose_backend import PoseBackend, normalize_pose_backend, pose_backend_ctx
from .base import Recordable, ReplayPolicy
from .manifest import SessionManifest
from .recordables._utils import is_missing_xform_ops_error
from .registry import rehydrate
from .storage import SessionReader


class EpisodeReplayer:
    """Read-side of the HDF5 V2 session format.

    Interactive usage::

        replayer = EpisodeReplayer("/tmp/demos/episode.hdf5")
        replayer.start_replay(episode=0, on_finished=lambda: print("done"))
        # ... let the app run ...
        replayer.stop_replay()   # stage reverts to pre-replay state
        replayer.close()

    Offline usage::

        replayer = EpisodeReplayer(path)
        replayer.replay_episode(0)   # applies every frame synchronously
        replayer.close()

    Args:
        hdf5_path: Path to the recorded HDF5 V2 session file.
        policy: :class:`ReplayPolicy` controlling strict / best-effort error handling.
        pose_backend: Backend for shared pose-batch writes. Defaults to ``"usd"``
            for correctness with nested xforms; ``"usdrt"`` / ``"fabric"`` need
            FSD and trade correctness for write throughput on flat scenes.
    """

    def __init__(
        self,
        hdf5_path: str,
        *,
        policy: ReplayPolicy | None = None,
        pose_backend: PoseBackend = "usd",
    ) -> None:
        hdf5_path = os.path.abspath(os.path.expanduser(hdf5_path))
        if not os.path.exists(hdf5_path):
            raise FileNotFoundError(f"EpisodeReplayer: HDF5 session not found: {hdf5_path}")
        self._hdf5_path = hdf5_path
        self._reader = SessionReader(hdf5_path)
        self._policy = policy if policy is not None else ReplayPolicy()
        self._pose_backend: PoseBackend = normalize_pose_backend(pose_backend)
        self._prepared_episode: str | None = None
        self._prepared_num_frames: int = 0
        self._prepared: list[Recordable] = []
        self._sim_time_cache: np.ndarray | None = None
        self._frames_cache: dict[str, dict[str, np.ndarray]] = {}
        # Pose batch is split into ancestry-ordered tiers so parent writes flush
        # before children compute local poses (otherwise children lag a frame on
        # nested articulations).
        self._replay_pose_batch_tiers: list[Any] = []
        self._replay_pose_batch_tier_indices: list[np.ndarray] = []
        # Per-tier ``positions[sel]`` selector: ``slice`` for contiguous full
        # ranges (zero-copy view), ``np.ndarray`` otherwise (fancy indexing).
        self._replay_pose_batch_tier_selectors: list[slice | np.ndarray] = []
        self._replay_pose_batch_slot_by_id: dict[int, tuple[int, int]] = {}
        self._replay_pose_positions: np.ndarray | None = None
        self._replay_pose_orientations: np.ndarray | None = None
        self._replay_pose_batch_reset_tiers: set[int] = set()

        self._replay_edit_layer: Any = None
        self._replay_prev_edit_target: Any = None

        self._replay_sub: Any = None
        self._replay_frame: int = 0
        self._replay_total: int = 0
        self._replay_loop: bool = False
        self._replay_seek_timeline: bool = True
        self._replay_paused: bool = False
        self._replay_on_applied: Callable[[int], None] | None = None
        self._replay_on_finished: Callable[[], None] | None = None

    # ------------------------------------------------------------------ properties
    @property
    def hdf5_path(self) -> str:
        return self._hdf5_path

    @property
    def policy(self) -> ReplayPolicy:
        return self._policy

    @property
    def is_replaying(self) -> bool:
        """``True`` while :meth:`start_replay` is attached (before :meth:`stop_replay`)."""
        return self._replay_sub is not None

    @property
    def is_paused(self) -> bool:
        """``True`` while an active replay is paused via :meth:`pause_replay`."""
        return self._replay_sub is not None and self._replay_paused

    @property
    def current_frame(self) -> int:
        """Last applied frame index during an active replay; ``0`` when idle."""
        return self._replay_frame

    @property
    def total_frames(self) -> int:
        """Frame count of the current replay session; ``0`` when idle."""
        return self._replay_total

    @property
    def prepared_recordables(self) -> list[Recordable]:
        return list(self._prepared)

    @property
    def pose_batch_size(self) -> int:
        """Number of prims in the shared replay pose batch; ``0`` when no batch is active."""
        if self._replay_pose_positions is None:
            return 0
        return int(self._replay_pose_positions.shape[0])

    @property
    def pose_batch_tier_count(self) -> int:
        """Number of ancestry-ordered tiers in the shared replay pose batch.

        Diagnostic for the topological split: ``1`` for flat batches (one
        ``set_world_poses`` per frame), ``2`` for the typical
        articulation-nested-under-moving-xform case, higher for deeper
        hierarchies. Frame cost scales with this count.
        """
        return len(self._replay_pose_batch_tiers)

    @property
    def pose_backend(self) -> PoseBackend:
        """Active backend for the shared pose-batch writes (``"usd"`` / ``"usdrt"`` / ``"fabric"``)."""
        return self._pose_backend

    def list_episodes(self) -> list[str]:
        return self._reader.list_episodes()

    def num_frames(self, episode: int | str) -> int:
        return self._reader.num_frames(episode)

    def episode_attrs(self, episode: int | str) -> dict[str, Any]:
        """Return the HDF5 attribute dict for the given episode (e.g. ``num_frames``, ``success``)."""
        return self._reader.episode_attrs(episode)

    def manifest(self) -> SessionManifest:
        return self._reader.manifest()

    def session_metadata(self) -> dict[str, Any]:
        return dict(self._reader.manifest().session)

    # ------------------------------------------------------------------ preparation
    def prepare_episode(self, episode: int | str) -> None:
        """Rehydrate recordables, bind them to the live stage, and prefetch frames.

        The full episode is read into memory once (one contiguous read per channel) so
        :meth:`apply_frame` becomes an in-memory index instead of a per-tick HDF5 slab
        read. This is the main lever for replay throughput and for keeping the main
        thread unblocked during playback.

        In ``best_effort`` mode (default), missing types / bindings are logged and skipped.
        In ``strict`` mode, any failure aborts with :class:`RuntimeError`.
        """
        episode_name = self._reader.normalize_episode(episode)
        self._release_prepared()

        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("EpisodeReplayer: no USD stage loaded; cannot prepare episode.")

        self._push_replay_edit_target()

        manifest = self._reader.manifest()
        prepared: list[Recordable] = []
        try:
            for entry in manifest.tracks:
                try:
                    rec = rehydrate(entry)
                except KeyError as exc:
                    if self._policy.strictness == "strict":
                        raise
                    carb.log_warn(f"[EpisodeReplayer] unknown recordable type in manifest: {exc}")
                    continue
                try:
                    rec.on_session_open(stage)
                except Exception as exc:
                    if self._policy.strictness == "strict":
                        raise RuntimeError(f"[EpisodeReplayer] on_session_open failed for {rec.group}: {exc}") from exc
                    carb.log_warn(f"[EpisodeReplayer] bind failed for {rec.group}: {exc}")
                    continue
                try:
                    rec.on_episode_start()
                except Exception as exc:
                    carb.log_warn(f"[EpisodeReplayer] on_episode_start failed for {rec.group}: {exc}")
                prepared.append(rec)
        except Exception:
            self._prepared = prepared
            try:
                self._release_prepared()
            finally:
                self._pop_replay_edit_target()
            raise

        self._prepared = prepared
        self._prepared_episode = episode_name
        self._prepared_num_frames = self._reader.num_frames(episode_name)
        self._build_replay_pose_batch(prepared)
        self._sim_time_cache = self._load_sim_time(episode_name)
        try:
            self._frames_cache = self._prefetch_frames(episode_name, prepared)
        except Exception:
            # Pose batch was already built above; ``_release_prepared`` calls
            # ``_teardown_replay_pose_batch`` so this single cleanup path covers it.
            try:
                self._release_prepared()
            finally:
                self._pop_replay_edit_target()
            raise

    def _prefetch_frames(self, episode_name: str, recordables: list[Recordable]) -> dict[str, dict[str, np.ndarray]]:
        """Load every prepared recordable's datasets fully into memory."""
        return self._prefetch_groups(self._reader, episode_name, [rec.group for rec in recordables])

    def _prefetch_frames_from_new_reader(
        self, episode_name: str, groups: tuple[str, ...]
    ) -> dict[str, dict[str, np.ndarray]]:
        """Load frames on a worker-owned HDF5 reader.

        Pure I/O — no USD / stage interaction — so this body is also safe to execute
        from a worker thread via :meth:`prepare_episode_async`. Opening a dedicated
        read-only HDF5 handle inside that worker avoids sharing h5py objects across
        threads and keeps the UI thread out of the prefetch path.
        """
        reader = SessionReader(self._hdf5_path)
        try:
            return self._prefetch_groups(reader, episode_name, groups)
        finally:
            reader.close()

    def _prefetch_groups(
        self, reader: SessionReader, episode_name: str, groups: list[str] | tuple[str, ...]
    ) -> dict[str, dict[str, np.ndarray]]:
        """Load every requested group as full time-series arrays."""
        cache: dict[str, dict[str, np.ndarray]] = {}
        for group in groups:
            try:
                cache[group] = reader.read_group_all_frames(episode_name, group)
            except Exception as exc:
                if self._policy.strictness == "strict":
                    raise
                carb.log_warn(f"[EpisodeReplayer] prefetch failed for {group}: {exc}")
        return cache

    async def prepare_episode_async(self, episode: int | str) -> None:
        """Async wrapper around :meth:`prepare_episode` that offloads the prefetch.

        Stage binding (``on_session_open`` / edit target push) must stay on the main
        thread — USD is not thread-safe. The prefetch itself (pure HDF5 reads) is
        moved to the default executor so the main thread stays responsive while a
        large episode is being loaded.
        """
        import asyncio

        episode_name = self._reader.normalize_episode(episode)
        self._release_prepared()

        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("EpisodeReplayer: no USD stage loaded; cannot prepare episode.")

        self._push_replay_edit_target()

        manifest = self._reader.manifest()
        prepared: list[Recordable] = []
        try:
            for entry in manifest.tracks:
                try:
                    rec = rehydrate(entry)
                except KeyError as exc:
                    if self._policy.strictness == "strict":
                        raise
                    carb.log_warn(f"[EpisodeReplayer] unknown recordable type in manifest: {exc}")
                    continue
                try:
                    rec.on_session_open(stage)
                except Exception as exc:
                    if self._policy.strictness == "strict":
                        raise RuntimeError(f"[EpisodeReplayer] on_session_open failed for {rec.group}: {exc}") from exc
                    carb.log_warn(f"[EpisodeReplayer] bind failed for {rec.group}: {exc}")
                    continue
                try:
                    rec.on_episode_start()
                except Exception as exc:
                    carb.log_warn(f"[EpisodeReplayer] on_episode_start failed for {rec.group}: {exc}")
                prepared.append(rec)
        except Exception:
            self._prepared = prepared
            try:
                self._release_prepared()
            finally:
                self._pop_replay_edit_target()
            raise

        loop = asyncio.get_running_loop()
        try:
            groups = tuple(rec.group for rec in prepared)
            cache = await loop.run_in_executor(None, self._prefetch_frames_from_new_reader, episode_name, groups)
        except BaseException:
            self._prepared = prepared
            try:
                self._release_prepared()
            finally:
                self._pop_replay_edit_target()
            raise

        self._prepared = prepared
        self._prepared_episode = episode_name
        self._prepared_num_frames = self._reader.num_frames(episode_name)
        self._build_replay_pose_batch(prepared)
        self._sim_time_cache = self._load_sim_time(episode_name)
        self._frames_cache = cache

    def _release_prepared(self) -> None:
        for rec in list(self._prepared):
            try:
                rec.on_episode_end()
            except Exception:
                pass
            try:
                rec.on_session_close()
            except Exception:
                pass
        self._prepared.clear()
        self._prepared_episode = None
        self._prepared_num_frames = 0
        self._sim_time_cache = None
        self._frames_cache = {}
        self._teardown_replay_pose_batch()

    def _load_sim_time(self, episode_name: str) -> np.ndarray | None:
        manifest = self._reader.manifest()
        for entry in manifest.tracks:
            if entry.get("type") == "sim_time":
                try:
                    return self._reader.read_channel(episode_name, entry["group"], "sim_time").astype(np.float64)
                except Exception:
                    return None
        return None

    # ------------------------------------------------------------------ application
    def apply_frame(self, frame_index: int) -> None:
        """Read one frame from the prefetched cache and apply it via every recordable."""
        if self._prepared_episode is None:
            raise RuntimeError("EpisodeReplayer: call prepare_episode() before apply_frame().")
        n = self._prepared_num_frames
        if frame_index < 0 or frame_index >= n:
            raise IndexError(f"frame_index {frame_index} out of range [0, {n}).")
        batch_recs: list[Recordable] = []
        batch_frames: dict[str, dict[str, np.ndarray]] = {}
        batch_fill_error: Exception | None = None
        for rec in self._prepared:
            cached = self._frames_cache.get(rec.group)
            if cached is None:
                continue
            try:
                frame = {chan: arr[frame_index] for chan, arr in cached.items()}
            except Exception as exc:
                if self._policy.strictness == "strict":
                    raise
                carb.log_warn(f"[EpisodeReplayer] frame slice failed for {rec.group}: {exc}")
                continue
            slot = self._replay_pose_batch_slot_by_id.get(id(rec))
            if (
                slot is not None
                and self._replay_pose_positions is not None
                and self._replay_pose_orientations is not None
            ):
                batch_recs.append(rec)
                batch_frames[rec.group] = frame
                if batch_fill_error is None:
                    try:
                        self._fill_replay_pose_batch_slice(rec, frame, slot)
                    except Exception as exc:
                        if self._policy.strictness == "strict":
                            raise
                        batch_fill_error = exc
                continue
            self._apply_recordable_frame(rec, frame)

        if batch_recs:
            if batch_fill_error is not None:
                carb.log_warn(
                    f"[EpisodeReplayer] pose batch frame fill failed; falling back per recordable: {batch_fill_error}"
                )
                self._apply_recordable_frames(batch_recs, batch_frames)
            else:
                try:
                    self._apply_replay_pose_batch()
                except Exception as exc:
                    if self._policy.strictness == "strict":
                        raise
                    carb.log_warn(f"[EpisodeReplayer] pose batch apply failed; falling back per recordable: {exc}")
                    self._apply_recordable_frames(batch_recs, batch_frames)

    def _apply_recordable_frames(
        self, recordables: list[Recordable], frames_by_group: dict[str, dict[str, np.ndarray]]
    ) -> None:
        """Apply a set of recordable frames through their individual ``apply`` methods."""
        for rec in recordables:
            frame = frames_by_group.get(rec.group)
            if frame is not None:
                self._apply_recordable_frame(rec, frame)

    def _apply_recordable_frame(self, rec: Recordable, frame: dict[str, np.ndarray]) -> None:
        """Apply one non-batched recordable frame with policy-aware error handling."""
        try:
            rec.apply(frame, policy=self._policy)
        except Exception as exc:
            if self._policy.strictness == "strict":
                raise
            carb.log_warn(f"[EpisodeReplayer] apply failed for {rec.group}: {exc}")

    def _fill_replay_pose_batch_slice(
        self, rec: Recordable, frame: dict[str, np.ndarray], slot: tuple[int, int]
    ) -> None:
        """Copy one pose recordable's frame into the shared replay pose buffers.

        Frame payloads come in two shapes: scalar pose recordables (one prim) emit
        ``position``/``orientation`` keys, batched ones (e.g. articulations) emit
        ``positions``/``orientations``. A partial mix of singular and plural keys is
        always a producer-side bug, so we surface it as a descriptive
        :class:`ValueError` rather than letting a downstream ``KeyError`` mask the
        true cause.
        """
        assert self._replay_pose_positions is not None
        assert self._replay_pose_orientations is not None
        start, end = slot
        count = end - start

        has_singular_pos = "position" in frame
        has_singular_rot = "orientation" in frame
        has_plural_pos = "positions" in frame
        has_plural_rot = "orientations" in frame

        if has_singular_pos and has_singular_rot:
            positions = np.asarray(frame["position"], dtype=np.float32).reshape(1, 3)
            orientations = np.asarray(frame["orientation"], dtype=np.float32).reshape(1, 4)
        elif has_plural_pos and has_plural_rot:
            positions = np.asarray(frame["positions"], dtype=np.float32).reshape(count, 3)
            orientations = np.asarray(frame["orientations"], dtype=np.float32).reshape(count, 4)
        else:
            present = sorted(k for k in ("position", "orientation", "positions", "orientations") if k in frame)
            raise ValueError(
                f"{rec.group} replay frame must provide either 'position' + 'orientation' (scalar) "
                f"or 'positions' + 'orientations' (batched); got {present!r}."
            )

        if positions.shape[0] != count or orientations.shape[0] != count:
            raise ValueError(
                f"{rec.group} provided {positions.shape[0]} position rows / "
                f"{orientations.shape[0]} orientation rows for replay batch slot of length {count}."
            )
        self._replay_pose_positions[start:end] = positions
        self._replay_pose_orientations[start:end] = orientations

    def _apply_replay_pose_batch(self) -> None:
        """Apply the shared world-pose batch tier-by-tier from ancestors to descendants.

        Each tier is one ``XformPrim`` covering prims whose nearest in-batch ancestor
        is in a strictly-earlier tier. Calling ``set_world_poses`` per tier ensures
        parent writes are flushed to the stage before children's local-from-world
        computations run, so an articulation nested under a moving xform tracks the
        wrapper exactly instead of lagging it by one frame (the stutter symptom).

        ``XformPrim.reset_xform_op_properties`` is retried once **per tier** —
        each tier's first write may hit a freshly-defined prim that triggers
        the missing-xformOps error, and the reset must scope to that tier so
        siblings stay unaffected. The retry is gated by a per-tier set so a
        later tier never inherits an earlier tier's already-reset state.
        """
        if not self._replay_pose_batch_tiers:
            return
        assert self._replay_pose_positions is not None
        assert self._replay_pose_orientations is not None

        positions = self._replay_pose_positions
        orientations = self._replay_pose_orientations
        with pose_backend_ctx(self._pose_backend):
            for tier_idx, (tier_batch, tier_selector) in enumerate(
                zip(self._replay_pose_batch_tiers, self._replay_pose_batch_tier_selectors, strict=True)
            ):
                tier_positions = positions[tier_selector]
                tier_orientations = orientations[tier_selector]

                def _write(
                    batch: Any = tier_batch, pos: np.ndarray = tier_positions, quat: np.ndarray = tier_orientations
                ) -> None:
                    batch.set_world_poses(positions=pos, orientations=quat)

                try:
                    _write()
                except Exception as exc:
                    if tier_idx in self._replay_pose_batch_reset_tiers or not is_missing_xform_ops_error(exc):
                        raise
                    tier_batch.reset_xform_op_properties()
                    self._replay_pose_batch_reset_tiers.add(tier_idx)
                    _write()

    def _build_replay_pose_batch(self, recordables: list[Recordable]) -> None:
        """Build ancestry-ordered ``XformPrim`` tiers for replaying world-pose recordables.

        Recordables with overlapping ancestry (e.g. an :class:`XformRecordable` for a
        wrapper xform plus an :class:`ArticulationRecordable` for a robot nested under
        it) are split across tiers so the wrapper's pose write flushes before the
        articulation links' writes are computed.
        """
        self._teardown_replay_pose_batch()

        all_paths: list[str] = []
        recs_with_paths: list[tuple[Recordable, list[str]]] = []
        for rec in recordables:
            try:
                paths = rec.pose_paths()
            except Exception as exc:
                carb.log_warn(f"[EpisodeReplayer] pose_paths() raised for {rec.group}: {exc}")
                continue
            if not paths:
                continue
            recs_with_paths.append((rec, list(paths)))
            all_paths.extend(paths)

        if not all_paths:
            return

        cursor = 0
        for rec, paths in recs_with_paths:
            start = cursor
            cursor += len(paths)
            self._replay_pose_batch_slot_by_id[id(rec)] = (start, cursor)
        self._replay_pose_positions = np.empty((cursor, 3), dtype=np.float32)
        self._replay_pose_orientations = np.empty((cursor, 4), dtype=np.float32)

        try:
            from isaacsim.core.experimental.prims import XformPrim
        except Exception as exc:
            carb.log_warn(
                f"[EpisodeReplayer] failed to import XformPrim for replay pose batch ({exc}); "
                "falling back to per-recordable apply."
            )
            self._teardown_replay_pose_batch()
            return

        tier_assignment = _assign_ancestry_tiers(all_paths)
        if tier_assignment is None:
            carb.log_warn(
                f"[EpisodeReplayer] failed to compute ancestry tiers over {len(all_paths)} prims; "
                "falling back to per-recordable apply."
            )
            self._teardown_replay_pose_batch()
            return

        try:
            total = len(all_paths)
            for tier_global_indices in tier_assignment:
                tier_paths = [all_paths[i] for i in tier_global_indices]
                self._replay_pose_batch_tiers.append(XformPrim(tier_paths))
                tier_indices_arr = np.asarray(tier_global_indices, dtype=np.int64)
                self._replay_pose_batch_tier_indices.append(tier_indices_arr)
                # Single contiguous tier covering all prims is the common
                # un-nested case; use a slice so ``positions[selector]`` is a
                # view instead of fancy-indexing a fresh copy every frame.
                if (
                    len(tier_indices_arr) == total
                    and tier_indices_arr[0] == 0
                    and tier_indices_arr[-1] == total - 1
                    and bool(np.array_equal(tier_indices_arr, np.arange(total, dtype=np.int64)))
                ):
                    self._replay_pose_batch_tier_selectors.append(slice(0, total))
                else:
                    self._replay_pose_batch_tier_selectors.append(tier_indices_arr)
        except Exception as exc:
            carb.log_warn(
                f"[EpisodeReplayer] failed to build replay pose batch over {len(all_paths)} prims "
                f"({exc}); falling back to per-recordable apply."
            )
            self._teardown_replay_pose_batch()
            return

        carb.log_info(
            f"[EpisodeReplayer] Replay pose batch: {len(recs_with_paths)} recordables, "
            f"{cursor} prim{'s' if cursor != 1 else ''}, "
            f"{len(tier_assignment)} tier{'s' if len(tier_assignment) != 1 else ''}, "
            f"backend={self._pose_backend!r}."
        )

    def _teardown_replay_pose_batch(self) -> None:
        self._replay_pose_batch_tiers = []
        self._replay_pose_batch_tier_indices = []
        self._replay_pose_batch_tier_selectors = []
        self._replay_pose_batch_slot_by_id = {}
        self._replay_pose_positions = None
        self._replay_pose_orientations = None
        self._replay_pose_batch_reset_tiers = set()

    def replay_episode(
        self,
        episode: int | str = 0,
        *,
        render_interval: int = 1,
        start_frame: int = 0,
        end_frame: int | None = None,
        pre_frame_hook: Callable[[int], None] | None = None,
        post_frame_hook: Callable[[int], None] | None = None,
        app_update_per_frame: bool = True,
    ) -> None:
        """Offline loop: apply each frame, optionally drive ``kit.app.update`` and user hooks."""
        self.prepare_episode(episode)
        total = self._prepared_num_frames
        if end_frame is None:
            end_frame = total
        if start_frame < 0 or end_frame > total or start_frame > end_frame:
            raise ValueError(f"Invalid frame range [{start_frame}, {end_frame}) for {total} frames.")
        if render_interval < 1:
            raise ValueError(f"render_interval must be >= 1, got {render_interval}.")

        app_update = None
        if app_update_per_frame:
            import omni.kit.app

            app_update = omni.kit.app.get_app().update

        for f in range(start_frame, end_frame):
            if pre_frame_hook is not None:
                pre_frame_hook(f)
            self.apply_frame(f)
            if app_update is not None:
                app_update()
            if post_frame_hook is not None and (f % render_interval == 0):
                post_frame_hook(f)

    # ------------------------------------------------------------------ interactive replay
    def start_replay(
        self,
        *,
        episode: int | str = 0,
        loop: bool = False,
        seek_timeline: bool = True,
        on_applied: Callable[[int], None] | None = None,
        on_finished: Callable[[], None] | None = None,
    ) -> None:
        """Start a visual replay driven by ``omni.kit.app`` updates.

        Each app tick advances the cursor by one recorded frame and applies it. When
        ``seek_timeline`` is ``True`` (default), the Kit timeline is also seeked to
        the recorded ``sim_time`` of that frame (without playing it) so stage-authored
        USD animations evaluate in lockstep. The timeline is never played — physics
        is not stepped — and all pose writes land in an anonymous USD sublayer so
        the root stage is never mutated.

        Calling :meth:`start_replay` while a replay is already attached restarts it
        cleanly (stops the previous run and begins a new one).

        Args:
            episode: Episode index or name to replay.
            loop: If ``True``, wrap the cursor back to ``0`` at the end instead of
                stopping.
            seek_timeline: If ``True`` (default), advance the Kit timeline to each
                frame's recorded ``sim_time``. Set to ``False`` to replay only the
                recorded prim poses and leave the timeline untouched.
            on_applied: Optional callback ``(frame_index) -> None`` fired after each
                applied frame.
            on_finished: Optional callback fired when the cursor reaches the last
                frame in non-loop mode (exactly once per run).
        """
        if self.is_replaying:
            self.stop_replay()
        self.prepare_episode(episode)
        self._attach_replay_driver(
            loop=loop,
            seek_timeline=seek_timeline,
            on_applied=on_applied,
            on_finished=on_finished,
        )

    async def start_replay_async(
        self,
        *,
        episode: int | str = 0,
        loop: bool = False,
        seek_timeline: bool = True,
        on_applied: Callable[[int], None] | None = None,
        on_finished: Callable[[], None] | None = None,
    ) -> None:
        """Async variant of :meth:`start_replay` that offloads the HDF5 prefetch.

        Prefer this from UI callers — stage binding stays on the main thread but the
        usually-expensive prefetch runs on the default executor, so the app stays
        interactive while a large episode is being loaded.
        """
        if self.is_replaying:
            self.stop_replay()
        await self.prepare_episode_async(episode)
        self._attach_replay_driver(
            loop=loop,
            seek_timeline=seek_timeline,
            on_applied=on_applied,
            on_finished=on_finished,
        )

    def _attach_replay_driver(
        self,
        *,
        loop: bool,
        seek_timeline: bool,
        on_applied: Callable[[int], None] | None,
        on_finished: Callable[[], None] | None,
    ) -> None:
        """Wire the app-update subscription that drives one recorded frame per tick.

        Assumes :meth:`prepare_episode` (or its async equivalent) has already populated
        ``_prepared`` / ``_frames_cache`` for the target episode.
        """
        import omni.kit.app

        total = self._prepared_num_frames
        if total <= 0:
            self._release_prepared()
            self._pop_replay_edit_target()
            raise RuntimeError("EpisodeReplayer: prepared episode is empty; cannot start replay.")

        self._replay_total = total
        self._replay_frame = 0
        self._replay_loop = bool(loop)
        self._replay_seek_timeline = bool(seek_timeline)
        self._replay_on_applied = on_applied
        self._replay_on_finished = on_finished

        self._replay_paused = False
        self._apply_replay_frame(self._replay_frame)

        def _on_app_update(_evt: Any) -> None:
            if self._replay_paused:
                return
            try:
                next_idx = self._replay_frame + 1
                if next_idx >= self._replay_total:
                    if self._replay_loop:
                        next_idx = 0
                    else:
                        on_fin = self._replay_on_finished
                        self.stop_replay()
                        if on_fin is not None:
                            try:
                                on_fin()
                            except Exception as exc:
                                carb.log_warn(f"[EpisodeReplayer] on_finished raised: {exc}")
                        return
                self._replay_frame = next_idx
                self._apply_replay_frame(next_idx)
            except Exception as exc:
                carb.log_error(f"[EpisodeReplayer] replay tick failed: {exc}")

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._replay_sub = dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=_on_app_update,
            observer_name=f"EpisodeReplayer.replay.{id(self)}",
        )

    def stop_replay(self) -> None:
        """Stop an active replay and revert the stage to its pre-replay state.

        Unsubscribes from the app update stream, releases per-recordable bindings,
        and pops the anonymous sublayer that received the replay pose writes. After
        this call the root stage is back to whatever it was authoring before
        :meth:`start_replay`. Safe to call when no replay is attached.
        """
        self._replay_sub = None
        self._replay_frame = 0
        self._replay_total = 0
        self._replay_loop = False
        self._replay_seek_timeline = True
        self._replay_paused = False
        self._replay_on_applied = None
        self._replay_on_finished = None
        self._release_prepared()
        self._pop_replay_edit_target()

    def pause_replay(self) -> None:
        """Pause an active replay without releasing bindings or the sublayer.

        Stops advancing the frame cursor on subsequent app updates; the last applied
        frame stays on the stage. No-op when no replay is attached or when already
        paused. Resume with :meth:`resume_replay`.
        """
        if self._replay_sub is None:
            return
        self._replay_paused = True

    def resume_replay(self) -> None:
        """Resume a paused replay. No-op when not paused or no replay is attached."""
        if self._replay_sub is None:
            return
        self._replay_paused = False

    def step_frame(self, delta: int = 1) -> int | None:
        """Move the replay cursor by ``delta`` frames and apply the new frame.

        Clamped to ``[0, total_frames - 1]``; never wraps even when ``loop=True``.
        Automatically pauses the replay so the stepped frame is not immediately
        overwritten by the next app-update tick — resume with :meth:`resume_replay`
        (or the panel's Pause/Resume button) to continue playback.

        Args:
            delta: Signed frame offset (``+1`` / ``-1`` for single-step, larger
                values for jumps).

        Returns:
            The newly applied frame index, or ``None`` when no replay is attached.
        """
        if self._replay_sub is None or self._replay_total <= 0:
            return None
        new_idx = max(0, min(self._replay_total - 1, self._replay_frame + int(delta)))
        if new_idx == self._replay_frame and self._replay_paused:
            return new_idx
        self._replay_paused = True
        self._replay_frame = new_idx
        self._apply_replay_frame(new_idx)
        return new_idx

    def _apply_replay_frame(self, idx: int) -> None:
        self.apply_frame(idx)
        if self._replay_seek_timeline:
            self._seek_timeline_to_frame(idx)
        on_applied = self._replay_on_applied
        if on_applied is not None:
            try:
                on_applied(idx)
            except Exception as exc:
                carb.log_warn(f"[EpisodeReplayer] on_applied raised: {exc}")

    def _seek_timeline_to_frame(self, idx: int) -> None:
        """Seek the Kit timeline to the recorded ``sim_time`` of ``idx`` without playing it.

        Silently no-op when the session has no ``sim_time`` track or when the timeline
        interface is not available. The timeline is only seeked — never played — so
        physics is not stepped.
        """
        times = self._sim_time_cache
        if times is None or len(times) == 0:
            return
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            origin = float(times[0])
            t = float(times[idx]) - origin
            timeline.set_current_time(t)
        except Exception as exc:
            carb.log_warn(f"[EpisodeReplayer] timeline seek to frame {idx} failed: {exc}")

    # ------------------------------------------------------------------ anon sublayer
    def _push_replay_edit_target(self) -> None:
        if self._replay_edit_layer is not None:
            return
        try:
            import omni.usd
            from pxr import Usd

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            anon = Sdf.Layer.CreateAnonymous("episode_replay")
            session_layer = stage.GetSessionLayer()
            session_layer.subLayerPaths.insert(0, anon.identifier)
            self._replay_prev_edit_target = stage.GetEditTarget()
            stage.SetEditTarget(Usd.EditTarget(anon))
            self._replay_edit_layer = anon
        except Exception as exc:
            carb.log_warn(f"[EpisodeReplayer] Failed to push replay edit target (writes will hit root layer): {exc}")
            self._replay_edit_layer = None
            self._replay_prev_edit_target = None

    def _pop_replay_edit_target(self) -> None:
        if self._replay_edit_layer is None and self._replay_prev_edit_target is None:
            return
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage is not None:
                if self._replay_prev_edit_target is not None:
                    try:
                        stage.SetEditTarget(self._replay_prev_edit_target)
                    except Exception:
                        pass
                if self._replay_edit_layer is not None:
                    session_layer = stage.GetSessionLayer()
                    ident = self._replay_edit_layer.identifier
                    paths = list(session_layer.subLayerPaths)
                    if ident in paths:
                        paths.remove(ident)
                        session_layer.subLayerPaths = paths
        except Exception as exc:
            carb.log_warn(f"[EpisodeReplayer] Failed to pop replay edit target: {exc}")
        finally:
            self._replay_edit_layer = None
            self._replay_prev_edit_target = None

    # ------------------------------------------------------------------ teardown
    def close(self) -> None:
        """Full teardown: stop any active replay and close the HDF5 reader."""
        self.stop_replay()
        self._reader.close()

    def __enter__(self) -> EpisodeReplayer:
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()


def _assign_ancestry_tiers(paths: list[str]) -> list[list[int]] | None:
    """Group prim path indices into ancestry-ordered tiers.

    Tier ``k`` contains indices whose nearest ancestor that is also in ``paths``
    sits in some tier ``< k``. Tier ``0`` therefore contains indices whose
    in-batch ancestors are empty (the batch's own roots).

    Args:
        paths: List of absolute USD prim paths. Order is preserved within tiers.

    Returns:
        List of tiers (lists of indices into ``paths``), or ``None`` if the input
        contains malformed paths. Empty input returns an empty list.

    Properties:
        * Every input index appears in exactly one tier.
        * For any pair of indices ``(parent, child)`` where ``parent`` is an
          ancestor of ``child`` in ``paths``, the parent's tier index is strictly
          less than the child's. Sibling and unrelated paths land in the same tier.
        * Duplicate paths are tolerated: every index is tier-assigned, and
          duplicates share the same tier (they map to the same ancestor chain),
          so two writes to the same prim happen back-to-back inside one
          ``XformPrim.set_world_poses`` call - the last write wins, matching
          the per-recordable apply path's behavior.

    Path parsing goes through :class:`pxr.Sdf.Path` so variant selections,
    property paths, and trailing slashes - which a naive ``rfind('/')`` walk
    mishandles - return ``None`` (caller falls back to per-recordable apply)
    instead of computing wrong ancestors.
    """
    if not paths:
        return []

    sdf_paths: list[Any] = []
    in_batch: dict[str, int] = {}
    for idx, path in enumerate(paths):
        if not isinstance(path, str) or not Sdf.Path.IsValidPathString(path):
            return None
        sdf_path = Sdf.Path(path)
        if not sdf_path.IsAbsolutePath() or not sdf_path.IsPrimPath():
            return None
        sdf_paths.append(sdf_path)
        in_batch.setdefault(path, idx)

    abs_root = Sdf.Path.absoluteRootPath

    nearest_ancestor_idx: list[int] = []
    for path, sdf_path in zip(paths, sdf_paths, strict=True):
        ancestor_idx = -1
        canonical_self = in_batch[path]
        cursor = sdf_path.GetParentPath()
        while cursor and cursor != abs_root:
            cursor_str = cursor.pathString
            if cursor_str in in_batch and in_batch[cursor_str] != canonical_self:
                ancestor_idx = in_batch[cursor_str]
                break
            cursor = cursor.GetParentPath()
        nearest_ancestor_idx.append(ancestor_idx)

    # Iterative tier assignment via memoized depth-walk. Each index walks up to its
    # in-batch ancestor chain at most once, so total work is O(N * max_depth) which
    # in practice is O(N) since ancestor chains are short.
    tier_by_idx: list[int] = [-1] * len(paths)
    for start in range(len(paths)):
        if tier_by_idx[start] != -1:
            continue
        chain: list[int] = []
        cursor_idx = start
        while cursor_idx != -1 and tier_by_idx[cursor_idx] == -1:
            chain.append(cursor_idx)
            cursor_idx = nearest_ancestor_idx[cursor_idx]
        base_tier = 0 if cursor_idx == -1 else tier_by_idx[cursor_idx] + 1
        for offset, idx in enumerate(reversed(chain)):
            tier_by_idx[idx] = base_tier + offset

    max_tier = max(tier_by_idx)
    tiers: list[list[int]] = [[] for _ in range(max_tier + 1)]
    for i, tier in enumerate(tier_by_idx):
        tiers[tier].append(i)
    return tiers
