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

"""Main record / replay panel for the standalone Episode Recorder window.

Teleop-agnostic: the panel discovers built-in recordable targets under a USD
root (articulations, rigid bodies, loose xforms), opens an
:class:`EpisodeRecorder` session, and drives recording / replay through the
Kit timeline. Extra channels (e.g. teleop controllers, head pose) are
contributed by other extensions via
:func:`isaacsim.replicator.episode_recorder.register_session_injector`.
"""

from __future__ import annotations

import asyncio
import glob
import os
import time
from typing import Any

import carb
import carb.eventdispatcher
import carb.settings
import omni.kit.app
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.episode_recorder import (
    EPISODE_BINDING_EVENT,
    ArticulationRecordable,
    EpisodeRecorder,
    EpisodeReplayer,
    ReplayPolicy,
    RigidBodyRecordable,
    SamplingConfig,
    TimelineDrivenEpisodeController,
    XformRecordable,
    apply_session_injectors,
    dispatch_episode_command,
    export_stage_snapshot,
    target_discovery,
)

from .ui_helpers import (
    CLR_DIM,
    CLR_GREEN,
    CLR_RED,
    CLR_YELLOW,
    GLYPHS,
    INDENT,
    ROW_HEIGHT,
    ROW_SPACING,
    SECTION_SPACING,
    STATUS_HEIGHT,
    open_dir,
    set_status,
)

_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.episode_recorder.ui"

_DEFAULT_ROOT_PATH = "/World"
_DEFAULT_FILE_PREFIX = "episode"
_DEFAULT_OUTPUT_SUBDIR = "_episode_recorder"  # Created under cwd.
_DEFAULT_AUTO_START_ON_PLAY = True
_DEFAULT_REPLAY_SEEK_TIMELINE = True
_REPLAY_UI_UPDATE_INTERVAL = 0.1
_REPLAY_LOG_INTERVAL = 1.0

_TARGETS_FRAME_KEY = "targets"
_REPLAY_FRAME_KEY = "replay"

# Pose-batch backend ComboBox options. Keep the order: index 0 == default ("usd").
# ``"usdrt"`` / ``"fabric"`` require Fabric Scene Delegate; ``EpisodeRecorder`` /
# ``EpisodeReplayer`` already coerce them back to ``"usd"`` with a carb warning
# when FSD is disabled, so an "unsafe" selection here is never silently fatal.
_POSE_BACKEND_OPTIONS: tuple[str, ...] = ("usd", "usdrt", "fabric")
_DEFAULT_POSE_BACKEND_RECORD = "usd"
_DEFAULT_POSE_BACKEND_REPLAY = "usd"


class EpisodeRecorderPanel:
    """Standalone record / replay panel.

    The panel owns a single :class:`EpisodeRecorder` / :class:`EpisodeReplayer`
    pair at any time. Opening a session auto-subscribes to
    :class:`SessionEvents` so UI state tracks recorder state transitions
    regardless of whether they come from the UI buttons, the timeline
    controller, or an external :data:`EPISODE_CMD_EVENT` dispatch.
    """

    def __init__(self) -> None:
        self._settings = carb.settings.get_settings()
        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/root_path", _DEFAULT_ROOT_PATH)
        self._settings.set_default_string(
            f"{_SETTINGS_PREFIX}/output_dir",
            os.path.join(os.getcwd(), _DEFAULT_OUTPUT_SUBDIR),
        )
        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/file_prefix", _DEFAULT_FILE_PREFIX)
        self._settings.set_default_bool(f"{_SETTINGS_PREFIX}/auto_start_on_play", _DEFAULT_AUTO_START_ON_PLAY)
        self._settings.set_default_bool(f"{_SETTINGS_PREFIX}/replay_seek_timeline", _DEFAULT_REPLAY_SEEK_TIMELINE)
        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/pose_backend_record", _DEFAULT_POSE_BACKEND_RECORD)
        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/pose_backend_replay", _DEFAULT_POSE_BACKEND_REPLAY)

        self._collapsed: dict[str, bool] = {
            _TARGETS_FRAME_KEY: True,
            _REPLAY_FRAME_KEY: False,
        }

        self._root_path_field: ui.StringField | None = None
        self._output_dir_field: ui.StringField | None = None
        self._file_prefix_field: ui.StringField | None = None
        self._auto_start_model: ui.SimpleBoolModel | None = None
        self._auto_start_check: ui.CheckBox | None = None
        self._export_snapshot_btn: ui.Button | None = None
        self._pose_backend_record_combo: ui.ComboBox | None = None
        self._pose_backend_replay_combo: ui.ComboBox | None = None

        self._targets_content: ui.Frame | None = None
        self._session_btn: ui.Button | None = None
        self._episode_btn: ui.Button | None = None
        self._discover_btn: ui.Button | None = None
        self._session_label: ui.Label | None = None
        self._status_label: ui.Label | None = None

        self._articulation_paths: dict[str, str] = {}
        self._rigid_body_paths: dict[str, str] = {}
        self._xform_paths: dict[str, str] = {}
        self._articulation_checks: dict[str, ui.SimpleBoolModel] = {}
        self._rigid_body_checks: dict[str, ui.SimpleBoolModel] = {}
        self._xform_checks: dict[str, ui.SimpleBoolModel] = {}

        self._recorder: EpisodeRecorder | None = None
        self._timeline_controller: TimelineDrivenEpisodeController | None = None
        self._recorder_event_subs: list = []
        self._was_recording: bool = False
        self._episode_count: int = 0

        self._replay_file_field: ui.StringField | None = None
        self._replay_latest_btn: ui.Button | None = None
        self._replay_load_btn: ui.Button | None = None
        self._replay_episode_combo: ui.ComboBox | None = None
        self._replay_info_label: ui.Label | None = None
        self._replay_btn: ui.Button | None = None
        self._replay_pause_btn: ui.Button | None = None
        self._replay_prev_btn: ui.Button | None = None
        self._replay_next_btn: ui.Button | None = None
        self._replay_seek_timeline_model: ui.SimpleBoolModel | None = None
        self._replay_seek_timeline_check: ui.CheckBox | None = None
        self._replay_status_label: ui.Label | None = None
        self._replay_progress_label: ui.Label | None = None
        self._replay_warning_label: ui.Label | None = None
        self._replay_warning_row: ui.HStack | None = None
        self._replayer: EpisodeReplayer | None = None
        self._replay_episodes: list[str] = []
        self._replay_frame_counts: list[int] = []
        self._replay_attached: bool = False
        self._replay_start_task: asyncio.Task | None = None
        self._replay_active_episode: int | None = None
        self._last_replay_ui_update_time: float = 0.0
        self._last_replay_log_time: float = 0.0

        self._bindings: dict[str, dict] = {}
        self._binding_badge: ui.Label | None = None
        self._binding_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=EPISODE_BINDING_EVENT,
            on_event=self._on_binding_event,
            observer_name="isaacsim.replicator.episode_recorder.ui._on_binding_event",
            order=0,
        )

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self) -> None:
        """Build the Episode Recorder panel UI."""
        with ui.VStack(spacing=SECTION_SPACING):
            self._build_targets_section()
            self._build_session_options_section()
            self._build_record_pose_backend_row()
            self._build_session_control_section()
            with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                ui.Spacer(width=INDENT)
                self._session_label = ui.Label("No session", style={"color": CLR_DIM}, word_wrap=True)
            with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                ui.Spacer(width=INDENT)
                self._status_label = ui.Label("Idle", style={"color": CLR_DIM}, word_wrap=True)
            self._build_replay_section()

        self._sync_controls()

    def _build_targets_section(self) -> None:
        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            ui.Label("USD Root:", width=65, tooltip="Prim path to scan for recordable targets")
            self._root_path_field = ui.StringField(width=ui.Fraction(1))
            self._root_path_field.model.set_value(self._load_str("root_path"))
            self._root_path_field.model.add_end_edit_fn(lambda _m: self._save_str("root_path", self._get_root_path()))
            self._discover_btn = ui.Button(
                "Discover",
                width=70,
                clicked_fn=self._on_discover_clicked,
                tooltip="List articulations, rigid bodies, and plain xforms under the root path",
            )

        targets_frame = ui.CollapsableFrame(
            "Discovered Targets",
            height=0,
            collapsed=self._collapsed[_TARGETS_FRAME_KEY],
            style=get_style(),
        )
        with targets_frame:
            targets_frame.set_collapsed_changed_fn(lambda c, k=_TARGETS_FRAME_KEY: self._collapsed.__setitem__(k, c))
            with ui.ScrollingFrame(
                height=160,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                self._targets_content = ui.Frame()
                with self._targets_content:
                    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                        ui.Spacer(width=INDENT)
                        ui.Label(
                            "Click 'Discover' to list recordable prims under the root path.",
                            style={"color": CLR_DIM},
                        )

    def _build_session_options_section(self) -> None:
        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            ui.Label("Output Dir:", width=75, tooltip="Directory where HDF5 recordings are written")
            self._output_dir_field = ui.StringField(width=ui.Fraction(1))
            self._output_dir_field.model.set_value(self._load_str("output_dir"))
            self._output_dir_field.model.add_end_edit_fn(
                lambda _m: self._save_str("output_dir", self._get_output_dir())
            )
            ui.Button(
                f"{GLYPHS['open_folder']}",
                width=20,
                clicked_fn=lambda: open_dir(self._get_output_dir()),
                tooltip="Open the output directory in the OS file explorer",
            )
            self._export_snapshot_btn = ui.Button(
                "Export Scene",
                width=90,
                clicked_fn=self._on_export_snapshot_clicked,
                tooltip=(
                    "Export a flattened USD of the current stage to\n"
                    "<output_dir>/stage_snapshot.usd (+ sidecar JSON).\n"
                    "Only needs to be run once per scene - subsequent sessions auto-link it."
                ),
            )

        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            ui.Label("File Prefix:", width=75, tooltip="Prefix used when naming new episode files")
            self._file_prefix_field = ui.StringField(width=ui.Fraction(1))
            self._file_prefix_field.model.set_value(self._load_str("file_prefix"))
            self._file_prefix_field.model.add_end_edit_fn(
                lambda _m: self._save_str("file_prefix", self._get_file_prefix())
            )

        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            self._auto_start_model = ui.SimpleBoolModel(
                self._load_bool("auto_start_on_play", _DEFAULT_AUTO_START_ON_PLAY)
            )
            self._auto_start_check = ui.CheckBox(model=self._auto_start_model, width=18)
            ui.Label(
                "Auto-start recording on timeline Play",
                tooltip=(
                    "When checked, pressing Play on the timeline automatically starts a new\n"
                    "episode. When unchecked, recording only starts when you click 'Start' -\n"
                    "you can start / stop recording at any time while the timeline is playing."
                ),
            )
            self._auto_start_model.add_value_changed_fn(self._on_auto_start_changed)

    def _build_record_pose_backend_row(self) -> None:
        """Render the record-side pose-batch backend selector above Open Session.

        Backend used by the per-tick ``XformPrim.get_world_poses`` batch read
        on :class:`EpisodeRecorder`. Reads cannot trigger the nested-articulation
        parent-lag bug (no writes happen during sampling), so FSD-backed reads
        are safe speedups when Fabric Scene Delegate is enabled. Persisted to
        carb settings under
        ``/persistent/exts/isaacsim.replicator.episode_recorder.ui/`` so the
        choice survives Kit restarts; ``"usdrt"`` / ``"fabric"`` selections fall
        back to ``"usd"`` with a carb warning when FSD is unavailable.

        The replay-side selector lives next to the Episode dropdown in
        :meth:`_build_replay_section` so each backend sits with the controls
        it actually drives.
        """
        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            ui.Label(
                "Pose Backend:",
                width=110,
                tooltip=(
                    "Backend used by the recorder's per-tick pose-batch READ.\n"
                    "  - usd:    safe default; pure USD reads.\n"
                    "  - usdrt:  Fabric Scene Delegate via IFabricHierarchy\n"
                    "            (may be faster - benchmark per scene).\n"
                    "  - fabric: Fabric Scene Delegate direct\n"
                    "            (may be faster - benchmark per scene).\n"
                    "Reads can never trigger the nested-articulation stutter,\n"
                    "so FSD-backed options are usually safe during recording.\n"
                    "Falls back to 'usd' with a warning if FSD is disabled."
                ),
            )
            record_idx = self._pose_backend_index(self._load_str("pose_backend_record"))
            self._pose_backend_record_combo = ui.ComboBox(record_idx, *_POSE_BACKEND_OPTIONS, width=ui.Fraction(1))
            self._pose_backend_record_combo.model.add_item_changed_fn(self._on_pose_backend_record_changed)

    def _build_session_control_section(self) -> None:
        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT)
            self._session_btn = ui.Button(
                "Open Session",
                width=110,
                clicked_fn=self._on_session_toggle,
                tooltip=(
                    "Create the HDF5 file and subscribe to simulation events.\n"
                    "Text flips to 'Close Session' while a session is open."
                ),
            )
            self._episode_btn = ui.Button(
                "Start",
                width=70,
                clicked_fn=self._on_episode_toggle,
                tooltip=(
                    "Start / end a recording inside the open session.\n"
                    "Works whether the timeline is stopped or already playing;\n"
                    "if the timeline is stopped it is played first and recording begins\n"
                    "as soon as physics starts ticking."
                ),
                enabled=False,
            )
            self._binding_badge = ui.Label(
                "",
                width=0,
                style={"color": CLR_GREEN, "font_size": 12},
                tooltip="External inputs (e.g. VR controller buttons) currently wired to this recorder.",
            )
        self._refresh_binding_badge()

    def _build_replay_section(self) -> None:
        replay_frame = ui.CollapsableFrame(
            "Replay",
            height=0,
            collapsed=self._collapsed[_REPLAY_FRAME_KEY],
            style=get_style(),
        )
        with replay_frame:
            replay_frame.set_collapsed_changed_fn(lambda c, k=_REPLAY_FRAME_KEY: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=SECTION_SPACING):
                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("File:", width=65, tooltip="HDF5 session to replay")
                    self._replay_file_field = ui.StringField(width=ui.Fraction(1))
                    self._replay_latest_btn = ui.Button(
                        "Latest",
                        width=60,
                        clicked_fn=self._on_replay_latest,
                        tooltip="Fill with the most recent *.hdf5 in the output dir",
                    )
                    self._replay_load_btn = ui.Button(
                        "Load",
                        width=55,
                        clicked_fn=self._on_replay_load,
                        tooltip="Open the HDF5 file and populate the episode list",
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Episode:", width=65, tooltip="Episode to replay")
                    self._replay_episode_combo = ui.ComboBox(0, width=ui.Fraction(1))
                    self._replay_info_label = ui.Label(
                        "",
                        style={"color": CLR_DIM},
                        width=150,
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "Pose Backend:",
                        width=110,
                        tooltip=(
                            "Backend used by the replayer's per-tier pose-batch WRITE.\n"
                            "  - usd:    recommended default; the ancestry-ordered tier\n"
                            "            split + USD writes is what avoids the stutter\n"
                            "            on articulations nested under moving xforms.\n"
                            "  - usdrt / fabric: only for benchmarking flat scenes;\n"
                            "            may exhibit one-frame parent-lag on nested\n"
                            "            hierarchies even with the tier split.\n"
                            "Falls back to 'usd' with a warning if FSD is disabled.\n"
                            "Applied on Load."
                        ),
                    )
                    replay_idx = self._pose_backend_index(self._load_str("pose_backend_replay"))
                    self._pose_backend_replay_combo = ui.ComboBox(
                        replay_idx, *_POSE_BACKEND_OPTIONS, width=ui.Fraction(1)
                    )
                    self._pose_backend_replay_combo.model.add_item_changed_fn(self._on_pose_backend_replay_changed)

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._replay_btn = ui.Button(
                        f"{GLYPHS['timeline_play']}",
                        width=26,
                        clicked_fn=self._on_replay_toggle,
                        tooltip=(
                            "Start / stop replay of the selected episode. Each app tick applies\n"
                            "one recorded frame; if 'Seek timeline' is checked the Kit timeline\n"
                            "is seeked (not played) to that frame's recorded sim_time so stage\n"
                            "animations keep up without running physics. Pose writes go into an\n"
                            "anonymous sublayer that is dropped on stop, reverting the stage to\n"
                            "its pre-replay state."
                        ),
                        enabled=False,
                    )
                    self._replay_pause_btn = ui.Button(
                        f"{GLYPHS['timeline_pause']}",
                        width=26,
                        clicked_fn=self._on_replay_pause_toggle,
                        tooltip=(
                            "Pause or resume the running replay. The last applied frame stays on\n"
                            "the stage while paused; stopping still pops the anonymous sublayer."
                        ),
                        enabled=False,
                    )
                    self._replay_prev_btn = ui.Button(
                        f"{GLYPHS['timeline_prev']}",
                        width=26,
                        clicked_fn=lambda: self._on_replay_step(-1),
                        tooltip="Step one frame backward (pauses replay).",
                        enabled=False,
                    )
                    self._replay_next_btn = ui.Button(
                        f"{GLYPHS['timeline_next']}",
                        width=26,
                        clicked_fn=lambda: self._on_replay_step(1),
                        tooltip="Step one frame forward (pauses replay).",
                        enabled=False,
                    )
                    self._replay_seek_timeline_model = ui.SimpleBoolModel(
                        self._load_bool("replay_seek_timeline", _DEFAULT_REPLAY_SEEK_TIMELINE)
                    )
                    self._replay_seek_timeline_check = ui.CheckBox(model=self._replay_seek_timeline_model, width=18)
                    ui.Label(
                        "Seek timeline",
                        width=0,
                        tooltip=(
                            "When enabled, each applied frame also seeks the Kit timeline to its\n"
                            "recorded sim_time (without playing it) so stage-authored USD animations\n"
                            "stay in sync. Disable to replay only the recorded prim poses and leave\n"
                            "the timeline untouched."
                        ),
                    )
                    self._replay_seek_timeline_model.add_value_changed_fn(self._on_replay_seek_timeline_changed)

                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._replay_status_label = ui.Label("Idle", style={"color": CLR_DIM}, word_wrap=True)

                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._replay_progress_label = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

                with ui.HStack(spacing=ROW_SPACING, visible=False) as warning_row:
                    ui.Spacer(width=INDENT)
                    self._replay_warning_label = ui.Label(
                        "",
                        style={"color": CLR_RED, "font_size": 12},
                        word_wrap=True,
                    )
                self._replay_warning_row = warning_row

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def _on_discover_clicked(self) -> None:
        root = self._get_root_path()
        try:
            arts = target_discovery.discover_articulations_under(root)
            raw_rigids = target_discovery.discover_rigid_bodies_under(root, exclude_articulation_descendants=True)
            raw_xforms = target_discovery.discover_xforms_under(
                root,
                include_articulations=False,
                include_rigid_bodies=False,
                exclude_articulation_descendants=True,
            )
        except Exception as exc:
            set_status(self._status_label, f"Discovery failed: {exc}", CLR_RED, emit_terminal=True)
            return

        used_names = set(arts.keys())
        rigids: dict[str, str] = {}
        for name, path in raw_rigids.items():
            rigids[target_discovery.assign_unique_name(name, used_names)] = path

        xforms: dict[str, str] = {}
        existing_paths = set(arts.values()) | set(rigids.values())
        for name, path in raw_xforms.items():
            if path in existing_paths:
                continue
            xforms[target_discovery.assign_unique_name(name, used_names)] = path

        self._articulation_paths = arts
        self._rigid_body_paths = rigids
        self._xform_paths = xforms
        self._articulation_checks = {
            name: ui.SimpleBoolModel(
                self._articulation_checks[name].get_value_as_bool() if name in self._articulation_checks else True
            )
            for name in arts
        }
        self._rigid_body_checks = {
            name: ui.SimpleBoolModel(
                self._rigid_body_checks[name].get_value_as_bool() if name in self._rigid_body_checks else True
            )
            for name in rigids
        }
        self._xform_checks = {
            name: ui.SimpleBoolModel(
                self._xform_checks[name].get_value_as_bool() if name in self._xform_checks else True
            )
            for name in xforms
        }
        self._render_discovered_targets()
        set_status(
            self._status_label,
            (
                f"Discovered {len(arts)} articulation(s), {len(rigids)} rigid body(ies), "
                f"{len(xforms)} xform(s) under '{root}'"
            ),
            CLR_GREEN,
            emit_terminal=True,
        )

    def _render_discovered_targets(self) -> None:
        if self._targets_content is None:
            return
        self._targets_content.clear()
        with self._targets_content:
            with ui.VStack(spacing=2):
                if not self._articulation_paths and not self._rigid_body_paths and not self._xform_paths:
                    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                        ui.Spacer(width=INDENT)
                        ui.Label(
                            "(no recordable prims found - add RigidBodyAPI / ArticulationRootAPI)",
                            style={"color": CLR_DIM},
                        )
                    return
                if self._articulation_paths:
                    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                        ui.Spacer(width=INDENT)
                        ui.Label("Articulations", style={"color": CLR_YELLOW})
                    for name, path in self._articulation_paths.items():
                        self._render_target_row(name, path, self._articulation_checks[name])
                if self._rigid_body_paths:
                    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                        ui.Spacer(width=INDENT)
                        ui.Label("Rigid Bodies", style={"color": CLR_YELLOW})
                    for name, path in self._rigid_body_paths.items():
                        self._render_target_row(name, path, self._rigid_body_checks[name])
                if self._xform_paths:
                    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                        ui.Spacer(width=INDENT)
                        ui.Label("Xforms", style={"color": CLR_YELLOW})
                    for name, path in self._xform_paths.items():
                        self._render_target_row(name, path, self._xform_checks[name])

    def _render_target_row(self, name: str, path: str, model: ui.SimpleBoolModel) -> None:
        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
            ui.Spacer(width=INDENT * 2)
            ui.CheckBox(model=model, width=18)
            ui.Label(name, width=120)
            ui.Label(path, style={"color": CLR_DIM})

    # ------------------------------------------------------------------
    # Session control
    # ------------------------------------------------------------------

    def _on_session_toggle(self) -> None:
        if self._recorder is None:
            self._open_session()
        else:
            self._close_session()

    def _open_session(self) -> None:
        selected_arts = self._selected("articulation")
        selected_rigid_bodies = self._selected("rigid_body")
        selected_xforms = self._selected("xform")
        if not selected_arts and not selected_rigid_bodies and not selected_xforms:
            set_status(
                self._status_label,
                "Nothing selected - click Discover and tick at least one target.",
                CLR_YELLOW,
            )
            return

        try:
            recorder = EpisodeRecorder(
                self._get_output_dir(),
                file_prefix=self._get_file_prefix(),
                sampling=SamplingConfig(),
                pose_backend=self._get_pose_backend_record(),
            )
            for name, path in selected_arts.items():
                recorder.add(ArticulationRecordable(group=f"state/{name}", prim_path=path))
            for name, path in selected_xforms.items():
                recorder.add(XformRecordable(group=f"state/{name}", prim_path=path))
            for name, path in selected_rigid_bodies.items():
                recorder.add(RigidBodyRecordable(group=f"state/{name}", prim_path=path))
            apply_session_injectors(recorder)
            self._recorder = recorder
            hdf5_path = recorder.open_session()
            self._subscribe_recorder_events(recorder)
            self._timeline_controller = TimelineDrivenEpisodeController(
                recorder, auto_start_on_play=self._get_auto_start_on_play()
            )
            self._timeline_controller.enable()
        except Exception as exc:
            self._close_recorder_silently()
            set_status(self._status_label, f"open_session failed: {exc}", CLR_RED, emit_terminal=True)
            self._sync_controls()
            return

        self._was_recording = False
        self._episode_count = 0
        set_status(self._session_label, f"File: {os.path.basename(hdf5_path)}", CLR_DIM)
        set_status(
            self._status_label,
            (
                f"Session open - {len(selected_arts)} articulation(s), "
                f"{len(selected_rigid_bodies)} rigid body(ies), {len(selected_xforms)} xform(s)"
            ),
            CLR_YELLOW,
            emit_terminal=True,
        )
        self._sync_controls()

    def _selected(self, kind: str) -> dict[str, str]:
        paths = {
            "articulation": (self._articulation_paths, self._articulation_checks),
            "rigid_body": (self._rigid_body_paths, self._rigid_body_checks),
            "xform": (self._xform_paths, self._xform_checks),
        }[kind]
        path_map, check_map = paths
        return {
            name: path for name, path in path_map.items() if check_map.get(name) and check_map[name].get_value_as_bool()
        }

    def _close_session(self) -> None:
        if self._recorder is None:
            return
        episode_count = self._episode_count
        close_errors = self._close_recorder_silently()
        if close_errors:
            set_status(
                self._status_label,
                f"Session closed with warnings ({episode_count} episode(s)) - check terminal logs.",
                CLR_YELLOW,
                emit_terminal=True,
            )
        else:
            set_status(
                self._status_label,
                f"Session closed ({episode_count} episode(s))",
                CLR_GREEN,
                emit_terminal=True,
            )
        set_status(self._session_label, "No session", CLR_DIM)
        self._sync_controls()

    def _on_episode_toggle(self) -> None:
        if self._recorder is None:
            set_status(self._status_label, "Open a session first.", CLR_YELLOW)
            return

        session_id = self._recorder.session_id
        if self._is_recording():
            dispatch_episode_command("end", session_id=session_id, success=True)
            return

        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
            if self._get_auto_start_on_play():
                set_status(
                    self._status_label,
                    "Timeline playing - recording will start on the first physics step.",
                    CLR_YELLOW,
                    emit_terminal=True,
                )
                return

        dispatch_episode_command("start", session_id=session_id)

    def _on_auto_start_changed(self, model: ui.SimpleBoolModel) -> None:
        value = bool(model.get_value_as_bool())
        self._save_bool("auto_start_on_play", value)
        if self._timeline_controller is not None:
            self._timeline_controller.set_auto_start_on_play(value)

    def _on_replay_seek_timeline_changed(self, model: ui.SimpleBoolModel) -> None:
        """Persist the Seek-timeline replay option; applies to the next Start Replay.

        Args:
            model: Checkbox model containing the new persisted state.
        """
        self._save_bool("replay_seek_timeline", bool(model.get_value_as_bool()))

    def _on_pose_backend_record_changed(self, model: ui.AbstractItemModel, _item: object) -> None:
        """Persist the recorder pose-backend selection; applies to the next Open Session.

        Args:
            model: ComboBox model containing the selected backend index.
            _item: ComboBox item emitted by the UI callback.
        """
        self._save_str("pose_backend_record", self._pose_backend_from_combo_model(model))

    def _on_pose_backend_replay_changed(self, model: ui.AbstractItemModel, _item: object) -> None:
        """Persist the replayer pose-backend selection; applies to the next Load + Replay.

        Args:
            model: ComboBox model containing the selected backend index.
            _item: ComboBox item emitted by the UI callback.
        """
        self._save_str("pose_backend_replay", self._pose_backend_from_combo_model(model))

    @staticmethod
    def _pose_backend_index(value: str) -> int:
        """Map a backend string to its ComboBox index.

        Args:
            value: Backend name to look up.

        Returns:
            ComboBox index for ``value``, or zero when it is unknown.
        """
        try:
            return _POSE_BACKEND_OPTIONS.index(value)
        except ValueError:
            return 0

    @staticmethod
    def _pose_backend_from_combo_model(model: ui.AbstractItemModel) -> str:
        """Read the active option from a backend ComboBox.

        Args:
            model: ComboBox model containing the active option index.

        Returns:
            Selected backend name, or the record default when the index is invalid.
        """
        idx = model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(_POSE_BACKEND_OPTIONS):
            return _POSE_BACKEND_OPTIONS[idx]
        return _DEFAULT_POSE_BACKEND_RECORD

    def _get_pose_backend_record(self) -> str:
        """Return the active record-side pose backend.

        Returns:
            Backend selected in the combo if built, otherwise the persisted setting.
        """
        if self._pose_backend_record_combo is not None:
            return self._pose_backend_from_combo_model(self._pose_backend_record_combo.model)
        return self._load_str("pose_backend_record") or _DEFAULT_POSE_BACKEND_RECORD

    def _get_pose_backend_replay(self) -> str:
        """Return the active replay-side pose backend.

        Returns:
            Backend selected in the combo if built, otherwise the persisted setting.
        """
        if self._pose_backend_replay_combo is not None:
            return self._pose_backend_from_combo_model(self._pose_backend_replay_combo.model)
        return self._load_str("pose_backend_replay") or _DEFAULT_POSE_BACKEND_REPLAY

    def _on_export_snapshot_clicked(self) -> None:
        """Export a scene-level USD + sidecar into the current output directory."""
        output_dir = self._get_output_dir()
        try:
            usd_path = export_stage_snapshot(output_dir)
        except Exception as exc:
            set_status(
                self._status_label,
                f"Export scene snapshot failed: {exc}",
                CLR_RED,
                emit_terminal=True,
            )
            return
        set_status(
            self._status_label,
            f"Scene snapshot: {os.path.basename(usd_path)} in {output_dir}",
            CLR_GREEN,
            emit_terminal=True,
        )

    # ------------------------------------------------------------------
    # Replay control
    # ------------------------------------------------------------------

    def _on_replay_latest(self) -> None:
        output_dir = self._get_output_dir()
        pattern = os.path.join(output_dir, f"{self._get_file_prefix()}_*.hdf5")
        matches = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
        if not matches:
            set_status(
                self._replay_status_label,
                f"No '{os.path.basename(pattern)}' files in {output_dir}",
                CLR_YELLOW,
                emit_terminal=True,
            )
            return
        if self._replay_file_field is not None:
            self._replay_file_field.model.set_value(matches[0])
        set_status(
            self._replay_status_label,
            f"Selected {os.path.basename(matches[0])}",
            CLR_DIM,
        )

    def _on_replay_load(self) -> None:
        self._close_replayer_silently()

        path = (
            self._replay_file_field.model.get_value_as_string().strip() if self._replay_file_field is not None else ""
        )
        if not path:
            set_status(self._replay_status_label, "Pick a file first.", CLR_YELLOW)
            return
        path = os.path.abspath(os.path.expanduser(path))
        if not os.path.isfile(path):
            set_status(self._replay_status_label, f"File not found: {path}", CLR_RED, emit_terminal=True)
            return

        try:
            replayer = EpisodeReplayer(
                path,
                policy=ReplayPolicy(strictness="best_effort"),
                pose_backend=self._get_pose_backend_replay(),
            )
        except Exception as exc:
            set_status(
                self._replay_status_label,
                f"Could not open session: {exc}",
                CLR_RED,
                emit_terminal=True,
            )
            return

        try:
            episodes = replayer.list_episodes()
            frame_counts = [replayer.num_frames(name) for name in episodes]
        except Exception as exc:
            replayer.close()
            set_status(
                self._replay_status_label,
                f"Could not list episodes: {exc}",
                CLR_RED,
                emit_terminal=True,
            )
            return

        if not episodes:
            replayer.close()
            set_status(
                self._replay_status_label,
                "Session has no episodes.",
                CLR_YELLOW,
                emit_terminal=True,
            )
            return

        self._replayer = replayer
        self._replay_episodes = episodes
        self._replay_frame_counts = frame_counts
        self._populate_replay_episode_combo()
        self._sync_controls()
        set_status(
            self._replay_status_label,
            f"Loaded {os.path.basename(path)} ({len(episodes)} episode(s))",
            CLR_GREEN,
            emit_terminal=True,
        )
        self._refresh_replay_stage_check()

    def _populate_replay_episode_combo(self) -> None:
        if self._replay_episode_combo is None:
            return
        model = self._replay_episode_combo.model
        for child in model.get_item_children():
            model.remove_item(child)
        for name, frames in zip(self._replay_episodes, self._replay_frame_counts):
            model.append_child_item(None, ui.SimpleStringModel(f"{name}  ({frames} frames)"))
        if self._replay_episodes:
            model.get_item_value_model().set_value(0)
            self._update_replay_info_label(0)
            model.add_item_changed_fn(
                lambda m, _i: self._update_replay_info_label(m.get_item_value_model().get_value_as_int())
            )

    def _update_replay_info_label(self, index: int) -> None:
        if self._replay_info_label is None or self._replayer is None:
            return
        if not (0 <= index < len(self._replay_episodes)):
            return
        try:
            metadata = self._replayer.episode_attrs(self._replay_episodes[index])
        except Exception:
            metadata = {}
        success = metadata.get("success", None)
        bits = [f"{self._replay_frame_counts[index]} frames"]
        if success is not None:
            bits.append(f"success={bool(success)}")
        self._replay_info_label.text = ", ".join(bits)

    def _on_replay_toggle(self) -> None:
        if self._replay_attached:
            self._stop_replay(reason="user")
        else:
            self._start_replay()

    def _on_replay_step(self, delta: int) -> None:
        """Step the active replay by ``delta`` frames.

        Args:
            delta: Number of frames to move relative to the current replay frame.
        """
        replayer = self._replayer
        if replayer is None or not self._replay_attached:
            return
        try:
            replayer.step_frame(delta)
        except Exception as exc:
            carb.log_warn(f"[EpisodeRecorderPanel] step frame failed: {exc}")
            return
        self._sync_controls()

    def _on_replay_pause_toggle(self) -> None:
        """Toggle pause/resume on the active replay; no-op when idle or still preparing."""
        replayer = self._replayer
        if replayer is None or not self._replay_attached:
            return
        try:
            if replayer.is_paused:
                replayer.resume_replay()
            else:
                replayer.pause_replay()
        except Exception as exc:
            carb.log_warn(f"[EpisodeRecorderPanel] pause toggle failed: {exc}")
            return
        self._sync_controls()

    def _start_replay(self) -> None:
        if self._recorder is not None:
            set_status(
                self._replay_status_label,
                "Close the recording session first.",
                CLR_YELLOW,
            )
            return
        if self._replayer is None or not self._replay_episodes:
            set_status(self._replay_status_label, "Load a session first.", CLR_YELLOW)
            return
        if self._replay_episode_combo is None:
            return
        if self._replay_start_task is not None and not self._replay_start_task.done():
            set_status(self._replay_status_label, "Replay is already being prepared.", CLR_YELLOW)
            return
        index = self._replay_episode_combo.model.get_item_value_model().get_value_as_int()
        if not (0 <= index < len(self._replay_episodes)):
            set_status(self._replay_status_label, "Pick an episode.", CLR_YELLOW)
            return

        try:
            timeline = omni.timeline.get_timeline_interface()
            if timeline.is_playing():
                timeline.pause()
        except Exception as exc:
            carb.log_warn(f"[EpisodeRecorder][UI] timeline pause failed (continuing): {exc}")

        name = self._replay_episodes[index]
        frames = self._replay_frame_counts[index] if 0 <= index < len(self._replay_frame_counts) else 0
        set_status(
            self._replay_status_label,
            f"Preparing replay (episode {name}, {frames} frames)...",
            CLR_YELLOW,
        )
        self._replay_start_task = asyncio.ensure_future(self._start_replay_async(index))

    async def _start_replay_async(self, episode: int) -> None:
        """Run :meth:`EpisodeReplayer.start_replay_async` without freezing the UI.

        The HDF5 prefetch happens on a worker thread; USD binding and the per-tick
        apply loop remain on the main thread where they must be.

        Args:
            episode: Episode index selected in the replay ComboBox.
        """
        if self._replayer is None:
            return
        replayer = self._replayer
        try:
            await replayer.start_replay_async(
                episode=episode,
                seek_timeline=self._get_replay_seek_timeline(),
                on_applied=self._on_replay_frame_applied,
                on_finished=self._on_replay_finished,
            )
        except asyncio.CancelledError:
            return
        except Exception as exc:
            if self._replayer is replayer:
                set_status(
                    self._replay_status_label,
                    f"Replay start failed: {exc}",
                    CLR_RED,
                    emit_terminal=True,
                )
                self._replay_attached = False
                self._sync_controls()
            return
        finally:
            if self._replay_start_task is not None and self._replay_start_task.done():
                self._replay_start_task = None

        if self._replayer is not replayer:
            return
        self._replay_attached = True
        self._replay_active_episode = episode
        self._last_replay_ui_update_time = 0.0
        self._last_replay_log_time = 0.0
        name = self._replay_episodes[episode] if 0 <= episode < len(self._replay_episodes) else "?"
        frames = self._replay_frame_counts[episode] if 0 <= episode < len(self._replay_frame_counts) else 0
        carb.log_info(
            f"[EpisodeRecorder][UI] Replay: starting (episode {name}, {frames} frames, "
            f"file={os.path.basename(replayer.hdf5_path)})"
        )
        set_status(
            self._replay_status_label,
            f"Replaying {name} ({frames} frames)",
            CLR_GREEN,
        )
        if self._replay_progress_label is not None:
            self._replay_progress_label.text = f"Frame 1 / {frames}"
        self._sync_controls()

    def _on_replay_frame_applied(self, frame_index: int) -> None:
        """Update the in-UI frame indicator and terminal log for each applied frame.

        Args:
            frame_index: Zero-based replay frame index applied by the replayer.
        """
        if self._replay_active_episode is None:
            return
        total = (
            self._replay_frame_counts[self._replay_active_episode]
            if 0 <= self._replay_active_episode < len(self._replay_frame_counts)
            else 0
        )
        if self._replay_progress_label is not None:
            now = time.perf_counter()
            is_edge_frame = frame_index == 0 or (total > 0 and frame_index + 1 >= total)
            if is_edge_frame or now - self._last_replay_ui_update_time >= _REPLAY_UI_UPDATE_INTERVAL:
                self._replay_progress_label.text = f"Frame {frame_index + 1} / {total}"
                self._last_replay_ui_update_time = now
            if is_edge_frame or now - self._last_replay_log_time >= _REPLAY_LOG_INTERVAL:
                carb.log_info(f"[EpisodeRecorder][UI] Replay: frame {frame_index + 1}/{total}")
                self._last_replay_log_time = now

    def _on_replay_finished(self) -> None:
        """Called by the replayer when it reaches the last frame in non-loop mode."""
        self._stop_replay(reason="finished")

    def _stop_replay(self, *, reason: str = "user") -> None:
        was_attached = self._replay_attached
        self._cancel_replay_start_task()
        if self._replayer is not None:
            try:
                self._replayer.stop_replay()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder][UI] stop_replay raised: {exc}")
        self._replay_attached = False
        self._replay_active_episode = None
        self._last_replay_ui_update_time = 0.0
        self._last_replay_log_time = 0.0
        if was_attached:
            carb.log_info(f"[EpisodeRecorder][UI] Replay: stopped (reason={reason})")
            status_map = {
                "user": ("Replay stopped.", CLR_DIM),
                "finished": ("Replay finished.", CLR_GREEN),
                "stage_closed": ("Replay stopped (stage closed).", CLR_YELLOW),
            }
            text, color = status_map.get(reason, ("Replay stopped.", CLR_DIM))
            set_status(self._replay_status_label, text, color, emit_terminal=reason != "user")
        else:
            set_status(self._replay_status_label, "Replay stopped.", CLR_DIM)
        if self._replay_progress_label is not None:
            self._replay_progress_label.text = ""
        self._sync_controls()

    def _cancel_replay_start_task(self) -> None:
        """Cancel an in-flight :meth:`_start_replay_async` if the user hit Stop mid-prepare."""
        task = self._replay_start_task
        self._replay_start_task = None
        if task is not None and not task.done():
            task.cancel()

    def _close_replayer_silently(self) -> None:
        self._cancel_replay_start_task()
        if self._replay_attached and self._replayer is not None:
            try:
                self._replayer.stop_replay()
            except Exception:
                pass
        self._replay_attached = False
        self._replay_active_episode = None
        self._last_replay_ui_update_time = 0.0
        self._last_replay_log_time = 0.0
        if self._replayer is not None:
            try:
                self._replayer.close()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder][UI] replayer.close raised: {exc}")
        self._replayer = None
        self._replay_episodes = []
        self._replay_frame_counts = []
        if self._replay_episode_combo is not None:
            model = self._replay_episode_combo.model
            for child in model.get_item_children():
                model.remove_item(child)
        if self._replay_info_label is not None:
            self._replay_info_label.text = ""
        if self._replay_progress_label is not None:
            self._replay_progress_label.text = ""
        self._clear_replay_warning()

    def _refresh_replay_stage_check(self) -> None:
        """Compare prim paths in the loaded session manifest against the current stage.

        Surfaces a red warning row when one or more required prims are missing so the
        user is not surprised by silent fall-through at replay time.
        """
        if self._replay_warning_label is None:
            return
        if self._replayer is None:
            self._clear_replay_warning()
            return

        try:
            manifest = self._replayer.manifest()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"[EpisodeRecorder][UI] manifest read failed: {exc}")
            self._clear_replay_warning()
            return

        required_paths: list[str] = []
        seen_paths: set[str] = set()
        for entry in manifest.tracks:
            if not isinstance(entry, dict):
                continue
            prim_path = entry.get("prim_path")
            if prim_path:
                key = str(prim_path)
                if key not in seen_paths:
                    seen_paths.add(key)
                    required_paths.append(key)
            for link_path in entry.get("link_paths") or ():
                key = str(link_path)
                if key and key not in seen_paths:
                    seen_paths.add(key)
                    required_paths.append(key)
        if not required_paths:
            self._clear_replay_warning()
            return

        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        except Exception:
            stage = None
        if stage is None:
            self._set_replay_warning("No stage is currently open. Replay will fail until you load the matching scene.")
            return

        missing: list[str] = []
        for path in required_paths:
            if not stage.GetPrimAtPath(path).IsValid():
                missing.append(path)
        if not missing:
            self._clear_replay_warning()
            return
        preview = ", ".join(missing[:3])
        if len(missing) > 3:
            preview += f", \u2026 ({len(missing) - 3} more)"
        self._set_replay_warning(
            f"{len(missing)} recorded prim/link path(s) not found in the current stage: {preview}. "
            "Open the matching scene before pressing Play."
        )

    def _set_replay_warning(self, message: str) -> None:
        if self._replay_warning_label is not None:
            self._replay_warning_label.text = message
        if self._replay_warning_row is not None:
            self._replay_warning_row.visible = True

    def _clear_replay_warning(self) -> None:
        if self._replay_warning_label is not None:
            self._replay_warning_label.text = ""
        if self._replay_warning_row is not None:
            self._replay_warning_row.visible = False

    # ------------------------------------------------------------------
    # Recorder SessionEvents observer
    # ------------------------------------------------------------------

    def _subscribe_recorder_events(self, recorder: EpisodeRecorder) -> None:
        """Bind UI refresh to :class:`SessionEvents`.

        Events fire synchronously from the recorder's own state transitions, so
        ``recorder.is_recording`` is authoritative when the handler runs.

        Args:
            recorder: Recorder whose episode lifecycle events drive the UI state.
        """
        self._unsubscribe_recorder_events()
        self._recorder_event_subs = [
            recorder.events.add_episode_started(lambda _idx: self._refresh_recording_state()),
            recorder.events.add_episode_ended(lambda _idx, _s, _f: self._refresh_recording_state()),
        ]

    def _unsubscribe_recorder_events(self) -> None:
        for unsub in self._recorder_event_subs:
            try:
                unsub()
            except Exception:
                pass
        self._recorder_event_subs.clear()

    def _is_recording(self) -> bool:
        return self._recorder is not None and self._recorder.is_recording

    def _refresh_recording_state(self) -> None:
        def update_ui() -> None:
            recording = self._is_recording()
            if recording and not self._was_recording:
                self._episode_count += 1
                set_status(
                    self._status_label,
                    f"Recording episode #{self._episode_count}",
                    CLR_GREEN,
                    emit_terminal=True,
                )
            elif self._was_recording and not recording:
                set_status(
                    self._status_label,
                    f"Standby - {self._episode_count} episode(s) captured",
                    CLR_YELLOW,
                    emit_terminal=True,
                )
            self._was_recording = recording
            self._sync_controls()

        import asyncio

        asyncio.get_event_loop().call_soon_threadsafe(update_ui)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_stage_closed(self) -> None:
        """Close the active recorder / replayer when the stage goes away."""
        was_recording = self._is_recording()
        active_hdf5_path = self._recorder.hdf5_path if self._recorder is not None else None
        was_replay_attached = self._replay_attached

        if self._replay_attached:
            self._stop_replay(reason="stage_closed")
        self._close_recorder_silently()
        self._close_replayer_silently()
        set_status(self._session_label, "No session", CLR_DIM)
        set_status(self._status_label, "Session closed (stage closed)", CLR_YELLOW)
        set_status(self._replay_status_label, "Replay released (stage closed)", CLR_YELLOW)
        self._sync_controls()

        if was_recording and active_hdf5_path:
            self._notify_stage_close_session(active_hdf5_path)
        elif was_replay_attached:
            self._notify_stage_close_replay()

    def _notify_stage_close_session(self, hdf5_path: str) -> None:
        """Surface a non-blocking notification when an active recording was force-stopped.

        Args:
            hdf5_path: Path to the HDF5 file flushed during stage close.
        """
        message = f"Stage closed while recording was in progress.\nCaptured frames were flushed to:\n{hdf5_path}"
        carb.log_warn(f"[EpisodeRecorder][UI] {message}")
        try:
            from omni.kit.notification_manager import NotificationStatus, post_notification

            post_notification(message, status=NotificationStatus.WARNING, duration=8.0)
        except Exception:
            pass

    def _notify_stage_close_replay(self) -> None:
        """Surface a non-blocking notification when an active replay was force-stopped."""
        message = "Stage closed while replay was active. Replay was stopped."
        carb.log_warn(f"[EpisodeRecorder][UI] {message}")
        try:
            from omni.kit.notification_manager import NotificationStatus, post_notification

            post_notification(message, status=NotificationStatus.WARNING, duration=5.0)
        except Exception:
            pass

    def destroy(self) -> None:
        """Close any active session / replayer. Safe to call from window teardown."""
        self._close_recorder_silently()
        self._close_replayer_silently()
        self._binding_event_sub = None
        self._bindings.clear()

    def _on_binding_event(self, event: Any) -> None:
        """Track external bindings advertised on :data:`EPISODE_BINDING_EVENT`.

        Bindings carrying a non-empty ``session_id`` are dropped when they target a
        different recorder than the one this panel currently owns; this prevents
        cross-talk between concurrent sessions sharing the same event bus. Events
        without a ``session_id`` (or with a falsy one) are treated as global
        advertisements and always counted.

        Args:
            event: Event-dispatcher object containing the binding payload.
        """
        try:
            payload = dict(event.payload) if hasattr(event, "payload") else {}
            action = payload.get("action")
            binding_id = payload.get("binding_id")
            if not binding_id:
                return
            event_session_id = payload.get("session_id")
            if event_session_id and not self._matches_active_session(event_session_id):
                return
            if action == "attach":
                self._bindings[binding_id] = payload
            elif action == "detach":
                self._bindings.pop(binding_id, None)
            else:
                return
            self._refresh_binding_badge()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"[EpisodeRecorder][UI] _on_binding_event failed: {exc}")

    def _matches_active_session(self, event_session_id: str) -> bool:
        """Return True when a binding event's ``session_id`` targets this panel's recorder.

        When no recorder is bound yet, scoped events cannot be addressed to this panel,
        so they are dropped. Once a recorder exists, scoped events must match its
        ``session_id`` exactly.

        Args:
            event_session_id: Session identifier carried by the binding event.

        Returns:
            True when the event targets the active recorder, False otherwise.
        """
        recorder = self._recorder
        if recorder is None:
            return False
        try:
            current_session_id = recorder.session_id
        except Exception:
            return False
        if not current_session_id:
            return False
        return str(event_session_id) == str(current_session_id)

    def _refresh_binding_badge(self) -> None:
        """Update the binding badge label based on currently advertised bindings."""
        if self._binding_badge is None:
            return
        if not self._bindings:
            self._binding_badge.text = ""
            self._binding_badge.tooltip = (
                "External inputs (e.g. VR controller buttons) currently wired to this recorder."
            )
            return

        labels: list[str] = []
        details: list[str] = []
        for payload in self._bindings.values():
            label = payload.get("label") or payload.get("binding_id") or "binding"
            command = payload.get("command") or "?"
            labels.append(str(label))
            details.append(f"{label} \u2192 {command}")

        if len(labels) == 1:
            self._binding_badge.text = f"\u25cf {labels[0]}"
        else:
            self._binding_badge.text = f"\u25cf {len(labels)} bindings"
        self._binding_badge.tooltip = "External inputs currently wired to this recorder:\n  " + "\n  ".join(details)

    def _close_recorder_silently(self) -> list[str]:
        errors: list[str] = []
        if self._timeline_controller is not None:
            try:
                self._timeline_controller.disable()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder][UI] timeline_controller.disable raised: {exc}")
                errors.append(str(exc))
            self._timeline_controller = None

        self._unsubscribe_recorder_events()

        if self._recorder is not None:
            try:
                self._recorder.close_session()
            except Exception as exc:
                carb.log_warn(f"[EpisodeRecorder][UI] close_session raised: {exc}")
                errors.append(str(exc))
            self._recorder = None

        self._was_recording = False
        return errors

    # ------------------------------------------------------------------
    # UI state
    # ------------------------------------------------------------------

    def _sync_controls(self) -> None:
        session_open = self._recorder is not None
        recording = self._is_recording()
        replay_loaded = self._replayer is not None
        replay_attached = self._replay_attached

        if self._session_btn is not None:
            self._session_btn.text = "Close Session" if session_open else "Open Session"
            self._session_btn.enabled = not replay_attached

        if self._episode_btn is not None:
            self._episode_btn.text = "End" if recording else "Start"
            self._episode_btn.enabled = session_open and not replay_attached

        options_editable = not session_open and not replay_attached
        for widget in (
            self._root_path_field,
            self._output_dir_field,
            self._file_prefix_field,
            self._export_snapshot_btn,
            self._discover_btn,
        ):
            if widget is not None:
                widget.enabled = options_editable
        if self._auto_start_check is not None:
            self._auto_start_check.enabled = not replay_attached

        if self._replay_btn is not None:
            self._replay_btn.text = f"{GLYPHS['timeline_stop']}" if replay_attached else f"{GLYPHS['timeline_play']}"
            self._replay_btn.enabled = (replay_loaded and not session_open) or replay_attached
        if self._replay_pause_btn is not None:
            is_paused = bool(replay_attached and self._replayer is not None and self._replayer.is_paused)
            self._replay_pause_btn.text = f"{GLYPHS['timeline_play']}" if is_paused else f"{GLYPHS['timeline_pause']}"
            self._replay_pause_btn.enabled = replay_attached
        if self._replay_prev_btn is not None or self._replay_next_btn is not None:
            cur = self._replayer.current_frame if (replay_attached and self._replayer is not None) else 0
            total = self._replayer.total_frames if (replay_attached and self._replayer is not None) else 0
            if self._replay_prev_btn is not None:
                self._replay_prev_btn.enabled = replay_attached and cur > 0
            if self._replay_next_btn is not None:
                self._replay_next_btn.enabled = replay_attached and cur < max(0, total - 1)
        for widget in (self._replay_file_field, self._replay_latest_btn, self._replay_load_btn):
            if widget is not None:
                widget.enabled = not replay_attached and not session_open
        if self._replay_episode_combo is not None:
            self._replay_episode_combo.enabled = replay_loaded and not replay_attached
        if self._replay_seek_timeline_check is not None:
            self._replay_seek_timeline_check.enabled = not replay_attached

    # ------------------------------------------------------------------
    # Settings helpers
    # ------------------------------------------------------------------

    def _get_root_path(self) -> str:
        if self._root_path_field is None:
            return _DEFAULT_ROOT_PATH
        return self._root_path_field.model.get_value_as_string().strip() or _DEFAULT_ROOT_PATH

    def _get_output_dir(self) -> str:
        default = os.path.join(os.getcwd(), _DEFAULT_OUTPUT_SUBDIR)
        if self._output_dir_field is None:
            return default
        return self._output_dir_field.model.get_value_as_string().strip() or default

    def _get_file_prefix(self) -> str:
        if self._file_prefix_field is None:
            return _DEFAULT_FILE_PREFIX
        return self._file_prefix_field.model.get_value_as_string().strip() or _DEFAULT_FILE_PREFIX

    def _get_auto_start_on_play(self) -> bool:
        if self._auto_start_model is None:
            return self._load_bool("auto_start_on_play", _DEFAULT_AUTO_START_ON_PLAY)
        return bool(self._auto_start_model.get_value_as_bool())

    def _get_replay_seek_timeline(self) -> bool:
        if self._replay_seek_timeline_model is None:
            return self._load_bool("replay_seek_timeline", _DEFAULT_REPLAY_SEEK_TIMELINE)
        return bool(self._replay_seek_timeline_model.get_value_as_bool())

    def _load_str(self, key: str) -> str:
        return self._settings.get_as_string(f"{_SETTINGS_PREFIX}/{key}") or ""

    def _save_str(self, key: str, value: str) -> None:
        self._settings.set_string(f"{_SETTINGS_PREFIX}/{key}", value)

    def _load_bool(self, key: str, default: bool) -> bool:
        full_key = f"{_SETTINGS_PREFIX}/{key}"
        value = self._settings.get(full_key)
        if value is None:
            return default
        return bool(value)

    def _save_bool(self, key: str, value: bool) -> None:
        self._settings.set_bool(f"{_SETTINGS_PREFIX}/{key}", bool(value))
