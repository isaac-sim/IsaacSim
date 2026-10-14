# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Telemetry panel: source loading, time window, chunk table."""

from __future__ import annotations

import asyncio
import os
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from math import inf
from pathlib import Path
from typing import Any, Optional

import carb
import omni.kit.app
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import StringField, TextBlock
from isaacsim.gui.components.ui_utils import dropdown_builder
from isaacsim.robot_setup.sysid.ingest import load_trajectory
from isaacsim.robot_setup.sysid.ingest.config_types import (
    TrajectoryIngestError,
    TrajectoryLoadConfig,
    TrajectorySourceType,
)
from isaacsim.robot_setup.sysid.telemetry_quality import build_telemetry_quality_report
from isaacsim.robot_setup.sysid.training_logs import (
    attach_companion_training_details,
    find_companion_mcap_path,
    phase_chunks_from_metadata,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryCsvError, TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import (
    DEFAULT_AUTO_MIN_CHUNK_SECONDS,
    DEFAULT_AUTO_TRAIN_FRACTION,
    DEFAULT_EXCITATION_TAGS,
    TELEMETRY_CHUNK_ROLE_TRAIN,
    TELEMETRY_CHUNK_ROLE_VALIDATION,
    TelemetryChunkRunSpec,
    TrajectorySegmentError,
    build_trajectory_chunks,
    resolve_chunk_specs,
)
from isaacsim.robot_setup.sysid.ui.numeric_inputs import ExactFloatField
from isaacsim.robot_setup.sysid.ui.styles import MUTED_LABEL_STYLE

_TELEMETRY_SOURCE_LABELS = ("CSV", "ROS 2 Bag", "MCAP", "LeRobot")
_TELEMETRY_SOURCE_TYPES = {
    "CSV": TrajectorySourceType.CSV,
    "ROS 2 Bag": TrajectorySourceType.ROS2_BAG,
    "MCAP": TrajectorySourceType.MCAP,
    "LeRobot": TrajectorySourceType.LEROBOT,
}
_CHUNK_ROLE_LABELS = ("Train", "Validation")
_CHUNK_ROLE_TYPES = {
    "Train": TELEMETRY_CHUNK_ROLE_TRAIN,
    "Validation": TELEMETRY_CHUNK_ROLE_VALIDATION,
}
_CHUNK_EXCITATION_LABELS = tuple(DEFAULT_EXCITATION_TAGS)
_AUTO_SPLIT_MIN_PERCENT = 5.0
_AUTO_SPLIT_MAX_PERCENT = 95.0
_CHUNK_TABLE_EMPTY_HEIGHT = 72
_CHUNK_TABLE_HEADER_HEIGHT = 44
_CHUNK_TABLE_ROW_HEIGHT = 26
_CHUNK_TABLE_PADDING = 8
# Cap the visible rows so a heavy chunk set scrolls inside the panel instead of
# pushing the rest of the workflow off-screen. Extra rows are reachable by scroll.
_CHUNK_TABLE_MAX_VISIBLE_ROWS = 8
_CHUNK_TIMELINE_HEIGHT = 62
_CHUNK_TIMELINE_TRACK_HEIGHT = 14
_CHUNK_TIMELINE_MIN_FRACTION = 1e-6
_CHUNK_TIMELINE_GAP_COLOR = 0xFF383838
_CHUNK_TIMELINE_TRAIN_COLOR = 0xFF5A9E58
_CHUNK_TIMELINE_VALIDATION_COLOR = 0xFFC28D4A
_CHUNK_TIMELINE_DEFAULT_COLOR = 0xFF6D887C
_CHUNK_PREVIEW_PLAYBACK_SPEED = 4.0
_CHUNK_PREVIEW_PLAYBACK_FPS = 30.0
_CHUNK_PREVIEW_PLAYBACK_MIN_FRAMES = 8
_CHUNK_PREVIEW_PLAYBACK_MAX_FRAMES = 240

_CSV_TOOLTIP = (
    "Columns: time, then q1..qN (positions). Optional dq1..dqN (velocities) and cmd1..cmdN (commands). "
    "Supports 1+N, 1+2N, or 1+3N columns; header row recommended. Units match articulation DOFs (rad or m)."
)
_MAPPING_TOOLTIP = (
    "Optional JSON/YAML mapping file. CSV: column_map_example.json. Bag/MCAP: topic_map_schema.yaml. "
    "Declare what the effort channel measures with 'torque_semantics: link_side|external' "
    "(external, e.g. Franka tau_ext_hat_filtered, carries no parameter information contact-free)."
)
_CHUNK_MANIFEST_TOOLTIP = (
    "Optional run folder, collection_manifest.json, or phase_events.jsonl used to import train/validation chunks."
)


@dataclass(frozen=True)
class _ChunkTimelineSegment:
    kind: str
    fraction: float
    color: int
    tooltip: str


def _normalize_path(path: str) -> str:
    return path.strip().strip('"').strip("'")


def _load_blocking(
    path: str,
    source_type: TrajectorySourceType,
    mapping_path: str,
    chunk_manifest_path: str,
    command_alignment_seconds: float = 0.0,
) -> TrajectoryDataset:
    """Run in a thread-pool executor - no UI calls allowed here.

    Mapping and chunk metadata paths are always explicit user inputs.

    Args:
        path: Path to use.
        source_type: Source type value.
        mapping_path: Mapping path value.
        chunk_manifest_path: Chunk manifest path value.
        command_alignment_seconds: Command alignment in seconds.

    Returns:
        The resulting value.
    """
    # Source-exists check (may involve filesystem globs for MCAP).
    if source_type == TrajectorySourceType.MCAP:
        resolved = find_companion_mcap_path(path)
        if not os.path.isfile(resolved):
            raise TrajectoryIngestError(f"MCAP file not found: {path}")
    elif source_type not in (TrajectorySourceType.ROS2_BAG, TrajectorySourceType.LEROBOT):
        if not os.path.isfile(path):
            raise TrajectoryIngestError(f"File not found: {path}")

    mapping = mapping_path or None
    if source_type == TrajectorySourceType.CSV:
        config = TrajectoryLoadConfig.csv(path, column_mapping_path=mapping)
    elif source_type == TrajectorySourceType.ROS2_BAG:
        config = TrajectoryLoadConfig.ros2_bag(path, topic_mapping_path=mapping)
    elif source_type == TrajectorySourceType.MCAP:
        config = TrajectoryLoadConfig.mcap(path, topic_mapping_path=mapping)
    else:
        config = TrajectoryLoadConfig.lerobot(path)
    # Same contract as TelemetryRunSpec.to_load_config(): the loader shifts the
    # command channel by the alignment offset, so the loaded trajectory is already realigned.
    config.command_alignment_seconds = float(command_alignment_seconds)

    trajectory = load_trajectory(config)
    if chunk_manifest_path:
        manifest = Path(chunk_manifest_path).expanduser()
        if not manifest.exists():
            raise TrajectoryIngestError(f"Chunk manifest path not found: {manifest}")
        trajectory = attach_companion_training_details(trajectory, str(manifest))
    return trajectory


def _actionable_error(exc: Exception, source_type: TrajectorySourceType) -> str:
    """Return a short, user-friendly error string with remediation hints.

    Args:
        exc: Exc value.
        source_type: Source type value.

    Returns:
        The resulting value.
    """
    msg = str(exc)
    low = msg.lower()
    if "not found" in low or "no such file" in low:
        return "File not found. Check the path is correct."
    if "column" in low and ("mismatch" in low or "expect" in low or "count" in low):
        return f"{msg}. Verify the CSV has: time, then one column per joint DOF."
    if "strictly increasing" in low:
        return "Timestamps are not strictly increasing. Check for duplicate rows or clock resets in the data."
    if "joints" in low or "dof" in low:
        return f"{msg}. The number of data columns must match the robot's DOF count."
    if source_type == TrajectorySourceType.LEROBOT and ("timeout" in low or "connect" in low or "network" in low):
        return "Could not reach LeRobot/HuggingFace Hub. Check your network connection or use a local dataset path."
    return msg


def _telemetry_item_filter(source_type: TrajectorySourceType) -> Any:
    def _filter(item: Any) -> bool:
        if item is None:
            return True
        path = getattr(item, "path", None) or str(item)
        if not path:
            return True
        if getattr(item, "is_folder", False) or os.path.isdir(path):
            return source_type in (
                TrajectorySourceType.ROS2_BAG,
                TrajectorySourceType.MCAP,
                TrajectorySourceType.LEROBOT,
            )
        lower = path.lower()
        if source_type == TrajectorySourceType.CSV:
            return lower.endswith(".csv")
        if source_type == TrajectorySourceType.MCAP:
            return lower.endswith(".mcap")
        if source_type == TrajectorySourceType.ROS2_BAG:
            return lower.endswith((".mcap", ".db3", ".yaml", ".bag"))
        return True

    return _filter


def _mapping_item_filter(item: Any) -> bool:
    if item is None:
        return True
    path = getattr(item, "path", None) or str(item)
    if not path:
        return True
    if getattr(item, "is_folder", False) or os.path.isdir(path):
        return True
    return path.lower().endswith((".json", ".yaml", ".yml"))


def _chunk_manifest_item_filter(item: Any) -> bool:
    if item is None:
        return True
    path = getattr(item, "path", None) or str(item)
    if not path:
        return True
    if getattr(item, "is_folder", False) or os.path.isdir(path):
        return True
    name = os.path.basename(path).lower()
    return name in ("collection_manifest.json", "phase_events.jsonl", "run_metadata.json")


class TelemetryPanel:
    """Build and manage the Telemetry collapsible section.

    Args:
        on_trajectory_changed: Optional callback fired after telemetry is loaded or cleared.
        on_run_state_changed: Optional callback fired when telemetry readiness changes.
        on_preview_requested: Optional callback used to create a trajectory preview.
        on_preview_cleared: Optional callback used to clear the trajectory preview.
    """

    def __init__(
        self,
        on_trajectory_changed: Callable[[Optional[TrajectoryDataset]], None] | None = None,
        on_run_state_changed: Callable[[], None] | None = None,
        on_preview_requested: Callable[[TrajectoryDataset, float], str] | None = None,
        on_preview_cleared: Callable[[], None] | None = None,
    ) -> None:
        self._on_trajectory_changed = on_trajectory_changed
        self._on_run_state_changed = on_run_state_changed
        self._on_preview_requested = on_preview_requested
        self._on_preview_cleared = on_preview_cleared

        self._trajectory: Optional[TrajectoryDataset] = None
        self._telemetry_chunks: list[TelemetryChunkRunSpec] = []
        self._source_label: str = "CSV"
        self._chunk_role_label: str = "Train"
        self._chunk_excitation_label: str = "custom"
        self._preview_chunk_index: int = 0
        self._load_task: Optional[asyncio.Task] = None
        self._preview_play_task: Optional[asyncio.Task] = None
        self._loaded_source_signature: Optional[tuple] = None
        self._preview_enabled = False
        self._controls_enabled = True

        self._source_path_field: Optional[StringField] = None
        self._mapping_path_field: Optional[StringField] = None
        self._chunk_manifest_path_field: Optional[StringField] = None
        self._command_alignment_field: Optional[ExactFloatField] = None
        self._load_button: Optional[ui.Button] = None
        self._preview_button: Optional[ui.Button] = None
        self._start_time_field: Optional[ExactFloatField] = None
        self._end_time_field: Optional[ExactFloatField] = None
        self._source_summary_label: Optional[TextBlock] = None
        self._chunk_name_field: Optional[StringField] = None
        self._chunk_weight_field: Optional[ExactFloatField] = None
        self._chunk_custom_excitation_field: Optional[StringField] = None
        self._chunk_name_model: Optional[ui.SimpleStringModel] = None
        self._chunk_weight_model = None
        self._chunk_weight_widget: Optional[ui.FloatField] = None
        self._chunk_custom_excitation_model: Optional[ui.SimpleStringModel] = None
        self._auto_split_model = None
        self._auto_split_fraction_model = None
        self._auto_split_min_seconds_model = None
        self._auto_split_widgets: list = []
        self._auto_split_hint_label: Optional[ui.Label] = None
        self._chunk_timeline_frame: Optional[ui.Frame] = None
        self._chunk_table_frame: Optional[ui.Frame] = None
        self._chunk_status_label: Optional[TextBlock] = None
        self._status_label: Optional[TextBlock] = None

        self.wrapped_ui_elements: list = []

    # ------------------------------------------------------------------ build

    def build(self) -> None:
        """Handle build."""
        with ui.VStack(spacing=6):
            dropdown_builder(
                label="Source type",
                default_val=0,
                items=list(_TELEMETRY_SOURCE_LABELS),
                tooltip="Telemetry file format",
                on_clicked_fn=self._on_source_type_changed,
            )
            self._source_path_field = StringField(
                "Data source path",
                tooltip=_CSV_TOOLTIP,
                default_value="",
                on_value_changed_fn=self._on_source_changed,
                use_folder_picker=True,
                item_filter_fn=lambda item: _telemetry_item_filter(self.get_source_type())(item),
                folder_dialog_title="Select telemetry source",
                folder_button_title="Select",
            )
            self._source_path_field.file_picker_frame.width = ui.Length(28)
            self._mapping_path_field = StringField(
                "Mapping config",
                tooltip=_MAPPING_TOOLTIP,
                default_value="",
                on_value_changed_fn=self._on_source_changed,
                use_folder_picker=True,
                item_filter_fn=_mapping_item_filter,
                folder_dialog_title="Select mapping config",
                folder_button_title="Select",
            )
            self._mapping_path_field.file_picker_frame.width = ui.Length(28)
            self._chunk_manifest_path_field = StringField(
                "Chunk manifest",
                tooltip=_CHUNK_MANIFEST_TOOLTIP,
                default_value="",
                on_value_changed_fn=self._on_source_changed,
                use_folder_picker=True,
                item_filter_fn=_chunk_manifest_item_filter,
                folder_dialog_title="Select chunk manifest or run folder",
                folder_button_title="Select",
            )
            self._chunk_manifest_path_field.file_picker_frame.width = ui.Length(28)
            self._command_alignment_field = ExactFloatField(
                "Command alignment (s)",
                default_value=0.0,
                step=0.001,
                format="%.4f",
                tooltip=(
                    "Common-mode command transport offset removed at load: command'(t) = command(t - offset), "
                    "uniform across joints. Run Check (step 4) to measure it from the telemetry; leave 0 "
                    "when commands and responses are already aligned. Changing it requires a reload."
                ),
                on_value_changed_fn=self._on_source_changed,
            )
            with ui.HStack(spacing=8, height=28):
                ui.Spacer(width=ui.Fraction(1))
                self._load_button = ui.Button(
                    "Load Trajectory",
                    width=128,
                    height=24,
                    clicked_fn=self._on_load_trajectory_clicked,
                    tooltip="Load the selected telemetry source, mapping config, and chunk manifest.",
                )
            ui.Label("Analysis window", style={"font_size": 13})
            with ui.HStack(spacing=8):
                self._start_time_field = ExactFloatField(
                    "Start time (s)",
                    default_value=0.0,
                    tooltip="Optimization window start (seconds)",
                    on_value_changed_fn=self._on_analysis_window_changed,
                )
                self._end_time_field = ExactFloatField(
                    "End time (s)",
                    default_value=1.0,
                    tooltip="Optimization window end (seconds)",
                    on_value_changed_fn=self._on_analysis_window_changed,
                )
                self.wrapped_ui_elements.extend([self._start_time_field, self._end_time_field])
            self._source_summary_label = TextBlock(
                "Telemetry",
                "Load telemetry to inspect samples and channels.",
                include_copy_button=False,
                num_lines=3,
            )

            ui.Label("Dataset partitioning", style={"font_size": 14})
            ui.Label(
                "Choose an automatic validation holdout or define non-overlapping manual chunks.",
                word_wrap=True,
                style=MUTED_LABEL_STYLE,
            )
            ui.Label("Automatic validation holdout", style={"font_size": 13})
            with ui.HStack(spacing=6, height=24):
                ui.Label(
                    "Enabled",
                    width=92,
                    tooltip="Automatically hold out the trailing part of the window for validation.",
                )
                auto_split_cb = ui.CheckBox(width=22, height=22)
                self._auto_split_model = auto_split_cb.model
                self._auto_split_model.set_value(False)
                self._auto_split_model.add_value_changed_fn(lambda _m: self._on_auto_split_changed())
                ui.Label(
                    "Reserve the trailing part of the analysis window for validation.",
                    tooltip="Manual chunks take precedence over automatic holdout.",
                )
                ui.Spacer()
            with ui.HStack(spacing=6, height=24):
                ui.Label(
                    "Training share (%)",
                    width=112,
                    alignment=ui.Alignment.LEFT_CENTER,
                    tooltip="Percentage of the analysis window used for training.",
                )
                fraction_field = ui.FloatField(
                    width=72,
                    height=22,
                    tooltip="Enter 5-95; the remaining percentage is held out for validation.",
                )
                fraction_field.model.set_min(_AUTO_SPLIT_MIN_PERCENT)
                fraction_field.model.set_max(_AUTO_SPLIT_MAX_PERCENT)
                fraction_field.model.set_value(DEFAULT_AUTO_TRAIN_FRACTION * 100.0)
                fraction_field.model.add_value_changed_fn(lambda _m: self._on_auto_split_changed())
                self._auto_split_fraction_model = fraction_field.model
                ui.Label("Minimum duration (s)", width=126, tooltip="Shortest partition the split may produce.")
                min_seconds_field = ui.FloatField(
                    width=72,
                    height=22,
                    tooltip="Shorter windows fall back to one training chunk.",
                )
                min_seconds_field.model.set_min(0.0)
                min_seconds_field.model.set_max(inf)
                min_seconds_field.model.set_value(DEFAULT_AUTO_MIN_CHUNK_SECONDS)
                min_seconds_field.model.add_value_changed_fn(lambda _m: self._on_auto_split_changed())
                self._auto_split_min_seconds_model = min_seconds_field.model
                ui.Spacer()
                self._auto_split_widgets = [auto_split_cb, fraction_field, min_seconds_field]
            # Disabled widgets do not surface their tooltip, so state the reason inline.
            self._auto_split_hint_label = ui.Label(
                "",
                height=0,
                visible=False,
                word_wrap=True,
                style={"font_size": 12, "color": 0xFF9AA0A6},
            )
            ui.Label("Manual chunk editor", style={"font_size": 13})
            with ui.HStack(spacing=6, height=24):
                ui.Label("Name", width=42, tooltip="Optional chunk display name")
                self._chunk_name_model = ui.SimpleStringModel("")
                ui.StringField(
                    model=self._chunk_name_model,
                    width=120,
                    height=22,
                    tooltip="Optional chunk display name",
                )
                ui.Label("Role", width=34, tooltip="Train chunks optimize; validation chunks are held out.")
                role_box = ui.ComboBox(
                    0,
                    *_CHUNK_ROLE_LABELS,
                    width=86,
                    height=22,
                    tooltip="Train chunks optimize; validation chunks are held out.",
                )
                role_box.model.add_item_changed_fn(
                    lambda m, _v: self._on_chunk_role_changed(_CHUNK_ROLE_LABELS[m.get_item_value_model().as_int])
                )
                ui.Label("Train weight", width=76, tooltip="Relative weight used only for training chunks.")
                self._chunk_weight_widget = ui.FloatField(
                    width=64,
                    height=22,
                    tooltip="Relative optimization weight; validation chunks do not use a weight.",
                )
                self._chunk_weight_widget.model.set_min(1e-6)
                self._chunk_weight_widget.model.set_max(inf)
                self._chunk_weight_widget.model.set_value(1.0)
                self._chunk_weight_model = self._chunk_weight_widget.model

            with ui.HStack(spacing=6, height=24):
                ui.Label("Excitation", width=70, tooltip="Tag used for grouped validation metrics.")
                excitation_box = ui.ComboBox(
                    _CHUNK_EXCITATION_LABELS.index("custom"),
                    *_CHUNK_EXCITATION_LABELS,
                    width=108,
                    height=22,
                    tooltip="Tag used for grouped validation metrics.",
                )
                excitation_box.model.add_item_changed_fn(
                    lambda m, _v: self._on_chunk_excitation_changed(
                        _CHUNK_EXCITATION_LABELS[m.get_item_value_model().as_int]
                    )
                )
                ui.Label("Custom tag", width=68, tooltip="Optional domain-specific excitation tag")
                self._chunk_custom_excitation_model = ui.SimpleStringModel("custom")
                ui.StringField(
                    model=self._chunk_custom_excitation_model,
                    width=110,
                    height=22,
                    tooltip="Used for custom excitation, or to override a predefined tag.",
                )
                ui.Button(
                    "Add Chunk",
                    width=86,
                    height=22,
                    clicked_fn=self._on_add_chunk,
                    tooltip="Add the current analysis window as a manual chunk.",
                )
                ui.Button(
                    "Remove All",
                    width=86,
                    height=22,
                    clicked_fn=self._on_clear_chunks,
                    tooltip="Remove all manual chunks.",
                )

            with ui.HStack(spacing=6, height=24):
                ui.Label("Viewport preview", width=104, tooltip="Inspect chunk ground-truth poses in the viewport.")
                ui.Spacer()
                ui.Button(
                    "Previous",
                    width=66,
                    height=22,
                    clicked_fn=lambda: self._on_step_preview_chunk(-1),
                    tooltip="Preview the previous chunk.",
                )
                self._preview_button = ui.Button(
                    "Preview",
                    width=68,
                    height=22,
                    clicked_fn=self._on_preview_gt_clicked,
                    tooltip="Show GT link/pose markers for the selected chunk sample in the viewport.",
                )
                ui.Button(
                    "Play",
                    width=52,
                    height=22,
                    clicked_fn=self._on_play_preview_chunk_clicked,
                    tooltip=f"Animate the selected chunk preview at {_CHUNK_PREVIEW_PLAYBACK_SPEED:g}x.",
                )
                ui.Button(
                    "Next",
                    width=48,
                    height=22,
                    clicked_fn=lambda: self._on_step_preview_chunk(1),
                    tooltip="Preview the next chunk.",
                )

            self._chunk_timeline_frame = ui.Frame(height=_CHUNK_TIMELINE_HEIGHT, build_fn=self._build_chunk_timeline)
            self._chunk_table_frame = ui.ScrollingFrame(
                height=ui.Length(self._chunk_table_height()),
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                build_fn=self._build_chunk_table,
            )
            self._chunk_status_label = TextBlock(
                "Partition status",
                "No manual chunks. The analysis window will run as one training chunk.",
                include_copy_button=False,
                num_lines=3,
            )

            self._status_label = TextBlock("Status", "", include_copy_button=False, num_lines=3)

            self.wrapped_ui_elements.extend(
                [
                    self._source_path_field,
                    self._mapping_path_field,
                    self._chunk_manifest_path_field,
                    self._command_alignment_field,
                    self._source_summary_label,
                    self._chunk_status_label,
                    self._status_label,
                ]
            )

    # ---------------------------------------------------------------- getters

    def get_trajectory(self) -> Optional[TrajectoryDataset]:
        """Get trajectory.

        Returns:
            The resulting value.
        """
        return self._trajectory

    def get_loaded_signature(self) -> Optional[tuple]:
        """Identity of the currently loaded telemetry source (None before a load).

        Returns:
            The resulting value.
        """
        return self._loaded_source_signature

    def get_source_path(self) -> str:
        """Get source path.

        Returns:
            The resulting value.
        """
        if self._source_path_field is None:
            return ""
        return _normalize_path(self._source_path_field.get_value())

    def get_mapping_path(self) -> str:
        """Get mapping path.

        Returns:
            The resulting value.
        """
        if self._mapping_path_field is None:
            return ""
        return _normalize_path(self._mapping_path_field.get_value())

    def get_chunk_manifest_path(self) -> str:
        """Get chunk manifest path.

        Returns:
            The resulting value.
        """
        if self._chunk_manifest_path_field is None:
            return ""
        return _normalize_path(self._chunk_manifest_path_field.get_value())

    def get_source_type(self) -> TrajectorySourceType:
        """Get source type.

        Returns:
            The resulting value.
        """
        return _TELEMETRY_SOURCE_TYPES.get(self._source_label, TrajectorySourceType.CSV)

    def get_command_alignment_seconds(self) -> float:
        """Get command alignment in seconds.

        Returns:
            The resulting value.
        """
        return float(self._command_alignment_field.get_value()) if self._command_alignment_field else 0.0

    def get_time_window(self) -> tuple[float, float]:
        """Get time window.

        Returns:
            The resulting value.
        """
        t0 = float(self._start_time_field.get_value()) if self._start_time_field else 0.0
        t1 = float(self._end_time_field.get_value()) if self._end_time_field else 1.0
        return t0, t1

    def get_chunk_specs(self) -> list[TelemetryChunkRunSpec]:
        """Get chunk specs.

        Returns:
            The resulting value.
        """
        if self._trajectory is None:
            raise TrajectoryCsvError("Load a valid telemetry source first.")
        auto_split, train_fraction, min_chunk_seconds = self.get_auto_split()
        return resolve_chunk_specs(
            self._trajectory,
            self._telemetry_chunks,
            auto_split=auto_split,
            train_fraction=train_fraction,
            min_chunk_seconds=min_chunk_seconds,
            window=self.get_time_window(),
        )

    def has_explicit_chunks(self) -> bool:
        """Return whether explicit chunks.

        Returns:
            The resulting value.
        """
        return bool(self._telemetry_chunks)

    def get_auto_split(self) -> tuple[bool, float, float]:
        """Return ``(enabled, train_fraction, min_chunk_seconds)``.

        Returns:
            The resulting value.
        """
        enabled = bool(self._auto_split_model.get_value_as_bool()) if self._auto_split_model else False
        fraction = (
            float(self._auto_split_fraction_model.get_value_as_float()) / 100.0
            if self._auto_split_fraction_model
            else DEFAULT_AUTO_TRAIN_FRACTION
        )
        min_seconds = (
            float(self._auto_split_min_seconds_model.get_value_as_float())
            if self._auto_split_min_seconds_model
            else DEFAULT_AUTO_MIN_CHUNK_SECONDS
        )
        if enabled and not 0.05 <= fraction <= 0.95:
            raise ValueError("Training share must be between 5% and 95%.")
        if enabled and min_seconds < 0.0:
            raise ValueError("Minimum duration must be non-negative.")
        return enabled, fraction, min_seconds

    def chunk_validation_error(self, *, require_train: bool) -> str:
        """Handle chunk validation error.

        Args:
            require_train: Require train value.

        Returns:
            The resulting value.
        """
        if self._trajectory is None:
            return ""
        try:
            specs = self._telemetry_chunks if self._telemetry_chunks else self.get_chunk_specs()
            build_trajectory_chunks(self._trajectory, specs, require_train=require_train, reject_overlaps=True)
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            return str(exc)
        return ""

    # ---------------------------------------------------------------- setters

    def set_status(self, text: str) -> None:
        """Set status.

        Args:
            text: Text to display.
        """
        if self._status_label is not None:
            self._status_label.set_text(text)

    def set_command_alignment_seconds(self, value: float, *, reload: bool = True) -> None:
        """Set the command alignment field and (by default) reload the telemetry.

        The alignment is applied by the loader, never to the cached trajectory
        (an in-place shift would compound across applies), so a change only
        takes effect through a reload.

        Args:
            value: Value to apply.
            reload: Reload value.
        """
        if self._command_alignment_field is not None:
            # Fires the value-changed callback, which clears the stale trajectory.
            self._command_alignment_field.set_value(float(value))
        if reload and self.get_source_path():
            self._on_load_trajectory_clicked()

    def set_auto_split(
        self,
        enabled: bool,
        train_fraction: float | None = None,
        min_chunk_seconds: float | None = None,
    ) -> None:
        """Set the automatic-holdout controls (recipe application).

        Args:
            enabled: Whether the option is enabled.
            train_fraction: Training fraction in the range 0-1.
            min_chunk_seconds: Minimum partition duration in seconds.
        """
        # A programmatic `set_value` bypasses the field's own min/max, so a recipe
        # could otherwise install a share the user cannot type.
        if self._auto_split_fraction_model is not None and train_fraction is not None:
            percent = min(max(float(train_fraction) * 100.0, _AUTO_SPLIT_MIN_PERCENT), _AUTO_SPLIT_MAX_PERCENT)
            self._auto_split_fraction_model.set_value(percent)
        if self._auto_split_min_seconds_model is not None and min_chunk_seconds is not None:
            self._auto_split_min_seconds_model.set_value(max(float(min_chunk_seconds), 0.0))
        if self._auto_split_model is not None:
            # Fires the value-changed callback (chunk refresh + run-state update).
            self._auto_split_model.set_value(bool(enabled))

    def set_enabled(self, enabled: bool) -> None:
        """Set enabled.

        Args:
            enabled: Whether the option is enabled.
        """
        self._controls_enabled = bool(enabled)
        for w in (
            self._source_path_field,
            self._mapping_path_field,
            self._chunk_manifest_path_field,
            self._command_alignment_field,
            self._start_time_field,
            self._end_time_field,
        ):
            if w is not None:
                w.enabled = enabled
        if self._load_button is not None:
            self._load_button.enabled = bool(enabled) and not self._is_loading()
        if self._preview_button is not None:
            self._preview_button.enabled = bool(enabled)
        self._refresh_auto_split_enabled()

    # ---------------------------------------------------------------- cleanup

    def cleanup(self) -> None:
        """Release resources."""
        self._stop_preview_playback()
        self._set_preview_enabled(False, clear=True)
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        self._load_task = None
        self._preview_play_task = None
        for element in self.wrapped_ui_elements:
            if hasattr(element, "cleanup"):
                element.cleanup()
        self.wrapped_ui_elements.clear()
        self._source_path_field = None
        self._mapping_path_field = None
        self._chunk_manifest_path_field = None
        self._command_alignment_field = None
        self._load_button = None
        self._preview_button = None
        self._start_time_field = None
        self._end_time_field = None
        self._source_summary_label = None
        self._chunk_name_field = None
        self._chunk_weight_field = None
        self._chunk_custom_excitation_field = None
        self._chunk_name_model = None
        self._chunk_weight_model = None
        self._chunk_weight_widget = None
        self._chunk_custom_excitation_model = None
        self._preview_chunk_index = 0
        self._auto_split_model = None
        self._auto_split_fraction_model = None
        self._auto_split_min_seconds_model = None
        self._auto_split_widgets = []
        self._chunk_timeline_frame = None
        self._chunk_table_frame = None
        self._chunk_status_label = None
        self._status_label = None
        self._loaded_source_signature = None
        self._preview_enabled = False

    # --------------------------------------------------------------- private

    def _on_source_type_changed(self, label: str) -> None:
        self._source_label = label
        self._on_source_changed(None)

    def _on_source_changed(self, _model: Any) -> None:
        self._stop_preview_playback()
        self._set_preview_enabled(False, clear=True)

        # Cancel any in-flight load; loading is explicit and the path set changed.
        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        self._load_task = None
        self._set_load_button_loading(False)

        path = self.get_source_path()
        signature = self._source_signature()
        if self._trajectory is not None and self._loaded_source_signature == signature:
            return

        self._trajectory = None
        self._telemetry_chunks.clear()
        self._preview_chunk_index = 0
        self._loaded_source_signature = None
        if path:
            self._set_source_summary("Telemetry source changed. Click Load Trajectory.")
            self.set_status("Telemetry source changed. Click Load Trajectory.")
        else:
            self._set_source_summary("Load telemetry to inspect samples and channels.")
            self.set_status("")
        self._refresh_chunk_table()
        self._notify_trajectory_changed()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _on_load_trajectory_clicked(self) -> None:
        self._stop_preview_playback()
        self._set_preview_enabled(False, clear=True)

        path = self.get_source_path()
        if not path:
            self._trajectory = None
            self._telemetry_chunks.clear()
            self._preview_chunk_index = 0
            self._loaded_source_signature = None
            self._set_source_summary("Choose a telemetry source path, then Load Trajectory.")
            self._set_chunk_status("No manual chunks. The analysis window will run as one training chunk.")
            self._refresh_chunk_table()
            self.set_status("Select a telemetry source before loading.")
            self._notify_trajectory_changed()
            if self._on_run_state_changed:
                self._on_run_state_changed()
            return

        if self._load_task is not None and not self._load_task.done():
            self._load_task.cancel()
        self._load_task = None

        # Capture UI state on the main thread now; do not read widgets from a worker.
        source_type = self.get_source_type()
        mapping_path = self.get_mapping_path()
        chunk_manifest_path = self.get_chunk_manifest_path()
        command_alignment_seconds = self.get_command_alignment_seconds()

        # Immediate visual feedback so the user knows something is happening.
        filename = Path(path).name or path
        self._set_source_summary(f"Loading {filename}...")
        self.set_status("Loading...")
        self._set_load_button_loading(True)

        self._load_task = asyncio.ensure_future(
            self._load_trajectory_async(path, source_type, mapping_path, chunk_manifest_path, command_alignment_seconds)
        )

    async def _load_trajectory_async(
        self,
        path: str,
        source_type: TrajectorySourceType,
        mapping_path: str,
        chunk_manifest_path: str,
        command_alignment_seconds: float,
    ) -> None:
        """Background load and UI update without blocking the main thread.

        Args:
            path: Path to use.
            source_type: Source type value.
            mapping_path: Mapping path value.
            chunk_manifest_path: Chunk manifest path value.
            command_alignment_seconds: Command delay seconds value.
        """

        def _inputs_changed() -> bool:
            # A newer task may have started, or the user edited an input mid-load.
            return (
                self.get_source_path() != path
                or self.get_source_type() != source_type
                or self.get_mapping_path() != mapping_path
                or self.get_chunk_manifest_path() != chunk_manifest_path
                or self.get_command_alignment_seconds() != command_alignment_seconds
            )

        if _inputs_changed():
            self._finish_load_task_if_current()
            return

        loop = asyncio.get_event_loop()
        try:
            trajectory = await loop.run_in_executor(
                None,
                lambda: _load_blocking(path, source_type, mapping_path, chunk_manifest_path, command_alignment_seconds),
            )
        except asyncio.CancelledError:
            self._finish_load_task_if_current()
            return
        except (TrajectoryCsvError, TrajectoryIngestError) as exc:
            if _inputs_changed():
                self._finish_load_task_if_current()
                return
            msg = _actionable_error(exc, source_type)
            self._trajectory = None
            self._telemetry_chunks.clear()
            self._loaded_source_signature = None
            self._set_source_summary(f"Error: {msg}")
            self._refresh_chunk_table()
            carb.log_error(str(exc))
            self.set_status(f"Error: {msg}")
            self._notify_trajectory_changed()
            if self._on_run_state_changed:
                self._on_run_state_changed()
            self._finish_load_task_if_current()
            return
        except Exception as exc:
            if _inputs_changed():
                self._finish_load_task_if_current()
                return
            self._trajectory = None
            self._telemetry_chunks.clear()
            self._loaded_source_signature = None
            self._set_source_summary(f"Load error: {exc}")
            self._refresh_chunk_table()
            carb.log_error(str(exc))
            self.set_status(f"Load error: {exc}")
            self._notify_trajectory_changed()
            if self._on_run_state_changed:
                self._on_run_state_changed()
            self._finish_load_task_if_current()
            return

        # Stale check again - another load may have started while in the executor.
        if _inputs_changed():
            self._finish_load_task_if_current()
            return

        self._trajectory = trajectory
        self._telemetry_chunks.clear()
        self._preview_chunk_index = 0
        self._loaded_source_signature = (
            path,
            source_type,
            mapping_path,
            chunk_manifest_path,
            command_alignment_seconds,
        )
        companion = phase_chunks_from_metadata(trajectory)
        if companion:
            self._telemetry_chunks.extend(companion)
        t0 = float(trajectory.times[0])
        t1 = float(trajectory.times[-1])
        if self._start_time_field:
            self._start_time_field.set_value(t0)
        if self._end_time_field:
            self._end_time_field.set_value(t1)
        self._set_source_summary(self._source_summary_text(trajectory))
        if companion:
            train_n = sum(1 for c in companion if c.role == TELEMETRY_CHUNK_ROLE_TRAIN)
            self._set_chunk_status(
                f"Imported {len(companion)} chunks from companion training logs "
                f"({train_n} train, {len(companion) - train_n} validation)."
            )
        else:
            self._set_chunk_status("No manual chunks. The analysis window will run as one training chunk.")
        self._refresh_chunk_table()
        n = int(trajectory.times.shape[0])
        self.set_status(f"Loaded {n} samples, {trajectory.num_joints} joints, t=[{t0:.3f}, {t1:.3f}] s")
        self._notify_trajectory_changed()
        if self._on_run_state_changed:
            self._on_run_state_changed()
        self._finish_load_task_if_current()

    def _notify_trajectory_changed(self) -> None:
        if self._on_trajectory_changed:
            self._on_trajectory_changed(self._trajectory)

    def _source_signature(self) -> tuple:
        return (
            self.get_source_path(),
            self.get_source_type(),
            self.get_mapping_path(),
            self.get_chunk_manifest_path(),
            self.get_command_alignment_seconds(),
        )

    def _is_loading(self) -> bool:
        return self._load_task is not None and not self._load_task.done()

    def _set_load_button_loading(self, loading: bool) -> None:
        if self._load_button is None:
            return
        self._load_button.text = "Loading..." if loading else "Load Trajectory"
        self._load_button.enabled = self._controls_enabled and not loading

    def _finish_load_task_if_current(self) -> None:
        if self._load_task is asyncio.current_task():
            self._load_task = None
        self._set_load_button_loading(False)

    def _set_preview_enabled(self, enabled: bool, *, clear: bool = False) -> None:
        self._preview_enabled = bool(enabled)
        if self._preview_button is not None:
            self._preview_button.text = "Hide" if self._preview_enabled else "Preview"
            self._preview_button.tooltip = (
                "Clear GT link/pose markers from the viewport."
                if self._preview_enabled
                else "Show GT link/pose markers for the selected chunk sample in the viewport."
            )
        if clear and self._on_preview_cleared is not None:
            try:
                self._on_preview_cleared()
            except Exception as exc:
                carb.log_warn(f"SysId: could not clear GT preview: {exc}")

    # chunk helpers
    def _on_chunk_role_changed(self, label: str) -> None:
        self._chunk_role_label = label
        if self._chunk_weight_widget is not None:
            is_training = label == "Train"
            self._chunk_weight_widget.enabled = is_training
            self._chunk_weight_widget.tooltip = (
                "Relative optimization weight applied to this training chunk."
                if is_training
                else "Validation chunks are held out and do not use an optimization weight."
            )

    def _on_chunk_excitation_changed(self, label: str) -> None:
        self._chunk_excitation_label = label
        if self._chunk_custom_excitation_model is not None and label != "custom":
            self._chunk_custom_excitation_model.set_value(label)
        if self._chunk_custom_excitation_field is not None and label != "custom":
            self._chunk_custom_excitation_field.set_value(label)

    def _chunk_name_value(self) -> str:
        if self._chunk_name_model is not None:
            return self._chunk_name_model.get_value_as_string().strip()
        if self._chunk_name_field is not None:
            return self._chunk_name_field.get_value().strip()
        return ""

    def _set_chunk_name_value(self, value: str) -> None:
        if self._chunk_name_model is not None:
            self._chunk_name_model.set_value(value)
        if self._chunk_name_field is not None:
            self._chunk_name_field.set_value(value)

    def _chunk_weight_value(self) -> float:
        if self._chunk_weight_model is not None:
            return float(self._chunk_weight_model.get_value_as_float())
        if self._chunk_weight_field is not None:
            return float(self._chunk_weight_field.get_value())
        return 1.0

    def _chunk_tag_value(self) -> str:
        if self._chunk_custom_excitation_model is not None:
            return self._chunk_custom_excitation_model.get_value_as_string().strip()
        if self._chunk_custom_excitation_field is not None:
            return self._chunk_custom_excitation_field.get_value().strip()
        return ""

    def _current_excitation(self) -> str:
        tag = self._chunk_tag_value()
        if self._chunk_excitation_label == "custom":
            return tag or "custom"
        if tag and tag != "custom" and tag != self._chunk_excitation_label:
            return tag
        return self._chunk_excitation_label or "custom"

    def _chunk_spec_from_editor(self, index: int | None = None) -> TelemetryChunkRunSpec:
        t0, t1 = self.get_time_window()
        role = _CHUNK_ROLE_TYPES.get(self._chunk_role_label, TELEMETRY_CHUNK_ROLE_TRAIN)
        name = self._chunk_name_value()
        weight = self._chunk_weight_value()
        default_idx = len(self._telemetry_chunks) + 1 if index is None else index + 1
        return TelemetryChunkRunSpec(
            name=name or f"{self._chunk_role_label} {default_idx}",
            role=role,
            excitation=self._current_excitation(),
            start=t0,
            end=t1,
            weight=weight,
        )

    def _on_add_chunk(self) -> None:
        self._stop_preview_playback()
        try:
            if self._trajectory is None:
                raise TrajectorySegmentError("Load telemetry before adding chunks.")
            spec = self._chunk_spec_from_editor()
            build_trajectory_chunks(self._trajectory, [*self._telemetry_chunks, spec], require_train=False)
            self._telemetry_chunks.append(spec)
            self._preview_chunk_index = len(self._telemetry_chunks) - 1
            self._set_chunk_name_value("")
            self._set_chunk_status(f"Added {spec.role} chunk '{spec.name}'.")
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            self._set_chunk_status(f"Chunk error: {exc}")
        self._refresh_chunk_table()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _preview_specs(self) -> list[TelemetryChunkRunSpec]:
        specs = self.get_chunk_specs()
        if not specs:
            raise TrajectorySegmentError("No chunks are available for preview.")
        return specs

    def _clamp_preview_chunk_index(self, specs: list[TelemetryChunkRunSpec]) -> int:
        if not specs:
            self._preview_chunk_index = 0
            return 0
        self._preview_chunk_index = max(0, min(self._preview_chunk_index, len(specs) - 1))
        return self._preview_chunk_index

    def _preview_chunk_sample(
        self,
        specs: list[TelemetryChunkRunSpec],
        index: int,
        sample_time: float,
        *,
        playback: bool = False,
    ) -> str:
        if self._trajectory is None:
            raise TrajectorySegmentError("Load telemetry before previewing GT poses.")
        if self._on_preview_requested is None:
            raise TrajectorySegmentError("Preview is not wired to a viewport handler.")
        if not specs:
            raise TrajectorySegmentError("No chunks are available for preview.")

        self._preview_chunk_index = max(0, min(index, len(specs) - 1))
        spec = specs[self._preview_chunk_index]
        message = self._on_preview_requested(self._trajectory, float(sample_time))
        verb = "Playing" if playback else "Chunk"
        self._set_chunk_status(
            f"{verb} {self._preview_chunk_index + 1}/{len(specs)} "
            f"'{spec.display_name(self._preview_chunk_index)}' at t={float(sample_time):.3f}s: {message}"
        )
        return message

    def _preview_chunk_at_index(self, index: int) -> None:
        self._stop_preview_playback()
        self._set_preview_enabled(True)
        try:
            specs = self._preview_specs()
            self._preview_chunk_index = max(0, min(index, len(specs) - 1))
            spec = specs[self._preview_chunk_index]
            self._preview_chunk_sample(specs, self._preview_chunk_index, float(spec.start))
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            self._set_chunk_status(f"Preview error: {exc}")

    def _on_preview_gt_clicked(self) -> None:
        if self._preview_enabled:
            self._stop_preview_playback()
            self._set_preview_enabled(False, clear=True)
            self._set_chunk_status("Preview disabled.")
            return
        self._preview_chunk_at_index(self._preview_chunk_index)

    def _on_step_preview_chunk(self, direction: int) -> None:
        try:
            specs = self._preview_specs()
            current = self._clamp_preview_chunk_index(specs)
            next_index = (current + int(direction)) % len(specs)
            self._preview_chunk_at_index(next_index)
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            self._set_chunk_status(f"Preview error: {exc}")

    def _is_preview_playing(self) -> bool:
        return self._preview_play_task is not None and not self._preview_play_task.done()

    def _stop_preview_playback(self, status: str | None = None) -> None:
        task = self._preview_play_task
        if task is not None and not task.done():
            task.cancel()
        self._preview_play_task = None
        if status:
            self._set_chunk_status(status)

    def _chunk_preview_playback_times(self, spec: TelemetryChunkRunSpec) -> list[float]:
        if self._trajectory is None:
            raise TrajectorySegmentError("Load telemetry before previewing GT poses.")
        sliced = self._trajectory.slice_time_window(float(spec.start), float(spec.end))
        raw_count = int(sliced.times.shape[0])
        if raw_count < 2:
            raise TrajectorySegmentError(f"Chunk '{spec.name}' must include at least 2 samples.")

        start = float(sliced.times[0])
        end = float(sliced.times[-1])
        duration = max(0.0, end - start)
        target_frames = int(duration / _CHUNK_PREVIEW_PLAYBACK_SPEED * _CHUNK_PREVIEW_PLAYBACK_FPS) + 1
        target_frames = max(target_frames, min(raw_count, _CHUNK_PREVIEW_PLAYBACK_MIN_FRAMES))
        frame_count = max(2, min(raw_count, _CHUNK_PREVIEW_PLAYBACK_MAX_FRAMES, target_frames))

        if frame_count >= raw_count:
            return [float(sample) for sample in sliced.times]

        step = duration / float(frame_count - 1)
        return [start + step * idx for idx in range(frame_count)]

    async def _wait_preview_playback_delay(self, delay_seconds: float) -> None:
        deadline = time.perf_counter() + max(0.0, float(delay_seconds))
        while time.perf_counter() < deadline:
            try:
                app = omni.kit.app.get_app()
                await app.next_update_async()
            except Exception:
                await asyncio.sleep(0)

    async def _play_preview_chunk_async(
        self,
        chunk_index: int,
        specs: list[TelemetryChunkRunSpec],
        sample_times: list[float],
    ) -> None:
        try:
            for sample_idx, sample_time in enumerate(sample_times):
                self._preview_chunk_sample(specs, chunk_index, sample_time, playback=True)
                if sample_idx + 1 >= len(sample_times):
                    break
                dt = max(0.0, float(sample_times[sample_idx + 1]) - float(sample_time))
                await self._wait_preview_playback_delay(dt / _CHUNK_PREVIEW_PLAYBACK_SPEED)

            spec = specs[max(0, min(chunk_index, len(specs) - 1))]
            self._set_chunk_status(
                f"Finished preview playback for chunk {chunk_index + 1}/{len(specs)} "
                f"'{spec.display_name(chunk_index)}'."
            )
        except asyncio.CancelledError:
            raise
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            self._set_chunk_status(f"Preview playback error: {exc}")
        finally:
            if self._preview_play_task is asyncio.current_task():
                self._preview_play_task = None

    def _on_play_preview_chunk_clicked(self) -> None:
        if self._is_preview_playing():
            self._stop_preview_playback("Preview playback stopped.")
            return
        try:
            specs = self._preview_specs()
            chunk_index = self._clamp_preview_chunk_index(specs)
            sample_times = self._chunk_preview_playback_times(specs[chunk_index])
            spec = specs[chunk_index]
            self._set_preview_enabled(True)
            self._set_chunk_status(
                f"Playing chunk {chunk_index + 1}/{len(specs)} '{spec.display_name(chunk_index)}' "
                f"at {_CHUNK_PREVIEW_PLAYBACK_SPEED:g}x."
            )
            self._preview_play_task = asyncio.ensure_future(
                self._play_preview_chunk_async(chunk_index, specs, sample_times)
            )
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError) as exc:
            self._set_chunk_status(f"Preview playback error: {exc}")

    def _on_clear_chunks(self) -> None:
        self._stop_preview_playback()
        self._set_preview_enabled(False, clear=True)
        self._telemetry_chunks.clear()
        self._preview_chunk_index = 0
        self._set_chunk_status("No manual chunks. The analysis window will run as one training chunk.")
        self._refresh_chunk_table()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _on_remove_chunk(self, index: int) -> None:
        self._stop_preview_playback()
        if 0 <= index < len(self._telemetry_chunks):
            removed = self._telemetry_chunks.pop(index)
            self._preview_chunk_index = min(self._preview_chunk_index, max(0, len(self._telemetry_chunks) - 1))
            self._set_chunk_status(f"Removed chunk '{removed.name}'.")
        self._refresh_chunk_table()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _on_chunk_cell_changed(self, index: int, field: str, value: Any) -> None:
        self._stop_preview_playback()
        if not (0 <= index < len(self._telemetry_chunks)):
            return
        try:
            if field in ("start", "end", "weight"):
                value = float(value)
            elif field == "role":
                value = _CHUNK_ROLE_TYPES.get(str(value), str(value).lower())
            else:
                value = str(value)
            self._telemetry_chunks[index] = replace(self._telemetry_chunks[index], **{field: value})
        except Exception as exc:
            self._set_chunk_status(f"Chunk edit error: {exc}")
        if field == "role":
            self._refresh_chunk_table()
        else:
            self._refresh_chunk_timeline()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _set_chunk_status(self, text: str) -> None:
        if self._chunk_status_label is not None:
            self._chunk_status_label.set_text(text)

    def _set_source_summary(self, text: str) -> None:
        if self._source_summary_label is not None:
            self._source_summary_label.set_text(text)

    def _chunk_table_height(self) -> int:
        if not self._telemetry_chunks:
            return _CHUNK_TABLE_EMPTY_HEIGHT
        visible_rows = min(len(self._telemetry_chunks), _CHUNK_TABLE_MAX_VISIBLE_ROWS)
        return _CHUNK_TABLE_HEADER_HEIGHT + visible_rows * _CHUNK_TABLE_ROW_HEIGHT + _CHUNK_TABLE_PADDING

    def _refresh_chunk_timeline(self) -> None:
        if self._chunk_timeline_frame is not None:
            self._chunk_timeline_frame.rebuild()

    def _refresh_chunk_table(self) -> None:
        self._refresh_chunk_timeline()
        self._refresh_auto_split_enabled()
        if self._chunk_table_frame is not None:
            self._chunk_table_frame.height = ui.Length(self._chunk_table_height())
            self._chunk_table_frame.rebuild()

    def _on_auto_split_changed(self) -> None:
        self._refresh_chunk_timeline()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _on_analysis_window_changed(self, _value: float) -> None:
        self._refresh_chunk_timeline()
        if self._on_run_state_changed:
            self._on_run_state_changed()

    def _refresh_auto_split_enabled(self) -> None:
        """Manual chunks win: grey out automatic holdout while any exist."""
        enabled = self._controls_enabled and not self._telemetry_chunks
        for widget in self._auto_split_widgets:
            widget.enabled = enabled
            widget.tooltip = (
                "Manual chunks take precedence; choose Remove All to use automatic holdout." if not enabled else ""
            )
        disabled_by_chunks = self._controls_enabled and bool(self._telemetry_chunks)
        if self._auto_split_hint_label is not None:
            self._auto_split_hint_label.visible = disabled_by_chunks
            self._auto_split_hint_label.height = ui.Length(0) if not disabled_by_chunks else ui.Length(18)
            self._auto_split_hint_label.text = (
                f"Automatic holdout settings are disabled while {len(self._telemetry_chunks)} "
                "manual chunk(s) exist. Remove the chunks or choose Remove All to re-enable automatic holdout."
            )

    def _chunk_timeline_bounds(self) -> tuple[float, float] | None:
        if self._trajectory is None or self._trajectory.times.shape[0] < 2:
            return None
        start = float(self._trajectory.times[0])
        end = float(self._trajectory.times[-1])
        if end <= start:
            return None
        return start, end

    def _chunk_timeline_specs(self) -> tuple[list[TelemetryChunkRunSpec], bool]:
        if self._trajectory is None:
            return [], False
        if self._telemetry_chunks:
            return list(self._telemetry_chunks), True
        try:
            specs = self.get_chunk_specs()
        except (TrajectoryCsvError, TrajectorySegmentError, ValueError):
            return [], False
        # Auto-materialized chunks carry real roles; color them like explicit ones.
        return specs, self.get_auto_split()[0]

    def _chunk_timeline_color(self, spec: TelemetryChunkRunSpec, explicit: bool) -> int:
        if not explicit:
            return _CHUNK_TIMELINE_DEFAULT_COLOR
        if spec.role == TELEMETRY_CHUNK_ROLE_VALIDATION:
            return _CHUNK_TIMELINE_VALIDATION_COLOR
        return _CHUNK_TIMELINE_TRAIN_COLOR

    def _chunk_timeline_segments(self) -> list[_ChunkTimelineSegment]:
        bounds = self._chunk_timeline_bounds()
        if bounds is None:
            return []
        data_start, data_end = bounds
        total = data_end - data_start
        specs, explicit = self._chunk_timeline_specs()
        cursor = data_start
        segments: list[_ChunkTimelineSegment] = []

        def append_gap(start: float, end: float) -> None:
            """Handle append gap.

            Args:
                start: Start value.
                end: End value.
            """
            if end <= start:
                return
            segments.append(
                _ChunkTimelineSegment(
                    kind="gap",
                    fraction=(end - start) / total,
                    color=_CHUNK_TIMELINE_GAP_COLOR,
                    tooltip=f"Unused: {start:.3f}s to {end:.3f}s",
                )
            )

        for spec_index, spec in sorted(enumerate(specs), key=lambda item: (item[1].start, item[1].end)):
            raw_start = float(spec.start)
            raw_end = float(spec.end)
            visible_start = min(max(raw_start, data_start), data_end)
            visible_end = min(max(raw_end, data_start), data_end)
            if visible_start > cursor:
                append_gap(cursor, visible_start)
            visible_start = max(visible_start, cursor)
            if visible_end > visible_start:
                label = spec.display_name(spec_index)
                clipped = raw_start < data_start or raw_end > data_end
                note = " clipped to loaded telemetry" if clipped else ""
                segments.append(
                    _ChunkTimelineSegment(
                        kind="chunk",
                        fraction=(visible_end - visible_start) / total,
                        color=self._chunk_timeline_color(spec, explicit),
                        tooltip=(
                            f"{label}: {spec.role}, {spec.excitation}, " f"{raw_start:.3f}s to {raw_end:.3f}s{note}"
                        ),
                    )
                )
                cursor = max(cursor, visible_end)

        append_gap(cursor, data_end)
        if not segments:
            append_gap(data_start, data_end)
        return segments

    def _build_chunk_timeline(self) -> None:
        bounds = self._chunk_timeline_bounds()
        if bounds is None:
            ui.Label("Timeline appears after telemetry loads.", height=20)
            return

        data_start, data_end = bounds
        with ui.VStack(spacing=2, height=_CHUNK_TIMELINE_HEIGHT):
            with ui.HStack(spacing=0, height=_CHUNK_TIMELINE_TRACK_HEIGHT):
                for segment in self._chunk_timeline_segments():
                    ui.Rectangle(
                        width=ui.Fraction(max(segment.fraction, _CHUNK_TIMELINE_MIN_FRACTION)),
                        height=_CHUNK_TIMELINE_TRACK_HEIGHT,
                        tooltip=segment.tooltip,
                        style={"background_color": segment.color, "border_radius": 2},
                    )
            with ui.HStack(height=16):
                ui.Label(f"{data_start:.3f}s", width=70)
                ui.Spacer()
                ui.Label(f"{data_end:.3f}s", width=70, alignment=ui.Alignment.RIGHT_CENTER)
            with ui.HStack(spacing=6, height=16):
                for label, color in (
                    ("Train", _CHUNK_TIMELINE_TRAIN_COLOR),
                    ("Validation", _CHUNK_TIMELINE_VALIDATION_COLOR),
                    ("Unused", _CHUNK_TIMELINE_GAP_COLOR),
                ):
                    ui.Rectangle(width=10, height=10, style={"background_color": color, "border_radius": 2})
                    ui.Label(label, width=64, style={"font_size": 11, "color": 0xFFAAAAAA})
                ui.Spacer()

    def _build_chunk_table(self) -> None:
        with ui.VStack(spacing=4):
            ui.Label(
                f"Configured manual chunks ({len(self._telemetry_chunks)})",
                height=20,
                style={"font_size": 12, "color": 0xFFB8B8B8},
            )
            with ui.HStack(spacing=4, height=22):
                for label, width in (
                    ("Name", 112),
                    ("Role", 78),
                    ("Tag", 88),
                    ("Start (s)", 68),
                    ("End (s)", 68),
                    ("Samples", 58),
                    ("Duration (s)", 72),
                    ("Weight", 54),
                ):
                    ui.Label(label, width=width)
                ui.Spacer(width=58)
            if not self._telemetry_chunks:
                ui.Label("No manual chunks. The analysis window is used as one training chunk.", height=20)
                return
            for idx, spec in enumerate(self._telemetry_chunks):
                samples, duration, warning = self._chunk_row_stats(spec)
                self._build_chunk_row(idx, spec, samples, duration, warning)

    def _chunk_row_stats(self, spec: TelemetryChunkRunSpec) -> tuple[str, str, str]:
        if self._trajectory is None:
            return "-", "-", ""
        try:
            sliced = self._trajectory.slice_time_window(spec.start, spec.end)
            n = int(sliced.times.shape[0])
            dur = f"{float(sliced.times[-1] - sliced.times[0]):.3f}" if n >= 2 else "-"
            warning = "" if n >= 2 else "needs 2+ samples"
            return str(n), dur, warning
        except Exception as exc:
            return "-", "-", str(exc)

    def _build_chunk_row(
        self, index: int, spec: TelemetryChunkRunSpec, samples: str, duration: str, warning: str
    ) -> None:
        with ui.HStack(spacing=4, height=24):
            name_model = ui.SimpleStringModel(spec.name)
            name_model.add_value_changed_fn(
                lambda m, i=index: self._on_chunk_cell_changed(i, "name", m.get_value_as_string())
            )
            ui.StringField(model=name_model, width=112, height=22, tooltip="Chunk name")

            role_index = 1 if spec.role == TELEMETRY_CHUNK_ROLE_VALIDATION else 0
            role_box = ui.ComboBox(role_index, *_CHUNK_ROLE_LABELS, width=78, height=22, tooltip="Chunk role")
            role_box.model.add_item_changed_fn(
                lambda m, _v, i=index: self._on_chunk_cell_changed(
                    i, "role", _CHUNK_ROLE_LABELS[m.get_item_value_model().as_int]
                )
            )
            excitation_model = ui.SimpleStringModel(spec.excitation)
            excitation_model.add_value_changed_fn(
                lambda m, i=index: self._on_chunk_cell_changed(
                    i, "excitation", m.get_value_as_string().strip() or "custom"
                )
            )
            ui.StringField(model=excitation_model, width=88, height=22, tooltip="Excitation tag")

            for fname, fval in (("start", spec.start), ("end", spec.end)):
                field = ui.FloatField(width=68, height=22, tooltip=f"Exact {fname} time in seconds")
                field.model.set_value(float(fval))
                field.model.add_value_changed_fn(
                    lambda m, i=index, f=fname: self._on_chunk_cell_changed(i, f, m.get_value_as_float())
                )

            sample_text = samples if not warning else f"{samples} !"
            sample_tooltip = warning or f"{samples} samples"
            ui.Label(sample_text, width=58, tooltip=sample_tooltip)
            ui.Label(duration, width=72, tooltip="Chunk duration in seconds")

            is_training = spec.role == TELEMETRY_CHUNK_ROLE_TRAIN
            weight_field = ui.FloatField(
                width=54,
                height=22,
                enabled=is_training,
                tooltip=(
                    "Relative optimization weight"
                    if is_training
                    else "Validation chunks are held out and do not use an optimization weight"
                ),
            )
            weight_field.model.set_min(1e-6)
            weight_field.model.set_max(inf)
            weight_field.model.set_value(float(spec.weight))
            weight_field.model.add_value_changed_fn(
                lambda m, i=index: self._on_chunk_cell_changed(i, "weight", m.get_value_as_float())
            )
            ui.Button(
                "Remove",
                width=58,
                height=22,
                tooltip=f"Remove chunk '{spec.display_name(index)}'",
                clicked_fn=lambda i=index: self._on_remove_chunk(i),
            )

    def _source_summary_text(self, trajectory: TrajectoryDataset) -> str:
        phase_count = 0
        if trajectory.metadata is not None:
            raw = trajectory.metadata.extra.get("phase_chunks", [])
            if isinstance(raw, list):
                phase_count = len(raw)
        phase_text = f"; companion chunks={phase_count}" if phase_count else ""
        if trajectory.torques is not None:
            try:
                from ..analytical_presolve import effort_semantics

                phase_text += f"; torque semantics={effort_semantics(trajectory)}"
            except Exception:
                pass
        try:
            quality = build_telemetry_quality_report(trajectory, self._telemetry_chunks or None)
            return quality.compact_summary() + phase_text
        except Exception:
            channels = ["position", "velocity", "command"]
            if trajectory.torques is not None:
                channels.append("torque")
            if trajectory.end_effector_poses is not None:
                channels.append("ee_pose")
            if trajectory.contact_forces is not None:
                channels.append("contact")
            return (
                f"{trajectory.times.shape[0]} samples, {trajectory.num_joints} joints, "
                f"duration={trajectory.duration:.3f}s; channels={', '.join(channels)}{phase_text}"
            )
