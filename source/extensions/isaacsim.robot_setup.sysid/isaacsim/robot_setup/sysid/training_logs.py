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

"""Discovery and parsing for companion SysID training detail logs."""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import carb

from .trajectory_csv import TrajectoryDataset
from .trajectory_segments import TelemetryChunkRunSpec, TrajectorySegmentError

_PHASE_EVENTS_FILE = "phase_events.jsonl"
_MANIFEST_FILE = "collection_manifest.json"
_RUN_METADATA_FILE = "run_metadata.json"
_TIME_KEYS = ("ros_time_sec", "wall_time", "monotonic_time")
_MAX_COMPANION_JSON_BYTES = 4 * 1024 * 1024
_MAX_PHASE_EVENTS_BYTES = 16 * 1024 * 1024
_MIN_PHASE_COVERAGE_RATIO = 0.5
_MIN_PHASE_CONTAINMENT_RATIO = 0.5


@dataclass
class CompanionTrainingDetails:
    """Parsed companion metadata for one telemetry recording."""

    root_path: str
    phase_events_path: str = ""
    manifest_path: str = ""
    run_metadata_path: str = ""
    topic_mapping_path: str = ""
    time_key: str = ""
    chunks: list[TelemetryChunkRunSpec] | None = None

    def to_extra_dict(self) -> dict[str, Any]:
        """Return metadata fields suitable for TrajectoryDatasetMetadata.extra.

        Returns:
            JSON-ready provenance fields for the discovered companion files and chunks.
        """
        return {
            "training_detail_root": self.root_path,
            "phase_events_path": self.phase_events_path,
            "collection_manifest_path": self.manifest_path,
            "run_metadata_path": self.run_metadata_path,
            "companion_topic_mapping_path": self.topic_mapping_path,
            "phase_events_time_key": self.time_key,
            "phase_chunks": [chunk.to_dict() for chunk in self.chunks or []],
        }


@dataclass
class _PhaseInterval:
    phase: str
    split: str
    start: float
    end: float
    time_key: str


def find_companion_root(source_path: str | Path) -> Path:
    """Return the likely run-folder root for a source file or folder.

    Args:
        source_path: Telemetry file, bag directory, or run directory.

    Returns:
        Directory expected to contain companion run metadata.
    """
    path = Path(source_path).expanduser()
    if path.is_file():
        parent = path.parent
        if parent.name.lower() == "bag":
            return parent.parent
        return parent
    if path.is_dir():
        if (path / _PHASE_EVENTS_FILE).is_file() or (path / _MANIFEST_FILE).is_file():
            return path
        if path.name.lower() == "bag":
            return path.parent
    return path


def find_companion_topic_mapping_path(source_path: str | Path) -> str:
    """Find a nearby topic-map file for ROS 2 bag/MCAP ingestion.

    Args:
        source_path: Telemetry path from which to locate the run directory.

    Returns:
        Nearby topic-map path, or an empty string when none exists.
    """
    root = find_companion_root(source_path)
    candidates = [
        root / "topic_map.yaml",
        root / "topic_map.yml",
        root / "topic_map.json",
        root / "franka_sysid_topic_map.yaml",
    ]
    candidates.extend(sorted(root.glob("*topic*map*.yaml")))
    candidates.extend(sorted(root.glob("*topic*map*.yml")))
    candidates.extend(sorted(root.glob("*topic*map*.json")))
    for candidate in candidates:
        if _is_safe_discovered_file(candidate, root):
            return str(candidate)
    return ""


def find_companion_mcap_path(source_path: str | Path) -> str:
    """Resolve a run folder, bag folder, or MCAP file to an MCAP file path.

    Args:
        source_path: MCAP file, bag directory, or run directory.

    Returns:
        Resolved MCAP path, or the original path when no recording is found.
    """
    path = Path(source_path).expanduser()
    if path.is_file() and path.suffix.lower() == ".mcap":
        return str(path)
    if not path.is_dir():
        return str(path)
    candidates: list[Path] = []
    candidates.extend(sorted(path.glob("*.mcap")))
    bag_dir = path / "bag"
    if bag_dir.is_dir():
        candidates.extend(sorted(bag_dir.glob("*.mcap")))
    # Depth-limited search (max 3 levels) avoids scanning large data trees.
    for pattern in ("*/*.mcap", "*/*/*.mcap"):
        candidates.extend(sorted(path.glob(pattern)))

    unique: dict[Path, Path] = {}
    for candidate in candidates:
        if _is_safe_discovered_file(candidate, path):
            unique.setdefault(candidate.resolve(), candidate)
    if len(unique) > 1:
        choices = ", ".join(str(candidate) for candidate in sorted(unique.values()))
        raise ValueError(
            "Multiple companion MCAP files were found; pass an explicit MCAP file path instead: " + choices
        )
    if unique:
        return str(next(iter(unique.values())))
    return str(path)


def load_companion_training_details(
    source_path: str | Path,
    trajectory: TrajectoryDataset,
) -> CompanionTrainingDetails | None:
    """Parse phase detail logs near telemetry and map them to trajectory chunks.

    Args:
        source_path: Telemetry path used to locate companion detail files.
        trajectory: Loaded trajectory used to map phase times into chunks.

    Returns:
        Parsed companion details, or ``None`` when no detail files are present.
    """
    root = find_companion_root(source_path)
    phase_events_path = root / _PHASE_EVENTS_FILE
    manifest_path = root / _MANIFEST_FILE
    run_metadata_path = root / _RUN_METADATA_FILE
    if not phase_events_path.is_file() and not manifest_path.is_file() and not run_metadata_path.is_file():
        return None

    manifest = _load_json_object(manifest_path)
    events = _load_jsonl_objects(phase_events_path)
    chunks, time_key = _chunks_from_events(events, manifest, trajectory)

    return CompanionTrainingDetails(
        root_path=str(root),
        phase_events_path=str(phase_events_path) if phase_events_path.is_file() else "",
        manifest_path=str(manifest_path) if manifest_path.is_file() else "",
        run_metadata_path=str(run_metadata_path) if run_metadata_path.is_file() else "",
        time_key=time_key,
        chunks=chunks,
    )


def attach_companion_training_details(
    trajectory: TrajectoryDataset,
    source_path: str | Path,
) -> TrajectoryDataset:
    """Attach parsed companion training details to trajectory metadata when present.

    Args:
        trajectory: Loaded trajectory whose metadata will be augmented.
        source_path: Telemetry path used to locate companion detail files.

    Returns:
        The input trajectory with companion provenance attached when available.
    """
    details = load_companion_training_details(source_path, trajectory)
    if details is None or trajectory.metadata is None:
        return trajectory
    trajectory.metadata.extra.update(details.to_extra_dict())
    return trajectory


def phase_chunks_from_metadata(trajectory: TrajectoryDataset | None) -> list[TelemetryChunkRunSpec]:
    """Read companion phase chunks from trajectory metadata.

    Args:
        trajectory: Loaded trajectory that may contain serialized phase chunks.

    Returns:
        Valid phase-chunk specifications reconstructed from trajectory metadata.
    """
    if trajectory is None or trajectory.metadata is None:
        return []
    raw = trajectory.metadata.extra.get("phase_chunks", [])
    if not isinstance(raw, list):
        return []
    chunks: list[TelemetryChunkRunSpec] = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            continue
        try:
            chunks.append(TelemetryChunkRunSpec.from_dict(item))
        except TrajectorySegmentError as exc:
            carb.log_warn(f"SysId: ignoring invalid companion phase chunk at index {index}: {exc}")
    return chunks


def _is_safe_discovered_file(candidate: Path, root: Path) -> bool:
    """Return whether a discovered file is safe to consume.

    Args:
        candidate: Discovered companion-file path.
        root: Directory that must contain the resolved candidate.

    Returns:
        Whether the path is a regular, non-symlinked file contained by the root.
    """
    try:
        resolved_root = root.resolve(strict=True)
        resolved_candidate = candidate.resolve(strict=True)
    except OSError:
        return False
    return candidate.is_file() and not candidate.is_symlink() and resolved_candidate.is_relative_to(resolved_root)


def _read_companion_text(path: Path, *, max_bytes: int, label: str) -> str | None:
    """Read a bounded, non-symlinked companion text file.

    Args:
        path: Companion text file to read.
        max_bytes: Maximum accepted file size.
        label: File-kind label used in diagnostics.

    Returns:
        Decoded text, or ``None`` when the file is absent or unsafe to read.
    """
    if not path.is_file():
        return None
    if path.is_symlink():
        carb.log_warn(f"SysId: refusing symlinked optional companion {label} file '{path}'.")
        return None
    try:
        size = path.stat().st_size
    except OSError as exc:
        carb.log_warn(f"SysId: could not inspect optional companion {label} file '{path}': {exc}")
        return None
    if size > max_bytes:
        carb.log_warn(f"SysId: optional companion {label} file '{path}' exceeds the {max_bytes}-byte safety limit.")
        return None
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        carb.log_warn(f"SysId: could not read optional companion {label} file '{path}': {exc}")
        return None


def _load_json_object(path: Path) -> dict[str, Any]:
    text = _read_companion_text(path, max_bytes=_MAX_COMPANION_JSON_BYTES, label="JSON")
    if text is None:
        return {}
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        carb.log_warn(f"SysId: could not parse optional companion JSON file '{path}': {exc}")
        return {}
    if not isinstance(payload, dict):
        carb.log_warn(f"SysId: optional companion JSON file '{path}' must contain a JSON object.")
        return {}
    return payload


def _load_jsonl_objects(path: Path) -> list[dict[str, Any]]:
    contents = _read_companion_text(path, max_bytes=_MAX_PHASE_EVENTS_BYTES, label="JSONL")
    if contents is None:
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(contents.splitlines(), start=1):
        text = line.strip()
        if not text:
            continue
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as exc:
            carb.log_warn(f"SysId: ignoring malformed optional companion JSONL row {line_number} in '{path}': {exc}")
            continue
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


def _chunks_from_events(
    events: list[dict[str, Any]],
    manifest: dict[str, Any],
    trajectory: TrajectoryDataset,
) -> tuple[list[TelemetryChunkRunSpec], str]:
    """Map companion phase events onto the trajectory window when they share its timebase.

    Numeric overlap alone does not prove that an event log belongs to the loaded recording:
    a stale wall-clock or monotonic-clock log from a different run can brush the trajectory
    window and produce bogus train/validation chunks. A candidate time key is therefore
    accepted only when its intervals cover at least ``_MIN_PHASE_COVERAGE_RATIO`` of the
    trajectory duration and at least ``_MIN_PHASE_CONTAINMENT_RATIO`` of the logged phase
    duration falls inside that window. Keys with weaker evidence are rejected with a warning.

    Args:
        events: Parsed phase-event rows from the companion log.
        manifest: Parsed collection manifest used for phase split and purpose lookups.
        trajectory: Loaded trajectory whose time window the events must match.

    Returns:
        Accepted phase chunks paired with the event time key they came from, or an empty
        list and an empty key when no time key provides sufficient evidence.
    """
    if not events:
        return [], ""
    manifest_by_phase = _manifest_phase_lookup(manifest)
    data_start = float(trajectory.times[0])
    data_end = float(trajectory.times[-1])
    selected_key = ""
    selected: list[_PhaseInterval] = []
    rejected: list[str] = []
    for key in _TIME_KEYS:
        intervals = _intervals_from_events(events, key, manifest_by_phase)
        overlapping = _overlapping_intervals(intervals, data_start, data_end)
        if not overlapping:
            continue
        coverage, containment = _phase_timebase_evidence(intervals, overlapping, data_start, data_end)
        if coverage < _MIN_PHASE_COVERAGE_RATIO or containment < _MIN_PHASE_CONTAINMENT_RATIO:
            rejected.append(f"{key} (coverage {coverage:.3f}, containment {containment:.3f})")
            continue
        if len(overlapping) > len(selected):
            selected = overlapping
            selected_key = key
    if selected:
        return _intervals_to_chunks(selected, manifest_by_phase, data_start=data_start, data_end=data_end), selected_key
    if rejected:
        carb.log_warn(
            "SysId: ignoring companion phase events because no time key matches the telemetry window "
            f"[{data_start:g}, {data_end:g}]; required coverage >= {_MIN_PHASE_COVERAGE_RATIO:.2f} and "
            f"containment >= {_MIN_PHASE_CONTAINMENT_RATIO:.2f}, got {', '.join(rejected)}."
        )
    return [], ""


def _phase_timebase_evidence(
    intervals: list[_PhaseInterval],
    overlapping: list[_PhaseInterval],
    data_start: float,
    data_end: float,
) -> tuple[float, float]:
    """Measure how strongly a candidate time key matches the trajectory window.

    Args:
        intervals: Phase intervals in the candidate timebase, before clipping.
        overlapping: The same intervals clipped to the trajectory window.
        data_start: First trajectory timestamp.
        data_end: Last trajectory timestamp.

    Returns:
        Fraction of the trajectory duration covered by the clipped intervals, paired with
        the fraction of the logged phase duration that falls inside the trajectory window.
    """
    duration = max(0.0, float(data_end) - float(data_start))
    if duration <= 0.0:
        return 0.0, 0.0
    covered = 0.0
    union_end = data_start
    for interval in sorted(overlapping, key=lambda item: item.start):
        start = max(interval.start, union_end)
        if interval.end > start:
            covered += interval.end - start
            union_end = interval.end
    logged = sum(max(0.0, interval.end - interval.start) for interval in intervals)
    clipped = sum(max(0.0, interval.end - interval.start) for interval in overlapping)
    containment = clipped / logged if logged > 0.0 else 0.0
    return covered / duration, containment


def _manifest_phase_lookup(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    phases = manifest.get("phases", [])
    if not isinstance(phases, list):
        return {}
    lookup = {}
    for item in phases:
        if isinstance(item, dict) and item.get("name"):
            lookup[str(item["name"])] = item
    return lookup


def _intervals_from_events(
    events: Iterable[dict[str, Any]],
    time_key: str,
    manifest_by_phase: dict[str, dict[str, Any]],
) -> list[_PhaseInterval]:
    active: dict[str, dict[str, Any]] = {}
    intervals: list[_PhaseInterval] = []
    for event in events:
        phase = str(event.get("phase", "")).strip()
        state = str(event.get("event", "")).strip().lower()
        if not phase or state not in ("start", "end"):
            continue
        if time_key not in event:
            continue
        try:
            timestamp = float(event[time_key])
        except (TypeError, ValueError):
            continue
        if state == "start":
            active[phase] = dict(event)
            active[phase][time_key] = timestamp
            continue
        start_event = active.pop(phase, None)
        if start_event is None:
            continue
        start = float(start_event[time_key])
        if timestamp <= start:
            continue
        split = str(
            event.get("split") or start_event.get("split") or manifest_by_phase.get(phase, {}).get("split") or "train"
        ).lower()
        intervals.append(_PhaseInterval(phase=phase, split=split, start=start, end=timestamp, time_key=time_key))
    return intervals


def _overlapping_intervals(
    intervals: list[_PhaseInterval],
    data_start: float,
    data_end: float,
) -> list[_PhaseInterval]:
    duration = max(0.0, float(data_end) - float(data_start))
    eps = max(1e-9, 1e-9 * max(1.0, duration))
    selected = []
    for interval in intervals:
        start = max(interval.start, data_start)
        end = min(interval.end, data_end)
        if end > start + eps:
            selected.append(
                _PhaseInterval(
                    phase=interval.phase,
                    split=interval.split,
                    start=start,
                    end=end,
                    time_key=interval.time_key,
                )
            )
    return selected


def _intervals_to_chunks(
    intervals: list[_PhaseInterval],
    manifest_by_phase: dict[str, dict[str, Any]],
    *,
    data_start: float,
    data_end: float,
) -> list[TelemetryChunkRunSpec]:
    counts: dict[str, int] = {}
    chunks: list[TelemetryChunkRunSpec] = []
    for interval in intervals:
        counts[interval.phase] = counts.get(interval.phase, 0) + 1
        suffix = "" if counts[interval.phase] == 1 else f"_{counts[interval.phase]}"
        role = "validation" if str(interval.split).lower().startswith("val") else "train"
        manifest = manifest_by_phase.get(interval.phase, {})
        chunks.append(
            TelemetryChunkRunSpec(
                name=f"{interval.phase}{suffix}",
                role=role,
                excitation=_infer_excitation(interval.phase, str(manifest.get("purpose", ""))),
                start=max(data_start, float(interval.start)),
                end=min(data_end, float(interval.end)),
                weight=1.0,
            )
        )
    return chunks


def _infer_excitation(phase: str, purpose: str) -> str:
    text = f"{phase} {purpose}".lower()
    if "prbs" in text:
        return "PRBS"
    if "chirp" in text:
        return "chirp"
    if "step" in text:
        return "step"
    if "static" in text or "hold" in text:
        return "gravity"
    if "sine" in text or "sweep" in text:
        return "sine"
    if "gravity" in text:
        return "gravity"
    if "contact" in text:
        return "contact"
    return "custom"
