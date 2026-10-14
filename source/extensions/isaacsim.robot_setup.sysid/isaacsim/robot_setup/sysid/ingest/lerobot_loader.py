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

"""Local LeRobot-format trajectory ingestion backed by PyArrow."""

from __future__ import annotations

import importlib
import importlib.util
import json
import os
import sys
from pathlib import Path
from typing import Any

import carb
import numpy as np

from ..trajectory_csv import TrajectoryDataset
from .config_types import TrajectoryIngestError, TrajectoryLoadConfig
from .trajectory_builder import build_trajectory_dataset

_POSITION_COLUMNS = ("observation.state", "observation.qpos", "observation/position")
_VELOCITY_COLUMNS = ("observation.velocity", "observation.dq", "observation/velocity")
_COMMAND_COLUMNS = ("action", "action.position", "action/position")
_TORQUE_COLUMNS = ("observation.torque", "observation.effort", "observation/torque")
_END_EFFECTOR_COLUMNS = ("observation.ee_pose", "observation.end_effector_pose")
_CONTACT_COLUMNS = ("observation.contact_force", "observation.contact_forces")
_ORDER_COLUMNS = ("frame_index", "index")
_TORQUE_SEMANTICS_VALUES = ("link_side", "external")
_PYARROW_DLL_DIRECTORIES: list[Any] = []


def _resolve_torque_semantics(info: dict[str, Any], has_torques: bool) -> str | None:
    """Read the dataset's declared torque semantics from ``meta/info.json``.

    LeRobot has no mapping file, so a dataset that ships an effort column has
    no other way to state whether it is link-side joint torque or an external
    wrench. Without the declaration every torque residual is rejected
    downstream, which looks like the loader silently dropped the channel.

    Args:
        info: Parsed ``meta/info.json`` contents.
        has_torques: Whether the episode actually carries a torque column.

    Returns:
        The declared semantics, or ``None`` when the dataset declares none.
    """
    value = info.get("torque_semantics")
    if isinstance(value, str) and value in _TORQUE_SEMANTICS_VALUES:
        return value
    if value is not None:
        raise TrajectoryIngestError(
            f"LeRobot meta/info.json 'torque_semantics' must be one of {', '.join(_TORQUE_SEMANTICS_VALUES)}; "
            f"got {value!r}."
        )
    if has_torques:
        carb.log_warn(
            "SysId: LeRobot dataset provides a torque column but meta/info.json declares no 'torque_semantics'; "
            'torque residuals will be rejected. Add "torque_semantics": "link_side" (or "external") to the '
            "dataset metadata."
        )
    return None


def _prepare_pyarrow_dll_search_path() -> None:
    if sys.platform != "win32" or _PYARROW_DLL_DIRECTORIES:
        return
    spec = importlib.util.find_spec("pyarrow")
    if spec is None or spec.origin is None:
        return
    package_dir = Path(spec.origin).resolve().parent
    for dll_dir in (package_dir.parent / "pyarrow.libs", package_dir):
        if dll_dir.is_dir():
            _PYARROW_DLL_DIRECTORIES.append(os.add_dll_directory(str(dll_dir)))


def _load_pyarrow_modules() -> tuple[Any, Any]:
    _prepare_pyarrow_dll_search_path()
    try:
        dataset = importlib.import_module("pyarrow.dataset")
        types = importlib.import_module("pyarrow.types")
    except ModuleNotFoundError as exc:
        if exc.name == "pyarrow":
            raise TrajectoryIngestError(
                "Local LeRobot-format ingestion requires PyArrow 24.0.0 in the active environment. "
                "PyArrow is not bundled until its third-party approval is complete."
            ) from exc
        raise
    return dataset, types


def _load_info(root: Path) -> dict[str, Any]:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise TrajectoryIngestError(f"LeRobot info.json not found at {info_path}")
    try:
        payload = json.loads(info_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise TrajectoryIngestError(f"Could not read LeRobot metadata at {info_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise TrajectoryIngestError(f"LeRobot metadata must contain a JSON object: {info_path}")
    return payload


def _find_column(names: set[str], candidates: tuple[str, ...]) -> str | None:
    return next((name for name in candidates if name in names), None)


def _column_to_matrix(table: Any, name: str, arrow_types: Any) -> np.ndarray:
    column = table.column(name).combine_chunks()
    if column.null_count:
        raise TrajectoryIngestError(f"LeRobot column '{name}' contains null values.")

    column_type = column.type
    if arrow_types.is_fixed_size_list(column_type):
        values = column.values
        if values.null_count:
            raise TrajectoryIngestError(f"LeRobot column '{name}' contains null vector elements.")
        try:
            matrix = np.asarray(values.to_numpy(zero_copy_only=False), dtype=np.float64).reshape(
                len(column), column_type.list_size
            )
        except (TypeError, ValueError) as exc:
            raise TrajectoryIngestError(f"LeRobot column '{name}' is not a numeric vector column.") from exc
    elif arrow_types.is_list(column_type) or arrow_types.is_large_list(column_type):
        try:
            matrix = np.asarray(column.to_pylist(), dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise TrajectoryIngestError(
                f"LeRobot column '{name}' contains non-numeric or inconsistent-length vectors."
            ) from exc
        if matrix.ndim != 2:
            raise TrajectoryIngestError(f"LeRobot column '{name}' must contain one numeric vector per frame.")
    elif arrow_types.is_integer(column_type) or arrow_types.is_floating(column_type):
        matrix = np.asarray(column.to_numpy(zero_copy_only=False), dtype=np.float64).reshape(-1, 1)
    else:
        raise TrajectoryIngestError(f"LeRobot column '{name}' has unsupported type {column_type}.")

    if matrix.shape[1] < 1 or not np.all(np.isfinite(matrix)):
        raise TrajectoryIngestError(f"LeRobot column '{name}' must contain finite numeric vectors.")
    return matrix


def _column_to_vector(table: Any, name: str, arrow_types: Any) -> np.ndarray:
    matrix = _column_to_matrix(table, name, arrow_types)
    if matrix.shape[1] != 1:
        raise TrajectoryIngestError(f"LeRobot column '{name}' must contain scalar values.")
    return matrix[:, 0]


def _optional_matrix(
    table: Any,
    names: set[str],
    candidates: tuple[str, ...],
    arrow_types: Any,
) -> tuple[np.ndarray | None, str | None]:
    name = _find_column(names, candidates)
    return (_column_to_matrix(table, name, arrow_types), name) if name is not None else (None, None)


def _find_parquet_files(root: Path) -> list[Path]:
    """Return contained parquet files without importing PyArrow.

    Args:
        root: Local LeRobot dataset root.

    Returns:
        Resolved parquet paths contained by the dataset's data directory.
    """
    data_root = root / "data"
    if not data_root.is_dir():
        raise TrajectoryIngestError(f"No parquet files found under {data_root}")
    try:
        resolved_data_root = data_root.resolve(strict=True)
    except OSError as exc:
        raise TrajectoryIngestError(f"Could not resolve LeRobot data directory {data_root}: {exc}") from exc

    parquet_files: list[Path] = []
    for candidate in sorted(data_root.glob("**/*.parquet")):
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as exc:
            raise TrajectoryIngestError(f"Could not resolve LeRobot parquet path {candidate}: {exc}") from exc
        if not resolved.is_relative_to(resolved_data_root):
            raise TrajectoryIngestError(
                f"LeRobot parquet path escapes the dataset data directory through a symlink: {candidate}"
            )
        if resolved.is_file():
            parquet_files.append(resolved)
    if not parquet_files:
        raise TrajectoryIngestError(f"No parquet files found under {data_root}")
    return parquet_files


def _read_episode_table(
    root: Path,
    episode_index: int,
    arrow_dataset: Any,
    *,
    parquet_files: list[Path] | None = None,
) -> Any:
    data_root = root / "data"
    parquet_files = parquet_files if parquet_files is not None else _find_parquet_files(root)
    try:
        dataset = arrow_dataset.dataset([str(path) for path in parquet_files], format="parquet")
    except Exception as exc:
        raise TrajectoryIngestError(f"Could not open LeRobot parquet data under {data_root}: {exc}") from exc

    schema_names = set(dataset.schema.names)
    if "episode_index" not in schema_names:
        raise TrajectoryIngestError("LeRobot parquet data is missing the required 'episode_index' column.")

    candidates = {
        "episode_index",
        "timestamp",
        *_ORDER_COLUMNS,
        *_POSITION_COLUMNS,
        *_VELOCITY_COLUMNS,
        *_COMMAND_COLUMNS,
        *_TORQUE_COLUMNS,
        *_END_EFFECTOR_COLUMNS,
        *_CONTACT_COLUMNS,
    }
    columns = sorted(schema_names.intersection(candidates))
    try:
        table = dataset.to_table(
            columns=columns,
            filter=arrow_dataset.field("episode_index") == episode_index,
        )
    except Exception as exc:
        raise TrajectoryIngestError(f"Could not read LeRobot episode {episode_index}: {exc}") from exc
    if table.num_rows == 0:
        raise TrajectoryIngestError(f"LeRobot episode index {episode_index} was not found in {data_root}.")

    order_name = _find_column(set(table.column_names), _ORDER_COLUMNS)
    if order_name is not None:
        try:
            table = table.sort_by([(order_name, "ascending")])
        except Exception as exc:
            raise TrajectoryIngestError(f"Could not order LeRobot episode by '{order_name}': {exc}") from exc
    return table


def load_lerobot_trajectory(config: TrajectoryLoadConfig) -> TrajectoryDataset:
    """Load low-dimensional telemetry from a local LeRobot-format dataset.

    Args:
        config: Local dataset path and episode selection.

    Returns:
        Validated trajectory data for the selected episode.
    """
    root = Path(config.source_path).expanduser()
    if not root.is_dir():
        raise TrajectoryIngestError(f"LeRobot local dataset directory not found: {root}")

    try:
        episode_index = int(config.lerobot_episode_index)
    except (TypeError, ValueError) as exc:
        raise TrajectoryIngestError("LeRobot episode index must be an integer.") from exc
    if episode_index < 0:
        raise TrajectoryIngestError("LeRobot episode index must be non-negative.")

    info = _load_info(root)
    parquet_files = _find_parquet_files(root)
    arrow_dataset, arrow_types = _load_pyarrow_modules()
    table = _read_episode_table(root, episode_index, arrow_dataset, parquet_files=parquet_files)
    names = set(table.column_names)

    position_name = _find_column(names, _POSITION_COLUMNS)
    if position_name is None:
        available = ", ".join(sorted(names))
        raise TrajectoryIngestError(
            "LeRobot data has no supported joint-position column. Expected one of "
            f"{', '.join(_POSITION_COLUMNS)}; available columns: {available}."
        )
    positions = _column_to_matrix(table, position_name, arrow_types)
    velocities, velocity_name = _optional_matrix(table, names, _VELOCITY_COLUMNS, arrow_types)
    commands, command_name = _optional_matrix(table, names, _COMMAND_COLUMNS, arrow_types)
    torques, torque_name = _optional_matrix(table, names, _TORQUE_COLUMNS, arrow_types)
    end_effector_poses, end_effector_name = _optional_matrix(table, names, _END_EFFECTOR_COLUMNS, arrow_types)
    contact_forces, contact_name = _optional_matrix(table, names, _CONTACT_COLUMNS, arrow_types)

    if "timestamp" in names:
        times = _column_to_vector(table, "timestamp", arrow_types)
        time_source = "timestamp"
    else:
        frame_name = _find_column(names, _ORDER_COLUMNS)
        if frame_name is None:
            raise TrajectoryIngestError(
                "LeRobot data requires 'timestamp', 'frame_index', or 'index' to construct a timebase."
            )
        try:
            fps = float(info["fps"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TrajectoryIngestError(
                "LeRobot metadata requires a finite positive 'fps' when no timestamp column is present."
            ) from exc
        if not np.isfinite(fps) or fps <= 0.0:
            raise TrajectoryIngestError(
                "LeRobot metadata requires a finite positive 'fps' when no timestamp column is present."
            )
        frame_indices = _column_to_vector(table, frame_name, arrow_types)
        times = (frame_indices - frame_indices[0]) / fps
        time_source = f"{frame_name}/fps"

    selected_columns = {
        "positions": position_name,
        "velocities": velocity_name,
        "commands": command_name,
        "torques": torque_name,
        "end_effector_poses": end_effector_name,
        "contact_forces": contact_name,
    }
    extra: dict[str, Any] = {
        "episode_index": episode_index,
        "loader": "local_pyarrow",
        "time_source": time_source,
        "selected_columns": {key: value for key, value in selected_columns.items() if value is not None},
    }
    for key in ("codebase_version", "dataset_version", "license"):
        if key in info:
            extra[key] = info[key]
    semantics = _resolve_torque_semantics(info, torques is not None)
    if semantics is not None:
        extra["torque_semantics"] = semantics

    return build_trajectory_dataset(
        times=times,
        positions=positions,
        velocities=velocities,
        commands=commands,
        torques=torques,
        end_effector_poses=end_effector_poses,
        contact_forces=contact_forces,
        source_type="lerobot",
        source_path=str(root),
        extra=extra,
    )
