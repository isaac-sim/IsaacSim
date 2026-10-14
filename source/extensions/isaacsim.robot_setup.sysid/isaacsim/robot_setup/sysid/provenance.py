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

"""Dataset and parameter provenance for SysID runs."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .parameter_types import ParameterCategory, SysIdParameterEntry
from .provenance_schema import (
    ATTR_CALIBRATION_PARAMETER_NAMES,
    ATTR_CALIBRATION_PARAMETER_VALUES,
    ATTR_DATASET_SOURCE_PATH,
    ATTR_DATASET_SOURCE_TYPE,
    ATTR_FINAL_COST,
    ATTR_ITERATIONS,
    ATTR_OPTIMIZED_AT,
    ATTR_OPTIMIZER_BACKEND,
    ATTR_PARAMETER_CATEGORIES,
    ATTR_PARAMETER_NAMES,
    ATTR_PARAMETER_TYPES,
    ATTR_PARAMETER_VALUES,
    ATTR_PROVENANCE_JSON,
    ATTR_ROBOT_PRIM_PATH,
    ATTR_SCHEMA_VERSION,
    ATTR_SYSID_PARAMETER_NAMES,
    ATTR_SYSID_PARAMETER_VALUES,
    SYSID_PROVENANCE_SCHEMA_VERSION,
)


@dataclass
class TrajectoryDatasetMetadata:
    """Provenance metadata attached to a loaded :class:`TrajectoryDataset`."""

    source_type: str
    source_path: str
    num_samples: int
    num_joints: int
    duration_seconds: float
    column_mapping_path: str | None = None
    topic_mapping_path: str | None = None
    loaded_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this metadata.

        Returns:
            Dataclass fields as JSON-compatible values.
        """
        return asdict(self)

    def to_json(self) -> str:
        """Serialize this metadata to formatted JSON text.

        Returns:
            Indented JSON metadata text.
        """
        return json.dumps(self.to_dict(), indent=2)


def build_dataset_metadata(
    *,
    source_type: str,
    source_path: str,
    times: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    commands: np.ndarray,
    torques: np.ndarray | None = None,
    end_effector_poses: np.ndarray | None = None,
    contact_forces: np.ndarray | None = None,
    column_mapping_path: str | None = None,
    topic_mapping_path: str | None = None,
    extra: dict[str, Any] | None = None,
) -> TrajectoryDatasetMetadata:
    """Construct lightweight metadata without hashing trajectory contents.

    Args:
        source_type: Loader format used for the trajectory.
        source_path: User-selected dataset path.
        times: Sample timestamps with shape ``(T,)``.
        positions: Joint positions with shape ``(T, N)``.
        velocities: Joint velocities with shape ``(T, N)``.
        commands: Controller commands with shape ``(T, N)``.
        torques: Optional measured joint torques.
        end_effector_poses: Optional measured end-effector poses.
        contact_forces: Optional measured contact forces.
        column_mapping_path: CSV column-map source, when applicable.
        topic_mapping_path: Bag or MCAP topic-map source, when applicable.
        extra: Additional loader-specific provenance.

    Returns:
        Dataset dimensions, duration, source, and mapping metadata.
    """
    duration = float(times[-1] - times[0]) if times.shape[0] >= 2 else 0.0
    return TrajectoryDatasetMetadata(
        source_type=source_type,
        source_path=source_path,
        num_samples=int(times.shape[0]),
        num_joints=int(positions.shape[1]),
        duration_seconds=duration,
        column_mapping_path=column_mapping_path,
        topic_mapping_path=topic_mapping_path,
        extra=extra or {},
    )


@dataclass(frozen=True)
class ParameterProvenanceEntry:
    """One optimized parameter value with category and addressing metadata."""

    name: str
    value: float
    param_type: str
    category: str
    dof_index: int
    link_index: int
    component_index: int = 0

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this parameter entry.

        Returns:
            Dataclass fields as JSON-compatible values.
        """
        return asdict(self)


@dataclass
class SysIdProvenanceRecord:
    """Full provenance record written to USD and optional JSON sidecars."""

    schema_version: str
    dataset_source_type: str
    dataset_source_path: str
    optimized_at: str
    optimizer_backend: str
    final_cost: float
    iterations: int
    robot_prim_path: str
    parameters: list[dict[str, Any]] = field(default_factory=list)
    sysid_parameters: list[dict[str, Any]] = field(default_factory=list)
    calibration_parameters: list[dict[str, Any]] = field(default_factory=list)
    validation_metrics: list[dict[str, Any]] = field(default_factory=list)
    simulation_engine: str = ""
    newton_config: dict[str, Any] | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a deterministic JSON-ready representation of this record.

        Returns:
            Dataclass fields as a deterministic mapping.
        """
        return asdict(self)

    def to_json(self) -> str:
        """Serialize this record to formatted JSON.

        Returns:
            Deterministically ordered, indented JSON record text.
        """
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> SysIdProvenanceRecord:
        """Create a provenance record from JSON or USD mirror attributes.

        Args:
            payload: Serialized input payload.

        Returns:
            Provenance record populated with defaults for optional fields.
        """
        return cls(
            schema_version=str(payload.get("schema_version", SYSID_PROVENANCE_SCHEMA_VERSION)),
            dataset_source_type=str(payload.get("dataset_source_type", "")),
            dataset_source_path=str(payload.get("dataset_source_path", "")),
            optimized_at=str(payload.get("optimized_at", "")),
            optimizer_backend=str(payload.get("optimizer_backend", "")),
            final_cost=float(payload.get("final_cost", 0.0)),
            iterations=int(payload.get("iterations", 0)),
            robot_prim_path=str(payload.get("robot_prim_path", "")),
            parameters=list(payload.get("parameters", []) or []),
            sysid_parameters=list(payload.get("sysid_parameters", []) or []),
            calibration_parameters=list(payload.get("calibration_parameters", []) or []),
            validation_metrics=list(payload.get("validation_metrics", []) or []),
            simulation_engine=str(payload.get("simulation_engine", "")),
            newton_config=payload.get("newton_config"),
            extra=dict(payload.get("extra", {}) or {}),
        )


def split_parameters_by_category(
    entries: list[SysIdParameterEntry],
    theta: list[float] | np.ndarray,
    *,
    num_joints: int,
) -> tuple[
    list[ParameterProvenanceEntry],
    list[ParameterProvenanceEntry],
    list[ParameterProvenanceEntry],
]:
    """Split optimized theta into all/sysid/calibration provenance entries.

    Args:
        entries: Metadata for each optimized theta column.
        theta: Optimized parameter values in entry order.
        num_joints: Joint count used to produce display names.

    Returns:
        All entries, SysID entries, and calibration entries in stable order.
    """
    values = np.asarray(theta, dtype=np.float64).reshape(-1)
    if values.shape[0] != len(entries):
        raise ValueError(
            f"Provenance parameter count mismatch: received {values.shape[0]} values for {len(entries)} entries."
        )
    all_entries: list[ParameterProvenanceEntry] = []
    sysid_entries: list[ParameterProvenanceEntry] = []
    calibration_entries: list[ParameterProvenanceEntry] = []
    for idx, entry in enumerate(entries):
        category = entry.resolved_category()
        provenance = ParameterProvenanceEntry(
            name=entry.display_name(num_joints),
            value=float(values[idx]),
            param_type=entry.param_type.value,
            category=category.value,
            dof_index=int(entry.dof_index),
            link_index=int(entry.link_index),
            component_index=int(entry.component_index),
        )
        all_entries.append(provenance)
        if category == ParameterCategory.SYSID:
            sysid_entries.append(provenance)
        else:
            calibration_entries.append(provenance)
    return all_entries, sysid_entries, calibration_entries


def build_provenance_record(
    *,
    config: Any,
    final_status: Any,
    optimizer_backend: str,
    robot_prim_path: str,
    validation_metrics: list[Any] | None = None,
    validation_comparison: dict[str, Any] | None = None,
    simulation_engine: str = "",
    newton_config: dict[str, Any] | None = None,
) -> SysIdProvenanceRecord:
    """Build a provenance record from the optimizer config and selected final status.

    Args:
        config: Optimizer configuration containing trajectory and parameter metadata.
        final_status: Final optimizer status containing theta, cost, and acceptance.
        optimizer_backend: Concrete backend used for the solve.
        robot_prim_path: Robot prim associated with the result.
        validation_metrics: Optional held-out validation summaries.
        validation_comparison: Optional authored-versus-tuned validation comparison.
        simulation_engine: Rollout engine used for optimization.
        newton_config: Optional serialized Newton runtime settings.

    Returns:
        Complete provenance record ready for USD or JSON persistence.
    """
    trajectory = getattr(config, "trajectory", None)
    metadata = getattr(trajectory, "metadata", None)
    metadata_dict = metadata.to_dict() if hasattr(metadata, "to_dict") else {}
    source_type = str(metadata_dict.get("source_type", ""))
    source_path = str(metadata_dict.get("source_path", ""))
    theta = list(getattr(final_status, "theta", []) or [])
    entries = list(getattr(config, "param_entries", []) or [])
    num_joints = int(getattr(trajectory, "num_joints", 1) or 1)
    all_params, sysid_params, calibration_params = split_parameters_by_category(entries, theta, num_joints=num_joints)
    metric_payloads = [
        metric.to_dict() if hasattr(metric, "to_dict") else dict(metric) for metric in (validation_metrics or [])
    ]
    return SysIdProvenanceRecord(
        schema_version=SYSID_PROVENANCE_SCHEMA_VERSION,
        dataset_source_type=source_type,
        dataset_source_path=source_path,
        optimized_at=datetime.now(timezone.utc).isoformat(),
        optimizer_backend=str(optimizer_backend),
        final_cost=float(getattr(final_status, "cost", 0.0)),
        iterations=int(getattr(final_status, "iteration", 0)),
        robot_prim_path=str(robot_prim_path),
        parameters=[entry.to_dict() for entry in all_params],
        sysid_parameters=[entry.to_dict() for entry in sysid_params],
        calibration_parameters=[entry.to_dict() for entry in calibration_params],
        validation_metrics=metric_payloads,
        simulation_engine=str(simulation_engine),
        newton_config=newton_config,
        extra={
            "accepted": bool(getattr(final_status, "accepted", False)),
            "validation_comparison": dict(validation_comparison or {}),
        },
    )


class ProvenanceWriter:
    """Read and write SysID provenance records on USD stages and JSON sidecars."""

    def write_to_stage(self, stage: Any, robot_prim_path: str, record: SysIdProvenanceRecord) -> None:
        """Write searchable USD attributes plus the canonical JSON record.

        Args:
            stage: Writable USD stage.
            robot_prim_path: Robot prim that owns the custom provenance attributes.
            record: Canonical provenance values to persist.
        """
        prim = stage.GetPrimAtPath(robot_prim_path)
        if not prim or not prim.IsValid():
            raise ValueError(f"Robot prim path is not valid: {robot_prim_path}")
        self._set_attr(prim, ATTR_SCHEMA_VERSION, record.schema_version, "String")
        self._set_attr(prim, ATTR_DATASET_SOURCE_TYPE, record.dataset_source_type, "String")
        self._set_attr(prim, ATTR_DATASET_SOURCE_PATH, record.dataset_source_path, "String")
        self._set_attr(prim, ATTR_OPTIMIZED_AT, record.optimized_at, "String")
        self._set_attr(prim, ATTR_OPTIMIZER_BACKEND, record.optimizer_backend, "Token")
        self._set_attr(prim, ATTR_FINAL_COST, float(record.final_cost), "Double")
        self._set_attr(prim, ATTR_ITERATIONS, int(record.iterations), "Int")
        self._set_attr(prim, ATTR_ROBOT_PRIM_PATH, record.robot_prim_path, "String")
        self._set_attr(prim, ATTR_PARAMETER_NAMES, _names(record.parameters), "StringArray")
        self._set_attr(prim, ATTR_PARAMETER_VALUES, _values(record.parameters), "DoubleArray")
        self._set_attr(
            prim,
            ATTR_PARAMETER_CATEGORIES,
            _categories(record.parameters),
            "TokenArray",
        )
        self._set_attr(prim, ATTR_PARAMETER_TYPES, _types(record.parameters), "TokenArray")
        self._set_attr(
            prim,
            ATTR_SYSID_PARAMETER_NAMES,
            _names(record.sysid_parameters),
            "StringArray",
        )
        self._set_attr(
            prim,
            ATTR_SYSID_PARAMETER_VALUES,
            _values(record.sysid_parameters),
            "DoubleArray",
        )
        self._set_attr(
            prim,
            ATTR_CALIBRATION_PARAMETER_NAMES,
            _names(record.calibration_parameters),
            "StringArray",
        )
        self._set_attr(
            prim,
            ATTR_CALIBRATION_PARAMETER_VALUES,
            _values(record.calibration_parameters),
            "DoubleArray",
        )
        self._set_attr(prim, ATTR_PROVENANCE_JSON, record.to_json(), "String")

    def read_from_stage(self, stage: Any, robot_prim_path: str) -> SysIdProvenanceRecord:
        """Read a provenance record from USD, preferring the canonical JSON attribute.

        Args:
            stage: USD stage containing the robot.
            robot_prim_path: Robot prim that owns the provenance attributes.

        Returns:
            Canonical JSON record, or a record reconstructed from mirror attributes.
        """
        prim = stage.GetPrimAtPath(robot_prim_path)
        if not prim or not prim.IsValid():
            raise ValueError(f"Robot prim path is not valid: {robot_prim_path}")
        json_text = _get_attr(prim, ATTR_PROVENANCE_JSON, "")
        if json_text:
            return SysIdProvenanceRecord.from_dict(json.loads(str(json_text)))
        names = list(_get_attr(prim, ATTR_PARAMETER_NAMES, []) or [])
        values = list(_get_attr(prim, ATTR_PARAMETER_VALUES, []) or [])
        categories = list(_get_attr(prim, ATTR_PARAMETER_CATEGORIES, []) or [])
        types = list(_get_attr(prim, ATTR_PARAMETER_TYPES, []) or [])
        params = []
        for idx, name in enumerate(names):
            params.append(
                {
                    "name": str(name),
                    "value": float(values[idx]) if idx < len(values) else 0.0,
                    "category": str(categories[idx]) if idx < len(categories) else "",
                    "param_type": str(types[idx]) if idx < len(types) else "",
                    "dof_index": -1,
                    "link_index": -1,
                    "component_index": 0,
                }
            )
        sysid_names = {str(name) for name in (_get_attr(prim, ATTR_SYSID_PARAMETER_NAMES, []) or [])}
        calibration_names = {str(name) for name in (_get_attr(prim, ATTR_CALIBRATION_PARAMETER_NAMES, []) or [])}
        return SysIdProvenanceRecord(
            schema_version=str(_get_attr(prim, ATTR_SCHEMA_VERSION, SYSID_PROVENANCE_SCHEMA_VERSION)),
            dataset_source_type=str(_get_attr(prim, ATTR_DATASET_SOURCE_TYPE, "")),
            dataset_source_path=str(_get_attr(prim, ATTR_DATASET_SOURCE_PATH, "")),
            optimized_at=str(_get_attr(prim, ATTR_OPTIMIZED_AT, "")),
            optimizer_backend=str(_get_attr(prim, ATTR_OPTIMIZER_BACKEND, "")),
            final_cost=float(_get_attr(prim, ATTR_FINAL_COST, 0.0)),
            iterations=int(_get_attr(prim, ATTR_ITERATIONS, 0)),
            robot_prim_path=str(_get_attr(prim, ATTR_ROBOT_PRIM_PATH, robot_prim_path)),
            parameters=params,
            sysid_parameters=[param for param in params if str(param.get("name", "")) in sysid_names],
            calibration_parameters=[param for param in params if str(param.get("name", "")) in calibration_names],
            extra={"calibration_parameter_names": sorted(calibration_names)},
        )

    def export_json_sidecar(self, path: str, record: SysIdProvenanceRecord) -> str:
        """Write a deterministic JSON provenance sidecar and return its path.

        Args:
            path: Destination JSON sidecar path.
            record: Provenance values to serialize.

        Returns:
            Expanded destination path.
        """
        file_path = Path(path).expanduser()
        file_path.parent.mkdir(parents=True, exist_ok=True)
        file_path.write_text(record.to_json() + "\n", encoding="utf-8")
        return str(file_path)

    @staticmethod
    def _set_attr(prim: Any, name: str, value: Any, type_name: str) -> None:
        attr = prim.GetAttribute(name)
        if attr is None or not attr.IsValid():
            attr = prim.CreateAttribute(name, _sdf_type(type_name), custom=True)
        attr.Set(value)


def _names(parameters: list[dict[str, Any]]) -> list[str]:
    return [str(param.get("name", "")) for param in parameters]


def _values(parameters: list[dict[str, Any]]) -> list[float]:
    return [float(param.get("value", 0.0)) for param in parameters]


def _categories(parameters: list[dict[str, Any]]) -> list[str]:
    return [str(param.get("category", "")) for param in parameters]


def _types(parameters: list[dict[str, Any]]) -> list[str]:
    return [str(param.get("param_type", "")) for param in parameters]


def _get_attr(prim: Any, name: str, default: Any) -> Any:
    attr = prim.GetAttribute(name)
    if attr is None or not attr.IsValid():
        return default
    value = attr.Get()
    return default if value is None else value


def _sdf_type(type_name: str) -> Any:
    from pxr import Sdf

    return getattr(Sdf.ValueTypeNames, type_name)
