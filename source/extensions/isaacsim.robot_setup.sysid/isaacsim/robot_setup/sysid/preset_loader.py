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

"""YAML parameter presets for common SysID workflows."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass, field
from importlib.resources import files
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .optimizer_config import OptimizerBackendConfig
from .residual_config import ResidualWeightConfig

if TYPE_CHECKING:
    from .parameter_space import ParameterSpace


class ParameterPresetError(RuntimeError):
    """Raised when a SysID parameter preset cannot be loaded."""


_RESIDUAL_WEIGHTS_MODES = ("explicit", "auto")


@dataclass(frozen=True)
class ParameterPreset:
    """A named SysID workflow preset (optionally a full workflow recipe).

    The recipe sections (``simulation``, ``solver``, ``chunking``,
    ``residual_weights_mode``) are optional and default to empty/explicit so
    parameter-only presets keep working unchanged.
    """

    name: str
    description: str = ""
    include_basic: bool = True
    include_com_offsets: bool = False
    include_inertia_log_cholesky: bool = False
    include_joint_limit_scales: bool = False
    include_per_link_mass: bool = False
    include_command_delay: bool = False
    optimizer_backend: dict[str, Any] = field(default_factory=dict)
    residual_weights: dict[str, Any] = field(default_factory=dict)
    #: "explicit" applies `residual_weights` verbatim; "auto" derives channel
    #: weights from telemetry channel presence (requires a loaded trajectory).
    residual_weights_mode: str = "explicit"
    #: Optional engine settings: {engine, newton: {solver, featherstone_substeps, mujoco_substeps, device}}.
    simulation: dict[str, Any] = field(default_factory=dict)
    #: Optional solver overrides: max_iterations, max_rollout_steps, backend fields.
    solver: dict[str, Any] = field(default_factory=dict)
    #: Optional chunking policy: {mode: "auto", train_fraction, min_chunk_seconds}.
    chunking: dict[str, Any] = field(default_factory=dict)
    recommended_scenes: tuple[str, ...] = field(default_factory=tuple)
    notes: tuple[str, ...] = field(default_factory=tuple)

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ParameterPreset":  # noqa: D102
        if "name" not in payload:
            raise ParameterPresetError("Preset YAML must include a name.")
        residual_weights_mode = str(payload.get("residual_weights_mode", "explicit"))
        if residual_weights_mode not in _RESIDUAL_WEIGHTS_MODES:
            raise ParameterPresetError(
                f"residual_weights_mode must be one of {_RESIDUAL_WEIGHTS_MODES}, got '{residual_weights_mode}'."
            )
        return cls(
            name=str(payload["name"]),
            description=str(payload.get("description", "")),
            include_basic=bool(payload.get("include_basic", True)),
            include_com_offsets=bool(payload.get("include_com_offsets", False)),
            include_inertia_log_cholesky=bool(payload.get("include_inertia_log_cholesky", False)),
            include_joint_limit_scales=bool(payload.get("include_joint_limit_scales", False)),
            include_per_link_mass=bool(payload.get("include_per_link_mass", False)),
            include_command_delay=bool(payload.get("include_command_delay", False)),
            optimizer_backend=dict(payload.get("optimizer_backend", {}) or {}),
            residual_weights=dict(payload.get("residual_weights", {}) or {}),
            residual_weights_mode=residual_weights_mode,
            simulation=dict(payload.get("simulation", {}) or {}),
            solver=dict(payload.get("solver", {}) or {}),
            chunking=dict(payload.get("chunking", {}) or {}),
            recommended_scenes=tuple(str(item) for item in payload.get("recommended_scenes", ()) or ()),
            notes=tuple(str(item) for item in payload.get("notes", ()) or ()),
        )

    def build_parameter_space(self, *, num_joints: int, num_links: int) -> ParameterSpace:  # noqa: D102
        # ParameterSpace includes USD read/write support and therefore imports
        # pxr. Keep it behind the operation that actually needs it so preset
        # discovery and parsing remain available in a base standalone install.
        from .parameter_space import ParameterSpace

        return ParameterSpace.for_robot_extended(
            num_joints=num_joints,
            num_links=num_links,
            include_basic=self.include_basic,
            include_com_offsets=self.include_com_offsets,
            include_inertia_log_cholesky=self.include_inertia_log_cholesky,
            include_joint_limit_scales=self.include_joint_limit_scales,
            include_per_link_mass=self.include_per_link_mass,
            include_command_delay=self.include_command_delay,
        )

    def build_residual_weight_config(self) -> ResidualWeightConfig:  # noqa: D102
        return ResidualWeightConfig.from_dict(dict(self.residual_weights))

    def build_optimizer_backend_config(self) -> OptimizerBackendConfig:  # noqa: D102
        return OptimizerBackendConfig.from_dict(dict(self.optimizer_backend))


def builtin_preset_dir() -> Path:
    """Return the package-owned built-in preset directory.

    Returns:
        Directory containing built-in preset resources.
    """
    return Path(files("isaacsim.robot_setup.sysid").joinpath("resources", "presets"))


def _load_mapping(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ParameterPresetError(f"Parameter preset not found: {path}")
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".yaml", ".yml"):
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise ParameterPresetError("PyYAML is required to read SysID YAML presets.") from exc
        payload = yaml.safe_load(text)
    else:
        payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ParameterPresetError(f"Parameter preset must contain a mapping: {path}")
    return payload


def load_parameter_preset(path: str | Path) -> ParameterPreset:  # noqa: D103
    return ParameterPreset.from_dict(_load_mapping(Path(path).expanduser()))


def list_builtin_parameter_presets() -> list[str]:  # noqa: D103
    preset_dir = builtin_preset_dir()
    if not preset_dir.is_dir():
        return []
    return sorted(path.stem for path in preset_dir.glob("*.y*ml"))


def load_builtin_parameter_preset(name: str) -> ParameterPreset:  # noqa: D103
    cleaned = name.strip().replace(" ", "_").lower()
    preset_dir = builtin_preset_dir()
    candidates = (
        preset_dir / f"{cleaned}.yaml",
        preset_dir / f"{cleaned}.yml",
    )
    for path in candidates:
        if path.is_file():
            return load_parameter_preset(path)
    available = ", ".join(list_builtin_parameter_presets()) or "<none>"
    raise ParameterPresetError(f"Unknown built-in SysID preset '{name}'. Available presets: {available}.")
