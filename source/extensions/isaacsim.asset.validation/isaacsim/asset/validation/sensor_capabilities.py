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

"""SimReady capability metadata for authored RTX sensor assets.

The catalog is intentionally kept separate from the validators. It contains
the requirement contract, including non-constraining policy statements; PICs,
review statuses, and discussion remain in the source spreadsheet. A requirement
receives an implementation only when a validator is explicitly registered for it.
"""

from __future__ import annotations

import json
import re
from dataclasses import replace
from pathlib import Path

import usd_validation_nvidia
from usd_validation_nvidia.capabilities import Capability, Feature, Parameter, ParameterType, Requirement

__all__ = [
    "CAPABILITIES",
    "FEATURES",
    "FEATURE_SENSOR_PRIM_TYPES",
    "REQUIREMENTS",
    "get_requirement",
]

_VERSION = "0.1.0"
_REQUIREMENT_CODE_PREFIX = "com.nvidia.isaac.sensor."
_CATALOG_PATH = Path(__file__).with_name("data") / "sensor_capabilities.json"


def _slug(value: str) -> str:
    """Return a stable lowercase identifier fragment.

    Args:
        value: Display name to convert.

    Returns:
        Identifier containing lowercase letters, digits, and underscores.
    """
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


def _load_requirements() -> tuple[dict[str, Requirement], dict[str, list[Requirement]]]:
    """Load requirement metadata generated from the reviewed spreadsheet.

    Returns:
        Requirements by short ID and requirements grouped by capability name.
    """
    rows = json.loads(_CATALOG_PATH.read_text(encoding="utf-8"))
    requirements: dict[str, Requirement] = {}
    grouped: dict[str, list[Requirement]] = {}
    for row in rows:
        requirement_id = row["Requirement ID"]
        capability_name = row["Capability name"]
        requirement = Requirement(
            code=f"{_REQUIREMENT_CODE_PREFIX}{requirement_id}",
            version=_VERSION,
            display_name=row["Requirement name"],
            message=row["Requirement"],
            path=f"sensor/{_slug(capability_name)}/{requirement_id.lower()}.html",
            compatibility="kit-110.4",
            validator=None,
            tags=("sensor", "asset-authoring", _slug(capability_name)),
        )
        if requirement_id in requirements:
            raise ValueError(f"Duplicate sensor requirement ID: {requirement_id}")
        requirements[requirement_id] = requirement
        grouped.setdefault(capability_name, []).append(requirement)
    return requirements, grouped


REQUIREMENTS, _REQUIREMENTS_BY_CAPABILITY = _load_requirements()

CAPABILITIES = {
    name: Capability(
        id=f"com.nvidia.isaac.sensor.{_slug(name)}",
        version=_VERSION,
        path=f"sensor/{_slug(name)}/capability.html",
        requirements=requirements,
    )
    for name, requirements in _REQUIREMENTS_BY_CAPABILITY.items()
}

_FEATURE_CAPABILITY_NAMES = {
    "Camera.RGB": (
        "Shippable Asset Structure",
        "Render Graph",
        "Core Sensor Contract",
        "Camera Core",
        "RGB Output",
        "Camera Lens Calibration",
        "SPG Post-Processing",
    ),
    "Camera.Depth.SingleView": (
        "Shippable Asset Structure",
        "Render Graph",
        "Core Sensor Contract",
        "Camera Core",
        "RGB Output",
        "Camera Lens Calibration",
        "Depth Single-View Output",
        "SPG Post-Processing",
    ),
    "Lidar.Generic": (
        "Shippable Asset Structure",
        "Render Graph",
        "Core Sensor Contract",
        "RTX Lidar Core",
    ),
    "Camera.ISP.Generic": (
        "Shippable Asset Structure",
        "Render Graph",
        "Core Sensor Contract",
        "Camera Core",
        "Generic Camera Core ISP",
        "Camera Lens Calibration",
        "SPG Post-Processing",
    ),
    "Radar.Generic.WpmDmat": (
        "Shippable Asset Structure",
        "Render Graph",
        "Core Sensor Contract",
        "RTX Radar WPM-DMAT",
    ),
}

FEATURE_SENSOR_PRIM_TYPES = {
    "Camera.RGB": "Camera",
    "Camera.Depth.SingleView": "Camera",
    "Lidar.Generic": "OmniLidar",
    "Camera.ISP.Generic": "Camera",
    "Radar.Generic.WpmDmat": "OmniRadar",
}

_SENSOR_PRIM_TYPE_PARAMETER = "SENSOR_PRIM_TYPE"
_SCOPED_REQUIREMENT_IDS = frozenset(
    {
        "ASSET-STRUCT-003",
        "RENDER-ASSOC-003",
        "RENDER-ASSOC-004",
        "SENSOR-SCHEMA-001",
        "SENSOR-TIME-001",
        "SENSOR-MODEL-002",
        "SENSOR-MODEL-003",
    }
)


def _feature_requirement(feature_name: str, requirement: Requirement) -> Requirement:
    """Attach feature scope to the requirement that supplies shared-rule parameters."""
    if requirement.code.removeprefix(_REQUIREMENT_CODE_PREFIX) not in _SCOPED_REQUIREMENT_IDS:
        return requirement
    return replace(
        requirement,
        parameters=(
            Parameter(
                display_name=_SENSOR_PRIM_TYPE_PARAMETER,
                type=ParameterType.ENUM,
                assigned_value=FEATURE_SENSOR_PRIM_TYPES[feature_name],
                enum_values=tuple(sorted(set(FEATURE_SENSOR_PRIM_TYPES.values()))),
            ),
        ),
    )


FEATURES = {
    name: Feature(
        id=f"com.nvidia.isaac.sensor.{_slug(name)}",
        version=_VERSION,
        path=f"sensor/features/{_slug(name)}.html",
        requirements=[
            _feature_requirement(name, requirement)
            for capability_name in capability_names
            for requirement in CAPABILITIES[capability_name].requirements
        ],
    )
    for name, capability_names in _FEATURE_CAPABILITY_NAMES.items()
}


def get_requirement(requirement_id: str) -> Requirement:
    """Return a sensor requirement by its spreadsheet ID.

    Args:
        requirement_id: Short requirement ID, for example ``CAMERA-ISP-001``.

    Returns:
        Concrete requirement metadata used by the validation registry.
    """
    return REQUIREMENTS[requirement_id]


usd_validation_nvidia.register_requirements(*REQUIREMENTS.values())(None)
usd_validation_nvidia.register_capabilities(CAPABILITIES.values())
usd_validation_nvidia.register_features(FEATURES.values())
