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

"""Common OmniSensor API and model metadata rules."""

from __future__ import annotations

import math
from typing import Any

import usd_validation_nvidia
from pxr import Usd

from .sensor_rule_utils import RuleRegistrar, attribute_value, fail, feature_sensor_prims

_CATEGORY = "IsaacSim.Sensor.Shared"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policy("SENSOR-MODEL-001")


@_RULES.rule("SENSOR-SCHEMA-001")
def _sensor_api(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for sensor in feature_sensor_prims(checker, stage):
        if not sensor.HasAPI("OmniSensorAPI"):
            fail(checker, requirement, f"Sensor {sensor.GetPath()} does not conform to OmniSensorAPI", sensor)


@_RULES.rule("SENSOR-TIME-001")
def _tick_rate(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for sensor in feature_sensor_prims(checker, stage):
        attribute = sensor.GetAttribute("omni:sensor:tickRate")
        value = attribute.Get() if attribute else None
        if value is None or not math.isfinite(value) or value < 0:
            fail(
                checker,
                requirement,
                f"Sensor {sensor.GetPath()} tickRate must be finite and nonnegative",
                attribute or sensor,
            )


def _model_string(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any, name: str
) -> None:
    for sensor in feature_sensor_prims(checker, stage):
        model_name = attribute_value(sensor, "omni:sensor:modelName")
        if model_name and not str(attribute_value(sensor, name) or "").strip():
            fail(checker, requirement, f"Sensor {sensor.GetPath()} resolves an empty {name}", sensor.GetAttribute(name))


_RULES.check(
    "SENSOR-MODEL-002",
    lambda checker, stage, requirement: _model_string(checker, stage, requirement, "omni:sensor:modelVendor"),
)
_RULES.check(
    "SENSOR-MODEL-003",
    lambda checker, stage, requirement: _model_string(checker, stage, requirement, "omni:sensor:marketName"),
)
