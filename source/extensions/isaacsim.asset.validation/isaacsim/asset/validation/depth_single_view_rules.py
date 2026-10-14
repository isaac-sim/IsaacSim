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

"""Single-view depth-output rules for Camera RenderProducts."""

from __future__ import annotations

import math
from functools import partial
from typing import Any

import usd_validation_nvidia
from pxr import Usd, UsdRender

from .camera_rule_utils import _DEPTH_API, _source_names
from .sensor_rule_utils import (
    AttributeNamespace,
    RuleRegistrar,
    attribute_value,
    fail,
    is_finite,
    make_bound_check,
    render_products,
)

_RULES = RuleRegistrar("IsaacSim.Sensor.Camera.Depth")
_DEPTH_PREFIX = "omni:rtx:post:depthSensor:"
_DEPTH = AttributeNamespace(_DEPTH_PREFIX)
_DEPTH_OUTPUTS = frozenset(
    {
        "DepthSensorDistance",
        "DepthSensorPointCloudPosition",
        "DepthSensorPointCloudColor",
        "DepthSensorImager",
        "DepthSensorOutputVisualization",
    }
)


def _depth_products(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in render_products(stage) if prim.HasAPI(_DEPTH_API)]


_DEPTH_BOUND = partial(make_bound_check, _depth_products, _DEPTH)


@_RULES.rule("DEPTH-SCHEMA-001")
def _depth_schema(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if not _depth_products(stage):
        fail(checker, requirement, "No RenderProduct has OmniSensorDepthSensorSingleViewAPI applied", stage)


def _depth_output_count(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    source_name: str,
    *,
    at_least_one: bool = False,
) -> None:
    for render_product in _depth_products(stage):
        names = _source_names(render_product)
        count = sum(name in _DEPTH_OUTPUTS for name in names) if at_least_one else names.count(source_name)
        if count < 1 if at_least_one else count != 1:
            description = (
                f"at least one depth output RenderVar with sourceName in {sorted(_DEPTH_OUTPUTS)}"
                if at_least_one
                else f"exactly one RenderVar with sourceName {source_name!r}"
            )
            fail(
                checker,
                requirement,
                f"Depth RenderProduct {render_product.GetPath()} must order {description}; found sourceNames {names}",
                UsdRender.Product(render_product).GetOrderedVarsRel(),
            )


@_RULES.rule("DEPTH-PARAM-001")
def _depth_finite(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in _depth_products(stage):
        for attribute in prim.GetAttributes():
            if attribute.GetName().startswith(_DEPTH_PREFIX) and not is_finite(attribute.Get()):
                fail(checker, requirement, f"{attribute.GetPath()} contains a non-finite value", attribute)


@_RULES.rule("DEPTH-PARAM-012")
def _ordered_depth_bounds(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in _depth_products(stage):
        minimum = attribute_value(prim, f"{_DEPTH_PREFIX}minDistance")
        maximum = attribute_value(prim, f"{_DEPTH_PREFIX}maxDistance")
        if minimum is None or maximum is None or maximum <= minimum:
            fail(
                checker,
                requirement,
                f"Depth RenderProduct {prim.GetPath()} maxDistance must be greater than minDistance",
                prim,
            )


for requirement_id, source_name in (
    ("DEPTH-OUTPUT-001", "LdrColor"),
    ("DEPTH-OUTPUT-002", "Camera3dPositionSD"),
    ("DEPTH-OUTPUT-003", "DistanceToImagePlaneSD"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, source_name=source_name: _depth_output_count(
            checker, stage, requirement, source_name
        ),
    )
_RULES.check(
    "DEPTH-OUTPUT-004",
    lambda checker, stage, requirement: _depth_output_count(checker, stage, requirement, "", at_least_one=True),
)
for requirement_id, name, predicate, description in (
    ("DEPTH-PARAM-002", "focalLengthPixel", lambda value: value > 0, "greater than zero"),
    ("DEPTH-PARAM-003", "maxDisparityPixel", lambda value: value > 0, "greater than zero"),
    ("DEPTH-PARAM-004", "maxDistance", lambda value: value > 0, "greater than zero"),
    ("DEPTH-PARAM-005", "sensorSizePixel", lambda value: value >= 1, "at least 1"),
    ("DEPTH-PARAM-006", "noiseDownscaleFactorPixel", lambda value: value >= 1, "at least 1"),
    ("DEPTH-PARAM-007", "noiseMean", lambda value: value >= 0, "nonnegative"),
    ("DEPTH-PARAM-008", "noiseSigma", lambda value: value >= 0, "nonnegative"),
    ("DEPTH-PARAM-009", "baselineMM", lambda value: math.isfinite(value), "finite"),
    ("DEPTH-PARAM-010", "minDistance", lambda value: value >= 0, "nonnegative"),
    ("DEPTH-PARAM-011", "effectiveDistance", lambda value: value >= 0, "nonnegative"),
    ("DEPTH-PARAM-013", "confidenceThreshold", lambda value: 0 <= value <= 1, "in [0, 1]"),
    ("DEPTH-PARAM-014", "rgbDepthOutputMode", lambda value: value in range(8), "an integer in [0, 7]"),
):
    _RULES.check(requirement_id, _DEPTH_BOUND(name, predicate, description))
