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

"""Render-product and RenderVar association rules for sensor assets."""

from __future__ import annotations

from typing import Any

import usd_validation_nvidia
from pxr import Usd, UsdRender

from .sensor_rule_utils import (
    SENSOR_TYPE_NAMES,
    RuleRegistrar,
    fail,
    feature_sensor_prims,
    feature_sensor_type,
    ordered_render_vars,
    render_products,
    render_var_source_name,
    warn,
)

_CATEGORY = "IsaacSim.Sensor.Shared"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policy("RENDER-VAR-002")


@_RULES.rule("RENDER-ASSOC-001")
def _one_sensor_per_render_product(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetCameraRel()
        targets = relationship.GetTargets()
        if len(targets) != 1:
            fail(
                checker,
                requirement,
                f"RenderProduct {prim.GetPath()} resolves {len(targets)} camera targets; expected exactly one",
                relationship,
            )


@_RULES.rule("RENDER-ASSOC-002")
def _sensor_target_existence(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetCameraRel()
        for path in relationship.GetTargets():
            if not stage.GetPrimAtPath(path):
                fail(checker, requirement, f"RenderProduct {prim.GetPath()} targets missing prim {path}", relationship)


@_RULES.rule("RENDER-ASSOC-003")
def _render_product_coverage(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    targeted = {path for prim in render_products(stage) for path in UsdRender.Product(prim).GetCameraRel().GetTargets()}
    for sensor in feature_sensor_prims(checker, stage):
        if sensor.GetPath() not in targeted:
            warn(checker, requirement, f"Sensor {sensor.GetPath()} is not targeted by a RenderProduct", sensor)


@_RULES.rule("RENDER-ASSOC-004")
def _sensor_target_type(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    expected_type = feature_sensor_type(checker)
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetCameraRel()
        for path in relationship.GetTargets():
            target = stage.GetPrimAtPath(path)
            if target and (
                target.GetTypeName() != expected_type
                if expected_type
                else target.GetTypeName() not in SENSOR_TYPE_NAMES
            ):
                description = repr(expected_type) if expected_type else f"one of {sorted(SENSOR_TYPE_NAMES)}"
                fail(
                    checker,
                    requirement,
                    f"RenderProduct {prim.GetPath()} camera target {path} has type {target.GetTypeName()!r}; "
                    f"expected {description}",
                    relationship,
                )


@_RULES.rule("RENDER-PRODUCT-001")
def _resolution_shape(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        attribute = UsdRender.Product(prim).GetResolutionAttr()
        value = attribute.Get()
        try:
            valid = value is not None and len(value) == 2
        except TypeError:
            valid = False
        if not valid:
            fail(
                checker,
                requirement,
                f"RenderProduct {prim.GetPath()} does not resolve a two-component resolution",
                attribute,
            )


@_RULES.rule("RENDER-PRODUCT-002")
def _positive_resolution(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        attribute = UsdRender.Product(prim).GetResolutionAttr()
        value = attribute.Get()
        if value is None or len(value) != 2 or value[0] <= 0 or value[1] <= 0:
            fail(checker, requirement, f"RenderProduct {prim.GetPath()} resolution must be positive", attribute)


@_RULES.rule("RENDER-VAR-001")
def _render_var_presence(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetOrderedVarsRel()
        if not any(target_prim and target_prim.IsA(UsdRender.Var) for _, target_prim in ordered_render_vars(prim)):
            fail(checker, requirement, f"RenderProduct {prim.GetPath()} does not order a RenderVar", relationship)


@_RULES.rule("RENDER-VAR-003")
def _ordered_var_validity(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetOrderedVarsRel()
        for path, target in ordered_render_vars(prim):
            if not target or not target.IsA(UsdRender.Var):
                fail(checker, requirement, f"orderedVars target {path} is not a valid RenderVar", relationship)


@_RULES.rule("RENDER-VAR-004")
def _unique_source_name(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        seen: set[str] = set()
        relationship = UsdRender.Product(prim).GetOrderedVarsRel()
        for _, render_var in ordered_render_vars(prim):
            source_name = render_var_source_name(render_var)
            if source_name and source_name in seen:
                fail(
                    checker,
                    requirement,
                    f"RenderProduct {prim.GetPath()} orders duplicate sourceName {source_name!r}",
                    relationship,
                )
            seen.add(source_name)


@_RULES.rule("RENDER-VAR-005")
def _nonempty_source_name(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in render_products(stage):
        relationship = UsdRender.Product(prim).GetOrderedVarsRel()
        for path, render_var in ordered_render_vars(prim):
            if render_var and render_var.IsA(UsdRender.Var) and not render_var_source_name(render_var):
                fail(checker, requirement, f"RenderVar {path} resolves an empty sourceName", relationship)
