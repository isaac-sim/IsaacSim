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

"""Shippable sensor-asset root and presence rules."""

from __future__ import annotations

import math
from typing import Any

import usd_validation_nvidia
from pxr import Gf, Sdf, Usd, UsdGeom

from .sensor_rule_utils import (
    SENSOR_TYPE_NAMES,
    RuleRegistrar,
    fail,
    feature_sensor_prims,
    feature_sensor_type,
    render_products_targeting,
)

_CATEGORY = "IsaacSim.Sensor.Shared"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policy("ASSET-STRUCT-002")
_ROOT_TRANSFORM_TOLERANCE = 1e-6


def _whole_stage_default_prim(stage: Usd.Stage) -> Usd.Prim:
    return stage.GetDefaultPrim() if Sdf.Path.absoluteRootPath in stage.GetPopulationMask().GetPaths() else Usd.Prim()


def _root_transform_samples(stage: Usd.Stage) -> list[tuple[Usd.Prim, str, Gf.Matrix4d]]:
    root = _whole_stage_default_prim(stage)
    if not root or root.GetTypeName() != "Xform":
        return []
    xformable = UsdGeom.Xformable(root)
    times = sorted({sample for op in xformable.GetOrderedXformOps() for sample in op.GetTimeSamples()})
    return [(root, "default", xformable.GetLocalTransformation(Usd.TimeCode.Default()))] + [
        (root, f"time {sample:g}", xformable.GetLocalTransformation(Usd.TimeCode(sample))) for sample in times
    ]


@_RULES.rule("ASSET-ROOT-003")
def _default_prim(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if Sdf.Path.absoluteRootPath not in stage.GetPopulationMask().GetPaths():
        return
    default_name = stage.GetRootLayer().defaultPrim
    if not default_name:
        fail(checker, requirement, "The sensor asset must author a defaultPrim for whole-stage validation", stage)
    elif not stage.GetDefaultPrim():
        fail(checker, requirement, f"Authored defaultPrim {default_name!r} does not resolve", stage)


@_RULES.rule("ASSET-ROOT-004")
def _default_prim_type(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    root = _whole_stage_default_prim(stage)
    if root and root.GetTypeName() != "Xform":
        fail(checker, requirement, f"defaultPrim {root.GetPath()} must have typeName Xform", root)


@_RULES.rule("ASSET-ROOT-005")
def _default_prim_translation(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for root, time, matrix in _root_transform_samples(stage):
        if not Gf.IsClose(matrix.ExtractTranslation(), Gf.Vec3d(0), _ROOT_TRANSFORM_TOLERANCE):
            fail(checker, requirement, f"defaultPrim {root.GetPath()} translation at {time} must be (0, 0, 0)", root)
            break


@_RULES.rule("ASSET-ROOT-006")
def _default_prim_rotation(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for root, time, matrix in _root_transform_samples(stage):
        try:
            quaternion = Gf.Transform(matrix).GetRotation().GetQuaternion()
            valid = Gf.IsClose(quaternion.GetImaginary(), Gf.Vec3d(0), _ROOT_TRANSFORM_TOLERANCE) and math.isclose(
                abs(quaternion.GetReal()), 1, abs_tol=_ROOT_TRANSFORM_TOLERANCE
            )
        except (RuntimeError, ValueError):
            valid = False
        if not valid:
            fail(checker, requirement, f"defaultPrim {root.GetPath()} rotation at {time} must be identity", root)
            break


@_RULES.rule("ASSET-ROOT-007")
def _default_prim_scale(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for root, time, matrix in _root_transform_samples(stage):
        try:
            valid = Gf.IsClose(Gf.Transform(matrix).GetScale(), Gf.Vec3d(1), _ROOT_TRANSFORM_TOLERANCE)
        except (RuntimeError, ValueError):
            valid = False
        if not valid:
            fail(checker, requirement, f"defaultPrim {root.GetPath()} scale at {time} must be (1, 1, 1)", root)
            break


@_RULES.rule("ASSET-STRUCT-003")
def _render_product_presence(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    sensor_paths = {sensor.GetPath() for sensor in feature_sensor_prims(checker, stage)}
    if not render_products_targeting(stage, sensor_paths=sensor_paths):
        expected_type = feature_sensor_type(checker)
        description = repr(expected_type) if expected_type else f"one of {sorted(SENSOR_TYPE_NAMES)}"
        fail(
            checker,
            requirement,
            f"No RenderProduct targets any sensor prim of type {description} in the validated scope",
            stage,
        )
