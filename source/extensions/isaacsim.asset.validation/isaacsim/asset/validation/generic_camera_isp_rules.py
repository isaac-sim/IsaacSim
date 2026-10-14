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

"""Generic Camera Core ISP schema and pipeline rules."""

from __future__ import annotations

import base64
import binascii
import math
from functools import partial
from pathlib import Path
from typing import Any

import usd_validation_nvidia
from pxr import Usd, UsdRender

from .camera_rule_utils import _ISP_OUTPUTS, _cameras, _source_names
from .sensor_rule_utils import (
    AttributeNamespace,
    RuleRegistrar,
    attribute_value,
    fail,
    is_finite,
    make_bound_check,
    render_products_targeting,
    warn,
)

_RULES = RuleRegistrar("IsaacSim.Sensor.Camera.ISP")
_CAMERA_ISP_API = "OmniSensorGenericCameraCoreAPI"
_ISP_PREFIX = "omni:sensor:core:"
_ISP = AttributeNamespace(_ISP_PREFIX)
_MAINTAINED_ISP_OUTPUTS = frozenset({"OmniCameraSensorPreIsp", "OmniCameraSensorPostIsp"})


def _isp_cameras(stage: Usd.Stage) -> list[Usd.Prim]:
    return [camera for camera in _cameras(stage) if camera.HasAPI(_CAMERA_ISP_API)]


_ISP_BOUND = partial(make_bound_check, _isp_cameras, _ISP)


def _isp_products(stage: Usd.Stage) -> list[Usd.Prim]:
    return [
        product
        for product in render_products_targeting(
            stage, sensor_paths={camera.GetPath() for camera in _isp_cameras(stage)}
        )
        if set(_source_names(product)) & (_ISP_OUTPUTS | {"HdrColor"})
    ]


@_RULES.rule("CAMERA-ISP-001")
def _isp_schema(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if not _isp_cameras(stage):
        fail(checker, requirement, "No Camera has OmniSensorGenericCameraCoreAPI applied", stage)


@_RULES.rule("CAMERA-ISP-002")
def _isp_identity(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    expected = {
        "omni:sensor:modelName": "CameraCore",
        "omni:sensor:modelVendor": "NVIDIA",
        "omni:sensor:marketName": "Generic",
    }
    for camera in _isp_cameras(stage):
        actual = {name: str(attribute_value(camera, name) or "") for name in expected}
        if actual != expected:
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} dispatch identity is {actual}, expected {expected}",
                camera,
            )


@_RULES.rule("CAMERA-ISP-003")
def _isp_one_camera(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    isp_paths = {camera.GetPath() for camera in _isp_cameras(stage)}
    for product in _isp_products(stage):
        relationship = UsdRender.Product(product).GetCameraRel()
        count = sum(path in isp_paths for path in relationship.GetTargets())
        if count != 1 or len(relationship.GetTargets()) != 1:
            fail(
                checker,
                requirement,
                f"ISP RenderProduct {product.GetPath()} must resolve exactly one Camera",
                relationship,
            )


def _isp_output_count(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    source_name: str | None,
) -> None:
    products = _isp_products(stage)
    if source_name is None:
        covered = {path for product in products for path in UsdRender.Product(product).GetCameraRel().GetTargets()}
        for camera in _isp_cameras(stage):
            if camera.GetPath() not in covered:
                fail(
                    checker,
                    requirement,
                    f"Camera {camera.GetPath()} has no ISP RenderProduct with a maintained output; "
                    f"expected a targeting RenderProduct that orders sourceName in {sorted(_MAINTAINED_ISP_OUTPUTS)}",
                    camera,
                )
    for product in products:
        names = _source_names(product)
        count = (
            sum(name in _MAINTAINED_ISP_OUTPUTS for name in names) if source_name is None else names.count(source_name)
        )
        if count < 1 if source_name is None else count != 1:
            description = (
                f"at least one maintained ISP output RenderVar with sourceName in {sorted(_MAINTAINED_ISP_OUTPUTS)}"
                if source_name is None
                else f"exactly one RenderVar with sourceName {source_name!r}"
            )
            fail(
                checker,
                requirement,
                f"ISP RenderProduct {product.GetPath()} must order {description}; found sourceNames {names}",
                UsdRender.Product(product).GetOrderedVarsRel(),
            )


@_RULES.rule("CAMERA-ISP-006")
def _isp_finite(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _isp_cameras(stage):
        for attribute in camera.GetAttributes():
            if attribute.GetName().startswith(_ISP_PREFIX) and not is_finite(attribute.Get()):
                fail(checker, requirement, f"{attribute.GetPath()} contains a non-finite value", attribute)


def _isp_array_length(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    valid_length: Any,
    description: str,
) -> None:
    name = f"{_ISP_PREFIX}{short_name}"
    for camera in _isp_cameras(stage):
        attribute = camera.GetAttribute(name)
        value = attribute.Get() if attribute else None
        if value is None or not valid_length(len(value)):
            fail(checker, requirement, f"{name} on {camera.GetPath()} must contain {description}", attribute or camera)


@_RULES.rule("CAMERA-ISP-013")
def _both_float_outputs(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _isp_cameras(stage):
        float16 = attribute_value(camera, f"{_ISP_PREFIX}colorCorrectionOutputFloat16")
        float32 = attribute_value(camera, f"{_ISP_PREFIX}colorCorrectionOutputFloat32")
        if float16 and float32:
            warn(
                checker,
                requirement,
                f"Camera {camera.GetPath()} enables float16 and float32 color-correction outputs; Kit 110.4 selects float32",
                camera,
            )


def _isp_token(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    allowed: set[str],
    *,
    condition: Any = None,
) -> None:
    _ISP_BOUND(short_name, lambda value: str(value) in allowed, f"one of {sorted(allowed)}", condition=condition)(
        checker, stage, requirement
    )


def _pair_component(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    component: int,
    predicate: Any,
    description: str,
) -> None:
    name = f"{_ISP_PREFIX}{short_name}"
    for camera in _isp_cameras(stage):
        attribute = camera.GetAttribute(name)
        values = attribute.Get() if attribute else None
        if values is None or any(not predicate(pair[component]) for pair in values):
            fail(
                checker,
                requirement,
                f"Component {component} of every {name} pair must be {description}",
                attribute or camera,
            )


def _strictly_ordered_pairs(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    component: int,
    *,
    strict: bool,
) -> None:
    name = f"{_ISP_PREFIX}{short_name}"
    for camera in _isp_cameras(stage):
        attribute = camera.GetAttribute(name)
        values = attribute.Get() if attribute else None
        if values is None:
            fail(checker, requirement, f"{name} does not resolve", attribute or camera)
            continue
        sequence = [pair[component] for pair in values]
        compare = (lambda left, right: left < right) if strict else (lambda left, right: left <= right)
        if any(not compare(left, right) for left, right in zip(sequence, sequence[1:])):
            qualifier = "strictly increasing" if strict else "nondecreasing"
            fail(checker, requirement, f"Component {component} of {name} must be {qualifier}", attribute)


def _companding_alignment(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    wide: bool,
) -> None:
    coefficients_name = f"{_ISP_PREFIX}compandingLinearCompandCoeff"
    alignment_name = f"{_ISP_PREFIX}compandingAlignment"
    for camera in _isp_cameras(stage):
        coefficients = attribute_value(camera, coefficients_name) or []
        alignment = attribute_value(camera, alignment_name)
        if not coefficients or alignment is None:
            continue
        maximum = max(pair[1] for pair in coefficients)
        is_wide = maximum > 65535
        if is_wide != wide:
            continue
        if wide:
            valid = alignment <= 31
            description = "at most 31 for a 32-bit companding output"
        else:
            output_bits = max(1, math.ceil(math.log2(maximum + 1))) if maximum >= 0 else 0
            valid = output_bits - 1 <= alignment <= 15
            description = f"in [{output_bits - 1}, 15] for the {output_bits}-bit companding output"
        if not valid:
            fail(
                checker,
                requirement,
                f"{alignment_name} on {camera.GetPath()} must be {description}",
                camera.GetAttribute(alignment_name),
            )


@_RULES.rule("CAMERA-ISP-034")
def _base64_program(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    name = f"{_ISP_PREFIX}ispSmodelCameraProgram"
    for camera in _isp_cameras(stage):
        attribute = camera.GetAttribute(name)
        value = attribute.Get() if attribute else None
        try:
            decoded = base64.b64decode(value or "", validate=True)
        except (binascii.Error, ValueError):
            decoded = b""
        if not decoded:
            fail(
                checker,
                requirement,
                f"{name} on {camera.GetPath()} must contain nonempty valid Base64",
                attribute or camera,
            )


def _root_relative_path(stage: Usd.Stage, value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    layer_path = stage.GetRootLayer().realPath
    return Path(layer_path).parent / path if layer_path else path


@_RULES.rule("CAMERA-ISP-037")
def _raw_path(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    name = f"{_ISP_PREFIX}introspectionInputRawPath"
    for camera in _isp_cameras(stage):
        if not attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRaw"):
            continue
        attribute = camera.GetAttribute(name)
        path = _root_relative_path(stage, str(attribute.Get() or "")) if attribute else Path()
        if not path.is_file():
            fail(
                checker,
                requirement,
                f"{name} on {camera.GetPath()} does not resolve to a readable file",
                attribute or camera,
            )


@_RULES.rule("CAMERA-ISP-043")
def _raw_capacity(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    bytes_per_value = {"UINT8": 1, "UINT16": 2, "UINT32": 4, "FLOAT16": 2, "FLOAT32": 4}
    for camera in _isp_cameras(stage):
        if not attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRaw"):
            continue
        path_value = str(attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawPath") or "")
        path = _root_relative_path(stage, path_value)
        if not path.is_file():
            continue
        width = attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawWidth") or 0
        height = attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawHeight") or 0
        front = attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawSkipFront") or 0
        back = attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawSkipBack") or 0
        data_type = str(attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRawFileDataType") or "")
        required = front + width * height * bytes_per_value.get(data_type, 0) + back
        if path.stat().st_size < required:
            fail(
                checker,
                requirement,
                f"RAW input {path} contains {path.stat().st_size} bytes; configured frame requires at least {required}",
                camera.GetAttribute(f"{_ISP_PREFIX}introspectionInputRawPath"),
            )


_RULES.check(
    "CAMERA-ISP-004",
    lambda checker, stage, requirement: _isp_output_count(checker, stage, requirement, "HdrColor"),
)
_RULES.check(
    "CAMERA-ISP-005",
    lambda checker, stage, requirement: _isp_output_count(checker, stage, requirement, None),
)
for requirement_id, name in (
    ("CAMERA-ISP-007", "colorCorrectionWhiteBalance"),
    ("CAMERA-ISP-008", "colorFilterArrayCfaCf00"),
    ("CAMERA-ISP-009", "colorFilterArrayCfaCf01"),
    ("CAMERA-ISP-010", "colorFilterArrayCfaCf10"),
    ("CAMERA-ISP-011", "colorFilterArrayCfaCf11"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, name=name: _isp_array_length(
            checker, stage, requirement, name, lambda length: length == 3, "exactly three elements"
        ),
    )
_RULES.check(
    "CAMERA-ISP-012",
    _ISP_BOUND(
        "colorCorrectionFullwellBlack",
        lambda value: value > 0,
        "greater than zero when saturation is enabled",
        condition=lambda camera: bool(attribute_value(camera, f"{_ISP_PREFIX}colorCorrectionApplySaturation")),
    ),
)
_RULES.check(
    "CAMERA-ISP-014",
    lambda checker, stage, requirement: _isp_token(
        checker,
        stage,
        requirement,
        "colorFilterArrayCfaSemantic",
        {"RGGB", "GRBG", "BGGR", "RCCB", "CRBC", "BCCR", "RCCC", "CCCR", "CRCC"},
    ),
)
_RULES.check(
    "CAMERA-ISP-015",
    lambda checker, stage, requirement: _isp_token(
        checker, stage, requirement, "colorFilterArrayOutputDataTypeFormat", {"FLOAT32", "UINT32"}
    ),
)
_RULES.check(
    "CAMERA-ISP-016",
    _ISP_BOUND("colorFilterArrayMaximalValue", lambda value: value > 0, "greater than zero"),
)
_RULES.check(
    "CAMERA-ISP-018",
    lambda checker, stage, requirement: _isp_array_length(
        checker, stage, requirement, "noiseHdrCombinationData", lambda length: length >= 1, "at least one pair"
    ),
)
for requirement_id, component, predicate, description in (
    ("CAMERA-ISP-019", 0, lambda value: value > 0, "greater than zero"),
    ("CAMERA-ISP-020", 1, lambda value: value >= 0, "nonnegative"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, component=component, predicate=predicate, description=description: _pair_component(
            checker,
            stage,
            requirement,
            "noiseHdrCombinationData",
            component,
            predicate,
            description,
        ),
    )
_RULES.check(
    "CAMERA-ISP-021",
    lambda checker, stage, requirement: _strictly_ordered_pairs(
        checker, stage, requirement, "noiseHdrCombinationData", 1, strict=True
    ),
)
for requirement_id, name in (
    ("CAMERA-ISP-022", "noiseDarkShotNoiseGain"),
    ("CAMERA-ISP-023", "noiseDarkShotNoiseSigma"),
    ("CAMERA-ISP-030", "compandingPrePedestal"),
    ("CAMERA-ISP-031", "compandingPostPedestal"),
):
    _RULES.check(requirement_id, _ISP_BOUND(name, lambda value: value >= 0, "nonnegative"))
_RULES.check(
    "CAMERA-ISP-024",
    lambda checker, stage, requirement: _isp_array_length(
        checker,
        stage,
        requirement,
        "compandingLinearCompandCoeff",
        lambda length: 2 <= length <= 255,
        "2 through 255 pairs",
    ),
)
_RULES.check(
    "CAMERA-ISP-025",
    _ISP_BOUND("compandingLinearCompandCoeff", is_finite, "finite"),
)
_RULES.check(
    "CAMERA-ISP-026",
    lambda checker, stage, requirement: _strictly_ordered_pairs(
        checker, stage, requirement, "compandingLinearCompandCoeff", 0, strict=True
    ),
)
_RULES.check(
    "CAMERA-ISP-027",
    lambda checker, stage, requirement: _pair_component(
        checker, stage, requirement, "compandingLinearCompandCoeff", 1, lambda value: value >= 0, "nonnegative"
    ),
)
_RULES.check(
    "CAMERA-ISP-028",
    lambda checker, stage, requirement: _strictly_ordered_pairs(
        checker, stage, requirement, "compandingLinearCompandCoeff", 1, strict=False
    ),
)
_RULES.check(
    "CAMERA-ISP-029",
    _ISP_BOUND(
        "compandingLinearCompandCoeff",
        lambda values: bool(values) and values[-1][1] > 0,
        "a curve whose final output is greater than zero",
    ),
)
_RULES.check(
    "CAMERA-ISP-032",
    lambda checker, stage, requirement: _companding_alignment(checker, stage, requirement, wide=False),
)
_RULES.check(
    "CAMERA-ISP-033",
    lambda checker, stage, requirement: _companding_alignment(checker, stage, requirement, wide=True),
)
_RULES.check(
    "CAMERA-ISP-035",
    lambda checker, stage, requirement: _isp_token(
        checker, stage, requirement, "ispSmodelRawDataPaddingMode", {"NONE", "RIGHT", "LEFT"}
    ),
)
_RULES.check(
    "CAMERA-ISP-036",
    lambda checker, stage, requirement: _isp_token(
        checker, stage, requirement, "ispSmodelSensorRawFormat", {"RAW8", "RAW10", "RAW12", "RAW14", "RAW20", "RAW24"}
    ),
)
for requirement_id, name, predicate, description in (
    ("CAMERA-ISP-038", "introspectionInputRawWidth", lambda value: value > 0, "greater than zero"),
    ("CAMERA-ISP-039", "introspectionInputRawHeight", lambda value: value > 0, "greater than zero"),
    ("CAMERA-ISP-040", "introspectionInputRawSkipFront", lambda value: value >= 0, "nonnegative"),
    ("CAMERA-ISP-041", "introspectionInputRawSkipBack", lambda value: value >= 0, "nonnegative"),
):
    _RULES.check(
        requirement_id,
        _ISP_BOUND(
            name,
            predicate,
            description,
            condition=lambda camera: bool(attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRaw")),
        ),
    )
_RULES.check(
    "CAMERA-ISP-042",
    lambda checker, stage, requirement: _isp_token(
        checker,
        stage,
        requirement,
        "introspectionInputRawFileDataType",
        {"UINT8", "UINT16", "UINT32", "FLOAT16", "FLOAT32"},
        condition=lambda camera: bool(attribute_value(camera, f"{_ISP_PREFIX}introspectionInputRaw")),
    ),
)
_RULES.check(
    "CAMERA-ISP-044",
    _ISP_BOUND(
        "introspectionOutputFileDirectory",
        lambda value: bool(str(value).strip()),
        "nonempty when file output is enabled",
        condition=lambda camera: bool(attribute_value(camera, f"{_ISP_PREFIX}introspectionOutputFile")),
    ),
)
