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

"""Atomic authored-asset rules for the Kit 110.4 Generic Lidar Core model."""

from __future__ import annotations

import re
from collections.abc import Callable
from functools import partial
from typing import Any

import usd_validation_nvidia
from pxr import Usd

from .sensor_rule_utils import (
    AttributeNamespace,
    RuleRegistrar,
    applied_instances,
    as_sequence,
    fail,
    make_bound_check,
    native_render_vars,
    numeric_values_are_finite,
    render_products_targeting,
    render_var_channels,
    render_var_source_name,
    scoped_prims,
)

_CATEGORY = "IsaacSim.Sensor.Lidar"
_CORE_API = "OmniSensorGenericLidarCoreAPI"
_EMITTER_API = "OmniSensorGenericLidarCoreEmitterStateAPI"
_CORE_PREFIX = "omni:sensor:Core:"
_EMITTER_ROOT = f"{_CORE_PREFIX}emitterState:"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policies("LIDAR-TIMING-001")
_CORE = AttributeNamespace(_CORE_PREFIX)
_EMITTER = AttributeNamespace(_EMITTER_ROOT)

_NATIVE_SOURCE_NAMES = frozenset({"GenericModelOutput", "PointCloud", "RtxSensorMetadata"})
_CHANNELLED_SOURCE_NAMES = frozenset({"GenericModelOutput", "PointCloud"})
_GMO_CHANNELS = frozenset({"NONE", "BASIC", "EXTRA", "FULL"})
_POINT_CLOUD_CHANNELS = frozenset(
    {
        "Coordinates",
        "Intensity",
        "TimeOffsetNs",
        "Flags",
        "EmitterId",
        "ChannelId",
        "MaterialId",
        "TickId",
        "HitNormal",
        "Velocity",
        "ObjectId",
        "EchoId",
        "TickState",
    }
)

_REQUIRED_EMITTER_ARRAYS = ("azimuthDeg", "elevationDeg", "fireTimeNs", "channelId")
_OPTIONAL_EMITTER_ARRAYS = (
    "rangeId",
    "vertOffsetM",
    "horOffsetM",
    "distanceCorrectionM",
    "focalDistM",
    "focalSlope",
    "emitterPeakPowerW",
    "bank",
    "roi",
)
_FLOAT_EMITTER_ARRAYS = (
    "azimuthDeg",
    "elevationDeg",
    "vertOffsetM",
    "horOffsetM",
    "distanceCorrectionM",
    "focalDistM",
    "focalSlope",
    "emitterPeakPowerW",
)
_FLOAT_CORE_ATTRIBUTES = (
    "customFrameOfReferenceTrafo",
    "nearRangeM",
    "farRangeM",
    "reflectionPowerFraction",
    "transmissionPowerFraction",
    "startAzimuthOffsetDeg",
    "validStartAzimuthDeg",
    "validEndAzimuthDeg",
    "minAzimuthROI",
    "maxAzimuthROI",
    "rangeResolutionM",
    "rangeAccuracyM",
    "minDistBetweenEchosM",
    "rangeOffsetM",
    "peakPowerW",
    "minReflectance",
    "minReflectionRangeM",
    "waveLengthNm",
    "focusDistM",
    "Msquared",
    "beamWaistHorM",
    "beamWaistVerM",
    "divergenceHorDeg",
    "divergenceVerDeg",
    "aspectRatio",
    "effectiveApertureSizeM",
    "quantumEfficiency",
    "pixelPitch",
    "calibrationGain",
    "bitDepthResolution",
    "azimuthErrorMean",
    "azimuthErrorStd",
    "elevationErrorMean",
    "elevationErrorStd",
    "originErrorMean",
    "originErrorStd",
    "intensityScalePercent",
    "intensityMappingEncoding",
    "intensityMappingDecoding",
    "rangesMinM",
    "rangesMaxM",
)
_EMITTER_ATTRIBUTE_NAMES = frozenset((*_REQUIRED_EMITTER_ARRAYS, *_OPTIONAL_EMITTER_ARRAYS, "isROIState"))


def _lidars(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in scoped_prims(stage) if prim.GetTypeName() == "OmniLidar"]


@_RULES.rule("LIDAR-STRUCT-001")
def _lidar_presence(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if not _lidars(stage):
        fail(checker, requirement, "The validated asset subtree must contain an OmniLidar prim", stage)


def _lidar_products(stage: Usd.Stage) -> list[Usd.Prim]:
    return render_products_targeting(stage, sensor_type_names={"OmniLidar"})


def _native_render_vars(stage: Usd.Stage) -> list[tuple[Usd.Prim, Usd.Prim]]:
    return native_render_vars(_lidar_products(stage))


def _emitter_instances(lidar: Usd.Prim) -> list[str]:
    return applied_instances(lidar, _EMITTER_API)


def _all_emitter_instances(stage: Usd.Stage) -> list[tuple[Usd.Prim, str]]:
    return [(prim, instance) for prim in scoped_prims(stage) for instance in _emitter_instances(prim)]


@_RULES.rule("LIDAR-VAR-001")
def _source_name_allowlist(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for _product, render_var in _native_render_vars(stage):
        source_name = render_var_source_name(render_var)
        if source_name not in _NATIVE_SOURCE_NAMES:
            fail(
                checker,
                requirement,
                f"Native Lidar RenderVar {render_var.GetPath()} has unsupported sourceName {source_name!r}; "
                f"expected one of {sorted(_NATIVE_SOURCE_NAMES)}",
                render_var,
            )


@_RULES.rule("LIDAR-VAR-002")
def _channels_api_applicability(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for _product, render_var in _native_render_vars(stage):
        if (
            render_var.HasAPI("RenderVarChannelsAPI")
            and render_var_source_name(render_var) not in _CHANNELLED_SOURCE_NAMES
        ):
            fail(
                checker,
                requirement,
                f"RenderVarChannelsAPI on {render_var.GetPath()} requires sourceName in "
                f"{sorted(_CHANNELLED_SOURCE_NAMES)}; found {render_var_source_name(render_var)!r}",
                render_var,
            )


@_RULES.rule("LIDAR-VAR-004")
def _gmo_channels(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for _product, render_var in _native_render_vars(stage):
        if render_var_source_name(render_var) != "GenericModelOutput":
            continue
        channels = render_var_channels(render_var)
        if channels and (len(channels) != 1 or channels[0] not in _GMO_CHANNELS):
            fail(
                checker,
                requirement,
                f"GenericModelOutput RenderVar {render_var.GetPath()} may omit channels or specify exactly one "
                f"from {sorted(_GMO_CHANNELS)}; found {channels}",
                render_var,
            )


@_RULES.rule("LIDAR-VAR-005")
def _point_cloud_channels(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for _product, render_var in _native_render_vars(stage):
        if render_var_source_name(render_var) != "PointCloud":
            continue
        channels = render_var_channels(render_var)
        invalid = sorted(set(channels) - _POINT_CLOUD_CHANNELS)
        if len(channels) != len(set(channels)) or invalid:
            fail(
                checker,
                requirement,
                f"PointCloud RenderVar {render_var.GetPath()} requires unique channels from "
                f"{sorted(_POINT_CLOUD_CHANNELS)}; found {channels}",
                render_var,
            )


@_RULES.rule("LIDAR-VAR-007")
def _native_output_presence(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    products_with_native_output = {product.GetPath() for product, _render_var in _native_render_vars(stage)}
    for product in _lidar_products(stage):
        if product.GetPath() not in products_with_native_output:
            fail(
                checker,
                requirement,
                f"Lidar RenderProduct {product.GetPath()} must order at least one native RenderVar with sourceName "
                f"in {sorted(_NATIVE_SOURCE_NAMES)}",
                product,
            )


@_RULES.rule("LIDAR-SCHEMA-001")
def _core_api(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if not lidar.HasAPI(_CORE_API):
            fail(checker, requirement, f"Lidar {lidar.GetPath()} does not conform to {_CORE_API}", lidar)


@_RULES.rule("LIDAR-MODEL-001")
def _model_name(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        value = (
            lidar.GetAttribute("omni:sensor:modelName").Get() if lidar.GetAttribute("omni:sensor:modelName") else None
        )
        if value != "LidarCore":
            fail(checker, requirement, f"Lidar {lidar.GetPath()} modelName is {value!r}; expected 'LidarCore'", lidar)


@_RULES.rule("LIDAR-EMITTER-001")
def _required_emitter_state(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if not _emitter_instances(lidar):
            fail(checker, requirement, f"Lidar {lidar.GetPath()} has no composed emitter-state instance", lidar)


@_RULES.rule("LIDAR-EMITTER-002")
def _emitter_instance_names(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim, instance in _all_emitter_instances(stage):
        if re.fullmatch(r"s[0-9]{3}", instance) is None:
            fail(checker, requirement, f"Emitter-state instance {instance!r} must match s###", prim)


@_RULES.rule("LIDAR-EMITTER-003")
def _emitter_target_type(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim, instance in _all_emitter_instances(stage):
        if prim.GetTypeName() != "OmniLidar":
            fail(
                checker,
                requirement,
                f"Emitter-state instance {instance!r} is applied to {prim.GetTypeName() or 'untyped'} {prim.GetPath()}",
                prim,
            )


@_RULES.rule("LIDAR-EMITTER-004")
def _emitter_core_dependency(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for prim, instance in _all_emitter_instances(stage):
        if not prim.HasAPI(_CORE_API):
            fail(
                checker,
                requirement,
                f"Emitter-state instance {instance!r} on {prim.GetPath()} lacks {_CORE_API}",
                prim,
            )


@_RULES.rule("LIDAR-EMITTER-005")
def _emitter_namespace(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim, instance in _all_emitter_instances(stage):
        expected = f"{_EMITTER_ROOT}{instance}:"
        for attribute in prim.GetAttributes():
            name = attribute.GetName()
            if name.startswith(_EMITTER_ROOT) and f":{instance}:" in name and not name.startswith(expected):
                fail(
                    checker,
                    requirement,
                    f"Emitter-state attribute {attribute.GetPath()} is outside namespace {expected}",
                    attribute,
                )


@_RULES.rule("LIDAR-ARRAY-001")
def _required_emitter_arrays(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for lidar in _lidars(stage):
        count = int(_CORE.value(lidar, "numberOfEmitters") or 0)
        for instance in _emitter_instances(lidar):
            for name in _REQUIRED_EMITTER_ARRAYS:
                values = as_sequence(_EMITTER.value(lidar, name, instance))
                if len(values) != count:
                    fail(
                        checker,
                        requirement,
                        f"Emitter state {instance!r} {name} has {len(values)} elements; expected {count}",
                        _EMITTER.attribute(lidar, name, instance) or lidar,
                    )


@_RULES.rule("LIDAR-ARRAY-002")
def _optional_emitter_arrays(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for lidar in _lidars(stage):
        count = int(_CORE.value(lidar, "numberOfEmitters") or 0)
        for instance in _emitter_instances(lidar):
            for name in _OPTIONAL_EMITTER_ARRAYS:
                values = as_sequence(_EMITTER.value(lidar, name, instance))
                if values and len(values) != count:
                    fail(
                        checker,
                        requirement,
                        f"Emitter state {instance!r} {name} has {len(values)} elements; expected zero or {count}",
                        _EMITTER.attribute(lidar, name, instance) or lidar,
                    )


@_RULES.rule("LIDAR-ARRAY-003")
def _channel_id_range(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        count = int(_CORE.value(lidar, "numberOfChannels") or 0)
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "channelId", instance)
            invalid = [
                value for value in as_sequence(attribute.Get() if attribute else None) if value < 1 or value > count
            ]
            if invalid:
                fail(
                    checker,
                    requirement,
                    f"Emitter state {instance!r} channelId contains values outside [1, {count}]: {invalid}",
                    attribute or lidar,
                )


def _emitter_nonnegative(name: str) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
        for lidar in _lidars(stage):
            for instance in _emitter_instances(lidar):
                attribute = _EMITTER.attribute(lidar, name, instance)
                invalid = [value for value in as_sequence(attribute.Get() if attribute else None) if value < 0]
                if invalid:
                    fail(
                        checker,
                        requirement,
                        f"Emitter state {instance!r} {name} contains negative values: {invalid}",
                        attribute or lidar,
                    )

    return check


@_RULES.rule("LIDAR-PARAM-001")
def _finite_parameters(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if lidar.HasAPI(_CORE_API):
            tick_rate = lidar.GetAttribute("omni:sensor:tickRate")
            if tick_rate and not numeric_values_are_finite(tick_rate.Get(), missing=True):
                fail(checker, requirement, f"Lidar {lidar.GetPath()} tickRate contains a non-finite value", tick_rate)
            for name in _FLOAT_CORE_ATTRIBUTES:
                attribute = _CORE.attribute(lidar, name)
                if attribute and not numeric_values_are_finite(attribute.Get(), missing=True):
                    fail(checker, requirement, f"Lidar {lidar.GetPath()} {name} contains a non-finite value", attribute)
        for instance in _emitter_instances(lidar):
            for name in _FLOAT_EMITTER_ARRAYS:
                attribute = _EMITTER.attribute(lidar, name, instance)
                if attribute and not numeric_values_are_finite(attribute.Get(), missing=True):
                    fail(
                        checker,
                        requirement,
                        f"Emitter state {instance!r} {name} contains a non-finite value",
                        attribute,
                    )


_core_bound = partial(
    make_bound_check,
    _lidars,
    _CORE,
    message=lambda lidar, name, value, description: f"Lidar {lidar.GetPath()} {name}={value!r} {description}",
)


@_RULES.rule("LIDAR-PARAM-008")
def _sensing_range(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        near = _CORE.value(lidar, "nearRangeM")
        far = _CORE.value(lidar, "farRangeM")
        if near is None or far is None or not (0 <= near < far):
            fail(checker, requirement, f"Lidar {lidar.GetPath()} requires 0 <= nearRangeM < farRangeM", lidar)


@_RULES.rule("LIDAR-PARAM-035")
def _beam_representation(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        waist = (_CORE.value(lidar, "beamWaistHorM"), _CORE.value(lidar, "beamWaistVerM"))
        divergence = (_CORE.value(lidar, "divergenceHorDeg"), _CORE.value(lidar, "divergenceVerDeg"))
        waist_form = all(value is not None and value > 0 for value in waist)
        divergence_form = all(value is not None and value > 0 for value in divergence)
        waist_only = waist_form and all(value == 0 for value in divergence)
        divergence_only = divergence_form and all(value == 0 for value in waist)
        if not (waist_only or divergence_only):
            fail(
                checker,
                requirement,
                f"Lidar {lidar.GetPath()} must provide exactly one complete waist or divergence beam form",
                lidar,
            )


def _core_array_size(name: str, size: int) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
        for lidar in _lidars(stage):
            values = as_sequence(_CORE.value(lidar, name))
            if len(values) != size:
                fail(
                    checker,
                    requirement,
                    f"Lidar {lidar.GetPath()} {name} has {len(values)} elements; expected {size}",
                    _CORE.attribute(lidar, name) or lidar,
                )

    return check


def _range_cardinality(
    name: str,
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
        for lidar in _lidars(stage):
            count = int(_CORE.value(lidar, "rangeCount") or 0)
            values = as_sequence(_CORE.value(lidar, name))
            if len(values) != count:
                fail(
                    checker,
                    requirement,
                    f"Lidar {lidar.GetPath()} {name} has {len(values)} elements; expected {count}",
                    _CORE.attribute(lidar, name) or lidar,
                )

    return check


@_RULES.rule("LIDAR-RANGE-005")
def _range_pair_ordering(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        minimums = as_sequence(_CORE.value(lidar, "rangesMinM"))
        maximums = as_sequence(_CORE.value(lidar, "rangesMaxM"))
        for index, (minimum, maximum) in enumerate(zip(minimums, maximums)):
            if minimum > maximum:
                fail(
                    checker,
                    requirement,
                    f"Lidar {lidar.GetPath()} range {index} has minimum {minimum} greater than maximum {maximum}",
                    lidar,
                )


@_RULES.rule("LIDAR-RANGE-006")
def _range_id_cardinality(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        emitter_count = int(_CORE.value(lidar, "numberOfEmitters") or 0)
        for instance in _emitter_instances(lidar):
            values = as_sequence(_EMITTER.value(lidar, "rangeId", instance))
            if values and len(values) != emitter_count:
                fail(
                    checker,
                    requirement,
                    f"Emitter state {instance!r} rangeId has {len(values)} elements; expected zero or {emitter_count}",
                    _EMITTER.attribute(lidar, "rangeId", instance) or lidar,
                )


@_RULES.rule("LIDAR-RANGE-007")
def _range_id_domain(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        upper = max(1, int(_CORE.value(lidar, "rangeCount") or 0))
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "rangeId", instance)
            invalid = [
                value for value in as_sequence(attribute.Get() if attribute else None) if value < 0 or value >= upper
            ]
            if invalid:
                fail(
                    checker,
                    requirement,
                    f"Emitter state {instance!r} rangeId contains values outside [0, {upper}): {invalid}",
                    attribute or lidar,
                )


@_RULES.rule("LIDAR-RANGE-008")
def _multi_range_ids(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if int(_CORE.value(lidar, "rangeCount") or 0) <= 1:
            continue
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "rangeId", instance)
            if not as_sequence(attribute.Get() if attribute else None):
                fail(
                    checker,
                    requirement,
                    f"Emitter state {instance!r} requires rangeId for multi-range Lidar",
                    attribute or lidar,
                )


@_RULES.rule("LIDAR-FIRE-001")
def _fire_times_sorted(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "fireTimeNs", instance)
            values = as_sequence(attribute.Get() if attribute else None)
            if any(current > following for current, following in zip(values, values[1:])):
                fail(checker, requirement, f"Emitter state {instance!r} fireTimeNs is not nondecreasing", attribute)


@_RULES.rule("LIDAR-FIRE-002")
def _fire_time_bound(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        rate = int(_CORE.value(lidar, "patternFiringRateHz") or 0)
        if rate <= 0:
            continue
        bound = 1_000_000_000 // rate
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "fireTimeNs", instance)
            values = as_sequence(attribute.Get() if attribute else None)
            if values and values[-1] > bound:
                fail(
                    checker,
                    requirement,
                    f"Emitter state {instance!r} final fireTimeNs {values[-1]} exceeds tick bound {bound}",
                    attribute,
                )


@_RULES.rule("LIDAR-INTENSITY-001")
def _intensity_mapping_token(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    allowed = {"LINEAR", "NONLINEAR", "NONLINEAR_ENCODING_ONLY", "NONLINEAR_DECODING_ONLY"}
    for lidar in _lidars(stage):
        value = _CORE.value(lidar, "intensityMappingType")
        if value not in allowed:
            fail(
                checker,
                requirement,
                f"Lidar {lidar.GetPath()} intensityMappingType {value!r} is unsupported; "
                f"expected one of {sorted(allowed)}",
                _CORE.attribute(lidar, "intensityMappingType") or lidar,
            )


@_RULES.rule("LIDAR-COND-001")
def _custom_transform_size(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "outputFrameOfReference") == "CUSTOM":
            values = as_sequence(_CORE.value(lidar, "customFrameOfReferenceTrafo"))
            if len(values) != 6:
                fail(checker, requirement, f"Lidar {lidar.GetPath()} custom transform must have six elements", lidar)


@_RULES.rule("LIDAR-COND-003")
def _solid_num_lines(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "scanType") == "SOLID_STATE" and int(_CORE.value(lidar, "numLines") or 0) <= 0:
            fail(checker, requirement, f"Solid-state Lidar {lidar.GetPath()} numLines must be positive", lidar)


@_RULES.rule("LIDAR-COND-004")
def _solid_ray_cardinality(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "scanType") != "SOLID_STATE":
            continue
        lines = int(_CORE.value(lidar, "numLines") or 0)
        values = as_sequence(_CORE.value(lidar, "numRaysPerLine"))
        if len(values) != lines:
            fail(
                checker,
                requirement,
                f"Solid-state Lidar {lidar.GetPath()} has {len(values)} ray counts; expected {lines}",
                lidar,
            )


@_RULES.rule("LIDAR-COND-005")
def _solid_ray_storage(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "scanType") == "SOLID_STATE":
            attribute = _CORE.attribute(lidar, "numRaysPerLine")
            invalid = [
                value for value in as_sequence(attribute.Get() if attribute else None) if value < 0 or value > 65535
            ]
            if invalid:
                fail(checker, requirement, f"Solid-state ray counts must lie in [0, 65535]; found {invalid}", attribute)


@_RULES.rule("LIDAR-COND-006")
def _solid_ray_sum(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "scanType") != "SOLID_STATE":
            continue
        values = as_sequence(_CORE.value(lidar, "numRaysPerLine"))
        emitters = int(_CORE.value(lidar, "numberOfEmitters") or 0)
        if sum(values) != emitters:
            fail(
                checker,
                requirement,
                f"Solid-state Lidar {lidar.GetPath()} ray-count sum {sum(values)} does not equal {emitters}",
                lidar,
            )


@_RULES.rule("LIDAR-COND-009")
def _solid_bank_indices(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _CORE.value(lidar, "scanType") != "SOLID_STATE":
            continue
        lines = int(_CORE.value(lidar, "numLines") or 0)
        for instance in _emitter_instances(lidar):
            attribute = _EMITTER.attribute(lidar, "bank", instance)
            for value in as_sequence(attribute.Get() if attribute else None):
                if value >= lines:
                    fail(
                        checker,
                        requirement,
                        f"Emitter state {instance!r} bank {value} is not less than numLines {lines}",
                        attribute,
                    )


def _active_roi_states(lidar: Usd.Prim) -> list[str]:
    if _CORE.value(lidar, "scanType") != "ROTARY":
        return []
    return [instance for instance in _emitter_instances(lidar) if bool(_EMITTER.value(lidar, "isROIState", instance))]


@_RULES.rule("LIDAR-COND-010")
def _active_roi_interval(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        if _active_roi_states(lidar) and float(_CORE.value(lidar, "minAzimuthROI") or 0) > 360:
            fail(checker, requirement, f"Active rotary ROI on {lidar.GetPath()} requires minAzimuthROI <= 360", lidar)


@_RULES.rule("LIDAR-COND-011")
def _active_roi_cardinality(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        emitters = int(_CORE.value(lidar, "numberOfEmitters") or 0)
        for instance in _active_roi_states(lidar):
            attribute = _EMITTER.attribute(lidar, "roi", instance)
            values = as_sequence(attribute.Get() if attribute else None)
            if len(values) != emitters:
                fail(
                    checker,
                    requirement,
                    f"Active ROI state {instance!r} has {len(values)} roi elements; expected {emitters}",
                    attribute or lidar,
                )


@_RULES.rule("LIDAR-COND-012")
def _active_roi_membership(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for lidar in _lidars(stage):
        for instance in _active_roi_states(lidar):
            attribute = _EMITTER.attribute(lidar, "roi", instance)
            if not any(as_sequence(attribute.Get() if attribute else None)):
                fail(
                    checker,
                    requirement,
                    f"Active ROI state {instance!r} contains no enabled emitter",
                    attribute or lidar,
                )


_RULES.check("LIDAR-ARRAY-004", _emitter_nonnegative("focalDistM"))
_RULES.check("LIDAR-ARRAY-005", _emitter_nonnegative("emitterPeakPowerW"))


_POSITIVE_PARAMETERS = {
    "LIDAR-PARAM-002": "scanRateBaseHz",
    "LIDAR-PARAM-003": "patternFiringRateHz",
    "LIDAR-PARAM-004": "numberOfEmitters",
    "LIDAR-PARAM-005": "numberOfChannels",
    "LIDAR-PARAM-006": "maxReturns",
    "LIDAR-PARAM-007": "stateResolutionStep",
    "LIDAR-PARAM-024": "rangeResolutionM",
    "LIDAR-PARAM-025": "focusDistM",
    "LIDAR-PARAM-026": "peakPowerW",
    "LIDAR-PARAM-027": "waveLengthNm",
    "LIDAR-PARAM-028": "pulseTimeNs",
    "LIDAR-PARAM-029": "aspectRatio",
    "LIDAR-PARAM-030": "effectiveApertureSizeM",
    "LIDAR-PARAM-031": "pixelPitch",
    "LIDAR-PARAM-032": "calibrationGain",
    "LIDAR-PARAM-033": "bitDepthResolution",
}
for _requirement_id, _attribute_name in _POSITIVE_PARAMETERS.items():
    _RULES.check(
        _requirement_id,
        _core_bound(_attribute_name, lambda value: value > 0, "must be greater than zero"),
    )


_NONNEGATIVE_PARAMETERS = {
    "LIDAR-PARAM-009": "azimuthErrorStd",
    "LIDAR-PARAM-010": "elevationErrorStd",
    "LIDAR-PARAM-015": "quantumEfficiency",
    "LIDAR-PARAM-016": "rangeAccuracyM",
    "LIDAR-PARAM-017": "minDistBetweenEchosM",
    "LIDAR-PARAM-018": "rangeOffsetM",
    "LIDAR-PARAM-020": "beamWaistHorM",
    "LIDAR-PARAM-021": "beamWaistVerM",
    "LIDAR-PARAM-022": "divergenceHorDeg",
    "LIDAR-PARAM-023": "divergenceVerDeg",
}
for _requirement_id, _attribute_name in _NONNEGATIVE_PARAMETERS.items():
    _RULES.check(
        _requirement_id,
        _core_bound(_attribute_name, lambda value: value >= 0, "must be nonnegative"),
    )

for _requirement_id, _attribute_name in (
    ("LIDAR-PARAM-014", "minReflectance"),
    ("LIDAR-PARAM-019", "minReflectionRangeM"),
):
    _RULES.check(_requirement_id, _core_bound(_attribute_name, lambda value: value > 0, "must be positive"))

_RULES.check(
    "LIDAR-PARAM-011",
    _core_bound(
        "originErrorStd", lambda values: all(value >= 0 for value in values), "must be componentwise nonnegative"
    ),
)
_RULES.check(
    "LIDAR-PARAM-012",
    _core_bound("reflectionPowerFraction", lambda value: value >= 0, "must be nonnegative"),
)
_RULES.check(
    "LIDAR-PARAM-013",
    _core_bound("transmissionPowerFraction", lambda value: value >= 0, "must be nonnegative"),
)
_RULES.check(
    "LIDAR-PARAM-034",
    _core_bound("Msquared", lambda value: value >= 1, "must be at least one"),
)
_RULES.check("LIDAR-PARAM-036", _core_array_size("originErrorMean", 3))
_RULES.check("LIDAR-PARAM-037", _core_array_size("originErrorStd", 3))
_RULES.check("LIDAR-RANGE-001", _range_cardinality("rangesMinM"))
_RULES.check("LIDAR-RANGE-002", _range_cardinality("rangesMaxM"))
_RULES.check(
    "LIDAR-INTENSITY-002",
    _core_bound("intensityScalePercent", lambda value: value > 0, "must be greater than zero"),
)
del _attribute_name, _requirement_id
