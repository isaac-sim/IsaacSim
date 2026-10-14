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

"""Atomic authored-asset rules for the Kit 110.4 Radar WPM-DMAT model."""

from __future__ import annotations

import math
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
    warn,
)

_CATEGORY = "IsaacSim.Sensor.Radar"
_CORE_API = "OmniSensorGenericRadarWpmDmatAPI"
_SCAN_API = "OmniSensorGenericRadarWpmDmatScanCfgAPI"
_CORE_PREFIX = "omni:sensor:WpmDmat:"
_SCAN_ROOT = f"{_CORE_PREFIX}scan:"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policies(
    "RADAR-SCHEMA-010",
    "RADAR-OUTPUT-004",
    "RADAR-OUTPUT-007",
    "RADAR-OUTPUT-009",
    "RADAR-OUTPUT-010",
    "RADAR-PARAM-006",
    "RADAR-PARAM-095",
)
_CORE = AttributeNamespace(_CORE_PREFIX)
_SCAN = AttributeNamespace(_SCAN_ROOT)

_NATIVE_SOURCE_NAMES = frozenset({"GenericModelOutput", "PointCloud", "RtxSensorMetadata"})
_CHANNELLED_SOURCE_NAMES = frozenset({"GenericModelOutput", "PointCloud"})
_GMO_CHANNELS = frozenset({"NONE", "BASIC"})
_POINT_CLOUD_CHANNELS = frozenset({"Coordinates", "RCS", "TimeOffsetNs", "RadialVelocityMs", "Flags"})
_WINDOWS = frozenset({"RECT", "HANN", "HAMMING", "BLACKMAN"})
_ANTENNA_MODES = frozenset({"CONSTANT", "COSINE_FALLOFF", "CUSTOM", "PATCH_ARRAY_MODEL"})
_UINT32_MAX = 2**32 - 1
_INT32_MAX = 2**31 - 1
_C_LIGHT = 299_792_458.0

_SCAN_FLOAT_ATTRIBUTES = (
    "raysPerDeg",
    "powerFactor",
    "maxRangeM",
    "maxAzAngDeg",
    "maxElAngDeg",
    "rangeResM",
    "velResMps",
    "boreAzResDeg",
    "boreElResDeg",
    "carrierFrequency",
    "chirpDuration",
    "chirpBandwidth",
    "chirpRepetitionTime",
    "frequencyStepBetweenChirps",
    "supportFactor",
    "txAzimuthElementSpacing",
    "txElevationElementSpacing",
    "txAzimuthSteeringAngleDeg",
    "txElevationSteeringAngleDeg",
    "rxAzimuthElementSpacing",
    "rxElevationElementSpacing",
    "rxAzimuthSteeringAngleDeg",
    "rxElevationSteeringAngleDeg",
    "apertureAzimuthElementSpacing",
    "apertureElevationElementSpacing",
    "cfarMinVal",
    "cfarOffset",
    "cfarNoiseMean",
    "cfarNoiseSDev",
    "maxVelMpsSequence",
    "rcsTuningCoefficients",
    "azimuthRadNoiseMean",
    "azimuthRadNoiseSDev",
    "rangeNoiseMean",
    "rangeNoiseSDev",
    "velocityNoiseMean",
    "velocityNoiseSDev",
    "elevationRadNoiseMean",
    "elevationRadNoiseSDev",
    "exponentialDecayFactor",
    "debugForceRange",
    "debugForceV",
    "debugForceAz",
    "debugForceEl",
    "rxAntennaAzDeg",
    "rxAntennaElDeg",
    "rxAntennaCoGain",
    "rxAntennaCrossGain",
    "txAntennaAzDeg",
    "txAntennaElDeg",
    "txAntennaCoGain",
    "txAntennaCrossGain",
)


def _radars(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in scoped_prims(stage) if prim.GetTypeName() == "OmniRadar"]


@_RULES.rule("RADAR-STRUCT-001")
def _radar_presence(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if not _radars(stage):
        fail(checker, requirement, "The validated asset subtree must contain an OmniRadar prim", stage)


def _radar_candidates(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in scoped_prims(stage) if prim.GetTypeName() == "OmniRadar" or prim.HasAPI(_CORE_API)]


def _scan_instances(radar: Usd.Prim) -> list[str]:
    return applied_instances(radar, _SCAN_API)


def _scans(stage: Usd.Stage) -> list[tuple[Usd.Prim, str]]:
    return [(radar, instance) for radar in _radars(stage) for instance in _scan_instances(radar)]


def _native_render_vars(stage: Usd.Stage) -> list[Usd.Prim]:
    products = render_products_targeting(stage, sensor_type_names={"OmniRadar"})
    return [render_var for _product, render_var in native_render_vars(products)]


@_RULES.rule("RADAR-SCHEMA-001")
def _prim_type(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for prim in _radar_candidates(stage):
        if prim.GetTypeName() != "OmniRadar":
            fail(
                checker,
                requirement,
                f"Radar WPM-DMAT API is applied to non-OmniRadar prim {prim.GetPath()}",
                prim,
            )


def _required_api(
    api_name: str,
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar in _radars(stage):
            if not radar.HasAPI(api_name):
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} lacks {api_name}",
                    radar,
                )

    return check


@_RULES.rule("RADAR-SCHEMA-005")
def _default_scan(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar in _radars(stage):
        if "s001" not in _scan_instances(radar):
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} lacks default scan instance s001",
                radar,
            )


@_RULES.rule("RADAR-SCHEMA-006")
def _scan_cardinality(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar in _radars(stage):
        count = len(_scan_instances(radar))
        if count < 1 or count > 255:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} has {count} scan instances; expected 1 through 255",
                radar,
            )


def _scan_names(
    predicate: Callable[[str], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            if not predicate(instance):
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} scan instance {instance!r} {description}",
                    radar,
                )

    return check


@_RULES.rule("RADAR-SCHEMA-009")
def _case_collisions(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar in _radars(stage):
        groups: dict[str, list[str]] = {}
        for instance in _scan_instances(radar):
            groups.setdefault(instance.lower(), []).append(instance)
        for names in groups.values():
            if len(names) > 1:
                warn(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} has case-fold-colliding scans {names}",
                    radar,
                )


@_RULES.rule("RADAR-OUTPUT-001")
def _native_sources(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
        source = render_var_source_name(render_var)
        if source not in _NATIVE_SOURCE_NAMES:
            fail(
                checker,
                requirement,
                f"Radar RenderVar {render_var.GetPath()} has unsupported sourceName {source!r}; "
                f"expected one of {sorted(_NATIVE_SOURCE_NAMES)}",
                render_var,
            )


@_RULES.rule("RADAR-OUTPUT-002")
def _gmo_cardinality(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
        if render_var_source_name(render_var) == "GenericModelOutput" and len(render_var_channels(render_var)) > 1:
            fail(
                checker,
                requirement,
                f"Radar GMO RenderVar {render_var.GetPath()} has more than one channel",
                render_var,
            )


@_RULES.rule("RADAR-OUTPUT-003")
def _gmo_domain(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
        channels = render_var_channels(render_var)
        if (
            render_var_source_name(render_var) == "GenericModelOutput"
            and len(channels) == 1
            and channels[0] not in _GMO_CHANNELS
        ):
            fail(
                checker,
                requirement,
                f"Radar GMO RenderVar {render_var.GetPath()} has invalid channel {channels[0]!r}; "
                f"expected one of {sorted(_GMO_CHANNELS)}",
                render_var,
            )


@_RULES.rule("RADAR-OUTPUT-005")
def _point_cloud_domain(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
        channels = render_var_channels(render_var)
        invalid = sorted(set(channels) - _POINT_CLOUD_CHANNELS)
        if render_var_source_name(render_var) == "PointCloud" and invalid:
            fail(
                checker,
                requirement,
                f"Radar PointCloud RenderVar {render_var.GetPath()} has invalid channels {invalid}; "
                f"expected values from {sorted(_POINT_CLOUD_CHANNELS)}",
                render_var,
            )


@_RULES.rule("RADAR-OUTPUT-006")
def _point_cloud_unique(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
        channels = render_var_channels(render_var)
        if render_var_source_name(render_var) == "PointCloud" and len(channels) != len(set(channels)):
            fail(
                checker,
                requirement,
                f"Radar PointCloud RenderVar {render_var.GetPath()} has duplicate channels",
                render_var,
            )


@_RULES.rule("RADAR-OUTPUT-008")
def _channels_applicability(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _native_render_vars(stage):
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


_core_predicate = partial(
    make_bound_check,
    _radars,
    _CORE,
    message=lambda radar, name, value, description: f"Radar {radar.GetPath()} {name}={value!r} {description}",
)


def _scan_predicate(
    name: str,
    predicate: Callable[[Any], bool],
    description: str,
    *,
    active: Callable[[Usd.Prim, str], bool] | None = None,
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            if active is not None and not active(radar, instance):
                continue
            value = _SCAN.value(radar, name, instance)
            if value is None or not predicate(value):
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} scan {instance!r} {name}={value!r} {description}",
                    _SCAN.attribute(radar, name, instance) or radar,
                )

    return check


@_RULES.rule("RADAR-PARAM-004")
def _custom_transform_size(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar in _radars(stage):
        values = as_sequence(_CORE.value(radar, "customFrameOfReferenceTrafo"))
        if len(values) != 6:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} custom transform has {len(values)} elements",
                radar,
            )


@_RULES.rule("RADAR-PARAM-005")
def _custom_transform_finite(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for radar in _radars(stage):
        attribute = _CORE.attribute(radar, "customFrameOfReferenceTrafo")
        if attribute and not numeric_values_are_finite(attribute.Get()):
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} custom transform contains non-finite values",
                attribute,
            )


@_RULES.rule("RADAR-PARAM-010")
def _scan_finiteness(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        invalid = [
            name for name in _SCAN_FLOAT_ATTRIBUTES if not numeric_values_are_finite(_SCAN.value(radar, name, instance))
        ]
        if invalid:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} scan {instance!r} has non-finite values in {invalid}",
                radar,
            )


def _ray_dimension(
    angle_name: str,
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            angle = _SCAN.value(radar, angle_name, instance)
            density = _SCAN.value(radar, "raysPerDeg", instance)
            derived = (
                2 * angle * density if numeric_values_are_finite(angle) and numeric_values_are_finite(density) else -1
            )
            if derived < 1:
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} scan {instance!r} derives ray dimension {derived}; "
                    "expected at least 1",
                    radar,
                )

    return check


@_RULES.rule("RADAR-PARAM-020")
def _ray_product(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        density = _SCAN.value(radar, "raysPerDeg", instance)
        azimuth = _SCAN.value(radar, "maxAzAngDeg", instance)
        elevation = _SCAN.value(radar, "maxElAngDeg", instance)
        if not all(numeric_values_are_finite(value) for value in (density, azimuth, elevation)):
            continue
        azimuth_count = int(2 * azimuth * density)
        elevation_count = int(2 * elevation * density)
        if azimuth_count * elevation_count > _UINT32_MAX:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} scan {instance!r} ray product exceeds {_UINT32_MAX} (UINT32_MAX)",
                radar,
            )


def _array_cardinality(
    name: str, predicate: Callable[[int], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    return _scan_predicate(name, lambda value: predicate(len(as_sequence(value))), description)


def _array_values(
    name: str, predicate: Callable[[Any], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    return _scan_predicate(
        name,
        lambda value: all(predicate(item) for item in as_sequence(value)),
        description,
    )


def _rcs_component(
    index: int, predicate: Callable[[float], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    return _scan_predicate(
        "rcsTuningCoefficients",
        lambda value: len(as_sequence(value)) > index and predicate(as_sequence(value)[index]),
        description,
    )


def _effective_cfar_names(radar: Usd.Prim) -> tuple[tuple[str, str], ...]:
    dimensions = (("cfarRnT", "cfarRnG"), ("cfarVnT", "cfarVnG"))
    if _CORE.value(radar, "cfarMode") == "4D":
        dimensions += (("cfarAznT", "cfarAznG"), ("cfarElnT", "cfarElnG"))
    return dimensions


@_RULES.rule("RADAR-PARAM-049")
def _cfar_width(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    names = (
        "cfarRnT",
        "cfarRnG",
        "cfarVnT",
        "cfarVnG",
        "cfarAznT",
        "cfarAznG",
        "cfarElnT",
        "cfarElnG",
    )
    for radar, instance in _scans(stage):
        invalid = [name for name in names if not 0 <= int(_SCAN.value(radar, name, instance) or 0) <= 255]
        if invalid:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} scan {instance!r} has CFAR fields outside [0, 255]: {invalid}",
                radar,
            )


@_RULES.rule("RADAR-PARAM-050")
def _cfar_guard_order(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        for test_name, guard_name in _effective_cfar_names(radar):
            test = int(_SCAN.value(radar, test_name, instance) or 0)
            guard = int(_SCAN.value(radar, guard_name, instance) or 0)
            if (test or guard) and test <= guard:
                fail(
                    checker,
                    requirement,
                    f"Radar scan {instance!r} requires {test_name} > {guard_name}",
                    radar,
                )


@_RULES.rule("RADAR-PARAM-051")
def _cfar_background(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        if not any(
            int(_SCAN.value(radar, test_name, instance) or 0) > int(_SCAN.value(radar, guard_name, instance) or 0)
            for test_name, guard_name in _effective_cfar_names(radar)
        ):
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} scan {instance!r} needs at least one active CFAR dimension "
                "with test cells greater than guard cells",
                radar,
            )


def _is_fast_spec(radar: Usd.Prim, instance: str) -> bool:
    return _SCAN.value(radar, "datacubeMode", instance) == "FAST" and bool(_SCAN.value(radar, "binsFromSpec", instance))


def _is_fast_explicit(radar: Usd.Prim, instance: str) -> bool:
    return _SCAN.value(radar, "datacubeMode", instance) == "FAST" and not bool(
        _SCAN.value(radar, "binsFromSpec", instance)
    )


def _is_waveform(radar: Usd.Prim, instance: str) -> bool:
    return _SCAN.value(radar, "datacubeMode", instance) == "WAVEFORM"


def _derived_bins(radar: Usd.Prim, instance: str) -> dict[str, int] | None:
    maximums = as_sequence(_SCAN.value(radar, "maxVelMpsSequence", instance))
    values = (
        _SCAN.value(radar, "maxRangeM", instance),
        _SCAN.value(radar, "rangeResM", instance),
        _SCAN.value(radar, "velResMps", instance),
        _SCAN.value(radar, "maxAzAngDeg", instance),
        _SCAN.value(radar, "boreAzResDeg", instance),
        _SCAN.value(radar, "maxElAngDeg", instance),
        _SCAN.value(radar, "boreElResDeg", instance),
    )
    if not maximums or not all(numeric_values_are_finite(value) for value in (*values, maximums[0])):
        return None
    (
        max_range,
        range_res,
        velocity_res,
        max_azimuth,
        azimuth_res,
        max_elevation,
        elevation_res,
    ) = values
    if min(range_res, velocity_res, azimuth_res, elevation_res) <= 0:
        return None
    azimuth_denominator = math.sin(math.radians(azimuth_res) / 2) * math.cos(math.radians(azimuth_res) / 2)
    elevation_denominator = math.sin(math.radians(elevation_res) / 2) * math.cos(math.radians(elevation_res) / 2)
    if azimuth_denominator <= 0 or elevation_denominator <= 0:
        return None
    return {
        "rBins": math.ceil(max_range / range_res),
        "vBins": math.ceil(2 * maximums[0] / velocity_res),
        "azBins": math.ceil(math.sin(math.radians(max_azimuth)) / azimuth_denominator),
        "elBins": math.ceil(math.sin(math.radians(max_elevation)) / elevation_denominator),
    }


def _derived_bin_domain(
    name: str,
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            if not _is_fast_spec(radar, instance):
                continue
            bins = _derived_bins(radar, instance)
            if bins is None or not 1 <= bins[name] <= 65535:
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} scan {instance!r} derived {name} must lie in [1, 65535]",
                    radar,
                )

    return check


def _effective_bins(radar: Usd.Prim, instance: str) -> dict[str, int] | None:
    if _is_waveform(radar, instance):
        return {
            "rBins": int(_SCAN.value(radar, "fastTimePaddingSamples", instance) or 0),
            "vBins": int(_SCAN.value(radar, "slowTimePaddingSamples", instance) or 0),
            "azBins": int(_SCAN.value(radar, "azimuthPaddingSamples", instance) or 0),
            "elBins": int(_SCAN.value(radar, "elevationPaddingSamples", instance) or 0),
        }
    if _is_fast_spec(radar, instance):
        return _derived_bins(radar, instance)
    if _is_fast_explicit(radar, instance):
        return {name: int(_SCAN.value(radar, name, instance) or 0) for name in ("rBins", "vBins", "azBins", "elBins")}
    return None


def _bin_product(
    names: tuple[str, ...], limit: int, label: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            bins = _effective_bins(radar, instance)
            if bins is not None and math.prod(bins[name] for name in names) > limit:
                fail(
                    checker,
                    requirement,
                    f"Radar {radar.GetPath()} scan {instance!r} {label} exceeds {limit}",
                    radar,
                )

    return check


@_RULES.rule("RADAR-PARAM-088")
def _waveform_determinant(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    common = 4 * math.pi / _C_LIGHT
    for radar, instance in _scans(stage):
        if not _is_waveform(radar, instance):
            continue
        samples = int(_SCAN.value(radar, "fastTimeSamples", instance) or 0)
        carrier = _SCAN.value(radar, "carrierFrequency", instance)
        duration = _SCAN.value(radar, "chirpDuration", instance)
        bandwidth = _SCAN.value(radar, "chirpBandwidth", instance)
        repetition = _SCAN.value(radar, "chirpRepetitionTime", instance)
        step_between = _SCAN.value(radar, "frequencyStepBetweenChirps", instance)
        if samples <= 0 or not all(
            numeric_values_are_finite(value) for value in (carrier, duration, bandwidth, repetition, step_between)
        ):
            continue
        fast_time_step = duration / samples
        frequency_step = bandwidth / samples
        determinant = (common * step_between) * (common * carrier * fast_time_step) - (
            common * carrier * repetition
        ) * (common * frequency_step)
        if abs(determinant) < 1e-12:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} scan {instance!r} waveform determinant is singular",
                radar,
            )


def _custom_roles(radar: Usd.Prim, instance: str) -> list[str]:
    return [role for role in ("tx", "rx") if _SCAN.value(radar, f"{role}AntennaGainMode", instance) == "CUSTOM"]


def _patch_roles(radar: Usd.Prim, instance: str) -> list[str]:
    return [
        role for role in ("tx", "rx") if _SCAN.value(radar, f"{role}AntennaGainMode", instance) == "PATCH_ARRAY_MODEL"
    ]


def _custom_axes(
    predicate: Callable[[list[float]], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            for role in _custom_roles(radar, instance):
                for axis in ("AzDeg", "ElDeg"):
                    name = f"{role}Antenna{axis}"
                    values = as_sequence(_SCAN.value(radar, name, instance))
                    if not predicate(values):
                        fail(
                            checker,
                            requirement,
                            f"Radar scan {instance!r} {name} {description}",
                            radar,
                        )

    return check


def _custom_gains(
    predicate: Callable[[list[float], int], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            for role in _custom_roles(radar, instance):
                expected = len(as_sequence(_SCAN.value(radar, f"{role}AntennaAzDeg", instance))) * len(
                    as_sequence(_SCAN.value(radar, f"{role}AntennaElDeg", instance))
                )
                for kind in ("CoGain", "CrossGain"):
                    name = f"{role}Antenna{kind}"
                    values = as_sequence(_SCAN.value(radar, name, instance))
                    if not predicate(values, expected):
                        fail(
                            checker,
                            requirement,
                            f"Radar scan {instance!r} {name} {description}",
                            radar,
                        )

    return check


def _patch_role_value(
    suffix: str, predicate: Callable[[Any], bool], description: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            for role in _patch_roles(radar, instance):
                name = f"{role}{suffix}"
                value = _SCAN.value(radar, name, instance)
                if not predicate(value):
                    fail(
                        checker,
                        requirement,
                        f"Radar scan {instance!r} {name}={value!r} {description}",
                        radar,
                    )

    return check


@_RULES.rule("RADAR-PARAM-100")
def _custom_coverage(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        azimuth = float(_SCAN.value(radar, "maxAzAngDeg", instance) or 0)
        elevation = float(_SCAN.value(radar, "maxElAngDeg", instance) or 0)
        for role in _custom_roles(radar, instance):
            azimuth_axis = as_sequence(_SCAN.value(radar, f"{role}AntennaAzDeg", instance))
            elevation_axis = as_sequence(_SCAN.value(radar, f"{role}AntennaElDeg", instance))
            if (
                len(azimuth_axis) >= 2
                and len(elevation_axis) >= 2
                and (
                    azimuth_axis[0] > -azimuth
                    or azimuth_axis[-1] < azimuth
                    or elevation_axis[0] > -elevation
                    or elevation_axis[-1] < elevation
                )
            ):
                warn(
                    checker,
                    requirement,
                    f"Radar scan {instance!r} {role} custom pattern does not cover its angular extent",
                    radar,
                )


_RULES.check("RADAR-SCHEMA-002", _required_api(_CORE_API))
_RULES.check("RADAR-SCHEMA-003", _required_api("OmniSensorAPI"))
_RULES.check("RADAR-SCHEMA-004", _required_api("OmniSensorEncryptionAPI"))
_RULES.check("RADAR-SCHEMA-007", _scan_names(bool, "must be nonempty"))
_RULES.check(
    "RADAR-SCHEMA-008",
    _scan_names(lambda name: bool(name) and not name[0].isdigit(), "must begin with a non-digit character"),
)


_RULES.check(
    "RADAR-PARAM-001",
    _core_predicate("elementsCoordsType", {"CARTESIAN", "SPHERICAL"}.__contains__, "must be CARTESIAN or SPHERICAL"),
)
_RULES.check(
    "RADAR-PARAM-002",
    _core_predicate(
        "outputFrameOfReference",
        {"SENSOR", "WORLD", "CUSTOM"}.__contains__,
        "must be SENSOR, WORLD, or CUSTOM",
    ),
)
_RULES.check(
    "RADAR-PARAM-003",
    _core_predicate("cfarMode", {"2D", "4D"}.__contains__, "must be 2D or 4D"),
)
_RULES.check(
    "RADAR-PARAM-007",
    _core_predicate(
        "waveLengthMm",
        lambda value: numeric_values_are_finite(value) and value > 0,
        "must be finite and positive",
    ),
)
_RULES.check(
    "RADAR-PARAM-008",
    _core_predicate("traceTreeDepth", lambda value: 0 <= value <= 255, "must lie in [0, 255]"),
)


@_RULES.rule("RADAR-PARAM-009")
def _tick_rate(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar in _radars(stage):
        attribute = radar.GetAttribute("omni:sensor:tickRate")
        value = attribute.Get() if attribute else None
        if value is None or not numeric_values_are_finite(value) or value <= 0:
            fail(
                checker,
                requirement,
                f"Radar {radar.GetPath()} tickRate must be finite and positive",
                attribute or radar,
            )


_POSITIVE_SCAN_PARAMETERS = {
    "RADAR-PARAM-011": "raysPerDeg",
    "RADAR-PARAM-012": "maxRangeM",
    "RADAR-PARAM-013": "maxAzAngDeg",
    "RADAR-PARAM-014": "maxElAngDeg",
    "RADAR-PARAM-015": "powerFactor",
}
for _requirement_id, _name in _POSITIVE_SCAN_PARAMETERS.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value > 0, "must be positive"),
    )
_RULES.check(
    "RADAR-PARAM-016",
    _scan_predicate("maxAzAngDeg", lambda value: value <= 90, "must not exceed 90"),
)
_RULES.check(
    "RADAR-PARAM-017",
    _scan_predicate("maxElAngDeg", lambda value: value <= 90, "must not exceed 90"),
)
_RULES.check("RADAR-PARAM-018", _ray_dimension("maxAzAngDeg"))
_RULES.check("RADAR-PARAM-019", _ray_dimension("maxElAngDeg"))
_RULES.check(
    "RADAR-PARAM-022",
    _scan_predicate("elevMode", {"NO_EL", "POS_EL", "FULL_EL"}.__contains__, "must be NO_EL, POS_EL, or FULL_EL"),
)
_RULES.check(
    "RADAR-PARAM-023",
    _scan_predicate("datacubeMode", {"FAST", "WAVEFORM"}.__contains__, "must be FAST or WAVEFORM"),
)

_WINDOW_NAMES = (
    "windowSlowTime",
    "windowFastTime",
    "txWindowAzimuth",
    "txWindowElevation",
    "rxWindowAzimuth",
    "rxWindowElevation",
    "apertureWindowAzimuth",
    "apertureWindowElevation",
)


@_RULES.rule("RADAR-PARAM-024")
def _window_domains(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        invalid = {
            name: _SCAN.value(radar, name, instance)
            for name in _WINDOW_NAMES
            if _SCAN.value(radar, name, instance) not in _WINDOWS
        }
        if invalid:
            fail(
                checker,
                requirement,
                f"Radar scan {instance!r} has invalid window tokens {invalid}; "
                f"expected one of {sorted(_WINDOWS)} for each field",
                radar,
            )


_RULES.check(
    "RADAR-PARAM-025",
    _scan_predicate("txAntennaGainMode", _ANTENNA_MODES.__contains__, f"must be one of {sorted(_ANTENNA_MODES)}"),
)
_RULES.check(
    "RADAR-PARAM-026",
    _scan_predicate("rxAntennaGainMode", _ANTENNA_MODES.__contains__, f"must be one of {sorted(_ANTENNA_MODES)}"),
)
_RULES.check(
    "RADAR-PARAM-027",
    _array_cardinality(
        "maxVelMpsSequence",
        lambda size: 1 <= size <= 256,
        "must have 1 through 256 elements",
    ),
)
_RULES.check(
    "RADAR-PARAM-028",
    _array_values("maxVelMpsSequence", numeric_values_are_finite, "must contain finite values"),
)
_RULES.check(
    "RADAR-PARAM-029",
    _array_values("maxVelMpsSequence", lambda value: value > 0, "must contain positive values"),
)
_RULES.check(
    "RADAR-PARAM-030",
    _array_cardinality("rcsTuningCoefficients", lambda size: size == 3, "must have three elements"),
)
_RULES.check(
    "RADAR-PARAM-031",
    _array_values("rcsTuningCoefficients", numeric_values_are_finite, "must contain finite values"),
)
_RULES.check(
    "RADAR-PARAM-032",
    _rcs_component(1, lambda value: value > 0, "scale must be positive"),
)
_NONNEGATIVE_SCAN_PARAMETERS = {
    "RADAR-PARAM-034": "azimuthRadNoiseSDev",
    "RADAR-PARAM-035": "elevationRadNoiseSDev",
    "RADAR-PARAM-036": "rangeNoiseSDev",
    "RADAR-PARAM-037": "velocityNoiseSDev",
}
for _requirement_id, _name in _NONNEGATIVE_SCAN_PARAMETERS.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value >= 0, "must be nonnegative"),
    )
_RULES.check(
    "RADAR-PARAM-039",
    _rcs_component(2, lambda value: value >= 0, "noise deviation must be nonnegative"),
)

_FINITE_MEANS = {
    "RADAR-PARAM-040": "azimuthRadNoiseMean",
    "RADAR-PARAM-041": "elevationRadNoiseMean",
    "RADAR-PARAM-042": "rangeNoiseMean",
    "RADAR-PARAM-043": "velocityNoiseMean",
    "RADAR-PARAM-044": "cfarNoiseMean",
}
for _requirement_id, _name in _FINITE_MEANS.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, numeric_values_are_finite, "must be finite"),
    )
_RULES.check(
    "RADAR-PARAM-045",
    _scan_predicate("exponentialDecayFactor", numeric_values_are_finite, "must be finite"),
)
_RULES.check(
    "RADAR-PARAM-046",
    _scan_predicate("cfarMinVal", lambda value: value >= 0, "must be nonnegative"),
)
_RULES.check(
    "RADAR-PARAM-047",
    _scan_predicate("cfarOffset", lambda value: value >= 0, "must be nonnegative"),
)
_RULES.check(
    "RADAR-PARAM-048",
    _scan_predicate("cfarNoiseSDev", lambda value: value >= 0, "must be nonnegative"),
)

for _requirement_id, _name in {
    "RADAR-PARAM-052": "rangeResM",
    "RADAR-PARAM-053": "velResMps",
    "RADAR-PARAM-054": "boreAzResDeg",
    "RADAR-PARAM-055": "boreElResDeg",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value > 0, "must be positive", active=_is_fast_spec),
    )
for _requirement_id, _name in {
    "RADAR-PARAM-056": "rBins",
    "RADAR-PARAM-057": "vBins",
    "RADAR-PARAM-058": "azBins",
    "RADAR-PARAM-059": "elBins",
}.items():
    _RULES.check(_requirement_id, _derived_bin_domain(_name))
for _requirement_id, _name in {
    "RADAR-PARAM-060": "rBins",
    "RADAR-PARAM-061": "vBins",
    "RADAR-PARAM-062": "azBins",
    "RADAR-PARAM-063": "elBins",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(
            _name,
            lambda value: 1 <= value <= 65535,
            "must lie in [1, 65535]",
            active=_is_fast_explicit,
        ),
    )
_RULES.check(
    "RADAR-PARAM-064",
    _bin_product(("rBins", "vBins", "azBins", "elBins"), _INT32_MAX, "datacube product"),
)
_RULES.check(
    "RADAR-PARAM-065",
    _bin_product(("rBins", "vBins"), _INT32_MAX, "CFAR product"),
)

_RULES.check(
    "RADAR-PARAM-067",
    _scan_predicate(
        "waveformMode",
        lambda value: value in (0, 1, 2),
        "must be 0, 1, or 2",
        active=_is_waveform,
    ),
)
for _requirement_id, _name in {
    "RADAR-PARAM-068": "slowTimeSamples",
    "RADAR-PARAM-069": "fastTimeSamples",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value > 0, "must be positive", active=_is_waveform),
    )
for _requirement_id, _name in {
    "RADAR-PARAM-070": "azimuthPaddingSamples",
    "RADAR-PARAM-071": "elevationPaddingSamples",
    "RADAR-PARAM-072": "slowTimePaddingSamples",
    "RADAR-PARAM-073": "fastTimePaddingSamples",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(
            _name,
            lambda value: 1 <= value <= 65535,
            "must lie in [1, 65535]",
            active=_is_waveform,
        ),
    )


def _padding_coverage(
    padding: str, source: str
) -> Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]:
    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for radar, instance in _scans(stage):
            if _is_waveform(radar, instance) and int(_SCAN.value(radar, padding, instance) or 0) < int(
                _SCAN.value(radar, source, instance) or 0
            ):
                fail(
                    checker,
                    requirement,
                    f"Radar scan {instance!r} {padding} is smaller than {source}",
                    radar,
                )

    return check


for _requirement_id, _padding, _source in (
    ("RADAR-PARAM-074", "azimuthPaddingSamples", "apertureArrayAzimuthElements"),
    ("RADAR-PARAM-075", "elevationPaddingSamples", "apertureArrayElevationElements"),
    ("RADAR-PARAM-076", "slowTimePaddingSamples", "slowTimeSamples"),
    ("RADAR-PARAM-077", "fastTimePaddingSamples", "fastTimeSamples"),
):
    _RULES.check(
        _requirement_id,
        _padding_coverage(_padding, _source),
    )

for _requirement_id, _name in {
    "RADAR-PARAM-078": "carrierFrequency",
    "RADAR-PARAM-079": "chirpDuration",
    "RADAR-PARAM-080": "chirpBandwidth",
    "RADAR-PARAM-081": "supportFactor",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value > 0, "must be positive", active=_is_waveform),
    )


@_RULES.rule("RADAR-PARAM-082")
def _chirp_relation(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for radar, instance in _scans(stage):
        if _is_waveform(radar, instance) and _SCAN.value(radar, "chirpRepetitionTime", instance) < _SCAN.value(
            radar, "chirpDuration", instance
        ):
            fail(
                checker,
                requirement,
                f"Radar scan {instance!r} chirp repetition is shorter than duration",
                radar,
            )


_RULES.check(
    "RADAR-PARAM-083",
    _scan_predicate(
        "frequencyStepBetweenChirps",
        numeric_values_are_finite,
        "must be finite",
        active=_is_waveform,
    ),
)
for _requirement_id, _name in {
    "RADAR-PARAM-084": "apertureArrayAzimuthElements",
    "RADAR-PARAM-085": "apertureArrayElevationElements",
    "RADAR-PARAM-086": "apertureAzimuthElementSpacing",
    "RADAR-PARAM-087": "apertureElevationElementSpacing",
}.items():
    _RULES.check(
        _requirement_id,
        _scan_predicate(_name, lambda value: value > 0, "must be positive", active=_is_waveform),
    )
_RULES.check(
    "RADAR-PARAM-090",
    _custom_axes(lambda values: len(values) >= 2, "must contain at least two elements"),
)
_RULES.check(
    "RADAR-PARAM-091",
    _custom_axes(lambda values: all(numeric_values_are_finite(value) for value in values), "must be finite"),
)
_RULES.check(
    "RADAR-PARAM-092",
    _custom_axes(
        lambda values: all(a < b for a, b in zip(values, values[1:])),
        "must be strictly increasing",
    ),
)
_RULES.check(
    "RADAR-PARAM-093",
    _custom_gains(
        lambda values, expected: len(values) == expected,
        "has incorrect grid cardinality",
    ),
)
_RULES.check(
    "RADAR-PARAM-094",
    _custom_gains(
        lambda values, _expected: all(numeric_values_are_finite(value) for value in values),
        "must be finite",
    ),
)
_RULES.check(
    "RADAR-PARAM-096",
    _patch_role_value("ArrayAzimuthElements", lambda value: value > 0, "must be positive"),
)
_RULES.check(
    "RADAR-PARAM-097",
    _patch_role_value("ArrayElevationElements", lambda value: value > 0, "must be positive"),
)
_RULES.check(
    "RADAR-PARAM-098",
    _patch_role_value("AzimuthElementSpacing", lambda value: value > 0, "must be positive"),
)
_RULES.check(
    "RADAR-PARAM-099",
    _patch_role_value("ElevationElementSpacing", lambda value: value > 0, "must be positive"),
)

del _name, _padding, _requirement_id, _source
