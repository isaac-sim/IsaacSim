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

"""Atomic lens-distortion and generalized-projection LUT validation rules."""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import Any, BinaryIO

import usd_validation_nvidia
from pxr import Ar, Sdf, Usd

from .sensor_rule_utils import (
    RuleRegistrar,
    attribute_value,
    fail,
    is_finite,
    prims,
    warn,
)

_RULES = RuleRegistrar("IsaacSim.Sensor.Camera.Lens")

_LENS_APIS = {
    "OmniLensDistortionFthetaAPI": ("ftheta", "omni:lensdistortion:ftheta:"),
    "OmniLensDistortionKannalaBrandtK3API": (
        "kannalaBrandtK3",
        "omni:lensdistortion:kannalaBrandtK3:",
    ),
    "OmniLensDistortionRadTanThinPrismAPI": (
        "radTanThinPrism",
        "omni:lensdistortion:radTanThinPrism:",
    ),
    "OmniLensDistortionLutAPI": ("lut", "omni:lensdistortion:lut:"),
    "OmniLensDistortionOmniDirectionalStereoAPI": (
        "omniDirectionalStereo",
        "omni:lensdistortion:omniDirectionalStereo:",
    ),
    "OmniLensDistortionOpenCvFisheyeAPI": (
        "opencvFisheye",
        "omni:lensdistortion:opencvFisheye:",
    ),
    "OmniLensDistortionOpenCvPinholeAPI": (
        "opencvPinhole",
        "omni:lensdistortion:opencvPinhole:",
    ),
}
_DEPRECATED_APIS = frozenset(
    {
        "OmniLensDistortionFthetaAPI",
        "OmniLensDistortionKannalaBrandtK3API",
        "OmniLensDistortionRadTanThinPrismAPI",
    }
)
_LOSSLESS_EXR_COMPRESSION = frozenset({0, 1, 2, 3, 4})
_EXR_MAGIC = 20000630


def _cameras(stage: Usd.Stage) -> list[Usd.Prim]:
    return prims(stage, {"Camera"})


def _applied_lens_apis(camera: Usd.Prim) -> list[str]:
    applied = set(camera.GetAppliedSchemas())
    return [schema for schema in _LENS_APIS if schema in applied]


def _lens_value(camera: Usd.Prim, api_name: str, short_name: str) -> Any:
    return attribute_value(camera, f"{_LENS_APIS[api_name][1]}{short_name}")


@_RULES.rule("CAMERA-LENS-001")
def _one_lens_api(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        applied = _applied_lens_apis(camera)
        if len(applied) > 1:
            fail(checker, requirement, f"Camera {camera.GetPath()} applies multiple lens APIs: {applied}", camera)


@_RULES.rule("CAMERA-LENS-002")
def _model_consistency(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        applied = _applied_lens_apis(camera)
        if len(applied) != 1:
            continue
        expected = _LENS_APIS[applied[0]][0]
        attribute = camera.GetAttribute("omni:lensdistortion:model")
        actual = str(attribute.Get() or "") if attribute else ""
        if actual != expected:
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} lens model is {actual!r}; expected {expected!r}",
                attribute or camera,
            )


@_RULES.rule("CAMERA-LENS-003")
def _finite_lens_values(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        if not _applied_lens_apis(camera):
            continue
        for attribute in camera.GetAttributes():
            if attribute.GetName().startswith("omni:lensdistortion:") and not is_finite(attribute.Get()):
                fail(checker, requirement, f"{attribute.GetPath()} contains a non-finite value", attribute)


def _lens_bound(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    predicate: Any,
    description: str,
    *,
    apis: set[str] | None = None,
) -> None:
    for camera in _cameras(stage):
        for api_name in _applied_lens_apis(camera):
            if apis is not None and api_name not in apis:
                continue
            name = f"{_LENS_APIS[api_name][1]}{short_name}"
            attribute = camera.GetAttribute(name)
            if not attribute:
                continue
            value = attribute.Get()
            if value is None or not predicate(value):
                fail(checker, requirement, f"{name} on {camera.GetPath()} must be {description}", attribute)


@_RULES.rule("CAMERA-LENS-014")
def _deprecated_lens(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        for api_name in set(_applied_lens_apis(camera)) & _DEPRECATED_APIS:
            warn(checker, requirement, f"Camera {camera.GetPath()} uses deprecated {api_name} in Kit 110.4", camera)


def _sample_interval(end: float, count: int = 1025) -> list[float]:
    return [end * index / (count - 1) for index in range(count)]


def _analytic_domain(camera: Usd.Prim, api_name: str) -> tuple[float, float]:
    width = float(_lens_value(camera, api_name, "nominalWidth") or 0)
    height = float(_lens_value(camera, api_name, "nominalHeight") or 0)
    center = _lens_value(camera, api_name, "opticalCenter") or (width / 2, height / 2)
    radius = max(math.hypot(x - center[0], y - center[1]) for x in (0.0, width) for y in (0.0, height))
    theta = math.radians(float(_lens_value(camera, api_name, "maxFov") or 0) / 2)
    return radius, theta


def _polynomial(coefficients: list[float], value: float) -> float:
    return sum(coefficient * value**index for index, coefficient in enumerate(coefficients))


def _polynomial_derivative(coefficients: list[float], value: float) -> float:
    return sum(index * coefficient * value ** (index - 1) for index, coefficient in enumerate(coefficients) if index)


def _ftheta_coefficients(camera: Usd.Prim) -> list[float]:
    return [float(_lens_value(camera, "OmniLensDistortionFthetaAPI", f"k{index}") or 0) for index in range(5)]


@_RULES.rule("CAMERA-LENS-015")
def _ftheta_finite(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    api_name = "OmniLensDistortionFthetaAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        radius, _ = _analytic_domain(camera, api_name)
        coefficients = _ftheta_coefficients(camera)
        if any(not math.isfinite(_polynomial(coefficients, value)) for value in _sample_interval(radius)):
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} F-theta polynomial is non-finite in its radial domain",
                camera,
            )


def _ftheta_monotonic(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    simple: bool,
) -> None:
    api_name = "OmniLensDistortionFthetaAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        radius, theta_end = _analytic_domain(camera, api_name)
        coefficients = _ftheta_coefficients(camera)
        values = _sample_interval(radius)
        admitted_values = [value for value in values if 0 <= _polynomial(coefficients, value) <= theta_end]
        derivatives = [_polynomial_derivative(coefficients, value) for value in admitted_values]
        if simple:
            valid = all(abs(value) > 1e-12 for value in derivatives)
        else:
            # A continuous, strictly increasing map whose endpoint values cover
            # the admitted theta interval has exactly one radial inverse root.
            valid = (
                _polynomial(coefficients, 0) <= 0
                and _polynomial(coefficients, radius) >= theta_end
                and all(value > 0 for value in derivatives)
            )
        if not valid:
            description = "has a zero derivative" if simple else "is not one-to-one"
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} F-theta inverse {description} in the nominal domain",
                camera,
            )


def _kannala_map(camera: Usd.Prim, theta: float) -> float:
    api_name = "OmniLensDistortionKannalaBrandtK3API"
    coefficients = [float(_lens_value(camera, api_name, f"k{index}") or 0) for index in range(4)]
    theta_squared = theta * theta
    return theta * (
        1 + sum(coefficient * theta_squared ** (index + 1) for index, coefficient in enumerate(coefficients))
    )


def _kannala_derivative(camera: Usd.Prim, theta: float) -> float:
    api_name = "OmniLensDistortionKannalaBrandtK3API"
    coefficients = [float(_lens_value(camera, api_name, f"k{index}") or 0) for index in range(4)]
    theta_squared = theta * theta
    return 1 + sum(
        (2 * index + 3) * coefficient * theta_squared ** (index + 1) for index, coefficient in enumerate(coefficients)
    )


def _kannala_check(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    derivative: bool,
) -> None:
    api_name = "OmniLensDistortionKannalaBrandtK3API"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        _, theta_end = _analytic_domain(camera, api_name)
        values = _sample_interval(theta_end)
        valid = (
            all(
                math.isfinite(_kannala_derivative(camera, theta)) and _kannala_derivative(camera, theta) > 0
                for theta in values
            )
            if derivative
            else all(math.isfinite(_kannala_map(camera, theta)) for theta in values)
        )
        if not valid:
            description = "does not have a positive derivative" if derivative else "is non-finite"
            fail(checker, requirement, f"Camera {camera.GetPath()} Kannala-Brandt map {description}", camera)


def _radtan_coefficients(camera: Usd.Prim, prefix: str, count: int) -> list[float]:
    api_name = "OmniLensDistortionRadTanThinPrismAPI"
    return [float(_lens_value(camera, api_name, f"{prefix}{index}") or 0) for index in range(count)]


def _radtan_radial(camera: Usd.Prim, theta: float) -> float:
    theta_squared = theta * theta
    coefficients = _radtan_coefficients(camera, "k", 6)
    return theta * (
        1 + sum(coefficient * theta_squared ** (index + 1) for index, coefficient in enumerate(coefficients))
    )


def _radtan_radial_derivative(camera: Usd.Prim, theta: float) -> float:
    theta_squared = theta * theta
    coefficients = _radtan_coefficients(camera, "k", 6)
    return 1 + sum(
        (2 * index + 3) * coefficient * theta_squared ** (index + 1) for index, coefficient in enumerate(coefficients)
    )


def _radtan_radial_check(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    increasing: bool,
) -> None:
    api_name = "OmniLensDistortionRadTanThinPrismAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        _, theta_end = _analytic_domain(camera, api_name)
        values = _sample_interval(theta_end)
        if increasing:
            valid = all(
                math.isfinite(_radtan_radial_derivative(camera, theta)) and _radtan_radial_derivative(camera, theta) > 0
                for theta in values
            )
            description = "is not strictly increasing"
        else:
            valid = all(math.isfinite(_radtan_radial(camera, theta)) for theta in values)
            description = "is non-finite"
        if not valid:
            fail(checker, requirement, f"Camera {camera.GetPath()} RadTan radial map {description}", camera)


def _radtan_domain(camera: Usd.Prim) -> tuple[float, float, float, float]:
    api_name = "OmniLensDistortionRadTanThinPrismAPI"
    width = float(_lens_value(camera, api_name, "nominalWidth") or 0)
    height = float(_lens_value(camera, api_name, "nominalHeight") or 0)
    center = _lens_value(camera, api_name, "opticalCenter") or (width / 2, height / 2)
    if width == 0:
        return (math.inf, math.inf, math.inf, math.inf)
    xs = (-2 * center[0] / width, 2 * (width - center[0]) / width)
    # The renderer flips image Y and normalizes both axes by nominalWidth.
    ys = (2 * (center[1] - height) / width, 2 * center[1] / width)
    return min(xs), max(xs), min(ys), max(ys)


def _radtan_jacobian(camera: Usd.Prim, x: float, y: float) -> tuple[float, float, float, float]:
    p0, p1 = _radtan_coefficients(camera, "p", 2)
    s0, s1, s2, s3 = _radtan_coefficients(camera, "s", 4)
    radius_squared = x * x + y * y
    vx = 2 * (p0 + s0) + 4 * radius_squared * s1
    vy = 2 * (p1 + s2) + 4 * radius_squared * s3
    dot_product = p0 * x + p1 * y
    # This is I + d(tangential + thin-prism)/d(xr), the matrix inverted
    # by FishEyeRadTanThinPrism::unproject in ProjectionUtils.hlsl.
    return (
        1 + 2 * dot_product + 2 * x * p0 + vx * x,
        2 * x * p1 + vx * y,
        2 * y * p0 + vy * x,
        1 + 2 * dot_product + 2 * y * p1 + vy * y,
    )


def _radtan_jacobian_check(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    nonsingular: bool,
) -> None:
    api_name = "OmniLensDistortionRadTanThinPrismAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        valid = True
        for x, y in _grid(_radtan_domain(camera), 65):
            jacobian = _radtan_jacobian(camera, x, y)
            if not all(math.isfinite(value) for value in jacobian):
                valid = False
                break
            determinant = jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2]
            if nonsingular and abs(determinant) <= 1e-12:
                valid = False
                break
        if not valid:
            description = "is singular" if nonsingular else "contains a non-finite entry"
            fail(checker, requirement, f"Camera {camera.GetPath()} RadTan inverse Jacobian {description}", camera)


def _opencv_domain(camera: Usd.Prim, api_name: str) -> tuple[float, float, float, float]:
    size = _lens_value(camera, api_name, "imageSize") or (0, 0)
    fx = float(_lens_value(camera, api_name, "fx") or 0)
    fy = float(_lens_value(camera, api_name, "fy") or 0)
    cx = float(_lens_value(camera, api_name, "cx") or 0)
    cy = float(_lens_value(camera, api_name, "cy") or 0)
    xs = ((0 - cx) / fx, (size[0] - cx) / fx) if fx else (math.inf, math.inf)
    ys = ((0 - cy) / fy, (size[1] - cy) / fy) if fy else (math.inf, math.inf)
    return min(xs), max(xs), min(ys), max(ys)


def _grid(bounds: tuple[float, float, float, float], count: int = 65) -> list[tuple[float, float]]:
    xmin, xmax, ymin, ymax = bounds
    return [
        (xmin + (xmax - xmin) * x / (count - 1), ymin + (ymax - ymin) * y / (count - 1))
        for x in range(count)
        for y in range(count)
    ]


def _fisheye_map(camera: Usd.Prim, theta: float) -> float:
    api_name = "OmniLensDistortionOpenCvFisheyeAPI"
    coefficients = [float(_lens_value(camera, api_name, f"k{index}") or 0) for index in range(1, 5)]
    return theta * (1 + sum(coefficient * theta ** (2 * index) for index, coefficient in enumerate(coefficients, 1)))


def _fisheye_derivative(camera: Usd.Prim, theta: float) -> float:
    api_name = "OmniLensDistortionOpenCvFisheyeAPI"
    coefficients = [float(_lens_value(camera, api_name, f"k{index}") or 0) for index in range(1, 5)]
    return 1 + sum(
        (2 * index + 1) * coefficient * theta ** (2 * index) for index, coefficient in enumerate(coefficients, 1)
    )


def _fisheye_theta_edge(camera: Usd.Prim) -> float | None:
    radius = max(math.hypot(x, y) for x, y in _grid(_opencv_domain(camera, "OmniLensDistortionOpenCvFisheyeAPI"), 2))
    if not math.isfinite(radius) or radius <= 0:
        return None
    previous_theta = 0.0
    previous_value = -radius
    for theta in _sample_interval(math.pi, 4097)[1:]:
        value = _fisheye_map(camera, theta) - radius
        if math.isfinite(value) and previous_value <= 0 <= value:
            lower, upper = previous_theta, theta
            for _ in range(64):
                middle = (lower + upper) / 2
                if _fisheye_map(camera, middle) < radius:
                    lower = middle
                else:
                    upper = middle
            return (lower + upper) / 2
        previous_theta, previous_value = theta, value
    return None


@_RULES.rule("CAMERA-LENS-024")
def _fisheye_edge(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    api_name = "OmniLensDistortionOpenCvFisheyeAPI"
    for camera in _cameras(stage):
        if api_name in _applied_lens_apis(camera) and _fisheye_theta_edge(camera) is None:
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} has no finite positive OpenCV fisheye edge solution",
                camera,
            )


@_RULES.rule("CAMERA-LENS-025")
def _fisheye_increasing(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    api_name = "OmniLensDistortionOpenCvFisheyeAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        edge = _fisheye_theta_edge(camera)
        if edge is not None and any(_fisheye_derivative(camera, theta) <= 0 for theta in _sample_interval(edge)):
            fail(
                checker, requirement, f"Camera {camera.GetPath()} OpenCV fisheye map is not strictly increasing", camera
            )


def _pinhole_coefficients(camera: Usd.Prim) -> dict[str, float]:
    api_name = "OmniLensDistortionOpenCvPinholeAPI"
    return {
        name: float(_lens_value(camera, api_name, name) or 0)
        for name in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2", "s1", "s2", "s3", "s4")
    }


def _pinhole_distort(camera: Usd.Prim, x: float, y: float) -> tuple[float, float]:
    c = _pinhole_coefficients(camera)
    r2 = x * x + y * y
    r4 = r2 * r2
    r6 = r4 * r2
    numerator = 1 + c["k1"] * r2 + c["k2"] * r4 + c["k3"] * r6
    denominator = 1 + c["k4"] * r2 + c["k5"] * r4 + c["k6"] * r6
    radial = numerator / denominator
    xd = x * radial + 2 * c["p1"] * x * y + c["p2"] * (r2 + 2 * x * x) + c["s1"] * r2 + c["s2"] * r4
    yd = y * radial + c["p1"] * (r2 + 2 * y * y) + 2 * c["p2"] * x * y + c["s3"] * r2 + c["s4"] * r4
    return xd, yd


def _numeric_jacobian(function: Any, x: float, y: float) -> tuple[float, float, float, float]:
    step = 1e-5 * max(1.0, abs(x), abs(y))
    xp = function(x + step, y)
    xm = function(x - step, y)
    yp = function(x, y + step)
    ym = function(x, y - step)
    return (
        (xp[0] - xm[0]) / (2 * step),
        (yp[0] - ym[0]) / (2 * step),
        (xp[1] - xm[1]) / (2 * step),
        (yp[1] - ym[1]) / (2 * step),
    )


def _pinhole_denominator(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    nonzero: bool,
) -> None:
    api_name = "OmniLensDistortionOpenCvPinholeAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        c = _pinhole_coefficients(camera)
        values = []
        for x, y in _grid(_opencv_domain(camera, api_name)):
            r2 = x * x + y * y
            values.append(1 + c["k4"] * r2 + c["k5"] * r2**2 + c["k6"] * r2**3)
        valid = (
            all(abs(value) > 1e-12 for value in values) if nonzero else all(math.isfinite(value) for value in values)
        )
        if not valid:
            description = "zero" if nonzero else "non-finite"
            fail(
                checker,
                requirement,
                f"Camera {camera.GetPath()} OpenCV pinhole denominator becomes {description}",
                camera,
            )


def _pinhole_jacobian(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    *,
    nonsingular: bool,
) -> None:
    api_name = "OmniLensDistortionOpenCvPinholeAPI"
    for camera in _cameras(stage):
        if api_name not in _applied_lens_apis(camera):
            continue
        valid = True
        for x, y in _grid(_opencv_domain(camera, api_name), 33):
            try:
                jacobian = _numeric_jacobian(lambda px, py: _pinhole_distort(camera, px, py), x, y)
            except (ValueError, ZeroDivisionError):
                valid = False
                break
            if not all(math.isfinite(value) for value in jacobian):
                valid = False
                break
            if nonsingular and abs(jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2]) <= 1e-12:
                valid = False
                break
        if not valid:
            description = "is singular" if nonsingular else "contains a non-finite entry"
            fail(checker, requirement, f"Camera {camera.GetPath()} OpenCV pinhole Jacobian {description}", camera)


def _resolve_asset(attribute: Usd.Attribute) -> Path | None:
    value = attribute.Get() if attribute else None
    if not isinstance(value, Sdf.AssetPath) or not value.path:
        return None
    resolved = value.resolvedPath or str(Ar.GetResolver().Resolve(value.path))
    return Path(resolved or value.path)


def _lut_attributes(stage: Usd.Stage, short_name: str) -> list[tuple[Usd.Prim, Usd.Attribute]]:
    name = f"omni:lensdistortion:lut:{short_name}"
    return [
        (camera, camera.GetAttribute(name))
        for camera in _cameras(stage)
        if "OmniLensDistortionLutAPI" in _applied_lens_apis(camera)
    ]


def _read_cstring(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\0", offset)
    if end < 0:
        raise ValueError("unterminated OpenEXR string")
    return data[offset:end].decode("ascii"), end + 1


def _read_stream_cstring(stream: BinaryIO) -> str:
    value = bytearray()
    while (character := stream.read(1)) != b"\0":
        if not character:
            raise ValueError("unterminated OpenEXR string")
        value.extend(character)
    return value.decode("ascii")


def _read_exr_header(path: Path) -> dict[str, Any]:
    file_size = path.stat().st_size
    attributes: dict[str, bytes] = {}
    with path.open("rb") as stream:
        prefix = stream.read(8)
        if len(prefix) < 8 or struct.unpack_from("<I", prefix)[0] != _EXR_MAGIC:
            raise ValueError("not an OpenEXR file")
        while True:
            name = _read_stream_cstring(stream)
            if not name:
                break
            _read_stream_cstring(stream)  # Attribute type; the checks below use only the payload.
            length = stream.read(4)
            if len(length) != 4:
                raise ValueError("truncated OpenEXR attribute")
            size = struct.unpack("<I", length)[0]
            if stream.tell() + size > file_size:
                raise ValueError("truncated OpenEXR attribute payload")
            if name in {"channels", "compression", "dataWindow"}:
                payload = stream.read(size)
                if len(payload) != size:
                    raise ValueError("truncated OpenEXR attribute payload")
                attributes[name] = payload
            else:
                stream.seek(size, 1)
    channels: dict[str, int] = {}
    channel_data = attributes.get("channels", b"")
    channel_offset = 0
    while channel_offset < len(channel_data) and channel_data[channel_offset] != 0:
        name, channel_offset = _read_cstring(channel_data, channel_offset)
        if channel_offset + 16 > len(channel_data):
            raise ValueError("truncated OpenEXR channel")
        channels[name] = struct.unpack_from("<i", channel_data, channel_offset)[0]
        channel_offset += 16
    compression = (attributes.get("compression") or b"\xff")[0]
    data_window = attributes.get("dataWindow", b"")
    if len(data_window) != 16:
        raise ValueError("missing OpenEXR dataWindow")
    xmin, ymin, xmax, ymax = struct.unpack("<4i", data_window)
    return {
        "channels": channels,
        "compression": compression,
        "width": xmax - xmin + 1,
        "height": ymax - ymin + 1,
    }


def _lut_file(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any, short_name: str
) -> None:
    for camera, attribute in _lut_attributes(stage, short_name):
        path = _resolve_asset(attribute)
        if path is None or not path.is_file():
            fail(
                checker,
                requirement,
                f"{attribute.GetName()} on {camera.GetPath()} does not resolve to a file",
                attribute or camera,
            )


def _lut_header(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
    requirement: Any,
    short_name: str,
    predicate: Any,
    description: str,
) -> None:
    for camera, attribute in _lut_attributes(stage, short_name):
        path = _resolve_asset(attribute)
        if path is None or not path.is_file():
            continue
        try:
            header = _read_exr_header(path)
            valid = predicate(header)
        except (OSError, UnicodeDecodeError, ValueError, struct.error):
            valid = False
        if not valid:
            fail(checker, requirement, f"{attribute.GetName()} on {camera.GetPath()} must {description}", attribute)


@_RULES.rule("CAMERA-LUT-021")
def _matching_aspect_ratio(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        if "OmniLensDistortionLutAPI" not in _applied_lens_apis(camera):
            continue
        enter = camera.GetAttribute("omni:lensdistortion:lut:rayEnterDirectionTexture")
        exit_ = camera.GetAttribute("omni:lensdistortion:lut:rayExitPositionTexture")
        enter_path, exit_path = _resolve_asset(enter), _resolve_asset(exit_)
        if not enter_path or not exit_path or not enter_path.is_file() or not exit_path.is_file():
            continue
        try:
            enter_header = _read_exr_header(enter_path)
            exit_header = _read_exr_header(exit_path)
            matches = enter_header["width"] * exit_header["height"] == exit_header["width"] * enter_header["height"]
        except (OSError, UnicodeDecodeError, ValueError, struct.error):
            continue
        if not matches:
            fail(checker, requirement, f"Camera {camera.GetPath()} LUT textures have different aspect ratios", camera)


def _lut_cameras(stage: Usd.Stage) -> list[Usd.Prim]:
    return [camera for camera in _cameras(stage) if "OmniLensDistortionLutAPI" in _applied_lens_apis(camera)]


def _lut_with_optional_alpha(stage: Usd.Stage) -> list[Usd.Prim]:
    cameras = []
    for camera, attribute in _lut_attributes(stage, "rayEnterDirectionTexture"):
        path = _resolve_asset(attribute)
        if path is None or not path.is_file():
            continue
        try:
            if "A" in _read_exr_header(path)["channels"]:
                cameras.append(camera)
        except (OSError, UnicodeDecodeError, ValueError, struct.error):
            continue
    return cameras


for requirement_id, name, predicate, description, apis in (
    ("CAMERA-LENS-004", "nominalWidth", lambda value: value > 0, "greater than zero", None),
    ("CAMERA-LENS-005", "nominalHeight", lambda value: value > 0, "greater than zero", None),
    (
        "CAMERA-LENS-006",
        "imageSize",
        lambda value: len(value) == 2 and all(component > 0 for component in value),
        "two positive components",
        {"OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"},
    ),
    ("CAMERA-LENS-007", "maxFov", lambda value: 0 < value <= 360, "in (0, 360] degrees", _DEPRECATED_APIS),
    ("CAMERA-LENS-008", "opticalCenter", is_finite, "finite", None),
    (
        "CAMERA-LENS-009",
        "cx",
        math.isfinite,
        "finite",
        {"OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"},
    ),
    (
        "CAMERA-LENS-010",
        "cy",
        math.isfinite,
        "finite",
        {"OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"},
    ),
    (
        "CAMERA-LENS-011",
        "fx",
        lambda value: value > 0,
        "greater than zero",
        {"OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"},
    ),
    (
        "CAMERA-LENS-012",
        "fy",
        lambda value: value > 0,
        "greater than zero",
        {"OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"},
    ),
    (
        "CAMERA-LENS-013",
        "interpupillaryDistance",
        lambda value: value > 0,
        "greater than zero",
        {"OmniLensDistortionOmniDirectionalStereoAPI"},
    ),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, name=name, predicate=predicate, description=description, apis=apis: _lens_bound(
            checker, stage, requirement, name, predicate, description, apis=set(apis) if apis is not None else None
        ),
    )
_RULES.check(
    "CAMERA-LENS-016",
    lambda checker, stage, requirement: _ftheta_monotonic(checker, stage, requirement, simple=False),
)
_RULES.check(
    "CAMERA-LENS-017",
    lambda checker, stage, requirement: _ftheta_monotonic(checker, stage, requirement, simple=True),
)
_RULES.check(
    "CAMERA-LENS-018",
    lambda checker, stage, requirement: _kannala_check(checker, stage, requirement, derivative=False),
)
_RULES.check(
    "CAMERA-LENS-019",
    lambda checker, stage, requirement: _kannala_check(checker, stage, requirement, derivative=True),
)
_RULES.check(
    "CAMERA-LENS-020",
    lambda checker, stage, requirement: _radtan_radial_check(checker, stage, requirement, increasing=False),
)
_RULES.check(
    "CAMERA-LENS-021",
    lambda checker, stage, requirement: _radtan_radial_check(checker, stage, requirement, increasing=True),
)
_RULES.check(
    "CAMERA-LENS-022",
    lambda checker, stage, requirement: _radtan_jacobian_check(checker, stage, requirement, nonsingular=False),
)
_RULES.check(
    "CAMERA-LENS-023",
    lambda checker, stage, requirement: _radtan_jacobian_check(checker, stage, requirement, nonsingular=True),
)
_RULES.check(
    "CAMERA-LENS-026",
    lambda checker, stage, requirement: _pinhole_denominator(checker, stage, requirement, nonzero=False),
)
_RULES.check(
    "CAMERA-LENS-027",
    lambda checker, stage, requirement: _pinhole_denominator(checker, stage, requirement, nonzero=True),
)
_RULES.check(
    "CAMERA-LENS-028",
    lambda checker, stage, requirement: _pinhole_jacobian(checker, stage, requirement, nonsingular=False),
)
_RULES.check(
    "CAMERA-LENS-029",
    lambda checker, stage, requirement: _pinhole_jacobian(checker, stage, requirement, nonsingular=True),
)
for requirement_id, short_name in (
    ("CAMERA-LUT-001", "rayEnterDirectionTexture"),
    ("CAMERA-LUT-002", "rayExitPositionTexture"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, short_name=short_name: _lut_file(checker, stage, requirement, short_name),
    )
for requirement_id in ("CAMERA-LUT-003", "CAMERA-LUT-004"):
    _RULES.manual(
        requirement_id,
        _lut_cameras,
        reason=(
            "Static validation confirms the OpenEXR container in the other LUT rules, but Kit 110.4 "
            "does not expose its renderer texture decoder to usd-validation; decode success requires manual review"
        ),
    )
for requirement_id, short_name in (
    ("CAMERA-LUT-005", "rayEnterDirectionTexture"),
    ("CAMERA-LUT-006", "rayExitPositionTexture"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, short_name=short_name: _lut_header(
            checker,
            stage,
            requirement,
            short_name,
            lambda header: header["compression"] in _LOSSLESS_EXR_COMPRESSION,
            "use lossless OpenEXR compression 0 (NO_COMPRESSION), 1 (RLE), 2 (ZIPS), 3 (ZIP), or 4 (PIZ)",
        ),
    )
for requirement_id, short_name, channel in (
    ("CAMERA-LUT-007", "rayEnterDirectionTexture", "R"),
    ("CAMERA-LUT-008", "rayEnterDirectionTexture", "G"),
    ("CAMERA-LUT-009", "rayEnterDirectionTexture", "B"),
    ("CAMERA-LUT-010", "rayExitPositionTexture", "R"),
    ("CAMERA-LUT-011", "rayExitPositionTexture", "G"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, short_name=short_name, channel=channel: _lut_header(
            checker,
            stage,
            requirement,
            short_name,
            lambda header: channel in header["channels"],
            f"contain a named {channel} channel",
        ),
    )
for requirement_id, short_name, channel in (
    ("CAMERA-LUT-012", "rayEnterDirectionTexture", "R"),
    ("CAMERA-LUT-013", "rayEnterDirectionTexture", "G"),
    ("CAMERA-LUT-014", "rayEnterDirectionTexture", "B"),
    ("CAMERA-LUT-015", "rayExitPositionTexture", "R"),
    ("CAMERA-LUT-016", "rayExitPositionTexture", "G"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, short_name=short_name, channel=channel: _lut_header(
            checker,
            stage,
            requirement,
            short_name,
            lambda header: header["channels"].get(channel) == 2,
            f"encode channel {channel} as OpenEXR FLOAT",
        ),
    )
_RULES.manual(
    "CAMERA-LUT-032",
    _lut_with_optional_alpha,
    reason="Optional ray-entry A channel is present; its texel finiteness requires OpenEXR pixel decoding review",
)
for requirement_id, short_name, dimension in (
    ("CAMERA-LUT-017", "rayEnterDirectionTexture", "width"),
    ("CAMERA-LUT-018", "rayEnterDirectionTexture", "height"),
    ("CAMERA-LUT-019", "rayExitPositionTexture", "width"),
    ("CAMERA-LUT-020", "rayExitPositionTexture", "height"),
):
    _RULES.check(
        requirement_id,
        lambda checker, stage, requirement, short_name=short_name, dimension=dimension: _lut_header(
            checker,
            stage,
            requirement,
            short_name,
            lambda header: header[dimension] > 0,
            f"have a positive decoded {dimension}",
        ),
    )
for requirement_id in (
    "CAMERA-LUT-022",
    "CAMERA-LUT-023",
    "CAMERA-LUT-024",
    "CAMERA-LUT-025",
    "CAMERA-LUT-026",
    "CAMERA-LUT-027",
    "CAMERA-LUT-028",
    "CAMERA-LUT-029",
    "CAMERA-LUT-030",
    "CAMERA-LUT-031",
):
    _RULES.manual(
        requirement_id,
        _lut_cameras,
        reason=(
            "The authored USD stores only a texture asset path; proving decoded texel values or their physical "
            "correspondence requires an image decoder and calibration ground truth unavailable to static validation"
        ),
    )
