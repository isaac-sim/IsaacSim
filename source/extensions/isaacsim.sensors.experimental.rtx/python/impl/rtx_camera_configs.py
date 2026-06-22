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

"""Configuration definitions for supported RTX Camera sensors in Isaac Sim.

This module defines the supported RTX Camera sensor USD assets together with
the metadata used by the UI extension (vendor grouping, menu display name,
default stage prim prefix, depth-sensor flag).

Vendor and prim prefix are *derived* from the asset path; only fields that
cannot be reliably inferred (display name, depth-sensor flag, variant spec)
need to be authored explicitly per entry. Use :func:`get_camera_metadata` to
obtain the fully normalized metadata for a given asset path.
"""

import pathlib
from typing import Any

#: Default Camera variant set name (used for entries whose variants are flat strings).
SUPPORTED_CAMERA_VARIANT_SET_NAME = "sensor"

#: Display-name overrides for vendors whose ``/Isaac/Sensors/<Vendor>/`` folder
#: name is CamelCased without spaces. Add an entry only when the on-disk vendor
#: folder differs from the desired display label.
_VENDOR_DISPLAY_OVERRIDES: dict[str, str] = {
    "LeopardImaging": "Leopard Imaging",
}

#: Map of supported Camera asset paths to their per-config metadata.
#:
#: Unlike ``SUPPORTED_LIDAR_CONFIGS``, ``SUPPORTED_RADAR_CONFIGS``, and
#: ``SUPPORTED_ACOUSTIC_CONFIGS`` (whose values are the variant spec directly),
#: each value here is a metadata ``dict`` so the camera registry can also drive
#: the vendor-grouped UI menu and distinguish depth sensors. The supported keys are:
#:
#: - ``"display_name"`` (``str``, required): Human-readable name shown in menu items.
#: - ``"is_depth_sensor"`` (``bool``, optional): When ``True``, the UI creates a
#:   :class:`~isaacsim.sensors.experimental.rtx.SingleViewDepthCameraSensor` instead of a
#:   plain ``Xform`` reference. Defaults to ``False`` when missing.
#: - ``"variants"`` (``set[str] | list[dict[str, str]]``, optional): Variant spec
#:   in the same shape as ``SUPPORTED_LIDAR_CONFIGS`` values. Defaults to
#:   ``set()`` when missing.
#:
#: ``vendor`` and ``prim_prefix`` are derived from the asset path; use
#: :func:`get_camera_metadata` to obtain the normalized dict including those fields.
SUPPORTED_CAMERA_CONFIGS: dict[str, dict[str, Any]] = {
    # Orbbec
    "/Isaac/Sensors/Orbbec/Gemini2/orbbec_gemini2_v1.0.usd": {
        "display_name": "Orbbec Gemini 2",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Orbbec/FemtoMega/orbbec_femtomega_v1.0.usd": {
        "display_name": "Orbbec FemtoMega",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Orbbec/Gemini335/orbbec_gemini_335.usd": {
        "display_name": "Orbbec Gemini 335",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Orbbec/Gemini335L/orbbec_gemini_335L.usd": {
        "display_name": "Orbbec Gemini 335L",
        "is_depth_sensor": True,
    },
    # Leopard Imaging
    "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd": {"display_name": "Hawk"},
    "/Isaac/Sensors/LeopardImaging/Owl/owl.usd": {"display_name": "Owl"},
    # Luxonis
    "/Isaac/Sensors/Luxonis/OAK4-D/oak4_d.usd": {
        "display_name": "Luxonis OAK4-D",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Luxonis/OAK4-D_Wide/oak4_d_wide.usd": {
        "display_name": "Luxonis OAK4-D Wide",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Luxonis/OAK-D_Pro_PoE/oak_d_pro_poe.usd": {
        "display_name": "Luxonis OAK-D Pro PoE",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Luxonis/OAK-D_Pro_W_PoE/oak_d_pro_w_poe.usd": {
        "display_name": "Luxonis OAK-D Pro W PoE",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/Luxonis/OAK-D_ToF/oak_d_tof.usd": {
        "display_name": "Luxonis OAK-D ToF",
        "is_depth_sensor": True,
    },
    # RealSense
    "/Isaac/Sensors/RealSense/D455/rsd455.usd": {
        "display_name": "Realsense D455",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/RealSense/D457/rsd457.usd": {
        "display_name": "Realsense D457",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/RealSense/D555/rsd555.usd": {
        "display_name": "Realsense D555",
        "is_depth_sensor": True,
    },
    # Sensing
    "/Isaac/Sensors/Sensing/SG2/H100F1A/SG2-AR0233C-5200-G2A-H100F1A.usd": {
        "display_name": "Sensing SG2-AR0233C-5200-G2A-H100F1A",
    },
    "/Isaac/Sensors/Sensing/SG2/H60YA/Camera_SG2_OX03CC_5200_GMSL2_H60YA.usd": {
        "display_name": "Sensing SG2-OX03CC-5200-GMSL2-H60YA",
    },
    "/Isaac/Sensors/Sensing/SG3/H190XA/SG3S-ISX031C-GMSL2F-H190XA.usd": {
        "display_name": "Sensing SG3-ISX031C-GMSL2F-H190XA",
    },
    "/Isaac/Sensors/Sensing/SG5/H100SA/SG5-IMX490C-5300-GMSL2-H110SA.usd": {
        "display_name": "Sensing SG5-IMX490C-5300-GMSL2-H110SA",
    },
    "/Isaac/Sensors/Sensing/SG8/H120YA/SG8S-AR0820C-5300-G2A-H120YA.usd": {
        "display_name": "Sensing SG8S-AR0820C-5300-G2A-H120YA",
    },
    "/Isaac/Sensors/Sensing/SG8/H30YA/SG8S-AR0820C-5300-G2A-H30YA.usd": {
        "display_name": "Sensing SG8S-AR0820C-5300-G2A-H30YA",
    },
    "/Isaac/Sensors/Sensing/SG8/H60SA/SG8S-AR0820C-5300-G2A-H60SA.usd": {
        "display_name": "Sensing SG8S-AR0820C-5300-G2A-H60SA",
    },
    # SICK
    "/Isaac/Sensors/SICK/Inspector83x/SICK_Inspector83x.usd": {"display_name": "Inspector83x"},
    "/Isaac/Sensors/SICK/InspectorP61x/SICK_InspectorP61x.usd": {"display_name": "InspectorP61x"},
    "/Isaac/Sensors/SICK/safeVisionary2/SICK_safeVisionary2.usd": {
        "display_name": "safeVisionary2",
        "is_depth_sensor": True,
    },
    "/Isaac/Sensors/SICK/Visionary-T_Mini/SICK_Visionary-T_Mini.usd": {
        "display_name": "Visionary-T Mini",
        "is_depth_sensor": True,
    },
    # Stereolabs
    "/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usd": {
        "display_name": "ZED_X",
        "is_depth_sensor": True,
    },
}


def _camera_vendor(config_path: str) -> str:
    """Return the display vendor name for a registered camera asset.

    Vendor is taken from the fourth path component (``/Isaac/Sensors/<Vendor>/...``)
    and run through ``_VENDOR_DISPLAY_OVERRIDES`` to handle CamelCase folder
    names that should display with spaces (e.g. ``LeopardImaging`` -> ``"Leopard Imaging"``).
    Returns an empty string when *config_path* is not a conventional Isaac Sim asset path.

    Args:
        config_path: Registered camera asset path.

    Returns:
        Display vendor name.
    """
    parts = pathlib.Path(config_path).parts
    raw = parts[3] if len(parts) > 3 else ""
    return _VENDOR_DISPLAY_OVERRIDES.get(raw, raw)


def _camera_prim_prefix(config_path: str) -> str:
    """Return the default stage prim path prefix for a registered camera asset.

    Derived from the USD file stem with hyphens and dots replaced by underscores
    so the result is a valid USD prim name. Each registered asset has a unique
    stem, so the resulting prefixes are also unique and stable across runs.

    Args:
        config_path: Registered camera asset path.

    Returns:
        Default stage prim path prefix.
    """
    return "/" + pathlib.Path(config_path).stem.replace("-", "_").replace(".", "_")


def get_camera_metadata(config_path: str) -> dict[str, Any]:
    """Return the normalized metadata for a registered camera asset.

    Combines the explicitly authored fields from ``SUPPORTED_CAMERA_CONFIGS``
    with the path-derived ``vendor`` and ``prim_prefix`` to produce a single
    dict suitable for UI consumption.

    Args:
        config_path: Asset path that must appear as a key in
            ``SUPPORTED_CAMERA_CONFIGS``.

    Returns:
        Dict with keys ``"vendor"``, ``"display_name"``, ``"prim_prefix"``,
        ``"is_depth_sensor"``, and ``"variants"``.

    Raises:
        KeyError: If *config_path* is not registered.
    """
    raw = SUPPORTED_CAMERA_CONFIGS[config_path]
    return {
        "vendor": _camera_vendor(config_path),
        "display_name": raw["display_name"],
        "prim_prefix": _camera_prim_prefix(config_path),
        "is_depth_sensor": raw.get("is_depth_sensor", False),
        "variants": raw.get("variants", set()),
    }
