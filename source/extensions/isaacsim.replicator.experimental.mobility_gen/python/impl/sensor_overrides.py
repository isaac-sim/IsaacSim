# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utilities for saving and restoring sensor calibration overrides across recordings."""

from __future__ import annotations

import os

from pxr import Sdf

_SENSOR_OVERRIDES_FILENAME = "sensor_overrides.usda"

# Camera attributes that are safe to include in the calibration override file.
# Only pure USD values are included — OmniGraph-connected attributes are excluded
# because sublayer opinions on connected attributes shadow the OmniGraph connection
# and crash the SDGPipeline.  cameraProjectionType IS included because it is a
# plain USD attribute on nova_carter cameras, not an OmniGraph-connected attribute.
_CAMERA_CALIBRATION_ATTRS = frozenset(
    (
        # OpenUSD pinhole intrinsics (UsdGeom.Camera)
        "focalLength",
        "horizontalAperture",
        "verticalAperture",
        "horizontalApertureOffset",
        "verticalApertureOffset",
        "clippingRange",
        "clippingPlanes",
        "fStop",
        "focusDistance",
        # Legacy Omniverse distortion (Layer 3) — what nova_carter uses.
        # Hawk cameras: physicalDistortionModel=rational_polynomial (opencvPinhole), 8 coeffs.
        # Owl cameras:  physicalDistortionModel=kannalaBrandt, 4 coeffs.
        "cameraProjectionType",
        "physicalDistortionModel",
        "physicalDistortionCoefficients",
        # Fisheye polynomial (ftheta) — also present on nova_carter cameras.
        "fthetaPolyA",
        "fthetaPolyB",
        "fthetaPolyC",
        "fthetaPolyD",
        "fthetaPolyE",
        "fthetaCx",
        "fthetaCy",
        "fthetaWidth",
        "fthetaHeight",
        # Modern Omniverse lens distortion API (omni:lensdistortion applied schema).
        "omni:lensdistortion:model",
        "omni:lensdistortion:k1",
        "omni:lensdistortion:k2",
        "omni:lensdistortion:k3",
        "omni:lensdistortion:k4",
        "omni:lensdistortion:k5",
        "omni:lensdistortion:k6",
        "omni:lensdistortion:p1",
        "omni:lensdistortion:p2",
        # opencvPinhole sub-namespace (modern API, used by Isaac Sim 5.0+).
        "omni:lensdistortion:opencvPinhole:fx",
        "omni:lensdistortion:opencvPinhole:fy",
        "omni:lensdistortion:opencvPinhole:cx",
        "omni:lensdistortion:opencvPinhole:cy",
        "omni:lensdistortion:opencvPinhole:imageSize",
        "omni:lensdistortion:opencvPinhole:k1",
        "omni:lensdistortion:opencvPinhole:k2",
        "omni:lensdistortion:opencvPinhole:k3",
        "omni:lensdistortion:opencvPinhole:k4",
        "omni:lensdistortion:opencvPinhole:k5",
        "omni:lensdistortion:opencvPinhole:k6",
        "omni:lensdistortion:opencvPinhole:p1",
        "omni:lensdistortion:opencvPinhole:p2",
        "omni:lensdistortion:opencvPinhole:s1",
        "omni:lensdistortion:opencvPinhole:s2",
        "omni:lensdistortion:opencvPinhole:s3",
        "omni:lensdistortion:opencvPinhole:s4",
        # opencvFisheye sub-namespace (equidistant/kannalaBrandt model).
        "omni:lensdistortion:opencvFisheye:fx",
        "omni:lensdistortion:opencvFisheye:fy",
        "omni:lensdistortion:opencvFisheye:cx",
        "omni:lensdistortion:opencvFisheye:cy",
        "omni:lensdistortion:opencvFisheye:imageSize",
        "omni:lensdistortion:opencvFisheye:k1",
        "omni:lensdistortion:opencvFisheye:k2",
        "omni:lensdistortion:opencvFisheye:k3",
        "omni:lensdistortion:opencvFisheye:k4",
        # Extrinsics
        "xformOp:translate",
        "xformOp:rotateXYZ",
        "xformOp:rotateXZY",
        "xformOp:rotateYXZ",
        "xformOp:rotateYZX",
        "xformOp:rotateZXY",
        "xformOp:rotateZYX",
        "xformOp:orient",
        "xformOp:transform",
        "xformOpOrder",
    )
)


def save_sensor_overrides(
    robot_prim_path: str,
    output_dir: str,
    root_layer: "Sdf.Layer | None" = None,
    stage: "Usd.Stage | None" = None,
) -> None:
    """Save user calibration overrides on the robot prim subtree to sensor_overrides.usda.

    Extracts only the attribute opinions present in the root layer for the robot
    prim subtree — the changes a user made in the Isaac Sim UI on top of the base
    robot USD asset.  Physics runtime state and programmatically-built prims (chase
    camera) are stripped so the file contains only the calibration overrides.

    Args:
        robot_prim_path: USD prim path of the robot root (e.g. ``"/World/robot"``).
        output_dir: Directory in which to write ``sensor_overrides.usda``.
        root_layer: Root layer of the current stage.  Resolved from the live Kit
            stage when *None* (default); pass explicitly for testing.
        stage: Live USD stage used to resolve composed prim types.  Resolved from
            the Kit context when *None* (default); pass explicitly for testing.
    """
    if root_layer is None or stage is None:
        from isaacsim.core.experimental.utils.stage import get_current_stage

        _live = get_current_stage()
        if root_layer is None:
            root_layer = _live.GetRootLayer()
        if stage is None:
            stage = _live

    if not root_layer.GetPrimAtPath(robot_prim_path):
        return

    diff_layer = Sdf.Layer.CreateAnonymous(".usda")

    def _copy_calibration(src_spec: Sdf.PrimSpec, dest_path: Sdf.Path) -> bool:
        """Recursively copy only calibration overrides into diff_layer.

        Skips the chase camera (always rebuilt programmatically) and non-camera
        prims.  Over-specs in the root layer carry no typeName — the Camera type
        lives in the referenced robot USD — so we check the composed type via the
        live stage instead of src_spec.typeName.  Returns True only if something
        worth saving was written, so empty prims are never created.
        """
        if src_spec.name == "chase_camera":
            return False

        # Use the composed stage type to identify Camera prims — over-specs in the
        # root layer authored by UI edits have typeName="" even for camera prims.
        composed_prim = stage.GetPrimAtPath(src_spec.path)
        is_camera = composed_prim.IsValid() and composed_prim.GetTypeName() == "Camera"

        # Sdf.PrimSpec.properties iterates PropertySpec objects, not name strings —
        # use prop.name to compare against the calibration-attr name set.
        attr_names = (
            [prop.name for prop in src_spec.properties if prop.name in _CAMERA_CALIBRATION_ATTRS] if is_camera else []
        )

        child_had_content = False
        for child in src_spec.nameChildren.values():
            if _copy_calibration(child, dest_path.AppendChild(child.name)):
                child_had_content = True

        if not attr_names and not child_had_content:
            return False

        dest_spec = Sdf.CreatePrimInLayer(diff_layer, dest_path)
        dest_spec.specifier = Sdf.SpecifierOver
        if src_spec.typeName:
            dest_spec.typeName = src_spec.typeName

        for attr_name in attr_names:
            Sdf.CopySpec(
                root_layer,
                src_spec.path.AppendProperty(attr_name),
                diff_layer,
                dest_path.AppendProperty(attr_name),
            )

        return True

    # Create ancestor over-specs so dest_path is a valid location in diff_layer.
    dest_path = Sdf.Path(robot_prim_path)
    ancestors: list[Sdf.Path] = []
    p = dest_path.GetParentPath()
    while p != Sdf.Path.absoluteRootPath and p.IsAbsolutePath():
        ancestors.append(p)
        p = p.GetParentPath()
    for ancestor in reversed(ancestors):
        Sdf.CreatePrimInLayer(diff_layer, ancestor).specifier = Sdf.SpecifierOver

    src_spec = root_layer.GetPrimAtPath(robot_prim_path)
    if src_spec:
        _copy_calibration(src_spec, dest_path)

    os.makedirs(output_dir, exist_ok=True)
    diff_layer.Export(os.path.join(output_dir, _SENSOR_OVERRIDES_FILENAME))


def apply_sensor_overrides(robot_prim_path: str, recording_path: str, stage: "Usd.Stage | None" = None) -> None:
    """Apply saved calibration overrides from sensor_overrides.usda onto the live stage.

    Reads attribute default values via the Sdf layer API rather than opening a
    full USD stage.  This avoids creating a separate stage context that would
    send Kit-level USD change notifications and hang the subsequent
    simulation_app.update() call.

    No-op if ``sensor_overrides.usda`` does not exist in *recording_path* (backward
    compatible with recordings that predate this feature).

    Args:
        robot_prim_path: USD prim path of the robot root (e.g. ``"/World/robot"``).
        recording_path: Directory of the recording, expected to contain ``sensor_overrides.usda``.
        stage: Live USD stage to apply overrides onto.  Resolved from the Kit
            stage when *None* (default); pass explicitly for testing.
    """
    override_path = os.path.join(recording_path, _SENSOR_OVERRIDES_FILENAME)
    if not os.path.exists(override_path):
        return

    if stage is None:
        from isaacsim.core.experimental.utils.stage import get_current_stage

        stage = get_current_stage()

    import carb

    override_layer = Sdf.Layer.FindOrOpen(override_path)
    if override_layer is None:
        return

    def _apply(spec_path: Sdf.Path) -> None:
        spec = override_layer.GetPrimAtPath(spec_path)
        if not spec:
            return
        prim = stage.GetPrimAtPath(spec_path)
        if prim.IsValid():
            for attr_name, prop_spec in spec.properties.items():
                if not isinstance(prop_spec, Sdf.AttributeSpec):
                    continue
                value = prop_spec.default
                if value is None:
                    continue
                attr = prim.GetAttribute(attr_name)
                if attr.IsValid():
                    attr.Set(value)
                    carb.log_info(f"[sensor_overrides] set {spec_path}.{attr_name} = {value}")
        for child_name in spec.nameChildren.keys():
            _apply(spec_path.AppendChild(child_name))

    with Sdf.ChangeBlock():
        _apply(Sdf.Path(robot_prim_path))


def log_camera_properties(stage: "Usd.Stage", robot_prim_path: str) -> None:
    """Log camera calibration properties under the robot prim."""
    import carb
    from pxr import Usd

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return
    for prim in Usd.PrimRange(robot_prim):
        if prim.GetTypeName() == "Camera" and prim.GetName() != "chase_camera":

            def _get(name: str) -> object:
                a = prim.GetAttribute(name)
                return a.Get() if a.IsValid() else None

            carb.log_info(
                f"[sensor_overrides] camera '{prim.GetPath()}'\n"
                f"  pinhole     : focalLength={_get('focalLength')}"
                f"  hAperture={_get('horizontalAperture')}"
                f"  vAperture={_get('verticalAperture')}\n"
                f"  projection  : cameraProjectionType={_get('cameraProjectionType')}\n"
                f"  distortion  : physicalDistortionModel={_get('physicalDistortionModel')}"
                f"  coeffs={_get('physicalDistortionCoefficients')}\n"
                f"  ftheta      : polyA={_get('fthetaPolyA')} polyB={_get('fthetaPolyB')}"
                f"  polyC={_get('fthetaPolyC')} polyD={_get('fthetaPolyD')} polyE={_get('fthetaPolyE')}"
                f"  cx={_get('fthetaCx')} cy={_get('fthetaCy')}"
                f"  w={_get('fthetaWidth')} h={_get('fthetaHeight')}\n"
                f"  omni:lens   : model={_get('omni:lensdistortion:model')}"
                f"  k1={_get('omni:lensdistortion:k1')} k2={_get('omni:lensdistortion:k2')}"
                f"  k3={_get('omni:lensdistortion:k3')} p1={_get('omni:lensdistortion:p1')}"
                f"  p2={_get('omni:lensdistortion:p2')}\n"
                f"  opencvPinhole: fx={_get('omni:lensdistortion:opencvPinhole:fx')}"
                f"  fy={_get('omni:lensdistortion:opencvPinhole:fy')}"
                f"  cx={_get('omni:lensdistortion:opencvPinhole:cx')}"
                f"  cy={_get('omni:lensdistortion:opencvPinhole:cy')}"
                f"  size={_get('omni:lensdistortion:opencvPinhole:imageSize')}"
                f"  k1={_get('omni:lensdistortion:opencvPinhole:k1')}"
                f"  k2={_get('omni:lensdistortion:opencvPinhole:k2')}"
                f"  p1={_get('omni:lensdistortion:opencvPinhole:p1')}"
                f"  p2={_get('omni:lensdistortion:opencvPinhole:p2')}\n"
                f"  opencvFisheye: fx={_get('omni:lensdistortion:opencvFisheye:fx')}"
                f"  fy={_get('omni:lensdistortion:opencvFisheye:fy')}"
                f"  cx={_get('omni:lensdistortion:opencvFisheye:cx')}"
                f"  cy={_get('omni:lensdistortion:opencvFisheye:cy')}"
                f"  size={_get('omni:lensdistortion:opencvFisheye:imageSize')}"
                f"  k1={_get('omni:lensdistortion:opencvFisheye:k1')}"
                f"  k2={_get('omni:lensdistortion:opencvFisheye:k2')}"
                f"  k3={_get('omni:lensdistortion:opencvFisheye:k3')}"
                f"  k4={_get('omni:lensdistortion:opencvFisheye:k4')}"
            )
