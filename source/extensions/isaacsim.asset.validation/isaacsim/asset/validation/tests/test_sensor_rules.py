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

"""Tests for atomic shared and camera authored-sensor validation rules."""

from __future__ import annotations

import io
import struct
import tempfile
from pathlib import Path
from unittest.mock import patch

import omni.kit.test
import usd_validation_nvidia
from isaacsim.asset.validation.lens_lut_rules import _read_exr_header
from isaacsim.asset.validation.sensor_capabilities import CAPABILITIES, FEATURES, get_requirement
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

from .utils import validate as _validate

_ROOT = Sdf.Path("/SensorAsset")
_CAMERA = _ROOT.AppendChild("Camera")
_PRODUCT = _ROOT.AppendChild("RenderProduct")


def _add_render_var(stage: Usd.Stage, product: UsdRender.Product, name: str) -> Usd.Prim:
    """Add and order one RenderVar beneath a RenderProduct."""
    path = product.GetPath().AppendChild(name.replace(".", "_"))
    render_var = UsdRender.Var.Define(stage, path)
    render_var.CreateSourceNameAttr(name)
    relationship = product.CreateOrderedVarsRel()
    relationship.AddTarget(path)
    return render_var.GetPrim()


def _camera_stage(*, second_camera: bool = False, kit_first: bool = False) -> Usd.Stage:
    """Build a valid camera asset with an RGB RenderProduct."""
    stage = Usd.Stage.CreateInMemory()
    if kit_first:
        UsdGeom.Camera.Define(stage, "/AAA_OmniverseKit_Persp")
    root = UsdGeom.Xform.Define(stage, _ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    camera = UsdGeom.Camera.Define(stage, _CAMERA).GetPrim()
    camera.ApplyAPI("OmniSensorAPI")
    product = UsdRender.Product.Define(stage, _PRODUCT)
    product.CreateCameraRel().SetTargets([_CAMERA])
    product.CreateResolutionAttr(Gf.Vec2i(1920, 1080))
    _add_render_var(stage, product, "LdrColor")
    if second_camera:
        path = _ROOT.AppendChild("SecondCamera")
        other = UsdGeom.Camera.Define(stage, path).GetPrim()
        other.ApplyAPI("OmniSensorAPI")
        other_product = UsdRender.Product.Define(stage, _ROOT.AppendChild("SecondProduct"))
        other_product.CreateCameraRel().SetTargets([path])
        other_product.CreateResolutionAttr(Gf.Vec2i(640, 480))
        _add_render_var(stage, other_product, "LdrColor")
    return stage


def _validate_feature(stage: Usd.Stage, feature_name: str) -> list[usd_validation_nvidia.Issue]:
    """Validate a stage through a feature so its requirement parameters are active."""
    engine = usd_validation_nvidia.ValidationEngine(init_rules=False, variants=True)
    engine.enable_feature(FEATURES[feature_name])
    return list(engine.validate(stage).issues)


def _failure_messages(stage: Usd.Stage, *requirement_ids: str) -> list[str]:
    """Return only failure and error messages."""
    return [
        issue.message
        for issue in _validate(stage, *requirement_ids)
        if issue.severity in (usd_validation_nvidia.IssueSeverity.FAILURE, usd_validation_nvidia.IssueSeverity.ERROR)
    ]


def _add_depth(stage: Usd.Stage) -> Usd.Prim:
    """Apply the depth schema and required RenderVars to the camera RenderProduct."""
    product_prim = stage.GetPrimAtPath(_PRODUCT)
    product_prim.ApplyAPI("OmniSensorDepthSensorSingleViewAPI")
    product = UsdRender.Product(product_prim)
    for name in (
        "Camera3dPositionSD",
        "DistanceToImagePlaneSD",
        "DepthSensorDistance",
    ):
        _add_render_var(stage, product, name)
    return product_prim


def _add_isp(stage: Usd.Stage) -> Usd.Prim:
    """Apply Generic Camera Core and add required ISP RenderVars."""
    camera = stage.GetPrimAtPath(_CAMERA)
    camera.ApplyAPI("OmniSensorGenericCameraCoreAPI")
    camera.GetAttribute("omni:sensor:modelName").Set("CameraCore")
    camera.GetAttribute("omni:sensor:modelVendor").Set("NVIDIA")
    camera.GetAttribute("omni:sensor:marketName").Set("Generic")
    product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
    _add_render_var(stage, product, "HdrColor")
    _add_render_var(stage, product, "OmniCameraSensorPostIsp")
    return camera


def _exr_attribute(name: str, type_name: str, payload: bytes) -> bytes:
    return name.encode() + b"\0" + type_name.encode() + b"\0" + struct.pack("<I", len(payload)) + payload


def _minimal_exr(path: Path, *, compression: int = 3, channels: tuple[str, ...] = ("R", "G", "B")) -> None:
    """Write a minimal header plus payload accepted by the static EXR parser."""
    channel_list = (
        b"".join(channel.encode() + b"\0" + struct.pack("<iB3xii", 2, 0, 1, 1) for channel in channels) + b"\0"
    )
    data_window = struct.pack("<4i", 0, 0, 3, 1)
    content = (
        struct.pack("<II", 20000630, 2)
        + _exr_attribute("channels", "chlist", channel_list)
        + _exr_attribute("compression", "compression", bytes([compression]))
        + _exr_attribute("dataWindow", "box2i", data_window)
        + b"\0"
        + b"pixel-data"
    )
    path.write_bytes(content)


class TestAtomicSensorRules(omni.kit.test.AsyncTestCase):
    """Validate shared structure and requirement isolation."""

    async def test_whole_stage_requires_resolving_default_prim(self) -> None:
        """An unmasked asset with no or stale defaultPrim fails ASSET-ROOT-003."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Scope.Define(stage, _ROOT)
        self.assertEqual(len(_failure_messages(Usd.Stage.CreateInMemory(), "ASSET-ROOT-003")), 1)
        self.assertEqual(len(_failure_messages(stage, "ASSET-ROOT-003")), 1)

        stage.GetRootLayer().defaultPrim = "Missing"
        self.assertEqual(len(_failure_messages(stage, "ASSET-ROOT-003")), 1)

        stage.SetDefaultPrim(stage.GetPrimAtPath(_ROOT))
        self.assertEqual(_failure_messages(stage, "ASSET-ROOT-003"), [])

    async def test_default_prim_must_be_xform(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Scope.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())

        self.assertEqual(len(_failure_messages(stage, "ASSET-ROOT-004")), 1)
        self.assertEqual(_failure_messages(stage, "ASSET-ROOT-005", "ASSET-ROOT-006", "ASSET-ROOT-007"), [])

    async def test_default_prim_transform_components_are_independent(self) -> None:
        for add_op, value, requirement_id in (
            ("AddTranslateOp", Gf.Vec3d(1, 0, 0), "ASSET-ROOT-005"),
            ("AddRotateXYZOp", Gf.Vec3f(0, 0, 30), "ASSET-ROOT-006"),
            ("AddScaleOp", Gf.Vec3f(1, 2, 1), "ASSET-ROOT-007"),
        ):
            stage = Usd.Stage.CreateInMemory()
            root = UsdGeom.Xform.Define(stage, _ROOT)
            stage.SetDefaultPrim(root.GetPrim())
            getattr(root, add_op)().Set(value)

            issues = _validate(stage, "ASSET-ROOT-005", "ASSET-ROOT-006", "ASSET-ROOT-007")
            self.assertEqual([issue.requirement for issue in issues], [get_requirement(requirement_id)])

    async def test_default_prim_transform_checks_time_samples(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        translation = root.AddTranslateOp()
        translation.Set(Gf.Vec3d(0), 1)
        translation.Set(Gf.Vec3d(1, 0, 0), 2)

        issues = _validate(stage, "ASSET-ROOT-005")
        self.assertEqual(len(issues), 1)
        self.assertIn("time 2", issues[0].message)

    async def test_default_prim_shear_is_not_mistaken_for_identity(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        root.AddTransformOp().Set(Gf.Matrix4d(1, 0.5, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1))

        self.assertTrue(_failure_messages(stage, "ASSET-ROOT-006", "ASSET-ROOT-007"))

    async def test_default_prim_excludes_kit_camera_and_render_product(self) -> None:
        """Viewport prims outside the authored asset do not cause or satisfy sensor checks."""
        stage = _camera_stage()
        viewport = UsdGeom.Camera.Define(stage, "/OmniverseKit_Persp").GetPrim()
        viewport_product = UsdRender.Product.Define(stage, "/OmniverseKit_ViewportProduct")
        viewport_product.CreateCameraRel().SetTargets([viewport.GetPath()])
        viewport_product.CreateResolutionAttr(Gf.Vec2i(0, 0))

        issues = _validate(
            stage,
            "ASSET-STRUCT-003",
            "RENDER-ASSOC-003",
            "RENDER-PRODUCT-002",
            "RENDER-VAR-001",
            "SENSOR-SCHEMA-001",
            "CAMERA-OUTPUT-001",
        )
        self.assertEqual(issues, [])

        stage.RemovePrim(_PRODUCT)
        issues = _validate(stage, "ASSET-STRUCT-003", "RENDER-ASSOC-003", "RENDER-PRODUCT-002")
        self.assertEqual(
            {issue.requirement for issue in issues},
            {get_requirement("ASSET-STRUCT-003"), get_requirement("RENDER-ASSOC-003")},
        )
        self.assertNotIn("OmniverseKit", " ".join(issue.message for issue in issues))

    async def test_multiple_sensors_and_products_are_allowed(self) -> None:
        """Shared rules do not reintroduce the legacy exactly-one global cardinality."""
        stage = _camera_stage(second_camera=True)

        self.assertEqual(
            _failure_messages(
                stage,
                "CAMERA-STRUCT-001",
                "ASSET-STRUCT-003",
                "RENDER-ASSOC-001",
                "RENDER-ASSOC-003",
                "CAMERA-OUTPUT-001",
            ),
            [],
        )

    async def test_one_render_product_satisfies_presence_but_uncovered_sensor_warns(self) -> None:
        """One covered sensor suffices; another uncovered sensor is advisory only."""
        stage = _camera_stage()
        UsdGeom.Camera.Define(stage, _ROOT.AppendChild("SecondCamera"))

        issues = _validate(stage, "ASSET-STRUCT-003", "RENDER-ASSOC-003")

        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)
        self.assertIn("/SensorAsset/SecondCamera", issues[0].message)
        self.assertEqual(issues[0].requirement, get_requirement("RENDER-ASSOC-003"))

    async def test_no_render_product_fails_presence_and_warns_per_sensor(self) -> None:
        """Missing all coverage is one failure plus one warning per sensor."""
        stage = _camera_stage()
        stage.RemovePrim(_PRODUCT)
        UsdGeom.Camera.Define(stage, _ROOT.AppendChild("SecondCamera"))

        issues = _validate(stage, "ASSET-STRUCT-003", "RENDER-ASSOC-003")

        self.assertEqual([issue.severity for issue in issues].count(usd_validation_nvidia.IssueSeverity.FAILURE), 1)
        self.assertEqual([issue.severity for issue in issues].count(usd_validation_nvidia.IssueSeverity.WARNING), 2)

    async def test_render_product_presence_uses_selected_prim_subtree(self) -> None:
        """A product elsewhere on the stage cannot satisfy a selected prim's scope."""
        stage = Usd.Stage.CreateInMemory()
        root = Sdf.Path("/SensorAsset")
        for group_name in ("Covered", "Uncovered"):
            group = root.AppendChild(group_name)
            UsdGeom.Scope.Define(stage, group)
            UsdGeom.Camera.Define(stage, group.AppendChild("Camera"))
        covered = root.AppendChild("Covered")
        product = UsdRender.Product.Define(stage, covered.AppendChild("RenderProduct"))
        product.CreateCameraRel().SetTargets([covered.AppendChild("Camera")])
        mask = Usd.StagePopulationMask()
        mask.Add(root.AppendChild("Uncovered"))
        stage.SetPopulationMask(mask)

        issues = _validate(stage, "ASSET-STRUCT-003", "RENDER-ASSOC-003")

        self.assertEqual(len(issues), 2)
        self.assertEqual(
            {issue.severity for issue in issues},
            {usd_validation_nvidia.IssueSeverity.FAILURE, usd_validation_nvidia.IssueSeverity.WARNING},
        )
        self.assertNotIn("/SensorAsset/Covered/Camera", " ".join(issue.message for issue in issues))

    async def test_selected_prim_overrides_default_prim(self) -> None:
        """An explicit population mask selects its subtree even when defaultPrim is valid."""
        stage = _camera_stage()
        selected = Sdf.Path("/SelectedAsset")
        UsdGeom.Camera.Define(stage, selected.AppendChild("Camera"))
        mask = Usd.StagePopulationMask()
        mask.Add(selected)
        stage.SetPopulationMask(mask)

        issues = _validate(
            stage,
            "ASSET-ROOT-003",
            "ASSET-ROOT-004",
            "ASSET-ROOT-005",
            "ASSET-ROOT-006",
            "ASSET-ROOT-007",
            "ASSET-STRUCT-003",
            "RENDER-ASSOC-003",
        )

        self.assertEqual(
            {issue.requirement for issue in issues},
            {get_requirement("ASSET-STRUCT-003"), get_requirement("RENDER-ASSOC-003")},
        )

    async def test_render_graph_rules_report_independent_defects(self) -> None:
        """Target cardinality, existence, and resolution can be enabled independently."""
        stage = _camera_stage()
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        product.GetCameraRel().SetTargets([Sdf.Path("/Missing"), _CAMERA])
        product.GetResolutionAttr().Set(Gf.Vec2i(0, 1080))

        cardinality = _failure_messages(stage, "RENDER-ASSOC-001")
        existence = _failure_messages(stage, "RENDER-ASSOC-002")
        resolution = _failure_messages(stage, "RENDER-PRODUCT-002")
        self.assertEqual(len(cardinality), 1)
        self.assertEqual(len(existence), 1)
        self.assertEqual(len(resolution), 1)

    async def test_duplicate_and_empty_source_names_are_atomic(self) -> None:
        """Duplicate and empty sourceName failures belong to separate requirements."""
        stage = _camera_stage()
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        duplicate = UsdRender.Var.Define(stage, product.GetPath().AppendChild("DuplicateLdrColor"))
        duplicate.CreateSourceNameAttr("LdrColor")
        product.GetOrderedVarsRel().AddTarget(duplicate.GetPath())
        empty = _add_render_var(stage, product, "Temporary")
        UsdRender.Var(empty).GetSourceNameAttr().Set("")

        self.assertEqual(len(_failure_messages(stage, "RENDER-VAR-004")), 1)
        self.assertEqual(len(_failure_messages(stage, "RENDER-VAR-005")), 1)

    async def test_all_variants_are_validated(self) -> None:
        """Variant traversal catches an invalid unselected resolution on the default root."""
        stage = _camera_stage(kit_first=True)
        self.assertNotEqual(stage.GetPseudoRoot().GetChildren()[0].GetPath(), _ROOT)
        root = stage.GetPrimAtPath(_ROOT)
        resolution = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT)).GetResolutionAttr()
        resolution.Clear()
        variants = root.GetVariantSets().AddVariantSet("Configuration")
        for name, value in (("Valid", Gf.Vec2i(1920, 1080)), ("Invalid", Gf.Vec2i(0, 1080))):
            variants.AddVariant(name)
            variants.SetVariantSelection(name)
            with variants.GetVariantEditContext():
                resolution.Set(value)
        variants.SetVariantSelection("Valid")

        self.assertEqual(len(_failure_messages(stage, "RENDER-PRODUCT-002")), 1)

    async def test_nested_variant_is_validated_once(self) -> None:
        """An invalid variant on a child RenderProduct is checked without duplicate issues."""
        stage = _camera_stage()
        product = stage.GetPrimAtPath(_PRODUCT)
        resolution = UsdRender.Product(product).GetResolutionAttr()
        resolution.Clear()
        variants = product.GetVariantSets().AddVariantSet("Resolution")
        for name, value in (("Valid", Gf.Vec2i(1920, 1080)), ("Invalid", Gf.Vec2i(0, 1080))):
            variants.AddVariant(name)
            variants.SetVariantSelection(name)
            with variants.GetVariantEditContext():
                resolution.Set(value)
        variants.SetVariantSelection("Valid")

        self.assertEqual(len(_failure_messages(stage, "RENDER-PRODUCT-002")), 1)

    async def test_selected_feature_scopes_shared_sensor_type(self) -> None:
        """A Lidar asset cannot satisfy Camera.RGB's camera-presence requirement."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Scope.Define(stage, _ROOT)
        lidar = stage.DefinePrim(_ROOT.AppendChild("Lidar"), "OmniLidar")
        lidar.ApplyAPI("OmniSensorAPI")
        product = UsdRender.Product.Define(stage, _PRODUCT)
        product.CreateCameraRel().SetTargets([lidar.GetPath()])
        product.CreateResolutionAttr(Gf.Vec2i(1920, 1080))
        _add_render_var(stage, product, "LdrColor")

        issues = _validate_feature(stage, "Camera.RGB")
        presence = [issue for issue in issues if issue.requirement == get_requirement("CAMERA-STRUCT-001")]
        coverage = [issue for issue in issues if issue.requirement == get_requirement("ASSET-STRUCT-003")]
        self.assertEqual(len(presence), 1)
        self.assertEqual(len(coverage), 1)
        self.assertIn("Camera prim", presence[0].message)


class TestDepthRules(omni.kit.test.AsyncTestCase):
    """Validate the RenderProduct-scoped depth schema."""

    async def test_valid_depth_product_passes_schema_outputs_and_bounds(self) -> None:
        """A depth API on RenderProduct with required AOVs passes representative rules."""
        stage = _camera_stage()
        _add_depth(stage)

        self.assertEqual(
            _failure_messages(
                stage,
                "DEPTH-SCHEMA-001",
                "DEPTH-OUTPUT-001",
                "DEPTH-OUTPUT-002",
                "DEPTH-OUTPUT-003",
                "DEPTH-OUTPUT-004",
                "DEPTH-PARAM-002",
                "DEPTH-PARAM-012",
                "DEPTH-PARAM-013",
            ),
            [],
        )

    async def test_depth_schema_is_not_expected_on_camera(self) -> None:
        """The validator follows schema applicability and accepts the API on RenderProduct."""
        stage = _camera_stage()
        product = _add_depth(stage)

        self.assertTrue(product.HasAPI("OmniSensorDepthSensorSingleViewAPI"))
        self.assertFalse(stage.GetPrimAtPath(_CAMERA).HasAPI("OmniSensorDepthSensorSingleViewAPI"))
        self.assertEqual(_failure_messages(stage, "DEPTH-SCHEMA-001"), [])

    async def test_missing_depth_output_lists_supported_source_names(self) -> None:
        """An input-only depth product reports the supported output RenderVars."""
        stage = _camera_stage()
        product = UsdRender.Product(_add_depth(stage))
        product.GetOrderedVarsRel().RemoveTarget(_PRODUCT.AppendChild("DepthSensorDistance"))

        messages = _failure_messages(stage, "DEPTH-OUTPUT-004")
        self.assertEqual(len(messages), 1)
        for name in (
            "DepthSensorDistance",
            "DepthSensorImager",
            "DepthSensorPointCloudColor",
            "DepthSensorPointCloudPosition",
            "DepthSensorOutputVisualization",
        ):
            self.assertIn(name, messages[0])
        self.assertIn("found sourceNames", messages[0])

    async def test_depth_parameter_failures_are_independent(self) -> None:
        """An invalid focal length does not make unrelated confidence validation fail."""
        stage = _camera_stage()
        product = _add_depth(stage)
        product.GetAttribute("omni:rtx:post:depthSensor:focalLengthPixel").Set(0.0)

        self.assertEqual(len(_failure_messages(stage, "DEPTH-PARAM-002")), 1)
        self.assertEqual(_failure_messages(stage, "DEPTH-PARAM-013"), [])


class TestCameraIspRules(omni.kit.test.AsyncTestCase):
    """Validate Generic Camera Core ISP authoring."""

    async def test_default_isp_schema_identity_and_pipeline_pass(self) -> None:
        """Built-in API defaults satisfy identity and graph requirements."""
        stage = _camera_stage()
        _add_isp(stage)

        self.assertEqual(
            _failure_messages(
                stage,
                "CAMERA-ISP-001",
                "CAMERA-ISP-002",
                "CAMERA-ISP-003",
                "CAMERA-ISP-004",
                "CAMERA-ISP-005",
                "CAMERA-ISP-007",
                "CAMERA-ISP-024",
                "CAMERA-ISP-026",
            ),
            [],
        )

    async def test_crbc_remains_schema_valid_despite_renderer_conflict(self) -> None:
        """The documented renderer defect is tracked as a conflict, not an asset rule."""
        stage = _camera_stage()
        camera = _add_isp(stage)
        camera.GetAttribute("omni:sensor:core:colorFilterArrayCfaSemantic").Set("CRBC")

        self.assertEqual(_failure_messages(stage, "CAMERA-ISP-014"), [])

    async def test_isp_requires_maintained_output_but_allows_debug_outputs(self) -> None:
        """Debug outputs can coexist with, but cannot replace, PreIsp or PostIsp."""
        stage = _camera_stage()
        _add_isp(stage)
        product = UsdRender.Product(stage.GetPrimAtPath(_PRODUCT))
        _add_render_var(stage, product, "IspModelGenericA")
        self.assertEqual(_failure_messages(stage, "CAMERA-ISP-005"), [])
        product.GetOrderedVarsRel().RemoveTarget(_PRODUCT.AppendChild("OmniCameraSensorPostIsp"))
        messages = _failure_messages(stage, "CAMERA-ISP-005")
        self.assertEqual(len(messages), 1)
        self.assertIn("OmniCameraSensorPreIsp", messages[0])
        self.assertIn("OmniCameraSensorPostIsp", messages[0])

    async def test_isp_output_is_required_even_when_only_rgb_product_exists(self) -> None:
        stage = _camera_stage()
        stage.GetPrimAtPath(_CAMERA).ApplyAPI("OmniSensorGenericCameraCoreAPI")
        messages = _failure_messages(stage, "CAMERA-ISP-005")
        self.assertEqual(len(messages), 1)
        self.assertIn("OmniCameraSensorPostIsp", messages[0])

    async def test_float_output_precedence_is_a_warning(self) -> None:
        """Enabling both output flags reports Kit's float32 precedence without failure."""
        stage = _camera_stage()
        camera = _add_isp(stage)
        camera.GetAttribute("omni:sensor:core:colorCorrectionOutputFloat16").Set(True)
        camera.GetAttribute("omni:sensor:core:colorCorrectionOutputFloat32").Set(True)

        issues = _validate(stage, "CAMERA-ISP-013")
        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_companding_rules_are_independently_addressable(self) -> None:
        """A repeated x coordinate fails ordering while point count remains valid."""
        stage = _camera_stage()
        camera = _add_isp(stage)
        camera.GetAttribute("omni:sensor:core:compandingLinearCompandCoeff").Set([Gf.Vec2f(0, 0), Gf.Vec2f(0, 1)])

        self.assertEqual(_failure_messages(stage, "CAMERA-ISP-024"), [])
        self.assertEqual(len(_failure_messages(stage, "CAMERA-ISP-026")), 1)

    async def test_hdr_gains_and_thresholds_have_independent_zero_bounds(self) -> None:
        """A zero threshold is valid, but a zero gain would be a divisor."""
        stage = _camera_stage()
        camera = _add_isp(stage)
        segments = camera.GetAttribute("omni:sensor:core:noiseHdrCombinationData")

        segments.Set([Gf.Vec2f(0.0, 0.0)])
        self.assertEqual(len(_failure_messages(stage, "CAMERA-ISP-019")), 1)
        self.assertEqual(_failure_messages(stage, "CAMERA-ISP-020"), [])

        segments.Set([Gf.Vec2f(1.0, -1.0)])
        self.assertEqual(_failure_messages(stage, "CAMERA-ISP-019"), [])
        self.assertEqual(len(_failure_messages(stage, "CAMERA-ISP-020")), 1)

    async def test_rgb_and_isp_products_can_share_a_camera(self) -> None:
        """Feature-specific product checks do not impose both pipelines on both products."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniSensorGenericCameraCoreAPI")
        camera.GetAttribute("omni:sensor:modelName").Set("CameraCore")
        camera.GetAttribute("omni:sensor:modelVendor").Set("NVIDIA")
        camera.GetAttribute("omni:sensor:marketName").Set("Generic")
        isp_product = UsdRender.Product.Define(stage, _ROOT.AppendChild("IspProduct"))
        isp_product.CreateCameraRel().SetTargets([_CAMERA])
        isp_product.CreateResolutionAttr(Gf.Vec2i(1920, 1080))
        _add_render_var(stage, isp_product, "HdrColor")
        _add_render_var(stage, isp_product, "OmniCameraSensorPreIsp")

        self.assertEqual(
            _failure_messages(stage, "CAMERA-OUTPUT-001", "CAMERA-ISP-004", "CAMERA-ISP-005"),
            [],
        )


class TestLensAndLutRules(omni.kit.test.AsyncTestCase):
    """Validate lens schema exclusivity and LUT metadata."""

    async def test_exr_header_reader_does_not_read_pixels(self) -> None:
        """Skip unrelated metadata without reading any pixel payload."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lut.exr"
            _minimal_exr(path, channels=("R", "G"))
            content = path.read_bytes()
            pixel_start = content.index(b"pixel-data")
            content = (
                content[: pixel_start - 1]
                + _exr_attribute("comment", "string", b"ignored")
                + content[pixel_start - 1 :]
            )
            path.write_bytes(content)
            pixel_start = content.index(b"pixel-data")

            class HeaderOnlyStream(io.BytesIO):
                def read(self, size: int = -1) -> bytes:
                    if size < 0 or self.tell() + size > pixel_start:
                        raise AssertionError("OpenEXR reader reached pixel data")
                    return super().read(size)

            with patch.object(Path, "open", return_value=HeaderOnlyStream(content)):
                header = _read_exr_header(path)
            self.assertEqual(header, {"channels": {"R": 2, "G": 2}, "compression": 3, "width": 4, "height": 2})

    async def test_exr_header_reader_rejects_truncated_attribute(self) -> None:
        """Skipping an unrelated attribute still checks its declared size."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truncated.exr"
            path.write_bytes(struct.pack("<II", 20000630, 2) + b"comment\0string\0" + struct.pack("<I", 4) + b"x")
            with self.assertRaisesRegex(ValueError, "truncated OpenEXR attribute payload"):
                _read_exr_header(path)

    async def test_multiple_lens_apis_fail_exclusivity(self) -> None:
        """Two composed lens APIs are rejected without depending on namespace placement."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionOpenCvFisheyeAPI")
        camera.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")

        self.assertEqual(len(_failure_messages(stage, "CAMERA-LENS-001")), 1)

    async def test_model_token_and_focal_bounds_are_atomic(self) -> None:
        """A zero fx and wrong model token report under separate requirements."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
        camera.GetAttribute("omni:lensdistortion:model").Set("opencvFisheye")
        camera.GetAttribute("omni:lensdistortion:opencvPinhole:fx").Set(0.0)

        self.assertEqual(len(_failure_messages(stage, "CAMERA-LENS-002")), 1)
        self.assertEqual(len(_failure_messages(stage, "CAMERA-LENS-011")), 1)
        self.assertEqual(_failure_messages(stage, "CAMERA-LENS-012"), [])

    async def test_kannala_brandt_uses_renderer_polynomial_convention(self) -> None:
        """The schema defaults pass the renderer's theta*(1+k0*theta^2+...) map."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionKannalaBrandtK3API")

        self.assertEqual(
            _failure_messages(stage, "CAMERA-LENS-018", "CAMERA-LENS-019"),
            [],
        )

    async def test_radtan_radial_and_inverse_jacobian_rules_are_atomic(self) -> None:
        """A singular thin-prism inverse does not invalidate the radial map rule."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionRadTanThinPrismAPI")
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:nominalWidth").Set(2.0)
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:nominalHeight").Set(2.0)
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:opticalCenter").Set(Gf.Vec2f(1, 1))
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:p0").Set(0.0)
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:p1").Set(0.0)
        for index in range(4):
            camera.GetAttribute(f"omni:lensdistortion:radTanThinPrism:s{index}").Set(0.0)
        camera.GetAttribute("omni:lensdistortion:radTanThinPrism:s0").Set(-1.0)

        self.assertEqual(_failure_messages(stage, "CAMERA-LENS-020"), [])
        self.assertEqual(len(_failure_messages(stage, "CAMERA-LENS-023")), 1)

    async def test_lut_header_channel_type_and_compression(self) -> None:
        """Static LUT validation reads OpenEXR channel and compression metadata."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionLutAPI")
        with tempfile.TemporaryDirectory() as directory:
            enter_path = Path(directory) / "enter.exr"
            exit_path = Path(directory) / "exit.exr"
            _minimal_exr(enter_path)
            _minimal_exr(exit_path, channels=("R", "G"))
            camera.GetAttribute("omni:lensdistortion:lut:rayEnterDirectionTexture").Set(Sdf.AssetPath(str(enter_path)))
            camera.GetAttribute("omni:lensdistortion:lut:rayExitPositionTexture").Set(Sdf.AssetPath(str(exit_path)))

            self.assertEqual(
                _failure_messages(
                    stage,
                    "CAMERA-LUT-001",
                    "CAMERA-LUT-002",
                    "CAMERA-LUT-003",
                    "CAMERA-LUT-004",
                    "CAMERA-LUT-005",
                    "CAMERA-LUT-006",
                    "CAMERA-LUT-007",
                    "CAMERA-LUT-010",
                    "CAMERA-LUT-012",
                    "CAMERA-LUT-015",
                    "CAMERA-LUT-021",
                ),
                [],
            )
            decode_issues = _validate(stage, "CAMERA-LUT-003", "CAMERA-LUT-004")
            self.assertEqual(len(decode_issues), 2)
            self.assertTrue(
                all(issue.severity == usd_validation_nvidia.IssueSeverity.WARNING for issue in decode_issues)
            )

    async def test_lut_compression_failure_names_supported_formats(self) -> None:
        """An unsupported OpenEXR compression reports accepted codes and names."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionLutAPI")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "enter.exr"
            _minimal_exr(path, compression=5)
            camera.GetAttribute("omni:lensdistortion:lut:rayEnterDirectionTexture").Set(Sdf.AssetPath(str(path)))

            messages = _failure_messages(stage, "CAMERA-LUT-005")
            self.assertEqual(len(messages), 1)
            self.assertIn("0 (NO_COMPRESSION)", messages[0])
            self.assertIn("4 (PIZ)", messages[0])

    async def test_optional_lut_alpha_requires_pixel_review_only_when_present(self) -> None:
        """Header checks can detect A, while compressed texels require decoding review."""
        stage = _camera_stage()
        camera = stage.GetPrimAtPath(_CAMERA)
        camera.ApplyAPI("OmniLensDistortionLutAPI")
        with tempfile.TemporaryDirectory() as directory:
            enter_path = Path(directory) / "enter.exr"
            _minimal_exr(enter_path)
            camera.GetAttribute("omni:lensdistortion:lut:rayEnterDirectionTexture").Set(Sdf.AssetPath(str(enter_path)))
            self.assertEqual(_validate(stage, "CAMERA-LUT-032"), [])

            _minimal_exr(enter_path, channels=("R", "G", "B", "A"))
            issues = _validate(stage, "CAMERA-LUT-032")
            self.assertEqual(len(issues), 1)
            self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_camera_capability_requirements_are_registered(self) -> None:
        """Every non-SPG camera capability row has a validator binding."""
        registry = usd_validation_nvidia.RequirementsRegistry()
        capability_names = (
            "Shippable Asset Structure",
            "Render Graph",
            "Core Sensor Contract",
            "Camera Core",
            "RGB Output",
            "Camera Lens Calibration",
            "Depth Single-View Output",
            "Generic Camera Core ISP",
        )
        missing = [
            requirement.code
            for capability_name in capability_names
            for requirement in CAPABILITIES[capability_name].requirements
            if not registry.is_implemented(requirement)
        ]

        self.assertEqual(missing, [])
