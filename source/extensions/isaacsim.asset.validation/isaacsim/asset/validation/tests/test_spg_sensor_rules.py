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

"""Tests for atomic RTX Sensor Processing Graph authored-asset rules."""

from __future__ import annotations

import tempfile
from pathlib import Path

import omni.kit.test
import usd_validation_nvidia
from isaacsim.asset.validation.sensor_capabilities import CAPABILITIES
from isaacsim.asset.validation.sensor_rule_utils import RULES_BY_REQUIREMENT_ID
from isaacsim.asset.validation.spg_sensor_rules import _launch_path
from pxr import Sdf, Usd, UsdGeom, UsdRender, UsdShade

from .utils import failures as _failures
from .utils import validate as _validate

_ROOT = Sdf.Path("/Asset")
_PRODUCT = _ROOT.AppendChild("Product")
_SOURCE_VAR = _PRODUCT.AppendChild("LdrColor")
_SHADER = _PRODUCT.AppendChild("Kernel")
_OUTPUT_VAR = _PRODUCT.AppendChild("Filtered")
_SPG_REQUIREMENT_IDS = tuple(
    requirement.code.rsplit(".", 1)[-1] for requirement in CAPABILITIES["SPG Post-Processing"].requirements
)


def _write_source_pair(directory: Path, *, entry: str = "kernel") -> Path:
    source = directory / "Kernel.cu"
    source.write_text(f'extern "C" __global__ void {entry}(int value) {{}}\n', encoding="utf-8")
    source.with_suffix(".cu.lua").write_text(
        f"function {entry}(inputs, outputs)\n"
        '    outputs["Filtered"] = cuda.image(1, 1, cuda.uchar4)\n'
        '    return cuda.kernel({args = {cuda.TextureObject(inputs["LdrColor"]), '
        'cuda.SurfaceObject(outputs["Filtered"])}})\n'
        "end\n",
        encoding="utf-8",
    )
    return source


def _add_source_shader(stage: Usd.Stage, path: Sdf.Path, source: Path, *, entry: str = "kernel") -> Usd.Prim:
    shader = UsdShade.Shader.Define(stage, path).GetPrim()
    shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("sourceAsset")
    shader.CreateAttribute("info:spg:sourceAsset", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(source)))
    shader.CreateAttribute("info:spg:sourceAsset:subIdentifier", Sdf.ValueTypeNames.Token).Set(entry)
    return shader


def _valid_stage(directory: Path) -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Scope.Define(stage, _ROOT)
    UsdRender.Product.Define(stage, _PRODUCT)

    source_var = UsdRender.Var.Define(stage, _SOURCE_VAR)
    source_var.CreateSourceNameAttr("LdrColor")
    source_aov = source_var.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque)

    shader = _add_source_shader(stage, _SHADER, _write_source_pair(directory))
    shader_input = shader.CreateAttribute("inputs:LdrColor", Sdf.ValueTypeNames.Opaque)
    shader_input.SetConnections([source_aov.GetPath()])
    shader_output = shader.CreateAttribute("outputs:Filtered", Sdf.ValueTypeNames.Opaque)

    output_var = UsdRender.Var.Define(stage, _OUTPUT_VAR)
    output_var.CreateSourceNameAttr("Filtered")
    output_aov = output_var.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque)
    output_aov.SetConnections([shader_output.GetPath()])
    return stage


class TestSpgSensorRules(omni.kit.test.AsyncTestCase):
    """Exercise classification, resources, topology, and source evidence."""

    async def test_default_prim_excludes_unrelated_spg_metadata(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Scope.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        material = UsdShade.Material.Define(stage, "/OmniverseKit_Viewport/Material")
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Shader")).GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")

        self.assertEqual(_failures(stage, "SPG-001"), [])

    async def test_every_spg_requirement_has_an_atomic_checker(self) -> None:
        """All 32 selected SPG rows are bound to distinct rule classes."""
        requirement_ids = set(_SPG_REQUIREMENT_IDS)

        self.assertEqual(len(requirement_ids), 32)
        self.assertTrue(requirement_ids <= RULES_BY_REQUIREMENT_ID.keys())
        self.assertEqual(
            len({RULES_BY_REQUIREMENT_ID[requirement_id] for requirement_id in requirement_ids}),
            len(requirement_ids),
        )

    async def test_valid_cuda_pipeline_passes_all_spg_failures(self) -> None:
        """A renderer-shaped source/SPG/RenderVar chain satisfies the complete capability."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))

            self.assertEqual(_failures(stage, *_SPG_REQUIREMENT_IDS), [])

    async def test_material_shader_is_not_misclassified_as_spg(self) -> None:
        """The identification rule reports SPG metadata hidden below a Material."""
        stage = Usd.Stage.CreateInMemory()
        material = UsdShade.Material.Define(stage, "/Asset/Looks/Material")
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Shader")).GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")

        issues = _failures(stage, "SPG-001")
        self.assertEqual(len(issues), 1)
        self.assertIn("not eligible for SPG population", issues[0].message)

    async def test_instance_proxy_shader_is_not_eligible_for_spg_population(self) -> None:
        """Detect SPG metadata inside an instance even though ordinary traversal skips its proxy."""
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, _ROOT)
        stage.SetDefaultPrim(root.GetPrim())
        prototype = UsdGeom.Xform.Define(stage, "/Asset/Source")
        shader = UsdShade.Shader.Define(stage, prototype.GetPath().AppendChild("Kernel")).GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        instance = UsdGeom.Xform.Define(stage, "/Asset/Instance").GetPrim()
        instance.GetReferences().AddInternalReference(prototype.GetPath())
        instance.SetInstanceable(True)

        issues = _failures(stage, "SPG-001")
        self.assertEqual(len(issues), 1)
        self.assertIn("/Asset/Instance/Kernel", issues[0].message)

    async def test_dormant_source_metadata_does_not_conflict_with_selected_id_form(
        self,
    ) -> None:
        """Kit selects implementationSource=id even when dormant source metadata is present."""
        stage = Usd.Stage.CreateInMemory()
        shader = UsdShade.Shader.Define(stage, "/Asset/BuiltIn").GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        shader.CreateAttribute("info:spg:sourceAsset", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("unused.cu"))

        self.assertEqual(_failures(stage, "SPG-001", "SPG-008", "SPG-009"), [])

    async def test_source_asset_requirements_are_independent(self) -> None:
        """Extension, existence, launch, and subIdentifier defects remain independently addressable."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "Kernel.txt"
            source.write_text("not a kernel", encoding="utf-8")
            stage = Usd.Stage.CreateInMemory()
            shader = _add_source_shader(stage, "/Asset/Kernel", source, entry="")

            self.assertEqual(_failures(stage, "SPG-009"), [])
            extension_issues = _failures(stage, "SPG-010")
            self.assertEqual(len(extension_issues), 1)
            self.assertIn("cubin", extension_issues[0].message)
            self.assertIn("slang", extension_issues[0].message)
            self.assertEqual(len(_failures(stage, "SPG-011")), 1)
            self.assertEqual(len(_failures(stage, "SPG-012")), 1)
            self.assertTrue(shader)

    async def test_package_launch_suffix_stays_inside_brackets(self) -> None:
        """Resolve the Lua companion relative to a packaged source asset."""
        self.assertEqual(
            _launch_path("pkg.usdz[inner/Kernel.cu]"),
            "pkg.usdz[inner/Kernel.cu.lua]",
        )

    async def test_resource_cardinality_type_and_source_are_atomic(self) -> None:
        """One malformed resource input reports through its three independent contracts."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            shader_input = stage.GetAttributeAtPath(_SHADER.AppendProperty("inputs:LdrColor"))
            shader_input.SetTypeName(Sdf.ValueTypeNames.Float)
            shader_input.SetConnections(
                [
                    _SOURCE_VAR.AppendProperty("omni:rtx:aov"),
                    Sdf.Path("/Missing.outputs:Color"),
                ]
            )

            self.assertEqual(len(_failures(stage, "SPG-014")), 1)
            self.assertEqual(len(_failures(stage, "SPG-015")), 1)
            self.assertEqual(len(_failures(stage, "SPG-016")), 1)

    async def test_missing_render_var_aov_is_detected_from_connection_intent(
        self,
    ) -> None:
        """A connection to a missing RenderVar AOV still activates SPG-013."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source_var = stage.GetPrimAtPath(_SOURCE_VAR)
            source_var.RemoveProperty("omni:rtx:aov")
            shader_input = stage.GetAttributeAtPath(_SHADER.AppendProperty("inputs:LdrColor"))
            shader_input.SetConnections([_SOURCE_VAR.AppendProperty("omni:rtx:aov")])

            self.assertEqual(len(_failures(stage, "SPG-013")), 1)

    async def test_typed_value_connections_require_matching_types(self) -> None:
        """Typed-input source cardinality and type compatibility do not depend on resources."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source = stage.DefinePrim("/Asset/Values", "Scope")
            int_source = source.CreateAttribute("intValue", Sdf.ValueTypeNames.Int)
            float_source = source.CreateAttribute("floatValue", Sdf.ValueTypeNames.Float)
            typed_input = stage.GetPrimAtPath(_SHADER).CreateAttribute("inputs:gain", Sdf.ValueTypeNames.Float)
            typed_input.SetConnections([int_source.GetPath(), float_source.GetPath()])

            self.assertEqual(len(_failures(stage, "SPG-017")), 1)
            self.assertEqual(len(_failures(stage, "SPG-018")), 1)

    async def test_legacy_parameter_namespace_warns_without_failing(self) -> None:
        """Keep renderer-supported `params:*` values accepted but deprecated."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            shader = stage.GetPrimAtPath(_SHADER)
            shader.CreateAttribute("params:sigma", Sdf.ValueTypeNames.Float).Set(1.0)

            issues = _validate(stage, "SPG-019")
            self.assertEqual(len(issues), 1)
            self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)
            self.assertIn("params:sigma", issues[0].message)

    async def test_duplicate_component_names_and_cycle_are_detected(self) -> None:
        """Cross-hierarchy edges form one component for name and cycle checks."""
        stage = Usd.Stage.CreateInMemory()
        first = UsdShade.Shader.Define(stage, "/Asset/A/Node").GetPrim()
        second = UsdShade.Shader.Define(stage, "/Asset/B/Node").GetPrim()
        for shader in (first, second):
            shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
            shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
            shader.CreateAttribute("inputs:X", Sdf.ValueTypeNames.Opaque)
            shader.CreateAttribute("outputs:X", Sdf.ValueTypeNames.Opaque)
        first.GetAttribute("inputs:X").SetConnections([second.GetAttribute("outputs:X").GetPath()])
        second.GetAttribute("inputs:X").SetConnections([first.GetAttribute("outputs:X").GetPath()])

        self.assertEqual(len(_failures(stage, "SPG-007")), 1)
        self.assertEqual(len(_failures(stage, "SPG-032")), 1)

    async def test_cross_product_output_binding_warns_without_rejecting(self) -> None:
        """A cross-product output is supported but receives the renderer staleness warning."""
        stage = Usd.Stage.CreateInMemory()
        first_product = UsdRender.Product.Define(stage, "/Asset/First")
        second_product = UsdRender.Product.Define(stage, "/Asset/Second")
        shader = UsdShade.Shader.Define(stage, first_product.GetPath().AppendChild("Node")).GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        output = shader.CreateAttribute("outputs:X", Sdf.ValueTypeNames.Opaque)
        render_var = UsdRender.Var.Define(stage, second_product.GetPath().AppendChild("Result"))
        render_var.CreateSourceNameAttr("Result")
        render_var.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque).SetConnections(
            [output.GetPath()]
        )

        issues = _validate(stage, "SPG-005")
        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_nested_shader_uses_renderer_direct_parent_cross_product_test(
        self,
    ) -> None:
        """The warning mirrors the translator's direct-parent comparison for nested shaders."""
        stage = Usd.Stage.CreateInMemory()
        product = UsdRender.Product.Define(stage, "/Asset/Product")
        UsdGeom.Scope.Define(stage, product.GetPath().AppendChild("Post"))
        shader = UsdShade.Shader.Define(stage, product.GetPath().AppendPath("Post/Node")).GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        output = shader.CreateAttribute("outputs:X", Sdf.ValueTypeNames.Opaque)
        render_var = UsdRender.Var.Define(stage, product.GetPath().AppendChild("Result"))
        render_var.CreateSourceNameAttr("Result")
        render_var.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque).SetConnections(
            [output.GetPath()]
        )

        issues = _validate(stage, "SPG-005")
        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_produced_render_var_contracts_are_independent(self) -> None:
        """Placement and connection count fail while renderer sourceName fallback remains valid."""
        stage = Usd.Stage.CreateInMemory()
        shader = UsdShade.Shader.Define(stage, "/Asset/Node").GetPrim()
        shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
        shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        first = shader.CreateAttribute("outputs:A", Sdf.ValueTypeNames.Opaque)
        second = shader.CreateAttribute("outputs:B", Sdf.ValueTypeNames.Opaque)
        render_var = UsdRender.Var.Define(stage, "/Asset/Result")
        render_var.CreateSourceNameAttr("")
        render_var.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque).SetConnections(
            [first.GetPath(), second.GetPath()]
        )

        self.assertEqual(len(_failures(stage, "SPG-020")), 1)
        self.assertEqual(len(_failures(stage, "SPG-021")), 1)
        self.assertEqual(_failures(stage, "SPG-023"), [])

    async def test_duplicate_spg_output_source_name_warns(self) -> None:
        """Report duplicate SPG AOV names without treating them as invalid assets."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            extra = UsdRender.Var.Define(stage, _PRODUCT.AppendChild("FilteredCopy"))
            extra.CreateSourceNameAttr("Filtered")
            extra.GetPrim().CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque).SetConnections(
                [_SHADER.AppendProperty("outputs:Filtered")]
            )

            issues = _validate(stage, "SPG-024")
            self.assertEqual(len(issues), 1)
            self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_missing_ports_and_reversed_edges_are_reported(self) -> None:
        """Port existence and direction remain distinct internal-edge checks."""
        stage = Usd.Stage.CreateInMemory()
        source = UsdShade.Shader.Define(stage, "/Asset/Source").GetPrim()
        destination = UsdShade.Shader.Define(stage, "/Asset/Destination").GetPrim()
        for shader in (source, destination):
            shader.CreateAttribute("info:implementationSource", Sdf.ValueTypeNames.Token).Set("id")
            shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("spg:rtx.spg.core/NoOp")
        bad_destination = destination.CreateAttribute("outputs:Destination", Sdf.ValueTypeNames.Opaque)
        bad_destination.SetConnections([source.GetPath().AppendProperty("inputs:Missing")])

        self.assertEqual(len(_failures(stage, "SPG-030")), 1)
        self.assertEqual(len(_failures(stage, "SPG-031")), 1)

    async def test_source_and_lua_entry_points_are_checked_separately(self) -> None:
        """CUDA and Lua entry mismatches fail their corresponding requirements."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source = _write_source_pair(Path(temp_dir), entry="actual")
            stage = Usd.Stage.CreateInMemory()
            _add_source_shader(stage, "/Asset/Kernel", source, entry="expected")

            self.assertEqual(len(_failures(stage, "SPG-036")), 1)
            self.assertEqual(len(_failures(stage, "SPG-037")), 1)

    async def test_ptx_visible_entry_is_recognized(self) -> None:
        """The PTX parser accepts the `.visible .entry` form emitted by nvcc."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "Kernel.ptx"
            source.write_text(
                ".visible .entry kernel(\n    .param .u32 value\n)\n{\n    ret;\n}\n",
                encoding="utf-8",
            )
            source.with_suffix(".ptx.lua").write_text("function kernel(inputs, outputs)\nend\n", encoding="utf-8")
            stage = Usd.Stage.CreateInMemory()
            _add_source_shader(stage, "/Asset/Kernel", source)

            self.assertEqual(_failures(stage, "SPG-036"), [])

    async def test_cuda_execution_requires_an_actually_bound_opaque_resource(
        self,
    ) -> None:
        """An output declaration alone is not a valid bound resource for SPG-035."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            stage.GetAttributeAtPath(_SHADER.AppendProperty("inputs:LdrColor")).SetTypeName(Sdf.ValueTypeNames.Float)
            stage.GetAttributeAtPath(_SHADER.AppendProperty("outputs:Filtered")).SetTypeName(Sdf.ValueTypeNames.Float)

            self.assertEqual(len(_failures(stage, "SPG-035")), 1)

    async def test_lua_declared_argument_compatibility(self) -> None:
        """Literal missing attributes and resource/value category mismatches fail SPG-038."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source = _write_source_pair(Path(temp_dir))
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  return cuda.kernel({args={cuda.TextureObject(inputs["gain"]), '
                'cuda.float(params["Missing"])}})\n'
                "end\n",
                encoding="utf-8",
            )
            stage = Usd.Stage.CreateInMemory()
            shader = _add_source_shader(stage, "/Asset/Kernel", source)
            shader.CreateAttribute("inputs:gain", Sdf.ValueTypeNames.Float)

            issues = _failures(stage, "SPG-038")
            self.assertEqual(len(issues), 2)
            self.assertTrue(any("undeclared" in issue.message for issue in issues))
            self.assertTrue(any("expects an opaque inputs:* port" in issue.message for issue in issues))

    async def test_lua_launch_wrappers_enforce_resource_direction(self) -> None:
        """Reject read wrappers on outputs and write wrappers on inputs."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source = Path(temp_dir) / "Kernel.cu"
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  return cuda.kernel({args={cuda.SurfaceObject(inputs["LdrColor"]), '
                'cuda.TextureObject(outputs["Filtered"])}})\n'
                "end\n",
                encoding="utf-8",
            )

            issues = _failures(stage, "SPG-038")
            self.assertEqual(len(issues), 2)
            self.assertTrue(any("opaque outputs:*" in issue.message for issue in issues))
            self.assertTrue(any("opaque inputs:*" in issue.message for issue in issues))

    async def test_lua_unknown_input_warns_pending_implicit_port_contract(self) -> None:
        """Do not reject a name that the renderer may supply implicitly."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source = Path(temp_dir) / "Kernel.cu"
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  return cuda.kernel({args={cuda.float(inputs["ImplicitValue"])}})\n'
                "end\n",
                encoding="utf-8",
            )

            issues = _validate(stage, "SPG-038")
            self.assertEqual(len(issues), 1)
            self.assertEqual(issues[0].severity, usd_validation_nvidia.IssueSeverity.WARNING)

    async def test_slang_parameter_block_accepts_resource_ports(self) -> None:
        """A Slang ParameterBlock may contain resource descriptors as well as values."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source = Path(temp_dir) / "Kernel.cu"
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  return slang.dispatch({bind={slang.ParameterBlock(inputs["LdrColor"])}})\n'
                "end\n",
                encoding="utf-8",
            )

            self.assertEqual(_validate(stage, "SPG-038"), [])

    async def test_lua_non_launch_reference_does_not_constrain_ports(self) -> None:
        """Ignore a table access that is not part of a CUDA launch call."""
        with tempfile.TemporaryDirectory() as temp_dir:
            stage = _valid_stage(Path(temp_dir))
            source = Path(temp_dir) / "Kernel.cu"
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  local optional = inputs["NotALaunchArgument"]\n'
                '  return cuda.kernel({args={cuda.TextureObject(inputs["LdrColor"])}})\n'
                "end\n",
                encoding="utf-8",
            )

            self.assertEqual(_validate(stage, "SPG-038"), [])

    async def test_lua_argument_check_ignores_inactive_entry_points(self) -> None:
        """Ports referenced only by another function do not constrain the selected entry."""
        with tempfile.TemporaryDirectory() as temp_dir:
            source = _write_source_pair(Path(temp_dir))
            source.with_suffix(".cu.lua").write_text(
                "function kernel(inputs, outputs)\n"
                '  outputs["Filtered"] = cuda.image(1, 1, cuda.uchar4)\n'
                "end\n"
                "function unused(inputs, outputs)\n"
                '  return cuda.float(inputs["NotDeclared"])\n'
                "end\n",
                encoding="utf-8",
            )
            stage = Usd.Stage.CreateInMemory()
            shader = _add_source_shader(stage, "/Asset/Kernel", source)
            shader.CreateAttribute("outputs:Filtered", Sdf.ValueTypeNames.Opaque)

            self.assertEqual(_failures(stage, "SPG-038"), [])
