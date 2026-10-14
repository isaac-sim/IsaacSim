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

"""Verify SPG authoring: SPGNode validation and RtxCamera.author_spg USD prim structure."""

import os
import shutil
import tempfile
from unittest import mock

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import RtxCamera, SPGNode
from pxr import Sdf, UsdRender, UsdShade


def _write_kernel(dir_path: str, stem: str, fn_name: str) -> str:
    """Write a stub ``.cu`` kernel and its co-located ``.cu.lua`` launch script.

    Args:
        dir_path: Directory to write into.
        stem: File stem (kernel file becomes ``<stem>.cu``).
        fn_name: Entry-point function name used in both files.

    Returns:
        Path to the written ``.cu`` file.
    """
    cu_path = os.path.join(dir_path, f"{stem}.cu")
    with open(cu_path, "w") as f:
        f.write(f'extern "C" __global__ void {fn_name}() {{}}\n')
    with open(cu_path + ".lua", "w") as f:
        f.write(f"function {fn_name}(inputs, outputs) end\n")
    return cu_path


class TestSPGNode(omni.kit.test.AsyncTestCase):
    """Structural validation performed by SPGNode on construction."""

    async def setUp(self) -> None:
        """Create a temporary directory for stub kernel files."""
        super().setUp()
        self._dir = tempfile.mkdtemp()

    async def tearDown(self) -> None:
        """Remove the temporary directory."""
        shutil.rmtree(self._dir, ignore_errors=True)
        super().tearDown()

    async def test_wrong_suffix_raises(self) -> None:
        """Reject a cuda_kernel path that is not a .cu file."""
        bad = os.path.join(self._dir, "kernel.txt")
        with open(bad, "w"):
            pass
        with self.assertRaises(ValueError):
            SPGNode("K", bad)

    async def test_missing_cu_raises(self) -> None:
        """Reject a cuda_kernel path that does not exist."""
        with self.assertRaises(FileNotFoundError):
            SPGNode("K", os.path.join(self._dir, "missing.cu"))

    async def test_missing_lua_raises(self) -> None:
        """Reject a .cu without a co-located .cu.lua launch script."""
        cu_path = os.path.join(self._dir, "K.cu")
        with open(cu_path, "w"):
            pass
        with self.assertRaises(FileNotFoundError):
            SPGNode("K", cu_path)

    async def test_valid_node_defaults(self) -> None:
        """Construct a valid node; sub_identifier defaults to the node name."""
        cu_path = _write_kernel(self._dir, "GrayscaleKernel", "grayscale")
        node = SPGNode("GrayscaleKernel", cu_path, inputs=["LdrColor"], outputs=["LdrGrayscale"])
        self.assertEqual(node.sub_identifier, "GrayscaleKernel")
        self.assertEqual(node._resolved_cu, str(os.path.realpath(cu_path)))
        self.assertEqual(node._resolved_lua, str(os.path.realpath(cu_path + ".lua")))

    async def test_input_param_name_collision_raises(self) -> None:
        """Reject a node whose input and param share a name (same inputs: namespace)."""
        cu_path = _write_kernel(self._dir, "K", "k")
        with self.assertRaises(ValueError):
            SPGNode("K", cu_path, inputs=["strength"], params={"strength": 1.0})

    async def test_duplicate_output_raises(self) -> None:
        """Reject a node with duplicate output names."""
        cu_path = _write_kernel(self._dir, "K", "k")
        with self.assertRaises(ValueError):
            SPGNode("K", cu_path, outputs=["Out", "Out"])

    async def test_unsupported_param_type_raises(self) -> None:
        """Reject a param whose value is not a bool, int, or float at construction time."""
        cu_path = _write_kernel(self._dir, "K", "k")
        with self.assertRaises(TypeError):
            SPGNode("K", cu_path, params={"x": "bad"})


class TestAuthorSPG(omni.kit.test.AsyncTestCase):
    """USD prim structure authored by RtxCamera.author_spg."""

    async def setUp(self) -> None:
        """Create an empty stage, a camera, and stub grayscale/invert kernels."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        self._stage = stage_utils.get_current_stage(backend="usd")
        self._dir = tempfile.mkdtemp()
        self._grayscale_cu = _write_kernel(self._dir, "GrayscaleKernel", "grayscale")
        self._invert_cu = _write_kernel(self._dir, "InvertKernel", "invert")
        self._cam = RtxCamera("/World/camera")

    async def tearDown(self) -> None:
        """Close the stage and remove the temporary directory."""
        shutil.rmtree(self._dir, ignore_errors=True)
        stage_utils.close_stage()
        super().tearDown()

    def _grayscale_node(self) -> SPGNode:
        return SPGNode(
            "GrayscaleKernel",
            self._grayscale_cu,
            sub_identifier="grayscale",
            inputs=["LdrColor"],
            outputs=["LdrGrayscale"],
        )

    def _invert_node(self) -> SPGNode:
        return SPGNode(
            "InvertKernel",
            self._invert_cu,
            sub_identifier="invert",
            inputs=["Image"],
            outputs=["Inverted"],
            params={"strength": 1.0},
        )

    # -- render product creation --

    async def test_creates_render_product_targeting_sensor(self) -> None:
        """Create a render product whose camera relationship targets the sensor prim."""
        rp_path = self._cam.author_spg(self._grayscale_node())
        self.assertEqual(rp_path, "/World/camera_RenderProduct")
        rp = self._stage.GetPrimAtPath(rp_path)
        self.assertTrue(rp.IsValid())
        self.assertEqual(rp.GetTypeName(), "RenderProduct")
        targets = rp.GetRelationship("camera").GetTargets()
        self.assertEqual([str(t) for t in targets], ["/World/camera"])

    async def test_shader_prim_structure(self) -> None:
        """Author a Shader with SPG source asset, sub-identifier, and opaque ports."""
        rp_path = self._cam.author_spg(self._grayscale_node())
        shader_prim = self._stage.GetPrimAtPath(f"{rp_path}/GrayscaleKernel")
        self.assertTrue(shader_prim.IsValid())
        shader = UsdShade.Shader(shader_prim)
        self.assertEqual(shader.GetImplementationSource(), UsdShade.Tokens.sourceAsset)
        self.assertEqual(shader.GetSourceAsset("spg").path, self._grayscale_cu)
        self.assertEqual(shader.GetSourceAssetSubIdentifier("spg"), "grayscale")
        self.assertEqual(shader_prim.GetAttribute("inputs:LdrColor").GetTypeName(), Sdf.ValueTypeNames.Opaque)
        self.assertEqual(shader_prim.GetAttribute("outputs:LdrGrayscale").GetTypeName(), Sdf.ValueTypeNames.Opaque)

    async def test_typed_params(self) -> None:
        """Author typed params as typed shader inputs with the requested value."""
        rp_path = self._cam.author_spg(self._invert_node())
        shader_prim = self._stage.GetPrimAtPath(f"{rp_path}/InvertKernel")
        strength = shader_prim.GetAttribute("inputs:strength")
        self.assertEqual(strength.GetTypeName(), Sdf.ValueTypeNames.Float)
        self.assertAlmostEqual(strength.Get(), 1.0)

    async def test_unique_names_enforced(self) -> None:
        """Reject two nodes that share a name."""
        with self.assertRaises(ValueError):
            self._cam.author_spg([self._grayscale_node(), self._grayscale_node()])

    async def test_node_name_aov_collision_raises(self) -> None:
        """Reject a node whose name collides with a bare AOV name in connections.

        A shader prim at ``{rp}/{name}`` and a render var at ``{rp}/{aov}`` would
        otherwise be authored at the same prim path.
        """
        node = SPGNode(
            "LdrColor",  # collides with the bare "LdrColor" AOV endpoint below
            self._grayscale_cu,
            sub_identifier="grayscale",
            inputs=["LdrColor"],
            outputs=["LdrGrayscale"],
        )
        with self.assertRaises(ValueError):
            self._cam.author_spg(
                node,
                connections=[("LdrColor", "LdrColor.inputs:LdrColor")],
            )

    # -- connections --

    async def test_chained_connections(self) -> None:
        """Wire a grayscale -> invert chain and the input/output render vars."""
        rp_path = self._cam.author_spg(
            [self._grayscale_node(), self._invert_node()],
            connections=[
                ("LdrColor", "GrayscaleKernel.inputs:LdrColor"),
                ("GrayscaleKernel.outputs:LdrGrayscale", "InvertKernel.inputs:Image"),
                ("InvertKernel.outputs:Inverted", "LdrInverted"),
            ],
        )
        # source render var
        ldr_color = self._stage.GetPrimAtPath(f"{rp_path}/LdrColor")
        self.assertEqual(ldr_color.GetTypeName(), "RenderVar")
        self.assertEqual(ldr_color.GetAttribute("sourceName").Get(), "LdrColor")
        self.assertEqual(ldr_color.GetAttribute("omni:rtx:aov").GetTypeName(), Sdf.ValueTypeNames.Opaque)
        # shader input consumes the source AOV
        gs_in = self._stage.GetAttributeAtPath(f"{rp_path}/GrayscaleKernel.inputs:LdrColor")
        self.assertEqual([str(c) for c in gs_in.GetConnections()], [f"{rp_path}/LdrColor.omni:rtx:aov"])
        # shader-to-shader chaining
        inv_in = self._stage.GetAttributeAtPath(f"{rp_path}/InvertKernel.inputs:Image")
        self.assertEqual([str(c) for c in inv_in.GetConnections()], [f"{rp_path}/GrayscaleKernel.outputs:LdrGrayscale"])
        # output render var connects to the shader output
        ldr_inv = self._stage.GetPrimAtPath(f"{rp_path}/LdrInverted")
        self.assertEqual(ldr_inv.GetTypeName(), "RenderVar")
        self.assertEqual(
            [str(c) for c in ldr_inv.GetAttribute("omni:rtx:aov").GetConnections()],
            [f"{rp_path}/InvertKernel.outputs:Inverted"],
        )
        # orderedVars carries both referenced render vars
        ordered = [str(t) for t in self._stage.GetPrimAtPath(rp_path).GetRelationship("orderedVars").GetTargets()]
        self.assertIn(f"{rp_path}/LdrColor", ordered)
        self.assertIn(f"{rp_path}/LdrInverted", ordered)

    async def test_missing_source_aov_warns_and_creates(self) -> None:
        """Warn and create a render var when a source AOV has none yet."""
        with mock.patch("carb.log_warn") as log_warn:
            rp_path = self._cam.author_spg(
                self._grayscale_node(),
                connections=[("LdrColor", "GrayscaleKernel.inputs:LdrColor")],
            )
        self.assertTrue(log_warn.called)
        self.assertTrue(self._stage.GetPrimAtPath(f"{rp_path}/LdrColor").IsValid())

    async def test_existing_source_aov_no_warn(self) -> None:
        """Reuse a pre-authored source render var without warning."""
        rp = UsdRender.Product.Define(self._stage, "/World/camera_RenderProduct")
        rp.CreateCameraRel().SetTargets([Sdf.Path("/World/camera")])
        rv = self._stage.DefinePrim("/World/camera_RenderProduct/LdrColor", "RenderVar")
        rv.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set("LdrColor")
        rv.CreateAttribute("omni:rtx:aov", Sdf.ValueTypeNames.Opaque)
        with mock.patch("carb.log_warn") as log_warn:
            self._cam.author_spg(
                self._grayscale_node(),
                connections=[("LdrColor", "GrayscaleKernel.inputs:LdrColor")],
            )
        self.assertFalse(log_warn.called)

    # -- render product resolution --

    async def test_reuses_existing_render_product(self) -> None:
        """Author onto an existing render product tied to the sensor instead of a new one."""
        rp = UsdRender.Product.Define(self._stage, "/World/PreauthoredRP")
        rp.CreateCameraRel().SetTargets([Sdf.Path("/World/camera")])
        rp_path = self._cam.author_spg(self._grayscale_node())
        self.assertEqual(rp_path, "/World/PreauthoredRP")
        self.assertFalse(self._stage.GetPrimAtPath("/World/camera_RenderProduct").IsValid())

    async def test_explicit_render_product_path(self) -> None:
        """Author onto an explicitly requested render product path, creating it if needed."""
        rp_path = self._cam.author_spg(self._grayscale_node(), render_product="/World/Explicit")
        self.assertEqual(rp_path, "/World/Explicit")
        self.assertTrue(self._stage.GetPrimAtPath("/World/Explicit/GrayscaleKernel").IsValid())

    async def test_explicit_render_product_wrong_type_raises(self) -> None:
        """Reject an explicit render_product path whose prim is not a RenderProduct."""
        stage_utils.define_prim("/World/NotAProduct", "Xform")
        with self.assertRaises(ValueError):
            self._cam.author_spg(self._grayscale_node(), render_product="/World/NotAProduct")

    async def test_reuses_render_product_under_render_scope(self) -> None:
        """Reuse a render product authored under a /Render scope (canonical location)."""
        self._stage.DefinePrim("/Render", "Scope")
        rp = UsdRender.Product.Define(self._stage, "/Render/GrayscaleDemo")
        rp.CreateCameraRel().SetTargets([Sdf.Path("/World/camera")])
        rp_path = self._cam.author_spg(self._grayscale_node())
        self.assertEqual(rp_path, "/Render/GrayscaleDemo")

    async def test_invalid_connection_endpoint_raises(self) -> None:
        """Reject a connection endpoint that names an undeclared shader port."""
        with self.assertRaises(ValueError):
            self._cam.author_spg(
                self._grayscale_node(),
                connections=[("LdrColor", "GrayscaleKernel.inputs:DoesNotExist")],
            )

    # -- copy_to --

    async def test_copy_to_dedup_shared_cu(self) -> None:
        """Copy a shared .cu and its .cu.lua once and repoint shader source assets."""
        copy_dir = tempfile.mkdtemp()
        try:
            node_a = SPGNode("A", self._grayscale_cu, sub_identifier="grayscale", outputs=["OutA"])
            node_b = SPGNode("B", self._grayscale_cu, sub_identifier="grayscale", outputs=["OutB"])
            rp_path = self._cam.author_spg([node_a, node_b], copy_to=copy_dir)
            copied_cu = os.path.join(copy_dir, "GrayscaleKernel.cu")
            copied_lua = os.path.join(copy_dir, "GrayscaleKernel.cu.lua")
            self.assertTrue(os.path.isfile(copied_cu))
            self.assertTrue(os.path.isfile(copied_lua))
            for name in ("A", "B"):
                shader = UsdShade.Shader(self._stage.GetPrimAtPath(f"{rp_path}/{name}"))
                self.assertEqual(shader.GetSourceAsset("spg").path, copied_cu)
        finally:
            shutil.rmtree(copy_dir, ignore_errors=True)

    async def test_rejects_multiple_prims(self) -> None:
        """Reject SPG authoring when the camera wrapper holds more than one prim."""
        RtxCamera("/World/camera_1")
        batched = RtxCamera("/World/camera.*")
        self.assertGreater(len(batched), 1)
        with self.assertRaisesRegex(ValueError, "author_spg supports a single sensor prim"):
            batched.author_spg(self._grayscale_node())
