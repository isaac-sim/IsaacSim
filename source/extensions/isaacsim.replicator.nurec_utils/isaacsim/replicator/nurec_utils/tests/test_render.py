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

"""render.py helpers on synthetic in-memory stages (no rendering): camera-xform save/restore and
the SPG RenderProduct clone.
"""

from __future__ import annotations

import omni.kit.test
from pxr import Gf, Sdf, Usd, UsdGeom


class TestCameraXformRestore(omni.kit.test.AsyncTestCase):
    """A `render_at_pose` camera-xform override is captured and fully restored on close."""

    async def test_override_then_restore_round_trips(self) -> None:
        """Snapshot -> override -> restore returns the camera's original xform-op order and reset flag."""
        from isaacsim.replicator.nurec_utils.render import (
            _override_camera_pose,
            _restore_camera_xform,
            _snapshot_camera_xform,
        )

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World/rig")
        UsdGeom.Camera.Define(stage, "/World/rig/cam").AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
        cam_path = "/World/rig/cam"
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
        orig_order = list(xform.GetXformOpOrderAttr().Get() or [])
        orig_reset = xform.GetResetXformStack()

        snapshot = _snapshot_camera_xform(stage, cam_path)
        _override_camera_pose(stage, cam_path).Set(Gf.Matrix4d(1.0))

        self.assertIn("xformOp:transform", list(xform.GetXformOpOrderAttr().Get() or []))
        self.assertTrue(xform.GetResetXformStack())

        _restore_camera_xform(stage, cam_path, snapshot)

        self.assertEqual(list(xform.GetXformOpOrderAttr().Get() or []), orig_order)
        self.assertEqual(xform.GetResetXformStack(), orig_reset)


class TestCopyRenderProduct(omni.kit.test.AsyncTestCase):
    """copy_render_product clones an authored SPG RenderProduct into the session layer."""

    @staticmethod
    def _make_spg_stage() -> Usd.Stage:
        """In-memory stage with one authored SPG RenderProduct (/Render/src) for /World/cam_a.

        Returns:
            The in-memory stage with the authored RenderProduct.
        """
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Camera.Define(stage, "/World/cam_a")
        UsdGeom.Camera.Define(stage, "/World/cam_b")
        stage.DefinePrim("/Render", "Scope")
        rp = stage.DefinePrim("/Render/src", "RenderProduct")
        rp.CreateAttribute("resolution", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(640, 480))
        rp.CreateRelationship("camera").SetTargets(["/World/cam_a"])
        rp.CreateRelationship("orderedVars").SetTargets(["/Render/src/LdrColor"])
        stage.DefinePrim("/Render/src/LdrColor", "RenderVar")
        shader = stage.DefinePrim("/Render/src/ppisp", "Shader")
        shader.CreateAttribute("info:spg:sourceAsset", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath("ppisp_controller_0.cu")
        )
        shader.CreateRelationship("renderProduct").SetTargets(["/Render/src"])
        return stage

    async def test_clone_remaps_targets_and_rewrites_spg_assets(self) -> None:
        """The clone repoints to the new camera, keeps resolution, remaps internal targets, and
        package-qualifies its SPG asset paths.
        """
        from isaacsim.replicator.nurec_utils.render import copy_render_product

        stage = self._make_spg_stage()
        width, height = copy_render_product(
            stage, "/Render/src", "/Render/dst", "/World/cam_b", package_path="scene.usdz"
        )

        self.assertEqual((width, height), (640, 480))
        dst = stage.GetPrimAtPath("/Render/dst")
        self.assertTrue(dst.IsValid())
        self.assertEqual([str(t) for t in dst.GetRelationship("camera").GetTargets()], ["/World/cam_b"])
        self.assertEqual(tuple(dst.GetAttribute("resolution").Get()), (640, 480))
        self.assertEqual([str(t) for t in dst.GetRelationship("orderedVars").GetTargets()], ["/Render/dst/LdrColor"])

        clone_shader = stage.GetPrimAtPath("/Render/dst/ppisp")
        self.assertEqual(
            clone_shader.GetAttribute("info:spg:sourceAsset").Get().path, "scene.usdz[ppisp_controller_0.cu]"
        )
        self.assertEqual([str(t) for t in clone_shader.GetRelationship("renderProduct").GetTargets()], ["/Render/dst"])

    async def test_clone_is_separate_layer_and_leaves_source_unchanged(self) -> None:
        """The clone is a distinct prim authored only in the session layer with its own render var, and
        the authored RenderProduct keeps its camera, var, and un-rewritten SPG asset.
        """
        from isaacsim.replicator.nurec_utils.render import copy_render_product

        stage = self._make_spg_stage()
        copy_render_product(stage, "/Render/src", "/Render/dst", "/World/cam_b", package_path="scene.usdz")

        # The clone lives only in the session layer with its own render var, so it cannot share the
        # authored RenderProduct's render output.
        self.assertIsNotNone(stage.GetSessionLayer().GetPrimAtPath("/Render/dst"))
        self.assertIsNone(stage.GetRootLayer().GetPrimAtPath("/Render/dst"))
        self.assertTrue(stage.GetPrimAtPath("/Render/dst/LdrColor").IsValid())

        # The authored RenderProduct is untouched.
        src = stage.GetPrimAtPath("/Render/src")
        self.assertTrue(src.IsValid())
        self.assertEqual([str(t) for t in src.GetRelationship("camera").GetTargets()], ["/World/cam_a"])
        self.assertEqual([str(t) for t in src.GetRelationship("orderedVars").GetTargets()], ["/Render/src/LdrColor"])
        self.assertEqual(
            stage.GetPrimAtPath("/Render/src/ppisp").GetAttribute("info:spg:sourceAsset").Get().path,
            "ppisp_controller_0.cu",
        )
