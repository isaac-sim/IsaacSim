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

"""Tests for MobilityGenWriter.copy_stage / copy_init."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen.impl.writer import MobilityGenWriter
from pxr import Sdf, Usd, UsdGeom, UsdShade

# 1x1 PNG — body of every test texture; avoids a PIL dependency at test time.
_PNG_1x1 = bytes.fromhex(
    "89504E470D0A1A0A0000000D49484452000000010000000108060000001F15C489"
    "0000000D49444154789C636060606000000004000118A2A5DD0000000049454E44AE426082"
)


def _make_png(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(_PNG_1x1)


def _build_source_scene(scene_dir: str) -> str:
    """Build a USD covering: relative texture, UDIM family, Reference sub-USD
    with its own texture, Payload sub-USD with its own texture."""
    _make_png(os.path.join(scene_dir, "textures", "brick.png"))
    # Non-UDIM texture whose basename happens to contain a 4-digit segment in
    # the UDIM range — must NOT be treated as a UDIM template by the rewriter.
    _make_png(os.path.join(scene_dir, "textures", "icon.2048.png"))
    for udim in ("1001", "1002", "1003"):
        _make_png(os.path.join(scene_dir, "textures", f"tile.{udim}.png"))
    _make_png(os.path.join(scene_dir, "subusds", "tex", "ref_tex.png"))
    _make_png(os.path.join(scene_dir, "subusds", "tex", "payload_tex.png"))

    ref_usd = os.path.join(scene_dir, "subusds", "ref.usd")
    ref_stage = Usd.Stage.CreateNew(ref_usd)
    ref_root = UsdGeom.Xform.Define(ref_stage, "/Ref").GetPrim()
    ref_stage.SetDefaultPrim(ref_root)
    ref_tex = UsdShade.Shader.Define(ref_stage, "/Ref/Mat/Tex")
    ref_tex.CreateIdAttr("UsdUVTexture")
    ref_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./tex/ref_tex.png"))
    ref_stage.Save()

    pl_usd = os.path.join(scene_dir, "subusds", "payload.usd")
    pl_stage = Usd.Stage.CreateNew(pl_usd)
    pl_root = UsdGeom.Xform.Define(pl_stage, "/Payloaded").GetPrim()
    pl_stage.SetDefaultPrim(pl_root)
    pl_tex = UsdShade.Shader.Define(pl_stage, "/Payloaded/Mat/Tex")
    pl_tex.CreateIdAttr("UsdUVTexture")
    pl_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./tex/payload_tex.png"))
    pl_stage.Save()

    scene_usd = os.path.join(scene_dir, "scene.usd")
    stage = Usd.Stage.CreateNew(scene_usd)
    UsdGeom.Xform.Define(stage, "/World")

    ref_prim = UsdGeom.Xform.Define(stage, "/World/Referenced").GetPrim()
    ref_prim.GetReferences().AddReference("./subusds/ref.usd")
    pl_prim = UsdGeom.Xform.Define(stage, "/World/Payloaded").GetPrim()
    pl_prim.GetPayloads().AddPayload("./subusds/payload.usd")

    tex = UsdShade.Shader.Define(stage, "/World/Mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/brick.png"))

    icon_tex = UsdShade.Shader.Define(stage, "/World/Mat/IconTex")
    icon_tex.CreateIdAttr("UsdUVTexture")
    icon_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/icon.2048.png"))

    udim_tex = UsdShade.Shader.Define(stage, "/World/Mat/UdimTex")
    udim_tex.CreateIdAttr("UsdUVTexture")
    udim_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/tile.<UDIM>.png"))

    stage.Save()
    return scene_usd


def _flatten(scene_usd: str, out_path: str) -> None:
    # Mirror `omni.usd.get_context().export_as_stage()`: composition arcs are
    # inlined but asset-typed attribute strings still resolve to their original
    # on-disk locations. That's the input shape `copy_stage` sees in production.
    Usd.Stage.Open(scene_usd).Flatten().Export(out_path)


def _texture_path(stage: Usd.Stage, prim_path: str) -> str:
    return UsdShade.Shader(stage.GetPrimAtPath(prim_path)).GetInput("file").Get().path


def _resolve(stage: Usd.Stage, raw_asset_path: str) -> str:
    return stage.GetRootLayer().ComputeAbsolutePath(raw_asset_path)


class TestCopyStageDependencies(omni.kit.test.AsyncTestCase):
    """copy_stage on a flattened .usd must produce a self-contained recording."""

    async def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix="test_writer_")
        scene_dir = os.path.join(self._tmp, "scene")
        os.makedirs(scene_dir)
        self._source_scene = _build_source_scene(scene_dir)
        self._cached_stage = os.path.join(self._tmp, "cached_stage.usd")
        _flatten(self._source_scene, self._cached_stage)
        self._recording_dir = os.path.join(self._tmp, "recording")

    async def tearDown(self):
        shutil.rmtree(self._tmp, ignore_errors=True)

    async def test_recording_stage_is_written(self):
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()
        self.assertTrue(os.path.exists(os.path.join(self._recording_dir, "stage.usd")))

    async def test_all_external_assets_copied(self):
        # Includes textures inlined from flattened Reference/Payload arcs.
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()

        copied_basenames = set()
        for root, _, files in os.walk(os.path.join(self._recording_dir, "assets")):
            for f in files:
                copied_basenames.add(f)

        for expected in (
            "brick.png",
            "icon.2048.png",
            "ref_tex.png",
            "payload_tex.png",
            "tile.1001.png",
            "tile.1002.png",
            "tile.1003.png",
        ):
            self.assertIn(expected, copied_basenames, f"{expected} was not copied")

    async def test_rewritten_paths_resolve_inside_recording(self):
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()

        recorded = Usd.Stage.Open(os.path.join(self._recording_dir, "stage.usd"))
        for prim_path, label in [
            ("/World/Mat/Tex", "top-level texture"),
            ("/World/Mat/IconTex", "non-UDIM texture with 4-digit segment in basename"),
            ("/World/Referenced/Mat/Tex", "texture inside flattened Reference"),
            ("/World/Payloaded/Mat/Tex", "texture inside flattened Payload"),
        ]:
            raw = _texture_path(recorded, prim_path)
            resolved = _resolve(recorded, raw)
            self.assertTrue(
                os.path.exists(resolved),
                f"{label}: rewritten asset path {raw!r} does not resolve to an existing file",
            )
            self.assertEqual(
                os.path.commonpath([resolved, self._recording_dir]),
                self._recording_dir,
                f"{label}: resolved path {resolved!r} is outside recording dir",
            )

    async def test_udim_placeholder_preserved(self):
        # `<UDIM>` token must survive the rewrite — one authored ref per family,
        # not one per tile. Non-UDIM textures with a 4-digit segment in their
        # basename (e.g. `icon.2048.png`) must NOT be turned into `<UDIM>` refs.
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()

        recorded = Usd.Stage.Open(os.path.join(self._recording_dir, "stage.usd"))
        udim_raw = _texture_path(recorded, "/World/Mat/UdimTex")
        self.assertIn("<UDIM>", udim_raw, f"UDIM placeholder dropped from authored path {udim_raw!r}")
        for udim in ("1001", "1002", "1003"):
            concrete = _resolve(recorded, udim_raw.replace("<UDIM>", udim))
            self.assertTrue(os.path.exists(concrete), f"UDIM tile {udim} not present at {concrete!r}")

        icon_raw = _texture_path(recorded, "/World/Mat/IconTex")
        self.assertNotIn("<UDIM>", icon_raw, f"non-UDIM basename `icon.2048.png` rewritten to {icon_raw!r}")

    async def test_source_layer_not_mutated(self):
        # The layer returned by ComputeAllDependencies is in USD's global
        # registry; rewriting in place would corrupt any live consumer.
        cached_mtime = os.path.getmtime(self._cached_stage)
        cached_size = os.path.getsize(self._cached_stage)
        source_mtime = os.path.getmtime(self._source_scene)
        source_size = os.path.getsize(self._source_scene)

        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()

        self.assertEqual(os.path.getmtime(self._cached_stage), cached_mtime)
        self.assertEqual(os.path.getsize(self._cached_stage), cached_size)
        self.assertEqual(os.path.getmtime(self._source_scene), source_mtime)
        self.assertEqual(os.path.getsize(self._source_scene), source_size)

    async def test_recording_is_relocatable(self):
        # Asset paths must be relative to stage.usd, not absolute.
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()

        moved = os.path.join(self._tmp, "moved")
        shutil.copytree(self._recording_dir, moved)
        moved_stage = Usd.Stage.Open(os.path.join(moved, "stage.usd"))
        for prim_path, label in [
            ("/World/Mat/Tex", "top-level texture"),
            ("/World/Referenced/Mat/Tex", "Reference-anchored texture"),
            ("/World/Payloaded/Mat/Tex", "Payload-anchored texture"),
        ]:
            raw = _texture_path(moved_stage, prim_path)
            resolved = _resolve(moved_stage, raw)
            self.assertTrue(os.path.exists(resolved), f"{label}: {raw!r} did not resolve after move")
            self.assertEqual(
                os.path.commonpath([resolved, moved]),
                moved,
                f"{label}: resolves outside moved dir ({resolved!r})",
            )


class TestCopyStageUsdz(omni.kit.test.AsyncTestCase):
    """USDZ inputs are self-contained — copy_stage must leave them alone."""

    async def test_usdz_input_is_plain_copied(self):
        from pxr import UsdUtils

        with tempfile.TemporaryDirectory() as tmp:
            # USDZ packages must be constructed via `UsdUtils.CreateNewUsdzPackage`;
            # `Sdf.Layer.Export(*.usdz)` is rejected by the package-layer writer.
            inner = os.path.join(tmp, "inner.usda")
            stage = Usd.Stage.CreateNew(inner)
            UsdGeom.Xform.Define(stage, "/World")
            stage.Save()
            src = os.path.join(tmp, "src.usdz")
            self.assertTrue(UsdUtils.CreateNewUsdzPackage(inner, src))
            src_size = os.path.getsize(src)

            recording_dir = os.path.join(tmp, "recording")
            writer = MobilityGenWriter(recording_dir, async_write=False)
            try:
                writer.copy_stage(src)
            finally:
                writer.close()

            dst = os.path.join(recording_dir, "stage.usdz")
            self.assertTrue(os.path.exists(dst))
            self.assertEqual(os.path.getsize(dst), src_size)
            self.assertFalse(os.path.isdir(os.path.join(recording_dir, "assets")))


class TestCopyInitWithAssets(omni.kit.test.AsyncTestCase):
    """copy_init must propagate the sibling assets/ tree, and tolerate its absence."""

    async def test_assets_subdir_is_propagated(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "src")
            os.makedirs(os.path.join(src, "assets", "abcd1234"))
            with open(os.path.join(src, "stage.usd"), "w") as f:
                f.write("#usda 1.0\n")
            with open(os.path.join(src, "config.json"), "w") as f:
                f.write("{}")
            with open(os.path.join(src, "assets", "abcd1234", "brick.png"), "wb") as f:
                f.write(_PNG_1x1)
            os.makedirs(os.path.join(src, "occupancy_map"))

            dst = os.path.join(tmp, "dst")
            writer = MobilityGenWriter(dst, async_write=False)
            try:
                writer.copy_init(src)
            finally:
                writer.close()

            self.assertTrue(os.path.exists(os.path.join(dst, "stage.usd")))
            self.assertTrue(os.path.exists(os.path.join(dst, "assets", "abcd1234", "brick.png")))

    async def test_missing_assets_subdir_is_tolerated(self):
        # Legacy recordings have no assets/ dir; copy_init must still succeed.
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "src")
            os.makedirs(src)
            with open(os.path.join(src, "stage.usd"), "w") as f:
                f.write("#usda 1.0\n")
            with open(os.path.join(src, "config.json"), "w") as f:
                f.write("{}")
            os.makedirs(os.path.join(src, "occupancy_map"))

            dst = os.path.join(tmp, "dst")
            writer = MobilityGenWriter(dst, async_write=False)
            try:
                writer.copy_init(src)
            finally:
                writer.close()

            self.assertTrue(os.path.exists(os.path.join(dst, "stage.usd")))
            self.assertFalse(os.path.isdir(os.path.join(dst, "assets")))
