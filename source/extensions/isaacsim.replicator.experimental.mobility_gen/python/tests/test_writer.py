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

"""Tests for MobilityGen recording-stage caching (collect_input / copy_stage / copy_init)."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen.impl.writer import MobilityGenWriter, collect_input
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


def _make_text(path: str, body: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(body)


def _build_source_scene(scene_dir: str) -> str:
    """Build a non-flattened USD with textures, references, payloads, and kernels.

    The scene covers relative texture paths, a UDIM family, a Reference sub-USD
    and a Payload sub-USD, plus an SPG `.cu` kernel with a sibling `.cu.lua`
    launcher that is undeclared in USD.

    Args:
        scene_dir: Directory where the source scene and assets are created.

    Returns:
        Path to the created source USD scene.
    """
    _make_png(os.path.join(scene_dir, "textures", "brick.png"))
    for udim in ("1001", "1002", "1003"):
        _make_png(os.path.join(scene_dir, "textures", f"tile.{udim}.png"))
    _make_png(os.path.join(scene_dir, "subusds", "tex", "ref_tex.png"))
    _make_png(os.path.join(scene_dir, "subusds", "tex", "payload_tex.png"))
    # SPG kernel pair: `.cu` is declared via an asset attribute; `.cu.lua` is a
    # sibling found by naming convention only (no USD attribute references it).
    _make_text(os.path.join(scene_dir, "kernels", "foo.cu"), "// cuda kernel\n")
    _make_text(os.path.join(scene_dir, "kernels", "foo.cu.lua"), "-- launcher\n")

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

    udim_tex = UsdShade.Shader.Define(stage, "/World/Mat/UdimTex")
    udim_tex.CreateIdAttr("UsdUVTexture")
    udim_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/tile.<UDIM>.png"))

    kernel = UsdGeom.Xform.Define(stage, "/World/Kernel").GetPrim()
    kernel.CreateAttribute("info:spg:sourceAsset", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./kernels/foo.cu"))

    stage.Save()
    return scene_usd


def _resolved_texture(stage: Usd.Stage, prim_path: str) -> str:
    """Return the resolved path of a shader's `file` input.

    Resolution is anchored at the layer that authored the input so textures
    inside a sub-USD resolve correctly.

    Args:
        stage: USD stage containing the shader prim.
        prim_path: USD prim path of the shader.

    Returns:
        Resolved texture path, or an empty string when no asset is authored.
    """
    asset = UsdShade.Shader(stage.GetPrimAtPath(prim_path)).GetInput("file").Get()
    return asset.resolvedPath if asset else ""


def _exists_under(root: str, basename: str) -> bool:
    """True if a file named `basename` exists anywhere under `root`.

    Args:
        root: Directory tree to search.
        basename: File name to find.

    Returns:
        True if a matching file exists under ``root``.
    """
    return any(basename in files for _, _, files in os.walk(root))


class TestCollectInputUsd(omni.kit.test.AsyncTestCase):
    """Validate self-contained caches collected from USD inputs.

    `collect_input` on a `.usd` must produce a relocatable cache whose
    references all resolve inside it.
    """

    async def setUp(self) -> None:
        """Create a source USD scene and cache directory."""
        self._tmp = tempfile.mkdtemp(prefix="test_writer_")
        scene_dir = os.path.join(self._tmp, "scene")
        os.makedirs(scene_dir)
        self._source_scene = _build_source_scene(scene_dir)
        self._cache_dir = os.path.join(self._tmp, "cache")

    async def tearDown(self) -> None:
        """Remove the temporary cache workspace."""
        shutil.rmtree(self._tmp, ignore_errors=True)

    async def test_returns_stage_usd(self) -> None:
        """Collecting a USD returns the cached stage path."""
        stage_path = await collect_input(self._source_scene, self._cache_dir)
        self.assertEqual(stage_path, os.path.join(self._cache_dir, "stage.usd"))
        self.assertTrue(os.path.isfile(stage_path))

    async def test_textures_resolve_inside_cache(self) -> None:
        """Texture dependencies resolve inside the collected cache."""
        await collect_input(self._source_scene, self._cache_dir)
        recorded = Usd.Stage.Open(os.path.join(self._cache_dir, "stage.usd"))
        for prim_path, label in [
            ("/World/Mat/Tex", "top-level texture"),
            ("/World/Referenced/Mat/Tex", "texture inside referenced sub-USD"),
            ("/World/Payloaded/Mat/Tex", "texture inside payloaded sub-USD"),
        ]:
            resolved = _resolved_texture(recorded, prim_path)
            self.assertTrue(resolved and os.path.exists(resolved), f"{label}: did not resolve")
            self.assertEqual(
                os.path.commonpath([resolved, self._cache_dir]),
                self._cache_dir,
                f"{label}: resolved outside cache ({resolved!r})",
            )

    async def test_udim_tiles_collected(self) -> None:
        """UDIM texture tiles are collected into the cache."""
        await collect_input(self._source_scene, self._cache_dir)
        for udim in ("1001", "1002", "1003"):
            self.assertTrue(_exists_under(self._cache_dir, f"tile.{udim}.png"), f"UDIM tile {udim} not collected")

    async def test_referenced_cu_collected(self) -> None:
        """Referenced CUDA kernels are collected into the cache."""
        # `.cu` is declared via an asset attribute (info:spg:sourceAsset), so it is
        # a USD dependency and must be collected.
        await collect_input(self._source_scene, self._cache_dir)
        self.assertTrue(_exists_under(self._cache_dir, "foo.cu"), "referenced .cu was not collected")

    async def test_unreferenced_companion_skipped(self) -> None:
        """Unreferenced CUDA companion files are not collected from USD inputs."""
        # `.cu.lua` is referenced by no USD path, so it is not collected. Such
        # scenes must be provided as `.usdz` to keep the companion.
        await collect_input(self._source_scene, self._cache_dir)
        self.assertFalse(_exists_under(self._cache_dir, "foo.cu.lua"), "unreferenced .cu.lua should be skipped")

    async def test_no_hash_bucketing(self) -> None:
        """Collected USD caches do not use hash-bucketed asset directories."""
        await collect_input(self._source_scene, self._cache_dir)
        self.assertFalse(os.path.isdir(os.path.join(self._cache_dir, "assets")))

    async def test_source_not_mutated(self) -> None:
        """Collecting input does not mutate the source USD file."""
        source_mtime = os.path.getmtime(self._source_scene)
        source_size = os.path.getsize(self._source_scene)
        await collect_input(self._source_scene, self._cache_dir)
        self.assertEqual(os.path.getmtime(self._source_scene), source_mtime)
        self.assertEqual(os.path.getsize(self._source_scene), source_size)

    async def test_cache_is_relocatable(self) -> None:
        """Collected caches remain resolvable after being moved."""
        await collect_input(self._source_scene, self._cache_dir)
        moved = os.path.join(self._tmp, "moved")
        shutil.copytree(self._cache_dir, moved)
        moved_stage = Usd.Stage.Open(os.path.join(moved, "stage.usd"))
        for prim_path in ("/World/Mat/Tex", "/World/Referenced/Mat/Tex", "/World/Payloaded/Mat/Tex"):
            resolved = _resolved_texture(moved_stage, prim_path)
            self.assertTrue(resolved and os.path.exists(resolved), f"{prim_path} did not resolve after move")
            self.assertEqual(os.path.commonpath([resolved, moved]), moved)


class TestCopyStage(omni.kit.test.AsyncTestCase):
    """copy_stage must copy the cache tree verbatim into the recording dir."""

    async def setUp(self) -> None:
        """Create a collected cache and destination recording directory."""
        self._tmp = tempfile.mkdtemp(prefix="test_writer_copy_")
        scene_dir = os.path.join(self._tmp, "scene")
        os.makedirs(scene_dir)
        self._source_scene = _build_source_scene(scene_dir)
        self._cache_dir = os.path.join(self._tmp, "cache")
        self._cached_stage = await collect_input(self._source_scene, self._cache_dir)
        self._recording_dir = os.path.join(self._tmp, "recording")

    async def tearDown(self) -> None:
        """Remove the temporary copy-stage workspace."""
        shutil.rmtree(self._tmp, ignore_errors=True)

    async def test_recording_mirrors_cache(self) -> None:
        """The recording stage copy mirrors the collected cache."""
        writer = MobilityGenWriter(self._recording_dir, async_write=False)
        try:
            writer.copy_stage(self._cached_stage)
        finally:
            writer.close()
        self.assertTrue(os.path.isfile(os.path.join(self._recording_dir, "stage.usd")))
        # References still resolve from the recording copy.
        recorded = Usd.Stage.Open(os.path.join(self._recording_dir, "stage.usd"))
        resolved = _resolved_texture(recorded, "/World/Referenced/Mat/Tex")
        self.assertTrue(resolved and os.path.exists(resolved))
        self.assertEqual(os.path.commonpath([resolved, self._recording_dir]), self._recording_dir)


class TestCollectInputUsdz(omni.kit.test.AsyncTestCase):
    """USDZ inputs are byte-copied — every member (incl. `.cu.lua`) is preserved."""

    async def test_usdz_input_is_byte_copied_with_members_preserved(self) -> None:
        """USDZ input archives are byte-copied with all members preserved."""
        import zipfile

        from pxr import UsdUtils

        with tempfile.TemporaryDirectory() as tmp:
            # Build a USDZ that includes an SPG `.cu` and its `.cu.lua` companion.
            src_dir = os.path.join(tmp, "src")
            inner = os.path.join(src_dir, "inner.usda")
            os.makedirs(src_dir)
            stage = Usd.Stage.CreateNew(inner)
            UsdGeom.Xform.Define(stage, "/World")
            stage.Save()
            _make_text(os.path.join(src_dir, "foo.cu"), "// k\n")
            _make_text(os.path.join(src_dir, "foo.cu.lua"), "-- l\n")
            src = os.path.join(tmp, "src.usdz")
            self.assertTrue(UsdUtils.CreateNewUsdzPackage(inner, src))
            # Ensure the companion is inside the archive (CreateNewUsdzPackage only
            # packs declared members; add the undeclared `.cu.lua` to mimic NuRec).
            with zipfile.ZipFile(src, "a") as zf:
                if "foo.cu.lua" not in zf.namelist():
                    zf.write(os.path.join(src_dir, "foo.cu.lua"), "foo.cu.lua")
            src_size = os.path.getsize(src)
            with zipfile.ZipFile(src) as zf:
                src_members = set(zf.namelist())

            cache_dir = os.path.join(tmp, "cache")
            stage_path = await collect_input(src, cache_dir)

            self.assertEqual(stage_path, os.path.join(cache_dir, "stage.usdz"))
            self.assertEqual(os.path.getsize(stage_path), src_size)  # byte-for-byte
            self.assertFalse(os.path.isdir(os.path.join(cache_dir, "assets")))
            # All archive members survive — including the undeclared `.cu.lua`.
            with zipfile.ZipFile(stage_path) as zf:
                self.assertEqual(set(zf.namelist()), src_members)


class TestCopyInit(omni.kit.test.AsyncTestCase):
    """copy_init must copy stage + dependency files + config + occupancy, but not state."""

    async def test_init_files_copied_state_skipped(self) -> None:
        """Initial replay files are copied while recorded state is skipped."""
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "src")
            os.makedirs(src)
            _make_text(os.path.join(src, "stage.usd"), "#usda 1.0\n")
            _make_text(os.path.join(src, "config.json"), "{}")
            _make_png(os.path.join(src, "textures", "brick.png"))  # sibling dependency
            os.makedirs(os.path.join(src, "occupancy_map"))
            # per-step recorded output that must NOT be copied
            _make_text(os.path.join(src, "state", "common", "00000000.npz"), "x")

            dst = os.path.join(tmp, "dst")
            writer = MobilityGenWriter(dst, async_write=False)
            try:
                writer.copy_init(src)
            finally:
                writer.close()

            self.assertTrue(os.path.exists(os.path.join(dst, "stage.usd")))
            self.assertTrue(os.path.exists(os.path.join(dst, "config.json")))
            self.assertTrue(os.path.exists(os.path.join(dst, "textures", "brick.png")))
            self.assertTrue(os.path.isdir(os.path.join(dst, "occupancy_map")))
            self.assertFalse(os.path.isdir(os.path.join(dst, "state")))
