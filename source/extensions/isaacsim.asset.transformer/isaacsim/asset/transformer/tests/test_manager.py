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

"""Tests for the asset transformer manager and registry."""

import os
import struct
import tempfile
import types
import zlib
from unittest.mock import patch

import omni.kit.test
from isaacsim.asset.transformer.manager import AssetTransformerManager, RuleRegistry, _collect_assets
from isaacsim.asset.transformer.models import RuleConfigurationParam, RuleProfile, RuleSpec
from isaacsim.asset.transformer.rule_interface import RuleInterface
from pxr import Sdf, Usd, UsdGeom


def _write_1x1_png(path: str) -> None:
    """Write a valid 1x1 RGBA PNG so any consumer can decode it.

    PNG layout: 8-byte signature + IHDR + IDAT + IEND chunks with correct CRC32
    values. Used by tests that copy texture assets and verify the copied files
    remain valid images on disk.
    """

    def _chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 6, 0, 0, 0)  # 1x1, 8-bit, RGBA
    # Filter byte 0 + 4 bytes of RGBA pixel data, then zlib-compressed.
    idat = zlib.compress(b"\x00" + b"\x00\x00\x00\x00", level=9)
    with open(path, "wb") as fh:
        fh.write(signature)
        fh.write(_chunk(b"IHDR", ihdr))
        fh.write(_chunk(b"IDAT", idat))
        fh.write(_chunk(b"IEND", b""))


class _DummyRule(RuleInterface):
    """Minimal rule implementation for manager tests."""

    def process_rule(self) -> None:
        """Record a dummy log entry and affected stage."""
        self.log_operation("dummy-run")
        self.add_affected_stage(self.destination_path or "memory")
        return

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return an empty configuration parameter list.

        Returns:
            Empty list of configuration parameters.

        """
        return []


class _FakeLayer:
    """Minimal layer mock supporting Export, Save, dirty, and realPath."""

    def __init__(self) -> None:
        self.realPath = ""
        self.dirty = False

    def Export(self, path: str) -> bool:  # noqa: N802
        """Record the export path and report success (test stub).

        Args:
            path: Destination path passed to ``Export``.

        Returns:
            Always ``True`` for the fake layer.
        """
        self.realPath = path
        return True

    def Save(self) -> None:  # noqa: N802
        """No-op save hook for the fake layer."""


class _FakeStage:
    """Minimal stage mock supporting GetRootLayer and Flatten."""

    def __init__(self) -> None:
        self._layer = _FakeLayer()

    def GetRootLayer(self) -> _FakeLayer:  # noqa: N802
        """Return the fake root layer instance.

        Returns:
            The ``_FakeLayer`` created in ``__init__``.
        """
        return self._layer

    def Flatten(self) -> _FakeLayer:  # noqa: N802
        """Return a new empty fake layer (test stub).

        Returns:
            A fresh ``_FakeLayer`` instance.
        """
        return _FakeLayer()


def _fake_usd(open_returns: object | None) -> types.SimpleNamespace:
    """Create a fake Usd module with a controllable Open result.

    Args:
        open_returns: Value returned by the fake Usd.Stage.Open call.

    Returns:
        Simple namespace mimicking the Usd module.

    """
    fake_stage_mod = types.SimpleNamespace()
    fake_stage_mod.Open = lambda path: open_returns
    fake_stage_mod.CreateInMemory = lambda: object()
    return types.SimpleNamespace(Stage=fake_stage_mod)


def _fake_sdf() -> types.SimpleNamespace:
    """Create a fake Sdf module with a controllable Layer.FindOrOpen result.

    Returns:
        Simple namespace mimicking the Sdf module.

    """
    fake_layer_mod = types.SimpleNamespace()
    fake_layer_mod.FindOrOpen = lambda path: None
    return types.SimpleNamespace(Layer=fake_layer_mod)


class TestManager(omni.kit.test.AsyncTestCase):
    """Async tests for AssetTransformerManager and RuleRegistry."""

    async def asyncSetUp(self) -> None:
        """Clear the rule registry before each test."""
        RuleRegistry().clear()

    async def asyncTearDown(self) -> None:
        """Clear the rule registry after each test."""
        RuleRegistry().clear()

    async def test_rule_registry_register_get_clear(self) -> None:
        """Verify registry register, get, and clear behaviors."""
        reg = RuleRegistry()
        reg.register(_DummyRule)
        fqcn = f"{_DummyRule.__module__}.{_DummyRule.__qualname__}"
        cls = reg.get(fqcn)
        self.assertIs(cls, _DummyRule)
        reg.clear()
        self.assertIsNone(reg.get(fqcn))

        class NotARule:  # noqa: D401
            """Not a RuleInterface subclass."""

        with self.assertRaises(TypeError):
            reg.register(NotARule)  # type: ignore[arg-type]

    async def test_manager_run_happy_path(self) -> None:
        """Verify manager run succeeds with a registered rule."""
        fake_stage = _FakeStage()
        fake_usd = _fake_usd(open_returns=fake_stage)
        fake_sdf = _fake_sdf()
        with (
            patch("isaacsim.asset.transformer.manager.Usd", fake_usd, create=True),
            patch("isaacsim.asset.transformer.manager.Sdf", fake_sdf, create=True),
            patch("isaacsim.asset.transformer.manager.os.makedirs"),
            patch("isaacsim.asset.transformer.rule_interface.Usd", fake_usd, create=True),
        ):
            reg = RuleRegistry()
            reg.clear()
            reg.register(_DummyRule)
            fqcn = f"{_DummyRule.__module__}.{_DummyRule.__qualname__}"
            profile = RuleProfile(
                profile_name="p",
                rules=[
                    RuleSpec(name="r1", type=fqcn, destination="out.usda"),
                    RuleSpec(name="disabled", type=fqcn, enabled=False),
                ],
                interface_asset_name="iface",
                output_package_root="/pkg",
            )
            mgr = AssetTransformerManager()
            report = mgr.run(input_stage="in.usda", profile=profile, package_root=None)
            self.assertEqual(len(report.results), 1)
            result = report.results[0]
            self.assertTrue(result.success)
            self.assertIsNone(result.error)
            self.assertTrue(result.log and result.log[-1]["message"] == "dummy-run")
            self.assertIn("out.usda", result.affected_stages)
            self.assertIsInstance(result.finished_at, str)
            self.assertIsInstance(report.finished_at, str)

    async def test_manager_run_missing_implementation_sets_error(self) -> None:
        """Verify missing rule implementations set error status."""
        fake_stage = _FakeStage()
        fake_usd = _fake_usd(open_returns=fake_stage)
        fake_sdf = _fake_sdf()
        with (
            patch("isaacsim.asset.transformer.manager.Usd", fake_usd, create=True),
            patch("isaacsim.asset.transformer.manager.Sdf", fake_sdf, create=True),
            patch("isaacsim.asset.transformer.manager.os.makedirs"),
            patch("isaacsim.asset.transformer.rule_interface.Usd", fake_usd, create=True),
        ):
            RuleRegistry().clear()
            unknown_type = "non.existent.Rule"
            profile = RuleProfile(profile_name="p", rules=[RuleSpec(name="r", type=unknown_type)])
            mgr = AssetTransformerManager()
            report = mgr.run(input_stage="in.usda", profile=profile)
            self.assertEqual(len(report.results), 1)
            res = report.results[0]
            self.assertFalse(res.success)
            self.assertIsInstance(res.error, str)
            self.assertIn("No rule implementation registered", res.error)
            self.assertIsInstance(res.finished_at, str)

    async def test_manager_run_source_open_failure_raises(self) -> None:
        """Verify source stage open failures raise errors."""
        fake_usd = _fake_usd(open_returns=None)
        with patch("isaacsim.asset.transformer.manager.Usd", fake_usd, create=True):
            mgr = AssetTransformerManager()
            profile = RuleProfile(profile_name="p", rules=[])
            with self.assertRaises(RuntimeError) as excinfo:
                mgr.run(input_stage="missing.usda", profile=profile)
            self.assertIn("Failed to open source stage", str(excinfo.exception))

    async def test_collect_assets_rewrites_source_relative_paths(self) -> None:
        """Regression test: relative asset paths must remap when source and output sit at different depths.

        Reproduces the original Bug 1 scenario: a source stage references an
        asset via a relative path; the output ``base.usd`` is written under a
        ``payloads/`` subdirectory at a different filesystem depth. Without
        anchoring dependency resolution to the source layer, the relative path
        survives verbatim in the output and resolves to a non-existent file.
        """
        with tempfile.TemporaryDirectory() as root:
            # Layout:
            #   <root>/src/stage/source.usda   <-- source stage
            #   <root>/src/textures/grid.png   <-- referenced via ../textures/grid.png
            #   <root>/out/                    <-- package_root (different depth)
            src_stage_dir = os.path.join(root, "src", "stage")
            textures_dir = os.path.join(root, "src", "textures")
            out_dir = os.path.join(root, "out")
            os.makedirs(src_stage_dir, exist_ok=True)
            os.makedirs(textures_dir, exist_ok=True)
            os.makedirs(out_dir, exist_ok=True)

            texture_path = os.path.join(textures_dir, "grid.png")
            _write_1x1_png(texture_path)

            source_stage_path = os.path.join(src_stage_dir, "source.usda")
            stage = Usd.Stage.CreateNew(source_stage_path)
            prim = UsdGeom.Xform.Define(stage, "/World")
            attr = prim.GetPrim().CreateAttribute("inputs:diffuse", Sdf.ValueTypeNames.Asset)
            attr.Set(Sdf.AssetPath("../textures/grid.png"))
            stage.GetRootLayer().Save()

            out_base_dir = os.path.join(out_dir, "payloads")
            os.makedirs(out_base_dir, exist_ok=True)
            out_base_path = os.path.join(out_base_dir, "base.usda")
            stage.GetRootLayer().Export(out_base_path)

            out_layer = Sdf.Layer.FindOrOpen(out_base_path)
            self.assertIsNotNone(out_layer, f"Failed to open exported layer at {out_base_path}")

            _collect_assets(out_layer, out_dir, source_layer_path=source_stage_path)

            # The texture must have been copied into <out>/source_assets/.
            copied_texture = os.path.join(out_dir, "source_assets", "grid.png")
            self.assertTrue(
                os.path.isfile(copied_texture),
                f"Asset was not copied to {copied_texture}",
            )

            # Every asset path in the output layer must resolve to an existing
            # file from the output layer's directory.
            out_layer_dir = os.path.dirname(out_base_path)
            reloaded = Usd.Stage.Open(out_base_path)
            broken: list[str] = []
            for p in reloaded.Traverse():
                for a in p.GetAttributes():
                    v = a.Get()
                    if isinstance(v, Sdf.AssetPath) and v.path:
                        path = v.path
                        if "://" in path:
                            continue
                        if os.path.isabs(path):
                            resolved = path
                        else:
                            resolved = os.path.normpath(os.path.join(out_layer_dir, path))
                        if not os.path.isfile(resolved):
                            broken.append(f"{p.GetPath()}.{a.GetName()} -> {resolved}")
            self.assertEqual(
                broken,
                [],
                f"Output layer still contains unresolvable asset paths: {broken}",
            )

    async def test_collect_assets_handles_sublayer_anchored_relative_paths(self) -> None:
        """Regression: relative asset paths authored in a SUBLAYER must remap correctly after flatten.

        The sublayer is *not* the root layer.

        Layout:

        ::

            <root>/src/main.usda       <-- root layer, sublayers ./detail/sub.usda
            <root>/src/detail/sub.usda <-- authors  ../../assets/tex.png
            <root>/assets/tex.png      <-- target asset

        The sublayer's relative path ``../../assets/tex.png`` is anchored at
        ``<root>/src/detail/``, NOT at the root layer's directory. Anchoring
        only at the root layer dir would resolve to ``<root>/assets/tex.png``
        in this case by accident, so to actually trigger the sublayer-anchor
        bug we put the asset somewhere only the sublayer's anchor reaches.
        """
        with tempfile.TemporaryDirectory() as root:
            src_dir = os.path.join(root, "src")
            sub_dir = os.path.join(src_dir, "detail")
            asset_dir = os.path.join(src_dir, "detail_assets")
            out_dir = os.path.join(root, "out")
            os.makedirs(sub_dir, exist_ok=True)
            os.makedirs(asset_dir, exist_ok=True)
            os.makedirs(out_dir, exist_ok=True)

            # Asset lives at <src>/detail_assets/tex.png. The sublayer's
            # relative path ``../detail_assets/tex.png`` resolves correctly
            # only when anchored at the SUBLAYER's directory (<src>/detail/).
            # Anchoring at the root layer's dir would produce
            # <src>/detail_assets/tex.png via ``./detail_assets/tex.png`` --
            # different STRING, so the bare-root anchor lookup misses.
            texture_path = os.path.join(asset_dir, "tex.png")
            _write_1x1_png(texture_path)

            # Sublayer that authors the asset reference.
            sub_path = os.path.join(sub_dir, "sub.usda")
            sub_stage = Usd.Stage.CreateNew(sub_path)
            prim = UsdGeom.Xform.Define(sub_stage, "/World/Detail")
            attr = prim.GetPrim().CreateAttribute("inputs:diffuse", Sdf.ValueTypeNames.Asset)
            # Path is RELATIVE to the sublayer's location.
            attr.Set(Sdf.AssetPath("../detail_assets/tex.png"))
            sub_stage.GetRootLayer().Save()

            # Root layer that sublayers the detail layer.
            root_path = os.path.join(src_dir, "main.usda")
            root_stage = Usd.Stage.CreateNew(root_path)
            root_stage.GetRootLayer().subLayerPaths.append("./detail/sub.usda")
            root_stage.GetRootLayer().Save()

            # Export the flattened stage to a different filesystem depth.
            out_base_dir = os.path.join(out_dir, "payloads")
            os.makedirs(out_base_dir, exist_ok=True)
            out_base_path = os.path.join(out_base_dir, "base.usda")
            flat_stage = Usd.Stage.Open(root_path)
            flat_layer = flat_stage.Flatten()
            self.assertTrue(flat_layer.Export(out_base_path))

            out_layer = Sdf.Layer.FindOrOpen(out_base_path)
            self.assertIsNotNone(out_layer)
            _collect_assets(out_layer, out_dir, source_layer_path=root_path)

            copied = os.path.join(out_dir, "source_assets", "tex.png")
            self.assertTrue(os.path.isfile(copied), f"Asset was not copied to {copied}")

            out_layer_dir = os.path.dirname(out_base_path)
            reloaded = Usd.Stage.Open(out_base_path)
            broken: list[str] = []
            for p in reloaded.Traverse():
                for a in p.GetAttributes():
                    v = a.Get()
                    if isinstance(v, Sdf.AssetPath) and v.path:
                        path = v.path
                        if "://" in path:
                            continue
                        if os.path.isabs(path):
                            resolved = path
                        else:
                            resolved = os.path.normpath(os.path.join(out_layer_dir, path))
                        if not os.path.isfile(resolved):
                            broken.append(f"{p.GetPath()}.{a.GetName()} -> {resolved} (raw: {path})")
            self.assertEqual(
                broken,
                [],
                f"Output layer still contains unresolvable asset paths: {broken}",
            )
