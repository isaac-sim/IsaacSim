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
from isaacsim.asset.transformer.manager import (
    _MAX_VARIANT_SELECTIONS_PROBED,
    AssetTransformerManager,
    RuleRegistry,
    _collect_assets,
)
from isaacsim.asset.transformer.models import RuleConfigurationParam, RuleProfile, RuleSpec
from isaacsim.asset.transformer.rule_interface import RuleInterface
from pxr import Sdf, Usd, UsdGeom


def _write_1x1_png(path: str) -> None:
    """Write a valid 1x1 RGBA PNG so any consumer can decode it.

    PNG layout: 8-byte signature + IHDR + IDAT + IEND chunks with correct CRC32
    values. Used by tests that copy texture assets and verify the copied files
    remain valid images on disk.

    Args:
        path: Destination PNG path.
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


def _package_temp_dir() -> tempfile.TemporaryDirectory:
    """Temporary directory for tests that open the transformed output package.

    An open USD layer is an OS-level file lock on Windows, so the crate-backed
    ``base.usd`` a test opens to inspect cannot be unlinked while that handle is
    alive. Cleanup is therefore best-effort, matching the
    ``shutil.rmtree(..., ignore_errors=True)`` convention used by the rules
    tests.

    Returns:
        Context manager yielding the temporary directory path.
    """
    return tempfile.TemporaryDirectory(ignore_cleanup_errors=True)


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

    def GetCompositionErrors(self) -> list:  # noqa: N802
        """Report no composition errors (test stub).

        Returns:
            Always an empty list.
        """
        return []

    def GetPseudoRoot(self) -> None:  # noqa: N802
        """Report no pseudo-root, short-circuiting the variant-error sweep.

        Returns:
            Always ``None``.
        """
        return


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
            # This fake USD stage tests rule execution; publication requires real layers.
            report = mgr._run_in_directory(input_stage="in.usda", profile=profile, package_root=None)
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

    async def test_manager_runs_rule_deferred_deletion_after_release(self) -> None:
        """Manager deletes a rule-deferred file only after dropping the working stage.

        Regression for the Windows ``[WinError 5] Access is denied`` failure: a
        rule cannot delete a file that backs the manager-owned working stage, so
        it registers the path via ``request_deletion`` and the manager removes it
        after releasing the stage. Uses real USD (no mocks) so the file handle is
        genuinely held and released.
        """

        class _ConvertToUsdaRule(RuleInterface):
            """Convert the working stage's root layer to .usda and defer the original's deletion."""

            def process_rule(self) -> str:
                root_layer = self.source_stage.GetRootLayer()
                original = root_layer.realPath
                new_path = os.path.splitext(original)[0] + ".usda"
                root_layer.Export(new_path)
                # The rule cannot delete ``original`` itself: it is the manager's
                # working stage. Drop our handle and hand the path to the manager.
                self.source_stage = None
                self.request_deletion(original)
                return new_path

            def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
                return []

        with tempfile.TemporaryDirectory() as root:
            input_path = os.path.join(root, "input.usda")
            stage = Usd.Stage.CreateNew(input_path)
            UsdGeom.Xform.Define(stage, "/World")
            stage.GetRootLayer().Save()
            del stage

            reg = RuleRegistry()
            reg.register(_ConvertToUsdaRule)
            rule_type = f"{_ConvertToUsdaRule.__module__}.{_ConvertToUsdaRule.__qualname__}"

            out_root = os.path.join(root, "out")
            profile = RuleProfile(
                profile_name="p",
                rules=[RuleSpec(name="convert", type=rule_type, destination="payloads")],
                base_name="base.usd",
            )
            mgr = AssetTransformerManager()
            report = mgr.run(input_stage=input_path, profile=profile, package_root=out_root)

            self.assertTrue(report.results[0].success, report.results[0].error)
            base_usd = os.path.join(out_root, "payloads", "base.usd")
            base_usda = os.path.join(out_root, "payloads", "base.usda")
            self.assertTrue(os.path.exists(base_usda), f"converted .usda missing: {base_usda}")
            self.assertFalse(
                os.path.exists(base_usd),
                "manager must delete the rule-deferred file after releasing the working stage",
            )

    async def _build_composed_fixture(self, fixture_root: str) -> str:
        """Author a source stage whose content arrives through all three composition arcs.

        Layout::

            <fixture_root>/root.usda  <-- subLayers ./sub.usda, references ./ref.usda,
                                          payloads ./pay.usda, and binds ./grid.png
            <fixture_root>/sub.usda   <-- contributes /Cube
            <fixture_root>/ref.usda   <-- contributes /Robot/RefMesh/Geom
            <fixture_root>/pay.usda   <-- contributes /Robot/PayNode/PayGeom

        Args:
            fixture_root: Directory to author the fixture into.

        Returns:
            Path to the root layer.
        """
        ref_path = os.path.join(fixture_root, "ref.usda")
        ref_stage = Usd.Stage.CreateNew(ref_path)
        UsdGeom.Mesh.Define(ref_stage, "/RefMesh")
        UsdGeom.Scope.Define(ref_stage, "/RefMesh/Geom")
        ref_stage.SetDefaultPrim(ref_stage.GetPrimAtPath("/RefMesh"))
        ref_stage.GetRootLayer().Save()

        sub_path = os.path.join(fixture_root, "sub.usda")
        sub_stage = Usd.Stage.CreateNew(sub_path)
        UsdGeom.Cube.Define(sub_stage, "/Cube")
        sub_stage.GetRootLayer().Save()

        pay_path = os.path.join(fixture_root, "pay.usda")
        pay_stage = Usd.Stage.CreateNew(pay_path)
        UsdGeom.Xform.Define(pay_stage, "/PayShape")
        UsdGeom.Sphere.Define(pay_stage, "/PayShape/PayGeom")
        pay_stage.SetDefaultPrim(pay_stage.GetPrimAtPath("/PayShape"))
        pay_stage.GetRootLayer().Save()

        _write_1x1_png(os.path.join(fixture_root, "grid.png"))

        root_path = os.path.join(fixture_root, "root.usda")
        root_stage = Usd.Stage.CreateNew(root_path)
        root_stage.GetRootLayer().subLayerPaths.append("./sub.usda")
        robot = UsdGeom.Xform.Define(root_stage, "/Robot")
        UsdGeom.Xform.Define(root_stage, "/Robot/RefMesh").GetPrim().GetReferences().AddReference("./ref.usda")
        UsdGeom.Xform.Define(root_stage, "/Robot/PayNode").GetPrim().GetPayloads().AddPayload("./pay.usda")
        attr = robot.GetPrim().CreateAttribute("inputs:diffuse", Sdf.ValueTypeNames.Asset)
        attr.Set(Sdf.AssetPath("./grid.png"))
        root_stage.GetRootLayer().Save()
        return root_path

    async def test_run_without_flatten_source_preserves_composed_prims(self) -> None:
        """The default ``flatten_source=False`` path must keep every composed prim.

        Exporting only the source root layer into ``payloads/`` leaves every
        relative sublayer, reference, and payload arc anchored against a
        directory that does not contain those layers, so all content arriving
        through an arc silently disappears from the output. The composed prim
        set must survive and no arc may be left unresolved.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)
            source_path = await self._build_composed_fixture(fixture_root)

            # Bind the stage: traversing a temporary would expire mid-iteration.
            source_stage = Usd.Stage.Open(source_path)
            expected = sorted(p.GetPath().pathString for p in source_stage.Traverse())
            # Guard the fixture itself: without arc content there is nothing to lose.
            self.assertIn("/Cube", expected, "fixture must contribute a sublayer prim")
            self.assertIn("/Robot/RefMesh/Geom", expected, "fixture must contribute a reference child")
            self.assertIn("/Robot/PayNode/PayGeom", expected, "fixture must contribute a payload child")

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertIsNotNone(report.output_stage_path)
            out_stage = Usd.Stage.Open(report.output_stage_path)
            actual = sorted(p.GetPath().pathString for p in out_stage.Traverse())

            composition_errors = [str(e) for e in out_stage.GetCompositionErrors()]
            self.assertEqual(
                composition_errors,
                [],
                f"output stage has unresolved composition arcs: {composition_errors}",
            )
            self.assertEqual(
                actual,
                expected,
                f"prims dropped from the output stage: {sorted(set(expected) - set(actual))}",
            )
            self.assertEqual(report.composition_errors, [], "report must not hide composition errors")

    async def test_run_without_flatten_source_keeps_arcs_resolvable(self) -> None:
        """The non-flatten path must keep arcs as arcs *and* keep them resolving.

        This is what distinguishes ``flatten_source=False`` from ``True``: the
        output stays a composed stage. Its arc targets are not collected into
        the package, so each arc has to be re-anchored to the source file it
        names rather than left pointing inside ``payloads/``.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)
            source_path = await self._build_composed_fixture(fixture_root)

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            out_layer = Sdf.Layer.FindOrOpen(report.output_stage_path)
            self.assertTrue(out_layer.subLayerPaths, "sublayer arc must survive in the output root layer")

            # Every arc target must resolve to a real file from the output layer.
            out_dir = os.path.dirname(report.output_stage_path)
            arc_paths = list(out_layer.subLayerPaths)
            for prim_path in ("/Robot/RefMesh", "/Robot/PayNode"):
                prim_spec = out_layer.GetPrimAtPath(prim_path)
                self.assertIsNotNone(prim_spec, f"{prim_path} missing from output layer")
                arc_paths.extend(ref.assetPath for ref in prim_spec.referenceList.prependedItems)
                arc_paths.extend(pay.assetPath for pay in prim_spec.payloadList.prependedItems)

            self.assertEqual(len(arc_paths), 3, f"expected sublayer, reference and payload arcs: {arc_paths}")
            for arc_path in arc_paths:
                resolved = os.path.normpath(os.path.join(out_dir, arc_path))
                self.assertTrue(os.path.isfile(resolved), f"arc target does not resolve: {arc_path} -> {resolved}")

            # Non-layer assets are still collected into the package.
            self.assertIn("grid.png", os.listdir(os.path.join(out_root, "source_assets")))

    async def test_run_with_flatten_source_still_bakes_composition(self) -> None:
        """Control: ``flatten_source=True`` keeps baking arcs and stays self-contained.

        Guards against a fix that accidentally routes the flatten path through
        layer localization, which would ship dependency layers the flattened
        output does not need.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)
            source_path = await self._build_composed_fixture(fixture_root)

            source_stage = Usd.Stage.Open(source_path)
            expected = sorted(p.GetPath().pathString for p in source_stage.Traverse())

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=True, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            out_stage = Usd.Stage.Open(report.output_stage_path)
            self.assertEqual(sorted(p.GetPath().pathString for p in out_stage.Traverse()), expected)
            self.assertEqual([str(e) for e in out_stage.GetCompositionErrors()], [])

            # Flatten bakes the arcs away, so no dependency layer should be copied.
            out_layer = Sdf.Layer.FindOrOpen(report.output_stage_path)
            self.assertEqual(list(out_layer.subLayerPaths), [], "flattened output must not declare sublayers")
            copied = os.listdir(os.path.join(out_root, "source_assets"))
            self.assertIn("grid.png", copied, "texture must still be collected")
            self.assertNotIn("sub.usda", copied, "flattened output must not ship dependency layers")

    async def test_run_reports_composition_errors_instead_of_failing_silently(self) -> None:
        """A source stage with a genuinely missing arc target must surface on the report.

        The manager previously returned a report with ``output_stage_path`` set
        and no error of any kind while the output stage was missing content.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            UsdGeom.Xform.Define(stage, "/Robot")
            # Arc target that does not exist anywhere, so no amount of collecting
            # can resolve it. The manager must say so rather than stay silent.
            UsdGeom.Xform.Define(stage, "/Robot/Missing").GetPrim().GetReferences().AddReference("./absent.usda")
            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertTrue(report.composition_errors, "unresolvable arc must be reported")
            self.assertTrue(
                any("absent.usda" in err for err in report.composition_errors),
                f"report must name the unresolved arc: {report.composition_errors}",
            )

    async def test_run_reports_composition_errors_inside_unselected_variants(self) -> None:
        """A broken arc inside a variant option nobody selected must still be reported.

        ``Usd.Stage.GetCompositionErrors`` only sees arcs the current variant
        selections compose, so a package can lose everything behind an
        unselected option while the report reads clean.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            stage.SetDefaultPrim(robot.GetPrim())
            variant_set = robot.GetPrim().GetVariantSets().AddVariantSet("Configuration")
            variant_set.AddVariant("good")
            variant_set.AddVariant("broken")

            variant_set.SetVariantSelection("broken")
            with variant_set.GetVariantEditContext():
                broken = UsdGeom.Xform.Define(stage, "/Robot/Broken")
                broken.GetPrim().GetReferences().AddReference("./absent.usda")

            # Leave "good" selected so the broken option is never composed.
            variant_set.SetVariantSelection("good")
            with variant_set.GetVariantEditContext():
                UsdGeom.Sphere.Define(stage, "/Robot/Good")

            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            # Guard the fixture: the default composition must itself be clean,
            # otherwise the assertion below would pass for the wrong reason.
            self.assertIsNone(report.output_stage_path, "invalid composition must not be published")
            self.assertEqual(os.listdir(out_root), [])
            fixture_stage = Usd.Stage.Open(source_path)
            self.assertEqual(
                [str(e) for e in fixture_stage.GetCompositionErrors()],
                [],
                "fixture must keep the default selection resolvable",
            )

            self.assertTrue(
                any("absent.usda" in err for err in report.composition_errors),
                f"error inside the unselected option must be reported: {report.composition_errors}",
            )
            self.assertTrue(
                any("{Configuration=broken}" in err for err in report.composition_errors),
                f"report must attribute the error to the option that exposed it: {report.composition_errors}",
            )

    async def test_run_reports_composition_errors_outside_the_default_prim(self) -> None:
        """A broken variant on a sibling of the default prim must still be reported.

        Sweeping from the default prim instead of the pseudo-root silently skips
        every sibling root, and stages pairing a default prim with
        ``/Environment`` or ``/Looks`` are the norm. The partial sweep that
        results is indistinguishable from a clean one.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            # A default prim is what narrows the sweep, so the fixture must set
            # one for this test to exercise the failure at all.
            stage.SetDefaultPrim(robot.GetPrim())
            UsdGeom.Sphere.Define(stage, "/Robot/Good")

            environment = UsdGeom.Xform.Define(stage, "/Environment")
            variant_set = environment.GetPrim().GetVariantSets().AddVariantSet("EnvConfig")
            variant_set.AddVariant("plain")
            variant_set.AddVariant("broken")
            variant_set.SetVariantSelection("broken")
            with variant_set.GetVariantEditContext():
                UsdGeom.Xform.Define(stage, "/Environment/Broken").GetPrim().GetReferences().AddReference(
                    "./absent_env.usda"
                )
            variant_set.SetVariantSelection("plain")

            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertIsNone(report.output_stage_path, "invalid composition must not be published")
            self.assertEqual(os.listdir(out_root), [])
            fixture_stage = Usd.Stage.Open(source_path)
            self.assertEqual(
                [str(e) for e in fixture_stage.GetCompositionErrors()],
                [],
                "fixture must keep the default composition resolvable",
            )
            self.assertTrue(
                any("absent_env.usda" in err for err in report.composition_errors),
                f"error outside the default prim must be reported: {report.composition_errors}",
            )
            self.assertFalse(
                report.composition_errors_truncated,
                "a two-option sweep must not report itself as truncated",
            )

    async def test_run_flags_a_truncated_composition_error_sweep(self) -> None:
        """Exhausting the selection budget must be reported, not hidden.

        The sweep is bounded, so a large enough asset stops being probed part
        way through. Returning the partial result unmarked makes a bigger asset
        look cleaner than a smaller one and leaves the caller unable to tell
        "clean" from "gave up".
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            stage.SetDefaultPrim(robot.GetPrim())
            variant_set = robot.GetPrim().GetVariantSets().AddVariantSet("Configuration")
            # One option past the budget is enough to force the early exit.
            option_count = _MAX_VARIANT_SELECTIONS_PROBED + 2
            for index in range(option_count):
                variant_set.AddVariant(f"opt_{index:03d}")
            variant_set.SetVariantSelection("opt_000")

            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertTrue(
                report.composition_errors_truncated,
                f"a sweep that stopped after {_MAX_VARIANT_SELECTIONS_PROBED} selections must say so",
            )
            self.assertIn(
                "composition_errors_truncated",
                report.to_dict(),
                "the truncation flag must reach the serialized report",
            )

    async def test_run_does_not_reanchor_a_broken_arc_to_an_unrelated_file(self) -> None:
        """A broken arc must be reported, not quietly repointed at a same-named file.

        Re-anchoring walks candidate anchor directories to find the file the
        source named. Accepting a match under a sublayer's directory turns a
        genuinely broken arc into a resolving arc pointing at different
        content, which is a silent wrong answer rather than a reported error.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            far_dir = os.path.join(fixture_root, "far")
            os.makedirs(far_dir, exist_ok=True)

            # A decoy with the name the root layer asks for, but somewhere the
            # root layer's own relative path could never reach.
            decoy = Usd.Stage.CreateNew(os.path.join(far_dir, "shared.usda"))
            UsdGeom.Xform.Define(decoy, "/Decoy")
            decoy.GetRootLayer().Save()
            del decoy

            sub_path = os.path.join(far_dir, "sub.usda")
            sub = Usd.Stage.CreateNew(sub_path)
            UsdGeom.Xform.Define(sub, "/Sub")
            sub.GetRootLayer().Save()
            del sub

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            stage.SetDefaultPrim(robot.GetPrim())
            # The sublayer is what puts far/ into the candidate anchor list.
            stage.GetRootLayer().subLayerPaths.append("./far/sub.usda")
            # Genuinely broken: there is no shared.usda beside the root layer.
            UsdGeom.Xform.Define(stage, "/Robot/Missing").GetPrim().GetReferences().AddReference("./shared.usda")
            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertIsNone(report.output_stage_path, "a broken arc must prevent publication")
            self.assertEqual(os.listdir(out_root), [])
            self.assertTrue(
                any("shared.usda" in err for err in report.composition_errors),
                f"broken arc must be reported instead: {report.composition_errors}",
            )

    async def test_run_survives_a_relpath_failure_across_volumes(self) -> None:
        """A path that cannot be made relative must not abort the transform.

        ``os.path.relpath`` raises on Windows when the source tree and the
        output package sit on different drives or across a UNC share, which is
        an ordinary layout. The arc still needs re-anchoring; failing the whole
        run over a path-formatting limit does not help anyone.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            dep_path = os.path.join(fixture_root, "dep.usda")
            dep = Usd.Stage.CreateNew(dep_path)
            dep.SetDefaultPrim(UsdGeom.Xform.Define(dep, "/Dep").GetPrim())
            dep.GetRootLayer().Save()
            del dep

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            stage.SetDefaultPrim(robot.GetPrim())
            UsdGeom.Xform.Define(stage, "/Robot/Dep").GetPrim().GetReferences().AddReference("./dep.usda")
            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")

            real_relpath = os.path.relpath

            def _relpath_across_volumes(path: str, start: str | None = None) -> str:
                """Stand in for ntpath's cross-mount refusal.

                Args:
                    path: Target path.
                    start: Directory to relate the target to.

                Returns:
                    The relative path for unrelated calls.

                Raises:
                    ValueError: When asked to relate the dependency to the output.
                """
                if start and "dep.usda" in path:
                    raise ValueError("path is on mount 'D:', start on mount 'C:'")
                return real_relpath(path, start) if start else real_relpath(path)

            with patch("isaacsim.asset.transformer.manager.os.path.relpath", _relpath_across_volumes):
                report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertIsNotNone(report.output_stage_path, "transform must still produce an output stage")
            out_layer = Sdf.Layer.FindOrOpen(report.output_stage_path)
            arcs = [str(ref.assetPath) for ref in out_layer.GetPrimAtPath("/Robot/Dep").referenceList.prependedItems]
            self.assertTrue(
                any(arc.endswith("dep.usda") for arc in arcs),
                f"arc must still name the dependency: {arcs}",
            )
            self.assertEqual(
                report.composition_errors,
                [],
                f"the fallback target must still resolve: {report.composition_errors}",
            )

    async def test_run_still_returns_a_report_when_the_error_sweep_fails(self) -> None:
        """A failed diagnostic must not cost the caller a successful run's report.

        The output is already saved by the time composition errors are
        collected, so letting that collection propagate would discard a report
        the caller has otherwise earned and leave every rule result unseen.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)

            source_path = os.path.join(fixture_root, "root.usda")
            stage = Usd.Stage.CreateNew(source_path)
            robot = UsdGeom.Xform.Define(stage, "/Robot")
            stage.SetDefaultPrim(robot.GetPrim())
            UsdGeom.Sphere.Define(stage, "/Robot/Good")
            stage.GetRootLayer().Save()
            del stage

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")

            def _explode(_stage: Usd.Stage) -> None:
                """Simulate a USD failure inside the diagnostic sweep.

                Args:
                    _stage: Unused stage argument.

                Raises:
                    RuntimeError: Always.
                """
                raise RuntimeError("simulated USD failure inside the diagnostic sweep")

            with patch("isaacsim.asset.transformer.manager._collect_composition_errors", _explode):
                report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            self.assertIsNotNone(report, "caller must still receive a report")
            self.assertIsNotNone(report.finished_at, "report must be closed even when the sweep fails")
            self.assertTrue(
                report.composition_errors_truncated,
                "an inconclusive sweep must not be reported as a complete one",
            )
            self.assertEqual(
                [],
                report.composition_errors,
                "a failed sweep must not invent an unresolved arc the output may not have",
            )

    async def test_run_fails_when_the_source_stage_cannot_be_exported(self) -> None:
        """A failed export must not leave an earlier run's base layer to be transformed.

        The export is what places the source content in the output package. If
        it fails while an earlier run's base layer still sits at that path, the
        pipeline transforms that stale content and reports success for an output
        the source never produced. A stale layer is what makes this silent: with
        nothing at the path, opening the working stage fails loudly on its own.
        """
        with _package_temp_dir() as root:
            payloads_dir = os.path.join(root, "out", "payloads")
            os.makedirs(payloads_dir, exist_ok=True)
            stale_stage = Usd.Stage.CreateNew(os.path.join(payloads_dir, "base.usd"))
            stale_stage.SetDefaultPrim(UsdGeom.Xform.Define(stale_stage, "/Stale").GetPrim())
            stale_stage.GetRootLayer().Save()
            del stale_stage

            failing_layer = types.SimpleNamespace(
                realPath=os.path.join(root, "src", "root.usda"),
                Export=lambda _path: False,
            )
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")

            with self.assertRaisesRegex(RuntimeError, "Failed to export source stage"):
                AssetTransformerManager().run(
                    types.SimpleNamespace(GetRootLayer=lambda: failing_layer),
                    profile,
                    package_root=os.path.join(root, "out"),
                )

    async def _build_nested_fixture(self, fixture_root: str) -> str:
        """Author a source stage whose dependency layer has a relative arc of its own.

        Layout::

            <fixture_root>/root.usda        <-- references ./sub/a.usda
            <fixture_root>/sub/a.usda       <-- references ../shared/b.usda
            <fixture_root>/shared/b.usda    <-- contributes /B/Geom

        The nesting is the point. ``a.usda`` resolves ``../shared/b.usda``
        against its own directory, so anything that relocates ``a.usda``
        without rewriting its arcs strands ``b.usda``.

        Args:
            fixture_root: Directory to author the fixture into.

        Returns:
            Path to the root layer.
        """
        shared_dir = os.path.join(fixture_root, "shared")
        sub_dir = os.path.join(fixture_root, "sub")
        os.makedirs(shared_dir, exist_ok=True)
        os.makedirs(sub_dir, exist_ok=True)

        b_stage = Usd.Stage.CreateNew(os.path.join(shared_dir, "b.usda"))
        UsdGeom.Xform.Define(b_stage, "/B")
        UsdGeom.Sphere.Define(b_stage, "/B/Geom")
        b_stage.SetDefaultPrim(b_stage.GetPrimAtPath("/B"))
        b_stage.GetRootLayer().Save()

        a_stage = Usd.Stage.CreateNew(os.path.join(sub_dir, "a.usda"))
        a_root = UsdGeom.Xform.Define(a_stage, "/A")
        UsdGeom.Xform.Define(a_stage, "/A/FromShared").GetPrim().GetReferences().AddReference("../shared/b.usda")
        a_stage.SetDefaultPrim(a_root.GetPrim())
        a_stage.GetRootLayer().Save()

        root_path = os.path.join(fixture_root, "root.usda")
        root_stage = Usd.Stage.CreateNew(root_path)
        world = UsdGeom.Xform.Define(root_stage, "/World")
        UsdGeom.Xform.Define(root_stage, "/World/Nested").GetPrim().GetReferences().AddReference("./sub/a.usda")
        root_stage.SetDefaultPrim(world.GetPrim())
        root_stage.GetRootLayer().Save()
        return root_path

    async def test_run_without_flatten_source_keeps_nested_dependency_arcs_resolvable(self) -> None:
        """A dependency layer's own relative arcs must keep resolving in the output.

        Every other fixture here uses dependency layers that sit beside the
        root and carry no arcs of their own, which cannot detect an approach
        that relocates dependency layers without rewriting what they point at.
        Copying layers flat into ``source_assets/`` strands every relative arc
        inside them.
        """
        with _package_temp_dir() as root:
            fixture_root = os.path.join(root, "src")
            os.makedirs(fixture_root, exist_ok=True)
            source_path = await self._build_nested_fixture(fixture_root)

            source_stage = Usd.Stage.Open(source_path)
            expected = sorted(p.GetPath().pathString for p in source_stage.Traverse())
            # Guard the fixture: the deepest prim only exists via the nested arc.
            self.assertIn(
                "/World/Nested/FromShared/Geom",
                expected,
                "fixture must contribute a prim through a nested arc",
            )

            out_root = os.path.join(root, "out")
            profile = RuleProfile(profile_name="p", flatten_source=False, rules=[], base_name="base.usd")
            report = AssetTransformerManager().run(source_path, profile, package_root=out_root)

            out_stage = Usd.Stage.Open(report.output_stage_path)
            actual = sorted(p.GetPath().pathString for p in out_stage.Traverse())
            self.assertEqual(
                actual,
                expected,
                f"prims lost behind the nested arc: {sorted(set(expected) - set(actual))}",
            )
            self.assertEqual(
                report.composition_errors,
                [],
                f"nested dependency arc left unresolved: {report.composition_errors}",
            )

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

    async def test_collect_assets_preserves_prior_collection_on_basename_collision(self) -> None:
        """Regression: collecting twice into one package_root must not overwrite the first asset.

        Two source stages each reference their own ``tex.png``. Collecting both
        into the same ``package_root`` previously let the second ``shutil.copy2``
        destroy the first, so the first stage silently resolved to the wrong
        image. The second asset must land under a suffixed name, the first must
        be untouched, and the collision must be reported.
        """

        def _make_stage(directory: str, payload: bytes) -> str:
            os.makedirs(directory, exist_ok=True)
            texture_path = os.path.join(directory, "tex.png")
            _write_1x1_png(texture_path)
            with open(texture_path, "ab") as handle:
                handle.write(payload)
            # Same size and mtime across fixtures, as mtime-preserving copies produce,
            # so only a byte comparison can tell them apart.
            os.utime(texture_path, (1700000000, 1700000000))
            stage_path = os.path.join(directory, "scene.usda")
            stage = Usd.Stage.CreateNew(stage_path)
            prim = UsdGeom.Xform.Define(stage, "/World")
            attr = prim.GetPrim().CreateAttribute("inputs:diffuse", Sdf.ValueTypeNames.Asset)
            attr.Set(Sdf.AssetPath("./tex.png"))
            stage.GetRootLayer().Save()
            return stage_path

        with tempfile.TemporaryDirectory() as root:
            package_root = os.path.join(root, "pkg")
            assets_dir = os.path.join(package_root, "source_assets")
            # Equal-length payloads so filecmp cannot decide on size alone.
            first_stage = _make_stage(os.path.join(root, "first"), b"AAAAA")
            second_stage = _make_stage(os.path.join(root, "second"), b"BBBBB")
            third_stage = _make_stage(os.path.join(root, "third"), b"BBBBB")
            fourth_stage = _make_stage(os.path.join(root, "fourth"), b"AAAAA")

            with open(os.path.join(root, "first", "tex.png"), "rb") as handle:
                first_bytes = handle.read()
            with open(os.path.join(root, "second", "tex.png"), "rb") as handle:
                second_bytes = handle.read()

            _collect_assets(Sdf.Layer.FindOrOpen(first_stage), package_root, source_layer_path=first_stage)
            collected_first = os.path.join(assets_dir, "tex.png")
            self.assertTrue(os.path.isfile(collected_first), "first collection must produce source_assets/tex.png")

            with self.assertLogs("isaacsim.asset.transformer.manager", level="WARNING") as captured:
                _collect_assets(Sdf.Layer.FindOrOpen(second_stage), package_root, source_layer_path=second_stage)

            with open(collected_first, "rb") as handle:
                preserved = handle.read()
            self.assertEqual(
                preserved,
                first_bytes,
                "second collection must not overwrite the asset collected by the first",
            )
            collected_second = os.path.join(assets_dir, "tex_1.png")
            self.assertTrue(os.path.isfile(collected_second), "colliding asset must be collected under a suffix")
            with open(collected_second, "rb") as handle:
                self.assertEqual(handle.read(), second_bytes)
            self.assertTrue(
                any("already holds different content" in message for message in captured.output),
                f"collision must be reported at WARNING level, got: {captured.output}",
            )

            # The second stage must now point at the suffixed copy, not at tex.png.
            second_reopened = Usd.Stage.Open(second_stage)
            remapped = [
                str(attribute.Get().path)
                for prim in second_reopened.Traverse()
                for attribute in prim.GetAttributes()
                if isinstance(attribute.Get(), Sdf.AssetPath)
            ]
            self.assertTrue(
                any("tex_1.png" in path for path in remapped),
                f"second stage must reference the suffixed asset, got: {remapped}",
            )

            # Identical content reuses the existing copy: no extra file, no copy, still reported.
            with patch("isaacsim.asset.transformer.manager.shutil.copy2") as copy_spy:
                with self.assertLogs("isaacsim.asset.transformer.manager", level="WARNING") as reuse_logs:
                    _collect_assets(Sdf.Layer.FindOrOpen(third_stage), package_root, source_layer_path=third_stage)
            copy_spy.assert_not_called()
            self.assertEqual(
                sorted(os.listdir(assets_dir)),
                ["tex.png", "tex_1.png"],
                "byte-identical content must reuse the existing collected asset",
            )
            self.assertTrue(
                any("reusing" in message for message in reuse_logs.output),
                f"reuse must be reported at WARNING level, got: {reuse_logs.output}",
            )
            third_reopened = Usd.Stage.Open(third_stage)
            third_remapped = [
                str(attribute.Get().path)
                for prim in third_reopened.Traverse()
                for attribute in prim.GetAttributes()
                if isinstance(attribute.Get(), Sdf.AssetPath)
            ]
            self.assertTrue(
                any("tex_1.png" in path for path in third_remapped),
                f"third stage must reference the reused suffixed asset, got: {third_remapped}",
            )

            # Identical content at the UNSUFFIXED destination is also a reuse, and also reported.
            with patch("isaacsim.asset.transformer.manager.shutil.copy2") as base_copy_spy:
                with self.assertLogs("isaacsim.asset.transformer.manager", level="WARNING") as base_logs:
                    _collect_assets(Sdf.Layer.FindOrOpen(fourth_stage), package_root, source_layer_path=fourth_stage)
            base_copy_spy.assert_not_called()
            self.assertEqual(sorted(os.listdir(assets_dir)), ["tex.png", "tex_1.png"])
            self.assertTrue(
                any("identical content" in message for message in base_logs.output),
                f"reuse at the unsuffixed destination must warn, got: {base_logs.output}",
            )

            fourth_reopened = Usd.Stage.Open(fourth_stage)
            fourth_remapped = [
                str(attribute.Get().path)
                for prim in fourth_reopened.Traverse()
                for attribute in prim.GetAttributes()
                if isinstance(attribute.Get(), Sdf.AssetPath)
            ]
            self.assertTrue(
                any("source_assets" in path and path.endswith("tex.png") for path in fourth_remapped),
                f"fourth stage must reference the reused base asset, got: {fourth_remapped}",
            )
            self.assertFalse(
                any("tex_1.png" in path for path in fourth_remapped),
                f"fourth stage must not reference the suffixed asset, got: {fourth_remapped}",
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
