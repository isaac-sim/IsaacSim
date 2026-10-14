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

"""Regression coverage for replacing existing transformer packages."""

import errno
import gc
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from isaacsim.asset.transformer import (
    AssetTransformerManager,
    RuleConfigurationParam,
    RuleInterface,
    RuleProfile,
    RuleRegistry,
    RuleSpec,
)
from pxr import Sdf, Usd


class _AdditionalLayerRule(RuleInterface):
    """Author a second generated layer and an absolute internal reference."""

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return no configuration parameters.

        Returns:
            Empty parameter list.
        """
        return []

    def process_rule(self) -> None:
        """Generate the reference target for publication and rollback tests."""
        path = str(Path(self.package_root) / "extra.usda")
        extra = Usd.Stage.CreateNew(path)
        extra.SetDefaultPrim(extra.DefinePrim("/Extra", "Xform"))
        extra.DefinePrim("/Extra/Current")
        extra.GetRootLayer().Save()
        self.source_stage.GetDefaultPrim().GetReferences().AddReference(path)
        self.add_affected_stage(path)


class _CyclicStageRule(_AdditionalLayerRule):
    """Model a recursive rule closure retaining its working stage until collection."""

    def process_rule(self) -> None:
        """Retain a stage cycle and optionally produce an unresolved reference."""
        self._cycle = self
        if self.args["params"].get("fail"):
            self.source_stage.GetDefaultPrim().GetReferences().AddReference("./missing.usda")


class TestPackageReplacement(unittest.TestCase):
    """Exercise replacement through the public standalone manager API."""

    def test_replacement_refreshes_loaded_layer_and_preserves_unrelated_files(self) -> None:
        """Replace existing opinions even when the old output is still open."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = Usd.Stage.CreateInMemory()
            source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
            source.GetRootLayer().Export(str(root / "input.usda"))
            profile = RuleProfile(profile_name="replacement", base_name="base.usda")
            manager = AssetTransformerManager()
            first = manager.run(str(root / "input.usda"), profile, str(root / "out"))
            old_output = Usd.Stage.Open(first.output_stage_path)
            old_output.DefinePrim("/Robot/Stale")
            old_output.GetRootLayer().Save()
            unrelated = root / "out" / "notes.txt"
            unrelated.write_text("user-owned", encoding="utf-8")
            second = manager.run(str(root / "input.usda"), profile, str(root / "out"))
            self.assertEqual(second.output_stage_path, first.output_stage_path)
            self.assertFalse(old_output.GetPrimAtPath("/Robot/Stale"))
            self.assertEqual(unrelated.read_text(encoding="utf-8"), "user-owned")
            self.assertEqual(second.composition_errors, [])
            self.assertEqual(list((root / "out").glob(".asset-*")), [])
            old_output = None
            source = None

    def test_failed_replacement_preserves_previous_package(self) -> None:
        """A rule failure must not overwrite the last successful export."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = Usd.Stage.CreateInMemory()
            source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
            source.GetRootLayer().Export(str(root / "input.usda"))
            profile = RuleProfile(profile_name="replacement", base_name="base.usda")
            manager = AssetTransformerManager()
            first = manager.run(str(root / "input.usda"), profile, str(root / "out"))
            before = Path(first.output_stage_path).read_bytes()
            profile.rules = [RuleSpec(name="Failure", type="missing.Rule")]
            report = manager.run(str(root / "input.usda"), profile, str(root / "out"))
            self.assertFalse(report.results[0].success)
            self.assertIsNone(report.output_stage_path)
            self.assertEqual(Path(first.output_stage_path).read_bytes(), before)
            self.assertEqual(list((root / "out").glob(".asset-*")), [])
            source = None

    def test_replacement_preserves_external_composition_paths(self) -> None:
        """Nonflattened source references still resolve after staging is removed."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            external = Usd.Stage.CreateNew(str(root / "external.usda"))
            external.SetDefaultPrim(external.DefinePrim("/Model", "Xform"))
            external.DefinePrim("/Model/ExternalChild")
            external.GetRootLayer().Save()
            source = Usd.Stage.CreateNew(str(root / "input.usda"))
            prim = source.DefinePrim("/Robot", "Xform")
            source.SetDefaultPrim(prim)
            prim.GetReferences().AddReference("./external.usda")
            source.GetRootLayer().Save()
            profile = RuleProfile(profile_name="replacement", base_name="base.usda", flatten_source=False)
            output = root / "nested" / "out"
            output.mkdir(parents=True)
            (output / "notes.txt").write_text("keep", encoding="utf-8")
            report = AssetTransformerManager().run(str(root / "input.usda"), profile, str(output))
            reopened = Usd.Stage.Open(report.output_stage_path)
            self.assertTrue(reopened.GetPrimAtPath("/Robot/ExternalChild"))
            self.assertEqual(report.composition_errors, [])
            self.assertNotIn(".asset-transformer-", reopened.GetRootLayer().ExportToString())
            reopened = None
            source = None
            external = None

    def test_internal_absolute_paths_and_publication_rollback(self) -> None:
        """Publish internal arcs correctly and undo a later filesystem failure."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = Usd.Stage.CreateNew(str(root / "input.usda"))
            source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
            source.GetRootLayer().Save()
            registry = RuleRegistry()
            registry.register(_AdditionalLayerRule)
            profile = RuleProfile(
                profile_name="replacement",
                base_name="base.usda",
                rules=[
                    RuleSpec(
                        name="Extra", type=f"{_AdditionalLayerRule.__module__}.{_AdditionalLayerRule.__qualname__}"
                    )
                ],
            )
            output = root / "out"
            output.mkdir()
            (output / "notes.txt").write_text("keep", encoding="utf-8")
            manager = AssetTransformerManager(registry)
            report = manager.run(str(root / "input.usda"), profile, str(output))
            opened = Usd.Stage.Open(report.output_stage_path)
            self.assertTrue(opened.GetPrimAtPath("/Robot/Current"))
            self.assertNotIn(".asset-transformer-", opened.GetRootLayer().ExportToString())
            self.assertEqual(report.results[0].affected_stages, [str(output / "extra.usda")])
            before = {path: path.read_bytes() for path in output.rglob("*") if path.is_file()}
            cached_before = opened.GetRootLayer().ExportToString()
            source.DefinePrim("/Robot/NewContent")
            source.GetRootLayer().Save()
            real_replace = os.replace
            calls = 0
            fail_calls = {2}

            def fail_second_replace(src: str, dst: str) -> None:
                """Simulate failure after one generated file has been published.

                Args:
                    src: Prepared file path.
                    dst: Published file path.

                Raises:
                    OSError: On the second replacement.
                """
                nonlocal calls
                calls += 1
                if calls in fail_calls:
                    raise OSError("simulated publication failure")
                real_replace(src, dst)

            with patch("isaacsim.asset.transformer.manager.os.replace", side_effect=fail_second_replace):
                with self.assertRaisesRegex(OSError, "simulated publication failure"):
                    manager.run(str(root / "input.usda"), profile, str(output))
            self.assertGreaterEqual(calls, 3)  # Includes restoring the first file.
            self.assertEqual({path: path.read_bytes() for path in output.rglob("*") if path.is_file()}, before)
            self.assertEqual(opened.GetRootLayer().ExportToString(), cached_before)
            self.assertFalse(opened.GetPrimAtPath("/Robot/NewContent"))
            # If the filesystem also rejects rollback, retain backups and
            # expose their location instead of removing the last good files.
            calls = 0
            fail_calls = {2, 3}
            with patch("isaacsim.asset.transformer.manager.os.replace", side_effect=fail_second_replace):
                with self.assertRaisesRegex(RuntimeError, "Recovery files are preserved"):
                    manager.run(str(root / "input.usda"), profile, str(output))
            recovery = list(output.glob(".asset-publication-*"))
            self.assertEqual(len(recovery), 1)
            backups = [path.read_bytes() for path in (recovery[0] / "backup").rglob("*") if path.is_file()]
            self.assertTrue(backups)
            self.assertTrue(all(content in before.values() for content in backups))
            opened = None
            source = None

    def test_in_place_nonflattened_cycle_restores_package(self) -> None:
        """Roll back when retained source arcs become cyclic after replacement."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "out"
            (output / "payloads").mkdir(parents=True)
            base = Usd.Stage.CreateNew(str(output / "payloads" / "base.usda"))
            base.SetDefaultPrim(base.DefinePrim("/Robot", "Xform"))
            base.DefinePrim("/Robot/Child")
            base.GetRootLayer().Save()
            interface = Usd.Stage.CreateNew(str(output / "robot.usda"))
            interface.SetDefaultPrim(interface.DefinePrim("/Robot", "Xform"))
            interface.GetDefaultPrim().GetReferences().AddReference("./payloads/base.usda")
            interface.GetRootLayer().Save()
            before = {path: path.read_bytes() for path in output.rglob("*.usda")}
            profile = RuleProfile(profile_name="replacement", base_name="base.usda", flatten_source=False)
            with self.assertRaisesRegex(RuntimeError, "flatten its composition first"):
                AssetTransformerManager().run(interface, profile, str(output))
            self.assertEqual({path: path.read_bytes() for path in output.rglob("*.usda")}, before)
            self.assertTrue(interface.GetPrimAtPath("/Robot/Child"))
            interface = None
            base = None

    def test_staged_crate_handles_released_before_cleanup(self) -> None:
        """Successful and failed replacement must release cyclic USD owners before deletion."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = Usd.Stage.CreateNew(str(root / "input.usda"))
            source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
            source.GetRootLayer().Save()
            output = root / "out"
            output.mkdir()
            (output / "notes.txt").write_text("keep", encoding="utf-8")
            registry = RuleRegistry()
            registry.register(_CyclicStageRule)
            spec = RuleSpec(name="Cycle", type=f"{_CyclicStageRule.__module__}.{_CyclicStageRule.__qualname__}")
            profile = RuleProfile(profile_name="replacement", base_name="base.usdc", rules=[spec])
            cleanup = tempfile.TemporaryDirectory.cleanup
            checked = []

            def check_cleanup(directory: tempfile.TemporaryDirectory) -> None:
                """Check that staged crate layers have no live USD owners before removal.

                Args:
                    directory: Temporary directory about to be removed.
                """
                if Path(directory.name).name.startswith(".asset-transformer-"):
                    retained = [str(path) for path in Path(directory.name).rglob("*.usdc") if Sdf.Layer.Find(str(path))]
                    checked.append(directory.name)
                    self.assertEqual(retained, [], "Live crate handles would prevent cleanup on Windows")
                cleanup(directory)

            replace = os.replace

            def check_replace(src: str, dst: str) -> None:
                """Require unreachable destination crate owners to be released before replacement.

                Args:
                    src: Prepared replacement file.
                    dst: Existing destination file.
                """
                if Path(dst).name == "base.usdc":
                    self.assertIsNone(Sdf.Layer.Find(dst), "Unreachable destination owners still hold the crate")
                replace(src, dst)

            was_enabled = gc.isenabled()
            gc.disable()  # Require deterministic ownership release, not a coincidental GC pass.
            try:
                # A first clean export also leaves cyclic owners; the next
                # replacement must release them before publishing over its crate.
                (output / "notes.txt").unlink()
                AssetTransformerManager(registry).run(str(root / "input.usda"), profile, str(output))
                with (
                    patch.object(tempfile.TemporaryDirectory, "cleanup", check_cleanup),
                    patch("isaacsim.asset.transformer.manager.os.replace", side_effect=check_replace),
                ):
                    for fail in (False, True):
                        spec.params = {"fail": fail}
                        report = AssetTransformerManager(registry).run(str(root / "input.usda"), profile, str(output))
                        self.assertEqual(bool(report.composition_errors), fail)
                        self.assertEqual(report.output_stage_path is None, fail)
                self.assertEqual(len(checked), 2)
                self.assertEqual(list(output.glob(".asset-transformer-*")), [])
            finally:
                gc.collect()
                if was_enabled:
                    gc.enable()
            source = None

    def test_empty_or_missing_destination_does_not_reuse_cached_layer(self) -> None:
        """Discard cached destination opinions even after its files are deleted."""
        for remove_directory in (False, True):
            with self.subTest(remove_directory=remove_directory), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source = Usd.Stage.CreateNew(str(root / "input.usda"))
                source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
                source.DefinePrim("/Robot/Current")
                source.GetRootLayer().Save()
                output = root / "out"
                profile = RuleProfile(profile_name="replacement", base_name="base.usda")
                manager = AssetTransformerManager()
                first = manager.run(str(root / "input.usda"), profile, str(output))
                old = Usd.Stage.Open(first.output_stage_path)
                old.DefinePrim("/Robot/Stale")
                old.GetRootLayer().Save()
                shutil.rmtree(output)
                if not remove_directory:
                    output.mkdir()
                self.assertIsNotNone(Sdf.Layer.Find(first.output_stage_path))
                report = manager.run(str(root / "input.usda"), profile, str(output))
                self.assertEqual(report.composition_errors, [])
                self.assertEqual(report.output_stage_path, first.output_stage_path)
                self.assertTrue(old.GetPrimAtPath("/Robot/Current"))
                self.assertFalse(old.GetPrimAtPath("/Robot/Stale"))
                disk = Sdf.Layer.CreateAnonymous("disk.usda")
                self.assertTrue(disk.Import(report.output_stage_path))
                self.assertIsNotNone(disk.GetPrimAtPath("/Robot/Current"))
                self.assertIsNone(disk.GetPrimAtPath("/Robot/Stale"))
                self.assertEqual(list(output.glob(".asset-*")), [])
                disk = None
                old = None
                source = None

    def test_locked_destination_reports_recovery_and_rolls_back(self) -> None:
        """Report locked destination files and restore any earlier publication."""
        for fail_at in (1, 2):
            with self.subTest(fail_at=fail_at), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source = Usd.Stage.CreateNew(str(root / "input.usda"))
                source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
                source.GetRootLayer().Save()
                registry = RuleRegistry()
                registry.register(_AdditionalLayerRule)
                profile = RuleProfile(
                    profile_name="replacement",
                    base_name="base.usda",
                    rules=[
                        RuleSpec(
                            name="Extra", type=f"{_AdditionalLayerRule.__module__}.{_AdditionalLayerRule.__qualname__}"
                        )
                    ],
                )
                output = root / "out"
                manager = AssetTransformerManager(registry)
                report = manager.run(str(root / "input.usda"), profile, str(output))
                old = Usd.Stage.Open(report.output_stage_path)
                before = {path: path.read_bytes() for path in output.rglob("*") if path.is_file()}
                cached_before = old.GetRootLayer().ExportToString()
                source.DefinePrim("/Robot/NewContent")
                source.GetRootLayer().Save()
                replace = os.replace
                calls = 0
                locked_path = None

                def fail_locked_replace(src: str, dst: str) -> None:
                    """Simulate Windows denying replacement of an open destination.

                    Args:
                        src: Prepared replacement file.
                        dst: Destination file.
                    """
                    nonlocal calls, locked_path
                    calls += 1
                    if calls == fail_at:
                        locked_path = dst
                        raise PermissionError(errno.EACCES, "File is open in another process", dst)
                    replace(src, dst)

                with patch("isaacsim.asset.transformer.manager.os.replace", side_effect=fail_locked_replace):
                    with self.assertRaisesRegex(PermissionError, "Close.*different package_root") as raised:
                        manager.run(str(root / "input.usda"), profile, str(output))
                self.assertEqual(raised.exception.filename, locked_path)
                self.assertEqual(raised.exception.errno, errno.EACCES)
                self.assertEqual({path: path.read_bytes() for path in output.rglob("*") if path.is_file()}, before)
                self.assertEqual(old.GetRootLayer().ExportToString(), cached_before)
                self.assertFalse(old.GetPrimAtPath("/Robot/NewContent"))
                self.assertEqual(list(output.glob(".asset-*")), [])
                old = None
                source = None
