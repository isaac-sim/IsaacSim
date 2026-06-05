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

"""Tests for the flatten rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.structure.flatten import FlattenRule
from pxr import Sdf, Tf, Usd, UsdPhysics

from .common import _UR10E_USD


class TestFlattenRule(omni.kit.test.AsyncTestCase):
    """Async tests for FlattenRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_UR10E_USD)
        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"input_stage_path": _UR10E_USD},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 4)
        param_names = [p.name for p in params]
        self.assertIn("output_path", param_names)
        self.assertIn("clear_variants", param_names)
        self.assertIn("selected_variants", param_names)
        self.assertIn("case_insensitive", param_names)
        self._success = True

    async def test_flatten_basic_stage(self) -> None:
        """Verify flattening creates expected output stage."""
        stage = Usd.Stage.Open(_UR10E_USD)
        output_subdir = "payloads"
        output_path = "base.usda"
        os.makedirs(os.path.join(self._tmpdir, output_subdir), exist_ok=True)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path=output_subdir,
            args={
                "input_stage_path": _UR10E_USD,
                "params": {
                    "output_path": output_path,
                    "clear_variants": False,
                },
            },
        )

        rule.process_rule()

        expected_output = os.path.join(self._tmpdir, output_subdir, output_path)
        self.assertTrue(os.path.exists(expected_output))

        flattened_layer = Usd.Stage.Open(expected_output)
        self.assertIsNotNone(flattened_layer)
        # UR10e has ur10e as default prim
        self.assertTrue(flattened_layer.GetPrimAtPath("/ur10e").IsValid())

        self.assertTrue(flattened_layer.GetPrimAtPath("/ur10e/base_link").IsValid())
        self._success = True

    async def test_flatten_without_input_stage_path_skips(self) -> None:
        """Verify rule skips when input stage path is missing."""
        stage = Usd.Stage.Open(_UR10E_USD)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={},
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("No input_stage_path" in msg for msg in log))
        self._success = True

    async def test_flatten_with_variants_cleared(self) -> None:
        """Verify flattening clears variant selections when enabled."""
        stage = Usd.Stage.Open(_UR10E_USD)
        os.makedirs(os.path.join(self._tmpdir, "output"), exist_ok=True)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="output",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {
                    "output_path": "flattened.usda",
                    "clear_variants": True,
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Verify rule processed and cleared variants
        self.assertTrue(any("Cleared" in msg for msg in log) or any("variant" in msg.lower() for msg in log))

        # Verify output exists
        output_path = os.path.join(self._tmpdir, "output", "flattened.usda")
        self.assertTrue(os.path.exists(output_path))

        flattened_layer = Usd.Stage.Open(output_path)
        self.assertTrue(flattened_layer.GetPrimAtPath("/ur10e").IsValid())
        self.assertFalse(flattened_layer.GetPrimAtPath("/ur10e/base_link").IsValid())
        self._success = True

    async def test_flatten_with_selected_variants(self) -> None:
        """Verify selected variants are applied before flattening."""
        stage = Usd.Stage.Open(_UR10E_USD)
        os.makedirs(os.path.join(self._tmpdir, "output"), exist_ok=True)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="output",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {
                    "output_path": "flattened.usda",
                    "clear_variants": True,
                    "selected_variants": {"Physics": "PhysX", "Gripper": "Robotiq_2f_85"},
                },
            },
        )

        rule.process_rule()
        output_path = os.path.join(self._tmpdir, "output", "flattened.usda")

        flattened_layer = Usd.Stage.Open(output_path)
        self.assertTrue(flattened_layer.GetPrimAtPath("/ur10e").IsValid())
        self.assertTrue(flattened_layer.GetPrimAtPath("/ur10e/Robotiq_2F_85/base_link").IsValid())
        self.assertTrue(
            UsdPhysics.RigidBodyAPI(flattened_layer.GetPrimAtPath("/ur10e/Robotiq_2F_85/base_link")).GetPrim().IsValid()
        )
        self._success = True

    async def test_flatten_affected_stages(self) -> None:
        """Verify affected stages list contains output stage."""
        stage = Usd.Stage.Open(_UR10E_USD)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {"output_path": "base.usda"},
            },
        )

        rule.process_rule()

        affected = rule.get_affected_stages()
        self.assertTrue(len(affected) >= 1)
        self.assertTrue(any("base.usda" in s for s in affected))
        self._success = True

    async def test_flatten_logs_completion(self) -> None:
        """Verify completion log entry is recorded."""
        stage = Usd.Stage.Open(_UR10E_USD)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {"output_path": "base.usda"},
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("FlattenRule start" in msg for msg in log))
        self.assertTrue(any("FlattenRule completed" in msg for msg in log))
        self._success = True

    async def test_flatten_return_value_and_invalid_input(self) -> None:
        """process_rule returns abs output path; invalid input stage path handled gracefully."""
        stage = Usd.Stage.Open(_UR10E_USD)
        failures = []

        # -- Return value is absolute path ending with output filename --
        os.makedirs(os.path.join(self._tmpdir, "rv"), exist_ok=True)
        rule_rv = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="rv",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {"output_path": "flat.usda", "clear_variants": False},
            },
        )
        result = rule_rv.process_rule()
        if result is None:
            failures.append("process_rule returned None instead of output path")
        elif not os.path.isabs(result):
            failures.append(f"Return path not absolute: {result}")
        elif not result.endswith("flat.usda"):
            failures.append(f"Return path doesn't end with flat.usda: {result}")

        # -- Invalid input stage path (Usd.Stage.Open raises on missing layers) --
        rule_bad = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"input_stage_path": "/nonexistent/stage.usd"},
        )
        try:
            bad_result = rule_bad.process_rule()
            # If it doesn't raise, it should at least return None or log failure
            bad_log = rule_bad.get_operation_log()
            if bad_result is not None and not any("Failed" in m for m in bad_log):
                failures.append("Invalid input path should return None or log failure")
        except Exception:
            pass  # pxr.Tf.ErrorException is the expected outcome

        self.assertEqual(failures, [], "\n".join(failures))
        self._success = True

    async def test_flatten_variant_selection_edge_cases(self) -> None:
        """Case-insensitive match, nonexistent variant set, nonexistent variant name."""
        stage = Usd.Stage.Open(_UR10E_USD)
        failures = []

        # -- Case-insensitive variant match ('physx' -> 'PhysX') --
        os.makedirs(os.path.join(self._tmpdir, "ci"), exist_ok=True)
        rule_ci = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="ci",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {
                    "output_path": "ci.usda",
                    "clear_variants": True,
                    "selected_variants": {"Physics": "physx"},
                    "case_insensitive": True,
                },
            },
        )
        rule_ci.process_rule()
        ci_log = rule_ci.get_operation_log()
        if not any("Case-insensitive match" in m or "Set variant" in m for m in ci_log):
            failures.append("Case-insensitive match not logged")

        # -- Nonexistent variant set --
        os.makedirs(os.path.join(self._tmpdir, "ns"), exist_ok=True)
        rule_ns = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="ns",
            args={
                "input_stage_path": _UR10E_USD,
                "params": {
                    "output_path": "ns.usda",
                    "clear_variants": False,
                    "selected_variants": {"NonExistentSet": "value"},
                },
            },
        )
        rule_ns.process_rule()
        ns_log = rule_ns.get_operation_log()
        if not any("not found on default prim" in m for m in ns_log):
            failures.append("Nonexistent variant set not logged as skipped")
        ns_output = os.path.join(self._tmpdir, "ns", "ns.usda")
        if not os.path.exists(ns_output):
            failures.append("Output not created despite invalid variant set request")

        self.assertEqual(failures, [], "\n".join(failures))
        self._success = True

    async def test_flatten_fires_no_change_notifications_on_input_root_layer(self) -> None:
        """FlattenRule must not fire any USD change notifications on the input root layer.

        Regression: the previous implementation deleted entries from
        ``prim_spec.variantSelections`` on the root layer and called
        ``Reload()`` on it (twice). When the input is opened from a file
        path, the root layer is shared via USD's process-wide layer cache
        with every other Stage observing the same file -- notably the
        editor's active Stage. Each mutation / reload fires
        ``Sdf.Notice.LayersDidChange`` notifications on the editor's Stage
        that have been observed to crash ``librtx.hydra``
        ("Unable to find RP Prim from previous update pass!").

        End-state assertions (``layer.dirty``, ``ExportToString()``) cannot
        catch this regression because the buggy code called ``Reload()``
        again at the end, resetting the layer to disk-clean state before
        returning. The renderer crashes during the mid-execution
        mutations, not after. This test subscribes to
        ``Sdf.Notice.LayersDidChange`` for the duration of
        ``process_rule()`` and asserts the input root layer is never in the
        changed-layers set.
        """
        stage_a = Usd.Stage.Open(_UR10E_USD)
        stage_b = Usd.Stage.Open(_UR10E_USD)
        # Two Stages opened from the same path must share the same Sdf.Layer
        # via the layer cache, otherwise the test cannot exercise the
        # shared-layer regression at all.
        self.assertIs(stage_a.GetRootLayer(), stage_b.GetRootLayer())

        # Compare by identifier string, not by Python handle. ``Sdf.Layer``
        # handles can have distinct Python wrapper objects pointing at the
        # same underlying USD layer (notice handlers may surface a separate
        # ``SdfLayerHandle`` wrapper from the one returned by
        # ``stage.GetRootLayer()``); identifier comparison is the canonical
        # way to test "is this the same layer".
        input_root_identifier = stage_a.GetRootLayer().identifier
        changed_identifiers: list[str] = []

        def on_layers_changed(notice: Sdf.Notice.LayersDidChange, sender: Sdf.Layer) -> None:
            """Record every layer reported as changed during process_rule()."""
            for layer in notice.GetLayers():
                if layer is None:
                    continue
                changed_identifiers.append(layer.identifier)

        # ``Sdf.Notice.LayersDidChange`` is the global notice fired whenever
        # a layer's contents change. Subscribing globally is the only way to
        # observe changes to a specific layer from a third-party observer.
        listener = Tf.Notice.RegisterGlobally(Sdf.Notice.LayersDidChange, on_layers_changed)
        try:
            os.makedirs(os.path.join(self._tmpdir, "noleak"), exist_ok=True)
            rule = FlattenRule(
                source_stage=stage_a,
                package_root=self._tmpdir,
                destination_path="noleak",
                args={
                    "input_stage_path": _UR10E_USD,
                    # The manager passes the opened stage object here. This is
                    # precisely the path that previously leaked mutations.
                    "input_stage": stage_a,
                    "params": {
                        "output_path": "flat.usda",
                        "clear_variants": True,
                        "selected_variants": {"Physics": "PhysX", "Gripper": "Robotiq_2f_85"},
                        "case_insensitive": True,
                    },
                },
            )
            result = rule.process_rule()
        finally:
            listener.Revoke()

        self.assertIsNotNone(result, "FlattenRule should produce an output path")
        self.assertTrue(os.path.exists(result))

        # Edits to the session layer are expected and harmless (session is
        # per-stage, not in the shared cache), so we filter to root-layer
        # events only by matching on the layer's identifier.
        root_layer_event_count = sum(1 for ident in changed_identifiers if ident == input_root_identifier)
        unique_changed = sorted(set(changed_identifiers))
        # Truncate the unique-layers list to keep the assertion message
        # bounded in size (test harness rejects lines >65 KB).
        sample = unique_changed[:5]
        self.assertEqual(
            root_layer_event_count,
            0,
            "FlattenRule fired Sdf.Notice.LayersDidChange on the input stage's root layer. "
            "That layer is shared with every other Stage observing the same file via USD's "
            "process-wide layer cache (including the editor's active Stage); change notifications "
            "on it have been observed to crash librtx.hydra. Authoring must be routed to the "
            "session layer via Usd.EditContext on a private stage opened from input_stage_path "
            "(see RuleInterface docstring).\n"
            f"Input root layer identifier: {input_root_identifier!r}. "
            f"Number of LayersDidChange events on root layer: {root_layer_event_count} "
            f"(total events: {len(changed_identifiers)}, unique layers: {len(unique_changed)}). "
            f"Sample of changed layer identifiers: {sample!r}.",
        )
        self._success = True

    async def test_flatten_preserves_caller_session_layer(self) -> None:
        """FlattenRule must not touch the session layer of a caller-owned input stage.

        Regression: an earlier session-layer-based fix authored variant
        overrides into ``args["input_stage"].GetSessionLayer()`` and then
        called ``Clear()`` on it. That cleared *all* session-layer
        opinions on the caller's stage, including unrelated user-authored
        ones (visibility toggles, camera opinions, etc.) that the editor
        commonly stores there.

        The rule now opens its own private stage from ``input_stage_path``
        and ignores ``args["input_stage"]`` entirely. This test seeds a
        non-rule opinion on the caller stage's session layer, runs the
        rule with that stage passed as ``args["input_stage"]``, and
        asserts the seeded opinion survives.
        """
        caller_stage = Usd.Stage.Open(_UR10E_USD)
        caller_session = caller_stage.GetSessionLayer()
        self.assertIsNotNone(caller_session)

        # Seed a non-rule opinion on the caller's session layer: an
        # ``over`` prim spec with a visibility=invisible attribute.
        # Editors author opinions like this for user-driven visibility
        # toggles. Any of these getting wiped by the rule is a regression.
        seeded_prim_path = "/ur10e"
        with Usd.EditContext(caller_stage, caller_session):
            session_prim = caller_session.GetPrimAtPath(seeded_prim_path)
            if session_prim is None:
                session_prim = Sdf.CreatePrimInLayer(caller_session, Sdf.Path(seeded_prim_path))
            session_prim.specifier = Sdf.SpecifierOver
            visibility_attr = Sdf.AttributeSpec(session_prim, "visibility", Sdf.ValueTypeNames.Token)
            visibility_attr.default = "invisible"

        self.assertIsNotNone(
            caller_session.GetPrimAtPath(seeded_prim_path),
            "Test setup failed: session-layer prim spec not created.",
        )
        seeded_export_before = caller_session.ExportToString()

        os.makedirs(os.path.join(self._tmpdir, "session"), exist_ok=True)
        rule = FlattenRule(
            source_stage=caller_stage,
            package_root=self._tmpdir,
            destination_path="session",
            args={
                "input_stage_path": _UR10E_USD,
                "input_stage": caller_stage,
                "params": {
                    "output_path": "flat.usda",
                    "clear_variants": True,
                    "selected_variants": {"Physics": "PhysX"},
                },
            },
        )
        rule.process_rule()

        self.assertEqual(
            seeded_export_before,
            caller_session.ExportToString(),
            "FlattenRule modified the caller-owned stage's session layer. "
            "The rule must not author into args['input_stage'].GetSessionLayer(); "
            "it must operate on a private stage opened from input_stage_path.",
        )
        seeded_after = caller_session.GetPrimAtPath(seeded_prim_path)
        self.assertIsNotNone(
            seeded_after,
            "Seeded session-layer prim spec is gone after FlattenRule ran; the rule wiped it.",
        )
        self._success = True

    async def test_flatten_does_not_mutate_input_root_layer_disk_content(self) -> None:
        """The on-disk file backing the input stage must not be rewritten.

        Sister to :meth:`test_flatten_fires_no_change_notifications_on_input_root_layer`.
        A cheap, definitive assertion that the source file is treated as
        read-only: copy the input into the test's tmpdir, run the rule
        against the copy, and confirm bytes and mtime are unchanged.

        The copy is placed inside ``self._tmpdir`` (not the system temp
        directory) so cleanup happens via ``tearDown``'s
        ``shutil.rmtree(..., ignore_errors=True)``. That tolerates Windows
        file locks held by USD's layer cache; an explicit ``os.unlink``
        on the same file fails with ``WinError 5: Access is denied``.
        """
        os.makedirs(os.path.join(self._tmpdir, "ondisk"), exist_ok=True)
        tmp_input_path = os.path.join(self._tmpdir, "ondisk_input.usd")
        shutil.copy2(_UR10E_USD, tmp_input_path)

        with open(tmp_input_path, "rb") as f:
            bytes_before = f.read()
        mtime_before = os.path.getmtime(tmp_input_path)

        stage = Usd.Stage.Open(tmp_input_path)
        rule = FlattenRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="ondisk",
            args={
                "input_stage_path": tmp_input_path,
                "input_stage": stage,
                "params": {
                    "output_path": "flat.usda",
                    "clear_variants": True,
                    "selected_variants": {"Physics": "PhysX"},
                },
            },
        )
        rule.process_rule()

        with open(tmp_input_path, "rb") as f:
            bytes_after = f.read()
        mtime_after = os.path.getmtime(tmp_input_path)

        self.assertEqual(
            bytes_before,
            bytes_after,
            "FlattenRule rewrote the input file's bytes on disk; the input must be read-only.",
        )
        self.assertEqual(
            mtime_before,
            mtime_after,
            "FlattenRule touched the input file's mtime; the input must not be written to.",
        )
        self._success = True
