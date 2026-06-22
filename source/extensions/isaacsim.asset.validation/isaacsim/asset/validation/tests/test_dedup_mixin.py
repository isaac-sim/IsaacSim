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

"""Unit tests for DedupMixin and _resolve_at in util.py."""

import omni.kit.test
from isaacsim.asset.validation.util import DedupMixin, _resolve_at
from pxr import Usd, UsdGeom

# ---------------------------------------------------------------------------
# Stand in for BaseRuleChecker
# ---------------------------------------------------------------------------


class _RecordingBase:
    """Records forwarded _AddError/_AddWarning/_AddInfo calls for assertions."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str, object]] = []

    def _AddError(self, message: str, **kwargs: object) -> None:  # noqa: N802
        self.calls.append(("error", message, kwargs.get("at")))

    def _AddWarning(self, message: str, **kwargs: object) -> None:  # noqa: N802
        self.calls.append(("warning", message, kwargs.get("at")))

    def _AddInfo(self, message: str, **kwargs: object) -> None:  # noqa: N802
        self.calls.append(("info", message, kwargs.get("at")))


class _TestRule(DedupMixin, _RecordingBase):
    """Concrete rule that inherits dedup via mixin."""


# ---------------------------------------------------------------------------
# Helpers - fresh stage + prim per test, no cross-test state.
# ---------------------------------------------------------------------------


def _make_stage_and_prim(prim_path: str = "/World/Cube") -> tuple[Usd.Stage, Usd.Prim]:
    """Return an in-memory stage and an Xform prim at ``prim_path``.

    Args:
        prim_path: Path where the Xform prim is defined.

    Returns:
        The created stage and prim.
    """
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
    return stage, prim


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestResolveAt(omni.kit.test.AsyncTestCase):
    """Unit tests for the _resolve_at helper."""

    async def test_none_returns_empty_string(self) -> None:
        """None resolves to an empty string."""
        self.assertEqual(_resolve_at(None), "")

    async def test_stage_uses_root_layer_identifier(self) -> None:
        """Stages resolve to their root layer identifier."""
        stage = Usd.Stage.CreateInMemory()
        self.assertEqual(_resolve_at(stage), stage.GetRootLayer().identifier)

    async def test_prim_uses_get_path(self) -> None:
        """Prims resolve to their path string."""
        _stage, prim = _make_stage_and_prim("/World/Cube")
        self.assertEqual(_resolve_at(prim), "/World/Cube")

    async def test_unknown_type_falls_back_to_str(self) -> None:
        """Unknown objects resolve using ``str``."""
        self.assertEqual(_resolve_at(42), "42")
        self.assertEqual(_resolve_at("some/path"), "some/path")


class TestDedupMixinError(omni.kit.test.AsyncTestCase):
    """_AddError deduplication."""

    async def test_duplicate_error_emits_once(self) -> None:
        """Identical _AddError calls on the same key → forwarded once."""
        _stage, prim = _make_stage_and_prim()
        rule = _TestRule()
        rule._AddError("mesh missing", at=prim)
        rule._AddError("mesh missing", at=prim)
        rule._AddError("mesh missing", at=prim)
        self.assertEqual(len(rule.calls), 1)
        self.assertEqual(rule.calls[0], ("error", "mesh missing", prim))

    async def test_different_messages_both_emitted(self) -> None:
        """Same prim, different messages → both forwarded."""
        _stage, prim = _make_stage_and_prim()
        rule = _TestRule()
        rule._AddError("mesh missing", at=prim)
        rule._AddError("normals bad", at=prim)
        self.assertEqual(len(rule.calls), 2)


class TestDedupMixinWarning(omni.kit.test.AsyncTestCase):
    """_AddWarning deduplication."""

    async def test_duplicate_warning_emits_once(self) -> None:
        """Identical _AddWarning calls on the same key are forwarded once."""
        rule = _TestRule()
        rule._AddWarning("soft warning", at=None)
        rule._AddWarning("soft warning", at=None)
        self.assertEqual(len(rule.calls), 1)
        self.assertEqual(rule.calls[0][0], "warning")


class TestDedupMixinInfo(omni.kit.test.AsyncTestCase):
    """_AddInfo deduplication."""

    async def test_duplicate_info_emits_once(self) -> None:
        """Identical _AddInfo calls on the same key are forwarded once."""
        rule = _TestRule()
        rule._AddInfo("fyi note", at=None)
        rule._AddInfo("fyi note", at=None)
        self.assertEqual(len(rule.calls), 1)
        self.assertEqual(rule.calls[0][0], "info")


class TestDedupMixinInstanceIsolation(omni.kit.test.AsyncTestCase):
    """Per-instance _seen sets don't bleed across rule instances."""

    async def test_two_instances_have_independent_seen_sets(self) -> None:
        """Duplicate state stays local to each rule instance."""
        _stage, prim = _make_stage_and_prim("/World/Mesh")
        rule_a = _TestRule()
        rule_b = _TestRule()

        rule_a._AddError("duplicate issue", at=prim)
        rule_a._AddError("duplicate issue", at=prim)  # suppressed in a

        rule_b._AddError("duplicate issue", at=prim)
        rule_b._AddError("duplicate issue", at=prim)  # suppressed in b

        self.assertEqual(len(rule_a.calls), 1, "rule_a should have 1 forwarded call")
        self.assertEqual(len(rule_b.calls), 1, "rule_b should have 1 forwarded call")


class TestResolveAtInKey(omni.kit.test.AsyncTestCase):
    """``at=`` variants produce the correct key component."""

    async def test_at_none_key_component_is_empty_string(self) -> None:
        """at=None → key built with "" → subsequent call suppressed."""
        rule = _TestRule()
        rule._AddError("msg", at=None)
        rule._AddError("msg", at=None)
        self.assertEqual(len(rule.calls), 1)

    async def test_at_stage_uses_root_layer_identifier(self) -> None:
        """at= with GetRootLayer → identifier used in key; different stages forwarded."""
        stage_a = Usd.Stage.CreateInMemory()
        stage_b = Usd.Stage.CreateInMemory()
        rule = _TestRule()
        rule._AddError("msg", at=stage_a)
        rule._AddError("msg", at=stage_a)  # duplicate → suppressed
        rule._AddError("msg", at=stage_b)  # different identifier → forwarded
        self.assertEqual(len(rule.calls), 2)

    async def test_at_prim_uses_get_path(self) -> None:
        """at= with GetPath → str(path) used in key; different paths forwarded."""
        _stage_a, prim_a = _make_stage_and_prim("/World/A")
        _stage_b, prim_b = _make_stage_and_prim("/World/B")
        rule = _TestRule()
        rule._AddError("msg", at=prim_a)
        rule._AddError("msg", at=prim_a)  # duplicate → suppressed
        rule._AddError("msg", at=prim_b)  # different path → forwarded
        self.assertEqual(len(rule.calls), 2)

    async def test_at_unknown_type_falls_back_to_str(self) -> None:
        """at= unknown type → str(at) used in key."""
        rule = _TestRule()
        rule._AddError("msg", at=99)
        rule._AddError("msg", at=99)  # same str → suppressed
        rule._AddError("msg", at=100)  # different str → forwarded
        self.assertEqual(len(rule.calls), 2)


class TestSeverityDoesNotCrossContaminate(omni.kit.test.AsyncTestCase):
    """Severity string is part of the key, so error/warning/info are distinct."""

    async def test_same_message_different_severity_all_forwarded(self) -> None:
        """The same message is emitted once for each severity."""
        rule = _TestRule()
        rule._AddError("shared msg", at=None)
        rule._AddWarning("shared msg", at=None)
        rule._AddInfo("shared msg", at=None)
        self.assertEqual(len(rule.calls), 3)
