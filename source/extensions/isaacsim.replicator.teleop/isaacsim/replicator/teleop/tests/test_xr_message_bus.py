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

"""Regression tests for the XR Core message-bus boundary the teleop runtime exposes.

Two boundaries are covered:

* :func:`_extract_xr_teleop_command` - both shapes the CloudXR runtime is
  observed to emit (JSON-encoded string and inline dict) must yield the same
  command. The dict path was inadvertently dropped during a refactor and the
  command was silently swallowed; this test pins the contract.
* :func:`activate_pre_session_anchor` / :func:`restore_pre_session_anchor` -
  the snapshot/restore refcount must only advance when the underlying XR
  override actually committed, otherwise an XR module that comes online after
  activation can leave a permanently-overwritten anchor mode behind.
"""

from __future__ import annotations

from unittest.mock import patch

import omni.kit.test
from isaacsim.replicator.teleop import xr_anchor_manager as xam
from isaacsim.replicator.teleop.teleop_manager import _extract_xr_teleop_command


class ExtractXrTeleopCommandTests(omni.kit.test.AsyncTestCase):
    async def test_json_string_payload_extracts_command(self) -> None:
        payload = {"message": '{"command":"start teleop"}'}
        self.assertEqual(_extract_xr_teleop_command(payload), "start teleop")

    async def test_dict_payload_extracts_command(self) -> None:
        """The inline-dict payload shape must yield the same command as the JSON-encoded string."""
        payload = {"message": {"command": "start teleop"}}
        self.assertEqual(_extract_xr_teleop_command(payload), "start teleop")

    async def test_none_payload_returns_empty_string(self) -> None:
        self.assertEqual(_extract_xr_teleop_command(None), "")

    async def test_missing_message_returns_empty_string(self) -> None:
        self.assertEqual(_extract_xr_teleop_command({"other": "thing"}), "")

    async def test_invalid_json_string_returns_empty_string(self) -> None:
        self.assertEqual(_extract_xr_teleop_command({"message": "not valid json"}), "")

    async def test_dict_without_command_key_returns_empty_string(self) -> None:
        self.assertEqual(_extract_xr_teleop_command({"message": {"other": "value"}}), "")

    async def test_non_string_non_dict_message_returns_empty_string(self) -> None:
        self.assertEqual(_extract_xr_teleop_command({"message": 42}), "")


class _FakeXrSettings:
    """Minimal stand-in for the Kit ``XRSettings`` singleton used by the snapshot helpers."""

    def __init__(self, initial: dict[str, str | float] | None = None) -> None:
        self.values: dict[str, str | float] = dict(initial or {})
        self.writes: list[tuple[str, str | float]] = []

    def get_setting(self, token: str) -> str | float | None:
        return self.values.get(token)

    def set_setting(self, token: str, value: str | float) -> None:
        self.values[token] = value
        self.writes.append((token, value))


class PreSessionAnchorRefcountTests(omni.kit.test.AsyncTestCase):
    """Pin the refcount lifecycle described in :func:`activate_pre_session_anchor`."""

    async def setUp(self) -> None:
        # Force a clean global state for every test; a leaked refcount from a
        # prior test would mask the very bugs we want to catch here.
        xam._settings_snapshot = None
        xam._settings_snapshot_refs = 0

    async def tearDown(self) -> None:
        # Don't leave a stray refcount or snapshot behind for the rest of the suite.
        xam._settings_snapshot = None
        xam._settings_snapshot_refs = 0

    async def test_activate_returns_false_when_xr_unavailable(self) -> None:
        """No XR core - no activation, no refcount bump, restore is a safe no-op."""
        with patch.object(xam, "_xr_settings", lambda: None):
            self.assertFalse(xam.activate_pre_session_anchor())
        self.assertEqual(xam._settings_snapshot_refs, 0)
        self.assertIsNone(xam._settings_snapshot)
        xam.restore_pre_session_anchor()
        self.assertEqual(xam._settings_snapshot_refs, 0)

    async def test_late_xr_after_failed_activation_does_not_strip_anchor(self) -> None:
        """If activation no-op'd because XR was offline, a late-arriving XR module
        must not get its baseline overwritten by a stray restore."""
        with patch.object(xam, "_xr_settings", lambda: None):
            xam.activate_pre_session_anchor()
            xam.activate_pre_session_anchor()
        late_xs = _FakeXrSettings({xam._XR_TOKEN_ANCHOR_MODE: "active camera"})
        with patch.object(xam, "_xr_settings", lambda: late_xs):
            xam.restore_pre_session_anchor()
            xam.restore_pre_session_anchor()
        self.assertEqual(
            late_xs.values[xam._XR_TOKEN_ANCHOR_MODE],
            "active camera",
            "Late-arriving XR settings must not be overwritten by a restore that has no snapshot.",
        )

    async def test_nested_activation_only_restores_on_final_release(self) -> None:
        """Two activations + one restore must keep the override in place; the
        baseline returns only when the matching second restore lands."""
        xs = _FakeXrSettings({xam._XR_TOKEN_ANCHOR_MODE: "active camera"})
        with patch.object(xam, "_xr_settings", lambda: xs):
            self.assertTrue(xam.activate_pre_session_anchor())
            self.assertTrue(xam.activate_pre_session_anchor())
            self.assertEqual(xs.values[xam._XR_TOKEN_ANCHOR_MODE], "scene origin")

            xam.restore_pre_session_anchor()
            self.assertEqual(
                xs.values[xam._XR_TOKEN_ANCHOR_MODE],
                "scene origin",
                "Inner restore must keep the override active while the outer holder is alive.",
            )
            xam.restore_pre_session_anchor()
            self.assertEqual(
                xs.values[xam._XR_TOKEN_ANCHOR_MODE],
                "active camera",
                "Final restore must return the user's original anchor mode.",
            )

    async def test_restore_is_idempotent_when_refcount_is_zero(self) -> None:
        xs = _FakeXrSettings({xam._XR_TOKEN_ANCHOR_MODE: "active camera"})
        with patch.object(xam, "_xr_settings", lambda: xs):
            xam.restore_pre_session_anchor()
            xam.restore_pre_session_anchor()
        self.assertEqual(xam._settings_snapshot_refs, 0)
        self.assertEqual(xs.values[xam._XR_TOKEN_ANCHOR_MODE], "active camera")
