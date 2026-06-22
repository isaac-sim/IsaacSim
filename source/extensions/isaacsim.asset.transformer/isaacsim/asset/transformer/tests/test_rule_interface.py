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

"""Tests for RuleInterface logging and affected stage tracking."""

import types
from unittest.mock import patch

import omni.kit.test
from isaacsim.asset.transformer.models import RuleConfigurationParam
from isaacsim.asset.transformer.rule_interface import RuleInterface


class _NoOpRule(RuleInterface):
    """No-op rule implementation for interface tests."""

    def process_rule(self) -> None:
        """Record a no-op message and affected stage."""
        self.log_operation("noop")
        self.add_affected_stage("stage://mem")
        return

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return an empty configuration parameter list.

        Returns:
            Empty list of configuration parameters.

        """
        return []


class TestRuleInterface(omni.kit.test.AsyncTestCase):
    """Async tests for the rule interface helpers."""

    async def test_rule_interface_logging_and_affected_list(self) -> None:
        """Verify log collection and affected stage de-duplication."""
        fake_stage_mod = types.SimpleNamespace()
        fake_usd_mod = types.SimpleNamespace(Stage=fake_stage_mod)
        with patch("isaacsim.asset.transformer.rule_interface.Usd", fake_usd_mod, create=True):
            rule = _NoOpRule(source_stage=object(), package_root="/pkg", destination_path="", args={"destination": "x"})
            rule.process_rule()
            self.assertEqual(rule.get_operation_log()[-1], "noop")
            self.assertIn("stage://mem", rule.get_affected_stages())
            rule.add_affected_stage("stage://mem")
            self.assertEqual(rule.get_affected_stages().count("stage://mem"), 1)

    async def test_request_deletion_records_unique_paths(self) -> None:
        """Verify request_deletion de-duplicates, ignores empties, and returns a copy."""
        fake_usd_mod = types.SimpleNamespace(Stage=types.SimpleNamespace())
        with patch("isaacsim.asset.transformer.rule_interface.Usd", fake_usd_mod, create=True):
            rule = _NoOpRule(source_stage=object(), package_root="/pkg", destination_path="", args={})
            self.assertEqual(rule.get_pending_deletions(), [])
            rule.request_deletion("/tmp/a.usd")
            rule.request_deletion("/tmp/b.usd")
            rule.request_deletion("/tmp/a.usd")  # duplicate ignored
            rule.request_deletion("")  # empty ignored
            self.assertEqual(rule.get_pending_deletions(), ["/tmp/a.usd", "/tmp/b.usd"])
            # The accessor returns a copy; mutating it must not affect internal state.
            rule.get_pending_deletions().append("/tmp/c.usd")
            self.assertEqual(rule.get_pending_deletions(), ["/tmp/a.usd", "/tmp/b.usd"])
