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

"""Standalone smoke tests for isaacsim-asset-transformer."""

from __future__ import annotations

import sys
import unittest


class TestSmoke(unittest.TestCase):
    """Import and namespace validation."""

    def test_import_public_api(self) -> None:
        """Verify core public API classes are importable."""
        from isaacsim.asset.transformer import AssetTransformerManager, RuleRegistry
        from isaacsim.asset.transformer.models import (
            ExecutionReport,
            RuleConfigurationParam,
            RuleExecutionResult,
            RuleProfile,
            RuleSpec,
        )
        from isaacsim.asset.transformer.rule_interface import RuleInterface

        self.assertTrue(callable(AssetTransformerManager))
        self.assertTrue(callable(RuleRegistry))
        self.assertTrue(callable(RuleInterface))
        self.assertTrue(callable(RuleSpec))

    def test_no_omni_modules(self) -> None:
        """No omni.* modules should be loaded after import."""
        from isaacsim.asset.transformer import AssetTransformerManager  # noqa: F401

        omni_mods = [m for m in sys.modules if m.startswith("omni.")]
        self.assertEqual(omni_mods, [], f"Unexpected omni modules: {omni_mods}")


class TestFunctional(unittest.TestCase):
    """Data model and manager logic tests."""

    def test_rule_spec_round_trip(self) -> None:
        """Verify RuleSpec serialization round-trip."""
        from isaacsim.asset.transformer.models import RuleSpec

        spec = RuleSpec(name="Test", type="mod.Rule", params={"k": 1}, enabled=False)
        restored = RuleSpec.from_dict(spec.to_dict())
        self.assertEqual(restored, spec)

    def test_rule_profile_round_trip(self) -> None:
        """Verify RuleProfile serialization round-trip."""
        from isaacsim.asset.transformer.models import RuleProfile, RuleSpec

        spec = RuleSpec(name="A", type="m.A", params={"k": 2})
        profile = RuleProfile(profile_name="test", version="1.0", rules=[spec])
        data = profile.to_dict()
        restored = RuleProfile.from_dict(data)
        self.assertEqual(len(restored.rules), 1)
        self.assertEqual(restored.rules[0], spec)

    def test_rule_registry_instantiable(self) -> None:
        """Verify RuleRegistry can be instantiated and queried."""
        from isaacsim.asset.transformer import RuleRegistry

        registry = RuleRegistry()
        rules = registry.list_rules()
        self.assertIsInstance(rules, dict)


if __name__ == "__main__":
    unittest.main()
