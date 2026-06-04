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

"""Verify dynamic rule discovery and registration.

These tests guard the contract that every concrete :class:`RuleInterface`
subclass shipped under :mod:`isaacsim.asset.transformer.rules` is found by
:func:`discover_rule_classes` and registered with the global
:class:`RuleRegistry` by :func:`register_all_rules`.
"""

import omni.kit.test
from isaacsim.asset.transformer import RuleInterface, RuleRegistry
from isaacsim.asset.transformer.rules import discover_rule_classes, register_all_rules


class TestRuleRegistration(omni.kit.test.AsyncTestCase):
    """Async tests for the dynamic rule registration mechanism."""

    async def test_discovery_finds_rule_classes(self) -> None:
        """Discovery must return at least one concrete RuleInterface subclass."""
        rules = discover_rule_classes()
        self.assertGreater(len(rules), 0, "discover_rule_classes() returned no rules.")
        for cls in rules:
            self.assertTrue(
                issubclass(cls, RuleInterface),
                f"{cls!r} is not a subclass of RuleInterface",
            )

    async def test_discovery_returns_unique_classes(self) -> None:
        """Discovery should not yield the same class twice."""
        rules = discover_rule_classes()
        seen: set[type[RuleInterface]] = set()
        duplicates: list[str] = []
        for cls in rules:
            if cls in seen:
                duplicates.append(f"{cls.__module__}.{cls.__qualname__}")
            seen.add(cls)
        self.assertEqual(duplicates, [], "Duplicate rule classes discovered: " + ", ".join(duplicates))

    async def test_all_discovered_rules_are_registered(self) -> None:
        """After register_all_rules(), every discovered rule must be in the registry."""
        register_all_rules()
        registry = RuleRegistry()
        registered = registry.list_rules()

        missing: list[str] = []
        for cls in discover_rule_classes():
            fqcn = f"{cls.__module__}.{cls.__qualname__}"
            if fqcn not in registered:
                missing.append(fqcn)
            elif registered[fqcn] is not cls:
                missing.append(f"{fqcn} (registered as a different class object)")

        self.assertEqual(
            missing,
            [],
            "The following discovered rule classes are not in the RuleRegistry "
            "after register_all_rules():\n  - " + "\n  - ".join(missing),
        )
