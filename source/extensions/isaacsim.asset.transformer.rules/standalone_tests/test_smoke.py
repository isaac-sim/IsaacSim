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

"""Standalone smoke tests for isaacsim-asset-transformer-rules."""

from __future__ import annotations

import ast
import importlib
import os
import pkgutil
import sys
import unittest
from pathlib import Path

_EXCLUDED_TOP_LEVEL_PACKAGES = frozenset({"tests"})


def _iter_rule_module_names() -> list[str]:
    """Return shipped rule package modules, excluding Kit-only test modules."""
    import isaacsim.asset.transformer.rules as rules_pkg

    package_root = rules_pkg.__name__
    module_names = []
    for module_info in pkgutil.walk_packages(rules_pkg.__path__, prefix=f"{package_root}."):
        rel_name = module_info.name[len(package_root) + 1 :]
        if rel_name.split(".", 1)[0] in _EXCLUDED_TOP_LEVEL_PACKAGES:
            continue
        module_names.append(module_info.name)
    return sorted(module_names)


def _iter_rule_python_files() -> list[Path]:
    """Return Python files from the installed rules package, excluding tests."""
    import isaacsim.asset.transformer.rules as rules_pkg

    files = []
    for package_dir in rules_pkg.__path__:
        package_root = Path(package_dir)
        for path in package_root.rglob("*.py"):
            rel_parts = path.relative_to(package_root).parts
            if rel_parts and rel_parts[0] in _EXCLUDED_TOP_LEVEL_PACKAGES:
                continue
            files.append(path)
    return sorted(files)


class TestSmoke(unittest.TestCase):
    """Import and namespace validation."""

    def test_import_public_api(self) -> None:
        """Verify core public API is importable."""
        from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH, register_all_rules

        self.assertTrue(callable(register_all_rules))
        self.assertIsInstance(DEFAULT_PROFILE_PATH, str)

    def test_default_profile_exists(self) -> None:
        """Verify the shipped profile JSON exists on disk."""
        from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH

        self.assertTrue(os.path.isfile(DEFAULT_PROFILE_PATH), f"Missing: {DEFAULT_PROFILE_PATH}")

    def test_no_omni_modules(self) -> None:
        """No omni.* modules should be loaded after import."""
        from isaacsim.asset.transformer.rules import register_all_rules  # noqa: F401

        omni_mods = [m for m in sys.modules if m.startswith("omni.")]
        self.assertEqual(omni_mods, [], f"Unexpected omni modules: {omni_mods}")

    def test_rule_modules_import_in_standalone_env(self) -> None:
        """Every shipped rule module should import without Kit-only dependencies."""
        failures = []
        for module_name in _iter_rule_module_names():
            try:
                importlib.import_module(module_name)
            except Exception as exc:
                failures.append(f"{module_name}: {type(exc).__name__}: {exc}")

        self.assertEqual(failures, [], "Rule modules failed standalone import:\n" + "\n".join(failures))

    def test_pxr_imports_are_available_in_standalone_env(self) -> None:
        """Static guard against importing unavailable pxr schema modules."""
        import pxr

        failures = []
        for path in _iter_rule_python_files():
            tree = ast.parse(path.read_text(), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module == "pxr":
                    for alias in node.names:
                        if alias.name == "*":
                            failures.append(f"{path}: wildcard pxr import")
                        elif not hasattr(pxr, alias.name):
                            failures.append(f"{path}: unavailable pxr symbol '{alias.name}'")
                elif isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name.startswith("pxr."):
                            try:
                                importlib.import_module(alias.name)
                            except Exception as exc:
                                failures.append(f"{path}: unavailable module '{alias.name}': {exc}")

        self.assertEqual(failures, [], "Unavailable pxr imports found:\n" + "\n".join(failures))


class TestFunctional(unittest.TestCase):
    """Rule registration and profile loading tests."""

    def test_register_all_rules(self) -> None:
        """Verify register_all_rules populates the registry."""
        from isaacsim.asset.transformer import RuleRegistry
        from isaacsim.asset.transformer.rules import register_all_rules

        register_all_rules()
        registry = RuleRegistry()
        rules = registry.list_rules()
        self.assertGreaterEqual(len(rules), 12, f"Expected >= 12 rules, got {len(rules)}")
        self.assertIn(
            "isaacsim.asset.transformer.rules.isaac_sim.joint_state_api.JointStateAPIRule",
            rules,
            "JointStateAPIRule should register in standalone wheels",
        )

    def test_profile_json_parses(self) -> None:
        """Verify the shipped profile JSON is valid and loadable."""
        import json

        from isaacsim.asset.transformer.models import RuleProfile
        from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH

        with open(DEFAULT_PROFILE_PATH) as f:
            data = json.load(f)
        profile = RuleProfile.from_dict(data)
        self.assertGreater(len(profile.rules), 0)
        self.assertIsNotNone(profile.profile_name)


if __name__ == "__main__":
    unittest.main()
