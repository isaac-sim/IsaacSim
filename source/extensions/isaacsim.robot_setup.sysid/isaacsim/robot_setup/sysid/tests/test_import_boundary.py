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

"""Regression tests for the backend import boundary."""

from __future__ import annotations

import ast
import builtins
import importlib.util
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest import mock

try:
    import omni.kit.test

    _AsyncTestCase = omni.kit.test.AsyncTestCase
except ModuleNotFoundError as exc:
    if exc.name not in {"omni", "omni.kit", "omni.kit.test"}:
        raise
    _AsyncTestCase = unittest.IsolatedAsyncioTestCase


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_RUNTIME_PREFIXES = ("isaacsim.core", "isaacsim.gui", "newton", "omni", "pxr", "warp")
_RUNTIME_ADAPTER = _PACKAGE_ROOT / "isaac_sim_sysid_bridge.py"


def _is_runtime_import(name: str) -> bool:
    return any(name == prefix or name.startswith(f"{prefix}.") for prefix in _RUNTIME_PREFIXES)


def _load_source_module(module_name: str, path: Path) -> ModuleType:
    """Load one source file under an isolated module name.

    Args:
        module_name: Temporary module name used for the isolated import.
        path: Python source file to load.

    Returns:
        The loaded module.
    """
    spec = importlib.util.spec_from_file_location(
        module_name,
        path,
        submodule_search_locations=([str(path.parent)] if path.name == "__init__.py" else None),
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load test module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    return module


class TestImportBoundary(_AsyncTestCase):
    """Verify that portable imports do not accidentally bootstrap Kit."""

    def tearDown(self) -> None:
        """Remove dynamically loaded boundary-test modules."""
        for name in tuple(sys.modules):
            if name.startswith("_sysid_boundary_test"):
                sys.modules.pop(name, None)

    async def test_package_root_does_not_attempt_runtime_imports(self) -> None:
        """Ensure importing the package root never requests runtime modules."""
        original_import = builtins.__import__
        attempted: list[str] = []

        def guarded_import(
            name: str,
            globals: dict[str, Any] | None = None,
            locals: dict[str, Any] | None = None,
            fromlist: tuple[str, ...] = (),
            level: int = 0,
        ) -> Any:
            if level == 0 and _is_runtime_import(name):
                attempted.append(name)
                raise AssertionError(f"Package root attempted runtime import {name!r}")
            return original_import(name, globals, locals, fromlist, level)

        with mock.patch("builtins.__import__", side_effect=guarded_import):
            module = _load_source_module("_sysid_boundary_test", _PACKAGE_ROOT / "__init__.py")

        self.assertEqual(attempted, [])
        self.assertNotIn("Extension", module.__dict__)
        self.assertNotIn("__getattr__", module.__dict__)

    async def test_portable_modules_have_no_module_level_runtime_imports(self) -> None:
        """Guard every portable module instead of checking only the empty package root."""
        violations: list[str] = []
        for path in sorted(_PACKAGE_ROOT.rglob("*.py")):
            if path == _RUNTIME_ADAPTER or "tests" in path.relative_to(_PACKAGE_ROOT).parts:
                continue
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in tree.body:
                names: list[str] = []
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                    names = [node.module]
                for name in names:
                    if _is_runtime_import(name):
                        relative = path.relative_to(_PACKAGE_ROOT)
                        violations.append(f"{relative}:{node.lineno}: {name}")

        self.assertEqual(
            violations,
            [],
            "Module-level runtime imports found:\n" + "\n".join(violations),
        )

    async def test_runtime_probe_reraises_internal_module_import_failure(self) -> None:
        """Do not misreport a broken Kit installation as simple unavailability."""
        from isaacsim.robot_setup.sysid import runtime

        original_import = builtins.__import__

        def fail_internal_dependency(
            name: str,
            globals: dict[str, Any] | None = None,
            locals: dict[str, Any] | None = None,
            fromlist: tuple[str, ...] = (),
            level: int = 0,
        ) -> Any:
            if name == "omni.kit.app":
                raise ModuleNotFoundError(
                    "No module named 'kit_native_dependency'",
                    name="kit_native_dependency",
                )
            return original_import(name, globals, locals, fromlist, level)

        with (
            mock.patch("builtins.__import__", side_effect=fail_internal_dependency),
            self.assertRaises(ModuleNotFoundError) as raised,
        ):
            runtime.kit_runtime_available()

        self.assertEqual(raised.exception.name, "kit_native_dependency")

    async def test_settings_partial_runtime_uses_public_unavailable_error(self) -> None:
        """Normalize a partially installed Carb module to the adapter error contract."""
        from isaacsim.robot_setup.sysid.errors import SysIdRuntimeUnavailableError
        from isaacsim.robot_setup.sysid.settings import CarbOptimizerSettingsStore

        original_import = builtins.__import__

        def partial_carb(
            name: str,
            globals: dict[str, Any] | None = None,
            locals: dict[str, Any] | None = None,
            fromlist: tuple[str, ...] = (),
            level: int = 0,
        ) -> Any:
            if name == "carb.settings":
                return SimpleNamespace()
            return original_import(name, globals, locals, fromlist, level)

        with (
            mock.patch("builtins.__import__", side_effect=partial_carb),
            self.assertRaises(SysIdRuntimeUnavailableError),
        ):
            CarbOptimizerSettingsStore()
