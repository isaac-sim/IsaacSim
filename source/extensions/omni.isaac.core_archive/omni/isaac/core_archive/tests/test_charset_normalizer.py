# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify native-module precedence and the scope of the charset-normalizer workaround."""

import importlib
import sys
import tempfile
from importlib.machinery import EXTENSION_SUFFIXES, ExtensionFileLoader
from pathlib import Path

import omni.kit.test

from .._charset_normalizer import _FINDER, install


class TestCharsetNormalizerImporter(omni.kit.test.AsyncTestCase):
    """Verify package-local native imports while leaving Kit namespace imports intact."""

    async def test_native_modules_take_precedence(self) -> None:
        """Select both native modules even when their Python alternatives were created first."""
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            for name in ("cd", "md"):
                package.joinpath(f"{name}.py").write_text("")
                native = package / f"{name}{EXTENSION_SUFFIXES[0]}"
                native.write_bytes(b"")
            importlib.invalidate_caches()
            for name in ("cd", "md"):
                native = package / f"{name}{EXTENSION_SUFFIXES[0]}"
                spec = _FINDER.find_spec(f"charset_normalizer.{name}", [directory])
                self.assertIsNotNone(spec)
                self.assertEqual(spec.origin, str(native))
                self.assertIsInstance(spec.loader, ExtensionFileLoader)

    async def test_other_packages_and_top_level_are_not_intercepted(self) -> None:
        """Leave top-level selection and unrelated namespace imports to Kit."""
        for name in ("charset_normalizer", "charset_normalizer_extra.md", "omni.usd.schema", "isaacsim.core"):
            self.assertIsNone(_FINDER.find_spec(name, ["/unused"]))
        self.assertIsNone(_FINDER.find_spec("charset_normalizer.md"))

    async def test_install_is_idempotent(self) -> None:
        """Keep one finder ahead of Kit's cached module lookup after repeated installation."""
        install()
        install()
        self.assertIs(sys.meta_path[0], _FINDER)
        self.assertEqual(sys.meta_path.count(_FINDER), 1)
