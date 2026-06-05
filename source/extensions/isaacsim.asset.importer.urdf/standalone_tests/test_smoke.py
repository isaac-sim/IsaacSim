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

"""Standalone smoke tests for isaacsim-asset-importer-urdf."""

from __future__ import annotations

import sys
import unittest


class TestSmoke(unittest.TestCase):
    """Import and namespace validation."""

    def test_import_public_api(self) -> None:
        """Verify URDF importer classes are importable."""
        from isaacsim.asset.importer.urdf import URDFImporter

        self.assertTrue(callable(URDFImporter))

    def test_import_config(self) -> None:
        """Verify importer config is importable."""
        from isaacsim.asset.importer.urdf.impl.config import URDFImporterConfig

        self.assertTrue(callable(URDFImporterConfig))

    def test_no_omni_modules(self) -> None:
        """No omni.* modules should be loaded after import."""
        from isaacsim.asset.importer.urdf import URDFImporter  # noqa: F401

        omni_mods = [m for m in sys.modules if m.startswith("omni.")]
        self.assertEqual(omni_mods, [], f"Unexpected omni modules: {omni_mods}")

    def test_converter_available(self) -> None:
        """Verify urdf-usd-converter is installed and importable."""
        import urdf_usd_converter

        self.assertTrue(hasattr(urdf_usd_converter, "__version__"))


if __name__ == "__main__":
    unittest.main()
