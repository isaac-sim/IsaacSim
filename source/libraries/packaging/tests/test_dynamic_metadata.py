# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Test shared-version dynamic package metadata."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

PROVIDER_PATH = Path(__file__).parents[1] / "isaacsim_libraries_metadata.py"
PROVIDER_SPEC = importlib.util.spec_from_file_location("isaacsim_libraries_metadata_tests", PROVIDER_PATH)
if PROVIDER_SPEC is None or PROVIDER_SPEC.loader is None:
    raise RuntimeError(f"Cannot load dynamic metadata provider from {PROVIDER_PATH}")
provider = importlib.util.module_from_spec(PROVIDER_SPEC)
sys.modules[PROVIDER_SPEC.name] = provider
PROVIDER_SPEC.loader.exec_module(provider)


class DynamicMetadataTests(unittest.TestCase):
    """Validate generated internal Python requirements."""

    def test_dynamic_metadata_pins_names_to_the_shared_version(self) -> None:
        """Test pinning internal distribution names to the shared version."""
        version = PROVIDER_PATH.parents[1].joinpath("VERSION").read_text(encoding="utf-8").strip()

        metadata = provider.dynamic_metadata(
            {"field": "dependencies", "names": ["isaacsim-common", "isaacsim_foundation"]},
            {},
        )

        self.assertEqual(
            metadata,
            {"dependencies": [f"isaacsim-common=={version}", f"isaacsim_foundation=={version}"]},
        )

    def test_dynamic_metadata_rejects_duplicate_normalized_names(self) -> None:
        """Test rejecting distribution names that normalize to duplicates."""
        with self.assertRaisesRegex(RuntimeError, "must be unique"):
            provider.dynamic_metadata(
                {"field": "dependencies", "names": ["isaacsim-common", "isaacsim_common"]},
                {},
            )

    def test_dynamic_metadata_requires_the_dependencies_field(self) -> None:
        """Test requiring the dynamic dependencies field."""
        with self.assertRaisesRegex(RuntimeError, "field must be 'dependencies'"):
            provider.dynamic_metadata({"field": "version", "names": ["isaacsim-common"]}, {})


if __name__ == "__main__":
    unittest.main()
