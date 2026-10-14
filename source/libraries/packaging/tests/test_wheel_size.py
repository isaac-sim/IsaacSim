# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Test wheel size artifact metadata."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import ModuleType


def _load_wheel_module() -> ModuleType:
    """Load the wheel packaging tool.

    Returns:
        Loaded wheel module.
    """
    path = Path(__file__).parents[1] / "wheel.py"
    spec = importlib.util.spec_from_file_location("isaacsim_wheel_size_test_tool", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load wheel tool: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_WHEEL = _load_wheel_module()


class WheelSizeManifestTests(unittest.TestCase):
    """Wheel size manifest tests."""

    def test_write_manifest_reports_each_artifact_and_duplicate_content(self) -> None:
        """Report exact wheel files and byte-identical content."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_directory = Path(temporary_directory)
            shared_content = b"".join(index.to_bytes(4, "little") for index in range(100_000))
            for name in ("one-1.0-py3-none-any.whl", "two-1.0-py3-none-any.whl"):
                with zipfile.ZipFile(wheel_directory / name, "w", compression=zipfile.ZIP_DEFLATED) as wheel:
                    wheel.writestr(f"{name}/unique", name)
                    wheel.writestr("shared/content.bin", shared_content)

            manifest = _WHEEL._write_wheel_size_manifest(wheel_directory, "release")

            self.assertEqual(manifest["schema_version"], 1)
            self.assertEqual(manifest["configuration"], "release")
            self.assertEqual(
                [row["filename"] for row in manifest["wheels"]],
                [
                    "one-1.0-py3-none-any.whl",
                    "two-1.0-py3-none-any.whl",
                ],
            )
            self.assertGreater(manifest["totals"]["duplicate_compressed_size_bytes"], 0)
            written = json.loads((wheel_directory / "wheel_sizes.json").read_text(encoding="utf-8"))
            self.assertEqual(written, manifest)

    def test_build_manifest_rejects_empty_wheelhouse(self) -> None:
        """Reject a wheelhouse without artifacts."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            with self.assertRaisesRegex(RuntimeError, "No wheels found"):
                _WHEEL._build_wheel_size_manifest(Path(temporary_directory), "release")


if __name__ == "__main__":
    unittest.main()
