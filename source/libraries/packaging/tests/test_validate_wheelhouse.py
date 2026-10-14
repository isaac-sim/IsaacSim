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

"""Test clean wheelhouse installation and metrics."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import ModuleType
from unittest import mock


def _load_validator_module() -> ModuleType:
    """Load the wheelhouse validation tool.

    Returns:
        Loaded validation module.
    """
    path = Path(__file__).parents[1] / "validate_wheelhouse.py"
    spec = importlib.util.spec_from_file_location("isaacsim_validate_wheelhouse_test_tool", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load wheelhouse validator: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_VALIDATOR = _load_validator_module()


def _write_test_wheel(path: Path, requirements: tuple[str, ...] = ()) -> None:
    """Write an installable wheel with a source-library package manifest.

    Args:
        path: Wheel output path.
        requirements: Distribution requirements to encode in wheel metadata.
    """
    manifest = {
        "name": "acme-demo",
        "version": "1.0",
        "python_imports": ["acme_demo"],
    }
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as wheel:
        wheel.writestr("acme_demo/__init__.py", "VALUE = 1\n")
        wheel.writestr(
            "acme_demo-1.0.data/data/share/isaacsim/packages/acme_demo/package.json",
            json.dumps(manifest),
        )
        wheel.writestr(
            "acme_demo-1.0.dist-info/METADATA",
            "Metadata-Version: 2.2\nName: acme-demo\nVersion: 1.0\n"
            + "".join(f"Requires-Dist: {requirement}\n" for requirement in requirements),
        )
        wheel.writestr(
            "acme_demo-1.0.dist-info/WHEEL",
            "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
        )
        wheel.writestr("acme_demo-1.0.dist-info/RECORD", "")


def _write_dependency_wheel(path: Path) -> None:
    """Write a dependency wheel without a source-library package manifest.

    Args:
        path: Wheel output path.
    """
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as wheel:
        wheel.writestr("acme_dependency/__init__.py", "PAYLOAD = b'x' * 4096\n")
        wheel.writestr(
            "acme_dependency-2.0.dist-info/METADATA",
            "Metadata-Version: 2.2\nName: acme-dependency\nVersion: 2.0\n",
        )
        wheel.writestr(
            "acme_dependency-2.0.dist-info/WHEEL",
            "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
        )
        wheel.writestr("acme_dependency-2.0.dist-info/RECORD", "")


class ValidateWheelhouseTests(unittest.TestCase):
    """Clean wheelhouse validation tests."""

    def test_install_uses_required_dependency_sources(self) -> None:
        """Resolve public dependencies and required OVNewton artifacts."""
        wheel_directory = Path("/tmp/wheels")
        install_directory = Path("/tmp/install")
        cache_directory = Path("/tmp/cache")

        with (
            mock.patch.object(_VALIDATOR.subprocess, "run") as run,
            mock.patch.object(_VALIDATOR.time, "perf_counter", side_effect=(1.0, 2.0)),
        ):
            duration = _VALIDATOR._install_wheels(
                wheel_directory,
                install_directory,
                [wheel_directory / "acme_demo-1.0-py3-none-any.whl"],
                cache_directory,
            )

        self.assertEqual(duration, 1.0)
        command = run.call_args.args[0]
        self.assertIn(_VALIDATOR._PYPI_INDEX_URL, command)
        self.assertIn(_VALIDATOR._NVIDIA_PYPI_INDEX_URL, command)
        self.assertEqual(command.count("--find-links"), 1)
        self.assertEqual(command[command.index("--find-links") + 1], str(wheel_directory))
        self.assertFalse(any("artifactory" in argument for argument in command))

    def test_download_uses_required_dependency_sources(self) -> None:
        """Download dependencies using only public indexes and the local wheelhouse."""
        wheel_directory = Path("/tmp/wheels")
        download_directory = Path("/tmp/downloads")
        cache_directory = Path("/tmp/cache")

        with (
            mock.patch.object(_VALIDATOR.subprocess, "run") as run,
            mock.patch.object(_VALIDATOR.time, "perf_counter", side_effect=(1.0, 3.0)),
            mock.patch.object(Path, "mkdir"),
        ):
            duration = _VALIDATOR._download_wheel_dependencies(
                wheel_directory,
                download_directory,
                [wheel_directory / "acme_demo-1.0-py3-none-any.whl"],
                cache_directory,
            )

        self.assertEqual(duration, 2.0)
        command = run.call_args.args[0]
        self.assertIn("download", command)
        self.assertIn(_VALIDATOR._PYPI_INDEX_URL, command)
        self.assertIn(_VALIDATOR._NVIDIA_PYPI_INDEX_URL, command)
        self.assertEqual(command.count("--find-links"), 1)
        self.assertEqual(command[command.index("--find-links") + 1], str(wheel_directory))
        self.assertFalse(any("artifactory" in argument for argument in command))
        self.assertIn(str(download_directory), command)

    def test_validate_imports_uses_fresh_processes(self) -> None:
        """Import each module in a distinct child process."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            install_directory = Path(temporary_directory) / "site-packages"
            install_directory.mkdir()
            marker_path = Path(temporary_directory) / "imports.txt"
            module_source = (
                "import os\n"
                "from pathlib import Path\n"
                'with Path(os.environ["ISAACSIM_IMPORT_MARKER"]).open("a", encoding="utf-8") as stream:\n'
                '    stream.write(f"{os.getpid()}\\n")\n'
            )
            (install_directory / "acme_first.py").write_text(module_source, encoding="utf-8")
            (install_directory / "acme_second.py").write_text(module_source, encoding="utf-8")
            metadata_directory = install_directory / "acme_demo-1.0.dist-info"
            metadata_directory.mkdir()
            (metadata_directory / "METADATA").write_text(
                "Metadata-Version: 2.2\nName: acme-demo\nVersion: 1.0\n",
                encoding="utf-8",
            )
            manifests = [
                {
                    "name": "acme-demo",
                    "version": "1.0",
                    "python_imports": ["acme_first", "acme_second"],
                }
            ]

            with mock.patch.dict(os.environ, {"ISAACSIM_IMPORT_MARKER": str(marker_path)}):
                import_count = _VALIDATOR._validate_imports(install_directory, manifests)

            process_ids = marker_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(import_count, 2)
            self.assertEqual(len(set(process_ids)), 2)
            self.assertNotIn(str(os.getpid()), process_ids)

    def test_main_installs_imports_and_writes_metrics(self) -> None:
        """Install a wheel, import its module, and record the footprint."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            wheel_directory = root / "wheels"
            wheel_directory.mkdir()
            _write_test_wheel(wheel_directory / "acme_demo-1.0-py3-none-any.whl")
            metrics_path = root / "metrics.json"

            with mock.patch.object(
                sys,
                "argv",
                [
                    "validate_wheelhouse.py",
                    str(wheel_directory),
                    "--metrics-output",
                    str(metrics_path),
                ],
            ):
                status = _VALIDATOR._main()

            self.assertEqual(status, 0)
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            self.assertEqual(metrics["schema_version"], 4)
            self.assertEqual(metrics["wheel_count"], 1)
            self.assertEqual(metrics["package_count"], 1)
            self.assertEqual(metrics["import_count"], 1)
            self.assertGreater(metrics["install_duration_seconds"], 0)
            self.assertGreater(metrics["installed_size_bytes"], 0)
            self.assertEqual(metrics["installed_distribution_count"], 1)
            self.assertEqual(metrics["downloaded_dependency_distributions"], [])
            self.assertEqual(metrics["dependency_distributions"], [])
            self.assertEqual(len(metrics["wheels"]), 1)
            wheel_metrics = metrics["wheels"][0]
            self.assertEqual(wheel_metrics["filename"], "acme_demo-1.0-py3-none-any.whl")
            self.assertGreater(wheel_metrics["installed_size_bytes"], 0)
            self.assertGreaterEqual(
                wheel_metrics["dependency_install_size_bytes"], wheel_metrics["installed_size_bytes"]
            )
            self.assertEqual(wheel_metrics["dependency_distribution_count"], 1)

    def test_dependency_closure_includes_resolved_distribution(self) -> None:
        """Measure dependency files and distributions for each root wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            wheel_directory = root / "wheels"
            wheel_directory.mkdir()
            root_wheel = wheel_directory / "acme_demo-1.0-py3-none-any.whl"
            dependency_wheel = wheel_directory / "acme_dependency-2.0-py3-none-any.whl"
            _write_test_wheel(root_wheel, ("acme-dependency==2.0",))
            _write_dependency_wheel(dependency_wheel)

            rows = _VALIDATOR._measure_wheel_dependency_closures(
                wheel_directory, [root_wheel], root / "cache", use_indexes=False
            )

            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["dependency_distribution_count"], 2)
            self.assertGreater(rows[0]["dependency_install_size_bytes"], rows[0]["installed_size_bytes"])

            install_directory = root / "installed"
            _VALIDATOR._install_wheels(
                wheel_directory,
                install_directory,
                [root_wheel],
                root / "dependency-cache",
                use_indexes=False,
            )
            dependency_rows = _VALIDATOR._measure_installed_distributions(install_directory, {"acme-demo"})

            download_directory = root / "downloaded"
            _VALIDATOR._download_wheel_dependencies(
                wheel_directory,
                download_directory,
                [root_wheel],
                root / "download-cache",
                use_indexes=False,
            )
            downloaded_rows = _VALIDATOR._measure_downloaded_distributions(download_directory, {"acme-demo"})

            self.assertEqual([row["name"] for row in dependency_rows], ["acme-dependency"])
            self.assertEqual(dependency_rows[0]["version"], "2.0")
            self.assertGreater(dependency_rows[0]["installed_size_bytes"], 0)
            self.assertEqual([row["name"] for row in downloaded_rows], ["acme-dependency"])
            self.assertEqual(downloaded_rows[0]["version"], "2.0")
            self.assertEqual(downloaded_rows[0]["filename"], dependency_wheel.name)
            self.assertEqual(downloaded_rows[0]["archive_size_bytes"], dependency_wheel.stat().st_size)


if __name__ == "__main__":
    unittest.main()
