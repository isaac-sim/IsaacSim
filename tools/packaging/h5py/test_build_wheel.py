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

"""Test source integrity, package identity and platform routing without builds."""

from __future__ import annotations

import hashlib
import io
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

import build_wheel


class BuildWheelTests(unittest.TestCase):
    """Exercise fail-closed handling of packaging inputs."""

    def test_platform_matrix(self) -> None:
        self.assertEqual(build_wheel._platform_tag("Linux", "aarch64"), "manylinux_2_35_aarch64")
        self.assertEqual(build_wheel._platform_tag("Linux", "x86_64"), "manylinux_2_35_x86_64")
        self.assertEqual(build_wheel._platform_tag("Windows", "AMD64"), "win_amd64")
        for system, machine in [("Darwin", "arm64"), ("Windows", "ARM64"), ("Linux", "i686")]:
            with self.assertRaises(ValueError):
                build_wheel._platform_tag(system, machine)

    def test_linux_repair_enforces_platform(self) -> None:
        command = build_wheel._repair_command(Path("a.whl"), Path("prefix"), Path("out"), "manylinux_2_35_aarch64")
        self.assertIn("auditwheel", command)
        self.assertEqual(command[command.index("--plat") + 1], "manylinux_2_35_aarch64")

    def test_windows_repair_searches_built_dlls(self) -> None:
        command = build_wheel._repair_command(Path("a.whl"), Path("prefix"), Path("out"), "win_amd64")
        self.assertIn("delvewheel", command)
        self.assertEqual(command[command.index("--add-path") + 1], str(Path("prefix/bin")))

    def test_hash_mismatch_does_not_extract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "wrong.tar.gz"
            archive.write_bytes(b"wrong source")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                build_wheel._extract(archive, "hdf5", root / "out")
            self.assertFalse((root / "out").exists())

    def test_snapshot_uses_work_filesystem_without_system_temp(self) -> None:
        """Keep snapshot allocation on the work filesystem even if system temp is unavailable."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            work = root / "work"
            work.mkdir()
            archive = root / "wrong.tar.gz"
            archive.write_bytes(b"wrong source")
            with patch.object(tempfile, "gettempdir", side_effect=OSError("system temp unavailable")):
                with patch.object(tempfile, "TemporaryFile", wraps=tempfile.TemporaryFile) as create_snapshot:
                    with self.assertRaisesRegex(ValueError, "hash mismatch"):
                        build_wheel._extract(archive, "hdf5", work / "out")
            create_snapshot.assert_called_once_with(dir=work)
            self.assertEqual(list(work.iterdir()), [])

    def test_archive_traversal_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "source.tar"
            with tarfile.open(archive, "w") as stream:
                entry = tarfile.TarInfo("../escaped")
                entry.size = 1
                stream.addfile(entry, io.BytesIO(b"x"))
            digest = hashlib.sha256(archive.read_bytes()).hexdigest()
            with patch.dict(build_wheel.SOURCES, {"test": ("source", digest)}):
                with self.assertRaises(tarfile.FilterError):
                    build_wheel._extract(archive, "test", root / "out")
            self.assertFalse((root / "escaped").exists())

    def test_extract_uses_verified_snapshot_when_input_changes(self) -> None:
        """Ensure input changes after verification do not change extracted bytes."""

        def archive_bytes(content: bytes) -> bytes:
            buffer = io.BytesIO()
            with tarfile.open(fileobj=buffer, mode="w:gz") as stream:
                entry = tarfile.TarInfo("source/probe.txt")
                entry.size = len(content)
                stream.addfile(entry, io.BytesIO(content))
            return buffer.getvalue()

        verified = archive_bytes(b"verified")
        replacement = archive_bytes(b"replacement")
        open_tar = tarfile.open
        for replace_path in (False, True):
            with self.subTest(replace_path=replace_path), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                archive = root / "source.tar.gz"
                archive.write_bytes(verified)

                def change_input(*args: object, **kwargs: object) -> tarfile.TarFile:
                    if replace_path:
                        other = root / "replacement.tar.gz"
                        other.write_bytes(replacement)
                        other.replace(archive)
                    else:
                        archive.write_bytes(replacement)
                    return open_tar(*args, **kwargs)

                digest = hashlib.sha256(verified).hexdigest()
                with patch.dict(build_wheel.SOURCES, {"test": ("source", digest)}):
                    with patch.object(build_wheel.tarfile, "open", side_effect=change_input) as mocked_open:
                        result = build_wheel._extract(archive, "test", root / "out")
                mocked_open.assert_called_once()
                self.assertEqual(archive.read_bytes(), replacement)
                self.assertEqual((result / "probe.txt").read_bytes(), b"verified")

    def test_version_updates_metadata_and_runtime(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "h5py").mkdir()
            (root / "pyproject.toml").write_text("version = '3.16.0'\n")
            (root / "h5py/version.py").write_text(
                'version = "{0.major:d}.{0.minor:d}.{0.bugfix:d}".format(version_tuple)\n'
            )
            build_wheel._set_version(root)
            for path in [root / "pyproject.toml", root / "h5py/version.py"]:
                self.assertIn(build_wheel.H5PY_VERSION, path.read_text())
            with self.assertRaises(ValueError):
                build_wheel._set_version(root)

    def test_repaired_wheel_requires_both_native_libraries(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            wheel = Path(directory) / "candidate.whl"
            with zipfile.ZipFile(wheel, "w") as archive:
                archive.writestr("h5py.libs/libhdf5-test.so.320", b"fixture")
            with self.assertRaisesRegex(RuntimeError, "high-level"):
                build_wheel._check_bundled_hdf5(wheel)
            with zipfile.ZipFile(wheel, "a") as archive:
                archive.writestr("h5py.libs/libhdf5_hl-test.so.320", b"fixture")
            build_wheel._check_bundled_hdf5(wheel)


if __name__ == "__main__":
    unittest.main()
