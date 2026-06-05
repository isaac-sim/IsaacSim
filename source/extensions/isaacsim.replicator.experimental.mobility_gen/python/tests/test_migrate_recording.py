# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for the _migrate_recording function in migrate_recordings.py."""

import os
import tempfile
from pathlib import Path

import numpy as np
import omni.kit.test


# ---------------------------------------------------------------------------
# Load _migrate_recording from the standalone script via exec so we don't
# need to add it to any package path.
# ---------------------------------------------------------------------------
def _find_standalone_examples_root() -> Path:
    # Walk upward to find the directory that contains standalone_examples/.
    # In the source tree that's source/; in the build tree it's _build/.../release/.
    candidate = Path(__file__).resolve()
    while True:
        candidate = candidate.parent
        if (candidate / "standalone_examples").is_dir():
            return candidate
        if candidate.parent == candidate:
            raise FileNotFoundError("Cannot locate standalone_examples directory from " + str(Path(__file__)))


_SCRIPT = (
    _find_standalone_examples_root() / "standalone_examples" / "replicator" / "mobility_gen" / "migrate_recordings.py"
)
_globs: dict = {}
exec(compile(_SCRIPT.read_text(), str(_SCRIPT), "exec"), _globs)
_migrate_recording = _globs["_migrate_recording"]


def _make_legacy_recording(root: str, step_data: list[dict]) -> str:
    """Write legacy .npy files (pickled dicts) under root/state/common/ and return root."""
    common_dir = os.path.join(root, "state", "common")
    os.makedirs(common_dir, exist_ok=True)
    for i, data in enumerate(step_data):
        np.save(os.path.join(common_dir, f"{i:06d}.npy"), data)
    return root


class TestMigrateRecordingNoFiles(omni.kit.test.AsyncTestCase):
    """_migrate_recording must return 0 when no .npy files are present."""

    async def test_empty_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(_migrate_recording(tmp), 0)

    async def test_directory_without_common_subdir(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.makedirs(os.path.join(tmp, "state"))
            self.assertEqual(_migrate_recording(tmp), 0)


class TestMigrateRecordingSingleFile(omni.kit.test.AsyncTestCase):
    """_migrate_recording must convert one .npy to .npz and delete the original."""

    async def test_npy_converted_to_npz(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = {"pose_x": np.float32(1.0), "pose_y": np.float32(2.0)}
            _make_legacy_recording(tmp, [data])

            count = _migrate_recording(tmp)

            self.assertEqual(count, 1)
            common = os.path.join(tmp, "state", "common")
            self.assertFalse(
                os.path.exists(os.path.join(common, "000000.npy")),
                ".npy should be removed",
            )
            self.assertTrue(
                os.path.exists(os.path.join(common, "000000.npz")),
                ".npz should be created",
            )

    async def test_npz_contains_correct_keys(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = {"pose_x": np.float32(3.0), "joint_vel": np.array([0.1, 0.2])}
            _make_legacy_recording(tmp, [data])
            _migrate_recording(tmp)

            # `np.load` mmaps the .npz on Windows; close it before tempdir cleanup.
            with np.load(os.path.join(tmp, "state", "common", "000000.npz")) as npz:
                self.assertIn("pose_x", npz)
                self.assertIn("joint_vel", npz)
                np.testing.assert_allclose(npz["pose_x"], np.float32(3.0))
                np.testing.assert_allclose(npz["joint_vel"], [0.1, 0.2])

    async def test_none_values_excluded_from_npz(self):
        """None values in the legacy dict must be dropped (np.savez can't store None)."""
        with tempfile.TemporaryDirectory() as tmp:
            data = {"pose_x": np.float32(1.0), "optional": None}
            _make_legacy_recording(tmp, [data])
            _migrate_recording(tmp)

            with np.load(os.path.join(tmp, "state", "common", "000000.npz")) as npz:
                self.assertIn("pose_x", npz)
                self.assertNotIn("optional", npz)


class TestMigrateRecordingMultipleFiles(omni.kit.test.AsyncTestCase):
    """_migrate_recording must convert all .npy files in the directory."""

    async def test_count_matches_file_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            steps = [{"x": np.float32(i)} for i in range(5)]
            _make_legacy_recording(tmp, steps)

            count = _migrate_recording(tmp)

            self.assertEqual(count, 5)

    async def test_all_npy_removed_all_npz_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            steps = [{"x": np.float32(i)} for i in range(3)]
            _make_legacy_recording(tmp, steps)
            _migrate_recording(tmp)

            common = os.path.join(tmp, "state", "common")
            npy_files = list(Path(common).glob("*.npy"))
            npz_files = list(Path(common).glob("*.npz"))
            self.assertEqual(len(npy_files), 0, "All .npy files should be removed")
            self.assertEqual(len(npz_files), 3, "One .npz per original .npy")

    async def test_idempotent_on_already_migrated(self):
        """A directory with no .npy files returns 0 (already migrated)."""
        with tempfile.TemporaryDirectory() as tmp:
            steps = [{"x": np.float32(i)} for i in range(2)]
            _make_legacy_recording(tmp, steps)
            _migrate_recording(tmp)
            # Run again — should find nothing to migrate
            count = _migrate_recording(tmp)
            self.assertEqual(count, 0)
