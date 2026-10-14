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

"""Tests for keeping instance ID segmentation separate from semantic segmentation.

Instance ID frames used to carry the "segmentation" tag, so `state_dict_segmentation()` returned
both buffers and the instance ID frames were written into `state/segmentation/` and replayed as if
they were semantic data. These cover the split, and that recordings written before it still read.
"""

import os
import shutil
import tempfile
from unittest import mock

import numpy as np
import omni.kit.test
import omni.usd
import PIL.Image
from isaacsim.replicator.experimental.mobility_gen.impl import camera as camera_impl
from isaacsim.replicator.experimental.mobility_gen.impl.camera import MobilityGenCamera
from isaacsim.replicator.experimental.mobility_gen.impl.reader import MobilityGenReader
from isaacsim.replicator.experimental.mobility_gen.impl.writer import MobilityGenWriter

CAMERA_NAME = "robot.front_camera"
SEMANTIC_FOLDER = f"{CAMERA_NAME}.segmentation_image"
INSTANCE_ID_FOLDER = f"{CAMERA_NAME}.instance_id_segmentation_image"


def _frame(fill: int) -> np.ndarray:
    """Return a 4x4 RGBA frame filled with ``fill``, distinguishing one modality from another."""
    return np.full((4, 4, 4), fill, dtype=np.uint8)


def _instance_id_frame(fill: int) -> np.ndarray:
    """Return a uint32 instance ID frame with values that cannot be represented in a 16-bit PNG."""
    return np.full((4, 4), fill, dtype=np.uint32)


def _write_steps(recording_path: str, steps: int = 2) -> None:
    """Author the per-step `state/common` files the reader derives its step list from."""
    common_dir = os.path.join(recording_path, "state", "common")
    os.makedirs(common_dir, exist_ok=True)
    for step in range(steps):
        np.savez(os.path.join(common_dir, f"{step:08d}.npz"), position=np.zeros(3, dtype=np.float32))


def _write_png(recording_path: str, folder: str, name: str, step: int, fill: int) -> None:
    """Write a single frame directly, standing in for a recording made by an older version."""
    image_dir = os.path.join(recording_path, "state", folder, name)
    os.makedirs(image_dir, exist_ok=True)
    PIL.Image.fromarray(_frame(fill)).save(os.path.join(image_dir, f"{step:08d}.png"))


class TestInstanceIdSegmentationTag(omni.kit.test.AsyncTestCase):
    """The camera's instance ID buffer must not answer to the "segmentation" tag."""

    async def setUp(self) -> None:
        """Create a stage for camera construction."""
        # MobilityGenCamera resolves its prim on construction, which needs a stage.
        await omni.usd.get_context().new_stage_async()

    async def tearDown(self) -> None:
        """Reset the stage after each test."""
        await omni.usd.get_context().new_stage_async()

    async def test_instance_id_buffer_is_not_returned_as_segmentation(self) -> None:
        """Check that semantic state excludes the instance ID buffer."""
        camera = MobilityGenCamera("/World/Camera", (64, 64))

        segmentation = camera.state_dict_segmentation(prefix=CAMERA_NAME)

        self.assertIn(f"{CAMERA_NAME}.segmentation_image", segmentation)
        self.assertNotIn(
            f"{CAMERA_NAME}.instance_id_segmentation_image",
            segmentation,
            "instance ID frames must no longer be returned as semantic segmentation",
        )

    async def test_instance_id_buffer_has_its_own_state_dict(self) -> None:
        """Check that instance ID state excludes the semantic buffer."""
        camera = MobilityGenCamera("/World/Camera", (64, 64))

        instance_id = camera.state_dict_instance_id_segmentation(prefix=CAMERA_NAME)
        common = camera.state_dict_common(prefix=CAMERA_NAME)

        self.assertIn(f"{CAMERA_NAME}.instance_id_segmentation_image", instance_id)
        self.assertNotIn(f"{CAMERA_NAME}.segmentation_image", instance_id)
        self.assertNotIn(f"{CAMERA_NAME}.instance_id_segmentation_image", common)

    async def test_supported_instance_id_annotator_is_used(self) -> None:
        """Check that instance ID rendering uses the supported fast annotator."""
        camera = MobilityGenCamera("/World/Camera", (64, 64))
        camera._render_product = mock.sentinel.render_product
        annotator = mock.MagicMock()

        with mock.patch.object(
            camera_impl.rep.AnnotatorRegistry, "get_annotator", return_value=annotator
        ) as get_annotator:
            camera.enable_instance_id_segmentation_rendering()

        get_annotator.assert_called_once_with(
            "instance_id_segmentation_fast",
            init_params={"colorize": False},
        )
        annotator.attach.assert_called_once_with(mock.sentinel.render_product)


class TestInstanceIdSegmentationRoundTrip(omni.kit.test.AsyncTestCase):
    """Frames written to the new folder must come back as instance ID data."""

    async def setUp(self) -> None:
        """Create a temporary recording directory."""
        self._temp_dir = tempfile.mkdtemp()
        self._recording_path = os.path.join(self._temp_dir, "recording")
        os.makedirs(self._recording_path, exist_ok=True)

    async def tearDown(self) -> None:
        """Remove the temporary recording directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    async def test_writer_uses_a_separate_folder(self) -> None:
        """Check that the writer separates semantic and instance ID frames."""
        writer = MobilityGenWriter(self._recording_path, async_write=False)

        writer.write_state_dict_segmentation({SEMANTIC_FOLDER: _frame(1)}, step=0)
        writer.write_state_dict_instance_id_segmentation({INSTANCE_ID_FOLDER: _instance_id_frame(70_000)}, step=0)

        state_dir = os.path.join(self._recording_path, "state")
        instance_id_path = os.path.join(
            state_dir,
            "instance_id_segmentation",
            INSTANCE_ID_FOLDER,
            "00000000.npy",
        )
        self.assertTrue(os.path.isfile(instance_id_path))
        self.assertTrue(np.array_equal(np.load(instance_id_path), _instance_id_frame(70_000)))
        self.assertEqual(
            sorted(os.listdir(os.path.join(state_dir, "segmentation"))),
            [SEMANTIC_FOLDER],
            "instance ID frames must not be written into state/segmentation",
        )

    async def test_reader_separates_the_two_modalities(self) -> None:
        """Check that direct and aggregate reads preserve both modalities."""
        writer = MobilityGenWriter(self._recording_path, async_write=False)
        _write_steps(self._recording_path)
        for step in range(2):
            writer.write_state_dict_segmentation({SEMANTIC_FOLDER: _frame(1)}, step=step)
            writer.write_state_dict_instance_id_segmentation(
                {INSTANCE_ID_FOLDER: _instance_id_frame(70_000)},
                step=step,
            )

        reader = MobilityGenReader(self._recording_path)

        self.assertEqual(reader.segmentation_names, [SEMANTIC_FOLDER])
        self.assertEqual(reader.instance_id_segmentation_names, [INSTANCE_ID_FOLDER])

        semantic = reader.read_state_dict_segmentation(0)
        instance_id = reader.read_state_dict_instance_id_segmentation(0)
        full_state = reader[0]

        self.assertEqual(list(semantic), [SEMANTIC_FOLDER])
        self.assertEqual(list(instance_id), [INSTANCE_ID_FOLDER])
        self.assertTrue(np.array_equal(semantic[SEMANTIC_FOLDER], _frame(1)))
        self.assertTrue(np.array_equal(instance_id[INSTANCE_ID_FOLDER], _instance_id_frame(70_000)))
        self.assertTrue(np.array_equal(full_state[SEMANTIC_FOLDER], _frame(1)))
        self.assertTrue(np.array_equal(full_state[INSTANCE_ID_FOLDER], _instance_id_frame(70_000)))


class TestLegacyInstanceIdSegmentationRecording(omni.kit.test.AsyncTestCase):
    """Recordings written before the split kept instance ID frames in state/segmentation."""

    async def setUp(self) -> None:
        """Create a recording that uses the legacy folder layout."""
        self._temp_dir = tempfile.mkdtemp()
        self._recording_path = os.path.join(self._temp_dir, "recording")
        _write_steps(self._recording_path)
        # Both modalities under state/segmentation, as an older version would have written them.
        for step in range(2):
            _write_png(self._recording_path, "segmentation", SEMANTIC_FOLDER, step, fill=1)
            _write_png(self._recording_path, "segmentation", INSTANCE_ID_FOLDER, step, fill=2)

    async def tearDown(self) -> None:
        """Remove the temporary recording directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    async def test_legacy_frames_are_reported_as_instance_id(self) -> None:
        """Check that legacy instance ID frames are classified correctly."""
        reader = MobilityGenReader(self._recording_path)

        self.assertEqual(
            reader.segmentation_names,
            [SEMANTIC_FOLDER],
            "legacy instance ID frames must not be replayed as semantic segmentation",
        )
        self.assertEqual(reader.legacy_instance_id_segmentation_names, [INSTANCE_ID_FOLDER])

    async def test_legacy_frames_are_still_readable(self) -> None:
        """Check that direct and aggregate reads include legacy instance ID frames."""
        reader = MobilityGenReader(self._recording_path)

        instance_id = reader.read_state_dict_instance_id_segmentation(0)
        full_state = reader.read_state_dict(0)

        self.assertEqual(list(instance_id), [INSTANCE_ID_FOLDER])
        self.assertTrue(np.array_equal(instance_id[INSTANCE_ID_FOLDER], _frame(2)))
        self.assertTrue(np.array_equal(full_state[INSTANCE_ID_FOLDER], _frame(2)))
