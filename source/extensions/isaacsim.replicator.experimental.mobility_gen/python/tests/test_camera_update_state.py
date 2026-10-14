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

"""Tests that an empty annotator buffer drops the frame instead of re-persisting the previous one.

A Replicator annotator can hand back a zero-length buffer when the renderer has not yet produced a
frame for its render product. The camera must report that miss rather than leave the previous frame
in the buffer, otherwise the writer serializes a byte-identical duplicate image under a step index
whose recorded pose has moved.

The tests drive `MobilityGenCamera.update_state()` with a scripted annotator and write through a
real `MobilityGenWriter`, then read the result back with a real `MobilityGenReader`.
"""

from __future__ import annotations

import os
import tempfile
from unittest import mock

import numpy as np
import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen.impl import camera as camera_impl
from isaacsim.replicator.experimental.mobility_gen.impl.camera import MobilityGenCamera
from isaacsim.replicator.experimental.mobility_gen.impl.reader import MobilityGenReader
from isaacsim.replicator.experimental.mobility_gen.impl.writer import MobilityGenWriter
from pxr import Gf, Usd, UsdGeom

CAMERA_PRIM_PATH = "/World/robot/front_camera/left"
CAMERA_PREFIX = "robot.front_camera.left"
WIDTH = 6
HEIGHT = 4
POSE_DELTA_M = 0.8


class _ScriptedAnnotator:
    """Annotator stand-in that returns a scripted `get_data()` result per `update_state()` call.

    Args:
        frames: The results to return, in order. The last entry repeats once exhausted.
    """

    def __init__(self, frames: list) -> None:
        self._frames = list(frames)
        self._index = 0

    def get_data(self, do_array_copy: bool = True) -> object:
        """Return the next scripted frame.

        Args:
            do_array_copy: Accepted for signature compatibility with the real annotator; unused.

        Returns:
            The scripted frame for this call.
        """
        frame = self._frames[min(self._index, len(self._frames) - 1)]
        self._index += 1
        return frame


class TestCameraUpdateState(omni.kit.test.AsyncTestCase):
    """Test empty-annotator-buffer handling in MobilityGenCamera."""

    async def setUp(self) -> None:
        """Create an in-memory camera prim, a camera module, and an output directory."""
        self._temp_dir = tempfile.TemporaryDirectory()
        self.output_path = self._temp_dir.name
        self.stage = Usd.Stage.CreateInMemory()
        xform = UsdGeom.Xform.Define(self.stage, CAMERA_PRIM_PATH)
        self.translate_op = xform.AddTranslateOp()
        # The camera resolves its prim against the current stage; point it at the in-memory one so
        # the test neither depends on nor mutates the stage shared by the rest of the test process.
        with mock.patch.object(camera_impl, "get_prim_at_path", return_value=xform.GetPrim()):
            self.camera = MobilityGenCamera(CAMERA_PRIM_PATH, (WIDTH, HEIGHT))

    async def tearDown(self) -> None:
        """Remove the output directory."""
        self._temp_dir.cleanup()

    def _record(self, num_steps: int, drop_incomplete_steps: bool = False) -> list[int]:
        """Drive `num_steps` update/write cycles, translating the camera `POSE_DELTA_M` each step.

        Args:
            num_steps: The number of steps to record.
            drop_incomplete_steps: If True, apply the replay loop's policy — a step whose capture is
                incomplete writes nothing at all, common state included, so that every recorded step
                keeps exactly one image per modality.

        Returns:
            The step indices that were written.
        """
        writer = MobilityGenWriter(self.output_path, async_write=False)
        written_steps = []
        try:
            for step in range(num_steps):
                self.translate_op.Set(Gf.Vec3d(step * POSE_DELTA_M, 0.0, 0.0))
                self.camera.update_state()
                if drop_incomplete_steps and self.camera.named_missing_modalities(prefix=CAMERA_PREFIX):
                    continue
                writer.write_state_dict_common(self.camera.state_dict_common(prefix=CAMERA_PREFIX), step)
                writer.write_state_dict_rgb(self.camera.state_dict_rgb(prefix=CAMERA_PREFIX), step)
                writer.write_state_dict_depth(self.camera.state_dict_depth(prefix=CAMERA_PREFIX), step)
                written_steps.append(step)
        finally:
            writer.close()
        return written_steps

    def _image_path(self, modality: str, step: int, extension: str) -> str:
        """Build the on-disk path of a recorded image.

        Args:
            modality: The `state/<modality>` subdirectory name.
            step: The step index.
            extension: The file extension, without a leading dot.

        Returns:
            The absolute path of the image for that step.
        """
        folder = os.path.join(self.output_path, "state", modality, f"{CAMERA_PREFIX}.{modality}_image")
        return os.path.join(folder, f"{step:08d}.{extension}")

    def _assert_frames_differ(self, first_path: str, second_path: str, message: str) -> None:
        """Assert that two recorded frames are not byte-identical duplicates.

        A dropped frame satisfies the assertion: the step it belongs to writes no image at all.

        Args:
            first_path: Path of the earlier frame, which must exist.
            second_path: Path of the later frame, which may legitimately be absent.
            message: The failure message describing the duplicate.
        """
        self.assertTrue(os.path.exists(first_path))
        if not os.path.exists(second_path):
            return
        with open(first_path, "rb") as f:
            first_bytes = f.read()
        with open(second_path, "rb") as f:
            second_bytes = f.read()
        self.assertFalse(first_bytes == second_bytes, message)

    def _read_recorded_positions(self, steps: list[int]) -> list[np.ndarray]:
        """Read the recorded camera position for each step.

        Args:
            steps: The step indices to read.

        Returns:
            The recorded position for each step.
        """
        positions = []
        for step in steps:
            path = os.path.join(self.output_path, "state", "common", f"{step:08d}.npz")
            positions.append(dict(np.load(path))[f"{CAMERA_PREFIX}.position"])
        return positions

    async def test_empty_depth_buffer_does_not_duplicate_previous_frame(self) -> None:
        """An empty depth buffer must not persist the previous frame under the new step index."""
        self.camera._depth_annotator = _ScriptedAnnotator(
            [
                np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
                np.full((HEIGHT, WIDTH), 7.0, dtype=np.float32),
            ]
        )

        self._record(num_steps=3)

        positions = self._read_recorded_positions([0, 1])
        pose_delta = float(np.linalg.norm(positions[1] - positions[0]))
        self.assertAlmostEqual(pose_delta, POSE_DELTA_M, places=6)

        self._assert_frames_differ(
            self._image_path("depth", 0, "png"),
            self._image_path("depth", 1, "png"),
            f"the depth frame for step 1 is byte-identical to step 0 despite a {pose_delta:.3f} m pose delta; "
            "the empty buffer re-persisted the previous frame",
        )
        self.assertFalse(
            os.path.exists(self._image_path("depth", 1, "png")),
            "step 1 captured no depth frame, so no depth image may be written for it",
        )
        self.assertTrue(
            os.path.exists(self._image_path("depth", 2, "png")),
            "depth capture must resume on the step after a miss, not stay cleared",
        )
        self._assert_frames_differ(
            self._image_path("depth", 0, "png"),
            self._image_path("depth", 2, "png"),
            "the recovered depth frame for step 2 is byte-identical to step 0",
        )

    async def test_dropped_step_keeps_recording_readable(self) -> None:
        """Dropping a step must not leave a common state entry whose images are missing."""
        self.camera._depth_annotator = _ScriptedAnnotator(
            [
                np.zeros((0,), dtype=np.float32),
                np.full((HEIGHT, WIDTH), 3.0, dtype=np.float32),
            ]
        )

        written_steps = self._record(num_steps=2, drop_incomplete_steps=True)
        self.assertEqual(written_steps, [1], "the step with an empty depth buffer should have been dropped")

        reader = MobilityGenReader(self.output_path)
        self.assertEqual(reader.steps, [1])
        # Index 0 addresses the first *surviving* step; it must resolve to a complete step on disk.
        reader.read_state_dict(0)

    async def test_empty_rgb_buffer_does_not_duplicate_previous_frame(self) -> None:
        """An empty RGB buffer must not persist the previous frame under the new step index."""
        self.camera._rgb_annotator = _ScriptedAnnotator(
            [
                np.full((HEIGHT, WIDTH, 4), 10, dtype=np.uint8),
                np.zeros((0,), dtype=np.uint8),
                np.full((HEIGHT, WIDTH, 4), 200, dtype=np.uint8),
            ]
        )

        self._record(num_steps=3)

        self._assert_frames_differ(
            self._image_path("rgb", 0, "jpg"),
            self._image_path("rgb", 1, "jpg"),
            "the RGB frame for step 1 is byte-identical to step 0; the empty buffer re-persisted the previous frame",
        )
        self.assertFalse(
            os.path.exists(self._image_path("rgb", 1, "jpg")),
            "step 1 captured no RGB frame, so no RGB image may be written for it",
        )
        self.assertTrue(
            os.path.exists(self._image_path("rgb", 2, "jpg")),
            "RGB capture must resume on the step after a miss, not stay cleared",
        )
        self._assert_frames_differ(
            self._image_path("rgb", 0, "jpg"),
            self._image_path("rgb", 2, "jpg"),
            "the recovered RGB frame for step 2 is byte-identical to step 0",
        )

    async def test_every_missed_modality_warns_and_is_reported(self) -> None:
        """Every modality that misses a frame must warn and appear in the missing-modality roll-up."""
        empty_image = np.zeros((0,), dtype=np.float32)
        self.camera._rgb_annotator = _ScriptedAnnotator([np.zeros((0,), dtype=np.uint8)])
        self.camera._depth_annotator = _ScriptedAnnotator([empty_image])
        self.camera._normals_annotator = _ScriptedAnnotator([empty_image])
        self.camera._segmentation_annotator = _ScriptedAnnotator([{"data": empty_image, "info": {}}])
        self.camera._instance_id_segmentation_annotator = _ScriptedAnnotator([empty_image])

        with mock.patch.object(camera_impl.carb, "log_warn") as log_warn:
            self.camera.update_state()

        expected = {"rgb", "segmentation", "depth", "instance_id_segmentation", "normals"}
        self.assertEqual(set(self.camera.missing_modalities()), expected)
        self.assertEqual(self.camera.named_missing_modalities(prefix=CAMERA_PREFIX).keys(), {CAMERA_PREFIX})

        warned = " ".join(str(call.args[0]) for call in log_warn.call_args_list)
        for modality in expected:
            # The exact phrase keeps "segmentation" from matching the "instance_id_segmentation" warning.
            self.assertIn(
                f"empty {modality} buffer", warned, f"no warning was emitted for the missed '{modality}' frame"
            )

        # Reporting the miss is only half of it: the stale frame has to be gone from the buffer, or a
        # caller that does not consult missing_modalities() still persists a duplicate.
        for name in (
            "rgb_image",
            "segmentation_image",
            "segmentation_info",
            "depth_image",
            "instance_id_segmentation_image",
            "instance_id_segmentation_info",
            "normals_image",
        ):
            self.assertIsNone(
                getattr(self.camera, name).value, f"'{name}' kept its previous frame after the modality missed"
            )

    async def test_instance_id_fast_annotator_updates_from_raw_array(self) -> None:
        """Store the fast annotator's raw uint32 array without expecting legacy metadata."""
        instance_ids = np.full((HEIGHT, WIDTH), 70_000, dtype=np.uint32)
        self.camera._instance_id_segmentation_annotator = _ScriptedAnnotator([instance_ids])

        self.camera.update_state()

        self.assertTrue(np.array_equal(self.camera.instance_id_segmentation_image.value, instance_ids))
        self.assertIsNone(self.camera.instance_id_segmentation_info.value)

    async def test_invalid_prim_clears_every_buffer(self) -> None:
        """A camera prim that goes invalid must not leave the last good step's frames or pose behind."""
        self.camera._rgb_annotator = _ScriptedAnnotator([np.full((HEIGHT, WIDTH, 4), 10, dtype=np.uint8)])
        self.camera._depth_annotator = _ScriptedAnnotator([np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)])

        self.camera.update_state()
        self.assertIsNotNone(self.camera.rgb_image.value, "the first step should capture normally")
        self.assertIsNotNone(self.camera.position.value)

        # Removing the prim invalidates the handle the camera resolved in __init__, the one way this
        # path is reached after a camera has already captured a step.
        self.stage.RemovePrim(CAMERA_PRIM_PATH)
        self.camera.update_state()

        self.assertEqual(self.camera.missing_modalities(), ["pose"])
        for name, buffer in self.camera.buffers().items():
            self.assertIsNone(buffer.value, f"'{name}' survived an invalid prim and is now a stale frame")

    async def test_unexpected_rgb_buffer_shape_drops_the_frame(self) -> None:
        """An RGB buffer in neither known layout must drop the frame rather than retain the previous one."""
        self.camera._rgb_annotator = _ScriptedAnnotator(
            [
                np.full((HEIGHT, WIDTH, 4), 10, dtype=np.uint8),
                # Non-empty, but neither the (H, W, C) nor the flat (H*W*C,) layout the camera reads.
                np.full((HEIGHT, WIDTH), 99, dtype=np.uint8),
            ]
        )

        with mock.patch.object(camera_impl.carb, "log_warn") as log_warn:
            self.camera.update_state()
            self.assertIsNotNone(self.camera.rgb_image.value)
            self.camera.update_state()

        self.assertIsNone(self.camera.rgb_image.value, "an unreadable RGB buffer must not retain the previous frame")
        self.assertEqual(self.camera.missing_modalities(), ["rgb"])
        warned = " ".join(str(call.args[0]) for call in log_warn.call_args_list)
        self.assertIn("unexpected rgb buffer shape", warned)
