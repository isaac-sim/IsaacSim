# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for synthetic recorder timeline control behavior."""

import os
import shutil

import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.replicator.synthetic_recorder.synthetic_recorder import SyntheticRecorder
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestRecorderData(omni.kit.test.AsyncTestCase):
    """Test that the recorder writes the correct number of frames."""

    async def setUp(self) -> None:
        """Set up a new stage before each test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Wait for assets to finish loading after each test."""
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def setup_stage_with_semantics(self) -> None:
        """Create a stage with semantically labeled primitives."""
        await omni.usd.get_context().new_stage_async()
        rep.functional.create.cube(semantics=[("class", "cube")])
        rep.functional.create.sphere(position=(1, 1, 0), semantics=[("class", "sphere")])

    async def run_recorder_capture(self, num_frames: int, play_timeline: bool, control_timeline: bool) -> None:
        """Run a recording capture and validate timeline behavior and output frame count."""
        test_name = f"_out_num_frames_{num_frames}_play_timeline_{play_timeline}_control_timeline_{control_timeline}"
        print(f"Starting test: {test_name}")

        recorder = SyntheticRecorder()
        recorder.rt_subframes = 0
        recorder.verbose = True
        recorder.writer_name = "BasicWriter"
        recorder.writer_params = {
            "rgb": True,
            "bounding_box_2d_tight": False,
            "bounding_box_2d_loose": False,
            "semantic_segmentation": False,
            "colorize_semantic_segmentation": False,
            "instance_id_segmentation": False,
            "colorize_instance_id_segmentation": False,
            "instance_segmentation": False,
            "colorize_instance_segmentation": False,
            "distance_to_camera": False,
            "distance_to_image_plane": False,
            "colorize_depth": False,
            "bounding_box_3d": False,
            "occlusion": False,
            "normals": False,
            "motion_vectors": False,
            "camera_params": False,
            "pointcloud": False,
            "pointcloud_include_unlabelled": False,
            "skeleton_data": False,
        }
        recorder.rp_data = [["/OmniverseKit_Persp", 512, 512, ""]]

        # Test parameters
        recorder.num_frames = num_frames
        recorder.control_timeline = control_timeline

        # Configure backend
        out_dir_path = os.path.join(os.getcwd(), test_name)
        recorder.backend_type = "DiskBackend"
        recorder.backend_params = {"output_dir": out_dir_path}

        # Clear the files in the output directory, keep the directory
        if os.path.exists(out_dir_path):
            shutil.rmtree(out_dir_path)

        # Check id the recorder should capture with the timeline running
        timeline = omni.timeline.get_timeline_interface()
        if play_timeline:
            timeline.play()
            await omni.kit.app.get_app().next_update_async()
        print(f"Timeline is_playing: {timeline.is_playing()}; current_time: {timeline.get_current_time()}")

        # Check if the timeline is in the correct state
        self.assertTrue(
            timeline.is_playing() == play_timeline,
            f"Timeline is not in the correct play state. Expected: {play_timeline}, Actual: {timeline.is_playing()}",
        )

        # Start the recorder and wait for all the data to be captured
        await recorder.start_stop_async()
        await omni.kit.app.get_app().next_update_async()
        print(f"Timeline is_playing: {timeline.is_playing()}; current_time: {timeline.get_current_time()}")

        # Check if the output directory was created
        self.assertTrue(os.path.exists(out_dir_path), f"Output directory {out_dir_path} was not created.")

        # If control_timeline is True, the timeline should not be playing after recording.
        if control_timeline:
            self.assertTrue(
                not timeline.is_playing(),
                "Timeline is still playing after recording when control_timeline is True.",
            )

        # If control_timeline is False and play_timeline is True, the timeline should still be playing.
        if not control_timeline and play_timeline:
            self.assertTrue(
                timeline.is_playing(),
                "Timeline is not playing after recording when control_timeline is False and play_timeline is True.",
            )

        # If not control_timeline and play_timeline is True, the timeline should be advanced.
        if not control_timeline and play_timeline:
            self.assertTrue(
                timeline.get_current_time() > 0.0,
                "Timeline did not advance after recording when control_timeline or play_timeline is True.",
            )

        # Check if the output directory contains the correct number of written frames
        self.assertTrue(
            validate_folder_contents(out_dir_path, {"png": num_frames}),
            f"Expected {num_frames} frames in {out_dir_path}, but found: {os.listdir(out_dir_path)}",
        )

    async def test_recorder_data_play_timeline_false_control_timeline_false(self) -> None:
        """Test recording with timeline stopped and no timeline control."""
        await self.setup_stage_with_semantics()
        await self.run_recorder_capture(num_frames=5, play_timeline=False, control_timeline=False)

    async def test_recorder_data_play_timeline_false_control_timeline_true(self) -> None:
        """Test recording with timeline stopped and timeline control enabled."""
        await self.setup_stage_with_semantics()
        await self.run_recorder_capture(num_frames=5, play_timeline=False, control_timeline=True)

    async def test_recorder_data_play_timeline_true_control_timeline_false(self) -> None:
        """Test recording with timeline playing and no timeline control."""
        await self.setup_stage_with_semantics()
        await self.run_recorder_capture(num_frames=5, play_timeline=True, control_timeline=False)

    async def test_recorder_data_play_timeline_true_control_timeline_true(self) -> None:
        """Test recording with timeline playing and timeline control enabled."""
        await self.setup_stage_with_semantics()
        await self.run_recorder_capture(num_frames=5, play_timeline=True, control_timeline=True)
