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

"""Tests for synthetic recorder output file validation."""

import os

import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from isaacsim.replicator.synthetic_recorder.synthetic_recorder import RecorderState, SyntheticRecorder
from isaacsim.storage.native import get_assets_root_path_async
from isaacsim.test.utils.file_validation import validate_folder_contents

BASIC_WRITER_ANNOTATORS = (
    "rgb",
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
    "distance_to_camera",
    "distance_to_image_plane",
    "bounding_box_3d",
    "occlusion",
    "normals",
    "motion_vectors",
    "camera_params",
    "pointcloud",
    "skeleton_data",
)

BASIC_WRITER_ANNOTATORS_ARGS = (
    "colorize_semantic_segmentation",
    "colorize_instance_id_segmentation",
    "colorize_instance_segmentation",
    "pointcloud_include_unlabelled",
    "colorize_depth",
)

SEMANTICS_DEPENDENT_ANNOTATORS = {
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
    "bounding_box_3d",
    "occlusion",
}

EXPECTED_FILES_PER_FRAME_BY_ANNOTATOR = {
    "rgb": {
        "png": 1,
    },
    "bounding_box_2d_tight": {
        "npy": 1,
        "json": 2,
    },
    "bounding_box_2d_loose": {
        "npy": 1,
        "json": 2,
    },
    "semantic_segmentation": {
        "png": 1,
        "json": 1,
    },
    "instance_id_segmentation": {
        "png": 1,
        "json": 1,
    },
    "instance_segmentation": {
        "png": 1,
        "json": 2,
    },
    "distance_to_camera": {
        "npy": 1,
        "png": 1,
    },
    "distance_to_image_plane": {
        "npy": 1,
        "png": 1,
    },
    "bounding_box_3d": {
        "npy": 1,
        "json": 2,
    },
    "occlusion": {
        "npy": 1,
    },
    "normals": {
        "png": 1,
    },
    "motion_vectors": {
        "npy": 1,
    },
    "camera_params": {
        "json": 1,
    },
    "pointcloud": {
        "npy": 5,
    },
    "skeleton_data": {
        "json": 1,
    },
}


def compute_expected_file_counts(
    num_frames: int, include_semantics: bool = True, include_skeleton: bool = False, include_metadata: bool = True
) -> dict:
    """Compute expected file counts by extension for validation.

    Args:
        num_frames: Number of frames to compute counts for.
        include_semantics: Whether to include semantics-dependent annotators.
        include_skeleton: Whether to include skeleton data annotators.
        include_metadata: Whether to include metadata files.

    Returns:
        Dictionary mapping file extensions to expected counts.
    """
    expected_counts = {}

    for annotator, per_frame_counts in EXPECTED_FILES_PER_FRAME_BY_ANNOTATOR.items():
        if not include_semantics and annotator in SEMANTICS_DEPENDENT_ANNOTATORS:
            continue
        if not include_skeleton and annotator == "skeleton_data":
            continue

        for extension, count_per_frame in per_frame_counts.items():
            total_count = count_per_frame * num_frames
            expected_counts[extension] = expected_counts.get(extension, 0) + total_count

    if include_metadata:
        expected_counts["txt"] = expected_counts.get("txt", 0) + 1

    return expected_counts


class TestRecorderBasic(omni.kit.test.AsyncTestCase):
    """Test the basic functionality of the recorder."""

    async def setUp(self) -> None:
        """Set up a new stage before each test."""
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Wait for assets to finish loading and clean up after each test."""
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def setup_stage_with_no_semantics(self) -> None:
        """Create a stage with primitives but no semantic labels."""
        rep.functional.create.cube()
        rep.functional.create.sphere(position=(1, 1, 0))

    async def setup_stage_with_semantics(self) -> None:
        """Create a stage with semantically labeled primitives."""
        rep.functional.create.cube(semantics={"class": "my_cube"})
        rep.functional.create.sphere(position=(1, 1, 0), semantics={"class": "my_sphere"})

    async def setup_stage_with_skeleton_data(self) -> None:
        """Create a stage with a skeletal character asset."""
        SKELETAL_ASSET_PATH = "/NVIDIA/Assets/Characters/Reallusion/Worker/Worker.usd"
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + SKELETAL_ASSET_PATH
        rep.functional.create.reference(asset_path, scale=(0.01, 0.01, 0.01), semantics={"class": "worker"})

    async def run_recorder_loop_basic_writer_all_annotators_async(
        self, num_iterations: int, num_frames: int, out_dir: str, rp_data: object
    ) -> None:
        """Run the recorder loop with the basic writer and all annotators enabled.

        Args:
            num_iterations: Number of recorder loop iterations to run.
            num_frames: Number of frames to capture.
            out_dir: Output directory for recorder data.
            rp_data: Render product data used by the recorder.
        """
        # Create a new instance of the SyntheticRecorder
        recorder = SyntheticRecorder()
        recorder.num_frames = num_frames
        recorder.rt_subframes = 0
        recorder.control_timeline = False
        recorder.verbose = True

        # Use the basic writer with all annotators enabled
        recorder.writer_name = "BasicWriter"
        recorder.writer_params = dict.fromkeys(BASIC_WRITER_ANNOTATORS, True)
        recorder.writer_params.update(dict.fromkeys(BASIC_WRITER_ANNOTATORS_ARGS, True))

        # Render products, will created and destroyed every new capture
        recorder.rp_data = rp_data

        # Configure backend for each iteration
        for i in range(num_iterations):
            test_out_dir = f"{out_dir}_{i}"
            out_dir_path = os.path.join(os.getcwd(), test_out_dir)
            recorder.backend_type = "DiskBackend"
            recorder.backend_params = {"output_dir": out_dir_path}
            print(f"Starting recorder {i}; writing data to {out_dir_path}")
            await recorder.start_stop_async()
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(
                recorder.get_state() == RecorderState.STOPPED, "Recorder did not stop after start_stop_async()"
            )

    async def test_recorder_empty_stage(self) -> None:
        """Test recording on an empty stage produces expected output files."""
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            [cam_path, 256, 256, "my_cam_rp_name"],
        ]
        num_iterations = 1
        num_frames = 3
        out_dir = "_out_sdrec_test_empty"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            test_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            # Empty stage should not have semantics related annotators nor skeleton data
            expected_counts = compute_expected_file_counts(
                num_frames=num_frames, include_semantics=False, include_skeleton=False
            )
            self.assertTrue(
                validate_folder_contents(test_out_dir, expected_counts),
                f"Folder validation failed for {test_out_dir} with expected counts: {expected_counts}",
            )

    async def test_recorder_no_semantics(self) -> None:
        """Test recording a stage with no semantic labels."""
        await self.setup_stage_with_no_semantics()
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            [cam_path, 256, 256, "my_cam_rp_name"],
        ]
        num_iterations = 1
        num_frames = 3
        out_dir = "_out_sdrec_test_no_semantics"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            test_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            # No semantics should not have semantics related annotators
            expected_counts = compute_expected_file_counts(
                num_frames=num_frames, include_semantics=False, include_skeleton=False
            )
            self.assertTrue(
                validate_folder_contents(test_out_dir, expected_counts),
                f"Folder validation failed for {test_out_dir} with expected counts: {expected_counts}",
            )

    async def test_recorder_with_semantics(self) -> None:
        """Test recording a stage with semantic labels produces semantic annotator outputs."""
        await self.setup_stage_with_semantics()
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            [cam_path, 256, 256, "my_cam_rp_name"],
        ]
        num_iterations = 1
        num_frames = 3
        out_dir = "_out_sdrec_test_with_semantics"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            test_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            # With semantics should have semantics related annotators
            expected_counts = compute_expected_file_counts(
                num_frames=num_frames, include_semantics=True, include_skeleton=False
            )
            self.assertTrue(
                validate_folder_contents(test_out_dir, expected_counts),
                f"Folder validation failed for {test_out_dir} with expected counts: {expected_counts}",
            )

    async def test_recorder_with_skeleton_data(self) -> None:
        """Test recording a stage with skeleton data produces skeleton annotator outputs."""
        await self.setup_stage_with_skeleton_data()
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            [cam_path, 256, 256, "my_cam_rp_name"],
        ]
        num_iterations = 1
        num_frames = 3
        out_dir = "_out_sdrec_test_with_skeleton_data"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            test_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            # With skeleton data should have skeleton data annotator
            expected_counts = compute_expected_file_counts(
                num_frames=num_frames, include_semantics=True, include_skeleton=True
            )
            self.assertTrue(
                validate_folder_contents(test_out_dir, expected_counts),
                f"Folder validation failed for {test_out_dir} with expected counts: {expected_counts}",
            )

    async def test_recorder_multiple_iterations(self) -> None:
        """Test recording multiple iterations with multiple render products."""
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            ["/OmniverseKit_Persp", 256, 256, "my_rp_name"],
            [cam_path, 256, 256, "my_cam_rp_name"],
        ]
        num_iterations = 3
        num_frames = 3
        out_dir = "_out_sdrec_test_multiple_iterations"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            base_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            subfolders = ["my_rp_name", "my_cam_rp_name"]
            for subfolder in subfolders:
                test_subfolder_dir = os.path.join(base_out_dir, subfolder)
                # Multiple iterations should have the same expected counts, metadata is not included in subfolders
                expected_counts = compute_expected_file_counts(
                    num_frames=num_frames, include_semantics=False, include_skeleton=False, include_metadata=False
                )
                self.assertTrue(
                    validate_folder_contents(test_subfolder_dir, expected_counts, recursive=True),
                    f"Folder validation failed for {test_subfolder_dir} with expected counts: {expected_counts}",
                )

    async def test_recorder_multiple_renders_products(self) -> None:
        """Test recording with multiple render products including named and unnamed."""
        cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), name="my_rep_camera")
        cam_path = cam.GetPath()
        rp_data = [
            ["/OmniverseKit_Persp", 256, 256, "my_rp_name"],
            ["/OmniverseKit_Persp", 256, 256, ""],
            [cam_path, 256, 256, "my_cam_rp_name"],
            [cam_path, 256, 256, ""],
        ]
        num_iterations = 1
        num_frames = 3
        out_dir = "_out_sdrec_test_multiple_renders_products"
        await self.run_recorder_loop_basic_writer_all_annotators_async(
            num_iterations=num_iterations, num_frames=num_frames, out_dir=out_dir, rp_data=rp_data
        )
        for i in range(num_iterations):
            base_out_dir = os.path.join(os.getcwd(), f"{out_dir}_{i}")
            # Multiple annotators are split into subfolders, unnamed render products are in the form "Replicator_{i}"
            subfolders = ["my_rp_name", "my_cam_rp_name", "Replicator", "Replicator_01"]
            for subfolder in subfolders:
                test_subfolder_dir = os.path.join(base_out_dir, subfolder)
                # Multiple renders products should have the same expected counts, metadata is not included in subfolders
                expected_counts = compute_expected_file_counts(
                    num_frames=num_frames, include_semantics=False, include_skeleton=False, include_metadata=False
                )
                self.assertTrue(
                    validate_folder_contents(test_subfolder_dir, expected_counts, recursive=True),
                    f"Folder validation failed for {test_subfolder_dir} with expected counts: {expected_counts}",
                )
