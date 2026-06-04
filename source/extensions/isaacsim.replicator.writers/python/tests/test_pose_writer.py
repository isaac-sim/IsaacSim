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

import json
import os
import tempfile

import numpy as np
import omni.kit
import omni.replicator.core as rep
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents
from isaacsim.test.utils.image_comparison import compare_images_in_directories


def compare_nested_structures_with_tolerance(data1, data2, path="", rtol=1e-5, atol=1e-5):
    if isinstance(data1, (list, tuple)) and isinstance(data2, (list, tuple)):
        if len(data1) != len(data2):
            return f"Length mismatch at {path}: {len(data1)} != {len(data2)}"
        if any(isinstance(x, float) for x in data1 + data2):
            try:
                np.testing.assert_allclose(data1, data2, rtol=rtol, atol=atol)
            except AssertionError:
                return f"Value mismatch at {path}: {data1} != {data2}"
        else:
            for i, (v1, v2) in enumerate(zip(data1, data2)):
                error = compare_nested_structures_with_tolerance(v1, v2, f"{path}[{i}]")
                if error:
                    return error
    elif isinstance(data1, dict) and isinstance(data2, dict):
        keys1, keys2 = set(data1.keys()), set(data2.keys())
        if keys1 != keys2:
            extra1 = keys1 - keys2
            extra2 = keys2 - keys1
            msg = []
            if extra1:
                msg.append(f"Extra keys in first dict at {path}: {extra1}")
            if extra2:
                msg.append(f"Extra keys in second dict at {path}: {extra2}")
            return "\n".join(msg)
        for key in keys1:
            # If quaternion, compare it with its negated quaternion as well since it represents the same rotation
            if (
                "quat" in key
                and isinstance(data1[key], list)
                and isinstance(data2[key], list)
                and len(data1[key]) == 4
                and len(data2[key]) == 4
            ):
                q1 = np.array(data1[key])
                q2 = np.array(data2[key])
                try:
                    np.testing.assert_allclose(q1, q2, rtol=1e-5, atol=1e-5)
                except AssertionError:
                    try:
                        np.testing.assert_allclose(q1, -q2, rtol=1e-5, atol=1e-5)
                    except AssertionError:
                        return f"Quaternion mismatch at {path}.{key}: {data1[key]} != {data2[key]} (and not negative of each other)"
            else:
                error = compare_nested_structures_with_tolerance(
                    data1[key], data2[key], f"{path}.{key}" if path else key
                )
                if error:
                    return error
    elif isinstance(data1, float) and isinstance(data2, float):
        try:
            np.testing.assert_allclose(data1, data2, rtol=1e-5, atol=1e-5)
        except AssertionError:
            return f"Float mismatch at {path}: {data1} != {data2}"
    elif data1 != data2:
        return f"Value mismatch at {path}: {data1} != {data2}"
    return None


class TestPoseWriter(omni.kit.test.AsyncTestCase):

    RGB_MEAN_DIFF_TOLERANCE = 10

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        await omni.usd.get_context().close_stage_async()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    def _validate_and_compare_output(self, out_dir, golden_dir):
        """Validate folder contents and compare images/JSON with golden data."""
        test_dir_rp1 = os.path.join(out_dir, "rp1")
        test_dir_rp2 = os.path.join(out_dir, "rp2")
        golden_dir_rp1 = os.path.join(golden_dir, "rp1")
        golden_dir_rp2 = os.path.join(golden_dir, "rp2")

        # Check folder contents
        folder_contents_success_rp1 = validate_folder_contents(path=test_dir_rp1, expected_counts={"json": 1, "png": 2})
        self.assertTrue(folder_contents_success_rp1, f"Output directory contents validation failed for {test_dir_rp1}")
        folder_contents_success_rp2 = validate_folder_contents(path=test_dir_rp2, expected_counts={"json": 1, "png": 2})
        self.assertTrue(folder_contents_success_rp2, f"Output directory contents validation failed for {test_dir_rp2}")

        # Compare images for rp1
        result_rp1 = compare_images_in_directories(
            golden_dir=golden_dir_rp1,
            test_dir=test_dir_rp1,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=True,
            print_per_file_results=True,
        )
        self.assertTrue(result_rp1["all_passed"], f"Image comparison failed for rp1: {result_rp1}")

        # Compare images for rp2
        result_rp2 = compare_images_in_directories(
            golden_dir=golden_dir_rp2,
            test_dir=test_dir_rp2,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=True,
            print_per_file_results=True,
        )
        self.assertTrue(result_rp2["all_passed"], f"Image comparison failed for rp2: {result_rp2}")

        # Compare JSON for rp1
        golden_rp1_json = os.path.join(golden_dir_rp1, "000000.json")
        test_rp1_json = os.path.join(test_dir_rp1, "000000.json")
        with open(golden_rp1_json, "r") as f:
            golden_rp1_data = json.load(f)
        with open(test_rp1_json, "r") as f:
            test_rp1_data = json.load(f)
        error = compare_nested_structures_with_tolerance(test_rp1_data, golden_rp1_data, rtol=1e-5, atol=1e-5)
        self.assertIsNone(error, f"'/rp1' comparison failed:\n{error}")

        # Compare JSON for rp2
        golden_rp2_json = os.path.join(golden_dir_rp2, "000000.json")
        test_rp2_json = os.path.join(test_dir_rp2, "000000.json")
        with open(golden_rp2_json, "r") as f:
            golden_rp2_data = json.load(f)
        with open(test_rp2_json, "r") as f:
            test_rp2_data = json.load(f)
        error = compare_nested_structures_with_tolerance(test_rp2_data, golden_rp2_data, rtol=1e-5, atol=1e-5)
        self.assertIsNone(error, f"'/rp2' comparison failed:\n{error}")

    async def _setup_stage(self):
        """Setup stage with world xform, dome light, and test cubes."""
        await omni.usd.get_context().new_stage_async()
        rep.functional.create.xform(name="World")
        rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
        rep.functional.create.cube(
            position=(0, 0, 0),
            rotation=(0, 0, 0),
            scale=(1, 1, 1),
            name="Cube1",
            parent="/World",
            semantics=[("class", "cube")],
        )
        rep.functional.create.cube(
            position=(-1, -1, 0),
            rotation=(0, 0, 45),
            scale=(1, 1, 1),
            name="Cube2",
            parent="/World",
            semantics=[("class", "cube_rotated")],
        )
        rep.functional.create.cube(
            position=(1, 1, 0),
            rotation=(75, 65, 0),
            scale=(1.5, 0.5, 1),
            name="Cube3",
            parent="/World",
            semantics=[("class", "cube_scaled")],
        )

    async def test_pose_writer_with_pinhole_camera(self):
        """Test PoseWriter with pinhole camera projection."""
        await self._setup_stage()

        width, height = 512, 512

        cam1 = rep.functional.create.camera(
            position=(0, 0, 10), look_at=(0, 0, 0), name="PinholeCamera1", parent="/World"
        )
        cam2 = rep.functional.create.camera(
            position=(-4, -4, 8), look_at=(0, 0, 0), name="PinholeCamera2", parent="/World"
        )

        rp1 = rep.create.render_product(cam1, (width, height), name="rp1")
        rp2 = rep.create.render_product(cam2, (width, height), name="rp2")
        render_products = [rp1, rp2]

        out_dir = tempfile.mkdtemp(prefix="test_pose_writer_pinhole_")
        print(f"Output directory: {out_dir}")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=out_dir)
        writer = rep.writers.get("PoseWriter")
        writer.initialize(backend=backend, use_subfolders=True, write_debug_images=True)
        writer.attach(render_products)

        await rep.orchestrator.step_async(rt_subframes=16)
        await rep.orchestrator.wait_until_complete_async()

        writer.detach()
        for rp in render_products:
            rp.destroy()

        golden_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "golden", "_out_test_pose_writer_pinhole"
        )
        self._validate_and_compare_output(out_dir, golden_dir)

    async def test_pose_writer_with_fisheye_camera(self):
        """Test PoseWriter with fisheye polynomial camera projection."""
        await self._setup_stage()

        width, height = 512, 512
        camera_kwargs = {
            "parent": "/World",
            "projection_type": "fisheyePolynomial",
            "focal_length": 24.0,
            "horizontal_aperture": 20.955,
            "clipping_range": (0.1, 100.0),
            "fisheye_nominal_width": float(width),
            "fisheye_nominal_height": float(height),
            "fisheye_optical_centre_x": width / 2.0,
            "fisheye_optical_centre_y": height / 2.0,
            "fisheye_max_fov": 180.0,
            "fisheye_polynomial_a": 0.0,
            "fisheye_polynomial_b": 0.00245,
        }

        cam1 = rep.functional.create.camera(
            position=(0, 0, 10), look_at=(0, 0, 0), name="FisheyeCamera1", **camera_kwargs
        )
        cam2 = rep.functional.create.camera(
            position=(-4, -4, 8), look_at=(0, 0, 0), name="FisheyeCamera2", **camera_kwargs
        )

        rp1 = rep.create.render_product(cam1, (width, height), name="rp1")
        rp2 = rep.create.render_product(cam2, (width, height), name="rp2")
        render_products = [rp1, rp2]

        out_dir = tempfile.mkdtemp(prefix="test_pose_writer_fisheyePolynomial_")
        print(f"Output directory: {out_dir}")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=out_dir)
        writer = rep.writers.get("PoseWriter")
        writer.initialize(backend=backend, use_subfolders=True, write_debug_images=True)
        writer.attach(render_products)

        await rep.orchestrator.step_async(rt_subframes=16)
        await rep.orchestrator.wait_until_complete_async()

        writer.detach()
        for rp in render_products:
            rp.destroy()

        golden_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "golden", "_out_test_pose_writer_fisheyePolynomial"
        )
        self._validate_and_compare_output(out_dir, golden_dir)

    async def test_pose_writer_with_pinhole_opencv_camera(self):
        """Test PoseWriter with pinholeOpenCV camera projection."""
        await self._setup_stage()

        width, height = 512, 512
        camera_kwargs = {
            "parent": "/World",
            "projection_type": "pinholeOpenCV",
            "focal_length": 24.0,
            "horizontal_aperture": 20.955,
            "clipping_range": (0.1, 100.0),
            "fisheye_nominal_width": float(width),
            "fisheye_nominal_height": float(height),
            "fisheye_optical_centre_x": width / 2.0,
            "fisheye_optical_centre_y": height / 2.0,
            "openCV_focal_x": 400.0,
            "openCV_focal_y": 400.0,
        }

        cam1 = rep.functional.create.camera(
            position=(0, 0, 10), look_at=(0, 0, 0), name="PinholeOpenCVCamera1", **camera_kwargs
        )
        cam2 = rep.functional.create.camera(
            position=(-4, -4, 8), look_at=(0, 0, 0), name="PinholeOpenCVCamera2", **camera_kwargs
        )

        rp1 = rep.create.render_product(cam1, (width, height), name="rp1")
        rp2 = rep.create.render_product(cam2, (width, height), name="rp2")
        render_products = [rp1, rp2]

        out_dir = tempfile.mkdtemp(prefix="test_pose_writer_pinholeOpenCV_")
        print(f"Output directory: {out_dir}")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=out_dir)
        writer = rep.writers.get("PoseWriter")
        writer.initialize(backend=backend, use_subfolders=True, write_debug_images=True)
        writer.attach(render_products)

        await rep.orchestrator.step_async(rt_subframes=16)
        await rep.orchestrator.wait_until_complete_async()

        writer.detach()
        for rp in render_products:
            rp.destroy()

        golden_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "golden", "_out_test_pose_writer_pinholeOpenCV"
        )
        self._validate_and_compare_output(out_dir, golden_dir)

    async def test_pose_writer_with_generalized_projection_camera(self):
        """Test PoseWriter with generalizedProjection camera projection."""
        await self._setup_stage()

        width, height = 512, 512
        camera_kwargs = {
            "parent": "/World",
            "projection_type": "generalizedProjection",
            "focal_length": 24.0,
            "horizontal_aperture": 20.955,
            "clipping_range": (0.1, 100.0),
        }

        cam1 = rep.functional.create.camera(
            position=(0, 0, 10), look_at=(0, 0, 0), name="GeneralizedProjectionCamera1", **camera_kwargs
        )
        cam2 = rep.functional.create.camera(
            position=(-4, -4, 8), look_at=(0, 0, 0), name="GeneralizedProjectionCamera2", **camera_kwargs
        )

        rp1 = rep.create.render_product(cam1, (width, height), name="rp1")
        rp2 = rep.create.render_product(cam2, (width, height), name="rp2")
        render_products = [rp1, rp2]

        out_dir = tempfile.mkdtemp(prefix="test_pose_writer_generalizedProjection_")
        print(f"Output directory: {out_dir}")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=out_dir)
        writer = rep.writers.get("PoseWriter")
        writer.initialize(backend=backend, use_subfolders=True, write_debug_images=True)
        writer.attach(render_products)

        await rep.orchestrator.step_async(rt_subframes=16)
        await rep.orchestrator.wait_until_complete_async()

        writer.detach()
        for rp in render_products:
            rp.destroy()

        golden_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "golden", "_out_test_pose_writer_generalizedProjection"
        )
        self._validate_and_compare_output(out_dir, golden_dir)
