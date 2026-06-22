# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the SingleViewDepthCameraSensor class."""

import os
from typing import Any

import cv2
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims.tests.common import check_allclose, cprint, draw_sample
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import SingleViewDepthCameraSensor, draw_annotator_data_to_image

from .test_camera_sensor import parametrize, populate_stage

RESOLUTION = (256, 320)  # following OpenCV/NumPy convention (height, width)
EXPECTED_ANNOTATOR_SPEC = {
    "bounding_box_2d_loose": {"type": np.ndarray},
    "bounding_box_2d_tight": {"type": np.ndarray},
    "bounding_box_3d": {"type": np.ndarray},
    "distance_to_camera": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "distance_to_image_plane": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "instance_id_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    "instance_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    "motion_vectors": {"channels": 2, "dtype": wp.float32, "type": wp.array},
    "normals": {"channels": 3, "dtype": wp.float32, "type": wp.array},
    "pointcloud": {"type": wp.array},
    "semantic_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    "semantic_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    # single view depth sensor annotators
    "depth_sensor_distance": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "depth_sensor_imager": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "depth_sensor_point_cloud_color": {"channels": 4, "dtype": wp.uint8, "type": wp.array},
    "depth_sensor_point_cloud_position": {"channels": 3, "dtype": wp.float32, "type": wp.array},
}


class TestSingleViewDepthCameraSensor(omni.kit.test.AsyncTestCase):
    """Test case class for the SingleViewDepthCameraSensor class."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        self.maxDiff = None  # show all diffs
        self.save_images = False  # whether to save images

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_baseline(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor baseline can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_baseline(np.array(v0).item())
            output = prim.get_sensor_baseline()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_disparity_confidence(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor disparity confidence can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_disparity_confidence(np.array(v0).item())
            output = prim.get_sensor_disparity_confidence()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_maximum_disparity(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor maximum disparity can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_maximum_disparity(np.array(v0).item())
            output = prim.get_sensor_maximum_disparity()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_enabled_post_processing(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the enabled post processing flag can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for item in [False, True]:
            prim.set_enabled_post_processing(item)
            output = prim.get_enabled_post_processing()
            self.assertEqual(item, output, msg=f"Given: {item}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_focal_length(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor focal length can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_focal_length(np.array(v0).item())
            output = prim.get_sensor_focal_length()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_distance_cutoffs(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor distance cutoffs can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for (v0, expected_v0), (v1, expected_v1) in zip(
            draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]),
            draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]),
        ):
            prim.set_sensor_distance_cutoffs(np.array(v0).item(), np.array(v1).item())
            output = prim.get_sensor_distance_cutoffs()
            self.assertAlmostEqual(expected_v0.item(), output[0], msg=f"Given: {v0}")
            self.assertAlmostEqual(expected_v1.item(), output[1], msg=f"Given: {v1}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_disparity_noise_downscale(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor disparity noise downscale can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_disparity_noise_downscale(np.array(v0).item())
            output = prim.get_sensor_disparity_noise_downscale()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_noise_parameters(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor noise parameters can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for (v0, expected_v0), (v1, expected_v1) in zip(
            draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]),
            draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]),
        ):
            prim.set_sensor_noise_parameters(np.array(v0).item(), np.array(v1).item())
            output = prim.get_sensor_noise_parameters()
            self.assertAlmostEqual(expected_v0.item(), output[0], msg=f"Given: {v0}")
            self.assertAlmostEqual(expected_v1.item(), output[1], msg=f"Given: {v1}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_enabled_outlier_removal(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the enabled outlier removal flag can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.bool, types=[list]):
            prim.set_enabled_outlier_removal(np.array(v0).item())
            output = prim.get_enabled_outlier_removal()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_output_mode(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor output mode can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.int32, low=0, high=8, types=[list]):
            prim.set_sensor_output_mode(np.array(v0).item())
            output = prim.get_sensor_output_mode()
            self.assertEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": []},
        populate_stage_func=populate_stage,
    )
    async def test_sensor_size(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the sensor size can be set and retrieved correctly.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for v0, expected_v0 in draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[list]):
            prim.set_sensor_size(np.array(v0).item())
            output = prim.get_sensor_size()
            self.assertAlmostEqual(expected_v0.item(), output, msg=f"Given: {v0}")

    # --------------------------------------------------------------------

    @parametrize(
        prim_class=SingleViewDepthCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that sensor data is correctly retrieved for all annotators.

        Args:
            prim: Single-view depth camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        prim.camera.set_focal_lengths(1.814756)
        prim.camera.set_focus_distances(400.0)
        prim.set_sensor_baseline(55)
        prim.set_sensor_focal_length(891.0)
        prim.set_sensor_size(1280.0)
        prim.set_sensor_maximum_disparity(110.0)
        prim.set_sensor_disparity_confidence(0.99)
        prim.set_sensor_noise_parameters(0.5, 1.0)
        prim.set_sensor_disparity_noise_downscale(1.0)
        prim.set_sensor_distance_cutoffs(0.5, 9999.9)
        prim.set_enabled_post_processing(True)
        for path in prim.camera.paths:
            ViewportManager.set_camera_view(path, eye=[1.0, 0.5, 1.0], target=[0.0, 0.0, 0.25])
        # test cases
        for annotator in sorted(EXPECTED_ANNOTATOR_SPEC.keys()):
            cprint(f"  |    |-- annotator: {annotator}")
            spec = EXPECTED_ANNOTATOR_SPEC[annotator]
            data, info = None, {}
            for i in range(10):
                await app_utils.update_app_async()
                data, info = prim.get_data(annotator)
                if data is not None:
                    break
            if data is None:
                raise RuntimeError(f"No data available from '{annotator}' annotator after {i + 1} steps")
            else:
                cprint(f"  |    |    |-- data available after {i + 1} steps")

            # check data
            if annotator in [
                "bounding_box_2d_tight",
                "bounding_box_2d_loose",
                "bounding_box_3d",
                "pointcloud",
            ]:
                # - type
                self.assertIsInstance(
                    data, spec["type"], f"'{annotator}' annotator type {type(data)} != {spec['type']}"
                )
            else:
                # - shape
                shape = (*RESOLUTION, spec["channels"])
                self.assertEqual(data.shape, shape, f"'{annotator}' annotator shape {data.shape} != {shape}")
                # - type
                self.assertIsInstance(
                    data, spec["type"], f"'{annotator}' annotator type {type(data)} != {spec['type']}"
                )
                # - dtype
                dtype = spec["dtype"]
                self.assertEqual(data.dtype, dtype, f"'{annotator}' annotator dtype {data.dtype} != {dtype}")
                # - out
                out = wp.empty(shape, dtype=dtype, device=data.device)
                prim.get_data(annotator, out=out)
                check_allclose(data, out)
                # - info
                if annotator == "instance_id_segmentation":
                    cprint(f"  |    |    |-- {info}")
                    self.assertIn("idToLabels", info)
                    id_to_labels = info["idToLabels"]
                    self.assertTrue(
                        set(id_to_labels.values()).issubset(
                            {"INVALID", "/World/Cone", "/World/Cube", "/World/GroundPlane/geom", "/World/Sphere"}
                        ),
                        msg=f"Annotator info mismatch for '{annotator}' ({set(id_to_labels.values())})",
                    )
                elif annotator == "instance_segmentation":
                    cprint(f"  |    |    |-- {info}")
                    self.assertIn("idToLabels", info)
                    self.assertIn("idToSemantics", info)
                    self.assertEqual(set(info["idToLabels"].keys()), {0, 1, 2, 3, 4})
                    self.assertEqual(set(info["idToSemantics"].keys()), {0, 1, 2, 3, 4})
                    idToLabels = info["idToLabels"]
                    idToSemantics = info["idToSemantics"]
                    expected_label_to_semantics = {
                        "BACKGROUND": {"class": "BACKGROUND"},
                        "UNLABELLED": {"class": "UNLABELLED"},
                        "/World/Cone": {"shape": "cone"},
                        "/World/Cube": {"shape": "cube"},
                        "/World/Sphere": {"shape": "sphere", "class": "label_a,label_b"},
                    }
                    for expected_label, expected_semantics in expected_label_to_semantics.items():
                        id = None
                        for key, label in idToLabels.items():
                            if label == expected_label:
                                id = key
                                break
                        self.assertIsNotNone(id, f"Label '{expected_label}' not found in idToLabels")
                        self.assertEqual(idToSemantics[id], expected_semantics)
                elif annotator == "semantic_segmentation":
                    expected_values = [
                        {"class": "BACKGROUND"},
                        {"class": "UNLABELLED"},
                        {"shape": "cube"},
                        {"shape": "sphere", "class": "label_a,label_b"},
                        {"shape": "cone"},
                    ]
                    cprint(f"  |    |    |-- {info}")
                    self.assertEqual(
                        sorted(info.keys()), ["idToLabels"], msg=f"Annotator info mismatch for '{annotator}'"
                    )
                    self.assertEqual(
                        sorted(info["idToLabels"].keys()),
                        ["0", "1", "2", "3", "4"],
                        msg=f"Annotator info mismatch for '{annotator}'",
                    )
                    for value in info["idToLabels"].values():
                        self.assertIn(value, expected_values, msg=f"Annotator info mismatch for '{annotator}'")
                else:
                    self.assertDictEqual({}, info, msg=f"Annotator info mismatch for '{annotator}'")

            # render data and save it in the sub-folder `data` in the same folder as the test file
            image = draw_annotator_data_to_image(annotator=annotator, data=data, info=info)
            if self.save_images:
                filedir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
                filename = f"single_view_depth_camera_sensor_{operation}_{num_prims}_{annotator}.png"
                filepath = os.path.join(filedir, filename)
                os.makedirs(filedir, exist_ok=True)
                print(f"Saving image to {filepath}")
                cv2.imwrite(filepath, image)
