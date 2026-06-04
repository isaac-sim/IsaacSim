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

"""Tests for the TiledCameraSensor class."""

import os
from collections.abc import Callable
from typing import Any, Literal

import cv2
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.semantics as semantics_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, GroundPlane, Sphere, SphereLight
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.experimental.prims.tests.common import check_allclose, cprint
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import TiledCameraSensor, draw_annotator_data_to_image

MAX_NUM_PRIMS = 5
RESOLUTION = (256, 320)  # following OpenCV/NumPy convention (height, width)
TILED_RESOLUTION = (512, 960)  # following OpenCV/NumPy convention (height, width)
EXPECTED_ANNOTATOR_SPEC = {
    "distance_to_camera": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "distance_to_image_plane": {"channels": 1, "dtype": wp.float32, "type": wp.array},
    "instance_id_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    "instance_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
    "motion_vectors": {"channels": 2, "dtype": wp.float32, "type": wp.array},
    "normals": {"channels": 3, "dtype": wp.float32, "type": wp.array},
    "rgb": {"channels": 3, "dtype": wp.uint8, "type": wp.array},
    "rgba": {"channels": 4, "dtype": wp.uint8, "type": wp.array},
    "semantic_segmentation": {"channels": 1, "dtype": wp.uint32, "type": wp.array},
}


def parametrize(
    *,
    instances: list[Literal["one", "many"]] = None,
    operations: list[Literal["wrap", "create"]] = None,
    prim_class: type,
    prim_class_kwargs: dict = None,
    populate_stage_func: Callable[[int, Literal["wrap", "create"]], None],
    populate_stage_func_kwargs: dict = None,
    max_num_prims: int = MAX_NUM_PRIMS,
) -> Callable:
    """Parametrize a test method with instance and operation combinations.

    Args:
        instances: List of instance types to test (e.g., ``["one", "many"]``).
        operations: List of operations to test (e.g., ``["wrap", "create"]``).
        prim_class: The prim class to instantiate.
        prim_class_kwargs: Additional keyword arguments for the prim class constructor.
        populate_stage_func: Function to populate the USD stage before each test.
        populate_stage_func_kwargs: Additional keyword arguments for the populate stage function.
        max_num_prims: Maximum number of prims to create.

    Returns:
        A decorator that wraps the test method with parametrized execution.
    """
    if populate_stage_func_kwargs is None:
        populate_stage_func_kwargs = {}
    if prim_class_kwargs is None:
        prim_class_kwargs = {}
    if operations is None:
        operations = ["wrap", "create"]
    if instances is None:
        instances = ["one", "many"]

    def decorator(func: Callable) -> Callable:
        async def wrapper(self: Any) -> None:
            for instance in instances:
                for operation in operations:
                    assert instance in ["one", "many"], f"Invalid instance: {instance}"
                    assert operation in ["wrap", "create"], f"Invalid operation: {operation}"
                    cprint(f"  |-- instance: {instance}, operation: {operation}")
                    # populate stage
                    await populate_stage_func(max_num_prims, operation, **populate_stage_func_kwargs)
                    # parametrize test
                    if operation == "wrap":
                        paths = "/World/A_0" if instance == "one" else "/World/A_.*"
                    elif operation == "create":
                        paths = "/World/A_0" if instance == "one" else [f"/World/A_{i}" for i in range(max_num_prims)]
                    prim = prim_class(paths, **prim_class_kwargs)
                    num_prims = 1 if instance == "one" else max_num_prims
                    # run test function
                    app_utils.play(commit=True)
                    await app_utils.update_app_async()
                    try:
                        await func(self, prim=prim, num_prims=num_prims, operation=operation)
                    finally:
                        app_utils.stop(commit=True)
                        await app_utils.update_app_async()
                        del prim  # needed to destroy/release everything before the next test
                        await app_utils.update_app_async(steps=3)

        return wrapper

    return decorator


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Populate the USD stage with test prims for tiled camera sensor tests.

    Args:
        max_num_prims: Maximum number of camera prims to create.
        operation: The operation type, either ``"wrap"`` or ``"create"``.
        **kwargs: Additional keyword arguments (unused).
    """
    # create new stage
    await stage_utils.create_new_stage_async()
    # wait for the viewport to be ready
    await ViewportManager.wait_for_viewport_async()
    # define a light prim
    sphere_light = SphereLight("/World/SphereLight", positions=[1.0, -1.0, 1.0])
    sphere_light.set_intensities(intensities=100000)
    # define a ground plane prim
    ground_plane = GroundPlane("/World/GroundPlane")
    # define some shapes
    cone = Cone("/World/Cone", radii=0.5, heights=1.0, positions=[0.0, 0.0, 0.0], colors=[1.0, 0.0, 0.0])
    cube = Cube("/World/Cube", sizes=0.5, positions=[-0.5, 0.25, 0.25], colors=[0.0, 1.0, 0.0])
    sphere = Sphere("/World/Sphere", radii=0.25, positions=[0.25, -0.35, 0.25], colors=[0.0, 0.0, 1.0])
    # - add collision
    GeomPrim(cone.paths, apply_collision_apis=True)
    GeomPrim(cube.paths, apply_collision_apis=True)
    GeomPrim(sphere.paths, apply_collision_apis=True)
    # - add labels for semantic segmentation
    semantics_utils.add_labels(cone.paths[0], labels="cone", taxonomy="shape")
    semantics_utils.add_labels(cube.paths[0], labels="cube", taxonomy="shape")
    semantics_utils.add_labels(sphere.paths[0], labels="sphere", taxonomy="shape")
    semantics_utils.add_labels(sphere.paths[0], labels=["label_a", "label_b"])
    # define camera prims
    if operation == "wrap":
        for i in range(max_num_prims):
            stage_utils.define_prim(f"/World/A_{i}", "Camera")


class TestTiledCameraSensor(omni.kit.test.AsyncTestCase):
    """Test case class for the TiledCameraSensor class."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        self.maxDiff = None  # show all diffs
        self.frame = None  # frame used as a background for rendering data
        self.save_images = False  # whether to save images

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_len(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the number of cameras in the tiled sensor matches the expected count."""
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that tiled camera sensor data is correctly retrieved for all annotators."""
        for i, path in enumerate(prim.camera.paths):
            ViewportManager.set_camera_view(path, eye=[3.0, 1.25, 0.25 + i], target=[0.0, 0.0, 0.25])
        # test cases
        for annotator in sorted(EXPECTED_ANNOTATOR_SPEC.keys()):
            cprint(f"  |    |-- annotator: {annotator} (number of cameras: {num_prims})")
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
            # - shape
            shape = (num_prims, *RESOLUTION, spec["channels"])
            self.assertEqual(data.shape, shape, f"'{annotator}' annotator shape {data.shape} != {shape}")
            # - type
            self.assertIsInstance(data, spec["type"], f"'{annotator}' annotator type {type(data)} != {spec['type']}")
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
                self.assertEqual(sorted(info.keys()), ["idToLabels"], msg=f"Annotator info mismatch for '{annotator}'")
                self.assertEqual(
                    sorted(info["idToLabels"].keys()),
                    ["0", "1", "2", "3", "4"],
                    msg=f"Annotator info mismatch for '{annotator}'",
                )
                for value in info["idToLabels"].values():
                    self.assertIn(value, expected_values, msg=f"Annotator info mismatch for '{annotator}'")
            else:
                self.assertDictEqual({}, info, msg=f"Annotator info mismatch for '{annotator}'")

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_tiled_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that tiled camera sensor data is correctly retrieved in tiled format for all annotators."""
        for i, path in enumerate(prim.camera.paths):
            ViewportManager.set_camera_view(path, eye=[3.0, 1.25, 0.25 + i], target=[0.0, 0.0, 0.25])
        # get frame
        self.frame = None
        for i in range(10):
            await app_utils.update_app_async()
            if prim.get_data("rgb", tiled=True)[0] is not None:
                break
        await app_utils.update_app_async(steps=3)
        self.frame = prim.get_data("rgb", tiled=True)[0]  # get next available frames to avoid rendering artifacts
        # test cases
        for annotator in sorted(EXPECTED_ANNOTATOR_SPEC.keys()):
            cprint(f"  |    |-- annotator: {annotator} (number of cameras: {num_prims})")
            spec = EXPECTED_ANNOTATOR_SPEC[annotator]
            data, info = None, {}
            for i in range(10):
                await app_utils.update_app_async()
                data, info = prim.get_data(annotator, tiled=True)
                if data is not None:
                    break
            if data is None:
                raise RuntimeError(f"No data available from '{annotator}' annotator after {i + 1} steps")
            else:
                cprint(f"  |    |    |-- data available after {i + 1} steps")

            # check data
            # - shape
            shape = (*(RESOLUTION if num_prims == 1 else TILED_RESOLUTION), spec["channels"])
            self.assertEqual(data.shape, shape, f"'{annotator}' annotator shape {data.shape} != {shape}")
            # - type
            self.assertIsInstance(data, spec["type"], f"'{annotator}' annotator type {type(data)} != {spec['type']}")
            # - dtype
            dtype = spec["dtype"]
            self.assertEqual(data.dtype, dtype, f"'{annotator}' annotator dtype {data.dtype} != {dtype}")
            # - out
            out = wp.empty(shape, dtype=dtype, device=data.device)
            prim.get_data(annotator, tiled=True, out=out)
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
                    for id, label in idToLabels.items():
                        if label == expected_label:
                            break
                    self.assertIsNotNone(id, f"Label '{expected_label}' not found in idToLabels")
                    self.assertEqual(idToSemantics[id], expected_semantics)
                cprint(f"  |    |    |-- {info}")
            elif annotator == "semantic_segmentation":
                expected_values = [
                    {"class": "BACKGROUND"},
                    {"class": "UNLABELLED"},
                    {"shape": "cube"},
                    {"shape": "sphere", "class": "label_a,label_b"},
                    {"shape": "cone"},
                ]
                cprint(f"  |    |    |-- {info}")
                self.assertEqual(sorted(info.keys()), ["idToLabels"], msg=f"Annotator info mismatch for '{annotator}'")
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
            image = draw_annotator_data_to_image(annotator=annotator, data=data, info=info, frame=self.frame)
            if self.save_images:
                filedir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
                filename = f"tiled_camera_sensor_{operation}_{num_prims}_{annotator}.png"
                filepath = os.path.join(filedir, filename)
                os.makedirs(filedir, exist_ok=True)
                print(f"Saving image to {filepath}")
                cv2.imwrite(filepath, image)
