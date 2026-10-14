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

"""Tests for the CameraSensor class."""

import os
from collections.abc import Callable
from typing import Any, Literal

import cv2
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.semantics as semantics_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, GroundPlane, Sphere, SphereLight
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.experimental.prims.tests.common import check_allclose, cprint
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera, draw_annotator_data_to_image

from .common import FakeAnnotator, normalize_semantics

RESOLUTION = (320, 400)  # following OpenCV/NumPy convention (height, width)
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
    max_num_prims: int = 1,
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
        instances = ["one"]

    def decorator(func: Callable) -> Callable:
        async def wrapper(self: Any) -> None:
            for instance in instances:
                for operation in operations:
                    assert instance in ["one"], f"Invalid instance: {instance}. Only one instance is supported"
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
    """Populate the USD stage with test prims for camera sensor tests.

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
    GroundPlane("/World/GroundPlane")
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
            prim = stage_utils.define_prim(f"/World/A_{i}", "Camera")
            prim.ApplyAPI("OmniSensorAPI")


class TestCameraSensor(omni.kit.test.AsyncTestCase):
    """Test case class for the CameraSensor class."""

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

    async def test_image_annotator_buffer_mismatch_is_reported(self) -> None:
        """Report mismatched image annotator buffers before reshaping."""
        sensor = CameraSensor.__new__(CameraSensor)
        sensor._resolution = (2, 3)
        sensor._hydra_texture = type("HydraTexture", (), {"path": "/Render/Fake"})()
        sensor._annotators_spec = {"rgb": {"name": "rgb", "channels": 4, "output_channels": 3, "dtype": wp.uint8}}
        sensor._annotators = {"rgb": FakeAnnotator(np.zeros(23, dtype=np.uint8))}

        try:
            with self.assertRaisesRegex(RuntimeError, "returned 23 elements, expected 24"):
                sensor.get_data("rgb")
        finally:
            sensor._writers = {}
            sensor._annotators = {}
            sensor._hydra_texture = None

    async def test_get_data_returns_none_during_warm_up(self) -> None:
        """Return ``None`` and preserve info while the annotator has no payload."""
        sensor = CameraSensor.__new__(CameraSensor)
        sensor._resolution = (2, 3)
        sensor._hydra_texture = None
        sensor._data_ready = False
        sensor._annotators_spec = {"rgb": {"name": "rgb", "channels": 4, "output_channels": 3, "dtype": wp.uint8}}
        sensor._annotators = {"rgb": FakeAnnotator({"data": None, "info": {"frameId": 7}})}

        try:
            self.assertEqual(sensor.get_data("rgb"), (None, {"frameId": 7}))
            # `has_data()` is gated on the render product, so assert the latch a missing payload drives
            self.assertFalse(sensor._data_ready)
        finally:
            sensor._writers = {}
            sensor._annotators = {}
            sensor._hydra_texture = None

    async def test_rejects_multiple_prims(self) -> None:
        """Reject a path that resolves to more than one camera prim."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        RtxCamera("/World/cam_0")
        RtxCamera("/World/cam_1")
        with self.assertRaisesRegex(ValueError, "Only one RtxCamera prim is supported"):
            CameraSensor("/World/cam_.*", resolution=RESOLUTION, annotators=["rgb"])

    async def test_rejects_multiple_prim_rtx_camera(self) -> None:
        """Reject a pre-built RtxCamera that wraps more than one prim."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        camera = RtxCamera(["/World/cam_0", "/World/cam_1"])
        with self.assertRaisesRegex(ValueError, "Only one RtxCamera prim is supported"):
            CameraSensor(camera, resolution=RESOLUTION, annotators=["rgb"])

    async def test_rejects_multiple_prims_without_creating_them(self) -> None:
        """Reject multiple non-existing paths before any Camera prim is created."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        paths = ["/World/new_cam_0", "/World/new_cam_1"]
        with self.assertRaisesRegex(ValueError, "Only one RtxCamera prim is supported"):
            CameraSensor(paths, resolution=RESOLUTION, annotators=["rgb"])
        stage = stage_utils.get_current_stage(backend="usd")
        for path in paths:
            self.assertFalse(stage.GetPrimAtPath(path).IsValid())

    async def test_rejects_multiple_prims_without_applying_schemas(self) -> None:
        """Reject multiple existing paths before the sensor schema is applied to them."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        paths = [f"/World/plain_cam_{i}" for i in range(2)]
        for path in paths:
            stage_utils.define_prim(path, "Camera")
        with self.assertRaisesRegex(ValueError, "Only one RtxCamera prim is supported"):
            CameraSensor("/World/plain_cam_.*", resolution=RESOLUTION, annotators=["rgb"])
        stage = stage_utils.get_current_stage(backend="usd")
        for path in paths:
            self.assertFalse(stage.GetPrimAtPath(path).HasAPI("OmniSensorAPI"))

    async def test_has_data(self) -> None:
        """Report completed render data before a frame is fetched."""
        await populate_stage(1, "wrap")
        sensor = CameraSensor("/World/A_0", resolution=RESOLUTION, annotators=["rgb"])
        try:
            self.assertFalse(sensor.has_data())
            app_utils.play(commit=True)
            for _ in range(20):
                await app_utils.update_app_async()
                if sensor.has_data():
                    break
            self.assertTrue(sensor.has_data())
            # Render completion can precede the `rgb` annotator's first frame.
            data, _ = sensor.get_data("rgb")
            for _ in range(20):
                if data is not None:
                    break
                await app_utils.update_app_async()
                data, _ = sensor.get_data("rgb")
            self.assertIsNotNone(data)
            # the state is scoped to the attached annotators and to owning the render product
            sensor.detach_annotators("rgb")
            self.assertFalse(sensor.has_data())
            sensor._invalidate_sensor()
            self.assertFalse(sensor.has_data())
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)

    @parametrize(
        prim_class=CameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that camera sensor data is correctly retrieved for all annotators.

        Args:
            prim: Camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for path in prim.camera.paths:
            ViewportManager.set_camera_view(path, eye=[3.0, 1.25, 1.0], target=[0.0, 0.0, 0.25])
        # get frame
        self.frame = None
        for i in range(10):
            await app_utils.update_app_async()
            if prim.get_data("rgb")[0] is not None:
                break
        await app_utils.update_app_async(steps=3)
        self.frame = prim.get_data("rgb")[0]  # get next available frames to avoid rendering artifacts
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
                        self.assertEqual(
                            normalize_semantics(idToSemantics[id]), normalize_semantics(expected_semantics)
                        )
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
                    normalized_expected_values = [normalize_semantics(value) for value in expected_values]
                    for value in info["idToLabels"].values():
                        self.assertIn(
                            normalize_semantics(value),
                            normalized_expected_values,
                            msg=f"Annotator info mismatch for '{annotator}'",
                        )
                else:
                    self.assertDictEqual({}, info, msg=f"Annotator info mismatch for '{annotator}'")

            # render data and save it in the sub-folder `data` in the same folder as the test file
            image = draw_annotator_data_to_image(annotator=annotator, data=data, info=info, frame=self.frame)
            if self.save_images:
                filedir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
                filename = f"camera_sensor_{operation}_{num_prims}_{annotator}.png"
                filepath = os.path.join(filedir, filename)
                os.makedirs(filedir, exist_ok=True)
                print(f"Saving image to {filepath}")
                cv2.imwrite(filepath, image)

    @parametrize(
        prim_class=CameraSensor,
        prim_class_kwargs={
            "resolution": RESOLUTION,
            "annotators": ["rgb"],
            "render_vars": ["HdrColor"],
        },
        populate_stage_func=populate_stage,
    )
    async def test_render_vars(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that custom render_vars are accepted and the sensor still produces data.

        Args:
            prim: Camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        for path in prim.camera.paths:
            ViewportManager.set_camera_view(path, eye=[3.0, 1.25, 1.0], target=[0.0, 0.0, 0.25])
        for i in range(10):
            await app_utils.update_app_async()
            data, _ = prim.get_data("rgb")
            if data is not None:
                break
        self.assertIsNotNone(data, "No RGB data available after 10 steps with render_vars=['HdrColor']")
        self.assertEqual(data.shape, (*RESOLUTION, 3))

    async def test_multiple_camera_sensors_return_depth(self) -> None:
        """Two camera sensors with independent render products both return depth data."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        sphere_light = SphereLight("/World/SphereLight", positions=[1.0, -1.0, 1.0])
        sphere_light.set_intensities(intensities=100000)
        GroundPlane("/World/GroundPlane")
        Cube("/World/Cube", sizes=0.5, positions=[0.0, 0.0, 0.25], colors=[0.0, 1.0, 0.0])
        sensors = [
            CameraSensor("/World/Camera_0", resolution=RESOLUTION, annotators=["rgb", "distance_to_image_plane"]),
            CameraSensor("/World/Camera_1", resolution=RESOLUTION, annotators=["rgb", "distance_to_image_plane"]),
        ]
        camera_views = (
            ([3.0, 1.25, 1.0], [0.0, 0.0, 0.25]),
            ([-3.0, -1.25, 1.0], [0.0, 0.0, 0.25]),
        )
        for sensor, (eye, target) in zip(sensors, camera_views):
            ViewportManager.set_camera_view(sensor.camera.paths[0], eye=eye, target=target)

        app_utils.play(commit=True)
        await app_utils.update_app_async()
        try:
            ready = [False] * len(sensors)
            for _ in range(20):
                await app_utils.update_app_async()
                for i, sensor in enumerate(sensors):
                    data, _ = sensor.get_data("distance_to_image_plane")
                    if data is not None:
                        ready[i] = True
                        self.assertEqual(data.shape, (*RESOLUTION, 1))
                if all(ready):
                    break
            self.assertEqual(ready, [True, True], "No depth data available from every camera sensor")
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            sensors.clear()
            await app_utils.update_app_async(steps=3)

    async def _create_unlabelled_pointcloud_scene(self, **sensor_kwargs: Any) -> CameraSensor:
        """Create an unlabeled cube scene for pointcloud annotator tests.

        Args:
            **sensor_kwargs: Additional arguments for the camera sensor.

        Returns:
            Camera sensor configured for point-cloud output.
        """
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        sphere_light = SphereLight("/World/SphereLight", positions=[1.0, -1.0, 1.0])
        sphere_light.set_intensities(intensities=100000)
        Cube("/World/Cube", sizes=0.5, positions=[0.0, 0.0, 0.25], colors=[0.0, 1.0, 0.0])
        sensor = CameraSensor(
            "/World/Camera",
            resolution=(320, 320),
            annotators=["pointcloud"],
            **sensor_kwargs,
        )
        ViewportManager.set_camera_view(sensor.camera.paths[0], eye=[3.0, 1.25, 1.0], target=[0.0, 0.0, 0.25])
        app_utils.play(commit=True)
        await app_utils.update_app_async()
        return sensor

    async def test_pointcloud_includes_unlabelled_by_default(self) -> None:
        """Pointcloud annotator returns unlabeled geometry by default."""
        sensor = await self._create_unlabelled_pointcloud_scene()
        try:
            data = None
            for _ in range(20):
                await app_utils.update_app_async()
                data, _ = sensor.get_data("pointcloud")
                if data is not None:
                    break
            self.assertIsNotNone(data, "No pointcloud data available for unlabeled geometry")
            self.assertEqual(data.shape[1], 3)
            self.assertGreater(data.shape[0], 0)
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_pointcloud_init_params_can_exclude_unlabelled(self) -> None:
        """Pointcloud annotator init params can restore semantic-only filtering."""
        sensor = await self._create_unlabelled_pointcloud_scene(
            annotator_init_params={"pointcloud": {"includeUnlabelled": False}}
        )
        try:
            data = None
            for _ in range(20):
                await app_utils.update_app_async()
                data, _ = sensor.get_data("pointcloud")
                if data is not None:
                    break
            self.assertIsNone(data, "Pointcloud data should be filtered out for unlabeled geometry")
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_resolution_required_without_pre_authored_render_product(self) -> None:
        """Constructing a CameraSensor without a resolution and no pre-authored render product raises ValueError."""
        await stage_utils.create_new_stage_async()
        prim = stage_utils.define_prim("/World/Camera", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        with self.assertRaises(ValueError):
            CameraSensor("/World/Camera", annotators=[])
