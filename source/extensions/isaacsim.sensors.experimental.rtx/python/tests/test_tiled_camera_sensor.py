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
from unittest import mock

import cv2
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.semantics as semantics_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Camera, Cone, Cube, GroundPlane, Sphere, SphereLight
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.experimental.prims.tests.common import check_allclose, cprint
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    RtxCamera,
    SensorRuntime,
    TiledCameraSensor,
    draw_annotator_data_to_image,
)

from .common import FakeAnnotator, normalize_semantics

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

    async def test_rtx_camera_authored_prim_batching(self) -> None:
        """Batch RtxCamera-authored prims and preserve their sensor schema and tick rate."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        paths = [f"/World/RtxCamera_{i}" for i in range(2)]
        cameras = [RtxCamera(path, tick_rate=30.0) for path in paths]

        batched = RtxCamera("/World/RtxCamera_.*")
        self.assertEqual(list(batched.paths), paths)
        self.assertEqual(len(batched.camera), 2)
        for prim in batched.prims:
            self.assertTrue(prim.HasAPI("OmniSensorAPI"))
            self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 30.0)

        sensor = TiledCameraSensor("/World/RtxCamera_.*", resolution=RESOLUTION, annotators=["rgb"])
        self.assertEqual(list(sensor.authoring_object.paths), paths)
        self.assertIsInstance(sensor.authoring_object, RtxCamera)
        self.assertIs(sensor.camera, sensor.authoring_object.camera)
        for prim in sensor.authoring_object.prims:
            self.assertTrue(prim.HasAPI("OmniSensorAPI"))
            self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 30.0)

        del sensor
        del cameras
        del batched
        await app_utils.update_app_async(steps=3)

    async def test_construct_applies_tick_rate(self) -> None:
        """Apply a shared tick rate when constructing the tiled sensor from paths."""
        await populate_stage(2, "wrap")
        sensor = TiledCameraSensor("/World/A_.*", resolution=RESOLUTION, annotators=["rgb"], tick_rate=15.0)
        try:
            self.assertEqual(len(sensor), 2)
            for prim in sensor.authoring_object.prims:
                self.assertTrue(prim.HasAPI("OmniSensorAPI"))
                self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 15.0)
        finally:
            del sensor
            await app_utils.update_app_async(steps=3)

    async def _count_unique_tiles(self, sensor: TiledCameraSensor, cube: Cube, steps: int) -> tuple[int, list[int]]:
        """Render a moving scene and count how often each tile produced a new image.

        A moving cube makes every rendered frame visually distinct, so the number of unique
        tile contents is the number of times that camera actually rendered.

        Args:
            sensor: Tiled sensor to fetch batched frames from.
            cube: Cube translated on every step to keep consecutive renders distinct.
            steps: Number of app updates to run.

        Returns:
            Two-element tuple containing the number of fetched frames and the per-tile
            count of unique images.
        """
        app_utils.play(commit=True)
        fetched = 0
        unique = [set() for _ in range(len(sensor))]
        for i in range(steps):
            cube.set_world_poses(positions=[[3.0 + i * 0.02, 0.0, 0.5]])
            await app_utils.update_app_async()
            data, _ = sensor.get_data("rgb")
            if data is None:
                continue
            fetched += 1
            tiles = data.numpy()
            for tile_index, seen in enumerate(unique):
                seen.add(tiles[tile_index].tobytes())
        return fetched, [len(seen) for seen in unique]

    async def _setup_tick_rate_scene(self, tick_rates: list[float | None]) -> Cube:
        """Create a lit stage with a cube in front of one camera per requested tick rate.

        Args:
            tick_rates: Tick rate per camera, in Hz. ``None`` leaves the camera in
                autotrigger mode, so it renders on every frame.

        Returns:
            The cube to translate between renders.
        """
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        sphere_light = SphereLight("/World/SphereLight", positions=[0.0, 0.0, 5.0])
        sphere_light.set_intensities(intensities=100000)
        for i, tick_rate in enumerate(tick_rates):
            RtxCamera(
                f"/World/cam_{i}",
                tick_rate=tick_rate,
                positions=[[0.0, i * 1.0, 0.5]],
                orientations=[[0.5, 0.5, -0.5, -0.5]],  # look along +X, towards the cube
            )
        return Cube("/World/Cube", sizes=1.0, positions=[3.0, 0.0, 0.5], colors=[1.0, 0.0, 0.0])

    async def test_shared_tick_rate_throttles_the_batch(self) -> None:
        """Render a tiled batch less often than the app updates when all cameras share a tick rate."""
        steps = 120
        cube = await self._setup_tick_rate_scene([5.0, 5.0])
        sensor = TiledCameraSensor("/World/cam_.*", resolution=(120, 160), annotators=["rgb"])
        try:
            fetched, unique = await self._count_unique_tiles(sensor, cube, steps)
            cprint(f"shared tick rate: fetched={fetched} unique={unique}")
            self.assertGreater(fetched, steps // 2, "the sensor did not deliver frames")
            for tile_index, count in enumerate(unique):
                self.assertGreater(count, 1, f"tile {tile_index} never produced a distinct frame")
                # at 5 Hz the batch must render far less often than the app updates
                self.assertLess(
                    count,
                    fetched * 0.5,
                    f"tickRate not honored: tile {tile_index} rendered {count} distinct frames "
                    f"out of {fetched} fetched frames",
                )
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)

    async def _measure_mixed_tick_rates(self, tick_rates: list[float], steps: int) -> tuple[int, list[int]]:
        """Build a mixed-tick-rate batch, assert it warns, and measure how often each tile rendered.

        Args:
            tick_rates: Tick rate per camera, in Hz, in batch order.
            steps: Number of app updates to run.

        Returns:
            Two-element tuple containing the number of fetched frames and the per-tile
            count of unique images.
        """
        cube = await self._setup_tick_rate_scene(tick_rates)
        with mock.patch("carb.log_warn") as log_warn:
            sensor = TiledCameraSensor("/World/cam_.*", resolution=(120, 160), annotators=["rgb"])
        self.assertTrue(
            any("different 'omni:sensor:tickRate' values" in str(call) for call in log_warn.call_args_list),
            f"expected a mixed tick rate warning, got: {log_warn.call_args_list}",
        )
        try:
            fetched, unique = await self._count_unique_tiles(sensor, cube, steps)
            cprint(f"mixed tick rates {tick_rates}: fetched={fetched} unique={unique}")
            # no tile is scheduled independently: they all render together
            for count in unique[1:]:
                self.assertAlmostEqual(count, unique[0], delta=max(2, fetched // 20))
            return fetched, unique
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_mixed_tick_rates_follow_the_first_camera(self) -> None:
        """Schedule a mixed-tick-rate batch by its first camera, ignoring the other rates.

        A tiled render product is ticked as a whole, so a camera cannot be throttled relative to
        the others in the same batch. The rate comes from the first camera in the batch, which
        makes the result depend on camera ordering rather than on the slowest or fastest rate.
        This test pins that limitation, so a renderer change that adds per-tile scheduling is
        caught here rather than silently changing behavior.
        """
        steps = 120
        slow_first_fetched, slow_first = await self._measure_mixed_tick_rates([5.0, 20.0], steps)
        fast_first_fetched, fast_first = await self._measure_mixed_tick_rates([20.0, 5.0], steps)
        # the same pair of rates yields a different batch rate purely from the camera ordering,
        # so neither the slowest nor the fastest rate governs the batch
        self.assertGreater(
            fast_first[0],
            slow_first[0] * 2,
            f"expected the batch to follow its first camera, but 20/5 Hz rendered {fast_first} of "
            f"{fast_first_fetched} frames while 5/20 Hz rendered {slow_first} of {slow_first_fetched}",
        )
        # each ordering tracks its own first camera rather than rendering on every frame
        self.assertLess(fast_first[0], fast_first_fetched * 0.75)
        self.assertLess(slow_first[0], slow_first_fetched * 0.25)

    async def test_construct_from_camera_object(self) -> None:
        """Author a tiled sensor from an existing optical ``Camera`` wrapper."""
        await populate_stage(2, "wrap")
        camera = Camera("/World/A_.*")
        sensor = TiledCameraSensor(camera, resolution=RESOLUTION, annotators=["rgb"], tick_rate=25.0)
        try:
            self.assertIsInstance(sensor.authoring_object, RtxCamera)
            self.assertEqual(list(sensor.authoring_object.paths), list(camera.paths))
            for prim in sensor.authoring_object.prims:
                self.assertTrue(prim.HasAPI("OmniSensorAPI"))
                self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 25.0)
        finally:
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_rtx_camera_instance_ignores_authoring_kwargs(self) -> None:
        """Preserve the tick rate configured on a supplied ``RtxCamera`` instance."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        camera = RtxCamera([f"/World/cam_{i}" for i in range(2)], tick_rate=30.0)
        with mock.patch("carb.log_warn") as log_warn:
            sensor = TiledCameraSensor(camera, resolution=RESOLUTION, annotators=["rgb"], tick_rate=10.0)
        self.assertTrue(
            any("arguments are ignored" in str(call) for call in log_warn.call_args_list),
            f"expected an ignored authoring arguments warning, got: {log_warn.call_args_list}",
        )
        try:
            self.assertIs(sensor.authoring_object, camera)
            for prim in sensor.authoring_object.prims:
                self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 30.0)
        finally:
            del sensor
            del camera
            await app_utils.update_app_async(steps=3)

    async def test_construct_applies_schemas_and_attributes(self) -> None:
        """Forward ``schemas`` and ``attributes`` to every camera authored from paths."""
        await populate_stage(2, "wrap")
        sensor = TiledCameraSensor(
            "/World/A_.*",
            resolution=RESOLUTION,
            annotators=["rgb"],
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
            attributes={"focalLength": 24.0},
        )
        try:
            for prim in sensor.authoring_object.prims:
                self.assertTrue(prim.HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
                self.assertAlmostEqual(prim.GetAttribute("focalLength").Get(), 24.0)
        finally:
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_invalid_annotator_does_not_create_camera_prims(self) -> None:
        """Reject an invalid annotator before creating cameras from non-existing paths."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        paths = ["/World/new_cam_0", "/World/new_cam_1"]
        with self.assertRaises(ValueError):
            TiledCameraSensor(paths, resolution=RESOLUTION, annotators=["invalid_annotator"])
        stage = stage_utils.get_current_stage(backend="usd")
        for path in paths:
            self.assertFalse(stage.GetPrimAtPath(path).IsValid())

    async def test_invalid_annotator_does_not_apply_camera_schemas(self) -> None:
        """Reject an invalid annotator before applying schemas to existing cameras."""
        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()
        paths = [f"/World/plain_cam_{i}" for i in range(2)]
        for path in paths:
            stage_utils.define_prim(path, "Camera")
        with self.assertRaises(ValueError):
            TiledCameraSensor("/World/plain_cam_.*", resolution=RESOLUTION, annotators=["invalid_annotator"])
        stage = stage_utils.get_current_stage(backend="usd")
        for path in paths:
            self.assertFalse(stage.GetPrimAtPath(path).HasAPI("OmniSensorAPI"))

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_len(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that the number of cameras in the tiled sensor matches the expected count.

        Args:
            prim: Tiled camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that tiled camera sensor data is correctly retrieved for all annotators.

        Args:
            prim: Tiled camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
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
                    self.assertEqual(normalize_semantics(idToSemantics[id]), normalize_semantics(expected_semantics))
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
                normalized_expected_values = [normalize_semantics(value) for value in expected_values]
                for value in info["idToLabels"].values():
                    self.assertIn(
                        normalize_semantics(value),
                        normalized_expected_values,
                        msg=f"Annotator info mismatch for '{annotator}'",
                    )
            else:
                self.assertDictEqual({}, info, msg=f"Annotator info mismatch for '{annotator}'")

    @parametrize(
        prim_class=TiledCameraSensor,
        prim_class_kwargs={"resolution": RESOLUTION, "annotators": list(EXPECTED_ANNOTATOR_SPEC.keys())},
        populate_stage_func=populate_stage,
    )
    async def test_tiled_data(self, prim: Any, num_prims: int, operation: str) -> None:
        """Test that tiled camera sensor data is correctly retrieved in tiled format for all annotators.

        Args:
            prim: Tiled camera sensor under test.
            num_prims: Number of camera prims created by the parametrized fixture.
            operation: Parametrized fixture operation name.
        """
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
                    self.assertEqual(normalize_semantics(idToSemantics[id]), normalize_semantics(expected_semantics))
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
                filename = f"tiled_camera_sensor_{operation}_{num_prims}_{annotator}.png"
                filepath = os.path.join(filedir, filename)
                os.makedirs(filedir, exist_ok=True)
                print(f"Saving image to {filepath}")
                cv2.imwrite(filepath, image)

    async def test_shared_runtime_lifecycle(self) -> None:
        """Drive the annotator and render product lifecycle inherited from ``SensorRuntime``."""
        await populate_stage(3, "wrap")
        sensor = TiledCameraSensor("/World/A_.*", resolution=RESOLUTION, annotators=["rgb"])
        try:
            self.assertIsInstance(sensor, SensorRuntime)
            # a batched runtime wraps every matched camera instead of rejecting more than one prim
            self.assertEqual(len(sensor), 3)
            self.assertIsInstance(sensor.authoring_object, RtxCamera)
            self.assertIs(sensor.camera, sensor.authoring_object.camera)
            self.assertEqual(list(sensor.camera.paths), list(sensor.authoring_object.paths))
            self.assertTrue(str(sensor.render_product.GetPath()).startswith("/Render"))
            self.assertEqual(sensor.annotators, ["rgb"])
            sensor.attach_annotators("normals")
            self.assertEqual(sensor.annotators, ["normals", "rgb"])
            sensor.detach_annotators("normals")
            self.assertEqual(sensor.annotators, ["rgb"])
            with self.assertRaises(ValueError):
                sensor.attach_annotators("invalid_annotator_xyz")
        finally:
            del sensor
            await app_utils.update_app_async(steps=3)

    async def test_get_data_returns_none_during_warm_up(self) -> None:
        """Return ``None`` and preserve info while the annotator has no payload."""
        sensor = TiledCameraSensor.__new__(TiledCameraSensor)
        sensor._resolution = RESOLUTION
        sensor._tiled_resolution = TILED_RESOLUTION
        sensor._hydra_texture = None
        sensor._data_ready = False
        sensor._annotators_spec = {"rgb": {"name": "rgb", "channels": 4, "output_channels": 3, "dtype": wp.uint8}}
        sensor._annotators = {"rgb": FakeAnnotator({"data": None, "info": {"frameId": 7}})}

        try:
            self.assertEqual(sensor.get_data("rgb"), (None, {"frameId": 7}))
            # `has_data()` is gated on the render product, so assert the latch a missing payload drives
            self.assertFalse(sensor._data_ready)
        finally:
            sensor._annotators = {}
            sensor._hydra_texture = None

    async def test_has_data(self) -> None:
        """Report completed render data before a frame is fetched."""
        await populate_stage(2, "wrap")
        sensor = TiledCameraSensor("/World/A_.*", resolution=RESOLUTION, annotators=["rgb"])
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

    async def test_get_data_after_warm_up_wait(self) -> None:
        """Fetch batched frames from plain USD Camera prims after bounding the warm-up wait.

        A missing GPU render engine is not observable at construction time: Replicator returns a valid
        USD RenderProduct whether or not an engine was bound. So the sensor cannot raise there, and that
        state is instead detected by waiting for frames and then checking ``has_data``.
        """
        resolution = (120, 160)  # following OpenCV/NumPy convention (height, width)
        await populate_stage(2, "wrap")
        sensor = TiledCameraSensor("/World/A_.*", resolution=resolution, annotators=["rgb"])
        try:
            self.assertFalse(sensor.has_data())
            app_utils.play(commit=True)
            for _ in range(20):
                await app_utils.update_app_async()
                if sensor.has_data():
                    break
            self.assertTrue(sensor.has_data())
            data, info = sensor.get_data("rgb")
            self.assertIsNotNone(data)
            self.assertEqual(tuple(data.shape), (2, *resolution, 3))
            self.assertIsInstance(info, dict)
            with self.assertRaises(ValueError):
                sensor.get_data("invalid_annotator_xyz")
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            del sensor
            await app_utils.update_app_async(steps=3)
