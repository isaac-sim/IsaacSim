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

"""Validate zero-sigma output and default-noise activity for supported depth cameras."""

import gc
import re
import tempfile
from pathlib import Path
from typing import Any

import carb
import cv2
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cone, Cube, GroundPlane, Sphere, SphereLight
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.sensors.experimental.rtx import (
    SUPPORTED_CAMERA_CONFIGS,
    RtxCamera,
    SingleViewDepthCameraSensor,
    get_camera_metadata,
)
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Usd, UsdGeom, UsdPhysics

SAVE_DEBUG_IMAGES = False
UPDATE_GOLDEN_IMAGES = False

_ZERO_SIGMA_GOLDEN_DIR = Path(__file__).resolve().parent / "data" / "golden" / "depth_sensor_output_zero_sigma"
_DEBUG_DIR = Path(tempfile.gettempdir()) / "isaacsim.sensors.experimental.rtx" / "depth_sensor_output"
# Zeroing sigma leaves quantized disparity. Small render changes can move boundary pixels between
# bands, so bound the affected pixel share instead of averaging their large differences.
_ZERO_SIGMA_LEVEL_TOLERANCE = 2.0
_ZERO_SIGMA_DIFFERING_PIXEL_PERCENTAGE = 10.0
_DEPTH_PREVIEW_RANGE_METERS = (0.5, 10.0)
# These authored fine-disparity products expose more band boundaries than the fallback and D45x
# configurations. Their limits are about twice the largest affected share seen in repeated runs.
_ZERO_SIGMA_DIFFERING_PIXEL_PERCENTAGE_OVERRIDES = {
    "/Isaac/Sensors/Luxonis/OAK4-D/oak4_d.usd": 15.0,
    "/Isaac/Sensors/Luxonis/OAK4-D_Wide/oak4_d_wide.usd": 15.0,
    "/Isaac/Sensors/Luxonis/OAK-D_Pro_PoE/oak_d_pro_poe.usd": 15.0,
    "/Isaac/Sensors/Luxonis/OAK-D_Pro_W_PoE/oak_d_pro_w_poe.usd": 15.0,
    "/Isaac/Sensors/Orbbec/Gemini335L/orbbec_gemini_335L.usd": 12.0,
}
_WARMUP_UPDATES = 15
_MAXIMUM_UPDATES = 60
# Emitted frames discarded per render product to allow parameter propagation and renderer warm-up.
_SETTLE_FRAMES = 5
# Updates allowed while waiting for every render product to emit its settled frames.
_SETTLE_MAXIMUM_UPDATES = 240
# The depth change default sigma introduces must at least double the zero-sigma drift and jitter.
_NOISE_ACTIVITY_RATIO = 2.0
_FALLBACK_DEPTH_RESOLUTION = (256, 320)
_CAMERA_EYE = Gf.Vec3d(1.0, 0.5, 1.0)
_CAMERA_TARGET = Gf.Vec3d(0.0, 0.0, 0.25)
_CAMERA_POSE_ATOL = 1e-4
_DEPTH_FRAME_WRITER_NAME = "_DepthSensorOutputFrameWriter"


class _DepthSensorOutputFrameWriter(rep.Writer):
    """Collect consecutive frames through sensor-output callbacks.

    Multi-tick scheduling can produce zero or multiple sensor frames per app update. Callbacks
    avoid treating a cached ``get_data()`` payload as a new frame.
    """

    def __init__(self) -> None:
        self.version = "1.0.0"
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("DepthSensorDistance")]
        self.frames: list[dict[str, Any]] = []

    def write_metadata(self) -> None:
        """Skip metadata output for this in-memory writer."""

    def write(self, data: dict[str, Any]) -> None:
        for render_product_data in data.get("renderProducts", {}).values():
            payload = render_product_data.get("DepthSensorDistance")
            info = {}
            if isinstance(payload, dict):
                info = dict(payload.get("info", {}))
                payload = payload.get("data")
            if payload is None:
                continue
            array = np.asarray(payload)
            if array.size:
                self.frames.append({"data": np.array(array, copy=True), "info": info})


if _DEPTH_FRAME_WRITER_NAME not in rep.WriterRegistry.get_writers():
    rep.WriterRegistry.register(_DepthSensorOutputFrameWriter)


def _slug(value: str) -> str:
    """Return a filesystem-safe identifier."""
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


def _golden_filename(config_path: str, render_product_path: str) -> str:
    """Return a unique golden filename for a render product."""
    render_product_name = re.sub(r"(?i)(?:_?render_?product)$", "_rp", Path(render_product_path).name)
    return f"{_slug(Path(config_path).stem)}__{_slug(render_product_name)}.png"


def _differing_pixel_percentage(golden: np.ndarray, test: np.ndarray, tolerance: float) -> float:
    """Return the pixel share whose image value differs beyond a tolerance."""
    difference = np.abs(golden.astype(np.int16) - test.astype(np.int16))
    return float(np.mean(difference > tolerance) * 100.0)


def _depth_preview(depth: np.ndarray) -> np.ndarray:
    """Return a fixed-range grayscale depth preview."""
    depth = depth.squeeze()
    valid = np.isfinite(depth) & (depth > 0.0)
    minimum, maximum = _DEPTH_PREVIEW_RANGE_METERS
    normalized = np.zeros(depth.shape, dtype=np.float32)
    normalized[valid] = np.clip((depth[valid] - minimum) / (maximum - minimum), 0.0, 1.0)
    preview = ((1.0 - normalized) * 255.0).astype(np.uint8)
    preview[~valid] = 0
    return preview


def _save_depth_preview(depth: np.ndarray, path: Path) -> None:
    """Save a fixed-range depth preview."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), _depth_preview(depth)):
        raise RuntimeError(f"Failed to save depth preview '{path}'.")


def _assert_camera_view(camera_path: str) -> None:
    """Catch parent-transform regressions by checking the camera's world pose."""
    camera_prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(camera_path)
    world_transform = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    eye = np.array(list(world_transform.ExtractTranslation()))
    np.testing.assert_allclose(
        eye,
        np.array(list(_CAMERA_EYE)),
        atol=_CAMERA_POSE_ATOL,
        err_msg=f"Camera '{camera_path}' is not at the requested eye position.",
    )

    # USD cameras look down their local -Z axis.
    forward = np.array(list(world_transform.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))))
    expected_forward = np.array(list(_CAMERA_TARGET)) - np.array(list(_CAMERA_EYE))
    np.testing.assert_allclose(
        forward / np.linalg.norm(forward),
        expected_forward / np.linalg.norm(expected_forward),
        atol=_CAMERA_POSE_ATOL,
        err_msg=f"Camera '{camera_path}' is not aimed at the requested target.",
    )


def _set_depth_camera_view(camera_path: str) -> None:
    """Aim a camera from the shared test pose."""
    ViewportManager.set_camera_view(camera_path, eye=list(_CAMERA_EYE), target=list(_CAMERA_TARGET))
    _assert_camera_view(camera_path)


async def _capture_depth_frames(
    frame_writers: dict[str, _DepthSensorOutputFrameWriter], config_path: str
) -> dict[str, dict[str, Any]]:
    """Return one complete frame set after renderer warm-up."""
    for update_index in range(1, _MAXIMUM_UPDATES + 1):
        await app_utils.update_app_async()
        if update_index == _WARMUP_UPDATES:
            for writer in frame_writers.values():
                writer.frames.clear()
        if update_index >= _WARMUP_UPDATES and all(writer.frames for writer in frame_writers.values()):
            return {name: writer.frames[-1] for name, writer in frame_writers.items()}

    missing = sorted(name for name, writer in frame_writers.items() if not writer.frames)
    raise TimeoutError(
        f"Timed out after {_MAXIMUM_UPDATES} updates waiting for depth output from "
        f"config '{config_path}'. Missing render products: {missing}."
    )


def _depth_mae(first: np.ndarray, second: np.ndarray) -> float | None:
    """Return mean absolute depth difference over common finite pixels, or None if there are none."""
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    valid = np.isfinite(first) & np.isfinite(second)
    if not valid.any():
        return None
    return float(np.mean(np.abs(second[valid] - first[valid])))


async def _measure_settled_depth_frames(
    frame_writers: dict[str, _DepthSensorOutputFrameWriter], config_path: str
) -> dict[str, tuple[np.ndarray, float | None]]:
    """Return a settled frame and the renderer's frame-to-frame mean absolute difference for each render product.

    Wait for both app warm-up and emitted sensor frames, then use the latest pair. Sensor callbacks
    avoid counting cached data as a new frame when app updates outpace sensor ticks.

    Adjacent-frame mean absolute difference provides a local jitter estimate; it does not bound drift across captures.
    """
    required_frames = _SETTLE_FRAMES + 2
    for writer in frame_writers.values():
        writer.frames.clear()
    for update_index in range(1, _SETTLE_MAXIMUM_UPDATES + 1):
        await app_utils.update_app_async()
        if update_index >= _WARMUP_UPDATES and all(
            len(writer.frames) >= required_frames for writer in frame_writers.values()
        ):
            break

    short = {name: len(writer.frames) for name, writer in frame_writers.items() if len(writer.frames) < required_frames}
    if short:
        raise TimeoutError(
            f"Timed out after {_SETTLE_MAXIMUM_UPDATES} updates waiting for {required_frames} emitted frames "
            f"from config '{config_path}'. Render products short of the settling margin: {short}."
        )

    return {
        name: (
            np.asarray(writer.frames[-1]["data"], dtype=np.float64),
            _depth_mae(writer.frames[-2]["data"], writer.frames[-1]["data"]),
        )
        for name, writer in frame_writers.items()
    }


def _attach_depth_frame_writers(
    render_products: dict[str, tuple[str, Any]],
) -> dict[str, _DepthSensorOutputFrameWriter]:
    """Attach one emitted-frame collector per render product."""
    # Only sensor emissions, not cached poll results, should enter the noise comparison.
    return {name: sensor.attach_writer(_DEPTH_FRAME_WRITER_NAME) for name, (_, sensor) in render_products.items()}


def _detach_depth_frame_writers(render_products: dict[str, tuple[str, Any]]) -> None:
    """Detach emitted-frame collectors."""
    for _, sensor in render_products.values():
        sensor.detach_writer(_DEPTH_FRAME_WRITER_NAME)


def _configure_fallback_depth_sensor(sensor: SingleViewDepthCameraSensor) -> None:
    """Apply the shared depth model to a generated render product."""
    sensor.set_sensor_baseline(55.0)
    sensor.set_sensor_focal_length(891.0)
    sensor.set_sensor_size(1280.0)
    sensor.set_sensor_maximum_disparity(110.0)
    sensor.set_sensor_disparity_confidence(0.99)
    sensor.set_sensor_noise_parameters(noise_mean=0.5, noise_sigma=1.0)
    sensor.set_sensor_disparity_noise_downscale(1.0)
    sensor.set_sensor_distance_cutoffs(minimum_distance=0.5, maximum_distance=9999.9)


def _disable_asset_rigid_bodies(root_prim: Usd.Prim) -> None:
    """Prevent sensor cameras from moving during simulation."""
    rigid_body_apis = [
        UsdPhysics.RigidBodyAPI(prim) for prim in Usd.PrimRange(root_prim) if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    for rigid_body_api in rigid_body_apis:
        rigid_body_api.GetRigidBodyEnabledAttr().Set(False)

    enabled_paths = [
        str(rigid_body_api.GetPrim().GetPath())
        for rigid_body_api in rigid_body_apis
        if rigid_body_api.GetRigidBodyEnabledAttr().Get()
    ]
    if enabled_paths:
        raise RuntimeError(f"Failed to disable rigid bodies in the sensor asset: {enabled_paths}.")


def _create_depth_sensors(rtx_camera: RtxCamera) -> list[SingleViewDepthCameraSensor]:
    """Reuse authored depth products or configure a fallback product."""
    asset_root_path = getattr(rtx_camera, "_asset_root_path", None)
    if asset_root_path is None:
        return []
    stage = stage_utils.get_current_stage(backend="usd")
    root_prim = stage.GetPrimAtPath(asset_root_path)
    if not root_prim.IsValid():
        carb.log_warn(f"Asset root prim at {asset_root_path} is not valid; skipping depth setup.")
        return []

    _disable_asset_rigid_bodies(root_prim)

    authored_render_products: list[tuple[Usd.Prim, str]] = []
    for child in Usd.PrimRange(root_prim):
        if not (
            child.GetTypeName() == "RenderProduct"
            and child.HasAPI("OmniSensorDepthSensorSingleViewAPI")
            and child.HasRelationship("camera")
        ):
            continue
        targets = child.GetRelationship("camera").GetTargets()
        if len(targets) != 1:
            continue
        authored_render_products.append((child, str(targets[0])))

    sensors: list[SingleViewDepthCameraSensor] = []
    for render_product, camera_path in authored_render_products:
        _set_depth_camera_view(camera_path)
        sensor = SingleViewDepthCameraSensor(camera_path, annotators="depth_sensor_distance")
        if str(sensor.render_product.GetPath()) != str(render_product.GetPath()):
            raise RuntimeError(
                f"Expected depth sensor for '{camera_path}' to attach to '{render_product.GetPath()}', "
                f"but it attached to '{sensor.render_product.GetPath()}'."
            )
        sensors.append(sensor)

    if sensors:
        return sensors

    # Assets without an authored depth product use one likely depth camera with a shared model.
    cameras = [child for child in Usd.PrimRange(root_prim) if child.GetTypeName() == "Camera"]
    depth_cameras = [
        camera for camera in cameras if any(hint in str(camera.GetPath()).lower() for hint in ("depth", "tof"))
    ]
    if not depth_cameras:
        depth_cameras = [camera for camera in cameras if "left" in str(camera.GetPath()).lower()]
    if not depth_cameras and cameras:
        depth_cameras = cameras[:1]
    elif depth_cameras:
        depth_cameras = depth_cameras[-1:]

    for camera in depth_cameras:
        camera_path = str(camera.GetPath())
        _set_depth_camera_view(camera_path)
        sensor = SingleViewDepthCameraSensor(
            camera_path,
            resolution=_FALLBACK_DEPTH_RESOLUTION,
            annotators="depth_sensor_distance",
        )
        _configure_fallback_depth_sensor(sensor)
        # Use the source camera name because generated render-product paths are not stable.
        sensor._source_render_product_path = (
            f"{asset_root_path}/GeneratedDepthRenderProducts/{camera.GetName()}_render_product"
        )
        sensors.append(sensor)
    return sensors


class TestDepthSensorOutput(omni.kit.test.AsyncTestCase):
    """Validate zero-sigma output and default-noise activity for registered depth cameras."""

    async def setUp(self) -> None:
        """Create the shared static scene."""
        super().setUp()
        self._retained_cameras: list[RtxCamera] = []
        self._retained_sensors: list[SingleViewDepthCameraSensor] = []

        await stage_utils.create_new_stage_async()
        await ViewportManager.wait_for_viewport_async()

        sphere_light = SphereLight("/World/SphereLight", positions=[1.0, -1.0, 1.0])
        sphere_light.set_intensities(intensities=100000)
        GroundPlane("/World/GroundPlane")
        Cone("/World/Cone", radii=0.5, heights=1.0, positions=[0.0, 0.0, 0.0], colors=[1.0, 0.0, 0.0])
        Cube("/World/Cube", sizes=0.5, positions=[-0.5, 0.25, 0.25], colors=[0.0, 1.0, 0.0])
        Sphere("/World/Sphere", radii=0.25, positions=[0.25, -0.35, 0.25], colors=[0.0, 0.0, 1.0])

    async def tearDown(self) -> None:
        """Release sensors, stop playback, and clear the stage."""
        app_utils.stop(commit=True)
        await app_utils.update_app_async()

        self._retained_sensors.clear()
        self._retained_cameras.clear()
        gc.collect()
        await app_utils.update_app_async(steps=3)
        await stage_utils.create_new_stage_async()
        super().tearDown()

    async def _prepare_render_products(self, config_path: str) -> dict[str, tuple[str, Any]]:
        """Load and immobilize an asset, then configure its depth products."""
        metadata = get_camera_metadata(config_path)
        self.assertTrue(metadata["is_depth_sensor"], f"Config is not marked as a depth sensor: {config_path}")

        rtx_camera = RtxCamera.create(
            path=stage_utils.generate_next_free_path(metadata["prim_prefix"]),
            usd_path=get_assets_root_path() + config_path,
        )
        self._retained_cameras.append(rtx_camera)
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

        sensors = _create_depth_sensors(rtx_camera)
        self._retained_sensors.extend(sensors)
        self.assertGreater(
            len(sensors),
            0,
            f"Config '{config_path}' has no usable camera for depth output.",
        )

        render_products: dict[str, tuple[str, Any]] = {}
        stage = stage_utils.get_current_stage(backend="usd")
        for sensor in sensors:
            for camera_path in sensor.camera.paths:
                self.assertTrue(
                    stage.GetPrimAtPath(camera_path).IsValid(),
                    f"Config '{config_path}' did not load camera prim '{camera_path}'.",
                )
                _set_depth_camera_view(camera_path)

            render_product_path = str(sensor.render_product.GetPath())
            render_product_name = render_product_path.rsplit("/", maxsplit=1)[-1]
            self.assertNotIn(
                render_product_name,
                render_products,
                f"Config '{config_path}' produced duplicate render-product name '{render_product_name}'.",
            )
            render_products[render_product_name] = (render_product_path, sensor)

        self.assertEqual(
            len(render_products),
            len(sensors),
            f"Config '{config_path}' did not expose every depth-sensor render product.",
        )

        for _, sensor in render_products.values():
            sensor.set_enabled_post_processing(True)
        return render_products

    async def _validate_depth_sensor_output_zero_sigma(self, config_path: str) -> None:
        """Compare zero-sigma depth against fixed-range PNG goldens."""
        render_products = await self._prepare_render_products(config_path)
        differing_pixel_percentage = _ZERO_SIGMA_DIFFERING_PIXEL_PERCENTAGE_OVERRIDES.get(
            config_path, _ZERO_SIGMA_DIFFERING_PIXEL_PERCENTAGE
        )

        for _, sensor in render_products.values():
            # Keep noise_mean: it controls disparity quantization, and zeroing it removes depth output.
            sensor.set_sensor_noise_parameters(noise_sigma=0.0)
        await app_utils.update_app_async(steps=3)

        frame_writers = _attach_depth_frame_writers(render_products)
        app_utils.play(commit=True)
        try:
            frames = await _capture_depth_frames(frame_writers, config_path)
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            _detach_depth_frame_writers(render_products)

        for render_product_name, (render_product_path, sensor) in render_products.items():
            frame = frames[render_product_name]
            raw = frame["data"]
            height, width = sensor.resolution
            self.assertEqual(
                raw.size,
                height * width,
                f"Config '{config_path}', render product '{render_product_path}' returned "
                f"{raw.size} depth values for resolution {(height, width)}.",
            )
            raw = raw.reshape((height, width, 1))

            finite_values = raw[np.isfinite(raw)]
            self.assertGreater(
                finite_values.size,
                0,
                f"Config '{config_path}', render product '{render_product_path}' returned no finite depth values.",
            )
            self.assertGreater(
                np.ptp(finite_values),
                0.0,
                f"Config '{config_path}', render product '{render_product_path}' returned constant depth values.",
            )

            stable_render_product_path = getattr(sensor, "_source_render_product_path", render_product_path)
            filename = _golden_filename(config_path, stable_render_product_path)
            golden_path = _ZERO_SIGMA_GOLDEN_DIR / filename
            generated_path = _DEBUG_DIR / filename
            image = _depth_preview(raw)

            if SAVE_DEBUG_IMAGES:
                _save_depth_preview(raw, generated_path)

            if UPDATE_GOLDEN_IMAGES:
                _save_depth_preview(raw, golden_path)
                carb.log_warn(f"Updated depth-sensor golden image at: {golden_path}")
                continue

            if not golden_path.exists() and not SAVE_DEBUG_IMAGES:
                _save_depth_preview(raw, generated_path)
            self.assertTrue(
                golden_path.exists(),
                f"Golden image not found for config '{config_path}', render product '{render_product_path}'; "
                f"golden image '{golden_path}', generated image '{generated_path}'. "
                "Set UPDATE_GOLDEN_IMAGES = True to generate the golden.",
            )
            golden = cv2.imread(str(golden_path), cv2.IMREAD_GRAYSCALE)
            if golden is None and not SAVE_DEBUG_IMAGES:
                _save_depth_preview(raw, generated_path)
            self.assertIsNotNone(
                golden,
                f"Failed to load golden image for config '{config_path}', render product '{render_product_path}'; "
                f"golden image '{golden_path}', generated image '{generated_path}'.",
            )
            if golden.shape != image.shape and not SAVE_DEBUG_IMAGES:
                _save_depth_preview(raw, generated_path)
            self.assertEqual(
                golden.shape,
                image.shape,
                f"Golden shape mismatch for config '{config_path}', render product '{render_product_path}', "
                f"golden image '{golden_path}', generated image '{generated_path}': "
                f"{golden.shape} != {image.shape}.",
            )

            # Bound sparse quantization-boundary changes directly; a mean can hide their pixel share.
            actual_differing_percentage = _differing_pixel_percentage(
                golden,
                image,
                _ZERO_SIGMA_LEVEL_TOLERANCE,
            )
            if actual_differing_percentage > differing_pixel_percentage and not SAVE_DEBUG_IMAGES:
                _save_depth_preview(raw, generated_path)
            self.assertLessEqual(
                actual_differing_percentage,
                differing_pixel_percentage,
                f"Golden comparison failed for config '{config_path}', render product '{render_product_path}', "
                f"golden image '{golden_path}', generated image '{generated_path}' "
                f"with at most {differing_pixel_percentage}% of pixels differing by more than "
                f"{_ZERO_SIGMA_LEVEL_TOLERANCE} levels; actual: {actual_differing_percentage}%.",
            )

    async def _validate_depth_sensor_noise(self, config_path: str) -> None:
        """Verify default sigma perturbs depth relative to zero sigma."""
        render_products = await self._prepare_render_products(config_path)
        default_sigmas = {
            name: sensor.get_sensor_noise_parameters()[1] for name, (_, sensor) in render_products.items()
        }
        for _, sensor in render_products.values():
            sensor.set_sensor_noise_parameters(noise_sigma=0.0)
        await app_utils.update_app_async(steps=3)

        frame_writers = _attach_depth_frame_writers(render_products)
        app_utils.play(commit=True)
        try:
            quiet_before = await _measure_settled_depth_frames(frame_writers, config_path)
            for name, (_, sensor) in render_products.items():
                sensor.set_sensor_noise_parameters(noise_sigma=default_sigmas[name])
            noisy_frames = await _measure_settled_depth_frames(frame_writers, config_path)
            for _, sensor in render_products.values():
                sensor.set_sensor_noise_parameters(noise_sigma=0.0)
            quiet_frames = await _measure_settled_depth_frames(frame_writers, config_path)
        finally:
            app_utils.stop(commit=True)
            await app_utils.update_app_async()
            _detach_depth_frame_writers(render_products)

        for render_product_name, (render_product_path, _) in render_products.items():
            before_frame, before_jitter = quiet_before[render_product_name]
            noisy_frame, _ = noisy_frames[render_product_name]
            quiet_frame, after_jitter = quiet_frames[render_product_name]
            self.assertIsNotNone(
                before_jitter,
                f"Config '{config_path}', render product '{render_product_path}' "
                "emitted no comparable initial zero-sigma frame pair.",
            )
            self.assertIsNotNone(
                after_jitter,
                f"Config '{config_path}', render product '{render_product_path}' "
                "emitted no comparable zero-sigma frame pair.",
            )
            # Compare all three phases on the same pixels, so invalid-depth changes cannot bias the baseline.
            valid = np.isfinite(before_frame) & np.isfinite(noisy_frame) & np.isfinite(quiet_frame)
            self.assertTrue(
                valid.any(),
                f"Config '{config_path}', render product '{render_product_path}' "
                "emitted no comparable default-sigma and zero-sigma frames.",
            )
            # Bracket the noisy capture with zero sigma to measure drift over the full experiment.
            # A single adjacent-frame pair can underestimate that drift and accept an ineffective sigma edit.
            # Absolute differences avoid squaring large silhouette and disparity-band jumps.
            drift_mae = _depth_mae(before_frame[valid], quiet_frame[valid])
            jitter_mae = max(drift_mae, before_jitter, after_jitter)
            noise_mae = min(
                _depth_mae(noisy_frame[valid], before_frame[valid]),
                _depth_mae(noisy_frame[valid], quiet_frame[valid]),
            )
            self.assertGreater(
                noise_mae,
                jitter_mae * _NOISE_ACTIVITY_RATIO,
                f"Config '{config_path}', render product '{render_product_path}' shows no noise contribution: "
                f"minimum default-sigma versus zero-sigma mean absolute difference {noise_mae}, zero-sigma drift/jitter mean absolute difference {jitter_mae}, "
                f"required ratio {_NOISE_ACTIVITY_RATIO}.",
            )


def _make_depth_sensor_output_zero_sigma_test(config_path: str) -> Any:
    """Return a zero-sigma golden-output test for one depth-sensor config."""

    async def test_depth_sensor_output_zero_sigma(self: TestDepthSensorOutput) -> None:
        await self._validate_depth_sensor_output_zero_sigma(config_path)

    test_depth_sensor_output_zero_sigma.__doc__ = (
        f"Test depth-sensor output for ``{config_path}`` with noise sigma set to zero."
    )
    return test_depth_sensor_output_zero_sigma


def _make_depth_sensor_noise_test(config_path: str) -> Any:
    """Return a noise-response test for one depth-sensor config."""

    async def test_depth_sensor_noise(self: TestDepthSensorOutput) -> None:
        await self._validate_depth_sensor_noise(config_path)

    test_depth_sensor_noise.__doc__ = f"Test depth-sensor noise for ``{config_path}``."
    return test_depth_sensor_noise


def _install_depth_sensor_tests() -> None:
    """Attach a zero-sigma golden test and default-noise activity test per camera."""
    for config_path in SUPPORTED_CAMERA_CONFIGS:
        if get_camera_metadata(config_path)["is_depth_sensor"]:
            slug = _slug(config_path)
            for name, test in (
                (
                    f"test_depth_sensor_output_zero_sigma__{slug}",
                    _make_depth_sensor_output_zero_sigma_test(config_path),
                ),
                (f"test_depth_sensor_noise__{slug}", _make_depth_sensor_noise_test(config_path)),
            ):
                test.__name__ = name
                setattr(TestDepthSensorOutput, name, test)


_install_depth_sensor_tests()
del _install_depth_sensor_tests
