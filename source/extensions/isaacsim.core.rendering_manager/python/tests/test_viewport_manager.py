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

"""Verifies ViewportManager camera, viewport, render product, and resolution utilities against live USD viewport state. The tests cover camera selection and pose updates, viewport window discovery, and render product lookup."""

from typing import Any
from unittest import mock

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
import omni.kit.test
from isaacsim.core.rendering_manager import ViewportManager
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender, Vt

_SETTING_RATE_LIMIT_ENABLED = "/app/runLoops/main/rateLimitEnabled"


class TestViewportManager(omni.kit.test.AsyncTestCase):
    """Tests viewport, render product, camera, and resolution helper APIs."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        await stage_utils.create_new_stage_async()
        # ---------------

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        # ------------------
        stage_utils.close_stage()
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    @staticmethod
    def _define_render_product(
        path: str,
        *,
        target_path: str | None = None,
        resolution: tuple[int, int] = (1, 1),
        device_ids: list[int] | None = None,
    ) -> UsdRender.Product:
        stage = stage_utils.get_current_stage()
        render_product = UsdRender.Product.Define(stage, path)
        render_product.GetResolutionAttr().Set(Gf.Vec2i(*resolution))
        if target_path is not None:
            render_product.GetCameraRel().SetTargets([target_path])
        if device_ids is not None:
            render_product.GetPrim().CreateAttribute("deviceIds", Sdf.ValueTypeNames.UIntArray).Set(
                Vt.UIntArray(device_ids)
            )
        return render_product

    @staticmethod
    def _get_device_ids(render_product: UsdRender.Product) -> list[int]:
        attribute = render_product.GetPrim().GetAttribute("deviceIds")
        if not attribute.IsValid():
            return []
        device_ids = attribute.Get()
        return [] if device_ids is None else [int(device_id) for device_id in device_ids]

    async def test_00_wait_for_viewport(self) -> None:  # 00 ensures that this test is run first
        """Test waiting for viewport readiness."""
        # test cases
        # - no frames are waited for
        result = await ViewportManager.wait_for_viewport_async(max_frames=0)
        self.assertTupleEqual(result, (False, 0), f"Viewport should not be ready if no frames are waited for")
        # - 1 frame is waited for (without sleep time)
        result = await ViewportManager.wait_for_viewport_async(max_frames=1, sleep_time=0.0)
        self.assertTupleEqual(result, (False, 1), f"Viewport should not be ready after 1 frame (without sleep time)")
        # - wait for the default number of frames (with default sleep time)
        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")

    async def test_set_camera(self) -> None:
        """Set cameras through each supported viewport and render product source."""
        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        # test conditions
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        sources = [
            None,
            viewport_api,
            viewport_api.render_product_path,
            "Viewport",
            stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path),
            UsdRender.Product(stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path)),
        ]
        cameras = [
            "/OmniverseKit_Persp",
            "/OmniverseKit_Top",
            "/OmniverseKit_Front",
            "/OmniverseKit_Right",
            stage_utils.define_prim("/CustomCamera", "Camera"),
        ]
        for source in sources:
            for camera in cameras:
                ViewportManager.set_camera(camera, render_product_or_viewport=source)
        # exception
        with self.assertRaisesRegex(ValueError, "not a valid USD Camera prim"):
            ViewportManager.set_camera("/Invalid/Source")

    async def test_get_camera(self) -> None:
        """Resolve USD cameras from each supported viewport and render product source."""
        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        # test conditions
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        sources = [
            None,
            viewport_api,
            viewport_api.render_product_path,
            "Viewport",
            stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path),
            UsdRender.Product(stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path)),
        ]
        for source in sources:
            camera = ViewportManager.get_camera(source)
            self.assertIsInstance(camera, UsdGeom.Camera)

    async def test_get_viewport_and_render_product(self) -> None:
        """Resolve viewport APIs and USD render products from supported source types."""

        def _check_viewport(source: Any) -> None:
            self.assertIn("ViewportAPI", ViewportManager.get_viewport_api(source).__class__.__name__)

        def _check_render_product(source: Any) -> None:
            self.assertIsInstance(ViewportManager.get_render_product(source), UsdRender.Product)

        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        # test cases
        # - unspecified source
        _check_viewport(None)
        _check_render_product(None)
        # - viewport API
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        _check_viewport(viewport_api)
        _check_render_product(viewport_api)
        # - str
        # -- render product path
        render_product_path = viewport_api.render_product_path
        self.assertIsInstance(render_product_path, str)
        self.assertIsNone(ViewportManager.get_viewport_api(render_product_path))
        _check_render_product(render_product_path)
        # -- viewport name
        _check_viewport("Viewport")
        _check_render_product("Viewport")
        # - USD prim
        # -- render product
        render_product_prim = stage_utils.get_current_stage().GetPrimAtPath(render_product_path)
        self.assertIsInstance(render_product_prim, Usd.Prim)
        self.assertIsNone(ViewportManager.get_viewport_api(render_product_prim))
        _check_render_product(render_product_prim)
        _check_render_product(UsdRender.Product(render_product_prim))
        # - unknown source
        self.assertIsNone(ViewportManager.get_viewport_api("/Invalid/Source"))
        self.assertIsNone(ViewportManager.get_render_product("/Invalid/Source"))

    async def test_get_resolution(self) -> None:
        """Read viewport resolution through each supported source type."""
        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        # test cases
        expected_resolution = (1280, 720)
        # - unspecified source
        resolution = ViewportManager.get_resolution()
        self.assertTupleEqual(resolution, expected_resolution)
        # - viewport API
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        resolution = ViewportManager.get_resolution(viewport_api)
        self.assertTupleEqual(resolution, expected_resolution)
        # - str
        # -- render product path
        render_product_path = viewport_api.render_product_path
        self.assertIsInstance(render_product_path, str)
        resolution = ViewportManager.get_resolution(render_product_path)
        self.assertTupleEqual(resolution, expected_resolution)
        # -- viewport name
        resolution = ViewportManager.get_resolution("Viewport")
        self.assertTupleEqual(resolution, expected_resolution)
        # - USD prim
        # -- render product
        render_product_prim = stage_utils.get_current_stage().GetPrimAtPath(render_product_path)
        self.assertIsInstance(render_product_prim, Usd.Prim)
        resolution = ViewportManager.get_resolution(render_product_prim)
        self.assertTupleEqual(resolution, expected_resolution)
        resolution = ViewportManager.get_resolution(UsdRender.Product(render_product_prim))
        self.assertTupleEqual(resolution, expected_resolution)
        # exception
        with self.assertRaisesRegex(ValueError, "Unable to get resolution: unknown"):
            resolution = ViewportManager.get_resolution("/Invalid/Source")

    async def test_optimize_render_products_balances_and_reallocates_cameras(self) -> None:
        """Balance camera render products and update assignments after their resolutions change."""
        root_path = "/Render/OmniverseKit/HydraTextures/TestOptimize"
        root = stage_utils.define_prim(root_path, "Scope")
        stage_utils.define_prim("/World/CameraHigh", "Camera")
        stage_utils.define_prim("/World/CameraLowA", "Camera")
        stage_utils.define_prim("/World/CameraLowB", "Camera")
        high_render_product = self._define_render_product(
            f"{root_path}/CameraHigh",
            target_path="/World/CameraHigh",
            resolution=(200, 100),
        )
        low_render_product_a = self._define_render_product(
            f"{root_path}/CameraLowA",
            target_path="/World/CameraLowA",
            resolution=(100, 100),
        )
        low_render_product_b = self._define_render_product(
            f"{root_path}/CameraLowB",
            target_path="/World/CameraLowB",
            resolution=(100, 100),
        )

        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=2):
            assignments = ViewportManager.optimize_render_products(root)

        self.assertDictEqual(
            assignments,
            {
                f"{root_path}/CameraHigh": [0],
                f"{root_path}/CameraLowA": [1],
                f"{root_path}/CameraLowB": [1],
            },
        )
        self.assertListEqual(self._get_device_ids(high_render_product), [0])
        self.assertListEqual(self._get_device_ids(low_render_product_a), [1])
        self.assertListEqual(self._get_device_ids(low_render_product_b), [1])

        high_render_product.GetResolutionAttr().Set(Gf.Vec2i(50, 50))
        low_render_product_a.GetResolutionAttr().Set(Gf.Vec2i(300, 100))
        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=2):
            assignments = ViewportManager.optimize_render_products(root)

        self.assertDictEqual(
            assignments,
            {
                f"{root_path}/CameraHigh": [1],
                f"{root_path}/CameraLowA": [0],
                f"{root_path}/CameraLowB": [1],
            },
        )
        self.assertListEqual(self._get_device_ids(high_render_product), [1])
        self.assertListEqual(self._get_device_ids(low_render_product_a), [0])
        self.assertListEqual(self._get_device_ids(low_render_product_b), [1])

    async def test_optimize_render_products_preserves_fixed_and_viewport_products(self) -> None:
        """Account for explicit fixed loads without moving non-camera or viewport render products."""
        root_path = "/Render/OmniverseKit/HydraTextures/TestFixed"
        stage_utils.define_prim(root_path, "Scope")
        legacy_lidar = stage_utils.define_prim("/World/LegacyLidar", "Camera")
        legacy_lidar.CreateAttribute("cameraSensorType", Sdf.ValueTypeNames.Token).Set("lidar")
        stage_utils.define_prim("/World/ViewportCamera", "Camera")
        stage_utils.define_prim("/World/CameraHigh", "Camera")
        stage_utils.define_prim("/World/CameraLow", "Camera")
        fixed_render_product = self._define_render_product(
            f"{root_path}/FixedLidar",
            target_path="/World/LegacyLidar",
            resolution=(100, 100),
            device_ids=[0],
        )
        viewport_render_product = self._define_render_product(
            f"{root_path}/omni_kit_widget_viewport_ViewportTexture_0",
            target_path="/World/ViewportCamera",
            resolution=(200, 100),
            device_ids=[0],
        )
        high_render_product = self._define_render_product(
            f"{root_path}/CameraHigh",
            target_path="/World/CameraHigh",
            resolution=(100, 100),
        )
        low_render_product = self._define_render_product(
            f"{root_path}/CameraLow",
            target_path="/World/CameraLow",
            resolution=(10, 10),
        )

        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=2):
            assignments = ViewportManager.optimize_render_products(root_path)

        self.assertDictEqual(
            assignments,
            {
                f"{root_path}/CameraHigh": [1],
                f"{root_path}/CameraLow": [1],
            },
        )
        self.assertListEqual(self._get_device_ids(fixed_render_product), [0])
        self.assertListEqual(self._get_device_ids(viewport_render_product), [0])
        self.assertListEqual(self._get_device_ids(high_render_product), [1])
        self.assertListEqual(self._get_device_ids(low_render_product), [1])

    async def test_optimize_render_products_validates_root_and_no_op_conditions(self) -> None:
        """Reject invalid roots and preserve renderer defaults when allocation would not help."""
        with self.assertRaisesRegex(ValueError, "is not a valid USD prim"):
            ViewportManager.optimize_render_products("/Render/Missing")

        root_path = "/Render/OmniverseKit/HydraTextures/TestNoOp"
        stage_utils.define_prim(root_path, "Scope")
        stage_utils.define_prim("/World/CameraA", "Camera")
        stage_utils.define_prim("/World/CameraB", "Camera")
        render_product_a = self._define_render_product(
            f"{root_path}/CameraA",
            target_path="/World/CameraA",
            resolution=(200, 100),
        )
        render_product_b = self._define_render_product(
            f"{root_path}/CameraB",
            target_path="/World/CameraB",
            resolution=(100, 100),
        )

        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=1):
            self.assertDictEqual(ViewportManager.optimize_render_products(root_path), {})
        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=4):
            self.assertDictEqual(ViewportManager.optimize_render_products(root_path), {})
        with mock.patch.object(ViewportManager, "_get_gpu_count", return_value=2):
            self.assertDictEqual(ViewportManager.optimize_render_products(render_product_a), {})

        self.assertFalse(render_product_a.GetPrim().GetAttribute("deviceIds").IsValid())
        self.assertFalse(render_product_b.GetPrim().GetAttribute("deviceIds").IsValid())

    async def test_set_resolution(self) -> None:
        """Set and verify viewport resolution through each supported source type."""
        status, frames = await ViewportManager.wait_for_viewport_async()
        self.assertTrue(status, f"Viewport not ready after {frames} frames")
        # test conditions
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        sources = [
            None,
            viewport_api,
            viewport_api.render_product_path,
            "Viewport",
            stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path),
            UsdRender.Product(stage_utils.get_current_stage().GetPrimAtPath(viewport_api.render_product_path)),
        ]
        resolutions = [(640, 480), (1280, 720)]
        # test cases
        for source in sources:
            for resolution in resolutions:
                ViewportManager.set_resolution(resolution, render_product_or_viewport=source)
                self.assertTupleEqual(
                    ViewportManager.get_resolution(source),
                    resolution,
                    (
                        f"Source: {source} (type: {type(source)},"
                        f" viewport: {ViewportManager.get_viewport_api(source)},"
                        f" render product: {ViewportManager.get_render_product(source)})"
                    ),
                )
        # exception
        with self.assertRaisesRegex(ValueError, "Unable to set resolution: unknown"):
            ViewportManager.set_resolution(resolution, render_product_or_viewport="/Invalid/Source")

    async def test_viewport_windows(self) -> None:
        """Create, list, filter, and destroy viewport windows by title patterns."""
        # test cases
        # - default window
        windows = ViewportManager.get_viewport_windows()
        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0].title, "Viewport")
        # - create viewport windows
        for i, camera in enumerate(
            [
                None,
                "/OmniverseKit_Persp",
                "/OmniverseKit_Top",
                "/OmniverseKit_Front",
                "/OmniverseKit_Right",
                stage_utils.define_prim("/CustomCamera", "Camera"),
            ]
        ):
            if camera is None:
                window = ViewportManager.create_viewport_window(title="Custom Title")
            else:
                window = ViewportManager.create_viewport_window(camera=camera)
            self.assertEqual(window.title, f"Viewport {i + 1}" if camera is not None else "Custom Title")
            self.assertEqual(
                window.viewport_api.camera_path,
                prim_utils.get_prim_path(camera) if camera is not None else "/OmniverseKit_Persp",
            )
        # - get viewport windows
        # -- all
        windows = ViewportManager.get_viewport_windows()
        self.assertEqual(len(windows), 7)
        self.assertListEqual(
            sorted([window.title for window in windows]),
            [
                "Custom Title",
                "Viewport",
                "Viewport 2",
                "Viewport 3",
                "Viewport 4",
                "Viewport 5",
                "Viewport 6",
            ],
        )
        # -- using regex
        windows = ViewportManager.get_viewport_windows(include=[".*[2-5]", "Custom Title"], exclude=["Viewport 4"])
        self.assertListEqual(
            [window.title for window in windows], ["Custom Title", "Viewport 2", "Viewport 3", "Viewport 5"]
        )
        # - destroy windows
        # -- using regex
        destroyed_window_titles = ViewportManager.destroy_viewport_windows(
            include=["Viewport", "Custom.*", "Viewport 5"], exclude=["Viewport 5"]
        )
        self.assertListEqual(destroyed_window_titles, ["Viewport", "Custom Title"])
        self.assertListEqual(
            [window.title for window in ViewportManager.get_viewport_windows()],
            ["Viewport 2", "Viewport 3", "Viewport 4", "Viewport 5", "Viewport 6"],
        )
        # -- all
        destroyed_window_titles = ViewportManager.destroy_viewport_windows()
        self.assertEqual(len(ViewportManager.get_viewport_windows()), 0)

    async def test_camera_view(self) -> None:
        """Set camera eye, target, center of interest, and collinear look directions."""

        def _check_camera(
            camera: Any,
            position: Any,
            orientation: Any,
            coi: Any = None,
            *,
            rtol: float = 1e-03,
            atol: float = 1e-05,
        ) -> None:
            pose = xform_utils.get_world_pose(camera)
            np.testing.assert_allclose(pose[0].numpy(), np.array(position), rtol=rtol, atol=atol)
            np.testing.assert_allclose(
                np.abs(np.dot(pose[1].numpy(), np.array(orientation))), 1.0, rtol=rtol, atol=atol
            )
            if coi is not None:
                prim = prim_utils.get_prim_at_path(camera)
                local_coi = prim.GetAttribute("omni:kit:centerOfInterest").Get()
                world_coi = (
                    UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(local_coi)
                )
                np.testing.assert_allclose(world_coi, np.array(coi), rtol=rtol, atol=atol)

        def _set_world_coi(camera: Any, target: Any) -> None:
            prim = prim_utils.get_prim_at_path(camera)
            world_inverse_transform = (
                UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
            )
            local_coi = world_inverse_transform.Transform(Gf.Vec3d(*target))
            prim.GetAttribute("omni:kit:centerOfInterest").Set(local_coi)

        def _check_world_look_at(camera: Any, eye: Any, target: Any) -> None:
            world_transform = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            actual_eye = np.array(world_transform.ExtractTranslation())
            actual_forward = np.array(world_transform.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)))
            actual_up = np.array(world_transform.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0)))

            expected_eye = np.asarray(eye)
            expected_forward = np.asarray(target) - expected_eye
            expected_forward /= np.linalg.norm(expected_forward)
            world_up = np.array([0.0, 0.0, 1.0] if stage_utils.get_stage_up_axis() == "Z" else [0.0, 1.0, 0.0])
            expected_up = world_up - expected_forward * np.dot(world_up, expected_forward)
            expected_up /= np.linalg.norm(expected_up)

            np.testing.assert_allclose(actual_eye, expected_eye, atol=1e-5)
            np.testing.assert_allclose(actual_forward / np.linalg.norm(actual_forward), expected_forward, atol=1e-5)
            np.testing.assert_allclose(actual_up / np.linalg.norm(actual_up), expected_up, atol=1e-5)

        def _reset_pose(prim: Any) -> None:
            omni.kit.commands.execute(
                "TransformPrimSRTCommand",
                path=prim_utils.get_prim_path(prim),
                new_translation=Gf.Vec3d(0, 0, 0),
                new_rotation_euler=Gf.Vec3d(0, 0, 0),
            )

        prim = stage_utils.define_prim("/Camera", "Camera")
        path = prim.GetPath().pathString
        camera = UsdGeom.Camera(prim)
        _reset_pose(prim)
        _check_camera(camera, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        # test cases
        # - no center of interest (COI)
        # -- no eye, no target
        ViewportManager.set_camera_view(prim)
        _check_camera(camera, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        # --no eye, target
        ViewportManager.set_camera_view(path, target=[1.0, 2.0, 3.0])
        _check_camera(camera, [0.0, 0.0, 0.0], [0.3063, 0.9237, -0.2180, -0.0723])
        # --eye, no target
        ViewportManager.set_camera_view(camera, eye=[-1.0, -2.0, -3.0])
        _check_camera(camera, [-1.0, -2.0, -3.0], [0.3063, 0.9237, -0.2180, -0.0723])
        # -- eye, target
        ViewportManager.set_camera_view(prim, eye=[1.1, -2.2, 3.3], target=[-4.4, 5.5, -6.6])
        _check_camera(camera, [1.1, -2.2, 3.3], [0.8838, 0.3544, 0.1136, 0.2832])
        # - center of interest (COI)
        _reset_pose(prim)
        attribute = prim_utils.create_prim_attribute(
            prim, name="omni:kit:centerOfInterest", type_name=Sdf.ValueTypeNames.Vector3d
        )
        attribute.Set(Gf.Vec3d(0.5, 2.1, -0.7))
        _check_camera(camera, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], coi=[0.5, 2.1, -0.7])
        # -- no eye, no target
        ViewportManager.set_camera_view(path)
        _check_camera(camera, [0.0, 0.0, 0.0], [0.8033, 0.5840, -0.0685, -0.0943], coi=[0.5, 2.1, -0.7])
        # --no eye, target
        # --- non-relative tracking
        ViewportManager.set_camera_view(camera, target=[1.0, 2.0, 3.0], relative_tracking=False)
        _check_camera(camera, [0.0, 0.0, 0.0], [0.3063, 0.9237, -0.2180, -0.0723], coi=[1.0, 2.0, 3.0])
        # --- relative tracking
        _set_world_coi(camera, [0.5, 2.1, -0.7])
        ViewportManager.set_camera_view(prim, target=[1.0, 2.0, 3.0], relative_tracking=True)
        _check_camera(camera, [0.5, -0.1, 3.7], [0.3063, 0.9237, -0.2180, -0.0723], coi=[1.0, 2.0, 3.0])
        # --eye, no target
        _set_world_coi(camera, [0.5, 2.1, -0.7])
        ViewportManager.set_camera_view(path, eye=[-1.0, -2.0, -3.0])
        _check_camera(camera, [-1.0, -2.0, -3.0], [0.5087, 0.8430, -0.1493, -0.0901], coi=[0.5, 2.1, -0.7])
        # -- eye, target
        ViewportManager.set_camera_view(camera, eye=[1.1, -2.2, 3.3], target=[-4.4, 5.5, -6.6])
        _check_camera(camera, [1.1, -2.2, 3.3], [0.8838, 0.3544, 0.1136, 0.2832], coi=[-4.4, 5.5, -6.6])
        # - special cases (collinearity)
        eye = [1.0, 1.0, 1.0]
        # -- X-forward
        ViewportManager.set_camera_view(camera, eye=eye, target=[2.0, 1.0, 1.0])
        _check_camera(camera, eye, [0.5, 0.5, -0.5, -0.5], coi=[2.0, 1.0, 1.0])
        # -- X-backward
        ViewportManager.set_camera_view(camera, eye=eye, target=[0.0, 1.0, 1.0])
        _check_camera(camera, eye, [0.5, 0.5, 0.5, 0.5], coi=[0.0, 1.0, 1.0])
        # -- Y-forward/up
        ViewportManager.set_camera_view(camera, eye=eye, target=[1.0, 2.0, 1.0])
        _check_camera(camera, eye, [0.707, 0.707, 0.0, 0.0], coi=[1.0, 2.0, 1.0])
        # -- Y-backward/down
        ViewportManager.set_camera_view(camera, eye=eye, target=[1.0, 0.0, 1.0])
        _check_camera(camera, eye, [0.0, 0.0, 0.707, 0.707], coi=[1.0, 0.0, 1.0])
        # -- Z-forward/up
        ViewportManager.set_camera_view(camera, eye=eye, target=[1.0, 1.0, 2.0])
        _check_camera(camera, eye, [0.0, -0.707, 0.707, 0.0], coi=[1.0, 1.0, 2.0])
        # -- Z-backward/down
        ViewportManager.set_camera_view(camera, eye=eye, target=[1.0, 1.0, 0.0])
        _check_camera(camera, eye, [0.707, 0.0, 0.0, -0.707], coi=[1.0, 1.0, 0.0])
        # -- same as eye
        ViewportManager.set_camera_view(camera, eye=eye, target=eye)
        _check_camera(camera, eye, [0.5, 0.5, -0.5, -0.5], coi=eye)
        # - parented camera (non-identity parent)
        stage = stage_utils.get_current_stage()
        translated_rig = UsdGeom.Xform.Define(stage, "/World/TranslatedRig")
        translated_rig.ClearXformOpOrder()
        translated_rig.AddTranslateOp().Set(Gf.Vec3d(100.0, 0.0, 0.0))
        translated_cam = UsdGeom.Camera.Define(stage, "/World/TranslatedRig/Cam")
        translated_cam.ClearXformOpOrder()
        translated_cam.AddTranslateOp().Set(Gf.Vec3d(5.0, 6.0, 7.0))
        before_target_only = xform_utils.get_world_pose(translated_cam.GetPrim())
        ViewportManager.set_camera_view(translated_cam.GetPrim(), target=[0.0, 0.0, 0.0])
        _check_world_look_at(translated_cam, before_target_only[0].numpy(), [0.0, 0.0, 0.0])
        ViewportManager.set_camera_view(translated_cam.GetPrim(), eye=[10.0, 2.0, 3.0], target=[0.0, 0.0, 0.0])
        _check_world_look_at(translated_cam, [10.0, 2.0, 3.0], [0.0, 0.0, 0.0])
        rotated_rig = UsdGeom.Xform.Define(stage, "/World/RotatedRig")
        rotated_rig.ClearXformOpOrder()
        rotated_rig.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 90.0))
        rotated_cam = UsdGeom.Camera.Define(stage, "/World/RotatedRig/Cam")
        rotated_cam.ClearXformOpOrder()
        rotated_cam.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
        ViewportManager.set_camera_view(rotated_cam.GetPrim(), eye=[10.0, 2.0, 3.0], target=[0.0, 0.0, 0.0])
        _check_world_look_at(rotated_cam, [10.0, 2.0, 3.0], [0.0, 0.0, 0.0])
        scaled_rig = UsdGeom.Xform.Define(stage, "/World/ScaledRig")
        scaled_rig.ClearXformOpOrder()
        scaled_rig.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
        scaled_cam = UsdGeom.Camera.Define(stage, "/World/ScaledRig/Cam")
        scaled_cam.ClearXformOpOrder()
        scaled_cam.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
        before_scaled_target_only = xform_utils.get_world_pose(scaled_cam.GetPrim())
        ViewportManager.set_camera_view(scaled_cam.GetPrim(), target=[0.0, 0.0, 0.0])
        _check_world_look_at(scaled_cam, before_scaled_target_only[0].numpy(), [0.0, 0.0, 0.0])
        ViewportManager.set_camera_view(scaled_cam.GetPrim(), eye=[10.0, 2.0, 3.0], target=[0.0, 0.0, 0.0])
        _check_world_look_at(scaled_cam, [10.0, 2.0, 3.0], [0.0, 0.0, 0.0])

    async def test_camera_view_center_of_interest_uses_camera_local_space(self) -> None:
        """Keep the Kit center-of-interest attribute local while accepting world coordinates."""

        def _create_coi(camera: UsdGeom.Camera, value: Gf.Vec3d) -> None:
            camera.GetPrim().CreateAttribute(
                "omni:kit:centerOfInterest",
                Sdf.ValueTypeNames.Vector3d,
                True,
                Sdf.VariabilityUniform,
            ).Set(value)

        def _get_world_coi(camera: UsdGeom.Camera) -> Gf.Vec3d:
            local_coi = camera.GetPrim().GetAttribute("omni:kit:centerOfInterest").Get()
            return camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(local_coi)

        def _get_world_rotation(camera: UsdGeom.Camera) -> Gf.Quatd:
            world_transform = Gf.Matrix4d(camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            world_transform.Orthonormalize()
            return world_transform.ExtractRotationQuat()

        def _assert_same_rotation(actual: Gf.Quatd, expected: Gf.Quatd, *, atol: float = 1e-6) -> None:
            dot = abs(actual.GetReal() * expected.GetReal() + Gf.Dot(actual.GetImaginary(), expected.GetImaginary()))
            np.testing.assert_allclose(dot, 1.0, atol=atol)

        stage = stage_utils.get_current_stage()
        target_values = [4.0, 1.0, -2.0]
        target = Gf.Vec3d(*target_values)

        # Compare a transformed camera directly with Kit's authoritative camera-state implementation.
        cameras = []
        for path in ("/World/Actual", "/World/Reference"):
            camera = UsdGeom.Camera.Define(stage, path)
            camera.ClearXformOpOrder()
            camera.AddTranslateOp().Set(Gf.Vec3d(2.0, -3.0, 6.0))
            camera.AddRotateXYZOp().Set(Gf.Vec3d(-15.0, 40.0, 25.0))
            _create_coi(camera, Gf.Vec3d(0.0, 0.0, -9.0))
            cameras.append(camera)
        ViewportManager.set_camera_view(cameras[0], target=target_values)
        ViewportCameraState(cameras[1].GetPath().pathString).set_target_world(target, True)
        actual_local_coi = cameras[0].GetPrim().GetAttribute("omni:kit:centerOfInterest").Get()
        reference_local_coi = cameras[1].GetPrim().GetAttribute("omni:kit:centerOfInterest").Get()
        np.testing.assert_allclose(actual_local_coi, reference_local_coi, atol=1e-5)
        np.testing.assert_allclose(_get_world_coi(cameras[0]), target, atol=1e-5)

        # Reading an already aligned local COI must not rotate the camera.
        camera = UsdGeom.Camera.Define(stage, "/World/NoArgument")
        camera.ClearXformOpOrder()
        camera.AddTranslateOp().Set(Gf.Vec3d(5.0, 5.0, 5.0))
        camera.AddRotateXYZOp().Set(Gf.Vec3d(54.73561, 0.0, 135.0))
        _create_coi(camera, Gf.Vec3d(0.0, 0.0, -8.660254))
        rotation_before = _get_world_rotation(camera)
        world_coi_before = _get_world_coi(camera)
        ViewportManager.set_camera_view(camera)
        _assert_same_rotation(_get_world_rotation(camera), rotation_before)
        np.testing.assert_allclose(_get_world_coi(camera), world_coi_before, atol=1e-5)

        # An eye-only update continues to aim at the same world-space COI.
        ViewportManager.set_camera_view(camera, eye=[1.0, 1.0, 1.0])
        world_transform = camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        eye = world_transform.ExtractTranslation()
        forward = Gf.Vec3d(world_transform.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))).GetNormalized()
        expected_forward = (world_coi_before - eye).GetNormalized()
        np.testing.assert_allclose(forward, expected_forward, atol=1e-5)
        np.testing.assert_allclose(_get_world_coi(camera), world_coi_before, atol=1e-5)

        # Relative tracking translates the existing eye-to-COI relationship without changing orientation or distance.
        camera = UsdGeom.Camera.Define(stage, "/World/Relative")
        camera.ClearXformOpOrder()
        camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 10.0))
        _create_coi(camera, Gf.Vec3d(0.0, 0.0, -10.0))
        rotation_before = _get_world_rotation(camera)
        ViewportManager.set_camera_view(camera, target=[5.0, 0.0, 0.0], relative_tracking=True)
        world_transform = camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        np.testing.assert_allclose(world_transform.ExtractTranslation(), [5.0, 0.0, 10.0], atol=1e-5)
        np.testing.assert_allclose(_get_world_coi(camera), [5.0, 0.0, 0.0], atol=1e-5)
        _assert_same_rotation(_get_world_rotation(camera), rotation_before)

        # Parent translation, rotation, and scale must not change the COI's camera-local convention.
        rig = UsdGeom.Xform.Define(stage, "/World/ScaledRigWithCoi")
        rig.ClearXformOpOrder()
        rig.AddTranslateOp().Set(Gf.Vec3d(7.0, -2.0, 5.0))
        rig.AddRotateXYZOp().Set(Gf.Vec3d(10.0, 35.0, -20.0))
        rig.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 2.0))
        camera = UsdGeom.Camera.Define(stage, "/World/ScaledRigWithCoi/Camera")
        camera.ClearXformOpOrder()
        camera.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
        _create_coi(camera, Gf.Vec3d(0.0, 0.0, -4.0))
        ViewportManager.set_camera_view(camera, target=target_values)
        np.testing.assert_allclose(_get_world_coi(camera), target, atol=1e-5)

        # An existing but unauthored COI must not block calls with enough explicit world-space information.
        camera = UsdGeom.Camera.Define(stage, "/World/UnauthoredTarget")
        camera.ClearXformOpOrder()
        camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        camera.GetPrim().CreateAttribute(
            "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
        )
        rotation_before = _get_world_rotation(camera)
        ViewportManager.set_camera_view(camera)
        _assert_same_rotation(_get_world_rotation(camera), rotation_before)
        ViewportManager.set_camera_view(camera, eye=[1.0, 2.0, 3.0])
        np.testing.assert_allclose(
            camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation(), [1.0, 2.0, 3.0]
        )
        self.assertIsNone(camera.GetPrim().GetAttribute("omni:kit:centerOfInterest").Get())
        ViewportManager.set_camera_view(camera, target=target_values)
        np.testing.assert_allclose(_get_world_coi(camera), target, atol=1e-5)

        camera = UsdGeom.Camera.Define(stage, "/World/UnauthoredEyeAndTarget")
        camera.ClearXformOpOrder()
        camera.GetPrim().CreateAttribute(
            "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
        )
        ViewportManager.set_camera_view(camera, eye=[-2.0, 3.0, 4.0], target=target_values)
        np.testing.assert_allclose(
            camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation(), [-2.0, 3.0, 4.0]
        )
        np.testing.assert_allclose(_get_world_coi(camera), target, atol=1e-5)
