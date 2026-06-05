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

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
import omni.kit.test
from isaacsim.core.rendering_manager import ViewportManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

_SETTING_RATE_LIMIT_ENABLED = "/app/runLoops/main/rateLimitEnabled"


class TestViewportManager(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        await stage_utils.create_new_stage_async()
        # ---------------

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        # ------------------
        stage_utils.close_stage()
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_00_wait_for_viewport(self):  # 00 ensures that this test is run first
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

    async def test_set_camera(self):
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

    async def test_get_camera(self):
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

    async def test_get_viewport_and_render_product(self):
        def _check_viewport(source):
            self.assertIn("ViewportAPI", ViewportManager.get_viewport_api(source).__class__.__name__)

        def _check_render_product(source):
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

    async def test_get_resolution(self):
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

    async def test_set_resolution(self):
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

    async def test_viewport_windows(self):
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

    async def test_camera_view(self):
        def _check_camera(
            camera,
            position,
            orientation,
            coi=None,
            *,
            rtol: float = 1e-03,
            atol: float = 1e-05,
        ):
            pose = xform_utils.get_world_pose(camera)
            np.testing.assert_allclose(pose[0].numpy(), np.array(position), rtol=rtol, atol=atol)
            np.testing.assert_allclose(
                np.abs(np.dot(pose[1].numpy(), np.array(orientation))), 1.0, rtol=rtol, atol=atol
            )
            if coi is not None:
                attribute = prim_utils.get_prim_at_path(camera).GetAttribute("omni:kit:centerOfInterest")
                np.testing.assert_allclose(attribute.Get(), np.array(coi), rtol=rtol, atol=atol)

        def _reset_pose(prim):
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
        attribute.Set(Gf.Vec3d(0.5, 2.1, -0.7))  # reset COI
        ViewportManager.set_camera_view(prim, target=[1.0, 2.0, 3.0], relative_tracking=True)
        _check_camera(camera, [0.5, -0.1, 3.7], [0.8033, 0.5840, -0.0685, -0.0943], coi=[1.0, 2.0, 3.0])
        # --eye, no target
        attribute.Set(Gf.Vec3d(0.5, 2.1, -0.7))  # reset COI
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
