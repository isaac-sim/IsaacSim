# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Validate ROS 2 camera info utilities against RTX camera render products.

The tests cover pinhole and fisheye camera models, OpenCV distortion fields,
render-product scaling, unsupported distortion fallback, and stereo relative
pose computation.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.camera_info_utils import compute_relative_pose, read_camera_info
from isaacsim.sensors.experimental.rtx import RtxCamera
from pxr import Gf, Sdf, UsdGeom

from .common import ROS2TestCase

_PINHOLE_COEFF_NAMES = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]
_FISHEYE_COEFF_NAMES = ["k1", "k2", "k3", "k4"]


def _apply_opencv_distortion(
    prim: Any, model: str, coefficients: list[float], coeff_names: list[str], image_size: tuple[int, int], **kwargs: Any
) -> None:
    """Author OpenCV pinhole or fisheye distortion attributes on a USD camera prim.

    Args:
        prim: USD camera prim to author.
        model: OpenCV distortion model token.
        coefficients: Distortion coefficient values.
        coeff_names: Attribute names corresponding to ``coefficients``.
        image_size: Distortion image size as width and height.
        **kwargs: Additional distortion attributes to author.
    """
    schema = f"OmniLensDistortionOpenCv{'Pinhole' if model == 'opencvPinhole' else 'Fisheye'}API"
    prim.ApplyAPI(schema)
    prim.GetAttribute("omni:lensdistortion:model").Set(model)
    prim.GetAttribute(f"omni:lensdistortion:{model}:imageSize").Set(Gf.Vec2i(image_size[0], image_size[1]))
    for i, val in enumerate(coefficients):
        prim.GetAttribute(f"omni:lensdistortion:{model}:{coeff_names[i]}").Set(float(val))
    for attr_name, attr_value in kwargs.items():
        prim.GetAttribute(f"omni:lensdistortion:{model}:{attr_name}").Set(float(attr_value))


class TestCameraInfoUtils(ROS2TestCase):
    """Exercise camera info conversion from Isaac Sim camera prims to ROS 2 messages."""

    async def setUp(self) -> None:
        """Create a fresh stage with an RTX camera, render product, and running timeline."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        self._stage = stage_utils.get_current_stage()
        self._width, self._height = 1280, 720

        camera_path = "/test_camera"
        self._rtx_camera = RtxCamera(
            camera_path,
            positions=[[0, 0, 100]],
            orientations=[[1, 0, 0, 0]],
        )

        self._render_product = rep.create.render_product(camera_path, resolution=(self._width, self._height))
        self._render_product_path = self._render_product.path
        self.assertIsNotNone(self._render_product_path)

        self._camera_prim = self._stage.GetPrimAtPath(camera_path)
        self.assertIsNotNone(self._camera_prim)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop simulation and replace the stage so camera/render-product state does not leak."""
        self._timeline.stop()
        await stage_utils.create_new_stage_async()
        await super().tearDown()

    async def test_read_camera_info_pinhole(self) -> None:
        """Verify default pinhole camera info has expected dimensions, matrices, and identity rectification."""
        camera_info, camera_prim = read_camera_info(self._render_product_path)

        self.assertIsNotNone(camera_info)
        self.assertIsNotNone(camera_prim)
        self.assertEqual(camera_info.width, self._width)
        self.assertEqual(camera_info.height, self._height)

        self.assertEqual(len(camera_info.k), 9)
        self.assertAlmostEqual(camera_info.k[0], camera_info.p[0])  # fx
        self.assertAlmostEqual(camera_info.k[2], camera_info.p[2])  # cx
        self.assertAlmostEqual(camera_info.k[4], camera_info.p[5])  # fy
        self.assertAlmostEqual(camera_info.k[5], camera_info.p[6])  # cy
        self.assertEqual(camera_info.k[8], 1.0)

        expected_identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.assertTrue(all(a == e for a, e in zip(camera_info.r, expected_identity)))

        self.assertEqual(len(camera_info.p), 12)
        self.assertAlmostEqual(camera_info.p[2], self._width * 0.5, delta=1.0)
        self.assertAlmostEqual(camera_info.p[6], self._height * 0.5, delta=1.0)

    async def test_read_camera_info_fisheye_unset_distortion(self) -> None:
        """Test that a camera with FTheta schema but no distortion attributes defaults to plumb_bob."""
        self._camera_prim.ApplyAPI("OmniLensDistortionFthetaAPI")
        UsdGeom.Camera(self._camera_prim).GetFocalLengthAttr().Set(24.0)
        UsdGeom.Camera(self._camera_prim).GetHorizontalApertureAttr().Set(36.0)
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(self._render_product_path)

        self.assertIsNotNone(camera_info)
        self.assertEqual(camera_info.distortion_model, "plumb_bob")
        self.assertEqual(len(camera_info.d), 5)
        self.assertTrue(all(coef == 0.0 for coef in camera_info.d))

    async def test_read_camera_info_fisheye_legacy_distortion_ignored(self) -> None:
        """Test that legacy physicalDistortionModel attributes are ignored (defaults to plumb_bob)."""
        self._camera_prim.ApplyAPI("OmniLensDistortionFthetaAPI")
        UsdGeom.Camera(self._camera_prim).GetFocalLengthAttr().Set(24.0)
        UsdGeom.Camera(self._camera_prim).GetHorizontalApertureAttr().Set(36.0)

        # Legacy attributes — should be ignored by read_camera_info
        self._camera_prim.CreateAttribute("physicalDistortionModel", Sdf.ValueTypeNames.Token).Set(
            "rational_polynomial"
        )
        self._camera_prim.CreateAttribute("physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray).Set(
            [0.1, 0.2, 0.01, 0.02, 0.003, 0.004, 0.0005, 0.0006]
        )
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(self._render_product_path)

        self.assertIsNotNone(camera_info)
        # Legacy distortion is no longer read — falls through to plumb_bob default
        self.assertEqual(camera_info.distortion_model, "plumb_bob")
        self.assertEqual(len(camera_info.d), 5)
        self.assertTrue(all(coef == 0.0 for coef in camera_info.d))

    async def test_read_camera_info_unsupported_distortion_model(self) -> None:
        """Test that an unsupported omni:lensdistortion:model falls back to plumb_bob."""
        self._camera_prim.ApplyAPI("OmniLensDistortionFthetaAPI")
        UsdGeom.Camera(self._camera_prim).GetFocalLengthAttr().Set(24.0)
        UsdGeom.Camera(self._camera_prim).GetHorizontalApertureAttr().Set(36.0)
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(self._render_product_path)

        self.assertIsNotNone(camera_info)
        self.assertEqual(camera_info.distortion_model, "plumb_bob")
        self.assertEqual(len(camera_info.d), 5)
        self.assertTrue(all(coef == 0.0 for coef in camera_info.d))

    async def test_read_camera_info_opencv_pinhole_distortion(self) -> None:
        """Verify OpenCV pinhole distortion maps to plumb_bob and rational_polynomial coefficients."""
        cam = UsdGeom.Camera(self._camera_prim)
        focal_length = cam.GetFocalLengthAttr().Get()
        h_aperture = cam.GetHorizontalApertureAttr().Get()
        v_aperture = cam.GetVerticalApertureAttr().Get()

        cx, cy = self._width / 2, self._height / 2
        fx = self._width * focal_length / h_aperture
        fy = self._height * focal_length / v_aperture

        # 5-parameter plumb_bob
        k1, k2, p1, p2, k3 = 0.1, 0.05, 0.01, 0.02, 0.003
        params_5 = [k1, k2, p1, p2, k3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        _apply_opencv_distortion(
            self._camera_prim,
            "opencvPinhole",
            params_5,
            _PINHOLE_COEFF_NAMES,
            (self._width, self._height),
            cx=cx,
            cy=cy,
            fx=fx,
            fy=fy,
        )
        await omni.kit.app.get_app().next_update_async()

        camera_info_5, _ = read_camera_info(self._render_product_path)
        self.assertIsNotNone(camera_info_5)
        self.assertEqual(camera_info_5.distortion_model, "plumb_bob")
        expected_5 = [k1, k2, p1, p2, k3]
        self.assertEqual(len(camera_info_5.d), len(expected_5))
        for i, (e, a) in enumerate(zip(expected_5, camera_info_5.d)):
            self.assertAlmostEqual(a, e, delta=1e-5, msg=f"Coefficient {i} mismatch (5 params)")

        # 12-parameter rational_polynomial
        k4, k5, k6 = 0.0015, 0.0008, 0.0004
        s1, s2, s3, s4 = 0.0003, 0.0002, 0.0001, 0.00005
        params_12 = [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
        _apply_opencv_distortion(
            self._camera_prim,
            "opencvPinhole",
            params_12,
            _PINHOLE_COEFF_NAMES,
            (self._width, self._height),
            cx=cx,
            cy=cy,
            fx=fx,
            fy=fy,
        )
        await omni.kit.app.get_app().next_update_async()

        camera_info_12, _ = read_camera_info(self._render_product_path)
        self.assertIsNotNone(camera_info_12)
        self.assertEqual(camera_info_12.distortion_model, "rational_polynomial")
        self.assertEqual(len(camera_info_12.d), 12)
        for i, (e, a) in enumerate(zip(params_12, camera_info_12.d)):
            self.assertAlmostEqual(a, e, delta=1e-5, msg=f"Coefficient {i} mismatch (12 params)")

    async def test_read_camera_info_opencv_fisheye_distortion(self) -> None:
        """Verify OpenCV fisheye distortion maps to the ROS 2 equidistant model."""
        cam = UsdGeom.Camera(self._camera_prim)
        focal_length = cam.GetFocalLengthAttr().Get()
        h_aperture = cam.GetHorizontalApertureAttr().Get()
        v_aperture = cam.GetVerticalApertureAttr().Get()

        cx, cy = self._width / 2, self._height / 2
        fx = self._width * focal_length / h_aperture
        fy = self._height * focal_length / v_aperture

        k1, k2, k3, k4 = 0.1, 0.05, 0.01, 0.002
        _apply_opencv_distortion(
            self._camera_prim,
            "opencvFisheye",
            [k1, k2, k3, k4],
            _FISHEYE_COEFF_NAMES,
            (self._width, self._height),
            cx=cx,
            cy=cy,
            fx=fx,
            fy=fy,
        )
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(self._render_product_path)
        self.assertIsNotNone(camera_info)
        self.assertEqual(camera_info.distortion_model, "equidistant")
        expected = [k1, k2, k3, k4]
        self.assertEqual(len(camera_info.d), len(expected))
        for i, (e, a) in enumerate(zip(expected, camera_info.d)):
            self.assertAlmostEqual(a, e, delta=1e-5, msg=f"Coefficient {i} mismatch")

    async def test_read_camera_info_opencv_pinhole_scales_to_render_product(self) -> None:
        """OpenCV pinhole intrinsics should scale when render product differs from authored imageSize (NVBug 6039737)."""
        usd_width, usd_height = self._width, self._height
        usd_cx, usd_cy = usd_width / 2, usd_height / 2
        horizontal_aperture, vertical_aperture = self._rtx_camera.camera.get_apertures()
        horizontal_aperture = horizontal_aperture.numpy().item()
        vertical_aperture = vertical_aperture.numpy().item()
        focal_length = self._rtx_camera.camera.get_focal_lengths().numpy().item()
        usd_fx = usd_width * focal_length / horizontal_aperture
        usd_fy = usd_height * focal_length / vertical_aperture

        # Author OpenCV pinhole params at the current (USD) resolution.
        _apply_opencv_distortion(
            self._camera_prim,
            "opencvPinhole",
            [0.1, 0.05, 0.01, 0.02, 0.003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            _PINHOLE_COEFF_NAMES,
            (usd_width, usd_height),
            cx=usd_cx,
            cy=usd_cy,
            fx=usd_fx,
            fy=usd_fy,
        )
        await omni.kit.app.get_app().next_update_async()

        # Create a render product at a different resolution; the USD imageSize attribute stays at the authored value.
        rp_width, rp_height = 640, 480
        scaled_rp = rep.create.render_product("/test_camera", resolution=(rp_width, rp_height))
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(scaled_rp.path)

        sx = rp_width / usd_width
        sy = rp_height / usd_height
        self.assertEqual(camera_info.width, rp_width)
        self.assertEqual(camera_info.height, rp_height)
        self.assertAlmostEqual(camera_info.k[0], usd_fx * sx, delta=1e-3)  # fx
        self.assertAlmostEqual(camera_info.k[2], usd_cx * sx, delta=1e-3)  # cx
        # read_camera_info forces fy to fx when square pixels are maintained, so compare against fx*sx.
        self.assertAlmostEqual(camera_info.k[4], usd_fx * sx, delta=1e-3)  # fy (forced square)
        self.assertAlmostEqual(camera_info.k[5], usd_cy * sy, delta=1e-3)  # cy

    async def test_read_camera_info_opencv_fisheye_scales_to_render_product(self) -> None:
        """OpenCV fisheye intrinsics should scale when render product differs from authored imageSize (NVBug 6039737)."""
        usd_width, usd_height = self._width, self._height
        usd_cx, usd_cy = usd_width / 2, usd_height / 2
        horizontal_aperture, vertical_aperture = self._rtx_camera.camera.get_apertures()
        horizontal_aperture = horizontal_aperture.numpy().item()
        vertical_aperture = vertical_aperture.numpy().item()
        focal_length = self._rtx_camera.camera.get_focal_lengths().numpy().item()
        usd_fx = usd_width * focal_length / horizontal_aperture
        usd_fy = usd_height * focal_length / vertical_aperture

        _apply_opencv_distortion(
            self._camera_prim,
            "opencvFisheye",
            [0.1, 0.05, 0.01, 0.002],
            _FISHEYE_COEFF_NAMES,
            (usd_width, usd_height),
            cx=usd_cx,
            cy=usd_cy,
            fx=usd_fx,
            fy=usd_fy,
        )
        await omni.kit.app.get_app().next_update_async()

        rp_width, rp_height = 640, 480
        scaled_rp = rep.create.render_product("/test_camera", resolution=(rp_width, rp_height))
        await omni.kit.app.get_app().next_update_async()

        camera_info, _ = read_camera_info(scaled_rp.path)

        sx = rp_width / usd_width
        sy = rp_height / usd_height
        self.assertEqual(camera_info.width, rp_width)
        self.assertEqual(camera_info.height, rp_height)
        self.assertAlmostEqual(camera_info.k[0], usd_fx * sx, delta=1e-3)  # fx
        self.assertAlmostEqual(camera_info.k[2], usd_cx * sx, delta=1e-3)  # cx
        self.assertAlmostEqual(camera_info.k[4], usd_fx * sx, delta=1e-3)  # fy (forced square)
        self.assertAlmostEqual(camera_info.k[5], usd_cy * sy, delta=1e-3)  # cy

    async def test_compute_relative_pose(self) -> None:
        """Verify relative pose recovers the 10 cm baseline between two unrotated cameras."""
        RtxCamera("/left_camera", positions=[[0, 0, 0]], orientations=[[1, 0, 0, 0]])
        RtxCamera("/right_camera", positions=[[0.1, 0, 0]], orientations=[[1, 0, 0, 0]])
        await omni.kit.app.get_app().next_update_async()

        translation, orientation = compute_relative_pose(
            left_camera_prim=self._stage.GetPrimAtPath("/left_camera"),
            right_camera_prim=self._stage.GetPrimAtPath("/right_camera"),
        )

        self.assertIsNotNone(translation)
        self.assertIsNotNone(orientation)

        # 10cm offset should appear in the relative translation
        total_offset = np.sqrt(translation[0] ** 2 + translation[1] ** 2 + translation[2] ** 2)
        self.assertAlmostEqual(total_offset, 0.1, delta=1e-4)

        # No rotation between cameras
        np.testing.assert_array_almost_equal(orientation, np.eye(3), decimal=5)
