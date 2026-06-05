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

"""Utilities for reading camera information and computing relative poses for ROS2 camera configurations."""

import carb
import cv2 as cv
import numpy as np
from isaacsim.core.experimental.utils import xform as xform_utils
from isaacsim.core.rendering_manager import ViewportManager
from pxr import Gf, Usd

# OpenCV distortion coefficient attribute names, matching the omni:lensdistortion schema.
OPENCV_PINHOLE_ATTRIBUTE_MAP = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]
OPENCV_FISHEYE_ATTRIBUTE_MAP = ["k1", "k2", "k3", "k4"]


def read_camera_info(render_product_path: str) -> tuple:
    """Reads camera prim attributes given render product path.

    Retrieves camera intrinsic parameters, distortion model, and distortion coefficients from the camera prim
    associated with the render product. Supports OpenCV Pinhole and OpenCV Fisheye distortion models, as well as
    legacy physical distortion models.

    Args:
        render_product_path: Path to the render product.

    Returns:
        A tuple containing the populated CameraInfo message and the camera prim object.
    """
    from sensor_msgs.msg import CameraInfo

    camera_info = CameraInfo()

    camera_prim = ViewportManager.get_camera(render_product_path).GetPrim()

    # Store CameraInfo distortion model and parameters
    lens_distortion_model = camera_prim.GetAttribute("omni:lensdistortion:model").Get()

    if lens_distortion_model == "opencvPinhole":

        usd_width, usd_height = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        cx = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:cx").Get()
        cy = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:cy").Get()
        fx = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:fx").Get()
        fy = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:fy").Get()
        pinhole = [0.0] * 12
        for i in range(12):
            pinhole[i] = camera_prim.GetAttribute(
                f"omni:lensdistortion:opencvPinhole:{OPENCV_PINHOLE_ATTRIBUTE_MAP[i]}"
            ).Get()

        # Scale intrinsics to the render-product resolution (distortion coefficients are scale-invariant).
        width, height = ViewportManager.get_resolution(render_product_path)
        if (width, height) != (usd_width, usd_height):
            sx, sy = width / usd_width, height / usd_height
            fx *= sx
            cx *= sx
            fy *= sy
            cy *= sy

        if pinhole[5:8] == [0.0] * 3:
            # Zeros provided for k4, k5, k6 coefficients
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = pinhole[:5]
        else:
            camera_info.distortion_model = "rational_polynomial"
            camera_info.d = pinhole
    elif lens_distortion_model == "opencvFisheye":
        usd_width, usd_height = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:imageSize").Get()
        cx = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:cx").Get()
        cy = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:cy").Get()
        fx = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:fx").Get()
        fy = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:fy").Get()
        fisheye = [0.0] * 4
        for i in range(4):
            fisheye[i] = camera_prim.GetAttribute(
                f"omni:lensdistortion:opencvFisheye:{OPENCV_FISHEYE_ATTRIBUTE_MAP[i]}"
            ).Get()

        # Scale intrinsics to the render-product resolution (distortion coefficients are scale-invariant).
        width, height = ViewportManager.get_resolution(render_product_path)
        if (width, height) != (usd_width, usd_height):
            sx, sy = width / usd_width, height / usd_height
            fx *= sx
            cx *= sx
            fy *= sy
            cy *= sy

        camera_info.distortion_model = "equidistant"
        camera_info.d = fisheye
    else:
        # No OmniLensDistortion schema — compute intrinsics from USD camera attributes
        if lens_distortion_model is not None:
            carb.log_error(
                f"Unsupported lens distortion model '{lens_distortion_model}'. "
                f"Only 'opencvPinhole' and 'opencvFisheye' are supported. "
                f"Legacy physical distortion models are no longer supported as of Isaac Sim 6.0. "
                f"Migrate to OmniLensDistortion schemas (OmniLensDistortionOpenCvPinholeAPI or OmniLensDistortionOpenCvFisheyeAPI)."
            )

        width, height = ViewportManager.get_resolution(render_product_path)
        focalLength = camera_prim.GetAttribute("focalLength").Get()
        horizontalAperture = camera_prim.GetAttribute("horizontalAperture").Get()
        verticalAperture = camera_prim.GetAttribute("verticalAperture").Get()

        fx = width * focalLength / horizontalAperture
        fy = height * focalLength / verticalAperture
        cx = width * 0.5
        cy = height * 0.5

        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5

    # Retrieve and store resolution
    camera_info.width = width
    camera_info.height = height
    # Set default intrinsic matrix (k)
    if fy != fx:
        carb.log_warn(f"Forcing fy to fx ({fy} != {fx}) when computing CameraInfo, as renderer assumes square pixels.")
        fy = fx
    camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    # Set default rectification matrix (r)
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # Set default projection matrix (p)
    camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    return camera_info, camera_prim


def compute_relative_pose(left_camera_prim: Usd.Prim, right_camera_prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    """Computes the relative pose between two camera prims for stereo camera configurations.

    Args:
        left_camera_prim: The left camera prim.
        right_camera_prim: The right camera prim.

    Returns:
        A tuple containing the translation vector and orientation (rotation matrix) from the left camera to the right
        camera.
    """
    # Compute relative transform -> translation, orientation
    relative_transform = xform_utils.get_relative_transform(source_prim=left_camera_prim, target_prim=right_camera_prim)
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(relative_transform)))
    rotation_vec = mat.GetRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    translation = np.array(mat.GetTranslation())
    orientation = np.ndarray(shape=[3, 3], dtype=float)
    cv.Rodrigues(src=np.asarray([rotation_vec[0], rotation_vec[1], rotation_vec[2]]), dst=orientation)

    return (translation, orientation)
