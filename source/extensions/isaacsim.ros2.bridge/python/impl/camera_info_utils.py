# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Tuple

import carb
import cv2 as cv
import numpy as np
import omni
import omni.syntheticdata
from isaacsim.core.utils.render_product import get_camera_prim_path, get_resolution
from isaacsim.core.utils.transformations import get_relative_transform
from isaacsim.sensors.camera.camera import OPENCV_FISHEYE_ATTRIBUTE_MAP, OPENCV_PINHOLE_ATTRIBUTE_MAP
from pxr import Gf, Usd


def read_camera_info(render_product_path: str) -> Tuple:
    """Reads camera prim attributes given render product path."""

    from sensor_msgs.msg import CameraInfo

    camera_info = CameraInfo()

    # Retrieve and store camera prim object
    camera_path = get_camera_prim_path(render_product_path=render_product_path)
    camera_prim = omni.usd.get_context().get_stage().GetPrimAtPath(camera_path)

    # Store CameraInfo distortion model and parameters
    lens_distortion_model = camera_prim.GetAttribute("omni:lensdistortion:model").Get()

    if lens_distortion_model == "opencvPinhole":

        width, height = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()
        cx = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:cx").Get()
        cy = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:cy").Get()
        fx = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:fx").Get()
        fy = camera_prim.GetAttribute("omni:lensdistortion:opencvPinhole:fy").Get()
        pinhole = [0.0] * 12
        for i in range(12):
            pinhole[i] = camera_prim.GetAttribute(
                f"omni:lensdistortion:opencvPinhole:{OPENCV_PINHOLE_ATTRIBUTE_MAP[i]}"
            ).Get()

        if pinhole[5:8] == [0.0] * 3:
            # Zeros provided for k4, k5, k6 coefficients
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = pinhole[:5]
        else:
            camera_info.distortion_model = "rational_polynomial"
            camera_info.d = pinhole
    elif lens_distortion_model == "opencvFisheye":
        width, height = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:imageSize").Get()
        cx = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:cx").Get()
        cy = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:cy").Get()
        fx = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:fx").Get()
        fy = camera_prim.GetAttribute("omni:lensdistortion:opencvFisheye:fy").Get()
        fisheye = [0.0] * 4
        for i in range(4):
            fisheye[i] = camera_prim.GetAttribute(
                f"omni:lensdistortion:opencvFisheye:{OPENCV_FISHEYE_ATTRIBUTE_MAP[i]}"
            ).Get()
        camera_info.distortion_model = "equidistant"
        camera_info.d = fisheye
    else:
        carb.log_warn(
            f"ROS2 CameraInfo support for lens distortion models beyond opencvPinhole and opencvFisheye is deprecated as of Isaac Sim 5.0, and will be removed in a future release."
        )

        width, height = get_resolution(render_product_path=render_product_path)
        focalLength = camera_prim.GetAttribute("focalLength").Get()
        horizontalAperture = camera_prim.GetAttribute("horizontalAperture").Get()

        # We assume that we have square pixels so fy = height * focalLength / (horizontalAperture * (height / width))
        fx = width * focalLength / horizontalAperture
        fy = height * focalLength / (horizontalAperture * (height / width))
        cx = width * 0.5
        cy = height * 0.5

        supported_physical_distortion_models = ["plumb_bob", "rational_polynomial", "equidistant"]
        physical_distortion = camera_prim.GetAttribute("physicalDistortionModel").Get()
        physical_distortion_coefs = camera_prim.GetAttribute("physicalDistortionCoefficients").Get()

        # Set default distortion model to plumb_bob without distortion
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        if physical_distortion not in supported_physical_distortion_models:
            carb.log_warn(
                f"Unsupported physical distortion model '{physical_distortion}'. Using plumb_bob with default coefficients."
            )
        elif physical_distortion == "plumb_bob" and physical_distortion_coefs != [0.0] * 5:
            carb.log_warn(
                f"Setting physical distortion coefficients to [0, 0, 0, 0, 0] for physicalDistortionModel == 'plumb_bob'."
            )
        else:
            camera_info.distortion_model = physical_distortion
            camera_info.d = list(physical_distortion_coefs)

    # Retrieve and store resolution
    camera_info.width = width
    camera_info.height = height
    # Set default intrinsic matrix (k)
    camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    # Set default rectification matrix (r)
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # Set default projection matrix (p)
    camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    return camera_info, camera_prim


def compute_relative_pose(left_camera_prim: Usd.Prim, right_camera_prim: Usd.Prim) -> Tuple[np.ndarray, np.ndarray]:

    # Compute relative transform -> translation, orientation
    relative_transform = get_relative_transform(target_prim=right_camera_prim, source_prim=left_camera_prim)
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(relative_transform)))
    rotation_vec = mat.GetRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    translation = np.array(mat.GetTranslation())
    orientation = np.ndarray(shape=[3, 3], dtype=float)
    cv.Rodrigues(src=np.asarray([rotation_vec[0], rotation_vec[1], rotation_vec[2]]), dst=orientation)

    return (translation, orientation)
