# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import random
import sys
from typing import List, Tuple

import carb
import cv2
import numpy as np
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, open_stage_async
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdLux

from .common import ROS2TestCase, get_qos_profile


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2CameraInfo(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        omni.usd.get_context().new_stage()
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        await omni.kit.app.get_app().next_update_async()

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):

        await super().tearDown()

    def imgmsg_to_cv2(self, img_msg):
        # encoding for RGB images is Type_RGB8 (token = rgb8) by default
        # dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = "uint8"
        n_channels = 3
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, int(img_msg.step / dtype.itemsize)), dtype=dtype, buffer=img_buf)
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width])
        else:
            im = np.ndarray(
                shape=(img_msg.height, int(img_msg.step / dtype.itemsize / n_channels), n_channels),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        return im

    async def test_monocular_camera_info(self):
        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await open_stage_async(self._assets_root_path + scene_path)

        camera_path = "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
        add_reference_to_stage(usd_path=self._assets_root_path + camera_path, prim_path="/Hawk")
        import rclpy

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CreateRenderProduct.inputs:cameraPrim", [Sdf.Path("/Hawk/left/camera_left")]),
                        ("CreateRenderProduct.inputs:height", 1200),
                        ("CreateRenderProduct.inputs:width", 1920),
                        ("CameraInfoPublish.inputs:topicName", "camera_info"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import CameraInfo

        self._camera_info = None

        def camera_info_callback(data):
            self._camera_info = data

        node = rclpy.create_node("camera_tester")
        camera_info_sub = node.create_subscription(CameraInfo, "camera_info", camera_info_callback, get_qos_profile())

        await omni.kit.app.get_app().next_update_async()

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        import time

        system_time = time.time()

        for num in range(5):
            print(f"Play #{num+1}")
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await simulate_async(1.5, callback=spin)
            for _ in range(10):
                if self._camera_info is None:
                    await simulate_async(1, callback=spin)

            self.assertIsNotNone(self._camera_info)

            self.assertEqual(self._camera_info.width, 1920)
            self.assertEqual(self._camera_info.height, 1200)
            self.assertGreaterEqual(self._camera_info.header.stamp.sec, 1)
            self.assertLess(self._camera_info.header.stamp.sec, system_time / 2.0)

            # Test contents of k matrix (function of width, height, focal length, apertures)
            self.assertAlmostEqual(self._camera_info.k[0], 1920.0 * 2.87343 / 5.76, places=2)
            self.assertAlmostEqual(self._camera_info.k[1], 0.0)
            self.assertAlmostEqual(self._camera_info.k[2], 1920.0 * 0.5)
            self.assertAlmostEqual(self._camera_info.k[3], 0.0)
            self.assertAlmostEqual(self._camera_info.k[4], 1200.0 * 2.87343 / 3.6, places=2)
            self.assertAlmostEqual(self._camera_info.k[5], 1200.0 * 0.5)
            self.assertAlmostEqual(self._camera_info.k[6], 0.0)
            self.assertAlmostEqual(self._camera_info.k[7], 0.0)
            self.assertAlmostEqual(self._camera_info.k[8], 1.0)

            # Test if r matrix is identity
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(self._camera_info.r[i * 3 + j], 1.0 if i == j else 0.0)

            # Test if p matrix is k matrix concatenated with 1x3 0 vector
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(self._camera_info.p[i * 4 + j], self._camera_info.k[i * 3 + j])
                self.assertAlmostEqual(self._camera_info.p[i * 4 + 3], 0.0)

            # Test distortion model and coefficients
            self.assertEqual(self._camera_info.distortion_model, "rational_polynomial")
            distortion_coefficients = [0.147811, -0.032313, -0.000194, -0.000035, 0.008823, 0.517913, -0.06708, 0.01695]
            for i in range(len(distortion_coefficients)):
                self.assertAlmostEqual(self._camera_info.d[i], distortion_coefficients[i])
            self._timeline.stop()

            # make sure all previous messages are cleared
            await omni.kit.app.get_app().next_update_async()
            spin()
            await omni.kit.app.get_app().next_update_async()
            self._camera_info = None

    def _add_light(self, name: str, position: List[float]) -> None:
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path(f"/World/SphereLight_{name}"))
        sphereLight.CreateRadiusAttr(6)
        sphereLight.CreateIntensityAttr(10000)
        SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose(position)

    def _add_checkerboard(self, position: List[float]) -> None:
        checkerboard_path = self._assets_root_path + "/Isaac/Props/Camera/checkerboard_6x10.usd"
        add_reference_to_stage(usd_path=checkerboard_path, prim_path="/calibration_target")
        SingleXFormPrim("/calibration_target", name="calibration_target", position=position)

    def _get_rectified_image(self, image_msg_raw, camera_info_msg, side):
        # Convert ROS2 image message data buffer to CV2 image
        image_raw = self.imgmsg_to_cv2(image_msg_raw)
        if self._visualize:
            cv2.imwrite(f"{side}_image_raw.png", image_raw)
        # Initialize the mapping arrays to rectify the raw image
        k = np.reshape(np.array(camera_info_msg.k), (3, 3))
        r = np.reshape(np.array(camera_info_msg.r), (3, 3))
        p = np.reshape(np.array(camera_info_msg.p), (3, 4))
        if camera_info_msg.distortion_model == "equidistant":
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K=k,
                D=np.array(camera_info_msg.d),
                R=r,
                P=p,
                size=(camera_info_msg.width, camera_info_msg.height),
                m1type=cv2.CV_32FC1,
            )
            return cv2.remap(src=image_raw, map1=map1, map2=map2, interpolation=cv2.INTER_LANCZOS4)
        else:
            map1, map2 = cv2.initUndistortRectifyMap(
                cameraMatrix=k,
                distCoeffs=np.array(camera_info_msg.d),
                R=r,
                newCameraMatrix=p,
                size=(camera_info_msg.width, camera_info_msg.height),
                m1type=cv2.CV_32FC1,
            )
            return cv2.remap(src=image_raw, map1=map1, map2=map2, interpolation=cv2.INTER_LANCZOS4)

    def _prepare_scene_for_stereo_camera(
        self, baseline: float, resolution: Tuple[int, int], focal_length: float, focus_distance: float
    ) -> Tuple[Camera, Camera]:
        """Add a stereo camera, checkerboard, and lights to the scene

        Args:
            baseline (float): Baseline distance between the two cameras
            resolution (Tuple[int, int]): Resolution of the cameras
            focal_length (float): Focal length of the cameras
            focus_distance (float): Focus distance of the cameras

        Returns:
            Tuple[Camera, Camera]: The left and right cameras
        """

        left_camera = Camera(
            prim_path="/left_camera",
            name="left_camera",
            resolution=resolution,
            position=np.array([0, baseline / 2.0, 0]),
        )
        left_camera.set_focal_length(focal_length)
        left_camera.set_focus_distance(focus_distance)

        right_camera = Camera(
            prim_path="/right_camera",
            name="right_camera",
            resolution=resolution,
            position=np.array([0, -baseline / 2.0, 0]),
        )
        right_camera.set_focal_length(focal_length)
        right_camera.set_focus_distance(focus_distance)

        # Create a light
        self._add_light(name="top", position=[0.0, 0.0, 12])
        self._add_light(name="bottom", position=[0.0, 0.0, -12])

        # Create a checkerboard
        self._add_checkerboard(position=[1.1, 0.0, -0.6])

        # Add an OmniGraph to publish the camera info and images
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("CreateRenderProductLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("CreateRenderProductRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("RGBPublishLeft", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("RGBPublishRight", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CreateRenderProductLeft.inputs:cameraPrim", [Sdf.Path("/left_camera")]),
                        ("CreateRenderProductLeft.inputs:height", 1024),
                        ("CreateRenderProductLeft.inputs:width", 2048),
                        ("CreateRenderProductRight.inputs:cameraPrim", [Sdf.Path("/right_camera")]),
                        ("CreateRenderProductRight.inputs:height", 1024),
                        ("CreateRenderProductRight.inputs:width", 2048),
                        ("CameraInfoPublish.inputs:topicName", "camera_info_left"),
                        ("CameraInfoPublish.inputs:topicNameRight", "camera_info_right"),
                        ("CameraInfoPublish.inputs:frameId", "frame_left"),
                        ("CameraInfoPublish.inputs:frameIdRight", "frame_right"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                        ("RGBPublishLeft.inputs:topicName", "rgb_left"),
                        ("RGBPublishLeft.inputs:type", "rgb"),
                        ("RGBPublishLeft.inputs:resetSimulationTimeOnStop", True),
                        ("RGBPublishRight.inputs:topicName", "rgb_right"),
                        ("RGBPublishRight.inputs:type", "rgb"),
                        ("RGBPublishRight.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                        ("RunOneSimulationFrame.outputs:step", "CreateRenderProductLeft.inputs:execIn"),
                        ("RunOneSimulationFrame.outputs:step", "CreateRenderProductRight.inputs:execIn"),
                        ("CreateRenderProductLeft.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        (
                            "CreateRenderProductLeft.outputs:renderProductPath",
                            "CameraInfoPublish.inputs:renderProductPath",
                        ),
                        ("CreateRenderProductRight.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        (
                            "CreateRenderProductRight.outputs:renderProductPath",
                            "CameraInfoPublish.inputs:renderProductPathRight",
                        ),
                        ("CreateRenderProductLeft.outputs:execOut", "RGBPublishLeft.inputs:execIn"),
                        (
                            "CreateRenderProductLeft.outputs:renderProductPath",
                            "RGBPublishLeft.inputs:renderProductPath",
                        ),
                        ("CreateRenderProductRight.outputs:execOut", "RGBPublishRight.inputs:execIn"),
                        (
                            "CreateRenderProductRight.outputs:renderProductPath",
                            "RGBPublishRight.inputs:renderProductPath",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

        return left_camera, right_camera

    async def _test_get_stereo_camera_messages(
        self, opencv_distortion_model: str, ros2_distortion_model: str, distortion_coefficients: List[float]
    ):
        """Get the camera info and images from the stereo camera

        Args:
            opencv_distortion_model (str): OpenCV distortion model to test.
            ros2_distortion_model (str): ROS2 distortion model to test.
            distortion_coefficients (List[float]): Distortion coefficients to test.
        """

        import rclpy
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image

        if not rclpy.ok():
            rclpy.init()

        # Create ROS nodes to receive camera data
        node_left = rclpy.create_node("camera_left_node")
        node_right = rclpy.create_node("camera_right_node")

        # Set up QoS profile matching the publisher
        cam_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        # Subscribe to camera topics
        def camera_info_left_callback(msg):
            self._camera_info_left = msg

        def camera_info_right_callback(msg):
            self._camera_info_right = msg

        def image_left_callback(msg):
            self._image_left = msg

        def image_right_callback(msg):
            self._image_right = msg

        # Create subscriptions
        camera_info_left_sub = node_left.create_subscription(
            CameraInfo, "camera_info_left", camera_info_left_callback, cam_info_qos
        )
        camera_info_right_sub = node_right.create_subscription(
            CameraInfo, "camera_info_right", camera_info_right_callback, cam_info_qos
        )
        image_left_sub = node_left.create_subscription(Image, "rgb_left", image_left_callback, cam_info_qos)
        image_right_sub = node_right.create_subscription(Image, "rgb_right", image_right_callback, cam_info_qos)

        # Start spinning the nodes
        def spin_left():
            rclpy.spin_once(node_left, timeout_sec=0.1)

        def spin_right():
            rclpy.spin_once(node_right, timeout_sec=0.1)

        # Wait for camera info and images to be received
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin_right)
        await simulate_async(1.5, callback=spin_left)

        self.assertIsNotNone(self._camera_info_left, f"Did not receive left camera_info for {opencv_distortion_model}")
        self.assertIsNotNone(
            self._camera_info_right, f"Did not receive right camera_info for {opencv_distortion_model}"
        )
        self.assertIsNotNone(self._image_left, f"Did not receive left image for {opencv_distortion_model}")
        self.assertIsNotNone(self._image_right, f"Did not receive right image for {opencv_distortion_model}")

        # Check CameraInfo distortion model and distortion coefficients
        self.assertEqual(self._camera_info_left.distortion_model, ros2_distortion_model)
        self.assertEqual(self._camera_info_right.distortion_model, ros2_distortion_model)
        for i, (expected, actual_left, actual_right) in enumerate(
            zip(distortion_coefficients, self._camera_info_left.d, self._camera_info_right.d)
        ):
            self.assertAlmostEqual(
                actual_left, expected, delta=1e-5, msg=f"Left coefficient {i} mismatch for {opencv_distortion_model}"
            )
            self.assertAlmostEqual(
                actual_right, expected, delta=1e-5, msg=f"Right coefficient {i} mismatch for {opencv_distortion_model}"
            )

        # Stop timeline
        self._timeline.stop()

    async def _test_stereo_rectification(self, opencv_distortion_model):
        """Test stereo rectification

        Args:
            opencv_distortion_model (str): OpenCV distortion model to test.
        """
        left_image_rect = self._get_rectified_image(self._image_left, self._camera_info_left, "left")
        right_image_rect = self._get_rectified_image(self._image_right, self._camera_info_right, "right")
        cv2.imwrite("left_image_rect.png", left_image_rect)
        cv2.imwrite("right_image_rect.png", right_image_rect)

        # Find checkerboard corners
        checkerboard_size = (6, 10)  # Internal corners on the checkerboard
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

        ret_left, corners_left = cv2.findChessboardCorners(left_image_rect, checkerboard_size, flags)
        ret_right, corners_right = cv2.findChessboardCorners(right_image_rect, checkerboard_size, flags)

        # Verify corners were found in both images
        self.assertTrue(ret_left, f"Could not find checkerboard corners in left image for {opencv_distortion_model}")
        self.assertTrue(ret_right, f"Could not find checkerboard corners in right image for {opencv_distortion_model}")

        # Extract the x and y coordinates of the corners in left_corners and right_corners
        x_coords_left = [c[0][0] for c in corners_left]
        y_coords_left = [c[0][1] for c in corners_left]
        x_coords_right = [c[0][0] for c in corners_right]
        y_coords_right = [c[0][1] for c in corners_right]
        if self._visualize:
            # Draw lines of the same color at the average row value for all corners
            # in the left and right image
            cv2.drawChessboardCorners(left_image_rect, checkerboard_size, corners_left, ret_left)
            cv2.drawChessboardCorners(right_image_rect, checkerboard_size, corners_right, ret_right)
            # Draw randomly colored lines connecting the corresponding corners
            for i in range(min(len(corners_left), len(corners_right))):
                average_y = (y_coords_left[i] + y_coords_right[i]) / 2
                pt1 = (0, int(average_y))
                pt2 = (left_image_rect.shape[1], int(average_y))
                random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                cv2.line(left_image_rect, pt1, pt2, random_color, 1)
                cv2.line(right_image_rect, pt1, pt2, random_color, 1)
            cv2.imwrite("left_image_rect.png", left_image_rect)
            cv2.imwrite("right_image_rect.png", right_image_rect)

        # Test 1: Check vertical alignment of corresponding corners
        # Compare row positions
        row_diffs = [
            abs(y_coords_left[i] - y_coords_right[i]) for i in range(min(len(corners_left), len(corners_right)))
        ]

        # Allow 4-pixel difference in vertical alignment
        CORNER_ROW_DIFF_THRESHOLD = 4
        if self._visualize:
            print("CORNER_ROW_DIFF_THRESHOLD :")
            print(CORNER_ROW_DIFF_THRESHOLD)
            print("row_diffs :")
            print(row_diffs)

        self.assertFalse(
            any(diff > CORNER_ROW_DIFF_THRESHOLD for diff in row_diffs),
            f"Difference between corners row values in left and right images exceeds threshold for {opencv_distortion_model}",
        )

        # Test 2: Check if epipolar lines are parallel
        EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD = 0.005
        epipolar_slopes = []
        for i in range(0, len(y_coords_left), checkerboard_size[0]):
            epipolar_slopes.append(
                abs(y_coords_left[i] - y_coords_left[i + 5]) / abs(x_coords_left[i] - x_coords_left[i + 5])
            )
            epipolar_slopes.append(
                abs(y_coords_right[i] - y_coords_right[i + 5]) / abs(x_coords_right[i] - x_coords_right[i + 5])
            )
        # Allow at most 2 points to exceed the threshold
        self.assertLessEqual(
            sum(np.array(epipolar_slopes) > EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD),
            2,
            f"Epipolar lines are not parallel for {opencv_distortion_model}!",
        )

    async def test_stereo_camera_opencv_pinhole(self):

        self._visualize = False
        left_camera, right_camera = self._prepare_scene_for_stereo_camera(
            baseline=0.15, resolution=(2048, 1024), focal_length=1.8, focus_distance=400
        )

        # Set distortion parameters
        pinhole = [0.1, 0.02, 0.01, 0.002, 0.003, 0.0004, 0.00005, 0.00005, 0.01, 0.002, 0.0003, 0.0004]
        left_camera.set_opencv_pinhole_properties(pinhole=pinhole)
        right_camera.set_opencv_pinhole_properties(pinhole=pinhole)

        # Retrieve and test basic validity of CameraInfo messages and raw images
        await self._test_get_stereo_camera_messages(
            opencv_distortion_model="opencvPinhole",
            ros2_distortion_model="rational_polynomial",
            distortion_coefficients=pinhole,
        )

        # Test stereo rectification
        await self._test_stereo_rectification(opencv_distortion_model="opencvPinhole")

    async def test_stereo_camera_opencv_fisheye(self):

        self._visualize = False
        left_camera, right_camera = self._prepare_scene_for_stereo_camera(
            baseline=0.15, resolution=(2048, 1024), focal_length=1.8, focus_distance=400
        )

        # Set distortion parameters
        fisheye = [0.1, 0.02, 0.003, 0.0004]
        left_camera.set_opencv_fisheye_properties(fisheye=fisheye)
        right_camera.set_opencv_fisheye_properties(fisheye=fisheye)

        # Retrieve and test basic validity of CameraInfo messages and raw images
        await self._test_get_stereo_camera_messages(
            opencv_distortion_model="opencvFisheye",
            ros2_distortion_model="equidistant",
            distortion_coefficients=fisheye,
        )

        # Test stereo rectification
        await self._test_stereo_rectification(opencv_distortion_model="opencvFisheye")
