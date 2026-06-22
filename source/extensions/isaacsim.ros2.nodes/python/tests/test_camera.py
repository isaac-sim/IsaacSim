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

"""Verify ROS 2 camera publishing.

Covers RGB, compressed H.264, moving-camera sequences, transform frames,
semantic labels, enable gating, tick-rate throttling, and depth point clouds.
"""

import math
import os
import shutil
import time
from typing import Any

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
import omni.usd
import rclpy
import usdrt.Sdf
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.core.experimental.utils import semantics as semantics_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.ros2.core.impl.ros2_image_test_utils import ros2_image_to_buffer
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.storage.native import get_assets_root_path
from isaacsim.test.utils.image_comparison import compare_arrays_within_tolerances
from isaacsim.test.utils.image_io import read_image_as_array, save_rgb_image
from pxr import PhysxSchema, Sdf
from sensor_msgs.msg import Image

from .common import add_carter_ros, add_cube, get_qos_profile


def _camera_orientation_at_angle_deg(angle_deg: float) -> Any:
    """Return quaternion (w,x,y,z) for camera at center looking at angle_deg in XY (0° = +X), up = world +Z.

    Camera local -Z is the view direction. Extrinsic ZYX Euler: Rz(angle-90) * Ry(0) * Rx(90)
    maps camera -Z to (cos(angle), sin(angle), 0) and camera +Y to world +Z.

    Args:
        angle_deg: Camera view angle in the XY plane.

    Returns:
        Camera orientation quaternion.
    """
    quat = transform_utils.euler_angles_to_quaternion([90.0, 0.0, angle_deg - 90.0], degrees=True, extrinsic=True)
    return quat.numpy().tolist()


def _view_angle_deg_from_quat_wxyz(quat_wxyz: Any) -> Any:
    """Angle in XY plane (degrees [0, 360)) that the camera is looking, from quat (w,x,y,z).

    Inverse of _camera_orientation_at_angle_deg: extract the extrinsic yaw and undo the -90° offset.

    Args:
        quat_wxyz: Camera orientation quaternion.

    Returns:
        Camera view angle in the XY plane.
    """
    euler = transform_utils.quaternion_to_euler_angles(quat_wxyz, degrees=True, extrinsic=True)
    yaw = float(euler.numpy()[2])  # output order is [roll, pitch, yaw] = [X, Y, Z]
    return (yaw + 90.0 + 360.0) % 360.0


def _create_rgb_camera_graph(graph_path: Any, camera_path: Any, topic_name: Any, width: Any, height: Any) -> None:
    """Create an OmniGraph that publishes RGB images from a camera via ROS2.

    Args:
        graph_path: OmniGraph path to create.
        camera_path: Camera prim path to render.
        topic_name: ROS 2 topic name.
        width: Render product width.
        height: Render product height.
    """
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("CreateRenderProduct.inputs:height", height),
                ("CreateRenderProduct.inputs:width", width),
                ("RGBPublish.inputs:topicName", topic_name),
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ],
        },
    )


def _match_buffered_images(image_buffer: Any, sim_times: Any, timestamp_tolerance: Any, label: Any = "") -> Any:
    """Match buffered (timestamp, image) pairs to target sim_times by closest timestamp.

    Args:
        image_buffer: List of (timestamp, ROS2 image message or image array) tuples.
        sim_times: Dict mapping target_angle -> sim_time to match against.
        timestamp_tolerance: Maximum allowed difference between image timestamp and sim_time.
        label: Optional prefix for log messages (e.g. "golden ").

    Returns:
        Dict mapping target_angle -> image_array for all matched targets.

    """
    matched = {}
    matched_ts = {}
    for target, target_sim_time in sim_times.items():
        best_match = None
        best_diff = float("inf")
        best_ts = None
        for ts, img in image_buffer:
            diff = abs(ts - target_sim_time)
            if diff < best_diff:
                best_diff = diff
                best_match = img
                best_ts = ts
        if best_diff <= timestamp_tolerance:
            if hasattr(best_match, "shape"):
                matched[target] = best_match
            else:
                matched[target] = ros2_image_to_buffer(
                    best_match,
                    normalize_color_order=True,
                    squeeze_singleton_channel=True,
                    copy=True,
                )
            matched_ts[target] = best_ts
            print(f"Matched {label}{target}° - " f"best_diff={best_diff:.6f}s (tolerance={timestamp_tolerance:.6f}s)")
        else:
            print(
                f"WARNING: No image matched {label}{target}° "
                f"(sim_time={target_sim_time:.6f}s, best_diff={best_diff:.6f}s)"
            )
    return matched, matched_ts


class TestRos2Camera(ROS2TestCase):
    """Verify ROS 2 camera image, label, TF, and point cloud publication graphs."""

    async def setUp(self) -> None:
        """Create a ROS 2 camera test stage with a fixed viewport resolution."""
        await super().setUp()

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

    async def test_camera(self) -> None:
        """Test camera."""
        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await stage_utils.open_stage_async(self._assets_root_path + scene_path)

        cube_1 = Cube("/cube_1", sizes=1.0, positions=[0, 0, 0], scales=[1.5, 1, 1])
        semantics_utils.add_labels(cube_1.prims[0], labels=["Cube0"])

        import rclpy
        import usdrt.Sdf

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("DepthPclPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("InstancePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("SemanticPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/OmniverseKit_Persp")]),
                        ("CreateRenderProduct.inputs:height", 600),
                        ("CreateRenderProduct.inputs:width", 800),
                        ("RGBPublish.inputs:topicName", "rgb"),
                        ("RGBPublish.inputs:type", "rgb"),
                        ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPublish.inputs:topicName", "depth"),
                        ("DepthPublish.inputs:type", "depth"),
                        ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPclPublish.inputs:topicName", "depth_pcl"),
                        ("DepthPclPublish.inputs:type", "depth_pcl"),
                        ("DepthPclPublish.inputs:resetSimulationTimeOnStop", True),
                        ("InstancePublish.inputs:topicName", "instance_segmentation"),
                        ("InstancePublish.inputs:type", "instance_segmentation"),
                        ("InstancePublish.inputs:resetSimulationTimeOnStop", True),
                        ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                        ("SemanticPublish.inputs:type", "semantic_segmentation"),
                        ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "DepthPclPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "InstancePublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "SemanticPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                        ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                        ("CreateRenderProduct.outputs:renderProductPath", "DepthPclPublish.inputs:renderProductPath"),
                        ("CreateRenderProduct.outputs:renderProductPath", "InstancePublish.inputs:renderProductPath"),
                        ("CreateRenderProduct.outputs:renderProductPath", "SemanticPublish.inputs:renderProductPath"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import Image, PointCloud2

        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None

        def rgb_callback(data: Any) -> None:
            self._rgb = data

        def depth_callback(data: Any) -> None:
            self._depth = data

        def depth_pcl_callback(data: Any) -> None:
            self._depth_pcl = data

        def instance_segmentation_callback(data: Any) -> None:
            self._instance_segmentation = data

        def semantic_segmentation_callback(data: Any) -> None:
            self._semantic_segmentation = data

        node = self.create_node("camera_tester")
        rgb_sub = self.create_subscription(node, Image, "rgb", rgb_callback, get_qos_profile())
        depth_sub = self.create_subscription(node, Image, "depth", depth_callback, get_qos_profile())
        depth_pcl_sub = self.create_subscription(node, PointCloud2, "depth_pcl", depth_pcl_callback, get_qos_profile())
        instance_segmentation_sub = self.create_subscription(
            node, Image, "instance_segmentation", instance_segmentation_callback, get_qos_profile()
        )
        semantic_segmentation_sub = self.create_subscription(
            node, Image, "semantic_segmentation", semantic_segmentation_callback, get_qos_profile()
        )

        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6.0, prev=0
        )

        # square pixels, vertical apertures are computed by the horizontal aperture
        # omni.kit.commands.execute(
        #     "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=4.5, prev=0
        # )

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        import time

        # Turn on SystemTime for timestamp of all camera publishers
        og.Controller.attribute("/ActionGraph/RGBPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/DepthPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/DepthPclPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/InstancePublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/SemanticPublish" + ".inputs:useSystemTime").set(True)

        await omni.kit.app.get_app().next_update_async()

        system_time = int(time.time())

        self._timeline.play()
        await self.simulate_until_condition(
            lambda: (
                self._rgb is not None
                and self._instance_segmentation is not None
                and self._semantic_segmentation is not None
            ),
            max_frames=600,
            per_frame_callback=spin,
        )

        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)

        self.assertGreaterEqual(self._rgb.header.stamp.sec, system_time)
        self.assertGreaterEqual(self._depth.header.stamp.sec, system_time)
        self.assertGreaterEqual(self._depth_pcl.header.stamp.sec, system_time)
        self.assertGreaterEqual(self._instance_segmentation.header.stamp.sec, system_time)
        self.assertGreaterEqual(self._semantic_segmentation.header.stamp.sec, system_time)

    async def test_rgb_golden_image_comparison(self) -> None:
        """Subscribe to an RGB image topic and compare received buffer against a golden image."""
        # Retrieve golden image from data/tests/golden_img folder
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden")
        golden_img_path = os.path.join(golden_dir, "nova_carter_warehouse_front_stereo_left_rgb.png")

        # Open the nova carter warehouse scene (following simulation_control pattern)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        assets_root_path = get_assets_root_path()
        warehouse_scene = assets_root_path + "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
        success, error = await stage_utils.open_stage_async(warehouse_scene)
        self.assertTrue(success, f"Failed to open stage: {error}")

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Setup ROS2 subscriber for the RGB image topic
        self._received_rgb_image = None

        def rgb_callback(data: Any) -> None:
            self._received_rgb_image = data

        node = self.create_node("rgb_image_test_node")
        rgb_sub = self.create_subscription(
            node, Image, "/front_stereo_camera/left/image_raw", rgb_callback, get_qos_profile()
        )

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        await omni.kit.app.get_app().next_update_async()

        # Move /World/Nova_Carter_ROS to -6, -1, 0 and 180 degree rotation around z axis
        # Quaternion for 180 deg rotation around z: (w=0, x=0, y=0, z=1)
        nova_carter = XformPrim("/World/Nova_Carter_ROS", reset_xform_op_properties=True)
        nova_carter.set_world_poses(positions=[-6, -1, 0], orientations=[0, 0, 0, 1])

        await omni.kit.app.get_app().next_update_async()

        # Hit Play on scene and wait for image
        self._timeline.play()
        await self.simulate_until_condition(
            lambda: self._received_rgb_image is not None,
            max_frames=300,
            per_frame_callback=spin,
        )

        # Verify image was received
        self.assertIsNotNone(self._received_rgb_image, "Failed to receive RGB image from topic")

        # Retrieve image buffer from subscriber
        received_array = ros2_image_to_buffer(
            self._received_rgb_image,
            normalize_color_order=True,
            squeeze_singleton_channel=True,
            copy=True,
        )

        # Hit Stop on scene
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Compare image with golden image
        golden_img_data = read_image_as_array(str(golden_img_path))

        # Handle channel mismatch between RGBA golden and RGB received
        if golden_img_data.ndim == 3 and golden_img_data.shape[2] == 4:
            golden_img_data = golden_img_data[:, :, :3]

        results = compare_arrays_within_tolerances(
            golden_img_data,
            received_array,
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=10,
            print_all_stats=True,
        )
        self.assertTrue(results["passed"], f"Image comparison failed: {results}")

    async def test_rgb_h264_compressed_golden_image_comparison(self) -> Any:
        """Subscribe to compressed RGB H.264, decode it, and compare against the golden image.

        Returns:
            None.
        """
        try:
            import PyNvVideoCodec as nvc
        except ImportError:
            self.skipTest("PyNvVideoCodec not available - skipping H264 decode test")

        import numpy as np
        from sensor_msgs.msg import CompressedImage

        # Ensure omni.replicator.nv extension is enabled (provides H264 hardware encoder)
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.set_extension_enabled_immediate("omni.replicator.nv", True)
        await omni.kit.app.get_app().next_update_async()

        # Retrieve golden image from data/tests/golden_img folder
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden")
        golden_img_path = os.path.join(golden_dir, "nova_carter_warehouse_front_stereo_left_rgb.png")

        # Open the nova carter warehouse scene
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        assets_root_path = get_assets_root_path()
        warehouse_scene = assets_root_path + "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
        success, error = await stage_utils.open_stage_async(warehouse_scene)
        self.assertTrue(success, f"Failed to open stage: {error}")

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Modify the existing front stereo camera left RGB publisher to use H264 compression
        og.Controller.attribute("/World/Nova_Carter_ROS/front_hawk/left_camera_publish_image.inputs:type").set(
            "rgb_h264"
        )

        og.Controller.attribute("/World/Nova_Carter_ROS/front_hawk/left_camera_publish_image.inputs:topicName").set(
            "left/image_raw/compressed"
        )

        await omni.kit.app.get_app().next_update_async()

        # Setup ROS2 subscriber for the compressed image topic
        self._received_compressed_image = None

        def compressed_callback(data: Any) -> None:
            self._received_compressed_image = data

        node = self.create_node("rgb_h264_test_node")
        compressed_sub = self.create_subscription(
            node,
            CompressedImage,
            "/front_stereo_camera/left/image_raw/compressed",
            compressed_callback,
            get_qos_profile(),
        )

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        await omni.kit.app.get_app().next_update_async()

        # Move /World/Nova_Carter_ROS to -6, -1, 0
        nova_carter = XformPrim("/World/Nova_Carter_ROS", reset_xform_op_properties=True)
        nova_carter.set_world_poses(positions=[-6, -1, 0], orientations=[0, 0, 0, 1])

        await omni.kit.app.get_app().next_update_async()

        # Play scene and wait for compressed image
        self._timeline.play()
        await self.simulate_until_condition(
            lambda: self._received_compressed_image is not None,
            max_frames=300,
            per_frame_callback=spin,
        )

        self.assertIsNotNone(self._received_compressed_image, "Failed to receive compressed image from topic")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Get the H264 bitstream from ROS CompressedImage message
        h264_bitstream = self._received_compressed_image.data.tobytes()

        # Decode H264 using PyNvVideoCodec (core Decoder + buffer demuxer)
        # Buffer feeder serves raw H264 elementary stream bytes to the demuxer
        class H264BufferFeeder:
            def __init__(self, data: Any) -> None:
                self._buffer = bytearray(data)
                self._pos = 0
                self._remaining = len(self._buffer)

            def feed_chunk(self, demuxer_buffer: Any) -> Any:
                chunk = min(self._remaining, len(demuxer_buffer))
                if chunk == 0:
                    return 0
                demuxer_buffer[:chunk] = self._buffer[self._pos : self._pos + chunk]
                self._pos += chunk
                self._remaining -= chunk
                return chunk

        feeder = H264BufferFeeder(h264_bitstream)
        dmx = nvc.CreateDemuxer(feeder.feed_chunk)
        dec = nvc.CreateDecoder(
            gpuid=0,
            codec=dmx.GetNvCodecId(),
            usedevicememory=False,
        )

        frames = []
        for pkt in dmx:
            for frame in dec.Decode(pkt):
                frames.append(frame)

        self.assertTrue(len(frames) > 0, f"Failed to decode H264 frame ({len(h264_bitstream)} bytes)")

        # Convert last decoded frame to numpy array via DLPack
        # Core decoder outputs NV12 (native format); convert to RGB
        decoded_np = np.from_dlpack(frames[-1])
        if decoded_np.dtype != np.uint8:
            decoded_np = np.clip(decoded_np, 0, 255).astype(np.uint8)

        # NV12 frame has shape (H * 3/2, W) — convert to RGB (H, W, 3)
        import cv2

        received_array = cv2.cvtColor(decoded_np, cv2.COLOR_YUV2RGB_NV12)

        # Compare image with golden image
        golden_img_data = read_image_as_array(str(golden_img_path))

        # Handle channel mismatch between RGBA golden and RGB received
        if golden_img_data.ndim == 3 and golden_img_data.shape[2] == 4:
            golden_img_data = golden_img_data[:, :, :3]

        # H264 compression is lossy, so we need a higher tolerance
        results = compare_arrays_within_tolerances(
            golden_img_data,
            received_array,
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=15,
            print_all_stats=True,
        )
        self.assertTrue(results["passed"], f"H264 compressed image comparison failed: {results}")

    async def test_spinning_camera_golden_images(self) -> None:
        """Two cameras on one spinning rigid body: compare physics images to golden images.

        Camera 1 is a Camera prim that also carries the RigidBodyAPI and spins at 90 deg/s.
        After camera 1 completes one full rotation, camera 2 is added *live* (no pause) as a
        child prim of camera 1 -- facing 180 degrees opposite, offset vertically, and tilted
        down ~9.5 degrees so it sees a completely different perspective while sharing the same
        spin.  No second rigid body is created.

        Two golden image sets are captured and validated:
          1a. Camera 1 first rotation (camera 2 does not exist yet).
          1b. Camera 1 second rotation + camera 2 first rotation (same rig angles).
          2.  Golden images by teleporting the rig to recorded angles.
          3.  Comparison: camera 1 reuses its first-rotation goldens for the
              second rotation (same speed), camera 2 uses its own goldens.
        """
        update_golden_images = False
        save_debug_images = False
        keyframe_angles_deg = list(range(0, 360, 30))
        camera_height = 0.5
        rotation_speed_deg_per_sec = 90
        cam2_vertical_offset = 1.0  # metres above camera 1 (local +Y = world +Z)
        cam2_tilt_deg = 10.0  # degrees downward tilt

        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "spinning_camera")

        # Open the pre-built scene USD (contains physics scene and scattered objects).
        scene_usd_path = os.path.join(golden_dir, "spinning_camera_scene.usda")
        await stage_utils.open_stage_async(scene_usd_path)
        await omni.kit.app.get_app().next_update_async()

        # Add the grid environment as a reference.
        stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/EnvGrid",
        )

        width, height = 640, 360

        # Single camera prim at center (Camera + rigid body, no CollisionAPI)
        camera_path = "/World/SpinningCamera"
        stage_utils.define_prim(camera_path, type_name="Camera")
        # RigidPrim automatically applies RigidBodyAPI, PhysxRigidBodyAPI, and MassAPI
        camera_rigid = RigidPrim(camera_path, masses=[0.1], reset_xform_op_properties=True)
        camera_rigid.set_enabled_gravities([False])
        # Zero damping so angular velocity is maintained exactly
        physx_api = PhysxSchema.PhysxRigidBodyAPI(camera_rigid.prims[0])
        physx_api.CreateAngularDampingAttr().Set(0.0)
        physx_api.CreateLinearDampingAttr().Set(0.0)
        camera_rigid.set_world_poses(
            positions=[[0.0, 0.0, camera_height]],
            orientations=[_camera_orientation_at_angle_deg(0.0)],
        )

        # Rotate camera around world Z (angular velocity in rad/s)
        camera_rigid.set_velocities(
            linear_velocities=[[0.0, 0.0, 0.0]],
            angular_velocities=[[0.0, 0.0, 0.0]],
        )
        await omni.kit.app.get_app().next_update_async()

        # ROS2 camera publisher
        _create_rgb_camera_graph("/ActionGraph", camera_path, "spinning_camera_rgb", width, height)
        await omni.kit.app.get_app().next_update_async()

        # Buffer all received ROS2 images with their timestamps
        image_buffer = []  # list of (timestamp, ROS2 Image message)

        def rgb_callback(data: Any) -> None:
            ts = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
            image_buffer.append((ts, data))

        node = self.create_node("spinning_camera_test_node")
        self.start_async_spinning(node)
        self.create_subscription(
            node,
            Image,
            "spinning_camera_rgb",
            rgb_callback,
            get_qos_profile(depth=100),
        )

        # ============================================================
        # STEP 1: Physics-based rotation - simulate and buffer images
        # ============================================================
        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=2)

        # Wait for the first ROS2 image to confirm the pipeline is running
        await self.simulate_until_condition(
            lambda: len(image_buffer) > 0,
        )

        stage_fps = self._timeline.get_time_codes_per_second()
        ros_drain_delay_sec = 1.0 / stage_fps

        async def _simulate_frames_with_ros_drain(max_frames: Any, per_frame_callback: Any = None) -> None:
            for _ in range(max_frames):
                await omni.kit.app.get_app().next_update_async()
                if per_frame_callback is not None:
                    per_frame_callback()
                # The ROS executor runs in a background thread. Yield wall-clock
                # time here so callbacks drain before the next frame is published.
                time.sleep(ros_drain_delay_sec)

        angle_tolerance_deg = 0.1
        # One full rotation plus extra frames for pipeline-delayed images to arrive
        rotation_frames = int((360.0 / rotation_speed_deg_per_sec) * stage_fps)
        pipeline_drain_frames = 30  # extra frames for delayed images to flush through
        total_frames = rotation_frames + pipeline_drain_frames

        # Record angle + sim_time at each target during the rotation
        recorded_sim_times = {}  # target_angle -> sim_time
        recorded_angles = {}  # target_angle -> actual_angle
        image_buffer.clear()

        camera_rigid.set_velocities(
            linear_velocities=[[0.0, 0.0, 0.0]],
            angular_velocities=[[0.0, 0.0, math.radians(rotation_speed_deg_per_sec)]],
        )

        print(f"Starting physics-based rotation capture ({rotation_speed_deg_per_sec} deg/s)...")

        def _record_angles_step() -> None:
            if len(recorded_sim_times) < len(keyframe_angles_deg):
                sim_time = SimulationManager.get_simulation_time()
                _, orientations = camera_rigid.get_world_poses()
                ori = orientations.numpy()[0]
                actual_angle = _view_angle_deg_from_quat_wxyz([ori[0], ori[1], ori[2], ori[3]])
                for target in keyframe_angles_deg:
                    if target in recorded_sim_times:
                        continue
                    angle_diff = abs(actual_angle - target)
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    if angle_diff <= angle_tolerance_deg:
                        recorded_sim_times[target] = sim_time
                        recorded_angles[target] = actual_angle
                        print(f"Angle {target}° at sim_time={sim_time:.6f}s (actual={actual_angle:.2f}°)")

        await _simulate_frames_with_ros_drain(total_frames, per_frame_callback=_record_angles_step)

        missing_angles = [a for a in keyframe_angles_deg if a not in recorded_sim_times]
        if missing_angles:
            self.fail(f"Did not observe angles during rotation: {missing_angles}")

        print(f"Buffered {len(image_buffer)} ROS2 images during rotation.")

        # Snapshot the first-rotation buffer; matching is deferred until after both
        # rotations finish so the rig isn't wasting simulation frames on processing.
        timestamp_tolerance = 1.5 / stage_fps
        image_buffer_r1 = list(image_buffer)

        # ===========================================================
        # ADD CAMERA 2 as child of camera 1 (while paused)
        # ============================================================
        # self._timeline.pause()
        # await omni.kit.app.get_app().next_update_async()

        print("[Live] Adding camera 2 as child of camera 1...")
        camera_path_2 = camera_path + "/Camera2"
        stage_utils.define_prim(camera_path_2, type_name="Camera")
        cam2_xform = XformPrim(camera_path_2, reset_xform_op_properties=True)
        cam2_local_quat = transform_utils.euler_angles_to_quaternion(
            [-cam2_tilt_deg, 180.0, 0.0], degrees=True, extrinsic=True
        )
        cam2_xform.set_local_poses(
            translations=[[0.0, cam2_vertical_offset, 0.0]],
            orientations=[cam2_local_quat.numpy().tolist()],
        )
        await omni.kit.app.get_app().next_update_async()

        # ROS2 camera 2 publisher Graph
        _create_rgb_camera_graph("/ActionGraph2", camera_path_2, "spinning_camera_2_rgb", width, height)
        await omni.kit.app.get_app().next_update_async()

        image_buffer_2 = []

        def rgb_callback_2(data: Any) -> None:
            ts = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
            image_buffer_2.append((ts, data))

        self.create_subscription(
            node,
            Image,
            "spinning_camera_2_rgb",
            rgb_callback_2,
            get_qos_profile(depth=100),
        )
        print("[Live] Camera 2 subscription added, background executor handles both cameras.")

        # ============================================================
        # STEP 1b: Both cameras rotate on the same rig.
        # Camera 1 continues its second rotation; camera 2 rides along for its first.
        # Only one set of keyframe times is needed (same rig orientation).
        # ============================================================
        image_buffer.clear()
        image_buffer_2.clear()

        recorded_sim_times_1b = {}
        recorded_angles_1b = {}
        angle_tolerance_1b_deg = 0.2

        print("[STEP 1b] Both cameras rotating (same rig)...")

        def _record_angles_1b_step() -> None:
            if len(recorded_sim_times_1b) < len(keyframe_angles_deg):
                sim_time = SimulationManager.get_simulation_time()
                _, orientations = camera_rigid.get_world_poses()
                ori = orientations.numpy()[0]
                actual_angle = _view_angle_deg_from_quat_wxyz([ori[0], ori[1], ori[2], ori[3]])
                for target in keyframe_angles_deg:
                    if target in recorded_sim_times_1b:
                        continue
                    angle_diff = abs(actual_angle - target)
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    if angle_diff <= angle_tolerance_1b_deg:
                        recorded_sim_times_1b[target] = sim_time
                        recorded_angles_1b[target] = actual_angle
                        print(f"  rig {target}° at sim_time={sim_time:.6f}s (actual={actual_angle:.2f}°)")

        await _simulate_frames_with_ros_drain(total_frames, per_frame_callback=_record_angles_1b_step)

        missing_angles_1b = [a for a in keyframe_angles_deg if a not in recorded_sim_times_1b]
        if missing_angles_1b:
            self.fail(f"[STEP 1b] Did not observe rig angles: {missing_angles_1b}")

        print(f"[STEP 1b] Buffered {len(image_buffer)} cam1 and {len(image_buffer_2)} cam2 images.")

        # Match camera 1 first rotation (deferred from STEP 1a to avoid wasting sim frames)
        physics_images, _ = _match_buffered_images(
            image_buffer_r1, recorded_sim_times, timestamp_tolerance, label="physics "
        )
        if save_debug_images:
            debug_dir = os.path.join(golden_dir, "debug_captured")
            os.makedirs(debug_dir, exist_ok=True)
            for target, img in physics_images.items():
                save_rgb_image(img, debug_dir, f"physics_angle_{target}.png")

        missing_images = [a for a in keyframe_angles_deg if a not in physics_images]
        if missing_images:
            self.fail(
                f"Could not match physics images for angles: {missing_images}. "
                f"Buffered {len(image_buffer_r1)} images, tolerance={timestamp_tolerance:.6f}s"
            )

        # Keep common timestamps in the log because missing common frames are useful
        # evidence, but do not require common timestamps for image comparison.
        cam1_by_ts = dict(image_buffer)
        cam2_by_ts = dict(image_buffer_2)
        cam1_timestamps = sorted(cam1_by_ts.keys())
        cam2_timestamps = sorted(cam2_by_ts.keys())
        common_timestamps = sorted(set(cam1_timestamps) & set(cam2_timestamps))

        print(f"\n=== Timestamp dump (cam1: {len(cam1_timestamps)}, cam2: {len(cam2_timestamps)}) ===")
        print(f"cam1 timestamps: {[f'{t:.6f}' for t in cam1_timestamps]}")
        print(f"cam2 timestamps: {[f'{t:.6f}' for t in cam2_timestamps]}")
        print(f"Common timestamps ({len(common_timestamps)}): {[f'{t:.6f}' for t in common_timestamps]}")
        cam1_only = sorted(set(cam1_timestamps) - set(cam2_timestamps))
        cam2_only = sorted(set(cam2_timestamps) - set(cam1_timestamps))
        if cam1_only:
            print(f"cam1 only ({len(cam1_only)}): {[f'{t:.6f}' for t in cam1_only]}")
        if cam2_only:
            print(f"cam2 only ({len(cam2_only)}): {[f'{t:.6f}' for t in cam2_only]}")
        print(f"Recorded rig sim_times: { {a: f'{t:.6f}' for a, t in sorted(recorded_sim_times_1b.items())} }")
        print("=== End timestamp dump ===\n")

        # Match each camera independently. A dropped cam2 frame should not force cam1
        # to compare against a later common timestamp.
        physics_images_1b, _ = _match_buffered_images(
            image_buffer, recorded_sim_times_1b, timestamp_tolerance, label="cam1 second rotation "
        )
        physics_images_2, _ = _match_buffered_images(
            image_buffer_2, recorded_sim_times_1b, timestamp_tolerance, label="cam2 "
        )

        if save_debug_images:
            debug_dir_1b = os.path.join(golden_dir, "debug_captured_camera_1_2nd")
            os.makedirs(debug_dir_1b, exist_ok=True)
            for target, img in physics_images_1b.items():
                save_rgb_image(img, debug_dir_1b, f"physics_angle_{target}.png")
            debug_dir_2 = os.path.join(golden_dir, "debug_captured_camera_2")
            os.makedirs(debug_dir_2, exist_ok=True)
            for target, img in physics_images_2.items():
                save_rgb_image(img, debug_dir_2, f"physics_angle_{target}.png")

        missing_1b_cam1 = [a for a in keyframe_angles_deg if a not in physics_images_1b]
        missing_1b_cam2 = [a for a in keyframe_angles_deg if a not in physics_images_2]
        if missing_1b_cam1 or missing_1b_cam2:
            self.fail(
                f"[STEP 1b] Could not match images for cam1 angles: {missing_1b_cam1}, "
                f"cam2 angles: {missing_1b_cam2}. "
                f"cam1={len(cam1_timestamps)} timestamps, cam2={len(cam2_timestamps)} timestamps, "
                f"common={len(common_timestamps)} timestamps, tolerance={timestamp_tolerance:.6f}s"
            )

        # ============================================================
        # STEP 2: Golden images - generate or load from disk
        #   golden_images  = camera 1 (STEP 1a angles, reused for 2nd rotation)
        #   golden_images_2 = camera 2 (STEP 1b rig angles, same as cam1)
        # ============================================================
        golden_images = {}
        golden_images_2 = {}

        if update_golden_images:
            # Zero the rig velocity so teleport sticks (sim still running)
            camera_rigid.set_velocities(
                linear_velocities=[[0.0, 0.0, 0.0]],
                angular_velocities=[[0.0, 0.0, 0.0]],
            )
            image_buffer.clear()
            image_buffer_2.clear()

            await self.simulate_until_condition(lambda: False, max_frames=15)

            # Single pass: teleport rig once per angle, capture both cameras together.
            print("Generating goldens (both cameras, single pass)...")
            for target_angle in keyframe_angles_deg:
                actual_angle = recorded_angles_1b[target_angle]
                camera_rigid.set_world_poses(
                    positions=[[0.0, 0.0, camera_height]],
                    orientations=[_camera_orientation_at_angle_deg(actual_angle)],
                )
                cam1_pre = len(image_buffer)
                cam2_pre = len(image_buffer_2)
                await self.simulate_until_condition(
                    lambda: len(image_buffer) > cam1_pre and len(image_buffer_2) > cam2_pre,
                    max_frames=30,
                )
                self.assertGreater(
                    len(image_buffer),
                    cam1_pre,
                    f"No new cam1 image after teleporting to {target_angle}°",
                )
                self.assertGreater(
                    len(image_buffer_2),
                    cam2_pre,
                    f"No new cam2 image after teleporting to {target_angle}°",
                )
                golden_images[target_angle] = ros2_image_to_buffer(
                    image_buffer[-1][1],
                    normalize_color_order=True,
                    squeeze_singleton_channel=True,
                    copy=True,
                )
                golden_images_2[target_angle] = ros2_image_to_buffer(
                    image_buffer_2[-1][1],
                    normalize_color_order=True,
                    squeeze_singleton_channel=True,
                    copy=True,
                )
                print(f"  {target_angle}° captured (cam1 + cam2)")

            self.stop_async_spinning(node)
            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            for target_angle in keyframe_angles_deg:
                save_rgb_image(golden_images[target_angle], golden_dir, f"angle_{target_angle}_camera_1.png")
                save_rgb_image(golden_images_2[target_angle], golden_dir, f"angle_{target_angle}_camera_2.png")
            print("Golden image generation complete.")
        else:
            self.stop_async_spinning(node)
            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            print("Loading existing golden images from disk...")
            for target_angle in keyframe_angles_deg:
                golden_path_1 = os.path.join(golden_dir, f"angle_{target_angle}_camera_1.png")
                golden_path_2 = os.path.join(golden_dir, f"angle_{target_angle}_camera_2.png")
                for p in [golden_path_1, golden_path_2]:
                    self.assertTrue(
                        os.path.isfile(p),
                        f"Golden image not found: {p}. Set update_golden_images=True to generate.",
                    )
                golden_img_1 = read_image_as_array(golden_path_1)
                if golden_img_1.ndim == 3 and golden_img_1.shape[2] == 4:
                    golden_img_1 = golden_img_1[:, :, :3]
                golden_images[target_angle] = golden_img_1
                golden_img_2 = read_image_as_array(golden_path_2)
                if golden_img_2.ndim == 3 and golden_img_2.shape[2] == 4:
                    golden_img_2 = golden_img_2[:, :, :3]
                golden_images_2[target_angle] = golden_img_2
                print(f"Loaded goldens for {target_angle}°")

        # ============================================================
        # STEP 3: Compare physics-captured images to golden images
        # ============================================================
        print("Comparing camera 1 (1st rotation) physics vs golden...")
        for target_angle in keyframe_angles_deg:
            print(f"Comparing camera 1 (1st rot) at {target_angle}°")
            results = compare_arrays_within_tolerances(
                golden_images[target_angle],
                physics_images[target_angle],
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10,
                print_all_stats=True,
            )
            self.assertTrue(
                results["passed"],
                f"Camera 1 (1st rotation) image comparison failed at {target_angle}°: {results}",
            )
        print("Comparing camera 1 (2nd rotation) physics vs golden...")
        for target_angle in keyframe_angles_deg:
            print(f"Comparing camera 1 (2nd rot) at {target_angle}°")
            results = compare_arrays_within_tolerances(
                golden_images[target_angle],
                physics_images_1b[target_angle],
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10,
                print_all_stats=True,
            )
            self.assertTrue(
                results["passed"],
                f"Camera 1 (2nd rotation) image comparison failed at {target_angle}°: {results}",
            )
        print("Comparing camera 2 physics vs golden...")
        for target_angle in keyframe_angles_deg:
            print(f"Comparing camera 2 at {target_angle}°")
            results = compare_arrays_within_tolerances(
                golden_images_2[target_angle],
                physics_images_2[target_angle],
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10,
                print_all_stats=True,
            )
            self.assertTrue(
                results["passed"],
                f"Camera 2 image comparison failed at {target_angle}°: {results}",
            )

    async def test_dual_camera_moving_cube(self) -> Any:
        """Two co-located cameras must produce matching images of a laterally moving cube.

        Both cameras share the same position and orientation. A cube is placed
        in front of them and teleported 0.5 m laterally each frame for 30 frames.
        Images from both cameras are collected via ROS2, matched by their
        simulation-time timestamps, and compared to verify identical output.

        Returns:
            None.
        """
        save_debug_images = False
        num_frames = 10
        cube_travel_distance = 4.0
        cube_step_m = cube_travel_distance / num_frames
        width, height = 640, 360
        camera_height = 0.5
        camera_pos = [0.0, 0.0, camera_height]
        cube_distance = 10.0
        cube_start_y = -(cube_travel_distance / 2.0)

        debug_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data", "dual_camera_moving_cube", "debug"
        )

        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await stage_utils.open_stage_async(self._assets_root_path + scene_path)
        await omni.kit.app.get_app().next_update_async()

        # Two cameras at the exact same world pose
        camera_path_1 = "/World/Camera1"
        camera_path_2 = "/World/Camera2"
        stage_utils.define_prim(camera_path_1, type_name="Camera")
        stage_utils.define_prim(camera_path_2, type_name="Camera")

        cam1_xform = XformPrim(camera_path_1, reset_xform_op_properties=True)
        cam2_xform = XformPrim(camera_path_2, reset_xform_op_properties=True)

        cam_orientation = _camera_orientation_at_angle_deg(0.0)
        cam1_xform.set_world_poses(positions=[camera_pos], orientations=[cam_orientation])
        cam2_xform.set_world_poses(positions=[camera_pos], orientations=[cam_orientation])
        await omni.kit.app.get_app().next_update_async()

        cube = Cube("/World/MovingCube", sizes=1.0, positions=[cube_distance, cube_start_y, camera_height])
        cube_xform = XformPrim("/World/MovingCube")
        await omni.kit.app.get_app().next_update_async()

        _create_rgb_camera_graph("/ActionGraph1", camera_path_1, "dual_cam_1_rgb", width, height)
        _create_rgb_camera_graph("/ActionGraph2", camera_path_2, "dual_cam_2_rgb", width, height)
        await omni.kit.app.get_app().next_update_async()

        image_buffer_1 = []
        image_buffer_2 = []

        def _to_image_array(image_msg: Any) -> Any:
            return ros2_image_to_buffer(
                image_msg,
                normalize_color_order=True,
                squeeze_singleton_channel=True,
                copy=True,
            )

        def rgb_callback_1(data: Any) -> None:
            ts = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
            image_buffer_1.append((ts, data))

        def rgb_callback_2(data: Any) -> None:
            ts = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
            image_buffer_2.append((ts, data))

        node = self.create_node("dual_camera_test_node")
        self.start_async_spinning(node)
        self.create_subscription(node, Image, "dual_cam_1_rgb", rgb_callback_1, get_qos_profile(depth=num_frames + 40))
        self.create_subscription(node, Image, "dual_cam_2_rgb", rgb_callback_2, get_qos_profile(depth=num_frames + 40))

        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=2)

        # Wait for both render pipelines to start producing images
        await self.simulate_until_condition(
            lambda: len(image_buffer_1) > 0 and len(image_buffer_2) > 0,
        )

        ros_drain_delay_sec = 1.0 / self._timeline.get_time_codes_per_second()
        image_buffer_1.clear()
        image_buffer_2.clear()
        capture_start_time = SimulationManager.get_simulation_time()
        print(f"Buffers cleared at sim_time={capture_start_time:.6f}s")

        # Move the cube 0.5 m laterally each frame
        print(f"Moving cube across {num_frames} frames ({cube_step_m} m/frame)...")
        for frame_idx in range(num_frames):
            cube_y = cube_start_y + frame_idx * cube_step_m
            cube_xform.set_world_poses(positions=[[cube_distance, cube_y, camera_height]])
            await omni.kit.app.get_app().next_update_async()
            time.sleep(ros_drain_delay_sec)

        # Extra frames so pipeline-delayed images flush through
        pipeline_drain_frames = 30
        for _ in range(pipeline_drain_frames):
            await omni.kit.app.get_app().next_update_async()
            time.sleep(ros_drain_delay_sec)

        self.stop_async_spinning(node)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        print(f"Buffered {len(image_buffer_1)} cam1 and {len(image_buffer_2)} cam2 images (raw).")

        # Keep only images with timestamps after the buffer-clear point
        image_buffer_1 = [(ts, img) for ts, img in image_buffer_1 if ts >= capture_start_time]
        image_buffer_2 = [(ts, img) for ts, img in image_buffer_2 if ts >= capture_start_time]
        print(
            f"After filtering (ts >= {capture_start_time:.6f}s): "
            f"{len(image_buffer_1)} cam1 and {len(image_buffer_2)} cam2 images."
        )

        self.assertGreater(len(image_buffer_1), 0, "No cam1 images after capture_start_time")
        self.assertGreater(len(image_buffer_2), 0, "No cam2 images after capture_start_time")

        if save_debug_images:
            if os.path.isdir(debug_dir):
                shutil.rmtree(debug_dir)
            os.makedirs(debug_dir)

        # Save all debug images first (both cameras, every frame)
        if save_debug_images:
            for idx, (ts, img_msg) in enumerate(image_buffer_1):
                save_rgb_image(_to_image_array(img_msg), debug_dir, f"cam1_{idx:03d}_ts_{ts:.6f}.png")
            for idx, (ts, img_msg) in enumerate(image_buffer_2):
                save_rgb_image(_to_image_array(img_msg), debug_dir, f"cam2_{idx:03d}_ts_{ts:.6f}.png")
            print(f"Saved {len(image_buffer_1)} cam1 + {len(image_buffer_2)} cam2 " f"debug images to {debug_dir}")

        # Compare only timestamps present in both cameras. Randomly dropped frames
        # should not fail the test as long as enough exact same-frame pairs remain.
        cam1_by_ts = dict(image_buffer_1)
        cam2_by_ts = dict(image_buffer_2)
        common_timestamps = sorted(set(cam1_by_ts) & set(cam2_by_ts))
        cam1_only = sorted(set(cam1_by_ts) - set(cam2_by_ts))
        cam2_only = sorted(set(cam2_by_ts) - set(cam1_by_ts))
        if cam1_only:
            print(f"cam1-only timestamps ({len(cam1_only)}): {[f'{ts:.6f}' for ts in cam1_only]}")
        if cam2_only:
            print(f"cam2-only timestamps ({len(cam2_only)}): {[f'{ts:.6f}' for ts in cam2_only]}")

        matched_pairs = 0
        comparison_failures = []

        for ts1 in common_timestamps:
            img1 = _to_image_array(cam1_by_ts[ts1])
            img2 = _to_image_array(cam2_by_ts[ts1])

            matched_pairs += 1
            print(f"Pair {matched_pairs}: ts={ts1:.6f}s")

            results = compare_arrays_within_tolerances(
                img1,
                img2,
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10,
                print_all_stats=True,
            )
            if not results["passed"]:
                comparison_failures.append((ts1, results))

        print(f"Compared {matched_pairs} image pairs with identical timestamps")

        self.assertGreaterEqual(
            matched_pairs,
            num_frames,
            f"Expected at least {num_frames} matching timestamp pairs, got {matched_pairs}. "
            f"cam1={len(image_buffer_1)}, cam2={len(image_buffer_2)}, "
            f"cam1_only={len(cam1_only)}, cam2_only={len(cam2_only)}",
        )
        self.assertEqual(
            len(comparison_failures),
            0,
            f"{len(comparison_failures)} of {matched_pairs} pairs failed image comparison: "
            f"{comparison_failures[0][1] if comparison_failures else 'N/A'}",
        )

    async def test_camera_tf_includes_180_x_rotation(self) -> None:
        """Camera prims in the TF tree must include a 180-deg x-axis rotation.

        Verifies the USD camera convention (-Z forward, +Y up) to ROS optical
        frame convention (+Z forward, +Y down) conversion applied by
        OgnIsaacComputeTransformTree.  Uses two cameras (identity and 90-deg Z
        rotated) and a plain Xform control to confirm the rotation is applied
        only to cameras and that it composes correctly with authored orientation.
        """
        from tf2_msgs.msg import TFMessage

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        stage_utils.define_prim("/CameraIdentity", "Camera")
        cam_id = XformPrim("/CameraIdentity", reset_xform_op_properties=True)
        cam_id.set_world_poses(positions=[[1.0, 2.0, 3.0]], orientations=[[1, 0, 0, 0]])

        cos45 = math.cos(math.radians(45))
        sin45 = math.sin(math.radians(45))
        stage_utils.define_prim("/CameraRotZ90", "Camera")
        cam_rot = XformPrim("/CameraRotZ90", reset_xform_op_properties=True)
        cam_rot.set_world_poses(positions=[[4.0, 5.0, 6.0]], orientations=[[cos45, 0, 0, sin45]])

        stage_utils.define_prim("/ControlXform", "Xform")
        ctrl = XformPrim("/ControlXform", reset_xform_op_properties=True)
        ctrl.set_world_poses(positions=[[7.0, 8.0, 9.0]], orientations=[[1, 0, 0, 0]])

        await omni.kit.app.get_app().next_update_async()

        self._camera_tf_data = None

        def tf_callback(data: TFMessage) -> None:
            self._camera_tf_data = data

        node = self.create_node("camera_tf_tester")
        self.create_subscription(node, TFMessage, "/tf_camera_test", tf_callback, get_qos_profile())

        try:
            og.Controller.edit(
                {"graph_path": "/CameraTFGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_camera_test"),
                        (
                            "ComputeTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/CameraIdentity"),
                                usdrt.Sdf.Path("/CameraRotZ90"),
                                usdrt.Sdf.Path("/ControlXform"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ComputeTF.inputs:execIn"),
                        ("ComputeTF.outputs:execOut", "PublishTF.inputs:execIn"),
                        ("ComputeTF.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
                        ("ComputeTF.outputs:childFrames", "PublishTF.inputs:childFrames"),
                        ("ComputeTF.outputs:translations", "PublishTF.inputs:translations"),
                        ("ComputeTF.outputs:orientations", "PublishTF.inputs:orientations"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._camera_tf_data is not None,
            max_frames=60,
            per_frame_callback=spin,
        )

        self.assertIsNotNone(self._camera_tf_data, "Expected TF data from camera TF test")

        tf_map = {t.child_frame_id: t for t in self._camera_tf_data.transforms}
        all_frames = list(tf_map.keys())

        cam_id_tf = tf_map.get("CameraIdentity")
        cam_rot_tf = tf_map.get("CameraRotZ90")
        ctrl_tf = tf_map.get("ControlXform")

        self.assertIsNotNone(cam_id_tf, f"CameraIdentity not in TF. Frames: {all_frames}")
        self.assertIsNotNone(cam_rot_tf, f"CameraRotZ90 not in TF. Frames: {all_frames}")
        self.assertIsNotNone(ctrl_tf, f"ControlXform not in TF. Frames: {all_frames}")

        # Control xform at identity — no camera rotation applied
        r = ctrl_tf.transform.rotation
        self.assertAlmostEqual(r.w, 1.0, places=5, msg="Control w")
        self.assertAlmostEqual(r.x, 0.0, places=5, msg="Control x")
        self.assertAlmostEqual(r.y, 0.0, places=5, msg="Control y")
        self.assertAlmostEqual(r.z, 0.0, places=5, msg="Control z")

        # Camera at identity: 180-deg x-rotation -> (x~1, y~0, z~0, w~0)
        r = cam_id_tf.transform.rotation
        self.assertAlmostEqual(r.w, 0.0, places=4, msg="CameraIdentity w")
        self.assertAlmostEqual(abs(r.x), 1.0, places=4, msg="CameraIdentity |x|")
        self.assertAlmostEqual(r.y, 0.0, places=4, msg="CameraIdentity y")
        self.assertAlmostEqual(r.z, 0.0, places=4, msg="CameraIdentity z")

        # Camera at 90-deg Z composed with 180-deg x -> (x~cos45, y~sin45, z~0, w~0)
        r = cam_rot_tf.transform.rotation
        self.assertAlmostEqual(r.w, 0.0, places=4, msg="CameraRotZ90 w")
        self.assertAlmostEqual(abs(r.x), cos45, places=4, msg="CameraRotZ90 |x|")
        self.assertAlmostEqual(abs(r.y), sin45, places=4, msg="CameraRotZ90 |y|")
        self.assertAlmostEqual(r.z, 0.0, places=4, msg="CameraRotZ90 z")

        # Positions should match authored values
        t = cam_id_tf.transform.translation
        self.assertAlmostEqual(t.x, 1.0, places=3)
        self.assertAlmostEqual(t.y, 2.0, places=3)
        self.assertAlmostEqual(t.z, 3.0, places=3)

        t = cam_rot_tf.transform.translation
        self.assertAlmostEqual(t.x, 4.0, places=3)
        self.assertAlmostEqual(t.y, 5.0, places=3)
        self.assertAlmostEqual(t.z, 6.0, places=3)

        t = ctrl_tf.transform.translation
        self.assertAlmostEqual(t.x, 7.0, places=3)
        self.assertAlmostEqual(t.y, 8.0, places=3)
        self.assertAlmostEqual(t.z, 9.0, places=3)

        self._timeline.stop()
        spin()

    async def test_semantic_labels_publishing(self) -> None:
        """Verify enableSemanticLabels publishes semantic labels on a separate topic."""
        Cube("/World/cube", sizes=1.0, positions=[2.0, 0.0, 0.0])
        semantics_utils.add_labels("/World/cube", labels=["TestCube"])

        from isaacsim.sensors.experimental.rtx import RtxCamera

        # Orientation (90, -90, 0) intrinsic XYZ = looking down +X toward the cube
        cam = RtxCamera("/World/camera", positions=[0.0, 0.0, 0.5], orientations=[0.5, 0.5, -0.5, -0.5])

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("SemanticPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera")]),
                    ("CreateRenderProduct.inputs:height", 480),
                    ("CreateRenderProduct.inputs:width", 640),
                    ("SemanticPublish.inputs:topicName", "semantic_seg"),
                    ("SemanticPublish.inputs:type", "semantic_segmentation"),
                    ("SemanticPublish.inputs:enableSemanticLabels", True),
                    ("SemanticPublish.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "SemanticPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "SemanticPublish.inputs:renderProductPath"),
                ],
            },
        )

        from std_msgs.msg import String

        label_data = None
        node = self.create_node("test_semantic_labels")
        self.start_async_spinning(node)

        def on_label(msg: Any) -> None:
            nonlocal label_data
            label_data = msg.data

        self.create_subscription(node, String, "semantic_labels", on_label, get_qos_profile(depth=10))

        self._timeline.play()
        # Wait for a label message that contains our TestCube label
        # Note: the pipeline lowercases semantic labels
        condition_met = await self.simulate_until_condition(
            lambda: label_data is not None and "testcube" in label_data.lower(), max_frames=180
        )
        self._timeline.stop()

        self.assertTrue(condition_met, f"testcube not found in semantic labels. Last received: {label_data}")

    async def test_enabled_input_disables_publishing(self) -> None:
        """Verify setting enabled=False stops message publishing."""
        Cube("/World/cube", sizes=1.0, positions=[2.0, 0.0, 0.0])

        from isaacsim.sensors.experimental.rtx import RtxCamera

        cam = RtxCamera("/World/camera", positions=[0.0, 0.0, 0.5], orientations=[0.5, 0.5, -0.5, -0.5])

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera")]),
                    ("CreateRenderProduct.inputs:height", 240),
                    ("CreateRenderProduct.inputs:width", 320),
                    ("RGBPublish.inputs:topicName", "rgb_enabled_test"),
                    ("RGBPublish.inputs:type", "rgb"),
                    ("RGBPublish.inputs:enabled", False),
                    ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ],
            },
        )

        msg_count = 0
        node = self.create_node("test_enabled")
        self.start_async_spinning(node)

        def on_image(msg: Any) -> None:
            nonlocal msg_count
            msg_count += 1

        self.create_subscription(node, Image, "rgb_enabled_test", on_image, get_qos_profile(depth=10))

        self._timeline.play()
        await self.simulate_until_condition(lambda: False, max_frames=60)
        self._timeline.stop()

        self.assertEqual(msg_count, 0, "Expected no messages when enabled=False")

    async def test_tick_rate_reduces_publish_frequency(self) -> None:
        """Verify that omni:sensor:tickRate throttles render product output and reduces publish rate.

        Creates two cameras: one at 10 Hz tickRate, one at autotrigger (0 = every frame).
        Both publish RGB via ROS2CameraHelper. After 120 frames (~2s at 60 Hz), the slow
        camera should have significantly fewer unique frames than the fast one.
        A moving cube ensures each rendered frame is visually distinct.
        """
        # Add a dome light so the scene is visible
        from pxr import UsdLux

        dome_light = UsdLux.DomeLight.Define(stage_utils.get_current_stage(), "/World/dome_light")
        dome_light.CreateIntensityAttr(1000)

        # Create a cube in the camera's FOV — we'll move it each frame to ensure unique renders
        cube = Cube("/World/cube", sizes=1.0, positions=[3.0, 0.0, 0.5], colors=[1, 0, 0])

        from isaacsim.sensors.experimental.rtx import RtxCamera

        cam_slow = RtxCamera(
            "/World/camera_slow",
            tick_rate=10.0,
            positions=[0.0, 0.0, 0.5],
            orientations=[0.5, 0.5, -0.5, -0.5],
        )
        cam_fast = RtxCamera(
            "/World/camera_fast",
            positions=[0.0, 1.0, 0.5],
            orientations=[0.5, 0.5, -0.5, -0.5],
        )

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRPSlow", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CreateRPFast", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGBSlow", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("RGBFast", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CreateRPSlow.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera_slow")]),
                    ("CreateRPSlow.inputs:height", 240),
                    ("CreateRPSlow.inputs:width", 320),
                    ("CreateRPFast.inputs:cameraPrim", [usdrt.Sdf.Path("/World/camera_fast")]),
                    ("CreateRPFast.inputs:height", 240),
                    ("CreateRPFast.inputs:width", 320),
                    ("RGBSlow.inputs:topicName", "rgb_slow"),
                    ("RGBSlow.inputs:type", "rgb"),
                    ("RGBSlow.inputs:resetSimulationTimeOnStop", True),
                    ("RGBFast.inputs:topicName", "rgb_fast"),
                    ("RGBFast.inputs:type", "rgb"),
                    ("RGBFast.inputs:resetSimulationTimeOnStop", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRPSlow.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "CreateRPFast.inputs:execIn"),
                    ("CreateRPSlow.outputs:execOut", "RGBSlow.inputs:execIn"),
                    ("CreateRPSlow.outputs:renderProductPath", "RGBSlow.inputs:renderProductPath"),
                    ("CreateRPFast.outputs:execOut", "RGBFast.inputs:execIn"),
                    ("CreateRPFast.outputs:renderProductPath", "RGBFast.inputs:renderProductPath"),
                ],
            },
        )

        import hashlib
        import tempfile

        SAVE_DEBUG_FRAMES = False  # Set True to save frames for debugging

        slow_count = 0
        fast_count = 0
        slow_unique_hashes = set()
        fast_unique_hashes = set()
        slow_debug_dir = os.path.join(tempfile.gettempdir(), "debug_tick_rate_slow")
        fast_debug_dir = os.path.join(tempfile.gettempdir(), "debug_tick_rate_fast")
        if SAVE_DEBUG_FRAMES:
            os.makedirs(slow_debug_dir, exist_ok=True)
            os.makedirs(fast_debug_dir, exist_ok=True)

        node = self.create_node("test_tick_rate")
        self.start_async_spinning(node)

        def _save_frame(msg: Any, directory: Any, count: Any) -> None:
            try:
                import numpy as _np
                from PIL import Image as PILImage

                arr = _np.frombuffer(bytes(msg.data), dtype=_np.uint8).reshape((msg.height, msg.width, 3))
                PILImage.fromarray(arr).save(os.path.join(directory, f"frame_{count:04d}.png"))
            except Exception:
                pass

        def on_slow(msg: Any) -> None:
            nonlocal slow_count
            slow_count += 1
            slow_unique_hashes.add(hashlib.md5(bytes(msg.data)).hexdigest())
            if SAVE_DEBUG_FRAMES:
                _save_frame(msg, slow_debug_dir, slow_count)

        def on_fast(msg: Any) -> None:
            nonlocal fast_count
            fast_count += 1
            fast_unique_hashes.add(hashlib.md5(bytes(msg.data)).hexdigest())
            if SAVE_DEBUG_FRAMES:
                _save_frame(msg, fast_debug_dir, fast_count)

        self.create_subscription(node, Image, "rgb_slow", on_slow, get_qos_profile(depth=100))
        self.create_subscription(node, Image, "rgb_fast", on_fast, get_qos_profile(depth=100))

        self._timeline.play()
        for i in range(120):
            # Move cube each frame to ensure visually distinct renders
            cube.set_world_poses(positions=[[3.0 + i * 0.01, 0.0, 0.5]])
            await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()

        carb.log_warn(
            f"tickRate test: slow_count={slow_count} (unique={len(slow_unique_hashes)}), "
            f"fast_count={fast_count} (unique={len(fast_unique_hashes)})"
        )
        if SAVE_DEBUG_FRAMES:
            carb.log_warn(f"Debug frames saved to {slow_debug_dir} and {fast_debug_dir}")

        # At 60 Hz sim rate, fast (autotrigger) should render ~120 unique frames,
        # slow (10 Hz) should render ~20 unique frames.
        # The writer may republish stale frames, so check unique image count.
        self.assertGreater(fast_count, 0, "Fast camera should have published messages")
        self.assertGreater(slow_count, 0, "Slow camera should have published some messages")
        if len(fast_unique_hashes) > 10:
            self.assertLess(
                len(slow_unique_hashes),
                len(fast_unique_hashes) * 0.5,
                f"tickRate not respected: slow has {len(slow_unique_hashes)} unique frames "
                f"(of {slow_count} msgs) vs fast {len(fast_unique_hashes)} unique frames "
                f"(of {fast_count} msgs). "
                f"Expected slow ~{len(fast_unique_hashes) * 10 // 60} unique frames at 10 Hz.",
            )

    async def test_camera_depth_to_pcl(self) -> None:
        """Test camera depth to pcl."""
        from sensor_msgs.msg import PointCloud2

        robot_path = await add_carter_ros(self._assets_root_path)
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        graph_path = robot_path + "/ROS_Cameras"

        og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled").set(False)

        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                graph_path,
                {
                    keys.CREATE_NODES: [("depthToPCL", "isaacsim.ros2.bridge.ROS2CameraHelper")],
                    keys.CONNECT: [
                        (graph_path + "/isaac_create_render_product_left.outputs:execOut", "depthToPCL.inputs:execIn"),
                        (graph_path + "/camera_frameId_left.inputs:value", "depthToPCL.inputs:frameId"),
                        (
                            graph_path + "/isaac_create_render_product_left.outputs:renderProductPath",
                            "depthToPCL.inputs:renderProductPath",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("depthToPCL.inputs:topicName", "/point_cloud_left"),
                        ("depthToPCL.inputs:type", "depth_pcl"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        og.Controller.set(
            og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled"), True
        )

        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        viewport_api.set_texture_resolution((1280, 720))

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2) -> None:
            self._point_cloud_data = data

        node = self.create_node("depth_point_cloud_tester")
        self.create_subscription(node, PointCloud2, "point_cloud_left", point_cloud_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._point_cloud_data is not None, max_frames=120, per_frame_callback=spin
        )

        self.assertIsNotNone(self._point_cloud_data)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        self.assertEqual(self._point_cloud_data.data[526327], 190)
        self.assertEqual(self._point_cloud_data.data[712187], 63)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()
