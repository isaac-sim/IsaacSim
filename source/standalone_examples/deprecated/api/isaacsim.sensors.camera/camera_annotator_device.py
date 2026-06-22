# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate camera annotator data retrieval and validation."""

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_annotator_device")
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni.timeline
import warp as wp
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import Camera


def test_camera_annotator_data(
    test_name: str, data: np.ndarray, expected_class: type, expected_dtype: type, expected_shape: tuple
) -> bool:
    """Validate camera annotator data against expected type, dtype, and shape."""
    print(f"{test_name}: data.shape: {data.shape}; dtype: {data.dtype}; type: {type(data)}")
    success = True
    if not isinstance(data, expected_class):
        carb.log_error(f"Data is {type(data)} but expected {expected_class}")
        success = False
    if data.dtype != expected_dtype:
        carb.log_error(f"Data dtype is {data.dtype} but expected {expected_dtype}")
        success = False
    if data.shape != expected_shape:
        carb.log_error(f"Data shape is {data.shape} but expected {expected_shape}")
        success = False
    return success


# Function to test RGBA output for different camera configurations
def test_rgba_output(camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera, rgba_shape: tuple) -> bool:
    """Tests the RGBA output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: rgba")

    print("-" * 40)
    print("Get rgba data using Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    default_rgba = camera_default.get_rgba()
    default_rgba_cpu = camera_default.get_rgba(device="cpu")
    default_rgba_cuda = camera_default.get_rgba(device="cuda")
    success = test_camera_annotator_data(
        test_name="None",
        data=default_rgba,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgba_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="cpu",
        data=default_rgba_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgba_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="cuda",
        data=default_rgba_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgba_shape,
    )
    test_1_passed = test_1_passed and success
    print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get rgba data using Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    cpu_rgba = camera_cpu.get_rgba()
    cpu_rgba_cpu = camera_cpu.get_rgba(device="cpu")
    cpu_rgba_cuda = camera_cpu.get_rgba(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_rgba_device_None",
        data=cpu_rgba,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgba_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgba_device_cpu",
        data=cpu_rgba_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgba_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgba_device_cuda",
        data=cpu_rgba_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgba_shape,
    )
    test_2_passed = test_2_passed and success
    print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print(f"Get rgba data using Camera(annotator_device='cuda'):")
    test_3_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    cuda_rgba = camera_cuda.get_rgba()
    cuda_rgba_cpu = camera_cuda.get_rgba(device="cpu")
    cuda_rgba_cuda = camera_cuda.get_rgba(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_rgba_device_None",
        data=cuda_rgba,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgba_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgba_device_cpu",
        data=cuda_rgba_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgba_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgba_device_cuda",
        data=cuda_rgba_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgba_shape,
    )
    test_3_passed = test_3_passed and success
    print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


# Function to test RGB output for different camera configurations
def test_rgb_output(camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera, rgb_shape: tuple) -> bool:
    """Tests the RGB output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: rgb")

    print("-" * 40)
    print("Get rgb data using Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    default_rgb = camera_default.get_rgb()
    default_rgb_cpu = camera_default.get_rgb(device="cpu")
    default_rgb_cuda = camera_default.get_rgb(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_rgb_device_None",
        data=default_rgb,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgb_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cpu",
        data=default_rgb_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgb_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cuda",
        data=default_rgb_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgb_shape,
    )
    test_1_passed = test_1_passed and success
    print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get rgb data using Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    cpu_rgb = camera_cpu.get_rgb()
    cpu_rgb_cpu = camera_cpu.get_rgb(device="cpu")
    cpu_rgb_cuda = camera_cpu.get_rgb(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_rgb_device_None",
        data=cpu_rgb,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgb_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cpu",
        data=cpu_rgb_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgb_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cuda",
        data=cpu_rgb_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgb_shape,
    )
    test_2_passed = test_2_passed and success
    print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get rgb data using Camera(annotator_device='cuda'):")
    print("---")
    test_3_passed = True
    # Get the data using the default device (set by annotator_device), or override the default device using cpu or cuda
    cuda_rgb = camera_cuda.get_rgb()
    cuda_rgb_cpu = camera_cuda.get_rgb(device="cpu")
    cuda_rgb_cuda = camera_cuda.get_rgb(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_rgb_device_None",
        data=cuda_rgb,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgb_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cpu",
        data=cuda_rgb_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.uint8,
        expected_shape=rgb_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_rgb_device_cuda",
        data=cuda_rgb_cuda,
        expected_class=wp.array,
        expected_dtype=wp.uint8,
        expected_shape=rgb_shape,
    )
    test_3_passed = test_3_passed and success
    print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


# Function to test Depth output for different camera configurations
def test_depth_output(camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera, depth_shape: tuple) -> bool:
    """Tests the Depth output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: depth")

    print("-" * 40)
    print("Get depth data using Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    default_depth = camera_default.get_depth()
    default_depth_cpu = camera_default.get_depth(device="cpu")
    default_depth_cuda = camera_default.get_depth(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_depth_device_None",
        data=default_depth,
        expected_class=np.ndarray,
        expected_dtype=np.float32,
        expected_shape=depth_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cpu",
        data=default_depth_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.float32,
        expected_shape=depth_shape,
    )
    test_1_passed = test_1_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cuda",
        data=default_depth_cuda,
        expected_class=wp.array,
        expected_dtype=wp.float32,
        expected_shape=depth_shape,
    )
    test_1_passed = test_1_passed and success
    print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get depth data using Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    cpu_depth = camera_cpu.get_depth()
    cpu_depth_cpu = camera_cpu.get_depth(device="cpu")
    cpu_depth_cuda = camera_cpu.get_depth(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_depth_device_None",
        data=cpu_depth,
        expected_class=np.ndarray,
        expected_dtype=np.float32,
        expected_shape=depth_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cpu",
        data=cpu_depth_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.float32,
        expected_shape=depth_shape,
    )
    test_2_passed = test_2_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cuda",
        data=cpu_depth_cuda,
        expected_class=wp.array,
        expected_dtype=wp.float32,
        expected_shape=depth_shape,
    )
    test_2_passed = test_2_passed and success
    print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get depth data using Camera(annotator_device='cuda'):")
    print("---")
    test_3_passed = True
    cuda_depth = camera_cuda.get_depth()
    cuda_depth_cpu = camera_cuda.get_depth(device="cpu")
    cuda_depth_cuda = camera_cuda.get_depth(device="cuda")
    success = test_camera_annotator_data(
        test_name="get_depth_device_None",
        data=cuda_depth,
        expected_class=wp.array,
        expected_dtype=wp.float32,
        expected_shape=depth_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cpu",
        data=cuda_depth_cpu,
        expected_class=np.ndarray,
        expected_dtype=np.float32,
        expected_shape=depth_shape,
    )
    test_3_passed = test_3_passed and success
    success = test_camera_annotator_data(
        test_name="get_depth_device_cuda",
        data=cuda_depth_cuda,
        expected_class=wp.array,
        expected_dtype=wp.float32,
        expected_shape=depth_shape,
    )
    test_3_passed = test_3_passed and success
    print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


# Function to test Pointcloud output for different camera configurations
def test_pointcloud_output(
    camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera, pointcloud_shape: tuple
) -> bool:
    """Tests the Pointcloud output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: pointcloud")

    world_frame_list = [True, False]

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        default_pointcloud = camera_default.get_pointcloud(world_frame=world_frame)
        default_pointcloud_cpu = camera_default.get_pointcloud(device="cpu", world_frame=world_frame)
        default_pointcloud_cuda = camera_default.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=default_pointcloud,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=default_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=default_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        cpu_pointcloud = camera_cpu.get_pointcloud(world_frame=world_frame)
        cpu_pointcloud_cpu = camera_cpu.get_pointcloud(device="cpu", world_frame=world_frame)
        cpu_pointcloud_cuda = camera_cpu.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=cpu_pointcloud,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=cpu_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=cpu_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device='cuda'):")
    print("---")
    test_3_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        cuda_pointcloud = camera_cuda.get_pointcloud(world_frame=world_frame)
        cuda_pointcloud_cpu = camera_cuda.get_pointcloud(device="cpu", world_frame=world_frame)
        cuda_pointcloud_cuda = camera_cuda.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=cuda_pointcloud,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=cuda_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=cuda_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


# Function to test Pointcloud (from depth) output for different camera configurations
def test_pointcloud_from_depth_output(
    camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera, pointcloud_shape: tuple
) -> bool:
    """Tests the Pointcloud (computed from depth) output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: pointcloud - from depth (remove pointcloud from frame)")

    # Test getting the data in world and camera frame
    world_frame_list = [True, False]

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        default_pointcloud = camera_default.get_pointcloud(world_frame=world_frame)
        default_pointcloud_cpu = camera_default.get_pointcloud(device="cpu", world_frame=world_frame)
        default_pointcloud_cuda = camera_default.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=default_pointcloud,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=default_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=default_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_1_passed = test_1_passed and success
        print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        cpu_pointcloud = camera_cpu.get_pointcloud(world_frame=world_frame)
        cpu_pointcloud_cpu = camera_cpu.get_pointcloud(device="cpu", world_frame=world_frame)
        cpu_pointcloud_cuda = camera_cpu.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=cpu_pointcloud,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=cpu_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=cpu_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_2_passed = test_2_passed and success
        print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Get pointcloud data using Camera(annotator_device='cuda'):")
    print("---")
    test_3_passed = True
    for world_frame in world_frame_list:
        print(f"world_frame: {world_frame}")
        print("---")
        cuda_pointcloud = camera_cuda.get_pointcloud(world_frame=world_frame)
        cuda_pointcloud_cpu = camera_cuda.get_pointcloud(device="cpu", world_frame=world_frame)
        cuda_pointcloud_cuda = camera_cuda.get_pointcloud(device="cuda", world_frame=world_frame)
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_None_world_frame_{world_frame}",
            data=cuda_pointcloud,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cpu_world_frame_{world_frame}",
            data=cuda_pointcloud_cpu,
            expected_class=np.ndarray,
            expected_dtype=np.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        success = test_camera_annotator_data(
            test_name=f"get_pointcloud_device_cuda_world_frame_{world_frame}",
            data=cuda_pointcloud_cuda,
            expected_class=wp.array,
            expected_dtype=wp.float32,
            expected_shape=pointcloud_shape,
        )
        test_3_passed = test_3_passed and success
        print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


# Function to test current frame output for different camera configurations
def test_current_frame_output(camera_default: Camera, camera_cpu: Camera, camera_cuda: Camera) -> bool:
    """Tests the current frame output for cameras with different annotator devices."""
    print("=" * 80)
    print("Testing: current frame")
    annotators_without_cuda_support = {"bounding_box_2d_tight", "bounding_box_2d_loose", "bounding_box_3d"}
    annotators_with_cuda_support = {
        "normals",
        "motion_vectors",
        "occlusion",
        "distance_to_image_plane",
        "distance_to_camera",
        "semantic_segmentation",
        "instance_id_segmentation",
        "instance_segmentation",
        "pointcloud",
    }
    # Get the current frame for the default, cpu and cuda cameras
    current_frame_default = camera_default.get_current_frame()
    current_frame_cpu = camera_cpu.get_current_frame()
    current_frame_cuda = camera_cuda.get_current_frame()

    print("-" * 40)
    print("Current frame from Camera(annotator_device=None):")
    print("---")
    test_1_passed = True
    # Current frame from the default and cpu cameras should only have numpy arrays
    for key in current_frame_default:
        data = current_frame_default[key]
        if isinstance(data, dict) and "data" in data:
            data = data["data"]
        if key in annotators_with_cuda_support:
            print(f"With CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, np.ndarray):
                carb.log_error(f"Data is {type(data)} but expected np.ndarray")
                test_1_passed = False
        if key in annotators_without_cuda_support:
            print(f"Without CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, np.ndarray):
                carb.log_error(f"Data is {type(data)} but expected np.ndarray")
                test_1_passed = False
    print("---")
    if test_1_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Current frame from Camera(annotator_device='cpu'):")
    print("---")
    test_2_passed = True
    for key in current_frame_cpu:
        data = current_frame_cpu[key]
        if isinstance(data, dict) and "data" in data:
            data = data["data"]
        if key in annotators_with_cuda_support:
            print(f"With CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, np.ndarray):
                carb.log_error(f"Data is {type(data)} but expected np.ndarray")
                test_2_passed = False
        if key in annotators_without_cuda_support:
            print(f"Without CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, np.ndarray):
                carb.log_error(f"Data is {type(data)} but expected np.ndarray")
                test_2_passed = False
    print("---")
    if test_2_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    print("-" * 40)
    print("Current frame from Camera(annotator_device='cuda'):")
    print("---")
    test_3_passed = True
    # Current frame from the cuda camera should have wp.arrays for cuda supported annotators and numpy arrays for the rest
    for key in current_frame_cuda:
        data = current_frame_cuda[key]
        if isinstance(data, dict) and "data" in data:
            data = data["data"]
        if key in annotators_with_cuda_support:
            print(f"With CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, wp.array):
                carb.log_error(f"Data is {type(data)} but expected wp.array")
                test_3_passed = False
        if key in annotators_without_cuda_support:
            print(f"Without CUDA: {key}, value type: {type(data)}")
            if not isinstance(data, np.ndarray):
                carb.log_error(f"Data is {type(data)} but expected np.ndarray")
                test_3_passed = False
    print("---")
    if test_3_passed:
        print("[PASS]")
    else:
        print("[FAIL]")
    print("---")

    return test_1_passed and test_2_passed and test_3_passed


###
# Setup world and the cameras
###
camera_orientation = euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy()
camera_resolution = (256, 256)
camera_position = np.array([0.0, 0.0, 5.0])

camera_default = Camera(
    prim_path="/World/camera_default",
    position=camera_position,
    orientation=camera_orientation,
    resolution=camera_resolution,
)
camera_cpu = Camera(
    prim_path="/World/camera_cpu",
    position=camera_position,
    orientation=camera_orientation,
    resolution=camera_resolution,
    annotator_device="cpu",
)
camera_cuda = Camera(
    prim_path="/World/camera_cuda",
    position=camera_position,
    orientation=camera_orientation,
    resolution=camera_resolution,
    annotator_device="cuda",
)

# Create ground plane and dome light
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

if args.test:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# Start the timeline and initialize the cameras
timeline = omni.timeline.get_timeline_interface()
timeline.play()
timeline.commit()

for camera in [camera_default, camera_cpu, camera_cuda]:
    camera.initialize()
    camera.set_lens_distortion_model("pinhole")
    camera.add_normals_to_frame()
    camera.add_motion_vectors_to_frame()
    camera.add_occlusion_to_frame()
    camera.add_distance_to_image_plane_to_frame()
    camera.add_distance_to_camera_to_frame()
    if camera._annotator_device is not None and camera._annotator_device != "cuda":
        camera.add_bounding_box_2d_tight_to_frame()
        camera.add_bounding_box_2d_loose_to_frame()
        camera.add_bounding_box_3d_to_frame()
    camera.add_semantic_segmentation_to_frame()
    camera.add_instance_id_segmentation_to_frame()
    camera.add_instance_segmentation_to_frame()
    camera.add_pointcloud_to_frame()

# Render a few frames to get the annotator data
for i in range(10):
    simulation_app.update()

# Define expected shapes
rgba_shape = (camera_resolution[1], camera_resolution[0], 4)
rgb_shape = (camera_resolution[1], camera_resolution[0], 3)
depth_shape = (camera_resolution[1], camera_resolution[0])
pointcloud_shape = (camera_resolution[0] * camera_resolution[1], 3)


# Test RGBA
result_test_rgba_output = test_rgba_output(camera_default, camera_cpu, camera_cuda, rgba_shape)

# Test RGB
result_test_rgb_output = test_rgb_output(camera_default, camera_cpu, camera_cuda, rgb_shape)

# Test Depth
result_test_depth_output = test_depth_output(camera_default, camera_cpu, camera_cuda, depth_shape)

# Test Pointcloud
result_test_pointcloud_output = test_pointcloud_output(camera_default, camera_cpu, camera_cuda, pointcloud_shape)

# Test Pointcloud from Depth
# Remove the pointcloud annotator to use the depth annotator for computing the pointcloud
for camera in [camera_default, camera_cpu, camera_cuda]:
    camera.remove_pointcloud_from_frame()

for i in range(10):
    simulation_app.update()
result_test_pointcloud_from_depth_output = test_pointcloud_from_depth_output(
    camera_default, camera_cpu, camera_cuda, pointcloud_shape
)

# Test Current Frame
result_test_current_frame_output = test_current_frame_output(camera_default, camera_cpu, camera_cuda)

# Print the results
print(f"test_rgba_output: {result_test_rgba_output}")
print(f"test_rgb_output: {result_test_rgb_output}")
print(f"test_depth_output: {result_test_depth_output}")
print(f"test_pointcloud_output: {result_test_pointcloud_output}")
print(f"test_pointcloud_from_depth_output: {result_test_pointcloud_from_depth_output}")
print(f"test_current_frame_output: {result_test_current_frame_output}")

# Example (after all test functions return booleans)
results = {
    "rgba": result_test_rgba_output,
    "rgb": result_test_rgb_output,
    "depth": result_test_depth_output,
    "pointcloud": result_test_pointcloud_output,
    "pointcloud_from_depth": result_test_pointcloud_from_depth_output,
    "current_frame": result_test_current_frame_output,
}

print("--- Test Summary ---")
for name, passed in results.items():
    print(f"{name}: {'PASS' if passed else 'FAIL'}")
print("--- End Summary ---")

if not all(results.values()):
    failed_tests = [name for name, passed in results.items() if not passed]
    raise Exception(f"Test run failed. Failing tests: {', '.join(failed_tests)}")

# Close the simulation app
simulation_app.close()
