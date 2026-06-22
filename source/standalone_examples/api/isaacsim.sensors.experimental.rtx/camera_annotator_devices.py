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

"""Validate camera annotator data retrieval on different devices (default CUDA, pre-allocated GPU, CPU via numpy).

This example demonstrates and validates:
- Default ``get_data()`` returns warp arrays on CUDA
- Pre-allocated ``out=`` parameter returns data on the specified device
- ``.numpy()`` conversion produces correct numpy arrays on CPU
- Shape/dtype/type checks for RGB, depth, and pointcloud annotators

Scene and camera parameters match the deprecated ``camera_annotator_device.py``
for output comparison.
"""

import argparse
import os
from typing import Any

parser = argparse.ArgumentParser(description="Camera annotator device validation example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_annotator_devices")
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni
import omni.usd
import warp as wp
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera


def validate(
    test_name: str, data: Any, expected_class: type[object], expected_dtype: object, expected_shape: tuple[int, ...]
) -> bool:
    """Validate data against expected type, dtype, and shape."""
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


# =============================================================================
# SETUP (matches deprecated camera_annotator_device.py scene)
# =============================================================================

camera_resolution = (256, 256)

cam = RtxCamera(
    "/World/camera",
    translations=np.array([0.0, 0.0, 5.0]),
    orientations=euler_angles_to_quaternion(np.array([0, 0, -90]), degrees=True, extrinsic=False).numpy(),
)

sensor = CameraSensor(
    cam,
    resolution=camera_resolution,
    annotators=["rgb", "distance_to_image_plane", "pointcloud"],
)

dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# Pre-allocate GPU output arrays
rgb_gpu = wp.empty((*camera_resolution, 3), dtype=wp.uint8, device="cuda:0")
depth_gpu = wp.empty((*camera_resolution, 1), dtype=wp.float32, device="cuda:0")

# =============================================================================
# RUN SIMULATION
# =============================================================================

timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(10):
    simulation_app.update()

# =============================================================================
# TEST RGB
# =============================================================================

rgb_shape = (*camera_resolution, 3)

print("=" * 80)
print("Testing: rgb")
print("-" * 40)

result_rgb = True

print("Default (CUDA warp array):")
rgb_default, _ = sensor.get_data("rgb")
result_rgb = validate("get_data_default", rgb_default, wp.array, wp.uint8, rgb_shape) and result_rgb

print("Pre-allocated GPU output:")
rgb_on_gpu, _ = sensor.get_data("rgb", out=rgb_gpu)
result_rgb = validate("get_data_out_gpu", rgb_on_gpu, wp.array, wp.uint8, rgb_shape) and result_rgb

print("As numpy (CPU):")
rgb_np = rgb_default.numpy()
result_rgb = validate("numpy", rgb_np, np.ndarray, np.uint8, rgb_shape) and result_rgb

print("[PASS]" if result_rgb else "[FAIL]")

# =============================================================================
# TEST DEPTH
# =============================================================================

depth_shape = (*camera_resolution, 1)

print("=" * 80)
print("Testing: depth (distance_to_image_plane)")
print("-" * 40)

result_depth = True

print("Default (CUDA warp array):")
depth_default, _ = sensor.get_data("distance_to_image_plane")
result_depth = validate("get_data_default", depth_default, wp.array, wp.float32, depth_shape) and result_depth

print("Pre-allocated GPU output:")
depth_on_gpu, _ = sensor.get_data("distance_to_image_plane", out=depth_gpu)
result_depth = validate("get_data_out_gpu", depth_on_gpu, wp.array, wp.float32, depth_shape) and result_depth

print("As numpy (CPU):")
depth_np = depth_default.numpy()
result_depth = validate("numpy", depth_np, np.ndarray, np.float32, depth_shape) and result_depth

print("[PASS]" if result_depth else "[FAIL]")

# =============================================================================
# TEST POINTCLOUD
# =============================================================================

print("=" * 80)
print("Testing: pointcloud")
print("-" * 40)

result_pc = True

print("Default (CUDA warp array):")
pc_default, _ = sensor.get_data("pointcloud")
if pc_default is not None:
    result_pc = validate("get_data_default", pc_default, wp.array, wp.float32, pc_default.shape) and result_pc
    print("As numpy (CPU):")
    pc_np = pc_default.numpy()
    result_pc = validate("numpy", pc_np, np.ndarray, np.float32, pc_np.shape) and result_pc
else:
    carb.log_warn("Pointcloud data is None")

print("[PASS]" if result_pc else "[FAIL]")

# =============================================================================
# SUMMARY
# =============================================================================

results = {
    "rgb": result_rgb,
    "depth": result_depth,
    "pointcloud": result_pc,
}

print("--- Test Summary ---")
for name, passed in results.items():
    print(f"{name}: {'PASS' if passed else 'FAIL'}")
print("--- End Summary ---")

if not all(results.values()):
    failed_tests = [name for name, passed in results.items() if not passed]
    raise Exception(f"Test run failed. Failing tests: {', '.join(failed_tests)}")

timeline.stop()
simulation_app.close()
