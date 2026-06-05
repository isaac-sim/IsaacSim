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

"""Configure a camera with the ISP (Image Signal Processing) pipeline via USD schema.

This example demonstrates how to:
- Apply the ``OmniSensorGenericCameraCoreAPI`` schema for ISP simulation
- Configure color correction, CFA (Color Filter Array) pattern, and companding
- Retrieve pre-ISP and post-ISP rendered output

The ``OmniSensorGenericCameraCoreAPI`` schema replaces the deprecated OmniGraph-based
ISP pipeline. All ISP parameters are configured declaratively as USD attributes on the
camera prim, with attribute prefix ``omni:sensor:core:``.

Key ISP stages:
1. **Color correction** — black level, white balance, color matrix, sensor response scaling
2. **CFA encoding** — Bayer pattern (GRBG, RGGB, BGGR, etc.) with configurable filter coefficients
3. **Companding** — piecewise-linear tone curve for HDR-to-SDR conversion
4. **Noise** — Gaussian and shot noise simulation
"""

import argparse
import os
import platform
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Camera ISP pipeline via USD schema.")
parser.add_argument(
    "--output-dir",
    type=str,
    default=os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_isp_pipeline"),
    help="Output directory for ISP pipeline images.",
)
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

if not (platform.system() == "Linux" and platform.machine() == "x86_64"):
    print(
        f"This example requires Linux x86_64 "
        f"(detected {platform.system()} {platform.machine()}).  "
        "The sample ISP program bundled with omni.sensors.nv.camera is only "
        "available on that platform.  To run on a different platform, supply "
        "your own ISP program, comment-out this check, and update the "
        "_isp_program_path variable in this script."
    )
    sys.exit(1)

NUM_FRAMES = 20

simulation_app = SimulationApp({"headless": True})

import cv2
import numpy as np
import omni
from isaacsim.core.experimental.objects import Cube, DistantLight
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

# The omni.sensors.nv.camera extension provides the GPU-side ISP pipeline that
# reads OmniSensorGenericCameraCoreAPI schema attributes and writes
# introspection output files.  It is not loaded by default.
enable_extension("omni.sensors.nv.camera")

# Load the example ISP program shipped with the extension.  The Smodel ISP
# adapter requires a base64-encoded program in the ispSmodelCameraProgram
# attribute; without it the pipeline errors with "No Model Library provided".
import base64

import omni.kit.app

_cam_ext = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("omni.sensors.nv.camera")
_isp_program_path = os.path.join(_cam_ext, "bin", "isp_program_example_3848_2168_r16.ispprg")
with open(_isp_program_path, "rb") as f:
    _isp_program_b64 = base64.b64encode(f.read()).decode("ascii")

os.makedirs(args.output_dir, exist_ok=True)


# =============================================================================
# ISP BINARY CONVERSION HELPERS
# =============================================================================
# The ISP introspection stage writes one binary file per pipeline stage.  Each
# file contains all rendered frames concatenated.  The helpers below extract the
# last frame and convert it to a viewable PNG.
#
# The demosaic and YUV→RGB routines are imported from the omni.sensors.nv.camera
# extension's image_utils module (no __init__.py in util/raw/, so we load it via
# importlib).  The multi-frame extraction is handled locally since the extension
# utilities assume single-frame files.

import importlib.util as _ilu

_iu_spec = _ilu.spec_from_file_location(
    "_camera_image_utils",
    os.path.join(_cam_ext, "omni", "sensors", "nv", "camera", "util", "raw", "image_utils.py"),
)
_camera_image_utils = _ilu.module_from_spec(_iu_spec)
_iu_spec.loader.exec_module(_camera_image_utils)

_demosaic = _camera_image_utils._demosaic_nearest_neighbor
_read_isp_ayuv = _camera_image_utils._read_isp_ayuv


def _read_last_frame(bin_path, frame_bytes):
    """Read the last *frame_bytes* from a (possibly multi-frame) binary file."""
    file_size = os.path.getsize(bin_path)
    if file_size < frame_bytes:
        raise ValueError(f"file too small ({file_size} < {frame_bytes})")
    with open(bin_path, "rb") as fh:
        fh.seek(file_size - frame_bytes)
        return fh.read(frame_bytes)


def _normalize_to_uint8(arr):
    """Normalize a float/int array to uint8 [0, 255]."""
    arr = np.nan_to_num(arr.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    max_val = arr.max()
    if max_val > 0:
        arr = arr / max_val
    return (np.clip(arr, 0, 1) * 255).astype(np.uint8)


def _convert_isp_bins_to_images(output_dir, height, width, cfa_pattern):
    """Convert ISP introspection .bin files to .png images."""
    import tempfile

    _RGBA_F16 = {"texread", "color", "isp"}
    _BAYER_U32 = {"cfa", "noise"}
    _BAYER_U16 = {"comp"}

    bin_files = sorted(f for f in os.listdir(output_dir) if f.endswith(".bin"))
    if not bin_files:
        return

    print(f"\nConverting ISP binaries to images ({width}x{height})...")

    for f in bin_files:
        bin_path = os.path.join(output_dir, f)
        stage = f.split("-", 1)[1].replace(".bin", "")

        try:
            if stage in _RGBA_F16:
                frame_bytes = height * width * 4 * np.dtype(np.float16).itemsize
                raw = _read_last_frame(bin_path, frame_bytes)
                img = np.frombuffer(raw, dtype=np.float16).reshape((height, width, 4))
                rgb = _normalize_to_uint8(img[:, :, :3])

            elif stage in _BAYER_U32:
                frame_bytes = height * width * np.dtype(np.uint32).itemsize
                raw = _read_last_frame(bin_path, frame_bytes)
                bayer = np.frombuffer(raw, dtype=np.uint32).reshape((height, width))
                rgb = _normalize_to_uint8(_demosaic(bayer, cfa_pattern))

            elif stage in _BAYER_U16:
                frame_bytes = height * width * np.dtype(np.uint16).itemsize
                raw = _read_last_frame(bin_path, frame_bytes)
                bayer = np.frombuffer(raw, dtype=np.uint16).reshape((height, width))
                rgb = _normalize_to_uint8(_demosaic(bayer, cfa_pattern))

            elif stage == "yuv":
                file_size = os.path.getsize(bin_path)
                pixels = height * width
                # _read_isp_ayuv expects uint16×4 (8 bpp).  If the file
                # instead stores uint8×4 (4 bpp) we fall back to an inline
                # conversion using the same YUV→RGB matrix.
                frame_8bpp = pixels * 8
                frame_4bpp = pixels * 4

                if file_size >= frame_8bpp and file_size % frame_8bpp == 0:
                    raw = _read_last_frame(bin_path, frame_8bpp)
                    tmp_path = None
                    try:
                        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as tmp:
                            tmp.write(raw)
                            tmp_path = tmp.name
                        rgb = _read_isp_ayuv(tmp_path, width, height)
                    finally:
                        if tmp_path and os.path.exists(tmp_path):
                            os.unlink(tmp_path)
                elif file_size >= frame_4bpp and file_size % frame_4bpp == 0:
                    raw = _read_last_frame(bin_path, frame_4bpp)
                    img = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))
                    norm = np.float64(2**8)
                    y = img[:, :, 2].astype(np.float64) / norm
                    u = img[:, :, 1].astype(np.float64) / norm - 0.5
                    v = img[:, :, 0].astype(np.float64) / norm - 0.5
                    yuv_to_rgb = np.array(
                        [
                            [1.0, 1.0, 1.0],
                            [-0.000007154783816076815, -0.3441331386566162, 1.7720025777816772],
                            [1.4019975662231445, -0.7141380310058594, 0.00001542569043522235],
                        ]
                    )
                    rgb = (np.clip(np.dot(np.dstack((y, u, v)), yuv_to_rgb), 0, 1) * 255).astype(np.uint8)
                else:
                    print(f"  {f}: unexpected YUV size ({file_size}), skipping")
                    continue

            else:
                print(f"  {f}: unknown stage '{stage}', skipping")
                continue

            out_path = os.path.join(output_dir, f"{stage}.png")
            rgb_uint8 = np.clip(rgb, 0, 255).astype(np.uint8)
            cv2.imwrite(out_path, cv2.cvtColor(rgb_uint8, cv2.COLOR_RGB2BGR))
            print(f"  {f} -> {stage}.png")

        except Exception as e:
            print(f"  {f}: conversion failed: {e}")


# =============================================================================
# CREATE SCENE
# =============================================================================

stage = get_current_stage()

# DistantLight emits along its local -Z axis. Orient it to face +X (toward the cubes).
light = DistantLight(
    "/World/light",
    orientations=np.array([0.5, 0.5, -0.5, -0.5]),  # wxyz, -Z → +X
)
light.set_intensities(3000.0)

for i, (x, y, color) in enumerate([(3, 0, [1, 0, 0]), (5, 2, [0, 1, 0]), (4, -2, [0, 0, 1])]):
    Cube(f"/World/cube_{i}", sizes=1.0, positions=np.array([float(x), float(y), 0.5]), colors=color)

# =============================================================================
# CREATE CAMERA WITH ISP PIPELINE SCHEMA
# =============================================================================
# OmniSensorGenericCameraCoreAPI inherits from OmniSensorAPI, so applying it
# also makes the prim a sensor (with tick rate support, etc.).
#
# The ISP attributes configure the entire image processing pipeline
# declaratively — no OmniGraph wiring needed.

cam = RtxCamera(
    "/World/camera",
    tick_rate=30.0,
    schemas=["OmniSensorGenericCameraCoreAPI"],
    attributes={
        # --- Sensor model identification (required to enable simulation) ---
        "omni:sensor:modelName": "CameraCore",
        "omni:sensor:modelVendor": "NVIDIA",
        "omni:sensor:marketName": "Generic",
        # --- Color Correction ---
        "omni:sensor:core:colorCorrectionBlack": 0.0,
        "omni:sensor:core:colorCorrectionFullwellBlack": 1.0,
        "omni:sensor:core:colorCorrectionSensorResponseScale": 1.0,
        "omni:sensor:core:colorCorrectionWhiteBalance": [1.0, 1.0, 1.0],
        # Color correction matrix (identity = no color shift)
        "omni:sensor:core:colorCorrectionMatrixRr": 1.0,
        "omni:sensor:core:colorCorrectionMatrixRg": 0.0,
        "omni:sensor:core:colorCorrectionMatrixRb": 0.0,
        "omni:sensor:core:colorCorrectionMatrixGr": 0.0,
        "omni:sensor:core:colorCorrectionMatrixGg": 1.0,
        "omni:sensor:core:colorCorrectionMatrixGb": 0.0,
        "omni:sensor:core:colorCorrectionMatrixBr": 0.0,
        "omni:sensor:core:colorCorrectionMatrixBg": 0.0,
        "omni:sensor:core:colorCorrectionMatrixBb": 1.0,
        # --- CFA (Color Filter Array) ---
        "omni:sensor:core:colorFilterArrayCfaSemantic": "GRBG",
        "omni:sensor:core:colorFilterArrayCfaCf00": [0.0, 1.0, 0.0],  # Green
        "omni:sensor:core:colorFilterArrayCfaCf01": [1.0, 0.0, 0.0],  # Red
        "omni:sensor:core:colorFilterArrayCfaCf10": [0.0, 0.0, 1.0],  # Blue
        "omni:sensor:core:colorFilterArrayCfaCf11": [0.0, 1.0, 0.0],  # Green
        # --- ISP (Smodel) program ---
        "omni:sensor:core:ispSmodelCameraProgram": _isp_program_b64,
        # --- Introspection: file output ---
        "omni:sensor:core:introspectionOutputFile": True,
        "omni:sensor:core:introspectionOutputFileDirectory": args.output_dir,
        "omni:sensor:core:introspectionOutputFileEachFrameOneFile": False,
        "omni:sensor:core:introspectionOutputFileOnlyLastFrame": True,
    },
    positions=np.array([0.0, 0.0, 0.5]),
    orientations=np.array([0.5, 0.5, -0.5, -0.5]),  # wxyz, face +X toward the cubes
)

# Set camera optical parameters.
# set_focal_lengths has a 10x multiplier (mm → USD tenths-of-mm), so pass
# 1.817 to get a USD value of 18.17.
cam.camera.set_focal_lengths(1.817)
cam.camera.set_clipping_ranges(0.01, 1000.0)
cam.camera.set_focus_distances(400.0)

print(f"Created camera with ISP pipeline at {cam.paths[0]}")
print(f"  CFA pattern: GRBG")
print(f"  Color correction: identity matrix, white balance [1,1,1]")
print(f"  Introspection output: {args.output_dir}")

# =============================================================================
# CREATE SENSOR AND RENDER
# =============================================================================

# The ISP pipeline requires HdrColor (input) and at least one of
# OmniCameraSensorPreIsp (raw Bayer) or OmniCameraSensorPostIsp (processed RGB)
# as render variables on the render product.
sensor = CameraSensor(
    cam,
    resolution=(480, 640),
    annotators=["rgb"],
    render_vars=["HdrColor", "OmniCameraSensorPreIsp", "OmniCameraSensorPostIsp"],
)

if args.test:
    stage.Export(os.path.join(args.output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

frame_count = 0
saved = False
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    if not saved:
        data, info = sensor.get_data("rgb")
        if data is not None:
            rgb_np = data.numpy()
            if rgb_np.max() > 0:
                saved = True
                print(f"\nFrame {frame_count}:")
                print(f"  RGB shape: {rgb_np.shape}, dtype: {rgb_np.dtype}")
                print(f"  Value range: [{rgb_np.min()}, {rgb_np.max()}]")

                out_path = os.path.join(args.output_dir, "isp_rgb_output.png")
                cv2.imwrite(out_path, cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR))
                print(f"  Saved: {out_path}")

    if frame_count >= NUM_FRAMES:
        break

if not saved:
    print("\nWarning: no non-black frame was captured in the allotted frames.")

# List any binary files the ISP introspection stage wrote, then convert to images.
introspection_files = [f for f in os.listdir(args.output_dir) if f.endswith(".bin")]
if introspection_files:
    print(f"\nISP introspection outputs in {args.output_dir}:")
    for f in sorted(introspection_files):
        size = os.path.getsize(os.path.join(args.output_dir, f))
        print(f"  {f}  ({size} bytes)")

    height, width = sensor.resolution
    _convert_isp_bins_to_images(args.output_dir, height, width, "GRBG")

timeline.stop()
simulation_app.close()
