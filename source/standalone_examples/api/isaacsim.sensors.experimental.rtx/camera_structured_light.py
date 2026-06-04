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

"""Capture a structured-light pattern sequence using Replicator Orchestrator.

This example demonstrates how to:

- Create a :class:`StructuredLightCamera` with a set of rational-time projector
  timestamps spanning 0 - 2 ms with variable spacing.
- Drive the Replicator Orchestrator with ``rep.orchestrator.step`` and a
  per-step ``delta_time`` equal to the interval between consecutive timestamps,
  so one RGB image is captured per projector pattern.
- Attach a Replicator :class:`BasicWriter` to the sensor's render product so
  RGB frames are written to disk automatically (``rgb_0000.png``,
  ``rgb_0001.png``, ...) without any custom image-save plumbing.

By default the example loads the structured-light patterns and projector
direction texture shipped with the :mod:`isaacsim.sensors.experimental.rtx`
extension (``tests/data/structured_light_camera/``). Override with
``--pattern-dir`` and ``--direction-texture`` to use custom assets for a
production capture workflow. Remote asset URIs (e.g.
``omniverse://server/...``) are supported; the example does not probe remote
paths for existence before passing them to USD.

Pass ``--test`` to additionally export the test stage to ``stage.usda`` in
the output directory (matching other examples in this directory). The
capture loop runs all 10 iterations either way.
"""

from __future__ import annotations

import argparse
import os
from fractions import Fraction
from pathlib import Path

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Structured-light pattern capture example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
parser.add_argument(
    "--pattern-dir",
    default=None,
    help="Directory containing pattern files named image_00.png .. image_09.png. "
    "Accepts local filesystem paths or asset-resolver URIs (e.g. omniverse://). "
    "If not provided, the example loads the bundled patterns from the extension's "
    "tests/data directory.",
)
parser.add_argument(
    "--direction-texture",
    default=None,
    help="Path to the projector direction texture (EXR). "
    "Accepts local filesystem paths or asset-resolver URIs (e.g. omniverse://). "
    "If not provided, the example loads the bundled direction texture from the "
    "extension's tests/data directory.",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_structured_light")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.experimental.rtx import CameraSensor, StructuredLightCamera
from pxr import Gf, UsdLux

# ------------------------------------------------------------------------------
# Pattern schedule
# ------------------------------------------------------------------------------
# 10 patterns with variable intervals spanning 0 .. 2 ms. Rational tuples avoid
# floating-point precision issues at sub-millisecond resolution.
TIMESTAMPS: list[tuple[int, int]] = [
    (0, 1),  # 0.000 ms  — projector 0
    (19, 100_000),  # 0.190 ms  — projector 1
    (41, 100_000),  # 0.410 ms  — projector 2
    (62, 100_000),  # 0.620 ms  — projector 3
    (4, 5_000),  # 0.800 ms  — projector 4
    (101, 100_000),  # 1.010 ms  — projector 5
    (61, 50_000),  # 1.220 ms  — projector 6
    (141, 100_000),  # 1.410 ms  — projector 7
    (179, 100_000),  # 1.790 ms  — projector 8
    (1, 500),  # 2.000 ms  — projector 9
]
NUM_PATTERNS = len(TIMESTAMPS)

RESOLUTION = (480, 640)  # (height, width)

# Default projector direction texture filename shipped with the extension.
_DEFAULT_DIRECTION_TEXTURE = "projector_opencv_pinhole_4000x2880_2025_10_08_10_51_18.exr"

# Calibrated Camera intrinsics matching the bundled projector direction texture (Zivid-style pinhole).
CAMERA_INTRINSICS: dict[str, object] = {
    "focalLength": 18.147562,
    "focusDistance": 400.0,
    "fStop": 0.0,
    "horizontalAperture": 20.955,
    "verticalAperture": 15.2908,
    "clippingRange": Gf.Vec2f(1.0, 10_000_000.0),
    "omni:lensdistortion:opencvPinhole:cx": 1226.726,
    "omni:lensdistortion:opencvPinhole:cy": 1067.802,
    "omni:lensdistortion:opencvPinhole:fx": 4025.18,
    "omni:lensdistortion:opencvPinhole:fy": 4023.9,
    "omni:lensdistortion:opencvPinhole:imageSize": Gf.Vec2i(2472, 2064),
    "omni:lensdistortion:opencvPinhole:p1": 0.01258562,
    "omni:lensdistortion:opencvPinhole:p2": -0.0007647774,
    "omni:lensdistortion:opencvPinhole:k1": 0.0003653244,
    "omni:lensdistortion:opencvPinhole:k2": 0.0,
    "omni:lensdistortion:opencvPinhole:k3": -0.0116704,
    "omni:lensdistortion:opencvPinhole:k4": 0.0,
}


# ------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------


def _is_local_path(value: str) -> bool:
    """Return True if ``value`` looks like a local filesystem path (no URI scheme)."""
    return "://" not in value


def _get_bundled_data_dir() -> Path:
    """Return the directory containing the bundled structured-light test assets.

    The assets live under the ``isaacsim.sensors.experimental.rtx`` extension at
    ``tests/data/structured_light_camera/``.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_path = ext_manager.get_extension_path_by_module("isaacsim.sensors.experimental.rtx")
    if not ext_path:
        raise RuntimeError(
            "Unable to locate the 'isaacsim.sensors.experimental.rtx' extension. "
            "Ensure it is enabled before running this example."
        )
    bundled = (
        Path(ext_path) / "isaacsim" / "sensors" / "experimental" / "rtx" / "tests" / "data" / "structured_light_camera"
    )
    if not bundled.is_dir():
        raise FileNotFoundError(
            f"Bundled structured-light data directory not found: {bundled}. "
            "Pass --pattern-dir and --direction-texture to supply custom assets."
        )
    return bundled


def _resolve_pattern_files() -> list[str | Path]:
    """Resolve the projector pattern paths from CLI args or bundled defaults."""
    if args.pattern_dir is not None:
        if _is_local_path(args.pattern_dir):
            pattern_dir_path = Path(args.pattern_dir)
            files: list[str | Path] = [pattern_dir_path / f"image_{i:02d}.png" for i in range(NUM_PATTERNS)]
            for p in files:
                if not Path(p).exists():
                    raise FileNotFoundError(f"Pattern file not found: {p}")
            return files
        # URI-style path — pass through to USD's asset resolver verbatim.
        base = args.pattern_dir.rstrip("/")
        return [f"{base}/image_{i:02d}.png" for i in range(NUM_PATTERNS)]
    bundled = _get_bundled_data_dir()
    files = [bundled / "patterns" / f"image_{i:02d}.png" for i in range(NUM_PATTERNS)]
    for p in files:
        if not Path(p).exists():
            raise FileNotFoundError(f"Bundled pattern file not found: {p}")
    return files


def _resolve_direction_texture() -> str | Path:
    """Resolve the projector direction texture from CLI args or bundled default."""
    if args.direction_texture is not None:
        if _is_local_path(args.direction_texture):
            direction_texture: str | Path = Path(args.direction_texture)
            if not Path(direction_texture).exists():
                raise FileNotFoundError(f"Direction texture not found: {direction_texture}")
            return direction_texture
        return args.direction_texture
    bundled = _get_bundled_data_dir()
    direction_texture = bundled / _DEFAULT_DIRECTION_TEXTURE
    if not Path(direction_texture).exists():
        raise FileNotFoundError(f"Bundled direction texture not found: {direction_texture}")
    return direction_texture


# ------------------------------------------------------------------------------
# Scene
# ------------------------------------------------------------------------------
# A large
# (1000-unit) white cube acts as an enclosure, with the camera and coincident
# projector at the origin. Because the projector light travels ~500 units before
# hitting a wall, the bundled structured-light patterns render with real
# contrast without saturating the tone-mapped RGB capture.

stage = omni.usd.get_context().get_stage()
stage.DefinePrim("/World", "Xform")

enclosure = Cube("/World/Cube", sizes=1.0, scales=np.array([1000.0, 1000.0, 1000.0]))
enclosure_material = OmniPbrMaterial("/World/Looks/wall_white_diffuse")
enclosure_material.set_input_values(name="diffuse_color_constant", values=[1.0, 1.0, 1.0])
enclosure.apply_visual_materials(materials=[enclosure_material])

# Ambient dome light (low intensity — the projector is the primary light source).
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(100.0)

# ------------------------------------------------------------------------------
# Camera + structured light projector
# ------------------------------------------------------------------------------
# The camera and projector sit at the origin, looking down -Z toward the
# enclosure's -Z wall (~500 units away). Intensity 150000 is appropriate for this scale.
# Standard UsdGeom.Camera attributes are authored via the constructor's ``attributes`` kwarg.

cam = StructuredLightCamera(
    "/World/camera",
    projector_light_patterns=_resolve_pattern_files(),
    projector_direction_texture=_resolve_direction_texture(),
    projector_timestamps=TIMESTAMPS,
    projector_intensity=150_000.0,
    projector_prim_path="/World/projector",
    projector_position=np.array([0.0, 0.0, 0.0]),
    projector_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    schemas=["OmniLensDistortionOpenCvPinholeAPI"],
    attributes=CAMERA_INTRINSICS,
)

sensor = CameraSensor(cam, resolution=RESOLUTION, annotators=[])
sensor.attach_writer("BasicWriter", output_dir=output_dir, rgb=True)

print(f"Created StructuredLightCamera at {cam.paths[0]}")
print(f"  Pattern count:   {cam.get_num_patterns()}")
print(f"  Cycle period:    {cam.get_projector_cycle_period()} s (rational)")
print(f"  Output dir:      {output_dir}")

if args.test:
    stage.Export(os.path.join(output_dir, "stage.usda"))

# ------------------------------------------------------------------------------
# Replicator capture loop
# ------------------------------------------------------------------------------
# Per-pattern deltas: the first step uses delta_time=0.0 (Replicator never
# advances time on the first captured frame), each subsequent step advances
# the timeline by the interval between consecutive timestamps.
fracs = [Fraction(*ts) for ts in TIMESTAMPS]
intervals = [fracs[0]] + [fracs[i] - fracs[i - 1] for i in range(1, NUM_PATTERNS)]

for pattern_idx, interval in enumerate(intervals):
    rep.orchestrator.step(rt_subframes=32, delta_time=float(interval))
    active = cam.get_active_pattern_index()
    print(
        f"Pattern {pattern_idx}: active={active}, interval={float(interval) * 1e3:.3f} ms "
        f"(BasicWriter frame rgb_{pattern_idx:04d}.png)"
    )

print(f"\nCapture complete. {NUM_PATTERNS} pattern frames written to {output_dir}.")

# ------------------------------------------------------------------------------
# Tear down
# ------------------------------------------------------------------------------
rep.orchestrator.stop()
rep.orchestrator.wait_until_complete()
simulation_app.close()
