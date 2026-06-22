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

"""Verifies that rendered RGB, segmentation, bounding boxes, and reference-time metadata match the latest moved cube state. Detects frame delay by comparing expected image-space positions and renderer reference time against live simulation time."""

USE_REPLICATOR_WRITER = True
CAMERA_PATH = "/camera"
CAMERA_POS = [0, 0, 25]
COLLECTION_STEPS = 10

# Maximum allowed gap between the action-graph reference time
# (SimulationManager.get_simulation_time()) and the image's reference time
# (getSimulationTimeAtTime(rpFabricTime)). 1 us is just above the rational-time
# rounding floor for 1/60 s; anything larger indicates a real frame lag between
# image capture and the sim step the rest of the world thinks is "now".
TIMESTAMP_TOLERANCE_S = 1e-6

# parse any command-line arguments specific to the standalone application
import argparse
import os
import sys
from typing import Any

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--resolution", type=str, default="256x256", help="Resolution (WxH)")
parser.add_argument(
    "--multitick",
    choices=["on", "off", "auto"],
    default="auto",
    help=(
        "Override multitick renderer settings before launching Kit. "
        "'on' forces supportMultiTickRate and perSensorTickTlas, 'off' disables them, 'auto' leaves "
        "whatever the experience .kit file or other CLI overrides specified."
    ),
)
parser.add_argument(
    "--zero-delay",
    choices=["on", "off"],
    default="on",
    help=(
        "Pick the experience .kit file used to launch Kit. 'on' (default) uses "
        "isaacsim.exp.base.zero_delay.kit, which adds app.hydraEngine.waitIdle=true "
        "and a late checkForHydraRenderComplete order on top of the base experience. "
        "'off' falls back to SimulationApp's default base experience, leaving render-"
        "completion timing unconstrained."
    ),
)
# Parse only known arguments, so that any (eg) Kit settings are passed through to the core Kit app
args, _ = parser.parse_known_args()

RESOLUTION = tuple([int(item) for item in args.resolution.split("x")])
PIXELS_PER_METER = 0.09765625 * RESOLUTION[0]

# Forward --multitick to the Kit-side carb settings via Kit's standard
# "--/path=value" CLI override mechanism. SimulationApp/Kit picks these up from
# sys.argv on launch, before the renderer initialises.
if args.multitick != "auto":
    multitick_enabled = "true" if args.multitick == "on" else "false"
    sys.argv.append(f"--/rtx/hydra/supportMultiTickRate={multitick_enabled}")
    sys.argv.append(f"--/rtx/rendering/perSensorTickTlas={multitick_enabled}")

simulation_app_kwargs = {}
if args.zero_delay == "on":
    simulation_app_kwargs["experience"] = f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.zero_delay.kit'

simulation_app = SimulationApp({"headless": True}, **simulation_app_kwargs)

import pprint

import carb
import cv2
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils.semantics import add_labels
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.camera import Camera
from omni.replicator.core import AnnotatorRegistry, Writer
from pxr import UsdGeom, UsdPhysics

# rep.settings.set_render_rtx_realtime(antialiasing="DLAA")


class CustomWriter(Writer):
    """Custom replicator writer that captures RGB, segmentation, and bounding box data."""

    def __init__(self) -> None:
        self.annotators = []
        self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))
        self.annotators.append(AnnotatorRegistry.get_annotator("semantic_segmentation"))
        self.annotators.append(AnnotatorRegistry.get_annotator("bounding_box_2d_tight"))

    def write(self, data: dict[str, Any]) -> None:
        """Cache annotator data without writing to disk."""
        # The base Writer class caches 'data' automatically, accessible via self.get_data()


def get_data(sensor: Camera | Writer) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get RGB, semantic segmentation and BBox from Camera or Writer (according to `USE_REPLICATOR_WRITER`)."""
    if USE_REPLICATOR_WRITER:
        rgb = sensor.get_data()["rgb"]
        semantic_segmentation = sensor.get_data()["semantic_segmentation"]["data"]
        bbox = sensor.get_data()["bounding_box_2d_tight"]["data"][0]
    else:
        rgb = sensor.get_rgba()
        semantic_segmentation = sensor._custom_annotators["semantic_segmentation"].get_data()["data"]
        bbox = sensor._custom_annotators["bounding_box_2d_tight"].get_data()["data"][0]

    semantic_segmentation = (semantic_segmentation * 255 / np.max(semantic_segmentation)).astype(np.uint8)
    semantic_segmentation = np.repeat(semantic_segmentation[:, :, np.newaxis], 3, axis=2)
    return rgb[:, :, :3], semantic_segmentation, bbox


def calculate_expected_bbox(position: np.ndarray) -> dict:
    """Calculate expected 2D bounding box from 3D world position.

    This uses the same projection logic as draw_data to compute where the cube
    should appear in the image based on its world position.

    Args:
        position: 3D world position of the cube center [x, y, z].

    Returns:
        Dictionary with x_min, y_min, x_max, y_max pixel coordinates.
    """
    # Project 3D position to 2D image coordinates
    # The cube has scale [1.0, 2.0, 0.2] and size 1.0, so actual dimensions are [1.0, 2.0, 0.2] meters
    # The bounding box should capture the full extent of the cube
    center_x = RESOLUTION[1] / 2 + PIXELS_PER_METER * position[0]
    center_y = RESOLUTION[1] / 2 + PIXELS_PER_METER * position[1]

    # Half-widths in pixels (cube scale is [1.0, 2.0, 0.2])
    half_width_x = PIXELS_PER_METER / 2  # 0.5 meters on each side
    half_width_y = PIXELS_PER_METER  # 1.0 meters on each side

    return {
        "x_min": int(center_x - half_width_x),
        "y_min": int(center_y - half_width_y),
        "x_max": int(center_x + half_width_x),
        "y_max": int(center_y + half_width_y),
    }


def validate_bbox(detected_bbox: dict, expected_bbox: dict, tolerance_pixels: float = 2.0) -> tuple[bool, float]:
    """Validate that detected bounding box matches expected position within tolerance.

    Args:
        detected_bbox: Detected bounding box from the sensor/writer.
        expected_bbox: Expected bounding box calculated from world position.
        tolerance_pixels: Maximum allowed error in pixels for bbox center.

    Returns:
        Tuple of (is_valid, error_distance) where error_distance is the pixel distance
        between detected and expected bbox centers.
    """
    # Calculate centers
    detected_center_x = (detected_bbox["x_min"] + detected_bbox["x_max"]) / 2
    detected_center_y = (detected_bbox["y_min"] + detected_bbox["y_max"]) / 2

    expected_center_x = (expected_bbox["x_min"] + expected_bbox["x_max"]) / 2
    expected_center_y = (expected_bbox["y_min"] + expected_bbox["y_max"]) / 2

    # Calculate Euclidean distance between centers
    error_distance = np.sqrt(
        (detected_center_x - expected_center_x) ** 2 + (detected_center_y - expected_center_y) ** 2
    )

    is_valid = error_distance <= tolerance_pixels

    return is_valid, error_distance


def draw_data(frame: np.ndarray, position: np.ndarray, bbox: dict[str, int], label: str) -> np.ndarray:
    """Draw position and bounding box annotations onto an image frame."""
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.rectangle(
        img=frame,
        pt1=(
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[0] - PIXELS_PER_METER / 2),
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[1] - PIXELS_PER_METER),
        ),
        pt2=(
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[0] + PIXELS_PER_METER / 2),
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[1] + PIXELS_PER_METER),
        ),
        color=(255, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.rectangle(
        img=frame,
        pt1=(bbox["x_min"], bbox["y_min"]),
        pt2=(bbox["x_max"], bbox["y_max"]),
        color=(0, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=label,
        org=(5, 15),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(0, 255, 255),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=f"position: {(round(position[0], 2), round(position[1], 2), round(position[2], 2))}",
        org=(5, 30),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(255, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=f'bbox: {(bbox["x_min"], bbox["y_min"])} {(bbox["x_max"], bbox["y_max"])}',
        org=(5, 45),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(0, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    return frame


def generate_result(data: list[dict[str, Any]], banner: list[str] | None = None) -> np.ndarray:
    """Generate a composite result image from collected frame data."""
    if banner is None:
        banner = []
    rgb_frames = []
    semantic_segmentation_frames = []
    for item in data:
        rgb_frames.append(draw_data(item["rgb"], item["position"], item["bbox"], item["label"]))
        semantic_segmentation_frames.append(
            draw_data(item["semantic_segmentation"], item["position"], item["bbox"], item["label"])
        )

    separator = np.full((RESOLUTION[0], 5, 3), 0, dtype=np.uint8)
    rgb_frames = [x for item in rgb_frames for x in (item, separator)][:-1]
    semantic_segmentation_frames = [x for item in semantic_segmentation_frames for x in (item, separator)][:-1]

    frame = cv2.vconcat([cv2.hconcat(rgb_frames), cv2.hconcat(semantic_segmentation_frames)])
    if banner:
        frame = cv2.copyMakeBorder(
            frame, top=25, bottom=0, left=0, right=0, borderType=cv2.BORDER_CONSTANT, value=[0] * 3
        )
        frame = cv2.putText(
            img=frame,
            text=", ".join(banner),
            org=(5, 15),
            fontFace=cv2.FONT_HERSHEY_PLAIN,
            fontScale=0.75,
            color=(255, 255, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
    return frame


simulation_app.update()

# Setup scene
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=CAMERA_POS, target=[0, 0, 0])

GroundPlane("/World/ground_plane")

# Create cube with physics
cube_prim = Cube(
    "/cube",
    positions=[[-3.0, 0.0, 0.1]],
    scales=[[1.0, 2.0, 0.2]],
    sizes=1.0,
)
UsdPhysics.RigidBodyAPI.Apply(cube_prim.prims[0])
UsdPhysics.CollisionAPI.Apply(cube_prim.prims[0])
rigid_cube = RigidPrim("/cube")
add_labels(cube_prim.prims[0], labels=["cube"], taxonomy="class")

camera = None
writer = None
if USE_REPLICATOR_WRITER:
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.DefinePrim(CAMERA_PATH, "Camera")
    UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(tuple(CAMERA_POS))
    render_product = rep.create.render_product(str(camera_prim.GetPrimPath()), resolution=RESOLUTION)
else:
    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array(CAMERA_POS),
        resolution=RESOLUTION,
        orientation=transform_utils.euler_angles_to_quaternion(np.array([[90, 90, 0]]), degrees=True).numpy().flatten(),
    )

app_utils.play()

if USE_REPLICATOR_WRITER:
    rep.WriterRegistry.register(CustomWriter)
    writer = rep.WriterRegistry.get("CustomWriter")
    writer.initialize()
    writer.attach([render_product])
    render_product_path = render_product.path
else:
    camera.initialize()
    camera.add_bounding_box_2d_tight_to_frame()
    camera.add_semantic_segmentation_to_frame()
    render_product_path = camera._render_product_path

# Attach a ReferenceTime annotator so we can read the rational time the renderer
# stamped on each frame. Combined with SimulationManager.get_simulation_time_at_time(),
# this lets us recover the simulation time the image was captured at and compare
# it against the live action-graph sim time at the same step.
reference_time_annotator = AnnotatorRegistry.get_annotator("ReferenceTime")
reference_time_annotator.attach([render_product_path])
sim_manager_iface = SimulationManager._simulation_manager_interface


def rational_to_tuple(value: Any) -> tuple[int, int]:
    """Return (numerator, denominator) for a RationalTime-like object."""
    return int(value.numerator), int(value.denominator)


def rational_to_float(value: tuple[int, int]) -> float:
    """Convert a rational tuple to a floating-point value."""
    numerator, denominator = value
    return numerator / denominator


def rational_before(lhs: tuple[int, int], rhs: tuple[int, int], tolerance_s: float = 0.0) -> bool:
    """Return whether lhs is earlier than rhs by more than tolerance_s."""
    return rational_to_float(lhs) < rational_to_float(rhs) - tolerance_s


def format_rational(value: tuple[int, int]) -> str:
    """Format a rational tuple for diagnostic output."""
    numerator, denominator = value
    return f"({numerator}, {denominator})"


def close_and_exit(exit_code: int) -> None:
    """Close SimulationApp and preserve the validation status."""
    simulation_app.close(exit_code=exit_code)
    sys.exit(exit_code)


def get_lookup_status(reference_time: tuple[int, int]) -> tuple[bool, str]:
    """Return whether reference_time is covered by TimeSampleStorage and a diagnostic string."""
    sample_count = sim_manager_iface.get_sample_count()
    if sample_count == 0:
        return False, "TimeSampleStorage has no samples"

    sample_range = sim_manager_iface.get_sample_range()
    if sample_range is None:
        return False, f"TimeSampleStorage has {sample_count} samples but no sample range"

    earliest, latest = sample_range
    earliest_time = rational_to_tuple(earliest)
    latest_time = rational_to_tuple(latest)
    if rational_before(reference_time, earliest_time, TIMESTAMP_TOLERANCE_S) or rational_before(
        latest_time, reference_time, TIMESTAMP_TOLERANCE_S
    ):
        return (
            False,
            f"rpFabricTime={format_rational(reference_time)} is outside TimeSampleStorage range "
            f"{format_rational(earliest_time)}..{format_rational(latest_time)} "
            f"({rational_to_float(earliest_time):.6f}s..{rational_to_float(latest_time):.6f}s)",
        )

    return (
        True,
        f"rpFabricTime={format_rational(reference_time)} is covered by TimeSampleStorage range "
        f"{format_rational(earliest_time)}..{format_rational(latest_time)}",
    )


def get_frame_timestamp() -> tuple[float, float, tuple[int, int], bool, str]:
    """Return (sim_time_now, sim_time_at_frame, reference_time, lookup_valid, lookup_status).

    - sim_time_now: live simulation time, as the action graph would publish for TF/state.
    - sim_time_at_frame: simulation time recovered from the renderer's rational time
      via TimeSampleStorage. With multitick on this is the rational time itself.
      With multitick off this is the result of an exact-match / interpolation lookup
      against per-frame samples authored by simulation_manager.
    - reference_time: the renderer's rational time for this frame (i.e. what
      rpFabricTime carries to the post-render synthdata graph).
    - lookup_valid/status: whether TimeSampleStorage contains samples covering
      reference_time. This catches false passes from get_simulation_time_at_time()
      falling back to current sim time when no lookup sample exists.
    """
    fabric_time_data = reference_time_annotator.get_data()
    numerator = int(fabric_time_data["referenceTimeNumerator"])
    denominator = int(fabric_time_data["referenceTimeDenominator"])
    reference_time = (numerator, denominator)
    sim_time_now = SimulationManager.get_simulation_time()
    lookup_valid, lookup_status = get_lookup_status(reference_time)
    sim_time_at_frame = sim_manager_iface.get_simulation_time_at_time(reference_time)
    return sim_time_now, sim_time_at_frame, reference_time, lookup_valid, lookup_status


# Do some warmup steps
for _ in range(5):
    simulation_app.update()

data = []
validation_errors = []
timestamp_errors = []


def record_timestamps(label: str) -> tuple[float, float, tuple[int, int], bool, str]:
    """Capture sim/frame timestamps for the most recent render and validate alignment.

    Logs the result, appends a timestamp error entry if the gap exceeds
    TIMESTAMP_TOLERANCE_S, and returns the raw values so callers can store them
    on the per-frame data dict.
    """
    sim_time_now, sim_time_at_frame, (num, denom), lookup_valid, lookup_status = get_frame_timestamp()
    delta_s = sim_time_now - sim_time_at_frame
    delta_ms = delta_s * 1000.0
    print(
        f"  [time] {label:>10}: sim_now={sim_time_now:.6f}s, "
        f"sim_at_frame={sim_time_at_frame:.6f}s, "
        f"delta={delta_ms:+.3f} ms, rpFabricTime=({num}, {denom}), lookup={'OK' if lookup_valid else 'MISS'}"
    )
    if not lookup_valid:
        msg = f"Frame '{label}': {lookup_status}"
        timestamp_errors.append(msg)
        print(f"[error] {msg}")
    if abs(delta_s) > TIMESTAMP_TOLERANCE_S:
        n_frames = abs(delta_s) / (1.0 / 60.0) if delta_s else 0.0
        relation = "behind" if delta_s > 0 else "ahead of"
        msg = (
            f"Frame '{label}': image rpFabricTime resolves to {abs(delta_ms):.3f} ms "
            f"({n_frames:.2f} render frames) {relation} live sim time "
            f"(tolerance: {TIMESTAMP_TOLERANCE_S * 1000:.3f} ms)"
        )
        timestamp_errors.append(msg)
        print(f"[error] {msg}")
    return sim_time_now, sim_time_at_frame, (num, denom), lookup_valid, lookup_status


# Get data and object info before running the collection steps
position = rigid_cube.get_world_poses()[0].numpy().flatten()
rgb, semantic_segmentation, bbox = get_data(camera or writer)
sim_time_now, sim_time_at_frame, ref_time, lookup_valid, lookup_status = record_timestamps("before")
data.append(
    {
        "position": position,
        "rgb": rgb,
        "semantic_segmentation": semantic_segmentation,
        "bbox": bbox,
        "label": "before",
        "sim_time_now": sim_time_now,
        "sim_time_at_frame": sim_time_at_frame,
        "reference_time": ref_time,
        "lookup_valid": lookup_valid,
        "lookup_status": lookup_status,
    }
)

# Validate initial frame
expected_bbox = calculate_expected_bbox(position)
is_valid, error_distance = validate_bbox(bbox, expected_bbox, tolerance_pixels=2.0)
if not is_valid:
    error_msg = f"Frame 'before': BBox center error = {error_distance:.2f} pixels (tolerance: 2.0 pixels)"
    validation_errors.append(error_msg)
    print(f"[error] {error_msg}")

# Do some collection steps
for i in range(COLLECTION_STEPS):
    # Move object
    position = rigid_cube.get_world_poses()[0].numpy().flatten()
    position[0] += 0.5
    rigid_cube.set_world_poses(positions=[position])

    # Step the simulation
    simulation_app.update()

    # Get data and object info
    position = rigid_cube.get_world_poses()[0].numpy().flatten()
    rgb, semantic_segmentation, bbox = get_data(camera or writer)
    sim_time_now, sim_time_at_frame, ref_time, lookup_valid, lookup_status = record_timestamps(f"step {i + 1}")
    data.append(
        {
            "position": position,
            "rgb": rgb,
            "semantic_segmentation": semantic_segmentation,
            "bbox": bbox,
            "label": f"step {i + 1}",
            "sim_time_now": sim_time_now,
            "sim_time_at_frame": sim_time_at_frame,
            "reference_time": ref_time,
            "lookup_valid": lookup_valid,
            "lookup_status": lookup_status,
        }
    )

    # Validate this frame
    expected_bbox = calculate_expected_bbox(position)
    is_valid, error_distance = validate_bbox(bbox, expected_bbox, tolerance_pixels=2.0)
    if not is_valid:
        error_msg = f"Frame {i + 1}: BBox center error = {error_distance:.2f} pixels (tolerance: 2.0 pixels)"
        validation_errors.append(error_msg)
        print(f"[error] {error_msg}")

# Export result
banner = [
    f"source: {'rep.Writer' if USE_REPLICATOR_WRITER else 'Camera'}",
    f"zero_delay: {args.zero_delay}",
    f"checkForHydraRenderComplete: {carb.settings.get_settings().get('/app/updateOrder/checkForHydraRenderComplete')}",
    f"app.hydraEngine.waitIdle: {carb.settings.get_settings().get('/app/hydraEngine/waitIdle')}",
    f"rtx.post.aa.op: {carb.settings.get_settings().get('/rtx/post/aa/op')}",
    f"supportMultiTickRate: {carb.settings.get_settings().get('/rtx/hydra/supportMultiTickRate')}",
    f"perSensorTickTlas: {carb.settings.get_settings().get('/rtx/rendering/perSensorTickTlas')}",
]
print("")
pprint.pprint(banner)
print("")
cv2.imwrite(f"result-{args.resolution}.png", generate_result(data, banner))

# Print validation summary
print("\n" + "=" * 80)
print("VALIDATION SUMMARY")
print("=" * 80)

print("\nTimestamp deltas (action-graph sim time minus image rpFabricTime sim time):")
print(f"  {'frame':>10}  {'sim_now (s)':>12}  {'sim_at_frame (s)':>16}  {'delta (ms)':>12}")
max_abs_delta_ms = 0.0
for item in data:
    delta_ms = (item["sim_time_now"] - item["sim_time_at_frame"]) * 1000.0
    max_abs_delta_ms = max(max_abs_delta_ms, abs(delta_ms))
    print(
        f"  {item['label']:>10}  {item['sim_time_now']:12.6f}  " f"{item['sim_time_at_frame']:16.6f}  {delta_ms:+12.3f}"
    )
print(f"  max |delta| = {max_abs_delta_ms:.3f} ms (tolerance: {TIMESTAMP_TOLERANCE_S * 1000:.3f} ms)")

failed = bool(validation_errors) or bool(timestamp_errors)
if failed:
    if validation_errors:
        print(f"\n[fatal] {len(validation_errors)} frame(s) had bounding box errors exceeding tolerance")
        print("\nBBox errors:")
        for error in validation_errors:
            print(f"  - {error}")
        print("\nThis indicates frame delay between object motion and rendered image.")
    if timestamp_errors:
        print(f"\n[fatal] {len(timestamp_errors)} frame(s) had timestamp errors exceeding tolerance")
        print("\nTimestamp errors:")
        for error in timestamp_errors:
            print(f"  - {error}")
        print(
            "\nThis indicates the renderer's rpFabricTime does not resolve to the live sim time, "
            "so any consumer that stamps images via getSimulationTimeAtTime "
            "(e.g. ROS2CameraHelper, IsaacReadSimulationTimeAnnotator) will lag the action graph."
        )
    print("=" * 80)
    close_and_exit(1)
else:
    print(f"\nSUCCESS: All {len(data)} frames passed validation")
    print("Bounding boxes matched expected positions within 2.0 pixel tolerance")
    print(f"Timestamp gaps stayed within {TIMESTAMP_TOLERANCE_S * 1000:.3f} ms")
    print("No frame delay detected between object motion and rendered image")
    print("=" * 80)
    close_and_exit(0)
