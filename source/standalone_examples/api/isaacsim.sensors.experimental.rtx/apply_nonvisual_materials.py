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

"""Apply non-visual materials to scene prims for RTX sensor simulation.

This example demonstrates how to:
- Create cubes using ``isaacsim.core.experimental.objects.Cube``
- Apply non-visual materials using ``isaacsim.core.experimental.materials.NonVisualMaterial``
- Encode non-visual material IDs using ``NonVisualMaterial.encode_material_ids()``
- Create a lidar with ``Lidar.create()`` and a ``LidarSensor``
- Attach a custom Writer to receive and inspect GMO intensity data
- Observe how different non-visual materials affect lidar intensity readings

Non-visual materials affect how RTX sensors (lidar, radar) perceive objects,
independent of their visual appearance. This allows:
- Simulating different physical material properties (metal, glass, rubber, etc.)
- Testing sensor behavior with various surface coatings (paint, clearcoat)
- Simulating special material attributes (emissive, retroreflective, transparent)

Available base materials include:
    Metals: aluminum, steel, iron, silver, brass, bronze, etc.
    Polymers: plastic, fiberglass, carbon_fiber, vinyl, nylon, etc.
    Glass: clear_glass, frosted_glass, one_way_mirror, mirror, etc.
    Other: asphalt, concrete, rubber, wood, fabric, leather, etc.

Available coatings: none, paint, clearcoat, paint_clearcoat

Available attributes: none, emissive, retroreflective, single_sided, visually_transparent

To visualize non-visual materials in the viewport:
    RTX - Real-Time 2.0 (in viewport) > Debug View > Non-Visual Material ID
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Apply non-visual materials for RTX sensors.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=False to visualize the scene and debug view.
# stableIds must be enabled so the lidar GMO carries per-point object IDs (StableIdMap),
# which the writer uses to bin return intensity per cube.
simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": ["--/rtx-transient/stableIds/enabled=true"],
    }
)

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "apply_nonvisual_materials")
os.makedirs(output_dir, exist_ok=True)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.replicator.core as rep
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.objects import Cube, DistantLight
from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
    parse_object_ids,
    parse_stable_id_map_data,
)
from omni.replicator.core import Writer

# isaacsim.sensors.rtx.nodes provides register_scalar_colored_point_cloud_writer
app_utils.enable_extension("isaacsim.sensors.rtx.nodes")

from isaacsim.sensors.rtx.nodes import register_scalar_colored_point_cloud_writer

# =============================================================================
# DEFINE SCENE OBJECTS WITH DIFFERENT MATERIALS
# =============================================================================
# Each cube will have a different combination of:
# - Visual color (how it appears in the viewport)
# - Non-visual material (how RTX sensors perceive it)

# Each cube is tuned to demonstrate a distinct RTX-sensor material effect so the
# per-object intensity binning below shows clearly separated populations:
#   cube_0: retroreflective sign -> very high, angle-independent intensity
#   cube_1: calibration Lambertian -> strong, known, consistent diffuse intensity
#   cube_2: transparent glass -> weak, sparse returns + pass-through (multi-return)
#   cube_3: frosted glass -> noisy/scattered intensity from volumetric imperfections
cube_configs = [
    {
        "path": "/World/cube_0",
        "position": np.array([3, 3, 0.5]),
        "scale": np.array([1, 10, 1]),
        "color": [1, 0, 0],  # Red (visual)
        "base": "aluminum",
        "coating": "paint",
        "attribute": "retroreflective",  # HIGH intensity: bright even at grazing angles
    },
    {
        "path": "/World/cube_1",
        "position": np.array([-7, 7, 0.5]),
        "scale": np.array([1, 1, 1]),
        "color": [0, 1, 0],  # Green (visual)
        "base": "calibration_lambertian",  # MID-HIGH: controlled, consistent diffuse return
        "coating": "none",
        "attribute": "none",
    },
    {
        "path": "/World/cube_2",
        "position": np.array([-3, 3, 0.5]),
        "scale": np.array([1, 1, 1]),
        "color": [0, 0, 1],  # Blue (visual)
        "base": "clear_glass",  # TRANSPARENCY: weak returns, beams pass through
        "coating": "none",
        "attribute": "visually_transparent",
    },
    {
        "path": "/World/cube_3",
        "position": np.array([-3, -3, 0.5]),
        "scale": np.array([10, 1, 1]),
        "color": [1, 1, 0],  # Yellow (visual)
        "base": "frosted_glass",  # NOISE: volumetric particulates scatter the beam
        "coating": "none",
        "attribute": "none",
    },
]


# =============================================================================
# CREATE LIGHTING
# =============================================================================
def _emission_orientation(direction):
    """wxyz quaternion rotating a DistantLight's default -Z emission axis to `direction`."""
    d = np.asarray(direction, dtype=float)
    d /= np.linalg.norm(d)
    default = np.array([0.0, 0.0, -1.0])
    axis = np.cross(default, d)
    axis_norm = np.linalg.norm(axis)
    dot = float(np.clip(np.dot(default, d), -1.0, 1.0))
    if axis_norm < 1e-8:
        # Parallel (identity) or anti-parallel (180 deg about X).
        return np.array([1.0, 0.0, 0.0, 0.0]) if dot > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    axis /= axis_norm
    half = np.arccos(dot) / 2.0
    xyz = axis * np.sin(half)
    return np.array([np.cos(half), xyz[0], xyz[1], xyz[2]])


# Angle the sun so it comes from above and rakes "inward" toward cube_2 (at [-3, 3]):
# photons travel down (-Z) and toward -X / +Y.
light = DistantLight(
    "/World/light",
    orientations=_emission_orientation([-1.0, 1.0, -1.0]),
)
light.set_intensities(3000.0)

# =============================================================================
# CREATE CUBES WITH NON-VISUAL MATERIALS
# =============================================================================
# The experimental ``Cube`` and ``NonVisualMaterial`` classes provide a clean API
# for constructing scene objects and assigning material properties. The pattern is:
#
#   1. Create a ``Cube`` at the desired path
#   2. Create a ``NonVisualMaterial`` as a child of the cube
#   3. Apply the material to the cube via ``cube.apply_visual_materials(material)``
#   4. Optionally encode the material ID for later comparison with GMO data

print(f"\n{'='*60}")
print("Creating cubes with non-visual materials")
print(f"{'='*60}")

stage = stage_utils.get_current_stage(backend="usd")

material_ids = {}

for config in cube_configs:
    # Create the cube
    cube = Cube(
        config["path"],
        positions=config["position"],
        scales=config["scale"],
        colors=config["color"],
    )

    # Create a non-visual material and apply it to the cube. The material carries only the SimReady
    # semantic attributes; no surface shader is needed for the renderer to resolve the material ID.
    material = NonVisualMaterial(
        f"{config['path']}/material",
        bases=config["base"],
        coatings=config["coating"],
        attributes=config["attribute"],
    )
    cube.apply_visual_materials(material)

    # Encode the material ID (useful for matching against GMO materialId field)
    material_id = NonVisualMaterial.encode_material_ids(material).numpy().item()
    material_ids[config["path"]] = material_id

    print(f"  {config['path']}:")
    print(f"    Visual color: {config['color']}")
    print(f"    Non-visual: base={config['base']}, coating={config['coating']}, attribute={config['attribute']}")
    print(f"    Encoded material ID: {material_id}")

# =============================================================================
# CREATE RTX LIDAR WITH GMO ANNOTATOR
# =============================================================================
# Create a lidar sensor positioned at the center of the scene, pointing outward.
# The ``generic-model-output`` annotator provides raw sensor data including
# intensity, which varies by surface material.

lidar = Lidar.create(
    "/World/lidar",
    config="Example_Rotary",
    translations=np.array([-5, 5, 0.5]),
    aux_output_level="FULL",
    attributes={
        "omni:sensor:Core:maxReturns": 3,
        "omni:sensor:Core:peakPowerW": 5.0,
    },
)

# Export the fully-authored scene (geometry + shader-connected materials + lidar prim) now, before
# wrapping the lidar in a LidarSensor or attaching any writers.
if args.test:
    stage_path = os.path.join(output_dir, "stage.usda")
    stage.Export(stage_path)
    print(f"Exported stage to {stage_path}")

sensor = LidarSensor(lidar, annotators=[])

print(f"\n{'='*60}")
print(f"Created RTX Lidar at {lidar.paths[0]}")
print(f"{'='*60}")


# =============================================================================
# CUSTOM WRITER FOR GMO MATERIAL INSPECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings its own ``GenericModelOutput`` annotator, so the sensor
# does not need to specify one.


class GmoMaterialInspectWriter(Writer):
    """Writer that parses GenericModelOutput and prints one-time overall + per-cube intensity histograms."""

    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        # StableIdMap resolves each point's objId to the USD prim (cube) that was hit.
        self.annotators = [
            rep.annotators.get("GenericModelOutput"),
            rep.annotators.get("StableIdMap"),
        ]
        # Guard so the histograms are only emitted once, on the first frame with returns.
        self._histogram_done = False

    def write(self, data: dict[str, object]) -> None:
        """Inspect GenericModelOutput material intensity data.

        Args:
            data: Writer payload containing GenericModelOutput data grouped by render product.
        """
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements == 0:
                continue

            if self._histogram_done:
                continue

            intensities = np.asarray(gmo.scalar, dtype=np.float32)
            print(
                f"{gmo.numElements} points, "
                f"intensity min={intensities.min():.4f}, "
                f"max={intensities.max():.4f}, "
                f"mean={intensities.mean():.4f}"
            )

            # Overall distribution across all returns.
            self._print_intensity_histogram(intensities, title="Return intensity histogram (all returns)")

            # Per-cube distribution, keyed by the stable object ID of each return.
            sid_raw = rp_data.get("StableIdMap")
            if isinstance(sid_raw, dict):
                sid_raw = sid_raw.get("data")
            has_obj_ids = getattr(gmo, "objId", None) is not None and gmo.objId.size > 0
            if sid_raw is not None and has_obj_ids:
                stable_id_map = parse_stable_id_map_data(sid_raw)
                obj_ids = np.asarray(parse_object_ids(gmo.objId), dtype=object)
                # Order cubes by descending mean intensity to make the material spread obvious.
                unique_ids = list(np.unique(obj_ids))
                unique_ids.sort(key=lambda oid: float(intensities[obj_ids == oid].mean()), reverse=True)
                for oid in unique_ids:
                    mask = obj_ids == oid
                    prim_path = stable_id_map.get(int(oid), "<unknown>")
                    self._print_intensity_histogram(
                        intensities[mask],
                        title=f"{prim_path}  (object ID {int(oid)})",
                    )
            else:
                print(
                    "  (per-cube binning unavailable: needs aux_output_level='FULL' and "
                    "--/rtx-transient/stableIds/enabled=true)"
                )

            self._histogram_done = True

    @staticmethod
    def _print_intensity_histogram(
        intensities: np.ndarray, title: str = "Return intensity histogram", num_bins: int = 20, width: int = 50
    ) -> None:
        """Print an ASCII histogram of return intensity to the console."""
        counts, edges = np.histogram(intensities, bins=num_bins)
        peak = int(counts.max()) if counts.size else 0
        print(f"\n{'='*72}")
        print(f"{title}  [{intensities.size} points, mean={float(intensities.mean()):.4f}]")
        print(f"{'='*72}")
        if peak == 0:
            print("  (no returns to bin)")
            return
        for i, count in enumerate(counts):
            bar = "#" * int(round(width * count / peak))
            print(f"  [{edges[i]:7.4f}, {edges[i + 1]:7.4f})  {bar:<{width}} {int(count)}")
        print(f"{'='*72}\n")


rep.WriterRegistry.register(GmoMaterialInspectWriter)
sensor.attach_writer("GmoMaterialInspectWriter")

print("Attached GmoMaterialInspectWriter to sensor")

# Attach a debug draw point cloud writer that colors returns by per-point intensity.
# register_scalar_colored_point_cloud_writer builds the
# IsaacExtractRTXSensorPointCloud -> IsaacMapScalarsToColors -> DebugDrawPointCloud graph.
intensity_writer = register_scalar_colored_point_cloud_writer(scalar="intensity")
sensor.attach_writer(intensity_writer, size=0.05)  # Point size in meters

print(f"Attached {intensity_writer} (point cloud colored by intensity) to sensor")

# =============================================================================
# INSTRUCTIONS FOR VIEWING NON-VISUAL MATERIALS
# =============================================================================
print(f"\n{'='*60}")
print("To visualize non-visual materials in the viewport:")
print("  1. In the viewport, click the render mode dropdown")
print("  2. Select 'RTX - Real-Time 2.0'")
print("  3. Click 'Debug View' dropdown")
print("  4. Select 'Non-Visual Material ID'")
print("Each material will appear as a different color in this view.")
print(f"{'='*60}\n")

# =============================================================================
# RUN SIMULATION AND PRINT INTENSITY DATA
# =============================================================================
app_utils.play()

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 10):
    simulation_app.update()
    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
app_utils.stop()
simulation_app.close()
