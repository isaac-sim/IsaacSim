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

# headless=False to visualize the scene and debug view
simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "apply_nonvisual_materials")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.objects import Cube, DistantLight
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

# =============================================================================
# DEFINE SCENE OBJECTS WITH DIFFERENT MATERIALS
# =============================================================================
# Each cube will have a different combination of:
# - Visual color (how it appears in the viewport)
# - Non-visual material (how RTX sensors perceive it)

cube_configs = [
    {
        "path": "/World/cube_0",
        "position": np.array([3, 3, 0.5]),
        "scale": np.array([1, 5, 1]),
        "color": [1, 0, 0],  # Red (visual)
        "base": "aluminum",
        "coating": "paint",
        "attribute": "emissive",
    },
    {
        "path": "/World/cube_1",
        "position": np.array([3, -3, 0.5]),
        "scale": np.array([1, 1, 1]),
        "color": [0, 1, 0],  # Green (visual)
        "base": "steel",
        "coating": "clearcoat",
        "attribute": "emissive",
    },
    {
        "path": "/World/cube_2",
        "position": np.array([-3, 3, 0.5]),
        "scale": np.array([1, 1, 1]),
        "color": [0, 0, 1],  # Blue (visual)
        "base": "concrete",
        "coating": "paint",
        "attribute": "emissive",
    },
    {
        "path": "/World/cube_3",
        "position": np.array([-3, -3, 0.5]),
        "scale": np.array([5, 1, 1]),
        "color": [1, 1, 0],  # Yellow (visual)
        "base": "concrete",
        "coating": "clearcoat",
        "attribute": "retroreflective",
    },
]

# =============================================================================
# CREATE LIGHTING
# =============================================================================
light = DistantLight("/World/light")
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

material_ids = {}

for config in cube_configs:
    # Create the cube
    cube = Cube(
        config["path"],
        positions=config["position"],
        scales=config["scale"],
        colors=config["color"],
    )

    # Create a non-visual material and apply it to the cube
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
    translations=np.array([0, 0, 0.5]),
    aux_output_level="FULL",
)

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
    """Writer that parses GenericModelOutput and prints intensity stats."""

    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]

    def write(self, data: dict[str, object]) -> None:
        """Inspect GenericModelOutput material intensity data."""
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements > 0:
                print(
                    f"{gmo.numElements} points, "
                    f"intensity min={gmo.scalar.min():.4f}, "
                    f"max={gmo.scalar.max():.4f}, "
                    f"mean={gmo.scalar.mean():.4f}"
                )


rep.WriterRegistry.register(GmoMaterialInspectWriter)
sensor.attach_writer("GmoMaterialInspectWriter")

print("Attached GmoMaterialInspectWriter to sensor")

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
if args.test:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 10):
    simulation_app.update()
    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
