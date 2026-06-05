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

"""Apply non-visual materials to scene prims for RTX sensor simulation.

This example demonstrates how to:
- Create visual materials (OmniPBR) for rendering appearance
- Apply non-visual materials that affect RTX sensor behavior
- Use the apply_nonvisual_material() function with base, coating, and attribute parameters

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

parser = argparse.ArgumentParser(description="Apply non-visual materials to scene prims.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=False to visualize the scene and debug view
simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "apply_nonvisual_materials")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni.kit.commands
import omni.timeline
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.rtx import LidarRtx, apply_nonvisual_material, get_gmo_data

# =============================================================================
# DEFINE SCENE OBJECTS WITH DIFFERENT MATERIALS
# =============================================================================
# Each cube will have a different combination of:
# - Visual material (color you see in the viewport)
# - Non-visual material (how RTX sensors perceive it)

cube_configs = [
    {
        "position": np.array([1, 1, 2]),
        "scale": np.array([1, 5, 1]),
        "color": np.array([255, 0, 0]),  # Red (visual)
        "nonvisual": ("aluminum", "paint", "emissive"),  # (base, coating, attribute)
    },
    {
        "position": np.array([1, 1, -2]),
        "scale": np.array([1, 1, 1]),
        "color": np.array([0, 255, 0]),  # Green (visual)
        "nonvisual": ("steel", "clearcoat", "emissive"),
    },
    {
        "position": np.array([-1, 1, -2]),
        "scale": np.array([1, 1, 1]),
        "color": np.array([0, 0, 255]),  # Blue (visual)
        "nonvisual": ("one_way_mirror", "none", "single_sided"),
    },
    {
        "position": np.array([-1, -3, -2]),
        "scale": np.array([5, 1, 1]),
        "color": np.array([255, 255, 0]),  # Yellow (visual)
        "nonvisual": ("concrete", "paint", "emissive"),
    },
]

# =============================================================================
# CREATE LIGHTING
# =============================================================================
# Add a distant light so the visual materials are visible in the viewport

omni.kit.commands.execute(
    "CreatePrim",
    prim_type="DistantLight",
    attributes={"inputs:angle": 1.0, "inputs:intensity": 3000},
)

# =============================================================================
# CREATE CUBES WITH VISUAL AND NON-VISUAL MATERIALS
# =============================================================================
print(f"\n{'='*60}")
print("Creating cubes with visual and non-visual materials")
print(f"{'='*60}")

for i, config in enumerate(cube_configs):
    # Create the visual cube
    cube = VisualCuboid(
        prim_path=f"/World/cube_{i}",
        name=f"cube_{i}",
        position=config["position"],
        color=config["color"],
        scale=config["scale"],
    )

    # Create a visual material (OmniPBR) for rendering appearance
    visual_material = OmniPBR(
        prim_path=f"/Looks/cube_{i}/material",
        name=f"cube_{i}_material",
        color=config["color"],
    )

    # Apply non-visual material properties to the visual material prim
    # This affects how RTX sensors perceive the object
    base, coating, attribute = config["nonvisual"]
    apply_nonvisual_material(
        prim=visual_material.prim,
        base=base,
        coating=coating,
        attribute=attribute,
    )

    # Apply the material to the cube
    cube.apply_visual_material(visual_material)

    print(f"Cube {i}:")
    print(f"    Visual color: RGB{tuple(config['color'])}")
    print(f"    Non-visual: base={base}, coating={coating}, attribute={attribute}")

# =============================================================================
# CREATE RTX LIDAR WITH GMO ANNOTATOR
# =============================================================================
# Create an RTX Lidar to sense the cubes with different non-visual materials.
# The GenericModelOutput (GMO) annotator provides raw sensor data including intensity.

ANNOTATOR_NAME = "GenericModelOutput"

lidar = LidarRtx(
    prim_path="/World/lidar",
    name="lidar",
    position=np.array([0.5, 0, -1]),
)
lidar.initialize()
lidar.attach_annotator(ANNOTATOR_NAME)

print(f"\n{'='*60}")
print(f"Created RTX Lidar at position (0.5, 0, -1)")
print(f"Attached {ANNOTATOR_NAME} annotator for intensity data")
print(f"{'='*60}")

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

if args.test:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION AND PRINT INTENSITY DATA
# =============================================================================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 10):
    simulation_app.update()

    # Get GMO data from the lidar
    data = lidar.get_current_frame()[ANNOTATOR_NAME]
    if len(data) > 0:
        gmo = get_gmo_data(data)
        if gmo.numElements > 0:
            # Intensity is stored in the 'scalar' field of GMO
            intensity = gmo.scalar
            print(
                f"Frame {frame_count}: {gmo.numElements} points, "
                f"intensity min={intensity.min():.4f}, max={intensity.max():.4f}, mean={intensity.mean():.4f}"
            )

    frame_count += 1

timeline.stop()

# =============================================================================
# CLEANUP
# =============================================================================
simulation_app.close()
