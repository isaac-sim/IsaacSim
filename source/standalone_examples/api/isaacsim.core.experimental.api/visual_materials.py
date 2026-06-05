# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate creating and applying visual materials using the experimental API.

This example demonstrates how to create and apply visual materials to primitives using the Isaac Sim core
(experimental) API.

The example serves to illustrate the following concepts:
- How to create OmniPBR materials with textures and color properties.
- How to create OmniGlass materials with refractive properties.
- How to apply materials to primitives.
- How to modify material properties after application.
- How to configure texture mapping parameters (UVW projection, scale, translation).

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Stage creation and population.
3. Example logic.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse
import random
import sys

import carb

# 1. --------------------------------------------------------------------


parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.materials import OmniGlassMaterial, OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.storage.native import get_assets_root_path

# Verify assets path is available
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Materials/Textures/Synthetic/bubbles_2.png"

# 2. --------------------------------------------------------------------

# Setup stage programatically:
# - Create a new stage
stage_utils.create_new_stage()
# - Add ground plane
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
# - Add distant light
distant_light = DistantLight("/World/DistantLight")
distant_light.set_intensities(300)

# Create textured material (`OmniPBR`).
# - Create an `OmniPBR` material instance at the specified USD path
textured_material = OmniPbrMaterial("/World/visual_cube_material")
# - Set the base diffuse color to red (RGB: 1.0, 0.0, 0.0)
textured_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])  # - Red color
# - Set the diffuse texture path to the `bubbles_2.png` image
textured_material.set_input_values("diffuse_texture", asset_path)
# - Enable UVW projection to allow texture mapping on primitives without UV coordinates (like cubes).
textured_material.set_input_values("project_uvw", True)
# - Set texture scale to control repetition (1.0, 1.0 means no scaling/repetition)
textured_material.set_input_values("texture_scale", [1.0, 1.0])
# - Set texture translation to offset the texture position (0.5 units in U direction, 0 in V direction).
textured_material.set_input_values("texture_translate", [0.5, 0])

# Create glass material (`OmniGlass`).
# - Generate a random RGB color for the glass material
glass_color = [random.random(), random.random(), random.random()]
# - Create an `OmniGlass` material instance at the specified USD path
glass = OmniGlassMaterial("/World/visual_cube_material_2")
# - Set the index of refraction (IOR) to 1.25 (typical for glass)
glass.set_input_values("glass_ior", 1.25)
# - Set the glass depth/thickness to 0.001 units (controls light absorption through the material).
glass.set_input_values("depth", 0.001)
# - Disable thin-walled mode (use full volumetric glass simulation).
glass.set_input_values("thin_walled", False)
# - Set the glass tint color to the randomly generated `glass_color`
glass.set_input_values("glass_color", glass_color)

# Create `cube_1` with `textured_material`
cube_1 = Cube(
    paths="/World/new_cube_1",
    positions=[0, 0, 0.5],
    sizes=1.0,
)
cube_1.apply_visual_materials(textured_material)

# Create `cube_2` with `glass` material
cube_2 = Cube(
    paths="/World/new_cube_2",
    positions=[2, 0.39, 0.5],
    sizes=1.0,
)
cube_2.apply_visual_materials(glass)

# Get applied visual material from `cube_2` and change its color
visual_materials = cube_2.get_applied_visual_materials()
if visual_materials[0] is not None:
    visual_materials[0].set_input_values("glass_color", [1.0, 0.5, 0.0])

# 3. --------------------------------------------------------------------

# Start timeline for simulation.
app_utils.play()
simulation_app.update()

# Run simulation
for i in range(10000):
    simulation_app.update()
    if args.test is True:
        break

print("Finished simulating for 10000 steps")

# Stop timeline before closing to avoid shutdown crashes.
app_utils.stop()

# Close the `SimulationApp`.
simulation_app.close()
