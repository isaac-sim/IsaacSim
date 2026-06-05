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

"""Demonstrate basic scene setup with visual and physics cubes."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager

# Create a new stage
stage_utils.create_new_stage()

# Add Ground Plane
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])

# Add Light Source
distant_light = DistantLight("/DistantLight")
distant_light.set_intensities(300)

# Create materials for cubes
yellow_material = PreviewSurfaceMaterial("/Materials/yellow")
yellow_material.set_input_values("diffuseColor", [1.0, 1.0, 0.0])  # - Yellow (RGB 255,255,0 -> normalized)

green_material = PreviewSurfaceMaterial("/Materials/green")
green_material.set_input_values("diffuseColor", [0.0, 1.0, 0.0])  # - Green (RGB 0,255,0 -> normalized)

cyan_material = PreviewSurfaceMaterial("/Materials/cyan")
cyan_material.set_input_values("diffuseColor", [0.0, 1.0, 1.0])  # - Cyan (RGB 0,255,255 -> normalized)

# Add Visual Cubes
visual_cube = Cube(
    paths="/visual_cube",
    positions=[0, 0.5, 1.0],
    sizes=0.3,
)
visual_cube.apply_visual_materials(yellow_material)

visual_cube_static = Cube(
    paths="/visual_cube_static",
    positions=[0.5, 0, 0.5],
    sizes=0.3,
)
visual_cube_static.apply_visual_materials(green_material)

# Add Physics Cubes
dynamic_cube = Cube(
    paths="/dynamic_cube",
    positions=[0, -0.5, 1.5],
    sizes=0.3,
)
dynamic_cube.apply_visual_materials(cyan_material)

# - Apply physics to `dynamic_cube`
dynamic_rigid = RigidPrim(paths="/dynamic_cube")
dynamic_geom = GeomPrim(paths="/dynamic_cube", apply_collision_apis=True)

SimulationManager.set_physics_dt(1.0 / 60.0)

# Start timeline
app_utils.play()
simulation_app.update()

# Start the simulator.
for i in range(3):
    # - Reset simulation
    app_utils.stop()
    app_utils.play()
    simulation_app.update()

    print("Simulator running", i)
    if i == 1:
        print("Adding Physics Properties to the Visual Cube")
        visual_rigid = RigidPrim("/visual_cube")  # - Apply physics to `visual_cube`

    if i == 2:
        print("Adding Collision Properties to the Visual Cube")
        visual_geometry = GeomPrim("/visual_cube", apply_collision_apis=True)  # - Apply collision to `visual_cube`

    for j in range(100):
        SimulationManager.step()
        RenderingManager.render()
        simulation_app.update()  # - Stepping through the simulation
