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

"""Demonstrate creating cubes with different physics properties using the experimental API.

This example demonstrates how to create cubes with different physics properties using the Isaac Sim core
(experimental) API.

The example serves to illustrate the following concepts:
- How to create visual-only primitives (no physics).
- How to create dynamic rigid bodies with physics simulation.
- How to apply visual materials to primitives.
- How to set initial velocities and reset primitives to their initial states.

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Stage creation and population.
3. Example logic.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

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
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

# 2. --------------------------------------------------------------------

# Setup stage programatically:
# - Create a new stage
stage_utils.create_new_stage()
# - Add ground plane
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
# - Add distant light
distant_light = DistantLight("/World/DistantLight")
distant_light.set_intensities(300)

# Create materials
white_material = PreviewSurfaceMaterial("/Visual_materials/white")
white_material.set_input_values("diffuseColor", [1.0, 1.0, 1.0])

red_material = PreviewSurfaceMaterial("/Visual_materials/red")
red_material.set_input_values("diffuseColor", [1.0, 0.0, 0.0])

blue_material = PreviewSurfaceMaterial("/Visual_materials/blue")
blue_material.set_input_values("diffuseColor", [0.0, 0.0, 1.0])

# Cube 1: Visual only (no physics) - white cube.
cube_1_shape = Cube(
    paths="/World/new_cube_1",
    positions=[0, 0, 0.5],
    sizes=1.0,
    scales=[0.3, 0.3, 0.3],
)
cube_1_shape.apply_visual_materials(white_material)

# Cube 2: Dynamic (with physics) - red cube.
cube_2_shape = Cube(
    paths="/World/new_cube_2",
    positions=[0, 0, 1.0],
    sizes=1.0,
    scales=[0.6, 0.5, 0.2],
)
cube_2_shape.apply_visual_materials(red_material)
# - Apply physics (RigidPrim) and collision (GeomPrim)
cube_2 = RigidPrim(paths=cube_2_shape.paths)
cube_2_geometry = GeomPrim(paths=cube_2_shape.paths, apply_collision_apis=True)

# Cube 3: Dynamic (with physics and initial velocity) - blue cube.
cube_3_shape = Cube(
    paths="/World/new_cube_3",
    positions=[0, 0, 3.0],
    sizes=1.0,
    scales=[0.1, 0.1, 0.1],
)
cube_3_shape.apply_visual_materials(blue_material)
# - Apply physics (RigidPrim) and collision (GeomPrim)
cube_3 = RigidPrim(paths=cube_3_shape.paths)
cube_3_geometry = GeomPrim(paths=cube_3_shape.paths, apply_collision_apis=True)

# 3. --------------------------------------------------------------------

# Start timeline for physics simulation.
app_utils.play()
simulation_app.update()  # - Allow physics to initialize

# Set initial velocity for `cube_3` after timeline starts (needs physics to be initialized).
cube_3.set_velocities(linear_velocities=[0, 0, 0.4])

# Store initial states for reset.
cube_2_initial_position = [0, 0, 1.0]
cube_2_initial_orientation = [1.0, 0.0, 0.0, 0.0]  # - Identity quaternion (w, x, y, z)
cube_3_initial_position = [0, 0, 3.0]
cube_3_initial_orientation = [1.0, 0.0, 0.0, 0.0]
cube_3_initial_linear_velocity = [0, 0, 0.4]

# Run simulation loop with periodic resets.
for i in range(5):
    # - Reset cubes to initial positions and velocities
    cube_2.set_world_poses(
        positions=[cube_2_initial_position],
        orientations=[cube_2_initial_orientation],
    )
    cube_2.set_velocities(linear_velocities=[0, 0, 0], angular_velocities=[0, 0, 0])

    cube_3.set_world_poses(
        positions=[cube_3_initial_position],
        orientations=[cube_3_initial_orientation],
    )
    cube_3.set_velocities(
        linear_velocities=[cube_3_initial_linear_velocity],
        angular_velocities=[0, 0, 0],
    )

    # Step simulation and monitor `cube_2` state.
    for j in range(500):
        simulation_app.update()
        # - Get velocities and pose from `cube_2`
        linear_velocity, angular_velocity = cube_2.get_velocities()
        positions, orientations = cube_2.get_world_poses()

        print(f"Angular velocity: {angular_velocity}")
        print(f"World pose - Position: {positions}, Orientation: {orientations}")
        if args.test is True:
            break
    if args.test is True:
        break

# Close the `SimulationApp`.
simulation_app.close()
