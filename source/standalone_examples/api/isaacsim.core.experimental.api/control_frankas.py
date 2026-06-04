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

"""Demonstrate adding and controlling multiple Franka Panda robots using the experimental API.

This example demonstrates how to add multiple Franka Panda robots to a scene and control their joint positions
using the Isaac Sim core (experimental) API.

The example serves to illustrate the following concepts:
- How to add multiple instances of the same robot asset to a stage.
- How to use USD variants to customize robot configurations.
- How to get DOF indices for specific joints by name.
- How to set initial poses and default states for articulated systems.
- How to control robot joints using DOF position targets (both all joints and specific joints).
- How to query joint positions during simulation.
- How to reset robots to their default states.

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Stage creation and population.
3. Example logic.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse
import sys

import carb
import numpy as np

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
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

# 2. --------------------------------------------------------------------

# Verify assets path is available
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Setup stage programatically:
# - Create a new stage
stage_utils.create_new_stage()
# - Add ground plane
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
# - Add distant light
distant_light = DistantLight("/World/DistantLight")
distant_light.set_intensities(300)
# - Add Franka robots with variants
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(
    usd_path=asset_path,
    path="/World/Franka_1",
    variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")],
)
stage_utils.add_reference_to_stage(
    usd_path=asset_path,
    path="/World/Franka_2",
    variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")],
)

# Create articulation wrappers
articulated_system_1 = Articulation("/World/Franka_1")
articulated_system_2 = Articulation("/World/Franka_2")

# Set initial positions BEFORE starting physics (to avoid collisions).
articulated_system_1.set_world_poses(positions=[0.0, 2.0, 0.0])
articulated_system_2.set_world_poses(positions=[0.0, -2.0, 0.0])

# Set default state for reset functionality
articulated_system_1.set_default_state(
    positions=[0.0, 2.0, 0.0],
    dof_positions=[1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5],
)
articulated_system_2.set_default_state(
    positions=[0.0, -2.0, 0.0],
    dof_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
)

# Get DOF index for `panda_joint2` on robot 1 (demonstrates how to get DOF indices by joint name).
dof_indices = articulated_system_1.get_dof_indices("panda_joint2")
print("DOF indices array for panda_joint2:", dof_indices)
# - Convert `wp.array` to Python int (for single DOF)
dof_index = int(dof_indices.numpy()[0]) if hasattr(dof_indices, "numpy") else int(dof_indices[0])
print("DOF index:", dof_index)

# 3. --------------------------------------------------------------------

# Start timeline for physics simulation.
app_utils.play()
simulation_app.update()

# Run simulation loop with periodic resets and joint control.
for i in range(5):
    print("Resetting...")
    # - Reset robots to default state (positions and joint states)
    articulated_system_1.reset_to_default_state()
    articulated_system_2.reset_to_default_state()
    for j in range(500):
        simulation_app.update()
        if j == 100:
            # - Control robot 1: Set target for a specific joint (`panda_joint2`) using `dof_index`.
            articulated_system_1.set_dof_position_targets(-1.5, dof_indices=[dof_index])
            # - Control robot 2: Set target for all joints
            articulated_system_2.set_dof_position_targets([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5])
        if j == 400:
            # - Query and print joint positions
            print("Franka 1's joint positions are:", articulated_system_1.get_dof_positions())
            print("Franka 2's joint positions are:", articulated_system_2.get_dof_positions())
    if args.test is True:
        break

# Print final joint positions for robot 1 (demonstrates querying joint positions).
print(
    "Franka 1 final joint positions: ",
    np.array2string(
        (
            articulated_system_1.get_dof_positions().numpy()
            if hasattr(articulated_system_1.get_dof_positions(), "numpy")
            else articulated_system_1.get_dof_positions()
        ),
        precision=3,
        suppress_small=True,
    ),
)

# Close the `SimulationApp`.
simulation_app.close()
