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

"""Demonstrate physics and rendering callbacks using the experimental API.

This example demonstrates how to register and use physics and rendering callbacks using the Isaac Sim core
(experimental) API.

The example serves to illustrate the following concepts:
- How to register physics callbacks that execute after each physics step.
- How to register rendering callbacks that execute on each new frame.
- How to control a robot's joint positions from within a physics callback.
- How to query simulation state (step count, time, joint positions) from callbacks.
- How to separately step physics and rendering.

The source code is organized into 4 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Helper function definitions (callback functions).
3. Stage creation and population.
4. Example logic.
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

simulation_app = SimulationApp({"headless": True})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.storage.native import get_assets_root_path

# 2. --------------------------------------------------------------------


def step_callback_1(dt, context):
    """Physics callback to set joint position target."""
    robot.set_dof_position_targets(-1.5, dof_indices=[dof_index])


def step_callback_2(dt, context):
    """Physics callback to print joint position and simulation time."""
    step_count = SimulationManager.get_num_physics_steps()
    dof_positions = robot.get_dof_positions(dof_indices=[dof_index])
    # - Convert `wp.array` to Python float for single value
    position_value = (
        float(dof_positions.numpy()[0, 0]) if hasattr(dof_positions, "numpy") else float(dof_positions[0, 0])
    )
    print("Current joint 2 position @ step " + str(step_count) + " : " + str(position_value))
    print("Time:", SimulationManager.get_simulation_time())


def render_callback(event):
    """Render callback to print render frame."""
    print("Render Frame")


# 3. --------------------------------------------------------------------

# Setup stage programatically:
# - Create new stage
stage_utils.create_new_stage()
# - Add Franka robot with variants using experimental API
assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(
    usd_path=asset_path,
    path="/Franka",
    variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")],
)

# Initialize physics before creating articulation.
SimulationManager.set_physics_dt(1.0 / 60.0)

# Create articulation wrapper using experimental API.
robot = Articulation("/Franka")
# - Get DOF index - experimental API returns `wp.array`, get first element from `dof_indices` for single DOF.
dof_indices = robot.get_dof_indices("panda_joint2")
dof_index = int(dof_indices.numpy()[0]) if hasattr(dof_indices, "numpy") else int(dof_indices[0])

# 4. --------------------------------------------------------------------

# Start timeline
app_utils.play()

# Register physics callbacks using `SimulationManager`.
SimulationManager.register_callback(step_callback_1, IsaacEvents.POST_PHYSICS_STEP)
SimulationManager.register_callback(step_callback_2, IsaacEvents.POST_PHYSICS_STEP)

# Register render callback using `RenderingManager`.
RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=render_callback)

# Simulate 60 timesteps (physics only, no rendering).
for i in range(60):
    print("Step", i)
    SimulationManager.step()
    simulation_app.update()
    if args.test is True:
        break

# Render one frame
RenderingManager.render()
simulation_app.update()

# Stop timeline before closing
app_utils.stop()

# Close the `SimulationApp`.
simulation_app.close()
