# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate robot articulation control with Franka arm and Nova Carter."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # - Start the simulation app, with GUI open

import sys

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# Preparing the scene.
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Create a new stage
stage_utils.create_new_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)

# Add ground plane
GroundPlane("/World/GroundPlane", positions=[0, 0, 0])

# Add distant light
distant_light = DistantLight("/World/DistantLight")
distant_light.set_intensities(300)

# Set camera view
set_camera_view(eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp")

# Add Franka
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(
    usd_path=asset_path,
    path="/World/Arm",
)  # - Add robot to stage

# Set initial pose before creating articulation.
arm_transform = XformPrim("/World/Arm")
arm_transform.set_world_poses(positions=[0.0, 1.0, 0.0])

arm = Articulation("/World/Arm")  # - Create an articulation object `arm`

# Add Carter
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
stage_utils.add_reference_to_stage(
    usd_path=asset_path,
    path="/World/Car",
)

# Set initial pose before creating articulation.
car_transform = XformPrim("/World/Car")
car_transform.set_world_poses(positions=[0.0, -1.0, 0.0])

car = Articulation("/World/Car")  # - Create an articulation object `car`

# Initialize physics
SimulationManager.set_physics_dt(1.0 / 60.0)

# Start timeline
app_utils.play()
simulation_app.update()

for i in range(4):
    print("Running cycle:", i)
    if i == 1 or i == 3:
        print("Moving")
        # - Move the `arm`
        arm.set_dof_positions([-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04])
        # - Move the `car`
        car.set_dof_velocities([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    if i == 2:
        print("Stopping")
        # - Reset the `arm`
        arm.set_dof_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # - Stop the `car`
        car.set_dof_velocities([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    for j in range(100):
        # - Step the simulation, both rendering and physics.
        SimulationManager.step()
        RenderingManager.render()
        simulation_app.update()
        # - Print the joint positions of `car` at every physics step
        if i == 3:
            car_joint_positions = car.get_dof_positions()
            print("Car joint positions:", car_joint_positions)

# Stop timeline before closing
app_utils.stop()
simulation_app.close()
