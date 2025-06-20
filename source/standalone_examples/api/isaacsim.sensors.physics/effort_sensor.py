# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# In this example, please drag the cube along the arm and see how the effort measurement from the effort sensor changes

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.physics import EffortSensor
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60, rendering_dt=1.0 / 60)
my_world.scene.add_default_ground_plane(z_position=-1)

asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/Articulation")
arm_joint = "/Articulation/Arm/RevoluteJoint"
prim = get_prim_at_path(arm_joint)
joint = UsdPhysics.RevoluteJoint(prim)
joint.CreateAxisAttr("Y")

DynamicCuboid(
    prim_path="/World/Cube",
    name="cube_1",
    position=np.array([1.5, 0, 0.2]),
    color=np.array([255, 0, 0]),
    size=0.1,
    mass=1,
)

my_world.reset()
effort_sensor = EffortSensor(prim_path=arm_joint)
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        reading = effort_sensor.get_sensor_reading()
        print(f"Sensor Time: {reading.time}   Value: {reading.value}   Validity: {reading.is_valid}")

        if reset_needed:
            my_world.reset()
            reset_needed = False

simulation_app.close()
