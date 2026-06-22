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

"""Demonstrate contact sensor usage."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": ["--enable", "isaacsim.sensors.physics"],
    }
)

import argparse
import sys

import carb
import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.physics import ContactSensor
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/Ant/ant.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Ant")

ant = my_world.scene.add(Articulation("/World/Ant/torso", name="ant", translations=np.array([[0, 0, 1.5]])))

# Tick the stage so the USD reference is fully loaded
simulation_app.update()

ant_foot_prim_names = ["right_back_foot", "left_back_foot", "front_right_foot", "front_left_foot"]

# The Ant USD only has CollisionAPI on child collision meshes, not on the
# foot prims themselves.  The deprecated ContactSensor requires CollisionAPI
# on the *parent* of the sensor prim, so apply it to each foot link.
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
for name in ant_foot_prim_names:
    foot_prim = stage.GetPrimAtPath(f"/World/Ant/{name}")
    if foot_prim.IsValid() and not foot_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(foot_prim)

translations = np.array(
    [[0.38202, -0.40354, -0.0887], [-0.4, -0.40354, -0.0887], [-0.4, 0.4, -0.0887], [0.4, 0.4, -0.0887]]
)

ant_sensors = []
for i in range(4):
    ant_sensors.append(
        my_world.scene.add(
            ContactSensor(
                prim_path="/World/Ant/" + ant_foot_prim_names[i] + "/contact_sensor",
                name=f"ant_contact_sensor_{i}",
                min_threshold=0,
                max_threshold=10000000,
                radius=0.1,
                translation=translations[i],
            )
        )
    )

ant_sensors[0].add_raw_contact_data_to_frame()
my_world.reset()
reset_needed = False
frame_count = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        print(ant_sensors[0].get_current_frame())
        if reset_needed:
            my_world.reset()
            reset_needed = False
        frame_count += 1
        if args.test and frame_count >= 10:
            break

simulation_app.close()
