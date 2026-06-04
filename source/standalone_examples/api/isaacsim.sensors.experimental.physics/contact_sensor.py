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

"""Demonstrate experimental contact sensor usage."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import Contact, ContactSensor
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane")
DistantLight("/World/DistantLight").set_intensities(1000)
asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/Ant/ant.usd"
stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Ant")

ant = Articulation("/World/Ant/torso", translations=np.array([[0, 0, 1.5]]), reset_xform_op_properties=True)

ant_foot_prim_names = ["right_back_foot", "left_back_foot", "front_right_foot", "front_left_foot"]

translations = np.array(
    [[0.38202, -0.40354, -0.0887], [-0.4, -0.40354, -0.0887], [-0.4, 0.4, -0.0887], [0.4, 0.4, -0.0887]]
)

ant_sensors = []
for i in range(4):
    sensor = ContactSensor(
        Contact.create(
            "/World/Ant/" + ant_foot_prim_names[i] + "/contact_sensor",
            min_threshold=0,
            max_threshold=10000000,
            radius=0.1,
            translations=translations[i : i + 1],
        )
    )
    ant_sensors.append(sensor)

ant_sensors[0].add_raw_contact_data_to_frame()

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()

reset_needed = False
frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        print(ant_sensors[0].get_data())
        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False
        frame_count += 1
        if args.test and frame_count >= 20:
            break

simulation_app.close()
