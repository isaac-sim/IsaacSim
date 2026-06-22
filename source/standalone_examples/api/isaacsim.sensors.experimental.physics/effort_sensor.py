# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate experimental effort sensor measurement."""

# In this example, please drag the cube along the arm and see how the effort measurement from the effort sensor changes

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import EffortSensor
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane", positions=[0, 0, -1])
DistantLight("/World/DistantLight").set_intensities(1000)

asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
stage_utils.add_reference_to_stage(usd_path=asset_path, path="/Articulation")
arm_joint = "/Articulation/Arm/RevoluteJoint"
prim = get_prim_at_path(arm_joint)
joint = UsdPhysics.RevoluteJoint(prim)
joint.CreateAxisAttr("Y")

cube_shape = Cube(
    paths="/World/Cube",
    positions=[1.5, 0, 0.2],
    sizes=0.1,
)
RigidPrim(paths=cube_shape.paths)
GeomPrim(paths=cube_shape.paths, apply_collision_apis=True)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
app_utils.update_app(steps=2)

effort_sensor = EffortSensor(path=arm_joint)
reset_needed = False
frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        reading = effort_sensor.get_sensor_reading()
        print(f"Sensor Time: {reading.time}   Value: {reading.value}   Validity: {reading.is_valid}")

        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False
    frame_count += 1
    if args.test and frame_count >= 10:
        break

simulation_app.close()
