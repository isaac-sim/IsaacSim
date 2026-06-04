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

"""Test the physics contact sensor on a dynamic cube."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": True,
        "extra_args": ["--enable", "isaacsim.sensors.physics"],
    }
)

import sys

import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.physics import _sensor
from pxr import Gf

timeline = omni.timeline.get_timeline_interface()
cs = _sensor.acquire_contact_sensor_interface()

world = World(stage_units_in_meters=1.0)

# add a cube in the world
cube_path = "/World/cube"
cube_1 = world.scene.add(DynamicCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 1.5]), size=1.0))
# Add a plane for cube to collide with
world.scene.add_default_ground_plane()

# Setup contact sensor on cube
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateContactSensor",
    path="/Contact_Sensor",
    parent=cube_path,
    min_threshold=0,
    max_threshold=100000000,
    color=Gf.Vec4f(1, 1, 1, 1),
    radius=-1,
    sensor_period=1.0 / 60.0,
    translation=Gf.Vec3d(0, 0, 0),
)

# start simulation and initialize physics
world.initialize_physics()
timeline.play()

for frame in range(100):
    world.step(render=False)

print("cube pose", cube_1.get_world_pose())

# Get processed contact data
reading = cs.get_sensor_reading(cube_path + "/Contact_Sensor")

if not reading.is_valid:
    print("[fatal] No contact sensor readings")
    sys.exit(1)

if reading.is_valid:
    print(str(reading))


# Cleanup
timeline.stop()
simulation_app.close()
