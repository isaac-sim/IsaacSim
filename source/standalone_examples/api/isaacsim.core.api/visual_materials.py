# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import random
import sys

import carb
from isaacsim.core.api import World
from isaacsim.core.api.materials.omni_glass import OmniGlass
from isaacsim.core.api.materials.omni_pbr import OmniPBR
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Materials/Textures/Synthetic/bubbles_2.png"

my_world = World(stage_units_in_meters=1.0)

textured_material = OmniPBR(
    prim_path="/World/visual_cube_material",
    name="omni_pbr",
    color=np.array([1, 0, 0]),
    texture_path=asset_path,
    texture_scale=[1.0, 1.0],
    texture_translate=[0.5, 0],
)

glass = OmniGlass(
    prim_path=f"/World/visual_cube_material_2",
    ior=1.25,
    depth=0.001,
    thin_walled=False,
    color=np.array([random.random(), random.random(), random.random()]),
)

cube_1 = my_world.scene.add(
    VisualCuboid(
        prim_path="/new_cube_1",
        name="visual_cube",
        position=np.array([0, 0, 0.5]),
        size=1.0,
        color=np.array([255, 255, 255]),
        visual_material=textured_material,
    )
)

cube_2 = my_world.scene.add(
    VisualCuboid(
        prim_path="/new_cube_2",
        name="visual_cube_2",
        position=np.array([2, 0.39, 0.5]),
        size=1.0,
        color=np.array([255, 255, 255]),
        visual_material=glass,
    )
)

visual_material = cube_2.get_applied_visual_material()
visual_material.set_color(np.array([1.0, 0.5, 0.0]))

my_world.scene.add_default_ground_plane()

my_world.reset()
for i in range(10000):
    my_world.step(render=True)
    if args.test is True:
        break

print("Finished simulating for 10000 steps")

simulation_app.close()
