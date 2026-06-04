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

"""Test XFormPrim view with cloned Franka robots and visual materials."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import random
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.materials.omni_glass import OmniGlass
from isaacsim.core.cloner import Cloner
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0, backend="numpy")
my_world.scene.add_default_ground_plane()
num_objects = 3
my_cloner = Cloner()

asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
root_path = "/World/Group"
root_group_path = root_path + "_0"
group_paths = my_cloner.generate_paths(root_path, num_objects)
define_prim(prim_path=root_group_path)
add_reference_to_stage(usd_path=asset_path, prim_path=root_group_path + "/Franka")
define_prim(root_group_path + "/Frame")
define_prim(root_group_path + "/Frame/Target")
my_cloner.clone(root_group_path, group_paths)

frankas_view = XFormPrim(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Franka", name="frankas_view")
targets_view = XFormPrim(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Frame/Target", name="targets_view")
frames_view = XFormPrim(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Frame", name="frames_view")

glass_1 = OmniGlass(
    prim_path=f"/World/franka_glass_material_1",
    ior=1.25,
    depth=0.001,
    thin_walled=False,
    color=np.array([random.random(), random.random(), random.random()]),
)

glass_2 = OmniGlass(
    prim_path=f"/World/franka_glass_material_2",
    ior=1.25,
    depth=0.001,
    thin_walled=False,
    color=np.array([random.random(), random.random(), random.random()]),
)
# new_positions = torch.tensor([[10.0, 10.0, 0], [-40, -40, 0], [40, 40, 0]])
# new_orientations = euler_angles_to_quats(
#     torch.tensor([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0], [0, 0, -np.pi / 2.0]])
# )

new_positions = np.array([[10.0, 10.0, 0], [-40, -40, 0], [40, 40, 0]])
new_orientations = euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0], [0, 0, -np.pi / 2.0]]))

frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations)
frankas_view.apply_visual_materials(visual_materials=glass_1, indices=[1])
frankas_view.apply_visual_materials(visual_materials=[glass_1, glass_2], indices=[2, 0])
print(frankas_view.get_applied_visual_materials(indices=[2, 0]))
print(frankas_view.get_applied_visual_materials())

my_world.reset()

for i in range(10):
    my_world.step(render=True)
simulation_app.close()
