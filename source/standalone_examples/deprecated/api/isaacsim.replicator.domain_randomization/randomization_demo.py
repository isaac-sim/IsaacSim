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

"""Demonstrate domain randomization using replicator physics views."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicSphere
from isaacsim.core.cloner import GridCloner
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

# create the world
world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder, closing app..")
    simulation_app.close()
usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/defaultGroundPlane")

# set up grid cloner
cloner = GridCloner(spacing=1.5)
cloner.define_base_env("/World/envs")
define_prim("/World/envs/env_0")

# set up the first environment
DynamicSphere(prim_path="/World/envs/env_0/object", radius=0.1, position=np.array([0.75, 0.0, 0.2]))
add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    prim_path="/World/envs/env_0/franka",
)

# clone environments
num_envs = 4
prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

# creates the views and set up world
object_view = RigidPrim(prim_paths_expr="/World/envs/*/object", name="object_view")
franka_view = Articulation("/World/envs/*/franka", name="franka_view")
world.scene.add(object_view)
world.scene.add(franka_view)
world.reset()

num_dof = franka_view.num_dof

# set up randomization with isaacsim.replicator, imported as dr
import isaacsim.replicator.domain_randomization as dr
import omni.replicator.core as rep

dr.physics_view.register_simulation_context(world)
dr.physics_view.register_rigid_prim_view(object_view)
dr.physics_view.register_articulation_view(franka_view)

with dr.trigger.on_rl_frame(num_envs=num_envs):
    with dr.gate.on_interval(interval=20):
        dr.physics_view.randomize_simulation_context(
            operation="scaling", gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0))
        )
    with dr.gate.on_interval(interval=50):
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name, operation="direct", force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0))
        )
    with dr.gate.on_interval(interval=10):
        dr.physics_view.randomize_articulation_view(
            view_name=franka_view.name,
            operation="direct",
            joint_velocities=rep.distribution.uniform(tuple([-2] * num_dof), tuple([2] * num_dof)),
        )
    with dr.gate.on_env_reset():
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name,
            operation="additive",
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
            velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        dr.physics_view.randomize_articulation_view(
            view_name=franka_view.name,
            operation="additive",
            joint_positions=rep.distribution.uniform(tuple([-0.5] * num_dof), tuple([0.5] * num_dof)),
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
        )


frame_idx = 0
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

while simulation_app.is_running():
    if world.is_playing():
        reset_inds = list()
        if frame_idx % 200 == 0:
            # triggers reset every 200 steps
            reset_inds = np.arange(num_envs)
        dr.physics_view.step_randomization(reset_inds)
        world.step(render=True)
        frame_idx += 1
        if args.test and frame_idx >= 10:
            break

simulation_app.close()
