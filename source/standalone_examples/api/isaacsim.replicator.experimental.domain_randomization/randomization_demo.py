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

"""Domain randomization demo for reinforcement learning.

This demo showcases domain randomization with multiple cloned environments:
- Gravity randomization
- Rigid body force randomization
- Articulation joint velocity randomization
- Environment reset with position/joint position randomization
"""

import argparse

parser = argparse.ArgumentParser(description="Domain randomization demo")
parser.add_argument(
    "--env-url",
    type=str,
    default="/Isaac/Environments/Grid/default_environment.usd",
    help="Environment USD path relative to assets root ('none' for procedural ground plane + dome light)",
)
parser.add_argument("--num-envs", type=int, default=4, help="Number of cloned environments")
parser.add_argument("--reset-interval", type=int, default=50, help="Frames between environment resets")
parser.add_argument("--max-frames", type=int, default=200, help="Maximum simulation frames (0 for infinite)")
args, unknown = parser.parse_known_args()

# Store configuration (treat "none" or empty as procedural environment)
env_url = args.env_url if args.env_url.lower() not in ("", "none") else None
num_envs = args.num_envs
reset_interval = args.reset_interval
max_frames = args.max_frames

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import isaacsim.replicator.experimental.domain_randomization as dr
import omni.replicator.core as rep
from isaacsim.core.cloner import GridCloner
from isaacsim.core.experimental.objects import DomeLight, GroundPlane, Sphere
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, define_prim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

# Print configuration
print("=" * 60)
print("Domain Randomization Demo Configuration")
print("=" * 60)
print(f"  Environment URL: {env_url if env_url else '(procedural ground plane + dome light)'}")
print(f"  Number of cloned environments: {num_envs}")
print(f"  Reset interval: {reset_interval} frames")
print(f"  Max frames: {max_frames if max_frames > 0 else 'infinite'}")
print("=" * 60)

# Get assets root path (needed for robot reference)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Set up environment
if env_url:
    add_reference_to_stage(usd_path=assets_root_path + env_url, path="/World/defaultGroundPlane")
else:
    dome_light = DomeLight("/World/DomeLight")
    dome_light.set_intensities(1000)
    GroundPlane("/World/defaultGroundPlane", sizes=100.0)

# Set up grid cloner for multiple environments
cloner = GridCloner(spacing=1.5)
cloner.define_base_env("/World/envs")
define_prim("/World/envs/env_0")

# Set up the first environment with sphere and robot
Sphere("/World/envs/env_0/object", radii=0.1, positions=[0.75, 0.0, 0.2])
GeomPrim("/World/envs/env_0/object", apply_collision_apis=True)
RigidPrim("/World/envs/env_0/object", masses=0.02)
add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    path="/World/envs/env_0/franka",
)

# Clone environments
prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

# Create views before initializing physics
object_view = RigidPrim("/World/envs/.*/object")
franka_view = Articulation("/World/envs/.*/franka")

# Initialize physics
SimulationManager.initialize_physics()

num_dof = franka_view.num_dofs

# Register physics views for domain randomization
dr.physics_view.register_simulation_context()
dr.physics_view.register_rigid_prim_view(object_view, name="object_view")
dr.physics_view.register_articulation_view(franka_view, name="franka_view")

# Configure randomization triggers and gates
with dr.trigger.on_rl_frame(num_envs=num_envs):
    with dr.gate.on_interval(interval=20):
        dr.physics_view.randomize_simulation_context(
            operation="scaling", gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0))
        )
    with dr.gate.on_interval(interval=25):
        dr.physics_view.randomize_rigid_prim_view(
            view_name="object_view", operation="direct", force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0))
        )
    with dr.gate.on_interval(interval=10):
        dr.physics_view.randomize_articulation_view(
            view_name="franka_view",
            operation="direct",
            joint_velocities=rep.distribution.uniform(tuple([-2] * num_dof), tuple([2] * num_dof)),
        )
    with dr.gate.on_env_reset():
        dr.physics_view.randomize_rigid_prim_view(
            view_name="object_view",
            operation="additive",
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
            velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        dr.physics_view.randomize_articulation_view(
            view_name="franka_view",
            operation="additive",
            joint_positions=rep.distribution.uniform(tuple([-0.5] * num_dof), tuple([0.5] * num_dof)),
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
        )

# Run simulation loop
frame_idx = 0
run_indefinitely = max_frames == 0
while simulation_app.is_running() and (run_indefinitely or frame_idx < max_frames):
    reset_inds = list()
    if frame_idx > 0 and frame_idx % reset_interval == 0:
        reset_inds = list(range(num_envs))
        print(f"Reset #{frame_idx // reset_interval} at frame {frame_idx}")
    dr.physics_view.step_randomization(reset_inds)
    SimulationManager.step()
    simulation_app.update()
    frame_idx += 1

print(f"Simulation complete after {frame_idx} frames")

simulation_app.close()
