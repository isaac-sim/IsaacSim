# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Benchmark UR10 robot articulation simulation performance."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--device", type=str, default="cpu", help="simulation device, cpu or cuda")
parser.add_argument("--visual", type=bool, default=False, help="Render for debugging purposes")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
n_gpu = args.num_gpus
n_frames = args.num_frames
device = args.device
visual = args.visual

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not visual, "max_gpu_count": n_gpu})

from functools import partial

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.physics.core
import omni.timeline
from isaacsim.core.api import PhysicsContext, World
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

torch = import_module("torch")

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robots_ur10",
    workflow_metadata={
        "metadata": [
            {"name": "num_robots", "data": n_robot},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
            {"name": "device", "data": device},
        ]
    },
    backend_type=args.backend_type,
)

# Something about this being in an array makes it work as a global variable inside the physics sub
timestep = [0]

observed_positions, observed_velocities = [], []
commanded_positions, commanded_velocities = [], []

# v_max is the maximum velocity that each joint will hit in its range of motion
v_max = torch.tensor([2.09, 2.09, 3.14, 3.14, 3.14, 3.14])

# T is the period of each sinusoid
T = torch.tensor([9.43, 9.43, 6.28, 6.28, 6.28, 6.28])

joint_indices = torch.arange(6)

robot_path = "/ur10"


def get_clipped_joint_ranges(articulation_view):
    """Compute joint ranges clipped to a 2-pi window for the given articulation."""
    lower_limit, upper_limit = articulation_view.get_dof_limits()
    # convert to numpy
    lower_limit = torch.from_numpy(lower_limit.numpy())
    upper_limit = torch.from_numpy(upper_limit.numpy())

    l = lower_limit.clone()
    u = upper_limit.clone()
    d = upper_limit - lower_limit
    mask = d > 2 * torch.pi

    if torch.any(mask):
        l[mask] = (upper_limit[mask] - lower_limit[mask]) / 2 + lower_limit[mask] - torch.pi
        u[mask] = (upper_limit[mask] - lower_limit[mask]) / 2 + lower_limit[mask] + torch.pi

    return l, u


def get_joint_commands(articulation_view, v_max, T, joint_indices):
    """Generate sinusoidal position and velocity command functions for the joints."""
    lower_joint_limits, upper_joint_limits = get_clipped_joint_ranges(articulation_view)

    # Convert joint_indices to numpy if it's a torch tensor
    joint_indices_np = joint_indices.cpu().numpy() if torch.is_tensor(joint_indices) else joint_indices

    lower_joint_limits = lower_joint_limits[:, joint_indices_np]
    upper_joint_limits = upper_joint_limits[:, joint_indices_np]

    p_0 = lower_joint_limits + (upper_joint_limits - lower_joint_limits) / 2

    position = lambda t: p_0 - v_max * T / torch.pi * torch.cos(torch.pi * t / T)
    velocity = lambda t: v_max * torch.sin(torch.pi * t / T)

    return position, velocity


def on_physics_step(articulation_view, position_commands, velocity_commands, step, context):
    """Apply joint position and velocity commands on each physics step."""
    if position_commands is None:
        return
    timestep[0] += step
    if timestep[0] > 5:
        return

    dof_indices_np = joint_indices.cpu().numpy() if torch.is_tensor(joint_indices) else joint_indices

    # convert to numpy
    dof_positions = articulation_view.get_dof_positions(dof_indices=dof_indices_np).numpy()
    dof_velocities = articulation_view.get_dof_velocities(dof_indices=dof_indices_np).numpy()

    observed_positions.append(dof_positions)
    observed_velocities.append(dof_velocities)

    position_command = position_commands(timestep[0])
    velocity_command = velocity_commands(timestep[0])

    position_command_np = position_command.cpu().numpy() if torch.is_tensor(position_command) else position_command
    velocity_command_np = velocity_command.cpu().numpy() if torch.is_tensor(velocity_command) else velocity_command

    commanded_positions.append(position_command_np)
    commanded_velocities.append(velocity_command_np)

    articulation_view.set_dof_position_targets(positions=position_command_np, dof_indices=dof_indices_np)
    articulation_view.set_dof_velocity_targets(velocities=velocity_command_np, dof_indices=dof_indices_np)


benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

get_active_viewport().updates_enabled = visual

robot_usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"

my_world = World(backend="torch", device=device)
PhysicsContext(physics_dt=1.0 / 60.0)
MAX_IN_LINE = 10
positions = np.zeros((n_robot, 3))
robot_prim_paths = []
for i in range(n_robot):
    robot_prim_path = "/Robots/Robot_" + str(i)
    robot_prim_paths.append(robot_prim_path)
    # position the robot
    robot_position = np.array([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
    positions[i, :] = robot_position
    stage_utils.add_reference_to_stage(robot_usd_path, path=robot_prim_path)


omni.kit.app.get_app().update()
my_world.scene.add_default_ground_plane(z_position=-1)

robot_view = Articulation(robot_prim_paths, positions=positions, reset_xform_op_properties=True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

position_commands, velocity_commands = get_joint_commands(robot_view, v_max, T, joint_indices)
physics_subscription = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
    pre_step=False, order=0, on_update=partial(on_physics_step, robot_view, position_commands, velocity_commands)
)

position_command = position_commands(0)
velocity_command = velocity_commands(0)

position_command_np = position_command.cpu().numpy() if torch.is_tensor(position_command) else position_command
velocity_command_np = velocity_command.cpu().numpy() if torch.is_tensor(velocity_command) else velocity_command
dof_indices_np = joint_indices.cpu().numpy() if torch.is_tensor(joint_indices) else joint_indices

robot_view.set_dof_positions(positions=position_command_np, dof_indices=dof_indices_np)

commanded_positions.append(position_command_np)
commanded_velocities.append(velocity_command_np)

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark", warmup_frames=15)

for _ in range(0, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

physics_subscription = None

timeline.stop()
simulation_app.close()
