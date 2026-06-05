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

"""Benchmark core world operations including cloning, views, and simulation steps."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to clone.")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_envs = args.num_envs

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.benchmark.services")

import sys

import carb
import numpy as np
from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.cloner import GridCloner
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, define_prim
from isaacsim.storage.native import get_assets_root_path


def define_environment():
    """Define the base environment with cubes and robot assets."""
    define_prim(path="/World/env_0", type_name="Xform")
    XformPrim("/World/env_0", positions=np.array([[0.0, 0.0, 0.0]]), reset_xform_op_properties=True)
    cube_1 = VisualCuboid(
        prim_path="/World/env_0/new_cube_1",
        name="visual_cube",
        position=np.array([0, 0, 0.5]),
        size=0.3,
        color=np.array([255, 255, 255]),
    )

    cube_2 = DynamicCuboid(
        prim_path="/World/env_0/new_cube_2",
        name="cube_1",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )

    cube_3 = DynamicCuboid(
        prim_path="/World/env_0/new_cube_3",
        name="cube_2",
        position=np.array([0, 0, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()

    asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    robot1 = add_reference_to_stage(usd_path=asset_path, path="/World/env_0/Franka_1")
    robot1.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot1.GetVariantSet("Mesh").SetVariantSelection("Quality")
    robot2 = add_reference_to_stage(usd_path=asset_path, path="/World/env_0/Franka_2")
    robot2.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot2.GetVariantSet("Mesh").SetVariantSelection("Quality")
    XformPrim("/World/env_0/Franka_1", positions=np.array([[0.0, 2.0, 0.0]]), reset_xform_op_properties=True)
    XformPrim("/World/env_0/Franka_2", positions=np.array([[0.0, -2.0, 0.0]]), reset_xform_op_properties=True)


def clone_environments():
    """Clone the base environment using GridCloner."""
    cloner = GridCloner(spacing=1)
    cloner.define_base_env("/World")
    prim_paths = cloner.generate_paths("/World/env", n_envs)
    cloner.clone(source_prim_path="/World/env_0", prim_paths=prim_paths, replicate_physics=True, copy_from_source=False)


def create_cube_views():
    """Create geometry and rigid body views for the cloned cubes."""
    # Experimental prims don't need to be added to the scene registry
    visual_cube_view = GeomPrim(paths="/World/env_.*/new_cube_1")
    rigid_cube_view_1 = RigidPrim(paths="/World/env_.*/new_cube_2")
    rigid_cube_view_2 = RigidPrim(paths="/World/env_.*/new_cube_3")


def create_articulation_views():
    """Create articulation views for the cloned Franka robots."""
    global articulation_view_1
    # Experimental prims don't need to be added to the scene registry
    articulation_view_1 = Articulation(paths="/World/env_.*/Franka_1")
    articulation_view_2 = Articulation(paths="/World/env_.*/Franka_2")


# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_world",
    workflow_metadata={"metadata": []},
    backend_type=args.backend_type,
)

benchmark.set_phase("world_creation", start_recording_frametime=False, start_recording_runtime=True)
my_world = World(stage_units_in_meters=1.0)
benchmark.store_measurements()

my_world.scene.add_default_ground_plane()


benchmark.set_phase("env_creation", start_recording_frametime=False, start_recording_runtime=True)
define_environment()
benchmark.store_measurements()

benchmark.set_phase("env_cloning", start_recording_frametime=False, start_recording_runtime=True)
clone_environments()
benchmark.store_measurements()

benchmark.set_phase("cube_views_creation", start_recording_frametime=False, start_recording_runtime=True)
create_cube_views()
benchmark.store_measurements()

benchmark.set_phase("articulation_views_creation", start_recording_frametime=False, start_recording_runtime=True)
create_articulation_views()
benchmark.store_measurements()

benchmark.set_phase("get_world_pose_articulation_no_sim", start_recording_frametime=False, start_recording_runtime=True)
articulation_view_1.get_world_poses()
benchmark.store_measurements()

benchmark.set_phase("world_resetting", start_recording_frametime=False, start_recording_runtime=True)
my_world.reset()
benchmark.store_measurements()

benchmark.set_phase("world_step_w_render", start_recording_frametime=True, start_recording_runtime=False)
for i in range(100):
    articulation_view_1.set_dof_position_targets(positions=np.random.randn(n_envs, 9))
    my_world.step(render=True)
benchmark.store_measurements()

benchmark.set_phase("world_step_no_render", start_recording_frametime=True, start_recording_runtime=False)
for i in range(100):
    articulation_view_1.set_dof_position_targets(positions=np.random.randn(n_envs, 9))
    my_world.step(render=False)
benchmark.store_measurements()

benchmark.set_phase("get_world_pose_articulation_w_sim", start_recording_frametime=False, start_recording_runtime=True)
articulation_view_1.get_world_poses()
benchmark.store_measurements()


benchmark.stop()
simulation_app.close()
