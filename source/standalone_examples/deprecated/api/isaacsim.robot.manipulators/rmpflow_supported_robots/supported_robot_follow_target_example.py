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

"""Demonstrate RMPflow-based follow-target for supported robots."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
from pprint import pprint

import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import cuboid
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)
from isaacsim.robot_motion.motion_generation.lula import RmpFlow
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Usd, UsdGeom

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "-v",
    "--verbose",
    action="store_true",
    default=True,
    help="Print useful runtime information such as the list of supported robots",
)
parser.add_argument(
    "--robot-name",
    type=str,
    default="Cobotta_Pro_900",
    help="Key to use to access RMPflow config files for a specific robot.",
)
parser.add_argument(
    "--usd-path",
    type=str,
    default="/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd",
    help="Path to supported robot on Nucleus Server",
)
parser.add_argument("--add-orientation-target", action="store_true", default=False, help="Add orientation target")
args, unknown = parser.parse_known_args()

robot_name = args.robot_name
usd_path = get_assets_root_path() + args.usd_path
prim_path = "/my_robot"

add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

# Use default grid environment like other manipulator examples

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

robot = my_world.scene.add(Robot(prim_path=prim_path, name=robot_name))

if args.verbose:
    print("Names of supported robots with provided RMPflow config")
    print("\t", list(get_supported_robot_policy_pairs().keys()))
    print()

# The load_supported_motion_policy_config() function is currently the simplest way to load supported robots.
# In the future, Isaac Sim will provide a centralized registry of robots with Lula robot description files
# and RMP configuration files stored alongside the robot USD.
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

if args.verbose:
    print(
        f"Successfully referenced RMPflow config for {robot_name}.  Using the following parameters to initialize RmpFlow class:"
    )
    pprint(rmp_config)
    print()

# Initialize an RmpFlow object
rmpflow = RmpFlow(**rmp_config)

physics_dt = 1 / 60.0
articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, physics_dt)

articulation_controller = robot.get_articulation_controller()

# Make a target to follow
target_cube = cuboid.VisualCuboid(
    "/World/target", position=np.array([0.5, 0, 0.5]), color=np.array([1.0, 0, 0]), size=0.1
)

# Make an obstacle to avoid
obstacle = cuboid.VisualCuboid(
    "/World/obstacle", position=np.array([0.8, 0, 0.5]), color=np.array([0, 1.0, 0]), size=0.1
)
rmpflow.add_obstacle(obstacle)

# Frame the existing viewport camera on the robot (like pressing F)
stage = omni.usd.get_context().get_stage()
active_viewport = get_active_viewport()
if active_viewport and active_viewport.camera_path:
    usd_camera = UsdGeom.Camera.Get(stage, active_viewport.camera_path)
    if usd_camera:
        usd_camera.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
    resolution = active_viewport.resolution
    aspect_ratio = resolution[0] / resolution[1]
    omni.kit.commands.execute(
        "FramePrimsCommand",
        prim_to_move=active_viewport.camera_path,
        prims_to_frame=[prim_path],
        time_code=Usd.TimeCode.Default(),
        aspect_ratio=aspect_ratio,
        zoom=0.6,
    )


my_world.reset()
reset_needed = False
frame_count = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        # Set rmpflow target to be the current position of the target cube.
        if args.add_orientation_target:
            target_orientation = target_cube.get_world_pose()[1]
        else:
            target_orientation = None

        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0], target_orientation=target_orientation
        )

        # Query the current obstacle position
        rmpflow.update_world()

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
    frame_count += 1
    if args.test and frame_count >= 10:
        break

simulation_app.close()
