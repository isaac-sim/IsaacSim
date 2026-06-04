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

"""Demonstrate UR10e follow-target with IK solver."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import numpy as np
from controller.ik_solver import KinematicsSolver
from isaacsim.core.api import World
from tasks.follow_target import FollowTarget

my_world = World(stage_units_in_meters=1.0)
# Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="ur10e_follow_target", target_position=np.array([0.5, 0, 0.5]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("ur10e_follow_target").get_params()
target_name = task_params["target_name"]["value"]
ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)

# initialize the ik solver
ik_solver = KinematicsSolver(my_ur10e)
articulation_controller = my_ur10e.get_articulation_controller()

# run the simulation
frame_count = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()

        observations = my_world.get_observations()
        actions, succ = ik_solver.compute_inverse_kinematics(
            target_position=observations[target_name]["position"],
            target_orientation=observations[target_name]["orientation"],
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            print("IK did not converge to a solution.  No action is being taken.")
    frame_count += 1
    if args.test and frame_count >= 10:
        break
simulation_app.close()
