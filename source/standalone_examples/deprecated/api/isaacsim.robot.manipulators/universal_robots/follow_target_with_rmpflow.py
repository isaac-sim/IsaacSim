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

"""Demonstrate UR10 follow-target using RMPflow motion policy."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)


from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.universal_robots.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.examples.universal_robots.tasks import FollowTarget

my_world = World(stage_units_in_meters=1.0)
my_task = FollowTarget(name="follow_target_task", attach_gripper=True)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("follow_target_task").get_params()
ur10_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_ur10 = my_world.scene.get_object(ur10_name)
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_ur10, attach_gripper=True)
articulation_controller = my_ur10.get_articulation_controller()
reset_needed = False
frame_count = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
    frame_count += 1
    if args.test and frame_count >= 10:
        break

simulation_app.close()
