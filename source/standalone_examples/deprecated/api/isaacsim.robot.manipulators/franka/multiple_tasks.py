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

"""Demonstrate Franka robot executing multiple pick-and-place tasks."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)

import argparse

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.examples.franka.tasks import PickPlace

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
tasks = []
num_of_tasks = 2
for i in range(num_of_tasks):
    tasks.append(PickPlace(name="task" + str(i), offset=np.array([0, (i * 2) - 3, 0])))
    my_world.add_task(tasks[-1])
for _ in range(10):
    simulation_app.update()
my_world.reset()
frankas = []
cube_names = []
for i in range(num_of_tasks):
    task_params = tasks[i].get_params()
    frankas.append(my_world.scene.get_object(task_params["robot_name"]["value"]))
    cube_names.append(task_params["cube_name"]["value"])

controllers = []
for i in range(num_of_tasks):
    controllers.append(
        PickPlaceController(name="pick_place_controller", gripper=frankas[i].gripper, robot_articulation=frankas[i])
    )
    controllers[-1].reset()

articulation_controllers = []
for i in range(num_of_tasks):
    articulation_controllers.append(frankas[i].get_articulation_controller())

my_world.pause()
reset_needed = False
frame_count = 0
if args.test:
    my_world.play()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            for i in range(num_of_tasks):
                controllers[i].reset()
            reset_needed = False
        if args.test and frame_count >= 10:
            break
        observations = my_world.get_observations()
        for i in range(num_of_tasks):
            articulation_controllers.append(frankas[i].get_articulation_controller())
            actions = controllers[i].forward(
                picking_position=observations[cube_names[i]]["position"],
                placing_position=observations[cube_names[i]]["target_position"],
                current_joint_positions=observations[frankas[i].name]["joint_positions"],
                end_effector_offset=np.array([0, 0, 0]),
            )
            articulation_controllers[i].apply_action(actions)
        frame_count += 1

simulation_app.close()
