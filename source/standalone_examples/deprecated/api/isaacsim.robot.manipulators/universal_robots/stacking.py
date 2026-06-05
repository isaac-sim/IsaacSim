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

"""Demonstrate UR10 stacking task."""

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


import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.universal_robots.controllers import StackingController
from isaacsim.robot.manipulators.examples.universal_robots.tasks import Stacking

my_world = World(stage_units_in_meters=1.0)
my_task = Stacking()
my_world.add_task(my_task)
my_world.reset()
robot_name = my_task.get_params()["robot_name"]["value"]
my_ur10 = my_world.scene.get_object(robot_name)
my_controller = StackingController(
    name="stacking_controller",
    gripper=my_ur10.gripper,
    robot_articulation=my_ur10,
    picking_order_cube_names=my_task.get_cube_names(),
    robot_observation_name=robot_name,
)
articulation_controller = my_ur10.get_articulation_controller()

i = 0
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
        actions = my_controller.forward(observations=observations, end_effector_offset=np.array([0.0, 0.0, 0.02]))
        articulation_controller.apply_action(actions)
    frame_count += 1
    if args.test and frame_count >= 10:
        break

simulation_app.close()
