# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.examples.universal_robots.tasks import BinFilling

my_world = World(stage_units_in_meters=1.0)
my_task = BinFilling()
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
my_ur10 = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(name="pick_place_controller", gripper=my_ur10.gripper, robot_articulation=my_ur10)
articulation_controller = my_ur10.get_articulation_controller()

added_screws = False
reset_needed = False
task_completed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        task_completed = False
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            added_screws = False
            reset_needed = False
            task_completed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["bin_name"]["value"]]["position"],
            placing_position=observations[task_params["bin_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, -0.098, 0.03]),
            end_effector_orientation=euler_angles_to_quat(np.array([np.pi, 0, np.pi / 2.0])),
        )
        if not added_screws and my_controller.get_current_event() == 6 and not my_controller.is_paused():
            my_controller.pause()
            my_task.add_screws(screws_number=20)
            added_screws = True
        if my_controller.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True
        articulation_controller.apply_action(actions)
simulation_app.close()
