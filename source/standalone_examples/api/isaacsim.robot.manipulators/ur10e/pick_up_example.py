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
from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from tasks.pick_place import PickPlace

my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=20 / 200)

target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
my_task = PickPlace(name="ur10e_pick_place", target_position=target_position, cube_size=np.array([0.1, 0.0515, 0.1]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("ur10e_pick_place").get_params()
ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)
# initialize the controller

my_controller = PickPlaceController(name="controller", robot_articulation=my_ur10e, gripper=my_ur10e.gripper)
task_params = my_world.get_task("ur10e_pick_place").get_params()
articulation_controller = my_ur10e.get_articulation_controller()

reset_needed = False
task_completed = False

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
            task_completed = False
        if my_world.current_time_step_index == 0:
            my_controller.reset()

        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # This offset needs tuning as well
            end_effector_offset=np.array([0, 0, 0.20]),
        )
        if my_controller.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True
        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True


simulation_app.close()
