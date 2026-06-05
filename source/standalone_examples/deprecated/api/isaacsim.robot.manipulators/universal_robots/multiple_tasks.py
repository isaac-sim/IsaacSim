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

"""Demonstrate UR10 robot executing multiple tasks concurrently."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)

import argparse

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka.controllers.stacking_controller import (
    StackingController as FrankaStackingController,
)
from isaacsim.robot.manipulators.examples.franka.tasks import Stacking as FrankaStacking
from isaacsim.robot.manipulators.examples.universal_robots.controllers import (
    StackingController as UR10StackingController,
)
from isaacsim.robot.manipulators.examples.universal_robots.tasks import Stacking as UR10Stacking
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
tasks = []
num_of_tasks = 2

tasks.append(FrankaStacking(name="task_0", offset=np.array([0, -2, 0])))
my_world.add_task(tasks[-1])
tasks.append(UR10Stacking(name="task_1", offset=np.array([0.5, 0.5, 0])))
my_world.add_task(tasks[-1])
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
kaya_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd"
my_kaya = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Kaya",
        name="my_kaya",
        wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
        create_robot=True,
        usd_path=kaya_asset_path,
        position=np.array([-1, 0, 0]),
    )
)
jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([-1.5, -1.5, 0]),
    )
)

for _ in range(10):
    simulation_app.update()
my_world.reset()
robots = []
for i in range(num_of_tasks):
    task_params = tasks[i].get_params()
    robots.append(my_world.scene.get_object(task_params["robot_name"]["value"]))

controllers = []
controllers.append(
    FrankaStackingController(
        name="pick_place_controller",
        gripper=robots[0].gripper,
        robot_articulation=robots[0],
        picking_order_cube_names=tasks[0].get_cube_names(),
        robot_observation_name=robots[0].name,
    )
)
controllers[-1].reset()
controllers.append(
    UR10StackingController(
        name="pick_place_controller",
        gripper=robots[1].gripper,
        robot_articulation=robots[1],
        picking_order_cube_names=tasks[1].get_cube_names(),
        robot_observation_name=robots[1].name,
    )
)
controllers[-1].reset()

kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.prim_path, com_prim_path="/World/Kaya/base_link/control_offset"
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()
kaya_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

jetbot_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)

articulation_controllers = []
for i in range(num_of_tasks):
    articulation_controllers.append(robots[i].get_articulation_controller())

i = 0
my_world.pause()
reset_needed = False
if args.test:
    my_world.play()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            controllers[0].reset()
            controllers[1].reset()
            kaya_controller.reset()
            jetbot_controller.reset()
            i = 0
            reset_needed = False
        observations = my_world.get_observations()
        actions = controllers[0].forward(observations=observations, end_effector_offset=np.array([0, 0, 0]))
        articulation_controllers[0].apply_action(actions)
        actions = controllers[1].forward(observations=observations, end_effector_offset=np.array([0, 0, 0.02]))
        articulation_controllers[1].apply_action(actions)
        if i >= 0 and i < 500:
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0.2, 0.0, 0.0]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.1, 0]))
        elif i >= 500 and i < 1000:
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0, 0.2, 0.0]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.0, np.pi / 10]))
        elif i >= 1000 and i < 1500:
            my_kaya.apply_wheel_actions(kaya_controller.forward(command=[0, 0.0, 0.6]))
            my_jetbot.apply_wheel_actions(jetbot_controller.forward(command=[0.1, 0]))
        if args.test and i >= 10:
            break
        i += 1


simulation_app.close()
