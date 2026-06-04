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

"""Demonstrate rotating PhysX lidar sensor."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.physx import RotatingLidarPhysX
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["left_wheel", "right_wheel"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0.5]),
    )
)

my_lidar = my_world.scene.add(
    RotatingLidarPhysX(
        prim_path="/World/Carter/chassis_link/lidar", name="lidar", translation=np.array([-0.06, 0, 0.38])
    )
)

cube_1 = my_world.scene.add(
    DynamicCuboid(prim_path="/World/cube", name="cube_1", position=np.array([2, 2, 2.5]), scale=np.array([20, 0.2, 5]))
)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube_2", name="cube_2", position=np.array([2, -2, 2.5]), scale=np.array([20, 0.2, 5])
    )
)

my_controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)

my_world.reset()
my_lidar.add_depth_data_to_frame()
my_lidar.add_point_cloud_data_to_frame()
my_lidar.enable_visualization()
i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        # print(imu_sensor.get_current_frame())
        if i >= 0 and i < 1000:
            # print(my_lidar.get_current_frame())
            # forward
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        elif i >= 1000 and i < 1265:
            # rotate
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.0, np.pi / 12]))
        elif i >= 1265 and i < 2000:
            # forward
            my_carter.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
        elif i == 2000:
            i = 0
        i += 1
    if args.test is True:
        break
simulation_app.close()
