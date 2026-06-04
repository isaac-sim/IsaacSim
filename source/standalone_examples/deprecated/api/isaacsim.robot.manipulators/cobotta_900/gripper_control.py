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

"""Demonstrate Cobotta Pro 900 parallel gripper open and close control."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise Exception("Could not find Isaac Sim assets folder")
asset_path = assets_root_path + "/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")
# define the gripper
gripper = ParallelGripper(
    # We chose the following values while inspecting the articulation
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),
    action_deltas=np.array([-0.628, 0.628]),
)
# define the manipulator
my_denso = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/cobotta",
        name="cobotta_robot",
        end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
        gripper=gripper,
    )
)
# set the default positions of the other gripper joints to be opened so
# that its out of the way of the joints we want to control when gripping an object for instance.
joints_default_positions = np.zeros(12)
joints_default_positions[7] = 0.628
joints_default_positions[8] = 0.628
my_denso.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        i += 1
        gripper_positions = my_denso.gripper.get_joint_positions()
        if i < 500:
            # close the gripper slowly
            my_denso.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1])
            )
        if i > 500:
            # open the gripper slowly
            my_denso.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1])
            )
        if i == 1000:
            i = 0
    if args.test is True:
        break

simulation_app.close()
