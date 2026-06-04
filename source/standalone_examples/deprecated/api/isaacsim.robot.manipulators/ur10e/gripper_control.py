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

"""Demonstrate UR10e gripper control."""

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
asset_path = assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/ur")
# define the gripper
gripper = ParallelGripper(
    # We chose the following values while inspecting the articulation
    end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
    joint_prim_names=["finger_joint"],
    joint_opened_positions=np.array([0]),
    joint_closed_positions=np.array([40]),
    action_deltas=np.array([-40]),
    use_mimic_joints=True,
)
# define the manipulator
my_ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/ur",
        name="ur10_robot",
        end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
        gripper=gripper,
    )
)

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
        gripper_positions = my_ur10.gripper.get_joint_positions()
        if i < 400:
            # close the gripper slowly
            my_ur10.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] + 0.1]))
        if i > 400:
            # open the gripper slowly
            my_ur10.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] - 0.1]))
        if i == 800:
            i = 0
    if args.test is True:
        break

simulation_app.close()
