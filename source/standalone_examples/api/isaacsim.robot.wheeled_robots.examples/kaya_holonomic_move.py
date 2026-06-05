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

"""Demonstrate Kaya holonomic movement."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.objects import DomeLight
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import HolonomicController
from isaacsim.robot.experimental.wheeled_robots.robots import (
    HolonomicRobotUsdSetup,
    WheeledRobot,
)
from isaacsim.storage.native import get_assets_root_path

DEVICE = "cpu"

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")
kaya_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)

my_kaya = WheeledRobot(
    paths="/World/Kaya",
    wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
    usd_path=kaya_asset_path,
    positions=[0.0, 0.0, 0.02],
    orientations=[1.0, 0.0, 0.0, 0.0],
)

kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.paths[0], com_prim_path="/World/Kaya/base_link/control_offset"
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()
my_controller = HolonomicController(
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

# Setup simulation and disable GPU dynamics for this example.
SimulationManager.setup_simulation(dt=1.0 / 60.0, device=DEVICE)
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
app_utils.play()
app_utils.update_app(steps=10)

# -----------------------------------------------------------------------------
# Simulation loop: apply holonomic commands when `timeline` is playing.
# -----------------------------------------------------------------------------
i = 0
reset_needed = False
frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False
        if i >= 0 and i < 500:
            velocities = my_controller.forward([0.4, 0.0, 0.0])
            my_kaya.apply_wheel_actions(velocities)
        elif i >= 500 and i < 1000:
            velocities = my_controller.forward([0.0, 0.4, 0.0])
            my_kaya.apply_wheel_actions(velocities)
        elif i >= 1000 and i < 1200:
            velocities = my_controller.forward([0.0, 0.0, 0.05])
            my_kaya.apply_wheel_actions(velocities)
        elif i == 1200:
            i = 0
        i += 1
    frame_count += 1
    if args.test and frame_count >= 10:
        break

app_utils.stop()
simulation_app.close()
