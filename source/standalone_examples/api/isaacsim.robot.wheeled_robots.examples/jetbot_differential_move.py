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

"""Demonstrate Jetbot differential drive movement."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
from isaacsim.core.experimental.objects import DomeLight
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path

DEVICE = "cpu"

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")
jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)

my_jetbot = WheeledRobot(
    paths="/World/Jetbot",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=jetbot_asset_path,
    positions=[0.0, 0.0, 0.05],
)
my_controller = DifferentialController(wheel_radius=0.03, wheel_base=0.1125)

# Setup simulation and disable GPU dynamics for this example.
SimulationManager.setup_simulation(dt=1.0 / 60.0, device=DEVICE)
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
app_utils.play()
app_utils.update_app(steps=10)

# -----------------------------------------------------------------------------
# Simulation loop: apply differential commands when `timeline` is playing.
# -----------------------------------------------------------------------------
i = 0
reset_needed = False
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
        if i >= 0 and i < 1000:
            velocities = my_controller.forward([0.1, 0.0])
            my_jetbot.apply_wheel_actions(velocities)
        elif i >= 1000 and i < 1300:
            velocities = my_controller.forward([0.0, np.pi / 12])
            my_jetbot.apply_wheel_actions(velocities)
        elif i >= 1300 and i < 2000:
            velocities = my_controller.forward([0.1, 0.0])
            my_jetbot.apply_wheel_actions(velocities)
        elif i == 2000:
            i = 0
        i += 1
    if args.test:
        break

app_utils.stop()
simulation_app.close()
