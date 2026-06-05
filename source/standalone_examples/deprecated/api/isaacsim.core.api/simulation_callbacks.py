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

"""Demonstrate physics and rendering callbacks for robot joint control and state queries."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")

simulation_context = SimulationContext()
add_reference_to_stage(asset_path, "/Franka")

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize()
dof_ptr = art.get_dof_index("panda_joint2")

simulation_context.play()


def step_callback_1(step_size):
    """Set the target joint position for panda_joint2."""
    art.set_joint_positions([[-1.5]], joint_indices=[dof_ptr])


def step_callback_2(step_size):
    """Print the current joint position and simulation time."""
    print(
        "Current joint 2 position @ step "
        + str(simulation_context.current_time_step_index)
        + " : "
        + str(art.get_joint_positions(joint_indices=[dof_ptr]).item())
    )
    print("TIME: ", simulation_context.current_time)


def render_callback(event):
    """Print a message on each render frame."""
    print("Render Frame")


simulation_context.add_physics_callback("physics_callback_1", step_callback_1)
simulation_context.add_physics_callback("physics_callback_2", step_callback_2)
simulation_context.add_render_callback("render_callback", render_callback)
# Simulate 60 timesteps
for i in range(60):
    print("step", i)
    simulation_context.step(render=False)
# Render one frame
simulation_context.render()

simulation_context.stop()
simulation_app.close()
