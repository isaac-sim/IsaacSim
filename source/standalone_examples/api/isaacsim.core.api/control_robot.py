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

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot = add_reference_to_stage(usd_path=asset_path, prim_path="/Franka")
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")

simulation_context = SimulationContext()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize()
dof_ptr = art.get_dof_index("panda_joint2")

simulation_context.play()
# NOTE: before interacting with physics directly you need to step physics for one step at least
# simulation_context.step(render=True) which happens inside .play()
for i in range(1000):
    art.set_joint_positions([[-1.5]], joint_indices=[dof_ptr])
    simulation_context.step(render=True)

print("Reached: ", np.array2string(art.get_joint_positions(), precision=3, suppress_small=True))

simulation_context.stop()
simulation_app.close()
