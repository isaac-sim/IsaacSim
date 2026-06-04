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

"""Demonstrate cloning ant robots using GridCloner and controlling them as articulations."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.cloner import GridCloner
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane")
DistantLight("/World/DistantLight").set_intensities(1000)

# create initial robot
asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/Ant/ant.usd"
stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Ants/Ant_0")

# create GridCloner instance
cloner = GridCloner(spacing=2)

# generate paths for clones
target_paths = cloner.generate_paths("/World/Ants/Ant", 4)

# clone
position_offsets = np.array([[0, 0, 1]] * 4)
cloner.clone(
    source_prim_path="/World/Ants/Ant_0",
    prim_paths=target_paths,
    position_offsets=position_offsets,
    replicate_physics=True,
    base_env_path="/World/Ants",
)

# create Articulation
ants = Articulation("/World/Ants/.*/torso")

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()
positions, orientations = ants.get_world_poses()
print(f"Positions:\n{positions.numpy()}")
print(f"Orientations (wxyz):\n{orientations.numpy()}")

for i in range(1000):
    simulation_app.update()
simulation_app.close()
