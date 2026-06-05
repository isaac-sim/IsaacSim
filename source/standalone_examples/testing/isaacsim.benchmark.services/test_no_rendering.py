# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test benchmark services with no rendering enabled."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

app_utils.enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# ----------------------------------------------------------------------
# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_physx_lidar",
)


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(
    asset_path, "/World/panda", variants=[("Gripper", "AlternateFinger"), ("Mesh", "Quality")]
)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
while stage_utils.is_stage_loading():
    simulation_app.update()
print("Loading Complete")

benchmark.set_phase("benchmark")

SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()
simulation_app.update()

for _ in range(0, 1000):
    SimulationManager.step()

benchmark.store_measurements()
min_physics_time = 0
for measurement in benchmark._test_phases[0].measurements:
    if "Min Physics Frametime" in measurement.name:
        min_physics_time = measurement.value

benchmark.stop()
app_utils.stop()

simulation_app.update()

assert min_physics_time > 0
simulation_app.close()
