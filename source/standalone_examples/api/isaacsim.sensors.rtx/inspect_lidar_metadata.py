# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.rtx import LidarRtx, get_gmo_data
from isaacsim.storage.native import get_assets_root_path

LIDAR_AUX_DATA_LEVELS = {"NONE": 0, "BASIC": 1, "EXTRA": 2, "FULL": 3}
# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "--aux-data-level",
    default="BASIC",
    choices=list(LIDAR_AUX_DATA_LEVELS.keys()),
    help="Lidar prim auxiliary data level.",
)
args, unknown = parser.parse_known_args()

aux_data_level = args.aux_data_level

# Create a world
my_world = World(stage_units_in_meters=1.0)

# Load the small warehouse scene
assets_root_path = get_assets_root_path()
add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd", prim_path="/World/warehouse"
)

# Place a basic lidar in the scene, overriding attributes as necessary
custom_attributes = {"omni:sensor:Core:auxOutputType": args.aux_data_level}
lidar = my_world.scene.add(LidarRtx(prim_path="/World/lidar", name="lidar", **custom_attributes))

# Initialize the lidar and attach an annotator
ANNOTATOR_NAME = "GenericModelOutput"
lidar.initialize()
lidar.attach_annotator(ANNOTATOR_NAME)


# Run for a few frames, inspecting the data at each step
def inspect_lidar_metadata(frame: int, gmo_buffer: dict) -> None:
    # Read GenericModelOutput struct from buffer
    gmo = get_gmo_data(gmo_buffer)
    # Print some useful information
    carb.log_warn(f"Frame {frame} -- Lidar auxiliary level: {args.aux_data_level}")
    carb.log_warn(f"numElements: {gmo.numElements}")
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["BASIC"]:
        carb.log_warn(f"Scan complete: {gmo.scanComplete}")
        carb.log_warn(f"azimuthOffset: {gmo.azimuthOffset}")
        carb.log_warn(f"emitterId is a {type(gmo.emitterId)} of size {gmo.numElements}")
        carb.log_warn(f"channelId is a {type(gmo.channelId)} of size {gmo.numElements}")
        carb.log_warn(f"tickId is a {type(gmo.tickId)} of size {gmo.numElements}")
        carb.log_warn(f"echoId is a {type(gmo.echoId)} of size {gmo.numElements}")
        carb.log_warn(f"tickStates is a {type(gmo.tickStates)} of size {gmo.numElements}")
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["EXTRA"]:
        carb.log_warn(f"objId is a {type(gmo.objId)} of size {gmo.numElements}")
        carb.log_warn(f"matId is a {type(gmo.matId)} of size {gmo.numElements}")
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["FULL"]:
        carb.log_warn(f"hitNormals is a {type(gmo.hitNormals)} of size {gmo.numElements}")
        carb.log_warn(f"velocities is a {type(gmo.velocities)} of size {gmo.numElements}")
    return


timeline = omni.timeline.get_timeline_interface()
timeline.play()
i = 0
# Run for 10 frames in test mode
print(aux_data_level)
while simulation_app.is_running() and (not args.test or i < 10):
    simulation_app.update()
    data = lidar.get_current_frame()[ANNOTATOR_NAME]
    if len(data) > 0:
        inspect_lidar_metadata(frame=i, gmo_buffer=data)
    i += 1
timeline.stop()

simulation_app.close()
