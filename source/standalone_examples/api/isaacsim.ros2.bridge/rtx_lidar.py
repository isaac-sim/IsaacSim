# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Demonstrate RTX lidar sensor with ROS 2 PointCloud2 and LaserScan publishing."""

import argparse
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# Example for creating RTX lidar sensors and publishing PointCloud2 / LaserScan data
simulation_app = SimulationApp({"headless": False})
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path

# Enable the ROS 2 bridge so the RtxLidar*ROS2Publish* writers are registered.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
stage_utils.add_reference_to_stage(
    assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/background"
)
simulation_app.update()


def _read_laser_scan_metadata(prim) -> dict:
    """Read scan configuration from a rotary OmniLidar prim for LaserScan publishing.

    Mirrors the metadata extraction performed by ``OgnROS2RtxLidarHelper`` so the
    LaserScan writer can be initialized directly from the prim authored by
    ``Lidar.create()``.
    """
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("LaserScan: scanRateBaseHz or patternFiringRateHz is 0 on the lidar prim")
    return dict(
        horizontalFov=360.0,
        horizontalResolution=360.0 * rotation_rate / firing_rate,
        depthRange=[near_range, far_range],
        rotationRate=rotation_rate,
        azimuthRange=[-180.0, 180.0],
    )


# Create the 3D rotating RTX Lidar. Example_Rotary scans at 10 Hz, so tick_rate must be 10
# (see isaac_sim_sensors_multitick_lidar_tickrate_must_match_scanrate).
lidar = Lidar.create(
    path="/sensor",
    config="Example_Rotary",
    tick_rate=10.0,
    translations=[[0.0, 0.0, 1.0]],
)

# Create a 2D rotary RTX Lidar with the Example_Rotary_2D config.
lidar_2D = Lidar.create(
    path="/sensor_2D",
    config="Example_Rotary_2D",
    tick_rate=10.0,
    translations=[[0.0, 0.0, 1.0]],
)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# LidarSensor wraps the authoring prim, creates the render product, and exposes
# attach_writer() for both registered short names ("draw-point-cloud") and any
# Replicator writer registry name (the ROS 2 publishers).
sensor_3d = LidarSensor(lidar, annotators=[])
sensor_2d = LidarSensor(lidar_2D, annotators=[])

# 3D Lidar -> ROS 2 PointCloud2 publisher.
sensor_3d.attach_writer(
    "RtxLidarROS2PublishPointCloud",
    topicName="point_cloud",
    frameId="base_scan",
)

# 2D Lidar -> ROS 2 LaserScan publisher. The writer requires scan geometry
# (horizontal FOV/resolution, depth range, rotation rate, azimuth range), which
# OgnROS2RtxLidarHelper normally reads from the lidar prim. Replicate that here
# so the standalone API matches the OG node's behaviour.
laser_scan_meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(lidar_2D.paths[0]))
sensor_2d.attach_writer(
    "RtxLidarROS2PublishLaserScan",
    topicName="scan",
    frameId="base_scan",
    **laser_scan_meta,
)

# Visualize both point clouds in the viewport with distinct colors so the 3D and
# 2D scans are easy to tell apart (RGBA in [0, 1]).
sensor_3d.attach_writer(
    "draw-point-cloud",
    color=[0.0, 1.0, 0.5, 1.0],  # bright green
    size=0.05,
)
sensor_2d.attach_writer(
    "draw-point-cloud",
    color=[1.0, 0.2, 0.8, 1.0],  # bright magenta
    size=0.08,
)

simulation_app.update()

app_utils.play()

frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1
    if args.test and frame_count >= 10:
        break

# cleanup and shutdown
app_utils.stop()
simulation_app.close()
