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

"""RTX Lidar integration with a robot using the LidarRtx class.

This example demonstrates how to:
- Use the LidarRtx class to manage a lidar sensor on an existing robot prim
- Attach annotators to collect point cloud data
- Enable debug draw visualization via the class API
- Access frame data through get_current_frame()
- Use pause/resume for data acquisition control

The LidarRtx class provides a high-level API for working with RTX lidars,
handling render product creation and annotator management automatically.

For basic lidar creation using commands, see create_lidar_basic.py.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="RTX Lidar robot integration using LidarRtx class.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=False required for debug draw visualization
simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "lidar_robot_integration")
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.rtx import LidarRtx
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

# Locate Isaac Sim assets folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# =============================================================================
# SET UP WORLD AND ROBOT
# =============================================================================
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Load NovaCarter robot which includes a lidar sensor
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0]),
    )
)

# Add walls for the lidar to detect (widened spacing to avoid robot collision)
cube_1 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/wall_left", name="wall_left", position=np.array([5, 5, 2.5]), scale=np.array([20, 0.2, 5])
    )
)
cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/wall_right", name="wall_right", position=np.array([5, -5, 2.5]), scale=np.array([20, 0.2, 5])
    )
)

# =============================================================================
# SET UP CAMERA FOR OBSERVATION
# =============================================================================
# Create a camera positioned to observe the robot and lidar point cloud
stage = omni.usd.get_context().get_stage()
camera_path = "/World/ObservationCamera"
camera_prim = stage.DefinePrim(camera_path, "Camera")
camera = UsdGeom.Camera(camera_prim)

# Position camera to view the scene from an elevated angle
xform = UsdGeom.Xformable(camera_prim)
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(-30, 0, 10))  # Position: behind and above
xform.AddRotateXYZOp().Set(Gf.Vec3f(75, 0, -90))  # Look down at scene

# Set the active viewport to use this camera
viewport = get_active_viewport()
if viewport:
    viewport.camera_path = camera_path
    carb.log_info(f"Set active viewport camera to {camera_path}")

# =============================================================================
# CREATE LIDARRTX FROM EXISTING ROBOT SENSOR
# =============================================================================
# The NovaCarter robot already has a lidar sensor at this path. LidarRtx can
# wrap an existing OmniLidar prim, creating render product and managing
# annotators automatically.

lidar_prim_path = "/World/Carter/chassis_link/sensors/XT_32/PandarXT_32_10hz"
my_lidar = my_world.scene.add(LidarRtx(prim_path=lidar_prim_path, name="lidar"))

carb.log_info(f"Created LidarRtx wrapper for existing sensor at {lidar_prim_path}")

# =============================================================================
# INITIALIZE AND CONFIGURE SENSOR
# =============================================================================
# Reset world to initialize all objects
my_world.reset()

# Attach annotator for point cloud extraction
# Available annotators:
#   - "IsaacCreateRTXLidarScanBuffer": XYZ point cloud with optional metadata channels
#   - "IsaacComputeRTXLidarFlatScan": 2D flat scan data
#   - "GenericModelOutput": Raw sensor output (recommended for direct access)
#   - "StableIdMap": Object ID to prim path mapping
my_lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")
carb.log_info("Attached point cloud annotator")

# Enable debug draw visualization
# This is equivalent to: my_lidar.attach_writer("RtxLidarDebugDrawPointCloud")
my_lidar.enable_visualization()
carb.log_info("Enabled debug draw visualization")

# Get render product path (useful for custom annotator/writer attachment)
render_product_path = my_lidar.get_render_product_path()
carb.log_info(f"Render product path: {render_product_path}")

# =============================================================================
# SET UP ROBOT CONTROLLER
# =============================================================================
my_controller = DifferentialController(name="simple_control", wheel_radius=0.04295, wheel_base=0.4132)

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION
# =============================================================================
# Use omni.timeline for simulation control
timeline = omni.timeline.get_timeline_interface()
timeline.play()

carb.log_info("Starting simulation - robot will drive in a circular pattern")
carb.log_info("Observe the lidar point cloud updating in real-time")

frame_count = 0
was_playing = False

while simulation_app.is_running():
    simulation_app.update()

    # Only process when timeline is playing (physics is active)
    if not timeline.is_playing():
        was_playing = False
        continue

    # Reinitialize physics when timeline restarts after being stopped
    if not was_playing:
        my_world.reset()
        my_controller.reset()
        was_playing = True

    # =============================================================================
    # ACCESS SENSOR DATA
    # =============================================================================
    # get_current_frame() returns a dict with:
    #   - "rendering_time": Simulation time when frame was rendered
    #   - "rendering_frame": Frame number tuple (numerator, denominator)
    #   - "<annotator_name>": Data from each attached annotator
    frame_data = my_lidar.get_current_frame()

    # Get point cloud data from the attached annotator
    point_cloud = frame_data.get("IsaacCreateRTXLidarScanBuffer")
    if point_cloud is not None and len(point_cloud) > 0:
        # Point cloud is Nx3 array of XYZ coordinates
        num_points = len(point_cloud)
        if frame_count % 60 == 0:  # Print every ~1 second
            carb.log_info(f"Frame {frame_count}: {num_points} points detected")

    # =============================================================================
    # DRIVE ROBOT IN A CIRCULAR PATTERN
    # =============================================================================
    # Continuous circular motion: forward velocity + constant angular velocity
    my_carter.apply_wheel_actions(my_controller.forward(command=[0.3, np.pi / 8]))

    frame_count += 1

    # Exit after a few frames in test mode
    if args.test and frame_count > 10:
        break

# =============================================================================
# CLEANUP
# =============================================================================
# Disable visualization before closing
my_lidar.disable_visualization()
timeline.stop()
simulation_app.close()
