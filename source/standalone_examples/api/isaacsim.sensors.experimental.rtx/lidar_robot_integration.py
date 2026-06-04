# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""RTX Lidar integration with a NovaCarter robot using the new experimental API.

This example demonstrates how to:
- Load a NovaCarter robot and create a lidar parented to its chassis
- Use ``Lidar.create()`` to attach a lidar at a specific robot location
- Attach a custom Writer to receive and print GMO point counts
- Attach the ``RtxSensorDebugDrawPointCloud`` writer for visualization
- Drive the robot in a circle while collecting lidar data

For basic lidar creation, see ``create_lidar_basic.py``.
For vendor configs and variants, see ``create_lidar_with_config_and_variants.py``.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="RTX Lidar robot integration using the new experimental API.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=False required for debug draw visualization
simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "lidar_robot_integration")
os.makedirs(output_dir, exist_ok=True)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from isaacsim.storage.native import get_assets_root_path

app_utils.enable_extension("isaacsim.sensors.rtx.nodes")
from omni.kit.viewport.utility import get_active_viewport
from omni.replicator.core import Writer
from pxr import Gf, UsdGeom

# =============================================================================
# LOCATE ASSETS
# =============================================================================
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# =============================================================================
# SET UP WORLD AND ROBOT
# =============================================================================
stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane")

# Load NovaCarter robot
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
my_carter = WheeledRobot(
    "/World/Carter",
    wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
    usd_path=asset_path,
    positions=np.array([[0, 0.0, 0]]),
)

# Add walls for the lidar to detect
wall_left = Cube(
    paths="/World/wall_left",
    positions=np.array([[5, 5, 2.5]]),
    scales=np.array([[20, 0.2, 5]]),
)
RigidPrim(paths="/World/wall_left")
GeomPrim(paths="/World/wall_left", apply_collision_apis=True)

wall_right = Cube(
    paths="/World/wall_right",
    positions=np.array([[5, -5, 2.5]]),
    scales=np.array([[20, 0.2, 5]]),
)
RigidPrim(paths="/World/wall_right")
GeomPrim(paths="/World/wall_right", apply_collision_apis=True)

# =============================================================================
# SET UP CAMERA FOR OBSERVATION
# =============================================================================
usd_stage = omni.usd.get_context().get_stage()
camera_path = "/World/ObservationCamera"
camera_prim = usd_stage.DefinePrim(camera_path, "Camera")

xform = UsdGeom.Xformable(camera_prim)
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(-30, 0, 10))
xform.AddRotateXYZOp().Set(Gf.Vec3f(75, 0, -90))

viewport = get_active_viewport()
if viewport:
    viewport.camera_path = camera_path
    print(f"Set active viewport camera to {camera_path}")

# =============================================================================
# CREATE LIDAR PARENTED TO THE ROBOT CHASSIS
# =============================================================================
# Use Lidar.create() to create a lidar and parent it to the robot chassis.
# The lidar is positioned at the chassis link so it moves with the robot.

lidar = Lidar.create(
    "/World/Carter/chassis_link/lidar",
    config="Example_Rotary",
    translations=np.array([0, 0, 0.5]),  # Raised slightly above chassis
    aux_output_level="FULL",
)

print(f"Created lidar at {lidar.paths[0]}")

# =============================================================================
# CREATE LIDAR SENSOR FOR DATA ACCESS
# =============================================================================
# LidarSensor wraps a Lidar authoring object and provides runtime data access.
# We pass ``annotators=[]`` because the custom writer brings its own annotator.
# The ``draw-point-cloud`` writer provides real-time visualization.

sensor = LidarSensor(lidar, annotators=[], writers=["draw-point-cloud"])

print("Created LidarSensor with debug draw writer")


# =============================================================================
# CUSTOM WRITER FOR GMO ROBOT INSPECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# It prints point count every 60 frames to monitor lidar output during driving.


class GmoRobotInspectWriter(Writer):
    """Writer that parses GenericModelOutput and prints point count periodically."""

    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self._frame_count = 0

    def write(self, data):
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements > 0 and self._frame_count % 60 == 0:
                print(f"Frame {self._frame_count}: {gmo.numElements} points detected")
        self._frame_count += 1


rep.WriterRegistry.register(GmoRobotInspectWriter)
sensor.attach_writer("GmoRobotInspectWriter")

print("Attached GmoRobotInspectWriter to sensor")

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# INITIALIZE WORLD AND CONTROLLER
# =============================================================================
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
my_controller = DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)

# =============================================================================
# RUN SIMULATION
# =============================================================================
app_utils.play()
simulation_app.update()

print("Starting simulation - robot will drive in a circular pattern")
print("Observe the lidar point cloud updating in real-time")

frame_count = 0

while simulation_app.is_running():
    simulation_app.update()

    if not app_utils.is_playing():
        continue

    # =============================================================================
    # DRIVE ROBOT IN A CIRCULAR PATTERN
    # =============================================================================
    wheel_velocities = my_controller.forward(command=[0.3, np.pi / 8])
    my_carter.apply_wheel_actions(wheel_velocities)

    frame_count += 1

    if args.test and frame_count > 10:
        break

# =============================================================================
# CLEANUP
# =============================================================================
app_utils.stop()
simulation_app.close()
