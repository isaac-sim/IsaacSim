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

"""
Manus Glove and Vive Tracker Visualization Sample

This sample demonstrates simultaneous tracking of Manus gloves and Vive trackers
in Isaac Sim. The devices are visualized as colored cubes:
- Red cubes (0.01 size): Manus glove joints
- Blue cubes (0.02 size): Vive tracker positions

IMPORTANT: To avoid resource contention and crashes, ensure Manus and Vive devices
are connected to different USB controllers/buses. Use 'lsusb -t' to identify
different buses and connect devices accordingly.
"""

import os
import numpy as np
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp(
    {"headless": False}, 
    experience=f'/home/yuanchenl/Desktop/IsaacSim/source/apps/isaacsim.exp.base.xr.openxr.kit'
)

import carb
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.prims import create_prim, set_prim_visibility
from omni.isaac.core.prims import XFormPrim
from pxr import Gf, Sdf, UsdGeom, UsdLux

# Import our XR device integration
try:
    from isaacsim.xr.input_devices.impl.xr_device_integration import get_xr_device_integration
    carb.log_info("Successfully imported XR device integration helper")
except ImportError as e:
    carb.log_error(f"Failed to import XR device integration: {e}")
    simulation_app.close()
    exit(1)

# Create world and lighting
my_world = World(stage_units_in_meters=1.0)

# Add Light Source
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# Create cube prototypes
hidden_prim = create_prim("/Hidden/Prototypes", "Scope")

# Red cube for Manus gloves (small)
manus_cube_path = "/Hidden/Prototypes/ManusCube"
VisualCuboid(
    prim_path=manus_cube_path,
    size=0.01, 
    color=np.array([255, 0, 0]),  # Red color
)

# Blue cube for Vive trackers (larger)
vive_cube_path = "/Hidden/Prototypes/ViveCube"
VisualCuboid(
    prim_path=vive_cube_path,
    size=0.02, 
    color=np.array([0, 0, 255]),  # Blue color
)

set_prim_visibility(hidden_prim, False)

# Create point instancer for cubes
instancer_path = "/World/DeviceCubeInstancer"
point_instancer = UsdGeom.PointInstancer.Define(my_world.stage, instancer_path)
point_instancer.CreatePrototypesRel().SetTargets([Sdf.Path(manus_cube_path), Sdf.Path(vive_cube_path)])

max_devices = 60

# Initially hide all cubes until devices are tracked (index 2 = hidden, since we have 2 prototypes: 0=manus, 1=vive)
point_instancer.CreateProtoIndicesAttr().Set([2 for _ in range(max_devices)])

# Initialize positions and orientations
positions = [Gf.Vec3f(0.0, 0.0, 0.0) for i in range(max_devices)]
point_instancer.CreatePositionsAttr().Set(positions)

orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0) for _ in range(max_devices)]
point_instancer.CreateOrientationsAttr().Set(orientations)

# Add instancer to world scene
instancer_prim = XFormPrim(prim_path=instancer_path)
my_world.scene.add(instancer_prim)

# Get XR device integration from extension (to avoid singleton conflicts)
xr_integration = get_xr_device_integration()
carb.log_info("Using XR device integration from extension")

my_world.reset()
reset_needed = False

# Frame counter for sequential device updates
frame_counter = 0

# Get attribute references for faster access
positions_attr = point_instancer.GetPositionsAttr()
orientations_attr = point_instancer.GetOrientationsAttr()
proto_idx_attr = point_instancer.GetProtoIndicesAttr()

carb.log_info("Starting Manus Glove and Vive Tracker visualization with sequential updates")
carb.log_info("Red cubes (0.01) for Manus gloves, Blue cubes (0.02) for Vive trackers")

# Main simulation loop
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False

        # Sequential device updates to avoid resource contention
        # Alternate between Manus and Vive every frame
        update_manus = (frame_counter % 2 == 0)
        frame_counter += 1
        
        # Get current cube arrays
        current_positions = positions_attr.Get()
        current_orientations = orientations_attr.Get()
        proto_indices = proto_idx_attr.Get()

        # Initialize all cubes as hidden (index 2 = hidden, since we have 2 prototypes: 0=manus, 1=vive)
        proto_indices = [2 for _ in range(max_devices)]
        proto_idx_attr.Set(proto_indices)
        cube_idx = 0
        
        if update_manus:
            xr_integration.update_manus()
        else:
            xr_integration.update_vive()
        
        all_device_data = xr_integration.get_all_device_data()
        manus_data = all_device_data.get('manus_gloves', {})
        vive_data = all_device_data.get('vive_trackers', {})
            
        # Process Manus gloves (red cubes, index 0)
        for joint, joint_data in manus_data.items():
            if cube_idx >= max_devices:
                break
            pos, ori = joint_data['position'], joint_data['orientation']
            current_positions[cube_idx] = Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2]))
            current_orientations[cube_idx] = Gf.Quath(float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3]))
            proto_indices[cube_idx] = 0  # Red Manus cube
            cube_idx += 1

        # Process Vive trackers (blue cubes, index 1)
        for joint, joint_data in vive_data.items():
            if cube_idx >= max_devices:
                break
            pos, ori = joint_data['position'], joint_data['orientation']
            current_positions[cube_idx] = Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2]))
            current_orientations[cube_idx] = Gf.Quath(float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3]))
            proto_indices[cube_idx] = 1  # Blue Vive cube
            cube_idx += 1
        
        # Update the instancer with new positions and orientations
        positions_attr.Set(current_positions)
        orientations_attr.Set(current_orientations)
        proto_idx_attr.Set(proto_indices)

# Cleanup
xr_integration.cleanup()
simulation_app.close() 