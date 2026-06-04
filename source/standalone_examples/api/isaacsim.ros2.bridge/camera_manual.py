# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate manual ROS 2 camera publishing."""

import argparse
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

CAMERA_STAGE_PATH = "/Camera"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RealTimePathTracing", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom

# enable ROS2 bridge extension
app_utils.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

stage_utils.set_stage_units(meters_per_unit=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
stage_utils.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Creating a Camera prim
camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
xform_api = UsdGeom.XformCommonAPI(camera_prim)
xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
camera_prim.GetHorizontalApertureAttr().Set(21)
camera_prim.GetVerticalApertureAttr().Set(16)
camera_prim.GetProjectionAttr().Set("perspective")
camera_prim.GetFocalLengthAttr().Set(24)
camera_prim.GetFocusDistanceAttr().Set(400)
camera_prim.GetPrim().CreateAttribute("exposure:time", Sdf.ValueTypeNames.Float).Set(0.02)
camera_prim.GetPrim().CreateAttribute("exposure:responsivity", Sdf.ValueTypeNames.Float).Set(1.10267)
camera_prim.GetPrim().CreateAttribute("exposure:fStop", Sdf.ValueTypeNames.Float).Set(5.0)

simulation_app.update()

# Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
keys = og.Controller.Keys
ros_camera_graph, _, _, _ = og.Controller.edit(
    {
        "graph_path": ROS_CAMERA_GRAPH_PATH,
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
            ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
            ("createRenderProduct.inputs:width", 1280),
            ("createRenderProduct.inputs:height", 720),
            ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
        ],
    },
)

# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
og.Controller.evaluate_sync(ros_camera_graph)

simulation_app.update()

# Use the IsaacSimulationGate step value to block execution on specific frames.
# Get the render product path from the IsaacCreateRenderProduct node output.
render_product_path = og.Controller.attribute(
    f"{ROS_CAMERA_GRAPH_PATH}/createRenderProduct.outputs:renderProductPath"
).get()

import omni.syntheticdata._syntheticdata as sd

# Get name of rendervar for RGB sensor type
rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

# Get path to IsaacSimulationGate node in RGB pipeline
rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    rv_rgb + "IsaacSimulationGate", render_product_path
)

rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)

# Get path to IsaacSimulationGate node in Depth pipeline
depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    rv_depth + "IsaacSimulationGate", render_product_path
)

# Get path to IsaacSimulationGate node in CameraInfo pipeline
camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
    "PostProcessDispatch" + "IsaacSimulationGate", render_product_path
)

# Need to initialize physics getting any articulation..etc
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

app_utils.play()
simulation_app.update()

frame = 0

while simulation_app.is_running() and app_utils.is_playing():
    # Run with a fixed step size
    simulation_app.update()

    if app_utils.is_playing():
        # Rotate camera by 0.5 degree every frame
        xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        # Set the step value for the simulation gates to zero to stop execution
        og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(0)
        og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(0)
        og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(0)

        # Publish the ROS rgb image message every 5 frames
        if frame % 5 == 0:
            # Enable rgb Branch node to start publishing rgb image
            og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(1)

        # Publish the ROS Depth image message every 60 frames
        if frame % 60 == 0:
            # Enable depth Branch node to start publishing depth image
            og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(1)

        # Publish the ROS Camera Info message every frame
        og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(1)

        frame = frame + 1
        if args.test and frame >= 10:
            break

app_utils.stop()
simulation_app.close()
