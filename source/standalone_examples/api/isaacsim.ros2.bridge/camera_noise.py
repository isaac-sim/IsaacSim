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

"""Demonstrate ROS 2 camera publishing with GPU-based noise augmentation."""

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

simulation_app = SimulationApp(CONFIG)
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
import omni.replicator.core as rep
import warp as wp
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Sdf, UsdGeom

# enable ROS bridge extension
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

# grab our render product and directly set the camera prim
render_product_path = get_active_viewport().get_render_product_path()
ViewportManager.set_camera(CAMERA_STAGE_PATH)


# GPU Noise Kernel for illustrative purposes, input is rgba, outputs rgb
@wp.kernel
def image_gaussian_noise_warp(
    data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8), seed: int, sigma: float = 0.5
):
    """Apply Gaussian noise to an image using warp."""
    i, j = wp.tid()
    dim_i = data_out.shape[0]
    dim_j = data_out.shape[1]
    pixel_id = i * dim_i + j
    state_r = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 0))
    state_g = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 1))
    state_b = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 2))

    data_out[i, j, 0] = wp.uint8(float(data_in[i, j, 0]) + (255.0 * sigma * wp.randn(state_r)))
    data_out[i, j, 1] = wp.uint8(float(data_in[i, j, 1]) + (255.0 * sigma * wp.randn(state_g)))
    data_out[i, j, 2] = wp.uint8(float(data_in[i, j, 2]) + (255.0 * sigma * wp.randn(state_b)))


# register new augmented annotator that adds noise to rgba and then outputs to rgb to the ROS publisher can publish
rep.annotators.register(
    name="rgb_gaussian_noise",
    annotator=rep.annotators.augment_compose(
        source_annotator=rep.annotators.get("rgb", device="cuda"),
        augmentations=[
            rep.annotators.Augmentation.from_function(
                image_gaussian_noise_warp, sigma=0.1, seed=1234, data_out_shape=(-1, -1, 3)
            ),
        ],
    ),
)

# Create a new writer with the augmented image
rep.writers.register_node_writer(
    name=f"CustomROS2PublishImage",
    node_type_id="isaacsim.ros2.bridge.ROS2PublishImage",
    annotators=[
        "rgb_gaussian_noise",
        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
            "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
        ),
    ],
    category="custom",
)
# Register writer for Replicator telemetry tracking
(
    rep.WriterRegistry._default_writers.append("CustomROS2PublishImage")
    if "CustomROS2PublishImage" not in rep.WriterRegistry._default_writers
    else None
)

# Create the new writer and attach to our render product
writer = rep.writers.get(f"CustomROS2PublishImage")
writer.initialize(topicName="rgb_augmented", frameId="sim_camera")
writer.attach([render_product_path])

simulation_app.update()
# Need to initialize physics getting any articulation..etc
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play()
simulation_app.update()

frame = 0

while simulation_app.is_running():
    # Run with a fixed step size
    simulation_app.update()

    if app_utils.is_playing():
        # Rotate camera by 0.5 degree every frame
        xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        frame = frame + 1
        if args.test and frame >= 10:
            break

app_utils.stop()
simulation_app.close()
