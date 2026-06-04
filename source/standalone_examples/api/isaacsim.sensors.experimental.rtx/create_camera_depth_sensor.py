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

"""Create a depth camera sensor asset with an embedded template render product.

This example demonstrates how to:
- Create an ``RtxCamera`` prim and embed a template ``RenderProduct`` prim with
  ``OmniSensorDepthSensorSingleViewAPI`` applied
- Export the resulting USD asset for later use with ``SingleViewDepthCameraSensor``

The exported asset can be loaded as a USD reference (e.g. via ``RtxCamera.create``)
and ``SingleViewDepthCameraSensor`` will automatically detect the embedded render
product and copy its depth sensor attributes.
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Create camera depth sensor asset example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp()

from isaacsim.core.experimental.utils.stage import define_prim, get_current_stage
from isaacsim.sensors.experimental.rtx import RtxCamera, SingleViewDepthCameraSensor

output_dir = os.path.join(
    os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_camera_depth_sensor"
)
os.makedirs(output_dir, exist_ok=True)

# Create a root XForm on the stage
root = define_prim("/root", "Xform")

# Create a new RtxCamera prim on the stage
cam = RtxCamera("/root/Camera")

# Create a Scope to hold the template render product for the depth sensor
define_prim("/root/TemplateRenderProduct", "Scope")

# Create the template render product with OmniSensorDepthSensorSingleViewAPI applied
# and custom depth sensor attributes
depth_sensor_attributes = {
    "omni:rtx:post:depthSensor:baselineMM": 42,
}
SingleViewDepthCameraSensor.add_template_render_product(
    parent_prim_path="/root/TemplateRenderProduct",
    camera_prim_path="/root/Camera",
    **depth_sensor_attributes,
)

# Set default prim and export
stage = get_current_stage(backend="usd")
stage.SetDefaultPrim(root)
stage.Export(os.path.join(output_dir, "example_camera_with_depth_sensor.usd"))

if args.test:
    stage.Export(os.path.join(output_dir, "stage.usda"))

simulation_app.close()
