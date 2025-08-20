# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaacsim import SimulationApp

simulation_app = SimulationApp()

from isaacsim.core.utils.stage import get_current_stage
from isaacsim.sensors.camera import Camera, SingleViewDepthSensorAsset

# Create a root XForm on the stage
stage = get_current_stage()
root = stage.DefinePrim("/root", "Xform")

# Create a new Camera prim on the stage
camera = Camera(prim_path="/root/Camera")

# Create a new Scope to contain the template render product for the depth sensor
scope = stage.DefinePrim("/root/TemplateRenderProduct", "Scope")

# Create a new template render product, applying the OmniSensorDepthSensorSingleViewAPI schema and setting custom attributes
depth_sensor_attributes = {
    "omni:rtx:post:depthSensor:baselineMM": 42,
}
render_product = SingleViewDepthSensorAsset.add_template_render_product(
    parent_prim_path="/root/TemplateRenderProduct",
    camera_prim_path="/root/Camera",
    **depth_sensor_attributes,
)

# Set default prim as the camera
stage.SetDefaultPrim(root)

# Export the stage
stage.Export("example_camera_with_depth_sensor.usd")
