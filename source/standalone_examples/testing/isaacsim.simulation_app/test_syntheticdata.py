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

"""Test synthetic data sensor outputs for a robot scene."""

import sys

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp()
sys.stdout.flush()
import omni

simulation_app.update()
omni.usd.get_context().new_stage()
simulation_app.update()

from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils.semantics import add_labels
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport

viewport_api = get_active_viewport()
render_product_path = viewport_api.get_render_product_path()

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    resolution=(1280, 720),
    render_product_path=render_product_path,
)
# play to start capturing data
omni.timeline.get_timeline_interface().play()
simulation_app.update()
camera.initialize()


cube = Cube(
    "/new_cube_1",
    positions=[[5.0, 3, 1.0]],
    scales=[[0.6, 0.5, 0.2]],
    sizes=1.0,
)
add_labels(cube.prims[0], labels=["cube"], taxonomy="class")
simulation_app.update()
for annotator in [
    "pointcloud",
    "normals",
    "motion_vectors",
    "occlusion",
    "distance_to_image_plane",
    "distance_to_camera",
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "bounding_box_3d",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
]:
    getattr(camera, "add_{}_to_frame".format(annotator))()

for _ in range(5):
    simulation_app.update()
rgba = camera.get_rgba()
print(rgba.size)

if rgba.size != 1280 * 720 * 4:
    import carb

    carb.log_error(f"[fatal] RGB buffer has size of {rgba.size} which is not {1280*720*4}")
    sys.exit(1)
# Cleanup application
simulation_app.close()
