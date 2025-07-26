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
import os
from typing import Tuple

from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.replicator.mobility_gen.impl.camera import MobilityGenCamera
from isaacsim.replicator.mobility_gen.impl.common import Module
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import join_sdf_paths
from isaacsim.storage.native import get_assets_root_path


class HawkCamera(Module):

    usd_url: str = get_assets_root_path() + "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
    resolution: Tuple[int, int] = (960, 600)
    left_camera_path: str = "left/camera_left"
    right_camera_path: str = "right/camera_right"

    def __init__(self, left: MobilityGenCamera, right: MobilityGenCamera):
        self.left = left
        self.right = right

    @classmethod
    def build(cls, prim_path: str) -> "HawkCamera":

        stage = get_current_stage()

        add_reference_to_stage(usd_path=cls.usd_url, prim_path=prim_path)

        return cls.attach(prim_path)

    @classmethod
    def attach(cls, prim_path: str) -> "HawkCamera":

        left_camera = MobilityGenCamera(join_sdf_paths(prim_path, cls.left_camera_path), cls.resolution)
        right_camera = MobilityGenCamera(join_sdf_paths(prim_path, cls.right_camera_path), cls.resolution)

        return HawkCamera(left_camera, right_camera)
