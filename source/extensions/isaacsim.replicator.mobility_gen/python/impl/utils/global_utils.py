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


import isaacsim.core
import omni.kit
from pxr import Sdf


def new_world(physics_dt: float = 0.01, stage_units_in_meters: float = 1.0) -> isaacsim.core.api.World:
    world = get_world()
    if world is not None:
        isaacsim.core.api.World.clear_instance()
    isaacsim.core.api.World(physics_dt=physics_dt, stage_units_in_meters=stage_units_in_meters)
    return isaacsim.core.api.World.instance()


def get_world() -> isaacsim.core.api.World:
    return isaacsim.core.api.World.instance()


def join_sdf_paths(*subpaths):

    path = Sdf.Path(subpaths[0])

    for subpath in subpaths[1:]:

        # consecutive paths are relative
        subpath = subpath.strip("/")

        if len(subpath) > 0:
            path = path.AppendPath(subpath)

    return str(path)
