# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for managing Isaac Sim World instances and SDF path operations in the mobility generation system."""

import isaacsim.core
from pxr import Sdf


def new_world(physics_dt: float = 0.01, stage_units_in_meters: float = 1.0) -> isaacsim.core.api.World:
    """Create a new Isaac Sim World instance.

    Clears any existing World instance and creates a new one with the specified parameters.

    Args:
        physics_dt: Physics simulation timestep in seconds.
        stage_units_in_meters: Units conversion factor for stage units to meters.

    Returns:
        The newly created World instance.
    """
    world = get_world()
    if world is not None:
        isaacsim.core.api.World.clear_instance()
    isaacsim.core.api.World(physics_dt=physics_dt, stage_units_in_meters=stage_units_in_meters)
    return isaacsim.core.api.World.instance()


def get_world() -> isaacsim.core.api.World:
    """Current Isaac Sim World instance.

    Returns:
        The active World instance.
    """
    return isaacsim.core.api.World.instance()


def join_sdf_paths(*subpaths: str) -> str:
    """Join multiple subpaths into a single SDF path.

    Args:
        *subpaths: Path components to join. The first component is used as the base path,
            and subsequent components are treated as relative paths.

    Returns:
        The joined SDF path as a string.
    """
    path = Sdf.Path(subpaths[0])

    for subpath in subpaths[1:]:

        # consecutive paths are relative
        subpath = subpath.strip("/")

        if len(subpath) > 0:
            path = path.AppendPath(subpath)

    return str(path)
