# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated collision utility functions."""

from __future__ import annotations

# python
import numpy as np

# omniverse
import omni.physics.core

# isaacsim
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, PhysicsSchemaTools, UsdGeom


def ray_cast(
    position: np.array, orientation: np.array, offset: np.array, max_dist: float = 100.0
) -> tuple[None | str, float]:
    """Projects a raycast forward along x axis with specified offset.

    If a hit is found within the maximum distance, then the object's prim path and distance to it is returned.
    Otherwise, a None and 10000 is returned.

    Args:
        position: Origin's position for ray cast.
        orientation: Origin's orientation for ray cast.
        offset: Offset for ray cast.
        max_dist: Maximum distance to test for collisions in stage units.

    Returns:
        Path to geometry that was hit and hit distance, returns None, 10000 if no hit occurred.
    """
    input_tr = Gf.Matrix4f()
    input_tr.SetTranslate(Gf.Vec3f(*position.tolist()))
    input_tr.SetRotateOnly(Gf.Quatf(*orientation.tolist()))
    offset_transform = Gf.Matrix4f()
    offset_transform.SetTranslate(Gf.Vec3f(*offset.tolist()))
    raycast_tf = offset_transform * input_tr
    trans = raycast_tf.ExtractTranslation()
    direction = raycast_tf.ExtractRotation().TransformDir((1, 0, 0))
    origin = (trans[0], trans[1], trans[2])
    ray_dir = (direction[0], direction[1], direction[2])

    ret, hit = omni.physics.core.get_physics_scene_query_interface().raycast_closest(origin, ray_dir, max_dist, True)
    if ret:
        usdGeom = UsdGeom.Mesh.Get(get_current_stage(), PhysicsSchemaTools.intToSdfPath(hit.rigid_body))
        distance = hit.distance
        return usdGeom.GetPath().pathString, distance
    return None, 10000.0
