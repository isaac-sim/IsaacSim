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

"""Common utilities for RTX sensor tests."""

import numpy as np
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.sensors.rtx import (
    apply_nonvisual_material,
    get_material_id,
)


def create_sarcophagus(enable_nonvisual_material: bool = True) -> dict:
    """Create a sarcophagus-shaped test environment made of cubes.

    Args:
        enable_nonvisual_material: Whether to apply nonvisual materials to the cubes.

    Returns:
        Dictionary mapping cube prim paths to their material information.
    """
    # Autogenerate sarcophagus
    dims = [(10, 5, 7), (15, 9, 11), (20, 13, 15), (25, 17, 19)]
    i = 0
    cube_info = {}
    for l, h1, h2 in dims:
        h = h1 + h2
        x_sign = -1 if 0 < i < 3 else 1
        y_sign = -1 if i > 1 else 1
        signs = np.array([x_sign, y_sign, 1])

        # Place cube normal to x-axis
        cube = VisualCuboid(
            prim_path=f"/World/cube_{i*4}",
            name=f"cube_{i*4}",
            position=np.multiply(signs, np.array([l + 0.5, l / 2, h1 - h / 2])),
            scale=np.array([1, l, h]),
        )
        if enable_nonvisual_material:
            material = OmniPBR(
                prim_path=f"/World/cube_{i*4}/material",
                name=f"cube_{i*4}_material",
                color=np.array([1, 0, 0]),
            )
            apply_nonvisual_material(material.prim, "aluminum", "paint", "emissive")
            cube.apply_visual_material(material)
            cube_info[f"/World/cube_{i*4}"] = {"material_id": get_material_id(material.prim)}

        # place cube normal to y-axis
        cube = VisualCuboid(
            prim_path=f"/World/cube_{i*4+1}",
            name=f"cube_{i*4+1}",
            position=np.multiply(signs, np.array([l / 2, l + 0.5, h1 - h / 2])),
            scale=np.array([l, 1, h]),
        )
        if enable_nonvisual_material:
            material = OmniPBR(
                prim_path=f"/World/cube_{i*4+1}/material",
                name=f"cube_{i*4+1}_material",
                color=np.array([0, 1, 0]),
            )
            apply_nonvisual_material(material.prim, "steel", "clearcoat", "emissive")
            cube.apply_visual_material(material)
            cube_info[f"/World/cube_{i*4+1}"] = {"material_id": get_material_id(material.prim)}

        # place cube normal to z-axis, top
        cube = VisualCuboid(
            prim_path=f"/World/cube_{i*4+2}",
            name=f"cube_{i*4+2}",
            position=np.multiply(signs, np.array([l / 2, l / 2, h1 + 0.5])),
            scale=np.array([l, l, 1]),
        )
        if enable_nonvisual_material:
            material = OmniPBR(
                prim_path=f"/World/cube_{i*4+2}/material",
                name=f"cube_{i*4+2}_material",
                color=np.array([0, 0, 1]),
            )
            apply_nonvisual_material(material.prim, "concrete", "clearcoat", "emissive")
            cube.apply_visual_material(material)
            cube_info[f"/World/cube_{i*4+2}"] = {"material_id": get_material_id(material.prim)}

        # place cube normal to z-axis, bottom
        cube = VisualCuboid(
            prim_path=f"/World/cube_{i*4+3}",
            name=f"cube_{i*4+3}",
            position=np.multiply(signs, np.array([l / 2, l / 2, -h2 - 0.5])),
            scale=np.array([l, l, 1]),
        )
        if enable_nonvisual_material:
            material = OmniPBR(
                prim_path=f"/World/cube_{i*4+3}/material",
                name=f"cube_{i*4+3}_material",
                color=np.array([1, 1, 0]),
            )
            apply_nonvisual_material(material.prim, "concrete", "paint", "emissive")
            cube.apply_visual_material(material)
            cube_info[f"/World/cube_{i*4+3}"] = {"material_id": get_material_id(material.prim)}

        i += 1
    return cube_info
