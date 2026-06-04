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
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.objects import Cube


def create_sarcophagus(apply_nonvisual_material: bool = True) -> dict:
    """Create a sarcophagus-shaped test environment made of cubes.

    Args:
        apply_nonvisual_material: Whether to apply nonvisual materials to the shapes.

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
        cube = Cube(
            f"/World/cube_{i*4}",
            sizes=1.0,
            positions=np.multiply(signs, np.array([l + 0.5, l / 2, h1 - h / 2])),
            scales=np.array([1, l, h]),
            colors=[1, 0, 0],
        )
        if apply_nonvisual_material:
            material = NonVisualMaterial(
                f"/World/cube_{i*4}/material",
                bases="aluminum",
                coatings="paint",
                attributes="emissive",
            )
            cube.apply_visual_materials(material)
            cube_info[f"/World/cube_{i*4}"] = {
                "material_id": NonVisualMaterial.encode_material_ids(material).numpy().item()
            }

        # place cube normal to y-axis
        cube = Cube(
            f"/World/cube_{i*4+1}",
            sizes=1.0,
            positions=np.multiply(signs, np.array([l / 2, l + 0.5, h1 - h / 2])),
            scales=np.array([l, 1, h]),
            colors=[0, 1, 0],
        )
        if apply_nonvisual_material:
            material = NonVisualMaterial(
                f"/World/cube_{i*4+1}/material",
                bases="steel",
                coatings="clearcoat",
                attributes="emissive",
            )
            cube.apply_visual_materials(material)
            cube_info[f"/World/cube_{i*4+1}"] = {
                "material_id": NonVisualMaterial.encode_material_ids(material).numpy().item()
            }

        # place cube normal to z-axis, top
        cube = Cube(
            f"/World/cube_{i*4+2}",
            sizes=1.0,
            positions=np.multiply(signs, np.array([l / 2, l / 2, h1 + 0.5])),
            scales=np.array([l, l, 1]),
            colors=[0, 0, 1],
        )
        if apply_nonvisual_material:
            material = NonVisualMaterial(
                f"/World/cube_{i*4+2}/material",
                bases="concrete",
                coatings="clearcoat",
                attributes="emissive",
            )
            cube.apply_visual_materials(material)
            cube_info[f"/World/cube_{i*4+2}"] = {
                "material_id": NonVisualMaterial.encode_material_ids(material).numpy().item()
            }

        # place cube normal to z-axis, bottom
        cube = Cube(
            f"/World/cube_{i*4+3}",
            sizes=1.0,
            positions=np.multiply(signs, np.array([l / 2, l / 2, -h2 - 0.5])),
            scales=np.array([l, l, 1]),
            colors=[1, 1, 0],
        )
        if apply_nonvisual_material:
            material = NonVisualMaterial(
                f"/World/cube_{i*4+3}/material",
                bases="concrete",
                coatings="paint",
                attributes="emissive",
            )
            cube.apply_visual_materials(material)
            cube_info[f"/World/cube_{i*4+3}"] = {
                "material_id": NonVisualMaterial.encode_material_ids(material).numpy().item()
            }

        i += 1
    return cube_info
