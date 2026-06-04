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

"""Base class for visual material representations in Isaac Sim."""

from pxr import Usd, UsdShade


class VisualMaterial(object):
    """Base class for visual material representations.

    Args:
        name: Name identifier for the material.
        prim_path: USD prim path for the material.
        prim: The USD prim object.
        shaders_list: List of shaders used by the material.
        material: The USD material object.

    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        prim: Usd.Prim,
        shaders_list: list[UsdShade.Shader],
        material: UsdShade.Material,
    ) -> None:
        self._shaders_list = shaders_list
        self._material = material
        self._name = name
        self._prim_path = prim_path
        self._prim = prim
        return

    @property
    def material(self) -> UsdShade.Material:
        """Get the USD material object.

        Returns:
            The UsdShade.Material object.

        """
        return self._material

    @property
    def shaders_list(self) -> list[UsdShade.Shader]:
        """Get the list of shaders used by the material.

        Returns:
            List of UsdShade.Shader objects.

        """
        return self._shaders_list

    @property
    def name(self) -> str:
        """Get the material name.

        Returns:
            The material name.

        """
        return self._name

    @property
    def prim_path(self) -> str:
        """Get the USD prim path.

        Returns:
            The prim path string.

        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """Get the USD prim object.

        Returns:
            The Usd.Prim object.

        """
        return self._prim
