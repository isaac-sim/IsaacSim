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

"""High level wrapper for creating/encapsulating USD Preview Surface material prims."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import warp as wp
from pxr import Sdf, Usd

from .visual_material import VisualMaterial


class PreviewSurfaceMaterial(VisualMaterial):
    """High level wrapper for creating/encapsulating USD Preview Surface (``UsdPreviewSurface``) material prims.

    The ``UsdPreviewSurface`` is intended to model a modern physically based surface that strikes a balance between
    expressiveness and reliable interchange between engines and real-time rendering clients.

    .. note::

        This class creates or wraps (one of both) USD Preview Surface prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the USD Preview Surface prims.
        * If the prim paths do not exist, USD Preview Surface prims are created at each path and a wrapper is placed over them.

    On visual materials, the shader parameters are encoded as inputs.
    The following table summarizes the ``UsdPreviewSurface`` material shader inputs:

    .. list-table::
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"diffuseColor"``
          - Color reflected from the material surface when light hits it.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"emissiveColor"``
          - Color of the light emitted by the material.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"specularColor"``
          - Color of the highlights that appear on a surface when it reflects light.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"useSpecularWorkflow"``
          - Operation mode (specular workflow: 1, metallic workflow: 0).
          - ``(N, 1)``
          - ``int``
          - ``[0, 1]``
        * - ``"metallic"``
          - Metallic (1.0 for metallic surfaces and 0.0 for non-metallic surfaces, in between for mixed surfaces).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"roughness"``
          - Roughness (specular lobe).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"clearcoat"``
          - Clearcoat (second specular lobe amount).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"clearcoatRoughness"``
          - Clearcoat roughness (second specular lobe roughness).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"opacity"``
          - Opacity (0.0 for fully transparent, 1.0 for fully opaque, in between for translucent).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"opacityThreshold"``
          - Opacity threshold.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"ior"``
          - Index of refraction (IOR).
          - ``(N, 1)``
          - ``float``
          -
        * - ``"normal"``
          - Normal vector.
          - ``(N, 3)``
          - ``float``
          - ``[-1.0, 1.0]``
        * - ``"displacement"``
          - Displacement (in the direction of the normal).
          - ``(N, 1)``
          - ``float``
          -

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.

    Raises:
        ValueError: If resulting paths are mixed or invalid.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create visual material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = PreviewSurfaceMaterial(paths)  # doctest: +NO_CHECK
    """

    def __init__(self, paths: str | list[str]) -> None:
        # get or create prims
        self._shaders = []
        self._materials = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = self.resolve_paths(paths)
        # - get prims
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                material, shader = self._get_material_and_shader(stage, path)
                assert material is not None, f"The wrapped prim at path {path} is not a USD Material"
                assert shader is not None, f"The wrapped prim at path {path} is not a USD Shader"
                shader_id = shader.GetIdAttr().Get()
                assert (
                    shader_id == "UsdPreviewSurface"
                ), f"The USD Shader at path {path} (with id {shader_id}) is not a USD Preview Surface material"
                self._materials.append(material)
                self._shaders.append(shader)
        # - create prims
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                omni.kit.commands.execute("CreatePreviewSurfaceMaterialPrim", mtl_path=path, select_new_prim=False)
                material, shader = self._get_material_and_shader(stage, path)
                assert (
                    material is not None and shader is not None
                ), f"Unable to create USD Preview Surface material at path {path}"
                shader_id = shader.GetIdAttr().Get()
                assert (
                    shader_id == "UsdPreviewSurface"
                ), f"The USD Shader at path {path} (with id {shader_id}) is not a USD Preview Surface material"
                self._materials.append(material)
                self._shaders.append(shader)
        # initialize base class
        super().__init__(paths, resolve_paths=False)

        self._inputs = {
            "diffuseColor": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "emissiveColor": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "specularColor": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "useSpecularWorkflow": {"type": Sdf.ValueTypeNames.Int, "range": (0, 1)},
            "metallic": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "roughness": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "clearcoat": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "clearcoatRoughness": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "opacity": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "opacityThreshold": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "ior": {"type": Sdf.ValueTypeNames.Float},
            "normal": {"type": Sdf.ValueTypeNames.Float3, "range": (-1.0, 1.0)},
            "displacement": {"type": Sdf.ValueTypeNames.Float},
        }

    """
    Static methods.
    """

    @staticmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating material instances of this type.

        Backends: :guilabel:`usd`.

        .. warning::

            Since this method is static, the output is always on the CPU.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating material instances (shape ``(N, 1)``).

        Example:

        .. code-block:: python

            >>> # check if the following prims at paths are valid for creating instances
            >>> result = PreviewSurfaceMaterial.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [[False]
             [ True]]
        """
        data = []
        stage = stage_utils.get_current_stage(backend="usd")
        for item in paths if isinstance(paths, (list, tuple)) else [paths]:
            status = False
            path = item if isinstance(item, str) else item.GetPath()
            material, shader = VisualMaterial._get_material_and_shader(stage, path)
            if material is not None and shader is not None:
                shader_id = shader.GetIdAttr().Get()
                status = shader_id == "UsdPreviewSurface"
            data.append(status)
        return ops_utils.place(data, device="cpu").reshape((-1, 1))
