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

"""High level wrapper for creating/encapsulating Omniverse Glass (``OmniGlass``) material prims."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import warp as wp
from pxr import Sdf, Usd

from .visual_material import VisualMaterial


class OmniGlassMaterial(VisualMaterial):
    """High level wrapper for creating/encapsulating Omniverse Glass (``OmniGlass``) material prims.

    The ``OmniGlass`` is an improved physical glass model that simulates light transmission
    through thin walled and transmissive surfaces.

    .. note::

        This class creates or wraps (one of both) OmniGlass prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the OmniGlass prims.
        * If the prim paths do not exist, OmniGlass prims are created at each path and a wrapper is placed over them.

    On visual materials, the shader parameters are encoded as inputs.
    The following tables summarize the ``OmniGlass`` material shader inputs (by group):

    .. list-table:: Color
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"glass_color"``
          - Color of the glass.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"glass_color_texture"``
          - Texture to be used for the glass color.
          - ``(N,)``
          - ``str``
          -
        * - ``"depth"``
          - Volume absorption scale (control how much light is absorbed through the surface).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1000.0]``

    .. list-table:: Roughness
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"frosting_roughness"``
          - Roughness of the glass material.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"roughness_texture_influence"``
          - Influence of the roughness texture on the glass material.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"roughness_texture"``
          - Roughness texture.
          - ``(N,)``
          - ``str``
          -

    .. list-table:: Refraction
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"glass_ior"``
          - Glass Index of refraction (IOR).
          - ``(N, 1)``
          - ``float``
          -
        * - ``"thin_walled"``
          - Whether the glass is thin-walled (when enabled, the material is considered thin-walled).
          - ``(N, 1)``
          - ``bool``
          -

    .. list-table:: Reflection
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"reflection_color_texture"``
          - Reflection color texture.
          - ``(N,)``
          - ``str``
          -
        * - ``"reflection_color"``
          - Reflection color.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``

    .. list-table:: Normal
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"normal_map_texture"``
          - Normal map texture.
          - ``(N,)``
          - ``str``
          -
        * - ``"normal_map_strength"``
          - Strength of the normal map.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"geometry_normal_roughness_strength"``
          - Normal map to roughness weight (enables and weights roughness induced by normal maps).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"flip_tangent_u"``
          - Flip tangent U.
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"flip_tangent_v"``
          - Flip tangent V.
          - ``(N, 1)``
          - ``bool``
          -

    .. list-table:: Opacity
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"enable_opacity"``
          - Enable the use of cutout opacity.
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"cutout_opacity"``
          - Opacity value, when opacity map is not valid.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"cutout_opacity_texture"``
          - Opacity map.
          - ``(N,)``
          - ``str``
          -
        * - ``"cutout_opacity_mono_source"``
          - Determines how to lookup opacity from the supplied texture.
          - ``(N, 1)``
          - ``int``
          -
        * - ``"opacity_threshold"``
          - Opacity threshold.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``

    .. list-table:: UV
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"project_uvw"``
          - Enable Project UVW Coordinates (UV coordinates will be generated by projecting them from a coordinate system).
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"world_or_object"``
          - Enable world space (when enabled, uses world space for projection, otherwise object space is used).
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"uv_space_index"``
          - UV space index.
          - ``(N, 1)``
          - ``int``
          - ``[0, 3]``
        * - ``"texture_translate"``
          - Controls position of texture.
          - ``(N, 2)``
          - ``float``
          -
        * - ``"texture_rotate"``
          - Rotates angle of texture (in degrees).
          - ``(N, 1)``
          - ``float``
          -
        * - ``"texture_scale"``
          - Controls the repetition of the texture.
          - ``(N, 2)``
          - ``float``
          -

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.

    Raises:
        ValueError: If resulting paths are mixed or invalid.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import OmniGlassMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create visual material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = OmniGlassMaterial(paths)  # doctest: +NO_CHECK
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
                source_asset = shader.GetSourceAsset("mdl")
                assert source_asset is not None, f"The USD Shader at path {path} is not an OmniGlass material"
                assert (
                    "omniglass.mdl" in source_asset.path.lower()
                ), f"The USD Shader at path {path} (with source asset {source_asset.path}) is not an OmniGlass material"
                self._materials.append(material)
                self._shaders.append(shader)
        # - create prims
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                omni.kit.commands.execute(
                    "CreateMdlMaterialPrim",
                    mtl_url="OmniGlass.mdl",
                    mtl_name="OmniGlass",
                    mtl_path=path,
                    select_new_prim=False,
                )
                material, shader = self._get_material_and_shader(stage, path)
                assert (
                    material is not None and shader is not None
                ), f"Unable to create OmniGlass material at path {path}"
                source_asset = shader.GetSourceAsset("mdl").path
                assert (
                    "omniglass.mdl" in source_asset.lower()
                ), f"The USD Shader at path {path} (with source asset {source_asset}) is not an OmniGlass material"
                self._materials.append(material)
                self._shaders.append(shader)
        # initialize base class
        super().__init__(paths, resolve_paths=False)

        self._inputs = {
            # Color
            "glass_color": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "glass_color_texture": {"type": Sdf.ValueTypeNames.Asset},
            "depth": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1000.0)},
            # Roughness
            "frosting_roughness": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "roughness_texture_influence": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "roughness_texture": {"type": Sdf.ValueTypeNames.Asset},
            # Refraction
            "glass_ior": {"type": Sdf.ValueTypeNames.Float},
            "thin_walled": {"type": Sdf.ValueTypeNames.Bool},
            # Reflection
            "reflection_color_texture": {"type": Sdf.ValueTypeNames.Asset},
            "reflection_color": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            # Normal
            "normal_map_texture": {"type": Sdf.ValueTypeNames.Asset},
            "normal_map_strength": {"type": Sdf.ValueTypeNames.Float},
            "geometry_normal_roughness_strength": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "flip_tangent_u": {"type": Sdf.ValueTypeNames.Bool},
            "flip_tangent_v": {"type": Sdf.ValueTypeNames.Bool},
            # Opacity
            "enable_opacity": {"type": Sdf.ValueTypeNames.Bool},
            "cutout_opacity": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "cutout_opacity_texture": {"type": Sdf.ValueTypeNames.Asset},
            "cutout_opacity_mono_source": {"type": Sdf.ValueTypeNames.Int},
            "opacity_threshold": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            # UV (UVW Projection)
            "project_uvw": {"type": Sdf.ValueTypeNames.Bool},
            "world_or_object": {"type": Sdf.ValueTypeNames.Bool},
            "uv_space_index": {"type": Sdf.ValueTypeNames.Int, "range": (0, 3)},
            # UV (UVW Adjustments)
            "texture_translate": {"type": Sdf.ValueTypeNames.Float2},
            "texture_rotate": {"type": Sdf.ValueTypeNames.Float},
            "texture_scale": {"type": Sdf.ValueTypeNames.Float2},
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
            >>> result = OmniGlassMaterial.are_of_type(["/World", "/World/prim_0"])
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
                source_asset = shader.GetSourceAsset("mdl")
                if source_asset is not None:
                    status = "omniglass.mdl" in source_asset.path.lower()
            data.append(status)
        return ops_utils.place(data, device="cpu").reshape((-1, 1))
