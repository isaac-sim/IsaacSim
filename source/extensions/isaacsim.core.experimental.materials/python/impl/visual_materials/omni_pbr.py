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

"""High level wrapper for creating/encapsulating Omniverse Physically-Based Rendering (``OmniPBR``) material prims."""

from __future__ import annotations

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import warp as wp
from pxr import Sdf, Usd

from .visual_material import VisualMaterial


class OmniPbrMaterial(VisualMaterial):
    """High level wrapper for creating/encapsulating Omniverse Physically-Based Rendering (``OmniPBR``) material prims.

    The ``OmniPBR`` is the default Physically based material available in Omniverse.
    This material can describe most opaque dielectric or non-dielectric materials.

    .. note::

        This class creates or wraps (one of both) OmniPBR prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the OmniPBR prims.
        * If the prim paths do not exist, OmniPBR prims are created at each path and a wrapper is placed over them.

    On visual materials, the shader parameters are encoded as inputs.
    The following tables summarize the ``OmniPBR`` material shader inputs (by group):

    .. list-table:: Albedo
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"diffuse_color_constant"``
          - Albedo base color.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"diffuse_texture"``
          - Albedo map.
          - ``(N,)``
          - ``str``
          -
        * - ``"albedo_desaturation"``
          - Albedo map desaturation.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"albedo_add"``
          - Adds constant value to the diffuse color.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"albedo_brightness"``
          - Multiplier for the diffuse color.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"diffuse_tint"``
          - Color tint (when enabled, this color value is multiplied over the final albedo color).
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``

    .. list-table:: Reflectivity
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"reflection_roughness_constant"``
          - Roughness amount (Higher values lead to more blurry reflections).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"reflection_roughness_texture_influence"``
          - Roughness map influence (blends between the constant value and the lookup of the roughness texture).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"reflectionroughness_texture"``
          - Roughness map.
          - ``(N,)``
          - ``str``
          -
        * - ``"metallic_constant"``
          - Metallic amount.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"metallic_texture_influence"``
          - Metallic map influence (blends between the constant value and the lookup of the metallic texture).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"metallic_texture"``
          - Metallic map.
          - ``(N,)``
          - ``str``
          -
        * - ``"specular_level"``
          - Specular level (intensity) of the material.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"enable_ORM_texture"``
          - Enable ORM texture (extracts occlusion, roughness and metallic textures from the RGB channels).
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"ORM_texture"``
          - ORM map (texture that has occlusion, roughness and metallic maps stored in the RGB channels).
          - ``(N,)``
          - ``str``
          -

    .. list-table:: Ambient Occlusion (AO)
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"ao_to_diffuse"``
          - AO to diffuse (amount of ambient occlusion multiplied against the diffuse color channel).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"ao_texture"``
          - Ambient occlusion map (texture for the material).
          - ``(N,)``
          - ``str``
          -

    .. list-table:: Emissive
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"enable_emission"``
          - Enable emission (enables the emission of light from the material).
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"emissive_color"``
          - Emission color.
          - ``(N, 3)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"emissive_color_texture"``
          - Emission color map.
          - ``(N,)``
          - ``str``
          -
        * - ``"emissive_mask_texture"``
          - Emission color mask map.
          - ``(N,)``
          - ``str``
          -
        * - ``"emissive_intensity"``
          - Emission intensity.
          - ``(N, 1)``
          - ``float``
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
        * - ``"opacity_texture"``
          - Opacity map.
          - ``(N,)``
          - ``str``
          -
        * - ``"opacity_constant"``
          - Opacity amount, when opacity map is not valid.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"enable_opacity_texture"``
          - Enable or disable the usage of the opacity texture map.
          - ``(N, 1)``
          - ``bool``
          -
        * - ``"opacity_mode"``
          - Determines how to lookup opacity from the supplied texture.
          - ``(N, 1)``
          - ``int``
          -
        * - ``"opacity_threshold"``
          - Opacity threshold.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``

    .. list-table:: Normal
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"geometry_normal_roughness_strength"``
          - Normal map to roughness weight (enables and weights roughness induced by normal maps).
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"bump_factor"``
          - Normal strength.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"normalmap_texture"``
          - Normal map.
          - ``(N,)``
          - ``str``
          -
        * - ``"detail_bump_factor"``
          - Detail normal strength.
          - ``(N, 1)``
          - ``float``
          -
        * - ``"detail_normalmap_texture"``
          - Detail normal map.
          - ``(N,)``
          - ``str``
          -
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
        * - ``"detail_texture_translate"``
          - Controls the position of the detail texture.
          - ``(N, 2)``
          - ``float``
          -
        * - ``"detail_texture_rotate"``
          - Rotates angle of the detail texture (in degrees).
          - ``(N, 1)``
          - ``float``
          -
        * - ``"detail_texture_scale"``
          - Controls the repetition of the detail texture.
          - ``(N, 2)``
          - ``float``
          -

    .. list-table:: Geometry
        :header-rows: 1

        * - Input name
          - Description
          - Shape / Length
          - Type
          - Range
        * - ``"round_edges_radius"``
          - Influence radius around the edges (in meters).
          - ``(N, 1)``
          - ``float``
          -
        * - ``"round_edges_roundness"``
          - Determines how round the edge will look.
          - ``(N, 1)``
          - ``float``
          - ``[0.0, 1.0]``
        * - ``"round_edges_across_materials"``
          - Round edge across materials.
          - ``(N, 1)``
          - ``bool``
          -

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.

    Raises:
        ValueError: If resulting paths are mixed or invalid.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import OmniPbrMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create visual material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = OmniPbrMaterial(paths)  # doctest: +NO_CHECK
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
                assert source_asset is not None, f"The USD Shader at path {path} is not an OmniPBR material"
                assert (
                    "omnipbr.mdl" in source_asset.path.lower()
                ), f"The USD Shader at path {path} (with source asset {source_asset.path}) is not an OmniPBR material"
                self._materials.append(material)
                self._shaders.append(shader)
        # - create prims
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                omni.kit.commands.execute(
                    "CreateMdlMaterialPrim",
                    mtl_url="OmniPBR.mdl",
                    mtl_name="OmniPBR",
                    mtl_path=path,
                    select_new_prim=False,
                )
                material, shader = self._get_material_and_shader(stage, path)
                assert material is not None and shader is not None, f"Unable to create OmniPBR material at path {path}"
                source_asset = shader.GetSourceAsset("mdl").path
                assert (
                    "omnipbr.mdl" in source_asset.lower()
                ), f"The USD Shader at path {path} (with source asset {source_asset}) is not an OmniPBR material"
                self._materials.append(material)
                self._shaders.append(shader)
        # initialize base class
        super().__init__(paths, resolve_paths=False)

        self._inputs = {
            # Albedo
            "diffuse_color_constant": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "diffuse_texture": {"type": Sdf.ValueTypeNames.Asset},
            "albedo_desaturation": {"type": Sdf.ValueTypeNames.Float},
            "albedo_add": {"type": Sdf.ValueTypeNames.Float},
            "albedo_brightness": {"type": Sdf.ValueTypeNames.Float},
            "diffuse_tint": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            # Reflectivity
            "reflection_roughness_constant": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "reflection_roughness_texture_influence": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "reflectionroughness_texture": {"type": Sdf.ValueTypeNames.Asset},
            "metallic_constant": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "metallic_texture_influence": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "metallic_texture": {"type": Sdf.ValueTypeNames.Asset},
            "specular_level": {"type": Sdf.ValueTypeNames.Float},
            # Reflectivity (ORM)
            "enable_ORM_texture": {"type": Sdf.ValueTypeNames.Bool},
            "ORM_texture": {"type": Sdf.ValueTypeNames.Asset},
            # AO
            "ao_to_diffuse": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "ao_texture": {"type": Sdf.ValueTypeNames.Asset},
            # Emissive
            "enable_emission": {"type": Sdf.ValueTypeNames.Bool},
            "emissive_color": {"type": Sdf.ValueTypeNames.Float3, "range": (0.0, 1.0)},
            "emissive_color_texture": {"type": Sdf.ValueTypeNames.Asset},
            "emissive_mask_texture": {"type": Sdf.ValueTypeNames.Asset},
            "emissive_intensity": {"type": Sdf.ValueTypeNames.Float},
            # Opacity
            "enable_opacity": {"type": Sdf.ValueTypeNames.Bool},
            "opacity_texture": {"type": Sdf.ValueTypeNames.Asset},
            "opacity_constant": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "enable_opacity_texture": {"type": Sdf.ValueTypeNames.Bool},
            "opacity_mode": {"type": Sdf.ValueTypeNames.Int},
            "opacity_threshold": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            # Normal
            "geometry_normal_roughness_strength": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "bump_factor": {"type": Sdf.ValueTypeNames.Float},
            "normalmap_texture": {"type": Sdf.ValueTypeNames.Asset},
            "detail_bump_factor": {"type": Sdf.ValueTypeNames.Float},
            "detail_normalmap_texture": {"type": Sdf.ValueTypeNames.Asset},
            "flip_tangent_u": {"type": Sdf.ValueTypeNames.Bool},
            "flip_tangent_v": {"type": Sdf.ValueTypeNames.Bool},
            # UV (UVW Projection)
            "project_uvw": {"type": Sdf.ValueTypeNames.Bool},
            "world_or_object": {"type": Sdf.ValueTypeNames.Bool},
            "uv_space_index": {"type": Sdf.ValueTypeNames.Int, "range": (0, 3)},
            # UV (UVW Adjustments)
            "texture_translate": {"type": Sdf.ValueTypeNames.Float2},
            "texture_rotate": {"type": Sdf.ValueTypeNames.Float},
            "texture_scale": {"type": Sdf.ValueTypeNames.Float2},
            "detail_texture_translate": {"type": Sdf.ValueTypeNames.Float2},
            "detail_texture_rotate": {"type": Sdf.ValueTypeNames.Float},
            "detail_texture_scale": {"type": Sdf.ValueTypeNames.Float2},
            # Geometry
            "round_edges_radius": {"type": Sdf.ValueTypeNames.Float},
            "round_edges_roundness": {"type": Sdf.ValueTypeNames.Float, "range": (0.0, 1.0)},
            "round_edges_across_materials": {"type": Sdf.ValueTypeNames.Bool},
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
            >>> result = OmniPbrMaterial.are_of_type(["/World", "/World/prim_0"])
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
                    status = "omnipbr.mdl" in source_asset.path.lower()
            data.append(status)
        return ops_utils.place(data, device="cpu").reshape((-1, 1))
