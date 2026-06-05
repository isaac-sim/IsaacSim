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

"""Base class for visual materials that provides common functionality for USD material shader manipulation."""

from __future__ import annotations

from abc import ABC, abstractmethod

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.usd
import warp as wp
from isaacsim.core.experimental.prims import Prim
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import Sdf, Usd, UsdShade


class VisualMaterial(Prim, ABC):
    """Base class for visual materials.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        resolve_paths: Whether to resolve the given paths (true) or use them as is (false).
    """

    def __init__(self, paths: str | list[str], *, resolve_paths: bool = True) -> None:
        super().__init__(paths, resolve_paths=resolve_paths)
        if not hasattr(self, "_materials"):
            self._materials = []
        if not hasattr(self, "_shaders"):
            self._shaders = []
        self._inputs = {}

    """
    Properties.
    """

    @property
    def materials(self) -> list[UsdShade.Material]:
        """USD materials encapsulated by the wrapper.

        Returns:
            List of USD materials.

        Example:

        .. code-block:: python

            >>> prims.materials
            [UsdShade.Material(Usd.Prim(</World/prim_0>)),
             UsdShade.Material(Usd.Prim(</World/prim_1>)),
             UsdShade.Material(Usd.Prim(</World/prim_2>))]
        """
        return self._materials

    @property
    def shaders(self) -> list[UsdShade.Shader]:
        """USD shaders encapsulated by the wrapper.

        Returns:
            List of USD shaders.

        Example:

        .. code-block:: python

            >>> prims.shaders
            [UsdShade.Shader(Usd.Prim(</World/prim_0/Shader>)),
             UsdShade.Shader(Usd.Prim(</World/prim_1/Shader>)),
             UsdShade.Shader(Usd.Prim(</World/prim_2/Shader>))]
        """
        return self._shaders

    """
    Methods.
    """

    def set_input_values(
        self,
        name: str,
        values: str | bool | int | float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set shaders' input values.

        Args:
            name: Shader input name.
            values: Input values (shape and data type depend on the input name).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            ValueError: If the input name is invalid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import OmniGlassMaterial, OmniPbrMaterial, PreviewSurfaceMaterial
            >>>
            >>> # set same glass color (green) for all prims (OmniGlass)
            >>> if isinstance(prims, OmniGlassMaterial):
            ...     prims.set_input_values(name="glass_color", values=[0.0, 1.0, 0.0])
            >>>
            >>> # set the Albedo map (textures) for all prims (OmniPBR)
            >>> if isinstance(prims, OmniPbrMaterial):
            ...     prims.set_input_values(name="diffuse_texture", values=[
            ...         "/local/path/to/texture_0",
            ...         "/local/path/to/texture_1",
            ...         "/local/path/to/texture_2",
            ...     ])
            >>>
            >>> # set same diffuse color (red) for all prims (USD Preview Surface)
            >>> if isinstance(prims, PreviewSurfaceMaterial):
            ...     prims.set_input_values(name="diffuseColor", values=[1.0, 0.0, 0.0])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        spec = self._inputs.get(name)
        if spec is None:
            raise ValueError(f"Invalid input name: {name}. Supported inputs: {', '.join(self._inputs.keys())}")
        self._set_input_values(name=name, values=values, spec=spec, indices=indices)

    def get_input_values(self, name: str, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get shaders' input values.

        .. warning::

            Inputs are not initialized by default. They must be initialized before being read (see :py:meth:`set_input_values`).

        Args:
            name: Shader input name.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Values (shape and data type depend on the input name).

        Raises:
            AssertionError: Wrapped prims are not valid.
            ValueError: If the input name is invalid.
            RuntimeError: If the shader input is not initialized.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import OmniGlassMaterial, OmniPbrMaterial, PreviewSurfaceMaterial
            >>>
            >>> # get the glass IOR of all prims (OmniGlass)
            >>> if isinstance(prims, OmniGlassMaterial):
            ...     prims.set_input_values(name="glass_ior", values=[1.5])
            ...     glass_ior = prims.get_input_values(name="glass_ior")
            >>>
            >>> # get the metallic amount of all prims (OmniPBR)
            >>> if isinstance(prims, OmniPbrMaterial):
            ...     prims.set_input_values(name="metallic_constant", values=[0.5])
            ...     metallic_constant = prims.get_input_values(name="metallic_constant")
            >>>
            >>> # get the opacity of all prims (USD Preview Surface)
            >>> if isinstance(prims, PreviewSurfaceMaterial):
            ...     prims.set_input_values(name="opacity", values=[0.1])
            ...     opacity = prims.get_input_values(name="opacity")
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        spec = self._inputs.get(name)
        if spec is None:
            raise ValueError(f"Invalid input name: {name}. Supported inputs: {', '.join(self._inputs.keys())}")
        return self._get_input_values(name=name, spec=spec, indices=indices)

    """
    Static methods.
    """

    @staticmethod
    @abstractmethod
    def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array:
        """Check if the prims at the given paths are valid for creating material instances of this type.

        Args:
            paths: Prim paths (or prims) to check for.

        Returns:
            Boolean flags indicating if the prims are valid for creating material instances (shape ``(N, 1)``).
        """

    @staticmethod
    def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[VisualMaterial | None]:
        """Fetch instances of visual material from prims (or prim paths) at the given paths.

        Backends: :guilabel:`usd`.

        Args:
            paths: Prim paths (or prims) to get visual material instances from.

        Returns:
            List of visual material instances or ``None`` if the prim is not a supported visual material type.

        Example:

        .. code-block:: python

            >>> import omni.kit.commands
            >>> from isaacsim.core.experimental.materials import VisualMaterial
            >>>
            >>> # given a USD stage with the prims at paths /World, /World/A (USD Preview Surface)
            >>> omni.kit.commands.execute(
            ...     "CreatePreviewSurfaceMaterialPrim",
            ...     mtl_path=f"/World/A",
            ...     select_new_prim=False,
            ... )  # doctest: +NO_CHECK
            >>>
            >>> # fetch visual material instances
            >>> VisualMaterial.fetch_instances(["/World", "/World/A"])
            [None, <isaacsim.core.experimental.materials.impl.visual_materials.preview_surface.PreviewSurfaceMaterial object at 0x...>]
        """
        # defer imports to avoid circular dependencies
        from .omni_glass import OmniGlassMaterial
        from .omni_pbr import OmniPbrMaterial
        from .preview_surface import PreviewSurfaceMaterial

        classes = [OmniGlassMaterial, OmniPbrMaterial, PreviewSurfaceMaterial]

        instances = []
        stage = stage_utils.get_current_stage(backend="usd")
        for item in paths if isinstance(paths, (list, tuple)) else [paths]:
            prim = stage.GetPrimAtPath(item) if isinstance(item, str) else item
            instance = None
            for cls in classes:
                if cls.are_of_type(prim).numpy().item():
                    instance = cls(prim.GetPath().pathString)
                    break
            instances.append(instance)
        return instances

    """
    Internal methods.
    """

    def _set_input_values(
        self,
        *,
        name: str,
        values: str | bool | int | float | list | np.ndarray | wp.array,
        spec: dict,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set shader input values.

        Args:
            name: Shader input name.
            values: Input values to set on the shaders.
            spec: Input specification containing type and validation information.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
        """
        sdf_type, sdf_type_class = self._parse_sdf_type(spec["type"])
        place_func, _, set_func, _, _ = self._get_sdf_type_spec(sdf_type, sdf_type_class)
        # accommodate values
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        values = place_func(values, indices)
        # clip values (if range is provided)
        range_ = spec.get("range")
        if range_ is not None and not np.all((values >= range_[0]) & (values <= range_[1])):
            carb.log_warn(f"Input values for '{name}' are out of range. Values will be clipped to the range {range_}")
            values = np.clip(values, range_[0], range_[1])
        # set values
        for i, index in enumerate(indices.numpy()):
            shader = self.shaders[index]
            if shader.GetInput(name).Get() is None:
                shader.CreateInput(name, sdf_type)
            shader.GetInput(name).Set(set_func(values, i))

    def _get_input_values(
        self,
        *,
        name: str,
        spec: dict,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> list | wp.array:
        """Get shader input values.

        Args:
            name: Shader input name.
            spec: Input specification containing type information.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Values from the shader inputs.
        """
        sdf_type, sdf_type_class = self._parse_sdf_type(spec["type"])
        _, create_func, _, get_func, return_func = self._get_sdf_type_spec(sdf_type, sdf_type_class)
        # get values
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = create_func(indices)
        for i, index in enumerate(indices.numpy()):
            shader = self.shaders[index]
            value = shader.GetInput(name).Get()
            if value is None:
                raise RuntimeError(f"The attribute {name} is not initialized")
            data[i] = get_func(value)
        return return_func(data, self._device)

    """
    Internal static methods.
    """

    @staticmethod
    def _get_material_and_shader(
        stage: Usd.Stage, path: str
    ) -> tuple[UsdShade.Material | None, UsdShade.Shader | None]:
        """Get the material and shader for a given material path.

        Args:
            stage: USD stage containing the material.
            path: Path to the material prim.

        Returns:
            Two-elements tuple. 1) USD Material, if found. 2) USD Shader, if found.
        """
        material = None
        shader = None
        # material
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.IsA(UsdShade.Material):
            material = UsdShade.Material(prim)
        # shader
        if material is not None:
            shader = UsdShade.Shader(omni.usd.get_shader_from_material(prim, get_prim=True))
        if shader is None:
            for name in ["Shader", "shader"]:
                prim = stage.GetPrimAtPath(f"{path}/{name}")
                if prim.IsValid() and prim.IsA(UsdShade.Shader):
                    shader = UsdShade.Shader(prim)
                    break
        return material, shader

    @staticmethod
    def _parse_sdf_type(type_name: str | Sdf.ValueTypeName) -> tuple[Sdf.ValueTypeName, type]:
        """Parse a Sdf value type name into Sdf type and type class.

        Args:
            type_name: The Sdf value type name to parse.

        Returns:
            A tuple containing the Sdf ValueTypeName and its corresponding Python type class.

        Raises:
            ValueError: If the type name is invalid.
        """
        sdf_type = None
        if isinstance(type_name, Sdf.ValueTypeName):
            sdf_type = type_name
        elif isinstance(type_name, str):
            for item in dir(Sdf.ValueTypeNames):
                if type_name.lower() == item.lower():
                    sdf_type = getattr(Sdf.ValueTypeNames, item)
                    sdf_type = sdf_type if isinstance(sdf_type, Sdf.ValueTypeName) else None
                    break
        if sdf_type is None:
            # see https://openusd.org/release/api/_usd__page__datatypes.html
            raise ValueError(f"Invalid type name: {type_name} ({type(type_name)})")
        # get the Sdf type class
        sdf_type_class = sdf_type.type.pythonClass
        if sdf_type_class is None:
            sdf_type_class = type(sdf_type.defaultValue)
        return sdf_type, sdf_type_class

    @staticmethod
    def _get_sdf_type_spec(
        sdf_type: Sdf.ValueTypeName, sdf_type_class: type
    ) -> tuple[callable, callable, callable, callable]:
        """Get helper functions to place, create, set and get values for a given Sdf type.

        Args:
            sdf_type: The Sdf ValueTypeName to get helpers for.
            sdf_type_class: The Python type class associated with the Sdf type.

        Returns:
            A tuple of four callables (place_func, create_func, set_func, get_func, return_func) for handling
            the specified Sdf type.

        Raises:
            NotImplementedError: If the Sdf type is not supported.
        """
        # Gf.Vec*
        if hasattr(sdf_type_class, "__isGfVec"):
            dimension = sdf_type_class.dimension
            dtype = {"d": np.float64, "f": np.float32, "h": np.int32, "i": np.int32}[sdf_type.type.typeName[-1]]
            place_func = lambda data, indices: ops_utils.place(data, device="cpu").numpy().reshape((-1, dimension))
            create_func = lambda indices: np.zeros((indices.shape[0], dimension), dtype=dtype)
            set_func = lambda data, index: sdf_type_class(*data[0 if data.shape[0] == 1 else index].tolist())
            get_func = lambda value: np.array(value, dtype=dtype)
            return_func = lambda data, device: ops_utils.place(data, device=device)
        # bool, int, float
        elif sdf_type_class in [bool, int, float]:
            dtype = {bool: np.bool_, int: np.int32, float: np.float32}[sdf_type_class]
            place_func = lambda data, indices: ops_utils.place(data, device="cpu").numpy().reshape((-1, 1))
            create_func = lambda indices: np.zeros((indices.shape[0], 1), dtype=dtype)
            set_func = lambda data, index: sdf_type_class(data[0 if data.shape[0] == 1 else index].item())
            get_func = lambda value: sdf_type_class(value)
            return_func = lambda data, device: ops_utils.place(data, device=device)
        # Asset
        elif sdf_type == Sdf.ValueTypeNames.Asset:
            place_func = lambda data, indices: np.broadcast_to(
                np.array([data] if isinstance(data, str) else data, dtype=object), (indices.shape[0],)
            )
            create_func = lambda indices: np.empty((indices.shape[0],), dtype=object)
            set_func = lambda data, index: data[index]
            get_func = lambda value: value.path
            return_func = lambda data, device: data.tolist()
        else:
            raise NotImplementedError(f"Sdf type: {sdf_type}, Sdf type class: {sdf_type_class}")
        return place_func, create_func, set_func, get_func, return_func
