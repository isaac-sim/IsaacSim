# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from omni.physx.bindings import _physx as physx_bindings
from omni.physx.scripts import deformableUtils
from pxr import Usd

from .physics_material import PhysicsMaterial


class VolumeDeformableMaterial(PhysicsMaterial):
    """High level wrapper for creating/encapsulating Volume Deformable material prims.

    .. warning::

        The deformable materials require the Deformable feature (beta) to be enabled.
        Deformable feature (beta) can be enabled in *apps/.kit* experience files by setting
        ``physics.enableDeformableBeta = true`` under the ``[settings.persistent]`` section.

    .. note::

        This class creates or wraps (one of both) Volume Deformable material prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the Volume Deformable material prims.
        * If the prim paths do not exist, Volume Deformable material prims are created at each path
          and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        static_frictions: Static friction coefficients (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        dynamic_frictions: Dynamic friction coefficients (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        youngs_moduli: Young's moduli (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        poissons_ratios: Poisson's ratios (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        densities: Densities (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).

    Raises:
        RuntimeError: If the Deformable feature (beta) is disabled.
        ValueError: If material type is invalid.
        ValueError: If resulting paths are mixed or invalid.
        AssertionError: If the created/wrapped prims are not valid USD Materials.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import VolumeDeformableMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create volume deformable material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = VolumeDeformableMaterial(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        static_frictions: list | np.ndarray | wp.array | None = None,
        dynamic_frictions: list | np.ndarray | wp.array | None = None,
        youngs_moduli: list | np.ndarray | wp.array | None = None,
        poissons_ratios: list | np.ndarray | wp.array | None = None,
        densities: list | np.ndarray | wp.array | None = None,
    ) -> None:
        # check for deformable feature (beta)
        setting_name = physx_bindings.SETTING_ENABLE_DEFORMABLE_BETA
        enabled = carb.settings.get_settings().get(setting_name)
        if not enabled:
            setting_name = (setting_name[1:] if setting_name.startswith("/") else setting_name).replace("/", ".")
            raise RuntimeError(
                "Deformable materials require Deformable feature (beta) to be enabled. "
                f"Enable it in .kit experience settings ('{setting_name} = true') to use them."
            )
        # get or create prims
        self._materials = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = self.resolve_paths(paths)
        if existent_paths:
            paths = existent_paths
            material_types = set()
            for path in existent_paths:
                material = self._get_material(stage, path)
                assert material is not None, f"The wrapped prim at path {path} is not a USD Material"
                self._materials.append(material)
                # get material type
                prim = material.GetPrim()
                # - surface
                if prim_utils.has_api(
                    prim,
                    [
                        "OmniPhysicsBaseMaterialAPI",
                        "OmniPhysicsDeformableMaterialAPI",
                        "OmniPhysicsSurfaceDeformableMaterialAPI",
                    ],
                ):
                    material_types.add("surface")
                # - volume
                elif prim_utils.has_api(prim, ["OmniPhysicsBaseMaterialAPI", "OmniPhysicsDeformableMaterialAPI"]):
                    material_types.add("volume")
                # - unknown
                else:
                    material_types.add("<unknown>")
            # check material type
            material_types = list(material_types)
            if len(material_types) > 1:
                raise ValueError(
                    f"Mixed material detected: {material_types}. Ensure all wrapped prims are of the same material type"
                )
            if material_types[0] != "volume":
                raise ValueError(f"Invalid material type detected. Expected: 'volume', got: '{material_types[0]}'")
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                deformableUtils.add_deformable_material(stage, path)
                material = self._get_material(stage, path)
                assert material is not None, f"The created prim at path {path} is not a USD Material"
                self._materials.append(material)
        # initialize base class
        super().__init__(paths, resolve_paths=False)
        # initialize instance from arguments
        if static_frictions is not None or dynamic_frictions is not None:
            self.set_friction_coefficients(static_frictions, dynamic_frictions)
        if youngs_moduli is not None:
            self.set_youngs_moduli(youngs_moduli)
        if poissons_ratios is not None:
            self.set_poissons_ratios(poissons_ratios)
        if densities is not None:
            self.set_densities(densities)

    """
    Methods.
    """

    def set_friction_coefficients(
        self,
        static_frictions: list | np.ndarray | wp.array = None,
        dynamic_frictions: list | np.ndarray | wp.array = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the friction coefficients of the prims.

        Backends: :guilabel:`usd`.

        Args:
            static_frictions: Static friction coefficients (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            dynamic_frictions: Dynamic friction coefficients (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither static_frictions nor dynamic_frictions are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same friction coefficients (static: 0.5, dynamic: 0.2) for all prims
            >>> prims.set_friction_coefficients(static_frictions=[0.5], dynamic_frictions=[0.2])
            >>>
            >>> # set only the dynamic friction coefficient for the second prim
            >>> prims.set_friction_coefficients(dynamic_frictions=[0.3], indices=[1])
        """
        assert (
            static_frictions is not None or dynamic_frictions is not None
        ), "Both 'static_frictions' and 'dynamic_frictions' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if static_frictions is not None:
            static_frictions = ops_utils.place(static_frictions, device="cpu").numpy().reshape((-1, 1))
        if dynamic_frictions is not None:
            dynamic_frictions = ops_utils.place(dynamic_frictions, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            prim = self.prims[index]
            if static_frictions is not None:
                prim.GetAttribute("omniphysics:staticFriction").Set(
                    static_frictions[0 if static_frictions.shape[0] == 1 else i].item()
                )
            if dynamic_frictions is not None:
                prim.GetAttribute("omniphysics:dynamicFriction").Set(
                    dynamic_frictions[0 if dynamic_frictions.shape[0] == 1 else i].item()
                )

    def get_friction_coefficients(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the friction coefficients of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The static friction coefficients (shape ``(N, 1)``).
            2) The dynamic friction coefficients (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the friction coefficients of all prims
            >>> static_frictions, dynamic_frictions = prims.get_friction_coefficients()
            >>> static_frictions.shape, dynamic_frictions.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the friction coefficients of the first and last prims
            >>> static_frictions, dynamic_frictions = prims.get_friction_coefficients(indices=[0, 2])
            >>> static_frictions.shape, dynamic_frictions.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        static_frictions = np.zeros((indices.shape[0], 1), dtype=np.float32)
        dynamic_frictions = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            prim = self.prims[index]
            static_frictions[i][0] = prim.GetAttribute("omniphysics:staticFriction").Get()
            dynamic_frictions[i][0] = prim.GetAttribute("omniphysics:dynamicFriction").Get()
        return (
            ops_utils.place(static_frictions, device=self._device),
            ops_utils.place(dynamic_frictions, device=self._device),
        )

    def set_youngs_moduli(
        self,
        youngs_moduli: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the Young's modulus of the prims.

        Backends: :guilabel:`usd`.

        Args:
            youngs_moduli: Young's moduli (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same Young's modulus for all prims
            >>> prims.set_youngs_moduli(youngs_moduli=[500000.0])
            >>>
            >>> # set only the Young's modulus for the second prim
            >>> prims.set_youngs_moduli(youngs_moduli=[600000.0], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        youngs_moduli = ops_utils.place(youngs_moduli, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute("omniphysics:youngsModulus").Set(
                youngs_moduli[0 if youngs_moduli.shape[0] == 1 else i].item()
            )

    def get_youngs_moduli(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the Young's moduli of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The Young's moduli (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the Young's moduli of all prims
            >>> moduli = prims.get_youngs_moduli()
            >>> moduli.shape
            (3, 1)
            >>>
            >>> # get the Young's moduli of the first and last prims
            >>> moduli = prims.get_youngs_moduli(indices=[0, 2])
            >>> moduli.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.prims[index].GetAttribute("omniphysics:youngsModulus").Get()
        return ops_utils.place(data, device=self._device)

    def set_poissons_ratios(
        self,
        poissons_ratios: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the Poisson's ratio of the prims.

        Backends: :guilabel:`usd`.

        Args:
            poissons_ratios: Poisson's ratios (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same Poisson's ratio for all prims
            >>> prims.set_poissons_ratios(poissons_ratios=[0.45])
            >>>
            >>> # set only the Poisson's ratio for the second prim
            >>> prims.set_poissons_ratios(poissons_ratios=[0.4], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        poissons_ratios = ops_utils.place(poissons_ratios, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute("omniphysics:poissonsRatio").Set(
                poissons_ratios[0 if poissons_ratios.shape[0] == 1 else i].item()
            )

    def get_poissons_ratios(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the Poisson's ratios of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The Poisson's ratios (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the Poisson's ratios of all prims
            >>> ratios = prims.get_poissons_ratios()
            >>> ratios.shape
            (3, 1)
            >>>
            >>> # get the Poisson's ratios of the first and last prims
            >>> ratios = prims.get_poissons_ratios(indices=[0, 2])
            >>> ratios.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.prims[index].GetAttribute("omniphysics:poissonsRatio").Get()
        return ops_utils.place(data, device=self._device)

    def set_densities(
        self,
        densities: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the densities of the prims.

        Backends: :guilabel:`usd`.

        Args:
            densities: Densities (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same densities for all prims
            >>> prims.set_densities(densities=[1000])
            >>>
            >>> # set only the density for the second prim
            >>> prims.set_densities(densities=[1500], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        densities = ops_utils.place(densities, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            self.prims[index].GetAttribute("omniphysics:density").Set(
                densities[0 if densities.shape[0] == 1 else i].item()
            )

    def get_densities(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the densities of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The densities (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the densities of all prims
            >>> densities = prims.get_densities()
            >>> densities.shape
            (3, 1)
            >>>
            >>> # get the densities of the first and last prims
            >>> densities = prims.get_densities(indices=[0, 2])
            >>> densities.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            data[i][0] = self.prims[index].GetAttribute("omniphysics:density").Get()
        return ops_utils.place(data, device=self._device)

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
            >>> result = VolumeDeformableMaterial.are_of_type(["/World", "/World/prim_0"])
            >>> print(result)
            [[False]
             [ True]]
        """
        data = []
        stage = stage_utils.get_current_stage(backend="usd")
        for item in paths if isinstance(paths, (list, tuple)) else [paths]:
            status = False
            path = item if isinstance(item, str) else item.GetPath()
            material = PhysicsMaterial._get_material(stage, path)
            if material is not None:
                prim = material.GetPrim()
                status = prim_utils.has_api(prim, "OmniPhysicsDeformableMaterialAPI")
                status = status and not prim_utils.has_api(prim, "OmniPhysicsSurfaceDeformableMaterialAPI")
            data.append(status)
        return ops_utils.place(data, device="cpu").reshape((-1, 1))
