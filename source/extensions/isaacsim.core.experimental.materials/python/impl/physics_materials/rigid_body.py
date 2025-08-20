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

from typing import Literal

import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims.impl.prim import _MSG_PRIM_NOT_VALID
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

from .physics_material import PhysicsMaterial


class RigidBodyMaterial(PhysicsMaterial):
    """High level wrapper for creating/encapsulating Rigid Body material prims.

    .. note::

        This class creates or wraps (one of both) Rigid Body material prims according to the following rules:

        * If the prim paths exist, a wrapper is placed over the Rigid Body material prims.
        * If the prim paths do not exist, Rigid Body material prims are created at each path and a wrapper is placed over them.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        static_frictions: Static friction coefficients (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        dynamic_frictions: Dynamic friction coefficients (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        restitutions: Restitution coefficients (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        densities: Densities (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).

    Raises:
        ValueError: If resulting paths are mixed or invalid.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.materials import RigidBodyMaterial
        >>>
        >>> # given an empty USD stage with the /World Xform prim,
        >>> # create rigid body material at paths: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> paths = ["/World/prim_0", "/World/prim_1", "/World/prim_2"]
        >>> prims = RigidBodyMaterial(paths)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        static_frictions: list | np.ndarray | wp.array | None = None,
        dynamic_frictions: list | np.ndarray | wp.array | None = None,
        restitutions: list | np.ndarray | wp.array | None = None,
        densities: list | np.ndarray | wp.array | None = None,
    ) -> None:
        # get or create prims
        self._materials = []
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = self.resolve_paths(paths)
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                material = self._get_material(stage, path)
                assert material is not None, f"The wrapped prim at path {path} is not a USD Material"
                self._materials.append(material)
        else:
            paths = nonexistent_paths
            for path in nonexistent_paths:
                material = UsdShade.Material.Define(stage, path)
                self._materials.append(material)
        # initialize base class
        super().__init__(paths, resolve_paths=False)
        # apply physics material API
        PhysicsMaterial.ensure_api(self.prims, UsdPhysics.MaterialAPI)
        PhysicsMaterial.ensure_api(self.prims, PhysxSchema.PhysxMaterialAPI)
        # initialize instance from arguments
        if static_frictions is not None or dynamic_frictions is not None:
            self.set_friction_coefficients(static_frictions, dynamic_frictions)
        if restitutions is not None:
            self.set_restitution_coefficients(restitutions)
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
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            if static_frictions is not None:
                physics_material_api.GetStaticFrictionAttr().Set(
                    static_frictions[0 if static_frictions.shape[0] == 1 else i].item()
                )
            if dynamic_frictions is not None:
                physics_material_api.GetDynamicFrictionAttr().Set(
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
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            static_frictions[i][0] = physics_material_api.GetStaticFrictionAttr().Get()
            dynamic_frictions[i][0] = physics_material_api.GetDynamicFrictionAttr().Get()
        return (
            ops_utils.place(static_frictions, device=self._device),
            ops_utils.place(dynamic_frictions, device=self._device),
        )

    def set_restitution_coefficients(
        self,
        restitutions: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the restitution coefficients of the prims.

        Backends: :guilabel:`usd`.

        Args:
            restitutions: Restitution coefficients (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same restitution coefficients for all prims
            >>> prims.set_restitution_coefficients(restitutions=[0.005])
            >>>
            >>> # set only the restitution coefficient for the second prim
            >>> prims.set_restitution_coefficients(restitutions=[0.002], indices=[1])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        restitutions = ops_utils.place(restitutions, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            physics_material_api.GetRestitutionAttr().Set(restitutions[0 if restitutions.shape[0] == 1 else i].item())

    def get_restitution_coefficients(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the restitution coefficients of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The restitution coefficients (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the restitution coefficients of all prims
            >>> restitutions = prims.get_restitution_coefficients()
            >>> restitutions.shape
            (3, 1)
            >>>
            >>> # get the restitution coefficients of the first and last prims
            >>> restitutions = prims.get_restitution_coefficients(indices=[0, 2])
            >>> restitutions.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        data = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            data[i][0] = physics_material_api.GetRestitutionAttr().Get()
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
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            physics_material_api.GetDensityAttr().Set(densities[0 if densities.shape[0] == 1 else i].item())

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
            physics_material_api = PhysicsMaterial.ensure_api([self.prims[index]], UsdPhysics.MaterialAPI)[0]
            data[i][0] = physics_material_api.GetDensityAttr().Get()
        return ops_utils.place(data, device=self._device)

    def set_combine_modes(
        self,
        frictions: (
            Literal["average", "max", "min", "multiply"] | list[Literal["average", "max", "min", "multiply"]] | None
        ) = None,
        restitutions: (
            Literal["average", "max", "min", "multiply"] | list[Literal["average", "max", "min", "multiply"]] | None
        ) = None,
        dampings: (
            Literal["average", "max", "min", "multiply"] | list[Literal["average", "max", "min", "multiply"]] | None
        ) = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the way two material properties will be combined to yield a coefficient for a collision.

        Backends: :guilabel:`usd`.

        Args:
            frictions: Friction combine modes (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            restitutions: Restitution combine modes (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            dampings: Damping combine modes (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
                This argument is only relevant for compliant contact interactions.
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither frictions, restitutions nor dampings are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the combine modes (frictions: 'average', restitutions: 'max', dampings: 'min') for all prims
            >>> prims.set_combine_modes(frictions=["average"], restitutions=["max"], dampings=["min"])
        """
        assert (
            frictions is not None or restitutions is not None or dampings is not None
        ), "All 'frictions', 'restitutions' and 'dampings' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if frictions is not None:
            frictions = [frictions] if isinstance(frictions, str) else frictions
            frictions = np.broadcast_to(np.array(frictions, dtype=object), (indices.shape[0],))
        if restitutions is not None:
            restitutions = [restitutions] if isinstance(restitutions, str) else restitutions
            restitutions = np.broadcast_to(np.array(restitutions, dtype=object), (indices.shape[0],))
        if dampings is not None:
            dampings = [dampings] if isinstance(dampings, str) else dampings
            dampings = np.broadcast_to(np.array(dampings, dtype=object), (indices.shape[0],))
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            if frictions is not None:
                physx_material_api.GetFrictionCombineModeAttr().Set(frictions[i])
            if restitutions is not None:
                physx_material_api.GetRestitutionCombineModeAttr().Set(restitutions[i])
            if dampings is not None:
                physx_material_api.GetDampingCombineModeAttr().Set(dampings[i])

    def get_combine_modes(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[
        list[Literal["average", "max", "min", "multiply"]],
        list[Literal["average", "max", "min", "multiply"]],
        list[Literal["average", "max", "min", "multiply"]],
    ]:
        """Get the way two material properties will be combined to yield a coefficient for a collision.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Three-elements tuple. 1) The friction combine modes (shape ``(N,)``).
            2) The restitution combine modes (shape ``(N,)``).
            3) The damping combine modes (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the combine modes of all prims
            >>> frictions, restitutions, dampings = prims.get_combine_modes()
            >>> frictions
            ['average', 'average', 'average']
            >>> restitutions
            ['average', 'average', 'average']
            >>> dampings
            ['average', 'average', 'average']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        frictions = np.empty((indices.shape[0],), dtype=object)
        restitutions = np.empty((indices.shape[0],), dtype=object)
        dampings = np.empty((indices.shape[0],), dtype=object)
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            frictions[i] = physx_material_api.GetFrictionCombineModeAttr().Get()
            restitutions[i] = physx_material_api.GetRestitutionCombineModeAttr().Get()
            dampings[i] = physx_material_api.GetDampingCombineModeAttr().Get()
        return frictions.tolist(), restitutions.tolist(), dampings.tolist()

    def set_enabled_compliant_contacts(
        self,
        enabled: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enable or disable the compliant spring-damper contact effects of the prims.

        Backends: :guilabel:`usd`.

        .. warning::

            The spring stiffnesses (see :py:meth:`set_compliant_contact_gains`) must be set to a value greater than 0
            before enabling compliant contact effects.

        Args:
            enabled: Boolean flags to enable/disable compliant contact effects (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the compliant contact effects for all prims
            >>> prims.set_enabled_compliant_contacts([True])
            >>>
            >>> # disable the compliant contact effects for the first and last prims
            >>> prims.set_enabled_compliant_contacts([False], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = ops_utils.place(enabled, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            physx_material_api.GetCompliantContactAccelerationSpringAttr().Set(
                bool(enabled[0 if enabled.shape[0] == 1 else i].item())
            )

    def get_enabled_compliant_contacts(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the enabled state of the compliant spring-damper contact effects of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if the compliant contact effects are enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the compliant contact enabled state of all prims after enabling it for the second prim
            >>> prims.set_enabled_compliant_contacts([True], indices=[1])
            >>> print(prims.get_enabled_compliant_contacts())
            [[False]
             [ True]
             [False]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = np.zeros((indices.shape[0], 1), dtype=np.bool_)
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            enabled[i] = physx_material_api.GetCompliantContactAccelerationSpringAttr().Get()
        return ops_utils.place(enabled, device=self._device)

    def set_compliant_contact_gains(
        self,
        stiffnesses: list | np.ndarray | wp.array | None = None,
        dampings: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the compliant contact gains (stiffnesses and dampings) of the prims.

        Backends: :guilabel:`usd`.

        .. warning::

            The spring stiffnesses must be set to a value greater than 0 before enabling compliant contact effects.

        Args:
            stiffnesses: Stiffnesses (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            dampings: Dampings (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither stiffnesses nor dampings are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same compliant contact gains (stiffnesses: 100.0, dampings: 10.0) for all prims
            >>> prims.set_compliant_contact_gains(stiffnesses=[100.0], dampings=[10.0])
            >>>
            >>> # set only the damping for the second prim
            >>> prims.set_compliant_contact_gains(dampings=[15.0], indices=[1])
        """
        assert (
            stiffnesses is not None or dampings is not None
        ), "Both 'stiffnesses' and 'dampings' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if stiffnesses is not None:
            stiffnesses = ops_utils.place(stiffnesses, device="cpu").numpy().reshape((-1, 1))
        if dampings is not None:
            dampings = ops_utils.place(dampings, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            if stiffnesses is not None:
                physx_material_api.GetCompliantContactStiffnessAttr().Set(
                    stiffnesses[0 if stiffnesses.shape[0] == 1 else i].item()
                )
            if dampings is not None:
                physx_material_api.GetCompliantContactDampingAttr().Set(
                    dampings[0 if dampings.shape[0] == 1 else i].item()
                )

    def get_compliant_contact_gains(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the compliant contact gains (stiffnesses and dampings) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The stiffnesses (shape ``(N, 1)``).
            2) The dampings (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the compliant contact gains of all prims
            >>> stiffnesses, dampings = prims.get_compliant_contact_gains()
            >>> stiffnesses.shape, dampings.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the compliant contact gains of the first and last prims
            >>> stiffnesses, dampings = prims.get_compliant_contact_gains(indices=[0, 2])
            >>> stiffnesses.shape, dampings.shape
            ((2, 1), (2, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        stiffnesses = np.zeros((indices.shape[0], 1), dtype=np.float32)
        dampings = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            physx_material_api = PhysicsMaterial.ensure_api([self.prims[index]], PhysxSchema.PhysxMaterialAPI)[0]
            stiffnesses[i][0] = physx_material_api.GetCompliantContactStiffnessAttr().Get()
            dampings[i][0] = physx_material_api.GetCompliantContactDampingAttr().Get()
        return ops_utils.place(stiffnesses, device=self._device), ops_utils.place(dampings, device=self._device)

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
            >>> result = RigidBodyMaterial.are_of_type(["/World", "/World/prim_0"])
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
                status = prim_utils.has_api(material.GetPrim(), ["PhysicsMaterialAPI", "PhysxMaterialAPI"], test="any")
            data.append(status)
        return ops_utils.place(data, device="cpu").reshape((-1, 1))
