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
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.utils.numpy as numpy_utils
import numpy as np
import usdrt
import usdrt.Gf
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.prims import is_prim_non_root_articulation_link
from pxr import Gf, Usd, UsdGeom, UsdShade

from . import _fabric
from .prim import _MSG_PRIM_NOT_VALID, Prim


class XformPrim(Prim):
    """High level wrapper for manipulating ``Xform`` prims and their attributes.

    This class is a wrapper over one or more USD ``Xform`` prims in the stage to provide
    high-level functionality for manipulating transformations and other attributes.
    The prims are specified using paths that can include regular expressions for matching multiple prims.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        resolve_paths: Whether to resolve the given paths (true) or use them as is (false).
        positions: Positions in the world frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        translations: Translations in the local frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        scales: Scales to be applied to the prims (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims to a standard set.
            See :py:meth:`reset_xform_op_properties` for more details.

    Raises:
        ValueError: If no prims are found matching the specified path(s).
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.core.experimental.prims import XformPrim
        >>>
        >>> # given a USD stage with the Xform prims: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> # - create wrapper over single prim
        >>> prim = XformPrim("/World/prim_0")  # doctest: +NO_CHECK
        >>> # - create wrapper over multiple prims using regex
        >>> prims = XformPrim("/World/prim_.*")  # doctest: +NO_CHECK
        >>> prims.reset_xform_op_properties()
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Prim
        resolve_paths: bool = True,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = False,
    ) -> None:
        # define properties
        # - default state properties
        self._default_positions = None
        self._default_orientations = None
        # - fabric properties
        self._fabric_data = {}
        self._fabric_stage = None
        self._fabric_hierarchy = None
        self._fabric_view_index_attr = None
        # initialize base class
        super().__init__(paths, resolve_paths=resolve_paths)
        # initialize instance from arguments
        self._non_root_articulation_link = is_prim_non_root_articulation_link(prim_path=self.paths[0])
        # - reset xformOp properties
        if not self._non_root_articulation_link:
            if reset_xform_op_properties:
                self.reset_xform_op_properties()
        # - set specified values
        if positions is not None or translations is not None or orientations is not None or scales is not None:
            assert (
                positions is None or translations is None
            ), "Both 'positions' and 'translations' are specified. Specifie only one of them"
            if self._non_root_articulation_link:
                raise carb.log_warn(
                    (
                        "The prim is a non-root link in an articulation. "
                        "Specified values (positions, translations, orientations and/or scales) will not be set"
                    )
                )
            else:
                if positions is not None or orientations is not None:
                    self.set_world_poses(positions, orientations)
                if translations is not None or orientations is not None:
                    self.set_local_poses(translations, orientations)
                if scales is not None:
                    self.set_local_scales(scales)
        # setup physics-related configuration if simulation is running
        if SimulationManager._physics_sim_view__warp is not None:
            self._on_physics_ready(None)

    """
    Properties.
    """

    @property
    def is_non_root_articulation_link(self) -> bool:
        """Indicate if the wrapped prims are a non-root link in an articulation tree.

        Backends: :guilabel:`usd`.

        .. warning::

            Transformation of the poses of non-root links in an articulation tree are not supported.

        Returns:
            Whether the prims are a non-root link in an articulation tree.
        """
        return self._non_root_articulation_link

    """
    Methods.
    """

    def set_visibilities(
        self,
        visibilities: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the visibility state (whether prims are visible or invisible during rendering) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            visibilities: Boolean flags to set the visibility state (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # make all prims invisible
            >>> prims.set_visibilities([False])
            >>>
            >>> # make first and last prims invisible
            >>> prims.set_visibilities([True])  # restore visibility from previous call
            >>> prims.set_visibilities([False], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        visibilities = ops_utils.place(visibilities, device="cpu").numpy().reshape((-1, 1))
        broadcast = visibilities.shape[0] == 1
        for i, index in enumerate(indices.numpy()):
            imageable = UsdGeom.Imageable(self.prims[index])
            imageable.MakeVisible() if visibilities[0 if broadcast else i].item() else imageable.MakeInvisible()

    def get_visibilities(self, *, indices: list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the visibility state (whether prims are visible or invisible during rendering) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating the visibility state (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the visibility states of all prims
            >>> visibilities = prims.get_visibilities()
            >>> visibilities.list()
            [True, True, True]
            >>>
            >>> # get the visibility states of the first and last prims
            >>> visibilities = prims.get_visibilities(indices=[0, 2])
            >>> visibilities.list()
            [True, True]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        visibilities = np.zeros((indices.shape[0], 1), dtype="bool")
        for i, index in enumerate(indices.numpy()):
            visibilities[i, 0] = (
                UsdGeom.Imageable(self.prims[index]).ComputeVisibility(Usd.TimeCode.Default())
                != UsdGeom.Tokens.invisible
            )
        return ops_utils.place(visibilities, device=self._device)

    def get_default_state(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array | None, wp.array | None]:
        """Get the default state (positions and orientations) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The default positions in the world frame (shape ``(N, 3)``).
            2) The default orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            If the default state is not set using the :py:meth:`set_default_state` method, ``None`` is returned.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: If prims are non-root articulation links.
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert (
            not self._non_root_articulation_link
        ), "This view corresponds to non root links that are included in an articulation"
        # USD API
        if indices is not None:
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
        default_positions = self._default_positions
        if default_positions is not None and indices is not None:
            default_positions = default_positions[indices].contiguous()
        default_orientations = self._default_orientations
        if default_orientations is not None and indices is not None:
            default_orientations = default_orientations[indices].contiguous()
        return default_positions, default_orientations

    def set_default_state(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the default state (positions and orientations) of the prims.

        Backends: :guilabel:`usd`.

        .. hint::

            Prims can be reset to their default state by calling the :py:meth:`reset_to_default_state` method.

        Args:
            positions: Default positions in the world frame (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            orientations: Default orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither positions nor orientations are specified.
            AssertionError: Wrapped prims are not valid.
            AssertionError: If prims are non-root articulation links.
        """
        assert (
            positions is not None or orientations is not None
        ), "Both 'positions' and 'orientations' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert (
            not self._non_root_articulation_link
        ), "This view corresponds to non root links that are included in an articulation"
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
        if positions is not None:
            if self._default_positions is None:
                self._default_positions = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            positions = ops_utils.broadcast_to(
                positions, shape=(indices.shape[0], 3), dtype=wp.float32, device=self._device
            )
            wp.copy(self._default_positions[indices], positions)
        if orientations is not None:
            if self._default_orientations is None:
                default_orientations = np.zeros((len(self), 4), dtype=np.float32)
                default_orientations[:, 0] = 1.0
                self._default_orientations = wp.array(default_orientations, device=self._device)
            orientations = ops_utils.broadcast_to(
                orientations, shape=(indices.shape[0], 4), dtype=wp.float32, device=self._device
            )
            wp.copy(self._default_orientations[indices], orientations)

    def apply_visual_materials(
        self,
        materials: type["VisualMaterial"] | list[type["VisualMaterial"]],
        *,
        weaker_than_descendants: list | np.ndarray | wp.array | None = None,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Apply visual materials to the prims, and optionally, to their descendants.

        Backends: :guilabel:`usd`.

        Args:
            visual_materials: Visual materials to be applied to the prims (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            weaker_than_descendants: Boolean flags to indicate whether descendant materials should be overridden (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import OmniGlassMaterial
            >>>
            >>> # create a dark-red glass visual material
            >>> material = OmniGlassMaterial("/World/material/glass")
            >>> material.set_input_values("glass_ior", [1.25])
            >>> material.set_input_values("depth", [0.001])
            >>> material.set_input_values("thin_walled", [False])
            >>> material.set_input_values("glass_color", [0.5, 0.0, 0.0])
            >>>
            >>> prims.apply_visual_materials(material)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        # accommodate values and determine broadcast status
        if not isinstance(materials, (list, tuple)):
            materials = [materials]
        broadcast_materials = len(materials) == 1
        if weaker_than_descendants is None:
            weaker_than_descendants = [False]
        weaker_than_descendants = ops_utils.place(weaker_than_descendants, device="cpu").numpy().reshape((-1, 1))
        broadcast_weaker_than_descendants = weaker_than_descendants.shape[0] == 1
        # set values
        for i, index in enumerate(indices.numpy()):
            material_binding_api = XformPrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
            binding_strength = (
                UsdShade.Tokens.weakerThanDescendants
                if weaker_than_descendants[0 if broadcast_weaker_than_descendants else i].item()
                else UsdShade.Tokens.strongerThanDescendants
            )
            material_binding_api.Bind(
                materials[0 if broadcast_materials else i].materials[0], bindingStrength=binding_strength
            )

    def get_applied_visual_materials(
        self, *, indices: list | np.ndarray | wp.array | None = None
    ) -> list[type["VisualMaterial"] | None]:
        """Get the applied visual materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of applied visual materials (shape ``(N,)``). If a prim does not have a material, ``None`` is returned.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the applied visual material of the last wrapped prim
            >>> prims.get_applied_visual_materials(indices=[2])[0]
            <isaacsim.core.experimental.materials.impl.visual_materials.omni_glass.OmniGlassMaterial object at 0x...>
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        from isaacsim.core.experimental.materials import VisualMaterial  # defer imports to avoid circular dependencies

        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        materials = []
        for index in indices.numpy():
            material_binding_api = XformPrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
            material_path = str(material_binding_api.GetDirectBinding().GetMaterialPath())
            material = None
            if material_path:
                material = VisualMaterial.fetch_instances([material_path])[0]
                if material is None:
                    carb.log_warn(f"Unsupported visual material ({material_path}): {self.paths[index]}")
            materials.append(material)
        return materials

    def get_world_poses(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the poses (positions and orientations) in the world frame of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The positions in the world frame (shape ``(N, 3)``).
            2) The orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the world poses of all prims
            >>> positions, orientations = prims.get_world_poses()
            >>> positions.shape, orientations.shape
            ((3, 3), (3, 4))
            >>>
            >>> # get the world pose of the first prim
            >>> positions, orientations = prims.get_world_poses(indices=[0])
            >>> positions.shape, orientations.shape
            ((1, 3), (1, 4))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            positions = np.zeros((indices.shape[0], 3), dtype=np.float32)
            orientations = np.zeros((indices.shape[0], 4), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                transform = UsdGeom.Xformable(self.prims[index]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                transform.Orthonormalize()
                positions[i] = transform.ExtractTranslation()
                orientations[i] = np.array(
                    [transform.ExtractRotationQuat().GetReal(), *transform.ExtractRotationQuat().GetImaginary()]
                )
            return ops_utils.place(positions, device=self._device), ops_utils.place(orientations, device=self._device)
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            positions = np.zeros((indices.shape[0], 3), dtype=np.float32)
            orientations = np.zeros((indices.shape[0], 4), dtype=np.float32)
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                matrix = fabric_hierarchy.get_world_xform(usdrt.Sdf.Path(self.paths[index]))
                quaternion = matrix.RemoveScaleShear().ExtractRotationQuat()
                positions[i] = matrix.ExtractTranslation()
                orientations[i] = np.array([quaternion.GetReal(), *quaternion.GetImaginary()])
            return ops_utils.place(positions, device=self._device), ops_utils.place(orientations, device=self._device)
        # Fabric API
        elif backend == "fabric":
            self._get_fabric_hierarchy().update_world_xforms()
            # ensure fabric data and update selection if needed
            fabric_data = self._ensure_fabric_data("world-matrix")
            if not _fabric.update_fabric_selection(
                stage=self._fabric_stage,
                data=fabric_data,
                device=self._device,
                attr=self._fabric_view_index_attr,
                count=len(self),
            ):
                # TODO: should fall back to other backend???
                carb.log_error(f"Failed to update fabric selection for {fabric_data['attr']}")
                return
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            positions = fabric_data["cache"]["positions"]
            orientations = fabric_data["cache"]["orientations"]
            wp.launch(
                _fabric.wk_decompose_fabric_transformation_matrix_to_warp_arrays,
                dim=(indices.shape[0]),
                inputs=[
                    wp.fabricarray(fabric_data["selection"], fabric_data["attr"]),
                    positions,
                    orientations,
                    None,
                    indices,
                    fabric_data["mapping"],
                ],
                device=self._device,
            )
            return positions[indices].contiguous(), orientations[indices].contiguous()

    def set_world_poses(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the poses (positions and orientations) in the world frame of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        .. note::

            This method *teleports* prims to the specified poses.

        Args:
            positions: Positions in the world frame (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither positions nor orientations are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set random poses for all prims
            >>> positions = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> orientations = np.random.randn(3, 4)
            >>> orientations = orientations / np.linalg.norm(orientations, axis=-1, keepdims=True)  # normalize quaternions
            >>> prims.set_world_poses(positions, orientations)
        """
        assert (
            positions is not None or orientations is not None
        ), "Both 'positions' and 'orientations' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # there is no Fabric implementation (world-matrix is read-only), fall back to USDRT
        if backend == "fabric":
            backend = "usdrt"
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            # get position or orientation if undefined
            if positions is None or orientations is None:
                current_positions, current_orientations = self.get_world_poses(indices=indices)
                positions = current_positions if positions is None else positions
                orientations = current_orientations if orientations is None else orientations
            # accommodate and broadcast the pose
            positions = ops_utils.place(positions, device="cpu").numpy().reshape((-1, 3))
            if positions.shape[0] == 1:
                positions = positions.repeat(indices.shape[0], axis=0)
            orientations = ops_utils.place(orientations, device="cpu").numpy().reshape((-1, 4))
            if orientations.shape[0] == 1:
                orientations = orientations.repeat(indices.shape[0], axis=0)
            # compute parent transform
            parent_transforms = np.zeros((indices.shape[0], 4, 4), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                parent_transforms[i] = np.array(
                    UsdGeom.Xformable(self.prims[index].GetParent()).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype=np.float32,
                )
            # get and apply local transformation
            local_translations, local_orientations = numpy_utils.transformations.get_local_from_world(
                parent_transforms, positions, orientations
            )
            self.set_local_poses(translations=local_translations, orientations=local_orientations, indices=indices)
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            if positions is not None:
                positions = ops_utils.place(positions, device="cpu").numpy().reshape((-1, 3))
                broadcast_positions = positions.shape[0] == 1
            if orientations is not None:
                orientations = ops_utils.place(orientations, device="cpu").numpy().reshape((-1, 4))
                broadcast_orientations = orientations.shape[0] == 1
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                path = usdrt.Sdf.Path(self.paths[index])
                matrix = fabric_hierarchy.get_world_xform(path)
                if positions is not None:
                    matrix.SetTranslateOnly(usdrt.Gf.Vec3d(*positions[0 if broadcast_positions else i]))
                if orientations is not None:
                    matrix.SetRotateOnly(usdrt.Gf.Quatd(*orientations[0 if broadcast_orientations else i]))
                    scaling_matrix = usdrt.Gf.Matrix4d().SetIdentity().SetScale(usdrt.Gf.Transform(matrix).GetScale())
                    matrix = scaling_matrix * matrix
                fabric_hierarchy.set_world_xform(path, matrix)

    def get_local_poses(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the poses (translations and orientations) in the local frame of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The translations in the local frame (shape ``(N, 3)``).
            2) The orientations in the local frame (shape ``(N, 4)``, quaternion ``wxyz``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the local poses of all prims
            >>> translations, orientations = prims.get_local_poses()
            >>> translations.shape, orientations.shape
            ((3, 3), (3, 4))
            >>>
            >>> # get the local pose of the first prim
            >>> translations, orientations = prims.get_local_poses(indices=[0])
            >>> translations.shape, orientations.shape
            ((1, 3), (1, 4))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            translations = np.zeros((indices.shape[0], 3), dtype=np.float32)
            orientations = np.zeros((indices.shape[0], 4), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                transform = UsdGeom.Xformable(self.prims[index]).GetLocalTransformation(Usd.TimeCode.Default())
                transform.Orthonormalize()
                translations[i] = transform.ExtractTranslation()
                orientations[i] = np.array(
                    [transform.ExtractRotationQuat().GetReal(), *transform.ExtractRotationQuat().GetImaginary()]
                )
            return (
                ops_utils.place(translations, device=self._device),
                ops_utils.place(orientations, device=self._device),
            )
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            translations = np.zeros((indices.shape[0], 3), dtype=np.float32)
            orientations = np.zeros((indices.shape[0], 4), dtype=np.float32)
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                matrix = fabric_hierarchy.get_local_xform(usdrt.Sdf.Path(self.paths[index]))
                quaternion = matrix.RemoveScaleShear().ExtractRotationQuat()
                translations[i] = matrix.ExtractTranslation()
                orientations[i] = np.array([quaternion.GetReal(), *quaternion.GetImaginary()])
            return (
                ops_utils.place(translations, device=self._device),
                ops_utils.place(orientations, device=self._device),
            )
        # Fabric API
        elif backend == "fabric":
            # ensure fabric data and update selection if needed
            fabric_data = self._ensure_fabric_data("local-matrix")
            if not _fabric.update_fabric_selection(
                stage=self._fabric_stage,
                data=fabric_data,
                device=self._device,
                attr=self._fabric_view_index_attr,
                count=len(self),
            ):
                # TODO: should fall back to other backend???
                carb.log_error(f"Failed to update fabric selection for {fabric_data['attr']}")
                return
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            translations = fabric_data["cache"]["translations"]
            orientations = fabric_data["cache"]["orientations"]
            wp.launch(
                _fabric.wk_decompose_fabric_transformation_matrix_to_warp_arrays,
                dim=(indices.shape[0]),
                inputs=[
                    wp.fabricarray(fabric_data["selection"], fabric_data["attr"]),
                    translations,
                    orientations,
                    None,
                    indices,
                    fabric_data["mapping"],
                ],
                device=self._device,
            )
            return translations[indices].contiguous(), orientations[indices].contiguous()

    def set_local_poses(
        self,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the poses (translations and orientations) in the local frame of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        .. note::

            This method *teleports* prims to the specified poses.

        Args:
            translations: Translations in the local frame (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            orientations: Orientations in the local frame (shape ``(N, 4)``, quaternion ``wxyz``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither translations nor orientations are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set random poses for all prims
            >>> translations = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> orientations = np.random.randn(3, 4)
            >>> orientations = orientations / np.linalg.norm(orientations, axis=-1, keepdims=True)  # normalize quaternions
            >>> prims.set_local_poses(translations, orientations)
        """
        assert (
            translations is not None or orientations is not None
        ), "Both 'translations' and 'orientations' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            # accommodate and broadcast the pose
            if translations is not None:
                translations = ops_utils.place(translations, device="cpu").numpy().reshape((-1, 3))
                if translations.shape[0] == 1:
                    translations = translations.repeat(indices.shape[0], axis=0)
                translations = translations.tolist()
            if orientations is not None:
                orientations = ops_utils.place(orientations, device="cpu").numpy().reshape((-1, 4))
                if orientations.shape[0] == 1:
                    orientations = orientations.repeat(indices.shape[0], axis=0)
                orientations = orientations.tolist()
            # apply transformation
            for i, index in enumerate(indices.numpy()):
                prim = self.prims[index]
                property_names = prim.GetPropertyNames()
                if translations is not None:
                    assert (
                        "xformOp:translate" in property_names
                    ), f"Undefined 'xformOp:translate' property for {self.paths[index]}. Call '.reset_xform_op_properties()' first"
                    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*translations[i]))
                if orientations is not None:
                    assert (
                        "xformOp:orient" in property_names
                    ), f"Undefined 'xformOp:orient' property for {self.paths[index]}. Call '.reset_xform_op_properties()' first"
                    xform_op = prim.GetAttribute("xformOp:orient")
                    xform_op.Set((Gf.Quatf if xform_op.GetTypeName() == "quatf" else Gf.Quatd)(*orientations[i]))
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            # accommodate and broadcast the pose
            if translations is not None:
                translations = ops_utils.place(translations, device="cpu").numpy().reshape((-1, 3))
                broadcast_translations = translations.shape[0] == 1
            if orientations is not None:
                orientations = ops_utils.place(orientations, device="cpu").numpy().reshape((-1, 4))
                broadcast_orientations = orientations.shape[0] == 1
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                path = usdrt.Sdf.Path(self.paths[index])
                matrix = fabric_hierarchy.get_local_xform(path)
                if translations is not None:
                    matrix.SetTranslateOnly(usdrt.Gf.Vec3d(*translations[0 if broadcast_translations else i]))
                if orientations is not None:
                    matrix.SetRotateOnly(usdrt.Gf.Quatd(*orientations[0 if broadcast_orientations else i]))
                    scaling_matrix = usdrt.Gf.Matrix4d().SetIdentity().SetScale(usdrt.Gf.Transform(matrix).GetScale())
                    matrix = scaling_matrix * matrix
                fabric_hierarchy.set_local_xform(path, matrix)
        # Fabric API
        elif backend == "fabric":
            # ensure fabric data and update selection if needed
            fabric_data = self._ensure_fabric_data("local-matrix")
            if not _fabric.update_fabric_selection(
                stage=self._fabric_stage,
                data=fabric_data,
                device=self._device,
                attr=self._fabric_view_index_attr,
                count=len(self),
            ):
                # TODO: should fall back to other backend???
                carb.log_error(f"Failed to update fabric selection for {fabric_data['attr']}")
                return
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            # accommodate and broadcast the pose
            if translations is not None:
                translations = ops_utils.place(translations, dtype=wp.float32, device=self._device).reshape((-1, 3))
            if orientations is not None:
                orientations = ops_utils.place(orientations, dtype=wp.float32, device=self._device).reshape((-1, 4))
            wp.launch(
                _fabric.wk_compose_fabric_transformation_matrix_from_warp_arrays,
                dim=(indices.shape[0]),
                inputs=[
                    wp.fabricarray(fabric_data["selection"], fabric_data["attr"]),
                    translations,
                    orientations,
                    None,
                    translations.shape[0] == 1,
                    orientations.shape[0] == 1,
                    False,
                    indices,
                    fabric_data["mapping"],
                ],
                device=self._device,
            )

    def set_local_scales(
        self,
        scales: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the local scales of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        Args:
            scales: Scales to be applied to the prims (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set random positive scales for all prims
            >>> scales = np.random.uniform(low=0.5, high=1.5, size=(3, 3))
            >>> prims.set_local_scales(scales)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            scales = ops_utils.place(scales, device="cpu").numpy().reshape((-1, 3))
            broadcast = scales.shape[0] == 1
            scales = scales.tolist()
            for i, index in enumerate(indices.numpy()):
                prim = self.prims[index]
                property_names = prim.GetPropertyNames()
                assert (
                    "xformOp:scale" in property_names
                ), f"Undefined 'xformOp:scale' property for {self.paths[index]}. Call '.reset_xform_op_properties()' first"
                prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(*scales[0 if broadcast else i]))
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            scales = ops_utils.place(scales, device="cpu").numpy().reshape((-1, 3))
            broadcast = scales.shape[0] == 1
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                path = usdrt.Sdf.Path(self.paths[index])
                transform = usdrt.Gf.Transform(fabric_hierarchy.get_local_xform(path))
                transform.SetScale(usdrt.Gf.Vec3d(*scales[0 if broadcast else i]))
                fabric_hierarchy.set_local_xform(path, transform.GetMatrix())
        # Fabric API
        elif backend == "fabric":
            # ensure fabric data and update selection if needed
            fabric_data = self._ensure_fabric_data("local-matrix")
            if not _fabric.update_fabric_selection(
                stage=self._fabric_stage,
                data=fabric_data,
                device=self._device,
                attr=self._fabric_view_index_attr,
                count=len(self),
            ):
                # TODO: should fall back to other backend???
                carb.log_error(f"Failed to update fabric selection for {fabric_data['attr']}")
                return
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            scales = ops_utils.place(scales, dtype=wp.float32, device=self._device).reshape((-1, 3))
            wp.launch(
                _fabric.wk_compose_fabric_transformation_matrix_from_warp_arrays,
                dim=(indices.shape[0]),
                inputs=[
                    wp.fabricarray(fabric_data["selection"], fabric_data["attr"]),
                    None,
                    None,
                    scales,
                    False,
                    False,
                    scales.shape[0] == 1,
                    indices,
                    fabric_data["mapping"],
                ],
                device=self._device,
            )

    def get_local_scales(self, *, indices: list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the local scales of the prims.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Scales of the prims (shape ``(N, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get local scales of all prims
            >>> scales = prims.get_local_scales()
            >>> scales.shape
            (3, 3)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
        # USD API
        if backend == "usd":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            scales = np.zeros((indices.shape[0], 3), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                prim = self.prims[index]
                property_names = prim.GetPropertyNames()
                assert (
                    "xformOp:scale" in property_names
                ), f"Undefined 'xformOp:scale' property for {self.paths[index]}. Call '.reset_xform_op_properties()' first"
                scales[i] = np.array(prim.GetAttribute("xformOp:scale").Get(), dtype=np.float32)
            return ops_utils.place(scales, device=self._device)
        # USDRT API (with FSD and IFabricHierarchy)
        elif backend == "usdrt":
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            scales = np.zeros((indices.shape[0], 3), dtype=np.float32)
            fabric_hierarchy = self._get_fabric_hierarchy()
            for i, index in enumerate(indices.numpy()):
                transform = usdrt.Gf.Transform(fabric_hierarchy.get_local_xform(usdrt.Sdf.Path(self.paths[index])))
                scales[i] = transform.GetScale()
            return ops_utils.place(scales, device=self._device)
        # Fabric API
        elif backend == "fabric":
            # ensure fabric data and update selection if needed
            fabric_data = self._ensure_fabric_data("local-matrix")
            if not _fabric.update_fabric_selection(
                stage=self._fabric_stage,
                data=fabric_data,
                device=self._device,
                attr=self._fabric_view_index_attr,
                count=len(self),
            ):
                # TODO: should fall back to other backend???
                carb.log_error(f"Failed to update fabric selection for {fabric_data['attr']}")
                return
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            scales = fabric_data["cache"]["scales"]
            wp.launch(
                _fabric.wk_decompose_fabric_transformation_matrix_to_warp_arrays,
                dim=(indices.shape[0]),
                inputs=[
                    wp.fabricarray(fabric_data["selection"], fabric_data["attr"]),
                    None,
                    None,
                    scales,
                    indices,
                    fabric_data["mapping"],
                ],
                device=self._device,
            )
            return scales[indices].contiguous()

    def reset_xform_op_properties(self) -> None:
        """Reset the transformation operation attributes of the prims to a standard set.

        Backends: :guilabel:`usd`.

        It ensures that each prim has only the following transformations in the specified order.
        Any other transformation operations are removed, so they are not consumed.

        1. ``xformOp:translate`` (double precision)
        2. ``xformOp:orient`` (double precision)
        3. ``xformOp:scale`` (double precision)

        .. note::

            This method preserves the poses of the prims in the world frame while reorganizing the transformation operations.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # reset transform operations of all prims
            >>> prims.reset_xform_op_properties()
            >>>
            >>> # verify transform operations of the first wrapped prim
            >>> prims.prims[0].GetPropertyNames()
            [... 'xformOp:orient', 'xformOp:scale', 'xformOp:translate', 'xformOpOrder']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        properties_to_remove = [
            "xformOp:rotateX",
            "xformOp:rotateXZY",
            "xformOp:rotateY",
            "xformOp:rotateYXZ",
            "xformOp:rotateYZX",
            "xformOp:rotateZ",
            "xformOp:rotateZYX",
            "xformOp:rotateZXY",
            "xformOp:rotateXYZ",
            "xformOp:transform",
        ]
        for prim in self.prims:
            property_names = prim.GetPropertyNames()
            xformable = UsdGeom.Xformable(prim)
            # remove specified properties
            for property_name in properties_to_remove:
                if property_name in property_names:
                    prim.RemoveProperty(property_name)
            # get/add xformOp
            # - xformOp:translate
            if "xformOp:translate" in property_names:
                xform_op_translate = UsdGeom.XformOp(prim.GetAttribute("xformOp:translate"))
            else:
                xform_op_translate = xformable.AddXformOp(
                    UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
                )
            # - xformOp:orient
            if "xformOp:orient" in property_names:
                xform_op_orient = UsdGeom.XformOp(prim.GetAttribute("xformOp:orient"))
            else:
                xform_op_orient = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
            # - xformOp:scale
            if "xformOp:scale" in property_names:
                xform_op_scale = UsdGeom.XformOp(prim.GetAttribute("xformOp:scale"))
                if "xformOp:scale:unitsResolve" in property_names:
                    scale = np.array(prim.GetAttribute("xformOp:scale").Get()) * np.array(
                        prim.GetAttribute("xformOp:scale:unitsResolve").Get()
                    )
                    prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(*scale.tolist()))
                    prim.RemoveProperty("xformOp:scale:unitsResolve")
            else:
                xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
                xform_op_scale.Set(Gf.Vec3d(1.0, 1.0, 1.0))
            # reset operations order
            xformable.ClearXformOpOrder()
            xformable.SetXformOpOrder([xform_op_translate, xform_op_orient, xform_op_scale])
        # set pose
        positions, orientations = self.get_world_poses()
        self.set_world_poses(positions=positions, orientations=orientations)

    def reset_to_default_state(self, *, warn_on_non_default_state: bool = False) -> None:
        """Reset the prims to the specified default state.

        Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

        This method applies the default state defined using the :py:meth:`set_default_state` method.

        .. note::

            This method *teleports* prims to the specified default state (positions and orientations).

        .. warning::

            This method has no effect on non-root articulation links or when no default state is set.
            In this case, a warning message is logged if ``warn_on_non_default_state`` is ``True``.

        Args:
            warn_on_non_default_state: Whether to log a warning message when no default state is set.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get default state (no default state set at this point)
            >>> prims.get_default_state()
            (None, None)
            >>>
            >>> # set default state
            >>> # - random positions for each prim
            >>> # - same fixed orientation for all prim
            >>> positions = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> prims.set_default_state(positions, orientations=[1.0, 0.0, 0.0, 0.0])
            >>>
            >>> # get default state (default state is set)
            >>> prims.get_default_state()
            (array(shape=(3, 3), dtype=float32), array(shape=(3, 4), dtype=float32))
            >>>
            >>> # reset prims to default state
            >>> prims.reset_to_default_state()
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        if self._non_root_articulation_link:
            carb.log_warn("The prim is a non-root link in an articulation. The default state will not be set")
            return
        if self._default_positions is not None or self._default_orientations is not None:
            self.set_world_poses(self._default_positions, self._default_orientations)
        else:
            if warn_on_non_default_state:
                carb.log_warn(
                    "No default positions or orientations to reset. Call '.set_default_state(..)' first to initialize them"
                )

    """
    Internal methods.
    """

    def _get_fabric_hierarchy(self) -> usdrt.hierarchy.IFabricHierarchy:
        """Get the IFabricHierarchy interface."""
        if self._fabric_hierarchy is None:
            self._fabric_stage = stage_utils.get_current_stage(backend="fabric")
            self._fabric_hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
                self._fabric_stage.GetFabricId(), self._fabric_stage.GetStageIdAsStageId()
            )
        return self._fabric_hierarchy

    def _ensure_fabric_data(self, key: str) -> dict:
        """Ensure fabric-related data is initialized."""
        if self._fabric_view_index_attr is None:
            self._fabric_stage = stage_utils.get_current_stage(backend="fabric")
            # create fabric's view indices attribute
            self._fabric_view_index_attr = f"isaacsim:fabric:index:{hash(self)}"
            fabric_prims = [self._fabric_stage.GetPrimAtPath(path) for path in self._paths]
            for i, prim in enumerate(fabric_prims):
                prim.CreateAttribute(self._fabric_view_index_attr, usdrt.Sdf.ValueTypeNames.UInt, True)
                prim.GetAttribute(self._fabric_view_index_attr).Set(i)
        if key not in self._fabric_data:
            # create fabric data
            if key == "world-matrix":
                self._fabric_data[key] = {
                    "selection": None,
                    "attr": "omni:fabric:worldMatrix",  # computed, cached, and read-only world transform of any prim
                    "spec": ((usdrt.Sdf.ValueTypeNames.Matrix4d, "omni:fabric:worldMatrix", usdrt.Usd.Access.Read),),
                    "cache": {
                        "positions": wp.zeros((len(self), 3), dtype=wp.float32, device=self._device),
                        "orientations": wp.zeros((len(self), 4), dtype=wp.float32, device=self._device),
                        "scales": wp.zeros((len(self), 3), dtype=wp.float32, device=self._device),
                    },
                }
            elif key == "local-matrix":
                self._fabric_data[key] = {
                    "selection": None,
                    "attr": "omni:fabric:localMatrix",  # editable local transform of any prim
                    "spec": (
                        (usdrt.Sdf.ValueTypeNames.Matrix4d, "omni:fabric:localMatrix", usdrt.Usd.Access.ReadWrite),
                    ),
                    "cache": {
                        "translations": wp.zeros((len(self), 3), dtype=wp.float32, device=self._device),
                        "orientations": wp.zeros((len(self), 4), dtype=wp.float32, device=self._device),
                        "scales": wp.zeros((len(self), 3), dtype=wp.float32, device=self._device),
                    },
                }
        return self._fabric_data[key]
