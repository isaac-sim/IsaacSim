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

import weakref

import carb
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.utils.numpy as numpy_utils
import numpy as np
import omni.physics.tensors
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .prim import _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID, _MSG_PRIM_NOT_VALID
from .xform_prim import XformPrim


class RigidPrim(XformPrim):
    """High level wrapper for manipulating prims (that have Rigid Body API applied) and their attributes.

    This class is a wrapper over one or more USD prims in the stage to provide
    high-level functionality for manipulating rigid body properties, and other attributes.
    The prims are specified using paths that can include regular expressions for matching multiple prims.

    .. note::

        If the prims do not already have the Rigid Body API applied to them, it will be applied.

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
        masses: Masses of the rigid bodies (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        densities: Densities of the rigid bodies (shape ``(N, 1)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).

    Raises:
        ValueError: If no prims are found matching the specified path(s).
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> import omni.timeline
        >>> from isaacsim.core.experimental.prims import RigidPrim
        >>>
        >>> # given a USD stage with the prims: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> # - create wrapper over single prim (with masses)
        >>> prim = RigidPrim("/World/prim_0", masses=[1.0])  # doctest: +NO_CHECK
        >>> # - create wrapper over multiple prims using regex (with masses)
        >>> prims = RigidPrim("/World/prim_.*", masses=[1.0])  # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so that the Physics tensor entity becomes valid
        >>> omni.timeline.get_timeline_interface().play()
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
        # RigidPrim
        masses: list | np.ndarray | wp.array | None = None,
        densities: list | np.ndarray | wp.array | None = None,
    ) -> None:
        # define properties
        # - default state properties
        self._default_linear_velocities = None
        self._default_angular_velocities = None
        # - physics tensor entity properties
        self._physics_rigid_body_view = None
        # initialize base class
        super().__init__(
            paths,
            resolve_paths=resolve_paths,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        # apply rigid body API
        RigidPrim.ensure_api(self.prims, UsdPhysics.RigidBodyAPI)
        # initialize instance from arguments
        if masses is not None:
            self.set_masses(masses)
        if densities is not None:
            self.set_densities(densities)
        # setup subscriptions
        self._subscription_to_timeline_stop_event = (
            SimulationManager._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP),
                lambda event, obj=weakref.proxy(self): obj._on_timeline_stop(event),
            )
        )
        # setup physics-related configuration if simulation is running
        if SimulationManager._physics_sim_view__warp is not None:
            SimulationManager._physx_sim_interface.flush_changes()
            self._on_physics_ready(None)

    def __del__(self):
        super().__del__()
        self._subscription_to_timeline_stop_event = None
        if hasattr(self, "_physics_rigid_body_view"):
            del self._physics_rigid_body_view

    """
    Properties.
    """

    @property
    def num_shapes(self) -> int:
        """Number of shapes in the rigid body.

        Backends: :guilabel:`tensor`.

        Returns:
            Number of shapes in the rigid body.

        Raises:
            AssertionError: Physics tensor entity is not valid.
        """
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        return self._physics_rigid_body_view.max_shapes

    """
    Methods.
    """

    def is_physics_tensor_entity_valid(self) -> bool:
        """Check if the physics tensor entity is valid.

        Returns:
            Whether the physics tensor entity is valid.
        """
        return SimulationManager._physics_sim_view__warp is not None and self._physics_rigid_body_view is not None

    def set_world_poses(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the poses (positions and orientations) in the world frame of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

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
        backend = self._check_for_tensor_backend(
            backend_utils.get_current_backend(["tensor", "usd", "usdrt", "fabric"])
        )
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_transforms()  # shape: (N, 7), quaternion is xyzw
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            if positions is not None:
                positions = ops_utils.broadcast_to(
                    positions, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device
                )
                wp.copy(data[indices, :3], positions)
            if orientations is not None:
                orientations = ops_utils.broadcast_to(
                    orientations, shape=(indices.shape[0], 4), dtype=wp.float32, device=data.device
                )
                wp.copy(data[indices, wp.array([6, 3, 4, 5], dtype=wp.int32, device=data.device)], orientations)
            self._physics_rigid_body_view.set_transforms(data, indices)
        # USD/USDRT/Fabric API
        else:
            super().set_world_poses(positions=positions, orientations=orientations, indices=indices)

    def get_world_poses(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the poses (positions and orientations) in the world frame of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

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
        backend = self._check_for_tensor_backend(
            backend_utils.get_current_backend(["tensor", "usd", "usdrt", "fabric"])
        )
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_transforms()  # shape: (N, 7), quaternion is xyzw
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            return (
                data[indices, :3].contiguous().to(self._device),
                data[indices, wp.array([6, 3, 4, 5], dtype=wp.int32, device=data.device)].contiguous().to(self._device),
            )
        # USD/USDRT/Fabric API
        else:
            return super().get_world_poses(indices=indices)

    def get_local_poses(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the poses (translations and orientations) in the local frame of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

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
        backend = self._check_for_tensor_backend(
            backend_utils.get_current_backend(["tensor", "usd", "usdrt", "fabric"])
        )
        # Tensor API
        if backend == "tensor":
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            world_positions, world_orientations = self.get_world_poses(indices=indices)
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                parent_transforms[i] = np.array(
                    UsdGeom.Xformable(self.prims[index].GetParent()).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype=np.float32,
                )
            local_translations, local_orientations = numpy_utils.transformations.get_local_from_world(
                parent_transforms, world_positions.numpy(), world_orientations.numpy()
            )
            return (
                ops_utils.place(local_translations, device=self._device),
                ops_utils.place(local_orientations, device=self._device),
            )
        # USD/USDRT/Fabric API
        else:
            return super().get_local_poses(indices=indices)

    def set_local_poses(
        self,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the poses (translations and orientations) in the local frame of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

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
        backend = self._check_for_tensor_backend(
            backend_utils.get_current_backend(["tensor", "usd", "usdrt", "fabric"])
        )
        # Tensor API
        if backend == "tensor":
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
            if translations is None or orientations is None:
                current_translations, current_orientations = self.get_local_poses(indices=indices)
                translations = current_translations if translations is None else translations
                orientations = current_orientations if orientations is None else orientations
            translations = ops_utils.broadcast_to(
                translations, shape=(indices.shape[0], 3), dtype=wp.float32, device=self._device
            )
            orientations = ops_utils.broadcast_to(
                orientations, shape=(indices.shape[0], 4), dtype=wp.float32, device=self._device
            )
            # compute transforms
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                parent_transforms[i] = np.array(
                    UsdGeom.Xformable(self.prims[index].GetParent()).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype=np.float32,
                )
            world_positions, world_orientations = numpy_utils.transformations.get_world_from_local(
                parent_transforms, translations.numpy(), orientations.numpy()
            )
            self.set_world_poses(positions=world_positions, orientations=world_orientations, indices=indices)
        # USD/USDRT/Fabric API
        else:
            super().set_local_poses(translations=translations, orientations=orientations, indices=indices)

    def set_velocities(
        self,
        linear_velocities: list | np.ndarray | wp.array | None = None,
        angular_velocities: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the velocities (linear and angular) of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            linear_velocities: Linear velocities (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            angular_velocities: Angular velocities (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither linear_velocities nor angular_velocities are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set random velocities for all prims
            >>> linear_velocities = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> angular_velocities = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> prims.set_velocities(linear_velocities, angular_velocities)
        """
        assert (
            linear_velocities is not None or angular_velocities is not None
        ), "Both 'linear_velocities' and 'angular_velocities' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_velocities()  # shape: (N, 6)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            if linear_velocities is not None:
                linear_velocities = ops_utils.broadcast_to(
                    linear_velocities, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device
                )
                wp.copy(data[indices, :3], linear_velocities)
            if angular_velocities is not None:
                angular_velocities = ops_utils.broadcast_to(
                    angular_velocities, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device
                )
                wp.copy(data[indices, 3:], angular_velocities)
            self._physics_rigid_body_view.set_velocities(data, indices)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            if linear_velocities is not None:
                linear_velocities = ops_utils.place(linear_velocities, device="cpu").numpy().reshape((-1, 3))
            if angular_velocities is not None:
                angular_velocities = ops_utils.place(angular_velocities, device="cpu").numpy().reshape((-1, 3))
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.RigidBodyAPI)[0]
                if linear_velocities is not None:
                    rigid_body_api.GetVelocityAttr().Set(
                        Gf.Vec3f(*linear_velocities[0 if linear_velocities.shape[0] == 1 else i].tolist())
                    )
                if angular_velocities is not None:
                    rigid_body_api.GetAngularVelocityAttr().Set(
                        Gf.Vec3f(*np.rad2deg(angular_velocities[0 if angular_velocities.shape[0] == 1 else i]).tolist())
                    )

    def get_velocities(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array, wp.array]:
        """Get the velocities (linear and angular) of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The linear velocities (shape ``(N, 3)``). 2) The angular velocities (shape ``(N, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the velocities of all prims
            >>> linear_velocities, angular_velocities = prims.get_velocities()
            >>> linear_velocities.shape, angular_velocities.shape
            ((3, 3), (3, 3))
            >>>
            >>> # get the velocities of the first prim
            >>> linear_velocities, angular_velocities = prims.get_velocities(indices=[0])
            >>> linear_velocities.shape, angular_velocities.shape
            ((1, 3), (1, 3))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_velocities()  # shape: (N, 6)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            return data[indices, :3].contiguous().to(self._device), data[indices, 3:].contiguous().to(self._device)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            linear_velocities = np.zeros((indices.shape[0], 3), dtype=np.float32)
            angular_velocities = np.zeros((indices.shape[0], 3), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.RigidBodyAPI)[0]
                linear_velocities[i] = np.array(rigid_body_api.GetVelocityAttr().Get(), dtype=np.float32)
                angular_velocities[i] = np.array(rigid_body_api.GetAngularVelocityAttr().Get(), dtype=np.float32)
            return (
                ops_utils.place(linear_velocities, device=self._device),
                ops_utils.place(np.deg2rad(angular_velocities), device=self._device),
            )

    def apply_forces(
        self,
        forces: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
        local_frame: bool = False,
    ) -> None:
        """Apply forces at the link transforms on the prims.

        Backends: :guilabel:`tensor`.

        Args:
            forces: Forces to apply (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            local_frame: Whether to apply forces in the local frame (true) or world frame (false).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # apply an external force to all prims
            >>> forces = np.random.uniform(low=0, high=100, size=(3, 3))
            >>> prims.apply_forces(forces)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        forces = ops_utils.broadcast_to(forces, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device)
        wp.copy(data[indices], forces)
        self._physics_rigid_body_view.apply_forces(data, indices, is_global=not local_frame)

    def apply_forces_and_torques_at_pos(
        self,
        forces: list | np.ndarray | wp.array | None = None,
        torques: list | np.ndarray | wp.array | None = None,
        *,
        positions: list | np.ndarray | wp.array | None = None,
        indices: list | np.ndarray | wp.array | None = None,
        local_frame: bool = False,
    ) -> None:
        """Apply forces and torques at specified positions on the prims.

        Backends: :guilabel:`tensor`.

        Args:
            forces: Forces to apply (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            torques: Torques to apply (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            positions: Positions to apply forces and torques at (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            local_frame: Whether to apply forces and torques in the local frame (true) or world frame (false).

        Raises:
            AssertionError: If neither forces nor torques are specified.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # apply an external force and torque to all prims at a specific position
            >>> forces = np.random.uniform(low=0, high=100, size=(3, 3))
            >>> torques = np.random.uniform(low=0, high=100, size=(3, 3))
            >>> prims.apply_forces_and_torques_at_pos(forces, torques, positions=[0.1, 0.0, 0.0])
        """
        assert (
            forces is not None or torques is not None
        ), "Both 'forces' and 'torques' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        view_forces = None
        view_torques = None
        view_positions = None
        indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
        if forces is not None:
            view_forces = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            forces = ops_utils.broadcast_to(
                forces, shape=(indices.shape[0], 3), dtype=wp.float32, device=view_forces.device
            )
            wp.copy(view_forces[indices], forces)
        if torques is not None:
            view_torques = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            torques = ops_utils.broadcast_to(
                torques, shape=(indices.shape[0], 3), dtype=wp.float32, device=view_torques.device
            )
            wp.copy(view_torques[indices], torques)
        if positions is not None:
            view_positions = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            positions = ops_utils.broadcast_to(
                positions, shape=(indices.shape[0], 3), dtype=wp.float32, device=view_positions.device
            )
            wp.copy(view_positions[indices], positions)
        self._physics_rigid_body_view.apply_forces_and_torques_at_position(
            view_forces,
            view_torques,
            view_positions,
            indices=indices,
            is_global=not local_frame,
        )

    def get_masses(self, *, indices: list | np.ndarray | wp.array | None = None, inverse: bool = False) -> wp.array:
        """Get the masses of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            inverse: Whether to return inverse masses (true) or masses (false).

        Returns:
            The masses or inverse masses (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the masses of all prims
            >>> masses = prims.get_masses()
            >>> masses.shape
            (3, 1)
            >>>
            >>> # get the masses of the first and last prims
            >>> masses = prims.get_masses(indices=[0, 2])
            >>> masses.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            if inverse:
                data = self._physics_rigid_body_view.get_inv_masses()  # shape: (N, 1)
            else:
                data = self._physics_rigid_body_view.get_masses()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            return data[indices].contiguous().to(self._device)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            masses = np.zeros((indices.shape[0], 1), dtype=np.float32)
            for i, index in enumerate(indices.numpy()):
                mass_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.MassAPI)[0]
                masses[i] = mass_api.GetMassAttr().Get()
            if inverse:
                masses = 1.0 / (masses + 1e-8)
            return ops_utils.place(masses, device=self._device)

    def get_coms(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the center of mass (COM) pose (position and orientation) of the prims.

        Backends: :guilabel:`tensor`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The center of mass positions (shape ``(N, 3)``).
            2) The center of mass orientations (shape ``(N, 4)``, quaternion ``wxyz``).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the COM poses of all prims
            >>> positions, orientations = prims.get_coms()
            >>> positions.shape, orientations.shape
            ((3, 3), (3, 4))
            >>>
            >>> # get the COM poses of the first and last prims
            >>> positions, orientations = prims.get_coms(indices=[0, 2])
            >>> positions.shape, orientations.shape
            ((2, 3), (2, 4))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_rigid_body_view.get_coms()  # shape: (N, 7), quaternion is xyzw
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        return (
            data[indices, :3].contiguous().to(self._device),
            data[indices, wp.array([6, 3, 4, 5], dtype=wp.int32, device=data.device)].contiguous().to(self._device),
        )

    def get_inertias(self, *, indices: list | np.ndarray | wp.array | None = None, inverse: bool = False) -> wp.array:
        """Get the inertia tensors of the prims.

        Backends: :guilabel:`tensor`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            inverse: Whether to return inverse inertia tensors (true) or inertia tensors (false).

        Returns:
            The inertia tensors or inverse inertia tensors (shape ``(N, 9)``).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the inertia tensors of all prims
            >>> inertias = prims.get_inertias()
            >>> inertias.shape
            (3, 9)
            >>>
            >>> # get the inverse inertia tensors of the first and last prims
            >>> inertias = prims.get_inertias(indices=[0, 2], inverse=True)
            >>> inertias.shape
            (2, 9)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        if inverse:
            data = self._physics_rigid_body_view.get_inv_inertias()  # shape: (N, 9)
        else:
            data = self._physics_rigid_body_view.get_inertias()  # shape: (N, 9)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        return data[indices].contiguous().to(self._device)

    def set_masses(
        self,
        masses: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the masses of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            masses: Masses (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the masses for all prims
            >>> prims.set_masses([10.0])
            >>>
            >>> # set the masses for the first and last prims
            >>> prims.set_masses([20.0], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_masses()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            masses = ops_utils.broadcast_to(masses, shape=(indices.shape[0], 1), dtype=wp.float32, device=data.device)
            wp.copy(data[indices], masses)
            self._physics_rigid_body_view.set_masses(data, indices)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            masses = ops_utils.place(masses, device="cpu").numpy().reshape((-1, 1))
            for i, index in enumerate(indices.numpy()):
                mass_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.MassAPI)[0]
                mass_api.GetMassAttr().Set(masses[0 if masses.shape[0] == 1 else i].item())

    def set_inertias(
        self,
        inertias: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the inertia tensors of the prims.

        Backends: :guilabel:`tensor`.

        Args:
            inertias: Inertia tensors (shape ``(N, 9)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # set the inertia tensors for all prims
            >>> inertias = np.diag([0.1, 0.1, 0.1]).flatten()  # shape: (9,)
            >>> prims.set_inertias(inertias)
            >>>
            >>> # set the inertia tensors for the first and last prims
            >>> inertias = np.diag([0.2, 0.2, 0.2]).flatten()  # shape: (9,)
            >>> prims.set_inertias(inertias, indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_rigid_body_view.get_inertias()  # shape: (N, 9)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        inertias = ops_utils.broadcast_to(inertias, shape=(indices.shape[0], 9), dtype=wp.float32, device=data.device)
        wp.copy(data[indices], inertias)
        self._physics_rigid_body_view.set_inertias(data, indices)

    def set_coms(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the center of mass (COM) pose (position and orientation) of the prims.

        Backends: :guilabel:`tensor`.

        Args:
            positions: Center of mass positions (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            orientations: Center of mass orientations (shape ``(N, 4)``, quaternion ``wxyz``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither positions nor orientations are specified.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # set random COM poses for all prims
            >>> positions = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> orientations = np.random.randn(3, 4)
            >>> orientations = orientations / np.linalg.norm(orientations, axis=-1, keepdims=True)  # normalize quaternions
            >>> prims.set_coms(positions, orientations)
        """
        assert (
            positions is not None or orientations is not None
        ), "Both 'positions' and 'orientations' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_rigid_body_view.get_coms()  # shape: (N, 7)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        if positions is not None:
            positions = ops_utils.broadcast_to(
                positions, shape=(indices.shape[0], 3), dtype=wp.float32, device=data.device
            )
            wp.copy(data[indices, :3], positions)
        if orientations is not None:
            orientations = ops_utils.broadcast_to(
                orientations, shape=(indices.shape[0], 4), dtype=wp.float32, device=data.device
            )
            wp.copy(data[indices, wp.array([6, 3, 4, 5], dtype=wp.int32, device=data.device)], orientations)
        self._physics_rigid_body_view.set_coms(data, indices)

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

            >>> # set the densities for all prims
            >>> prims.set_densities([100])
            >>>
            >>> # set the densities for the first and last prims
            >>> prims.set_densities([200], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        densities = ops_utils.place(densities, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            mass_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.MassAPI)[0]
            mass_api.GetDensityAttr().Set(densities[0 if densities.shape[0] == 1 else i].item())

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
            >>> # get densities of the first and last prims
            >>> densities = prims.get_densities(indices=[0, 2])
            >>> densities.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        densities = np.zeros(shape=(indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            mass_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.MassAPI)[0]
            densities[i] = mass_api.GetDensityAttr().Get()
        return ops_utils.place(densities, device=self._device)

    def set_sleep_thresholds(
        self,
        thresholds: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the sleep thresholds of the prims.

        Backends: :guilabel:`usd`.

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details.

        Args:
            thresholds: Sleep thresholds (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the sleep thresholds for all prims
            >>> prims.set_sleep_thresholds([1e-5])
            >>>
            >>> # set the sleep thresholds for the first and last prims
            >>> prims.set_sleep_thresholds([1.5e-5], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        thresholds = ops_utils.place(thresholds, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            rigid_body_api = RigidPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxRigidBodyAPI)[0]
            rigid_body_api.GetSleepThresholdAttr().Set(thresholds[0 if thresholds.shape[0] == 1 else i].item())

    def get_sleep_thresholds(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the sleep thresholds of the prims.

        Backends: :guilabel:`usd`.

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The sleep thresholds (shape ``(N, 1)``).

        Example:

        .. code-block:: python

            >>> # get the sleep thresholds of all prims
            >>> thresholds = prims.get_sleep_thresholds()
            >>> thresholds.shape
            (3, 1)
            >>>
            >>> # get the sleep threshold of the first and last prims
            >>> thresholds = prims.get_sleep_thresholds(indices=[0, 2])
            >>> thresholds.shape
            (2, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        thresholds = np.zeros(shape=(indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            rigid_body_api = RigidPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxRigidBodyAPI)[0]
            thresholds[i] = rigid_body_api.GetSleepThresholdAttr().Get()
        return ops_utils.place(thresholds, device=self._device)

    def set_enabled_rigid_bodies(
        self,
        enabled: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enable or disable the rigid body dynamics of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        When disabled, the prims will not participate in the physics simulation.

        Args:
            enabled: Boolean flags to enable/disable rigid body dynamics (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the rigid body dynamics for all prims
            >>> prims.set_enabled_rigid_bodies([True])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_disable_simulations()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            enabled = ops_utils.place(enabled, dtype=wp.uint8, device=data.device).reshape((-1, 1))
            disabled = np.logical_not(enabled.numpy()).astype(np.uint8)  # negate values
            disabled = ops_utils.broadcast_to(disabled, shape=(indices.shape[0], 1), dtype=wp.uint8, device=data.device)
            wp.copy(data[indices], disabled)
            self._physics_rigid_body_view.set_disable_simulations(data, indices)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            enabled = ops_utils.place(enabled, device="cpu").numpy().reshape((-1, 1))
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.RigidBodyAPI)[0]
                rigid_body_api.GetRigidBodyEnabledAttr().Set(bool(enabled[0 if enabled.shape[0] == 1 else i].item()))

    def get_enabled_rigid_bodies(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the enabled state of the rigid body dynamics of the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if the rigid body dynamics is enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the rigid body dynamics enabled state of all prims
            >>> print(prims.get_enabled_rigid_bodies())
            [[ True]
             [ True]
             [ True]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_disable_simulations()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            enabled = np.logical_not(data[indices].contiguous().numpy()).astype(np.bool_)  # negate values
            return ops_utils.place(enabled, device=self._device)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            enabled = np.zeros((indices.shape[0], 1), dtype=np.bool_)
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], UsdPhysics.RigidBodyAPI)[0]
                enabled[i] = rigid_body_api.GetRigidBodyEnabledAttr().Get()
            return ops_utils.place(enabled, device=self._device)

    def set_enabled_gravities(
        self,
        enabled: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enable or disable the gravity on the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        When disabled, the prims will not be affected by the gravity.

        Args:
            enabled: Boolean flags to enable/disable gravity (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the gravity for all prims
            >>> prims.set_enabled_gravities([True])
            >>>
            >>> # disable the gravity for the first and last prims
            >>> prims.set_enabled_gravities([False], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_disable_gravities()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            enabled = ops_utils.place(enabled, dtype=wp.uint8, device=data.device).reshape((-1, 1))
            disabled = np.logical_not(enabled.numpy()).astype(np.uint8)  # negate values
            disabled = ops_utils.broadcast_to(disabled, shape=(indices.shape[0], 1), dtype=wp.uint8, device=data.device)
            wp.copy(data[indices], disabled)
            self._physics_rigid_body_view.set_disable_gravities(data, indices)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            enabled = ops_utils.place(enabled, device="cpu").numpy().reshape((-1, 1))
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxRigidBodyAPI)[0]
                rigid_body_api.GetDisableGravityAttr().Set(not bool(enabled[0 if enabled.shape[0] == 1 else i].item()))

    def get_enabled_gravities(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the enabled state of the gravity on the prims.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if the gravity is enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the gravity enabled state of all prims after disabling it for the second prim
            >>> prims.set_enabled_gravities([False], indices=[1])
            >>> print(prims.get_enabled_gravities())
            [[ True]
             [False]
             [ True]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        backend = self._check_for_tensor_backend(backend_utils.get_current_backend(["tensor", "usd"]))
        # Tensor API
        if backend == "tensor":
            data = self._physics_rigid_body_view.get_disable_gravities()  # shape: (N, 1)
            indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
            enabled = np.logical_not(data[indices].contiguous().numpy()).astype(np.bool_)  # negate values
            return ops_utils.place(enabled, device=self._device)
        # USD API
        else:
            indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
            enabled = np.zeros((indices.shape[0], 1), dtype=np.bool_)
            for i, index in enumerate(indices.numpy()):
                rigid_body_api = RigidPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxRigidBodyAPI)[0]
                enabled[i] = not rigid_body_api.GetDisableGravityAttr().Get()
            return ops_utils.place(enabled, device=self._device)

    def set_default_state(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        linear_velocities: list | np.ndarray | wp.array | None = None,
        angular_velocities: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the default state (positions, orientations, linear velocities and angular velocities) of the prims.

        Backends: :guilabel:`usd`.

        .. hint::

            Prims can be reset to their default state by calling the :py:meth:`reset_to_default_state` method.

        Args:
            positions: Default positions in the world frame (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            orientations: Default orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            linear_velocities: Default linear velocities (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            angular_velocities: Default angular velocities (shape ``(N, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: If prims are non-root articulation links.
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
        # update default values
        # - positions and orientations
        if positions is not None or orientations is not None:
            super().set_default_state(positions=positions, orientations=orientations, indices=indices)
        # - linear velocities
        if linear_velocities is not None:
            if self._default_linear_velocities is None:
                self._default_linear_velocities = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            linear_velocities = ops_utils.broadcast_to(
                linear_velocities, shape=(indices.shape[0], 3), dtype=wp.float32, device=self._device
            )
            wp.copy(self._default_linear_velocities[indices], linear_velocities)
        # - angular velocities
        if angular_velocities is not None:
            if self._default_angular_velocities is None:
                self._default_angular_velocities = wp.zeros((len(self), 3), dtype=wp.float32, device=self._device)
            angular_velocities = ops_utils.broadcast_to(
                angular_velocities, shape=(indices.shape[0], 3), dtype=wp.float32, device=self._device
            )
            wp.copy(self._default_angular_velocities[indices], angular_velocities)

    def get_default_state(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> tuple[wp.array | None, wp.array | None, wp.array | None, wp.array | None]:
        """Get the default state (positions, orientations, linear velocities and angular velocities) of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Four-elements tuple. 1) The default positions in the world frame (shape ``(N, 3)``).
            2) The default orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            3) The default linear velocities (shape ``(N, 3)``). 4) The default angular velocities (shape ``(N, 3)``).
            If the default state is not set using the :py:meth:`set_default_state` method, ``None`` is returned.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: If prims are non-root articulation links.
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        if indices is not None:
            indices = ops_utils.resolve_indices(indices, count=len(self), device=self._device)
        # get default values
        # - positions and orientations
        default_positions, default_orientations = super().get_default_state(indices=indices)
        # - linear velocities
        default_linear_velocities = self._default_linear_velocities
        if default_linear_velocities is not None and indices is not None:
            default_linear_velocities = default_linear_velocities[indices].contiguous()
        # - angular velocities
        default_angular_velocities = self._default_angular_velocities
        if default_angular_velocities is not None and indices is not None:
            default_angular_velocities = default_angular_velocities[indices].contiguous()
        return default_positions, default_orientations, default_linear_velocities, default_angular_velocities

    def reset_to_default_state(self, *, warn_on_non_default_state: bool = False) -> None:
        """Reset the prims to the specified default state.

        Backends: :guilabel:`tensor`, :guilabel:`usd`.

        This method applies the default state defined using the :py:meth:`set_default_state` method.

        .. note::

            This method *teleports* prims to the specified default state (positions and orientations)
            and sets the linear and angular velocities immediately.

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
            (None, None, None, None)
            >>>
            >>> # set default state
            >>> # - random positions for each prim
            >>> # - same fixed orientation for all prims
            >>> # - zero velocities for all prims
            >>> positions = np.random.uniform(low=-1, high=1, size=(3, 3))
            >>> prims.set_default_state(
            ...     positions=positions,
            ...     orientations=[1.0, 0.0, 0.0, 0.0],
            ...     linear_velocities=np.zeros(3),
            ...     angular_velocities=np.zeros(3),
            ... )
            >>>
            >>> # get default state (default state is set)
            >>> prims.get_default_state()
            (array(shape=(3, 3), dtype=float32),
             array(shape=(3, 4), dtype=float32),
             array(shape=(3, 3), dtype=float32),
             array(shape=(3, 3), dtype=float32))
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
        if self._default_linear_velocities is not None or self._default_angular_velocities is not None:
            self.set_velocities(self._default_linear_velocities, self._default_angular_velocities)
        else:
            if warn_on_non_default_state:
                carb.log_warn(
                    "No default linear or angular velocities to reset. Call '.set_default_state(..)' first to initialize them"
                )

    """
    Internal methods.
    """

    def _check_for_tensor_backend(self, backend: str, *, fallback_backend: str = "usd") -> str:
        """Check if the tensor backend is valid."""
        if backend == "tensor" and not self.is_physics_tensor_entity_valid():
            if backend_utils.is_backend_set():
                if backend_utils.should_raise_on_fallback():
                    raise RuntimeError(
                        f"Physics tensor entity is not valid. Fallback set to '{fallback_backend}' backend."
                    )
                carb.log_warn(
                    f"Physics tensor entity is not valid for use with 'tensor' backend. Falling back to '{fallback_backend}' backend."
                )
            return fallback_backend
        return backend

    """
    Internal callbacks.
    """

    def _on_physics_ready(self, event) -> None:
        """Handle physics ready event."""
        super()._on_physics_ready(event)
        # get physics simulation view
        physics_simulation_view = SimulationManager._physics_sim_view__warp
        if physics_simulation_view is None or not physics_simulation_view.is_valid:
            carb.log_warn(f"Invalid physics simulation view. RigidPrim ({self.paths}) will not be initialized")
            return
        # create rigid body view
        self._physics_rigid_body_view = physics_simulation_view.create_rigid_body_view(self.paths)
        # validate rigid body view
        if self._physics_rigid_body_view is None:
            carb.log_warn(f"Unable to create rigid body view for {self.paths}")
            return
        if not self._physics_rigid_body_view.check():
            carb.log_warn(
                f"Unable to create rigid body view for {self.paths}. Underlying physics objects are not valid"
            )
            self._physics_rigid_body_view = None

    def _on_timeline_stop(self, event) -> None:
        """Handle timeline stop event."""
        # invalidate rigid body view
        self._physics_rigid_body_view = None
