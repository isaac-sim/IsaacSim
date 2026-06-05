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

"""Provide high level functions for dealing with rigid body prims and their physics properties."""

from __future__ import annotations

import weakref

import carb
import carb.eventdispatcher
import numpy as np
import omni.physics.tensors
import omni.timeline
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.prims import get_prim_parent
from isaacsim.core.utils.types import DynamicsViewState
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .xform_prim import XFormPrim

torch = import_module("torch")


class RigidPrim(XFormPrim):
    """Provide high-level functions for prims that have Rigid Body API applied to them.

    Handle attributes and properties of single or multiple rigid body prims.

    Wrap all matching rigid prims found at the regex provided at the ``prim_paths_expr`` argument

    .. note::

        Each prim will have ``xformOp:orient``, ``xformOp:translate`` and ``xformOp:scale`` only post-init,
        unless it is a non-root articulation link.

        If the prims do not already have the Rigid Body API applied to them before init, it will apply it.

    .. warning::

        The rigid prim view object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_paths_expr: prim paths regex to encapsulate all prims that match it.
            example: "/World/Env[1-5]/Cube" will match /World/Env1/Cube,
            /World/Env2/Cube..etc.
            (a non regex prim path can also be used to encapsulate one rigid prim). Additionally a
            list of regex can be provided. example ["/World/Env[1-5]/Cube", "/World/Env[10-19]/Cube"].
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        positions: default positions in the world frame of the prims.
            shape is (N, 3).
        translations: default translations in the local frame of the prims
            (with respect to its parent prims). shape is (N, 3).
        orientations: default quaternion orientations in the world/ local frame of the prims
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (N, 4).
        scales: local scales to be applied to
            the prim's dimensions in the view. shape is (N, 3).
        visibilities: set to false for an invisible prim in
            the stage while rendering. shape is (N,).
        reset_xform_properties: True if the prims don't have the right set of xform properties
            (i.e: translate, orient and scale) ONLY and in that order.
            Set this parameter to False if the object were cloned using using
            the cloner api in isaacsim.core.cloner.
        masses: mass in kg specified for each prim in the view.
            shape is (N,).
        densities: density in kg/m^3 specified for each prim in the view.
            shape is (N,).
        linear_velocities: default linear velocity of each prim in the view
            (to be applied in the first frame and on resets).
            Shape is (N, 3).
        angular_velocities: default angular velocity of each prim in the view
            (to be applied in the first frame and on resets).
            Shape is (N, 3).
        track_contact_forces: if enabled, the view will track the net contact forces on each rigid prim in the view
        prepare_contact_sensors: if rigid prims in the view are not cloned from a prim in a prepared state,
            (although slow for large number of prims) this ensures that
            appropriate physics settings are applied on all the prim in the view.
        disable_stablization: disables the contact stabilization parameter in the physics context
        contact_filter_prim_paths_expr: a list of filter expressions which allows for tracking contact forces
            between prims and this subset through get_contact_force_matrix().
        max_contact_count: maximum number of contact data to report when detailed contact information is needed

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.cloner import GridCloner
        >>> from isaacsim.core.prims import RigidPrim
        >>> from pxr import UsdGeom
        >>>
        >>> env_zero_path = "/World/envs/env_0"
        >>> num_envs = 5
        >>>
        >>> # clone the environment (num_envs)
        >>> cloner = GridCloner(spacing=1.5)
        >>> cloner.define_base_env(env_zero_path)
        >>> UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform", "Xform")
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform/Cube", "Cube")
        >>> env_pos = cloner.clone(
        ...     source_prim_path=env_zero_path,
        ...     prim_paths=cloner.generate_paths("/World/envs/env", num_envs),
        ...     copy_from_source=True
        ... )
        >>>
        >>> # wrap the prims
        >>> prims = RigidPrim(prim_paths_expr="/World/envs/env.*/Xform", name="rigid_prim_view")
        >>> prims
        <isaacsim.core.prims.rigid_prim.RigidPrim object at 0x7f9a23b8bb80>
    """

    def __init__(
        self,
        prim_paths_expr: str | list[str],
        name: str = "rigid_prim_view",
        positions: np.ndarray | torch.Tensor | wp.array | None = None,
        translations: np.ndarray | torch.Tensor | wp.array | None = None,
        orientations: np.ndarray | torch.Tensor | wp.array | None = None,
        scales: np.ndarray | torch.Tensor | wp.array | None = None,
        visibilities: np.ndarray | torch.Tensor | wp.array | None = None,
        reset_xform_properties: bool = True,
        masses: np.ndarray | torch.Tensor | wp.array | None = None,
        densities: np.ndarray | torch.Tensor | wp.array | None = None,
        linear_velocities: np.ndarray | torch.Tensor | wp.array | None = None,
        angular_velocities: np.ndarray | torch.Tensor | wp.array | None = None,
        track_contact_forces: bool = False,
        prepare_contact_sensors: bool = True,
        disable_stablization: bool = True,
        contact_filter_prim_paths_expr: list[str] | None = None,
        max_contact_count: int = 0,
    ) -> None:
        if contact_filter_prim_paths_expr is None:
            contact_filter_prim_paths_expr = []
        self._physics_view = None
        self._num_shapes = None
        self._contact_filter_prim_paths_expr = contact_filter_prim_paths_expr
        self._track_contact_forces = track_contact_forces or len(contact_filter_prim_paths_expr) != 0
        if self._track_contact_forces:
            from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView

            self._contact_view = RigidContactView(
                prim_paths_expr=prim_paths_expr,
                filter_paths_expr=contact_filter_prim_paths_expr,
                name=name + "_contact",
                prepare_contact_sensors=prepare_contact_sensors,
                disable_stablization=disable_stablization,
                max_contact_count=max_contact_count,
            )
        XFormPrim.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
        )
        self._rigid_body_apis = [None] * self._count
        self._physx_rigid_body_apis = [None] * self._count
        self._mass_apis = [None] * self._count
        if not self._non_root_link:
            if linear_velocities is not None:
                self.set_linear_velocities(linear_velocities)
            if angular_velocities is not None:
                self.set_angular_velocities(angular_velocities)
        if masses is not None:
            RigidPrim.set_masses(self, masses)
        if densities is not None:
            RigidPrim.set_densities(self, densities)
        self._dynamics_default_state = DynamicsViewState(
            positions=None, linear_velocities=None, orientations=None, angular_velocities=None
        )
        self._apply_rigid_body_apis(prepare_contact_sensors or self._track_contact_forces)

        self._invalidation_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=lambda event, obj=weakref.proxy(self): obj._invalidate_physics_handle_callback(event),
            observer_name="isaacsim.core.prims.RigidPrim.initialize._invalidate_physics_handle_callback",
        )
        if SimulationManager.get_physics_sim_view() is not None:
            SimulationManager._physics_sim_interface.flush_changes()
            RigidPrim._on_physics_ready(self, None)
        return

    def __del__(self) -> None:
        """Clean up resources when the object is destroyed."""
        XFormPrim.__del__(self)
        if hasattr(self, "_physics_view"):
            del self._physics_view
        self._invalidation_callback = None
        return

    def _invalidate_physics_handle_callback(self, event: object) -> None:
        """Callback to invalidate the physics handle when physics simulation stops.

        Args:
            event: The event triggering the invalidation.
        """
        self._physics_view = None

    @property
    def num_shapes(self) -> int:
        """Number of rigid shapes for the prims in the view.

        Returns:
            Number of rigid shapes for the prims in the view.

        Example:

        .. code-block:: python

            >>> prims.num_shapes
            1
        """
        return self._num_shapes

    def is_physics_handle_valid(self) -> bool:
        """Check if rigid prim view's physics handler is initialized.

        .. warning::

            If the physics handler is not valid many of the methods that requires PhysX will return None.

        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.

        Example:

        .. code-block:: python

            >>> prims.is_physics_handle_valid()
            True
        """
        return SimulationManager.get_physics_sim_view() is not None and self._physics_view is not None

    def set_world_poses(
        self,
        positions: np.ndarray | torch.Tensor | wp.array | None = None,
        orientations: np.ndarray | torch.Tensor | wp.array | None = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        usd: bool = True,
    ) -> None:
        """Set poses of prims in the view with respect to the world's frame.

        .. warning::

            This method will change (teleport) the prim poses immediately to the specified value

        Args:
            positions: positions in the world frame of the prim. shape is (M, 3).
                Defaults to None, which means left unchanged.
            orientations: quaternion orientations in the world frame of the prims.
                quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                Defaults to None, which means left unchanged.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            usd: True to query from usd. Otherwise False to query from Fabric data.

        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> # reposition all rigid prims in row (x-axis)
            >>> positions = np.zeros((num_envs, 3))
            >>> positions[:,0] = np.arange(num_envs)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1))
            >>> prims.set_world_poses(positions, orientations)
            >>>
            >>> # reposition only the rigid prims for the first, middle and last of the 5 envs in column (y-axis)
            >>> positions = np.zeros((3, 3))
            >>> positions[:,1] = np.arange(3)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
            >>> prims.set_world_poses(positions, orientations, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_positions, current_orientations = self.get_world_poses(clone=False)
            if not hasattr(self, "_pose_buf"):
                self._pose_buf = self._backend_utils.create_zeros_tensor(
                    shape=[self.count, 7], dtype="float32", device=self._device
                )
            if positions is not None:
                positions = self._backend_utils.move_data(positions, self._device)
            if orientations is not None:
                orientations = self._backend_utils.move_data(orientations, self._device)
            pose = self._backend_utils.assign_pose(
                current_positions, current_orientations, positions, orientations, indices, self._device, self._pose_buf
            )
            self._physics_view.set_transforms(pose, indices)
            return
        else:
            XFormPrim.set_world_poses(self, positions=positions, orientations=orientations, indices=indices, usd=usd)
        return

    def get_world_poses(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        usd: bool = True,
    ) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]:
        """Get the poses of the prims in the view with respect to the world's frame.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.
            usd: True to query from usd. Otherwise False to query from Fabric data.

        Returns:
            first index is positions in the world frame of the prims. shape is (M, 3).
            second index is quaternion orientations in the world frame of the prims.
            quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        .. code-block:: python

            >>> # get all rigid prim poses with respect to the world's frame.
            >>> # Returned shape is position (5, 3) and orientation (5, 4) for the example: 5 envs
            >>> positions, orientations = prims.get_world_poses()
            >>> positions
            [[ 1.4999989e+00 -7.4999851e-01 -1.5118626e-07]
             [ 1.4999989e+00  7.5000149e-01 -2.5988294e-07]
             [-1.0017333e-06 -7.4999845e-01  7.6070329e-08]
             [-9.5906785e-07  7.5000149e-01  1.0593490e-07]
             [-1.5000011e+00 -7.4999851e-01  1.9655154e-07]]
            >>> orientations
            [[ 9.9999994e-01 -8.8168377e-07 -4.1946004e-07 -1.5067183e-08]
             [ 9.9999994e-01 -8.8691013e-07 -4.2665880e-07 -2.7188951e-09]
             [ 1.0000000e+00 -9.5171310e-07 -2.2615541e-07  5.5922797e-08]
             [ 1.0000000e+00 -8.9923367e-07 -1.4408238e-07  1.3476099e-08]
             [ 1.0000000e+00 -7.9806580e-07 -1.3064776e-07  5.3154917e-08]]
            >>>
            >>> # get only the rigid prim poses with respect to the world's frame for the first, middle and last of the 5 envs.
            >>> # Returned shape is position (3, 3) and orientation (3, 4) for the example: 3 envs selected
            >>> positions, orientations = prims.get_world_poses(indices=np.array([0, 2, 4]))
            >>> positions
            [[ 1.4999989e+00 -7.4999851e-01 -1.5118626e-07]
             [-1.0017333e-06 -7.4999845e-01  7.6070329e-08]
             [-1.5000011e+00 -7.4999851e-01  1.9655154e-07]]
            >>> orientations
            [[ 9.9999994e-01 -8.8168377e-07 -4.1946004e-07 -1.5067183e-08]
             [ 1.0000000e+00 -9.5171310e-07 -2.2615541e-07  5.5922797e-08]
             [ 1.0000000e+00 -7.9806580e-07 -1.3064776e-07  5.3154917e-08]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            pose = self._physics_view.get_transforms()
            if clone:
                pose = self._backend_utils.clone_tensor(pose, device=self._device)
            pos = pose[indices, 0:3]
            rot = self._backend_utils.xyzw2wxyz(pose[indices, 3:7])
            return pos, rot
        else:
            return XFormPrim.get_world_poses(self, indices=indices, usd=usd)

    def get_local_poses(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]:
        """Get prim poses in the view with respect to the local frame (the prim's parent frame).

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)

        Returns:
            first index is positions in the local frame of the prims. shape is (M, 3).
            second index is quaternion orientations in the local frame of the prims.
            quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        .. code-block:: python

            >>> # get all rigid prim poses with respect to the local frame.
            >>> # Returned shape is position (5, 3) and orientation (5, 4) for the example: 5 envs
            >>> positions, orientations = prims.get_local_poses()
            >>> positions
            [[-1.0728836e-06  1.4901161e-06 -1.5118626e-07]
             [-1.0728836e-06  1.4901161e-06 -2.5988294e-07]
             [-1.0017333e-06  1.5497208e-06  7.6070329e-08]
             [-9.5906785e-07  1.4901161e-06  1.0593490e-07]
             [-1.0728836e-06  1.4901161e-06  1.9655154e-07]]
            >>> orientations
            [[ 1.0000000e+00 -8.8174920e-07 -4.1949116e-07 -1.5068302e-08]
             [ 1.0000000e+00 -8.8696777e-07 -4.2668654e-07 -2.7190719e-09]
             [ 1.0000000e+00 -9.5164734e-07 -2.2613979e-07  5.5918935e-08]
             [ 1.0000000e+00 -8.9923157e-07 -1.4408204e-07  1.3476067e-08]
             [ 1.0000000e+00 -7.9806864e-07 -1.3064822e-07  5.3155105e-08]]
            >>>
            >>> # get only the rigid prim poses with respect to the local frame for the first, middle and last of the 5 envs.
            >>> # Returned shape is position (3, 3) and orientation (3, 4) for the example: 3 envs selected
            >>> positions, orientations = prims.get_local_poses(indices=np.array([0, 2, 4]))
            >>> positions
            [[-1.0728836e-06  1.4901161e-06 -1.5118626e-07]
             [-1.0017333e-06  1.5497208e-06  7.6070329e-08]
             [-1.0728836e-06  1.4901161e-06  1.9655154e-07]]
            >>> orientations
            [[ 1.0000000e+00 -8.8174920e-07 -4.1949116e-07 -1.5068302e-08]
             [ 1.0000000e+00 -9.5164734e-07 -2.2613979e-07  5.5918935e-08]
             [ 1.0000000e+00 -7.9806864e-07 -1.3064822e-07  5.3155105e-08]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            world_positions, world_orientations = self.get_world_poses(indices=indices)
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype=np.float32)
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                parent_transforms[write_idx] = np.array(
                    UsdGeom.Xformable(get_prim_parent(self._prims[i])).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype="float32",
                )
                write_idx += 1
            parent_transforms = self._backend_utils.convert(parent_transforms, dtype="float32", device=self._device)
            return self._backend_utils.get_local_from_world(
                parent_transforms, world_positions, world_orientations, self._device
            )
        else:
            return XFormPrim.get_local_poses(self, indices=indices)

    def set_local_poses(
        self,
        translations: np.ndarray | torch.Tensor | wp.array | None = None,
        orientations: np.ndarray | torch.Tensor | wp.array | None = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set prim poses in the view with respect to the local frame (the prim's parent frame).

        Args:
            translations: translations in the local frame of the prims
                (with respect to its parent prim). shape is (M, 3).
                Defaults to None, which means left unchanged.
            orientations: quaternion orientations in the local frame of the prims.
                quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                Defaults to None, which means left unchanged.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # reposition all rigid prims
            >>> positions = np.zeros((num_envs, 3))
            >>> positions[:,0] = np.arange(num_envs)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1))
            >>> prims.set_local_poses(positions, orientations)
            >>>
            >>> # reposition only the rigid prims for the first, middle and last of the 5 envs
            >>> positions = np.zeros((3, 3))
            >>> positions[:,1] = np.arange(3)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
            >>> prims.set_local_poses(positions, orientations, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            if translations is None or orientations is None:
                current_translations, current_orientations = RigidPrim.get_local_poses(self, indices=indices)
                if translations is None:
                    translations = current_translations
                if orientations is None:
                    orientations = current_orientations
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype=np.float32)
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                parent_transforms[write_idx] = np.array(
                    UsdGeom.Xformable(get_prim_parent(self._prims[i])).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype="float32",
                )
                write_idx += 1
            parent_transforms = self._backend_utils.convert(parent_transforms, dtype="float32", device=self._device)
            calculated_positions, calculated_orientations = self._backend_utils.get_world_from_local(
                parent_transforms, translations, orientations, self._device
            )
            RigidPrim.set_world_poses(
                self, positions=calculated_positions, orientations=calculated_orientations, indices=indices
            )
        else:
            XFormPrim.set_local_poses(self, translations=translations, orientations=orientations, indices=indices)
        return

    def set_linear_velocities(
        self,
        velocities: np.ndarray | torch.Tensor | wp.array | None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set the linear velocities of the prims in the view.

        The method does this through the PhysX API only. It has to be called after initialization.
        Note: This method is not supported for the gpu pipeline. ``set_velocities`` method should be used instead.

        .. warning::

            This method will immediately set the rigid prim state

        Args:
            velocities: linear velocities to set the rigid prims to. shape is (M, 3).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        .. hint::

            This method belongs to the methods used to set the rigid prim kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``)

        Example:

        .. code-block:: python

            >>> # set each rigid prim linear velocity to (1.0, 1.0, 1.0)
            >>> velocities = np.ones((num_envs, 3))
            >>> prims.set_linear_velocities(velocities)
            >>>
            >>> # set only the rigid prim linear velocities for the first, middle and last of the 5 envs
            >>> velocities = np.ones((3, 3))
            >>> prims.set_linear_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self._device is not None and "cuda" in self._device:
            carb.log_warn(
                "set_linear_velocities function is not supported for the gpu pipeline, use set_velocities instead."
            )
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

        if self.is_physics_handle_valid():
            current_velocities = self._physics_view.get_velocities()
            if self._backend == "warp":
                current_velocities = self._backend_utils.assign(
                    self._backend_utils.move_data(velocities, device=self._device),
                    current_velocities,
                    [indices, wp.array([0, 1, 2], device=self._device, dtype=wp.int32)],
                )
            else:
                current_velocities[indices, 0:3] = self._backend_utils.move_data(velocities, device=self._device)
            self._physics_view.set_velocities(current_velocities, indices)
        else:
            idx_count = 0
            indices = self._backend_utils.to_list(indices)
            velocities = self._backend_utils.to_list(velocities)
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                self._rigid_body_apis[i].GetVelocityAttr().Set(Gf.Vec3f(*velocities[idx_count]))
                idx_count += 1
            return

    def get_linear_velocities(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get the linear velocities of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            linear velocities of the prims in the view. shape is (M, 3).

        Example:

        .. code-block:: python

            >>> # get all rigid prim linear velocities. Returned shape is (5, 3) for the example: 5 envs, linear (3)
            >>> prims.get_linear_velocities()
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
            >>>
            >>> # get only the rigid prim linear velocities for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 3) for the example: 3 envs selected, linear (3)
            >>> prims.get_linear_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

        if self.is_physics_handle_valid():
            linear_velocities = self._physics_view.get_velocities()
            if clone:
                linear_velocities = self._backend_utils.clone_tensor(linear_velocities, device=self._device)
            return linear_velocities[indices, 0:3]
        else:
            linear_velocities = np.zeros(shape=(indices.shape[0], 3), dtype=np.float32)
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                linear_velocities[write_idx] = np.array(
                    self._rigid_body_apis[i].GetVelocityAttr().Get(), dtype=np.float32
                )
                write_idx += 1
            linear_velocities = self._backend_utils.convert(
                linear_velocities, dtype="float32", device=self._device, indexed=True
            )
            return linear_velocities

    def set_angular_velocities(
        self,
        velocities: np.ndarray | torch.Tensor | wp.array | None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set the angular velocities of the prims in the view.

        The method does this through the physx API only. It has to be called after initialization.
        Note: This method is not supported for the gpu pipeline. ``set_velocities`` method should be used instead.

        .. warning::

            This method will immediately set the rigid prim state

        Args:
            velocities: angular velocities to set the rigid prims to. shape is (M, 3).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        .. hint::

            This method belongs to the methods used to set the rigid prim kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``)

        Example:

        .. code-block:: python

            >>> # set each rigid prim linear velocity to (5.0, 5.0, 5.0)
            >>> velocities = np.full((num_envs, 3), fill_value=5.0)
            >>> prims.set_angular_velocities(velocities)
            >>>
            >>> # set only the rigid prim linear velocities for the first, middle and last of the 5 envs
            >>> velocities = np.full((3, 3), fill_value=5.0)
            >>> prims.set_angular_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if self._device is not None and "cuda" in self._device:
            carb.log_warn(
                "set_angular_velocities function is not supported for the gpu pipeline, use set_velocities instead."
            )
        if self.is_physics_handle_valid():
            current_velocities = self._physics_view.get_velocities()
            if self._backend == "warp":
                current_velocities = self._backend_utils.assign(
                    self._backend_utils.move_data(velocities, device=self._device),
                    current_velocities,
                    [indices, wp.array([3, 4, 5], device=self._device, dtype=wp.int32)],
                )
            else:
                current_velocities[indices, 3:6] = self._backend_utils.move_data(velocities, self._device)
            self._physics_view.set_velocities(current_velocities, indices)
        else:
            idx_count = 0
            indices = self._backend_utils.to_list(indices)
            velocities = np.rad2deg(self._backend_utils.to_numpy(velocities)).tolist()
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                self._rigid_body_apis[i].GetAngularVelocityAttr().Set(Gf.Vec3f(*velocities[idx_count]))
                idx_count += 1
        return

    def get_angular_velocities(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get the angular velocities of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            angular velocities of the prims in the view. shape is (M, 3).

        Example:

        .. code-block:: python

            >>> # get all rigid prim angular velocities. Returned shape is (5, 3) for the example: 5 envs, angular (3)
            >>> prims.get_angular_velocities()
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
            >>>
            >>> # get only the rigid prim angular velocities for the first, middle and last of the 5 envs
            >>> # Returned shape is (5, 3) for the example: 3 envs selected, angular (3)
            >>> prims.get_angular_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if self.is_physics_handle_valid():
            angular_velocities = self._physics_view.get_velocities()
            if clone:
                angular_velocities = self._backend_utils.clone_tensor(angular_velocities, device=self._device)
            return angular_velocities[indices, 3:6]
        else:
            angular_velocities = np.zeros(shape=(indices.shape[0], 3), dtype=np.float32)
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                angular_velocities[write_idx] = np.array(
                    self._rigid_body_apis[i].GetAngularVelocityAttr().Get(), dtype="float32"
                )
                write_idx += 1
            angular_velocities = self._backend_utils.convert(
                np.deg2rad(angular_velocities), dtype="float32", device=self._device, indexed=True
            )
            return angular_velocities

    def set_velocities(
        self,
        velocities: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set the linear and angular velocities of the prims in the view at once.

        The method does this through the PhysX API only. It has to be called after initialization

        .. warning::

            This method will immediately set the rigid prim state

        Args:
            velocities: linear and angular velocities respectively to set the rigid prims to. shape is (M, 6).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        .. hint::

            This method belongs to the methods used to set the rigid prim kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``)

        Example:

        .. code-block:: python

            >>> # set each rigid prim linear velocity to (1., 1., 1.) and angular velocity to (5., 5., 5.)
            >>> velocities = np.ones((num_envs, 6))
            >>> velocities[:,3:] = 5.0
            >>> prims.set_velocities(velocities)
            >>>
            >>> # set only the rigid prim velocities for the first, middle and last of the 5 envs
            >>> velocities = np.ones((3, 6))
            >>> velocities[:,3:] = 5.0
            >>> prims.set_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

        if self.is_physics_handle_valid():
            new_velocities = self._physics_view.get_velocities()
            new_velocities = self._backend_utils.assign(
                self._backend_utils.move_data(velocities, self._device), new_velocities, indices
            )
            self._physics_view.set_velocities(new_velocities, indices)
        else:
            self.set_linear_velocities(velocities[:, 0:3], indices)
            self.set_angular_velocities(velocities[:, 3:6], indices)
        return

    def get_velocities(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get the linear and angular velocities of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            linear and angular velocities of the prims in the view concatenated. shape is (M, 6).

        Example:

        .. code-block:: python

            >>> # get all rigid prim velocities. Returned shape is (5, 6) for the example: 5 envs, linear (3) and angular (3)
            >>> prims.get_velocities()
            [[0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]]
            >>>
            >>> # get only the rigid prim velocities for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 6) for the example: 3 envs selected, linear (3) and angular (3)
            >>> prims.get_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

        if self.is_physics_handle_valid():
            velocities = self._physics_view.get_velocities()
            if clone:
                velocities = self._backend_utils.clone_tensor(velocities, device=self._device)
            return velocities[indices]
        else:
            linear_velocities = self.get_linear_velocities(indices, clone)
            angular_velocities = self.get_angular_velocities(indices, clone)
            return self._backend_utils.tensor_cat([linear_velocities, angular_velocities], dim=-1, device=self._device)

    def apply_forces(
        self,
        forces: np.ndarray | torch.Tensor | wp.array | None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        is_global: bool = True,
    ) -> None:
        """Apply forces to prims in the view.

        Args:
            forces: forces to be applied to the prims.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            is_global: True if forces are in the global frame. Otherwise False.

        Example:

        .. code-block:: python

            >>> # apply an external force to all the rigid bodies to the indicated values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> forces = np.tile(np.array([2e5, 1e5, 0.0]), (num_envs, 1))
            >>> prims.apply_forces(forces)
            >>>
            >>> # apply an external force to the rigid bodies for the first, middle and last of the 5 envs
            >>> forces = np.tile(np.array([2e5, 1e5, 0.0]), (3, 1))
            >>> prims.apply_forces(forces, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            new_forces = self._backend_utils.create_zeros_tensor([self.count, 3], device=self._device, dtype="float32")
            new_forces = self._backend_utils.assign(
                self._backend_utils.move_data(forces.reshape((indices.shape[0], 3)), self._device), new_forces, indices
            )
            self._physics_view.apply_forces(new_forces, indices, is_global)
        else:
            carb.log_warn("Physics Simulation View is not created yet")

    def apply_forces_and_torques_at_pos(
        self,
        forces: np.ndarray | torch.Tensor | wp.array | None = None,
        torques: np.ndarray | torch.Tensor | wp.array | None = None,
        positions: np.ndarray | torch.Tensor | wp.array | None = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        is_global: bool = True,
    ) -> None:
        """Apply forces and torques to prims in the view. The forces and/or torques can be in local or global coordinates.

        The forces can applied at a location given by positions variable.

        Args:
            forces: forces to be applied to the prims. If not specified, no force will be applied.
            torques: torques to be applied to the prims. If not specified, no torque will be applied.
            positions: position of the forces with respect to the body frame.
                If not specified, the forces are applied at the origin of the body frame.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            is_global: True if forces, torques, and positions are in the global frame.
                False if forces, torques, and positions are in the local frame.

        Example:

        .. code-block:: python

            >>> # apply an external force and torque to all the rigid bodies to the indicated values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> forces = np.tile(np.array([2e5, 1e5, 0.0]), (num_envs, 1))
            >>> torques = np.tile(np.array([2e5, 1e5, 0.0]), (num_envs, 1))
            >>> prims.apply_forces_and_torques_at_pos(forces, torques)
            >>>
            >>> # apply an external force and torque to the rigid bodies for the first, middle and last of the 5 envs
            >>> forces = np.tile(np.array([2e5, 1e5, 0.0]), (3, 1))
            >>> torques = np.tile(np.array([2e5, 1e5, 0.0]), (3, 1))
            >>> prims.apply_forces_and_torques_at_pos(forces, torques, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

            new_forces = new_torques = new_positions = None
            if forces is not None:
                new_forces = self._backend_utils.create_zeros_tensor(
                    [self.count, 3], device=self._device, dtype="float32"
                )
                new_forces = self._backend_utils.assign(
                    self._backend_utils.move_data(forces, self._device), new_forces, indices
                )
                if positions is not None:
                    new_positions = self._backend_utils.create_zeros_tensor(
                        [self.count, 3], device=self._device, dtype="float32"
                    )
                    new_positions = self._backend_utils.assign(
                        self._backend_utils.move_data(positions, self._device), new_positions, indices
                    )

            if torques is not None:
                new_torques = self._backend_utils.create_zeros_tensor(
                    [self.count, 3], device=self._device, dtype="float32"
                )
                new_torques = self._backend_utils.assign(
                    self._backend_utils.move_data(torques, self._device), new_torques, indices
                )

            self._physics_view.apply_forces_and_torques_at_position(
                force_data=new_forces,
                torque_data=new_torques,
                position_data=new_positions,
                indices=indices,
                is_global=is_global,
            )

        else:
            carb.log_warn("Physics Simulation View is not created yet")

    def get_masses(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get rigid body masses of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            masses of in kg of prims in the view. shape is (M,).

        Example:

        .. code-block:: python

            >>> # get all rigid body masses. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_masses()
            [999.99994 999.99994 999.99994 999.99994 999.99994]
            >>>
            >>> # get rigid body masses for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_masses(indices=np.array([0, 2, 4]))
            [999.99994 999.99994 999.99994]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if self.is_physics_handle_valid():
            current_values = self._backend_utils.move_data(
                self._physics_view.get_masses().reshape(self._count), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            return current_values[indices]
        else:
            masses = np.zeros(shape=indices.shape[0], dtype=np.float32)
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._mass_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.MassAPI):
                        self._mass_apis[i] = UsdPhysics.MassAPI(self._prims[i])
                    else:
                        write_idx += 1
                        continue
                masses[write_idx] = self._mass_apis[i].GetMassAttr().Get()
                write_idx += 1
            masses = self._backend_utils.convert(masses, dtype="float32", device=self._device, indexed=True)
            return masses

    def get_inv_masses(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Get rigid body inverse masses of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            rigid body inverse masses of prims in the view. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # get all rigid body inverse masses. Returned shape is (5, 1) for the example: 5 envs
            >>> prims.get_inv_masses()
            [[0.001]
             [0.001]
             [0.001]
             [0.001]
             [0.001]]
            >>>
            >>> # get rigid body inverse masses for the first, middle and last of the 5 envs. Returned shape is (3, 1)
            >>> prims.get_inv_masses(indices=np.array([0, 2, 4]))
            [[0.001]
             [0.001]
             [0.001]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._backend_utils.move_data(self._physics_view.get_inv_masses(), self._device)
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            return current_values[indices]
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_inv_masses")
            return None

    def get_coms(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Get rigid body center of mass (COM) of bodies in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            rigid body center of mass positions and orientations of prims in the view.
            position shape is (M, 1, 3), orientation shape is (M, 1, 4).

        Example:

        .. code-block:: python

            >>> # get all rigid body center of mass.
            >>> # Returned shape is (5, 1, 3) for positions and (5, 1, 4) for orientations for the example: 5 envs
            >>> positions, orientations = prims.get_coms()
            >>> positions
            [[[0. 0. 0.]]
             [[0. 0. 0.]]
             [[0. 0. 0.]]
             [[0. 0. 0.]]
             [[0. 0. 0.]]]
            >>> orientations
            [[[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]]
            >>>
            >>> # get rigid body center of mass for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 1, 3) for positions and (3, 1, 4) for orientations
            >>> positions, orientations = prims.get_coms(indices=np.array([0, 2, 4]))
            >>> positions
            [[[0. 0. 0.]]
             [[0. 0. 0.]]
             [[0. 0. 0.]]]
            >>> orientations
            [[[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]
             [[1. 0. 0. 0.]]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_coms().reshape((self.count, 7)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            positions = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, 0:3
            ]
            orientations = self._backend_utils.xyzw2wxyz(
                current_values[self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, 3:7]
            )
            return positions, orientations
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_coms")
            return None

    def get_inertias(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Get rigid body inertias of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            rigid body inertias of prims in the view. Shape is (M, 9).

        Example:

        .. code-block:: python

            >>> # get all rigid body inertias. Returned shape is (5, 9) for the example: 5 envs
            >>> prims.get_inertias()
            [[166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]]
            >>>
            >>> # get rigid body inertias for the first, middle and last of the 5 envs. Returned shape is (3, 9)
            >>> prims.get_inertias(indices=np.array([0, 2, 4]))
            [[166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]
             [166.66667  0.  0.  0.  166.66667  0.  0.  0.  166.66667]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_inertias().reshape((self.count, 9)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            return current_values[indices]
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_inertias")
            return None

    def get_inv_inertias(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Get rigid body inverse inertias of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            rigid body inverse inertias of prims in the view. Shape is (M, 9).

        Example:

        .. code-block:: python

            >>> # get all rigid body inverse inertias. Returned shape is (5, 9) for the example: 5 envs
            >>> prims.get_inv_inertias()
            [[0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]]
            >>>
            >>> # get rigid body inverse inertias for the first, middle and last of the 5 envs. Returned shape is (3, 9)
            >>> prims.get_inv_inertias(indices=np.array([0, 2, 4]))
            [[0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]
             [0.006 0.    0.    0.    0.006 0.    0.    0.    0.006]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_inv_inertias().reshape((self.count, 9)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            return current_values[indices]
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_inv_inertias")
            return None

    def set_masses(
        self,
        masses: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set body masses for prims in the view.

        Args:
            masses: body masses for prims in kg. shape (M,).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the rigid body masses for all the rigid bodies to the indicated values.
            >>> prims.set_masses(np.full(num_envs, 10.0))
            >>>
            >>> # set the rigid body masses for the first, middle and last of the 5 envs
            >>> prims.set_masses(np.full(3, 10.0), indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            data = self._physics_view.get_masses().reshape(self.count)
            data = self._backend_utils.assign(self._backend_utils.move_data(masses, device="cpu"), data, indices)
            self._physics_view.set_masses(data, indices)
        else:
            read_idx = 0
            indices = self._backend_utils.to_list(indices)
            masses = self._backend_utils.to_list(masses)
            for i in indices:
                if self._mass_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.MassAPI):
                        self._mass_apis[i] = UsdPhysics.MassAPI(self._prims[i])
                    else:
                        self._mass_apis[i] = UsdPhysics.MassAPI.Apply(self._prims[i])
                self._mass_apis[i].GetMassAttr().Set(masses[read_idx])
                read_idx += 1
            return

    def set_inertias(
        self,
        values: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set rigid body inertias for prims in the view.

        Args:
            values: body inertias for prims in the view. shape (M, 1, 9).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the rigid body inertias for all the rigid bodies to the specified values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> inertias = np.tile(np.array([0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]), (num_envs, 1))
            >>> prims.set_inertias(inertias)
            >>>
            >>> # set the rigid body inertias for the first, middle and last of the 5 envs
            >>> inertias = np.tile(np.array([0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]), (3, 1))
            >>> prims.set_inertias(inertias, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            data = self._physics_view.get_inertias()
            data = self._backend_utils.assign(self._backend_utils.move_data(values, device="cpu"), data, indices)
            self._physics_view.set_inertias(data, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_inertias")

    def set_coms(
        self,
        positions: np.ndarray | torch.Tensor | wp.array = None,
        orientations: np.ndarray | torch.Tensor | wp.array = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set body center of mass (COM) positions and orientations for bodies in the view.

        Args:
            positions: body center of mass positions for bodies in the view. shape (M, 1, 3).
            orientations: body center of mass orientations for bodies in the view. shape (M, 1, 4).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the center of mass for all the rigid bodies to the specified values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> positions = np.tile(np.array([0.01, 0.02, 0.03]), (num_envs, 1, 1))
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1, 1))
            >>> prims.set_coms(positions, orientations)
            >>>
            >>> # set the rigid bodies center of mass for the first, middle and last of the 5 envs
            >>> positions = np.tile(np.array([0.01, 0.02, 0.03]), (3, 1, 1))
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1, 1))
            >>> prims.set_coms(positions, orientations, indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            coms = self._physics_view.get_coms().reshape((self.count, 7))
            if positions is not None:
                if self._backend == "warp":
                    coms = self._backend_utils.assign(
                        self._backend_utils.move_data(positions, device="cpu"),
                        coms,
                        [indices, wp.array([0, 1, 2], dtype=wp.int32, device="cpu")],
                    )
                else:
                    coms[self._backend_utils.expand_dims(indices, 1), 0:3] = self._backend_utils.move_data(
                        positions, device="cpu"
                    )
            if orientations is not None:
                if self._backend == "warp":
                    coms = self._backend_utils.assign(
                        self._backend_utils.move_data(self._backend_utils.wxyz2xyzw(orientations), device="cpu"),
                        coms,
                        [indices, wp.array([3, 4, 5, 6], dtype=wp.int32, device="cpu")],
                    )
                else:
                    coms[self._backend_utils.expand_dims(indices, 1), 3:7] = self._backend_utils.move_data(
                        self._backend_utils.wxyz2xyzw(orientations), device="cpu"
                    )
            self._physics_view.set_coms(coms, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_coms")

    def set_densities(
        self,
        densities: np.ndarray | torch.Tensor | wp.array | None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set densities of prims in the view.

        Args:
            densities: density in kg/m^3 specified for each prim in the view.
                shape is (M,). Defaults to None.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set all rigid body densities to the specified values
            >>> prims.set_densities(np.full(num_envs, 0.9))
            >>>
            >>> # set rigid body densities for the first, middle and last of the 5 envs
            >>> prims.set_densities(np.full(3, 0.9), indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        densities = self._backend_utils.to_list(densities)
        for i in indices:
            if self._mass_apis[i] is None:
                if self._prims[i].HasAPI(UsdPhysics.MassAPI):
                    self._mass_apis[i] = UsdPhysics.MassAPI(self._prims[i])
                else:
                    self._mass_apis[i] = UsdPhysics.MassAPI.Apply(self._prims[i])
            self._mass_apis[i].GetDensityAttr().Set(densities[read_idx])
            read_idx += 1
        return

    def get_densities(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get densities of prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)

        Returns:
            densities of prims in the view in kg/m^3. shape (M,).

        Example:

        .. code-block:: python

            >>> # get all rigid body densities. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_densities()
            [0. 0. 0. 0. 0.]
            >>>
            >>> # get rigid body densities for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_densities(indices=np.array([0, 2, 4]))
            [0. 0. 0.]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        densities = np.zeros(shape=(indices.shape[0]), dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._mass_apis[i] is None:
                if self._prims[i].HasAPI(UsdPhysics.MassAPI):
                    self._mass_apis[i] = UsdPhysics.MassAPI(self._prims[i])
                else:
                    write_idx += 1
                    continue
            densities[write_idx] = self._mass_apis[i].GetDensityAttr().Get()
            write_idx += 1
        densities = self._backend_utils.convert(densities, dtype="float32", device=self._device, indexed=True)
        return densities

    def set_sleep_thresholds(
        self,
        thresholds: np.ndarray | torch.Tensor | wp.array | None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set sleep thresholds of prims in the view.

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details

        Args:
            thresholds: Mass-normalized kinetic energy threshold below which
                an actor may go to sleep. Range: [0, inf)
                Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
                Units: distance^2 / second^2. shape (M,).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set all rigid body densities to the specified values
            >>> prims.set_sleep_thresholds(np.full(num_envs, 1e-5))
            >>>
            >>> # set rigid body densities for the first, middle and last of the 5 envs
            >>> prims.set_sleep_thresholds(np.full(3, 1e-5), indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        thresholds = self._backend_utils.to_list(thresholds)
        for i in indices:
            if self._physx_rigid_body_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                    rigid_api = PhysxSchema.PhysxRigidBodyAPI(self._prims[i])
                else:
                    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(self._prims[i])
                self._physx_rigid_body_apis[i] = rigid_api
            self._physx_rigid_body_apis[i].GetSleepThresholdAttr().Set(thresholds[read_idx])
            read_idx += 1
        return

    def get_sleep_thresholds(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get sleep thresholds of prims in the view.

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)

        Returns:
            Mass-normalized kinetic energy threshold below which
            an actor may go to sleep. Range: [0, inf). Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
            Units: distance^2 / second^2. shape (M,).

        Example:

        .. code-block:: python

            >>> # get all sleep threshold. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_sleep_thresholds()
            [5.e-05 5.e-05 5.e-05 5.e-05 5.e-05]
            >>>
            >>> # get sleep threshold for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_sleep_thresholds(indices=np.array([0, 2, 4]))
            [5.e-05 5.e-05 5.e-05]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        thresholds = np.zeros(indices.shape[0], dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._physx_rigid_body_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                    self._physx_rigid_body_apis[i] = PhysxSchema.PhysxRigidBodyAPI(self._prims[i])
                else:
                    write_idx += 1
                    continue
            thresholds[write_idx] = self._physx_rigid_body_apis[i].GetSleepThresholdAttr().Get()
            write_idx += 1
        thresholds = self._backend_utils.convert(thresholds, dtype="float32", device=self._device, indexed=True)
        return thresholds

    def enable_rigid_body_physics(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Enable rigid body physics (enabled by default).

        When enabled, the objects will be moved by external forces such as gravity and collisions

        Args:
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # enable the rigid body dynamics for all rigid bodies
            >>> prims.enable_rigid_body_physics()
            >>>
            >>> # enable the rigid body dynamics for the first, middle and last of the 5 envs
            >>> prims.enable_rigid_body_physics(indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            data = self._physics_view.get_disable_simulations().reshape(self._count)
            data = self._backend_utils.assign(
                self._backend_utils.create_tensor_from_list([False] * indices.shape[0], dtype="uint8"), data, indices
            )
            self._physics_view.set_disable_simulations(data, indices)
        else:
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                self._rigid_body_apis[i].GetRigidBodyEnabledAttr().Set(True)
            return

    def disable_rigid_body_physics(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Disable rigid body physics (enabled by default).

        When disabled, the objects will not be moved by external forces such as gravity and collisions

        Args:
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # enable the rigid body dynamics for all rigid bodies
            >>> prims.disable_rigid_body_physics()
            >>>
            >>> # enable the rigid body dynamics for the first, middle and last of the 5 envs
            >>> prims.disable_rigid_body_physics(indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            data = self._physics_view.get_disable_simulations().reshape(self._count)
            data = self._backend_utils.assign(
                self._backend_utils.create_tensor_from_list([True] * indices.shape[0], dtype="uint8"), data, indices
            )
            self._physics_view.set_disable_simulations(data, indices)
        else:
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
                    self._rigid_body_apis[i] = rigid_api
                self._rigid_body_apis[i].GetRigidBodyEnabledAttr().Set(False)
            return

    def enable_gravities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Enable gravity on rigid bodies (enabled by default).

        Args:
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # enable the gravity for all rigid bodies
            >>> prims.enable_gravities()
            >>>
            >>> # enable the rigid body gravity for the first, middle and last of the 5 envs
            >>> prims.enable_gravities(indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self.is_physics_handle_valid():
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            data = self._physics_view.get_disable_gravities().reshape(self._count)
            data = self._backend_utils.assign(
                self._backend_utils.create_tensor_from_list([False] * indices.shape[0], dtype="uint8"), data, indices
            )
            self._physics_view.set_disable_gravities(data, indices)
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._physx_rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                        rigid_api = PhysxSchema.PhysxRigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(self._prims[i])
                    self._physx_rigid_body_apis[i] = rigid_api
                self._physx_rigid_body_apis[i].GetDisableGravityAttr().Set(False)

    def disable_gravities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Disable gravity on rigid bodies (enabled by default).

        Args:
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # disable the gravity for all rigid bodies
            >>> prims.disable_gravities()
            >>>
            >>> # disable the rigid body gravity for the first, middle and last of the 5 envs
            >>> prims.disable_gravities(indices=np.array([0, 2, 4]))
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if self.is_physics_handle_valid():
            data = self._physics_view.get_disable_gravities().reshape(self._count)
            data = self._backend_utils.assign(
                self._backend_utils.create_tensor_from_list([True] * indices.shape[0], dtype="uint8"), data, indices
            )
            self._physics_view.set_disable_gravities(data, indices)
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                if self._physx_rigid_body_apis[i] is None:
                    if self._prims[i].HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                        rigid_api = PhysxSchema.PhysxRigidBodyAPI(self._prims[i])
                    else:
                        rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(self._prims[i])
                    self._physx_rigid_body_apis[i] = rigid_api
                self._physx_rigid_body_apis[i].GetDisableGravityAttr().Set(True)
            return

    def set_default_state(
        self,
        positions: np.ndarray | torch.Tensor | wp.array | None = None,
        orientations: np.ndarray | torch.Tensor | wp.array | None = None,
        linear_velocities: np.ndarray | torch.Tensor | wp.array | None = None,
        angular_velocities: np.ndarray | torch.Tensor | wp.array | None = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set the default state (position, orientation and linear and angular velocities) of prims in the view,.

        that will be used after each reset.

        .. note::

            The default states will be set during post-reset (e.g., calling ``.post_reset()`` or ``world.reset()`` methods)

        Args:
            positions: default positions in the world frame of the prim. shape is (M, 3).
            orientations: default quaternion orientations in the world frame of the prims.
                quaternion is scalar-first (w, x, y, z). shape is (M, 4).
            linear_velocities: default linear velocities of each prim in the view
                (to be applied in the first frame and on resets).
                Shape is (M, 3).
            angular_velocities: default angular velocities of each prim in the view
                (to be applied in the first frame and on resets).
                Shape is (M, 3).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # configure default states for all prims
            >>> positions = np.zeros((num_envs, 3))
            >>> positions[:, 0] = np.arange(num_envs)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1))
            >>> linear_velocities = np.zeros((num_envs, 3))
            >>> angular_velocities = np.zeros((num_envs, 3))
            >>> prims.set_default_state(
            ...     positions=positions,
            ...     orientations=orientations,
            ...     linear_velocities=linear_velocities,
            ...     angular_velocities=angular_velocities
            ... )
            >>>
            >>> # set default states during post-reset
            >>> prims.post_reset()
        """
        XFormPrim.set_default_state(self, positions=positions, orientations=orientations)
        if self._non_root_link:
            return
        if indices is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if positions is not None:
            if indices is None:
                self._dynamics_default_state.positions = positions
            else:
                if self._backend == "warp":
                    self._dynamics_default_state.positions = self._backend_utils.assign(
                        positions, self._dynamics_default_state.positions, indices
                    )
                else:
                    self._dynamics_default_state.positions[indices] = positions
        if orientations is not None:
            if indices is None:
                self._dynamics_default_state.orientations = orientations
            else:
                if self._backend == "warp":
                    self._dynamics_default_state.orientations = self._backend_utils.assign(
                        orientations, self._dynamics_default_state.orientations, indices
                    )
                else:
                    self._dynamics_default_state.orientations[indices] = orientations
        if linear_velocities is not None:
            if indices is None:
                self._dynamics_default_state.linear_velocities = linear_velocities
            else:
                if self._backend == "warp":
                    self._dynamics_default_state.linear_velocities = self._backend_utils.assign(
                        linear_velocities, self._dynamics_default_state.linear_velocities, indices
                    )
                else:
                    self._dynamics_default_state.linear_velocities[indices] = linear_velocities
        if angular_velocities is not None:
            if indices is None:
                self._dynamics_default_state.angular_velocities = angular_velocities
            else:
                if self._backend == "warp":
                    self._dynamics_default_state.angular_velocities = self._backend_utils.assign(
                        angular_velocities, self._dynamics_default_state.angular_velocities, indices
                    )
                else:
                    self._dynamics_default_state.angular_velocities[indices] = angular_velocities
        return

    def get_default_state(self) -> DynamicsViewState:
        """Default state (position, orientation and linear and angular velocities) of prims in the view,.

        that is used after each reset.

        Returns:
            The default state of the prims that is used after each reset.

        Example:

        .. code-block:: python

            >>> state = prims.get_default_state()
            <isaacsim.core.utils.types.DynamicsViewState object at 0x7f184e555480>
            >>> state
            >>> state.positions
            [[ 1.4999989e+00 -7.4999851e-01 -1.5118626e-07]
             [ 1.4999989e+00  7.5000149e-01 -2.5988294e-07]
             [-1.0017333e-06 -7.4999845e-01  7.6070329e-08]
             [-9.5906785e-07  7.5000149e-01  1.0593490e-07]
             [-1.5000011e+00 -7.4999851e-01  1.9655154e-07]]
            >>> state.orientations
            [[ 9.9999994e-01 -8.8168377e-07 -4.1946004e-07 -1.5067183e-08]
             [ 9.9999994e-01 -8.8691013e-07 -4.2665880e-07 -2.7188951e-09]
             [ 1.0000000e+00 -9.5171310e-07 -2.2615541e-07  5.5922797e-08]
             [ 1.0000000e+00 -8.9923367e-07 -1.4408238e-07  1.3476099e-08]
             [ 1.0000000e+00 -7.9806580e-07 -1.3064776e-07  5.3154917e-08]]
            >>> state.linear_velocities
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
            >>> state.angular_velocities
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        return self._dynamics_default_state

    def get_current_dynamic_state(self) -> DynamicsViewState:
        """Current rigid body states (position, orientation and linear and angular velocities).

        Returns:
            The dynamic state of the rigid bodies

        Example:

        .. code-block:: python

            >>> # for the example the rigid bodies are in free fall
            >>> state = prims.get_current_dynamic_state()
            <isaacsim.core.utils.types.DynamicsViewState object at 0x7f182bd72590>
            >>> state
            >>> state.positions
            [[   1.5       -0.75    -207.76808]
             [   1.5        0.75    -207.76808]
             [   0.        -0.75    -207.76808]
             [   0.         0.75    -207.76808]
             [  -1.5       -0.75    -207.76808]]
            >>> state.orientations
            [[1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]]
            >>> state.linear_velocities
            [[  0.         0.       -63.765312]
             [  0.         0.       -63.765312]
             [  0.         0.       -63.765312]
             [  0.         0.       -63.765312]
             [  0.         0.       -63.765312]]
            >>> state.angular_velocities
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        positions, orientations = self.get_world_poses()
        return DynamicsViewState(
            positions=positions,
            orientations=orientations,
            linear_velocities=self.get_linear_velocities(),
            angular_velocities=self.get_angular_velocities(),
        )

    def get_net_contact_forces(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Return the net contact forces on prims.

        .. note::

            This method requires that the contact forces of the prims in the view be tracked by defining
            the ``track_contact_forces`` argument to True (default to False) during view creation

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces.
                If the default value is used then the forces are in fact contact impulses

        Returns:
            Net contact forces of the prims with shape (M,3).

        Example:

        .. code-block:: python

            >>> # get the net contact force on all rigid bodies. Returned shape is (5, 3).
            >>> # For the example the view was instantiated with the extra parameter: track_contact_forces=True
            >>> prims.get_net_contact_forces()
            [[2.1967362e-05 0.0000000e+00 1.6349771e+02]
             [2.1967124e-05 0.0000000e+00 1.6349591e+02]
             [2.1967891e-05 0.0000000e+00 1.6350165e+02]
             [2.1967257e-05 0.0000000e+00 1.6349693e+02]
             [2.1966895e-05 0.0000000e+00 1.6349425e+02]]
            >>>
            >>> # get the net contact force on the rigid bodies for the first, middle and last of the 5 envs
            >>> prims.get_net_contact_forces(indices=np.array([0, 2, 4]))
            [[2.1967362e-05 0.0000000e+00 1.6349771e+02]
             [2.1967891e-05 0.0000000e+00 1.6350165e+02]
             [2.1966895e-05 0.0000000e+00 1.6349425e+02]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if self._track_contact_forces:
            return self._contact_view.get_net_contact_forces(indices, clone, dt)
        else:
            carb.log_warn(
                "contact forces cannot be retrieved with this API unless the RigidPrim is initialized with track_contact_forces= True."
            )
            return None

    def get_contact_force_matrix(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """Return the contact forces between the prims in the view and the filter prims.

        E.g., a matrix of dimension ``(self.count, _contact_view.num_filters, 3)`` where ``_contact_view.num_filters``
        is determined according to the ``contact_filter_prim_paths_expr`` parameter

        .. note::

            This method requires that the contact forces of the prims in the view be tracked by defining
            the ``contact_filter_prim_paths_expr`` argument to a list of the prim paths from which to generate
            the information and the ``max_contact_count`` argument be greater than 0 during view creation

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces.
                If the default value is used then the forces are in fact contact impulses

        Returns:
            Net contact forces of the prims with shape (M, self._contact_view.num_filters, 3).

        Example:

        .. code-block:: python

            >>> # for the example, the cubes are on top of each other. The view was instantiated with the following
            >>> # extra parameters: contact_filter_prim_paths_expr=["/World/envs/env_2/Xform"], max_contact_count=10
            >>> # This indicates that only contacts with the middle cube will be reported
            >>> prims.get_contact_force_matrix()
            [[[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
             [[-7.8665102e-03  8.3034458e-03 -4.9063504e+02]]
             [[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
             [[ 5.2445102e-03 -5.5358098e-03  3.2710065e+02]]
             [[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_contact_force_matrix(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_contact_force_matrix. Initialize the RigidPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None

    def get_contact_force_data(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> tuple[
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
    ]:
        """Get more detailed contact information between the prims in the view and the filter prims.

        Specifically, this method provides individual contact normals, contact points, contact separations as well as
        contact forces for each pair (the sum of which equals the forces that the ``get_contact_force_matrix``
        method provides as the force aggregate of a pair)

        Given to the dynamic nature of collision between bodies, this method will provide buffers of contact data which
        are arranged sequentially for each pair. The starting index and the number of contact data points for each pair
        in this stream can be realized from pair_contacts_start_indices, and pair_contacts_count tensors.
        They both have a dimension of ``(num_shapes, _contact_view.num_filters)`` where ``_contact_view.num_filters``
        is determined according to the ``contact_filter_prim_paths_expr`` parameter

        .. note::

            This method requires that the contact forces of the prims in the view be tracked by defining
            the ``contact_filter_prim_paths_expr`` argument to a list of the prim paths from which to generate
            the information and the ``max_contact_count`` argument be greater than 0 during view creation

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces.
                If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for normal forces with shape (max_contact_count, 1), points with shape
            (max_contact_count, 3), normals with shape (max_contact_count, 3), and distances with shape
            (max_contact_count, 1), as well as two tensors with shape (M, self.num_filters) to indicate the starting
            index and the number of contact data points per pair in the aforementioned buffers.

        Example:

        .. code-block:: python

            >>> # for the example, the cubes are on top of each other. The view was instantiated with the following
            >>> # extra parameters: contact_filter_prim_paths_expr=["/World/envs/env_2/Xform"], max_contact_count=10
            >>> # This indicates that only contacts with the middle cube will be reported
            >>> data = prims.get_contact_force_data()
            >>> data[0]  # normal forces
            [[-156.449   ]
             [ -81.736336]
             [-169.73076 ]
             [ -82.397804]
             [ 110.11985 ]
             [  59.646057]
             [  98.660545]
             [  58.43006 ]
             [   0.      ]
             [   0.      ]]
            >>> data[1]  # points
            [[-0.50145745  0.49872556  0.7056795 ]
             [-0.50184476 -0.5010655   0.7057198 ]
             [ 0.49793154 -0.50147027  0.70576656]
             [ 0.4983363   0.49833822  0.70572615]
             [ 0.49818155 -0.5016888   1.7058725 ]
             [ 0.49856627  0.4913648   1.7058672 ]
             [-0.49732465  0.4915302   1.705814  ]
             [-0.49748957 -0.501303    1.7058194 ]
             [ 0.          0.          0.        ]
             [ 0.          0.          0.        ]]
            >>> data[2]  # normals
            [[ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 1.6479074e-05 -1.6995813e-05  1.0000000e+00]
             [ 0.0000000e+00  0.0000000e+00  0.0000000e+00]
             [ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
            >>> data[3]  # distances
            [[ 6.3175990e-05]
             [ 5.8271162e-06]
             [-5.7399273e-05]
             [-1.0989098e-08]
             [ 1.6338757e-04]
             [ 1.4112510e-04]
             [ 7.1585178e-05]
             [ 9.3835908e-05]
             [ 0.0000000e+00]
             [ 0.0000000e+00]]
            >>> data[4]  # pair contacts count
            [[0]
             [4]
             [0]
             [4]
             [0]]
            >>> data[5]  # start indices of pair contacts
            [[0]
             [0]
             [4]
             [4]
             [8]]
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_contact_force_data(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_contact_force_data. Initialize the RigidPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None

    def get_friction_data(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> tuple[
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
        np.ndarray | torch.Tensor | wp.indexedarray,
    ]:
        """Get friction data between the prims in the view and the filter prims. Specifically, this method provides frictional contact forces,.

        and points. The data in reported for number of anchor points that includes tangential forces in a single tangent direction to contact normal.
        Given to the dynamic nature of collision between bodies, this method will provide buffers of friction data arranged sequentially for each pair.
        The starting index and the number of contact data points for each pair in this stream can be realized from pair_contacts_start_indices,
        and pair_contacts_count tensors. They both have a dimension of (self.num_shapes, self.num_filters) where filter_count is determined
        according to the filter_paths_expr parameter.

        Args:
            indices: indicies to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces.
                If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for tangential forces per patch (at number of anchor points, each in a single directions)
            with shape (max_contact_count, 3), points with shape (max_contact_count, 3),
            as well as two tensors with shape (M, self.num_filters) to indicate the starting index and the number of
            contact data points per pair in the aforementioned buffers.
        """
        if not self._is_valid:
            raise Exception(f"prim view {self._regex_prim_paths} is not a valid view")
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_friction_data(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_friction_data. Initialize the RigidPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API.

        .. note::

            For this particular class, calling this method will do nothing

        Args:
            physics_sim_view: current physics simulation view.

        Example:

        .. code-block:: python

            >>> prims.initialize()
        """
        if not self.is_physics_handle_valid():
            self._on_physics_ready(None)
        return

    def _on_physics_ready(self, event: object) -> None:
        """Initialize the physics simulation view for rigid body operations.

        Creates a rigid body view from the simulation manager, sets up default states, and initializes
        contact view if contact forces are being tracked.

        Args:
            event: Physics ready event (unused but required for event handling).
        """
        XFormPrim._on_physics_ready(self, event)
        simulation_view = SimulationManager.get_physics_sim_view()
        self._physics_view = simulation_view.create_rigid_body_view(
            [regular_expression.replace(".*", "*") for regular_expression in self._regex_prim_paths]
        )
        self._num_shapes = self._physics_view.max_shapes
        if not self._non_root_link:
            self._dynamics_default_state.positions = self._default_state.positions
            self._dynamics_default_state.orientations = self._default_state.orientations
            if self._dynamics_default_state.linear_velocities is None:
                linear_velocities = self.get_linear_velocities()
                self._dynamics_default_state.linear_velocities = (
                    linear_velocities.data if self._backend == "warp" else linear_velocities
                )
            if self._dynamics_default_state.angular_velocities is None:
                angular_velocities = self.get_angular_velocities()
                self._dynamics_default_state.angular_velocities = (
                    angular_velocities.data if self._backend == "warp" else angular_velocities
                )
        carb.log_info(f"Rigid Prim View Device: {self._device}")
        if self._track_contact_forces:
            self._contact_view.initialize(simulation_view)

    def _apply_rigid_body_apis(self, prepare_contact_reporter: bool = False) -> None:
        """Apply Rigid Body API to all prims in the view.

        Ensures each prim has the UsdPhysics.RigidBodyAPI applied and optionally configures
        contact reporting by disabling sleep threshold.

        Args:
            prepare_contact_reporter: Whether to configure prims for contact reporting by setting
                sleep threshold to 0.
        """
        for i in range(self.count):
            if self._prims[i].HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_api = UsdPhysics.RigidBodyAPI(self._prims[i])
            else:
                rigid_api = UsdPhysics.RigidBodyAPI.Apply(self._prims[i])
            self._rigid_body_apis[i] = rigid_api

            if prepare_contact_reporter:
                # disable sleeping, because sleeping bodies don't get contact reports
                rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(self._prims[i])
                rb_api.CreateSleepThresholdAttr().Set(0)
        return

    def _on_post_reset(self, event: object) -> None:
        """Reset the rigid prims to their default states (positions, orientations and linear and angular velocities).

        Args:
            event: Post-reset event (unused but required for event handling).

        Example:

        .. code-block:: python

            >>> prims.post_reset()
        """
        XFormPrim._on_post_reset(self, event)
        if not self._non_root_link:
            self.set_velocities(
                velocities=self._backend_utils.tensor_cat(
                    [self._dynamics_default_state.linear_velocities, self._dynamics_default_state.angular_velocities],
                    dim=-1,
                    device=self._device,
                )
            )
        return
