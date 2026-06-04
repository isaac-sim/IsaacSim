# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Rigid body view for Newton physics tensor interface."""

from __future__ import annotations

from typing import Any

import carb
import warp as wp

from .kernels import *
from .kernels import (
    apply_body_forces_at_position,
    cache_body_com,
    get_body_com_position_only,
    update_body_inv_inertia,
    update_body_inv_mass,
)
from .tensor_utils import convert_to_warp, wrap_input_tensor

# Import tensor types from omni.physics.tensors for compatibility
try:
    from omni.physics.tensors import float32, uint8, uint32
except ImportError:
    float32 = wp.float32
    uint8 = wp.uint8
    uint32 = wp.uint32

copy_data = True


class NewtonRigidBodyView:
    """View for rigid bodies in Newton physics simulation.

    Provides tensor-based access to rigid body properties like transforms,
    velocities, masses, etc.

    Args:
        backend: RigidBodySet backend instance from NewtonSimView.
        frontend: Tensor framework frontend.
    """

    def __init__(self, backend: Any, frontend: Any) -> None:
        self._backend = backend
        self._frontend = frontend
        self._newton_stage = backend.newton_stage
        self._model = backend.model
        self._sim_timestamp = 0

        # Create tensors for caching data
        self._transforms, self._transforms_desc = self._frontend.create_tensor((self.count, 7), float32)
        self._velocities, self._velocities_desc = self._frontend.create_tensor((self.count, 6), float32)
        self._accelerations, self._accelerations_desc = self._frontend.create_tensor((self.count, 6), float32)
        self._masses, self._masses_desc = self._frontend.create_tensor((self.count, 1), float32)
        self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor((self.count, 1), float32)
        self._coms, self._coms_desc = self._frontend.create_tensor((self.count, 7), float32)
        self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, 9), float32)
        self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor((self.count, 9), float32)
        self._disable_simulations, self._disable_simulations_desc = self._frontend.create_tensor((self.count, 1), uint8)
        self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor((self.count, 1), uint8)

        # Initialize COM cache with identity quaternions [qx=0, qy=0, qz=0, qw=1] for orientation part
        coms_warp = self._convert_to_warp(self._coms)
        coms_warp.fill_(0.0)  # type: ignore[union-attr]
        # Set qw=1 for identity quaternion (index 6 is qw in [x, y, z, qx, qy, qz, qw])
        if self.count > 0:

            coms_np = coms_warp.numpy()  # type: ignore[union-attr]
            coms_np[:, 6] = 1.0
            coms_warp.assign(coms_np)  # type: ignore[union-attr]

    def _wrap_input_tensor(self, tensor: Any, dtype: "wp.dtype | None" = None) -> "wp.array | None":  # type: ignore[name-defined]
        """Helper to wrap an input tensor as a warp array for kernel input.

        Args:
            tensor: Input tensor to wrap.
            dtype: Target dtype.

        Returns:
            Warp array or None.
        """
        return wrap_input_tensor(tensor, self._frontend.device, dtype)

    def _convert_to_warp(self, tensor: Any) -> wp.array | None:
        """Helper to convert tensor to warp array for kernel output.

        Args:
            tensor: Tensor to convert.

        Returns:
            Warp array or None.
        """
        return convert_to_warp(tensor, self._frontend.device)

    @property
    def count(self) -> int:
        """Number of rigid bodies in this view.

        Returns:
            The count of rigid bodies.
        """
        return self._backend.count

    @property
    def max_shapes(self) -> int:
        """Maximum number of shapes across all bodies.

        Returns:
            The maximum number of shapes.
        """
        return self._backend.max_shapes

    @property
    def body_paths(self) -> list[str]:
        """List of USD paths for the bodies.

        Returns:
            The USD paths for the bodies.
        """
        return self._backend.body_paths

    @property
    def body_names(self) -> list[str]:
        """List of names for the bodies.

        Returns:
            The names for the bodies.
        """
        return self._backend.body_names

    def update(self, dt: float) -> None:
        """Update simulation timestamp.

        Args:
            dt: Time delta.
        """
        self._sim_timestamp += dt  # type: ignore[assignment]

    @carb.profiler.profile
    def get_transforms(self, copy: bool = copy_data) -> Any:
        """Get body transforms [position(3) + quaternion(4)].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 7).
        """
        state = self._newton_stage.state_0
        if copy:
            wp.launch(
                get_body_pose,
                dim=self._backend.count,
                inputs=[state.body_q, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._transforms)],
                device=str(self._frontend.device),
            )
            return self._transforms
        else:
            return wp.indexedarray(state.body_q, self._backend.body_indices)

    @carb.profiler.profile
    def get_velocities(self, copy: bool = copy_data) -> Any:
        """Get body velocities [linear(3) + angular(3)].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 6).
        """
        state = self._newton_stage.state_0
        if copy:
            wp.launch(
                get_body_velocity,
                dim=self._backend.count,
                inputs=[state.body_qd, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._velocities)],
                device=str(self._frontend.device),
            )
            return self._velocities
        else:
            return wp.indexedarray(state.body_qd, self._backend.body_indices)

    @carb.profiler.profile
    def get_accelerations(self, copy: bool = copy_data) -> Any:
        """Get body accelerations [linear(3) + angular(3)].

        Requires ``body_qdd`` to be allocated on the state via
        ``model.request_state_attributes("body_qdd")`` before state creation.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 6), or None if body_qdd is not available.
        """
        state = self._newton_stage.state_0
        if state.body_qdd is None:
            carb.log_warn("body_qdd not allocated; call model.request_state_attributes('body_qdd') before play")
            return None
        if copy:
            wp.launch(
                get_body_velocity,
                dim=self._backend.count,
                inputs=[state.body_qdd, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._accelerations)],
                device=str(self._frontend.device),
            )
            return self._accelerations
        else:
            return wp.indexedarray(state.body_qdd, self._backend.body_indices)

    def get_masses(self, copy: bool = copy_data) -> Any:
        """Get body masses.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count,).
        """
        if copy:
            wp.launch(
                get_body_mass,
                dim=self._backend.count,
                inputs=[self._model.body_mass, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._masses)],
                device=str(self._frontend.device),
            )
            return self._masses
        else:
            return wp.indexedarray(self._model.body_mass, self._backend.body_indices)

    def get_inv_masses(self, copy: bool = copy_data) -> Any:
        """Get body inverse masses.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count,).
        """
        if copy:
            wp.launch(
                get_body_inv_mass,
                dim=self._backend.count,
                inputs=[self._model.body_inv_mass, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._inv_masses)],
                device=str(self._frontend.device),
            )
            return self._inv_masses
        else:
            return wp.indexedarray(self._model.body_inv_mass, self._backend.body_indices)

    def get_coms(self, copy: bool = copy_data) -> Any:
        """Get body centers of mass [position(3) + orientation(4)].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 7). Newton stores COM as offset in body frame.
            Position is read from Newton, orientation is cached.
        """
        # Read positions from Newton's body_com (only first 3 elements)
        wp.launch(
            get_body_com_position_only,
            dim=self._backend.count,
            inputs=[self._model.body_com, self._backend.body_indices],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )
        # Orientation (elements 3-6) remains from cache, updated by set_coms
        return self._coms

    def get_inertias(self, copy: bool = copy_data) -> Any:
        """Get body inertias as flattened 3x3 matrices.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 9).
        """
        if copy:
            wp.launch(
                get_body_inertia,
                dim=self._backend.count,
                inputs=[self._model.body_inertia, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._inertias)],
                device=str(self._frontend.device),
            )
            return self._inertias
        else:
            # Inertia is a mat33, need to flatten it
            wp.launch(
                get_body_inertia,
                dim=self._backend.count,
                inputs=[self._model.body_inertia, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._inertias)],
                device=str(self._frontend.device),
            )
            return self._inertias

    def get_inv_inertias(self, copy: bool = copy_data) -> Any:
        """Get body inverse inertias as flattened 3x3 matrices.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 9).
        """
        if copy:
            wp.launch(
                get_body_inv_inertia,
                dim=self._backend.count,
                inputs=[self._model.body_inv_inertia, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._inv_inertias)],
                device=str(self._frontend.device),
            )
            return self._inv_inertias
        else:
            wp.launch(
                get_body_inv_inertia,
                dim=self._backend.count,
                inputs=[self._model.body_inv_inertia, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._inv_inertias)],
                device=str(self._frontend.device),
            )
            return self._inv_inertias

    @carb.profiler.profile
    def set_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body transforms.

        For free rigid bodies, this also updates their FREE joint coordinates.

        Args:
            data: Transform data to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        state = self._newton_stage.state_0

        # Set body_q (body transforms in world space)
        wp.launch(
            set_body_pose,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[state.body_q],
            device=str(self._frontend.device),
        )

        # For free rigid bodies, also update the joint coordinates
        # Free bodies have a FREE joint connecting them to world
        # The joint's coordinates need to match the body transform
        wp.launch(
            update_free_joint_coords_from_body_q,
            dim=indices.shape[0],
            inputs=[
                state.body_q,
                self._wrap_input_tensor(indices),
                self._backend.body_indices,
                self._model.joint_child,
                self._model.joint_type,
                self._model.joint_q_start,
            ],
            outputs=[state.joint_q],
            device=str(self._frontend.device),
        )

    @carb.profiler.profile
    def set_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body velocities.

        Args:
            data: Velocity data to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        state = self._newton_stage.state_0
        wp.launch(
            set_body_velocity,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[state.body_qd],
            device=str(self._frontend.device),
        )

    @carb.profiler.profile
    def set_masses(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body masses.

        Args:
            data: Mass data to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        wp.launch(
            set_body_mass,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[self._model.body_mass],
            device=str(self._frontend.device),
        )
        # Update inverse mass when mass changes
        wp.launch(
            update_body_inv_mass,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
                self._model.body_mass,
            ],
            outputs=[self._model.body_inv_mass],
            device=str(self._frontend.device),
        )

    @carb.profiler.profile
    def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body centers of mass.

        Position is written to Newton's body_com. Orientation is cached in _coms buffer
        since Newton doesn't support COM orientation.

        Args:
            data: COM data to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        # Write position to Newton's body_com
        wp.launch(
            set_body_com,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[self._model.body_com],
            device=str(self._frontend.device),
        )
        # Cache the full COM data (position + orientation) for later retrieval
        wp.launch(
            cache_body_com,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )

    @carb.profiler.profile
    def set_inertias(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body inertias.

        Args:
            data: Inertia data to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        wp.launch(
            set_body_inertia,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
            ],
            outputs=[self._model.body_inertia],
            device=str(self._frontend.device),
        )
        # Update inverse inertia when inertia changes
        wp.launch(
            update_body_inv_inertia,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
                self._model.body_inertia,
            ],
            outputs=[self._model.body_inv_inertia],
            device=str(self._frontend.device),
        )

    def get_disable_simulations(self, copy: bool = copy_data) -> Any:
        """Get disable simulation flags for bodies.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 1) with uint8 values (0=enabled, 1=disabled).
            Newton doesn't have a direct equivalent; returns zeros (all enabled).
        """
        # Newton doesn't have a disable_simulation flag per body
        # Always return zeros (all bodies are enabled)
        return self._disable_simulations

    def set_disable_simulations(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set disable simulation flags for bodies.

        Newton doesn't support disabling individual bodies; this is a no-op.

        Args:
            data: Disable simulation flags to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        # Newton doesn't support disabling individual bodies
        # Log a warning if trying to disable bodies
        carb.log_warn(
            "Newton physics does not support disabling individual rigid bodies; set_disable_simulations is a no-op"
        )

    def get_disable_gravities(self, copy: bool = copy_data) -> Any:
        """Get disable gravity flags for bodies.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 1) with uint8 values (0=gravity enabled, 1=gravity disabled).
            Newton doesn't have per-body gravity flags; returns zeros (all enabled).
        """
        # Newton doesn't have per-body gravity disable flags
        # Always return zeros (gravity enabled for all bodies)
        return self._disable_gravities

    def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set disable gravity flags for bodies.

        Newton doesn't support per-body gravity disabling; this is a no-op.

        Args:
            data: Disable gravity flags to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.
        """
        # Newton doesn't support per-body gravity disable
        # Log a warning if trying to disable gravity
        carb.log_warn("Newton physics does not support per-body gravity disabling; set_disable_gravities is a no-op")

    def apply_forces(
        self,
        force_data: Any,
        indices: Any | None = None,
        is_global: bool = True,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply forces to rigid bodies (deprecated, use apply_forces_and_torques_at_position).

        Args:
            force_data: Forces to apply, shape (count, 3).
            indices: Indices of bodies to apply forces to. If None, applies to all bodies.
            is_global: If True, forces are in global frame. If False, in body local frame.
            indices_mask: Optional mask for the indices.
        """
        if indices is None:
            indices = wp.arange(self.count, dtype=wp.int32, device=str(self._frontend.device))  # type: ignore[attr-defined]

        self.apply_forces_and_torques_at_position(force_data, None, None, indices, is_global, indices_mask)

    @carb.profiler.profile
    def apply_forces_and_torques_at_position(
        self,
        force_data: Any | None,
        torque_data: Any | None,
        position_data: Any | None,
        indices: Any,
        is_global: bool = True,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply forces and torques to rigid bodies at specified positions.

        This function provides flexible force application with the following capabilities:
        - Apply forces at body center or at specified positions
        - Apply torques directly
        - Work in global or local coordinate frames
        - When position is specified with force, automatically computes induced torque

        Args:
            force_data: Forces to apply, shape (count, 3). Can be None.
            torque_data: Torques to apply, shape (count, 3). Can be None.
            position_data: Positions where forces are applied, shape (count, 3). Can be None.
                If specified with force_data, the force is applied at this position
                relative to the body's center of mass, generating additional torque.
            indices: Indices of bodies to apply forces to.
            is_global: If True, force/torque/position are in global frame. If False, in body local frame.
            indices_mask: Optional mask for the indices.
        """
        # Validate inputs
        has_force = force_data is not None
        has_torque = torque_data is not None
        has_position = position_data is not None

        if not has_force and not has_torque:
            carb.log_warn("No force or torque data provided to apply_forces_and_torques_at_position")
            return

        if has_position and not has_force:
            carb.log_error("position_data requires force_data to be provided")
            return

        # Wrap input tensors
        force_tensor = self._wrap_input_tensor(force_data) if has_force else None
        torque_tensor = self._wrap_input_tensor(torque_data) if has_torque else None
        position_tensor = self._wrap_input_tensor(position_data) if has_position else None
        indices_tensor = self._wrap_input_tensor(indices)

        # Get Newton state
        state = self._newton_stage.state_0

        # Create dummy tensors for optional inputs
        if not has_force:
            force_tensor = wp.zeros((self.count, 3), dtype=wp.float32, device=str(self._frontend.device))
        if not has_torque:
            torque_tensor = wp.zeros((self.count, 3), dtype=wp.float32, device=str(self._frontend.device))
        if not has_position:
            position_tensor = wp.zeros((self.count, 3), dtype=wp.float32, device=str(self._frontend.device))

        # Apply forces to state
        wp.launch(
            apply_body_forces_at_position,
            dim=indices_tensor.shape[0],  # type: ignore[union-attr]
            inputs=[
                force_tensor,
                torque_tensor,
                position_tensor,
                indices_tensor,
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
                state.body_q,
                self._model.body_com,
                is_global,
                has_force,
                has_torque,
                has_position,
            ],
            outputs=[state.body_f],
            device=str(self._frontend.device),
        )

    def check(self) -> bool:
        """Check if the rigid body view is valid and has bodies.

        Returns:
            True if the view has a valid backend with at least one body.
        """
        return self._backend is not None and self.count > 0
