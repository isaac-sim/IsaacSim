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

"""Provide the Newton rigid-body tensor view."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from isaacsim.common.logging import Logger

_log = Logger("isaacsim.physics_engines.ovnewton.impl.tensors")


def _no_profile(fn: Callable[..., Any]) -> Callable[..., Any]:
    """Return a callable without adding profiling instrumentation.

    Args:
        fn: Function to preserve.

    Returns:
        The original callable.

    """
    return fn


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

# Import tensor types from isaacsim.physics.manager.impl.tensors for compatibility
try:
    from isaacsim.physics.manager.impl.tensors import float32, uint8, uint32
except ImportError:
    float32 = wp.float32
    uint8 = wp.uint8
    uint32 = wp.uint32

copy_data = True


class NewtonRigidBodyView:
    """Expose a set of Newton rigid bodies through the tensor API.

    The view maps model storage into frontend tensors and supports indexed reads and writes across the selected bodies.

    Args:
        backend: Backend selection containing rigid-body and model indices.
        frontend: Tensor frontend used to allocate and wrap returned values.

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
        coms_warp.fill_(0.0)
        # Set qw=1 for identity quaternion (index 6 is qw in [x, y, z, qx, qy, qz, qw])
        if self.count > 0:

            coms_np = coms_warp.numpy()
            coms_np[:, 6] = 1.0
            coms_warp.assign(coms_np)

    def _wrap_input_tensor(self, tensor: Any, dtype: wp.dtype | None = None) -> wp.array | None:
        """Wrap an input tensor as a Warp array for kernel input.

        Args:
            tensor: Input tensor to wrap.
            dtype: Target dtype.

        Returns:
            Warp array or None.

        """
        return wrap_input_tensor(tensor, self._frontend.device, dtype)

    def _convert_to_warp(self, tensor: Any) -> wp.array | None:
        """Convert a tensor to a Warp array for kernel output.

        Args:
            tensor: Tensor to convert.

        Returns:
            Warp array or None.

        """
        return convert_to_warp(tensor, self._frontend.device)

    @property
    def count(self) -> int:
        """Get the number of rigid bodies in this view.

        Returns:
            The count of rigid bodies.

        """
        return self._backend.count

    @property
    def max_shapes(self) -> int:
        """Get the maximum number of collision shapes on a selected body.

        Returns:
            The maximum number of shapes.

        """
        return self._backend.max_shapes

    @property
    def body_paths(self) -> list[str]:
        """Get the USD paths of the selected bodies.

        Returns:
            The USD paths for the bodies.

        """
        return self._backend.body_paths

    @property
    def body_names(self) -> list[str]:
        """Get the names of the selected bodies.

        Returns:
            The names for the bodies.

        """
        return self._backend.body_names

    def update(self, dt: float) -> None:
        """Advance the view timestamp.

        Args:
            dt: Timestamp increment in seconds.

        """
        self._sim_timestamp += dt

    @_no_profile
    def get_transforms(self, copy: bool = copy_data) -> Any:
        """Get body transforms as position and quaternion values.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton transforms.

        Returns:
            Values with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order.

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

    @_no_profile
    def get_velocities(self, copy: bool = copy_data) -> Any:
        """Get body linear and angular velocities.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton spatial
                vectors.

        Returns:
            Values with shape ``(count, 6)`` in linear-then-angular order.

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

    @_no_profile
    def get_accelerations(self, copy: bool = copy_data) -> Any:
        """Get a compatibility acceleration result populated from body velocities.

        Newton does not expose accelerations through this view. This method returns velocity values and is not
        registered as a supported acceleration operation for maximal-coordinate solvers.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton spatial
                velocities.

        Returns:
            Velocity values with shape ``(count, 6)`` in linear-then-angular order.

        """
        state = self._newton_stage.state_0
        if copy:
            wp.launch(
                get_body_velocity,
                dim=self._backend.count,
                inputs=[state.body_qd, self._backend.body_indices],
                outputs=[self._convert_to_warp(self._accelerations)],
                device=str(self._frontend.device),
            )
            return self._accelerations
        else:
            return wp.indexedarray(state.body_qd, self._backend.body_indices)

    def get_masses(self, copy: bool = copy_data) -> Any:
        """Get body masses.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton mass
                storage.

        Returns:
            Mass values with shape ``(count, 1)``.

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
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton
                inverse-mass storage.

        Returns:
            Inverse-mass values with shape ``(count, 1)``.

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
            copy: Compatibility parameter. The method always refreshes positions in and returns view-owned storage.

        Returns:
            Values with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order. Positions are body-frame offsets
            read from Newton, while orientations are view-owned cached values.

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
            copy: Compatibility parameter. The method always refreshes and returns view-owned storage.

        Returns:
            Inertia values with shape ``(count, 9)`` in row-major order.

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
            copy: Compatibility parameter. The method always refreshes and returns view-owned storage.

        Returns:
            Inverse-inertia values with shape ``(count, 9)`` in row-major order.

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

    @_no_profile
    def set_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body transforms.

        For free rigid bodies, this also updates their FREE joint coordinates.

        Args:
            data: Transforms with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order.
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

    @_no_profile
    def set_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body velocities.

        Args:
            data: Velocities with shape ``(count, 6)`` in linear-then-angular order.
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

    @_no_profile
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

    @_no_profile
    def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set body center-of-mass data.

        Newton receives the position components as body-frame offsets. The view-owned cache stores the orientation
        components for subsequent reads.

        Args:
            data: Center-of-mass values with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order.
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
            ],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )

    @_no_profile
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
        """Get the compatibility buffer for per-body simulation-disable flags.

        Newton does not expose per-body simulation-disable state, so the returned buffer is not populated from the
        model.

        Args:
            copy: Compatibility parameter. The method always returns the view-owned buffer without populating it from
                Newton.

        Returns:
            Frontend tensor of shape ``(count, 1)``.

        """
        return self._disable_simulations

    def set_disable_simulations(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Accept per-body simulation-disable flags without modifying Newton.

        Newton doesn't support disabling individual bodies; this is a no-op.

        Args:
            data: Disable simulation flags to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.

        """
        # Newton doesn't support disabling individual bodies
        # Log a warning if trying to disable bodies
        _log.warning(
            "Newton physics does not support disabling individual rigid bodies; set_disable_simulations is a no-op"
        )

    def get_disable_gravities(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for per-body gravity-disable flags.

        Newton does not expose per-body gravity-disable state, so the returned buffer is not populated from the model.

        Args:
            copy: Compatibility parameter. The method always returns the view-owned buffer without populating it from
                Newton.

        Returns:
            Frontend tensor of shape ``(count, 1)``.

        """
        return self._disable_gravities

    def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Accept per-body gravity-disable flags without modifying Newton.

        Newton doesn't support per-body gravity disabling; this is a no-op.

        Args:
            data: Disable gravity flags to set.
            indices: Body indices.
            indices_mask: Optional mask for indices.

        """
        # Newton doesn't support per-body gravity disable
        # Log a warning if trying to disable gravity
        _log.warning("Newton physics does not support per-body gravity disabling; set_disable_gravities is a no-op")

    def apply_forces(
        self,
        force_data: Any,
        indices: Any | None = None,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply world-frame forces to selected rigid bodies.

        This compatibility helper delegates to :meth:`apply_forces_and_torques_at_position`.

        Args:
            force_data: Forces to apply, shape (count, 3).
            indices: Indices of bodies to apply forces to. If None, applies to all bodies.
            indices_mask: Optional mask for the indices.

        """
        if indices is None:
            indices = wp.arange(self.count, dtype=wp.int32, device=str(self._frontend.device))

        self.apply_forces_and_torques_at_position(force_data, None, None, indices, indices_mask)

    @_no_profile
    def apply_forces_and_torques_at_position(
        self,
        force_data: Any | None,
        torque_data: Any | None,
        position_data: Any | None,
        indices: Any,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply world-frame forces and torques to rigid bodies at specified positions.

        A force with no position acts at the body center of mass. A supplied position generates the corresponding
        moment about the center of mass. Calls without force or torque data, or with position data but no force data,
        are ignored after logging a diagnostic.

        Args:
            force_data: Forces with shape ``(count, 3)``, or None to apply only torques.
            torque_data: Torques with shape ``(count, 3)``, or None to apply only forces.
            position_data: Application positions with shape ``(count, 3)``, or None to apply forces at each body center
                of mass. Positions require ``force_data``.
            indices: Indices of bodies to apply forces to.
            indices_mask: Optional mask for the indices.

        """
        # Validate inputs
        has_force = force_data is not None
        has_torque = torque_data is not None
        has_position = position_data is not None

        if not has_force and not has_torque:
            _log.warning("No force or torque data provided to apply_forces_and_torques_at_position")
            return

        if has_position and not has_force:
            _log.error("position_data requires force_data to be provided")
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
            dim=indices_tensor.shape[0],
            inputs=[
                force_tensor,
                torque_tensor,
                position_tensor,
                indices_tensor,
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.body_indices,
                state.body_q,
                self._model.body_com,
                has_force,
                has_torque,
                has_position,
            ],
            outputs=[state.body_f],
            device=str(self._frontend.device),
        )

    def check(self) -> bool:
        """Check whether the rigid-body view contains a valid nonempty selection.

        Returns:
            True if the view has a valid backend with at least one body.

        """
        return self._backend is not None and self.count > 0
