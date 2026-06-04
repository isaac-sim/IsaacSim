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

"""Rigid contact view for Newton physics tensor interface."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import carb
import numpy as np
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager

if TYPE_CHECKING:
    from .backend import RigidContactSet
    from .frontends import NumpyFrontend, TorchFrontend, WarpFrontend

from .contact_kernels import (
    contact_data_kernel,
    contact_force_matrix_kernel,
    count_contacts_per_pair_kernel,
    count_raw_contacts_per_sensor_kernel,
    net_contact_forces_kernel,
    populate_contact_points_kernel,
    raw_contact_data_kernel,
)
from .kernels import *
from .tensor_utils import zero_tensor

# Import tensor types from omni.physics.tensors for compatibility
try:
    from omni.physics.tensors import float32, uint8, uint32
except ImportError:
    float32 = wp.float32
    uint8 = wp.uint8
    uint32 = wp.uint32

copy_data = True


def _extract_vec3_from_spatial(
    spatial_arr: "wp.array",
    device: str,
) -> "wp.array":
    """Extract the linear (first 3) components from a wp.spatial_vector array into wp.vec3.

    Args:
        spatial_arr: Array of spatial vectors (linear + angular).
        device: Warp device string for output array.

    Returns:
        Warp array of vec3 (linear components only).
    """
    n = spatial_arr.shape[0]
    out = wp.zeros(n, dtype=wp.vec3, device=device)
    wp.launch(_extract_vec3_kernel, dim=n, inputs=[spatial_arr], outputs=[out], device=device)
    return out


@wp.kernel
def _extract_vec3_kernel(
    src: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
    dst: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
) -> None:
    """Copy linear (first three) components of each spatial_vector to vec3.

    Args:
        src: Input array of spatial vectors.
        dst: Output array of vec3 (linear components).
    """
    tid = wp.tid()
    s = src[tid]
    dst[tid] = wp.spatial_top(s)


def _resolve_newton_contacts(contacts: Any, model: Any) -> tuple[Any, Any, Any, Any, Any] | None:
    """Resolve Newton Contacts to (count_ref, shape0, shape1, normal, force) for kernels.

    Newton uses rigid_contact_normal, rigid_contact_shape0/1.
    Force comes from contacts.force (spatial_vector, populated by MuJoCo solver)
    or falls back to rigid_contact_force (vec3).

    Args:
        contacts: Newton contacts object (rigid_contact_* attributes).
        model: Newton model (device, rigid_contact_max).

    Returns:
        Tuple (count_ref, shape0, shape1, normal, force) or None if invalid.
    """
    if contacts is None or model is None:
        return None
    normal = getattr(contacts, "rigid_contact_normal", None) or getattr(contacts, "normal", None)
    if normal is None:
        return None
    count_ref = getattr(contacts, "rigid_contact_count", None) or getattr(contacts, "n_contacts", None)
    if count_ref is None:
        return None
    shape0 = getattr(contacts, "rigid_contact_shape0", None)
    shape1 = getattr(contacts, "rigid_contact_shape1", None)
    if shape0 is None or shape1 is None:
        return None

    spatial_force = getattr(contacts, "force", None)
    if spatial_force is not None:
        force = _extract_vec3_from_spatial(spatial_force, model.device)
    else:
        force = getattr(contacts, "rigid_contact_force", None)  # type: ignore[assignment]
        if force is None:
            force = wp.zeros(model.rigid_contact_max, dtype=wp.vec3, device=model.device)
    return (count_ref, shape0, shape1, normal, force)


class NewtonRigidContactView:
    """View for contact sensors in Newton physics simulation.

    Provides tensor-based access to contact forces and contact data,
    matching PhysX IRigidContactView interface.

    Args:
        backend: RigidContactSet backend instance from NewtonSimView.
        frontend: Tensor framework frontend.
    """

    def __init__(self, backend: "RigidContactSet", frontend: "NumpyFrontend | TorchFrontend | WarpFrontend") -> None:
        self._backend = backend
        self._frontend = frontend
        self._newton_stage = backend.newton_stage
        self._model = backend.model
        self._sim_timestamp = 0
        self._max_contact_data_count = backend.max_contact_data_count if backend.max_contact_data_count > 0 else 1000

        # Create tensors for caching contact forces
        self._net_forces, self._net_forces_desc = self._frontend.create_tensor((self.sensor_count, 3), float32)

        # Use body-to-sensor mapping from backend (already created with correct original body indices)
        self._body_sensor_map = self._backend.body_sensor_map

        # Create body-to-filter mapping
        self._body_filter_map = None
        self._create_filter_mappings()

        # Create additional tensors for detailed contact data (lazy init)
        self._force_matrix = None
        self._contact_forces_buffer = None
        self._contact_points_buffer = None
        self._contact_normals_buffer = None
        self._contact_separations_buffer = None
        self._contact_counts = None
        self._contact_start_indices = None

        # Raw contact data buffers (lazy init)
        self._raw_forces_buffer = None
        self._raw_points_buffer = None
        self._raw_normals_buffer = None
        self._raw_separations_buffer = None
        self._raw_counts = None
        self._raw_start_indices = None
        self._raw_other_actor_ids = None

    def _create_filter_mappings(self) -> None:
        """Create body_filter_map[sensor_count, body_count+1] from resolved filter indices.

        Indexed directly by body index, matching PhysX where the filter map
        spans all bodies. The extra slot at body_count is for world body.
        """
        if self._model is None:
            return

        num_bodies = self._backend.world_body_idx + 1
        body_filter_map = np.full((self.sensor_count, num_bodies), -1, dtype=np.int32)

        if hasattr(self._backend, "filter_indices") and self._backend.filter_indices is not None:
            fi = (
                self._backend.filter_indices.numpy()
                if hasattr(self._backend.filter_indices, "numpy")
                else np.array(self._backend.filter_indices)
            )
            for sensor_idx in range(min(self.sensor_count, fi.shape[0])):
                for filter_idx in range(fi.shape[1]):
                    body_idx = int(fi[sensor_idx, filter_idx])
                    if 0 <= body_idx < num_bodies:
                        body_filter_map[sensor_idx, body_idx] = filter_idx
        elif hasattr(self._backend, "filter_paths") and self._backend.filter_paths:
            for sensor_idx in range(self.sensor_count):
                if sensor_idx < len(self._backend.filter_paths):
                    for filter_idx, filter_path in enumerate(self._backend.filter_paths[sensor_idx]):
                        try:
                            body_idx = list(self._model.body_label).index(filter_path)
                            body_filter_map[sensor_idx, body_idx] = filter_idx
                        except ValueError:
                            carb.log_warn(f"[NewtonRigidContactView] Filter body not found: {filter_path}")

        self._body_filter_map = wp.array(body_filter_map, dtype=wp.int32, device=self._model.device)  # type: ignore[assignment]

    @property
    def count(self) -> int:
        """Number of contact sensors in this view.

        Returns:
            Number of contact sensors.
        """
        return self._backend.count

    @property
    def sensor_count(self) -> int:
        """Number of contact sensors.

        Returns:
            Number of contact sensors.
        """
        return self._backend.sensor_count

    @property
    def filter_count(self) -> int:
        """Maximum number of filter bodies per sensor.

        Returns:
            Maximum number of filter bodies per sensor.
        """
        return self._backend.filter_count

    @property
    def sensor_names(self) -> list[str]:
        """List of sensor names.

        Returns:
            List of sensor names.
        """
        return self._backend.sensor_names

    @property
    def sensor_paths(self) -> list[str]:
        """List of USD paths for sensors.

        Returns:
            List of USD paths for sensors.
        """
        return self._backend.sensor_paths

    @property
    def filter_paths(self) -> list[list[str]]:
        """List of filter paths (list of lists, one list per sensor).

        Returns:
            List of filter paths with nested lists for each sensor.
        """
        return self._backend.filter_paths

    @property
    def filter_names(self) -> list[list[str]]:
        """List of filter names.

        Returns:
            List of filter names.
        """
        return self._backend.filter_names

    @property
    def max_contact_data_count(self) -> int:
        """Maximum number of contact data entries.

        Returns:
            Maximum contact data buffer size.
        """
        return self._max_contact_data_count

    def update(self, dt: float) -> None:
        """Update simulation timestamp.

        Args:
            dt: Time delta.
        """
        self._sim_timestamp += dt  # type: ignore[assignment]

    @carb.profiler.profile
    def get_net_contact_forces(self, dt: float, copy: bool = copy_data) -> Any:
        """Get net contact forces for each sensor.

        Args:
            dt: Time step for force calculation.
            copy: Whether to return a copy.

        Returns:
            A tensor of shape (sensor_count, 3) with net contact forces.
        """
        model = self._model
        zero_tensor(self._net_forces)

        contacts = self._newton_stage.contacts  # type: ignore[has-type]
        resolved = _resolve_newton_contacts(contacts, model)
        if resolved is None:
            return self._net_forces
        count_ref, shape0, shape1, _normal, force = resolved

        contact_count = count_ref.numpy()[0] if hasattr(count_ref, "numpy") else int(count_ref)
        if contact_count == 0:
            return self._net_forces

        if model.rigid_contact_max > 0:
            # Scale force by physics dt so net result matches PhysX convention
            wp.launch(
                kernel=net_contact_forces_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    force,
                    model.shape_body,
                    self._body_sensor_map,
                    self._backend.world_body_idx,
                    SimulationManager.get_physics_dt() / dt,
                ],
                outputs=[self._net_forces],
                device=model.device,
            )
        wp.synchronize_device(model.device)

        return self._net_forces

    def get_contact_force_matrix(self, dt: float, copy: bool = copy_data) -> wp.array:  # type: ignore[return-value]
        """Get contact force matrix (sensor x filter).

        Args:
            dt: Time step for force calculation.
            copy: Whether to return a copy.

        Returns:
            A tensor of shape (sensor_count, filter_count, 3).
        """
        # Lazy initialization
        if self._force_matrix is None:
            self._force_matrix, _ = self._frontend.create_tensor((self.sensor_count, self.filter_count, 3), float32)

        model = self._model

        if hasattr(self._force_matrix, "fill_"):
            self._force_matrix.fill_(0.0)  # type: ignore[attr-defined]
        elif hasattr(self._force_matrix, "zero_"):
            self._force_matrix.zero_()  # type: ignore[attr-defined]
        else:
            self._force_matrix[:] = 0.0  # type: ignore[index]

        contacts = self._newton_stage.contacts  # type: ignore[has-type]
        resolved = _resolve_newton_contacts(contacts, model)
        if resolved is None:
            return self._force_matrix  # type: ignore[return-value]
        count_ref, shape0, shape1, _normal, force = resolved
        contact_count = count_ref.numpy()[0] if hasattr(count_ref, "numpy") else int(count_ref)
        if model.rigid_contact_max > 0 and contact_count > 0:
            wp.launch(
                kernel=contact_force_matrix_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    force,
                    model.shape_body,
                    self._body_sensor_map,
                    self._body_filter_map,
                    self._backend.world_body_idx,
                    SimulationManager.get_physics_dt() / dt,
                    self.filter_count,
                ],
                outputs=[self._force_matrix],
                device=model.device,
            )
            wp.synchronize_device(model.device)

        return self._force_matrix  # type: ignore[return-value]

    def get_contact_data(self, dt: float, max_contact_data_count: int = 0, copy: bool = copy_data) -> Any:
        """Get detailed contact data (forces, points, normals, separations).

        Args:
            dt: Time step for force calculation.
            max_contact_data_count: Maximum number of contact points to store.
            copy: Whether to return copies.

        Returns:
            A tuple of (contact_forces, contact_points, contact_normals,
            contact_separations, contact_counts, contact_start_indices) where:

            - contact_forces: shape (max_contact_data_count, 1) - normal force magnitudes
            - contact_points: shape (max_contact_data_count, 3) - world space contact positions
            - contact_normals: shape (max_contact_data_count, 3) - contact normals
            - contact_separations: shape (max_contact_data_count, 1) - penetration depths
            - contact_counts: shape (sensor_count, filter_count) - number of contacts per pair
            - contact_start_indices: shape (sensor_count, filter_count) - buffer start indices
        """
        if max_contact_data_count <= 0:
            max_contact_data_count = self._max_contact_data_count

        if self._contact_forces_buffer is None or self._contact_forces_buffer.shape[0] != max_contact_data_count:
            self._contact_forces_buffer, _ = self._frontend.create_tensor((max_contact_data_count, 1), float32)
            self._contact_points_buffer, _ = self._frontend.create_tensor((max_contact_data_count, 3), float32)
            self._contact_normals_buffer, _ = self._frontend.create_tensor((max_contact_data_count, 3), float32)
            self._contact_separations_buffer, _ = self._frontend.create_tensor((max_contact_data_count, 1), float32)
            # Create uint32 arrays directly as Warp arrays (PyTorch doesn't support uint32 well)
            self._contact_counts = wp.zeros(  # type: ignore[assignment]
                (self.sensor_count, self.filter_count), dtype=wp.uint32, device=self._model.device
            )
            self._contact_start_indices = wp.zeros(  # type: ignore[assignment]
                (self.sensor_count, self.filter_count), dtype=wp.uint32, device=self._model.device
            )

        # Zero out buffers
        zero_tensor(self._contact_forces_buffer)
        zero_tensor(self._contact_points_buffer)
        zero_tensor(self._contact_normals_buffer)
        zero_tensor(self._contact_separations_buffer)
        self._contact_counts.zero_()  # type: ignore[attr-defined]
        self._contact_start_indices.zero_()  # type: ignore[attr-defined]

        state = self._newton_stage.state_0
        model = self._model

        contacts = self._newton_stage.contacts  # type: ignore[has-type]
        resolved = _resolve_newton_contacts(contacts, model)
        if resolved is None:
            return (
                self._contact_forces_buffer,
                self._contact_points_buffer,
                self._contact_normals_buffer,
                self._contact_separations_buffer,
                self._contact_counts,
                self._contact_start_indices,
            )
        count_ref, shape0, shape1, normal, force = resolved

        point0 = getattr(contacts, "rigid_contact_point0", None) or getattr(contacts, "position", None)
        point1 = getattr(contacts, "rigid_contact_point1", None) or getattr(contacts, "position", None)
        thick0 = getattr(contacts, "rigid_contact_thickness0", None) or wp.zeros(
            model.rigid_contact_max, dtype=wp.float32, device=model.device
        )
        thick1 = getattr(contacts, "rigid_contact_thickness1", None) or wp.zeros(
            model.rigid_contact_max, dtype=wp.float32, device=model.device
        )
        if point0 is None or point1 is None:
            return (
                self._contact_forces_buffer,
                self._contact_points_buffer,
                self._contact_normals_buffer,
                self._contact_separations_buffer,
                self._contact_counts,
                self._contact_start_indices,
            )

        # MuJoCo doesn't populate rigid_contact_point0/1 in Newton's Contacts.
        # Populate them from MuJoCo's world-space contact positions + body transforms.
        solver = self._newton_stage.solver  # type: ignore[has-type]
        mj_contact_pos = None
        if hasattr(solver, "mjw_data") and solver.mjw_data is not None:
            mj_contact = getattr(solver.mjw_data, "contact", None)
            if mj_contact is not None:
                mj_contact_pos = getattr(mj_contact, "pos", None)
        if mj_contact_pos is not None and point0 is not None and point1 is not None and state is not None:
            wp.launch(
                kernel=populate_contact_points_kernel,
                dim=model.rigid_contact_max,
                inputs=[count_ref, shape0, shape1, mj_contact_pos, model.shape_body, state.body_q],
                outputs=[point0, point1],
                device=model.device,
            )
            wp.synchronize_device(model.device)

        if model.rigid_contact_max > 0:
            forces_wp = self._to_warp_array(self._contact_forces_buffer)  # type: ignore[arg-type]
            points_wp = self._to_warp_array(self._contact_points_buffer)  # type: ignore[arg-type]
            normals_wp = self._to_warp_array(self._contact_normals_buffer)  # type: ignore[arg-type]
            separations_wp = self._to_warp_array(self._contact_separations_buffer)  # type: ignore[arg-type]
            counts_wp = self._contact_counts
            indices_wp = self._contact_start_indices

            wp.launch(
                kernel=count_contacts_per_pair_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    model.shape_body,
                    self._body_sensor_map,
                    self._body_filter_map,
                    self._backend.world_body_idx,
                ],
                outputs=[counts_wp],
                device=model.device,
            )
            wp.synchronize_device(model.device)

            counts_np = counts_wp.numpy().astype(np.uint32).reshape(-1)  # type: ignore[attr-defined]
            indices_np = np.zeros_like(counts_np, dtype=np.uint32)
            indices_np[1:] = np.cumsum(counts_np[:-1], dtype=np.uint32)
            indices_wp_flat = wp.array(indices_np, dtype=wp.uint32, device=model.device)  # type: ignore[arg-type, var-annotated]
            wp.copy(indices_wp, indices_wp_flat.reshape(indices_wp.shape))  # type: ignore[attr-defined, arg-type]
            counts_wp.zero_()  # type: ignore[attr-defined]

            body_q = (
                state.body_q
                if state is not None
                else wp.zeros(model.body_count, dtype=wp.transform, device=model.device)
            )
            wp.launch(
                kernel=contact_data_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    point0,
                    point1,
                    normal,
                    force,
                    thick0,
                    thick1,
                    model.shape_body,
                    body_q,
                    self._body_sensor_map,
                    self._body_filter_map,
                    self._backend.world_body_idx,
                    SimulationManager.get_physics_dt() / dt,
                    max_contact_data_count,
                ],
                outputs=[forces_wp, points_wp, normals_wp, separations_wp, counts_wp, indices_wp],
                device=model.device,
            )
            wp.synchronize_device(model.device)

        # Convert uint32 Warp arrays to frontend tensors for compatibility with API layer
        # PyTorch doesn't support uint32 indexing, so convert to int32 for PyTorch
        try:
            import torch

            if isinstance(self._contact_forces_buffer, torch.Tensor):
                # Convert uint32 to int32 since PyTorch doesn't support uint32 indexing
                counts_frontend = wp.to_torch(self._contact_counts).to(torch.int32)
                indices_frontend = wp.to_torch(self._contact_start_indices).to(torch.int32)
            else:
                counts_frontend = self._contact_counts
                indices_frontend = self._contact_start_indices
        except (ImportError, AttributeError):
            counts_frontend = self._contact_counts
            indices_frontend = self._contact_start_indices

        return (
            self._contact_forces_buffer,
            self._contact_points_buffer,
            self._contact_normals_buffer,
            self._contact_separations_buffer,
            counts_frontend,
            indices_frontend,
        )

    @carb.profiler.profile
    def get_raw_contact_data(self, dt: float, copy: bool = copy_data) -> Any:
        """Get raw contact data for all contacts per sensor without filter matching.

        Args:
            dt: Time step for force calculation.
            copy: Whether to return copies.

        Returns:
            A tuple of (force_buffer, point_buffer, normal_buffer, separation_buffer,
            count_buffer, start_indices_buffer, other_actor_ids_buffer) where:

            - force_buffer: shape (max_contact_data_count, 1) - force magnitudes
            - point_buffer: shape (max_contact_data_count, 3) - world contact positions
            - normal_buffer: shape (max_contact_data_count, 3) - contact normals
            - separation_buffer: shape (max_contact_data_count, 1) - penetration depths
            - count_buffer: shape (sensor_count,) - contacts per sensor
            - start_indices_buffer: shape (sensor_count,) - start index per sensor
            - other_actor_ids_buffer: shape (max_contact_data_count,) - uint64 body IDs
        """
        max_count = self._max_contact_data_count

        if self._raw_forces_buffer is None or self._raw_forces_buffer.shape[0] != max_count:
            self._raw_forces_buffer, _ = self._frontend.create_tensor((max_count, 1), float32)
            self._raw_points_buffer, _ = self._frontend.create_tensor((max_count, 3), float32)
            self._raw_normals_buffer, _ = self._frontend.create_tensor((max_count, 3), float32)
            self._raw_separations_buffer, _ = self._frontend.create_tensor((max_count, 1), float32)
            self._raw_counts = wp.zeros(self.sensor_count, dtype=wp.uint32, device=self._model.device)  # type: ignore[assignment]
            self._raw_start_indices = wp.zeros(self.sensor_count, dtype=wp.uint32, device=self._model.device)  # type: ignore[assignment]
            self._raw_other_actor_ids = wp.zeros(max_count, dtype=wp.uint64, device=self._model.device)  # type: ignore[assignment]

        zero_tensor(self._raw_forces_buffer)
        zero_tensor(self._raw_points_buffer)
        zero_tensor(self._raw_normals_buffer)
        zero_tensor(self._raw_separations_buffer)
        self._raw_counts.zero_()  # type: ignore[attr-defined]
        self._raw_start_indices.zero_()  # type: ignore[attr-defined]
        self._raw_other_actor_ids.zero_()  # type: ignore[attr-defined]

        state = self._newton_stage.state_0
        model = self._model

        contacts = self._newton_stage.contacts  # type: ignore[has-type]
        resolved = _resolve_newton_contacts(contacts, model)
        if resolved is None:
            return (
                self._raw_forces_buffer,
                self._raw_points_buffer,
                self._raw_normals_buffer,
                self._raw_separations_buffer,
                self._raw_counts,
                self._raw_start_indices,
                self._raw_other_actor_ids,
            )
        count_ref, shape0, shape1, normal, force = resolved

        point0 = getattr(contacts, "rigid_contact_point0", None) or getattr(contacts, "position", None)
        point1 = getattr(contacts, "rigid_contact_point1", None) or getattr(contacts, "position", None)
        thick0 = getattr(contacts, "rigid_contact_thickness0", None) or wp.zeros(
            model.rigid_contact_max, dtype=wp.float32, device=model.device
        )
        thick1 = getattr(contacts, "rigid_contact_thickness1", None) or wp.zeros(
            model.rigid_contact_max, dtype=wp.float32, device=model.device
        )
        if point0 is None or point1 is None:
            return (
                self._raw_forces_buffer,
                self._raw_points_buffer,
                self._raw_normals_buffer,
                self._raw_separations_buffer,
                self._raw_counts,
                self._raw_start_indices,
                self._raw_other_actor_ids,
            )

        # Populate contact points from MuJoCo world-space positions if needed
        solver = self._newton_stage.solver  # type: ignore[has-type]
        mj_contact_pos = None
        if hasattr(solver, "mjw_data") and solver.mjw_data is not None:
            mj_contact = getattr(solver.mjw_data, "contact", None)
            if mj_contact is not None:
                mj_contact_pos = getattr(mj_contact, "pos", None)
        if mj_contact_pos is not None and point0 is not None and point1 is not None and state is not None:
            wp.launch(
                kernel=populate_contact_points_kernel,
                dim=model.rigid_contact_max,
                inputs=[count_ref, shape0, shape1, mj_contact_pos, model.shape_body, state.body_q],
                outputs=[point0, point1],
                device=model.device,
            )
            wp.synchronize_device(model.device)

        if model.rigid_contact_max > 0:
            forces_wp = self._to_warp_array(self._raw_forces_buffer)  # type: ignore[arg-type]
            points_wp = self._to_warp_array(self._raw_points_buffer)  # type: ignore[arg-type]
            normals_wp = self._to_warp_array(self._raw_normals_buffer)  # type: ignore[arg-type]
            separations_wp = self._to_warp_array(self._raw_separations_buffer)  # type: ignore[arg-type]

            # Pass 1: count contacts per sensor
            wp.launch(
                kernel=count_raw_contacts_per_sensor_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    model.shape_body,
                    self._body_sensor_map,
                    self._backend.world_body_idx,
                ],
                outputs=[self._raw_counts],
                device=model.device,
            )
            wp.synchronize_device(model.device)

            # Prefix scan: compute start indices from counts
            counts_np = self._raw_counts.numpy().astype(np.uint32)  # type: ignore[attr-defined]
            indices_np = np.zeros_like(counts_np, dtype=np.uint32)
            indices_np[1:] = np.cumsum(counts_np[:-1], dtype=np.uint32)
            wp.copy(self._raw_start_indices, wp.array(indices_np, dtype=wp.uint32, device=model.device))  # type: ignore[arg-type]
            self._raw_counts.zero_()  # type: ignore[attr-defined]

            # Pass 2: write raw contact data
            body_q = (
                state.body_q
                if state is not None
                else wp.zeros(model.body_count, dtype=wp.transform, device=model.device)
            )
            wp.launch(
                kernel=raw_contact_data_kernel,
                dim=model.rigid_contact_max,
                inputs=[
                    count_ref,
                    shape0,
                    shape1,
                    point0,
                    point1,
                    normal,
                    force,
                    thick0,
                    thick1,
                    model.shape_body,
                    body_q,
                    self._body_sensor_map,
                    self._backend.world_body_idx,
                    SimulationManager.get_physics_dt() / dt,
                    max_count,
                ],
                outputs=[
                    forces_wp,
                    points_wp,
                    normals_wp,
                    separations_wp,
                    self._raw_counts,
                    self._raw_start_indices,
                    self._raw_other_actor_ids,
                ],
                device=model.device,
            )
            wp.synchronize_device(model.device)

        return (
            self._raw_forces_buffer,
            self._raw_points_buffer,
            self._raw_normals_buffer,
            self._raw_separations_buffer,
            self._raw_counts,
            self._raw_start_indices,
            self._raw_other_actor_ids,
        )

    def get_actor_paths_from_ids(self, actor_ids: wp.array) -> list[str]:
        """Convert body indices (from get_raw_contact_data) to USD paths.

        For Newton, actor IDs are body indices stored as uint64.

        Args:
            actor_ids: Tensor of body indices (uint64), or a slice of it.

        Returns:
            List of USD paths. Empty string for invalid indices.
        """
        ids_np = actor_ids.numpy() if hasattr(actor_ids, "numpy") else np.array(actor_ids)
        model = self._model
        body_labels = model.body_label if model is not None else []
        world_body_idx = self._backend.world_body_idx
        paths = []
        for body_idx_u64 in ids_np:
            body_idx = int(body_idx_u64)
            if body_idx == world_body_idx:
                paths.append("world")
            elif 0 <= body_idx < len(body_labels):
                paths.append(body_labels[body_idx])
            else:
                paths.append("")
        return paths

    def _to_warp_array(self, tensor: wp.array | np.ndarray) -> wp.array:
        """Convert frontend tensor to Warp array for kernel launch.

        Args:
            tensor: Frontend tensor to convert.

        Returns:
            Warp array representation.
        """
        if isinstance(tensor, wp.array):
            return tensor
        # For PyTorch/NumPy tensors, get underlying data pointer
        # This assumes the frontend properly exposes warp arrays
        if hasattr(tensor, "_warp_array"):
            return tensor._warp_array
        # Fallback: try to get the underlying array
        return tensor  # type: ignore[return-value]

    def check(self) -> bool:
        """Check if the view is valid and has sensors.

        Returns:
            True if view has a valid backend and at least one sensor.
        """
        return self._backend is not None and self.count > 0
