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

"""Provide the Newton articulation tensor view."""

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


import newton
import warp as wp

from .kernels import *
from .kernels import (
    apply_link_forces_at_position,
    build_ctrl_direct_dof_mapping,
    cache_link_com,
    get_link_com_position_only,
    sync_ctrl_direct_gains,
    sync_ctrl_direct_targets,
    update_inv_mass,
)
from .tensor_utils import convert_to_warp, wrap_input_tensor

# Import tensor types from isaacsim.physics.manager.impl.tensors for compatibility
try:
    from isaacsim.physics.manager.impl.tensors import float32, uint8, uint32
except ImportError:
    # Fallback if not available
    float32 = wp.float32
    uint8 = wp.uint8
    uint32 = wp.uint32

copy_data = True


class NewtonArticulationView:
    """Expose a set of Newton articulations through the tensor API.

    The view maps model storage into frontend tensors and supports indexed reads and writes across the selected
    articulations.

    Args:
        backend: Backend selection containing articulation and model indices.
        frontend: Tensor frontend used to allocate and wrap returned values.

    """

    def __init__(self, backend: Any, frontend: Any) -> None:
        self._backend = backend
        self._frontend = frontend
        self._newton_stage = backend.newton_stage
        self._model = backend.model
        self._sim_timestamp = 0
        self.ik_timestamp = 0
        self._ctrl_direct_dof_map: wp.array | None = None
        self._ctrl_direct_map_initialized = False
        self._ctrl_direct_biastype_set = False

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

    def _check_state(self) -> None:
        """Check whether Newton simulation state is initialized.

        Raises:
            RuntimeError: If Newton simulation state is not initialized.

        """
        if self._newton_stage.state_0 is None:
            raise RuntimeError(
                "Newton simulation state is not initialized. "
                "Make sure initialize_physics() has been called or wait for the warm start to complete. "
                f"(initialized={self._newton_stage.initialized}, model={self._newton_stage.model is not None})"
            )

    def _notify_joint_dof_properties_changed(self) -> None:
        """Notify the solver that joint DOF properties (gains, limits, etc.) have changed."""
        if self._newton_stage.solver is not None:
            try:
                self._newton_stage.solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)
            except AttributeError:
                pass

    def _get_ctrl_direct_dof_map(self) -> wp.array | None:
        """Build the DOF-to-CTRL_DIRECT-actuator mapping lazily.

        Returns:
            Warp array mapping model degrees of freedom to MuJoCo actuator indices, or None when the mapping cannot be
            built.

        """
        if not self._ctrl_direct_map_initialized:
            self._ctrl_direct_map_initialized = True
            try:
                self._ctrl_direct_dof_map = build_ctrl_direct_dof_mapping(self._model)
            except Exception as e:
                _log.warning(f"Failed to build CTRL_DIRECT DOF mapping: {e}")
                self._ctrl_direct_dof_map = None
        return self._ctrl_direct_dof_map

    def _set_ctrl_direct_biastype_affine(self) -> None:
        """Set biastype=AFFINE (1) for CTRL_DIRECT joint actuators in the solver's MuJoCo model.

        Without this, MuJoCo ignores biasprm values (biastype=NONE means bias=0),
        making PD position control impossible through gainprm/biasprm writes.
        """
        BIAS_AFFINE = 1

        # Update model.mujoco custom attribute
        dof_map = self._ctrl_direct_dof_map
        if dof_map is not None:
            dof_map_np = dof_map.numpy()
            newton_act_indices = sorted({int(a) for a in dof_map_np if a >= 0})
            mujoco_attrs = getattr(self._model, "mujoco", None)
            if mujoco_attrs is not None:
                bt = getattr(mujoco_attrs, "actuator_biastype", None)
                if bt is not None:
                    bt_np = bt.numpy()
                    for act_idx in newton_act_indices:
                        if act_idx < len(bt_np):
                            bt_np[act_idx] = BIAS_AFFINE
                    wp.copy(bt, wp.array(bt_np, dtype=bt.dtype, device=bt.device))

        solver = getattr(self._newton_stage, "solver", None)
        if solver is None:
            return

        # Find MuJoCo-model actuator indices for CTRL_DIRECT actuators
        ctrl_source = getattr(solver, "mjc_actuator_ctrl_source", None)
        if ctrl_source is None:
            return
        ctrl_np = ctrl_source.numpy()
        mjc_act_indices = [i for i in range(len(ctrl_np)) if int(ctrl_np[i]) == 1]
        if not mjc_act_indices:
            return

        # Update mj_model
        mj_model = getattr(solver, "mj_model", None)
        if mj_model is not None and hasattr(mj_model, "actuator_biastype"):
            for act_idx in mjc_act_indices:
                if act_idx < len(mj_model.actuator_biastype):
                    mj_model.actuator_biastype[act_idx] = BIAS_AFFINE

        # Update mjw_model (GPU Warp MuJoCo model) for simulation
        mjw_model = getattr(solver, "mjw_model", None)
        if mjw_model is not None:
            bt_warp = getattr(mjw_model, "actuator_biastype", None)
            if bt_warp is not None:
                bt_np = bt_warp.numpy()
                if bt_np.ndim == 2:
                    for act_idx in mjc_act_indices:
                        if act_idx < bt_np.shape[1]:
                            bt_np[:, act_idx] = BIAS_AFFINE
                elif bt_np.ndim == 1:
                    for act_idx in mjc_act_indices:
                        if act_idx < len(bt_np):
                            bt_np[act_idx] = BIAS_AFFINE
                wp.copy(bt_warp, wp.array(bt_np, dtype=bt_warp.dtype, device=bt_warp.device))

    def _sync_ctrl_direct_position_targets(self) -> None:
        """Sync joint_target_pos to control.mujoco.ctrl for CTRL_DIRECT joint actuators.

        On the first call, also sets biastype=AFFINE to enable PD control. This is
        deferred until position targets are available so the PD controller doesn't
        produce large forces against uninitialized (zero) ctrl values.
        """
        dof_map = self._get_ctrl_direct_dof_map()
        if dof_map is None:
            return
        model = self._model
        control = self._newton_stage.control
        mujoco_ctrl = getattr(getattr(control, "mujoco", None), "ctrl", None)
        if mujoco_ctrl is None:
            return
        nworlds = model.world_count if hasattr(model, "world_count") else 1
        dofs_per_world = model.joint_dof_count // max(nworlds, 1)
        ctrls_per_world = mujoco_ctrl.shape[0] // max(nworlds, 1)
        wp.launch(
            sync_ctrl_direct_targets,
            dim=(nworlds, dofs_per_world),
            inputs=[dof_map, control.joint_target_q, dofs_per_world, ctrls_per_world],
            outputs=[mujoco_ctrl],
            device=model.device,
        )

        if not self._ctrl_direct_biastype_set:
            self._ctrl_direct_biastype_set = True
            self._set_ctrl_direct_biastype_affine()

    def _sync_ctrl_direct_actuator_gains(self) -> None:
        """Sync joint_target_ke/kd to actuator gainprm/biasprm for CTRL_DIRECT joint actuators.

        Writes to model.mujoco arrays and notifies the solver. Does NOT activate PD
        control (biastype stays NONE until the first position target is synced).
        """
        dof_map = self._get_ctrl_direct_dof_map()
        if dof_map is None:
            return
        model = self._model
        mujoco_attrs = getattr(model, "mujoco", None)
        if mujoco_attrs is None:
            return
        gainprm = getattr(mujoco_attrs, "actuator_gainprm", None)
        biasprm = getattr(mujoco_attrs, "actuator_biasprm", None)
        if gainprm is None or biasprm is None:
            return
        nworlds = model.world_count if hasattr(model, "world_count") else 1
        dofs_per_world = model.joint_dof_count // max(nworlds, 1)

        wp.launch(
            sync_ctrl_direct_gains,
            dim=(dofs_per_world,),
            inputs=[dof_map, model.joint_target_ke, model.joint_target_kd],
            outputs=[gainprm, biasprm],
            device=model.device,
        )

        if self._newton_stage.solver is not None:
            try:
                self._newton_stage.solver.notify_model_changed(newton.ModelFlags.ACTUATOR_PROPERTIES)
            except AttributeError:
                pass

    @property
    def count(self) -> int:
        """Get the number of articulations in this view.

        Returns:
            Number of articulations.

        """
        return self._backend.count

    @property
    def max_dofs(self) -> int:
        """Get the maximum number of degrees of freedom across the articulations.

        Returns:
            Maximum DOF count.

        """
        return self._backend.max_dofs

    @property
    def max_links(self) -> int:
        """Get the maximum number of links across the articulations.

        Returns:
            Maximum link count.

        """
        return self._backend.max_links

    @property
    def max_shapes(self) -> int:
        """Get the maximum number of collision shapes across the articulations.

        Returns:
            Maximum shape count.

        """
        return self._backend.max_shapes

    @property
    def max_fixed_tendons(self) -> int:
        """Get the maximum number of fixed tendons across the articulations.

        Returns:
            Maximum fixed tendon count.

        """
        return self._backend.max_fixed_tendons

    @property
    def dof_paths(self) -> list[list[str]]:
        """Get degree-of-freedom paths for all articulations in the view.

        Returns:
            List of DOF path lists, one per articulation.

        """
        # For each articulation, get the DOF names and construct paths
        # DOF paths are typically the joint paths with DOF suffixes
        dof_paths_list = []
        for meta in self._backend.meta_types:
            # Use DOF names as paths (they should be full paths or we construct them)
            dof_paths_list.append(meta.dof_paths)
        return dof_paths_list

    @property
    def dof_names(self) -> list[list[str]]:
        """Get degree-of-freedom names for all articulations in the view.

        Returns:
            List of DOF name lists, one per articulation.

        """
        dof_names_list = []
        for meta in self._backend.meta_types:
            dof_names_list.append(meta.dof_names)
        return dof_names_list

    @property
    def link_paths(self) -> list[list[str]]:
        """Get link paths for all articulations in the view.

        Returns:
            List of link path lists, one per articulation.

        """
        link_paths_list = []
        for meta in self._backend.meta_types:
            link_paths_list.append(meta.link_paths)
        return link_paths_list

    @property
    def link_names(self) -> list[list[str]]:
        """Get link names for all articulations in the view.

        Returns:
            List of link name lists, one per articulation.

        """
        link_names_list = []
        for meta in self._backend.meta_types:
            link_names_list.append(meta.link_names)
        return link_names_list

    @property
    def joint_paths(self) -> list[list[str]]:
        """Get joint paths for all articulations in the view.

        Returns:
            List of joint path lists, one per articulation.

        """
        joint_paths_list = []
        for meta in self._backend.meta_types:
            joint_paths_list.append(meta.joint_paths)
        return joint_paths_list

    @property
    def joint_names(self) -> list[list[str]]:
        """Get joint names for all articulations in the view.

        Returns:
            List of joint name lists, one per articulation.

        """
        joint_names_list = []
        for meta in self._backend.meta_types:
            joint_names_list.append(meta.joint_names)
        return joint_names_list

    @property
    def prim_paths(self) -> list[str]:
        """Get articulation root prim paths.

        Returns:
            List of articulation root paths.

        """
        # Get articulation paths from the model
        prim_paths = []
        for arti_idx in self._backend.articulation_indices.numpy():
            prim_paths.append(self._model.articulation_label[arti_idx])
        return prim_paths

    @property
    def shared_metatype(self) -> Any | None:
        """Get representative metadata for this view.

        Returns:
            Metadata from the first articulation, or None when the view is empty.

        """
        if self.count == 0:
            return None
        return self._backend.shared_metatype

    @property
    def is_homogeneous(self) -> bool:
        """Check whether all articulations have equal link, joint, and degree-of-freedom counts.

        Returns:
            True when all articulations have the same numbers of links, joints, and degrees of freedom.

        """
        if self.count <= 1:
            return True

        # Check if all metatypes are the same by comparing key properties
        first_meta = self._backend.meta_types[0]
        for i in range(1, self.count):
            meta = self._backend.meta_types[i]
            if (
                len(meta.link_names) != len(first_meta.link_names)
                or len(meta.joint_names) != len(first_meta.joint_names)
                or len(meta.dof_names) != len(first_meta.dof_names)
            ):
                return False
        return True

    @property
    def jacobian_shape(self) -> tuple[int, int]:
        r"""Get the Jacobian matrix shape for the articulations.

        Returns:
            Tuple (rows, cols) where\:

            - Fixed base: rows = (max_links - 1) * 6, cols = max_dofs
            - Floating base: rows = max_links * 6, cols = max_dofs + 6

        """
        # Check if any articulation is floating base
        is_floating = False
        for meta in self._backend.meta_types:
            if not meta.fixed_base:
                is_floating = True
                break

        if is_floating:
            # Floating base: all links, DOFs include 6 root DOFs
            rows = self.max_links * 6
            cols = self.max_dofs + 6
        else:
            # Fixed base: exclude root link
            rows = (self.max_links - 1) * 6
            cols = self.max_dofs

        return (rows, cols)

    @property
    def generalized_mass_matrix_shape(self) -> tuple[int, int]:
        r"""Get the generalized mass-matrix shape for the articulations.

        Returns:
            Tuple (n, n) where\:

            - Fixed base: n = max_dofs
            - Floating base: n = max_dofs + 6

        """
        # Check if any articulation is floating base
        is_floating = False
        for meta in self._backend.meta_types:
            if not meta.fixed_base:
                is_floating = True
                break

        if is_floating:
            n = self.max_dofs + 6
        else:
            n = self.max_dofs

        return (n, n)

    def get_metatype(self, index: int) -> Any:
        """Get metadata type for a specific articulation.

        Args:
            index: Index of the articulation in the view (0 to count-1).

        Returns:
            ArticulationMetaType object containing link names, joint names, DOF names, etc.

        Raises:
            IndexError: If index is out of range.

        """
        if index < 0 or index >= self.count:
            raise IndexError(f"Articulation index {index} out of range [0, {self.count})")
        return self._backend.meta_types[index]

    def update(self, dt: float) -> None:
        """Advance the view timestamp.

        Args:
            dt: Timestamp increment in seconds.

        """
        self._sim_timestamp += dt

    @_no_profile
    def get_root_transforms(self, copy: bool = copy_data) -> Any:
        """Get root-body transforms as position and quaternion values.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton transforms.

        Returns:
            Values with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order.

        """
        self._check_state()
        state = self._newton_stage.state_0
        if copy:
            if not hasattr(self, "_root_transforms"):
                self._root_transforms, self._root_transforms_desc = self._frontend.create_tensor(
                    (self.count, 7), float32
                )
            wp.launch(
                get_body_pose,
                dim=self._backend.count,
                inputs=[state.body_q, self._backend.root_body_indices],
                outputs=[self._convert_to_warp(self._root_transforms)],
                device=str(self._frontend.device),
            )
            return self._root_transforms
        else:
            return wp.indexedarray(state.body_q, self._backend.root_body_indices)

    @_no_profile
    def get_root_velocities(self, copy: bool = copy_data) -> Any:
        """Get root-body linear and angular velocities.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton spatial
                vectors.

        Returns:
            Values with shape ``(count, 6)`` in linear-then-angular order.

        """
        self._check_state()
        state = self._newton_stage.state_0
        if copy:
            if not hasattr(self, "_root_velocities"):
                self._root_velocities, self._root_velocities_desc = self._frontend.create_tensor(
                    (self.count, 6), float32
                )
            wp.launch(
                get_body_velocity,
                dim=self._backend.count,
                inputs=[state.body_qd, self._backend.root_body_indices],
                outputs=[self._convert_to_warp(self._root_velocities)],
                device=str(self._frontend.device),
            )
            return self._root_velocities
        else:
            return wp.indexedarray(state.body_qd, self._backend.root_body_indices)

    @_no_profile
    def get_link_transforms(self, copy: bool = copy_data) -> Any:
        """Get per-link body transforms.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton transforms.

        Returns:
            Values with shape ``(count, max_links, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order. Padded link slots
            contain the identity transform.

        """
        self._check_state()
        state = self._newton_stage.state_0
        if copy:
            if not hasattr(self, "_link_transforms"):
                self._link_transforms, self._link_transforms_desc = self._frontend.create_tensor(
                    (self.count, self.max_links, 7), float32
                )
            wp.launch(
                get_link_pose,
                dim=(self.count, self.max_links),
                inputs=[state.body_q, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._link_transforms)],
                device=str(self._frontend.device),
            )
            return self._link_transforms
        else:
            return wp.indexedarray(state.body_q, self._backend.link_indices)

    @_no_profile
    def get_link_velocities(self, copy: bool = copy_data) -> Any:
        """Get per-link body velocities.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton spatial
                vectors.

        Returns:
            Values with shape ``(count, max_links, 6)`` in linear-then-angular order. Padded link slots contain zeros.

        """
        self._check_state()
        state = self._newton_stage.state_0
        if copy:
            if not hasattr(self, "_link_velocities"):
                self._link_velocities, self._link_velocities_desc = self._frontend.create_tensor(
                    (self.count, self.max_links, 6), float32
                )
            wp.launch(
                get_link_velocity,
                dim=(self.count, self.max_links),
                inputs=[state.body_qd, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._link_velocities)],
                device=str(self._frontend.device),
            )
            return self._link_velocities
        else:
            return wp.indexedarray(state.body_qd, self._backend.link_indices)

    def get_masses(self, copy: bool = copy_data) -> Any:
        """Get link masses.

        Args:
            copy: Whether to materialize a frontend tensor. If False, return an indexed Warp view of Newton mass
                storage.

        Returns:
            Values with shape ``(count, max_links)``.

        """
        if copy:
            if not hasattr(self, "_masses"):
                self._masses, self._masses_desc = self._frontend.create_tensor((self.count, self.max_links), float32)
            wp.launch(
                get_link_mass,
                dim=(self.count, self.max_links),
                inputs=[self._model.body_mass, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._masses)],
                device=str(self._frontend.device),
            )
            return self._masses
        else:
            return wp.indexedarray(self._model.body_mass, self._backend.link_indices)

    def get_inv_masses(self, copy: bool = copy_data) -> Any:
        """Get link inverse masses (1/mass).

        Args:
            copy: Whether to compute inverse masses in a frontend tensor. If False, return an indexed view of the
                underlying mass storage without inversion.

        Returns:
            Values with shape ``(count, max_links)``. With ``copy=False``, these are masses rather than inverse masses.

        """
        if copy:
            if not hasattr(self, "_inv_masses"):
                self._inv_masses, self._inv_masses_desc = self._frontend.create_tensor(
                    (self.count, self.max_links), float32
                )
            wp.launch(
                get_link_inv_mass,
                dim=(self.count, self.max_links),
                inputs=[self._model.body_mass, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._inv_masses)],
                device=str(self._frontend.device),
            )
            return self._inv_masses
        else:
            # For non-copy mode, return masses and let caller compute inverse
            return wp.indexedarray(self._model.body_mass, self._backend.link_indices)

    def get_inertias(self, copy: bool = copy_data) -> Any:
        """Get link inertias.

        Args:
            copy: Whether to refresh the cached frontend tensor from Newton inertia storage.

        Returns:
            Cached frontend tensor with shape ``(count, max_links, 9)`` in row-major order.

        """
        if not hasattr(self, "_inertias"):
            self._inertias, self._inertias_desc = self._frontend.create_tensor((self.count, self.max_links, 9), float32)
        if copy:
            wp.launch(
                get_link_inertia,
                dim=(self.count, self.max_links, 9),
                inputs=[self._model.body_inertia, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._inertias)],
                device=str(self._frontend.device),
            )
        return self._inertias

    def get_inv_inertias(self, copy: bool = copy_data) -> Any:
        """Get link inverse inertias.

        Args:
            copy: Whether to refresh the cached frontend tensor from Newton inverse-inertia storage.

        Returns:
            Cached frontend tensor with shape ``(count, max_links, 9)`` in row-major order.

        """
        if not hasattr(self, "_inv_inertias"):
            self._inv_inertias, self._inv_inertias_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 9), float32
            )
        if copy:
            wp.launch(
                get_link_inv_inertia,
                dim=(self.count, self.max_links, 9),
                inputs=[self._model.body_inv_inertia, self._backend.link_indices],
                outputs=[self._convert_to_warp(self._inv_inertias)],
                device=str(self._frontend.device),
            )
        return self._inv_inertias

    def get_coms(self, copy: bool = copy_data) -> Any:
        """Get link center of mass positions and orientations.

        Args:
            copy: Compatibility parameter. The method always refreshes positions in and returns view-owned storage.

        Returns:
            Values with shape ``(count, max_links, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order. Positions are
            body-frame offsets read from Newton, while orientations are view-owned cached values.

        """
        if not hasattr(self, "_coms"):
            self._coms, self._coms_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32)
            if self.count > 0 and self.max_links > 0:
                coms_warp = self._convert_to_warp(self._coms)
                coms_warp.fill_(0.0)

                coms_np = coms_warp.numpy()
                coms_np[:, :, 6] = 1.0
                coms_warp.assign(coms_np)
        wp.launch(
            get_link_com_position_only,
            dim=(self.count, self.max_links, 7),
            inputs=[self._model.body_com, self._backend.link_indices],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )
        return self._coms

    def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set link center-of-mass data.

        Newton receives the position components as body-frame offsets. The view-owned cache stores the orientation
        components for subsequent reads.

        Args:
            data: Center-of-mass values with shape ``(count, max_links, 7)`` in
                ``[x, y, z, qx, qy, qz, qw]`` order.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        if not hasattr(self, "_coms"):
            self._coms, self._coms_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32)
            if self.count > 0 and self.max_links > 0:
                coms_warp = self._convert_to_warp(self._coms)
                coms_warp.fill_(0.0)

                coms_np = coms_warp.numpy()
                coms_np[:, :, 6] = 1.0
                coms_warp.assign(coms_np)
        wp.launch(
            set_link_com,
            dim=(indices.shape[0], self.max_links, 3),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
            ],
            outputs=[self._model.body_com],
            device=str(self._frontend.device),
        )
        wp.launch(
            cache_link_com,
            dim=(indices.shape[0], self.max_links, 7),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
            ],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )

    @_no_profile
    def get_dof_positions(self, copy: bool = copy_data) -> Any:
        """Get joint positions.

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton position-coordinate storage.

        Returns:
            Values with shape ``(count, max_dofs)``.

        """
        if copy:
            if not hasattr(self, "_dof_positions"):
                self._dof_positions, self._dof_positions_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._newton_stage.state_0.joint_q, self._backend.dof_position_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_positions)],
                device=str(self._frontend.device),
            )
            return self._dof_positions
        else:
            return wp.indexedarray(self._newton_stage.state_0.joint_q, self._backend.dof_position_indices)

    @_no_profile
    def get_dof_velocities(self, copy: bool = copy_data) -> Any:
        """Get joint velocities.

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton velocity-coordinate storage.

        Returns:
            Values with shape ``(count, max_dofs)``.

        """
        if copy:
            if not hasattr(self, "_dof_velocities"):
                self._dof_velocities, self._dof_velocities_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._newton_stage.state_0.joint_qd, self._backend.dof_velocity_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_velocities)],
                device=str(self._frontend.device),
            )
            return self._dof_velocities
        else:
            return wp.indexedarray(self._newton_stage.state_0.joint_qd, self._backend.dof_velocity_indices)

    @_no_profile
    def get_dof_limits(self, copy: bool = copy_data) -> Any:
        """Get joint limits [lower, upper].

        Args:
            copy: Whether to materialize lower and upper limits in a frontend tensor. If False, return an indexed Warp
                view containing only lower-limit storage.

        Returns:
            Values with shape ``(count, max_dofs, 2)``. With ``copy=False``, only lower limits are returned.

        """
        if copy:
            if not hasattr(self, "_dof_limits"):
                self._dof_limits, self._dof_limits_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs, 2), float32
                )
            wp.launch(
                get_dof_limits,
                dim=(self.count, self.max_dofs),
                inputs=[
                    self._model.joint_limit_lower,
                    self._model.joint_limit_upper,
                    self._backend.dof_axis_indices,
                    self.max_dofs,
                ],
                outputs=[self._convert_to_warp(self._dof_limits)],
                device=str(self._frontend.device),
            )
            return self._dof_limits
        else:
            return wp.indexedarray(self._model.joint_limit_lower, self._backend.dof_axis_indices)

    @_no_profile
    def get_dof_stiffnesses(self, copy: bool = copy_data) -> Any:
        """Get joint stiffnesses (for position control).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton stiffness storage.

        Returns:
            Stiffness values with shape ``(count, max_dofs)``.

        """
        if copy:
            if not hasattr(self, "_dof_stiffnesses"):
                self._dof_stiffnesses, self._dof_stiffnesses_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._model.joint_target_ke, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_stiffnesses)],
                device=str(self._frontend.device),
            )
            return self._dof_stiffnesses
        else:
            return wp.indexedarray(self._model.joint_target_ke, self._backend.dof_axis_indices)

    @_no_profile
    def get_dof_dampings(self, copy: bool = copy_data) -> Any:
        """Get joint dampings (for velocity control).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton damping storage.

        Returns:
            Damping values with shape ``(count, max_dofs)``.

        """
        if copy:
            if not hasattr(self, "_dof_dampings"):
                self._dof_dampings, self._dof_dampings_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._model.joint_target_kd, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_dampings)],
                device=str(self._frontend.device),
            )
            return self._dof_dampings
        else:
            return wp.indexedarray(self._model.joint_target_kd, self._backend.dof_axis_indices)

    @_no_profile
    def get_dof_armatures(self, copy: bool = copy_data) -> Any:
        """Get joint armatures (rotor inertias).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton armature storage.

        Returns:
            Armature values with shape ``(count, max_dofs)``.

        """
        if copy:
            if not hasattr(self, "_dof_armatures"):
                self._dof_armatures, self._dof_armatures_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._model.joint_armature, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_armatures)],
                device=str(self._frontend.device),
            )
            return self._dof_armatures
        else:
            return wp.indexedarray(self._model.joint_armature, self._backend.dof_axis_indices)

    def get_dof_position_targets(self, copy: bool = copy_data) -> Any:
        """Get joint position targets (for position control).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton position-target storage.

        Returns:
            Position targets with shape ``(count, max_dofs)``.

        """
        control = self._newton_stage.control
        if copy:
            if not hasattr(self, "_dof_position_targets"):
                self._dof_position_targets, self._dof_position_targets_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[control.joint_target_q, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_position_targets)],
                device=str(self._frontend.device),
            )
            return self._dof_position_targets
        else:
            return wp.indexedarray(control.joint_target_q, self._backend.dof_axis_indices)

    def get_dof_velocity_targets(self, copy: bool = copy_data) -> Any:
        """Get joint velocity targets (for velocity control).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton velocity-target storage.

        Returns:
            Velocity targets with shape ``(count, max_dofs)``.

        """
        control = self._newton_stage.control
        if copy:
            if not hasattr(self, "_dof_velocity_targets"):
                self._dof_velocity_targets, self._dof_velocity_targets_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[control.joint_target_qd, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_velocity_targets)],
                device=str(self._frontend.device),
            )
            return self._dof_velocity_targets
        else:
            return wp.indexedarray(control.joint_target_qd, self._backend.dof_axis_indices)

    def _update_articulation_state(
        self, indices: Any, indices_mask: Any | None = None, update_positions: bool = True
    ) -> None:
        """Update articulation link transforms from generalized coordinates.

        Args:
            indices: Indices of articulations to update.
            indices_mask: Compatibility parameter reserved for masked updates.
            update_positions: Whether to evaluate forward kinematics. If False, no update is performed.

        """
        from newton import eval_fk

        # Only evaluate forward kinematics for position updates
        # For velocity updates, FK might reset the velocities we just set
        if update_positions:
            state = self._newton_stage.state_0
            # Restrict FK to the selected articulations instead of the whole model:
            # gather the selected view rows' model articulation ids and pass them, so
            # a one-environment reset re-evaluates one articulation, not the whole
            # model. eval_fk needs a contiguous array, so materialize the indexed view.
            idx = self._wrap_input_tensor(indices)
            arti_indices = wp.clone(wp.indexedarray(self._backend.articulation_indices, idx))
            eval_fk(self._model, state.joint_q, state.joint_qd, state, indices=arti_indices)

    @_no_profile
    def set_root_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set root body transforms for floating- and fixed-base articulations.

        Writes the new root pose into the root body_q, syncs the root joint from
        it (free-joint joint_q for a floating base, joint_X_p for a fixed base),
        then evaluates forward kinematics so the whole articulation follows the
        root rigidly.

        Args:
            data: Root transforms with shape ``(count, 7)`` in ``[x, y, z, qx, qy, qz, qw]`` order.
            indices: Indices of articulations to update.
            indices_mask: Optional mask for the indices.

        """
        self._check_state()

        state = self._newton_stage.state_0

        # Write the new root pose into the root body_q; the root-joint sync below
        # picks it up so eval_fk reproduces it.
        wp.launch(
            set_body_pose,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.root_body_indices,
            ],
            outputs=[state.body_q],
            device=str(self._frontend.device),
        )

        # The body_q write above does not touch the root joint, so eval_fk below
        # would rebuild body_q from the stale root joint and drop the new pose.
        # Sync the root joint from the just-set root body_q first: a floating
        # base's pose lives in the free joint's joint_q, a fixed base's in
        # joint_X_p. Only the root joint is touched, so actuated joints -- and
        # thus the rest of the articulation -- follow the root rigidly.
        wp.launch(
            update_root_joint_coords_from_body_q,
            dim=indices.shape[0],
            inputs=[
                state.body_q,
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.root_body_indices,
                self._backend.root_joint_indices,
                self._model.joint_type,
                self._model.joint_q_start,
                self._model.joint_X_c,
            ],
            outputs=[state.joint_q, self._model.joint_X_p],
            device=str(self._frontend.device),
        )

        # Update joint coordinates and evaluate forward kinematics
        self._update_articulation_state(indices, indices_mask, update_positions=True)

    @_no_profile
    def set_root_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set root body velocities.

        Both Newton state buffers receive the body spatial velocities. For maximal-coordinate solvers, floating-base
        generalized velocities are also synchronized from the current state.

        Args:
            data: Root velocities with shape ``(count, 6)`` in linear-then-angular order.
            indices: Indices of articulations to update.
            indices_mask: Optional mask for the indices.

        """
        self._check_state()
        state_0 = self._newton_stage.state_0
        state_1 = self._newton_stage.state_1

        # Set both state_0 and state_1 to prevent old values from being swapped back
        # This follows the pattern for velocity data (unlike position data which uses single state)
        for state in [state_0, state_1]:
            wp.launch(
                set_body_velocity,
                dim=indices.shape[0],
                inputs=[
                    self._wrap_input_tensor(data),
                    self._wrap_input_tensor(indices),
                    indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                    self._backend.root_body_indices,
                ],
                outputs=[state.body_qd],
                device=str(self._frontend.device),
            )
        # Sync the free-joint generalized velocity from the just-set body_qd so a
        # subsequent set_dof_* -- whose eval_fk rebuilds body_qd from joint_qd --
        # preserves this root velocity instead of losing it to a stale free-joint
        # joint_qd (the IsaacLab reset order sets root velocity, then joint pos/vel).
        # Only the root free joint is written (not a full eval_ik, which would also
        # rewrite actuated joint_qd from the just-changed root body_qd). Only state_0
        # needs it -- the set_dof_* setters read state_0, and after a step state_1 is
        # overwritten and the post-step eval_ik re-syncs whichever state is active.
        if self._newton_stage.solver_is_maximal:
            wp.launch(
                update_root_joint_qd_from_body_qd,
                dim=indices.shape[0],
                inputs=[
                    state_0.body_qd,
                    self._wrap_input_tensor(indices),
                    indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                    self._backend.root_body_indices,
                    self._backend.root_joint_indices,
                    self._model.joint_type,
                    self._model.joint_qd_start,
                    self._model.joint_X_p,
                ],
                outputs=[state_0.joint_qd],
                device=str(self._frontend.device),
            )

    @_no_profile
    def set_masses(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set link masses.

        Args:
            data: Mass values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_link_mass,
            dim=(indices.shape[0], self.max_links),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
            ],
            outputs=[self._model.body_mass],
            device=str(self._frontend.device),
        )
        # Update inverse masses when masses change
        wp.launch(
            update_inv_mass,
            dim=(indices.shape[0], self.max_links),
            inputs=[
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
                self._model.body_mass,
            ],
            outputs=[self._model.body_inv_mass],
            device=str(self._frontend.device),
        )

    @_no_profile
    def set_inertias(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set link inertias.

        Args:
            data: Inertia values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_link_inertia,
            dim=(indices.shape[0], self.max_links, 9),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
            ],
            outputs=[self._model.body_inertia],
            device=str(self._frontend.device),
        )

        # Update inverse inertias
        wp.launch(
            update_inv_inertia,
            dim=(indices.shape[0], self.max_links),
            inputs=[
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
                self._model.body_inertia,
            ],
            outputs=[self._model.body_inv_inertia],
            device=str(self._frontend.device),
        )

    @_no_profile
    def set_dof_positions(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint positions.

        Args:
            data: Position values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        self._check_state()
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_position_indices,
                self.max_dofs,
            ],
            outputs=[self._newton_stage.state_0.joint_q],
            device=str(self._frontend.device),
        )
        if self._newton_stage.solver_is_maximal:
            from newton import eval_fk

            state = self._newton_stage.state_0
            eval_fk(self._model, state.joint_q, state.joint_qd, state)

    @_no_profile
    def set_dof_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint velocities.

        Args:
            data: Velocity values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        self._check_state()
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_velocity_indices,
                self.max_dofs,
            ],
            outputs=[self._newton_stage.state_0.joint_qd],
            device=str(self._frontend.device),
        )
        if self._newton_stage.solver_is_maximal:
            # FK pushes the set joint_qd into body_qd for the solver; it does not write
            # joint_qd, so (unlike a direct body-velocity set) it is safe on this path.
            from newton import eval_fk

            state = self._newton_stage.state_0
            eval_fk(self._model, state.joint_q, state.joint_qd, state)

    @_no_profile
    def set_dof_stiffnesses(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint stiffnesses.

        Args:
            data: Stiffness values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_target_ke],
            device=str(self._frontend.device),
        )
        self._notify_joint_dof_properties_changed()
        self._sync_ctrl_direct_actuator_gains()

    @_no_profile
    def set_dof_dampings(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint dampings.

        Args:
            data: Damping values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_target_kd],
            device=str(self._frontend.device),
        )
        self._notify_joint_dof_properties_changed()
        self._sync_ctrl_direct_actuator_gains()

    @_no_profile
    def set_dof_armatures(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint armatures (rotor inertias).

        Args:
            data: Armature values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_armature],
            device=str(self._frontend.device),
        )
        # Notify solver so MuJoCo updates its DOF properties
        self._notify_joint_dof_properties_changed()

    @_no_profile
    def set_dof_position_targets(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint position targets.

        Args:
            data: Position target values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        control = self._newton_stage.control

        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[control.joint_target_q],
            device=str(self._frontend.device),
        )
        self._sync_ctrl_direct_position_targets()

    @_no_profile
    def set_dof_velocity_targets(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint velocity targets.

        Args:
            data: Velocity target values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        control = self._newton_stage.control
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[control.joint_target_qd],
            device=str(self._frontend.device),
        )

    @_no_profile
    def set_dof_actuation_forces(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint actuation forces/torques.

        Writes directly into control.joint_f which is consumed by the solver.

        Args:
            data: Actuation force values to set.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.

        """
        control = self._newton_stage.control
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[control.joint_f],
            device=str(self._frontend.device),
        )

    def get_dof_actuation_forces(self, copy: bool = copy_data) -> Any:
        """Get joint actuation forces/torques.

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton actuation-force storage.

        Returns:
            Actuation values with shape ``(count, max_dofs)``.

        """
        control = self._newton_stage.control
        if copy:
            if not hasattr(self, "_dof_actuation_forces"):
                self._dof_actuation_forces, self._dof_actuation_forces_desc = self._frontend.create_tensor(
                    (self.count, self.max_dofs), float32
                )
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[control.joint_f, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_actuation_forces)],
                device=str(self._frontend.device),
            )
            return self._dof_actuation_forces
        else:
            return wp.indexedarray(control.joint_f, self._backend.dof_axis_indices)

    def get_dof_max_forces(self, copy: bool = copy_data) -> Any:
        """Get joint maximum forces/torques (effort limits).

        Args:
            copy: Whether to materialize values in reusable frontend storage. If False, return an indexed Warp view of
                Newton effort-limit storage.

        Returns:
            Effort limits with shape ``(count, max_dofs)``.

        """
        if not hasattr(self, "_dof_max_forces"):
            self._dof_max_forces, self._dof_max_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )

        if copy:
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._model.joint_effort_limit, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_max_forces)],
                device=str(self._frontend.device),
            )
            return self._dof_max_forces
        else:
            return wp.indexedarray(self._model.joint_effort_limit, self._backend.dof_axis_indices)

    @_no_profile
    def set_dof_max_forces(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint maximum forces/torques (effort limits).

        Args:
            data: Maximum forces to set, shape (count, max_dofs).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_effort_limit],
            device=str(self._frontend.device),
        )
        self._notify_joint_dof_properties_changed()

    @_no_profile
    def set_dof_limits(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint limits (lower and upper bounds).

        Args:
            data: Joint limits to set, shape (count, max_dofs, 2) where last dim is [lower, upper].
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_limits,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_limit_lower, self._model.joint_limit_upper],
            device=str(self._frontend.device),
        )
        self._notify_joint_dof_properties_changed()

    @_no_profile
    def set_dof_max_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint maximum velocities.

        Args:
            data: Maximum velocities to set, shape (count, max_dofs).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        wp.launch(
            set_dof_attributes,
            dim=(indices.shape[0], self.max_dofs),
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.dof_axis_indices,
                self.max_dofs,
            ],
            outputs=[self._model.joint_velocity_limit],
            device=str(self._frontend.device),
        )
        # Notify solver so Newton updates its joint properties
        self._notify_joint_dof_properties_changed()

    @_no_profile
    def set_dof_drive_model_properties(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Accept joint drive-model properties without modifying Newton.

        Args:
            data: Drive model properties to set, shape (count, max_dofs, 3).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        _log.warning(
            "[isaacsim.physics.newton.tensors.articulation_view] set_dof_drive_model_properties "
            "is not yet implemented for Newton"
        )

    @_no_profile
    def set_dof_friction_properties(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Accept joint-friction properties without modifying Newton.

        Args:
            data: Friction properties to set, shape (count, max_dofs, 3).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        _log.warning(
            "[isaacsim.physics.newton.tensors.articulation_view] set_dof_friction_properties "
            "is not yet implemented for Newton"
        )

    @_no_profile
    def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Accept per-link gravity-disable flags without modifying Newton.

        Args:
            data: Gravity disable flags to set, shape (count, max_links).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.

        """
        _log.warning(
            "[isaacsim.physics.newton.tensors.articulation_view] set_disable_gravities "
            "is not yet implemented for Newton"
        )

    def apply_forces(
        self,
        force_data: Any,
        indices: Any | None = None,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply world-frame forces to selected articulation links.

        This compatibility helper delegates to :meth:`apply_forces_and_torques_at_position`.

        Args:
            force_data: Forces to apply, shape (count, max_links, 3).
            indices: Indices of articulations to apply forces to. If None, applies to all articulations.
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
        """Apply world-frame forces and torques to articulation links at specified positions.

        A force with no position acts at the link center of mass. A supplied position generates the corresponding
        moment about the center of mass. Calls without force or torque data, or with position data but no force data,
        are ignored after logging a diagnostic.

        Args:
            force_data: Forces with shape ``(count, max_links, 3)``, or None to apply only torques.
            torque_data: Torques with shape ``(count, max_links, 3)``, or None to apply only forces.
            position_data: Application positions with shape ``(count, max_links, 3)``, or None to apply forces at each
                link center of mass. Positions require ``force_data``.
            indices: Indices of articulations to apply forces to.
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
            force_tensor = wp.zeros(
                (self.count, self.max_links, 3), dtype=wp.float32, device=str(self._frontend.device)
            )
        if not has_torque:
            torque_tensor = wp.zeros(
                (self.count, self.max_links, 3), dtype=wp.float32, device=str(self._frontend.device)
            )
        if not has_position:
            position_tensor = wp.zeros(
                (self.count, self.max_links, 3), dtype=wp.float32, device=str(self._frontend.device)
            )

        # Apply forces to state
        wp.launch(
            apply_link_forces_at_position,
            dim=(indices_tensor.shape[0], self.max_links),
            inputs=[
                force_tensor,
                torque_tensor,
                position_tensor,
                indices_tensor,
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
                state.body_q,
                self._model.body_com,
                has_force,
                has_torque,
                has_position,
            ],
            outputs=[state.body_f],
            device=str(self._frontend.device),
        )

    def _subset_articulation_rows(self, arr: wp.array) -> wp.array:
        """Gather this view's articulation rows from a model-order result.

        ``eval_jacobian`` / ``eval_mass_matrix`` return one row per *model*
        articulation (``model.articulation_count``) in model order. This view
        may cover a subset in a different order, so gather the rows named by
        ``articulation_indices`` into a contiguous per-view array.

        Args:
            arr: Model-order result to subset.

        Returns:
            Contiguous result containing one row per articulation in view order.

        Raises:
            RuntimeError: If Newton returns no result for a model without articulations.

        """
        if arr is None:
            raise RuntimeError("Newton eval returned None; the model has no articulations")
        gathered = wp.empty((self.count,) + tuple(arr.shape[1:]), dtype=arr.dtype, device=arr.device)
        wp.copy(gathered, wp.indexedarray(arr, [self._backend.articulation_indices]))
        return gathered

    def get_generalized_mass_matrices(self, copy: bool = copy_data) -> Any:
        """Get generalized mass matrices H = JᵀMJ.

        Backed by ``newton.eval_mass_matrix`` (solver-independent); each matrix
        is symmetric positive-definite. Matches the PhysX ``(count, dofs, dofs)``
        layout, where ``dofs`` is the full generalized-coordinate count and
        includes the 6 root DOFs for a floating base — so it is larger than the
        actuated-only ``num-dofs`` metadata (e.g. 14 vs 8 for the Ant).

        Args:
            copy: Compatibility parameter. The method always returns a newly gathered Warp array.

        Returns:
            Mass matrices with shape ``(count, model_dofs, model_dofs)``.

        """
        self._check_state()
        state = self._newton_stage.state_0
        H = newton.eval_mass_matrix(self._model, state)
        return self._subset_articulation_rows(H)

    def get_jacobians(self, copy: bool = copy_data) -> Any:
        """Get spatial Jacobians mapping joint velocities to link body twists.

        Backed by ``newton.eval_jacobian`` (solver-independent); the per-link
        row block satisfies ``J_link @ joint_qd == body_qd[link]``. Rows are
        stacked per-joint 6-vectors, columns index DOFs — the PhysX (rows, cols)
        layout ``(count, max_joints_per_articulation*6, dofs)`` (== links*6 for a
        floating base), where ``dofs`` is the full generalized-coordinate count
        (actuated + 6 root for a floating base), larger than the actuated-only
        ``num-dofs`` metadata.

        Args:
            copy: Compatibility parameter. The method always returns a newly gathered Warp array.

        Returns:
            Jacobians with shape ``(count, max_joints_per_articulation * 6, max_dofs_per_articulation)``.

        """
        self._check_state()
        state = self._newton_stage.state_0
        J = newton.eval_jacobian(self._model, state)
        return self._subset_articulation_rows(J)

    def get_disable_gravities(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for per-link gravity-disable flags.

        Newton does not expose per-link gravity-disable state, so the returned buffer is not populated from the model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_links)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_disable_gravities is not yet implemented for Newton")
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor(
                (self.count, self.max_links), uint8
            )
        return self._disable_gravities

    def get_dof_max_velocities(self, copy: bool = copy_data) -> Any:
        """Get joint maximum velocities.

        Args:
            copy: Whether to refresh reusable frontend storage. If False, return an indexed Warp view of Newton
                velocity-limit storage.

        Returns:
            Velocity limits with shape ``(count, max_dofs)``.

        """
        if not hasattr(self, "_dof_max_velocities"):
            self._dof_max_velocities, self._dof_max_velocities_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        if copy:
            wp.launch(
                get_dof_attributes,
                dim=(self.count, self.max_dofs),
                inputs=[self._model.joint_velocity_limit, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_max_velocities)],
                device=str(self._frontend.device),
            )
            return self._dof_max_velocities
        else:
            return wp.indexedarray(self._model.joint_velocity_limit, self._backend.dof_axis_indices)

    def get_dof_projected_joint_forces(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for projected joint forces.

        Newton does not expose projected joint forces through this view, so the returned buffer is not populated from
        the model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_dof_projected_joint_forces is not yet implemented for Newton")
        if not hasattr(self, "_dof_projected_forces"):
            self._dof_projected_forces, self._dof_projected_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._dof_projected_forces

    def get_gravity_compensation_forces(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for gravity-compensation forces.

        Newton does not expose gravity-compensation forces through this view, so the returned buffer is not populated
        from the model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_gravity_compensation_forces is not yet implemented for Newton")
        if not hasattr(self, "_gravity_comp_forces"):
            self._gravity_comp_forces, self._gravity_comp_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._gravity_comp_forces

    def get_coriolis_and_centrifugal_compensation_forces(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for Coriolis and centrifugal compensation forces.

        Newton does not expose these compensation forces through this view, so the returned buffer is not populated
        from the model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_coriolis_and_centrifugal_compensation_forces is not yet implemented for Newton")
        if not hasattr(self, "_coriolis_forces"):
            self._coriolis_forces, self._coriolis_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._coriolis_forces

    def get_dof_friction_properties(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for joint-friction properties.

        The returned buffer is not populated from the Newton model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs, 3)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_dof_friction_properties is not yet implemented for Newton")
        if not hasattr(self, "_dof_friction_properties"):
            self._dof_friction_properties, _ = self._frontend.create_tensor((self.count, self.max_dofs, 3), float32)
        return self._dof_friction_properties

    def get_drive_types(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for joint drive types.

        The returned buffer is not populated from the Newton model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_drive_types is not yet implemented for Newton")
        if not hasattr(self, "_drive_types"):
            self._drive_types, self._drive_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8)
        return self._drive_types

    def get_dof_drive_model_properties(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for joint drive-model properties.

        The returned buffer is not populated from the Newton model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_dofs, 3)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_dof_drive_model_properties is not yet implemented for Newton")
        if not hasattr(self, "_dof_drive_model_properties"):
            self._dof_drive_model_properties, self._dof_drive_model_properties_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs, 3), float32
            )
        return self._dof_drive_model_properties

    def get_link_incoming_joint_force(self, copy: bool = copy_data) -> Any:
        """Get the compatibility buffer for incoming joint forces.

        The returned buffer is not populated from the Newton model.

        Args:
            copy: Compatibility parameter.

        Returns:
            View-owned buffer with shape ``(count, max_links, 6)``. Its contents are unspecified because Newton does not
            populate the buffer.

        """
        _log.warning("get_link_incoming_joint_force is not yet implemented for Newton")
        if not hasattr(self, "_link_incoming_forces"):
            self._link_incoming_forces, self._link_incoming_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )
        return self._link_incoming_forces

    def check(self) -> bool:
        """Check whether the articulation view contains a valid nonempty selection.

        Returns:
            True if the view has a valid backend with at least one articulation.

        """
        return self._backend is not None and self.count > 0
