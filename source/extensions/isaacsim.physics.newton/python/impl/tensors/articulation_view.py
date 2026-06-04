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

"""Articulation view for Newton physics tensor interface."""

from __future__ import annotations

from typing import Any

import carb
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

# Import tensor types from omni.physics.tensors for compatibility
try:
    from omni.physics.tensors import float32, uint8, uint32
except ImportError:
    # Fallback if not available
    float32 = wp.float32
    uint8 = wp.uint8
    uint32 = wp.uint32

copy_data = True


class NewtonArticulationView:
    """View for articulations in Newton physics simulation.

    Provides tensor-based access to articulation properties like joint positions,
    velocities, transforms, etc.

    Args:
        backend: ArticulationSet backend instance from NewtonSimView.
        frontend: Tensor framework frontend.
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
        """Convert tensor to warp array for kernel output.

        Args:
            tensor: Tensor to convert.

        Returns:
            Warp array or None.
        """
        return convert_to_warp(tensor, self._frontend.device)

    def _check_state(self) -> None:
        """Check if Newton state is initialized.

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
                self._newton_stage.solver.notify_model_changed(newton.solvers.SolverNotifyFlags.JOINT_DOF_PROPERTIES)
            except AttributeError:
                pass

    def _get_ctrl_direct_dof_map(self) -> wp.array | None:
        """Lazily build the DOF-to-CTRL_DIRECT-actuator mapping.

        Returns:
            Warp array mapping template DOF to mujoco:actuator index, or None.
        """
        if not self._ctrl_direct_map_initialized:
            self._ctrl_direct_map_initialized = True
            try:
                self._ctrl_direct_dof_map = build_ctrl_direct_dof_mapping(self._model)
            except Exception as e:
                carb.log_warn(f"Failed to build CTRL_DIRECT DOF mapping: {e}")
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
            inputs=[dof_map, control.joint_target_pos, dofs_per_world, ctrls_per_world],
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
                self._newton_stage.solver.notify_model_changed(newton.solvers.SolverNotifyFlags.ACTUATOR_PROPERTIES)
            except AttributeError:
                pass

    @property
    def count(self) -> int:
        """Number of articulations in this view.

        Returns:
            Number of articulations.
        """
        return self._backend.count

    @property
    def max_dofs(self) -> int:
        """Maximum number of DOFs across all articulations.

        Returns:
            Maximum DOF count.
        """
        return self._backend.max_dofs

    @property
    def max_links(self) -> int:
        """Maximum number of links across all articulations.

        Returns:
            Maximum link count.
        """
        return self._backend.max_links

    @property
    def max_shapes(self) -> int:
        """Maximum number of shapes across all articulations.

        Returns:
            Maximum shape count.
        """
        return self._backend.max_shapes

    @property
    def max_fixed_tendons(self) -> int:
        """Maximum number of fixed tendons across all articulations.

        Returns:
            Maximum fixed tendon count.
        """
        return self._backend.max_fixed_tendons

    @property
    def dof_paths(self) -> Any:
        """DOF paths for all articulations in the view.

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
        """Degree of freedom (DOF) names for all articulations in the view.

        Returns:
            List of DOF name lists, one per articulation.
        """
        dof_names_list = []
        for meta in self._backend.meta_types:
            dof_names_list.append(meta.dof_names)
        return dof_names_list

    @property
    def link_paths(self) -> list[list[str]]:
        """Link paths for all articulations in the view.

        Returns:
            List of link path lists, one per articulation.
        """
        link_paths_list = []
        for meta in self._backend.meta_types:
            link_paths_list.append(meta.link_paths)
        return link_paths_list

    @property
    def link_names(self) -> list[list[str]]:
        """Link names for all articulations in the view.

        Returns:
            List of link name lists, one per articulation.
        """
        link_names_list = []
        for meta in self._backend.meta_types:
            link_names_list.append(meta.link_names)
        return link_names_list

    @property
    def joint_paths(self) -> list[list[str]]:
        """Joint paths for all articulations in the view.

        Returns:
            List of joint path lists, one per articulation.
        """
        joint_paths_list = []
        for meta in self._backend.meta_types:
            joint_paths_list.append(meta.joint_paths)
        return joint_paths_list

    @property
    def joint_names(self) -> list[list[str]]:
        """Joint names for all articulations in the view.

        Returns:
            List of joint name lists, one per articulation.
        """
        joint_names_list = []
        for meta in self._backend.meta_types:
            joint_names_list.append(meta.joint_names)
        return joint_names_list

    @property
    def prim_paths(self) -> list[str]:
        """Articulation root prim paths.

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
        """Shared metadata type for articulations (if all same type).

        Returns:
            ArticulationMetaType object if all articulations have the same structure, None if different types or no articulations.
        """
        if self.count == 0:
            return None
        return self._backend.shared_metatype

    @property
    def is_homogeneous(self) -> bool:
        """Whether all articulations in the view have the same structure.

        Returns:
            True if all articulations have the same number of DOFs, links, and joints.
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
        r"""Jacobian matrix shape for articulations.

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
        r"""Generalized mass matrix shape for articulations.

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
        """Update simulation timestamp.

        Args:
            dt: Time delta.
        """
        self._sim_timestamp += dt  # type: ignore[assignment]

    @carb.profiler.profile
    def get_root_transforms(self, copy: bool = copy_data) -> Any:
        """Get root body transforms [position(3) + quaternion(4)].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 7).
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

    @carb.profiler.profile
    def get_root_velocities(self, copy: bool = copy_data) -> Any:
        """Get root body velocities [linear(3) + angular(3)].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, 6).
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

    def get_masses(self, copy: bool = copy_data) -> Any:
        """Get link masses.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links).
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links).
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links, 9).
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links, 9).
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
            copy: Whether to return a copy.

        Returns:
            Flat tensor that should be reshaped to (count, max_links, 7).
            Each COM has 7 values: position (3) + quaternion (4) in scalar-first (w,x,y,z) format.
            Newton stores COM as offset in body frame. Position is read from Newton,
            orientation is cached (Newton doesn't support COM orientation).
        """
        if not hasattr(self, "_coms"):
            self._coms, self._coms_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32)
            if self.count > 0 and self.max_links > 0:
                coms_warp = self._convert_to_warp(self._coms)
                coms_warp.fill_(0.0)  # type: ignore[union-attr]

                coms_np = coms_warp.numpy()  # type: ignore[union-attr]
                coms_np[:, :, 6] = 1.0
                coms_warp.assign(coms_np)  # type: ignore[union-attr]
        wp.launch(
            get_link_com_position_only,
            dim=(self.count, self.max_links, 7),
            inputs=[self._model.body_com, self._backend.link_indices],
            outputs=[self._convert_to_warp(self._coms)],
            device=str(self._frontend.device),
        )
        return self._coms

    def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set link center of mass positions and orientations.

        Args:
            data: Tensor of COM data, shape should be (count, max_links, 7) or flat.
            indices: Articulation indices.
            indices_mask: Optional mask for indices.
        """
        if not hasattr(self, "_coms"):
            self._coms, self._coms_desc = self._frontend.create_tensor((self.count, self.max_links, 7), float32)
            if self.count > 0 and self.max_links > 0:
                coms_warp = self._convert_to_warp(self._coms)
                coms_warp.fill_(0.0)  # type: ignore[union-attr]

                coms_np = coms_warp.numpy()  # type: ignore[union-attr]
                coms_np[:, :, 6] = 1.0
                coms_warp.assign(coms_np)  # type: ignore[union-attr]
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

    @carb.profiler.profile
    def get_dof_positions(self, copy: bool = copy_data) -> Any:
        """Get joint positions.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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

    @carb.profiler.profile
    def get_dof_velocities(self, copy: bool = copy_data) -> Any:
        """Get joint velocities.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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

    @carb.profiler.profile
    def get_dof_limits(self, copy: bool = copy_data) -> Any:
        """Get joint limits [lower, upper].

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs, 2).
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

    @carb.profiler.profile
    def get_dof_stiffnesses(self, copy: bool = copy_data) -> Any:
        """Get joint stiffnesses (for position control).

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs) on CPU (host memory).
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

    @carb.profiler.profile
    def get_dof_dampings(self, copy: bool = copy_data) -> Any:
        """Get joint dampings (for velocity control).

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs) on CPU (host memory).
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

    @carb.profiler.profile
    def get_dof_armatures(self, copy: bool = copy_data) -> Any:
        """Get joint armatures (rotor inertias).

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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
                inputs=[control.joint_target_pos, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_position_targets)],
                device=str(self._frontend.device),
            )
            return self._dof_position_targets
        else:
            return wp.indexedarray(control.joint_target_pos, self._backend.dof_axis_indices)

    def get_dof_velocity_targets(self, copy: bool = copy_data) -> Any:
        """Get joint velocity targets (for velocity control).

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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
                inputs=[control.joint_target_vel, self._backend.dof_axis_indices, self.max_dofs],
                outputs=[self._convert_to_warp(self._dof_velocity_targets)],
                device=str(self._frontend.device),
            )
            return self._dof_velocity_targets
        else:
            return wp.indexedarray(control.joint_target_vel, self._backend.dof_axis_indices)

    def _update_articulation_state(
        self, indices: Any, indices_mask: Any | None = None, update_positions: bool = True
    ) -> None:
        """Helper function to update articulation joint coordinates and evaluate forward kinematics.

            This should be called after modifying root transforms or velocities to ensure
            joint coordinates are consistent and all link transforms are updated.

        Args:
            indices: Indices of articulations to update.
            indices_mask: Optional mask for the indices.
            update_positions: Whether to update positions (True) or just velocities (False).
        """
        from newton import eval_fk

        # Only evaluate forward kinematics for position updates
        # For velocity updates, FK might reset the velocities we just set
        if update_positions:
            state = self._newton_stage.state_0
            eval_fk(self._model, state.joint_q, state.joint_qd, state)

    @carb.profiler.profile
    def set_root_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set root body transforms.

            Automatically detects whether each articulation is fixed-base or floating-base
            and handles them appropriately:
            - Fixed-base: Updates Model's joint_X_p (parent transforms)
            - Floating-base: Updates State's joint coordinates
            After setting transforms, forward kinematics is evaluated to update all link positions.

        Args:
            data: Root transforms to set (dtype=wp.transform).
            indices: Indices of articulations to update.
            indices_mask: Optional mask for the indices.
        """
        self._check_state()

        state = self._newton_stage.state_0

        # For fixed-base articulations, set the root transform in Model.joint_X_p
        # For floating-base articulations, set the root transform in State.joint_q via body_q

        # First, set the transforms in joint_X_p (for fixed-base articulations)
        wp.launch(
            set_body_pose,
            dim=indices.shape[0],
            inputs=[
                self._wrap_input_tensor(data),
                self._wrap_input_tensor(indices),
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.root_body_indices,
            ],
            outputs=[self._model.joint_X_p],  # Set in Model, not State
            device=str(self._frontend.device),
        )

        # Also set in body_q for floating-base articulations and FK processing
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

        # Update joint coordinates and evaluate forward kinematics
        self._update_articulation_state(indices, indices_mask, update_positions=True)

    @carb.profiler.profile
    def set_root_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set root body velocities.

        Automatically detects whether each articulation is fixed-base or floating-base:
        - Fixed-base: Sets body velocities directly (no joint coordinate updates needed)
        - Floating-base: Updates State's joint velocity coordinates
        After setting velocities, forward kinematics may be evaluated if needed.

        Args:
            data: Root velocities to set (dtype=wp.spatial_vector).
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
        # For floating-base articulations, we need to also update joint velocity coordinates
        # For fixed-base, the body velocities are sufficient
        # Note: We don't call FK for velocity updates as it might reset the velocities

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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
            outputs=[control.joint_target_pos],
            device=str(self._frontend.device),
        )
        self._sync_ctrl_direct_position_targets()

    @carb.profiler.profile
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
            outputs=[control.joint_target_vel],
            device=str(self._frontend.device),
        )

    @carb.profiler.profile
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
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

    @carb.profiler.profile
    def set_dof_drive_model_properties(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint drive model properties.

        Args:
            data: Drive model properties to set, shape (count, max_dofs, 3).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.
        """
        carb.log_warn(
            "[isaacsim.physics.newton.tensors.articulation_view] set_dof_drive_model_properties is not yet implemented for Newton"
        )

    @carb.profiler.profile
    def set_dof_friction_properties(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set joint friction properties.

        Args:
            data: Friction properties to set, shape (count, max_dofs, 3).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.
        """
        carb.log_warn(
            "[isaacsim.physics.newton.tensors.articulation_view] set_dof_friction_properties is not yet implemented for Newton"
        )

    @carb.profiler.profile
    def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None) -> None:
        """Set gravity disable flags for links.

        Args:
            data: Gravity disable flags to set, shape (count, max_links).
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.
        """
        carb.log_warn(
            "[isaacsim.physics.newton.tensors.articulation_view] set_disable_gravities is not yet implemented for Newton"
        )

    @carb.profiler.profile
    def update_joints(self, indices: Any, indices_mask: Any | None = None) -> None:
        """Update joint states after setting positions/velocities.

        This evaluates forward kinematics to update body transforms from joint states.

        Args:
            indices: Articulation indices to update.
            indices_mask: Optional mask for indices.
        """
        self._check_state()
        state = self._newton_stage.state_0
        model = self._model

        # Assign root states to joint coordinates
        wp.launch(
            assign_articulation_root_states,
            dim=indices.shape[0],
            inputs=[
                state.body_q,
                state.body_qd,
                indices,
                indices_mask,
                self._backend.articulation_indices,
                self._backend.root_body_indices,
                True,  # update_fixed_base_articulations
                False,  # relative_transforms
                model.joint_type,
                model.articulation_start,
                model.joint_q_start,
                model.joint_qd_start,
            ],
            outputs=[
                self._backend.q_ik,
                self._backend.qd_ik,
                model.joint_X_p,
            ],
            device=model.device,
        )

        # Evaluate forward kinematics
        newton.eval_fk(
            model,
            self._backend.q_ik,
            self._backend.qd_ik,
            state,
        )

    def apply_forces(
        self,
        force_data: Any,
        indices: Any | None = None,
        is_global: bool = True,
        indices_mask: Any | None = None,
    ) -> None:
        """Apply forces to articulation links (deprecated, use apply_forces_and_torques_at_position).

        Args:
            force_data: Forces to apply, shape (count, max_links, 3).
            indices: Indices of articulations to apply forces to. If None, applies to all articulations.
            is_global: If True, forces are in global frame. If False, in link local frame.
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
        """Apply forces and torques to articulation links at specified positions.

        This function provides flexible force application to articulation links:
        - Apply forces at link centers or at specified positions
        - Apply torques directly to links
        - Work in global or local coordinate frames
        - When position is specified with force, automatically computes induced torque

        Args:
            force_data: Forces to apply, shape (count, max_links, 3). Can be None.
            torque_data: Torques to apply, shape (count, max_links, 3). Can be None.
            position_data: Positions where forces are applied, shape (count, max_links, 3). Can be None.
                If specified with force_data, the force is applied at this position
                relative to the link's center of mass, generating additional torque.
            indices: Indices of articulations to apply forces to.
            is_global: If True, force/torque/position are in global frame. If False, in link local frame.
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
            dim=(indices_tensor.shape[0], self.max_links),  # type: ignore[union-attr]
            inputs=[
                force_tensor,
                torque_tensor,
                position_tensor,
                indices_tensor,
                indices_mask if indices_mask is None else self._wrap_input_tensor(indices_mask),
                self._backend.link_indices,
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

    def get_generalized_mass_matrices(self, copy: bool = copy_data) -> Any:
        """Get generalized mass matrices.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs, max_dofs).
        """
        carb.log_warn("get_generalized_mass_matrices is not yet implemented for Newton")
        if not hasattr(self, "_mass_matrices"):
            self._mass_matrices, self._mass_matrices_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs, self.max_dofs), float32
            )
        return self._mass_matrices

    def get_jacobians(self, copy: bool = copy_data) -> Any:
        """Get Jacobian matrices.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, num_links_effective, 6, num_dofs).
        """
        carb.log_warn("get_jacobians is not yet implemented for Newton")
        if not hasattr(self, "_jacobians"):
            rows, cols = self.jacobian_shape
            num_links_effective = rows // 6
            self._jacobians, self._jacobians_desc = self._frontend.create_tensor(
                (self.count, num_links_effective, 6, cols), float32
            )
        return self._jacobians

    def get_disable_gravities(self, copy: bool = copy_data) -> Any:
        """Get gravity disable flags for links.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links).
        """
        carb.log_warn("get_disable_gravities is not yet implemented for Newton")
        if not hasattr(self, "_disable_gravities"):
            self._disable_gravities, self._disable_gravities_desc = self._frontend.create_tensor(
                (self.count, self.max_links), uint8
            )
        return self._disable_gravities

    def get_dof_max_velocities(self, copy: bool = copy_data) -> Any:
        """Get joint maximum velocities.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
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
        """Get projected joint forces.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
        """
        carb.log_warn("get_dof_projected_joint_forces is not yet implemented for Newton")
        if not hasattr(self, "_dof_projected_forces"):
            self._dof_projected_forces, self._dof_projected_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._dof_projected_forces

    def get_gravity_compensation_forces(self, copy: bool = copy_data) -> Any:
        """Get gravity compensation forces.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
        """
        carb.log_warn("get_gravity_compensation_forces is not yet implemented for Newton")
        if not hasattr(self, "_gravity_comp_forces"):
            self._gravity_comp_forces, self._gravity_comp_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._gravity_comp_forces

    def get_coriolis_and_centrifugal_compensation_forces(self, copy: bool = copy_data) -> Any:
        """Get Coriolis and centrifugal compensation forces.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs).
        """
        carb.log_warn("get_coriolis_and_centrifugal_compensation_forces is not yet implemented for Newton")
        if not hasattr(self, "_coriolis_forces"):
            self._coriolis_forces, self._coriolis_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs), float32
            )
        return self._coriolis_forces

    def get_dof_friction_properties(self, copy: bool = copy_data) -> Any:
        """Get joint friction properties.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs, 3) where the last dimension contains
            [static_friction, dynamic_friction, viscous_friction].
        """
        carb.log_warn("get_dof_friction_properties is not yet implemented for Newton")
        if not hasattr(self, "_dof_friction_properties"):
            self._dof_friction_properties, _ = self._frontend.create_tensor((self.count, self.max_dofs, 3), float32)
        return self._dof_friction_properties

    def get_drive_types(self, copy: bool = copy_data) -> Any:
        """Get joint drive types.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs) of type uint8.
        """
        carb.log_warn("get_drive_types is not yet implemented for Newton")
        if not hasattr(self, "_drive_types"):
            self._drive_types, self._drive_types_desc = self._frontend.create_tensor((self.count, self.max_dofs), uint8)
        return self._drive_types

    def get_dof_drive_model_properties(self, copy: bool = copy_data) -> Any:
        """Get joint drive model properties.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_dofs, 3) where the last dimension contains
            [speed_effort_gradient, max_actuator_velocity, velocity_dependent_resistance].
        """
        carb.log_warn("get_dof_drive_model_properties is not yet implemented for Newton")
        if not hasattr(self, "_dof_drive_model_properties"):
            self._dof_drive_model_properties, self._dof_drive_model_properties_desc = self._frontend.create_tensor(
                (self.count, self.max_dofs, 3), float32
            )
        return self._dof_drive_model_properties

    def get_link_incoming_joint_force(self, copy: bool = copy_data) -> Any:
        """Get incoming joint forces for each link.

        Args:
            copy: Whether to return a copy.

        Returns:
            Tensor of shape (count, max_links, 6).
        """
        carb.log_warn("get_link_incoming_joint_force is not yet implemented for Newton")
        if not hasattr(self, "_link_incoming_forces"):
            self._link_incoming_forces, self._link_incoming_forces_desc = self._frontend.create_tensor(
                (self.count, self.max_links, 6), float32
            )
        return self._link_incoming_forces

    def check(self) -> bool:
        """Check if the articulation view is valid and has articulations.

        Returns:
            True if the view has a valid backend with at least one articulation.
        """
        return self._backend is not None and self.count > 0
