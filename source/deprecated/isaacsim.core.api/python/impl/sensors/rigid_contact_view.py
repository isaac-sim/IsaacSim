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

"""Provides high level functions to deal with rigid prims for tracking their contact interactions through filters."""

from __future__ import annotations

import carb
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.api.simulation_context.simulation_context import SimulationContext
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from pxr import PhysxSchema

torch = import_module("torch")


class RigidContactView(object):
    """Provides high level functions to deal with rigid prims (one or many) that track their contacts through filters.

    as well as their attributes/properties.

    This class wraps all matching rigid prims found at the regex provided at the ``prim_paths_expr`` argument

    .. warning::

        The rigid prim view object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_paths_expr: Prim paths regex to encapsulate all prims that match it.
            example: "/World/Env[1-5]/Cube" will match /World/Env1/Cube,
            /World/Env2/Cube..etc.
            (a non regex prim path can also be used to encapsulate one rigid prim).  Additionally a
            list of regex can be provided. example ["/World/Env[1-5]/Cube", "/World/Env[10-19]/Cube"].
        filter_paths_expr: List of prim paths regex to filter the contacts for each corresponding
            prim_paths_expr. Example: ["/World/envs/env_2/Xform"] will filter the contacts corresponding to
            the expression passed.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        prepare_contact_sensors: If rigid prims in the view are not cloned from a prim in a prepared state,
            (although slow for large number of prims) this ensures that
            appropriate physics settings are applied on all the prim in the view.
        disable_stablization: Disables the contact stabilization parameter in the physics context.
        disable_stabilization: Overrides disable_stablization when provided; disables contact stabilization in the physics context.
        max_contact_count: Maximum number of contact data to report when detailed contact information is needed.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.cloner import GridCloner
        >>> from isaacsim.core.api.sensors import RigidContactView
        >>> from pxr import UsdGeom
        >>>
        >>> env_zero_path = "/World/envs/env_0"
        >>> num_envs = 5
        >>>
        >>> # clone the environment (num_envs)
        >>> cloner = GridCloner(spacing=0)
        >>> cloner.define_base_env(env_zero_path)
        >>> UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform", "Xform")
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform/Cube", "Cube")
        >>> # position the cubes on top of each other
        >>> position_offsets = np.zeros((num_envs, 3))
        >>> position_offsets[:, 2] = np.arange(num_envs) * 1.1
        >>> env_pos = cloner.clone(
        ...     source_prim_path=env_zero_path,
        ...     prim_paths=cloner.generate_paths("/World/envs/env", num_envs),
        ...     position_offsets=position_offsets
        ...     copy_from_source=True,
        ... )
        >>>
        >>> # wrap the prims
        >>> prims = RigidContactView(
        ...     prim_paths_expr="/World/envs/env.*/Xform",
        ...     name="RigidContactView_view",
        ...     filter_paths_expr=["/World/envs/env_2/Xform"],
        ...     max_contact_count=10,
        ... )
        >>> prims
        <isaacsim.core.api.sensors.rigid_contact_view.RigidContactView object at 0x7f8d4eb1abf0>

    """

    def __init__(
        self,
        prim_paths_expr: str | list[str],
        filter_paths_expr: list[str] | list[list[str]],
        name: str = "rigid_contact_view",
        prepare_contact_sensors: bool = True,
        disable_stablization: bool = True,  # noqa: N803 — kept for backward compatibility
        disable_stabilization: bool | None = None,
        max_contact_count: int = 0,
    ) -> None:
        self._name = name
        if not isinstance(prim_paths_expr, list):
            prim_paths_expr = [prim_paths_expr]
        if len(filter_paths_expr) == 0:
            filter_paths_expr = [[]]
        elif not isinstance(filter_paths_expr[0], list):
            filter_paths_expr = [filter_paths_expr]
        self._regex_prim_paths = prim_paths_expr
        self._regex_filter_paths = filter_paths_expr
        self._prim_paths = None
        self._physics_view = None
        self._num_shapes = None
        self._num_filters = None
        self.max_contact_count = max_contact_count

        _disable_stab = disable_stabilization if disable_stabilization is not None else disable_stablization
        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
            if _disable_stab:
                SimulationContext.instance().get_physics_context().enable_stabilization(False)
        else:
            import isaacsim.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = None
            self._backend_utils = np_utils

        if prepare_contact_sensors:
            self._prim_paths = []
            for prim_path_expression in prim_paths_expr:
                self._prim_paths = self._prim_paths + find_matching_prim_paths(prim_path_expression)
            for path in self._prim_paths:
                self._prepare_contact_reporter(get_prim_at_path(path))

            self._filter_paths = []
            for expr in filter_paths_expr:
                for group_expr in expr:
                    matched = find_matching_prim_paths(group_expr)
                    self._filter_paths.extend(matched)
                    for path in matched:
                        self._prepare_contact_reporter(get_prim_at_path(path))
        return

    @property
    def num_shapes(self) -> int:
        """Number of rigid shapes for the prims in the view.

        Returns:
            Number of rigid shapes for the prims in the view.

        Example:

        .. code-block:: python

            >>> prims.num_shapes
            5

        """
        return self._num_shapes

    @property
    def num_filters(self) -> int:
        """Number of filters bodies that report their contact with the rigid prims.

        Returns:
            Number of filters bodies that report their contact with the rigid prims.

        Example:

        .. code-block:: python

            >>> prims.num_filters
            1

        """
        return self._num_filters

    def _prepare_contact_reporter(self, prim_at_path: object) -> None:
        """Prepares the contact reporter by removing the contact thresholds.

        Args:
            prim_at_path: Prim at the specified path to prepare for contact reporting.

        """
        if prim_at_path.HasAPI(PhysxSchema.PhysxContactReportAPI):
            cr_api = PhysxSchema.PhysxContactReportAPI(prim_at_path)
        else:
            cr_api = PhysxSchema.PhysxContactReportAPI.Apply(prim_at_path)

        cr_api.CreateThresholdAttr().Set(0)

    def is_physics_handle_valid(self) -> bool:
        """Check if rigid prim view's physics handler is initialized.

        .. warning::

            If the physics handler is not valid many of the methods that requires PhysX will return None.

        Returns:
            True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.

        Example:

        .. code-block:: python

            >>> prims.is_physics_handle_valid()
            True

        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API.

        .. note::

            If the rigid prim view has been added to the world scene (e.g., ``world.scene.add(prims)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        .. warning::

            This method needs to be called after each hard reset (e.g., Stop + Play on the timeline)
            before interacting with any other class method.

        Args:
            physics_sim_view: Current physics simulation view.

        Example:

        .. code-block:: python

            >>> prims.initialize()

        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info(f"initializing view for {self._name}")
        self._physics_sim_view = physics_sim_view
        self._physics_view = physics_sim_view.create_rigid_contact_view(
            [regular_expression.replace(".*", "*") for regular_expression in self._regex_prim_paths],
            filter_patterns=[
                [path.replace(".*", "*") for path in regex_filter_path]
                for regex_filter_path in self._regex_filter_paths
            ],
            max_contact_data_count=self.max_contact_count,
        )
        carb.log_info(f"Rigid Contact View Device: {self._device}")
        self._num_shapes = self._physics_view.sensor_count
        self._num_filters = self._physics_view.filter_count
        return

    def get_net_contact_forces(
        self, indices: np.ndarray | torch.Tensor | wp.array | None = None, clone: bool = True, dt: float = 1.0
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get the overall net contact forces on the prims in the view with respect to the world's frame.

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: Time step multiplier to convert the underlying impulses to forces. The function returns contact impulses if the default dt is used

        Returns:
            Net contact forces of the prims with shape (M,3).

        Example:

        .. code-block:: python

            >>> # get the net contact force on all rigid bodies. Returned shape is (5, 3).
            >>> prims.get_net_contact_forces()
            [[ 1.8731881e-03  5.4876995e-03  1.6408131e+02]
             [ 1.9060407e-02 -2.2513291e-02  1.6358723e+02]
             [-2.1011427e-02  3.5647806e-02  1.6371542e+02]
             [ 9.4006478e-05 -9.3258200e-03  1.6348369e+02]
             [ 9.3709816e-05 -9.2963902e-03  1.6296776e+02]]
            >>>
            >>> # get the net contact force on the rigid bodies for the first, middle and last of the 5 envs
            >>> prims.get_net_contact_forces(indices=np.array([0, 2, 4]))
            [[ 1.8731881e-03  5.4876995e-03  1.6408131e+02]
             [-2.1011427e-02  3.5647806e-02  1.6371542e+02]
             [ 9.3709816e-05 -9.2963902e-03  1.6296776e+02]]

        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            net_contact_forces = self._physics_view.get_net_contact_forces(dt)
            if clone:
                net_contact_forces = self._backend_utils.clone_tensor(net_contact_forces, device=self._device)
            return net_contact_forces[indices]
        else:
            carb.log_warn("Physics Simulation View is not created yet")
            return None

    def get_contact_force_matrix(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get the contact forces between the prims in the view and the filter prims.

        E.g., a matrix of dimension ``(num_shapes, num_filters, 3)`` where ``num_filters`` is
        determined according to the ``filter_paths_expr`` parameter

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: Time step multiplier to convert the underlying impulses to forces. The function returns contact impulses if the default dt is used

        Returns:
            Net contact forces between the view prim and the filter prims with shape (M, self.num_filters, 3).

        Example:

        .. code-block:: python

            >>> # get the contact forces between the prims and the filter prims (the cube in the middle)
            >>> prims.get_contact_force_matrix()
            [[[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
             [[ 2.2649009e-02 -1.3710857e-02 -4.9047806e+02]]
             [[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
             [[-3.3276828e-03 -2.3870371e-02  3.2733777e+02]]
             [[ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]]

        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            net_contact_forces = self._physics_view.get_contact_force_matrix(dt)
            if clone:
                net_contact_forces = self._backend_utils.clone_tensor(net_contact_forces, device=self._device)
            return net_contact_forces[indices, :, :]
        else:
            carb.log_warn("Physics Simulation View is not created yet")
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
        They both have a dimension of ``(num_shapes, num_filters)`` where ``num_filters`` is determined
        according to the ``filter_paths_expr`` parameter

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: Time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for normal forces with shape (max_contact_count, 1), points with shape (max_contact_count, 3), normals with shape (max_contact_count, 3),
            and distances with shape (max_contact_count, 1), as well as two tensors with shape (M, self.num_filters)
            to indicate the starting index and the number of contact data points per pair in the aforementioned buffers.

        Example:

        .. code-block:: python

            >>> # get detailed contact force data between the prims and the filter prims (the cube in the middle)
            >>> data = prims.get_contact_force_data()
            >>> data[0]  # normal forces
            [[-168.53815]
             [ -89.57392]
             [-156.10307]
             [ -75.17234]
             [  98.0681 ]
             [  52.56319]
             [ 108.26558]
             [  67.62025]
             [   0.     ]
             [   0.     ]]
            >>> data[1]  # points
            [[ 0.4948182  -0.49902824  1.5001888 ]
             [ 0.4950411   0.49933064  1.5001996 ]
             [-0.5024581   0.49930018  1.5001924 ]
             [-0.5024276  -0.49880558  1.5001817 ]
             [-0.5023767   0.497138    2.5001519 ]
             [-0.502735   -0.49877006  2.5001822 ]
             [ 0.4947694  -0.4989927   2.500226  ]
             [ 0.4949917   0.49677914  2.5001955 ]
             [ 0.          0.          0.        ]
             [ 0.          0.          0.        ]]
            >>> data[2]  # normals
            [[-4.3812128e-05  3.0501858e-05  1.0000000e+00]
             [-4.3812128e-05  3.0501858e-05  1.0000000e+00]
             [-4.3812128e-05  3.0501858e-05  1.0000000e+00]
             [-4.3812128e-05  3.0501858e-05  1.0000000e+00]
             [ 2.1408198e-06 -7.0731985e-05  1.0000000e+00]
             [ 2.1408198e-06 -7.0731985e-05  1.0000000e+00]
             [ 2.1408198e-06 -7.0731985e-05  1.0000000e+00]
             [ 2.1408198e-06 -7.0731985e-05  1.0000000e+00]
             [ 0.0000000e+00  0.0000000e+00  0.0000000e+00]
             [ 0.0000000e+00  0.0000000e+00  0.0000000e+00]]
            >>> data[3]  # distances
            [[ 3.7143487e-05]
             [-4.0254322e-06]
             [-4.0531158e-05]
             [ 6.0737699e-07]
             [ 1.9307560e-04]
             [ 9.2272363e-05]
             [ 4.6372414e-05]
             [ 1.4718286e-04]
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
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            (
                forces,
                points,
                normals,
                distances,
                pair_contacts_count,
                pair_contacts_start_indices,
            ) = self._physics_view.get_contact_data(dt)
            if clone:
                forces = self._backend_utils.clone_tensor(forces, device=self._device)
                points = self._backend_utils.clone_tensor(points, device=self._device)
                normals = self._backend_utils.clone_tensor(normals, device=self._device)
                distances = self._backend_utils.clone_tensor(distances, device=self._device)
                pair_contacts_count = self._backend_utils.clone_tensor(pair_contacts_count, device=self._device)
                pair_contacts_start_indices = self._backend_utils.clone_tensor(
                    pair_contacts_start_indices, device=self._device
                )
            return (
                forces,
                points,
                normals,
                distances,
                pair_contacts_count[indices, :],
                pair_contacts_start_indices[indices, :],
            )

        else:
            carb.log_warn("Physics Simulation View is not created yet")
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
        """Gets friction data between the prims in the view and the filter prims. Specifically, this method provides frictional contact forces,.

        and points. The data in reported for number of anchor points that includes tangential forces in a single tangent direction to contact normal.
        Given to the dynamic nature of collision between bodies, this method will provide buffers of friction data arranged sequentially for each pair.
        The starting index and the number of contact data points for each pair in this stream can be realized from pair_contacts_start_indices,
        and pair_contacts_count tensors. They both have a dimension of (self.num_shapes, self.num_filters) where filter_count is determined
        according to the filter_paths_expr parameter.

        Args:
            indices: Indicies to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: Time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for tangential forces per patch (at number of anchor points, each in a single directions)
            with shape (max_contact_count, 3), points with shape (max_contact_count, 3),
            as well as two tensors with shape (M, self.num_filters) to indicate the starting index and the number of
            contact data points per pair in the aforementioned buffers.

        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            forces, points, pair_contacts_count, pair_contacts_start_indices = self._physics_view.get_friction_data(dt)
            if clone:
                forces = self._backend_utils.clone_tensor(forces, device=self._device)
                points = self._backend_utils.clone_tensor(points, device=self._device)
                pair_contacts_count = self._backend_utils.clone_tensor(pair_contacts_count, device=self._device)
                pair_contacts_start_indices = self._backend_utils.clone_tensor(
                    pair_contacts_start_indices, device=self._device
                )
            return forces, points, pair_contacts_count[indices, :], pair_contacts_start_indices[indices, :]

        else:
            carb.log_warn("Physics Simulation View is not created yet")
            return None
