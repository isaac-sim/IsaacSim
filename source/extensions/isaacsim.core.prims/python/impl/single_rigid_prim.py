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
from typing import Optional, Sequence

import numpy as np
from isaacsim.core.utils.types import DynamicState

from .rigid_prim import RigidPrim
from .single_prim_wrapper import _SinglePrimWrapper


class SingleRigidPrim(_SinglePrimWrapper):
    """High level wrapper to deal with a rigid body prim (only one rigid body prim) and its attributes/properties.

    .. warning::

        The rigid body object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    .. note::

        If the prim does not already have the Rigid Body API applied to it before init, it will apply it.

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create.
        name (str, optional): shortname to be used as a key by Scene class.
                              Note: needs to be unique if the object is added to the Scene.
                              Defaults to "rigid_prim".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                   Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                      (with respect to its parent prim). shape is (3, ).
                                                      Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                      (depends if translation or position is specified).
                                                      quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                      Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties
                                                (i.e: translate, orient and scale) ONLY and in that order.
                                                Set this parameter to False if the object were cloned using using
                                                the cloner api in isaacsim.core.cloner. Defaults to True.
        mass (Optional[float], optional): mass in kg. Defaults to None.
        density (Optional[float], optional): density. Defaults to None.
        linear_velocity (Optional[np.ndarray], optional): linear velocity in the world frame. Defaults to None.
        angular_velocity (Optional[np.ndarray], optional): angular velocity in the world frame. Defaults to None.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.prims import SingleRigidPrim
        >>>
        >>> # create a Cube at the given path
        >>> stage_utils.get_current_stage().DefinePrim("/World/Xform", "Xform")
        >>> stage_utils.get_current_stage().DefinePrim("/World/Xform/Cube", "Cube")
        >>>
        >>> # wrap the prim as rigid prim
        >>> prim = SingleRigidPrim("/World/Xform")
        >>> prim
        <isaacsim.core.prims.single_rigid_prim.SingleRigidPrim object at 0x7fc4a7f56e90>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "rigid_prim",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        reset_xform_properties: bool = True,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:
        from isaacsim.core.simulation_manager import SimulationManager

        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        if position is not None:
            position = self._backend_utils.convert(position, self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if translation is not None:
            translation = self._backend_utils.convert(translation, self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if scale is not None:
            scale = self._backend_utils.convert(scale, self._device)
            scale = self._backend_utils.expand_dims(scale, 0)
        if visible is not None:
            visible = self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        if mass is not None:
            mass = self._backend_utils.create_tensor_from_list([mass], dtype="float32", device=self._device)
        if density is not None:
            density = self._backend_utils.create_tensor_from_list([density], dtype="float32", device=self._device)
        if linear_velocity is not None:
            linear_velocity = self._backend_utils.expand_dims(linear_velocity, 0)
        if angular_velocity is not None:
            angular_velocity = self._backend_utils.expand_dims(angular_velocity, 0)
        self._rigid_prim_view = RigidPrim(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            reset_xform_properties=reset_xform_properties,
            masses=mass,
            densities=density,
            linear_velocities=linear_velocity,
            angular_velocities=angular_velocity,
        )
        _SinglePrimWrapper.__init__(self, view=self._rigid_prim_view)
        return

    def set_linear_velocity(self, velocity: np.ndarray):
        """Set the linear velocity of the rigid body in stage

        .. warning::

            This method will immediately set the rigid prim state

        Args:
            velocity (np.ndarray): linear velocity to set the rigid prim to. Shape (3,).
        """
        velocity = self._backend_utils.expand_dims(velocity, 0)
        self._rigid_prim_view.set_linear_velocities(velocities=velocity)
        return

    def get_linear_velocity(self) -> np.ndarray:
        """Get the linear velocity of the rigid body

        Returns:
            np.ndarray: current linear velocity of the the rigid prim. Shape (3,).

        Example:

        .. code-block:: python

            >>> prim.get_linear_velocity()
            [ 1.0812164e-04  6.1415871e-05 -2.1341663e-04]
        """
        velocities = self._rigid_prim_view.get_linear_velocities()
        return velocities[0]

    def set_angular_velocity(self, velocity: np.ndarray) -> None:
        """Set the angular velocity of the rigid body in stage

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocity (np.ndarray): angular velocity to set the rigid prim to. Shape (3,).

        """
        velocity = self._backend_utils.expand_dims(velocity, 0)
        self._rigid_prim_view.set_angular_velocities(velocities=velocity)
        return

    def get_angular_velocity(self):
        """Get the angular velocity of the rigid body

        Returns:
            np.ndarray: current angular velocity of the the rigid prim. Shape (3,).

        """
        velocities = self._rigid_prim_view.get_angular_velocities()
        return velocities[0]

    def set_com(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Set the center of mass pose of the rigid body

        Args:
            position (np.ndarray): center of mass position. Shape (3,).
            orientation (np.ndarray): center of mass orientation. Shape (4,).

        """
        position = self._backend_utils.expand_dims(position, 0)
        orientation = self._backend_utils.expand_dims(orientation, 0)
        self._rigid_prim_view.set_coms(positions=position, orientations=orientation)
        return

    def get_com(self) -> float:
        """Get the center of mass pose of the rigid body

        Returns:
            np.ndarray: position of the center of mass of the rigid body.
            np.ndarray: orientation of the center of mass of the rigid body.

        """
        positions, orientations = self._rigid_prim_view.get_coms()
        return positions[0], orientations[0]

    def set_mass(self, mass: float) -> None:
        """Set the mass of the rigid body

        Args:
            mass (float): mass of the rigid body in kg.

        Example:

        .. code-block:: python

            >>> prim.set_mass(1.0)
        """
        masses = self._backend_utils.create_tensor_from_list([mass], dtype="float32")
        self._rigid_prim_view.set_masses(masses=masses)
        return

    def get_mass(self) -> float:
        """Get the mass of the rigid body

        Returns:
            float: mass of the rigid body in kg.

        Example:

        .. code-block:: python

            >>> prim.get_mass()
            0
        """
        masses = self._rigid_prim_view.get_masses()
        return masses[0]

    def set_density(self, density: float) -> None:
        """Set the density of the rigid body

        Args:
            mass (float): density of the rigid body.

        Example:

        .. code-block:: python

            >>> prim.set_density(0.9)
        """
        densities = self._backend_utils.create_tensor_from_list([density], dtype="float32")
        self._rigid_prim_view.set_densities(densities)
        return

    def get_density(self) -> float:
        """Get the density of the rigid body

        Returns:
            float: density of the rigid body.

        Example:

        .. code-block:: python

            >>> prim.get_density()
            0
        """
        return self._rigid_prim_view.get_densities()[0]

    def set_sleep_threshold(self, threshold: float) -> None:
        """Set the threshold for the rigid body to enter a sleep state

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details

        Args:
            threshold (float): Mass-normalized kinetic energy threshold below which
                                an actor may go to sleep. Range: [0, inf)
                                Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
                                Units: distance^2 / second^2.

        Example:

        .. code-block:: python

            >>> prim.set_sleep_threshold(1e-5)
        """
        thresholds = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._rigid_prim_view.set_sleep_thresholds(thresholds)
        return

    def get_sleep_threshold(self) -> float:
        """Get the threshold for the rigid body to enter a sleep state

        Search for *Rigid Body Dynamics* > *Sleeping* in |physx_docs| for more details

        Returns:
            float: Mass-normalized kinetic energy threshold below which
                    an actor may go to sleep. Range: [0, inf)
                    Defaults: 0.00005 * tolerancesSpeed* tolerancesSpeed
                    Units: distance^2 / second^2.

        Example:

        .. code-block:: python

            >>> prim.get_sleep_threshold()
            5e-05
        """
        return self._rigid_prim_view.get_sleep_thresholds()[0]

    def enable_rigid_body_physics(self) -> None:
        """Enable the rigid body physics

        When enabled, the object will be moved by external forces such as gravity and collisions

        Example:

        .. code-block:: python

            >>> prim.enable_rigid_body_physics()
        """
        self._rigid_prim_view.enable_rigid_body_physics()
        return

    def disable_rigid_body_physics(self) -> None:
        """Disable the rigid body physics

        When disabled, the object will not be moved by external forces such as gravity and collisions

        Example:

        .. code-block:: python

            >>> prim.disable_rigid_body_physics()
        """
        self._rigid_prim_view.disable_rigid_body_physics()
        return

    def set_default_state(
        self,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:
        """Set the default state of the prim (position, orientation and linear and angular velocities),
        that will be used after each reset

        .. note::

            The default states will be set during post-reset (e.g., calling ``.post_reset()`` or ``world.reset()`` methods)

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim.
                                    quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                    Defaults to None, which means left unchanged.
            linear_velocity (np.ndarray): linear velocity to set the rigid prim to. Shape (3,).
            angular_velocity (np.ndarray): angular velocity to set the rigid prim to. Shape (3,).

        Example:

        .. code-block:: python

            >>> prim.set_default_state(
            ...     position=np.array([1.0, 2.0, 3.0]),
            ...     orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            ...     linear_velocity=np.array([0.0, 0.0, 0.0]),
            ...     angular_velocity=np.array([0.0, 0.0, 0.0])
            ... )
            >>>
            >>> prim.post_reset()
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if linear_velocity is not None:
            linear_velocity = self._backend_utils.convert(linear_velocity, device=self._device)
            linear_velocity = self._backend_utils.expand_dims(linear_velocity, 0)
        if angular_velocity is not None:
            angular_velocity = self._backend_utils.convert(angular_velocity, device=self._device)
            angular_velocity = self._backend_utils.expand_dims(angular_velocity, 0)
        self._rigid_prim_view.set_default_state(
            positions=position,
            orientations=orientation,
            linear_velocities=linear_velocity,
            angular_velocities=angular_velocity,
        )
        return

    def get_default_state(self) -> DynamicState:
        """Get the default rigid body state (position, orientation and linear and angular velocities)

        Returns:
            DynamicState: returns the default state of the prim that is used after each reset

        Example:

        .. code-block:: python

            >>> state = prim.get_default_state()
            >>> state
            <isaacsim.core.utils.types.DynamicState object at 0x7f7411fcbe20>
            >>> state.position
            [-7.8622378e-07  1.4450421e-06  1.6135601e-07]
            >>> state.orientation
            [ 9.9999994e-01 -2.7194994e-07  2.9607077e-07  2.7016510e-08]
            >>> state.linear_velocity
            [0. 0. 0.]
            >>> state.angular_velocity
            [0. 0. 0.]
        """
        view_default_state = self._rigid_prim_view.get_default_state()
        default_state = self._dynamics_view_state_conversion(view_default_state)
        return default_state

    def get_current_dynamic_state(self) -> DynamicState:
        """Get the current rigid body state (position, orientation and linear and angular velocities)

        Returns:
            DynamicState: the dynamic state of the rigid body prim

        Example:

        .. code-block:: python

            >>> # for the example the rigid body is in free fall
            >>> state = prim.get_current_dynamic_state()
            >>> state
            <isaacsim.core.utils.types.DynamicState object at 0x7f740b36f670>
            >>> state.position
            [  0.99999857   2.0000017  -74.2862    ]
            >>> state.orientation
            [ 1.0000000e+00 -2.3961178e-07 -4.9891562e-09  4.9388258e-09]
            >>> state.linear_velocity
            [  0.        0.      -38.09554]
            >>> state.angular_velocity
            [0. 0. 0.]
        """
        view_default_state = self._rigid_prim_view.get_current_dynamic_state()
        return self._dynamics_view_state_conversion(view_default_state)

    def _dynamics_view_state_conversion(self, view_state):
        # TODO: a temp function
        position = None
        orientation = None
        linear_velocity = None
        angular_velocity = None
        if view_state.positions is not None:
            position = view_state.positions[0]
        if view_state.orientations is not None:
            orientation = view_state.orientations[0]
        if view_state.linear_velocities is not None:
            linear_velocity = view_state.linear_velocities[0]
        if view_state.angular_velocities is not None:
            angular_velocity = view_state.angular_velocities[0]
        return DynamicState(
            position=position,
            orientation=orientation,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )
