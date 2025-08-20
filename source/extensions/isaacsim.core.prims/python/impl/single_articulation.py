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
from typing import List, Optional, Sequence, Union

import carb
import numpy as np
import omni.kit.app
from isaacsim.core.utils.types import ArticulationAction, JointsState

from .articulation import Articulation
from .single_prim_wrapper import _SinglePrimWrapper


class SingleArticulation(_SinglePrimWrapper):
    """High level wrapper to deal with an articulation prim (only one articulation prim) and its attributes/properties.

    .. warning::

        The articulation object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create.
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "articulation".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. Shape is (3, ).
                                                    Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). Shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). Shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. Shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties
                                                (i.e: translate, orient and scale) ONLY and in that order.
                                                Set this parameter to False if the object were cloned using using
                                                the cloner api in isaacsim.core.cloner. Defaults to True.
        articulation_controller (Optional[ArticulationController], optional): a custom ArticulationController which
                                                                              inherits from it. Defaults to creating the
                                                                              basic ArticulationController.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.prims import SingleArticulation
        >>>
        >>> usd_path = "/home/<user>/Documents/Assets/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        >>> prim_path = "/World/envs/env_0/panda"
        >>>
        >>> # load the Franka Panda robot USD file
        >>> stage_utils.add_reference_to_stage(usd_path, prim_path)
        >>>
        >>> # wrap the prim as an articulation
        >>> prim = SingleArticulation(prim_path=prim_path, name="franka_panda")
        >>> prim
        <isaacsim.core.prims.single_articulation.SingleArticulation object at 0x7fdd165bf520>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "articulation",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        reset_xform_properties: bool = True,
        articulation_controller: Optional["ArticulationController"] = None,
        enable_residual_reports: bool = False,
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
        self._articulation_view = Articulation(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            reset_xform_properties=reset_xform_properties,
            enable_residual_reports=enable_residual_reports,
        )
        self._articulation_controller = articulation_controller
        if self._articulation_controller is None:
            from isaacsim.core.api.controllers.articulation_controller import ArticulationController

            self._articulation_controller = ArticulationController()
        _SinglePrimWrapper.__init__(self, view=self._articulation_view)
        return

    @property
    def handles_initialized(self) -> bool:
        """Check if articulation handler is initialized

        Returns:
            bool: whether the handler was initialized

        Example:

        .. code-block:: python

            >>> prim.handles_initialized
            True
        """
        return (
            self._articulation_view.is_physics_handle_valid()
            and self._articulation_controller._articulation_view is not None
        )

    @property
    def num_dof(self) -> int:
        """Number of DOF of the articulation

        Returns:
            int: amount of DOFs

        Example:

        .. code-block:: python

            >>> prim.num_dof
            9
        """
        return self._articulation_view.num_dof

    @property
    def num_bodies(self) -> int:
        """Number of articulation links

        Returns:
            int: number of links

        Example:

        .. code-block:: python

            >>> prim.num_bodies
            9
        """
        return self._articulation_view.num_bodies

    @property
    def dof_properties(self) -> np.ndarray:
        """Articulation DOF properties

        .. list-table:: DOF properties
            :header-rows: 1

            * - Index
              - Property name
              - Description
            * - 0
              - ``type``
              - DOF type: invalid/unknown/uninitialized (0), rotation (1), translation (2)
            * - 1
              - ``hasLimits``
              - Whether the DOF has limits
            * - 2
              - ``lower``
              - Lower DOF limit (in radians or meters)
            * - 3
              - ``upper``
              - Upper DOF limit (in radians or meters)
            * - 4
              - ``driveMode``
              - Drive mode for the DOF: force (1), acceleration (2)
            * - 5
              - ``maxVelocity``
              - Maximum DOF velocity. In radians/s, or stage_units/s
            * - 6
              - ``maxEffort``
              - Maximum DOF effort. In N or N*stage_units
            * - 7
              - ``stiffness``
              - DOF stiffness
            * - 8
              - ``damping``
              - DOF damping

        Returns:
            np.ndarray: named NumPy array of shape (num_dof, 9)

        Example:

        .. code-block:: python

            >>> # get properties for all DOFs
            >>> prim.dof_properties
            [(1,  True, -2.8973,  2.8973, 1, 1.0000000e+01, 5220., 60000., 3000.)
             (1,  True, -1.7628,  1.7628, 1, 1.0000000e+01, 5220., 60000., 3000.)
             (1,  True, -2.8973,  2.8973, 1, 5.9390470e+36, 5220., 60000., 3000.)
             (1,  True, -3.0718, -0.0698, 1, 5.9390470e+36, 5220., 60000., 3000.)
             (1,  True, -2.8973,  2.8973, 1, 5.9390470e+36,  720., 25000., 3000.)
             (1,  True, -0.0175,  3.7525, 1, 5.9390470e+36,  720., 15000., 3000.)
             (1,  True, -2.8973,  2.8973, 1, 1.0000000e+01,  720.,  5000., 3000.)
             (2,  True,  0.    ,  0.04  , 1, 3.4028235e+38,  720.,  6000., 1000.)
             (2,  True,  0.    ,  0.04  , 1, 3.4028235e+38,  720.,  6000., 1000.)]
            >>>
            >>> # property names
            >>> prim.dof_properties.dtype.names
            ('type', 'hasLimits', 'lower', 'upper', 'driveMode', 'maxVelocity', 'maxEffort', 'stiffness', 'damping')
            >>>
            >>> # get DOF upper limits
            >>> prim.dof_properties["upper"]
            [ 2.8973  1.7628  2.8973 -0.0698  2.8973  3.7525  2.8973  0.04    0.04  ]
            >>>
            >>> # get the last DOF (panda_finger_joint2) upper limit
            >>> prim.dof_properties["upper"][8]  # or prim.dof_properties[8][3]
            0.04
        """
        dtype = np.dtype(
            [
                ("type", int),
                ("hasLimits", bool),
                ("lower", float),
                ("upper", float),
                ("driveMode", int),
                ("maxVelocity", float),
                ("maxEffort", float),
                ("stiffness", float),
                ("damping", float),
            ]
        )
        properties = np.zeros(self.num_dof, dtype=dtype)
        properties["type"] = self._articulation_view.get_dof_types()[0]
        properties["lower"] = self._articulation_view.get_dof_limits()[0][:, 0]
        properties["upper"] = self._articulation_view.get_dof_limits()[0][:, 1]
        properties["hasLimits"] = properties["lower"] < properties["upper"]
        properties["driveMode"] = self._articulation_view.get_drive_types()[0]
        properties["maxEffort"] = self._articulation_view.get_max_efforts()[0]
        properties["maxVelocity"] = self._articulation_view.get_joint_max_velocities()[0]
        stiffnesses, dampings = self._articulation_view.get_gains()
        properties["stiffness"] = stiffnesses[0]
        properties["damping"] = dampings[0]
        # properties = self._dc_interface.get_articulation_dof_properties(self._handle)
        # properties["lower"] = self._articulation_view.get_dof_limits()[0][:, 0]
        # properties["upper"] = self._articulation_view.get_dof_limits()[0][:, 1]
        # print(properties)
        return properties

    @property
    def dof_names(self) -> List[str]:
        """List of prim names for each DOF.

        Returns:
            list(string): prim names

        Example:

        .. code-block:: python

            >>> prim.dof_names
            ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
             'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        """
        return self._articulation_view.dof_names

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and an articulation view using PhysX tensor API

        .. note::

            If the articulation has been added to the world scene (e.g., ``world.scene.add(prim)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        .. warning::

            This method needs to be called after each hard reset (e.g., Stop + Play on the timeline)
            before interacting with any other class method.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.

        Example:

        .. code-block:: python

            >>> prim.initialize()
        """
        carb.log_info("initializing handles for {}".format(self.prim_path))
        self._articulation_controller.initialize(self._articulation_view)
        self._articulation_view.initialize(physics_sim_view=physics_sim_view)
        return

    def get_dof_index(self, dof_name: str) -> int:
        """Get a DOF index given its name

        Args:
            dof_name (str): name of the DOF

        Returns:
            int: DOF index

        Example:

        .. code-block:: python

            >>> prim.get_dof_index("panda_finger_joint2")
            8
        """
        return self._articulation_view.get_dof_index(dof_name=dof_name)

    def get_articulation_body_count(self) -> int:
        """Get the number of bodies (links) that make up the articulation

        Returns:
            int: amount of bodies

        Example:

        .. code-block:: python

            >>> prim.get_articulation_body_count()
            12
        """
        return self._articulation_view.get_articulation_body_count()

    def disable_gravity(self) -> None:
        """Keep gravity from affecting the robot

        Example:

        .. code-block:: python

            >>> prim.disable_gravity()
        """

        self._articulation_view.set_body_disable_gravity(
            self._backend_utils.create_tensor_from_list([[True] * self.num_bodies], dtype="uint8")
        )
        return

    def enable_gravity(self) -> None:
        """Gravity will affect the robot

        Example:

        .. code-block:: python

            >>> prim.enable_gravity()
        """
        self._articulation_view.set_body_disable_gravity(
            self._backend_utils.create_tensor_from_list([[False] * self.num_bodies], dtype="uint8")
        )
        return

    def set_world_velocity(self, velocity: np.ndarray):
        """Set the articulation root velocity

        Args:
            velocity (np.ndarray): linear and angular velocity to set the root prim to. Shape (6,).

        """
        velocity = self._backend_utils.expand_dims(velocity, 0)
        self._articulation_view.set_velocities(velocities=velocity)
        return

    def get_world_velocity(self) -> np.ndarray:
        """Get the articulation root velocity

        Returns:
            np.ndarray: current velocity of the the root prim. Shape (3,).

        """
        velocities = self._articulation_view.get_velocities()
        return velocities[0]

    def get_position_residual(self, report_max: Optional[bool] = True) -> float:
        """Get physics solver position residuals for articulations. This is the residual across all joints that are part of articulations.

        The solver residuals are computed according to impulse variation normalized by the effective mass.

        Args:
            report_max (Optional[bool]): whether to report max or RMS residual. Defaults to True, i.e. max criteria

        Returns:
            float: solver position/velocity max/rms residual.
        """
        result = self._articulation_view.get_position_residuals(report_max=report_max)
        if result is not None:
            return result[0]
        return result

    def get_velocity_residual(self, report_max: Optional[bool] = True) -> float:
        """Get physics solver velocity residuals for articulations. This is the residual across all joints that are part of articulations.

        The solver residuals are computed according to impulse variation normalized by the effective mass.

        Args:
            report_max (Optional[bool]): whether to report max or RMS residual. Defaults to True, i.e. max criteria

        Returns:
            float: solver velocity max/rms residual.
        """
        result = self._articulation_view.get_velocity_residuals(report_max=report_max)
        if result is not None:
            return result[0]
        return result

    def set_joint_positions(
        self, positions: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """Set the articulation joint positions

        .. warning::

            This method will immediately set (teleport) the affected joints to the indicated value.
            Use the ``apply_action`` method to control robot joints.

        Args:
            positions (np.ndarray): articulation joint positions
            joint_indices (Optional[Union[list, np.ndarray]], optional): indices to specify which joints to manipulate.
                                                                         Defaults to None (all joints)

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_linear_velocity``, ``set_angular_velocity``, ``set_joint_positions``,
            ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set all the robot joints
            >>> prim.set_joint_positions(np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]))
            >>>
            >>> # set only the fingers in closed position: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
            >>> prim.set_joint_positions(np.array([0.04, 0.04]), joint_indices=np.array([7, 8]))
        """
        positions = self._backend_utils.expand_dims(positions, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_positions(positions=positions, joint_indices=joint_indices)
        return

    def get_joint_positions(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the articulation joint positions

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Returns:
            np.ndarray: all or selected articulation joint positions

        Example:

        .. code-block:: python

            >>> # get all joint positions
            >>> prim.get_joint_positions()
            [ 1.1999920e-02 -5.6962633e-01  1.3480479e-08 -2.8105433e+00  6.8284894e-06
              3.0301569e+00  7.3234749e-01  3.9912373e-02  3.9999999e-02]
            >>>
            >>> # get finger positions: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> prim.get_joint_positions(joint_indices=np.array([7, 8]))
            [0.03991237  3.9999999e-02]
        """
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_joint_positions(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def set_joint_velocities(
        self, velocities: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """Set the articulation joint velocities

        .. warning::

            This method will immediately set the affected joints to the indicated value.
            Use the ``apply_action`` method to control robot joints.

        Args:
            velocities (np.ndarray): articulation joint velocities
            joint_indices (Optional[Union[list, np.ndarray]], optional): indices to specify which joints to manipulate.
                                                                         Defaults to None (all joints)

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_linear_velocity``, ``set_angular_velocity``, ``set_joint_positions``,
            ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set all the robot joint velocities to 0.0
            >>> prim.set_joint_velocities(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            >>>
            >>> # set only the fingers velocities: panda_finger_joint1 (7) and panda_finger_joint2 (8) to -0.01
            >>> prim.set_joint_velocities(np.array([-0.01, -0.01]), joint_indices=np.array([7, 8]))
        """
        velocities = self._backend_utils.expand_dims(velocities, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_velocities(velocities=velocities, joint_indices=joint_indices)
        return

    def set_joint_efforts(self, efforts: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None) -> None:
        """Set the articulation joint efforts

        .. note::

            This method can be used for effort control. For this purpose, there must be no joint drive
            or the stiffness and damping must be set to zero.

        Args:
            efforts (np.ndarray): articulation joint efforts
            joint_indices (Optional[Union[list, np.ndarray]], optional): indices to specify which joints to manipulate.
                                                                         Defaults to None (all joints)

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_linear_velocity``, ``set_angular_velocity``, ``set_joint_positions``,
            ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set all the robot joint efforts to 0.0
            >>> prim.set_joint_efforts(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            >>>
            >>> # set only the fingers efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 10
            >>> prim.set_joint_efforts(np.array([10, 10]), joint_indices=np.array([7, 8]))
        """
        efforts = self._backend_utils.expand_dims(efforts, 0)
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        self._articulation_view.set_joint_efforts(efforts=efforts, joint_indices=joint_indices)
        return

    def get_joint_velocities(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the articulation joint velocities

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Returns:
            np.ndarray: all or selected articulation joint velocities

        Example:

        .. code-block:: python

            >>> # get all joint velocities
            >>> prim.get_joint_velocities()
            [ 1.91603772e-06 -7.67638255e-03 -2.19138826e-07  1.10636465e-02 -4.63412944e-05
              3.48245539e-02  8.84692147e-02  5.40335372e-04 1.02849208e-05]
            >>>
            >>> # get finger velocities: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> prim.get_joint_velocities(joint_indices=np.array([7, 8]))
            [5.4033537e-04 1.0284921e-05]
        """
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_joint_velocities(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_measured_joint_efforts(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Returns the efforts computed/measured by the physics solver of the joint forces in the DOF motion direction

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Raises:
            Exception: If the handlers are not initialized

        Returns:
            np.ndarray: all or selected articulation joint measured efforts

        Example:

        .. code-block:: python

            >>> # get all joint efforts
            >>> prim.get_measured_joint_efforts()
            [ 2.7897308e-06 -6.9083519e+00 -3.6398471e-06  1.9158335e+01 -4.3552645e-06
              1.1866090e+00 -4.7079347e-06  3.2339853e-04 -3.2044132e-04]
            >>>
            >>> # get finger efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> prim.get_measured_joint_efforts(joint_indices=np.array([7, 8]))
            [ 0.0003234  -0.00032044]
        """
        if self._articulation_view.is_physics_handle_valid() is None:
            raise Exception("handles are not initialized yet")
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_measured_joint_efforts(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_applied_joint_efforts(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the efforts applied to the joints set by the ``set_joint_efforts`` method

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Raises:
            Exception: If the handlers are not initialized

        Returns:
            np.ndarray: all or selected articulation joint applied efforts

        Example:

        .. code-block:: python

            >>> # get all applied joint efforts
            >>> prim.get_applied_joint_efforts()
            [ 0.  0.  0.  0.  0.  0.  0.  0.  0.]
            >>>
            >>> # get finger applied efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> prim.get_applied_joint_efforts(joint_indices=np.array([7, 8]))
            [0.  0.]
        """
        if self._articulation_view.is_physics_handle_valid() is None:
            raise Exception("handles are not initialized yet")
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_applied_joint_efforts(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_measured_joint_forces(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the measured joint reaction forces and torques (link incoming joint forces and torques) to external loads.

        Forces and torques are reported in the local body reference frame (child joint frame of the link's incoming joint).

        .. note::

            Since the *name->index* map for joints has not been exposed yet,
            it is possible to access the joint names and their indices through the articulation metadata.

            .. code-block:: python

                prim._articulation_view._metadata.joint_names  # list of names
                prim._articulation_view._metadata.joint_indices  # dict of name: index

            To retrieve a specific row for the link incoming joint force/torque use ``joint_index + 1``

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Raises:
            Exception: If the handlers are not initialized

        Returns:
            np.ndarray: measured joint forces and torques. Shape is (num_joint + 1, 6). Row index 0 is the incoming
            joint of the base link. For the last dimension the first 3 values are for forces and the last 3 for torques

        Example:

        .. code-block:: python

            >>> # get all measured joint forces and torques
            >>> prim.get_measured_joint_forces()
            [[ 0.0000000e+00  0.0000000e+00  0.0000000e+00  0.0000000e+00  0.0000000e+00  0.0000000e+00]
             [ 1.4995076e+02  4.2574748e-06  5.6364370e-04  4.8701895e-05 -6.9072924e+00  3.1881387e-05]
             [-2.8971717e-05 -1.0677823e+02 -6.8384506e+01 -6.9072924e+00 -5.4927128e-05  6.1222494e-07]
             [ 8.7120995e+01 -4.3871860e-05 -5.5795174e+01  5.3687054e-05 -2.4538563e+01  1.3333466e-05]
             [ 5.3519474e-05 -4.8109909e+01  6.0709282e+01  1.9157074e+01 -5.9258469e-05  8.2744418e-07]
             [-3.1691040e+01  2.3313689e-04  3.9990173e+01 -5.8968733e-05 -1.1863431e+00  2.2335558e-05]
             [-1.0809851e-04  1.5340537e+01 -1.5458489e+01  1.1863426e+00  6.1094368e-05 -1.5940281e-05]
             [-7.5418940e+00 -5.0814648e+00 -5.6512990e+00 -5.6385466e-05  3.8859999e-01 -3.4943256e-01]
             [ 4.7421460e+00 -3.1945827e+00  3.5528181e+00  5.5852943e-05  8.4794536e-03  7.6405057e-03]
             [ 4.0760727e+00  2.1640673e-01 -4.0513167e+00 -5.9565349e-04  1.1407082e-02  2.1432268e-06]
             [ 5.1680198e-03 -9.7754575e-02 -9.7093947e-02 -8.4155556e-12 -1.2910691e-12 -1.9347857e-11]
             [-5.1910793e-03  9.7588278e-02 -9.7106412e-02  8.4155573e-12  1.2910637e-12 -1.9347855e-11]]
            >>>
            >>> # get measured joint force and torque for the fingers
            >>> metadata = prim._articulation_view._metadata
            >>> joint_indices = 1 + np.array([
            ...     metadata.joint_indices["panda_finger_joint1"],
            ...     metadata.joint_indices["panda_finger_joint2"],
            ... ])
            >>> joint_indices
            [10 11]
            >>> prim.get_measured_joint_forces(joint_indices)
            [[ 5.1680198e-03 -9.7754575e-02 -9.7093947e-02 -8.4155556e-12 -1.2910691e-12 -1.9347857e-11]
             [-5.1910793e-03  9.7588278e-02 -9.7106412e-02  8.4155573e-12  1.2910637e-12 -1.9347855e-11]]
        """
        if self._articulation_view.is_physics_handle_valid() is None:
            raise Exception("handles are not initialized yet")
        if joint_indices is not None:
            joint_indices = self._backend_utils.expand_dims(joint_indices, 0)
        result = self._articulation_view.get_measured_joint_forces(joint_indices=joint_indices)
        if result is not None:
            result = result[0]
        return result

    def get_joints_default_state(self) -> JointsState:
        """Get the default joint states (positions and velocities).

        Returns:
            JointsState: an object that contains the default joint positions and velocities

        Example:

        .. code-block:: python

            >>> state = prim.get_joints_default_state()
            >>> state
            <isaacsim.core.utils.types.JointsState object at 0x7f04a0061240>
            >>>
            >>> state.positions
            [ 0.012  -0.57000005  0.  -2.81  0.  3.037  0.785398  0.04  0.04 ]
            >>> state.velocities
            [0. 0. 0. 0. 0. 0. 0. 0. 0.]
        """
        joints_state = self._articulation_view.get_joints_default_state()
        if joints_state is None:
            return None
        return JointsState(positions=joints_state.positions[0], velocities=joints_state.velocities[0], efforts=None)

    def set_joints_default_state(
        self,
        positions: Optional[np.ndarray] = None,
        velocities: Optional[np.ndarray] = None,
        efforts: Optional[np.ndarray] = None,
    ) -> None:
        """Set the joint default states (positions, velocities and/or efforts) to be applied after each reset.

        .. note::

            The default states will be set during post-reset (e.g., calling ``.post_reset()`` or ``world.reset()`` methods)

        Args:
            positions (Optional[np.ndarray], optional): joint positions. Defaults to None.
            velocities (Optional[np.ndarray], optional): joint velocities. Defaults to None.
            efforts (Optional[np.ndarray], optional): joint efforts. Defaults to None.

        Example:

        .. code-block:: python

            >>> # configure default joint states
            >>> prim.set_joints_default_state(
            ...     positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]),
            ...     velocities=np.zeros(shape=(prim.num_dof,)),
            ...     efforts=np.zeros(shape=(prim.num_dof,))
            ... )
            >>>
            >>> # set default states during post-reset
            >>> prim.post_reset()
        """
        if positions is not None:
            positions = self._backend_utils.expand_dims(positions, 0)
        if velocities is not None:
            velocities = self._backend_utils.expand_dims(velocities, 0)
        if efforts is not None:
            efforts = self._backend_utils.expand_dims(efforts, 0)
        self._articulation_view.set_joints_default_state(positions=positions, velocities=velocities, efforts=efforts)
        return

    def get_joints_state(self) -> JointsState:
        """Get the current joint states (positions and velocities)

        Returns:
            JointsState: an object that contains the current joint positions and velocities

        Example:

        .. code-block:: python

            >>> state = prim.get_joints_state()
            >>> state
            <isaacsim.core.utils.types.JointsState object at 0x7f02f6df57b0>
            >>>
            >>> state.positions
            [ 1.1999920e-02 -5.6962633e-01  1.3480479e-08 -2.8105433e+00 6.8284894e-06
              3.0301569e+00  7.3234749e-01  3.9912373e-02  3.9999999e-02]
            >>> state.velocities
            [ 1.91603772e-06 -7.67638255e-03 -2.19138826e-07  1.10636465e-02 -4.63412944e-05
              245539e-02  8.84692147e-02  5.40335372e-04  1.02849208e-05]
        """
        joints_state = self._articulation_view.get_joints_state()
        if joints_state is None:
            return None
        return JointsState(positions=joints_state.positions[0], velocities=joints_state.velocities[0], efforts=None)

    def get_articulation_controller(self) -> "ArticulationController":
        """Get the articulation controller

        .. note::

            If no ``articulation_controller`` was passed during class instantiation, a default controller
            of type ``ArticulationController`` (a Proportional-Derivative controller that can apply position targets,
            velocity targets and efforts) will be used

        Returns:
            ArticulationController: articulation controller

        Example:

        .. code-block:: python

            >>> prim.get_articulation_controller()
            <isaacsim.core.api.controllers.articulation_controller.ArticulationController object at 0x7f04a0060190>
        """
        return self._articulation_controller

    def set_linear_velocity(self, velocity: np.ndarray) -> None:
        """Set the linear velocity of the root articulation prim

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocity (np.ndarray): 3D linear velocity vector. Shape (3,).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_linear_velocity``, ``set_angular_velocity``, ``set_joint_positions``,
            ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> prim.set_linear_velocity(np.array([0.1, 0.0, 0.0]))
        """
        if velocity is not None:
            velocity = self._backend_utils.expand_dims(velocity, 0)
        return self._articulation_view.set_linear_velocities(velocities=velocity)

    def get_linear_velocity(self) -> np.ndarray:
        """Get the linear velocity of the root articulation prim

        Returns:
            np.ndarray:  3D linear velocity vector. Shape (3,)

        Example:

        .. code-block:: python

            >>> prim.get_linear_velocity()
            [0. 0. 0.]
        """
        result = self._articulation_view.get_linear_velocities()
        if result is not None:
            result = result[0]
        return result

    def set_angular_velocity(self, velocity: np.ndarray) -> None:
        """Set the angular velocity of the root articulation prim

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocity (np.ndarray): 3D angular velocity vector. Shape (3,)

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_linear_velocity``, ``set_angular_velocity``, ``set_joint_positions``,
            ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> prim.set_angular_velocity(np.array([0.1, 0.0, 0.0]))
        """
        if velocity is not None:
            velocity = self._backend_utils.expand_dims(velocity, 0)
        self._articulation_view.set_angular_velocities(velocities=velocity)

    def get_angular_velocity(self) -> np.ndarray:
        """Get the angular velocity of the root articulation prim

        Returns:
            np.ndarray: 3D angular velocity vector. Shape (3,)

        Example:

        .. code-block:: python

            >>> prim.get_angular_velocity()
            [0. 0. 0.]
        """
        result = self._articulation_view.get_angular_velocities()
        if result is not None:
            result = result[0]
        return result

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Apply joint positions, velocities and/or efforts to control an articulation

        Args:
            control_actions (ArticulationAction): actions to be applied for next physics step.
            indices (Optional[Union[list, np.ndarray]], optional): degree of freedom indices to apply actions to.
                                                                   Defaults to all degrees of freedom.

        .. hint::

            High stiffness makes the joints snap faster and harder to the desired target,
            and higher damping smoothes but also slows down the joint's movement to target

            * For position control, set relatively high stiffness and low damping (to reduce vibrations)
            * For velocity control, stiffness must be set to zero with a non-zero damping
            * For effort control, stiffness and damping must be set to zero

        Example:

        .. code-block:: python

            >>> from isaacsim.core.utils.types import ArticulationAction
            >>>
            >>> # move all the robot joints to the indicated position
            >>> action = ArticulationAction(joint_positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]))
            >>> prim.apply_action(action)
            >>>
            >>> # close the robot fingers: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
            >>> action = ArticulationAction(joint_positions=np.array([0.0, 0.0]), joint_indices=np.array([7, 8]))
            >>> prim.apply_action(action)
        """
        self._articulation_controller.apply_action(control_actions=control_actions)
        return

    def get_applied_action(self) -> ArticulationAction:
        """Get the last applied action

        Returns:
            ArticulationAction: last applied action. Note: a dictionary is used as the object's string representation

        Example:

        .. code-block:: python

            >>> # last applied action: joint_positions -> [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]
            >>> prim.get_applied_action()
            {'joint_positions': [0.0, -1.0, 0.0, -2.200000047683716, 0.0, 2.4000000953674316,
                                 0.800000011920929, 0.03999999910593033, 0.03999999910593033],
             'joint_velocities': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             'joint_efforts': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        """
        return self._articulation_controller.get_applied_action()

    def set_solver_position_iteration_count(self, count: int) -> None:
        """Set the solver (position) iteration count for the articulation

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        .. warning::

            Setting a higher number of iterations may improve the fidelity of the simulation, although it may affect its performance.

        Args:
            count (int): position iteration count

        Example:

        .. code-block:: python

            >>> prim.set_solver_position_iteration_count(64)
        """
        count = self._backend_utils.create_tensor_from_list([count], dtype="int32")
        self._articulation_view.set_solver_position_iteration_counts(count)
        return

    def get_solver_position_iteration_count(self) -> int:
        """Get the solver (position) iteration count for the articulation

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        Returns:
            int: position iteration count

        Example:

        .. code-block:: python

            >>> prim.get_solver_position_iteration_count()
            32
        """
        return self._articulation_view.get_solver_position_iteration_counts()[0]

    def set_solver_velocity_iteration_count(self, count: int):
        """Set the solver (velocity) iteration count for the articulation

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        .. warning::

            Setting a higher number of iterations may improve the fidelity of the simulation, although it may affect its performance.

        Args:
            count (int): velocity iteration count

        Example:

        .. code-block:: python

            >>> prim.set_solver_velocity_iteration_count(64)
        """
        count = self._backend_utils.create_tensor_from_list([count], dtype="int32")
        self._articulation_view.set_solver_velocity_iteration_counts(count)
        return

    def get_solver_velocity_iteration_count(self) -> int:
        """Get the solver (velocity) iteration count for the articulation

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        Returns:
            int: velocity iteration count

        Example:

        .. code-block:: python

            >>> prim.get_solver_velocity_iteration_count()
            32
        """
        return self._articulation_view.get_solver_velocity_iteration_counts()[0]

    def set_stabilization_threshold(self, threshold: float) -> None:
        """Set the mass-normalized kinetic energy below which the articulation may participate in stabilization

        Search for *Stabilization Threshold* in |physx_docs| for more details

        Args:
            threshold (float): stabilization threshold

        Example:

        .. code-block:: python

            >>> prim.set_stabilization_threshold(0.005)
        """
        threshold = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._articulation_view.set_stabilization_thresholds(threshold)
        return

    def get_stabilization_threshold(self) -> float:
        """Get the mass-normalized kinetic energy below which the articulation may participate in stabilization

        Search for *Stabilization Threshold* in |physx_docs| for more details

        Returns:
            float: stabilization threshold

        Example:

        .. code-block:: python

            >>> prim.get_stabilization_threshold()
            0.0009999999
        """
        return self._articulation_view.get_stabilization_thresholds()[0]

    def set_enabled_self_collisions(self, flag: bool) -> None:
        """Set the enable self collisions flag (``physxArticulation:enabledSelfCollisions``)

        Args:
            flag (bool): whether to enable self collisions

        Example:

        .. code-block:: python

            >>> prim.set_enabled_self_collisions(True)
        """
        flag = self._backend_utils.create_tensor_from_list([flag], dtype="bool")
        self._articulation_view.set_enabled_self_collisions(flag)
        return

    def get_enabled_self_collisions(self) -> int:
        """Get the enable self collisions flag (``physxArticulation:enabledSelfCollisions``)

        Returns:
            int: self collisions flag (boolean interpreted as int)

        Example:

        .. code-block:: python

            >>> prim.get_enabled_self_collisions()
            0
        """
        return self._articulation_view.get_enabled_self_collisions()[0]

    def set_sleep_threshold(self, threshold: float) -> None:
        """Set the threshold for articulations to enter a sleep state

        Search for *Articulations and Sleeping* in |physx_docs| for more details

        Args:
            threshold (float): sleep threshold

        Example:

        .. code-block:: python

            >>> prim.set_sleep_threshold(0.01)
        """
        threshold = self._backend_utils.create_tensor_from_list([threshold], dtype="float32")
        self._articulation_view.set_sleep_thresholds(threshold)
        return

    def get_sleep_threshold(self) -> float:
        """Get the threshold for articulations to enter a sleep state

        Search for *Articulations and Sleeping* in |physx_docs| for more details

        Returns:
            float: sleep threshold

        Example:

        .. code-block:: python

            >>> prim.get_sleep_threshold()
            0.005
        """
        return self._articulation_view.get_sleep_thresholds()[0]
