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

"""Implementation of RMPflow-based reactive motion controller using cuMotion for collision-free robot motion generation."""

from __future__ import annotations

import pathlib
from typing import Any

import cumotion
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp

from .configuration_loader import CumotionRobot
from .cumotion_world_interface import CumotionWorldInterface
from .utils import isaac_sim_to_cumotion_rotation, isaac_sim_to_cumotion_translation


class RmpFlowController(mg.BaseController):
    """Reactive motion policy controller using cuMotion's RMPflow algorithm.

    RMPflow (Riemannian Motion Policies) is a reactive control algorithm that generates
    smooth, collision-free motions by combining multiple motion policies in task and
    configuration space. This controller continuously updates joint commands based on
    the desired targets.

    .. note::

        For performance, ``reset()`` pre-allocates a single output ``RobotState``
        and ``forward()`` returns that same instance on every call, mutating its
        joint buffer in place. Callers that need to retain a value across
        ``forward()`` calls must copy it out (see ``forward()`` for details).

    Args:
        cumotion_robot: Robot containing kinematics and joint information.
        cumotion_world_interface: World interface providing collision geometry.
        robot_joint_space: The full ordered joint-space of the controlled robot.
        robot_site_space: The full ordered site-space of the controlled robot (used for validation of tool-frame name only).
        rmp_flow_configuration_filename: Path to the RMPflow YAML configuration file.
            If a relative path is provided, it is resolved relative to
            cumotion_robot.directory. If an absolute path is provided,
            it is used as-is. Defaults to "rmp_flow.yaml".
        tool_frame: Name of the tool frame for end-effector control. Defaults to None,
            which uses the first tool frame defined in the robot description.
        maximum_substep_size: Maximum timestep size (in seconds) used during Euler integration.
            When the frame duration exceeds this value, the integration is split into multiple
            substeps to improve numerical stability.

    Raises:
        ValueError: If `cumotion_robot.controlled_joint_names` is not a subset of `robot_joint_space`.
        ValueError: If `tool_frame` is not found in `robot_site_space`.
        ValueError: If `maximum_substep_size` is not strictly positive.
        RuntimeError: If no tool frames are available in the robot description and `tool_frame` is None.

    Example:

        .. code-block:: python

            controller = RmpFlowController(
                cumotion_robot=robot,
                cumotion_world_interface=world_interface,
                rmp_flow_configuration_filename="rmp_flow.yaml"
            )
            controller.reset(estimated_state, setpoint_state, t=0.0)
            desired_state = controller.forward(estimated_state, setpoint_state, t=0.1)
    """

    def __init__(
        self,
        cumotion_robot: CumotionRobot,
        cumotion_world_interface: CumotionWorldInterface,
        robot_joint_space: list[str],
        robot_site_space: list[str],
        rmp_flow_configuration_filename: pathlib.Path | str = "rmp_flow.yaml",
        tool_frame: str | None = None,
        maximum_substep_size: float = 1.0 / 120.0,
    ) -> None:

        if not set(cumotion_robot.controlled_joint_names).issubset(set(robot_joint_space)):
            raise ValueError(
                f"Cumotion controlled joints {cumotion_robot.controlled_joint_names} are not a subset of the robot_joint_space {robot_joint_space}."
            )

        self._robot_joint_space = robot_joint_space

        self._tool_frame = tool_frame

        if self._tool_frame is None:
            tool_frame_names = cumotion_robot.robot_description.tool_frame_names()
            if not tool_frame_names:
                raise RuntimeError("No tool frames available in robot description and no tool_frame was provided.")
            self._tool_frame = tool_frame_names[0]

        if self._tool_frame not in robot_site_space:
            raise ValueError(
                f"The specified tool name {self._tool_frame} is not in the list of controlled sites {robot_site_space}"
            )

        self._cumotion_world_interface = cumotion_world_interface

        if maximum_substep_size <= 0.0:
            raise ValueError("maximum_substep_size must be strictly positive.")

        self._maximum_substep_size = maximum_substep_size

        # there is no "RmpFlow" algorithm until we initialize.
        self._rmp_flow = None
        self._cumotion_robot = cumotion_robot

        self._output_position = None
        self._output_velocity = None
        self._previous_run_time = None

        # Pre-allocated output structures (populated in reset()).
        self._joint_acceleration = None
        self._output_joint_data = None
        self._output_joint_valid = None
        self._output_joint_indices = None
        self._output_joint_state = None
        self._cached_output_robot_state = None

        # Memoized host-side copy of the world-to-robot-base transform; keyed
        # on the wp.array identity returned by the world interface so it stays
        # valid as long as the base is not updated.
        self._cached_world_to_base_pos_wp: wp.array | None = None
        self._cached_world_to_base_quat_wp: wp.array | None = None
        self._cached_world_to_base_pos_np: np.ndarray | None = None
        self._cached_world_to_base_quat_np: np.ndarray | None = None

        # create the rmp-flow configuration. Do not initialize, which will let the user modify
        # the configuration should they so choose.
        rmp_flow_path = pathlib.Path(rmp_flow_configuration_filename)
        if rmp_flow_path.is_absolute():
            full_rmp_flow_path = rmp_flow_path
        else:
            full_rmp_flow_path = cumotion_robot.directory / rmp_flow_path

        self._rmp_flow_config = cumotion.create_rmpflow_config_from_file(
            rmpflow_config_file=full_rmp_flow_path,
            robot_description=cumotion_robot.robot_description,
            end_effector_frame=self._tool_frame,
            world_view=cumotion_world_interface.world_view,
        )

    def get_rmp_flow_config(self) -> cumotion.RmpFlowConfig:
        """Get the RMPflow configuration object.

        Returns the underlying cuMotion configuration object, allowing users to modify
        controller parameters before initialization.

        Returns:
            The cuMotion RMPflow configuration object.

        Example:

            .. code-block:: python

                config = controller.get_rmp_flow_config()
                config.set_param("cspace_target_rmp/damping_gain", 0.9)
        """
        return self._rmp_flow_config

    def forward(
        self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs: Any
    ) -> mg.RobotState | None:
        """Compute the desired joint action for the next time-step.

        Evaluates the RMPflow controller to compute desired joint positions and velocities
        based on the current desired robot state and target attractors. The controller integrates
        the computed accelerations over time.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Desired setpoint state containing target attractors. Can include
                joint positions, end-effector positions, and/or orientations. End-effector inputs
                must match the tool frame defined at initialization for this controller.
            t: Current clock time (simulation or real).
            **kwargs: Additional arguments for the controller.

        Returns:
            Robot state containing desired joint positions and velocities, or None if
            the controller is not initialized.

            .. warning::

                For performance, ``forward()`` returns the **same** ``RobotState``
                instance on every call and mutates its underlying joint buffer
                in place. The returned object is a live view, not a snapshot.

                * ``state_a is state_b`` is ``True`` for any two values returned
                  by ``forward()`` between ``reset()`` calls.
                * ``state.joints.data_array`` aliases the controller's internal
                  buffer and reflects the most recent ``forward()`` result.
                * ``state.joints.positions`` / ``.velocities`` are lazily
                  rebuilt after each ``forward()``; reading them after a
                  subsequent ``forward()`` returns the *new* values.
                * ``reset()`` re-allocates these objects, so references taken
                  before a ``reset()`` are orphaned (they keep their last
                  values but are no longer updated).

                If you need to retain a value across ``forward()`` calls (for
                interpolation, logging, etc.), copy it out immediately, e.g.
                ``positions = desired_state.joints.positions.numpy().copy()``.

        Example:

            .. code-block:: python

                desired_state = controller.forward(current_state, target_state, t=1.0)
                if desired_state is not None:
                    robot.set_dof_position_targets(
                        desired_state.joints.positions,
                        indices=[0],
                        dof_indices=robot.get_joint_indices(desired_state.joints.names)
                    )

                # Snapshot for later use; do not hold desired_state itself,
                # since the next forward() call will overwrite it.
                positions_t1 = desired_state.joints.positions.numpy().copy()
        """
        # if we haven't initialized our rmp flow algorithm, return:
        if self._rmp_flow is None:
            return None

        # update the worldview:
        self._cumotion_world_interface.world_view.update()

        # update the attractor according to the setpoint_state, if they exist:
        if setpoint_state is not None:
            # check if the joint states are defined, and matches all of the desired joints:
            joint_attractor = self._joint_position_from_robot_state(setpoint_state)
            if joint_attractor is not None:
                self._rmp_flow.set_cspace_attractor(joint_attractor)

            tool_position = self._tool_position_from_robot_state(setpoint_state)
            if tool_position is not None:
                self._rmp_flow.set_end_effector_position_attractor(tool_position)

            tool_orientation = self._tool_orientation_from_robot_state(setpoint_state)
            if tool_orientation is not None:
                self._rmp_flow.set_end_effector_orientation_attractor(tool_orientation)

        frame_duration = t - self._previous_run_time
        self._output_position, self._output_velocity = self._euler_integration(
            self._output_position, self._output_velocity, frame_duration
        )
        self._previous_run_time = t

        # Update the pre-allocated JointState data in-place and return the
        # cached RobotState; avoids the per-frame JointState.from_name() cost.
        self._output_joint_state._update_data_in_place(
            positions_np=self._output_position,
            velocities_np=self._output_velocity,
        )
        return self._cached_output_robot_state

    def reset(
        self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs: Any
    ) -> bool:
        """Reset the controller to a safe initial state.

        Initializes the RMPflow controller and sets the internal state to match the
        current robot configuration. This should be called before running the controller
        for the first time.

        .. note::

            ``reset()`` allocates a fresh output ``RobotState`` / ``JointState``
            pair (the objects subsequently returned by ``forward()``). Any
            ``RobotState`` reference obtained from a prior ``forward()`` call
            becomes orphaned after ``reset()``: it retains its last values but
            is no longer updated by future ``forward()`` calls.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Initial setpoint state (currently unused).
            t: Current clock time (simulation or real).
            **kwargs: Additional arguments for the controller.

        Returns:
            True if reset was successful, False otherwise.

        Example:

            .. code-block:: python

                success = controller.reset(current_state, None, t=0.0)
                if success:
                    print("Controller initialized successfully")
        """
        # Get all of the joint relevant joint-states from the estimated
        current_joint_positions = self._joint_position_from_robot_state(estimated_state)
        if current_joint_positions is None:
            self._output_position = None
            self._output_velocity = None

            # We don't have the required joint-positions to reset RmpFlow.
            return False

        self._rmp_flow = cumotion.create_rmpflow(self._rmp_flow_config)

        # Check if initialization was successful:
        if self._rmp_flow is None:
            # this is not a successful reset, we are not initialized!!
            self._output_position = None
            self._output_velocity = None
            return False

        # otherwise, reset the attractors:
        self._rmp_flow.clear_end_effector_orientation_attractor()
        self._rmp_flow.clear_end_effector_position_attractor()

        # Set the attractor to our current position.
        self._rmp_flow.set_cspace_attractor(current_joint_positions)

        # set the internal integrator to be stationary at the current joint position.
        self._output_position = current_joint_positions
        self._output_velocity = np.zeros_like(current_joint_positions)

        # Scratch buffer for joint accelerations; forward() zeroes it in place
        # each frame instead of allocating.
        self._joint_acceleration = np.zeros_like(current_joint_positions)

        # Pre-allocate the output JointState/RobotState so that forward() can
        # update data in place rather than calling JointState.from_name() each frame.
        n_joints = len(self._robot_joint_space)
        self._output_joint_data = wp.zeros(shape=[3, n_joints], dtype=wp.float32, device="cpu")
        self._output_joint_valid = wp.zeros(shape=[3, n_joints], dtype=wp.bool, device="cpu")

        joint_name_to_index = {name: i for i, name in enumerate(self._robot_joint_space)}
        self._output_joint_indices = np.array(
            [joint_name_to_index[name] for name in self._cumotion_robot.controlled_joint_names],
            dtype=np.intp,
        )

        valid_np = self._output_joint_valid.numpy()
        valid_np[0, self._output_joint_indices] = True  # positions row
        valid_np[1, self._output_joint_indices] = True  # velocities row

        self._output_joint_state = mg.JointState(
            self._robot_joint_space,
            self._output_joint_data,
            self._output_joint_valid,
        )
        self._cached_output_robot_state = mg.RobotState(joints=self._output_joint_state)

        self._previous_run_time = t

        # successful reset.
        return True

    def _joint_position_from_robot_state(self, state: mg.RobotState) -> np.ndarray | None:
        if state.joints is None:
            return None

        # Check whether all of the controller joint names are available in the input state:
        if not set(self._cumotion_robot.controlled_joint_names).issubset(set(state.joints.position_names)):
            return None

        # Get the joint values in the correct order:
        output = np.zeros(shape=[len(self._cumotion_robot.controlled_joint_names)])
        input_joint_positions = state.joints.positions.numpy()

        # map the cumotion indices to those used by isaac sim:
        cumotion_index_to_isaac_sim_index = [
            state.joints.position_names.index(cumotion_name)
            for cumotion_name in self._cumotion_robot.controlled_joint_names
        ]

        for i_cumotion in range(len(cumotion_index_to_isaac_sim_index)):
            output[i_cumotion] = input_joint_positions[cumotion_index_to_isaac_sim_index[i_cumotion]]

        return output

    def _get_world_to_robot_base_transform_numpy(self) -> tuple[np.ndarray, np.ndarray]:
        """Memoized host-side view of the world-to-robot-base transform.

        Refreshes the cached numpy copies only when the world interface returns
        a different ``wp.array`` instance (i.e. the base pose has changed).

        Returns:
            Tuple of (position_np, quaternion_np) numpy views of the cached
            world-to-base transform.
        """
        pos_wp, quat_wp = self._cumotion_world_interface.get_world_to_robot_base_transform()
        if pos_wp is not self._cached_world_to_base_pos_wp:
            self._cached_world_to_base_pos_wp = pos_wp
            self._cached_world_to_base_pos_np = pos_wp.numpy()
        if quat_wp is not self._cached_world_to_base_quat_wp:
            self._cached_world_to_base_quat_wp = quat_wp
            self._cached_world_to_base_quat_np = quat_wp.numpy()
        return self._cached_world_to_base_pos_np, self._cached_world_to_base_quat_np

    def _tool_position_from_robot_state(self, state: mg.RobotState) -> np.ndarray | None:
        if state.sites is None:
            return None

        # Check if the tool-frame we are controlling is in the input state:
        if not self._tool_frame in state.sites.position_names:
            return None

        # return the position based on the correct index:
        position_index = state.sites.position_names.index(self._tool_frame)
        position_world = np.array(state.sites.positions.numpy()[position_index])

        world_to_robot_base_position_np, world_to_robot_base_quaternion_np = (
            self._get_world_to_robot_base_transform_numpy()
        )

        return isaac_sim_to_cumotion_translation(
            position_world_to_target=position_world,
            position_world_to_base=world_to_robot_base_position_np,
            orientation_world_to_base=world_to_robot_base_quaternion_np,
        )

    def _tool_orientation_from_robot_state(self, state: mg.RobotState) -> np.ndarray | None:
        if state.sites is None:
            return None

        # Check if the tool-frame we are controlling is in the input state:
        if not self._tool_frame in state.sites.orientation_names:
            return None

        # return the orientations based on the correct index:
        orientation_index = state.sites.orientation_names.index(self._tool_frame)
        rotation_world_target = state.sites.orientations.numpy()[orientation_index]

        _, world_to_robot_base_quaternion_np = self._get_world_to_robot_base_transform_numpy()

        return isaac_sim_to_cumotion_rotation(
            orientation_world_to_target=rotation_world_target,
            orientation_world_to_base=world_to_robot_base_quaternion_np,
        )

    def _euler_integration(
        self, joint_positions: np.ndarray, joint_velocities: np.ndarray, frame_duration: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Perform Euler integration to advance robot joint states.

        Integrates the robot dynamics using multiple substeps to maintain numerical stability.
        The number of substeps is determined by `_maximum_substep_size`.

        Args:
            joint_positions: Starting joint positions.
            joint_velocities: Starting joint velocities.
            frame_duration: Total time duration to integrate over in seconds.

        Returns:
            Updated joint positions and velocities after integration.
        """
        if frame_duration <= 0.0:
            return joint_positions, joint_velocities
        num_steps = int(np.ceil(frame_duration / self._maximum_substep_size))
        policy_timestep = frame_duration / num_steps

        joint_acceleration = self._joint_acceleration
        for _ in range(num_steps):
            joint_acceleration[:] = 0.0
            self._rmp_flow.eval_accel(joint_positions, joint_velocities, joint_acceleration)
            joint_velocities += policy_timestep * joint_acceleration
            joint_positions += policy_timestep * joint_velocities

        return joint_positions, joint_velocities
