# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Reactive inverse kinematics controller using the PINK library for differential IK solving."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import carb
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import pink
import pink.tasks
import pinocchio as pin
import qpsolvers
import scipy.sparse as sp
import warp as wp
from pink.exceptions import NoSolutionFound

from .configuration_loader import PinkRobot
from .utils import (
    isaac_sim_position_quaternion_to_se3,
    map_joint_positions_to_pinocchio,
    map_pinocchio_velocity_to_joint_state,
)


class PinkIKController(mg.BaseController):
    """Reactive inverse kinematics controller using PINK's differential IK solver.

    Implements the ``BaseController`` interface by wrapping PINK's ``solve_ik`` into a
    closed-loop reactive controller. On each ``forward()`` call the controller:

    1. Updates the Pinocchio configuration from the estimated robot state.
    2. Updates task targets from the setpoint (end-effector pose, posture, etc.).
    3. Solves the QP to obtain a joint velocity.
    4. Integrates the velocity and returns the result as a ``RobotState``.

    The controller manages a ``FrameTask`` for end-effector tracking and an optional
    ``PostureTask`` for joint regularization. Users may supply additional PINK tasks,
    limits, and barriers through the constructor.

    Args:
        pink_robot: Robot loaded via :func:`load_pink_robot` or :func:`load_pink_supported_robot`.
        robot_joint_space: Full ordered joint-space of the controlled robot in Isaac Sim.
        robot_site_space: Full ordered site-space (frame names) of the controlled robot.
        tool_frame: Pinocchio frame name for the end-effector. If ``None``, the last frame
            in the model is used.
        position_cost: Cost weight(s) for the end-effector position task, in [cost]/[m].
            Scalar or 3D vector for anisotropic weighting.
        orientation_cost: Cost weight(s) for the end-effector orientation task, in [cost]/[rad].
            Scalar or 3D vector for anisotropic weighting.
        posture_cost: Cost weight for the posture regularization task, in [cost]/[rad].
            Set to 0.0 or ``None`` to disable.
        damping: Tikhonov regularization added to the QP Hessian for numerical stability.
        gain: Proportional gain for all managed tasks (0.0 to 1.0). A value of 1.0
            corresponds to dead-beat control (full error correction per step).
        lm_damping: Levenberg-Marquardt damping for the frame task.
        solver: QP solver backend name (e.g. ``"osqp"``, ``"clarabel"``).
        extra_tasks: Additional PINK Task instances to include in the QP objective.
        extra_limits: Additional PINK Limit instances beyond the default configuration
            and velocity limits.
        extra_barriers: PINK Barrier instances for safety constraints (e.g.
            ``SelfCollisionBarrier``, ``PositionBarrier``, ``BodySphericalBarrier``).
        pre_step_callback: Optional callable invoked at the start of each ``forward()``
            call, **after** the configuration has been updated from the estimated state
            but **before** ``solve_ik`` is called. Signature::

                callback(configuration: pink.Configuration, setpoint_state: RobotState | None)

            Use this to update targets on extra tasks that need per-step updates
            (e.g. ``RelativeFrameTask``, ``ComTask``, ``JointVelocityTask``).
        dt: Integration timestep in seconds used for ``solve_ik``.

    Example:

        .. code-block:: python

            from pink.tasks import RelativeFrameTask

            relative_task = RelativeFrameTask("frame_a", "frame_b",
                                              position_cost=1.0, orientation_cost=0.5)

            def update_relative_target(configuration, setpoint_state):
                relative_task.set_target_from_configuration(configuration)

            controller = PinkIKController(
                pink_robot=robot,
                robot_joint_space=articulation.dof_names,
                robot_site_space=["panda_hand"],
                tool_frame="panda_hand",
                extra_tasks=[relative_task],
                pre_step_callback=update_relative_target,
                dt=1.0 / 60.0,
            )
    """

    def __init__(
        self,
        pink_robot: PinkRobot,
        robot_joint_space: list[str],
        robot_site_space: list[str],
        *,
        tool_frame: str | None = None,
        position_cost: float | list[float] = 1.0,
        orientation_cost: float | list[float] = 1.0,
        posture_cost: float | None = 1e-3,
        damping: float = 1e-12,
        gain: float = 1.0,
        lm_damping: float = 0.0,
        solver: str = "osqp",
        extra_tasks: list | None = None,
        extra_limits: list | None = None,
        extra_barriers: list | None = None,
        pre_step_callback: Callable | None = None,
        dt: float,
    ) -> None:
        if not set(pink_robot.controlled_joint_names).issubset(set(robot_joint_space)):
            raise ValueError(
                f"PINK controlled joints {pink_robot.controlled_joint_names} "
                f"are not a subset of robot_joint_space {robot_joint_space}."
            )

        self._pink_robot = pink_robot
        self._robot_joint_space = robot_joint_space
        self._solver = solver
        self._damping = damping
        self._dt = dt

        # Resolve tool frame
        self._tool_frame = tool_frame
        if self._tool_frame is None:
            # Use the last operational frame in the model (heuristic for EE)
            frame_names = [pink_robot.model.frames[i].name for i in range(pink_robot.model.nframes)]
            if not frame_names:
                raise RuntimeError("Pinocchio model has no frames. Cannot determine tool frame.")
            self._tool_frame = frame_names[-1]

        if self._tool_frame not in robot_site_space:
            raise ValueError(f"Tool frame '{self._tool_frame}' is not in robot_site_space {robot_site_space}.")

        # Verify the frame exists in the Pinocchio model
        if not pink_robot.model.existFrame(self._tool_frame):
            raise ValueError(
                f"Frame '{self._tool_frame}' does not exist in the Pinocchio model. "
                f"Available frames: {[pink_robot.model.frames[i].name for i in range(pink_robot.model.nframes)]}"
            )

        # Create the end-effector frame task
        self._frame_task = pink.tasks.FrameTask(
            self._tool_frame,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            gain=gain,
            lm_damping=lm_damping,
        )

        # Optionally create posture task
        self._posture_task = None
        if posture_cost is not None and posture_cost > 0.0:
            self._posture_task = pink.tasks.PostureTask(cost=posture_cost, gain=gain)

        self._extra_tasks = list(extra_tasks) if extra_tasks else []
        self._extra_limits = list(extra_limits) if extra_limits else []
        self._extra_barriers = list(extra_barriers) if extra_barriers else []
        self._pre_step_callback = pre_step_callback

        # Internal state, initialized on reset()
        self._configuration: pink.Configuration | None = None
        self._q: np.ndarray | None = None

    def get_frame_task(self) -> pink.tasks.FrameTask:
        """Get the end-effector FrameTask for external configuration.

        Returns:
            The PINK FrameTask controlling end-effector tracking.
        """
        return self._frame_task

    def get_posture_task(self) -> pink.tasks.PostureTask | None:
        """Get the PostureTask if configured, for external target updates.

        Returns:
            The PINK PostureTask, or None if posture regularization is disabled.
        """
        return self._posture_task

    def forward(
        self,
        estimated_state: mg.RobotState,
        setpoint_state: mg.RobotState | None,
        t: float,
        **kwargs: Any,
    ) -> mg.RobotState | None:
        """Compute desired joint positions by solving the differential IK QP.

        Updates the Pinocchio configuration from ``estimated_state``, sets task targets
        from ``setpoint_state``, solves the QP, and integrates the resulting velocity.

        Args:
            estimated_state: Current estimated robot state (joint positions required).
            setpoint_state: Desired setpoint containing target site poses and/or joint
                posture targets. The tool frame must match the frame configured at init.
            t: Current simulation clock time (unused by the stateless QP, but required
                by the BaseController interface).
            **kwargs: Additional arguments (unused).

        Returns:
            RobotState containing desired joint positions and velocities for the
            controlled joints, or None if the controller is not yet initialized.
        """
        if self._configuration is None:
            return None

        # Update configuration from estimated state
        q = self._update_configuration_from_state(estimated_state)
        if q is None:
            return None

        # Update task targets from setpoint
        if setpoint_state is not None:
            self._update_targets_from_setpoint(setpoint_state)

        # Let user update extra task targets with the current configuration
        if self._pre_step_callback is not None:
            self._pre_step_callback(self._configuration, setpoint_state)

        # Assemble tasks
        tasks = [self._frame_task]
        if self._posture_task is not None:
            tasks.append(self._posture_task)
        tasks.extend(self._extra_tasks)

        limits = self._get_limits()

        # Solve IK
        try:
            velocity = _solve_ik(
                self._configuration,
                tasks,
                dt=self._dt,
                solver=self._solver,
                damping=self._damping,
                limits=limits,
                barriers=self._extra_barriers if self._extra_barriers else None,
            )
        except Exception as e:
            carb.log_warn(f"PINK solve_ik failed: {e}")
            return None

        # Integrate and update internal state.
        # Save pre-integration q for the output mapping to avoid double integration.
        q_current = self._q.copy()
        self._configuration.integrate_inplace(velocity, self._dt)
        self._q = self._configuration.q.copy()

        return mg.RobotState(
            joints=map_pinocchio_velocity_to_joint_state(
                velocity=velocity,
                model=self._pink_robot.model,
                controlled_joint_names=self._pink_robot.controlled_joint_names,
                robot_joint_space=self._robot_joint_space,
                dt=self._dt,
                q_current=q_current,
            )
        )

    def reset(
        self,
        estimated_state: mg.RobotState,
        setpoint_state: mg.RobotState | None,
        t: float,
        **kwargs: Any,
    ) -> bool:
        """Initialize the controller from the current robot state.

        Creates the PINK Configuration from ``estimated_state`` joint positions, sets
        the posture task target to the current configuration, and initializes the
        frame task target from the current end-effector pose.

        Args:
            estimated_state: Current estimated robot state (joint positions required).
            setpoint_state: Initial setpoint (currently unused during reset).
            t: Current simulation clock time.
            **kwargs: Additional arguments (unused).

        Returns:
            True if reset succeeded, False if joint positions could not be extracted.
        """
        joint_positions = self._extract_joint_positions(estimated_state)
        if joint_positions is None:
            self._configuration = None
            self._q = None
            return False

        q = map_joint_positions_to_pinocchio(
            joint_names=list(joint_positions.keys()),
            joint_positions=np.array(list(joint_positions.values())),
            model=self._pink_robot.model,
            q_current=self._pink_robot.q0.copy(),
        )

        self._q = q
        self._configuration = pink.Configuration(
            self._pink_robot.model,
            self._pink_robot.data,
            q,
            collision_model=self._pink_robot.collision_model,
            collision_data=self._pink_robot.collision_data,
        )

        # Initialize frame task target to current EE pose
        self._frame_task.set_target_from_configuration(self._configuration)

        # Initialize posture task target to current configuration
        if self._posture_task is not None:
            self._posture_task.set_target_from_configuration(self._configuration)

        return True

    def _extract_joint_positions(self, state: mg.RobotState) -> dict[str, float] | None:
        """Extract controlled joint positions from a RobotState as a name->value dict."""
        if state.joints is None:
            return None

        controlled = set(self._pink_robot.controlled_joint_names)
        available = set(state.joints.position_names)
        if not controlled.issubset(available):
            return None

        positions = state.joints.positions.numpy()
        result = {}
        for name in self._pink_robot.controlled_joint_names:
            idx = state.joints.position_names.index(name)
            result[name] = float(positions[idx])
        return result

    def _update_configuration_from_state(self, state: mg.RobotState) -> np.ndarray | None:
        """Update the PINK configuration from estimated joint positions."""
        joint_positions = self._extract_joint_positions(state)
        if joint_positions is None:
            return None

        q = map_joint_positions_to_pinocchio(
            joint_names=list(joint_positions.keys()),
            joint_positions=np.array(list(joint_positions.values())),
            model=self._pink_robot.model,
            q_current=self._q,
        )
        self._configuration.update(q)
        self._q = q
        return q

    def _update_targets_from_setpoint(self, setpoint: mg.RobotState) -> None:
        """Update task targets from the setpoint state."""
        # Update frame task from site poses
        if setpoint.sites is not None:
            tool_position = self._extract_tool_position(setpoint)
            tool_orientation = self._extract_tool_orientation(setpoint)

            if tool_position is not None and tool_orientation is not None:
                target_se3 = isaac_sim_position_quaternion_to_se3(tool_position, tool_orientation)
                self._frame_task.set_target(target_se3)
            elif tool_position is not None:
                # Position-only update: keep current orientation from the last target.
                # FrameTask stores the target as _target_T (private); fall back to
                # the current EE pose if the attribute is unavailable.
                current_target = getattr(self._frame_task, "_target_T", None)
                if current_target is None:
                    current_target = self._configuration.get_transform_frame_to_world(self._tool_frame)
                target_se3 = pin.SE3(current_target.rotation, _to_numpy_flat(tool_position))
                self._frame_task.set_target(target_se3)

        # Update posture task from joint targets
        if self._posture_task is not None and setpoint.joints is not None:
            joint_positions = self._extract_joint_positions(setpoint)
            if joint_positions is not None:
                q_target = map_joint_positions_to_pinocchio(
                    joint_names=list(joint_positions.keys()),
                    joint_positions=np.array(list(joint_positions.values())),
                    model=self._pink_robot.model,
                    q_current=self._pink_robot.q0.copy(),
                )
                self._posture_task.set_target(q_target)

    def _get_limits(self) -> list[Any] | None:
        """Get the flat PINK limit list for `solve_ik`.

        PINK owns the limit objects and expects `limits` to be `None` or an
        iterable of Limit objects. Passing a
        non-None iterable replaces PINK's defaults, so custom limits must be
        appended after the model's default limit objects.
        """
        if not self._extra_limits:
            return None

        model = self._configuration.model
        configuration_limit = model.configuration_limit
        velocity_limit = model.velocity_limit
        floating_base_velocity_limit = getattr(model, "floating_base_velocity_limit", None)

        limits_for_pink = [configuration_limit, velocity_limit]
        if floating_base_velocity_limit is not None:
            limits_for_pink.append(floating_base_velocity_limit)
        limits_for_pink.extend(self._extra_limits)
        return limits_for_pink

    def _extract_tool_position(self, state: mg.RobotState) -> np.ndarray | None:
        """Extract tool frame position from a RobotState's sites."""
        if state.sites is None or self._tool_frame not in state.sites.position_names:
            return None
        idx = state.sites.position_names.index(self._tool_frame)
        return state.sites.positions.numpy()[idx]

    def _extract_tool_orientation(self, state: mg.RobotState) -> np.ndarray | None:
        """Extract tool frame orientation from a RobotState's sites."""
        if state.sites is None or self._tool_frame not in state.sites.orientation_names:
            return None
        idx = state.sites.orientation_names.index(self._tool_frame)
        return state.sites.orientations.numpy()[idx]


def _to_numpy_flat(arr: np.ndarray | wp.array | list[float]) -> np.ndarray:
    """Convert to a flat numpy array."""
    if isinstance(arr, wp.array):
        return arr.numpy().flatten()
    return np.asarray(arr, dtype=np.float64).flatten()


def _solve_ik(
    configuration: pink.Configuration,
    tasks: list[Any],
    *,
    dt: float,
    solver: str,
    damping: float,
    limits: list[Any] | None,
    barriers: list[Any] | None,
) -> np.ndarray:
    """Solve PINK IK, pre-sparsifying OSQP matrices to avoid stderr warnings."""
    if solver != "osqp":
        return pink.solve_ik(
            configuration,
            tasks,
            dt=dt,
            solver=solver,
            damping=damping,
            limits=limits,
            barriers=barriers,
        )

    configuration.check_limits()
    problem = pink.build_ik(configuration, tasks, dt=dt, damping=damping, limits=limits, barriers=barriers)
    problem.P = sp.csc_matrix(problem.P)
    if problem.G is not None:
        problem.G = sp.csc_matrix(problem.G)
    if problem.A is not None:
        problem.A = sp.csc_matrix(problem.A)

    result = qpsolvers.solve_problem(problem, solver=solver)
    delta_q = result.x
    if not result.found or delta_q is None:
        raise NoSolutionFound(problem, result)
    return delta_q / dt
