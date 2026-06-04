# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""A collection of behavior tools for the decision framework (df).

This library is built on the underlying decision framework tooling of df.py. It provides specific
behaviors useful in concrete cases, sometimes tailored to specific robots. Pragmatically, we can
think of this library as being cortex dependent while df.py is cortex independent.
"""

from __future__ import annotations

import copy
import time
from abc import ABC, abstractmethod
from typing import Optional

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.core.utils.math import normalized
from isaacsim.cortex.framework.df import (
    DfAction,
    DfDecider,
    DfDecision,
    DfLogicalState,
    DfState,
    DfStateMachineDecider,
)
from isaacsim.cortex.framework.motion_commander import ApproachParams, MotionCommand, PosePq
from isaacsim.cortex.framework.robot import CortexRobot


class DfRobotApiContext(DfLogicalState):
    """A simple context object that captures the API for exposing the robot's API.

    Args:
        robot: The robot providing API access to the decider network.
    """

    def __init__(self, robot: CortexRobot) -> None:
        super().__init__()
        self.robot = robot


class DfBasicContext(DfRobotApiContext):
    """A simple instantiation of the robot API context.

    Use this context only in cases where no logical state is needed. If the aim is to derive from
    the context to add logical state monitoring, use DfRobotApiContext directly which enforces the
    implementation of the reset() method.
    """

    def reset(self) -> None:
        """Reset the context. No-op for the basic context."""


class DfDiagnosticsMonitor(ABC):
    """A utility class to simplify the monitoring of a context object.

    Monitors are called every cycle, typically at 60hz, which is too fast to easily read diagnostic
    information in real time on the screen. This class handles throttling the prints to a specific
    time delta between prints.

    Deriving classes should implement print_diagnostics(context) to print out the desired diagnostic
    information. Then add the monitor(context) method as a logical state monitor function.

    Note that it's often best to add the the diagnostics monitor as the last monitor so the logical
    state is fully updated by the other monitors before it's called.

    Usage:
        class MyDiagnosticsMonitor(DfDiagnosticsMonitor):
            def monitor(self, context):
                print("index: {}".format(context.index))

        class Context(DfRobotApiContext):
            def __init__(self, robot):
                super().__init__(robot)

                self.reset()
                self.diagnostics_monitor = MyDiagnosticsMonitor()
                self.add_monitors([Context.monitor_index, self.diagnostics_monitor.monitor])

            def reset(self):
                self.index = 0

            def monitor_index(self):
                self.index += 1

    Args:
        print_dt: The amount of times in seconds between prints.
    """

    def __init__(self, print_dt: Optional[float] = 1.0) -> None:
        self.print_dt = print_dt

        self.current_time = None
        self.time_at_start = None
        self.time_at_next_print = None

    @abstractmethod
    def print_diagnostics(self, context: DfLogicalState) -> None:
        """Override this method to print information about the context.

        Args:
            context: The context containing the logical state information to be printed.
        """
        raise NotImplementedError()

    @property
    def time_since_start(self) -> float:
        """The amount of time since the first call to this class's monitor.

        Returns:
            The time interval.
        """
        return self.current_time - self.time_at_start

    def monitor(self, context: DfLogicalState) -> None:
        """The monitor method which should be added to the list of monitors.

        This method ensures the diagnostics aren't printed more than the specified print_dt number
        of seconds apart.

        Args:
            context: The context containing the logical state information to be printed.
        """
        self.current_time = time.time()

        if self.time_at_start is None:
            self.time_at_start = self.current_time
            self.time_at_next_print = self.current_time

        if self.current_time - self.time_at_next_print >= 0.0:
            self.print_diagnostics(context)
            self.time_at_next_print += self.print_dt


class DfGoTarget(DfAction):
    """A DfAction taking as input (params) a MotionCommand and passing it to the MotionCommander.

    API of the robot.

    The robot motion commander is accessed through the robot's arm field and the command is expected
    to be passed as a parameter:

        self.context.robot.arm.send(self.params)

    On construction, a flag can be set to specifically set the target only once on entry. By
    default, that flag is False, so the command is resent every cycle. Note that sending every cycle
    adds reactivity since the command parameter can change every cycle.

    Args:
        set_target_only_on_entry: If True, send the command only on entry. Otherwise, send it every
            cycle (default).
    """

    def __init__(self, set_target_only_on_entry: Optional[bool] = False) -> None:
        super().__init__()
        self.set_target_only_on_entry = set_target_only_on_entry

    def __str__(self) -> str:
        """Return a string representation including the set_target_only_on_entry flag.

        Returns:
            String representation of the object.
        """
        return f"{super().__str__()}({self.set_target_only_on_entry})"

    def enter(self) -> None:
        """If set_target_only_on_entry is True, sends on the command once on entry. Otherwise, does.

        nothing.
        """
        if self.set_target_only_on_entry:
            self.context.robot.arm.send(self.params)

    def step(self) -> None:
        """If set_target_only_on_entry is False, sends the command every cycle. Otherwise, does.

        nothing.
        """
        if not self.set_target_only_on_entry:
            self.context.robot.arm.send(self.params)


class DfApproachTarget(DfDecider):
    """Takes a target transform as input (passed parameter) and approaches it as specified on.

    construction.

    The approach parameters are defined in one of two ways:
    1. Specifying an approach axis along with approach direction length and standard deviation
       explicitly (default).
    2. Specifying approach parameters in coordinates relative to the target. If the relative
       approach parameters are set, they take precedence over the settings in number 1.

    If approach_params is set, those parameters override any explicitly set parameters.

    Args:
        approach_along_axis:
            Which axis to approach along. The specified axis should be an index with the mapping
            0:ax, 1:ay, 2:az. Defaults to approaching along the z-axis.
        direction_length:
            The length of the direction parameter (the normalized approach vector itself is given by
            the chosen axis).
        std_dev:
            The standard deviation parameter passed to the approach params.
        approach_params_rel:
            Approach parameters defined in coordinates of the target. This structure overrides the
            above three parameters if specified.
    """

    def __init__(
        self,
        approach_along_axis: Optional[int] = 2,
        direction_length: Optional[float] = 0.1,
        std_dev: Optional[float] = 0.05,
        approach_params_rel: Optional[ApproachParams] = None,
    ) -> None:
        super().__init__()

        self.approach_along_axis = approach_along_axis
        self.direction_length = direction_length
        self.std_dev = std_dev
        self.approach_params_rel = approach_params_rel

        self.add_child("go_target", DfGoTarget())

    def __str__(self) -> str:
        """Return a string representation including approach axis and direction length.

        Returns:
            String representation of the object.
        """
        return f"{super().__str__()}({self.approach_along_axis},{self.direction_length})"

    def decide(self) -> DfDecision:
        """Chooses the motion command parameters to send down to the DfGoTarget action.

        This includes preventing the end-effector from twisting around awkwardly in longer range
        cross body motions.

        Returns:
            The DfGoTarget decision with appropriate calculated parameters.
        """
        target_T = self.params
        if target_T is None:
            return None

        eff_T = self.context.robot.arm.get_fk_T()

        target_R, target_p = math_util.unpack_T(target_T)
        eff_R, eff_p = math_util.unpack_T(eff_T)

        # If the end-effector would twist around awkwardly, make an intermediate target which will get
        # it to go around the right direction.
        #
        # TODO: generalize this to support choice of different dominate axes.
        eff_ax, eff_ay, eff_az = math_util.unpack_R(eff_R)
        target_ax, target_ay, target_az = math_util.unpack_R(target_R)
        if eff_ax.dot(target_ax) < -0.5:
            avg_p = 0.5 * (eff_p + target_p)
            avg_az = 0.5 * (eff_az + target_az)

            ref_ax = normalized(-avg_p)
            target_az = avg_az
            target_ax = math_util.proj_orth(ref_ax, target_az, normalize_res=True)
            target_ay = np.cross(target_az, target_ax)
            target_R = math_util.pack_R(target_ax, target_ay, target_az)

        approach_params = None
        approach_axis = target_R[:, self.approach_along_axis]
        _, _, az = math_util.unpack_R(target_R)
        if self.approach_params_rel is not None:
            direction = target_R.dot(self.approach_params_rel.direction)
            approach_params = ApproachParams(direction=direction, std_dev=self.std_dev)
        else:
            approach_params = ApproachParams(direction=self.direction_length * approach_axis, std_dev=self.std_dev)

        params = MotionCommand(PosePq(target_p, math_util.matrix_to_quat(target_R)), approach_params=approach_params)
        return DfDecision("go_target", params)


# Legacy naming
DfApproachGrasp = DfApproachTarget


class DfApproachTargetLinearly(DfDecider):
    """A decider node for calculating interpolated targets to make the end-effector move straight.

    toward a desired target.

    Generally, a motion policy cares about reaching a given target only at the end, and lets other
    sub-policies take precedent en route. For instance, collision avoidance, arm posturing, and
    joint limit avoidance might be more important than moving straight toward a target in most
    cases. However, this decider creates interpolated targets between the end-effector pose on entry
    and the desired target so the end-effector sticks to a straight line with precision.

    Args:
        step_length: How far to increment the target in units of meters each cycle. This divided by
            the cycle time gives the equivalent speed.
    """

    def __init__(self, step_length: float) -> None:
        super().__init__()

        self.step_length = step_length
        self.add_child("go_target", DfGoTarget())

    def __str__(self) -> str:
        """Return a string representation including the step length.

        Returns:
            String representation of the object.
        """
        return f"{super().__str__()}({self.step_length})"

    def enter(self) -> None:
        """Records the current end-effector configuration and calculates how much to increment the.

        (0, 1) interpolation betwene that end-effector configuration and the target based on the
        linear distance between the end-effector and target origins so steps have the desired step_length.
        """
        self.target_T = self.params
        self.init_eff_T = self.context.robot.arm.get_fk_T()
        self.position_offset = self.target_T[:3, 3] - self.init_eff_T[:3, 3]
        self.T_offset = self.target_T - self.init_eff_T

        dist = np.linalg.norm(self.position_offset)
        self.step_increment = self.step_length / dist
        self.current_alpha = 0.0

    def decide(self) -> DfDecision:
        """Calculates the current interpolated target and sends it down to the child as.

        MotionCommand parameters.

        Returns:
            The DfGoTarget decision with appropriate calculated parameters.
        """
        target_T = self.params
        if target_T is None:
            return None

        current_target_T = copy.deepcopy(self.init_eff_T)
        current_target_T += self.current_alpha * self.T_offset
        current_target_T = math_util.proj_T(current_target_T)
        self.current_alpha += self.step_increment
        if self.current_alpha > 1.0:
            self.current_alpha = 1.0

        target_R, target_p = math_util.unpack_T(current_target_T)
        params = MotionCommand(PosePq(target_p, math_util.matrix_to_quat(target_R)))

        return DfDecision("go_target", params)


class DfLift(DfDecider):
    """Lifts the end-effector to a desired height.

    Uses DfGoTarget() internally, calculating the target based on the forward kinematics in enter().

    Assumes the context has a MotionCommander in context.robot.arm.

    Args:
        height: The height to lift.
        axis: The index of the robot's base coordinate axis to lift in (x:0, y:1, z:2). Defaults to
            the z-axis (lifting up).
    """

    def __init__(self, height: float, axis: Optional[int] = 2) -> None:
        super().__init__()
        self.height = height
        self.axis = axis
        self.add_child("go_target", DfGoTarget())

    def __str__(self) -> str:
        """Return a string representation including the lift height and axis.

        Returns:
            String representation of the object.
        """
        return f"{super().__str__()}({self.height}, {self.axis})"

    def enter(self) -> None:
        """Sets a specific target a specific distance from the current end-effector.

        The target position is the end-effector position shifted along the specified axis a distance
        of height meters. The rotation is unchanged.
        """
        self.target_pq = self.context.robot.arm.get_fk_pq()
        self.target_pq.p[self.axis] += self.height

    def decide(self) -> DfDecision:
        """Passes the target to the DfGoTarget child as a MotionCommand parameter.

        Returns:
            The DfGoTarget decision with the lift target as parameter.
        """
        return DfDecision("go_target", MotionCommand(self.target_pq))


class DfMoveEndEffectorRel(DfDecider):
    """Moves the end-effector to a point relative to the end-effector's pose as measured on entry.

    Calculates the target as a world pose from the local information once during enter().

    Assumes the context has a MotionCommander in context.robot.arm.

    Args:
        p_local: The target point in coordinates relative to the end-effector on entry.
    """

    def __init__(self, p_local: np.ndarray) -> None:
        super().__init__()
        self.p_local = p_local
        self.add_child("go_target", DfGoTarget())

    def __str__(self) -> str:
        """Return a string representation including the local offset.

        Returns:
            String representation of the object.
        """
        return f"{super().__str__()}({self.p_local})"

    def enter(self) -> None:
        """Calculate the target based on the current end-effector pose and the relative p_local.

        offset passed in on construction.

        The target orientation remains constant.
        """
        eff_T = self.context.robot.arm.get_fk_T()
        R, p = math_util.unpack_T(eff_T)
        target_p = p + R.dot(self.p_local)
        target_q = math_util.matrix_to_quat(R)

        self.target_pq = PosePq(target_p, target_q)

    def decide(self) -> DfDecision:
        """Sends the calculated target down to the child DfGoTarget node as a MotionCommand.

        parameter.

        Returns:
            The DfGoTarget decision with the calculated target as parameter.
        """
        return DfDecision("go_target", MotionCommand(self.target_pq))


class DfOpenGripper(DfAction):
    """A simple gripper action that opens the gripper."""

    def enter(self) -> None:
        """Open the gripper on entry."""
        self.context.robot.gripper.open()


class DfCloseGripper(DfAction):
    """A simple gripper command to close the gripper to a specified width.

    Supports sending this decider node the width parameter from a parent. If it comes from a parent
    node, that overrides any default width value set on entry.
    """

    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        """Close the gripper on entry."""
        self.context.robot.gripper.close()


class DfMoveGripper(DfAction):
    """A gripper action to move the gripper to a specified width.

    Supports sending this action node the width parameter from a parent decider node. If it comes
    from a parent node, that overrides any default width value set on entry.

    Args:
        width: The width to move the gripper to. This value is optional since a parent node can send
            the parameter through its decision params.
    """

    def __init__(self, width: Optional[float] = None) -> None:
        super().__init__()
        self.width = width

    def enter(self) -> None:
        """Move the gripper to the specified width.

        A parent decider width parameter sent to this node takes precedence over the default
        specified on construction.
        """
        width = self.width
        if self.params is not None:
            width = self.params
        if width is None:
            raise RuntimeError(
                "A width must be specified either on construction or as a passed " + "decision parameter"
            )
        self.context.robot.gripper.move_to(width=width)


class GoHomeState(DfState):
    """State machine state that sends the robot to the home position.

    Note that this state can be wrapped in a DfStateMachineDecider to turn it into a decider node.
    See make_go_home().
    """

    def enter(self) -> None:
        """On entry, calculate and send the home motion command."""
        aji = self.context.robot.arm.aji  # Active joint indices
        home_config = self.context.robot.get_joints_default_state().positions[aji]
        self.target_T = self.context.robot.arm.get_fk_T(config=home_config)

        p, q = math_util.T2pq(self.target_T)
        command = MotionCommand(PosePq(p, q), posture_config=home_config)
        self.context.robot.arm.send(command)

    def step(self) -> DfState:
        """Each step monitor the progress toward the home target. Self transition until the target.

        is reached, then terminate.

        Returns:
            Self while moving toward home, None when the target is reached.
        """
        eff_T = self.context.robot.arm.get_fk_T()
        if np.linalg.norm(eff_T - self.target_T) < 0.01:
            return None

        return self


def make_go_home() -> DfStateMachineDecider:
    """Make a decider node wrapping the GoHomeState.

    Returns:
        A DfStateMachineDecider wrapping the GoHomeState.
    """
    return DfStateMachineDecider(GoHomeState())
