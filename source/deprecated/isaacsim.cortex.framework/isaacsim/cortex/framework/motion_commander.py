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

"""This module provides a motion commander that wraps a motion policy to give it a standard.

flexible command interface.

The command, defined by the MotionCommand class, allows users to specify not only a target pose, but
additionally information describing how the robot's end-effector should approach that target and how
the system should handle null space resolution (arm posture).

Fundamentally, a robotic system's state includes both position and velocity so specifying both a
target pose and the direction from which it should approach is both more natural and more complete.
Often this information is provided implicitly by scripting a little state machine that sends the
end-effector through an intermediate standoff pose before heading to the target. But the motion
commander makes the information a core part of the command so users don't have to rely as much on
way points.
"""

from __future__ import annotations

import copy
from typing import Optional

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.core.prims import SingleGeometryPrim, SingleXFormPrim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.cortex.framework.commander import Commander
from isaacsim.cortex.framework.cortex_object import CortexObject
from isaacsim.cortex.framework.smoothed_command import SmoothedCommand, TargetAdapter
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.motion_policy_interface import MotionPolicy

# The CortexObject wraps a core API object. Obstacles can either be the core API object or a
# CortexObject wrapped variant. All of these objects derive from SingleGeometryPrim, although the
# specifics of the supportant variants are policy specific. See the specific motion policy's
# obstacle support in isaacsim.robot_motion.motion_generation for details of which objects are supported.
CortexObstacleType = CortexObject | SingleGeometryPrim


class ApproachParams(object):
    """Parameters describing how to approach a target (in position). They generally describe a.

    funnel approaching the target from a particular direction.

    The approach direction is a 3D vector pointing in the direction of approach. It's magnitude
    defines the max offset from the position target the intermediate approach target will be shifted
    by. The std dev defines the length scale a radial basis (Gaussian) weight function that defines
    what fraction of the shift we take. The radial basis function is defined on the orthogonal
    distance to the line defined by the target and the direction vector.

    Intuitively, the normalized vector direction of the direction vector defines which direction to
    approach from, and it's magnitude defines how far back we want the end effector to come in from.
    The std dev defines how tighly the end-effector approaches along that line. Small std dev is
    tight around that approach line, large std dev is looser. A good value is often between 1 and 3
    cm (values of .01-.03 in meters).

    See calc_shifted_approach_target() for the specific implementation of how these parameters are
    used.

    Args:
        direction: The direction vector describing the direction to approach from.
        std_dev: The radial basis std dev characterizing how tightly to follow the approach
            direction.
    """

    def __init__(self, direction: np.ndarray, std_dev: float) -> None:
        self.direction = direction
        self.std_dev = std_dev

    def __str__(self) -> str:
        """Return a string representation of the approach parameters.

        Returns:
            String representation of the object.
        """
        return f"{{direction: {self.approach!s}, std_dev {self.std_dev!s}}}"


class PosePq:
    """A pose represented internally as a position p and quaternion orientation q.

    Args:
        p: The pose position
        q: The pose orientation as a quaternion.
    """

    def __init__(self, p: np.ndarray, q: np.ndarray) -> None:
        self.p = p
        self.q = q

    def as_tuple(self) -> tuple[np.ndarray, np.ndarray]:
        """Returns the pose as a (p,q) tuple.

        Returns:
            A tuple (p, q) of position and quaternion.
        """
        return self.p, self.q

    def to_T(self) -> np.ndarray:  # noqa: N802
        """Returns the pose as a homogeneous transform matrix T.

        Returns:
            A 4x4 homogeneous transform matrix.
        """
        return math_util.pack_Rp(quat_to_rot_matrix(self.q), self.p)


class MotionCommand:
    """Contains information about a motion command: an end-effector target (either full pose or.

    position only), optional approach parameters, and an optional posture configuration.

    The target pose is a full position and orientation target. The approach params define how the
    end-effector should approach that target (see ApproachParams above). And the posture config
    defines how the system should resolve redundancy and generally posture the arm throughout the
    movement.

    Users should set either target_pose or target_position, but not both. target_pose defines a full
    pose target for the end-effector; target_position defines a postion-only end-effector allowing
    the arm to move through the nullspace. That nullspace can be optionally biased by the posture
    configuration.

    Args:
        target_pose: A full pose end-effector target. Set this or target_position, but not both.
        target_position: A position-only end-effector target. Set this or target_pose, but not both.
        approach_params: Optional parameters describing how the end-effector should approach the
            target.
        posture_config: A configuration of all joints commanded by the MotionCommander to bias the
            motion in the null space of the target.

    Raises:
        TypeError: If either both target_pose and target_position are set or neither of them are set.
    """

    def __init__(
        self,
        target_pose: Optional[PosePq] = None,
        target_position: Optional[np.ndarray] = None,
        approach_params: Optional[ApproachParams] = None,
        posture_config: Optional[np.ndarray] = None,
    ) -> None:
        if target_pose is not None:
            if target_position is not None:
                raise TypeError("Cannot specify both a full pose and a position only command.")
            self.target_pose = target_pose
        else:
            if target_position is None:
                raise TypeError("Must specify either a full pose or position only command.")
            self.target_pose = PosePq(target_position, None)

        self.approach_params = approach_params
        self.posture_config = posture_config

    @property
    def has_approach_params(self) -> bool:
        """Determines whether approach parameters have been specified.

        Returns: True if they've been set, False otherwise.
        """
        return self.approach_params is not None

    @property
    def has_posture_config(self) -> bool:
        """Determines whether a posture config has been specified.

        Returns: True if it's been set, False otherwise.
        """
        return self.posture_config is not None


def calc_shifted_approach_target(target_T: np.ndarray, eff_T: np.ndarray, approach_params: np.ndarray) -> np.ndarray:
    """Calculates how the target should be shifted to implement the approach given the current.

    end-effector position.

    Args:
        target_T: Final target pose as a homogeneous transform matrix.
        eff_T: Current end effector pose as a homogeneous transform matrix.
        approach_params: The approach parameters.

    Returns:
        The shifted target position.
    """
    target_R, target_p = math_util.unpack_T(target_T)
    eff_R, eff_p = math_util.unpack_T(eff_T)

    direction = approach_params.direction
    std_dev = approach_params.std_dev

    v = eff_p - target_p
    an = math_util.normalized(direction)
    norm = np.linalg.norm
    dist = norm(v - np.dot(v, an) * an)
    dist += 0.5 * norm(target_R - eff_R) / 3
    alpha = 1.0 - np.exp(-0.5 * dist * dist / (std_dev * std_dev))
    shifted_target_p = target_p - alpha * direction

    return shifted_target_p


class MotionCommandAdapter(TargetAdapter):
    """A simple adapter class to extract the target information to pass into the SmoothedCommand.

    object.

    Args:
        command: The motion command being adapted.
    """

    def __init__(self, command: MotionCommand) -> None:
        self.command = command

    def get_position(self) -> np.ndarray:
        """Extract the position vector from the target pose.

        Returns:
            The position vector.
        """
        return self.command.target_pose.p

    def has_rotation(self) -> bool:
        """Determines whether there's a specified orientation in the target pose.

        Returns:
            True if the commanded target orientation has been set, False otherwise.
        """
        return self.command.target_pose.q is not None

    def get_rotation_matrix(self) -> np.array:
        """Converts the target pose orientation to a rotation matrix.

        Note that this method doesn't verify whether the rotation is set. Use has_rotation() to
        verify it's been set before calling this method.

        Returns:
            The 3x3 rotation matrix for the target orientation.
        """
        return quat_to_rot_matrix(self.command.target_pose.q)


class MotionCommander(Commander):
    """The motion commander provides an abstraction of motion for the cortex wherein a lower-level.

    policy implements the motion commands defined by MotionCommand objects.

    This class uses a target prim for setting targets. The target prim can be set to a target
    manually via a call to set_target() or it can be controlled using a gizmo through the IsaacSim
    viewport.

    See MotionCommand for a description of the information provided in the command. At a high-level,
    it includes the end-effector target, approach parameters describing how to approach the target,
    and a posture config informing the policy about null space choice.

    Args:
        amp: The ArticulationMotionPolicy interfacing to the underlying motion policy. Includes the
            reference to the underlying Articulation being controlled.
        target_prim: The target SingleXFormPrim defining where the current end-effector target is. This
            target pose is passed into amp to move the robot.
        use_smoothed_commands: Optional boolean signifying whether to smooth the commands coming in.
            Defaults to true. This smoothing helps reduce system jerk.
    """

    def __init__(
        self, amp: ArticulationMotionPolicy, target_prim: SingleXFormPrim, use_smoothed_commands: Optional[bool] = True
    ) -> None:
        super().__init__(amp._active_joints_view)

        self.robot = amp.get_robot_articulation()
        self.amp = amp
        self.smoothed_command = None
        if use_smoothed_commands:
            self.smoothed_command = SmoothedCommand()

        self.target_prim = None
        self.obstacles = {}  # Keep track of added obstacles.

        self._reset_target_print_to_eff = False
        self._is_target_position_only = False

        self.register_target_prim(target_prim)

    def reset(self) -> None:
        """Reset this motion commander. This method ensures that any internal integrators of the.

        motion policy are reset, as is the smoothed command.
        """
        # Resetting the motion policy removes the obstacles, so we need to add them back.
        self.motion_policy.reset()
        for _, obs in self.obstacles.items():
            self.add_obstacle(obs)
        if self.smoothed_command is not None:
            self.smoothed_command.reset()

        self._reset_target_print_to_eff = True

    def soft_reset(self) -> None:
        """Soft reset this motion commander. This method only resets the internal integrators and.

        nothing else.
        """
        self.motion_policy._robot_joint_positions = None
        self.motion_policy._robot_joint_velocities = None

    @property
    def num_controlled_joints(self) -> int:
        """Returns the number of joints commanded by this commander."""
        return self.amp.get_active_joints_subset().num_joints

    @property
    def motion_policy(self) -> MotionPolicy:
        """Returns the motion policy used to command the robot."""
        return self.amp.get_motion_policy()

    @property
    def aji(self) -> np.ndarray:
        """Returns the active joint indices. These are the indices into the full C-space.

        configuration vector of the joints which are actively controlled.
        """
        return self.amp.get_active_joints_subset().get_joint_subset_indices()

    def register_target_prim(self, target_prim: SingleXFormPrim) -> None:
        """Register the specified target prim with this commander.

        This prim will both visualize the commands being sent to the motion commander, and it can be
        used to manually control the robot using the OV viewport's gizmo.

        To manually control the target prim, make sure the latest command has been cleared by
        calling clear() on this MotionCommander (defined in the Commander base class).

        Args:
            target_prim: The target prim to register.
        """
        self.target_prim = CortexObject(target_prim)  # Target prim will be in units of meters.
        self._reset_target_print_to_eff = True

    def get_end_effector_pose(self, config: Optional[np.ndarray] = None) -> tuple[np.ndarray, np.ndarray]:
        """Returns the end-effector pose as a pair (p, R), where p is the position and R is the.

        rotation matrix.

        If config is None (default), it uses the current applied action (i.e. current integration
        state of the underlying motion policy which the robot is trying to follow).        By using
        the applied action (rather than measured simulation state) the behavior is robust and
        consistent regardless of simulated PD control nuances.

        Otherwise, if config is set, it calculates the forward kinematics for the specified joint
        config. config should contain only the commanded joints.

        Args:
            config: An optional config input with the configuration to evaluate the end-effector
                pose at. The config should specify joint values for each of the commanded joints
                only, as defined by the underlying articulation_subset. If it's not provided, the
                latest applied action is used.

        Returns:
            The pose as a pair (p, R) where p is the position vector and R is the rotation
            matrix.
        """
        if config is None:
            # No config was specified, so fill it in with the current applied action.
            action = self.articulation_subset.get_applied_action()
            config = np.array(action.joint_positions)

        p, R = self.motion_policy.get_end_effector_pose(config)
        p = math_util.to_meters(p)
        return p, R

    def get_fk_T(self, config: Optional[np.ndarray] = None) -> np.ndarray:  # noqa: N802
        """Returns the end-effector transform as a 4x4 homogeneous matrix T.

        Calls get_end_effector_pose() internally; see that method's docstring for details.

        Args:
            config: An optional joint configuration to evaluate at.

        Returns:
            A 4x4 homogeneous transform matrix.
        """
        p, R = self.get_end_effector_pose(config)
        return math_util.pack_Rp(R, p)

    def get_fk_pq(self, config: Optional[np.ndarray] = None) -> tuple[np.ndarray, np.ndarray]:
        """Returns the end-effector transform as a (<position>,<quaternion>) pair.

        Calls get_end_effector_pose(config) internally; see that method's docstring for details.

        Args:
            config: An optional joint configuration to evaluate at.

        Returns:
            A PosePq containing the position and quaternion.
        """
        p, R = self.get_end_effector_pose(config)
        return PosePq(p, math_util.matrix_to_quat(R))

    def get_fk_p(self, config: Optional[np.ndarray] = None) -> np.ndarray:
        """Returns the position components of end-effector pose.

        control frame.

        Calls get_end_effector_pose(config) internally; see that method's docstring for details.

        Args:
            config: An optional joint configuration to evaluate at.

        Returns:
            The 3D position vector.
        """
        p, _ = self.get_end_effector_pose(config)
        return p

    def get_fk_R(self, config: Optional[np.ndarray] = None) -> np.ndarray:  # noqa: N802
        """Returns the rotational portion of the end-effector pose as a rotation matrix.

        Calls get_end_effector_pose(config) internally; see that method's docstring for details.

        Args:
            config: An optional joint configuration to evaluate at.

        Returns:
            The 3x3 rotation matrix.
        """
        _, R = self.get_end_effector_pose(config)
        return R

    def send_end_effector(self, *args: object, **kwargs: object) -> None:
        """An alias for sending an explicit MotionCommand object via send(). The arguments should.

        match those of MotionCommand. This is for convenience only.

        Args:
            *args: Positional arguments passed to MotionCommand.
            **kwargs: Keyword arguments passed to MotionCommand.
        """
        self.send(MotionCommand(*args, **kwargs))

    def set_posture_config(self, posture_config: np.ndarray) -> None:
        """Set the posture configuration of the underlying motion policy.

        The posture configure should specify joint values for each of the commanded joints as
        defined by the underlying articulation_subset object.

        Args:
            posture_config: The posture config vector, one value for each commanded joint.
        """
        policy = self.motion_policy._policy
        policy.set_cspace_attractor(posture_config)

    def set_posture_config_to_default(self) -> None:
        """Set the posture config back to the default posture config."""
        posture_config = self.motion_policy.get_default_cspace_position_target()
        self.set_posture_config(posture_config)

    def step(self, dt: float) -> None:
        """Step the motion commander to process the latest command and the underlying policy.

        Args:
            dt: The time step of this step.
        """
        if self.latest_command is not None:
            self._step_command_smoothing(self.latest_command)

        self.amp.physics_dt = dt
        self._sync_end_effector_target_to_motion_policy()
        self.motion_policy.update_world()

        action = self.amp.get_next_articulation_action()
        self.robot.get_articulation_controller().apply_action(action)

    def add_obstacle(self, obs: CortexObstacleType) -> None:
        """Add an obstacle to the underlying motion policy.

        The motion policy is the one underlying the ArticulationMotionPolicy passed in on
        construction.

        All added obstacles are tracked and made accessible by name via the obstacles member dict.
        On reset, the underlying motion policy typically resets entirely, including removing all the
        obstacles, but by adding the obstacles through this interface they will be automatically
        added back on each reset so the set of obstacles remains consistent.

        If adding the obstacle to the underlying policy is unsuccessful, it prints a message and
        does not include it in the obstacles dict.

        Args:
            obs: An obstacle represented as a core API type which can be added to the underlying
                motion policy. This can be any object type supported by the underlying motion
                policy's add_obstacle() method. The obstacle can be wrapped in a CortexObject.
        """
        obs_add = obs
        if hasattr(obs_add, "obj"):
            obs_add = obs.obj

        success = self.motion_policy.add_obstacle(obs_add)
        if not success:
            print(f"<failed to add obs: {obs.name}>")
            return

        self.obstacles[obs.name] = obs

    def disable_obstacle(self, obs: CortexObstacleType) -> None:
        """Disable the given object as an obstacle in the underlying motion policy.

        Disabling can be done repeatedly without enabling. The result is the same, the obstacle is
        disabled. The object can either be a core API object or a CortexObject wrapping the core API
        object.

        Args:
            obs: The obstacle to disable. The obstacle can be any added to the underlying motion
                policy.
        """
        try:
            # Handle cortex objects -- extract the underlying core api object.
            if hasattr(obs, "obj"):
                obs = obs.obj
            self.motion_policy.disable_obstacle(obs)
        except Exception as e:
            err_substr = "Attempted to disable an already-disabled obstacle"
            if err_substr in str(e):
                print("<lula error caught and ignored (obs already disabled)>")
            else:
                raise e

    def enable_obstacle(self, obs: CortexObstacleType) -> None:
        """Enable the given obsect as an obstacle in the underlying motion policy.

        Enabling can be done repeatedly without disabling. The result is the same, the obstacle is
        enabled. The object can either be a core API object or a CortexObject wrapping the core API
        object.

        Args:
            obs: The obstacle to enable. The obstacle can be any added to the underlying motion
                policy.
        """
        try:
            # Handle cortex obsects -- extract the underlying core api obsect.
            if hasattr(obs, "obj"):
                obs = obs.obj
            self.motion_policy.enable_obstacle(obs)
        except Exception as e:
            err_substr = "Attempted to enable an already-enabled obstacle"
            if err_substr in str(e):
                print("<lula error caught and ignored (obs already enabled)>")
            else:
                raise e

    def _sync_end_effector_target_to_motion_policy(self) -> None:
        """Set the underlying motion generator's target to the pose in the target prim.

        Switches between only the position portion of the target_prim pose and using the entire
        pose based on the value of _is_target_position_only set in _step_command_smoothing().
        """
        if self._reset_target_print_to_eff:
            self._step_command_smoothing(MotionCommand(self.get_fk_pq()))
            self._reset_target_print_to_eff = False

        target_translation, target_orientation = self.target_prim.get_world_pose()
        if self._is_target_position_only:
            self.motion_policy.set_end_effector_target(target_translation)
        else:
            self.motion_policy.set_end_effector_target(target_translation, target_orientation)

    def _step_command_smoothing(self, command: MotionCommand) -> None:
        """Processes the command's approach parameters, smooths the resulting pose, and apply the.

        smoothed pose to the target_prim. Additionally, passes the posture config into the
        underlying motion policy.

        The target_prim will, therefore, render the current approach offset and smoothed target.
        When the target in the motion command is position-only, the rotation component is filled in
        with the end-effector's current rotation matrix so the target_prim will rotate with the
        end-effector's rotation.

        Note the posture configure should be a full C-space configuration for the robot.

        Args:
            command: The latest command applied to this motion commander.
        """
        eff_T = self.get_fk_T()
        eff_p = eff_T[:3, 3]
        eff_R = eff_T[:3, :3]

        command = copy.deepcopy(command)

        self._is_target_position_only = command.target_pose.q is None
        if self._is_target_position_only:
            command.target_pose.q = math_util.matrix_to_quat(eff_R)

        if command.has_approach_params:
            target_T = command.target_pose.to_T()
            command.target_pose.p = calc_shifted_approach_target(target_T, eff_T, command.approach_params)

        adapted_command = MotionCommandAdapter(command)
        if self.smoothed_command is not None:
            self.smoothed_command.update(adapted_command, command.posture_config, eff_p, eff_R)

            target_p = self.smoothed_command.x
            target_R = self.smoothed_command.R
            target_T = math_util.pack_Rp(target_R, target_p)
            target_R, target_p = math_util.unpack_T(target_T)
            target_posture = self.smoothed_command.q
        else:
            target_T = command.target_pose.to_T()
            target_R, target_p = math_util.unpack_T(target_T)
            target_posture = command.posture_config

        self.target_prim.set_world_pose(position=target_p, orientation=math_util.matrix_to_quat(target_R))

        if target_posture is not None:
            self.set_posture_config(target_posture)
