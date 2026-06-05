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

"""This module provides the base classes for representing robots and building their command APIs by.

adding commanders.

The Cortex collaborative systems functional pipeline includes:
1. perception: sensor data --> entities and transforms
2. world model: entities and transforms --> USD
3. logical state monitoring: USD --> logical state
4. decisions: USD and logical state --> commands
5. command API: commands --> articulation actions
6. control: articulation --> actions to movement

This module implements the command API layer translating higher level commands produced by the
decision layer to the commanders that process them into lower-level control commands. Commanders
wrap policies and define the command API used to modulate the underlying policies' behavior.

Operationally, the commanders are added to CortexRobot objects and made available to state machines
and decider networks (decision layer) through the context object. Two examples of commanders are the
MotionCommander (see motion_commander.py) and the GripperCommander (see below). A decider node might
access them via

class MyAction(DfAction):
    def step(self):
        self.context.robot.arm.send_end_effector(target_pose)
        self.context.robot.gripper.close()

Here, self.context.robot is an instance of CortexRobot which has arm and gripper members
containing, respectively, a MotionCommander and a GripperCommander. The API of those commander
objects are directly callable from decider nodes and state objects.

The CortexRobot base class provides an add_commander(name, commander) method which will add the
commander and create an attribute with the provided name. So CortexRobot objects can be used
directly to wrap a loaded USD robot prim on the stage and add custom commanders to it. The APIs of
those commanders then become the command API available to the decision layer. Alternatively, we
provide some base classes with built in common commanders such as MotionCommandedRobot and derive
some robot-specific cortex robot classes with full collections of relevant commanders, such as
CortexFranka and CortexUr10.

See Also:
- commander.py for the Commander base class interface.
- motion_commander.py and GripperCommander (below) for example commanders.
- standalone_examples/api/isaacsim.cortex.framework for complete examples.
"""

from __future__ import annotations

from abc import abstractmethod
from collections import OrderedDict
from collections.abc import Sequence
from typing import Optional

import isaacsim.robot_motion.motion_generation.interface_config_loader as icl
import numpy as np
import omni.physics.tensors
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.framework.commander import Commander
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path_or_die
from isaacsim.cortex.framework.cortex_world import CommandableArticulation, CortexWorld
from isaacsim.cortex.framework.motion_commander import CortexObstacleType, MotionCommander
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.lula.motion_policies import RmpFlow, RmpFlowSmoothed


class CortexGripper(Commander):
    """Base class for a commander representing a parallel gripper. Creates a command API mimicking.

    standard parallel gripper commands.

    The two main commands are:
    1. Move-to: Servo the gripper to a specified width at a specified speed.
    2. Close-to-grasp: Close the gripper until a defined force is felt.

    These can be implemented by a general command which takes width, speed, and force parameters,
    with speed and force being optional. This class provides both a general API for these commands,
    and semantic commands such as open(), close(), move_to(), and close_to_grasp().

    On construction, an articulation subset is given which defines which joints are controlled by
    this commander. The specific mapping from the width generalized coordinate to the joint angles
    (and back) are robot specific and need to be defined by the deriving class by overriding the
    abstract methods joints_to_width(joint_positions) and width_to_joints(width).

    Note that command state is stored in the underlying Articulation which is reset separately from
    the commanders, so this commander doesn't explicitly need to override reset().

    Args:
        articulation_subset: The subset of joints controlled by this commander.
        opened_width: The width the gripper is opened to on open().
        closed_width: The width the gripper is closed to on close().
    """

    def __init__(self, articulation_subset: ArticulationSubset, opened_width: float, closed_width: float) -> None:
        super().__init__(articulation_subset)

        self.opened_width = opened_width
        self.closed_width = closed_width

    class Command:
        """Specifies the command parameters, including width, speed, and force.

        WARNING: The force parameter is often used by physical robots, so we include it here so it
        can be handed off to the command sent to the physical robot. However, it's not currently
        used in simulation. The action will just servo the desired position to the specified width
        and physics will apply the force that's generated based on any contact constraints.

        Args:
            width: The width to servo the gripper to.
            speed: The speed to move at. Optional. If not specified, the underlying PD control
                defines the speed.
            force: Max contact force to control to. Optional: If not specified, this command is
                treated as a move-to servo command.
        """

        def __init__(self, width: float, speed: Optional[float] = None, force: Optional[float] = None) -> None:
            self.width = width
            self.speed = speed
            self.force = force

    def get_width(self) -> float:
        """Returns the current width of the gripper based on the joint positions (not the command).

        Returns:
            The current gripper width.
        """
        return self.joints_to_width(self.articulation_subset.get_joint_positions())

    @abstractmethod
    def joints_to_width(self, joint_positions: Sequence[float]) -> float:
        """Implemented by the deriving class to define how to map the joints in the articulation.

        subset to the width value.

        Args:
            joint_positions: The vector of joint angles contributing to the gripper width.

        Returns:
            The calculated width of the gripper.
        """
        raise NotImplementedError()

    @abstractmethod
    def width_to_joints(self, width: float) -> np.ndarray:
        """Implemented by the deriving class to define how to map the width value to the joints in.

        the articulation subset.

        Note that this is a one to many map, but we assume the width value is a generalized
        coordinate for the robot, so there is a deterministic map from the width value to the vector
        of joint values.

        Args:
            width: The width of the gripper.

        Returns:
            The calculated vector of joint value that make the gripper the specified width.
        """
        raise NotImplementedError()

    def move_to(self, width: float, speed: Optional[float] = 0.2) -> None:
        """Move-to command: Move the gripper to the specified width.

        Args:
            width: The desired width.
            speed: An optional speed specifier. Defaults to .2 meters per second.
        """
        self.send(CortexGripper.Command(width, speed=speed))

    def open(self, speed: Optional[float] = 0.2) -> None:
        """Open command: Move the gripper to its open width.

        Args:
            speed: An optional speed specifier. Defaults to .2 meters per second.
        """
        self.send(CortexGripper.Command(self.opened_width, speed=speed))

    def close(self, speed: Optional[float] = 0.2) -> None:
        """Close command: Move the gripper to the closed width.

        Args:
            speed: An optional speed specifier. Defaults to .2 meters per second.
        """
        self.send(CortexGripper.Command(self.closed_width, speed=speed))

    def close_to_grasp(self, speed: Optional[float] = 0.2, force: Optional[float] = 40.0) -> None:
        """Close-to-grasp command: Close the gripper toward the closed width until a desired force.

        is measured.

        Note that in simulation, this command is processed the same as close(), but the physics
        contact constraint against the object ensures the grasp. The command is recorded with the
        force value as the latest command, though, so the force information can be transmitted to
        a physical robot as needed where the value will be used.

        Args:
            speed: An optional speed specifier. Defaults to .2 meters per second.
            force: An optional force specifier. Defaults to 40 Newtons.
        """
        self.send(CortexGripper.Command(self.closed_width, speed=speed, force=force))

    def is_open(self, thresh: Optional[float] = 0.01) -> bool:
        """Checks whether the gripper is currently open (to within a threshold).

        Args:
            thresh: An optional threshold paramters. Defaults to 1cm.

        Returns:
            True if open, False otherwise.
        """
        return self.opened_width - self.get_width() <= thresh

    def is_closed(self, thresh: Optional[float] = 0.01) -> bool:
        """Checks whether the gripper is currently closed (to within a threshold).

        Args:
            thresh: An optional threshold paramters. Defaults to 1cm.

        Returns:
            True if open, False otherwise.
        """
        return self.get_width() - self.closed_width <= thresh

    def step(self, dt: float) -> None:
        """Step is called every cycle as the processing engine for the commands.

        Args:
            dt: The time interval in seconds between calls to step.
        """
        if self.command is None:
            return

        if self.command.speed is None:
            # Process the command, but clear it as well so future steps are aborted until a new
            # command is sent.
            self._apply_width_action(self.command.width)
            self.clear()

        if self.command.speed is not None:
            width_action = self._get_applied_width_action()

            max_delta_width = dt * self.command.speed
            interval = self.command.width - width_action
            if abs(interval) > max_delta_width:
                delta_width = max_delta_width * interval / abs(interval)
                width_action += delta_width
            else:
                width_action = self.command.width
            self._apply_width_action(width_action)

    def _apply_width_action(self, width_action: float) -> None:
        """Helper method to apply the given width action to the underlying articulation subset.

        Args:
            width_action: The action to apply specified as a generalized coordinate width value.
        """
        self.articulation_subset.apply_action(joint_positions=self.width_to_joints(width_action))

    def _get_applied_width_action(self) -> float:
        """Helper method to retrieve the latest applied width action from the underlying.

        articulation subset.

        Returns:
            The retrieved generalized coordinate width action.
        """
        return self.joints_to_width(self.articulation_subset.get_applied_action().joint_positions)


class FrankaGripper(CortexGripper):
    """Franka specific parallel gripper.

    Specifies the gripper joints, provides mappings from width to joints, and defines the franka
    opened and closed widths.

    Args:
        articulation: The Articulation object containing the finger joints that will be controlled
            by this parallel graipper.
    """

    def __init__(self, articulation: SingleArticulation) -> None:
        super().__init__(
            articulation_subset=ArticulationSubset(articulation, ["panda_finger_joint1", "panda_finger_joint2"]),
            opened_width=0.08,
            closed_width=0.0,
        )

    def joints_to_width(self, joint_positions: Sequence[float]) -> float:
        """The width is simply the sum of the two prismatic joints.

        Args:
            joint_positions: The values for joints ["panda_finger_joint1", "panda_finger_joint2"].

        Returns:
            The width of the gripper corresponding to those joint positions.
        """
        return joint_positions[0] + joint_positions[1]

    def width_to_joints(self, width: float) -> np.ndarray:
        """Each joint is half of the width since the width is their sum.

        Args:
            width: The width of the gripper

        Returns:
            The values for joints ["panda_finger_joint1", "panda_finger_joint2"] giving the
            requested gripper width.
        """
        return np.array([width / 2, width / 2])


class CortexRobot(CommandableArticulation):
    """A robot is an Articulation with a collection of commanders commanding in combination the.

    collection of all joints.

    This class provides an API for adding commanders that are made available via named attributes.
    This method can be used to set up a robot's command API procedurally without deriving from this
    class. For instance, we can construct a robot's command API using:

        usd_path = get_assets_root_path_or_die() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        robot = CortexRobot("franka", prim_path)
        robot.add_commander("arm", MotionCommander(...))
        robot.add_commander("gripper", GripperCommander)

    Then, from a behavior script, we can access the API using

        robot.arm.send_end_effector(target_position=desired_position)
        robot.gripper.open()

    See also the CortexFranka and CortexUr10 classes which do the same, but from derived classes.

    Note: In the future, a robot will be multiple articulations (such as a mobile base, an arm, and
    a separate gripper. But for now we restrict it to a single Articulation which represents a
    single PhysX articulation.

    The robot's Articulation must be already available as a prim in the USD stage before this object
    can wrap it.

    Args:
        name: A name to give this robot. The robot can be looked up in the CortexWorld by this name
            once it's been added.
        prim_path: The path to the Articulation prim representing this robot in the USD stage.
        position: The position of the robot base relative to the prim on which the robot resides.
        orientation: The orientation of the robot's base  relative to the prim on which the robot resides.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ) -> None:
        if position is None:
            position = np.zeros(3)
        super().__init__(name=name, prim_path=prim_path, translation=position, orientation=orientation)

        self.commanders_step_dt = CortexWorld.instance().get_physics_dt()
        self.commanders_reset_needed = False
        self.commanders = OrderedDict()

    def add_commander(self, name: str, commander: Commander, make_attr: Optional[bool] = True) -> None:
        """Add a commander with the specified name. If make_attr is specified (default), it.

        additionally creates an attribute with the name given in the name field for referencing the
        commander.

        This method can be used to fully construct a robot's command API.

        Args:
            name: The name to give the commander. This will be used as the attribute name as well if
                make_attr is True.
            commander: The commander being added.
            make_attr: If True, additionally creates an attribute of the name given in the name
                field.
        """
        if make_attr:
            # Makes attribute self.<name> containing the commander.
            setattr(self, name, commander)
        self.commanders[name] = commander

    def set_commanders_step_dt(self, commanders_step_dt: float) -> None:
        """Set the internal dt member which is passed to each commander during their step(dt).

        calls.

        Args:
            commanders_step_dt: The time delta to use for commander steps.
        """
        self.commanders_step_dt = commanders_step_dt

    def flag_commanders_for_reset(self) -> None:
        """Flag the commanders to be reset on the next call to step_commanders()."""
        self.commanders_reset_needed = True

    def step_commanders(self) -> None:
        """Step all commanders added to this robot.

        All commanders are stepped using the step dt stored internally. That commanders_step_dt is
        either the default physics step dt set on construction or the value passed in through a call
        to set_commanders_step_dt().
        """
        if CortexWorld.instance().is_playing():
            self._reset_commanders_if_needed()
            for _, commander in self.commanders.items():
                commander.step(self.commanders_step_dt)

    def reset_commanders(self) -> None:
        """Reset all commanders added to this robot."""
        for _, commander in self.commanders.items():
            commander.post_reset()

    def _reset_commanders_if_needed(self) -> None:
        """Reset all commanders only if flagged."""
        if self.commanders_reset_needed:
            self.reset_commanders()
            self.commanders_reset_needed = False


class DirectSubsetCommander(Commander):
    """A simple commander which just passes a position and velocity joint-space command directly.

    through to the commanded joints as defined by the underlying articulation subset.
    """

    class Command:
        """The command is the desired joint positons and velocities for the commanded joints.

        defined by the underlying articulation subset.

        Args:
            q: Desired joint positions.
            qd: Desired joint velocities.
        """

        def __init__(self, q: Optional[np.ndarray], qd: Optional[np.ndarray] = None) -> None:
            self.q = q
            self.qd = qd

    def step(self, dt: float) -> None:
        """Step the commander by passing the command directly into the underlying articulation.

        subset.

        Args:
            dt: Time step between calls. Unused here.
        """
        if self.command is not None:
            self.articulation_subset.apply_action(self.command.q, self.command.qd)


class MotionCommandedRobot(CortexRobot):
    """A motion commanded robot is a Cortex robot with a built in motion commander accessible.

    both as arm_commander and through a semantically nice 'arm' member.

    Args:
        name: The name of the robot.
        prim_path: The path to the Articulation prim on the USD stage representing the robot.
        motion_policy_config: The config for the motion policy represented as a dict. See
            icl.load_supported_motion_policy_config() for details.
        position: The position of the robot. See CortexRobot's position parameter for details.
        orientation: The orientation of the robot. See CortexRobot's orientation parameter for details.
        settings: Settings for configuring parameters to the motion commander.
    """

    class Settings:
        """Settings for configuring motion commander.

        Args:
            active_commander: Robots deriving from this class can deactivate the motion commander by
                setting this to False. For instance, we might have one robot actively generating
                behavior and another of the same type simply following that behavior.
            smoothed_rmpflow: Use the SmoothedRmpFlow policy which includes jerk limiting. Useful
                when commanding physical robots. If False, uses the standard RmpFlow.
            smoothed_commands: Use smoothed commands to avoid large discrete jumps in commands.
                Useful when commanding physical robots.
        """

        def __init__(
            self,
            active_commander: Optional[bool] = True,
            smoothed_rmpflow: Optional[bool] = True,
            smoothed_commands: Optional[bool] = True,
        ) -> None:
            self.active_commander = active_commander
            self.smoothed_rmpflow = smoothed_rmpflow
            self.smoothed_commands = smoothed_commands

    def __init__(
        self,
        name: str,
        prim_path: str,
        motion_policy_config: dict,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        settings: Optional[Settings] = Settings(),
    ) -> None:
        super().__init__(name=name, prim_path=prim_path, position=position, orientation=orientation)
        self.settings = settings

        if settings.smoothed_rmpflow:
            self.motion_policy = RmpFlowSmoothed(**motion_policy_config)
        else:
            self.motion_policy = RmpFlow(**motion_policy_config)
        if self.settings.active_commander:
            articulation_motion_policy = ArticulationMotionPolicy(
                robot_articulation=self, motion_policy=self.motion_policy, default_physics_dt=self.commanders_step_dt
            )
            target_prim = VisualCuboid("/World/motion_commander_target", size=0.01, color=np.array([0.15, 0.15, 0.15]))
            self.arm_commander = MotionCommander(
                articulation_motion_policy, target_prim, use_smoothed_commands=self.settings.smoothed_commands
            )
        else:
            self.arm_commander = DirectSubsetCommander(ArticulationSubset(self, self.motion_policy.get_active_joints()))
        self.add_commander("arm", self.arm_commander)

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """On initialization, gravity is disabled (mimicking gravity compensation) and the default.

        joints state is set to the motion policy's default cspace position target (its default posture
        config).

        Users generally don't need to call this method explicitly. It's handled automatically on
        reset() when this robot is added to the CortexWorld.

        Args:
            physics_sim_view: Sim information required by the underlying Articulation initialization.
        """
        super().initialize(physics_sim_view)
        self.disable_gravity()
        self.set_joints_default_state(positions=self.default_config)

    @property
    def default_config(self) -> np.ndarray:
        """Accessor for the default posture config from the underlying motion policy. All.

        dimensions not commanded by the motion commander are set to 0.

        Returns: The default posture config as a full dimensional vector of joints.
        """
        q = np.zeros(self.num_dof)
        indices = self.arm.articulation_subset.joint_indices
        q[indices] = self.motion_policy.get_default_cspace_position_target()
        return q

    @property
    def registered_obstacles(self) -> dict[str, CortexObstacleType]:
        """Convenience accessor for the dictionary of obstacles added to the motion commander.

        This is the collection of obstacles the robot will avoid. The dict is a mapping from
        obstacle name to the obstacle object.

        Returns: The dictionary of obstacles.
        """
        return self.arm_commander.obstacles

    def register_obstacle(self, obs: CortexObstacleType) -> None:
        """Add an obstacle to the underlying motion commander.

        Args:
            obs: The obstacle to add.
        """
        self.arm_commander.add_obstacle(obs)


class CortexFranka(MotionCommandedRobot):
    """The Cortex Franka contains commanders for commanding the end-effector (a MotionCommander.

    governing the full arm) and the gripper (a FrankaGripper governing the fingers).

    Each of these commanders are accessible via members arm and gripper.

    This object only wraps an existing USD Franka on the stage at the specified prim_path. To
    add it to the stage first then wrap it, use the add_franka_to_stage() method.

    Note that position and orientation are both relative to the prim the Franka sits on.

    Args:
        name: A name for the Franka robot. Robots added to the CortexWorld should all have unique names.
        prim_path: The path to the Franka prim in the USD stage.
        position: The position of the robot. See CortexRobot's position parameter for details.
        orientation: The orientation of the robot. See CortexRobot's orientation parameter for details.
        use_motion_commander: When set to True (default), uses the motion commander. Otherwise,
            includes only a DirectSubsetCommander for the arm.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        use_motion_commander: bool = True,
    ) -> None:
        motion_policy_config = icl.load_supported_motion_policy_config("Franka", "RMPflowCortex")
        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(
                active_commander=use_motion_commander, smoothed_rmpflow=True, smoothed_commands=True
            ),
        )

        self.gripper_commander = FrankaGripper(self)
        self.add_commander("gripper", self.gripper_commander)

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Initialize using MotionCommandedRobot and add custom gain settings.

        Users generally don't need to call this method explicitly. It's handled automatically on
        reset() when this robot is added to the CortexWorld.

        Args:
            physics_sim_view: Sim information required by the underlying Articulation initialization.
        """
        super().initialize(physics_sim_view)

        verbose = True
        kps = np.array([6000000.0, 6000000.0, 6000000.0, 6000000.0, 2500000.0, 1500000.0, 500000.0, 6000.0, 6000.0])
        kds = np.array([300000.0, 300000.0, 300000.0, 300000.0, 90000.0, 90000.0, 90000.0, 1000.0, 1000.0])
        if verbose:
            print("setting franka gains:")
            print(f"- kps: {kps}")
            print(f"- kds: {kds}")
        self.get_articulation_controller().set_gains(kps, kds)


def add_franka_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
    use_motion_commander: bool = True,
) -> CortexFranka:
    """Adds a Franka to the stage at the specified prim_path, then wrap it as a CortexFranka object.

    Args:
        name: A name for the Franka robot.
        prim_path: The path to the Franka prim in the USD stage.
        usd_path: An optional path to the Franka USD asset to add. If a specific path is not
            provided, a default Franka USD path is used.
        position: The position of the robot.
        orientation: The orientation of the robot.
        use_motion_commander: When True (default), uses the motion commander.

    Returns:
        The constructed CortexFranka object.
    """
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    else:
        usd_path = get_assets_root_path_or_die() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    return CortexFranka(name, prim_path, position, orientation, use_motion_commander)


class CortexUr10(MotionCommandedRobot):
    """Cortex UR10 robot with commanders for the end-effector and suction gripper.

    Each of these commanders are accessible via members commander and gripper.

    This object only wraps an existing USD UR10 on the stage at the specified prim_path. To
    add it to the stage first then wrap it, use the add_ur10_to_stage() method.

    Note that position and orientation are both relative to the prim the UR10 sits on.

    Args:
        name: A name for the UR10 robot. Robots added to the CortexWorld should all have unique names.
        prim_path: The path to the Franka prim in the USD stage.
        position: The position of the robot. See CortexRobot's position parameter for details.
        orientation: The orientation of the robot. See CortexRobot's orientation parameter for details.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ) -> None:
        motion_policy_config = icl.load_supported_motion_policy_config("UR10", "RMPflowCortex")
        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(smoothed_rmpflow=False, smoothed_commands=False),
        )

        self._end_effector_prim_path = prim_path + "/ee_link"
        self.suction_gripper = SurfaceGripper(
            end_effector_prim_path=self._end_effector_prim_path,
            surface_gripper_path=self._end_effector_prim_path + "/SurfaceGripper",
        )

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Initialize using MotionCommandedRobot and also initialize the suction gripper.

        Users generally don't need to call this method explicitly. It's handled automatically on
        reset() when this robot is added to the CortexWorld.

        Args:
            physics_sim_view: Sim information required by the underlying Articulation initialization.
        """
        super().initialize(physics_sim_view)
        self.suction_gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)

    def post_reset(self) -> None:
        """Add a ``post_reset()`` call on the suction gripper.

        Users generally don't need to call this method explicitly. It's handled automatically on
        reset() when this robot is added to the CortexWorld.
        """
        super().post_reset()
        self.suction_gripper.post_reset()


def add_ur10_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
) -> CortexUr10:
    """Adds a UR10 to the stage at the specified prim_path, then wrap it as a CortexUr10 object.

    Args:
        name: A name for the UR10 robot.
        prim_path: The path to the UR10 prim in the USD stage.
        usd_path: An optional path to the UR10 USD asset to add. If a specific path is not
            provided, a default UR10 USD path is used.
        position: The position of the robot.
        orientation: The orientation of the robot.

    Returns:
        The constructed CortexUr10 object.
    """
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    else:
        usd_path = get_assets_root_path_or_die() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    return CortexUr10(name, prim_path, position, orientation)
