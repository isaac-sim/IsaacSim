# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
from enum import IntEnum

import carb
import numpy as np
import omni
import omni.timeline as timeline
import pxr
import usd.schema.isaac.robot_schema as robot_schema
import usd.schema.isaac.robot_schema.utils as rs_utils
from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, Usd, UsdPhysics
from usd.schema.isaac.robot_schema.utils import RobotLinkNode

from .mass_query import query_prims


class GainsTestMode(IntEnum):
    SINUSOIDAL = 0
    STEP = 1
    USER_PROVIDED = 2


class JointMode(IntEnum):
    POSITION = 0
    VELOCITY = 1
    NONE = 2


def get_original_spec_for_drive_API(stage: pxr.Usd.Stage, joint_drive_path: str, drive_type):
    drive_prim = stage.GetPrimAtPath(joint_drive_path)
    attr = pxr.UsdPhysics.DriveAPI(drive_prim, drive_type).GetStiffnessAttr()
    if attr:
        composition_stack = attr.GetPropertyStack()
        #  First one should be where the attribute is originally authored
        if composition_stack:
            return composition_stack[0]
    return None


def matrix_norm(matrix: Gf.Matrix3f) -> float:
    np_matrix = np.array(
        [
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]],
        ]
    )

    return np.linalg.norm(np_matrix)


def compute_parallel_axis_inertia(I_C: Gf.Matrix3f, mass: float, d: Gf.Vec3f) -> Gf.Matrix3f:
    # Compute ||d||^2
    d_dot_d = d[0] ** 2 + d[1] ** 2 + d[2] ** 2

    # Identity matrix
    I3 = Gf.Matrix3f(1.0)

    # Outer product d * d^T
    outer_dd = Gf.Matrix3f(
        Gf.Vec3f(d[0] * d[0], d[0] * d[1], d[0] * d[2]),
        Gf.Vec3f(d[1] * d[0], d[1] * d[1], d[1] * d[2]),
        Gf.Vec3f(d[2] * d[0], d[2] * d[1], d[2] * d[2]),
    )

    # Displacement term: ||d||^2 * I - d*d^T
    D = Gf.Matrix3f(I3)
    D *= d_dot_d
    D -= outer_dd

    # Final inertia tensor: I_P = I_C + m * D
    D *= mass
    I_P = Gf.Matrix3f(I_C)
    I_P += D

    return I_P


def transform_inertia_tensor(
    I_principal: Gf.Vec3f, rotation: Gf.Quatf, mass: float, displacement: Gf.Vec3f
) -> Gf.Matrix3f:
    """
    Transform a diagonal principal-axis inertia tensor to world frame and apply parallel axis theorem.

    Args:
        I_principal (Gf.Matrix3f): Diagonal inertia tensor in principal axes frame.
        rotation (Gf.Matrix3f): Rotation matrix from principal frame to world frame.
        mass (float): Mass of the object.
        displacement (Gf.Vec3f): Vector from center of mass to new origin in world frame.

    Returns:
        Gf.Matrix3f: Inertia tensor in world frame about the new point.
    """

    I_principal = Gf.Matrix3f(I_principal[0], 0, 0, 0, I_principal[1], 0, 0, 0, I_principal[2])

    rot_matrix = Gf.Matrix3f().SetRotate(Gf.Rotation(rotation))
    # Step 1: Rotate inertia tensor to world space
    I_world = rot_matrix * I_principal * rot_matrix.GetTranspose()

    # Step 2: Parallel axis theorem
    d = displacement
    d_dot_d = d[0] ** 2 + d[1] ** 2 + d[2] ** 2

    # Identity and outer product
    I3 = Gf.Matrix3f(1.0)
    outer_dd = Gf.Matrix3f(
        d[0] * d[0],
        d[0] * d[1],
        d[0] * d[2],
        d[1] * d[0],
        d[1] * d[1],
        d[1] * d[2],
        d[2] * d[0],
        d[2] * d[1],
        d[2] * d[2],
    )

    D = Gf.Matrix3f(I3)
    D *= d_dot_d
    D -= outer_dd
    D *= mass

    # Final transformed inertia tensor
    I_final = Gf.Matrix3f(I_world)
    I_final += D

    return I_final


def find_articulation_root(stage: pxr.Usd.Stage, robot_path: str):
    articulations = [
        a for a in Usd.PrimRange(stage.GetPrimAtPath(robot_path)) if a.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not articulations:
        if robot_path == "/":
            return None
        return find_articulation_root(stage, pxr.Sdf.Path(robot_path).GetParentPath())
    return str(articulations[0].GetPath())


class GainTuner:
    def __init__(self):
        self._timeline = timeline.get_timeline_interface()
        self._test_duration = 5.0
        self.reset()

    def reset(self):
        self._initialized = False
        self._articulation = None
        self._articulation_root = None
        self._robot_prim_path = None
        self._robot = None
        self._joint_position_commands = []
        self._joint_velocity_commands = []

        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []

        self._joint_indices = None

        self._data_ready = False

        self._test_timestep = 0

        self._gains_test_generator = None
        self._joint_acumulated_inertia = {}

        self.step = 0

    def stop_test(self):
        self._articulation.reset_to_default_state()

    def on_reset(self):
        self._initialized = False
        if self._robot_prim_path:
            robot_path = self._robot_prim_path
            self.reset()
            self.setup(robot_path)
        else:
            self.setup(None)
        self.step = 0

    def setup(self, robot_path):
        if robot_path == self._robot_prim_path or robot_path is None:
            return
        stage = omni.usd.get_context().get_stage()

        self._robot_prim_path = robot_path
        self._robot = stage.GetPrimAtPath(robot_path)
        self._robot_links = [l for l in pxr.Usd.PrimRange(self._robot) if l.HasAPI(robot_schema.Classes.LINK_API.value)]
        self._link_mass = query_prims(stage, [l.GetPath() for l in self._robot_links])

        async def update_link_mass():
            playing = True
            if not self._timeline.is_playing():
                self._timeline.play()
                playing = False
            await omni.kit.app.get_app().next_update_async()
            self.compute_joints_acumulated_inertia()
            if not playing:
                self._timeline.stop()

        asyncio.ensure_future(update_link_mass())
        self._robot_tree = rs_utils.GenerateRobotLinkTree(stage, self._robot)
        self._articulation_root = find_articulation_root(stage, self._robot_prim_path)
        self._articulation = Articulation(self._articulation_root)
        joints = [stage.GetPrimAtPath(self._articulation.dof_paths[0][i]) for i in range(self._articulation.num_dofs)]
        robot_joints = rs_utils.GetAllRobotJoints(stage, self._robot, True)
        self._joints = {i: j for i, j in enumerate(joints) if j in robot_joints}
        # print(self._joint_acumulated_inertia)
        self._joint_names = {i: self._articulation.dof_names[i] for i in self._joints.keys()}
        self._joint_map = {self._joint_names[i]: self._joints[i] for i in self._joints.keys()}

        self._all_joint_indices = self._articulation.get_dof_indices(self._joint_names.values()).list()

        self.initialize()

    def get_dof_type(self, dof_index: int):
        return self._articulation.dof_types[dof_index]

    def __del__(self):
        self._articulation = None
        del self._articulation

    def initialize(self):
        if self._articulation and self._timeline.is_playing():
            positions, orientations = self._articulation.get_world_poses()
            linear_velocities, angular_velocities = self._articulation.get_velocities()
            dof_positions = self._articulation.get_dof_positions()
            dof_velocities = self._articulation.get_dof_velocities()
            dof_efforts = self._articulation.get_dof_efforts()
            self._articulation.set_default_state(
                positions=positions,
                orientations=orientations,
                linear_velocities=linear_velocities,
                angular_velocities=angular_velocities,
                dof_positions=dof_positions,
                dof_velocities=dof_velocities,
                dof_efforts=dof_efforts,
            )
            self._articulation.reset_to_default_state()
            self._initialized = True

    @property
    def initialized(self) -> bool:
        """Whether the gain tuner has been initialized.

        Returns:
            bool: True if initialized, False otherwise.
        """
        return self._initialized

    @initialized.setter
    def initialized(self, initialized: bool):
        """Set the initialized state.

        Args:
            initialized: True if initialized, False otherwise.
        """
        self._initialized = initialized

    @property
    def joint_range_maximum(self) -> float:
        """Get the joint range maximum.

        Returns:
            float: Joint range maximum.
        """
        return self._joint_range_maximum

    @joint_range_maximum.setter
    def joint_range_maximum(self, maximum: float):
        """Set the joint range maximum.

        Args:
            maximum: Joint range maximum.
        """
        self._joint_range_maximum = maximum

    @property
    def position_impulse(self) -> float:
        """Get the position impulse.

        Returns:
            float: Position impulse.
        """
        return self._position_impulse

    @position_impulse.setter
    def position_impulse(self, impulse: float):
        """Set the position impulse.

        Args:
            impulse: Position impulse.
        """
        self._position_impulse = impulse

    @property
    def velocity_impulse(self) -> float:
        """Get the velocity impulse.

        Returns:
            float: Velocity impulse.
        """
        return self._velocity_impulse

    @velocity_impulse.setter
    def velocity_impulse(self, impulse: float):
        """Set the velocity impulse.

        Args:
            impulse: Velocity impulse.
        """
        self._velocity_impulse = impulse

    @property
    def robot(self):
        """Get the robot prim.

        Returns:
            The robot prim.
        """
        return self._robot

    def compute_joints_acumulated_inertia(self):
        if not self._robot:
            return
        robot_transform = omni.usd.get_world_transform_matrix(self._robot)
        joint_inertia = {}
        for joint in self._joints.values():
            backward_acumulated_inertia = pxr.Gf.Matrix3f()
            forward_acumulated_inertia = pxr.Gf.Matrix3f()
            backward_links, forward_links = rs_utils.GetLinksFromJoint(self._robot_tree, joint)
            joint_pose = rs_utils.GetJointPose(self._robot, joint)
            for link in backward_links:
                link_path = link.GetPath()
                if not (self._link_mass[link_path].valid and self._link_mass[link_path].done):
                    return
                link_pose = omni.usd.get_world_transform_matrix(link)
                diag_inertia = self._link_mass[link_path].diagonal_inertia
                principal_axes = self._link_mass[link_path].principal_axes
                mass = self._link_mass[link_path].mass
                center_of_mass = self._link_mass[link_path].center_of_mass
                world_com = pxr.Gf.Vec3f(
                    (
                        link_pose
                        * pxr.Gf.Matrix4d().SetTranslate(pxr.Gf.Vec3d(*center_of_mass))
                        * robot_transform.GetInverse()
                    ).ExtractTranslation()
                )
                distance = world_com - joint_pose.ExtractTranslation()
                transformed_inertia = transform_inertia_tensor(diag_inertia, principal_axes, mass, distance)
                backward_acumulated_inertia += transformed_inertia
            for link in forward_links:
                link_path = link.GetPath()
                link_pose = omni.usd.get_world_transform_matrix(link)
                if not (self._link_mass[link_path].valid and self._link_mass[link_path].done):
                    return
                diag_inertia = self._link_mass[link_path].diagonal_inertia
                principal_axes = self._link_mass[link_path].principal_axes
                mass = self._link_mass[link_path].mass
                center_of_mass = self._link_mass[link_path].center_of_mass
                world_com = pxr.Gf.Vec3f(
                    (
                        link_pose
                        * pxr.Gf.Matrix4d().SetTranslate(pxr.Gf.Vec3d(*center_of_mass))
                        * robot_transform.GetInverse()
                    ).ExtractTranslation()
                )
                distance = world_com - joint_pose.ExtractTranslation()
                transformed_inertia = transform_inertia_tensor(diag_inertia, principal_axes, mass, distance)
                forward_acumulated_inertia += transformed_inertia

            m1 = matrix_norm(forward_acumulated_inertia.GetTranspose() * forward_acumulated_inertia)
            m2 = matrix_norm(backward_acumulated_inertia.GetTranspose() * backward_acumulated_inertia)
            eq_inertia = 1
            if (m1 + m2) > 0:
                eq_inertia = m1 * m2 / (m1 + m2)

            joint_inertia[joint] = eq_inertia

        self._joint_acumulated_inertia = joint_inertia

    def get_articulation(self):
        return self._articulation

    def get_all_joint_indices(self):
        return self._all_joint_indices

    def get_permanent_fixed_joint_indices(self):
        return self._permanent_fixed_joint_indices

    def get_test_duration(self):
        return self._test_duration

    def is_data_ready(self):
        return self._data_ready

    def set_test_duration(self, duration):
        self._test_duration = duration

    ############################ Run Gains Test ######################################
    # def get_next_action(self, timestep, position_command_fn, velocity_command_fn, joint_indices):
    #     p = position_command_fn(timestep)
    #     v = velocity_command_fn(timestep)
    #     return ArticulationAction(p, v, joint_indices=joint_indices)

    def sinusoidal_step(self, timestep, sequence_index: int):

        position_joint_dof_indices = [
            i
            for i in self.test_params["sequence"][sequence_index]["joint_indices"]
            if self.joint_modes[i] == JointMode.POSITION
        ]
        velocity_joint_dof_indices = [
            i
            for i in self.test_params["sequence"][sequence_index]["joint_indices"]
            if self.joint_modes[i] == JointMode.VELOCITY
        ]

        position_joint_indices = [
            i
            for i, j in enumerate(self.test_params["sequence"][sequence_index]["joint_indices"])
            if self.joint_modes[j] == JointMode.POSITION
        ]
        velocity_joint_indices = [
            i
            for i, j in enumerate(self.test_params["sequence"][sequence_index]["joint_indices"])
            if self.joint_modes[j] == JointMode.VELOCITY
        ]

        lower_joint_limits, upper_joint_limits = [
            np.array(i.list()) for i in self._articulation.get_dof_limits(dof_indices=position_joint_dof_indices)
        ]
        amplitudes = np.array(
            [self.test_params["sequence"][sequence_index]["joint_amplitudes"][i] for i in position_joint_indices]
        )
        offsets = np.array(
            [self.test_params["sequence"][sequence_index]["joint_offsets"][i] for i in position_joint_indices]
        )
        periods = np.array(
            [self.test_params["sequence"][sequence_index]["joint_periods"][i] for i in position_joint_indices]
        )
        phases = np.array(
            [self.test_params["sequence"][sequence_index]["joint_phases"][i] for i in position_joint_indices]
        )
        range = upper_joint_limits - lower_joint_limits
        center = (upper_joint_limits + lower_joint_limits) / 2
        next_position_step = np.clip(
            range * amplitudes * (np.sin(2 * np.pi * timestep / periods + phases)) + center + offsets,
            lower_joint_limits,
            upper_joint_limits,
        )

        periods = np.array(
            [self.test_params["sequence"][sequence_index]["joint_periods"][i] for i in velocity_joint_indices]
        )
        phases = np.array(
            [self.test_params["sequence"][sequence_index]["joint_phases"][i] for i in velocity_joint_indices]
        )
        v_max = self._articulation.get_dof_max_velocities(dof_indices=velocity_joint_dof_indices).numpy()
        next_velocity_step = v_max * 2 * np.pi / periods * np.sin(2 * np.pi * timestep / periods + phases)

        return position_joint_dof_indices, next_position_step, velocity_joint_dof_indices, next_velocity_step

    def step_step(self, timestep, sequence_index: int):
        """Generate square wave position and velocity commands.

        Args:
            timestep: Current simulation time
            sequence_index: Index of the current test sequence

        Returns:
            Tuple containing position joint indices, position commands, velocity joint indices, and velocity commands
        """
        position_joint_dof_indices = [
            i
            for i in self.test_params["sequence"][sequence_index]["joint_indices"]
            if self.joint_modes[i] == JointMode.POSITION
        ]
        velocity_joint_dof_indices = [
            i
            for i in self.test_params["sequence"][sequence_index]["joint_indices"]
            if self.joint_modes[i] == JointMode.VELOCITY
        ]

        position_joint_indices = [
            i
            for i, j in enumerate(self.test_params["sequence"][sequence_index]["joint_indices"])
            if self.joint_modes[j] == JointMode.POSITION
        ]
        velocity_joint_indices = [
            i
            for i, j in enumerate(self.test_params["sequence"][sequence_index]["joint_indices"])
            if self.joint_modes[j] == JointMode.VELOCITY
        ]
        # Position control
        lower_joint_limits, upper_joint_limits = [
            np.array(i.list()) for i in self._articulation.get_dof_limits(dof_indices=position_joint_dof_indices)
        ]
        step_max = np.array(
            [self.test_params["sequence"][sequence_index]["joint_step_max"][i] for i in position_joint_indices]
        )
        step_min = np.array(
            [self.test_params["sequence"][sequence_index]["joint_step_min"][i] for i in position_joint_indices]
        )
        periods = np.array(
            [self.test_params["sequence"][sequence_index]["joint_periods"][i] for i in position_joint_indices]
        )
        phases = np.array(
            [self.test_params["sequence"][sequence_index]["joint_phases"][i] for i in position_joint_indices]
        )

        # Square wave between step_min and step_max
        next_position_step = np.where(np.sin(2 * np.pi * timestep / periods + phases) >= 0, step_max, step_min)
        next_position_step = np.clip(next_position_step, lower_joint_limits, upper_joint_limits)

        # Velocity control
        periods = np.array(
            [self.test_params["sequence"][sequence_index]["joint_periods"][i] for i in velocity_joint_indices]
        )
        phases = np.array(
            [self.test_params["sequence"][sequence_index]["joint_phases"][i] for i in velocity_joint_indices]
        )
        v_max = self._articulation.get_dof_max_velocities(dof_indices=velocity_joint_dof_indices).numpy()

        # Square wave between -v_max and +v_max
        next_velocity_step = np.where(np.sin(2 * np.pi * timestep / periods + phases) >= 0, v_max, -v_max)

        return position_joint_dof_indices, next_position_step, velocity_joint_dof_indices, next_velocity_step

    def initialize_gains_test(self, test_params):
        self.test_params = test_params
        self._test_duration = test_params["test_duration"]
        indices = self.test_params["joint_indices"]
        stiffnesses, dampings = [i.list() for i in self._articulation.get_dof_gains()]
        self.joint_modes = {
            indices[index]: (
                JointMode.POSITION
                if stiffnesses[indices[index]] != 0
                else JointMode.VELOCITY if dampings[indices[index]] != 0 else JointMode.NONE
            )
            for index in range(len(indices))
        }
        self._test_timestep = 0
        self._data_ready = False
        self._gains_test_generator = self.gains_test_script()
        pass

    def _compute_gains_test_dof_error_terms(self, joint_index):
        if joint_index in self._joint_indices:
            remapped_joint_index = np.argmax(self._joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_position_commands[:, remapped_joint_index]
                        - self._observed_joint_positions[:, joint_index]
                    ),
                    axis=0,
                )
            )
            vel_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_velocity_commands[:, remapped_joint_index]
                        - self._observed_joint_velocities[:, joint_index]
                    ),
                    axis=0,
                )
            )
        else:
            remapped_joint_index = np.argmax(self._fixed_joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._fixed_positions[remapped_joint_index] - self._observed_joint_positions[:, joint_index]
                    ),
                    axis=0,
                )
            )
            vel_rmse = np.sqrt(np.mean(np.square(self._observed_joint_velocities[:, joint_index]), axis=0))
        return pos_rmse, vel_rmse

    def compute_gains_test_error_terms(self):
        pos_rmse = []
        vel_rmse = []
        for index in range(self._articulation.num_dof):
            dof_pos_rmse, dof_vel_rmse = self._compute_gains_test_dof_error_terms(index)
            pos_rmse.append(dof_pos_rmse)
            vel_rmse.append(dof_vel_rmse)
        return np.array(pos_rmse), np.array(vel_rmse)

    def update_gains_test(self, step: float):
        try:
            self.step = step
            next(self._gains_test_generator)
            self._test_timestep += step
        except StopIteration:
            self._v_max = None
            self._T = None
            return True

    def gains_test_script(self):
        if self._articulation is None:
            return
        if self.test_params is None:
            carb.log_error("Attempted to run gains test without first calling initialize_test()")
            return

        test_mode = self.test_params["test_mode"]
        # Stores Executed commands
        self._joint_position_commands = []
        self._joint_velocity_commands = []

        # Stores Observed Velocities
        self._observed_joint_positions = []
        self._observed_joint_velocities = []

        # Stores Time of Commands
        self._command_times = []

        if test_mode == GainsTestMode.SINUSOIDAL:
            next_step_fn = self.sinusoidal_step
        elif test_mode == GainsTestMode.STEP:
            next_step_fn = self.step_step

        pos_idx, pos_step, vel_idx, vel_step = next_step_fn(0, 0)

        for seq in range(len(self.test_params["sequence"])):
            seq_time = 0
            self._articulation.reset_to_default_state()
            pos_idx, pos_step, vel_idx, vel_step = next_step_fn(seq_time, seq)

            self._articulation.set_dof_position_targets(pos_step, dof_indices=pos_idx)
            self._articulation.set_dof_velocity_targets(vel_step, dof_indices=vel_idx)

            default_pos_targets = np.copy(self._articulation.get_dof_position_targets().numpy()[0])
            default_vel_targets = np.copy(self._articulation.get_dof_velocity_targets().numpy()[0])

            self._joint_position_commands.append(np.copy(default_pos_targets))
            self._joint_velocity_commands.append(np.copy(default_vel_targets))

            self._observed_joint_positions.append(self._articulation.get_dof_positions().numpy()[0])
            self._observed_joint_velocities.append(self._articulation.get_dof_velocities().numpy()[0])
            self._command_times.append(self._test_timestep)

            yield ()

            while seq_time < self._test_duration:

                seq_time += self.step
                pos_idx, pos_step, vel_idx, vel_step = next_step_fn(seq_time, seq)
                self._articulation.set_dof_position_targets(pos_step, dof_indices=pos_idx)
                self._articulation.set_dof_velocity_targets(vel_step, dof_indices=vel_idx)
                # Fill back the default targets with the modified targets
                # print(pos_idx, pos_step)
                # print("before", pos_idx, default_pos_targets)
                default_pos_targets[pos_idx] = pos_step
                default_vel_targets[vel_idx] = vel_step
                # print("after", pos_idx, default_pos_targets)

                self._joint_position_commands.append(np.copy(default_pos_targets))
                self._joint_velocity_commands.append(np.copy(default_vel_targets))
                self._command_times.append(self._test_timestep)

                yield ()  # One Physics Step Happens

                self._observed_joint_positions.append(self._articulation.get_dof_positions().numpy()[0])
                self._observed_joint_velocities.append(self._articulation.get_dof_velocities().numpy()[0])

        self._articulation.reset_to_default_state()

        self._joint_position_commands = np.array(self._joint_position_commands)
        self._joint_velocity_commands = np.array(self._joint_velocity_commands)
        self._observed_joint_positions = np.array(self._observed_joint_positions)
        self._observed_joint_velocities = np.array(self._observed_joint_velocities)
        self._command_times = np.array(self._command_times)

        self._data_ready = True

        return

    ############################ For Plotting #######################################

    def get_joint_states_from_gains_test(self, joint_index: int):
        if len(self._observed_joint_positions) == 0:
            return (None, None, None, None, None)
        if joint_index < self._joint_position_commands.shape[1]:
            return (
                self._joint_position_commands[:, joint_index],
                self._joint_velocity_commands[:, joint_index],
                self._observed_joint_positions[:, joint_index],
                self._observed_joint_velocities[:, joint_index],
                self._command_times,
            )
        else:
            return (None, None, None, None, None)
