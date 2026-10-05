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

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper


class RobotJointControlTask(BaseTask):
    """Task for robot joint control demonstration."""

    def __init__(self, name: str = "robot_joint_control_task") -> None:
        super().__init__(name=name, offset=None)
        self._robot = None
        self._robot_name = "franka_robot"

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        # Add ground plane
        scene.add_default_ground_plane()

        # Add Franka robot
        asset_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Robots/FrankaRobot/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")

        # Define gripper
        gripper = ParallelGripper(
            end_effector_prim_path="/World/Franka/panda_rightfinger",
            joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
            joint_opened_positions=np.array([0.04, 0.04]),
            joint_closed_positions=np.array([0.0, 0.0]),
            action_deltas=np.array([0.04, 0.04]),
        )

        # Create manipulator
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/Franka",
                name=self._robot_name,
                end_effector_prim_path="/World/Franka/panda_hand",
                gripper=gripper,
            )
        )

        # Add a target cube for reference
        scene.add(
            DynamicCuboid(
                prim_path="/World/TargetCube",
                name="target_cube",
                position=np.array([0.5, 0.0, 0.5]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        return

    def get_params(self) -> dict:
        return {"robot_name": {"value": self._robot_name, "modifiable": False}}

    def get_observations(self) -> dict:
        robot = self._robot
        observations = {
            self._robot_name: {
                "joint_positions": robot.get_joint_positions(),
                "end_effector_position": robot.end_effector.get_world_pose()[0],
                "end_effector_orientation": robot.end_effector.get_world_pose()[1],
            }
        }
        return observations

    def pre_step(self, control_index: int, simulation_time: float) -> None:
        return

    def post_reset(self) -> None:
        return


class RobotJointControl(BaseSample):
    """Example demonstrating direct robot joint position control."""

    def __init__(self) -> None:
        super().__init__()
        self._robot = None
        self._articulation_controller = None
        self._joint_positions = None
        self._num_joints = 9  # 7 arm joints + 2 gripper joints for Franka

    def setup_scene(self):
        world = self.get_world()
        world.add_task(RobotJointControlTask())
        return

    async def setup_post_load(self):
        self._task = list(self._world.get_current_tasks().values())[0]
        self._task_params = self._task.get_params()
        self._robot = self._world.scene.get_object(self._task_params["robot_name"]["value"])
        self._articulation_controller = self._robot.get_articulation_controller()
        self._joint_positions = self._robot.get_joint_positions()
        return

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("joint_control_step"):
            world.remove_physics_callback("joint_control_step")
        return

    def world_cleanup(self):
        self._robot = None
        self._articulation_controller = None
        self._joint_positions = None
        return

    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        if self._robot is not None:
            return self._robot.get_joint_positions()
        return np.zeros(self._num_joints)

    def get_num_joints(self) -> int:
        """Get number of joints."""
        return self._num_joints

    def set_joint_position(self, joint_index: int, position: float):
        """Set position for a specific joint."""
        if self._joint_positions is not None and 0 <= joint_index < len(self._joint_positions):
            self._joint_positions[joint_index] = position

    def apply_joint_positions(self):
        """Apply the current joint positions to the robot."""
        if self._articulation_controller is not None and self._joint_positions is not None:
            from isaacsim.core.api.utils.types import ArticulationAction
            action = ArticulationAction(joint_positions=self._joint_positions)
            self._articulation_controller.apply_action(action)

    async def _on_start_control_event_async(self, val):
        """Start or stop joint control simulation."""
        world = self.get_world()
        if val:
            await world.play_async()
            world.add_physics_callback("joint_control_step", self._on_joint_control_step)
        else:
            if world.physics_callback_exists("joint_control_step"):
                world.remove_physics_callback("joint_control_step")
        return

    def _on_joint_control_step(self, step_size):
        """Physics callback for joint control."""
        self.apply_joint_positions()
        return

    def move_to_home_position(self):
        """Move robot to home position."""
        # Franka home position
        home_positions = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])
        self._joint_positions = home_positions
        self.apply_joint_positions()

    def open_gripper(self):
        """Open the gripper."""
        if self._robot is not None:
            self._robot.gripper.open()

    def close_gripper(self):
        """Close the gripper."""
        if self._robot is not None:
            self._robot.gripper.close()
