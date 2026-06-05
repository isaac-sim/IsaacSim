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

"""Bin-filling task definition for the UR10 robot."""

import random

import carb
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.robot.manipulators.examples.universal_robots import UR10
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf


class BinFilling(BaseTask):
    """Task using UR10 robot to fill a bin with cubes and showcase the surface gripper torque/force limits.

    Args:
        name: Task name identifier. Should be unique if added to the World.
    """

    def __init__(self, name: str = "bin_filling") -> None:
        BaseTask.__init__(self, name=name, offset=None)
        self._ur10_robot = None
        self._packing_bin = None
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self._ur10_asset_path = self._assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_filling.usd"
        self._cube_size_m = 0.05
        self._cubes: list[DynamicCuboid] = []
        self._active_cubes = 0
        self._max_cubes = 50
        self._cubes_to_add = 0
        self._pipe_position = np.array([0, 0.85, 1.2]) / get_stage_units()
        self._target_position = np.array([0, 0.85, -0.44]) / get_stage_units()
        self._bin_initial_position = np.array([0.35, 0.15, -0.40]) / get_stage_units()
        self._bin_size = np.array([0.25, 0.35, 0.20]) / get_stage_units()
        return

    def get_current_num_of_cubes_to_add(self) -> int:
        """Number of cubes left to drop from the pipe.

        Returns:
            Number of cubes left to drop from the pipe.
        """
        return self._cubes_to_add

    def set_up_scene(self, scene: Scene) -> None:
        """Loads the stage USD and adds the robot and packing bin to the World's scene.

        Args:
            scene: The world's scene.
        """
        super().set_up_scene(scene)
        add_reference_to_stage(usd_path=self._ur10_asset_path, prim_path="/World/Scene")
        self._ur10_robot = scene.add(
            UR10(prim_path="/World/Scene/ur10", name="my_ur10", gripper_usd=None, attach_gripper=True)
        )
        self._ur10_robot.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        self._packing_bin = scene.add(
            SingleRigidPrim(
                prim_path="/World/Scene/bin",
                name="packing_bin",
                position=self._bin_initial_position,
                orientation=euler_angles_to_quat(np.array([0, 0, np.pi / 2])),
            )
        )

        # Pre-create all cube prims up-front (hidden + rigid bodies disabled).
        # This avoids creating/deleting USD references during simulation or during reset.
        self._create_cube_pool()
        return

    def _create_cube_pool(self) -> None:
        """Create the cube pool as dynamic cubes (hidden + rigid body physics disabled)."""
        if len(self._cubes) > 0:
            return

        offscreen = np.array([0.0, 0.0, -1000.0]) / get_stage_units()
        default_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        for i in range(self._max_cubes):
            prim_path = f"/World/cube_{i}"
            cube = DynamicCuboid(
                prim_path=prim_path,
                name=f"cube_{i}",
                position=offscreen,
                orientation=default_orientation,
                size=1.0,
                scale=np.array([self._cube_size_m, self._cube_size_m, self._cube_size_m]),
                visible=False,
                color=np.array([0.6, 0.6, 0.6]),
            )
            cube.disable_rigid_body_physics()
            self._cubes.append(cube)
        self._active_cubes = 0

    def get_observations(self) -> dict:
        """Returns current observations from the task needed for the behavioral layer at each time step.

           Observations:
            - packing_bin
                - position
                - orientation
                - target_position
                - size
            - my_ur10:
                - joint_positions
                - end_effector_position
                - end_effector_orientation

        Returns:
            Dictionary containing packing bin and robot observations.
        """
        joints_state = self._ur10_robot.get_joints_state()
        bin_position, bin_orientation = self._packing_bin.get_world_pose()
        end_effector_position, end_effector_orientation = self._ur10_robot.end_effector.get_world_pose()
        # TODO: change values with USD
        return {
            "packing_bin": {
                "position": bin_position,
                "orientation": bin_orientation,
                "target_position": self._target_position,
                "size": self._bin_size,
            },
            "my_ur10": {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
                "end_effector_orientation": end_effector_orientation,
            },
        }

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Executed before the physics step.

        Args:
            time_step_index: Current time step index
            simulation_time: Current simulation time.
        """
        BaseTask.pre_step(self, time_step_index=time_step_index, simulation_time=simulation_time)
        if self._cubes_to_add > 0 and self._active_cubes < len(self._cubes) and time_step_index % 30 == 0:
            self._add_cube()
        return

    def post_reset(self) -> None:
        """Executed after reseting the scene."""
        self._cubes_to_add = 0
        self._active_cubes = 0
        return

    def add_cubes(self, cubes_number: int = 10) -> None:
        """Adds number of cubes to be added by the pipe.

        Args:
            cubes_number: Number of cubes to be added by the pipe.
        """
        self._cubes_to_add += cubes_number
        return

    def _add_cube(self) -> None:
        """Activates and spawns the next cube from the pool at the pipe position with random orientation."""
        if self._active_cubes >= len(self._cubes):
            self._cubes_to_add = 0
            return
        orientation = np.array([random.random(), random.random(), random.random(), random.random()])
        orientation = orientation / np.linalg.norm(orientation)
        cube = self._cubes[self._active_cubes]
        cube.set_visibility(True)
        cube.set_world_pose(position=self._pipe_position, orientation=orientation)
        cube.enable_rigid_body_physics()
        self._active_cubes += 1
        self._cubes_to_add -= 1
        return

    def cleanup(self) -> None:
        """Deactivate spawned cubes when resetting (hide + disable rigid bodies)."""
        count = self._active_cubes
        if count <= 0:
            return

        offscreen = np.array([0.0, 0.0, -1000.0]) / get_stage_units()
        with Sdf.ChangeBlock():
            for i in range(count):
                cube = self._cubes[i]
                cube.set_visibility(False)
                cube.set_world_pose(position=offscreen, orientation=None)
                cube.disable_rigid_body_physics()
        self._active_cubes = 0
        return

    def get_params(self) -> dict:
        """Task parameters are.

            - bin_name
            - robot_name.

        Returns:
            Defined parameters of the task.
        """
        params_representation = {}
        params_representation["bin_name"] = {"value": self._packing_bin.name, "modifiable": False}
        params_representation["robot_name"] = {"value": self._ur10_robot.name, "modifiable": False}
        return params_representation
