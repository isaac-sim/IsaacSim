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

"""UR10 stacking scene setup and controller."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.robot.experimental.manipulators.examples.universal_robots.ur10 import UR10


class Stacking:
    """UR10 stacking scene setup and controller.

    Sets up a stacking scene with a UR10 robot and cubes for demonstration,
    and provides control logic for stacking operations.

    Args:
        robot_path: USD path where the robot should be created.
        cube_positions: List of initial cube positions.
        cube_size: Size of each cube.
        stack_target_position: Target position for stacking.
        offset: Additional offset to apply to positions.
        events_dt: Step counts for each phase.
        robot_name: Name/identifier for this robot (for logging purposes).
        usd_path: Optional path to a custom UR10 USD file (passed through to
            :class:`UR10`).  When ``None`` the packaged asset is used.
    """

    def __init__(
        self,
        robot_path: str = "/World/robot",
        cube_positions: list[np.ndarray] | None = None,
        cube_size: np.ndarray | None = None,
        stack_target_position: np.ndarray | None = None,
        offset: np.ndarray | None = None,
        events_dt: list[int] | None = None,
        robot_name: str = "",
        usd_path: str | None = None,
    ) -> None:
        self.robot_path = robot_path
        self.robot = None
        self._usd_path = usd_path
        self.cubes = []
        self.cube_paths = []
        self.robot_name = robot_name

        # Set default values
        if cube_size is None:
            cube_size = np.array([0.08, 0.08, 0.05])
        if cube_positions is None:
            # Cube positions: z should be cube_height/2 so cube sits on ground
            cube_positions = [
                np.array([0.25, 0.25, 0.025]),
                np.array([0.2, -0.2, 0.025]),
            ]
        if stack_target_position is None:
            stack_target_position = np.array([0.4, 0.4, 0.12])
        if offset is None:
            offset = np.array([0.0, 0.0, 0.0])

        self.cube_size = cube_size
        self.offset = offset
        self.cube_positions = [pos + offset for pos in cube_positions]
        self.stack_target_position = stack_target_position + offset

        # Define step counts for each phase
        self.events_dt = events_dt
        if self.events_dt is None:
            # Phase durations: [move_above, approach, grasp, lift, move_to_stack, release, retract]
            self.events_dt = [
                80,  # Phase 0: Move above cube
                40,  # Phase 1: Approach cube
                20,  # Phase 2: Close gripper to grasp
                130,  # Phase 3: Move cube to stack position
                20,  # Phase 4: Open gripper to release
                20,  # Phase 5: Retract
            ]

        # State tracking: [current_cube_index, current_phase, step_count]
        self._cube_index = 0
        self._phase = 0
        self._step = 0

    def setup_scene(self) -> None:
        """Set up the scene with robot, ground, lighting, and cubes."""
        stage = stage_utils.get_current_stage()
        if not stage.GetPrimAtPath("/World/ground_plane"):
            GroundPlane("/World/ground_plane")
            if not stage.GetPrimAtPath("/World/DistantLight"):
                distant_light = DistantLight("/World/DistantLight")
                distant_light.set_intensities(300)

        # Create UR10 robot with gripper
        self.robot = UR10(robot_path=self.robot_path, create_robot=True, attach_gripper=True, usd_path=self._usd_path)

        # Ensure xform ops exist on the articulation (required before set_world_poses on some USD assets)
        self.robot.reset_xform_op_properties()

        # Position robot at offset location (robot base should be at z=0, on the ground)
        robot_position = [self.offset[0], self.offset[1], 0.0]
        robot_orientation = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion (w, x, y, z)
        self.robot.set_world_poses(positions=robot_position, orientations=robot_orientation)

        # Create visual material for cubes (different colors for each cube)
        cube_colors = ["red", "green"]

        path_robot_name = self.robot_path.split("/")[-1] if "/" in self.robot_path else "robot"
        for i, cube_pos in enumerate(self.cube_positions):
            cube_path = f"/World/{path_robot_name}_Cube_{i}"
            self.cube_paths.append(cube_path)

            cube_shape = Cube(
                paths=cube_path,
                positions=cube_pos,
                sizes=1.0,
                scales=self.cube_size,
                colors=cube_colors[i % len(cube_colors)],
            )

            GeomPrim(paths=cube_shape.paths, apply_collision_apis=True)
            cube_rigid = RigidPrim(paths=cube_shape.paths)
            self.cubes.append(cube_rigid)

    def get_cube_names(self) -> list[str]:
        """Get the list of cube names/paths.

        Returns:
            List of cube prim paths.
        """
        return self.cube_paths

    def get_observations(self) -> dict:
        """Get current task observations.

        Returns:
            Dictionary with cube observations including positions.
        """
        observations = {}
        for i, (cube, cube_path) in enumerate(zip(self.cubes, self.cube_paths)):
            positions, orientations = cube.get_world_poses()
            # Convert to numpy if needed and get first element (assuming single cube per RigidPrim)
            if hasattr(positions, "numpy"):
                positions = positions.numpy()
            if hasattr(orientations, "numpy"):
                orientations = orientations.numpy()

            # Handle shape: if shape is (1, 3) or (1, 4), squeeze to (3,) or (4,)
            if positions.ndim > 1 and positions.shape[0] == 1:
                positions = positions[0]
            if orientations.ndim > 1 and orientations.shape[0] == 1:
                orientations = orientations[0]

            observations[cube_path] = {
                "position": positions,
                "orientation": orientations,
            }
        return observations

    def reset_cubes(self) -> None:
        """Reset all cubes to their initial positions."""
        for cube, initial_pos in zip(self.cubes, self.cube_positions):
            cube.set_world_poses(
                positions=initial_pos,
                orientations=[1.0, 0.0, 0.0, 0.0],
            )

    def reset_robot(self) -> None:
        """Reset the robot to its default pose."""
        if self.robot:
            self.robot.reset_to_default_pose()

    def reset(self) -> None:
        """Reset both robot and cubes and controller state."""
        self.reset_robot()
        self.reset_cubes()
        # Reset controller state
        self._cube_index = 0
        self._phase = 0
        self._step = 0

    def forward(self, ik_method: str = "damped-least-squares") -> bool:
        """Execute one step of the stacking operation using the specified IK method.

        Args:
            ik_method: The inverse kinematics method to use. Defaults to "damped-least-squares".

        Returns:
            True if a step was executed, False if the sequence is complete.
        """
        if self.is_done():
            return False

        if self.robot is None:
            return False

        # Get cube and target positions
        cube_path = self.cube_paths[self._cube_index]
        cube_prim = RigidPrim(cube_path)
        cube_pos = cube_prim.get_world_poses()[0].numpy()[0]

        # Calculate stack target position (each cube stacked on top of previous)
        cube_height = self.cube_size[2]  # Use z dimension as height
        stack_target_pos = np.array(
            [
                self.stack_target_position[0],
                self.stack_target_position[1],
                self.stack_target_position[2] + self._cube_index * cube_height,
            ]
        )

        goal_orientation = self.robot.get_downward_orientation()

        # Phase 0: Move above cube
        if self._phase == 0:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Moving above cube {self._cube_index}...")
            self.robot.open_gripper()

            goal_position = np.array([cube_pos[0], cube_pos[1], cube_pos[2] + 0.3])
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[0]:
                self._phase = 1
                self._step = 0

        # Phase 1: Approach cube
        elif self._phase == 1:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Approaching cube {self._cube_index}...")

            # Approach closer to cube top - cube center is at cube_pos, cube top is at cube_pos + cube_height/2
            goal_position = cube_pos + np.array([0.0, 0.0, self.cube_size[2] + 0.125])  # Slightly above cube top
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[1]:
                self._phase = 2
                self._step = 0

        # Phase 2: Grasp cube
        elif self._phase == 2:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Grasping cube {self._cube_index}...")

            self.robot.close_gripper()

            self._step += 1
            if self._step >= self.events_dt[2]:
                self._phase = 3
                self._step = 0

        # Phase 3: Move to stack position
        elif self._phase == 3:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Moving cube {self._cube_index} to stack position...")

            goal_position = stack_target_pos + np.array([0.0, 0.0, 0.2])
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[3]:
                self._phase = 4
                self._step = 0

        # Phase 4: Release cube
        elif self._phase == 4:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Releasing cube {self._cube_index}...")

            self.robot.open_gripper()

            self._step += 1
            if self._step >= self.events_dt[4]:
                self._phase = 5
                self._step = 0

        # Phase 5: Retract
        elif self._phase == 5:
            if self._step == 0:
                robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                print(f"{robot_id}: Retracting from cube {self._cube_index}...")

            _, current_position, _ = self.robot.get_current_state()
            goal_position = current_position + np.array([0.0, 0.0, 0.2])
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[5]:
                # Move to next cube
                self._cube_index += 1
                self._phase = 0
                self._step = 0
                if self._cube_index >= len(self.cube_paths):
                    robot_id = f"Robot {self.robot_name}" if self.robot_name else "Robot"
                    print(f"{robot_id}: Finished stacking all cubes!")

        return True

    def is_done(self) -> bool:
        """Check if the stacking sequence is complete.

        Returns:
            True if all cubes have been stacked. Otherwise False.
        """
        return self._cube_index >= len(self.cube_paths)
