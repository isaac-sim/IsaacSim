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

"""Interactive path planning example with Franka robot using cuMotion graph-based planner."""

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.experimental.manipulators.examples.franka.franka import Franka
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    GraphBasedMotionPlanner,
    load_cumotion_supported_robot,
)

ROBOT_PRIM_PATH = "/World/robot"
TARGET_PATH = "/World/TargetCube"

MAX_VELOCITIES = [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]
MAX_ACCELERATIONS = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]


class PathPlanning(BaseSample):
    """Interactive path planning example using a Franka robot with cuMotion graph-based planner.

    A Franka robot computes collision-free paths to a draggable target cube using cuMotion's
    graph-based motion planner (RRT variants). Wall obstacles can be dynamically added and
    removed during simulation. Each "Plan To Target" click computes a full path and executes
    the resulting trajectory.
    """

    def __init__(self) -> None:
        super().__init__()
        self._robot: Franka | None = None
        self._planner: GraphBasedMotionPlanner | None = None
        self._cumotion_robot = None
        self._world_binding: mg.WorldBinding | None = None
        self._target_object: GeomPrim | None = None
        self._controlled_joint_names: list[str] | None = None
        self._controlled_dof_indices: list[int] | None = None
        self._trajectory: mg.Trajectory | None = None
        self._physics_callback_id: int | None = None
        self._t = 0.0
        self._walls: list[str] = []
        self._wall_count = 0

    def setup_scene(self) -> None:
        """Set up the scene with Franka robot, target cube, and ground plane."""
        self._robot = Franka(robot_path=ROBOT_PRIM_PATH, create_robot=True)

        GroundPlane("/World/ground_plane")
        dome_light = DomeLight("/World/dome_light")
        dome_light.set_intensities(1000)

        Cube(
            TARGET_PATH,
            sizes=0.05,
            positions=[0.65, 0.3, 0.4],
            colors=[1.0, 0.0, 0.0],
        )

    async def setup_post_load(self) -> None:
        """Create the planner, world binding, and set the camera."""
        ViewportManager.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3], camera="/OmniverseKit_Persp")

        self._target_object = GeomPrim(TARGET_PATH)

        self._build_world_binding()
        self._build_planner()

    def _build_world_binding(self) -> None:
        """Scan for collision prims and create the cuMotion WorldBinding."""
        scene_query = mg.SceneQuery()
        robot_pos, robot_ori = self._robot.get_world_poses()
        objects = scene_query.get_prims_in_aabb(
            search_box_origin=robot_pos.numpy()[0],
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[ROBOT_PRIM_PATH, TARGET_PATH],
        )

        obstacle_strategy = mg.ObstacleStrategy()
        obstacle_strategy.set_default_configuration(Mesh, mg.ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cone, mg.ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cylinder, mg.ObstacleConfiguration("obb", 0.01))

        self._world_binding = mg.WorldBinding(
            world_interface=CumotionWorldInterface(),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        self._world_binding.initialize()
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
        self._world_binding.synchronize_transforms()

    def _build_planner(self) -> None:
        """Create the cuMotion graph-based planner for the Franka."""
        self._cumotion_robot = load_cumotion_supported_robot("franka")
        self._controlled_joint_names = self._cumotion_robot.controlled_joint_names
        self._controlled_dof_indices = self._robot.get_dof_indices(self._controlled_joint_names).list()

        self._planner = GraphBasedMotionPlanner(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface(),
        )

    async def setup_pre_reset(self) -> None:
        """Deregister physics callback, delete wall prims, and reset trajectory state."""
        self._remove_physics_callback()
        for path in self._walls:
            stage_utils.delete_prim(path)
        self._walls.clear()
        self._wall_count = 0
        self._trajectory = None
        self._t = 0.0

    async def setup_post_reset(self) -> None:
        """Reset the robot pose and rebuild world binding and planner after simulation reset."""
        if self._robot is not None:
            self._robot.reset_to_default_pose()
        self._build_world_binding()
        self._build_planner()

    async def setup_post_clear(self) -> None:
        """Release all references when the scene is cleared."""
        self._cleanup_all()

    def physics_cleanup(self) -> None:
        """Release all references on extension hot-reload."""
        self._cleanup_all()

    def _cleanup_all(self) -> None:
        """Release all internal references and reset state."""
        self._remove_physics_callback()
        self._planner = None
        self._cumotion_robot = None
        self._controlled_joint_names = None
        self._controlled_dof_indices = None
        self._robot = None
        self._world_binding = None
        self._target_object = None
        self._trajectory = None
        self._walls.clear()
        self._wall_count = 0
        self._t = 0.0

    # ------------------------------------------------------------------
    # Physics callback management
    # ------------------------------------------------------------------

    def _remove_physics_callback(self) -> None:
        """Deregister the active physics callback if one exists."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    # ------------------------------------------------------------------
    # Plan and execute
    # ------------------------------------------------------------------

    async def _on_plan_to_target_event_async(self) -> None:
        """Plan a collision-free path to the target and execute the trajectory."""
        if self._robot is None or self._planner is None:
            return

        self._remove_physics_callback()

        if not app_utils.is_playing():
            app_utils.play()
            await app_utils.update_app_async()

        # Synchronize world state before planning
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(self._robot.get_world_poses())
        self._world_binding.synchronize_transforms()

        # Read current arm joint positions (only the 7 controlled joints)
        q_initial = self._robot.get_dof_positions(dof_indices=self._controlled_dof_indices).numpy().flatten()

        # Read target position
        target_positions, _ = self._target_object.get_world_poses()
        target_pos = target_positions.numpy().flatten().tolist()

        # Plan (translation-only so the gripper orientation is unconstrained)
        path = self._planner.plan_to_translation_target(q_initial=q_initial, translation_target=target_pos)

        if path is None:
            carb.log_warn(f"No plan could be generated to target position: {target_pos}")
            return

        # Convert path to a minimal-time trajectory
        robot_joint_space = self._robot.dof_names
        self._trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=MAX_VELOCITIES,
            max_accelerations=MAX_ACCELERATIONS,
            robot_joint_space=robot_joint_space,
            active_joints=self._controlled_joint_names,
        )

        if self._trajectory is None:
            carb.log_warn("Failed to convert path to trajectory.")
            return

        # Start executing the trajectory
        self._t = 0.0
        self._physics_callback_id = SimulationManager.register_callback(
            self._physics_step, event=SimulationEvent.PHYSICS_POST_STEP
        )

    def _physics_step(self, dt: float, context: object) -> None:
        """Sample the trajectory and apply joint position targets at each physics step.

        Args:
            dt: Physics step size in seconds.
            context: Callback context supplied by the simulation manager.
        """
        if self._robot is None or self._trajectory is None:
            return

        if self._t > self._trajectory.duration:
            self._remove_physics_callback()
            self._trajectory = None
            return

        target_state = self._trajectory.get_target_state(self._t)
        if target_state is not None and target_state.joints.positions is not None:
            self._robot.set_dof_position_targets(
                positions=target_state.joints.positions,
                dof_indices=target_state.joints.position_indices,
            )

        self._t += dt

    # ------------------------------------------------------------------
    # Wall obstacle management (called from extension UI)
    # ------------------------------------------------------------------

    def _on_add_wall_event(self) -> None:
        """Add a wall obstacle to the scene and register it with cuMotion."""
        self._wall_count += 1
        path = f"/World/wall_{self._wall_count}"
        cube = Cube(
            path,
            sizes=1.0,
            scales=[0.1, 0.3, 0.6],
            positions=[0.6, -0.2, 0.2],
            orientations=[0.5, 0.0, 0.0, 0.2588],
            colors=[0.0, 0.0, 1.0],
        )
        GeomPrim(path, apply_collision_apis=True)
        self._walls.append(path)

        if self._world_binding is not None:
            self._world_binding.get_world_interface().add_cubes(
                prim_paths=[path],
                sizes=cube.get_sizes(),
                scales=cube.get_local_scales(),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=cube.get_world_poses(),
                enabled_array=wp.array([[True]], dtype=wp.bool),
            )

    def _on_remove_wall_event(self) -> None:
        """Disable the most recently added wall in cuMotion and remove it from the stage."""
        if not self._walls:
            return
        path = self._walls.pop()
        if self._world_binding is not None:
            self._world_binding.get_world_interface().update_obstacle_enables(
                prim_paths=[path],
                enabled_array=wp.array([[False]], dtype=wp.bool),
            )
        stage_utils.delete_prim(path)

    def _remove_all_walls(self) -> None:
        """Remove all dynamically added walls from the stage and cuMotion."""
        while self._walls:
            self._on_remove_wall_event()
        self._wall_count = 0

    def walls_exist(self) -> bool:
        """Return whether any dynamically added walls remain.

        Returns:
            True if any dynamically added walls remain.
        """
        return len(self._walls) > 0

    def is_trajectory_active(self) -> bool:
        """Return whether a trajectory is currently being executed.

        Returns:
            True if a trajectory is currently being executed.
        """
        return self._trajectory is not None
