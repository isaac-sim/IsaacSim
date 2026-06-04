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

"""Trajectory optimization example using cuMotion for Franka motion planning."""

from typing import Any

import cumotion
import numpy as np
import omni.kit.app
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    TrajectoryOptimizer,
    load_cumotion_supported_robot,
)
from isaacsim.robot_motion.cumotion.impl.utils import isaac_sim_to_cumotion_pose
from isaacsim.robot_motion.experimental.motion_generation import (
    ObstacleConfiguration,
    ObstacleStrategy,
    SceneQuery,
    TrackableApi,
    WorldBinding,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdPhysics

_ROBOT_PRIM_PATH = "/panda"
_TARGET_PRIM_PATH = "/World/target"
_OBSTACLE_PRIM_PATH = "/World/obstacle"
_PHYSICS_SCENE_PATH = "/World/PhysicsScene"


class FrankaTrajectoryOptimizerExample:
    """Trajectory optimization with cuMotion for a Franka robot.

    Owns the full scene and optimizer state for a single Franka demo:
      - stage creation and asset loading (:meth:`load_robot_config`, :meth:`load_assets`)
      - planning (:meth:`plan_to_cspace_target`, :meth:`plan_to_task_space_target`)
      - trajectory execution (:meth:`step`)
      - teardown (:meth:`cleanup`)
    """

    def __init__(self) -> None:
        self._articulation: Articulation | None = None
        self._trajectory = None
        self._trajectory_time = 0.0
        self._target: Cube | None = None
        self._cumotion_robot = None
        self._robot_prim_path: str | None = None
        self._controlled_dof_indices: np.ndarray | None = None
        self._q_initial: np.ndarray | None = None
        self._first_trajectory = True

    # ---------------------------------------------------------------- loading

    async def load_robot_config(self) -> None:
        """Create a fresh stage and load the cuMotion robot config.

        Fast phase: local files only, no network I/O.  Safe to call from a
        UI callback so the UI can build joint sliders before the slower
        :meth:`load_assets` step completes.
        """
        await stage_utils.create_new_stage_async(template="default stage")
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)

        self._robot_prim_path = _ROBOT_PRIM_PATH
        self._cumotion_robot = load_cumotion_supported_robot("franka")

    async def load_assets(self) -> None:
        """Load robot USD, target, and obstacle; initialise physics.

        Slow phase: pulls the Franka USD from the assets root (network I/O).
        Must be called after :meth:`load_robot_config`.
        """
        assets_root = await get_assets_root_path_async()
        path_to_robot_usd = assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        add_reference_to_stage(path_to_robot_usd, self._robot_prim_path)
        self._articulation = Articulation(self._robot_prim_path)

        # Target cube (non-collision, can be moved around).  Rotated 90° about Z.
        angle = np.pi / 2
        target_orientation = np.array([np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0])
        self._target = Cube(
            paths=_TARGET_PRIM_PATH, sizes=0.04, positions=[0.5, 0.0, 0.7], orientations=target_orientation
        )

        # Fixed cube obstacle with collision API.
        Cube(_OBSTACLE_PRIM_PATH, sizes=0.1, positions=[np.array([0.25, 0.0, 0.5])])
        GeomPrim(_OBSTACLE_PRIM_PATH, apply_collision_apis=True)

        # Initial joint configuration (slightly off the default so planning is meaningful).
        self._controlled_dof_indices = (
            self._articulation.get_dof_indices(self._cumotion_robot.controlled_joint_names).numpy().flatten()
        )
        self._q_initial = self._cumotion_robot.robot_description.default_cspace_configuration()
        self._q_initial[0] = -np.pi / 2
        self._q_initial[1] = -np.pi / 8
        self._first_trajectory = True

        ViewportManager.set_camera_view(camera="/OmniverseKit_Persp", eye=[2, 1.5, 2], target=[0, 0, 0])

        # Ensure a physics scene exists; allocate physics tensors without stepping.
        stage = stage_utils.get_current_stage()
        if not stage.GetPrimAtPath(_PHYSICS_SCENE_PATH).IsValid():
            UsdPhysics.Scene.Define(stage, _PHYSICS_SCENE_PATH)
        await omni.kit.app.get_app().next_update_async()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

    # --------------------------------------------------------------- UI helpers

    def is_robot_config_loaded(self) -> bool:
        """True once :meth:`load_robot_config` has populated the cuMotion robot."""
        return self._cumotion_robot is not None

    def get_controlled_joint_names(self) -> list[str]:
        """Names of joints the optimizer controls."""
        if self._cumotion_robot is None:
            return []
        return self._cumotion_robot.controlled_joint_names

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(lower, upper)`` joint limits for all controlled joints."""
        if self._cumotion_robot is None:
            return np.empty(0), np.empty(0)
        kinematics = self._cumotion_robot.kinematics
        n = kinematics.num_cspace_coords()
        lower = np.array([kinematics.cspace_coord_limits(i).lower for i in range(n)])
        upper = np.array([kinematics.cspace_coord_limits(i).upper for i in range(n)])
        return lower, upper

    def get_default_target_configuration(self) -> np.ndarray:
        """Default slider/target values: cuMotion default cspace configuration with joints 0/1 nudged."""
        default_q = self._cumotion_robot.robot_description.default_cspace_configuration().copy()
        default_q[0] = np.pi / 2
        default_q[1] = -np.pi / 3
        return default_q

    # --------------------------------------------------------------- planning

    def _fetch_initial_position(self) -> None:
        if self._first_trajectory:
            self._first_trajectory = False
        else:
            self._q_initial = (
                self._articulation.get_dof_positions(dof_indices=self._controlled_dof_indices)
                .numpy()
                .flatten()
                .astype(np.float64)
            )

    def _cleanup_debug_prims(self) -> None:
        """Delete all prims under 'CumotionDebug' to clean up old debug visualization."""
        try:
            stage_utils.get_current_stage()
        except ValueError:
            # There is no stage.
            return
        debug_prim_paths = prim_utils.find_matching_prim_paths(".*CumotionDebug.*", traverse=True)
        if not debug_prim_paths:
            return
        # Filter to root-level prims (deleting a parent removes its children).
        debug_prim_paths_set = set(debug_prim_paths)
        root_prim_paths = [p for p in debug_prim_paths if p.rsplit("/", 1)[0] not in debug_prim_paths_set]
        for prim_path in root_prim_paths:
            try:
                stage_utils.delete_prim(prim_path)
            except ValueError:
                pass

    def setup_world_and_optimizer(self) -> tuple[Any, Any, Any]:
        """Set up world binding and trajectory optimizer (shared by both target types).

        Returns:
            Tuple of robot config, world binding, and trajectory optimizer.
        """
        robot_config = load_cumotion_supported_robot("franka")

        scene_query = SceneQuery()
        robot_base_positions, robot_base_orientations = self._articulation.get_world_poses()
        search_origin = robot_base_positions.numpy()[0] if robot_base_positions.shape[0] > 0 else [0.0, 0.0, 0.0]
        objects = scene_query.get_prims_in_aabb(
            search_box_origin=search_origin,
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[self._robot_prim_path, _TARGET_PRIM_PATH],
        )

        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cone, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cylinder, ObstacleConfiguration("obb", 0.01))

        world_binding = WorldBinding(
            world_interface=CumotionWorldInterface(visualize_debug_prims=True),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
        )
        world_binding.initialize()
        world_binding.get_world_interface().update_world_to_robot_root_transforms(
            poses=(robot_base_positions, robot_base_orientations)
        )
        world_binding.synchronize_transforms()

        # Use controlled_joint_names (not dof_names) to match cuMotion's expected joint space.
        optimizer = TrajectoryOptimizer(
            cumotion_robot=robot_config,
            robot_joint_space=robot_config.controlled_joint_names,
            cumotion_world_interface=world_binding.get_world_interface(),
        )
        return robot_config, world_binding, optimizer

    def plan_to_cspace_target(self, q_target: Any = None) -> str | None:
        """Plan a trajectory to a C-space target.

        Args:
            q_target: Target configuration. If None, uses default modified configuration.

        Returns:
            Error message if planning failed, None if successful.
        """
        self._cleanup_debug_prims()
        robot_config, world_binding, optimizer = self.setup_world_and_optimizer()
        self._fetch_initial_position()

        if q_target is None:
            q_target = self.get_default_target_configuration()
        else:
            q_target = np.array(q_target)

        # Path constraints must be explicitly set (even to none()).
        cspace_target = cumotion.TrajectoryOptimizer.CSpaceTarget(
            q_target,
            translation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.TranslationPathConstraint.none(),
            orientation_path_constraint=cumotion.TrajectoryOptimizer.CSpaceTarget.OrientationPathConstraint.none(),
        )

        # Check for collisions at start and goal before planning.
        robot_world_inspector = cumotion.create_robot_world_inspector(
            robot_config.robot_description, world_binding.get_world_interface().world_view
        )
        world_binding.get_world_interface().world_view.update()
        if robot_world_inspector.in_self_collision(self._q_initial) or robot_world_inspector.in_collision_with_obstacle(
            self._q_initial
        ):
            self._trajectory = None
            return "Planning failed: Robot is in collision at initial configuration."
        if robot_world_inspector.in_self_collision(q_target) or robot_world_inspector.in_collision_with_obstacle(
            q_target
        ):
            self._trajectory = None
            return "Planning failed: Robot is in collision at target configuration."

        self._trajectory = optimizer.plan_to_goal(self._q_initial, cspace_target)
        if self._trajectory is None:
            return (
                "Planning failed: Unable to find a valid trajectory.\n\n"
                "Common issues:\n"
                "  - Start/goal configurations outside joint limits\n"
                "  - Start/goal configurations in collision\n"
                "  - Insufficient optimization parameters"
            )

        self._trajectory_time = 0.0
        return None

    def plan_to_task_space_target(self) -> str | None:
        """Plan a trajectory to a task-space target (from the target cube's world pose).

        Returns:
            Error message if planning failed, None if successful.
        """
        self._cleanup_debug_prims()
        robot_config, world_binding, optimizer = self.setup_world_and_optimizer()

        target_positions, target_orientations = self._target.get_world_poses()
        target_position_world = target_positions.numpy()[0]
        target_orientation_world = target_orientations.numpy()[0]

        robot_base_positions, robot_base_orientations = self._articulation.get_world_poses()

        target_pose_base = isaac_sim_to_cumotion_pose(
            position_world_to_target=target_position_world,
            orientation_world_to_target=target_orientation_world,
            position_world_to_base=robot_base_positions,
            orientation_world_to_base=robot_base_orientations,
        )

        task_space_target = cumotion.TrajectoryOptimizer.TaskSpaceTarget(
            translation_constraint=cumotion.TrajectoryOptimizer.TranslationConstraint.target(
                target_pose_base.translation
            ),
            orientation_constraint=cumotion.TrajectoryOptimizer.OrientationConstraint.terminal_target(
                target_pose_base.rotation
            ),
        )

        self._fetch_initial_position()

        robot_world_inspector = cumotion.create_robot_world_inspector(
            robot_config.robot_description, world_binding.get_world_interface().world_view
        )
        world_binding.get_world_interface().world_view.update()
        if robot_world_inspector.in_self_collision(self._q_initial) or robot_world_inspector.in_collision_with_obstacle(
            self._q_initial
        ):
            self._trajectory = None
            return "Planning failed: Robot is in collision at initial configuration."

        self._trajectory = optimizer.plan_to_goal(self._q_initial, task_space_target)
        if self._trajectory is None:
            return (
                "Planning failed: Unable to find a valid trajectory.\n\n"
                "Common issues:\n"
                "  - Start configuration outside joint limits or in collision\n"
                "  - Target pose unreachable\n"
                "  - Insufficient optimization parameters"
            )

        self._trajectory_time = 0.0
        return None

    # --------------------------------------------------------------- per-tick

    def step(self, dt: float) -> None:
        """Advance trajectory execution by ``dt``.

        Safe to call before a trajectory has been planned or while the
        articulation's physics tensors are not yet valid; both are no-ops.
        """
        if self._trajectory is None or self._articulation is None:
            return
        if not self._articulation.is_physics_tensor_entity_valid():
            return

        target_state = self._trajectory.get_target_state(self._trajectory_time)
        if target_state is not None and target_state.joints.positions is not None:
            # NOTE: usually you would set targets - for the purpose of demonstrating a
            # trajectory, we write directly to the physics joint positions of the robot.
            self._articulation.set_dof_positions(
                positions=target_state.joints.positions,
                dof_indices=target_state.joints.position_indices,
            )

        self._trajectory_time = min(self._trajectory_time + dt, self._trajectory.duration)

    # --------------------------------------------------------------- teardown

    def cleanup(self) -> None:
        """Drop all USD/prim-wrapper references so the stage can fully release.

        Called by the UI when the extension shuts down or when a new scene is
        about to be loaded.  Without this the soon-to-be-closed UsdStage often
        has a refcount > 1 (debug visible as the ``Unexpected reference count
        of 2 for UsdStage`` warning in omni.usd).
        """
        self._articulation = None
        self._target = None
        self._cumotion_robot = None
        self._trajectory = None
        self._controlled_dof_indices = None
        self._q_initial = None
        self._robot_prim_path = None
        self._first_trajectory = True
