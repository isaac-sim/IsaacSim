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

"""Interactive sample demonstrating UR10 target following with cuMotion RMPflow control."""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.experimental.manipulators.examples.universal_robots.ur10 import UR10
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)

ROBOT_PRIM_PATH = "/World/ur10_robot"
TARGET_PATH = "/World/TargetCube"


class FollowTarget(BaseSample):
    """Interactive sample demonstrating UR10 target following with cuMotion RMPflow.

    A UR10 robot arm tracks a draggable target cube using cuMotion's RMPflow algorithm
    for real-time motion planning with collision avoidance. Obstacles can be dynamically
    added and removed during simulation.
    """

    def __init__(self) -> None:
        super().__init__()
        self._robot: UR10 | None = None
        self._controller: RmpFlowController | None = None
        self._cumotion_robot = None
        self._world_binding: mg.WorldBinding | None = None
        self._target_object: GeomPrim | None = None
        self._physics_callback_id: int | None = None
        self._rmpflow_reset_needed = True
        self._t = 0.0
        self._obstacles: list[str] = []
        self._obstacle_count = 0

    def setup_scene(self) -> None:
        """Set up the scene with UR10 robot, target cube, and ground plane."""
        self._robot = UR10(robot_path=ROBOT_PRIM_PATH, create_robot=True, attach_gripper=False)

        GroundPlane("/World/ground_plane")
        dome_light = DomeLight("/World/dome_light")
        dome_light.set_intensities(1000)

        Cube(
            TARGET_PATH,
            sizes=0.05,
            positions=[0.5, 0.0, 0.3],
            colors=[1.0, 0.0, 0.0],
        )

    async def setup_post_load(self) -> None:
        """Create the controller, world binding, and set the camera."""
        ViewportManager.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3], camera="/OmniverseKit_Persp")

        self._target_object = GeomPrim(TARGET_PATH)

        self._build_world_binding()
        self._build_controller()

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

    def _build_controller(self) -> None:
        """Create the cuMotion RMPflow controller for the UR10."""
        self._cumotion_robot = load_cumotion_supported_robot("ur10")
        joint_space = self._robot.dof_names
        site_space = self._cumotion_robot.robot_description.tool_frame_names()
        self._tool_frame = site_space[0]
        self._controller = RmpFlowController(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface(),
            robot_joint_space=joint_space,
            robot_site_space=site_space,
            tool_frame=self._tool_frame,
        )
        self._controller.get_rmp_flow_config().set_param("cspace_target_rmp/damping_gain", 0.9)

    async def setup_pre_reset(self) -> None:
        """Deregister physics callback and reset tracking state."""
        self._remove_physics_callback()
        self._rmpflow_reset_needed = True
        self._t = 0.0

    async def setup_post_reset(self) -> None:
        """Reset the robot to its default pose after a simulation reset."""
        if self._robot is not None:
            self._robot.reset_to_default_pose()

    async def setup_post_clear(self) -> None:
        """Release all references when the scene is cleared."""
        self._cleanup_all()

    def physics_cleanup(self) -> None:
        """Release all references on extension hot-reload."""
        self._cleanup_all()

    def _cleanup_all(self) -> None:
        self._remove_physics_callback()
        self._controller = None
        self._cumotion_robot = None
        self._robot = None
        self._world_binding = None
        self._target_object = None
        self._obstacles.clear()
        self._obstacle_count = 0
        self._rmpflow_reset_needed = True
        self._t = 0.0

    # ------------------------------------------------------------------
    # Physics callback management
    # ------------------------------------------------------------------

    def _remove_physics_callback(self) -> None:
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    # ------------------------------------------------------------------
    # RMPflow state helpers
    # ------------------------------------------------------------------

    def _get_estimated_state(self) -> mg.RobotState:
        names = self._robot.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self._robot.get_dof_positions()),
                velocities=(names, self._robot.get_dof_velocities()),
            )
        )

    def _create_setpoint_state(self) -> mg.RobotState:
        target_positions, _ = self._target_object.get_world_poses()

        n = int(target_positions.shape[0])
        target_orientations = wp.array([[0.0, -1.0, 0.0, -1.0]] * n, dtype=wp.float32, device=target_positions.device)

        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[self._tool_frame],
                positions=([self._tool_frame], target_positions),
                orientations=([self._tool_frame], target_orientations),
            ),
        )

    # ------------------------------------------------------------------
    # Follow target event (called from extension UI)
    # ------------------------------------------------------------------

    async def _on_follow_target_event_async(self, val: bool) -> None:
        """Start or stop the follow-target physics loop.

        Args:
            val: True to start following, False to stop.
        """
        if val:
            if not app_utils.is_playing():
                app_utils.play()
                await app_utils.update_app_async()
            self._rmpflow_reset_needed = True
            self._physics_callback_id = SimulationManager.register_callback(
                self._physics_step, event=SimulationEvent.PHYSICS_POST_STEP
            )
        else:
            self._remove_physics_callback()

    def _physics_step(self, dt: float, context: object) -> None:
        """Per-step callback: reset or advance the RMPflow controller."""
        if self._robot is None or self._controller is None:
            return

        if self._rmpflow_reset_needed:
            self._t = 0.0
            estimated = self._get_estimated_state()
            setpoint = self._create_setpoint_state()
            if not self._controller.reset(estimated, setpoint, t=self._t):
                return
            self._rmpflow_reset_needed = False
        else:
            self._t += dt
            self._world_binding.get_world_interface().update_world_to_robot_root_transforms(
                self._robot.get_world_poses()
            )
            self._world_binding.synchronize_transforms()

            estimated = self._get_estimated_state()
            setpoint = self._create_setpoint_state()
            desired = self._controller.forward(estimated, setpoint, self._t)

            if desired is not None and desired.joints.positions is not None:
                self._robot.set_dof_position_targets(
                    positions=desired.joints.positions,
                    dof_indices=desired.joints.position_indices,
                )

    # ------------------------------------------------------------------
    # Obstacle management (called from extension UI)
    # ------------------------------------------------------------------

    def _on_add_obstacle_event(self) -> None:
        """Add a collision cube obstacle to the scene and register it with cuMotion."""
        self._obstacle_count += 1
        path = f"/World/obstacle_{self._obstacle_count}"
        offset = 0.1 * self._obstacle_count
        cube = Cube(path, sizes=0.05, positions=[0.35 + offset, 0.0, 0.45])
        GeomPrim(path, apply_collision_apis=True)
        self._obstacles.append(path)

        if self._world_binding is not None:
            self._world_binding.get_world_interface().add_cubes(
                prim_paths=[path],
                sizes=cube.get_sizes(),
                scales=cube.get_local_scales(),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=cube.get_world_poses(),
                enabled_array=wp.array([[True]], dtype=wp.bool),
            )

    def _on_remove_obstacle_event(self) -> None:
        """Disable the most recently added obstacle in cuMotion and remove it from the stage."""
        if not self._obstacles:
            return
        path = self._obstacles.pop()
        if self._world_binding is not None:
            self._world_binding.get_world_interface().update_obstacle_enables(
                prim_paths=[path],
                enabled_array=wp.array([[False]], dtype=wp.bool),
            )
        stage_utils.delete_prim(path)

    def obstacles_exist(self) -> bool:
        """Return whether any dynamically added obstacles remain."""
        return len(self._obstacles) > 0
