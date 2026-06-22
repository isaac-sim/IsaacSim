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

"""RMPflow controller with cuMotion integration example for Franka motion planning."""

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)
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


class FrankaRmpFlowExample:
    """RMPflow controller with cuMotion integration for a Franka robot.

    Owns the full scene and controller state for the demo:
      - stage creation and asset loading (:meth:`load`)
      - controller setup (:meth:`setup`)
      - per-tick update (:meth:`step`, :meth:`update`)
      - teardown (:meth:`cleanup`)
    """

    def __init__(self) -> None:
        self._controller: RmpFlowController | None = None
        self._articulation: Articulation | None = None
        self._target: Cube | None = None
        self._world_binding: WorldBinding | None = None
        self._controlled_joint_names: list[str] | None = None
        self._robot_prim_path: str | None = None
        self._robot_joint_space: list[str] | None = None
        self._robot_site_space: list[str] | None = None
        self._tool_frame: str | None = None
        self._sim_time = 0.0
        self._controller_reset = False

    # ---------------------------------------------------------------- loading

    async def load(self) -> None:
        """Create a fresh stage, load the Franka, and prime the RMPflow controller.

        Single-shot load: creates the stage, references in the robot USD,
        defines the obstacle and target, allocates physics tensors, and
        runs :meth:`setup` so the controller is ready when LOAD returns.
        """
        await stage_utils.create_new_stage_async(template="default stage")
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)

        self._robot_prim_path = _ROBOT_PRIM_PATH
        assets_root = await get_assets_root_path_async()
        add_reference_to_stage(assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", _ROBOT_PRIM_PATH)
        self._articulation = Articulation(_ROBOT_PRIM_PATH)

        self._target = Cube(paths=_TARGET_PRIM_PATH, sizes=0.04, positions=[0.5, 0.0, 0.25])

        # Fixed cube obstacle with collision API.
        Cube(_OBSTACLE_PRIM_PATH, sizes=0.05, positions=[np.array([0.4, 0.0, 0.65])])
        GeomPrim(_OBSTACLE_PRIM_PATH, apply_collision_apis=True)

        ViewportManager.set_camera_view(camera="/OmniverseKit_Persp", eye=[2, 1.5, 2], target=[0, 0, 0])

        # Ensure a physics scene exists; allocate physics tensors without stepping.
        stage = stage_utils.get_current_stage()
        if not stage.GetPrimAtPath(_PHYSICS_SCENE_PATH).IsValid():
            UsdPhysics.Scene.Define(stage, _PHYSICS_SCENE_PATH)
        await omni.kit.app.get_app().next_update_async()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

        self.setup()

    # ---------------------------------------------------------------- setup

    def setup(self) -> None:
        """Build the RMPflow controller and world binding for the current scene."""
        # Drop any prior world binding so its GPU resources can be reclaimed.
        if self._world_binding is not None:
            self._cleanup_debug_prims()
            self._world_binding = None
        self._cleanup_debug_prims()

        robot_config = load_cumotion_supported_robot("franka")

        scene_query = SceneQuery()
        robot_base_positions, robot_base_orientations = self._articulation.get_world_poses()
        search_origin = robot_base_positions.numpy()[0] if robot_base_positions.shape[0] > 0 else [0.0, 0.0, 0.0]
        objects = scene_query.get_prims_in_aabb(
            search_box_origin=search_origin,
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[self._robot_prim_path],
        )

        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cone, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cylinder, ObstacleConfiguration("obb", 0.01))

        self._world_binding = WorldBinding(
            world_interface=CumotionWorldInterface(device="cpu"),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
        )
        self._world_binding.initialize()
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(
            poses=(robot_base_positions, robot_base_orientations)
        )
        self._world_binding.synchronize_transforms()

        self._robot_joint_space = self._articulation.dof_names
        self._robot_site_space = robot_config.robot_description.tool_frame_names()
        self._tool_frame = self._robot_site_space[0]

        self._controller = RmpFlowController(
            cumotion_robot=robot_config,
            cumotion_world_interface=self._world_binding.get_world_interface(),
            robot_joint_space=self._robot_joint_space,
            robot_site_space=self._robot_site_space,
            tool_frame=self._tool_frame,
        )
        self._controlled_joint_names = robot_config.controlled_joint_names

        # Move the target into a reachable pose with the EE looking down.
        quat = transform_utils.euler_angles_to_quaternion([0, np.pi, 0]).numpy()
        self._target.set_world_poses(positions=np.array([[0.5, 0, 0.7]]), orientations=np.array([quat]))

        self._sim_time = 0.0
        self._controller_reset = False

    def reset(self) -> None:
        """Re-run :meth:`setup` so the controller is fresh for another run."""
        self.setup()

    # --------------------------------------------------------------- per-tick

    def step(self, dt: float) -> None:
        """Per-physics-step update, guarded against invalid physics tensors.

        Safe to call before :meth:`setup` has run or while the articulation's
        physics tensors are not yet valid; both cases are no-ops.

        Args:
            dt: Physics time step in seconds.
        """
        if self._articulation is None or self._controller is None or self._world_binding is None:
            return
        if not self._articulation.is_physics_tensor_entity_valid():
            return
        self.update(dt)

    def update(self, step: float) -> None:
        """Drive the RMPflow controller for one physics step.

        Args:
            step: Physics time step in seconds.
        """
        if self._controller is None or self._world_binding is None:
            return

        target_positions, target_orientations = self._target.get_world_poses()
        setpoint_state = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=self._robot_site_space,
                positions=([self._tool_frame], target_positions),
                orientations=([self._tool_frame], target_orientations),
            ),
        )

        if not self._controller_reset:
            # First step only: pin the world-to-robot transform and seed controller state.
            self._world_binding.get_world_interface().update_world_to_robot_root_transforms(
                self._articulation.get_world_poses()
            )
            estimated_state = mg.RobotState(
                joints=mg.JointState.from_name(
                    robot_joint_space=self._robot_joint_space,
                    positions=(self._robot_joint_space, self._articulation.get_dof_positions()),
                    velocities=(self._robot_joint_space, self._articulation.get_dof_velocities()),
                )
            )
        else:
            estimated_state = None

        self._world_binding.synchronize_transforms()

        if not self._controller_reset:
            self._controller_reset = self._controller.reset(estimated_state, setpoint_state, self._sim_time)

        desired_state = self._controller.forward(estimated_state, setpoint_state, self._sim_time)
        if desired_state is not None and desired_state.joints.positions is not None:
            self._articulation.set_dof_position_targets(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices,
            )

        self._sim_time += step

    # --------------------------------------------------------------- teardown

    def cleanup(self) -> None:
        """Drop all USD/prim-wrapper references so the stage can fully release.

        Without this the soon-to-be-closed UsdStage often has a refcount > 1
        (debug visible as the ``Unexpected reference count of 2 for UsdStage``
        warning in omni.usd).
        """
        if self._world_binding is not None:
            self._cleanup_debug_prims()
            self._world_binding = None
        self._controller = None
        self._articulation = None
        self._target = None
        self._controlled_joint_names = None
        self._robot_prim_path = None
        self._robot_joint_space = None
        self._robot_site_space = None
        self._tool_frame = None
        self._controller_reset = False
        self._sim_time = 0.0

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
