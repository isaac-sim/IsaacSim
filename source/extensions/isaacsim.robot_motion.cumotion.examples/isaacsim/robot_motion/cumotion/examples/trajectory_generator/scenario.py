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

"""Trajectory generation example with cuMotion for a UR10 robot."""

import carb
import cumotion
import numpy as np
import omni.kit.app
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import (
    TrajectoryGenerator,
    load_cumotion_supported_robot,
)
from isaacsim.robot_motion.cumotion.impl.utils import (
    cumotion_to_isaac_sim_pose,
    isaac_sim_to_cumotion_pose,
)
from isaacsim.storage.native import get_assets_root_path, get_assets_root_path_async
from pxr import UsdPhysics

_ROBOT_PRIM_PATH = "/ur10"
_TOOL_FRAME_NAME = "ee_link"
_PHYSICS_SCENE_PATH = "/World/PhysicsScene"
_VISUALIZED_FRAMES_PATH = "/visualized_frames"


class UR10TrajectoryGeneratorExample:
    """Trajectory generation with cuMotion for a UR10 robot.

    Demonstrates how to generate trajectories from:
      - C-space waypoints (:meth:`setup_cspace_trajectory`)
      - Task-space path specifications (:meth:`setup_taskspace_trajectory`)
      - Hybrid composite path specifications (:meth:`setup_hybrid_trajectory`)
    """

    def __init__(self) -> None:
        self._articulation: Articulation | None = None
        self._trajectory = None
        self._trajectory_time = 0.0
        self._robot_joint_space: list[str] | None = None
        self._controlled_joint_names: list[str] | None = None
        self._robot_config = None
        self._generator: TrajectoryGenerator | None = None
        self._tool_frame_name = _TOOL_FRAME_NAME

    # ---------------------------------------------------------------- loading

    async def load(self) -> None:
        """Create a fresh stage, load the UR10, allocate physics, and build the generator."""
        await stage_utils.create_new_stage_async(template="sunlight")
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)

        assets_root = await get_assets_root_path_async()
        add_reference_to_stage(assets_root + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd", _ROBOT_PRIM_PATH)
        self._articulation = Articulation(_ROBOT_PRIM_PATH)

        ViewportManager.set_camera_view(camera="/OmniverseKit_Persp", eye=[2, 1.5, 2], target=[0, 0, 0])

        # Ensure a physics scene exists; allocate physics tensors without stepping.
        stage = stage_utils.get_current_stage()
        if not stage.GetPrimAtPath(_PHYSICS_SCENE_PATH).IsValid():
            UsdPhysics.Scene.Define(stage, _PHYSICS_SCENE_PATH)
        await omni.kit.app.get_app().next_update_async()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

        self.setup()

    def setup(self) -> None:
        """Build the trajectory generator for the current articulation."""
        self._robot_config = load_cumotion_supported_robot("ur10")
        self._robot_joint_space = self._articulation.dof_names
        self._controlled_joint_names = self._robot_config.controlled_joint_names
        self._generator = TrajectoryGenerator(
            cumotion_robot=self._robot_config,
            robot_joint_space=self._robot_joint_space,
        )

    # ------------------------------------------------------------- trajectories

    def setup_cspace_trajectory(self) -> None:
        """Set up a C-space trajectory from a fixed sequence of waypoints."""
        c_space_points = np.array(
            [
                [-0.41, 0.5, -2.36, -1.28, 5.13, -4.71],
                [-1.43, 1.0, -2.58, -1.53, 6.0, -4.74],
                [-2.83, 0.34, -2.11, -1.38, 1.26, -4.71],
                [-0.41, 0.5, -2.36, -1.28, 5.13, -4.71],  # Return to initial
            ]
        )

        # Visualize c-space targets in task space.
        kinematics = self._robot_config.kinematics
        robot_base_positions, robot_base_orientations = self._articulation.get_world_poses()
        for i, point in enumerate(c_space_points):
            pose_base_to_ee = kinematics.pose(point, self._tool_frame_name)
            position_world, quaternion_world = cumotion_to_isaac_sim_pose(
                pose_base_to_target=pose_base_to_ee,
                position_world_to_base=robot_base_positions,
                orientation_world_to_base=robot_base_orientations,
            )
            self._add_target_frame(i, position_world.numpy(), quaternion_world.numpy())

        self._trajectory = self._generator.generate_trajectory_from_cspace_waypoints(waypoints=c_space_points)
        self._trajectory_time = 0.0

    def setup_taskspace_trajectory(self) -> None:
        """Set up a task-space trajectory from a fixed sequence of poses."""
        robot_base_positions, robot_base_orientations = self._articulation.get_world_poses()

        task_space_position_targets = np.array(
            [[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]]
        )
        task_space_orientation_targets = np.tile(np.array([0, 1, 0, 0]), (5, 1))

        for i, (position, orientation) in enumerate(zip(task_space_position_targets, task_space_orientation_targets)):
            self._add_target_frame(i, position, orientation)

        initial_pose = isaac_sim_to_cumotion_pose(
            position_world_to_target=task_space_position_targets[0],
            orientation_world_to_target=task_space_orientation_targets[0],
            position_world_to_base=robot_base_positions,
            orientation_world_to_base=robot_base_orientations,
        )
        path_spec = cumotion.create_task_space_path_spec(initial_pose)

        for i in range(1, len(task_space_position_targets)):
            target_pose = isaac_sim_to_cumotion_pose(
                position_world_to_target=task_space_position_targets[i],
                orientation_world_to_target=task_space_orientation_targets[i],
                position_world_to_base=robot_base_positions,
                orientation_world_to_base=robot_base_orientations,
            )
            path_spec.add_linear_path(target_pose)

        self._trajectory = self._generator.generate_trajectory_from_path_specification(
            path_specification=path_spec, tool_frame_name=self._tool_frame_name
        )
        self._trajectory_time = 0.0

    def setup_hybrid_trajectory(self) -> None:
        """Set up a composite trajectory combining task-space and C-space segments."""
        initial_c_space_robot_pose = np.array([0, 0, 0, 0, 0, 0])
        composite_spec = cumotion.create_composite_path_spec(initial_c_space_robot_pose)

        # Demonstrate every movement type available in a task-space path spec.
        angle0 = np.pi / 2
        axis0 = np.array([1.0, 0.0, 0.0])
        quat0 = np.array([np.cos(angle0 / 2), *(np.sin(angle0 / 2) * axis0)])
        t0 = np.array([0.3, -0.1, 0.3])
        pose0 = isaac_sim_to_cumotion_pose(position_world_to_target=t0, orientation_world_to_target=quat0)
        task_space_spec = cumotion.create_task_space_path_spec(pose0)

        # Linear path between two poses.
        t1 = np.array([0.3, -0.1, 0.5])
        angle1 = np.pi / 3
        axis1 = np.array([1.0, 0.0, 0.0])
        quat1 = np.array([np.cos(angle1 / 2), *(np.sin(angle1 / 2) * axis1)])
        pose1 = isaac_sim_to_cumotion_pose(position_world_to_target=t1, orientation_world_to_target=quat1)
        task_space_spec.add_linear_path(pose1)

        # Pure translation (constant rotation assumed).
        task_space_spec.add_translation(pose0.translation)
        # Pure rotation.
        task_space_spec.add_rotation(pose0.rotation)

        # Three-point arc with constant orientation.
        t2 = np.array([0.3, 0.3, 0.3])
        midpoint = np.array([0.3, 0, 0.5])
        task_space_spec.add_three_point_arc(t2, midpoint, constant_orientation=True)
        # Three-point arc with tangent orientation.
        task_space_spec.add_three_point_arc(t0, midpoint, constant_orientation=False)
        # Three-point arc with orientation target.
        pose2 = isaac_sim_to_cumotion_pose(position_world_to_target=t2, orientation_world_to_target=quat1)
        task_space_spec.add_three_point_arc_with_orientation_target(pose2, midpoint)
        # Tangent arc with constant orientation. Tangent arcs are circles between two points.
        task_space_spec.add_tangent_arc(t0, constant_orientation=True)
        # Tangent arc with tangent orientation.
        task_space_spec.add_tangent_arc(t2, constant_orientation=False)
        # Tangent arc with orientation target.
        task_space_spec.add_tangent_arc_with_orientation_target(pose0)

        # Demonstrate a C-space path spec.
        c_space_spec = cumotion.create_cspace_path_spec(initial_c_space_robot_pose)
        c_space_spec.add_cspace_waypoint(np.array([0, 0.5, -2.0, -1.28, 5.13, -4.71]))

        # Combine task-space and C-space specs into a composite spec.
        transition_mode = cumotion.CompositePathSpec.TransitionMode.FREE
        composite_spec.add_task_space_path_spec(task_space_spec, transition_mode)
        composite_spec.add_cspace_path_spec(c_space_spec, transition_mode)

        self._trajectory = self._generator.generate_trajectory_from_path_specification(
            path_specification=composite_spec, tool_frame_name=self._tool_frame_name
        )
        if self._trajectory is None:
            carb.log_warn(
                "No trajectory could be computed from composite path spec. The path may contain unreachable poses."
            )
        self._trajectory_time = 0.0

    # --------------------------------------------------------------- per-tick

    def step(self, dt: float) -> None:
        """Advance trajectory execution by ``dt``, guarded against invalid physics tensors.

        Safe to call before a trajectory has been planned or while the
        articulation's physics tensors are not yet valid; both are no-ops.
        """
        if self._trajectory is None or self._articulation is None:
            return
        if not self._articulation.is_physics_tensor_entity_valid():
            return
        self.update(dt)

    def update(self, step: float) -> None:
        """Apply the next trajectory sample to the articulation.

        Args:
            step: Physics time step in seconds.
        """
        if self._trajectory is None:
            return

        desired_state = self._trajectory.get_target_state(self._trajectory_time)
        if desired_state is not None:
            self._articulation.set_dof_positions(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices,
            )

        self._trajectory_time += step
        # Loop trajectory by resetting time when it reaches duration.
        if self._trajectory_time >= self._trajectory.duration:
            self._trajectory_time = 0.0

    def reset(self) -> None:
        """Clear any visualized frames and the currently active trajectory."""
        prim = prim_utils.get_prim_at_path(_VISUALIZED_FRAMES_PATH)
        if prim and prim.IsValid():
            stage_utils.delete_prim(_VISUALIZED_FRAMES_PATH)

        self._trajectory = None
        self._trajectory_time = 0.0

    # --------------------------------------------------------------- teardown

    def cleanup(self) -> None:
        """Drop all USD/prim-wrapper references so the stage can fully release.

        Without this the soon-to-be-closed UsdStage often has a refcount > 1
        (debug visible as the ``Unexpected reference count of 2 for UsdStage``
        warning in omni.usd).
        """
        self._articulation = None
        self._trajectory = None
        self._robot_joint_space = None
        self._controlled_joint_names = None
        self._robot_config = None
        self._generator = None

    # --------------------------------------------------------------- helpers

    def _add_target_frame(self, index: int, position: np.ndarray, orientation: np.ndarray) -> None:
        """Add a visualization frame prim at ``position`` / ``orientation``."""
        frame_path = f"{_VISUALIZED_FRAMES_PATH}/target_{index}"
        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", frame_path)
        frame = XformPrim(frame_path, reset_xform_op_properties=True)
        frame.set_world_poses(
            positions=np.array([position], dtype=np.float32),
            orientations=np.array([orientation], dtype=np.float32),
        )
        frame.set_local_scales(np.array([[0.04, 0.04, 0.04]], dtype=np.float32))
