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

"""Test module for validating robot motion trajectory generation functionality across multiple robot models."""

from __future__ import annotations

import asyncio
import json
import os

import carb

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
import lula
import numpy as np
import omni.kit.test
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats, rotvecs_to_quats
from isaacsim.core.utils.prims import delete_prim
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_trajectory import ArticulationTrajectory
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.trajectory_generator import (
    LulaCSpaceTrajectoryGenerator,
    LulaTaskSpaceTrajectoryGenerator,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdLux


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestTrajectoryGenerator(omni.kit.test.AsyncTestCase):
    """Test class for validating trajectory generation functionality in robotic motion planning.

    This class provides comprehensive test cases for various trajectory generators including Lula
    c-space and task-space trajectory generators. Tests cover multiple robot models such as
    Franka Panda, Cobotta Pro series, UR10, Fanuc CRX10IAL, FR3, and Techman TM12.

    The test suite validates trajectory generation accuracy by comparing generated trajectories
    against expected end-effector positions and orientations. Each test loads a robot USD file,
    sets up the simulation environment, generates trajectories using different path specifications,
    and verifies that the robot reaches target waypoints within specified distance thresholds.

    Key testing scenarios include:
    - C-space trajectory generation with cubic spline and linear interpolation
    - Task-space trajectory generation from point sequences
    - Task-space trajectory generation from path specifications including composite paths
    - Rectangular and circular path following with rotations
    - Solver parameter configuration and limit setting
    - Trajectory execution with articulation controllers

    The class inherits from omni.kit.test.AsyncTestCase to support asynchronous test execution
    required for USD stage operations and timeline control.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment before running each test.

        Initializes physics settings, timeline interface, extension manager, policy configuration,
        and creates a new USD stage for the test.
        """
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_motion.motion_generation")
        self._mg_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._mg_extension_path, "motion_policy_configs")
        self.assertTrue(os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")))
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        await create_new_stage_async()
        await update_stage_async()

    async def _create_light(self) -> None:
        """Create a sphere light in the USD stage.

        Adds a sphere light at position [6.5, 0, 12] with radius 2 and intensity 100000
        to provide illumination for the test scene.
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath().pathString)).set_world_pose([6.5, 0, 12])

    async def _prepare_stage(self, robot: object) -> None:
        """Prepare the USD stage for trajectory testing.

        Initializes the simulation context, creates lighting, configures the robot with
        specific gains for trajectory following, and starts the timeline.

        Args:
            robot: The robot to prepare for testing.
        """
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()

        world = World()

        await world.initialize_simulation_context_async()
        await self._create_light()

        self._timeline.play()
        await update_stage_async()

        robot.initialize()
        robot.disable_gravity()

        # These gains generically seem to do very well at following the trajectory generator output compared to
        # stiff gains with P=10^15 D=10^5
        robot.get_articulation_controller().set_gains(
            10**15 * np.ones(robot.num_dof), 10**12 * np.ones(robot.num_dof), save_to_usd=True
        )

        self._robot.post_reset()
        await update_stage_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up the test environment after running each test.

        Stops the timeline, waits for assets to finish loading, clears the motion generator,
        and removes the World instance.
        """
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._mg = None
        await update_stage_async()
        World.clear_instance()

    async def test_lula_c_space_traj_gen_franka(self) -> None:
        """Test Lula C-space trajectory generation with the Franka robot.

        Tests multiple trajectory scenarios including different waypoints, timestamps,
        and interpolation types to verify C-space trajectory generation functionality.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"

        ee_frame = "panda_rightfinger"

        task_space_traj = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6], [0, 0, 0.7]])

        orientation_target = rotvecs_to_quats(np.array([np.pi, 0, 0]))

        await self._test_lula_c_space_traj_gen(
            usd_path,
            robot_name,
            robot_prim_path,
            ee_frame,
            task_space_traj,
            orientation_target,
            distance_thresh=0.02,
        )

        task_space_traj = np.array([[0.5, 0, 0.5], [0, 0.5, 0.5], [-0.5, 0, 0.5]])

        await self._test_lula_c_space_traj_gen(
            usd_path,
            robot_name,
            robot_prim_path,
            ee_frame,
            task_space_traj,
            orientation_target,
            distance_thresh=0.02,
        )

        timestamps = np.array([0.0, 3, 6.0])
        await self._test_lula_c_space_traj_gen(
            usd_path,
            robot_name,
            robot_prim_path,
            ee_frame,
            task_space_traj,
            orientation_target,
            timestamps=timestamps,
            interp_type="linear",
            distance_thresh=0.02,
        )

    async def test_lula_c_space_traj_gen_cobotta(self) -> None:
        """Test Lula C-space trajectory generation with the Cobotta Pro 900 robot.

        Tests trajectory generation with and without specified timestamps to verify
        C-space trajectory functionality on the Denso Cobotta robot.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"

        ee_frame = "gripper_center"

        task_space_traj = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6]])

        orientation_target = rotvecs_to_quats(np.array([np.pi, 0, 0]))

        await self._test_lula_c_space_traj_gen(
            usd_path, robot_name, robot_prim_path, ee_frame, task_space_traj, orientation_target
        )

        timestamps = np.array([0.0, 3.0, 6.0])
        await self._test_lula_c_space_traj_gen(
            usd_path, robot_name, robot_prim_path, ee_frame, task_space_traj, orientation_target, timestamps=timestamps
        )

    async def _test_lula_c_space_traj_gen(
        self,
        usd_path: str,
        robot_name: str,
        robot_prim_path: str,
        ee_frame: str,
        task_space_targets: object,
        orientation_target: object,
        timestamps: object = None,
        interp_type: str = "cubic_spline",
        distance_thresh: float = 0.01,
    ) -> None:
        """Test Lula C-space trajectory generation for a specified robot configuration.

        Loads the robot, computes inverse kinematics for task space targets, generates
        a C-space trajectory, and executes it while verifying the robot reaches all targets
        within the specified distance threshold.

        Args:
            usd_path: Path to the robot USD file.
            robot_name: Name of the robot for configuration lookup.
            robot_prim_path: USD prim path where the robot will be placed.
            ee_frame: Name of the end effector frame.
            task_space_targets: Array of target positions in task space.
            orientation_target: Target orientation for the end effector.
            timestamps: Time points for trajectory waypoints.
            interp_type: Interpolation type for trajectory generation.
            distance_thresh: Maximum allowed distance error for target reaching.
        """
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)

        for i, target_pos in enumerate(task_space_targets):
            VisualCuboid(f"/targets/target_{i}", position=target_pos, size=0.05)

        self._robot = Robot(robot_prim_path)

        await self._prepare_stage(self._robot)

        iks = []
        ik = None
        for target_pos in task_space_targets:
            ik, succ = self._kinematics_solver.compute_inverse_kinematics(
                ee_frame, target_pos, target_orientation=orientation_target, warm_start=ik
            )
            if not succ:
                carb.log_error(f"Could not compute ik for given task_space position {target_pos}")
            iks.append(ik)
        iks = np.array(iks)

        self._trajectory_generator = LulaCSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        self._art_kinematics = ArticulationKinematicsSolver(self._robot, self._kinematics_solver, ee_frame)

        if timestamps is None:
            trajectory = self._trajectory_generator.compute_c_space_trajectory(iks)
        else:
            trajectory = self._trajectory_generator.compute_timestamped_c_space_trajectory(iks, timestamps, interp_type)
            for i in range(len(timestamps)):
                self.assertTrue(
                    np.all(np.isclose(trajectory.get_joint_targets(timestamps[i])[0] - iks[i], np.zeros(iks.shape[1]))),
                    trajectory.get_joint_targets(timestamps[i])[0] - iks[i],
                )

        self.assertFalse(trajectory is None)

        self._art_trajectory = ArticulationTrajectory(self._robot, trajectory, self._physics_dt)

        art_traj = self._art_trajectory.get_action_sequence()

        initial_positions = np.zeros(self._robot.num_dof)
        initial_positions[art_traj[0].joint_indices] = art_traj[0].joint_positions

        self._robot.set_joint_positions(initial_positions)
        self._robot.set_joint_velocities(np.zeros_like(initial_positions))
        await update_stage_async()

        target_dists = np.ones(len(task_space_targets))
        for action in art_traj:
            self._robot.apply_action(action)
            await update_stage_async()

            robot_pos = self._art_kinematics.compute_end_effector_pose()[0]

            diff = np.linalg.norm(task_space_targets - robot_pos, axis=1)
            mask = target_dists > diff
            target_dists[mask] = diff[mask]

        delete_prim("/targets")

        self.assertTrue(
            np.all(target_dists < distance_thresh),
            f"Did not hit every task_space target: Distance to targets = {target_dists}",
        )

    async def test_set_c_space_trajectory_solver_config_settings(self) -> None:
        """Test setting C-space trajectory solver configuration parameters.

        Verifies that various solver parameters including position limits, velocity limits,
        acceleration limits, jerk limits, and solver-specific settings can be properly
        configured for the Franka robot.
        """
        robot_name = "Franka"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)

        self._trajectory_generator = LulaCSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        self._trajectory_generator.set_c_space_position_limits(
            *self._trajectory_generator.get_c_space_position_limits()
        )
        self._trajectory_generator.set_c_space_velocity_limits(self._trajectory_generator.get_c_space_velocity_limits())
        self._trajectory_generator.set_c_space_acceleration_limits(
            self._trajectory_generator.get_c_space_acceleration_limits()
        )
        self._trajectory_generator.set_c_space_jerk_limits(self._trajectory_generator.get_c_space_jerk_limits())

        self._trajectory_generator.set_solver_param("max_segment_iterations", 10)
        self._trajectory_generator.set_solver_param("max_aggregate_iterations", 10)
        self._trajectory_generator.set_solver_param("convergence_dt", 0.5)
        self._trajectory_generator.set_solver_param("max_dilation_iterations", 5)
        self._trajectory_generator.set_solver_param("min_time_span", 0.5)
        self._trajectory_generator.set_solver_param("time_split_method", "uniform")
        self._trajectory_generator.set_solver_param("time_split_method", "chord_length")
        self._trajectory_generator.set_solver_param("time_split_method", "centripetal")

    async def test_lula_task_space_traj_gen_franka(self) -> None:
        """Test Lula task space trajectory generation with the Franka robot.

        Tests trajectory generation from multiple position and orientation targets
        to verify task space trajectory functionality on the Franka Panda robot.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"

        ee_frame = "panda_hand"

        pos_targets = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6], [0, 0, 0.7]])
        orient_targets = np.tile(rotvecs_to_quats(np.array([np.pi, 0, 0])), (len(pos_targets), 1))

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets
        )

    async def test_lula_task_space_traj_gen_cobotta_900(self) -> None:
        """Tests Lula task space trajectory generation for the Cobotta Pro 900 robot.

        Generates and validates rectangular and circular trajectory paths with rotations.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"
        ee_frame = "gripper_center"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

        path, pos_targets, orient_targets = await self._build_circle_path_with_rotations()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_cobotta_1300(self) -> None:
        """Tests Lula task space trajectory generation for the Cobotta Pro 1300 robot.

        Generates and validates a rectangular trajectory path.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Denso/CobottaPro1300/cobotta_pro_1300.usd"
        robot_name = "Cobotta_Pro_1300"
        robot_prim_path = "/cobotta_pro_1300"
        ee_frame = "gripper_center"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_crx10ial(self) -> None:
        """Tests Lula task space trajectory generation for the Fanuc CRX10IAL robot.

        Generates and validates rectangular and circular trajectory paths with rotations.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Fanuc/crx10ia_l/crx10ia_l.usd"
        robot_name = "Fanuc_CRX10IAL"
        robot_prim_path = "/crx10ia_l"
        ee_frame = "tool0"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

        path, pos_targets, orient_targets = await self._build_circle_path_with_rotations()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_fr3(self) -> None:
        """Tests Lula task space trajectory generation for the Franka FR3 robot.

        Generates and validates rectangular and circular trajectory paths with rotations.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
        robot_name = "FR3"
        robot_prim_path = "/fr3"
        ee_frame = "fr3_hand_tcp"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

        path, pos_targets, orient_targets = await self._build_circle_path_with_rotations()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_tm12(self) -> None:
        """Tests Lula task space trajectory generation for the Techman TM12 robot.

        Generates and validates a rectangular trajectory path with an offset position.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Techman/TM12/tm12.usd"
        robot_name = "Techman_TM12"
        robot_prim_path = "/tm12"
        ee_frame = "tool0"

        path, pos_targets, orient_targets = await self._build_rect_path(offset=np.array([0.3, 0, 0]))

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_ur10(self) -> None:
        """Test Lula task space trajectory generation with the UR10 robot.

        Tests both composite path specifications and circular paths with rotations
        to verify advanced task space trajectory generation capabilities on the UR10 robot.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"
        ee_frame = "ee_link"

        path, pos_targets, orient_targets = await self._build_rect_path(offset=np.array([0.0, 0, 0]))

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def _build_rect_path(
        self, rot_vec: np.ndarray = np.array([np.pi, 0, 0]), offset: np.ndarray = np.array([0, 0, 0])
    ) -> tuple:
        """Builds a rectangular trajectory path in task space.

        Creates a lula task space path specification for a rectangular trajectory with specified orientation and position offset.

        Args:
            rot_vec: Rotation vector defining the end effector orientation.
            offset: Position offset to apply to the rectangular path.

        Returns:
            A tuple containing the lula task space path specification, position targets, and orientation targets.
        """
        rect_path = np.array([[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]])
        rect_path += offset

        builder = lula.create_task_space_path_spec(
            lula.Pose3(lula.Rotation3(np.linalg.norm(rot_vec), rot_vec / np.linalg.norm(rot_vec)), rect_path[0])
        )

        builder.add_translation(rect_path[1])

        builder.add_translation(rect_path[2])

        builder.add_translation(rect_path[3])

        builder.add_translation(rect_path[4])

        path = builder

        position_targets = rect_path
        orientation_targets = rotvecs_to_quats(np.tile(rot_vec, (len(position_targets), 1)))

        return path, position_targets, orientation_targets

    async def _build_circle_path_with_rotations(self, offset: np.ndarray = np.array([0, 0, 0])) -> tuple:
        """Builds a circular trajectory path with rotations in task space.

        Creates a lula task space path specification with two three-point arcs forming a circular path and includes a final rotation.

        Args:
            offset: Position offset to apply to the circular path.

        Returns:
            A tuple containing the lula task space path specification, position targets, and orientation targets.
        """
        builder = lula.create_task_space_path_spec(
            lula.Pose3(lula.Rotation3(np.pi, np.array([1, 0, 0])), np.array([0.3, 0.2, 0.3]))
        )

        builder.add_three_point_arc(np.array([0.3, -0.2, 0.3]), np.array([0.3, 0, 0.6]), True)
        builder.add_three_point_arc(np.array([0.3, 0.2, 0.3]), np.array([0.3, 0, 0]), True)

        builder.add_rotation(lula.Rotation3(np.pi / 2, np.array([1, 0, 0])))

        position_targets = (
            np.array([[0.3, 0.2, 0.3], [0.3, 0, 0.6], [0.3, -0.2, 0.3], [0.3, 0, 0], [0.3, 0.2, 0.3], [0.3, 0.2, 0.3]])
            + offset
        )

        orientation_targets = rotvecs_to_quats(np.tile(np.array([np.pi, 0, 0]), (len(position_targets), 1)))
        orientation_targets[-1] = rotvecs_to_quats(np.array([np.pi / 2, 0, 0]))

        return builder, position_targets, orientation_targets

    async def _test_lula_task_space_trajectory_generator(
        self,
        usd_path: str,
        robot_name: str,
        robot_prim_path: str,
        ee_frame: str,
        task_space_targets: np.ndarray,
        orientation_targets: np.ndarray,
        built_path: object = None,
        distance_thresh: float = 0.01,
    ) -> None:
        """Tests the Lula task space trajectory generator for a specified robot configuration.

        Loads a robot, generates a task space trajectory either from target points or a pre-built path specification, and
        validates that the robot follows the trajectory within the specified distance threshold.

        Args:
            usd_path: Path to the robot USD file.
            robot_name: Name of the robot for loading configuration.
            robot_prim_path: USD prim path where the robot will be placed.
            ee_frame: End effector frame name for trajectory generation.
            task_space_targets: Target positions in 3D space.
            orientation_targets: Target orientations as quaternions.
            built_path: Pre-built lula path specification. If None, generates trajectory from target points.
            distance_thresh: Maximum allowed distance between robot position and targets for validation.
        """
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)
        self._trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        for i, target_pos in enumerate(task_space_targets):
            VisualCuboid(f"/targets/target_{i}", position=target_pos, size=0.05)

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        if built_path is None:
            trajectory = self._trajectory_generator.compute_task_space_trajectory_from_points(
                task_space_targets, orientation_targets, ee_frame
            )
            self.assertTrue(trajectory is not None, "Failed to generate trajectory")
        else:
            trajectory = self._trajectory_generator.compute_task_space_trajectory_from_path_spec(built_path, ee_frame)
            self.assertTrue(trajectory is not None, "Failed to generate trajectory")

        self._art_kinematics = ArticulationKinematicsSolver(self._robot, self._kinematics_solver, ee_frame)
        self._art_trajectory = ArticulationTrajectory(self._robot, trajectory, self._physics_dt)

        art_traj = self._art_trajectory.get_action_sequence()

        initial_positions = np.zeros(self._robot.num_dof)
        initial_positions[art_traj[0].joint_indices] = art_traj[0].joint_positions

        self._robot.set_joint_positions(initial_positions)
        self._robot.set_joint_velocities(np.zeros_like(initial_positions))
        await update_stage_async()

        target_dists = np.ones(len(task_space_targets))
        for action in art_traj:
            self._robot.apply_action(action)
            await update_stage_async()

            robot_pos, robot_orient = self._art_kinematics.compute_end_effector_pose()
            pos_diff = np.linalg.norm(task_space_targets - robot_pos, axis=1)
            orient_diff = np.linalg.norm(orientation_targets - rot_matrices_to_quats(robot_orient), axis=1)
            diff = pos_diff + orient_diff
            mask = target_dists > diff
            target_dists[mask] = diff[mask]

        delete_prim("/targets")
        self.assertTrue(
            np.all(target_dists < distance_thresh),
            f"Did not hit every task_space target: Distance to targets = {target_dists}",
        )

    async def test_set_task_space_trajectory_solver_config_settings(self) -> None:
        """Tests setting various solver configuration parameters for the Lula task space trajectory generator.

        Validates that all solver parameters can be set without errors, including C-space limits, solver parameters, and
        path conversion configuration settings.
        """
        robot_name = "Franka"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)

        self._trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        self._trajectory_generator.set_c_space_position_limits(
            *self._trajectory_generator.get_c_space_position_limits()
        )
        self._trajectory_generator.set_c_space_velocity_limits(self._trajectory_generator.get_c_space_velocity_limits())
        self._trajectory_generator.set_c_space_acceleration_limits(
            self._trajectory_generator.get_c_space_acceleration_limits()
        )
        self._trajectory_generator.set_c_space_jerk_limits(self._trajectory_generator.get_c_space_jerk_limits())

        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_segment_iterations", 10)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_aggregate_iterations", 10)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("convergence_dt", 0.5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_dilation_iterations", 5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("min_time_span", 0.5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "uniform")
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "chord_length")
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "centripetal")

        conversion_config = self._trajectory_generator.get_path_conversion_config()
        conversion_config.alpha = 1.3
        conversion_config.initial_s_step_size = 0.04
        conversion_config.initial_s_step_size_delta = 0.003
        conversion_config.max_iterations = 40
        conversion_config.max_position_deviation = 0.002
        conversion_config.min_position_deviation = 0.0015
        conversion_config.min_s_step_size = 1e-4
        conversion_config.min_s_step_size_delta = 1e-4
