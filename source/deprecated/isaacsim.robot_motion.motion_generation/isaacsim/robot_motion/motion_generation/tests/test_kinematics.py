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

"""Test suite for validating kinematics solvers in the isaacsim.robot_motion.motion_generation extension."""

from __future__ import annotations

import asyncio
import json
import os

import carb

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
import numpy as np
import omni.kit.test
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils import distance_metrics
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdLux


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestKinematics(omni.kit.test.AsyncTestCase):
    """Test suite for validating kinematics solvers in the isaacsim.robot_motion.motion_generation extension.

    This test class validates forward kinematics (FK) and inverse kinematics (IK) functionality for supported
    robot configurations including UR10 and Franka robots. It compares Lula kinematics solver results against
    USD robot frame transformations to ensure accuracy and consistency.

    The test suite covers:
    - Forward kinematics validation by comparing Lula solver results with USD robot frame poses
    - Inverse kinematics validation by verifying convergence to target positions and orientations
    - Property getter/setter functionality for kinematics solver configuration
    - Cross-validation between different kinematics computation methods

    Tests are performed with various robot configurations, base poses, and target configurations to ensure
    robust validation across different scenarios. The validation includes both translational and rotational
    accuracy checks with configurable tolerance thresholds.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment before each test.

        Initializes physics settings, timeline interface, extension manager, and loads policy configurations.
        """
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_motion.motion_generation")
        self._mg_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._mg_extension_path, "motion_policy_configs")
        self.assertTrue(os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")))
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment after each test.

        Stops timeline, waits for asset loading to complete, clears motion generation instance, and clears world instance.
        """
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._mg = None
        await update_stage_async()
        World.clear_instance()

    async def _create_light(self) -> None:
        """Create a sphere light in the scene.

        Adds a sphere light with radius 2 and intensity 100000 at position [6.5, 0, 12].
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath().pathString)).set_world_pose([6.5, 0, 12])

    async def _prepare_stage(self, robot: object) -> None:
        """Prepare the stage for testing with the given robot.

        Stops timeline, initializes world and simulation context, creates lighting, starts timeline,
        and configures robot with disabled gravity and solver iteration counts.

        Args:
            robot: The robot instance to initialize and configure.
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
        robot.set_solver_position_iteration_count(64)
        robot.set_solver_velocity_iteration_count(64)

        await update_stage_async()

    async def test_lula_fk_ur10(self) -> None:
        """Test forward kinematics for UR10 robot.

        Loads UR10 robot, performs forward kinematics test with specific joint targets,
        and verifies translational distance is less than 0.001 and rotational distance is less than 0.005.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"
        robot_root_path = "/ur10/root_joint"
        trans_dist, rot_dist = await self._test_lula_fk(
            usd_path,
            robot_name,
            robot_prim_path,
            robot_root_path,
            joint_target=-np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.2]),
        )
        self.assertTrue(np.all(trans_dist < 0.001))
        self.assertTrue(np.all(rot_dist < 0.005))

    async def test_lula_fk_franka(self) -> None:
        """Test forward kinematics for Franka robot.

        Loads Franka Panda robot, performs forward kinematics test with specific base pose and orientation,
        and verifies accuracy excluding known issues with finger frames and frame 0.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"
        trans_dist, rot_dist = await self._test_lula_fk(
            usd_path,
            robot_name,
            robot_prim_path,
            base_pose=np.array([0.10, 0, 1.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )
        # There is a known bug with the kinematics not matching on the Franka finger frames
        # and whatever is in frame 0
        self.assertTrue(np.all(trans_dist[1:-2] < 0.005), trans_dist)
        # first entry error appears here too.
        self.assertTrue(np.all(rot_dist[1:] < 0.005), rot_dist)

    async def _test_lula_fk(
        self,
        usd_path: str,
        robot_name: str,
        robot_prim_path: str,
        robot_root_path: str = None,
        joint_target: object = None,
        base_pose: object = np.zeros(3),
        base_orient: object = np.array([1, 0, 0, 0]),
    ) -> tuple:
        """Test forward kinematics by comparing Lula solver results with USD frame poses.

        Loads robot from USD file, initializes kinematics solver, moves robot to target position,
        and compares frame positions and orientations between Lula solver and USD representation.

        Args:
            usd_path: Path to the USD file containing the robot.
            robot_name: Name of the robot for loading kinematics configuration.
            robot_prim_path: USD prim path where the robot is referenced.
            robot_root_path: USD prim path to the robot root joint.
            joint_target: Target joint positions for the robot.
            base_pose: Base position of the robot in world coordinates.
            base_orient: Base orientation of the robot as a quaternion.

        Returns:
            A tuple containing (translational_distances, rotational_distances) arrays comparing
            Lula solver results with USD frame poses.
        """
        await create_new_stage_async()
        add_reference_to_stage(usd_path, robot_prim_path)

        omni.usd.get_context().get_stage().SetTimeCodesPerSecond(self._physics_fps)
        set_camera_view(eye=[3.5, 2.3, 2.1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        if robot_root_path is None:
            robot_root_path = robot_prim_path

        self._robot = Robot(robot_root_path)
        await self._prepare_stage(self._robot)
        self._robot.set_world_pose(base_pose, base_orient)

        self._kinematics.set_robot_base_pose(base_pose, base_orient)

        if joint_target is not None:
            self._robot.get_articulation_controller().apply_action(ArticulationAction(joint_target))

        # move towards target or default position
        await self.move_until_still(self._robot)

        frame_names = self._kinematics.get_all_frame_names()

        art_fk = ArticulationKinematicsSolver(self._robot, self._kinematics, frame_names[0])

        trans_dists = []
        rot_dist = []

        # save the distance between lula and usd frames for each frame that exists for both robot views
        for frame in frame_names:
            if is_prim_path_valid(robot_prim_path + "/" + frame):
                art_fk.set_end_effector_frame(frame)

                lula_frame_pos, lula_frame_rot = art_fk.compute_end_effector_pose()
                usd_frame_pos, usd_frame_rot = SingleXFormPrim(robot_prim_path + "/" + frame).get_world_pose()

                trans_dists.append(distance_metrics.weighted_translational_distance(lula_frame_pos, usd_frame_pos))
                rot_dist.append(
                    distance_metrics.rotational_distance_angle(lula_frame_rot, quats_to_rot_matrices(usd_frame_rot))
                )

        return np.array(trans_dists), np.array(rot_dist)

    async def test_lula_ik_ur10(self) -> None:
        """Test inverse kinematics for UR10 robot.

        Loads UR10 robot and performs inverse kinematics tests with position-only and
        position-orientation targets to verify solver convergence and accuracy.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"
        frame = "ee_link"

        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.40, 0.80]),
            None,
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.40, 0.80]),
            np.array([0.6, 0, 0, -1]),
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

    async def test_lula_ik_franka(self) -> None:
        """Test inverse kinematics for Franka robot.

        Loads Franka Panda robot and performs inverse kinematics tests on different end-effector frames
        with various target poses to verify solver convergence and accuracy.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"
        frame = "right_gripper"
        # await self._test_lula_ik(usd_path,robot_name,robot_prim_path,frame,np.array([40,30,60]),np.array([.1,0,0,-1]),1,.1)
        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.30, 0.60]),
            np.array([0.1, 0, 0, -1]),
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

        frame = "panda_hand"
        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.30, 0.60]),
            None,
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

    async def _test_lula_ik(
        self,
        usd_path: str,
        robot_name: str,
        robot_prim_path: str,
        frame: str,
        position_target: object,
        orientation_target: object,
        position_tolerance: float,
        orientation_tolerance: float,
        base_pose: object = np.zeros(3),
        base_orient: object = np.array([0, 0, 0, 1]),
    ) -> None:
        """Test inverse kinematics by solving for target pose and verifying solution accuracy.

        Loads robot, initializes kinematics solver, computes inverse kinematics solution for target pose,
        and verifies consistency between IK solution, forward kinematics, and USD robot frame poses.

        Args:
            usd_path: Path to the USD file containing the robot.
            robot_name: Name of the robot for loading kinematics configuration.
            robot_prim_path: USD prim path where the robot is referenced.
            frame: Name of the end-effector frame to solve for.
            position_target: Target position in world coordinates.
            orientation_target: Target orientation as a quaternion.
            position_tolerance: Tolerance for position accuracy.
            orientation_tolerance: Tolerance for orientation accuracy.
            base_pose: Base position of the robot in world coordinates.
            base_orient: Base orientation of the robot as a quaternion.
        """
        await create_new_stage_async()
        add_reference_to_stage(usd_path, robot_prim_path)
        omni.usd.get_context().get_stage().SetTimeCodesPerSecond(self._physics_fps)
        set_camera_view(eye=[3.5, 2.3, 2.1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._robot.set_world_pose(base_pose, base_orient)
        self._kinematics.set_robot_base_pose(base_pose, base_orient)

        art_ik = ArticulationKinematicsSolver(self._robot, self._kinematics, frame)

        # testing IK and ArticulationKinematicsSolver object wrapping IK
        alg_ik_action, success = art_ik.compute_inverse_kinematics(
            position_target, orientation_target, position_tolerance, orientation_tolerance
        )
        alg_ik, _ = self._kinematics.compute_inverse_kinematics(
            frame, position_target, orientation_target, None, position_tolerance, orientation_tolerance
        )
        self.assertTrue(success, "IK Solver did not converge to a solution")

        # check if USD robot can get to IK result
        self._robot.get_articulation_controller().apply_action(alg_ik_action)
        await self.move_until_still(self._robot)

        # check IK consistent with FK
        lula_pos, lula_rot = self._kinematics.compute_forward_kinematics(frame, joint_positions=alg_ik)
        self.assertTrue(
            distance_metrics.weighted_translational_distance(lula_pos, position_target) < position_tolerance
        )

        if orientation_target is not None:
            tgt_rot = quats_to_rot_matrices(orientation_target)
            rot_dist = distance_metrics.rotational_distance_angle(lula_rot, tgt_rot)
            self.assertTrue(rot_dist < orientation_tolerance, "Rotational distance too large: " + str(rot_dist))

        # check IK consistent with USD robot frames
        if is_prim_path_valid(robot_prim_path + "/" + frame):
            usd_pos, usd_rot = SingleXFormPrim(robot_prim_path + "/" + frame).get_world_pose()
            trans_dist = distance_metrics.weighted_translational_distance(usd_pos, position_target)
            self.assertTrue(trans_dist < position_tolerance, str(usd_pos) + str(position_target))
            if orientation_target is not None:
                rot_dist = distance_metrics.rotational_distance_angle(quats_to_rot_matrices(usd_rot), tgt_rot)
                self.assertTrue(rot_dist < orientation_tolerance)

        else:
            carb.log_warn("Frame " + frame + " does not exist on USD robot")

    async def test_lula_ik_properties(self) -> None:
        """Test property assignment and retrieval for LulaKinematicsSolver inverse kinematics configuration.

        Verifies that BFGS and CCD algorithm parameters, sampling settings, and tolerance values
        can be correctly assigned and retrieved from the kinematics solver.
        """
        robot_name = "UR10"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        lk = LulaKinematicsSolver(**kinematics_config)

        import lula

        lk.bfgs_cspace_limit_biasing = lula.CyclicCoordDescentIkConfig.CSpaceLimitBiasing.DISABLE
        self.assertTrue(lk.bfgs_cspace_limit_biasing == lula.CyclicCoordDescentIkConfig.CSpaceLimitBiasing.DISABLE)

        lk.bfgs_cspace_limit_biasing_weight = 0.1
        self.assertTrue(lk.bfgs_cspace_limit_biasing_weight == 0.1)

        lk.bfgs_cspace_limit_penalty_region = 0.1
        self.assertTrue(lk.bfgs_cspace_limit_penalty_region == 0.1)

        lk.bfgs_gradient_norm_termination = False
        self.assertFalse(lk.bfgs_gradient_norm_termination)

        lk.bfgs_gradient_norm_termination_coarse_scale_factor = 2.0
        self.assertTrue(lk.bfgs_gradient_norm_termination_coarse_scale_factor == 2.0)

        lk.bfgs_max_iterations = 101
        self.assertTrue(lk.bfgs_max_iterations == 101)

        lk.bfgs_orientation_weight = 0.5
        self.assertTrue(lk.bfgs_orientation_weight == 0.5)

        lk.bfgs_position_weight = 0.5
        self.assertTrue(lk.bfgs_position_weight == 0.5)

        lk.ccd_bracket_search_num_uniform_samples = 13
        self.assertTrue(lk.ccd_bracket_search_num_uniform_samples == 13)

        lk.ccd_descent_termination_delta = 0.01
        self.assertTrue(lk.ccd_descent_termination_delta == 0.01)

        lk.ccd_max_iterations = 15
        self.assertTrue(lk.ccd_max_iterations == 15)

        lk.ccd_orientation_weight = 0.3
        self.assertTrue(lk.ccd_orientation_weight == 0.3)

        lk.ccd_position_weight = 0.8
        self.assertTrue(lk.ccd_position_weight == 0.8)

        lk.cspace_seeds = []
        self.assertTrue(lk.cspace_seeds == [])

        lk.irwin_hall_sampling_order = 4
        self.assertTrue(lk.irwin_hall_sampling_order == 4)

        lk.max_num_descents = 51
        self.assertTrue(lk.max_num_descents == 51)

        lk.orientation_tolerance = 0.1
        self.assertTrue(lk.orientation_tolerance == 0.1)

        lk.position_tolerance = 0.1
        self.assertTrue(lk.position_tolerance == 0.1)

        lk.sampling_seed = 16
        self.assertTrue(lk.sampling_seed == 16)

    async def test_getters_and_setters(self) -> None:
        """Test getter and setter methods of LulaKinematicsSolver.

        Verifies that tolerance and cspace seed values can be set and retrieved correctly.
        Also validates that configuration limits loaded from Robot Description files match expected values
        for UR10 and Franka robots.
        """
        await create_new_stage_async()

        robot_name = "UR10"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        lk = LulaKinematicsSolver(**kinematics_config)

        lk.set_default_orientation_tolerance(0.1)
        self.assertTrue(lk.get_default_orientation_tolerance() == 0.1)

        lk.set_default_position_tolerance(0.2)
        self.assertTrue(lk.get_default_position_tolerance() == 0.2)

        lk.set_default_cspace_seeds(np.array([1, 2, 3, 4]))
        self.assertTrue(np.all(lk.get_default_cspace_seeds() == np.array([1, 2, 3, 4])))

        # Assert that getters for information loaded from Robot Description files matches expected values.

        pos_lim = lk.get_cspace_position_limits()
        self.assertTrue(np.allclose(pos_lim[0], [-6.2831, -6.2831, -3.1415, -6.2831, -6.2831, -6.2831], 0.0005))
        self.assertTrue(np.allclose(pos_lim[1], [6.2831, 6.2831, 3.1415, 6.2831, 6.2831, 6.2831], 0.0001))

        vel_lim = lk.get_cspace_velocity_limits()
        self.assertTrue(
            np.allclose(vel_lim, [2.0943951, 2.0943951, 3.1415927, 3.1415927, 3.1415927, 3.1415927], 0.0001)
        )

        self.assertTrue(np.all(lk.get_cspace_acceleration_limits() == [40.0] * 6))
        self.assertTrue(np.all(lk.get_cspace_jerk_limits() == [10000.0] * 6))

        # Test Franka because it has acceleration and jerk limits specified in Robot Description
        robot_name = "Franka"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        lk = LulaKinematicsSolver(**kinematics_config)

        accel_lim = lk.get_cspace_acceleration_limits()
        self.assertTrue(np.allclose(accel_lim, [15, 7.5, 10, 12.5, 15, 20, 20], 0.0001))

        jerk_lim = lk.get_cspace_jerk_limits()
        self.assertTrue(np.allclose(jerk_lim, [7500, 3750, 5000, 6250, 7500, 10000, 10000], 0.0001))

    async def move_until_still(self, robot: object, timeout: int = 500) -> int:
        """Move the robot until it reaches a stable position.

        Waits for the robot to stop moving by monitoring joint position stability across multiple frames.
        Returns early if the robot becomes still before the timeout.

        Args:
            robot: The robot to monitor for stability.
            timeout: Maximum number of frames to wait for stability.

        Returns:
            Number of frames elapsed when stability was reached, or timeout value if stability was not achieved.
        """
        h = 10
        positions = np.zeros((h, robot.num_dof))
        for i in range(timeout):
            positions[i % h] = robot.get_joint_positions()
            await update_stage_async()
            if i > h:
                std = np.std(positions, axis=0)
                if np.all(std < 0.001):
                    return i
        return timeout
