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

"""Tests for the motion policy functionality in the isaacsim.robot_motion.motion_generation extension."""

from __future__ import annotations

import asyncio
import json
import os

import isaacsim.core.api.objects as objects
import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
import numpy as np
import omni.client
import omni.kit.test
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils import distance_metrics
from isaacsim.core.utils.prims import delete_prim, is_prim_path_valid
from isaacsim.core.utils.rotations import gf_quat_to_np_array, quat_to_rot_matrix
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    open_stage_async,
    update_stage_async,
)

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.lula.motion_policies import RmpFlow
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdLux


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestMotionPolicy(omni.kit.test.AsyncTestCase):
    """Test class for motion generation functionality in the isaacsim.robot_motion.motion_generation extension.

    This class provides comprehensive test coverage for robot motion policies, including RmpFlow motion generation,
    obstacle avoidance, and articulation control. Tests are performed on various robot models such as Franka Panda,
    Universal Robots (UR3, UR5, UR10, etc.), Denso Cobotta, and other supported robot platforms.

    The test suite validates:
    - Configuration space and task space target reaching
    - Obstacle detection and avoidance capabilities
    - Motion policy initialization and lifecycle management
    - Visualization features for collision spheres and end effectors
    - State update handling and robot convergence behavior
    - Multi-robot platform compatibility

    Each test method sets up a specific robot configuration, creates the necessary motion policies, and verifies
    that the robot can successfully navigate to target positions while avoiding obstacles.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment before each test.

        Initializes physics settings, timeline interface, extension paths, policy configuration directory,
        and creates a new USD stage for testing motion policies.
        """
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_motion.motion_generation")
        self._articulation_policy_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._articulation_policy_extension_path, "motion_policy_configs")
        self.assertTrue(os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")))
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        await create_new_stage_async()

        await update_stage_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up the test environment after each test.

        Stops the timeline, waits for assets to finish loading, clears the articulation policy,
        and resets the World instance.
        """
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._articulation_policy = None
        await update_stage_async()
        World.clear_instance()

    async def _create_light(self) -> None:
        """Create a sphere light in the scene for visualization.

        Adds a sphere light at position [6.5, 0, 12] with radius 2 and intensity 100000
        to provide illumination for the test scene.
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(sphereLight.GetPath().pathString).set_world_pose([6.5, 0, 12])

    async def _prepare_stage(self, robot: object) -> None:
        """Prepare the simulation stage with the specified robot.

        Initializes the world simulation context, creates lighting, starts the timeline,
        and configures the robot with disabled gravity and increased solver iterations.

        Args:
            robot: The robot to initialize and configure for testing.
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

        self._robot.post_reset()
        await update_stage_async()

    async def test_rmpflow_cspace_target(self) -> None:
        """Test RMPflow motion policy with configuration space targets on a Franka Panda robot.

        Verifies that the robot can reach default and custom cspace targets, and that
        end effector targets take precedence over cspace targets when both are set.
        """
        asset_root_path = await get_assets_root_path_async()

        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        default_target = self._motion_policy.get_default_cspace_position_target()
        active_joints_subset = self._articulation_policy.get_active_joints_subset()

        # Can reach just a cspace target
        for i in range(180):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            if np.allclose(default_target, active_joints_subset.get_joint_positions(), atol=0.1):
                break

        self.assertTrue(
            np.allclose(default_target, active_joints_subset.get_joint_positions(), atol=0.1),
            f"{default_target} vs {active_joints_subset.get_joint_positions()}: Could not reach default cspace target in 300 frames!",
        )

        ee_target_position = np.array([0.5, 0, 0.5])
        self._motion_policy.set_end_effector_target(ee_target_position)

        new_target = np.array([1.0, 0, 1.0, -0.3, 0, 0.2, 0])
        self._motion_policy.set_cspace_target(new_target)

        # Check cspace attractor doesn't override the ee target
        for i in range(180):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            ee_pose = self._motion_policy.get_end_effector_pose(active_joints_subset.get_joint_positions())[0]
            if np.linalg.norm(ee_target_position - ee_pose) < 0.01:
                break

        ee_pose = self._motion_policy.get_end_effector_pose(active_joints_subset.get_joint_positions())[0]
        self.assertTrue(
            np.linalg.norm(ee_target_position - ee_pose) < 0.01,
            f"Could not reach taskspace target target in 240 frames! {np.linalg.norm(ee_target_position - ee_pose)}",
        )

        self._motion_policy.set_end_effector_target(None)

        # New cspace target is still active; check that robot reaches it
        for i in range(250):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            if np.allclose(new_target, active_joints_subset.get_joint_positions(), atol=0.1):
                break

        self.assertTrue(
            np.allclose(new_target, active_joints_subset.get_joint_positions(), atol=0.1),
            f"Could not reach new cspace target in 250 frames! {new_target} != {active_joints_subset.get_joint_positions()}",
        )

        self.assertTrue(
            np.allclose(self._motion_policy.get_default_cspace_position_target(), default_target),
            f"{self._motion_policy.get_default_cspace_position_target()} != {default_target}",
        )

    async def test_rmpflow_cobotta_900(self) -> None:
        """Test RMPflow motion policy on a Cobotta Pro 900 robot.

        Loads the Denso Cobotta Pro 900 robot and verifies that the RMPflow motion policy
        can successfully navigate to target positions while avoiding obstacles.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_cobotta_1300(self) -> None:
        """Test RMPflow motion policy on a Cobotta Pro 1300 robot.

        Loads the Denso Cobotta Pro 1300 robot and verifies that the RMPflow motion policy
        can successfully navigate to target positions while avoiding obstacles.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Denso/CobottaPro1300/cobotta_pro_1300.usd"
        robot_name = "Cobotta_Pro_1300"
        robot_prim_path = "/cobotta_pro_1300"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur3(self) -> None:
        """Test RMPflow motion policy on a UR3 robot.

        Loads the Universal Robots UR3 robot and verifies that the RMPflow motion policy
        can successfully navigate to target positions while avoiding obstacles.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur3/ur3.usd"
        robot_name = "UR3"
        robot_prim_path = "/ur3/root_joint"

        await self._simple_robot_rmpflow_test(
            usd_path, robot_prim_path, robot_name, target_pos=np.array([0.3, 0.3, 0.5])
        )

    async def test_rmpflow_ur3e(self) -> None:
        """Test RMPflow motion policy on a UR3e robot.

        Loads the Universal Robots UR3e robot and verifies that the RMPflow motion policy
        can successfully navigate to target positions while avoiding obstacles.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"
        robot_name = "UR3e"
        robot_prim_path = "/ur3e/root_joint"

        await self._simple_robot_rmpflow_test(
            usd_path, robot_prim_path, robot_name, target_pos=np.array([0.3, 0.3, 0.5])
        )

    async def test_rmpflow_ur5(self) -> None:
        """Test RMPflow motion policy on a UR5 robot.

        Loads the Universal Robots UR5 robot and verifies that the RMPflow motion policy
        can successfully navigate to target positions while avoiding obstacles.
        """
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
        robot_name = "UR5"
        robot_prim_path = "/ur5/root_joint"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur5e(self) -> None:
        """Tests RMPflow motion policy with the Universal Robots UR5e robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        robot_name = "UR5e"
        robot_prim_path = "/ur5e/root_joint"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur10(self) -> None:
        """Tests RMPflow motion policy with the Universal Robots UR10 robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10/root_joint"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur10e(self) -> None:
        """Tests RMPflow motion policy with the Universal Robots UR10e robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        robot_name = "UR10e"
        robot_prim_path = "/ur10e/root_joint"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur16e(self) -> None:
        """Tests RMPflow motion policy with the Universal Robots UR16e robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"
        robot_name = "UR16e"
        robot_prim_path = "/ur16e/root_joint"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rizon4(self) -> None:
        """Tests RMPflow motion policy with the Flexiv Rizon4 robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"
        robot_name = "Rizon4"
        robot_prim_path = "/A02L_MP"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.78, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_rs007l(self) -> None:
        """Tests RMPflow motion policy with the Kawasaki RS007L robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS007L/rs007l_onrobot_rg2.usd"
        robot_name = "RS007L"
        robot_prim_path = "/khi_rs007l"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rs007n(self) -> None:
        """Tests RMPflow motion policy with the Kawasaki RS007N robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS007N/rs007n_onrobot_rg2.usd"
        robot_name = "RS007N"
        robot_prim_path = "/khi_rs007n"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rs013n(self) -> None:
        """Tests RMPflow motion policy with the Kawasaki RS013N robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS013N/rs013n_onrobot_rg2.usd"
        robot_name = "RS013N"
        robot_prim_path = "/khi_rs013n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_rs025n(self) -> None:
        """Tests RMPflow motion policy with the Kawasaki RS025N robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS025N/rs025n_onrobot_rg2.usd"
        robot_name = "RS025N"
        robot_prim_path = "/khi_rs025n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_rs080n(self) -> None:
        """Tests RMPflow motion policy with the Kawasaki RS080N robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd"
        robot_name = "RS080N"
        robot_prim_path = "/khi_rs080n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_tm12(self) -> None:
        """Test RMPflow motion policy with Techman TM12 robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Techman/TM12/tm12.usd"
        robot_name = "Techman_TM12"
        robot_prim_path = "/tm12"

        obstacle_position = np.array([0.8, 0.25, 0.8])
        target_position = np.array([0.78, 0.4, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_kr210(self) -> None:
        """Test RMPflow motion policy with Kuka KR210 robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd"
        robot_name = "Kuka_KR210"
        robot_prim_path = "/kuka_kr210"

        obstacle_position = np.array([1.5, 1.25, 1.8])
        target_position = np.array([1.78, 1.4, 1.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_crx10ial(self) -> None:
        """Test RMPflow motion policy with Fanuc CRX10IAL robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/Fanuc/crx10ia_l/crx10ia_l.usd"
        robot_name = "Fanuc_CRX10IAL"
        robot_prim_path = "/crx10ia_l"

        obstacle_position = np.array([0.7, 0.25, 0.7])
        target_position = np.array([0.78, 0.4, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_fr3(self) -> None:
        """Test RMPflow motion policy with Franka FR3 robot."""
        assets_root_path = await get_assets_root_path_async()
        usd_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"
        robot_name = "FR3"
        robot_prim_path = "/fr3"

        obstacle_position = np.array([0.7, 0.25, 0.7])
        target_position = np.array([0.78, 0.4, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def _simple_robot_rmpflow_test(
        self,
        usd_path: str,
        prim_path: str,
        robot_name: str,
        target_pos: object = np.array([0.6, 0.3, 0.5]),
        obstacle_pos: object = np.array([0.3, 0.1, 0.5]),
    ) -> None:
        """Helper method to test RMPflow motion policy on a robot.

        Args:
            usd_path: Path to the robot USD file.
            prim_path: Prim path for the robot in the stage.
            robot_name: Name of the robot configuration.
            target_pos: Target position for the end effector.
            obstacle_pos: Position of the obstacle.
        """
        result, error = await open_stage_async(usd_path)

        rmp_config = interface_config_loader.load_supported_motion_policy_config(robot_name, "RMPflow")
        self._motion_policy = RmpFlow(**rmp_config)

        robot_prim_path = prim_path

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        timeout = 20

        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

    async def test_rmpflow_visualization_franka(self) -> None:
        """Test RMPflow visualization features with Franka Panda robot."""
        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        robot_prim_path = "/panda"

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        self._motion_policy.set_end_effector_target(np.array([0.4, 0.2, 0.4]))

        self._motion_policy.visualize_collision_spheres()
        self._motion_policy.visualize_end_effector_position()

        test_sphere = self._motion_policy.get_collision_spheres_as_prims()[16]
        test_ee_visual = self._motion_policy.get_end_effector_as_prim()

        panda_hand_prim = SingleXFormPrim("/panda/panda_hand")

        self._articulation_policy.move()

        for i in range(100):
            sphere_pos, _ = test_sphere.get_world_pose()
            ee_pos, _ = test_ee_visual.get_world_pose()

            hand_pose, _ = panda_hand_prim.get_world_pose()
            self.assertTrue(
                abs(np.linalg.norm(sphere_pos - ee_pos) - 0.0672) < 0.001,
                f"End effector visualization is not consistent with sphere visualization: {np.linalg.norm(sphere_pos - ee_pos)}",
            )
            self.assertTrue(
                abs(np.linalg.norm(hand_pose - ee_pos) - 0.10) < 0.01,
                f"Simulated robot moved too far from RMP belief robot: {np.linalg.norm(hand_pose - ee_pos)}",
            )

            self._motion_policy.update_world()
            self._articulation_policy.move()
            await update_stage_async()

        self._motion_policy.delete_collision_sphere_prims()
        self._motion_policy.delete_end_effector_prim()
        self.assertTrue(not is_prim_path_valid("/lula/end_effector"))
        self.assertTrue(not is_prim_path_valid("/lula/collision_sphere0"))

        self._motion_policy.set_end_effector_target(np.array([0.8, 0.2, 0.8]))

        test_sphere = self._motion_policy.get_collision_spheres_as_prims()[16]
        test_ee_visual = self._motion_policy.get_end_effector_as_prim()

        # self._articulation_policy.move()
        await update_stage_async()

        for _ in range(100):
            sphere_pos, _ = test_sphere.get_world_pose()
            ee_pos, _ = test_ee_visual.get_world_pose()

            hand_pose, _ = panda_hand_prim.get_world_pose()
            self.assertTrue(
                abs(np.linalg.norm(sphere_pos - ee_pos) - 0.0672) < 0.001,
                f"End effector visualization is not consistent with sphere visualization: {np.linalg.norm(sphere_pos - ee_pos) }",
            )
            self.assertTrue(
                abs(np.linalg.norm(hand_pose - ee_pos) - 0.10) < 0.01,
                f"Simulated robot moved too far from RMP belief robot: {np.linalg.norm(hand_pose - ee_pos)}",
            )

            self._motion_policy.update_world()
            self._articulation_policy.move()
            await update_stage_async()

        self._motion_policy.reset()
        self.assertTrue(not is_prim_path_valid("/lula/end_effector"))
        self.assertTrue(not is_prim_path_valid("/lula/collision_sphere0"))

    async def test_rmpflow_obstacle_adders(self) -> None:
        """Test obstacle management functions in RMPflow with various obstacle types."""
        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        # These obstacle types are supported by RmpFlow
        obstacles = [
            objects.cuboid.VisualCuboid("/visual_cube"),
            objects.cuboid.DynamicCuboid("/dynamic_cube"),
            objects.cuboid.FixedCuboid("/fixed_cube"),
            objects.sphere.VisualSphere("/visual_sphere"),
            objects.sphere.DynamicSphere("/dynamic_sphere"),
            objects.capsule.VisualCapsule("/visual_capsule"),
            objects.capsule.DynamicCapsule("/dynamic_capsule"),
            objects.ground_plane.GroundPlane("/ground_plane"),
        ]

        # check that all the supported world update functions return successfully without error
        for obstacle in obstacles:
            self.assertTrue(self._motion_policy.add_obstacle(obstacle))
            self.assertTrue(self._motion_policy.disable_obstacle(obstacle))
            self.assertTrue(self._motion_policy.enable_obstacle(obstacle))
            self.assertTrue(self._motion_policy.remove_obstacle(obstacle))

        # make sure lula cleaned up after removing ground plane : Lula creates a wide, flat cuboid to mimic the ground because it doesn't support ground planes directly
        self.assertFalse(is_prim_path_valid("/lula/ground_plane"))

        for obstacle in obstacles:
            self.assertTrue(self._motion_policy.add_obstacle(obstacle))
        for obstacle in obstacles:
            # obstacle already in there
            self.assertFalse(self._motion_policy.add_obstacle(obstacle))
        self._motion_policy.reset()
        for obstacle in obstacles:
            # obstacles should have been deleted in reset
            self.assertFalse(self._motion_policy.disable_obstacle(obstacle))
            self.assertFalse(self._motion_policy.enable_obstacle(obstacle))
            self.assertFalse(self._motion_policy.remove_obstacle(obstacle))

        self.assertFalse(is_prim_path_valid("/lula/ground_plane"))

    async def test_articulation_motion_policy_init_order(self) -> None:
        """Test initialization order of ArticulationMotionPolicy with robot."""
        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)

        # Make sure that initializing this before robot is initialized doesn't cause any issues
        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        self._timeline.play()
        await update_stage_async()

        await self._prepare_stage(self._robot)

        self._articulation_policy.get_next_articulation_action()

    async def test_rmpflow_on_franka(self) -> None:
        """Test RMPflow motion policy with Franka Panda robot including state updates."""
        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(False)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._robot.post_reset()

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        ground_truths = {
            "no_target": np.array(
                [
                    -4.4170367e-03,
                    -2.7524245e-01,
                    9.3539513e-04,
                    3.2967355e-02,
                    1.8063301e-04,
                    -4.3320316e-01,
                    4.4973856e-03,
                ]
            ),
            "target_no_obstacle": np.array(
                [0.22091845, -0.27475226, 0.20515294, 0.01469292, -0.03139969, -0.43752316, 0.00518845]
            ),
            "target_with_obstacle": np.array(
                [-0.01676518, -0.2309316, -0.21077305, -0.06896219, -0.15911254, -0.1659573, -0.00489121]
            ),
            "target_pos": np.array([0.40, 0.20, 0.40]),
            "obs_pos": np.array([0.3, 0.20, 0.50]),
        }
        await self.verify_policy_outputs(self._robot, ground_truths, dbg=False)

        timeout = 10

        await self._prepare_stage(self._robot)

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

    async def test_rmpflow_on_franka_ignore_state(self) -> None:
        """Test RMPflow motion policy with Franka Panda robot while ignoring state updates."""
        # Perform an internal rollout of robot state, ignoring simulated robot state updates

        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        """
        verify_policy_outputs() is not used here because
            1: The policy would not pass because it rolls out robot state internally rather than seeing
                that the robot is not moving, so the outputs become inconsistent.
            2: It is sufficient to confirm that the world state is updated correctly in
                test_rmpflow_on_franka_velocity_control().
        """
        await self._prepare_stage(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

    async def test_rmpflow_static_obstacles_franka(self) -> None:
        """Test RmpFlow motion policy on the Franka robot with static obstacles.

        Performs robot motion testing using RmpFlow with internal robot state rollout.
        Tests convergence to target positions in various robot base poses with
        static obstacle avoidance.
        """
        # Perform an internal rollout of robot state, ignoring simulated robot state updates

        asset_root_path = await get_assets_root_path_async()
        usd_path = asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        robot_prim_path = "/panda"

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos, static=True
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

    async def test_rmpflow_on_ur10(self) -> None:
        """Test RmpFlow motion policy on the UR10 robot.

        Verifies motion policy outputs against ground truth values and tests robot
        convergence to target positions from various base poses and orientations.
        """
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_prim_path = "/ur10"
        add_reference_to_stage(usd_path, robot_prim_path)
        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(False)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        ground_truths = {
            "no_target": np.array([-0.07558637, -0.035313368, -0.14294432, -0.24767338, 0.25070193, 2.879336e-10]),
            "target_no_obstacle": np.array(
                [-0.43079016, 0.18957902, 0.33274212, 0.46673688, -0.36309126, 6.501429e-10]
            ),
            "target_with_obstacle": np.array([-0.41449845, 0.11129826, 0.37269664, 0.4741332, -0.36905077, 0.0]),
            "target_pos": np.array([0.5, 0.0, 0.0]),
            "obs_pos": np.array([0.50, 0.0, -0.20]),
        }
        await self.verify_policy_outputs(self._robot, ground_truths, dbg=False)

        await self._prepare_stage(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.7])
        obstacle_pos = np.array([0.8, 0.1, 0.8])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.7, 0]))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.2, 0.0, 1.0), 90).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

    async def test_rmpflow_on_ur10_ignore_state(self) -> None:
        """Test RmpFlow motion policy on the UR10 robot with state updates ignored.

        Performs internal rollout of robot state while ignoring simulated robot state updates.
        Tests robot convergence to target positions from various base poses and orientations.
        """
        # Perform an internal rollout of robot state, ignoring simulated robot state updates
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_prim_path = "/ur10"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)
        await self._prepare_stage(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        """
        verify_policy_outputs() is not used here because
            1: The policy would not pass because it rolls out robot state internally rather than seeing
                that the robot is not moving, so the outputs become inconsistent.
            2: It is sufficient to confirm that the world state is updated correctly in
                test_rmpflow_on_franka_velocity_control().
        """
        await self._prepare_stage(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.7])
        obstacle_pos = np.array([0.8, 0.1, 0.8])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.7, 0]))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.2, 0.0, 1.0), 90).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

    async def reached_end_effector_target(
        self, target_trans: object, target_orient: object, trans_thresh: float = 0.02, rot_thresh: float = 0.1
    ) -> bool:
        """Check if the end effector has reached the target pose.

        Args:
            target_trans: Target translation position.
            target_orient: Target orientation quaternion.
            trans_thresh: Translation distance threshold.
            rot_thresh: Rotation distance threshold.

        Returns:
            True if the end effector is within the specified thresholds of the target pose.
        """
        ee_trans, ee_rot = self._motion_policy.get_end_effector_pose(
            self._articulation_policy.get_active_joints_subset().get_joint_positions()
        )  # TODO this only works for RMPflow, and will be updated in upcoming MR before there are non-RMPflow tests

        if target_orient is not None:
            target_rot = quat_to_rot_matrix(target_orient)
        else:
            target_rot = None

        if target_rot is None and target_trans is None:
            return True
        elif target_rot is None:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            return trans_dist < trans_thresh
        elif target_trans is None:
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return rot_dist < rot_thresh
        else:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return trans_dist < trans_thresh and rot_dist < rot_thresh

    async def add_block(
        self, path: str, offset: object, size: object = np.array([0.01, 0.01, 0.01]), collidable: bool = True
    ) -> object:
        """Add a block cuboid to the scene.

        Args:
            path: Prim path for the block.
            offset: Position offset for the block.
            size: Size array for the block dimensions.
            collidable: Whether the block should be collidable.

        Returns:
            The created cuboid object.
        """
        if collidable:
            cuboid = objects.cuboid.FixedCuboid(path, scale=size, size=1.0, position=offset)
            await update_stage_async()
        else:
            cuboid = objects.cuboid.VisualCuboid(path, scale=size, size=1.0, position=offset)

        return cuboid

    async def assertAlmostEqual(self, a: object, b: object, msg: str = "") -> None:  # noqa: N802
        """Assert that two arrays are almost equal within a tolerance.

        Args:
            a: First array to compare.
            b: Second array to compare.
            msg: Message to display on assertion failure.
        """
        # overriding method because it doesn't support iterables
        a = np.array(a)
        b = np.array(b)
        self.assertFalse(np.any(abs(a[a != np.array(None)] - b[b != np.array(None)]) > 1e-3), msg)

    async def simulate_until_target_reached(
        self, timeout: float, target_trans: object, target_orient: object = None
    ) -> tuple[bool, float]:
        """Simulate robot motion until the target is reached or timeout occurs.

        Args:
            timeout: Maximum simulation time in seconds.
            target_trans: Target translation position.
            target_orient: Target orientation quaternion.

        Returns:
            A tuple of (success, time_taken) where success indicates if the target was reached.
        """
        for frame in range(int(1 / self._physics_dt * timeout)):
            self._motion_policy.update_world()
            self._articulation_policy.move()
            await omni.kit.app.get_app().next_update_async()
            if await self.reached_end_effector_target(target_trans, target_orient=target_orient):
                return True, frame * self._physics_dt
        return False, timeout

    async def verify_policy_outputs(self, robot: object, ground_truths: dict, dbg: bool = False) -> None:
        """Verify motion policy outputs against ground truth values.

        The ground truths are obtained by running this method in dbg mode
        when certain that motion_generation is working as intended.

        If position_control is True, motion_generation is expected to be using position targets

        In dbg mode, the returned velocity target values will be printed
        and no assertions will be checked.

        Args:
            robot: The robot instance to test.
            ground_truths: Dictionary containing ground truth values for comparison.
            dbg: Enable debug mode to print outputs without assertions.

        Returns:
            None.
        """
        # outputs of mg in different scenarios
        no_target_truth = ground_truths["no_target"]
        target_no_obs_truth = ground_truths["target_no_obstacle"]
        target_obs_truth = ground_truths["target_with_obstacle"]

        # where to put the target and obstacle
        target_pos = ground_truths["target_pos"]
        obs_pos = ground_truths["obs_pos"]

        target_cube = await self.add_block("/scene/target", target_pos, size=0.05 * np.ones(3), collidable=False)

        await update_stage_async()

        obs = await self.add_block("/scene/obstacle", obs_pos, size=0.1 * np.ones(3))

        await update_stage_async()

        await self._prepare_stage(robot)
        await update_stage_async()

        self._motion_policy.set_end_effector_target(None)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nNo target:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                no_target_truth, mg_velocity_targets, f"{no_target_truth} != {mg_velocity_targets}"
            )

        # Just the target
        self._motion_policy.set_end_effector_target(target_pos)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        # Add the obstacle
        self._motion_policy.add_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_obs_truth, mg_velocity_targets, f"{target_obs_truth} != {mg_velocity_targets}"
            )

        # Disable the obstacle: check that it matches no obstacle at all
        self._motion_policy.disable_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and disabled obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        # Enable the obstacle: check consistency
        self._motion_policy.enable_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and enabled obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_obs_truth, mg_velocity_targets, f"{target_obs_truth} != {mg_velocity_targets}"
            )

        # Delete the obstacle: check consistency
        self._motion_policy.remove_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and deleted obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        delete_prim(obs.prim_path)
        delete_prim(target_cube.prim_path)
        return

    async def verify_robot_convergence(
        self,
        target_pos: object,
        timeout: float,
        target_orient: object = None,
        obs_pos: object = None,
        static: bool = False,
    ) -> None:
        """Assert that the robot can reach the target within a given timeout.

        Args:
            target_pos: Target position for the robot end effector.
            timeout: Maximum time allowed for convergence in seconds.
            target_orient: Target orientation quaternion.
            obs_pos: Position for obstacle placement.
            static: Whether obstacles should be treated as static.

        Returns:
            None.
        """
        # Assert that the robot can reach the target within a given timeout

        await self.add_block("/scene/target", target_pos, size=0.05 * np.ones(3), collidable=False)
        self._motion_policy.set_robot_base_pose(*self._robot.get_world_pose())

        await update_stage_async()
        obs_prim = None
        if obs_pos is not None:
            cuboid = await self.add_block("/scene/obstacle", obs_pos, size=0.1 * np.array([2.0, 3.0, 1.0]))
            self._motion_policy.add_obstacle(cuboid, static=static)

        self._motion_policy.set_end_effector_target(target_pos, target_orient)

        success, time_to_target = await self.simulate_until_target_reached(
            timeout, target_pos, target_orient=target_orient
        )
        if not success:
            ee_trans, ee_rot = self._motion_policy.get_end_effector_pose(
                self._articulation_policy.get_active_joints_subset().get_joint_positions()
            )
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_pos)
            msg = (
                f"Robot failed to converge within {timeout}s. "
                f"Final EE position: {ee_trans}, target: {target_pos}, "
                f"translational distance: {trans_dist:.6f} (threshold: 0.02)"
            )
            if target_orient is not None:
                target_rot = quat_to_rot_matrix(target_orient)
                rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
                msg += f", rotational distance: {rot_dist:.6f} (threshold: 0.1)"
            self.fail(msg)

        if obs_prim is not None:
            self._motion_policy.remove_obstacle(cuboid)

        return
