# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Validate Newton tensor actuation through the high-level Articulation API.

The tests drive a 12-DOF MuJoCo Go2 articulation and verify that SDK effort
commands, position targets, and PD gains cross the Newton C++ tensor bridge
into MuJoCo ``CTRL_DIRECT`` actuators. Effort-only checks disable gravity where
needed so joint velocity assertions measure commanded torque rather than leg
gravity loads.
"""

import asyncio
from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, UsdPhysics


def _set_robot_height(stage: Any, prim_path: str, height: float) -> None:
    """Set the robot's root prim z-translate without adding constraints.

    Args:
        stage: USD stage containing the robot.
        prim_path: Path to the robot root prim.
        height: Z translation to write on the robot root.
    """
    from pxr import Gf

    robot_prim = stage.GetPrimAtPath(prim_path)
    if not robot_prim.IsValid():
        return
    xformable = UsdGeom.Xformable(robot_prim)
    ops = xformable.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(0.0, 0.0, height))
            return
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height))


def _fix_robot_base(stage: Any, prim_path: str, robot_height: float = 1.0) -> None:
    """Fix the robot base to the world frame using a FixedJoint under the articulation.

    Args:
        stage: USD stage containing the robot.
        prim_path: Path to the robot root prim.
        robot_height: Initial z-position of the robot root [m].
    """
    from pxr import Usd

    robot_prim = stage.GetPrimAtPath(prim_path)
    if not robot_prim.IsValid():
        return

    base_body_path = None
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            base_body_path = str(prim.GetPath())
            break

    if base_body_path is None:
        return

    fixed_joint_path = prim_path + "/FixedBaseJoint"
    fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
    fixed_joint.CreateBody0Rel().SetTargets([])
    fixed_joint.CreateBody1Rel().SetTargets([base_body_path])

    _set_robot_height(stage, prim_path, robot_height)


def _set_mujoco_variant(stage: Any, prim_path: str) -> None:
    """Set the Physics variant to mujoco on the robot reference.

    Args:
        stage: USD stage containing the robot.
        prim_path: Path to the robot root prim.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    variant_sets = prim.GetVariantSets()
    if "Physics" not in variant_sets.GetNames():
        return
    variant_set = variant_sets.GetVariantSet("Physics")
    for available in variant_set.GetVariantNames():
        if available.lower() == "mujoco":
            variant_set.SetVariantSelection(available)
            return


class TestNewtonActuatorBridge(omni.kit.test.AsyncTestCase):
    """Verify that the Newton actuator bridge produces motion through the SDK Articulation."""

    NUM_DOFS = 12
    FREESTANDING_HEIGHT = 0.32
    FREESTANDING_MIN_BASE_Z = 0.2
    FREESTANDING_JOINT_TOL = 0.3
    FREESTANDING_STEPS = 30

    STAND_TARGETS_BY_NAME = {
        "FL_hip_joint": 0.0,
        "FL_thigh_joint": 0.67,
        "FL_calf_joint": -1.3,
        "FR_hip_joint": 0.0,
        "FR_thigh_joint": 0.67,
        "FR_calf_joint": -1.3,
        "RL_hip_joint": 0.0,
        "RL_thigh_joint": 0.67,
        "RL_calf_joint": -1.3,
        "RR_hip_joint": 0.0,
        "RR_thigh_joint": 0.67,
        "RR_calf_joint": -1.3,
    }
    STAND_KP = 60.0
    STAND_KD = 5.0

    def _get_stand_targets(self, robot: Any) -> np.ndarray:
        """Build standing target array matching the robot's DOF ordering.

        Args:
            robot: Articulation view with DOF names.

        Returns:
            Standing target positions ordered by robot DOF index.
        """
        targets = np.zeros(robot.num_dofs, dtype=np.float32)
        for i, name in enumerate(robot.dof_names):
            if name in self.STAND_TARGETS_BY_NAME:
                targets[i] = self.STAND_TARGETS_BY_NAME[name]
        return targets

    async def setUp(self) -> None:
        """Create a Newton CPU scene with ground and a MuJoCo-variant Go2 robot."""
        await stage_utils.create_new_stage_async()
        self._physics_rate = 200
        self._physics_dt = 1.0 / self._physics_rate

        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.set_physics_dt(self._physics_dt)

        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

        assets_root = get_assets_root_path()
        stage_utils.add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        self._prim_path = "/World/Robot"

        stage_utils.add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Samples/Mujoco_Menagerie/unitree_go2/go2/go2.usda",
            path=self._prim_path,
        )
        _set_mujoco_variant(self._stage, self._prim_path)

        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop the timeline and wait for pending USD loading before the next test."""
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def _create_robot(self, fixed_base: bool = True, robot_height: float = 0.1) -> Articulation:
        """Start simulation and create articulation view.

        Args:
            fixed_base: if True, attach the robot base to the world with a FixedJoint.
            robot_height: initial z-position of the robot root [m].

        Returns:
            Articulation view for the robot prim.
        """
        if fixed_base:
            _fix_robot_base(self._stage, self._prim_path, robot_height)
        else:
            _set_robot_height(self._stage, self._prim_path, robot_height)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        robot = Articulation(paths=self._prim_path)
        await omni.kit.app.get_app().next_update_async()
        return robot

    async def _step(self, n: int = 1) -> Any:
        for _ in range(n):
            await omni.kit.app.get_app().next_update_async()

    def _reset_state(self, robot: Any, positions: np.ndarray | None = None) -> None:
        """Reset DOF positions and zero velocities.

        Args:
            robot: Articulation view.
            positions: 1-D positions to write (length ``num_dofs``). If
                ``None``, positions are zeroed. Note that for the Go2 USD
                ``positions=0`` is outside the calf joint limits and produces
                a constraint force; tests that need a clean baseline should
                pass the standing pose instead.
        """
        num_dofs = robot.num_dofs
        if positions is None:
            pos = np.zeros((1, num_dofs), dtype=np.float32)
        else:
            pos = positions.reshape(1, -1).astype(np.float32)
        zero = np.zeros((1, num_dofs), dtype=np.float32)
        robot.set_dof_positions(wp.from_numpy(pos, dtype=wp.float32))
        robot.set_dof_velocities(wp.from_numpy(zero, dtype=wp.float32))

    def _disable_gravity(self) -> None:
        """Set the physics scene gravity magnitude to zero.

        Required for actuation-bridge tests that compare velocity responses
        to applied efforts: with kp=0 and kd=0 (effort mode), gravity-induced
        joint torques otherwise dominate small applied efforts on the legs.
        """
        scene = UsdPhysics.Scene.Get(self._stage, "/World/PhysicsScene")
        if scene:
            scene.CreateGravityMagnitudeAttr().Set(0.0)

    def _get_base_height(self, robot: Any) -> float:
        """Get the z-position of the robot's root link.

        Args:
            robot: Articulation view.

        Returns:
            Root link z-position [m].
        """
        positions, _ = robot.get_world_poses()
        return float(positions.numpy().flatten()[2])

    def _compute_pd_efforts(self, robot: Any, targets: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> np.ndarray:
        """Compute PD control efforts: F = Kp * (target - pos) - Kd * vel.

        Args:
            robot: Articulation view.
            targets: Target DOF positions.
            kp: Per-DOF proportional gains.
            kd: Per-DOF derivative gains.

        Returns:
            Effort command array with shape ``(1, num_dofs)``.
        """
        pos = robot.get_dof_positions().numpy().flatten()
        vel = robot.get_dof_velocities().numpy().flatten()
        efforts = kp * (targets - pos) - kd * vel
        return efforts.reshape(1, -1).astype(np.float32)

    async def test_effort_pd_converges_to_stand(self) -> None:
        """Manual PD efforts (via set_dof_efforts) drive the robot to standing pose."""
        robot = await self._create_robot()
        self.assertEqual(robot.num_dofs, self.NUM_DOFS)
        await self._step(5)

        kp = np.full(self.NUM_DOFS, 20.0, dtype=np.float32)
        kd = np.full(self.NUM_DOFS, 2.0, dtype=np.float32)

        robot.switch_dof_control_mode("effort")
        await self._step(2)

        self._reset_state(robot)
        await self._step(1)

        stand_targets = self._get_stand_targets(robot)

        for _ in range(40):
            efforts = self._compute_pd_efforts(robot, stand_targets, kp, kd)
            robot.set_dof_efforts(wp.from_numpy(efforts, dtype=wp.float32))
            await self._step(1)

        pos_final = robot.get_dof_positions().numpy().flatten()
        tolerance = 0.1
        for i in range(self.NUM_DOFS):
            self.assertLess(
                abs(pos_final[i] - stand_targets[i]),
                tolerance,
                f"DOF {i} should converge to standing target via PD efforts. "
                f"Expected {stand_targets[i]:.3f}, got {pos_final[i]:.3f}",
            )

    async def test_effort_pd_converges_to_stand_freestanding(self) -> None:
        """Manual PD efforts drive the free-standing robot to stand on its legs."""
        robot = await self._create_robot(fixed_base=False, robot_height=self.FREESTANDING_HEIGHT)
        self.assertEqual(robot.num_dofs, self.NUM_DOFS)
        await self._step(5)

        kp = np.full(self.NUM_DOFS, 60.0, dtype=np.float32)
        kd = np.full(self.NUM_DOFS, 2.0, dtype=np.float32)

        robot.switch_dof_control_mode("effort")
        await self._step(2)

        stand_targets = self._get_stand_targets(robot)

        for _ in range(self.FREESTANDING_STEPS):
            efforts = self._compute_pd_efforts(robot, stand_targets, kp, kd)
            robot.set_dof_efforts(wp.from_numpy(efforts, dtype=wp.float32))
            await self._step(1)

        base_z = self._get_base_height(robot)
        self.assertGreater(
            base_z,
            self.FREESTANDING_MIN_BASE_Z,
            f"Robot should remain upright (base z > {self.FREESTANDING_MIN_BASE_Z}m). Got z={base_z:.4f}m.",
        )

        pos_final = robot.get_dof_positions().numpy().flatten()
        for i in range(self.NUM_DOFS):
            self.assertLess(
                abs(pos_final[i] - stand_targets[i]),
                self.FREESTANDING_JOINT_TOL,
                f"DOF {i} should be within {self.FREESTANDING_JOINT_TOL} rad of standing target "
                f"(freestanding). Expected {stand_targets[i]:.3f}, got {pos_final[i]:.3f}",
            )

    async def test_effort_produces_velocity(self) -> None:
        """Applying constant effort should produce nonzero velocity on all DOFs."""
        self._disable_gravity()
        robot = await self._create_robot()
        self.assertEqual(robot.num_dofs, self.NUM_DOFS)
        await self._step(5)

        robot.switch_dof_control_mode("effort")
        await self._step(2)

        self._reset_state(robot, positions=self._get_stand_targets(robot))
        await self._step(1)

        efforts = np.full((1, robot.num_dofs), 5.0, dtype=np.float32)
        for _ in range(10):
            robot.set_dof_efforts(wp.from_numpy(efforts, dtype=wp.float32))
            await self._step(1)

        vel = robot.get_dof_velocities().numpy().flatten()
        for i in range(robot.num_dofs):
            self.assertGreater(
                abs(vel[i]),
                0.01,
                f"DOF {i}: effort should produce nonzero velocity, got {vel[i]:.4f}",
            )

    async def test_opposite_efforts_produce_opposite_velocities(self) -> None:
        """Reversing effort sign should reverse the velocity direction on most DOFs."""
        self._disable_gravity()
        robot = await self._create_robot()
        await self._step(5)

        robot.switch_dof_control_mode("effort")
        await self._step(2)

        stand = self._get_stand_targets(robot)
        self._reset_state(robot, positions=stand)
        await self._step(1)
        efforts_pos = np.full((1, robot.num_dofs), 5.0, dtype=np.float32)
        for _ in range(10):
            robot.set_dof_efforts(wp.from_numpy(efforts_pos, dtype=wp.float32))
            await self._step(1)
        vel_pos = robot.get_dof_velocities().numpy().flatten().copy()

        self._reset_state(robot, positions=stand)
        await self._step(1)
        efforts_neg = np.full((1, robot.num_dofs), -5.0, dtype=np.float32)
        for _ in range(10):
            robot.set_dof_efforts(wp.from_numpy(efforts_neg, dtype=wp.float32))
            await self._step(1)
        vel_neg = robot.get_dof_velocities().numpy().flatten().copy()

        for i in range(robot.num_dofs):
            self.assertGreater(
                abs(vel_pos[i]),
                0.01,
                f"DOF {i}: positive effort should produce nonzero velocity, got {vel_pos[i]:.4f}",
            )
            self.assertGreater(
                abs(vel_neg[i]),
                0.01,
                f"DOF {i}: negative effort should produce nonzero velocity, got {vel_neg[i]:.4f}",
            )
            self.assertLess(
                vel_pos[i] * vel_neg[i],
                0.0,
                f"DOF {i}: opposite efforts should produce opposite velocities. "
                f"pos_vel={vel_pos[i]:.4f}, neg_vel={vel_neg[i]:.4f}",
            )

    async def test_larger_effort_produces_larger_velocity(self) -> None:
        """Larger effort magnitude should produce larger velocity magnitude."""
        self._disable_gravity()
        robot = await self._create_robot()
        await self._step(5)

        robot.switch_dof_control_mode("effort")
        await self._step(2)

        stand = self._get_stand_targets(robot)
        steps = 10

        self._reset_state(robot, positions=stand)
        await self._step(1)
        small_effort = np.full((1, robot.num_dofs), 2.0, dtype=np.float32)
        for _ in range(steps):
            robot.set_dof_efforts(wp.from_numpy(small_effort, dtype=wp.float32))
            await self._step(1)
        vel_small = np.abs(robot.get_dof_velocities().numpy().flatten()).copy()

        self._reset_state(robot, positions=stand)
        await self._step(1)
        large_effort = np.full((1, robot.num_dofs), 10.0, dtype=np.float32)
        for _ in range(steps):
            robot.set_dof_efforts(wp.from_numpy(large_effort, dtype=wp.float32))
            await self._step(1)
        vel_large = np.abs(robot.get_dof_velocities().numpy().flatten()).copy()

        for i in range(robot.num_dofs):
            self.assertGreater(
                vel_large[i],
                vel_small[i],
                f"DOF {i}: larger effort should produce larger velocity. "
                f"small={vel_small[i]:.4f}, large={vel_large[i]:.4f}",
            )

    async def test_position_target_moves_joints(self) -> None:
        """Setting standing targets with SDK gains should move all DOFs from zero."""
        robot = await self._create_robot()
        await self._step(5)

        num_dofs = robot.num_dofs
        stand_targets = self._get_stand_targets(robot)
        stiffness = np.full((1, num_dofs), self.STAND_KP, dtype=np.float32)
        damping = np.full((1, num_dofs), self.STAND_KD, dtype=np.float32)
        robot.set_dof_gains(stiffness, damping)
        await self._step(2)

        self._reset_state(robot)
        await self._step(1)
        pos_before = robot.get_dof_positions().numpy().flatten().copy()

        targets = stand_targets.reshape(1, -1)
        robot.set_dof_position_targets(wp.from_numpy(targets, dtype=wp.float32))

        await self._step(12)

        pos_after = robot.get_dof_positions().numpy().flatten()
        for i in range(num_dofs):
            if stand_targets[i] != 0.0:
                self.assertGreater(
                    abs(pos_after[i] - pos_before[i]),
                    0.02,
                    f"DOF {i} should move toward standing target {stand_targets[i]:.3f}. "
                    f"Before: {pos_before[i]:.4f}, After: {pos_after[i]:.4f}",
                )

    async def test_position_target_converges_to_stand(self) -> None:
        """With SDK gains and enough steps, DOFs should converge to the standing pose."""
        robot = await self._create_robot()
        await self._step(5)

        num_dofs = robot.num_dofs
        stand_targets = self._get_stand_targets(robot)

        stiffness = np.full((1, num_dofs), self.STAND_KP, dtype=np.float32)
        damping = np.full((1, num_dofs), self.STAND_KD, dtype=np.float32)
        robot.set_dof_gains(stiffness, damping)
        await self._step(2)

        self._reset_state(robot)
        await self._step(1)

        targets = stand_targets.reshape(1, -1)
        robot.set_dof_position_targets(wp.from_numpy(targets, dtype=wp.float32))

        await self._step(25)

        pos_final = robot.get_dof_positions().numpy().flatten()
        tolerance = 0.05
        for i in range(num_dofs):
            self.assertLess(
                abs(pos_final[i] - stand_targets[i]),
                tolerance,
                f"DOF {i} should converge within {tolerance} rad of standing target. "
                f"Expected {stand_targets[i]:.3f}, got {pos_final[i]:.3f}",
            )

    async def test_position_target_converges_to_stand_freestanding(self) -> None:
        """Position targets drive the free-standing robot to stand on its legs."""
        robot = await self._create_robot(fixed_base=False, robot_height=self.FREESTANDING_HEIGHT)
        self.assertEqual(robot.num_dofs, self.NUM_DOFS)
        await self._step(5)

        num_dofs = robot.num_dofs
        stand_targets = self._get_stand_targets(robot)

        stiffness = np.full((1, num_dofs), self.STAND_KP, dtype=np.float32)
        damping = np.full((1, num_dofs), self.STAND_KD, dtype=np.float32)
        robot.set_dof_gains(stiffness, damping)
        await self._step(2)

        targets = stand_targets.reshape(1, -1)
        robot.set_dof_position_targets(wp.from_numpy(targets, dtype=wp.float32))

        await self._step(self.FREESTANDING_STEPS)

        base_z = self._get_base_height(robot)
        self.assertGreater(
            base_z,
            self.FREESTANDING_MIN_BASE_Z,
            f"Robot should remain upright (base z > {self.FREESTANDING_MIN_BASE_Z}m). Got z={base_z:.4f}m.",
        )

        pos_final = robot.get_dof_positions().numpy().flatten()
        for i in range(num_dofs):
            self.assertLess(
                abs(pos_final[i] - stand_targets[i]),
                self.FREESTANDING_JOINT_TOL,
                f"DOF {i} should be within {self.FREESTANDING_JOINT_TOL} rad of standing target "
                f"(freestanding). Expected {stand_targets[i]:.3f}, got {pos_final[i]:.3f}",
            )

    async def test_higher_stiffness_closer_to_target(self) -> None:
        """Higher stiffness should bring DOFs closer to standing targets."""
        robot = await self._create_robot()
        await self._step(5)

        num_dofs = robot.num_dofs
        stand_targets = self._get_stand_targets(robot)
        targets = stand_targets.reshape(1, -1)
        targets_wp = wp.from_numpy(targets, dtype=wp.float32)
        damping = np.full((1, num_dofs), self.STAND_KD, dtype=np.float32)
        steps = 15

        self._reset_state(robot)
        await self._step(1)
        stiffness_low = np.full((1, num_dofs), 10.0, dtype=np.float32)
        robot.set_dof_gains(stiffness_low, damping)
        robot.set_dof_position_targets(targets_wp)
        await self._step(steps)
        pos_low = robot.get_dof_positions().numpy().flatten().copy()

        self._reset_state(robot)
        await self._step(1)
        stiffness_high = np.full((1, num_dofs), 200.0, dtype=np.float32)
        robot.set_dof_gains(stiffness_high, damping)
        robot.set_dof_position_targets(targets_wp)
        await self._step(steps)
        pos_high = robot.get_dof_positions().numpy().flatten().copy()

        mean_error_low = np.mean(np.abs(pos_low - stand_targets))
        mean_error_high = np.mean(np.abs(pos_high - stand_targets))

        self.assertLess(
            mean_error_high,
            mean_error_low,
            f"Higher stiffness should produce lower mean error to standing targets. "
            f"kp=10 mean error: {mean_error_low:.4f}, kp=200 mean error: {mean_error_high:.4f}",
        )

    async def test_gains_override_usd_authored_gains(self) -> None:
        """Setting gains via the API should override gains from the USD asset."""
        robot = await self._create_robot()
        await self._step(5)

        num_dofs = robot.num_dofs
        stand_targets = self._get_stand_targets(robot)

        stiffness = np.full((1, num_dofs), 500.0, dtype=np.float32)
        damping = np.full((1, num_dofs), 50.0, dtype=np.float32)
        robot.set_dof_gains(stiffness, damping)

        self._reset_state(robot)
        await self._step(1)

        targets = stand_targets.reshape(1, -1)
        robot.set_dof_position_targets(wp.from_numpy(targets, dtype=wp.float32))

        await self._step(30)

        pos_final = robot.get_dof_positions().numpy().flatten()
        mean_error = np.mean(np.abs(pos_final - stand_targets))
        self.assertLess(
            mean_error,
            0.05,
            f"With high gains (kp=500, kd=50), mean error to standing target should be < 0.05. "
            f"Got {mean_error:.4f}. Targets: {stand_targets}, Final: {pos_final}",
        )
