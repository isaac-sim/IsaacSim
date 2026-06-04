# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for `ArticulationActuators`."""

from __future__ import annotations

import math

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.actuators import ActuatorConfig, ArticulationActuators
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf, Usd

_SIMPLE_ART_REL_PATH = "Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
_ART_ROOT = "/World/A_0"
_REVOLUTE_JOINT_PATH = "/World/A_0/Arm/RevoluteJoint"
_PRISMATIC_JOINT_PATH = "/World/A_0/Slider/PrismaticJoint"

_FRANKA_REL_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
_FRANKA_BASE_PATH = "/World/Franka"  # /World/Franka_0, /World/Franka_1, ...
_FRANKA_REGEX = "/World/Franka_.*"
_FRANKA_ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
_NUM_FRANKAS = 2


class TestArticulationActuators(omni.kit.test.AsyncTestCase):
    """Tests for `ArticulationActuators`."""

    async def setUp(self) -> None:
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        usd_path = f"{get_assets_root_path()}/{_SIMPLE_ART_REL_PATH}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=_ART_ROOT)
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _author_pd_actuator(
        self,
        actuator_prim_path: str,
        target_joint_path: Sdf.Path,
        *,
        kp: float = 100.0,
        kd: float = 10.0,
    ) -> Usd.Prim:
        """Define a `NewtonActuator` prim with `NewtonPDControlAPI` targeting `target_joint_path`.

        Args:
            actuator_prim_path: USD path at which to define the `NewtonActuator` prim.
            target_joint_path: Absolute `Sdf.Path` of the joint to actuate.
            kp: Proportional gain.
            kd: Derivative gain.

        Returns:
            The newly defined prim.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.DefinePrim(actuator_prim_path, "NewtonActuator")
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(kp)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(kd)
        prim.CreateRelationship("newton:targets").SetTargets([target_joint_path])
        return prim

    def _author_pid_actuator(
        self,
        actuator_prim_path: str,
        target_joint_path: Sdf.Path,
        *,
        kp: float = 100.0,
        ki: float = 50.0,
        kd: float = 10.0,
    ) -> Usd.Prim:
        """Define a `NewtonActuator` prim with `NewtonPIDControlAPI` targeting `target_joint_path`.

        Args:
            actuator_prim_path: USD path at which to define the `NewtonActuator` prim.
            target_joint_path: Absolute `Sdf.Path` of the joint to actuate.
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.

        Returns:
            The newly defined prim.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.DefinePrim(actuator_prim_path, "NewtonActuator")
        prim.AddAppliedSchema("NewtonPIDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(kp)
        prim.CreateAttribute("newton:ki", Sdf.ValueTypeNames.Float).Set(ki)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(kd)
        prim.CreateRelationship("newton:targets").SetTargets([target_joint_path])
        return prim

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    async def test_zero_actuator_passthrough(self) -> None:
        """Construct `ArticulationActuators` on an articulation with no `NewtonActuator` prims.

        The class must discover zero actuators, expose empty `actuated_dof_indices`, and
        allow `close()` to be called without error. `step_actuators` must also be a no-op.
        """
        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertEqual(len(actuated.actuators), 0, "Expected zero actuators on a vanilla articulation")
            self.assertEqual(actuated.actuated_dof_indices, [], "Expected empty actuated_dof_indices")
            actuated.step_actuators(step_dt=0.01, context=None)
        finally:
            actuated.close()

    async def test_single_pd_actuator_discovery(self) -> None:
        """Author one `NewtonActuator` and confirm DOF index and actuator count are correct."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path)

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertEqual(len(actuated.actuators), 1)
            expected_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
            self.assertEqual(actuated.actuated_dof_indices, [expected_dof_index])
        finally:
            actuated.close()

    async def test_multiple_actuators_sorted_indices(self) -> None:
        """Author actuators for both DOFs; `actuated_dof_indices` must be sorted by DOF index."""
        revolute_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prismatic_path = Sdf.Path(_PRISMATIC_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/revolute_actuator", revolute_path)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/prismatic_actuator", prismatic_path)

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self.assertEqual(len(actuated.actuators), 2)
            dof_paths = actuated.articulation.dof_paths[0]
            expected = sorted([dof_paths.index(str(revolute_path)), dof_paths.index(str(prismatic_path))])
            self.assertEqual(actuated.actuated_dof_indices, expected)
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    async def test_construction_with_physics_already_running(self) -> None:
        """Verify that drive gains are zeroed synchronously when constructed while physics is already live.

        Exercises the eager `_on_physics_ready` call at the end of `__init__` when
        `SimulationManager._physics_sim_view__warp` is already set.
        """
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
            stiffness, damping = actuated.articulation.get_dof_gains(dof_indices=target_dof_index)
            self.assertEqual(stiffness.numpy().item(), 0.0, "Stiffness must be zeroed immediately at construction")
            self.assertEqual(damping.numpy().item(), 0.0, "Damping must be zeroed immediately at construction")
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Subscription lifecycle
    # ------------------------------------------------------------------

    async def test_physics_callback_registered_by_default(self) -> None:
        """Verify that the physics pre-step callback registers after `PHYSICS_READY` when `auto_step_pre_physics` is unset."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNotNone(actuated._actuator_callback_id)
        finally:
            actuated.close()

    async def test_physics_callback_registered_when_enabled_at_construction(self) -> None:
        """Verify that the physics callback registers after `PHYSICS_READY` when opted in at construction."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNotNone(actuated._actuator_callback_id)
        finally:
            actuated.close()

    async def test_enable_auto_step_pre_physics_after_physics_ready(self) -> None:
        """Verify that calling `enable_auto_step_pre_physics()` while physics is live registers immediately."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=False)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNone(actuated._actuator_callback_id)

            actuated.enable_auto_step_pre_physics()
            self.assertIsNotNone(actuated._actuator_callback_id)
        finally:
            actuated.close()

    async def test_enable_auto_step_pre_physics_before_play(self) -> None:
        """Verify that calling `enable_auto_step_pre_physics()` before physics starts sets the flag and
        registers the callback immediately, and that it remains valid after `PHYSICS_READY` fires."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=False)
        try:
            actuated.enable_auto_step_pre_physics()
            self.assertTrue(actuated._auto_step_pre_physics)
            self.assertIsNotNone(actuated._actuator_callback_id)

            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            # `_on_physics_ready` is idempotent; same callback ID must survive.
            self.assertIsNotNone(actuated._actuator_callback_id)
        finally:
            actuated.close()

    async def test_physics_callback_cleared_on_timeline_stop(self) -> None:
        """Verify that `_actuator_callback_id` is `None` after `SIMULATION_STOPPED` while the enabled flag persists."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNotNone(actuated._actuator_callback_id)

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNone(actuated._actuator_callback_id)
            self.assertTrue(actuated._auto_step_pre_physics, "Enabled flag must survive timeline stop")
        finally:
            actuated.close()

    async def test_physics_callback_re_registered_on_replay(self) -> None:
        """Verify that after a stop/start cycle the physics callback is re-registered automatically."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNone(actuated._actuator_callback_id)

            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNotNone(actuated._actuator_callback_id)
        finally:
            actuated.close()

    async def test_disable_auto_step_pre_physics_prevents_re_registration(self) -> None:
        """Verify that `disable_auto_step_pre_physics()` clears the enabled flag so the callback is not
        re-registered on the next `PHYSICS_READY` event."""
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", Sdf.Path(_REVOLUTE_JOINT_PATH))

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNotNone(actuated._actuator_callback_id)

            actuated.disable_auto_step_pre_physics()
            self.assertIsNone(actuated._actuator_callback_id)
            self.assertFalse(actuated._auto_step_pre_physics)

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self.assertIsNone(actuated._actuator_callback_id, "Callback must not re-register after deregister")
        finally:
            actuated.close()

    async def test_lifecycle_callbacks_cleared_on_close(self) -> None:
        """Verify that `close()` deregisters and clears the lifecycle callback list."""
        actuated = ArticulationActuators(_ART_ROOT)
        self.assertEqual(len(actuated._lifecycle_callback_ids), 2)
        actuated.close()
        self.assertEqual(actuated._lifecycle_callback_ids, [])

    # ------------------------------------------------------------------
    # Drive gain zeroing (PHYSICS_READY integration)
    # ------------------------------------------------------------------

    async def test_drive_gains_zeroed_on_physics_ready(self) -> None:
        """Verify that drive stiffness and damping are zeroed on the actuated DOF when physics becomes ready."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path)

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            stiffness, damping = actuated.articulation.get_dof_gains(dof_indices=target_dof_index)
            self.assertEqual(stiffness.numpy().item(), 0.0, "Stiffness must be zeroed on actuated DOF")
            self.assertEqual(damping.numpy().item(), 0.0, "Damping must be zeroed on actuated DOF")
        finally:
            actuated.close()

    async def test_drive_gains_not_zeroed_on_non_actuated_dofs(self) -> None:
        """Verify that drive gains remain untouched on DOFs that have no corresponding actuator."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        non_actuated_path = Sdf.Path(_PRISMATIC_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path)

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            non_actuated_dof_index = actuated.articulation.dof_paths[0].index(str(non_actuated_path))
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            stiffness, damping = actuated.articulation.get_dof_gains(dof_indices=non_actuated_dof_index)
            self.assertNotEqual(stiffness.numpy().item(), 0.0, "Stiffness must be preserved on non-actuated DOF")
            self.assertNotEqual(damping.numpy().item(), 0.0, "Damping must be preserved on non-actuated DOF")
        finally:
            actuated.close()

    async def test_drive_gains_rezeroed_on_replay(self) -> None:
        """Verify that drive gains are re-zeroed on every `PHYSICS_READY` event, even when manually restored."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path)

        actuated = ArticulationActuators(_ART_ROOT)
        try:
            target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            actuated.articulation.set_dof_gains(stiffnesses=1.0, dampings=1.0, dof_indices=target_dof_index)
            stiffness, _ = actuated.articulation.get_dof_gains(dof_indices=target_dof_index)
            self.assertNotEqual(stiffness.numpy().item(), 0.0, "Gains must be non-zero before replay")

            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            stiffness, damping = actuated.articulation.get_dof_gains(dof_indices=target_dof_index)
            self.assertEqual(stiffness.numpy().item(), 0.0, "Stiffness must be re-zeroed on replay")
            self.assertEqual(damping.numpy().item(), 0.0, "Damping must be re-zeroed on replay")
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Motion (end-to-end physics)
    # ------------------------------------------------------------------

    async def test_actuator_drives_motion(self) -> None:
        """Verify that with the callback enabled, a PD actuator moves the joint toward a non-zero target."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path, kp=100.0, kd=100.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            target_position = 0.05
            actuated.articulation.set_dof_position_targets(positions=target_position, dof_indices=target_dof_index)
            start_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()
            self.assertEqual(start_position, 0.0)

            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()

            self.assertNotEqual(
                actuated.articulation.get_dof_efforts(dof_indices=target_dof_index).numpy().item(),
                0.0,
                "Actuator must apply non-zero effort",
            )
            final_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()
            self.assertTrue(
                math.fabs(final_position - target_position) < 1e-2,
                f"Joint must converge to {target_position} rad; got {final_position}",
            )
        finally:
            actuated.close()

    async def test_no_actuator_callback_no_motion(self) -> None:
        """Verify that without the callback enabled, a PD actuator does not move the joint toward a non-zero target."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path, kp=100.0, kd=100.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=False)
        target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            target_position = 0.05
            actuated.articulation.set_dof_position_targets(positions=target_position, dof_indices=target_dof_index)
            start_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()
            self.assertEqual(start_position, 0.0)

            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()

            self.assertEqual(
                actuated.articulation.get_dof_efforts(dof_indices=target_dof_index).numpy().item(),
                0.0,
                "Actuator must apply zero effort",
            )
            final_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()

            # This joint shouldn't move at all, since the drive gains are zeroed, and the
            # ArticulationActuators does not have its update callback enabled.
            self.assertAlmostEqual(
                final_position,
                0.0,
                places=2,
            )
        finally:
            actuated.close()

    async def test_manual_step_actuators_drives_motion(self) -> None:
        """Verify that calling `step_actuators` manually each frame moves the joint when auto-step is disabled."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path, kp=100.0, kd=100.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=False)
        target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            target_position = 0.05
            actuated.articulation.set_dof_position_targets(positions=target_position, dof_indices=target_dof_index)
            start_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()
            self.assertEqual(start_position, 0.0)

            for _ in range(120):
                actuated.step_actuators(step_dt=1.0 / 60.0, context=None)
                await omni.kit.app.get_app().next_update_async()

            final_position = actuated.articulation.get_dof_positions(dof_indices=target_dof_index).numpy().item()
            self.assertLess(
                math.fabs(final_position - target_position),
                1e-2,
                f"Joint must converge to {target_position} rad via manual stepping; got {final_position}",
            )
        finally:
            actuated.close()

    async def test_both_joints_converge_to_targets(self) -> None:
        """Verify that both DOFs converge to their respective targets when each has its own PD actuator."""
        revolute_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        prismatic_path = Sdf.Path(_PRISMATIC_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/revolute_actuator", revolute_path, kp=100.0, kd=100.0)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/prismatic_actuator", prismatic_path, kp=100.0, kd=100.0)

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        dof_paths = actuated.articulation.dof_paths[0]
        revolute_idx = dof_paths.index(str(revolute_path))
        prismatic_idx = dof_paths.index(str(prismatic_path))
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            revolute_target = 0.05
            prismatic_target = 0.04
            actuated.articulation.set_dof_position_targets(positions=revolute_target, dof_indices=revolute_idx)
            actuated.articulation.set_dof_position_targets(positions=prismatic_target, dof_indices=prismatic_idx)

            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()

            revolute_final = actuated.articulation.get_dof_positions(dof_indices=revolute_idx).numpy().item()
            prismatic_final = actuated.articulation.get_dof_positions(dof_indices=prismatic_idx).numpy().item()
            self.assertLess(
                math.fabs(revolute_final - revolute_target),
                1e-2,
                f"Revolute joint must converge to {revolute_target} rad; got {revolute_final}",
            )
            self.assertLess(
                math.fabs(prismatic_final - prismatic_target),
                1e-2,
                f"Prismatic joint must converge to {prismatic_target} m; got {prismatic_final}",
            )
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Reset / state
    # ------------------------------------------------------------------

    async def test_reset_clears_pid_integral(self) -> None:
        """Verify that after stepping with a non-zero position error, the PID integral is non-zero.

        Also verify that calling `reset()` zeros the integral so that the next step starts
        from a clean state.
        """
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pid_actuator(
            f"{_ART_ROOT}/Actuators/joint_actuator", target_joint_path, kp=100.0, ki=50.0, kd=10.0
        )

        actuated = ArticulationActuators(_ART_ROOT, auto_step_pre_physics=True)
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            target_dof_index = actuated.articulation.dof_paths[0].index(str(target_joint_path))
            actuated.articulation.set_dof_position_targets(positions=0.5, dof_indices=target_dof_index)

            for _ in range(30):
                await omni.kit.app.get_app().next_update_async()

            integral_values = actuated._cur_states[0].controller_state.integral.numpy()
            self.assertTrue(
                any(abs(v) > 0.0 for v in integral_values),
                "PID integral must be non-zero after stepping with sustained position error",
            )

            actuated.reset()

            integral_after_reset = actuated._cur_states[0].controller_state.integral.numpy()
            self.assertTrue(
                all(v == 0.0 for v in integral_after_reset),
                "PID integral must be zero after reset()",
            )
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------------

    async def test_actuator_targets_non_dof_path_raises(self) -> None:
        """Verify that a `NewtonActuator` whose target is not a DOF of the articulation raises `ValueError`."""
        stage = stage_utils.get_current_stage(backend="usd")
        non_dof_path = Sdf.Path(f"{_ART_ROOT}/Arm")
        prim = stage.DefinePrim(f"{_ART_ROOT}/Actuators/bad_actuator", "NewtonActuator")
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(100.0)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(10.0)
        prim.CreateRelationship("newton:targets").SetTargets([non_dof_path])

        with self.assertRaises(ValueError):
            ArticulationActuators(_ART_ROOT)

    async def test_duplicate_dof_raises(self) -> None:
        """Verify that two `NewtonActuator` prims targeting the same DOF raise `ValueError`."""
        target_joint_path = Sdf.Path(_REVOLUTE_JOINT_PATH)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/actuator_a", target_joint_path)
        self._author_pd_actuator(f"{_ART_ROOT}/Actuators/actuator_b", target_joint_path)

        with self.assertRaises(ValueError):
            ArticulationActuators(_ART_ROOT)

    # ------------------------------------------------------------------
    # Robustness
    # ------------------------------------------------------------------

    async def test_close_is_idempotent(self) -> None:
        """Verify that `close()` is safely callable multiple times without raising."""
        actuated = ArticulationActuators(_ART_ROOT)
        actuated.close()
        actuated.close()

    async def test_context_manager_closes_on_exit(self) -> None:
        """Verify that using `ArticulationActuators` as a context manager calls `close()` on exit.

        The context-manager protocol is the recommended teardown idiom over relying
        on `__del__`. After the `with` block exits, all lifecycle callbacks must be
        deregistered (i.e. `_lifecycle_callback_ids` is empty), even if the body
        raised.
        """
        with ArticulationActuators(_ART_ROOT) as actuated:
            self.assertEqual(len(actuated._lifecycle_callback_ids), 2)
        self.assertEqual(actuated._lifecycle_callback_ids, [])

        # Exit must still run close() if the body raises.
        actuated2 = ArticulationActuators(_ART_ROOT)
        with self.assertRaises(RuntimeError):
            with actuated2:
                self.assertEqual(len(actuated2._lifecycle_callback_ids), 2)
                raise RuntimeError("body raised")
        self.assertEqual(actuated2._lifecycle_callback_ids, [])

    async def test_disable_auto_step_pre_physics_when_not_registered_is_noop(self) -> None:
        """Verify that `disable_auto_step_pre_physics()` on a fresh default instance does not raise."""
        actuated = ArticulationActuators(_ART_ROOT)
        try:
            actuated.disable_auto_step_pre_physics()
        finally:
            actuated.close()

    async def test_step_actuators_is_noop_with_zero_actuators(self) -> None:
        """Verify that `step_actuators` on a zero-actuator instance does not raise."""
        actuated = ArticulationActuators(_ART_ROOT)
        try:
            actuated.step_actuators(step_dt=0.01, context=None)
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # from_actuators
    # ------------------------------------------------------------------

    def _make_pd_config(self, kp: float = 100.0, kd: float = 10.0) -> ActuatorConfig:
        """Build an `ActuatorConfig` with a `ControllerPD`."""
        import warp as wp
        from newton.actuators import ControllerPD

        return ActuatorConfig(
            controller=ControllerPD(
                kp=wp.array([kp], dtype=wp.float32),
                kd=wp.array([kd], dtype=wp.float32),
            )
        )

    def _make_ff_clamped_config(self, max_effort: float) -> ActuatorConfig:
        """Build an `ActuatorConfig` with kp=kd=0 and `ClampingMaxEffort`.

        Useful for testing that feedforward effort is symmetrically clamped to ±`max_effort`.

        Args:
            max_effort: Symmetric effort limit [N or N·m].

        Returns:
            An `ActuatorConfig` whose net output is ``clip(feedforward, -max_effort, +max_effort)``.
        """
        import warp as wp
        from newton.actuators import ClampingMaxEffort

        config = self._make_pd_config(kp=0.0, kd=0.0)
        config.clamping = [ClampingMaxEffort(max_effort=wp.array([max_effort], dtype=wp.float32))]
        return config

    def _make_pid_config(self, kp: float = 100.0, ki: float = 10.0, kd: float = 5.0) -> ActuatorConfig:
        """Build an `ActuatorConfig` with a `ControllerPID`."""
        import warp as wp
        from newton.actuators import ControllerPID

        return ActuatorConfig(
            controller=ControllerPID(
                kp=wp.array([kp], dtype=wp.float32),
                ki=wp.array([ki], dtype=wp.float32),
                kd=wp.array([kd], dtype=wp.float32),
                integral_max=wp.array([float("inf")], dtype=wp.float32),
            )
        )

    async def test_from_actuators_adds_python_actuator(self) -> None:
        """Verify that a Python-constructed `Actuator` is present when no USD prims are authored."""
        from newton.actuators import ControllerPD

        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [(self._make_pd_config(kp=200.0, kd=20.0), "RevoluteJoint")],
        )
        try:
            self.assertEqual(len(actuated.actuators), 1)
            self.assertIsInstance(actuated.actuators[0].controller, ControllerPD)
        finally:
            actuated.close()

    async def test_from_actuators_runs_on_cpu_device(self) -> None:
        """Verify that `from_actuators` succeeds when all Warp work is forced to CPU.

        Guards the CPU-only path by forcing Warp's active device to ``cpu`` during
        construction and manual stepping, validating that effort computation runs
        without device-mismatch errors.
        """
        import warp as wp

        with wp.ScopedDevice("cpu"):
            actuated = ArticulationActuators.from_actuators(
                _ART_ROOT,
                [(self._make_pd_config(kp=100.0, kd=100.0), "RevoluteJoint")],
                auto_step_pre_physics=False,
            )
            dof_index = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
            try:
                self._timeline.play()
                await omni.kit.app.get_app().next_update_async()

                target_position = 0.05
                actuated.articulation.set_dof_position_targets(positions=target_position, dof_indices=dof_index)

                for _ in range(5):
                    actuated.step_actuators(step_dt=1.0 / 60.0)
                    await omni.kit.app.get_app().next_update_async()

                applied = actuated.articulation.get_dof_efforts(dof_indices=dof_index).numpy().item()
                self.assertNotEqual(applied, 0.0, "CPU-only stepping must apply a non-zero effort.")
            finally:
                actuated.close()

    async def test_from_actuators_unknown_dof_raises(self) -> None:
        """Verify that `from_actuators` raises `ValueError` for an unrecognised DOF name."""
        with self.assertRaises(ValueError):
            ArticulationActuators.from_actuators(
                _ART_ROOT,
                [(self._make_pd_config(), "NonExistentJoint")],
            ).close()

    async def test_from_actuators_duplicate_dof_raises(self) -> None:
        """Verify that `from_actuators` raises `ValueError` when the same DOF appears twice."""
        with self.assertRaises(ValueError):
            ArticulationActuators.from_actuators(
                _ART_ROOT,
                [
                    (self._make_pd_config(), "RevoluteJoint"),
                    (self._make_pd_config(), "RevoluteJoint"),
                ],
            ).close()

    async def test_from_actuators_multiple_dofs(self) -> None:
        """Verify that both DOFs appear in `actuated_dof_indices` when two actuators are provided."""
        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [
                (self._make_pd_config(), "RevoluteJoint"),
                (self._make_pd_config(), "PrismaticJoint"),
            ],
        )
        try:
            self.assertEqual(len(actuated.actuators), 2)
            self.assertEqual(len(actuated.actuated_dof_indices), 2)
        finally:
            actuated.close()

    async def test_from_actuators_drives_motion(self) -> None:
        """Verify that a Python-built PD actuator drives the joint toward its position target."""
        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [(self._make_pd_config(kp=100.0, kd=100.0), "RevoluteJoint")],
            auto_step_pre_physics=True,
        )
        dof_index = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            target_position = 0.05
            actuated.articulation.set_dof_position_targets(positions=target_position, dof_indices=dof_index)
            self.assertEqual(actuated.articulation.get_dof_positions(dof_indices=dof_index).numpy().item(), 0.0)

            for _ in range(120):
                await omni.kit.app.get_app().next_update_async()

            final_position = actuated.articulation.get_dof_positions(dof_indices=dof_index).numpy().item()
            self.assertLess(
                math.fabs(final_position - target_position),
                1e-2,
                f"Joint must converge to {target_position} rad; got {final_position}",
            )
        finally:
            actuated.close()

    # ------------------------------------------------------------------
    # Feedforward effort targets
    # ------------------------------------------------------------------

    async def test_feedforward_effort_applied(self) -> None:
        """Verify that with zero PD gains, the effort written to PhysX equals the feedforward target exactly.

        Acts as the baseline check that `set_dof_feedforward_effort_targets` routes values
        through `step_actuators` and into `get_dof_efforts`.
        """
        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [(self._make_pd_config(kp=0.0, kd=0.0), "RevoluteJoint")],
            auto_step_pre_physics=False,
        )
        dof_index = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            feedforward_effort = 7.5
            actuated.set_dof_feedforward_effort_targets(feedforward_effort, dof_indices=dof_index)
            actuated.step_actuators(step_dt=1.0 / 60.0)

            applied = actuated.articulation.get_dof_efforts(dof_indices=dof_index).numpy().item()
            self.assertAlmostEqual(applied, feedforward_effort, places=3)
        finally:
            actuated.close()

    async def test_feedforward_effort_saturated_by_max_effort_clamping(self) -> None:
        """Verify that a feedforward effort exceeding `ClampingMaxEffort` is clamped to ±`max_effort`.

        With zero PD gains, the only contribution to effort is the feedforward value. The clamping
        stage must reduce an over-limit feedforward to exactly `max_effort`.
        """
        max_effort = 2.0
        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [(self._make_ff_clamped_config(max_effort), "RevoluteJoint")],
            auto_step_pre_physics=False,
        )
        dof_index = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            actuated.set_dof_feedforward_effort_targets(10.0, dof_indices=dof_index)
            actuated.step_actuators(step_dt=1.0 / 60.0)

            applied = actuated.articulation.get_dof_efforts(dof_indices=dof_index).numpy().item()
            self.assertAlmostEqual(applied, max_effort, places=3, msg="Effort must be clamped to max_effort")
        finally:
            actuated.close()

    async def test_feedforward_additive_with_pd_control(self) -> None:
        """Verify that feedforward is added on top of the PD control output.

        With kd=0 and the joint at rest (velocity=0), effort = kp*(target - pos) + feedforward.
        Starting from pos=0 with target=1.0 rad and kp=100, the expected applied effort is
        ``100 * 1.0 + feedforward``.
        """
        kp = 100.0
        feedforward_effort = 5.0
        position_target = 1.0

        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [(self._make_pd_config(kp=kp, kd=0.0), "RevoluteJoint")],
            auto_step_pre_physics=False,
        )
        dof_index = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            actuated.articulation.set_dof_position_targets(position_target, dof_indices=dof_index)
            actuated.set_dof_feedforward_effort_targets(feedforward_effort, dof_indices=dof_index)
            actuated.step_actuators(step_dt=1.0 / 60.0)

            applied = actuated.articulation.get_dof_efforts(dof_indices=dof_index).numpy().item()
            expected = kp * position_target + feedforward_effort
            self.assertAlmostEqual(applied, expected, places=3)
        finally:
            actuated.close()

    async def test_feedforward_set_simultaneously_for_all_dofs(self) -> None:
        """Verify that setting feedforward for all DOFs in one call assigns the correct value to each DOF.

        Exercises the multi-DOF indexing path in `set_dof_feedforward_effort_targets`
        where the `indices` array (length 1) and `dof_indices` array (length 2) differ in size —
        the scenario that requires `np.ix_` for correct outer-product assignment.
        With zero PD gains, each DOF's effort must equal its corresponding feedforward value.
        """
        import numpy as np

        actuated = ArticulationActuators.from_actuators(
            _ART_ROOT,
            [
                (self._make_pd_config(kp=0.0, kd=0.0), "RevoluteJoint"),
                (self._make_pd_config(kp=0.0, kd=0.0), "PrismaticJoint"),
            ],
            auto_step_pre_physics=False,
        )
        revolute_idx = int(actuated.articulation.get_dof_indices("RevoluteJoint").numpy()[0])
        prismatic_idx = int(actuated.articulation.get_dof_indices("PrismaticJoint").numpy()[0])
        try:
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

            revolute_ff = 9.0
            prismatic_ff = 3.0
            n_dofs = actuated.articulation.num_dofs
            efforts = np.zeros(n_dofs, dtype=np.float32)
            efforts[revolute_idx] = revolute_ff
            efforts[prismatic_idx] = prismatic_ff
            # Pass feedforward for all DOFs in a single call (no dof_indices ⟹ full DOF range).
            actuated.set_dof_feedforward_effort_targets(efforts)
            actuated.step_actuators(step_dt=1.0 / 60.0)

            revolute_applied = actuated.articulation.get_dof_efforts(dof_indices=revolute_idx).numpy().item()
            prismatic_applied = actuated.articulation.get_dof_efforts(dof_indices=prismatic_idx).numpy().item()
            self.assertAlmostEqual(revolute_applied, revolute_ff, places=3)
            self.assertAlmostEqual(prismatic_applied, prismatic_ff, places=3)
        finally:
            actuated.close()


class TestArticulationActuatorsMultiFranka(omni.kit.test.AsyncTestCase):
    """Tests that exercise `ArticulationActuators` over multiple Franka Panda instances."""

    async def setUp(self) -> None:
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        franka_usd_path = f"{get_assets_root_path()}/{_FRANKA_REL_PATH}"
        for i in range(_NUM_FRANKAS):
            stage_utils.add_reference_to_stage(
                usd_path=franka_usd_path,
                path=f"{_FRANKA_BASE_PATH}_{i}",
            )
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_per_joint_pd_effort_matches_formula(self) -> None:
        """Verify per-(instance, joint) PD effort matches ``kp*(target - q) - kd*qd``.

        Spawns ``_NUM_FRANKAS`` Franka Panda instances under ``/World/Franka_<i>`` and attaches
        a single ``ControllerPD`` per arm joint, with a *distinct* ``kp`` and ``kd`` on every
        joint so that no two cells of the resulting ``(n_robots, n_arm_dofs)`` effort matrix
        share the same expected value.  After a few unactuated physics ticks (so velocities
        are non-trivially non-zero from gravity), the test sets per-(instance, joint) position
        targets, calls ``step_actuators`` once, and asserts that the applied effort on each
        cell equals ``kp[joint] * (target - q) - kd[joint] * qd`` evaluated at the state
        ``step_actuators`` saw.
        """
        import numpy as np
        import warp as wp
        from newton.actuators import ControllerPD

        # Distinct gains per joint so each (joint, instance) cell of the effort matrix is
        # uniquely identifiable.  Picking values an order of magnitude apart makes off-by-one
        # mismatches between the joint and gain arrays trivially visible in failure messages.
        joint_kps = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
        joint_kds = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

        actuators: list[tuple[ActuatorConfig, str]] = []
        for joint_name, kp, kd in zip(_FRANKA_ARM_JOINTS, joint_kps, joint_kds):
            cfg = ActuatorConfig(
                controller=ControllerPD(
                    kp=wp.array([kp] * _NUM_FRANKAS, dtype=wp.float32),
                    kd=wp.array([kd] * _NUM_FRANKAS, dtype=wp.float32),
                )
            )
            actuators.append((cfg, joint_name))

        actuated = ArticulationActuators.from_actuators(
            _FRANKA_REGEX,
            actuators=actuators,
            # Manual stepping is required so that the state we read back matches the state
            # that step_actuators saw when computing the effort.  With auto-stepping, physics
            # advances after the pre-physics callback and the read-back state is one tick
            # ahead of the effort, breaking the formula check.
            auto_step_pre_physics=False,
        )
        try:
            self.assertEqual(len(actuated.articulation), _NUM_FRANKAS)
            self.assertEqual(len(actuated.actuators), len(_FRANKA_ARM_JOINTS))

            arm_dof_indices = actuated.articulation.get_dof_indices(_FRANKA_ARM_JOINTS).numpy()

            self._timeline.play()
            # Run a handful of unactuated ticks so the robot drifts under gravity and the
            # joint velocities are non-zero — without this, the kd term would be ~0 and
            # we'd lose the ability to distinguish a correct kd value from any other.
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()

            # Distinct per-(instance, joint) target so each effort cell is uniquely traceable.
            desired_q = np.array(
                [
                    [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70],
                    [-0.05, -0.10, -0.15, -0.20, -0.25, -0.30, -0.35],
                ],
                dtype=np.float32,
            )
            self.assertEqual(desired_q.shape, (_NUM_FRANKAS, len(_FRANKA_ARM_JOINTS)))
            actuated.articulation.set_dof_position_targets(
                positions=desired_q,
                dof_indices=arm_dof_indices,
            )

            # Compute and apply efforts once.  step_actuators reads (q, qd) and the position
            # targets, runs the PD controller for every actuator, and writes the resulting
            # effort to PhysX via set_dof_efforts.  No physics step happens here, so the
            # state we read on the next three lines is exactly what step_actuators saw.
            actuated.step_actuators(step_dt=1.0 / 60.0)

            measured_q = actuated.articulation.get_dof_positions(dof_indices=arm_dof_indices).numpy()
            measured_qd = actuated.articulation.get_dof_velocities(dof_indices=arm_dof_indices).numpy()
            applied_efforts = actuated.articulation.get_dof_efforts(dof_indices=arm_dof_indices).numpy()

            self.assertEqual(measured_q.shape, (_NUM_FRANKAS, len(_FRANKA_ARM_JOINTS)))
            self.assertEqual(measured_qd.shape, (_NUM_FRANKAS, len(_FRANKA_ARM_JOINTS)))
            self.assertEqual(applied_efforts.shape, (_NUM_FRANKAS, len(_FRANKA_ARM_JOINTS)))

            kp_arr = np.array(joint_kps, dtype=np.float32)  # shape (n_arm_dofs,) — broadcasts across instances
            kd_arr = np.array(joint_kds, dtype=np.float32)
            expected_efforts = kp_arr * (desired_q - measured_q) - kd_arr * measured_qd

            np.testing.assert_allclose(
                applied_efforts,
                expected_efforts,
                rtol=1e-4,
                atol=1e-3,
                err_msg=(
                    "Applied PD effort on each (Franka instance, arm joint) must equal "
                    "kp[joint]*(target - q) - kd[joint]*qd."
                ),
            )
        finally:
            actuated.close()
