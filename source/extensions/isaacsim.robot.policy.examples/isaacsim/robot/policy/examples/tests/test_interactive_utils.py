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

"""Tests for ``isaacsim.robot.policy.examples.interactive.utils``."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.policy.examples.interactive.go2.go2_example import Go2Example
from isaacsim.robot.policy.examples.interactive.humanoid.humanoid_example import HumanoidExample
from isaacsim.robot.policy.examples.interactive.quadruped.quadruped_example import QuadrupedExample
from isaacsim.robot.policy.examples.interactive.utils import (
    restore_physics_simulation_state,
    snapshot_physics_simulation_state,
)


class TestInteractiveUtils(omni.kit.test.AsyncTestCase):
    """Unit tests for the snapshot/restore helpers used by the interactive policy examples.

    These helpers exist to prevent the PhysX direct-GPU-API flag and fabric extension from
    leaking across scene clears: ``cuda`` device implies fabric + readback suppression, which
    must be undone when the example tears down so subsequent USD edits don't hit
    ``PxArticulationJointReducedCoordinate::setDriveTarget`` errors.
    """

    async def setUp(self):
        """Create a fresh stage and capture the current physics state for restoration."""
        await stage_utils.create_new_stage_async()
        self._initial_device, self._initial_fabric = snapshot_physics_simulation_state()
        # Start from a known baseline: CPU + fabric off.
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(False)

    async def tearDown(self):
        """Restore the pre-test physics state so this test doesn't leak global flags."""
        restore_physics_simulation_state(self._initial_device, self._initial_fabric)
        await omni.kit.app.get_app().next_update_async()

    async def test_snapshot_returns_current_state(self):
        """``snapshot_physics_simulation_state`` returns the live device and fabric flag."""
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(False)
        device, fabric_enabled = snapshot_physics_simulation_state()
        self.assertEqual(device, "cpu")
        self.assertFalse(fabric_enabled)

    async def test_restore_applies_device_and_fabric(self):
        """``restore_physics_simulation_state`` puts the device and fabric flag back."""
        # Flip into the "GPU-like" state that the interactive examples leave behind.
        SimulationManager.enable_fabric(True)
        self.assertTrue(SimulationManager.is_fabric_enabled())

        # Restore to a known CPU + fabric-off snapshot.
        restore_physics_simulation_state("cpu", False)

        self.assertEqual(SimulationManager.get_physics_sim_device(), "cpu")
        self.assertFalse(SimulationManager.is_fabric_enabled())

    async def test_restore_with_none_is_noop(self):
        """``None`` snapshot fields leave the corresponding live state untouched."""
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(True)
        device_before = SimulationManager.get_physics_sim_device()
        fabric_before = SimulationManager.is_fabric_enabled()

        restore_physics_simulation_state(None, None)

        self.assertEqual(SimulationManager.get_physics_sim_device(), device_before)
        self.assertEqual(SimulationManager.is_fabric_enabled(), fabric_before)

    async def test_snapshot_then_restore_roundtrip(self):
        """``snapshot`` followed by ``restore`` reverses any intermediate mutation."""
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(False)
        prev_device, prev_fabric = snapshot_physics_simulation_state()

        # Simulate the example flipping the device to cuda (which enables fabric).
        SimulationManager.set_physics_sim_device("cuda")
        self.assertTrue(SimulationManager.is_fabric_enabled())

        restore_physics_simulation_state(prev_device, prev_fabric)

        self.assertEqual(SimulationManager.get_physics_sim_device(), prev_device)
        self.assertEqual(SimulationManager.is_fabric_enabled(), prev_fabric)


class TestInteractiveExamplePhysicsStateRoundtrip(omni.kit.test.AsyncTestCase):
    """Verify each interactive example restores the global physics state on cleanup.

    Each example's ``setup_scene`` flips the physics sim device to ``cuda`` (which enables
    fabric and the PhysX direct-GPU API). If ``physics_cleanup`` / ``setup_post_clear`` does
    not undo that, the next session hits ``setDriveTarget`` errors when the user modifies USD.
    These tests exercise the snapshot → mutate → cleanup path on a fresh example instance
    without loading any robot assets.
    """

    async def setUp(self):
        """Establish a known baseline (CPU + fabric off) before each test."""
        await stage_utils.create_new_stage_async()
        self._initial_device, self._initial_fabric = snapshot_physics_simulation_state()
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(False)

    async def tearDown(self):
        """Put the global physics state back to whatever the previous test left it as."""
        restore_physics_simulation_state(self._initial_device, self._initial_fabric)
        await omni.kit.app.get_app().next_update_async()

    def _run_roundtrip(self, example) -> None:
        """Simulate ``setup_scene`` + ``physics_cleanup`` and assert state is restored."""
        before_device, before_fabric = snapshot_physics_simulation_state()

        # Mirror the snapshot performed at the top of ``setup_scene``.
        example._prev_physics_sim_device, example._prev_fabric_enabled = snapshot_physics_simulation_state()

        # Mirror the state mutations ``setup_scene`` performs (skip asset loading).
        SimulationManager.set_backend(example._world_settings["backend"])
        SimulationManager.set_physics_sim_device(example._world_settings["device"])

        self.assertIn("cuda", SimulationManager.get_physics_sim_device())
        self.assertTrue(SimulationManager.is_fabric_enabled())

        example.physics_cleanup()

        after_device, after_fabric = snapshot_physics_simulation_state()
        self.assertEqual(after_device, before_device)
        self.assertEqual(after_fabric, before_fabric)

    async def test_quadruped_example_restores_physics_state(self):
        """``QuadrupedExample`` cleanup must restore the prior device and fabric flag."""
        self._run_roundtrip(QuadrupedExample())

    async def test_go2_example_restores_physics_state(self):
        """``Go2Example`` cleanup must restore the prior device and fabric flag."""
        self._run_roundtrip(Go2Example())

    async def test_humanoid_example_restores_physics_state(self):
        """``HumanoidExample`` cleanup must restore the prior device and fabric flag."""
        self._run_roundtrip(HumanoidExample())
