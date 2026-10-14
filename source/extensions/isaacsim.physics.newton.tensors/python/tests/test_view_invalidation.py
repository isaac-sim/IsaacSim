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

"""Tensor views across a Newton re-initialization.

``NewtonStage`` rebuilds ``model``, ``control`` and ``state_0`` whenever it
re-initializes (any USD change marks it uninitialized). Views cache raw
pointers into those arrays, so an existing view must report ``check() ==
False`` and fail its queries afterwards instead of reading released buffers;
views created against the rebuilt model work.
"""

from __future__ import annotations

from typing import Any

from .test_helpers import NewtonTensorTestBase, run_on_device_configs


@run_on_device_configs()
class TestViewInvalidation(NewtonTensorTestBase):
    """Views created before a Newton rebuild become invalid; views created after it work."""

    def _reinitialize_newton(self) -> Any:
        """Rebuild the Newton model and states from the current USD stage, as a USD change does.

        Returns:
            The live Newton stage, now backed by new ``model`` and ``state_0`` objects.
        """
        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        old_model, old_state = newton_stage.model, newton_stage.state_0
        # The unified physics interface reports USD edits through `on_change`, which only marks the stage
        # uninitialized; the rebuild happens on the next step or tensor-view creation.
        newton_stage.on_change("/")
        newton_stage.initialize_newton(self.SIM_DEVICE)
        self.assertTrue(newton_stage.initialized, "Newton failed to re-initialize")
        self.assertIsNot(newton_stage.model, old_model)
        self.assertIsNot(newton_stage.state_0, old_state)
        return newton_stage

    async def test_rigid_body_and_contact_views_invalidate_after_reinitialization(self) -> None:
        """Verify rigid body and contact views report invalid and fail their queries after a Newton rebuild."""
        num_envs = self.setup_ball_grid(num_envs=4)
        sim = await self.create_sim()
        balls = sim.create_rigid_body_view("/envs/*/ball")
        contacts = sim.create_rigid_contact_view(["/envs/*/ball"])
        self.check_rigid_body_view(balls, num_envs)
        self.assertTrue(balls.check())
        self.assertTrue(contacts.check())
        self.start_playing()
        self.step(5)
        self.assertEqual(balls.get_transforms().numpy().reshape(-1, 7).shape[0], num_envs)

        self._reinitialize_newton()

        self.assertFalse(balls.check(), "rigid body view still reports valid after the Newton rebuild")
        self.assertFalse(contacts.check(), "rigid contact view still reports valid after the Newton rebuild")
        with self.assertRaisesRegex(Exception, "Failed to get rigid body transforms"):
            balls.get_transforms()

        # Views created against the rebuilt model work.
        fresh = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(fresh, num_envs)
        self.assertTrue(fresh.check())
        self.step(5)
        self.assertEqual(fresh.get_transforms().numpy().reshape(-1, 7).shape[0], num_envs)
        self.assertTrue(sim.create_rigid_contact_view(["/envs/*/ball"]).check())

    async def test_articulation_view_invalidates_after_reinitialization(self) -> None:
        """Verify an articulation view reports invalid and fails its queries after a Newton rebuild."""
        num_envs = self.setup_cartpole_grid(num_envs=2)
        sim = await self.create_sim()
        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(cartpoles, num_envs, 3, 2)
        self.assertTrue(cartpoles.check())
        self.start_playing()
        self.step(2)
        self.assertEqual(cartpoles.get_dof_positions().numpy().size, num_envs * 2)

        self._reinitialize_newton()

        self.assertFalse(cartpoles.check(), "articulation view still reports valid after the Newton rebuild")
        with self.assertRaisesRegex(Exception, "Failed to get DOF positions"):
            cartpoles.get_dof_positions()

        fresh = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(fresh, num_envs, 3, 2)
        self.assertTrue(fresh.check())
        self.step(2)
        self.assertEqual(fresh.get_dof_positions().numpy().size, num_envs * 2)
