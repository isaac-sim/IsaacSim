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

"""Verify public Newton runtime configuration behavior."""

import copy

import omni.kit.test
from isaacsim.physics.newton import (
    NewtonConfig,
    NewtonSolverOutputConfig,
    configure_newton,
    get_newton_config,
)


class TestNewtonConfig(omni.kit.test.AsyncTestCase):
    """Tests for ``configure_newton()`` runtime configuration."""

    async def setUp(self) -> None:
        """Save the current Newton configuration."""
        super().setUp()
        self._saved_config = copy.deepcopy(get_newton_config()) or NewtonConfig()

    async def tearDown(self) -> None:
        """Restore the previous Newton configuration."""
        configure_newton(self._saved_config)
        super().tearDown()

    async def test_configure_newton_updates_solver_outputs(self) -> None:
        """Configure optional solver-computed outputs before initialization."""
        config = NewtonConfig(
            solver_outputs=NewtonSolverOutputConfig(
                contact_forces=False,
                link_incoming_joint_force=True,
            ),
        )
        configure_newton(config)

        live = get_newton_config()
        self.assertIsNotNone(live)
        self.assertFalse(live.solver_outputs.contact_forces)
        self.assertTrue(live.solver_outputs.link_incoming_joint_force)
