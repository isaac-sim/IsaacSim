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

"""Tests for physics utility functions."""

import numpy as np
import omni.kit.commands
import omni.kit.test
from isaacsim.core.utils.physics import get_rigid_body_enabled, set_rigid_body_enabled
from pxr import UsdPhysics


class TestPhysics(omni.kit.test.AsyncTestCase):
    """Test cases for Physics."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_rigid_body_enabled(self) -> None:
        """Test rigid body enabled."""
        from isaacsim.core.utils.prims import create_prim
        from isaacsim.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World/Floor")
        cube = create_prim(
            "/World/Floor/thefloor", "Cube", position=np.array([75, 75, -150.1]), attributes={"size": 300}
        )
        create_prim("/World/Room", "Sphere", attributes={"radius": 1e3})
        UsdPhysics.RigidBodyAPI.Apply(cube)

        # None if rbapi not there
        result = get_rigid_body_enabled("/World/Room")
        self.assertEqual(result, None)

        # True is the default
        result = get_rigid_body_enabled("/World/Floor/thefloor")
        self.assertEqual(result, True)

        # Test set to False.
        set_rigid_body_enabled(False, "/World/Floor/thefloor")
        result = get_rigid_body_enabled("/World/Floor/thefloor")
        self.assertEqual(result, False)
