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

"""Test for shape."""

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
from isaacsim.core.experimental.objects import Capsule, Cone, Cube, Cylinder, Plane, Shape, Sphere
from isaacsim.core.experimental.prims.tests.common import cprint, draw_choice, draw_indices, parametrize


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs) -> None:
    """Populate stage."""
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            stage_utils.define_prim(f"/World/A_{i}", "Sphere")


class TestShape(omni.kit.test.AsyncTestCase):
    """Test shape."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_fetch_instances(self):
        """Test fetch instances."""
        await stage_utils.create_new_stage_async()
        # create shapes
        Capsule("/World/shape_01")
        Cone("/World/shape_02")
        Cube("/World/shape_03")
        Cylinder("/World/shape_04")
        Plane("/World/shape_05")
        Sphere("/World/shape_06")
        # fetch instances
        instances = Shape.fetch_instances(
            [
                "/World",
                "/World/shape_01",
                "/World/shape_02",
                "/World/shape_03",
                "/World/shape_04",
                "/World/shape_05",
                "/World/shape_06",
            ]
        )
        # check
        self.assertEqual(len(instances), 7)
        self.assertIsNone(instances[0])
        self.assertIsInstance(instances[1], Capsule)
        self.assertIsInstance(instances[2], Cone)
        self.assertIsInstance(instances[3], Cube)
        self.assertIsInstance(instances[4], Cylinder)
        self.assertIsInstance(instances[5], Plane)
        self.assertIsInstance(instances[6], Sphere)

    @parametrize(backends=["usd"], prim_class=Sphere, populate_stage_func=populate_stage)
    async def test_display_colors(self, prim, num_prims, device, backend):
        """Test display colors."""
        choices = [
            (0.1, 0.2, 0.3),  # RGB tuple
            "#aBc",  # case-insensitive short hex RGB
            "#0A1b2C",  # case-insensitive hex RGB
            "0.5",  # grayscale
            "k",  # basic color
            "AquaMarine",  # case-insensitive X11/CSS4 color with no spaces
            "xkcd:eggShell",  # case-insensitive  xkcd color
            "tab:Green",  # case-insensitive tableau color
            "C2",  # CN color specification
            "none",  # special value (fully transparent)
        ]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                prim.set_display_colors(v0, indices=indices)
