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

"""Validate DomeLight wrapping and dome-specific USD attributes.

The suite authors existing dome light prims for wrap-mode tests, verifies type
detection and collection length, and round-trips guide radii, texture file
paths, and texture format tokens through indexed get/set calls.
"""

from typing import Any, Literal

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import DomeLight as TargetLight
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Create a fresh stage and author existing dome lights for wrap-mode tests.

    Args:
        max_num_prims: Maximum number of prims to prepare on the stage.
        operation: Operation mode selected by parametrization.
        **kwargs: Additional arguments supplied by parametrization.
    """
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="DomeLight",
                prim_path=f"/World/A_{i}",
                attributes={"inputs:intensity": 30000},
                select_new_prim=False,
            )


class TestDomeLight(omni.kit.test.AsyncTestCase):
    """Exercise DomeLight type checks and texture-related attributes."""

    async def setUp(self) -> None:
        """Initialize the async fixture; parametrized cases create their own stages."""
        super().setUp()

    async def tearDown(self) -> None:
        """Finalize the async fixture without additional light cleanup."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_are_of_type(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test are of type.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        self.assertFalse(TargetLight.are_of_type("/World").numpy().item())
        self.assertTrue(TargetLight.are_of_type("/World/A_0").numpy().item())
        self.assertTrue(TargetLight.are_of_type(["/World/A_0"]).numpy().item())
        self.assertTrue(TargetLight.are_of_type(prim_utils.get_prim_at_path("/World/A_0")).numpy().item())
        self.assertTrue(TargetLight.are_of_type([prim_utils.get_prim_at_path("/World/A_0")]).numpy().item())

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_guide_radii(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test guide radii.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_guide_radii(v0, indices=indices)
                output = prim.get_guide_radii(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_texture_files(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test texture files.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=["a", "bc", "def"]):
                prim.set_texture_files(v0, indices=indices)
                output = prim.get_texture_files(indices=indices)
                check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_texture_formats(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test texture formats.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        choices = ["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                prim.set_texture_formats(v0, indices=indices)
                output = prim.get_texture_formats(indices=indices)
                check_lists(expected_v0, output)
