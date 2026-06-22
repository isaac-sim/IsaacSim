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

"""Validate SphereLight wrapping and sphere-specific USD attributes.

The suite authors existing sphere light prims for wrap-mode tests, verifies
type detection and collection length, and round-trips radius and
treat-as-point state through indexed get/set calls.
"""

from typing import Any, Literal

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import SphereLight as TargetLight
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    cprint,
    draw_indices,
    draw_sample,
    parametrize,
)


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Create a fresh stage and author existing sphere lights for wrap-mode tests.

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
                prim_type="SphereLight",
                prim_path=f"/World/A_{i}",
                attributes={"inputs:intensity": 30000},
                select_new_prim=False,
            )


class TestSphereLight(omni.kit.test.AsyncTestCase):
    """Exercise SphereLight type checks and point-source attributes."""

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
    async def test_radii(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test radii.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_radii(v0, indices=indices)
                output = prim.get_radii(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_enabled_treat_as_points(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled treat as points.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                prim.set_enabled_treat_as_points(v0, indices=indices)
                output = prim.get_enabled_treat_as_points(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))
