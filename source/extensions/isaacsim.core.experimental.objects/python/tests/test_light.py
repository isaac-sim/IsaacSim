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

"""Validate behavior shared by all supported USD light wrappers.

The suite checks Light instance dispatch across concrete light types and
verifies indexed USD-backend round trips for intensity, exposure, shaping
multipliers, normalization flags, color-temperature controls, and color
normalization.
"""

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import (
    CylinderLight,
    DiskLight,
    DistantLight,
    DomeLight,
    Light,
    RectLight,
    SphereLight,
)
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)

TargetLight = SphereLight


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


class TestLight(omni.kit.test.AsyncTestCase):
    """Exercise shared Light APIs through a representative SphereLight collection."""

    async def setUp(self) -> None:
        """Initialize the async fixture; parametrized cases create their own stages."""
        super().setUp()

    async def tearDown(self) -> None:
        """Finalize the async fixture without additional light cleanup."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_fetch_instances(self) -> None:
        """Test fetch instances."""
        await stage_utils.create_new_stage_async()
        # create lights
        CylinderLight("/World/light_01")
        DiskLight("/World/light_02")
        DistantLight("/World/light_03")
        DomeLight("/World/light_04")
        RectLight("/World/light_05")
        SphereLight("/World/light_06")
        # fetch instances
        instances = Light.fetch_instances(
            [
                "/World",
                "/World/light_01",
                "/World/light_02",
                "/World/light_03",
                "/World/light_04",
                "/World/light_05",
                "/World/light_06",
            ]
        )
        # check
        self.assertEqual(len(instances), 7)
        self.assertIsNone(instances[0])
        self.assertIsInstance(instances[1], CylinderLight)
        self.assertIsInstance(instances[2], DiskLight)
        self.assertIsInstance(instances[3], DistantLight)
        self.assertIsInstance(instances[4], DomeLight)
        self.assertIsInstance(instances[5], RectLight)
        self.assertIsInstance(instances[6], SphereLight)

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_intensities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test intensities.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_intensities(v0, indices=indices)
                output = prim.get_intensities(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_exposures(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test exposures.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_exposures(v0, indices=indices)
                output = prim.get_exposures(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_multipliers(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test multipliers.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
            ):
                prim.set_multipliers(v0, v1, indices=indices)
                output = prim.get_multipliers(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_enabled_normalizations(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled normalizations.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                prim.set_enabled_normalizations(v0, indices=indices)
                output = prim.get_enabled_normalizations(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_enabled_color_temperatures(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled color temperatures.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                prim.set_enabled_color_temperatures(v0, indices=indices)
                output = prim.get_enabled_color_temperatures(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_color_temperatures(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test color temperatures.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        prim.set_enabled_color_temperatures([True])  # enable use of color temperatures
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_color_temperatures(v0, indices=indices)
                output = prim.get_color_temperatures(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=TargetLight, populate_stage_func=populate_stage)
    async def test_colors(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test colors.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
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
            for v0, expected_v0 in draw_sample(shape=(expected_count, 3), dtype=wp.float32):
                prim.set_colors(v0, indices=indices)
                output = prim.get_colors(indices=indices)
                check_array(output, shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                prim.set_colors(v0, indices=indices)
