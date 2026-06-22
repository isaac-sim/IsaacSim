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

"""Validate Camera wrapper behavior for authored USD camera attributes.

The suite creates and wraps camera prim collections through the USD backend,
then verifies indexed get/set round trips for focal settings, apertures,
clipping ranges, shutter timing, stereo roles, projection modes, and
square-pixel normalization.
"""

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Camera
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
    """Create a fresh stage and author existing camera prims for wrap-mode tests.

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
            stage_utils.define_prim(f"/World/A_{i}", "Camera")


class TestCamera(omni.kit.test.AsyncTestCase):
    """Exercise Camera collection length and optical attribute APIs."""

    async def setUp(self) -> None:
        """Initialize the async fixture; parametrized cases create their own stages."""
        super().setUp()

    async def tearDown(self) -> None:
        """Finalize the async fixture without additional camera-specific cleanup."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_focal_lengths(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test focal lengths.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_focal_lengths(v0, indices=indices)
                output = prim.get_focal_lengths(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_focus_distances(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test focus distances.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_focus_distances(v0, indices=indices)
                output = prim.get_focus_distances(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_stereo_roles(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test stereo roles.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        choices = ["mono", "left", "right"]
        # test cases
        # - check the stereo roles before applying any role
        roles = prim.get_stereo_roles()
        check_lists(["mono"] * num_prims, roles)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.set_stereo_roles(v0, indices=indices)
                output = prim.get_stereo_roles(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.set_stereo_roles(v0)
            output = prim.get_stereo_roles()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_fstops(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test fstops.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_fstops(v0, indices=indices)
                output = prim.get_fstops(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_apertures(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test apertures.

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
                prim.set_apertures(v0, v1, indices=indices)
                output = prim.get_apertures(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_aperture_offsets(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test aperture offsets.

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
                prim.set_aperture_offsets(v0, v1, indices=indices)
                output = prim.get_aperture_offsets(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_projections(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test projections.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        choices = ["perspective", "orthographic"]
        # test cases
        # - check the projections before applying any projection
        projections = prim.get_projections()
        check_lists(["perspective"] * num_prims, projections)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.set_projections(v0, indices=indices)
                output = prim.get_projections(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.set_projections(v0)
            output = prim.get_projections()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_clipping_ranges(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test clipping ranges.

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
                prim.set_clipping_ranges(v0, v1, indices=indices)
                output = prim.get_clipping_ranges(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_shutter_times(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test shutter times.

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
                prim.set_shutter_times(v0, v1, indices=indices)
                output = prim.get_shutter_times(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=Camera, populate_stage_func=populate_stage)
    async def test_enforce_square_pixels(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enforce square pixels.

        Args:
            prim: Object wrapper collection under test.
            num_prims: Number of prims in the parametrized collection.
            device: Device expected for returned arrays.
            backend: Backend name selected by parametrization.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            # set apertures
            prim.set_apertures(
                draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[np.ndarray])[2][0],
                draw_sample(shape=(num_prims, 1), dtype=wp.float32, types=[np.ndarray])[2][0],
            )
            original_horizontal_apertures, original_vertical_apertures = prim.get_apertures(indices=indices)
            original_horizontal_apertures = original_horizontal_apertures.numpy()
            original_vertical_apertures = original_vertical_apertures.numpy()
            # enforce square pixels
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 2), dtype=wp.uint32, low=100, high=1000),
                draw_choice(shape=(expected_count,), choices=["horizontal", "vertical"]),
            ):
                prim.enforce_square_pixels(v0, modes=v1, indices=indices)
                # check apertures
                horizontal_apertures, vertical_apertures = prim.get_apertures(indices=indices)
                horizontal_apertures = horizontal_apertures.numpy()
                vertical_apertures = vertical_apertures.numpy()
                for i, (resolution, mode) in enumerate(zip(expected_v0, expected_v1)):
                    aspect_ratio = (resolution[1] / float(resolution[0])).item()
                    if mode == "horizontal":
                        expected_vertical_aperture = horizontal_apertures[i].item() / aspect_ratio
                        self.assertAlmostEqual(
                            expected_vertical_aperture,
                            vertical_apertures[i].item(),
                            places=5,
                            msg=f"Invalid vertical aperture: expected {expected_vertical_aperture}, got {vertical_apertures[i].item()}",
                        )
                    elif mode == "vertical":
                        expected_horizontal_aperture = vertical_apertures[i].item() * aspect_ratio
                        self.assertAlmostEqual(
                            expected_horizontal_aperture,
                            horizontal_apertures[i].item(),
                            places=5,
                            msg=f"Invalid horizontal aperture: expected {expected_horizontal_aperture}, got {horizontal_apertures[i].item()}",
                        )
                    else:
                        raise ValueError(f"Invalid mode: {mode}. Valid modes are 'horizontal' and 'vertical'")
