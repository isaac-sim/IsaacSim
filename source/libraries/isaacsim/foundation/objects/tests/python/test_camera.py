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

"""Test camera behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Camera

from .fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_paths(stage: Any, populate: Any) -> Any:
    paths = [f"/World/Prim{i}" for i in range(5)]
    if populate:
        for path in paths:
            stage.define_prim(path, "Camera")
        paths = "/World/Prim.*"
    return paths


"""
Test cases.
"""


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_focal_lengths(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test focal lengths.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_focal_lengths()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_focal_lengths(values)
    output = prims.get_focal_lengths()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_focus_distances(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test focus distances.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_focus_distances()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_focus_distances(values)
    output = prims.get_focus_distances()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_fstops(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test fstops.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_fstops()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_fstops(values)
    output = prims.get_fstops()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.strategies.lists(
        hypothesis.strategies.sampled_from(["mono", "left", "right"]), min_size=5, max_size=5
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_stereo_roles(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test stereo roles.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_stereo_roles()
    assert isinstance(output, list) and len(output) == 5
    # round-trip values
    prims.set_stereo_roles(values)
    output = prims.get_stereo_roles()
    isaacsim_test.check_equal(output, values)


@hypothesis.given(
    horizontal=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
    vertical=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_apertures(capsys: Any, stage: Any, populate: Any, horizontal: Any, vertical: Any) -> None:
    """Test apertures.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        horizontal: Horizontal values.
        vertical: Vertical values.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_apertures()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_apertures(horizontal, vertical)
    output = prims.get_apertures()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose([horizontal, vertical], list(output))


@hypothesis.given(
    horizontal=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
    vertical=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_aperture_offsets(capsys: Any, stage: Any, populate: Any, horizontal: Any, vertical: Any) -> None:
    """Test aperture offsets.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        horizontal: Horizontal values.
        vertical: Vertical values.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_aperture_offsets()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_aperture_offsets(horizontal, vertical)
    output = prims.get_aperture_offsets()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose([horizontal, vertical], list(output))


@hypothesis.given(
    projections=hypothesis.strategies.lists(
        hypothesis.strategies.sampled_from(["perspective", "orthographic"]), min_size=5, max_size=5
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_projections(capsys: Any, stage: Any, populate: Any, projections: Any) -> None:
    """Test projections.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        projections: Projections values.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_projections()
    assert isinstance(output, list) and len(output) == 5
    # round-trip values
    prims.set_projections(projections)
    output = prims.get_projections()
    isaacsim_test.check_equal(output, projections)


@hypothesis.given(
    near=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(min_value=0.01, max_value=1.0),
    ),
    far=hypothesis.extra.numpy.arrays(
        dtype=np.float32,
        shape=(5, 1),
        elements=isaacsim_test.finite_float_elements(min_value=10.0, max_value=10000.0),
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_clipping_ranges(capsys: Any, stage: Any, populate: Any, near: Any, far: Any) -> None:
    """Test clipping ranges.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        near: Near values.
        far: Far values.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_clipping_ranges()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_clipping_ranges(near, far)
    output = prims.get_clipping_ranges()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose([near, far], list(output))


@hypothesis.given(
    open_times=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
    close_times=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_shutter_times(capsys: Any, stage: Any, populate: Any, open_times: Any, close_times: Any) -> None:
    """Test shutter times.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        open_times: Open times in seconds.
        close_times: Close times in seconds.
    """
    prims = Camera(_get_paths(stage, populate))
    # get values
    output = prims.get_shutter_times()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    # round-trip values
    prims.set_shutter_times(open_times, close_times)
    output = prims.get_shutter_times()
    isaacsim_test.check_array(list(output), shape=(5, 1), dtype=wp.float64)
    isaacsim_test.check_allclose([open_times, close_times], list(output))


@pytest.mark.parametrize("populate", [True, False])
@pytest.mark.parametrize("mode", ["horizontal", "vertical"])
def test_enforce_square_pixels(capsys: Any, stage: Any, populate: Any, mode: Any) -> None:
    """Test enforce square pixels.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        mode: Operating mode.
    """
    prims = Camera(_get_paths(stage, populate))
    # 1920x1080 - aspect ratio width/height = 16/9
    prims.set_apertures(np.full((5, 1), 36.0), np.full((5, 1), 24.0))
    prims.enforce_square_pixels(np.full((5, 2), [1080.0, 1920.0]), modes=mode)
    horizontal, vertical = prims.get_apertures()
    if mode == "horizontal":
        # vertical adjusted: horizontal / aspect = 36 / (16/9) = 20.25
        isaacsim_test.check_allclose(horizontal.numpy(), np.full((5, 1), 36.0))
        isaacsim_test.check_allclose(vertical.numpy(), np.full((5, 1), 20.25))
    else:
        # horizontal adjusted: vertical * aspect = 24 * (16/9) ≈ 42.667
        isaacsim_test.check_allclose(vertical.numpy(), np.full((5, 1), 24.0))
        isaacsim_test.check_allclose(horizontal.numpy(), np.full((5, 1), 128.0 / 3.0))


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    type_names = ["Camera", "Xform", "Scope"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = Camera.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(Camera.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        Camera.are_of_type("/World/NonExistent")
