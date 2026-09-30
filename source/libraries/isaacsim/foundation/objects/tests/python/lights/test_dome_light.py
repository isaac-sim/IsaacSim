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

"""Test dome light behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import hypothesis.strategies
import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import DomeLight as Light

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_paths(stage: Any, populate: Any) -> Any:
    paths = [f"/World/Prim{i}" for i in range(5)]
    if populate:
        for path in paths:
            stage.define_prim(path, "DomeLight")
        paths = "/World/Prim.*"
    return paths


"""
Test cases.
"""


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_guide_radii(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test guide radii.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_guide_radii()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_guide_radii(values)
    output = prims.get_guide_radii()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.strategies.lists(
        hypothesis.strategies.text(alphabet="abcdefghijklmnopqrstuvwxyz", max_size=8),
        min_size=5,
        max_size=5,
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_texture_files(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test texture files.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    prims.get_texture_files()
    # round-trip values
    prims.set_texture_files(values)
    output = prims.get_texture_files()
    isaacsim_test.check_equal(output, values)


@hypothesis.given(
    values=hypothesis.strategies.lists(
        hypothesis.strategies.sampled_from(["automatic", "latlong", "mirroredBall", "angular", "cubeMapVerticalCross"]),
        min_size=5,
        max_size=5,
    ),
)
@pytest.mark.parametrize("populate", [True, False])
def test_texture_formats(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test texture formats.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    prims.get_texture_formats()
    # round-trip values
    prims.set_texture_formats(values)
    output = prims.get_texture_formats()
    isaacsim_test.check_equal(output, values)


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    type_names = ["DomeLight", "SphereLight", "Scope"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = Light.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(Light.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        Light.are_of_type("/World/NonExistent")
