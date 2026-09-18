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

"""Test light behavior."""

from typing import Any

import hypothesis
import hypothesis.extra.numpy
import isaacsim_test
import numpy as np
import pytest
import warp as wp

# the base Light class has no constructor binding; use a concrete SphereLight to exercise the shared API
from isaacsim.foundation.objects import Light as BaseLight
from isaacsim.foundation.objects import SphereLight as Light

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Utility functions.
"""


def _get_paths(stage: Any, populate: Any) -> Any:
    paths = [f"/World/Prim{i}" for i in range(5)]
    if populate:
        for path in paths:
            stage.define_prim(path, "SphereLight")
        paths = "/World/Prim.*"
    return paths


"""
Test cases.
"""


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_intensities(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test intensities.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_intensities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_intensities(values)
    output = prims.get_intensities()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_exposures(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test exposures.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_exposures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_exposures(values)
    output = prims.get_exposures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    diffuse=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
    specular=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_multipliers(capsys: Any, stage: Any, populate: Any, diffuse: Any, specular: Any) -> None:
    """Test multipliers.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        diffuse: Diffuse values.
        specular: Specular values.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_multipliers()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_multipliers(diffuse, specular)
    output = prims.get_multipliers()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose([diffuse, specular], output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_normalizations(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled normalizations.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_normalizations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_normalizations(values)
    output = prims.get_enabled_normalizations()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.bool_, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_enabled_color_temperatures(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test enabled color temperatures.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_enabled_color_temperatures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    # round-trip values
    prims.set_enabled_color_temperatures(values)
    output = prims.get_enabled_color_temperatures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.bool)
    isaacsim_test.check_equal(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 1)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_color_temperatures(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test color temperatures.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_color_temperatures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    # round-trip values
    prims.set_color_temperatures(values)
    output = prims.get_color_temperatures()
    isaacsim_test.check_array(output, shape=(5, 1), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


@hypothesis.given(
    values=hypothesis.extra.numpy.arrays(dtype=np.float32, shape=(5, 3)),
)
@pytest.mark.parametrize("populate", [True, False])
def test_colors(capsys: Any, stage: Any, populate: Any, values: Any) -> None:
    """Test colors.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        values: Values exercised by the test.
    """
    prims = Light(_get_paths(stage, populate))
    # get values
    output = prims.get_colors()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float32)
    # round-trip values
    prims.set_colors(values)
    output = prims.get_colors()
    isaacsim_test.check_array(output, shape=(5, 3), dtype=wp.float32)
    isaacsim_test.check_allclose(values, output)


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    type_names = ["SphereLight", "DistantLight", "Cube", "Scope"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = BaseLight.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(BaseLight.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        BaseLight.are_of_type("/World/NonExistent")
