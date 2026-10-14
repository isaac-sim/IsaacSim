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

"""Test shape behavior."""

from typing import Any

import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Cube, Shape

from ..fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

N = 5

"""
Utility functions.
"""


def _get_display_colors(prims: Any, indices: Any = None) -> Any:
    """Read the authored display colors, one RGB triple per prim.

    The wrappers only write display colors, so the primvar is read back as a plain attribute: one array of
    colors per prim, which the setter always authors with a single entry.

    Args:
        prims: Wrapper to read from.
        indices: Indices of prims to read. If not defined, all wrapped prims are read.

    Returns:
        Display colors (shape ``(N, 3)``).
    """
    output = prims.get_attribute_values("primvars:displayColor", indices=indices)
    isaacsim_test.check_array(output, shape=(len(output), 1, 3), dtype=wp.float32)
    return output.numpy().reshape(-1, 3)


"""
Test cases.
"""


def test_display_colors(capsys: Any, stage: Any) -> None:
    """Test display colors.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # the base Shape class cannot be instantiated, so a concrete shape exercises the inherited methods
    prims = Cube([f"/World/Prim{i}" for i in range(N)])
    # a single color token is broadcast to all the prims
    prims.set_display_colors("red")
    isaacsim_test.check_allclose(np.tile([1.0, 0.0, 0.0], (N, 1)), _get_display_colors(prims))
    # a single grayscale component is broadcast to the three channels
    prims.set_display_colors(0.25)
    isaacsim_test.check_allclose(np.full((N, 3), 0.25), _get_display_colors(prims))
    # one color token per prim, in any of the supported string representations
    prims.set_display_colors(["#00ff00", "k", "tab:blue", "0.5", "aquamarine"])
    expected = np.array(
        [[0, 1, 0], [0, 0, 0], [0.12156863, 0.46666667, 0.70588235], [0.5, 0.5, 0.5], [0.49803922, 1, 0.83137255]]
    )
    isaacsim_test.check_allclose(expected, _get_display_colors(prims))
    # numeric RGB triples, one per prim
    values = np.linspace(0.0, 1.0, N * 3, dtype=np.float32).reshape(N, 3)
    prims.set_display_colors(values)
    isaacsim_test.check_allclose(values, _get_display_colors(prims))
    # only the selected prims are processed
    prims.set_display_colors("red", indices=[1, 3])
    expected = values.copy()
    expected[[1, 3]] = [1.0, 0.0, 0.0]
    isaacsim_test.check_allclose(expected, _get_display_colors(prims))
    isaacsim_test.check_allclose(np.tile([1.0, 0.0, 0.0], (2, 1)), _get_display_colors(prims, indices=[1, 3]))
    # a single grayscale component per prim spreads over the three channels
    grays = np.linspace(0.0, 1.0, N, dtype=np.float32).reshape(N, 1)
    prims.set_display_colors(grays)
    isaacsim_test.check_allclose(np.tile(grays, (1, 3)), _get_display_colors(prims))
    # unrecognized color tokens are rejected
    with pytest.raises(ValueError):
        prims.set_display_colors("not-a-color")
    # colors that match neither one nor the number of selected prims are rejected
    with pytest.raises(ValueError):
        prims.set_display_colors(["red", "green"])
    with pytest.raises(ValueError):
        prims.set_display_colors([])
    # colors that cannot be broadcast to one RGB triple per prim are rejected instead of being reinterpreted
    with pytest.raises(ValueError):
        prims.set_display_colors(np.zeros((3, N), dtype=np.float32))
    # an empty selection has no prim to color
    with pytest.raises(ValueError):
        prims.set_display_colors("red", indices=[])


def test_are_of_type(capsys: Any, stage: Any) -> None:
    """Test are of type.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    # the base Shape class matches any USD Gprim, which includes meshes as well as the concrete shapes
    type_names = ["Cube", "Mesh", "Camera", "Scope"]
    for index, type_name in enumerate(type_names):
        stage.define_prim(f"/World/Prim{index}", type_name)
    paths = [f"/World/Prim{index}" for index in range(len(type_names))]
    # boolean flags are reported per prim, in the order the paths were given
    output = Shape.are_of_type(paths)
    isaacsim_test.check_array(output, shape=(len(type_names), 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([1, 1, 0, 0], dtype=np.bool_).reshape(-1, 1), output)
    # regular expressions are expanded against the active stage
    isaacsim_test.check_array(Shape.are_of_type("/World/Prim.*"), shape=(len(type_names), 1), dtype=wp.bool)
    # non-existing prims
    with pytest.raises(RuntimeError):
        Shape.are_of_type("/World/NonExistent")


def test_legacy_display_color_getter(stage: Any) -> None:
    """Preserve the M1 placeholder call with and without the indices keyword.

    Args:
        stage: Stage used by the test.
    """
    prims = Cube(["/World/LegacyColor"])
    for result in (prims.get_display_colors(), prims.get_display_colors(indices=[0])):
        isaacsim_test.check_array(result, shape=(3,), dtype=wp.float64)
        np.testing.assert_array_equal(result.numpy(), np.zeros(3))
