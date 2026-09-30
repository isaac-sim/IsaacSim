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

"""Test prim behavior."""

from typing import Any

import isaacsim.foundation.utils.prim as prim_utils
import pytest
from isaacsim.foundation.objects import Prim

from .fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture


def get_prim_type_name(path: str) -> str:
    """Get prim type name.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    return Prim([path]).get_type_name()[0]


def test_find_matching_prim_paths(capsys: Any, stage: Any) -> None:
    """Test find matching prim paths.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/A")
    for i in range(2):
        stage.define_prim(f"/World/A{i}")
        stage.define_prim(f"/World/A{i}/B")
        for j in range(3):
            stage.define_prim(f"/World/A{i}/B{j}")
            stage.define_prim(f"/World/A{i}/B{j}/C")
    # test cases
    # - valid prim path
    match = ["/World/A0/B0"]
    assert prim_utils.find_matching_prim_paths("/World/A0/B0") == match
    match = ["/World/A0/B0", "/World/A0/B0/C"]
    assert prim_utils.find_matching_prim_paths("/World/A0/B0", traverse=True) == match
    # - regex
    # --
    match = ["/World/A0/B0", "/World/A0/B1", "/World/A0/B2"]
    assert prim_utils.find_matching_prim_paths("/World/A0/B[0-9]") == match
    match = [
        "/World/A0/B0",
        "/World/A0/B0/C",
        "/World/A0/B1",
        "/World/A0/B1/C",
        "/World/A0/B2",
        "/World/A0/B2/C",
    ]
    assert prim_utils.find_matching_prim_paths("/World/A0/B[0-9]", traverse=True) == match
    # --
    match = ["/World/A0/B1", "/World/A0/B2", "/World/A1/B1", "/World/A1/B2"]
    assert prim_utils.find_matching_prim_paths("/World/.*/.*[1,2]") == match
    match = [
        "/World/A0/B1",
        "/World/A0/B1/C",
        "/World/A0/B2",
        "/World/A0/B2/C",
        "/World/A1/B1",
        "/World/A1/B1/C",
        "/World/A1/B2",
        "/World/A1/B2/C",
    ]
    assert prim_utils.find_matching_prim_paths("/World/.*/.*[1,2]", traverse=True) == match
    # --
    match = []
    assert prim_utils.find_matching_prim_paths(".*C.*") == []
    match = [
        "/World/A0/B0/C",
        "/World/A0/B1/C",
        "/World/A0/B2/C",
        "/World/A1/B0/C",
        "/World/A1/B1/C",
        "/World/A1/B2/C",
    ]
    assert prim_utils.find_matching_prim_paths(".*C.*", traverse=True) == match


def test_get_all_matching_child_prims(capsys: Any, stage: Any) -> None:
    """Test get all matching child prims.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World")
    stage.define_prim("/World/A0", "Sphere")
    for i in range(3):
        stage.define_prim(f"/World/A0/B{i}", "Cube" if i % 2 else "Sphere")
    for i in range(3):
        stage.define_prim(f"/World/A0/B0/C{i}", "Cube" if i % 2 else "Sphere")
    # test cases
    # - valid case
    predicate = lambda path: get_prim_type_name(path) == "Sphere"
    # -- max_depth: None
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate)
    assert children == ["/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"]
    # -- max_depth: 0
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=0)
    assert children == []
    # -- max_depth: 1
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=1)
    assert children == ["/World/A0/B0", "/World/A0/B2"]
    # -- max_depth: 2
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=2)
    assert children == ["/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"]
    # - self-include
    # -- max_depth: None
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, include_self=True)
    assert children == ["/World/A0", "/World/A0/B0", "/World/A0/B2", "/World/A0/B0/C0", "/World/A0/B0/C2"]
    # -- max_depth: 0
    children = prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, include_self=True, max_depth=0)
    assert children == ["/World/A0"]
    # exceptions
    with pytest.raises(TypeError):  # max_depth is defined as size_t in C++
        prim_utils.get_all_matching_child_prims("/World/A0", predicate=predicate, max_depth=-1)


def test_get_first_matching_child_prim(capsys: Any, stage: Any) -> None:
    """Test get first matching child prim.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World")
    stage.define_prim("/World/A")
    for i in range(5):
        stage.define_prim(f"/World/A/B{i}", "Cube" if i % 2 else "Sphere")
    # Sphere is only reachable two levels below /World/D0
    stage.define_prim("/World/D0", "Cube")
    stage.define_prim("/World/D0/E0", "Cube")
    stage.define_prim("/World/D0/E0/F0", "Sphere")
    # test cases
    # - valid case
    predicate = lambda path: get_prim_type_name(path) == "Sphere"
    child = prim_utils.get_first_matching_child_prim("/", predicate=predicate, include_self=True)
    assert child == "/World/A/B0"
    # - no match
    assert prim_utils.get_first_matching_child_prim("/World/A", predicate=lambda *_: False) is None
    # - max_depth
    # -- max_depth: None
    child = prim_utils.get_first_matching_child_prim("/World/D0", predicate=predicate)
    assert child == "/World/D0/E0/F0"
    # -- max_depth: 0
    assert prim_utils.get_first_matching_child_prim("/World/D0", predicate=predicate, max_depth=0) is None
    # -- max_depth: 1
    assert prim_utils.get_first_matching_child_prim("/World/D0", predicate=predicate, max_depth=1) is None
    # -- max_depth: 2
    child = prim_utils.get_first_matching_child_prim("/World/D0", predicate=predicate, max_depth=2)
    assert child == "/World/D0/E0/F0"
    # -- include_self, max_depth: 0
    child = prim_utils.get_first_matching_child_prim(
        "/World/D0/E0/F0", predicate=predicate, include_self=True, max_depth=0
    )
    assert child == "/World/D0/E0/F0"
    # - self-include
    predicate = lambda path: get_prim_type_name(path) == "Xform"
    # -- include self
    child = prim_utils.get_first_matching_child_prim("/World", predicate=predicate, include_self=True)
    assert child == "/World"
    # -- exclude self
    child = prim_utils.get_first_matching_child_prim("/World", predicate=predicate, include_self=False)
    assert child == "/World/A"
    # exceptions
    with pytest.raises(TypeError):  # max_depth is defined as size_t in C++
        prim_utils.get_first_matching_child_prim("/World/A", predicate=predicate, max_depth=-1)


def test_get_first_matching_parent_prim(capsys: Any, stage: Any) -> None:
    """Test get first matching parent prim.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World")
    stage.define_prim("/World/Cube", "Cube")
    stage.define_prim("/World/Cube/Sphere", "Sphere")
    # test cases
    # - valid case
    predicate = lambda path: get_prim_type_name(path) == "Xform"
    parent = prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate)
    assert parent == "/World"
    # - no match
    assert prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=lambda *_: False) is None
    # - root prim (pseudo-root prim)
    predicate = lambda path: path == "/"
    assert prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate) is None
    # - self-include
    predicate = lambda path: get_prim_type_name(path) == "Sphere"
    # -- include self
    parent = prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate, include_self=True)
    assert parent == "/World/Cube/Sphere"
    # -- exclude self
    assert (
        prim_utils.get_first_matching_parent_prim("/World/Cube/Sphere", predicate=predicate, include_self=False) is None
    )
