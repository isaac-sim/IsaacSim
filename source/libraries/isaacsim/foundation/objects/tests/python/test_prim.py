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

import isaacsim_test
import numpy as np
import pytest
import warp as wp
from isaacsim.foundation.objects import Prim

from .fixtures import stage  # noqa: F401 - imported so pytest can discover the fixture

"""
Test cases.
"""


def test_get_stage(capsys: Any, stage: Any) -> None:
    """Test get stage.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    assert Prim([], resolve_paths=False).get_stage().get_stage_id() == stage.get_stage_id()


def test_paths(capsys: Any, stage: Any) -> None:
    """Test paths.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/A", "Xform")
    stage.define_prim("/World/B", "Xform")
    # single prim
    assert Prim("/World/A").paths == ["/World/A"]
    # multiple prims
    assert Prim(["/World/A", "/World/B"]).paths == ["/World/A", "/World/B"]
    # unresolved paths (non-existing prims are kept as-is)
    assert Prim("/World/C", resolve_paths=False).paths == ["/World/C"]
    # property is read-only
    with pytest.raises(AttributeError):
        Prim("/World/A").paths = ["/World/B"]


def test_resolve_paths(capsys: Any, stage: Any) -> None:
    """Test resolve paths.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    for i in range(3):
        stage.define_prim(f"/World/A_{i}", "Xform")
        stage.define_prim(f"/World/A_{i}/B", "Cube")
    # valid cases
    # - single path
    existing_paths, nonexistent_paths = Prim.resolve_paths("/World/A_0")
    assert len(existing_paths) == 1 and len(nonexistent_paths) == 0
    # - single path (non-existing)
    existing_paths, nonexistent_paths = Prim.resolve_paths("/World/C")
    assert len(existing_paths) == 0 and len(nonexistent_paths) == 1
    # - regex
    existing_paths, nonexistent_paths = Prim.resolve_paths("/World/A_.*/B")
    assert len(existing_paths) == 3 and len(nonexistent_paths) == 0
    # exceptions
    # - mixed paths
    with pytest.raises(RuntimeError):
        Prim.resolve_paths(["/World/A_.*", "/World/C"])
    # - no existing or non-existing paths exist
    with pytest.raises(RuntimeError):
        Prim.resolve_paths("/World/A_.*/C")
    # - incomplete existing paths
    with pytest.raises(RuntimeError):
        Prim.resolve_paths(["/World/A_.*/B", "/World/A_.*/C"])
    # - incomplete non-existing paths
    with pytest.raises(RuntimeError):
        Prim.resolve_paths(["/World/C", "/World/A_.*/C"])


def test_get_name(capsys: Any, stage: Any) -> None:
    """Test get name.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/A/B")
    # single prim
    assert Prim("/World/A/B").get_name() == ["B"]
    assert Prim("/World/A").get_name() == ["A"]
    # multiple prims
    assert Prim(["/World/A/B", "/World/A"]).get_name() == ["B", "A"]


def test_get_type_name(capsys: Any, stage: Any) -> None:
    """Test get type name.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Cube", "Cube")
    stage.define_prim("/World/Xform", "Xform")
    # single prim
    assert Prim("/World/Cube").get_type_name() == ["Cube"]
    assert Prim("/World/Xform").get_type_name() == ["Xform"]
    # multiple prims
    assert Prim(["/World/Cube", "/World/Xform"]).get_type_name() == ["Cube", "Xform"]


def test_get_parent(capsys: Any, stage: Any) -> None:
    """Test get parent.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/A/B/C")
    # single prim
    assert Prim("/World/A/B/C").get_parent() == ["/World/A/B"]
    assert Prim("/World/A").get_parent() == ["/World"]
    # multiple prims
    assert Prim(["/World/A/B/C", "/World/A", "/"]).get_parent() == ["/World/A/B", "/World", ""]


def test_get_children(capsys: Any, stage: Any) -> None:
    """Test get children.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/A/X")
    stage.define_prim("/World/A/Y")
    stage.define_prim("/World/B")
    # single prim with children
    assert Prim("/World/A").get_children() == [["/World/A/X", "/World/A/Y"]]
    # leaf prim has no children
    assert Prim("/World/B").get_children() == [[]]
    # multiple prims
    assert Prim(["/World", "/World/B"]).get_children() == [["/World/A", "/World/B"], []]


@pytest.mark.parametrize("stage", [{"usd_path": isaacsim_test.resolve_resource_path("variant.usda")}], indirect=True)
def test_variants(capsys: Any, stage: Any) -> None:
    # get variant sets
    # - single prim
    """Test variants.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    result = Prim("/World/Sphere").get_variant_sets()
    assert len(result) == 1
    assert set(result[0].keys()) == {"color", "radius"}
    assert result[0]["color"] == ["blue", "red"]
    assert result[0]["radius"] == ["large", "small"]
    # - multiple prims
    result = Prim(["/World/Sphere", "/World/Sphere"]).get_variant_sets()
    assert len(result) == 2
    for entry in result:
        assert set(entry.keys()) == {"color", "radius"}
        assert entry["color"] == ["blue", "red"]
        assert entry["radius"] == ["large", "small"]

    # get variant selection
    # - single prim
    result = Prim("/World/Sphere").get_variant_selection()
    assert len(result) == 1
    assert set(result[0].keys()) == {"color", "radius"}
    assert result[0]["color"] == "red"
    assert result[0]["radius"] == "small"
    # - multiple prims
    result = Prim(["/World/Sphere", "/World/Sphere"]).get_variant_selection()
    assert len(result) == 2
    for entry in result:
        assert set(entry.keys()) == {"color", "radius"}
        assert entry["color"] == "red"
        assert entry["radius"] == "small"

    # set variant selection
    prim = Prim("/World/Sphere")
    prim.set_variant_selection({"color": "blue", "radius": "large"})
    result = prim.get_variant_selection()
    assert result[0]["color"] == "blue"
    assert result[0]["radius"] == "large"

    # set variant selection: invalid variant set/selection
    prim = Prim("/World/Sphere")
    with pytest.raises(ValueError):
        prim.set_variant_selection({"NonExistent": "red"})
    with pytest.raises(ValueError):
        prim.set_variant_selection({"radius": "Invalid"})


def test_get_applied_schemas(capsys: Any, stage: Any) -> None:
    """Test get applied schemas.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Cube", "Cube")
    stage.define_prim("/World/Sphere", "Sphere")

    # no applied schemas
    assert Prim("/World/Cube").get_applied_schemas() == [[]]

    # apply schemas and get them
    prim = Prim("/World/Cube")
    prim.apply_api("PhysicsRigidBodyAPI")
    prim.apply_api("PhysicsDriveAPI", instance_name="linear")
    prim.apply_api("PhysicsDriveAPI", instance_name="angular")
    result = prim.get_applied_schemas()
    assert len(result) == 1 and len(result[0]) == 3
    assert "PhysicsRigidBodyAPI" in result[0]
    assert "PhysicsDriveAPI:linear" in result[0]
    assert "PhysicsDriveAPI:angular" in result[0]

    # multiple prims
    prims = Prim(["/World/Cube", "/World/Sphere"])
    result = prims.get_applied_schemas()
    assert len(result) == 2
    assert len(result[0]) == 3 and len(result[1]) == 0


def test_is_a(capsys: Any, stage: Any) -> None:
    """Test is a.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Cube", "Cube")
    stage.define_prim("/World/Sphere", "Sphere")

    prim = Prim("/World/Cube")
    prims = Prim(["/World/Cube", "/World/Sphere"])

    # flags are reported per prim, one row each
    output = prim.is_a("Cube")
    isaacsim_test.check_array(output, shape=(1, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[1]], dtype=np.bool_), output)

    output = prims.is_a("Cube")
    isaacsim_test.check_array(output, shape=(2, 1), dtype=wp.bool)
    isaacsim_test.check_equal(np.array([[1], [0]], dtype=np.bool_), output)

    # the abstract base schema matches both
    isaacsim_test.check_equal(np.array([[1], [1]], dtype=np.bool_), prims.is_a("Gprim"))

    # subset of prims
    isaacsim_test.check_array(prims.is_a("Cube", indices=[1]), shape=(1, 1), dtype=wp.bool)

    # unknown schema type
    with pytest.raises(ValueError):
        prim.is_a("NonExistentType")


def test_api(capsys: Any, stage: Any) -> None:
    """Test api.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
    """
    stage.define_prim("/World/Cube", "Cube")
    stage.define_prim("/World/Sphere", "Sphere")

    prim = Prim("/World/Cube")
    prims = Prim(["/World/Cube", "/World/Sphere"])

    # flags are reported per prim, one row each
    isaacsim_test.check_array(prim.has_api("PhysicsRigidBodyAPI"), shape=(1, 1), dtype=wp.bool)
    isaacsim_test.check_array(prims.has_api("PhysicsRigidBodyAPI"), shape=(2, 1), dtype=wp.bool)
    isaacsim_test.check_array(prims.apply_api("PhysicsRigidBodyAPI"), shape=(2, 1), dtype=wp.bool)
    isaacsim_test.check_array(prims.remove_api("PhysicsRigidBodyAPI"), shape=(2, 1), dtype=wp.bool)

    # single-apply API: PhysicsRigidBodyAPI
    has_api = prim.has_api("PhysicsRigidBodyAPI")
    isaacsim_test.check_array(has_api, shape=(1, 1), dtype=wp.bool)
    assert not any(has_api.numpy())
    assert all(prim.apply_api("PhysicsRigidBodyAPI").numpy())
    assert all(prim.has_api("PhysicsRigidBodyAPI").numpy())
    assert all(prim.remove_api("PhysicsRigidBodyAPI").numpy())
    assert not any(prim.has_api("PhysicsRigidBodyAPI").numpy())

    assert not any(prims.has_api("PhysicsRigidBodyAPI").numpy())
    assert all(prims.apply_api("PhysicsRigidBodyAPI").numpy())
    assert all(prims.has_api("PhysicsRigidBodyAPI").numpy())
    assert all(prims.remove_api("PhysicsRigidBodyAPI").numpy())
    assert not any(prims.has_api("PhysicsRigidBodyAPI").numpy())

    # multi-apply API: PhysicsDriveAPI
    assert not any(prim.has_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert all(prim.apply_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert all(prim.has_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert not any(prim.has_api("PhysicsDriveAPI", instance_name="angular").numpy())
    assert all(prim.remove_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert not any(prim.has_api("PhysicsDriveAPI", instance_name="linear").numpy())

    assert not any(prims.has_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert all(prims.apply_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert all(prims.has_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert not any(prims.has_api("PhysicsDriveAPI", instance_name="angular").numpy())
    assert all(prims.remove_api("PhysicsDriveAPI", instance_name="linear").numpy())
    assert not any(prims.has_api("PhysicsDriveAPI", instance_name="linear").numpy())

    # unknown schema type
    with pytest.raises(ValueError):
        prim.has_api("NonExistentAPI")
    with pytest.raises(ValueError):
        prim.apply_api("NonExistentAPI")
    with pytest.raises(ValueError):
        prim.remove_api("NonExistentAPI")


@pytest.mark.parametrize("populate", [True, False])
@pytest.mark.parametrize("indices", [True, False])
def test_attribute_values(capsys: Any, stage: Any, populate: Any, indices: Any) -> None:
    """Test attribute values.

    Args:
        capsys: Pytest output-capture fixture.
        stage: Stage used by the test.
        populate: Whether to populate the test stage.
        indices: Indices values.
    """
    paths = [f"/World/Prim{i}" for i in range(5)]
    if populate:
        for path in paths:
            stage.define_prim(path, "Cube")
        paths = "/World/Prim.*"

    if not populate:
        with pytest.raises(RuntimeError):
            Prim(paths)
        return

    prim = Prim(paths)
    values = prim.get_attribute_values("size")
    prim.set_attribute_values("size", values)
    values = prim.get_attribute_values("size")
