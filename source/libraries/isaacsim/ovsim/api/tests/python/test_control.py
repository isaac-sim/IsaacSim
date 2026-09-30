# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test control behavior."""

from typing import Any

import isaacsim_test
import pytest
from isaacsim.ovsim.api import make_client

client = make_client("local")
authoring = client.control.authoring
simulation = client.control.simulation


def test_make_grpc_client_from_dict() -> None:
    """Test creating a gRPC client from a configuration dictionary."""
    remote = make_client("grpc", {"endpoint": "127.0.0.1:1"})
    assert remote.control is not None


"""
Stage operations.
"""


def test_create_stage() -> None:
    """Test create stage."""
    assert authoring.create_stage()
    assert authoring.close_stage()


def test_open_stage() -> None:
    """Test open stage."""
    usd_path = isaacsim_test.resolve_resource_path("variant.usda")
    assert authoring.open_stage(usd_path)
    assert authoring.close_stage()


def test_save_stage(tmp_path: Any) -> None:
    """Test save stage.

    Args:
        tmp_path: Temporary directory supplied by pytest.
    """
    output_path = str(tmp_path / "saved.usda")
    assert authoring.create_stage()
    assert authoring.define_prim("/World", "Xform")
    assert authoring.save_stage(output_path)
    assert authoring.close_stage()


def test_close_stage() -> None:
    """Test close stage."""
    assert authoring.create_stage()
    assert authoring.close_stage()
    with pytest.raises(RuntimeError):
        assert authoring.close_stage()


def test_add_reference_to_stage() -> None:
    """Test add reference to stage."""
    usd_path = isaacsim_test.resolve_resource_path("variant.usda")
    assert authoring.create_stage()
    assert authoring.define_prim("/World/Prim")
    assert authoring.add_reference_to_stage(usd_path, "/World/Prim")
    assert authoring.close_stage()


"""
Prim operations.
"""


def test_define_remove_prim() -> None:
    """Test define remove prim."""
    assert authoring.create_stage()

    assert authoring.define_prim("/World/A", "Xform")
    assert authoring.remove_prim("/World/A")
    with pytest.raises(Exception):
        authoring.remove_prim("/World/A")

    assert authoring.close_stage()


def test_move_prim() -> None:
    """Test move prim."""
    assert authoring.create_stage()

    assert authoring.define_prim("/World/A", "Xform")
    assert authoring.move_prim("/World/A", "/World/B")
    assert authoring.remove_prim("/World/B")
    with pytest.raises(Exception):
        authoring.remove_prim("/World/A")

    assert authoring.close_stage()


def test_create_remove_prim_attribute() -> None:
    """Test create remove prim attribute."""
    assert authoring.create_stage()

    assert authoring.define_prim("/World/A", "Xform")
    assert authoring.create_prim_attribute("/World/A", "myAttr", "float")
    assert authoring.remove_prim_attribute("/World/A", "myAttr")

    assert authoring.close_stage()


def test_create_prim_attribute_invalid_type() -> None:
    """Test create prim attribute invalid type."""
    assert authoring.create_stage()

    assert authoring.define_prim("/World/A", "Xform")
    with pytest.raises(Exception):
        authoring.create_prim_attribute("/World/A", "myAttr", "nonExistentType")

    assert authoring.close_stage()


def test_remove_prim_attribute_missing() -> None:
    """Test remove prim attribute missing."""
    assert authoring.create_stage()

    assert authoring.define_prim("/World/A", "Xform")
    with pytest.raises(Exception):
        authoring.remove_prim_attribute("/World/A", "nonExistentAttr")

    assert authoring.close_stage()


"""
Simulation operations.
"""


def test_simulation_no_ops() -> None:
    """Test simulation no ops."""
    simulation.play()
    simulation.pause()
    simulation.stop()
