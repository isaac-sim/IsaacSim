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

"""Test data behavior."""

import isaacsim.foundation.ovsim as ovsim
import pytest

data = ovsim.data
authoring = ovsim.control.authoring


def test_read_write_float_attribute() -> None:
    """Test read write float attribute."""
    assert authoring.create_stage()

    path = "/World/Cube"
    assert authoring.define_prim(path, "Cube")

    assert data.read(path, "size").numpy().item() == 2.0
    data.write(path, "size", 5.0)
    assert data.read(path, "size").numpy().item() == 5.0

    # TODO: Update when display color is supported
    assert data.read(path, "color").list() == []
    data.write(path, "color", "red")
    assert data.read(path, "color").list() == []

    assert authoring.close_stage()


def test_instance_dispatch() -> None:
    """Test that prims are dispatched to the wrapper matching their type."""
    assert authoring.create_stage()

    # a ground plane carries no physics API of its own and is identified by its child structure
    ground_plane = "/World/GroundPlane"
    assert authoring.define_prim(ground_plane, "GroundPlane")
    data.write(ground_plane, "position", [1.0, 2.0, 3.0])
    assert data.read(ground_plane, "position").numpy().flatten().tolist() == [1.0, 2.0, 3.0]

    # a rigid body is identified by its applied schema, and exposes the 'mass' alias
    rigid_body = "/World/RigidBody"
    assert authoring.define_prim(rigid_body, "Cube")
    assert authoring.define_prim(rigid_body, "RigidBody")
    data.write(rigid_body, "mass", 7.0)
    assert data.read(rigid_body, "mass").numpy().item() == 7.0

    # a plain cube is dispatched to the shape wrapper, which has no 'mass' alias
    cube = "/World/Cube"
    assert authoring.define_prim(cube, "Cube")
    assert data.read(cube, "size").numpy().item() == 2.0
    with pytest.raises(Exception):
        data.read(cube, "mass")

    assert authoring.close_stage()
