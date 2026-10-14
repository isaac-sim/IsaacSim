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

"""Exercise public OVNewton's metadata adapter against real OVStage population."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.mark.parametrize("axis", ["Y", "Z"])
@pytest.mark.parametrize("meters_per_unit", [1.0, 0.01])
def test_public_model_preserves_stage_units_and_axis(axis: str, meters_per_unit: float) -> None:
    """Preserve populated stage units and axis without changing the upstream parser."""
    from isaacsim.physics_engines.ovstage import setup

    setup()
    import ovstage
    import warp as wp
    from isaacsim.physics_engines.ovnewton.impl.stage_metadata import build_model
    from ovnewton._src import _parse

    original_reader = _parse.read_stage_units
    stage_text = f"""#usda 1.0
(
    metersPerUnit = {meters_per_unit}
    upAxis = "{axis}"
)
def PhysicsScene "Scene" {{}}
def Cube "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
) {{
    float physics:mass = 1
}}
"""
    with ovstage.Stage("metadata-contract") as stage, wp.ScopedDevice("cpu"):
        ovstage.population.open_usd_from_string(stage, stage_text, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(1).wait()
        model = build_model(stage)
        assert model.body_count == 1
        gravity = np.zeros(3)
        gravity["XYZ".index(axis)] = -9.81 / meters_per_unit
        np.testing.assert_allclose(model.gravity.numpy()[0], gravity, rtol=1e-6)
    assert _parse.read_stage_units is original_reader
