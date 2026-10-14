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

"""Exercise the public MuJoCo solver through the standalone Newton stage."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
def test_mujoco_solver_advances_populated_body(device: str) -> None:
    """Verify that the selected MuJoCo solver advances a populated body under gravity."""
    import warp as wp
    from isaacsim.physics_engines.ovnewton.impl.newton_config import NewtonConfig
    from isaacsim.physics_engines.ovnewton.impl.newton_stage import NewtonStage
    from isaacsim.physics_engines.ovnewton.impl.solver_config import SolverConfig, SolverType
    from isaacsim.physics_engines.ovstage import setup

    wp.init()
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA device is unavailable")
    setup()
    import ovstage

    stage_text = """#usda 1.0
(
    metersPerUnit = 1
    upAxis = "Z"
)
def PhysicsScene "Scene" {}
def Sphere "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
) {
    double radius = 0.1
    float physics:mass = 1
    double3 xformOp:translate = (0, 0, 2)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}
"""
    engine = NewtonStage(NewtonConfig(device=device, solver=SolverConfig(solver_type=SolverType.MUJOCO)))
    with ovstage.Stage("mujoco-public-runtime") as stage:
        ovstage.population.open_usd_from_string(stage, stage_text, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(1).wait()
        try:
            assert engine.initialize(stage)
            initial = np.asarray(engine.get_body_position("/Body"))
            for step in range(10):
                engine.simulate(0.01, step * 0.01)
                assert not engine.last_step_failed
            final = np.asarray(engine.get_body_position("/Body"))
            assert engine.step_count == 10
            assert np.isfinite(final).all()
            assert final[2] < initial[2] - 0.01
            np.testing.assert_allclose(final[:2], initial[:2], atol=1e-6)
        finally:
            engine.close()
