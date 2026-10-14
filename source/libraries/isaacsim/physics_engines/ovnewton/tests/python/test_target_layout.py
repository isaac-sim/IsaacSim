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

"""Check floating-base targets against both Newton storage layouts."""

from types import SimpleNamespace

import newton
import numpy as np
import pytest
import warp as wp
from isaacsim.physics_engines.ovnewton.impl.featherstone_solver import FeatherstoneSolver
from isaacsim.physics_engines.ovnewton.impl.tensors.articulation_view import NewtonArticulationView
from isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories import _WarpFrontendShim


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
@pytest.mark.parametrize("coordinate_layout", [False, True])
def test_floating_base_position_target_reaches_solver(
    device: str, coordinate_layout: bool, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Target writes leave the free base untouched and drive the intended hinge."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    monkeypatch.setattr(newton, "use_coord_layout_targets", coordinate_layout)
    builder = newton.ModelBuilder(gravity=wp.vec3(0.0, 0.0, 0.0))
    inertia = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    base = builder.add_link(mass=1.0, inertia=inertia)
    child = builder.add_link(mass=1.0, inertia=inertia)
    root = builder.add_joint_free(child=base)
    hinge = builder.add_joint_revolute(base, child, axis=wp.vec3(0.0, 0.0, 1.0), target_ke=100.0)
    builder.add_articulation([root, hinge])
    model = builder.finalize(device=device)
    control = model.control()
    backend = SimpleNamespace(
        model=model,
        newton_stage=SimpleNamespace(control=control),
        dof_position_indices=wp.array([7], dtype=int, device=device),
        dof_velocity_indices=wp.array([6], dtype=int, device=device),
        count=1,
        max_dofs=1,
    )
    view = NewtonArticulationView(backend, _WarpFrontendShim(device))
    before = control.joint_target_q.numpy().copy()
    view.set_dof_position_targets(
        wp.array([[0.25]], dtype=float, device=device), wp.array([0], dtype=int, device=device)
    )
    expected = before.copy()
    expected[7 if coordinate_layout else 6] = 0.25
    np.testing.assert_array_equal(control.joint_target_q.numpy(), expected)
    np.testing.assert_allclose(view.get_dof_position_targets(copy=False).numpy(), [0.25])
    np.testing.assert_allclose(view.get_dof_position_targets(copy=True).numpy(), [[0.25]])
    state_in, state_out = model.state(), model.state()
    FeatherstoneSolver(model).step(state_in, state_out, control, None, 0.001)
    assert state_out.joint_qd.numpy()[6] > 0.0
