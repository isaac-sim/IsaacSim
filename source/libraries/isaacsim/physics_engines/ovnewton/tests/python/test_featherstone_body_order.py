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

"""Mass assembly must use child-body indices rather than joint indices."""

import newton
import numpy as np
import pytest
import warp as wp
from isaacsim.physics_engines.ovnewton.impl.featherstone_solver import (
    FeatherstoneSolver,
    _assemble_mass,
)
from newton.solvers import SolverFeatherstone


def _model(device: str, permuted: bool, requires_grad: bool = False) -> newton.Model:
    builder = newton.ModelBuilder(gravity=wp.vec3(0.0, 0.0, 0.0))
    locations = [0.0, 3.0]
    for x in locations:
        builder.add_link(
            xform=wp.transform((x, 0.0, 0.0), wp.quat_identity()),
            mass=1.0,
            inertia=wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        )
    for body in [1, 0] if permuted else [0, 1]:
        joint = builder.add_joint_revolute(
            -1,
            body,
            parent_xform=wp.transform((locations[body], 0.0, 0.0), wp.quat_identity()),
            axis=wp.vec3(0.0, 1.0, 0.0),
        )
        builder.add_articulation([joint])
    return builder.finalize(device=device, requires_grad=requires_grad)


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
def test_mass_uses_child_body_order_and_preserves_upstream(device: str) -> None:
    """Two live solvers retain their own maps and the upstream update interval."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    upstream_step = SolverFeatherstone.step
    upstream_warp = upstream_step.__globals__["wp"]
    upstream_mass = upstream_step.__globals__["eval_rigid_mass"]
    models = [_model(device, permuted) for permuted in (True, False)]
    solvers = [FeatherstoneSolver(model, update_mass_matrix_interval=2) for model in models]
    states = [(model.state(), model.state(), model.state()) for model in models]
    controls = [model.control() for model in models]
    for control in controls:
        control.joint_f.fill_(1.0)
    for step in range(2):
        for solver, state, control in zip(solvers, states, controls, strict=True):
            solver.step(state[step], state[step + 1], control, None, 0.001)
            np.testing.assert_allclose(solver.H.numpy(), [1.0, 1.0], rtol=1e-5)
            np.testing.assert_allclose(state[step + 1].joint_qd.numpy(), [0.001 * (step + 1)] * 2, rtol=1e-5)
    assert SolverFeatherstone.step is upstream_step
    assert upstream_step.__globals__["wp"] is upstream_warp
    assert upstream_step.__globals__["eval_rigid_mass"] is upstream_mass


@wp.kernel
def _sum_velocity(velocity: wp.array(dtype=float), loss: wp.array(dtype=float)) -> None:
    wp.atomic_add(loss, 0, velocity[wp.tid()])


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
def test_permuted_mass_preserves_control_gradients(device: str) -> None:
    """The replacement mass launch remains visible to Warp's gradient tape."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    model = _model(device, True, requires_grad=True)
    solver = FeatherstoneSolver(model)
    state_in, state_out = model.state(), model.state()
    control = model.control()
    control.joint_f.fill_(1.0)
    loss = wp.zeros(1, device=device, requires_grad=True)
    with wp.Tape() as tape:
        solver.step(state_in, state_out, control, None, 0.001)
        wp.launch(
            _sum_velocity,
            dim=2,
            inputs=[state_out.joint_qd],
            outputs=[loss],
            device=device,
        )
    tape.backward(loss)
    np.testing.assert_allclose(control.joint_f.grad.numpy(), [0.001, 0.001], rtol=1e-5)


@wp.kernel
def _mass_loss(mass: wp.array(dtype=float), loss: wp.array(dtype=float)) -> None:
    loss[0] = mass[7] + 2.0 * mass[43]


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
def test_mass_kernel_gradient_follows_child_map(device: str) -> None:
    """Backward mass assembly scatters gradients to the mapped child inertia."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    starts = wp.array([0, 1], dtype=int, device=device)
    ends = wp.array([1, 2], dtype=int, device=device)
    offsets = wp.array([0, 36], dtype=int, device=device)
    children = wp.array([1, 0], dtype=int, device=device)
    inertias = wp.zeros(2, dtype=wp.spatial_matrix, device=device, requires_grad=True)
    mass = wp.zeros(72, device=device, requires_grad=True)
    loss = wp.zeros(1, device=device, requires_grad=True)
    with wp.Tape() as tape:
        wp.launch(
            _assemble_mass,
            dim=2,
            inputs=[starts, ends, offsets, inertias, children],
            outputs=[mass],
            device=device,
        )
        wp.launch(_mass_loss, dim=1, inputs=[mass], outputs=[loss], device=device)
    tape.backward(loss)
    expected = np.zeros((2, 6, 6), dtype=np.float32)
    expected[:, 1, 1] = [2.0, 1.0]
    np.testing.assert_array_equal(inertias.grad.numpy(), expected)


def test_permuted_mass_cuda_capture() -> None:
    """The solver-local launch proxy supports normal CUDA graph capture/replay."""
    if not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    model = _model("cuda:0", True)
    solver = FeatherstoneSolver(model)
    state_in, state_out = model.state(), model.state()
    control = model.control()
    control.joint_f.fill_(1.0)
    solver.step(state_in, state_out, control, None, 0.001)
    wp.synchronize_device("cuda:0")
    with wp.ScopedCapture(device="cuda:0") as capture:
        solver.step(state_in, state_out, control, None, 0.001)
    state_out.joint_qd.zero_()
    wp.capture_launch(capture.graph)
    np.testing.assert_allclose(state_out.joint_qd.numpy(), [0.001, 0.001], rtol=1e-5)
