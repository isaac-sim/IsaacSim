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

# ruff: noqa: ANN001, ANN002, ANN003, ANN202, D102

"""Tests for the contact-free differentiable Newton bridge.

The gradient-correctness test is the gate for this bridge: it compares autograd
gradients against central finite differences of the same loss on a programmatic
double pendulum (no USD stage required). Newton-dependent tests skip when the
optional Newton/Warp modules are not installed.
"""

from __future__ import annotations

import asyncio
from collections.abc import Sequence
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.env_bridge import SysIdEnvironmentBridgeError
from isaacsim.robot_setup.sysid.inertia_param import inertia_matrix_to_log_cholesky
from isaacsim.robot_setup.sysid.newton_diff_sysid_bridge import (
    FRICTION_BUDGET_SHARE,
    FRICTION_INV_VELOCITY_EPS,
    FRICTION_SLOPE_STABILITY_RATIO,
    MAX_AUTO_FEATHERSTONE_SUBSTEPS,
    NewtonDifferentiableSysIdBridge,
    _coupling_derating,
    _friction_slope_budget,
    _integrable_substeps,
    _require_finite_array,
    newton_differentiable_modules_available,
)
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
    default_bounds_for_type,
)
from isaacsim.robot_setup.sysid.run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NewtonSimulationRunSpec,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset

_NEWTON_AVAILABLE, _NEWTON_REASON = newton_differentiable_modules_available()


def _cuda_available() -> bool:
    if not _NEWTON_AVAILABLE:
        return False
    try:
        import warp as wp

        return wp.get_cuda_device_count() > 0
    except Exception:
        return False


_NUM_DOF = 2
_NUM_STEPS = 30
_DT = 0.01
_LINK0_INERTIA = np.diag([0.02, 0.03, 0.04])
_LINK1_INERTIA = np.diag([0.01, 0.015, 0.02])


class NewtonDifferentiableValidationTests(omni.kit.test.AsyncTestCase):
    """Validate fail-closed optimizer boundary helpers.

    Most of these drive the bound arithmetic directly and need no simulator.
    The two that measure a real articulated inertia say so by calling
    :meth:`_require_newton` first.
    """

    def _require_newton(self) -> None:
        """Skip a test that has to measure inertia from a built model."""
        if not _NEWTON_AVAILABLE:
            self.skipTest(_NEWTON_REASON)

    async def test_nonfinite_backward_arrays_are_rejected(self) -> None:
        """NaN adjoints must never be silently converted to zero gradients."""
        with self.assertRaises(SysIdEnvironmentBridgeError):
            _require_finite_array(np.asarray([1.0, np.nan], dtype=np.float32), "test adjoint")

    @staticmethod
    def _capped_eps(budget: np.ndarray, friction: np.ndarray, damping: np.ndarray) -> np.ndarray:
        """Mirror of the cap the torque kernel applies, for use in assertions."""
        headroom = np.maximum(budget - damping, 0.0)
        return np.minimum(FRICTION_INV_VELOCITY_EPS, headroom / np.maximum(friction, 1.0e-6))

    async def test_capped_friction_slope_contracts_the_adjoint_for_any_candidate(self) -> None:
        """The capped slope must decay the adjoint however large friction gets.

        Drives the scalar recurrence the taped reverse pass compounds,
        `|1 - slope * sub_dt / inertia|`, instead of re-deriving the production
        formula and comparing it against itself.
        """
        substeps, dt = 9, 1.0 / 60.0
        sub_dt = dt / substeps
        inertia = np.asarray([0.05, 0.2], dtype=np.float64)
        damping = np.asarray([0.4, 20.0], dtype=np.float64)
        budget = _friction_slope_budget(inertia, 1.0, substeps, dt)

        # 10.0 is the largest friction scale the parameter registry allows; the
        # last two are far outside it and must still hold.
        for scale in (0.0, 1.0, 10.0, 1.0e3, 1.0e6):
            friction = scale * np.asarray([5.0, 1.0], dtype=np.float64)
            slope = damping + friction * self._capped_eps(budget, friction, damping)
            multiplier = np.abs(1.0 - slope * sub_dt / inertia)
            self.assertTrue(np.all(multiplier <= 1.0), msg=f"scale={scale} multiplier={multiplier}")

    async def test_torque_kernel_spends_the_budget_the_mirror_says_it_does(self) -> None:
        """Launch the real kernel so the mirror above cannot drift from it.

        Every other assertion here goes through ``_capped_eps``, which is a
        re-implementation. Without this the kernel could stop charging the
        candidate's own damping against the budget -- the whole reason the cap
        lives in the kernel -- and nothing would notice.
        """
        self._require_newton()

        import warp as wp
        from isaacsim.robot_setup.sysid.newton_diff_sysid_bridge import _load_diff_kernels

        mapped_joint_torque, _gather, _scatter = _load_diff_kernels(wp)
        num_dof = 3
        velocity = np.asarray([0.7, -1.3, 0.4], dtype=np.float32)
        stiffness = np.zeros(num_dof, dtype=np.float32)
        # Spans the three branches: budget to spare, damping eating most of it,
        # and damping past the budget entirely.
        damping = np.asarray([0.2, 0.75, 4.0], dtype=np.float32)
        friction = np.asarray([0.3, 0.3, 0.3], dtype=np.float32)
        budget = np.asarray([50.0, 0.8, 0.8], dtype=np.float32)

        index = wp.array(np.arange(num_dof, dtype=np.int32), dtype=wp.int32, device="cpu")
        zeros2d = wp.zeros((1, num_dof), dtype=wp.float32, device="cpu")
        joint_f = wp.zeros(num_dof, dtype=wp.float32, device="cpu")
        sim_tau = wp.zeros((1, num_dof), dtype=wp.float32, device="cpu")
        wp.launch(
            mapped_joint_torque,
            dim=num_dof,
            inputs=[
                wp.zeros(num_dof, dtype=wp.float32, device="cpu"),
                wp.array(velocity, dtype=wp.float32, device="cpu"),
                index,
                index,
                zeros2d,
                zeros2d,
                0,
                wp.array(stiffness, dtype=wp.float32, device="cpu"),
                wp.array(damping, dtype=wp.float32, device="cpu"),
                wp.array(friction, dtype=wp.float32, device="cpu"),
                FRICTION_INV_VELOCITY_EPS,
                wp.array(budget, dtype=wp.float32, device="cpu"),
                zeros2d,
                0,
                wp.zeros(num_dof, dtype=wp.float32, device="cpu"),
                0,
                0,
                joint_f,
                sim_tau,
            ],
            device="cpu",
        )

        eps = self._capped_eps(budget.astype(np.float64), friction.astype(np.float64), damping.astype(np.float64))
        expected = -damping * velocity - friction * np.tanh(velocity * eps)
        np.testing.assert_allclose(np.asarray(joint_f.numpy()), expected, rtol=1e-5, atol=1e-6)
        # Pin the branches so a cap that silently stopped binding is visible.
        self.assertAlmostEqual(float(eps[0]), FRICTION_INV_VELOCITY_EPS)
        self.assertGreater(float(eps[1]), 0.0)
        self.assertLess(float(eps[1]), FRICTION_INV_VELOCITY_EPS)
        self.assertEqual(float(eps[2]), 0.0)

    async def test_friction_knee_survives_when_the_integrator_can_carry_it(self) -> None:
        """A heavily geared joint must keep the requested Coulomb knee."""
        budget = _friction_slope_budget(np.asarray([1.0], dtype=np.float64), 1.0, 9, 1.0 / 60.0)
        eps = self._capped_eps(budget, np.asarray([0.001]), np.asarray([0.4]))
        self.assertAlmostEqual(float(eps[0]), FRICTION_INV_VELOCITY_EPS)

    async def test_identifying_inertials_never_raises_the_measured_bound(self) -> None:
        """Adding an inertial parameter may only lower the budget's inertia.

        The budget is measured once but mass, centre of mass, and inertia are
        themselves identifiable, so the bound has to hold at the lowest
        inertials a candidate can reach. Centre-of-mass offset is the trap: a
        joint's articulated inertia is a Schur complement of `H` and is not
        monotone in the lever arm, so shortening an offset can *raise* the
        measured value above the baseline, which is itself reachable.
        """
        self._require_newton()

        import newton
        from isaacsim.robot_setup.sysid import newton_diff_sysid_bridge as bridge_module

        model = self._chain_model(2)
        dof0 = np.asarray([0, 1], dtype=np.int32)
        coord0 = np.asarray([0, 1], dtype=np.int32)
        rest = np.zeros(model.joint_coord_count, dtype=np.float32)
        bodies = 2

        def probe(entries):
            corners = bridge_module._candidate_inertial_corners(entries, bodies)
            # The baseline is inside every parameter's range, so omitting it
            # would let the bound sit above a value a candidate can reach. This
            # is asserted directly because the monotonicity check below only
            # catches it on fixtures where the offset happens to bind.
            self.assertTrue(
                any(corner.identity() for corner in corners),
                "the baseline inertials are reachable and must always be probed",
            )
            inertia, coupling = bridge_module._articulated_axis_inertia(
                None, model, newton.solvers.SolverFeatherstone, dof0, coord0, rest, corners, bodies
            )
            return np.asarray(inertia), coupling, len(corners)

        friction = [_entry(SysIdParameterType.JOINT_FRICTION, dof_index=0)]
        base_inertia, base_coupling, base_corners = probe(friction)
        self.assertEqual(base_corners, 1, "a friction-only run must not pay for extra corners")

        for label, extra in (
            ("mass", _entry(SysIdParameterType.LINK_MASS)),
            ("com", _entry(SysIdParameterType.LINK_COM_OFFSET_Z)),
            ("inertia", _entry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, component_index=2)),
        ):
            inertia, coupling, corners = probe([*friction, extra])
            self.assertGreater(corners, 1, f"{label} added no corner to probe")
            np.testing.assert_array_less(
                inertia,
                base_inertia + 1e-12,
                err_msg=f"identifying {label} raised the inertia bound above the baseline",
            )
            self.assertLessEqual(coupling, base_coupling + 1e-12, f"{label} raised the coupling factor")

        # Restoring matters: the rollout reads these arrays straight afterwards.
        self.assertAlmostEqual(float(np.asarray(model.body_mass.numpy())[0]), 1.0, places=5)

    async def test_softened_knee_is_reported_with_a_usable_substep_count(self) -> None:
        """A softened knee must be reported, and the advice must be achievable.

        The changelog promises the run names the substeps that would carry the
        full friction scale, so the number has to invert the whole budget chain
        including the coupled-mode derating. Dividing by the ratio and inertia
        alone understates it by `1 / coupling`, which on a long chain is a
        factor of three -- advice that silently does not work.
        """
        from isaacsim.robot_setup.sysid import newton_diff_sysid_bridge as bridge_module

        substeps, dt, coupling = 2, 0.01, 0.25
        inertia = np.asarray([0.008], dtype=np.float64)
        friction = np.asarray([5.0], dtype=np.float64)
        damping = np.asarray([0.4], dtype=np.float64)
        budget = _friction_slope_budget(inertia, coupling, substeps, dt)

        with patch("carb.log_warn") as log_warn:
            bridge_module._warn_softened_friction_knee(
                budget, inertia, friction, damping, 1.0, coupling, substeps, dt, np.asarray([3])
            )
        log_warn.assert_called_once()
        message = log_warn.call_args.args[0]
        self.assertIn("dof 3", message)

        needed = bridge_module._substeps_for_full_friction(
            float(budget[0]), float(friction[0]), float(damping[0]), float(inertia[0]), coupling, dt
        )
        self.assertIn(str(needed), message)
        # The advice has to actually clear the cap it is offered to escape.
        recovered = _friction_slope_budget(inertia, coupling, needed, dt)
        self.assertGreaterEqual(
            float(self._capped_eps(recovered, friction, damping)[0]),
            FRICTION_INV_VELOCITY_EPS - 1e-6,
        )
        # And a coupling-blind count would not have, which is what makes this
        # assertion worth having.
        blind = bridge_module._substeps_for_full_friction(
            float(budget[0]), float(friction[0]), float(damping[0]), float(inertia[0]), 1.0, dt
        )
        self.assertLess(blind, needed)

    async def test_probed_inertia_matches_the_solver_mass_matrix(self) -> None:
        """The probe must measure articulated inertia, not the CRB diagonal.

        The stability bound is set by `1 / (H^-1)_ii`, which for a joint with
        children is strictly below `H_ii` -- on this fixture by nearly 3x. The
        probe gets there by differencing accelerations under unit torques, so
        this checks it against an independent path: the solver's own `H`,
        inverted directly. Nothing else here would notice the probe drifting
        upward, and a budget built on too much inertia is exactly the
        NaN-gradient bug this cap exists to prevent.
        """
        self._require_newton()

        import newton
        from isaacsim.robot_setup.sysid import newton_diff_sysid_bridge as bridge_module

        for num_dof in (2, 4):
            model = self._chain_model(num_dof)
            dofs = np.arange(num_dof, dtype=np.int32)
            rest = np.zeros(model.joint_coord_count, dtype=np.float32)
            measured, _coupling = bridge_module._articulated_axis_inertia(
                None,
                model,
                newton.solvers.SolverFeatherstone,
                dofs,
                dofs,
                rest,
                (
                    bridge_module._WorstCaseInertials(
                        mass_scale=np.ones(num_dof),
                        inertia_scale=np.ones(num_dof),
                        com_offset=np.zeros((num_dof, 3)),
                    ),
                ),
                num_dof,
            )
            expected = 1.0 / np.diag(self._chain_inverse_mass(num_dof))
            np.testing.assert_allclose(np.asarray(measured), expected, rtol=1e-3, atol=1e-9)

            # Pin the distinction being relied on: the CRB diagonal is a
            # materially different, and unsafe, answer.
            crb = np.diag(np.linalg.inv(self._chain_inverse_mass(num_dof)))
            self.assertGreater(float(crb[0] / measured[0]), 1.5, msg=f"num_dof={num_dof}")

    async def test_coupled_modes_stay_inside_the_stability_limit(self) -> None:
        """The budget must bound the coupled modes, not just each diagonal.

        A per-DOF budget normalizes every diagonal entry of the symmetrized
        `H^-1 D` to the stability ratio, so its trace grows with the DOF count
        and its largest eigenvalue can exceed the symplectic-Euler limit of 2
        even though no single joint does. Built from the inverse mass matrix of
        an actual serial chain, since the coupling is what is under test.
        """
        self._require_newton()

        substeps, dt = 2, 1.0 / 100.0
        sub_dt = dt / substeps
        for num_dof in (1, 2, 3, 7):
            inverse_mass = self._chain_inverse_mass(num_dof)
            diagonal = np.diag(inverse_mass).copy()
            inertia = 1.0 / diagonal
            coupling = _coupling_derating(inverse_mass, diagonal)
            budget = _friction_slope_budget(inertia, coupling, substeps, dt)

            # Worst case the kernel can produce: every DOF saturates its budget.
            saturated = np.diag(budget)
            modes = np.linalg.eigvalsh(np.sqrt(saturated) @ inverse_mass @ np.sqrt(saturated))
            self.assertLess(
                float(modes.max()) * sub_dt,
                2.0,
                msg=f"num_dof={num_dof} coupling={coupling} breached the stability limit",
            )

        # The derating must actually bind for a coupled chain, or the test above
        # would pass for a budget that ignores coupling entirely.
        chain = self._chain_inverse_mass(3)
        self.assertLess(_coupling_derating(chain, np.diag(chain).copy()), 1.0)
        # A single DOF has nothing to couple to, so it must not be derated.
        single = self._chain_inverse_mass(1)
        self.assertAlmostEqual(_coupling_derating(single, np.diag(single).copy()), 1.0)

    @staticmethod
    def _chain_model(num_dof: int):  # noqa: ANN205
        """Return a finalized uniform serial chain.

        Uniform links make the articulated inertia strongly coupled, which is
        what the stability bound has to survive.

        Args:
            num_dof: Revolute joints in the chain.

        Returns:
            Finalized Newton model.
        """
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        links: list[int] = []
        joints: list[int] = []
        for index in range(num_dof):
            link = builder.add_link(
                mass=1.0,
                com=(0.0, 0.0, -0.1),
                inertia=np.diag([0.01, 0.012, 0.014]).tolist(),
                label=f"link{index}",
            )
            joints.append(
                builder.add_joint_revolute(
                    parent=-1 if index == 0 else links[-1],
                    child=link,
                    axis=(0.0, 1.0, 0.0),
                    parent_xform=wp.transform((0.0, 0.0, 0.0 if index == 0 else -0.2), wp.quat_identity()),
                    child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                )
            )
            links.append(link)
        builder.add_articulation(joints, label="chain")
        return builder.finalize()

    @classmethod
    def _chain_inverse_mass(cls, num_dof: int) -> np.ndarray:
        """Return the inverse joint-space mass matrix of a serial chain at rest.

        Args:
            num_dof: Revolute joints in the chain.

        Returns:
            ``(num_dof, num_dof)`` inverse mass matrix.
        """
        import newton

        model = cls._chain_model(num_dof)
        solver = newton.solvers.SolverFeatherstone(model)
        state_in, state_out, control = model.state(), model.state(), model.control()
        state_in.joint_q.zero_()
        state_in.joint_qd.zero_()
        control.joint_f.assign(np.zeros(model.joint_dof_count, dtype=np.float32))
        solver.step(state_in, state_out, control=control, contacts=None, dt=1.0e-5)
        mass_matrix = np.asarray(solver.H.numpy(), dtype=np.float64).reshape(num_dof, num_dof)
        return np.linalg.inv(mass_matrix)

    async def test_slope_budget_tracks_substeps_and_sample_period(self) -> None:
        """Halving sub_dt must double the slope the integrator can carry."""
        inertia = np.asarray([0.05], dtype=np.float64)
        base = _friction_slope_budget(inertia, 1.0, 4, 1.0 / 60.0)
        self.assertAlmostEqual(float(base[0]), FRICTION_SLOPE_STABILITY_RATIO * 0.05 * 4.0 * 60.0)
        self.assertAlmostEqual(float(_friction_slope_budget(inertia, 1.0, 8, 1.0 / 60.0)[0]), 2.0 * float(base[0]))
        self.assertAlmostEqual(float(_friction_slope_budget(inertia, 1.0, 4, 1.0 / 120.0)[0]), 2.0 * float(base[0]))

    def _assert_total_slope_stable(
        self,
        inertia: float,
        damping: float,
        passive: float,
        substeps: int,
        dt: float,
    ) -> None:
        """Assert the worst slope the kernel allows stays inside the stability limit.

        Args:
            inertia: Lowest reachable articulated inertia.
            damping: Candidate drive damping.
            passive: Model ``joint_damping``.
            substeps: Substep count the rollout integrates with.
            dt: Command sample period.
        """
        inertia_np = np.asarray([inertia])
        budget = _friction_slope_budget(inertia_np, 1.0, substeps, dt, np.asarray([passive]))
        for friction in (0.0, 1.0, 10.0, 1.0e3):
            friction_np = np.asarray([friction])
            eps = self._capped_eps(budget, friction_np, np.asarray([damping]))
            slope = passive + damping + friction * float(eps[0])
            ratio = slope * (dt / substeps) / inertia
            self.assertLess(ratio, 2.0, msg=f"friction={friction} ratio={ratio}")

    async def test_substeps_are_raised_instead_of_flooring_the_inertia(self) -> None:
        """Damping the configured sub_dt cannot carry must raise substeps.

        At this inertia the authored damping alone gives a stability ratio of
        1.61 at two substeps, and flooring the inertia let friction add enough
        on top to reach 3.23, past the limit of 2 where the adjoint diverges.
        """
        inertia, damping, dt, configured = 0.002479, 0.4, 0.01, 2
        substeps = _integrable_substeps(np.asarray([inertia]), np.asarray([damping]), None, 1.0, configured, dt)
        self.assertGreater(substeps, configured)
        budget = _friction_slope_budget(np.asarray([inertia]), 1.0, substeps, dt)
        self.assertLessEqual(damping, (1.0 - FRICTION_BUDGET_SHARE) * float(budget[0]) + 1e-12)
        self._assert_total_slope_stable(inertia, damping, 0.0, substeps, dt)

        # Never below the configured count, and capped when far out of reach.
        self.assertEqual(
            _integrable_substeps(np.asarray([1.0]), np.asarray([0.4]), None, 1.0, configured, dt), configured
        )
        self.assertEqual(
            _integrable_substeps(np.asarray([1.0e-9]), np.asarray([0.4]), None, 1.0, configured, dt),
            MAX_AUTO_FEATHERSTONE_SUBSTEPS,
        )

    async def test_inertia_is_floored_only_past_the_substep_cap(self) -> None:
        """Past the cap the budget falls back to the floor instead of starving friction.

        Identifying log-Cholesky inertia drives the reachable inertia towards
        zero, and with it the substep count the damping needs. Sizing the
        budget at the probe there would leave the baseline candidate no room
        for friction at all.
        """
        from isaacsim.robot_setup.sysid import newton_diff_sysid_bridge as bridge_module

        damping, dt, configured = np.asarray([0.3]), 0.01, 2
        tiny = np.asarray([2.0e-5])
        substeps = _integrable_substeps(tiny, damping, None, 1.0, configured, dt)
        self.assertEqual(substeps, MAX_AUTO_FEATHERSTONE_SUBSTEPS)
        floored = bridge_module._integrable_inertia(tiny, damping, None, 1.0, substeps, dt)
        self.assertGreater(float(floored[0]), float(tiny[0]))
        budget = _friction_slope_budget(floored, 1.0, substeps, dt)
        self.assertAlmostEqual(float(damping[0]), (1.0 - FRICTION_BUDGET_SHARE) * float(budget[0]), places=9)

        # Wherever the substeps suffice, the probe is used unchanged.
        reachable = np.asarray([0.002479])
        enough = _integrable_substeps(reachable, np.asarray([0.4]), None, 1.0, configured, dt)
        np.testing.assert_array_equal(
            bridge_module._integrable_inertia(reachable, np.asarray([0.4]), None, 1.0, enough, dt), reachable
        )

    async def test_passive_damping_is_taken_out_of_the_kernel_budget(self) -> None:
        """Passive joint damping must not be spent a second time by the kernel.

        On this fixture passive damping alone reaches a ratio of 1.5; adding
        the full kernel budget on top used to reach 2.5.
        """
        inertia, passive, dt, configured = 0.01, 1.5, 0.01, 1
        substeps = _integrable_substeps(
            np.asarray([inertia]), np.asarray([0.0]), np.asarray([passive]), 1.0, configured, dt
        )
        self._assert_total_slope_stable(inertia, 0.0, passive, substeps, dt)
        # At the configured count, passive damping alone is past the budget,
        # so nothing is left for the kernel.
        budget = _friction_slope_budget(np.asarray([inertia]), 1.0, configured, dt, np.asarray([passive]))
        self.assertEqual(float(budget[0]), 0.0)

    async def test_com_probe_applies_cartesian_axis_offsets(self) -> None:
        """The probe must move a centre of mass the way the optimizer does.

        ``LINK_COM_OFFSET_X`` adds to the X coordinate. Stretching the offset
        along the COM direction instead left this link's axis inertia at about
        0.00101 kg m^2 when the reachable candidate (X offset -0.05) has 0.0002.
        """
        self._require_newton()

        import newton
        import warp as wp
        from isaacsim.robot_setup.sysid import newton_diff_sysid_bridge as bridge_module

        builder = newton.ModelBuilder()
        link = builder.add_link(mass=1.0, com=(0.04, 0.2, 0.0), inertia=np.diag([1e-4, 1e-4, 1e-4]).tolist())
        joint = builder.add_joint_revolute(
            parent=-1,
            child=link,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
        )
        builder.add_articulation([joint], label="arm")
        model = builder.finalize()

        corners = bridge_module._candidate_inertial_corners([_entry(SysIdParameterType.LINK_COM_OFFSET_X)], 1)
        minimum, maximum = default_bounds_for_type(SysIdParameterType.LINK_COM_OFFSET_X)
        offsets = [tuple(corner.com_offset[0].tolist()) for corner in corners]
        self.assertIn((minimum, 0.0, 0.0), offsets)
        self.assertIn((maximum, 0.0, 0.0), offsets)

        dofs = np.asarray([0], dtype=np.int32)
        rest = np.zeros(model.joint_coord_count, dtype=np.float32)
        inertia, _coupling = bridge_module._articulated_axis_inertia(
            None, model, newton.solvers.SolverFeatherstone, dofs, dofs, rest, corners, 1
        )
        # I_yy + m * (x^2 + z^2) at x = 0.04 - 0.05.
        self.assertAlmostEqual(float(inertia[0]), 1e-4 + 0.01**2, delta=1e-5)


def _entry(
    ptype: SysIdParameterType,
    dof_index: int = -1,
    link_index: int = -1,
    component_index: int = 0,
):
    return SysIdParameterEntry(
        param_type=ptype,
        dof_index=dof_index,
        link_index=link_index,
        component_index=component_index,
    )


def _build_pendulum():
    import newton
    import warp as wp

    builder = newton.ModelBuilder()
    b0 = builder.add_link(mass=1.2, com=(0.0, 0.0, -0.15), inertia=_LINK0_INERTIA.tolist(), label="link0")
    b1 = builder.add_link(mass=0.7, com=(0.0, 0.0, -0.1), inertia=_LINK1_INERTIA.tolist(), label="link1")
    j0 = builder.add_joint_revolute(
        parent=-1,
        child=b0,
        axis=(0.0, 1.0, 0.0),
        parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
        child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
        target_ke=6.0,
        target_kd=0.4,
    )
    j1 = builder.add_joint_revolute(
        parent=b0,
        child=b1,
        axis=(0.0, 1.0, 0.0),
        parent_xform=wp.transform((0.0, 0.0, -0.3), wp.quat_identity()),
        child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
        target_ke=4.0,
        target_kd=0.3,
    )
    builder.add_articulation([j0, j1], label="pendulum")
    return builder


def _make_bridge(
    feedforward: str = "none",
    device: str = "cpu",
    cuda_graph_capture: bool = True,
    effort_clamp: str = "max_effort",
    max_force: float = float("inf"),
) -> tuple[NewtonDifferentiableSysIdBridge, torch.Tensor]:
    snapshots = [
        SimpleNamespace(
            stiffness=6.0,
            damping=0.4,
            friction=0.25,
            dynamic_friction=0.25,
            max_force=max_force,
        ),
        SimpleNamespace(
            stiffness=4.0,
            damping=0.3,
            friction=0.15,
            dynamic_friction=0.15,
            max_force=max_force,
        ),
    ]
    config = NewtonSimulationRunSpec(
        device=device,
        solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
        featherstone_substeps=2,
        feedforward=feedforward,
        cuda_graph_capture=cuda_graph_capture,
        effort_clamp=effort_clamp,
    )
    bridge = NewtonDifferentiableSysIdBridge(
        robot_prim_path="/pendulum",
        newton_config=config,
        joint_baselines=snapshots,
        num_joints=_NUM_DOF,
        num_links=2,
        robot_builder=_build_pendulum(),
    )
    times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
    zeros = np.zeros((_NUM_STEPS, _NUM_DOF), dtype=np.float64)
    commands = 0.4 * np.sin(2.0 * np.pi * 1.5 * times)[:, None] * np.array([[1.0, -0.7]])
    bridge.set_trajectory(
        TrajectoryDataset(
            times=times,
            positions=zeros.copy(),
            velocities=zeros.copy(),
            commands=commands,
        )
    )
    return bridge, torch.as_tensor(commands, dtype=torch.float32)


def _hold_trajectory(value: float, num_steps: int = _NUM_STEPS) -> TrajectoryDataset:
    """Constant-pose trajectory chunk; distinct `value`s give distinct feedforwards.

    Args:
        value: Value supplied for ``value``.
        num_steps: Value supplied for ``num_steps``.

    Returns:
        Result produced by the operation.
    """
    times = np.arange(num_steps, dtype=np.float64) * _DT
    hold = np.full((num_steps, _NUM_DOF), value, dtype=np.float64) * np.array([[1.0, -0.5]])
    return TrajectoryDataset(
        times=times,
        positions=hold.copy(),
        velocities=np.zeros_like(hold),
        commands=hold.copy(),
    )


def _count_feedforward_computes():
    """Patch the module-level feedforward computation with a counting wrapper.

    Returns:
        Result produced by the operation.
    """
    import isaacsim.robot_setup.sysid.newton_diff_sysid_bridge as diff_module

    calls = {"count": 0}
    original = diff_module._compute_feedforward

    def counting(*args, **kwargs):
        calls["count"] += 1
        return original(*args, **kwargs)

    diff_module._compute_feedforward = counting
    return calls, lambda: setattr(diff_module, "_compute_feedforward", original)


_GRADCHECK_ENTRIES = [
    _entry(SysIdParameterType.JOINT_FRICTION, dof_index=0),
    _entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0),
    _entry(SysIdParameterType.JOINT_DAMPING, dof_index=1),
    _entry(SysIdParameterType.LINK_MASS),
    _entry(SysIdParameterType.LINK_COM_OFFSET_Z),
    # Component 2 (log L11 -> I_yy) is the inertia entry that affects Y-axis planar dynamics.
    _entry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, component_index=2),
]


def _gradcheck_steps(entries: Sequence[SysIdParameterEntry]) -> np.ndarray:
    """Choose finite-difference steps from each parameter's numerical scale.

    The rollout accumulates in float32, which puts a floor on the usable step:
    a Richardson sweep on the log-Cholesky entry converges to the taped
    gradient at 1e-3 (0.02%) but degrades to 1.3% at 1e-4 and 3.2% at 1e-5 as
    cancellation takes over. Step *down* from here only to chase truncation
    error, and only after confirming the noise floor allows it.

    Friction's gradient on the gradcheck pendulum is about 1e-3 of the loss,
    so at 1e-3 the loss change sits near the float32 noise of the summed
    trajectory; steps from 3e-3 to 3e-2 agree with the taped gradient to 0.3%.

    Args:
        entries: Parameter entries being checked.

    Returns:
        One finite-difference step per entry, in matching order.
    """
    return np.asarray(
        [1e-2 if entry.param_type == SysIdParameterType.JOINT_FRICTION else 1e-3 for entry in entries],
        dtype=np.float64,
    )


class NewtonDiffBridgeConstructorTests(omni.kit.test.AsyncTestCase):
    """Configuration contracts that are validated before optional modules load."""

    async def test_rejects_mujoco_solver_configuration(self) -> None:
        with self.assertRaises(SysIdEnvironmentBridgeError):
            NewtonDifferentiableSysIdBridge(
                robot_prim_path="/robot",
                newton_config=NewtonSimulationRunSpec(solver="mujoco"),
            )


class NewtonDiffBridgeValidationTests(omni.kit.test.AsyncTestCase):
    """Parameter validation runs without Newton installed."""

    def _bridge_for_validation(self) -> NewtonDifferentiableSysIdBridge:
        if not _NEWTON_AVAILABLE:
            self.skipTest(_NEWTON_REASON)
        bridge, _commands = _make_bridge()
        return bridge

    async def test_rejects_armature_with_fallback_hint(self) -> None:
        bridge = self._bridge_for_validation()
        with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
            bridge.validate_parameter_entries([_entry(SysIdParameterType.JOINT_ARMATURE, dof_index=0)])
        message = str(raised.exception).lower()
        self.assertIn("armature", message)
        self.assertIn("mujoco", message)

    async def test_rejects_non_differentiable_types(self) -> None:
        bridge = self._bridge_for_validation()
        for ptype in (
            SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
            SysIdParameterType.JOINT_INTEGRAL_GAIN,
            SysIdParameterType.JOINT_LIMIT_LOWER_SCALE,
        ):
            with self.subTest(ptype=ptype.value):
                with self.assertRaises(SysIdEnvironmentBridgeError):
                    bridge.validate_parameter_entries([_entry(ptype, dof_index=0)])

    async def test_accepts_supported_types(self) -> None:
        bridge = self._bridge_for_validation()
        bridge.validate_parameter_entries(_GRADCHECK_ENTRIES)


class NewtonDiffBridgeRolloutTests(omni.kit.test.AsyncTestCase):
    """Gradient correctness and rollout behavior on a programmatic pendulum."""

    def setUp(self) -> None:
        if not _NEWTON_AVAILABLE:
            self.skipTest(_NEWTON_REASON)

    def _theta0(self) -> np.ndarray:
        lc0 = inertia_matrix_to_log_cholesky(_LINK0_INERTIA)
        return np.array([1.2, 0.9, 1.1, 1.05, 0.01, float(lc0[2]) + 0.05], dtype=np.float64)

    async def _loss(self, bridge, commands, theta_np, weights, requires_grad=False):
        theta = torch.as_tensor(theta_np, dtype=torch.float32).reshape(1, -1)
        if requires_grad:
            theta.requires_grad_(True)
        result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        w_pos, w_vel = weights
        return theta, (result.positions * w_pos).sum() + (result.velocities * w_vel).sum()

    async def test_cancellation_after_rollout_releases_backward_lease(self) -> None:
        bridge, commands = _make_bridge()
        update_count = 0

        async def cancel_after_rollout() -> None:
            nonlocal update_count
            update_count += 1
            if update_count == 2:
                raise asyncio.CancelledError()

        bridge._next_update_async = cancel_after_rollout
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1).requires_grad_(True)

        with self.assertRaises(asyncio.CancelledError):
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)

        self.assertFalse(bridge._backward_pending)

    async def test_gradients_match_finite_differences(self) -> None:
        bridge, commands = _make_bridge()
        generator = torch.Generator().manual_seed(3)
        weights = (
            torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5,
            0.1 * (torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5),
        )
        theta0 = self._theta0()

        theta, loss = await self._loss(bridge, commands, theta0, weights, requires_grad=True)
        loss.backward()
        grad_autodiff = theta.grad.reshape(-1).double().numpy()

        steps = _gradcheck_steps(_GRADCHECK_ENTRIES)
        for index in range(len(theta0)):
            plus = theta0.copy()
            plus[index] += steps[index]
            minus = theta0.copy()
            minus[index] -= steps[index]
            _, loss_plus = await self._loss(bridge, commands, plus, weights)
            _, loss_minus = await self._loss(bridge, commands, minus, weights)
            grad_fd = (float(loss_plus) - float(loss_minus)) / (2.0 * steps[index])
            with self.subTest(param=_GRADCHECK_ENTRIES[index].param_type.value):
                denominator = max(abs(grad_fd), 1e-4)
                self.assertLess(
                    abs(grad_autodiff[index] - grad_fd) / denominator,
                    5e-2,
                    msg=f"autodiff={grad_autodiff[index]:.6g} fd={grad_fd:.6g}",
                )

    async def test_batch_rows_are_independent_and_cache_reuse_is_deterministic(
        self,
    ) -> None:
        bridge, commands = _make_bridge()
        theta0 = self._theta0()
        theta_batch = torch.as_tensor(np.stack([theta0, theta0 * 1.1, theta0]), dtype=torch.float32)

        first = await bridge.run_rollout_async(theta_batch, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertTrue(torch.allclose(first.positions[0], first.positions[2], atol=1e-6))
        self.assertFalse(torch.allclose(first.positions[0], first.positions[1], atol=1e-4))

        second = await bridge.run_rollout_async(theta_batch, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertTrue(torch.allclose(first.positions, second.positions, atol=1e-6))
        self.assertTrue(torch.allclose(first.velocities, second.velocities, atol=1e-6))

    async def test_rejects_second_differentiable_forward_before_backward(self) -> None:
        bridge, commands = _make_bridge()
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1).requires_grad_(True)
        first = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)

        with self.assertRaises(SysIdEnvironmentBridgeError):
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)

        first.positions.sum().backward()
        second = await bridge.run_rollout_async(theta.detach(), _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertTrue(torch.all(torch.isfinite(second.positions)))

    async def test_chunk_reset_uses_trajectory_initial_state(self) -> None:
        bridge, commands = _make_bridge()
        theta0 = self._theta0()
        theta = torch.as_tensor(theta0, dtype=torch.float32).reshape(1, -1)
        baseline = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)

        times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
        offset = np.full((_NUM_STEPS, _NUM_DOF), 0.2, dtype=np.float64)
        bridge.set_trajectory(
            TrajectoryDataset(
                times=times,
                positions=offset.copy(),
                velocities=np.zeros_like(offset),
                commands=commands.numpy().astype(np.float64),
            )
        )
        shifted = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertFalse(torch.allclose(baseline.positions, shifted.positions, atol=1e-3))
        self.assertGreater(
            float(shifted.positions[0, 0].mean()),
            float(baseline.positions[0, 0].mean()),
        )

    async def test_torque_channel_matches_pd_friction_convention(self) -> None:
        bridge, commands = _make_bridge()
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertEqual(tuple(result.torques.shape), (1, _NUM_STEPS, _NUM_DOF))
        self.assertTrue(torch.all(torch.isfinite(result.torques)))

    async def test_effort_clamp_limits_applied_and_reported_torque(self) -> None:
        bridge, commands = _make_bridge(max_force=0.2)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1).requires_grad_(True)
        result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands * 20.0, _NUM_STEPS)

        self.assertLessEqual(float(result.torques.detach().abs().max()), 0.20001)
        result.torques.square().sum().backward()
        self.assertTrue(torch.all(torch.isfinite(theta.grad)))

    async def test_unresolved_joint_path_is_fatal(self) -> None:
        """A bad path must not silently fall back to Newton import order."""
        bridge, commands = _make_bridge()
        bridge._joint_paths = ["/pendulum/missing_joint", "/pendulum/joint_2"]
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)

        with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)

        self.assertIn("cannot uniquely map", str(raised.exception).lower())

    async def test_trajectory_may_cover_subset_of_actuated_joints(self) -> None:
        """A 3-joint model with 2-DOF telemetry maps by joint path; the extra joint holds."""
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        links = [
            builder.add_link(
                mass=1.0,
                com=(0.0, 0.0, -0.1),
                inertia=np.diag([0.01, 0.012, 0.014]).tolist(),
            )
            for _ in range(3)
        ]
        joints = []
        parent = -1
        for index, link in enumerate(links):
            joints.append(
                builder.add_joint_revolute(
                    parent=parent,
                    child=link,
                    axis=(0.0, 1.0, 0.0),
                    parent_xform=wp.transform((0.0, 0.0, 1.0 if index == 0 else -0.2), wp.quat_identity()),
                    child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                    target_ke=5.0,
                    target_kd=0.4,
                    label=f"/robot/joint{index}",
                )
            )
        builder.add_articulation(joints, label="chain")

        config = NewtonSimulationRunSpec(
            device="cpu",
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=2,
        )
        bridge = NewtonDifferentiableSysIdBridge(
            robot_prim_path="/robot",
            newton_config=config,
            joint_baselines=[SimpleNamespace(stiffness=5.0, damping=0.4, friction=0.1, dynamic_friction=0.1)] * 2,
            joint_paths=[
                "/robot/joint0",
                "/robot/joint2",
            ],  # telemetry covers joints 0 and 2 only
            num_joints=2,
            num_links=3,
            robot_builder=builder,
        )
        times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
        zeros = np.zeros((_NUM_STEPS, 2), dtype=np.float64)
        commands = 0.3 * np.sin(2.0 * np.pi * 1.5 * times)[:, None] * np.array([[1.0, -1.0]])
        bridge.set_trajectory(
            TrajectoryDataset(
                times=times,
                positions=zeros.copy(),
                velocities=zeros.copy(),
                commands=commands,
            )
        )
        entries = [
            _entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0),
            _entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=1),
        ]
        theta = torch.tensor([[1.0, 1.0]], requires_grad=True)
        result = await bridge.run_rollout_async(
            theta, entries, torch.as_tensor(commands, dtype=torch.float32), _NUM_STEPS
        )
        self.assertEqual(tuple(result.positions.shape), (1, _NUM_STEPS, 2))
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        # Both mapped joints respond to their commands; gradients still flow.
        self.assertGreater(float(result.positions[0, :, 0].detach().abs().max()), 0.02)
        self.assertGreater(float(result.positions[0, :, 1].detach().abs().max()), 0.02)
        result.positions.sum().backward()
        self.assertTrue(torch.all(torch.isfinite(theta.grad)))

    async def test_gravity_compensation_removes_static_sag_and_keeps_gradients(
        self,
    ) -> None:
        """The nominal-model feedforward holds a commanded pose against gravity.

        Without compensation the pendulum sags below a constant target by
        roughly gravity_torque / ke; with it the steady-state error should
        shrink by an order of magnitude, gradients must stay finite, and the
        torque channel must carry the feedforward (gravity-scale bias).
        """
        num_steps = 80
        times = np.arange(num_steps, dtype=np.float64) * _DT
        hold = np.full((num_steps, _NUM_DOF), 0.6, dtype=np.float64) * np.array([[1.0, -0.5]])
        theta = self._theta0()

        errors: dict[bool, float] = {}
        torque_means: dict[bool, np.ndarray] = {}
        for compensate in (False, True):
            bridge, _ = _make_bridge(feedforward="gravity" if compensate else "none")
            bridge.set_trajectory(
                TrajectoryDataset(
                    times=times,
                    positions=hold.copy(),  # feedforward evaluated at the held pose
                    velocities=np.zeros_like(hold),
                    commands=hold.copy(),
                )
            )
            theta_t = torch.as_tensor(theta, dtype=torch.float32).reshape(1, -1).requires_grad_(True)
            result = await bridge.run_rollout_async(
                theta_t,
                _GRADCHECK_ENTRIES,
                torch.as_tensor(hold, dtype=torch.float32),
                num_steps,
            )
            self.assertTrue(torch.all(torch.isfinite(result.positions)))
            tail = result.positions[0, num_steps // 2 :, :]
            errors[compensate] = float(
                (tail - torch.as_tensor(hold[num_steps // 2 :], dtype=torch.float32)).detach().abs().max()
            )
            torque_means[compensate] = result.torques[0].mean(dim=0).detach().numpy()
            result.positions.sum().backward()
            self.assertTrue(torch.all(torch.isfinite(theta_t.grad)))

        self.assertLess(errors[True], 0.15 * errors[False])
        # The compensated torque channel reports applied torque including the
        # feedforward; near-perfect holding means it is gravity-dominated.
        self.assertGreater(float(np.abs(torque_means[True] - torque_means[False]).max()), 0.1)

    async def test_gradients_match_finite_differences_with_gravity_compensation(
        self,
    ) -> None:
        """The feedforward is a taped constant; gradients must stay FD-exact."""
        bridge, commands = _make_bridge(feedforward="gravity")
        generator = torch.Generator().manual_seed(5)
        weights = (
            torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5,
            0.1 * (torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5),
        )
        theta0 = self._theta0()
        theta, loss = await self._loss(bridge, commands, theta0, weights, requires_grad=True)
        loss.backward()
        grad_autodiff = theta.grad.reshape(-1).double().numpy()
        steps = _gradcheck_steps(_GRADCHECK_ENTRIES)
        for index in range(len(theta0)):
            plus, minus = theta0.copy(), theta0.copy()
            plus[index] += steps[index]
            minus[index] -= steps[index]
            _, loss_plus = await self._loss(bridge, commands, plus, weights)
            _, loss_minus = await self._loss(bridge, commands, minus, weights)
            grad_fd = (float(loss_plus) - float(loss_minus)) / (2.0 * steps[index])
            with self.subTest(param=_GRADCHECK_ENTRIES[index].param_type.value):
                self.assertLess(
                    abs(grad_autodiff[index] - grad_fd) / max(abs(grad_fd), 1e-4),
                    5e-2,
                    msg=f"autodiff={grad_autodiff[index]:.6g} fd={grad_fd:.6g}",
                )

    async def test_feedforward_matches_analytic_single_pendulum(self) -> None:
        """The numpy Newton-Euler matches the closed-form single-pendulum torque.

        For a link with mass m, CoM at distance l below a revolute Y-axis joint,
        and CoM inertia I_yy: tau = (I_yy + m l^2) qdd + m g l sin(q).
        """
        import newton
        import warp as wp
        from isaacsim.robot_setup.sysid.newton_diff_sysid_bridge import (
            _compute_feedforward,
        )

        mass, length, iyy = 1.7, 0.22, 0.045
        builder = newton.ModelBuilder()
        link = builder.add_link(
            mass=mass,
            com=(0.0, 0.0, -length),
            inertia=np.diag([0.03, iyy, 0.05]).tolist(),
        )
        joint = builder.add_joint_revolute(
            parent=-1,
            child=link,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            target_ke=5.0,
            target_kd=0.5,
        )
        builder.add_articulation([joint], label="pendulum1")
        config = NewtonSimulationRunSpec(
            device="cpu",
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=2,
            feedforward="inverse_dynamics",
        )
        bridge = NewtonDifferentiableSysIdBridge(
            robot_prim_path="/p1",
            newton_config=config,
            joint_baselines=[SimpleNamespace(stiffness=5.0, damping=0.5, friction=0.0, dynamic_friction=0.0)],
            num_joints=1,
            num_links=1,
            robot_builder=builder,
        )
        num_steps = 40
        times = np.arange(num_steps, dtype=np.float64) * _DT
        q = 0.7 * np.sin(2.0 * np.pi * 1.2 * times)[:, None]
        qd = 0.7 * 2.0 * np.pi * 1.2 * np.cos(2.0 * np.pi * 1.2 * times)[:, None]
        qdd = -0.7 * (2.0 * np.pi * 1.2) ** 2 * np.sin(2.0 * np.pi * 1.2 * times)[:, None]
        bridge.set_trajectory(
            TrajectoryDataset(times=times, positions=q.copy(), velocities=qd.copy(), commands=q.copy())
        )
        theta = torch.tensor([[1.0]])
        entries = [_entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0)]
        await bridge.run_rollout_async(theta, entries, torch.as_tensor(q, dtype=torch.float32), 2)
        ctx = bridge._ctx_cache[1]

        feedforward = _compute_feedforward(ctx, newton, q, qd, qdd)[:, 0]
        gravity = 9.80665
        analytic = (iyy + mass * length * length) * qdd[:, 0] + mass * gravity * length * np.sin(q[:, 0])
        max_err = float(np.abs(feedforward - analytic).max())
        self.assertLess(max_err, 1e-3 * max(1.0, float(np.abs(analytic).max())))

    async def test_feedforward_accepts_locked_d6_base_joint(self) -> None:
        """A zero-DOF D6 base joint contributes no motion to the feedforward.

        Isaac Sim robot assets author the fixed base as a generic
        ``UsdPhysics.Joint`` with every axis locked, which Newton imports as a
        ``D6`` joint carrying no coordinates. That joint is kinematically a
        ``FIXED`` joint, so the pendulum torque must match the closed form.
        """
        import newton
        import warp as wp
        from isaacsim.robot_setup.sysid.feedforward import compute_feedforward

        mass, length, iyy = 1.7, 0.22, 0.045
        base_inertia = np.diag([0.01, 0.01, 0.01])
        link_inertia = np.diag([0.03, iyy, 0.05])
        builder = newton.ModelBuilder()
        base = builder.add_link(mass=0.9, com=(0.0, 0.0, 0.0), inertia=base_inertia.tolist())
        link = builder.add_link(mass=mass, com=(0.0, 0.0, -length), inertia=link_inertia.tolist())
        root = builder.add_joint_d6(parent=-1, child=base, linear_axes=[], angular_axes=[])
        joint = builder.add_joint_revolute(
            parent=base,
            child=link,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            target_ke=5.0,
            target_kd=0.5,
        )
        builder.add_articulation([root, joint], label="d6_base_pendulum")
        model = builder.finalize()
        self.assertEqual(int(model.joint_type.numpy()[0]), int(newton.JointType.D6))

        num_steps = 40
        times = np.arange(num_steps, dtype=np.float64) * _DT
        q = 0.7 * np.sin(2.0 * np.pi * 1.2 * times)[:, None]
        qd = 0.7 * 2.0 * np.pi * 1.2 * np.cos(2.0 * np.pi * 1.2 * times)[:, None]
        qdd = -0.7 * (2.0 * np.pi * 1.2) ** 2 * np.sin(2.0 * np.pi * 1.2 * times)[:, None]

        feedforward = compute_feedforward(
            newton,
            model,
            world_count=1,
            coords_per_world=1,
            dofs_per_world=1,
            coord_map_world0=np.array([0], dtype=np.int64),
            dof_map_world0=np.array([0], dtype=np.int64),
            default_joint_q_world0=np.zeros(1, dtype=np.float64),
            baseline_mass=np.array([0.9, mass], dtype=np.float64),
            baseline_com=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -length]], dtype=np.float64),
            baseline_inertia=np.stack([base_inertia, link_inertia]),
            q_steps=q,
            qd_steps=qd,
            qdd_steps=qdd,
        )[:, 0]

        gravity = 9.80665
        analytic = (iyy + mass * length * length) * qdd[:, 0] + mass * gravity * length * np.sin(q[:, 0])
        max_err = float(np.abs(feedforward - analytic).max())
        self.assertLess(max_err, 1e-3 * max(1.0, float(np.abs(analytic).max())))

    async def test_feedforward_rejects_unsupported_joint_with_named_type(self) -> None:
        """An unsupported joint type fails with its Newton name and the recovery setting."""
        import newton
        from isaacsim.robot_setup.sysid.feedforward import compute_feedforward

        builder = newton.ModelBuilder()
        link = builder.add_link(mass=1.0, com=(0.0, 0.0, 0.0), inertia=np.eye(3).tolist())
        root = builder.add_joint_ball(parent=-1, child=link)
        builder.add_articulation([root], label="ball_root")
        model = builder.finalize()

        with self.assertRaisesRegex(ValueError, r"joint type BALL .*joint 0\).*feedforward='none'"):
            compute_feedforward(
                newton,
                model,
                world_count=1,
                coords_per_world=4,
                dofs_per_world=3,
                coord_map_world0=np.array([0], dtype=np.int64),
                dof_map_world0=np.array([0], dtype=np.int64),
                default_joint_q_world0=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
                baseline_mass=np.array([1.0], dtype=np.float64),
                baseline_com=np.zeros((1, 3), dtype=np.float64),
                baseline_inertia=np.eye(3)[None, :, :],
                q_steps=np.zeros((4, 1), dtype=np.float64),
            )

    async def test_inverse_dynamics_feedforward_tracks_dynamic_trajectory(self) -> None:
        """Full-ID feedforward supplies the motion torque; PD correction shrinks.

        Uses a single-link pendulum so the joint-space mass matrix is
        configuration-independent and the remaining tracking error isolates the
        feedforward quality.
        """
        import newton
        import warp as wp

        mass, length, iyy = 1.7, 0.22, 0.045
        num_steps = 60
        times = np.arange(num_steps, dtype=np.float64) * _DT
        omega_cmd = 2.0 * np.pi * 1.2
        commands = 0.5 * np.sin(omega_cmd * times)[:, None]
        velocities = 0.5 * omega_cmd * np.cos(omega_cmd * times)[:, None]

        errors: dict[str, float] = {}
        for mode in ("none", "inverse_dynamics"):
            builder = newton.ModelBuilder()
            link = builder.add_link(
                mass=mass,
                com=(0.0, 0.0, -length),
                inertia=np.diag([0.03, iyy, 0.05]).tolist(),
            )
            joint = builder.add_joint_revolute(
                parent=-1,
                child=link,
                axis=(0.0, 1.0, 0.0),
                parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
                child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                target_ke=5.0,
                target_kd=0.5,
            )
            builder.add_articulation([joint], label="pendulum1")
            config = NewtonSimulationRunSpec(
                device="cpu",
                solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
                featherstone_substeps=4,
                feedforward=mode,
            )
            bridge = NewtonDifferentiableSysIdBridge(
                robot_prim_path="/p1",
                newton_config=config,
                joint_baselines=[SimpleNamespace(stiffness=5.0, damping=0.5, friction=0.0, dynamic_friction=0.0)],
                num_joints=1,
                num_links=1,
                robot_builder=builder,
            )
            bridge.set_trajectory(
                TrajectoryDataset(
                    times=times,
                    positions=commands.copy(),
                    velocities=velocities.copy(),
                    commands=commands.copy(),
                )
            )
            theta = torch.tensor([[1.0]], requires_grad=True)
            entries = [_entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0)]
            result = await bridge.run_rollout_async(
                theta,
                entries,
                torch.as_tensor(commands, dtype=torch.float32),
                num_steps,
            )
            self.assertTrue(torch.all(torch.isfinite(result.positions)))
            tail = slice(num_steps // 3, None)
            errors[mode] = float(
                (result.positions[0, tail] - torch.as_tensor(commands, dtype=torch.float32)[tail]).abs().mean().detach()
            )
            result.positions.sum().backward()
            self.assertTrue(torch.all(torch.isfinite(theta.grad)))
        self.assertLess(errors["inverse_dynamics"], 0.25 * errors["none"])

    async def test_feedforward_rejects_floating_base(self) -> None:
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        base = builder.add_link(mass=2.0, com=(0.0, 0.0, 0.0), inertia=np.diag([0.1, 0.1, 0.1]).tolist())
        base_joint = builder.add_joint_free(child=base)
        arm = builder.add_link(
            mass=0.7,
            com=(0.0, 0.0, -0.1),
            inertia=np.diag([0.01, 0.015, 0.02]).tolist(),
        )
        rev = builder.add_joint_revolute(
            parent=base,
            child=arm,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 0.2), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            target_ke=4.0,
            target_kd=0.3,
        )
        builder.add_articulation([base_joint, rev], label="floating")
        config = NewtonSimulationRunSpec(
            device="cpu",
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=2,
            feedforward="gravity",
        )
        bridge = NewtonDifferentiableSysIdBridge(
            robot_prim_path="/floating",
            newton_config=config,
            joint_baselines=[SimpleNamespace(stiffness=4.0, damping=0.3, friction=0.0, dynamic_friction=0.0)],
            num_joints=1,
            num_links=2,
            robot_builder=builder,
        )
        num_steps = 10
        times = np.arange(num_steps, dtype=np.float64) * _DT
        zeros = np.zeros((num_steps, 1), dtype=np.float64)
        bridge.set_trajectory(
            TrajectoryDataset(
                times=times,
                positions=zeros.copy(),
                velocities=zeros.copy(),
                commands=zeros.copy(),
            )
        )
        entries = [_entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0)]
        with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
            await bridge.run_rollout_async(
                torch.tensor([[1.0]]),
                entries,
                torch.as_tensor(zeros, dtype=torch.float32),
                num_steps,
            )
        self.assertIn("fixed-base", str(raised.exception))

    async def test_feedforward_computation_failure_is_fatal(self) -> None:
        import isaacsim.robot_setup.sysid.newton_diff_sysid_bridge as diff_module

        bridge, commands = _make_bridge(feedforward="gravity")
        bridge.set_trajectory(_hold_trajectory(0.2))
        original = diff_module._compute_feedforward

        def fail_feedforward(*_args, **_kwargs):
            raise RuntimeError("feedforward failed")

        diff_module._compute_feedforward = fail_feedforward
        try:
            theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
            with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
                await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        finally:
            diff_module._compute_feedforward = original
        self.assertIn("feedforward failed", str(raised.exception))

    async def test_feedforward_modes_are_applied(self) -> None:
        bridge_gravity, _ = _make_bridge(feedforward="gravity")
        self.assertEqual(bridge_gravity._feedforward_mode, "gravity")
        bridge_mode, _ = _make_bridge(feedforward="inverse_dynamics")
        self.assertEqual(bridge_mode._feedforward_mode, "inverse_dynamics")
        bridge_default, _ = _make_bridge()
        self.assertEqual(bridge_default._feedforward_mode, "none")

    async def test_capture_flag_off_keeps_uncaptured_path(self) -> None:
        """`cuda_graph_capture=False` must never populate the graph cache."""
        if not _cuda_available():
            self.skipTest("no CUDA device")
        bridge, commands = _make_bridge(device="cuda:0", cuda_graph_capture=False)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        for _ in range(3):
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertEqual(len(bridge._ctx_cache[1].capture_cache), 0)

    async def test_cuda_graph_replay_is_bit_exact_and_activates_on_second_rollout(
        self,
    ) -> None:
        """Replayed rollouts must reproduce the uncaptured rollout exactly.

        The first rollout of a shape runs uncaptured (warm-up), the second is
        captured into CUDA graphs and replayed. Both run identical launch
        sequences on identical buffers, so positions, velocities, AND autograd
        gradients must match bit-for-bit — any deviation means the graph froze
        state it should not have.
        """
        if not _cuda_available():
            self.skipTest("no CUDA device")
        bridge, commands = _make_bridge(device="cuda:0")
        theta_np = self._theta0()
        results = []
        for _ in range(2):
            theta = torch.as_tensor(theta_np, dtype=torch.float32).reshape(1, -1).requires_grad_(True)
            rollout = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
            (rollout.positions.sum() + rollout.velocities.sum()).backward()
            results.append(
                (
                    rollout.positions.detach().cpu(),
                    rollout.velocities.detach().cpu(),
                    theta.grad.detach().cpu().clone(),
                )
            )
        ctx = bridge._ctx_cache[1]
        self.assertEqual(len(ctx.capture_cache), 1, "capture did not activate on the second rollout")
        self.assertFalse(ctx.capture_failed)
        for plain, replay in zip(results[0], results[1]):
            self.assertTrue(torch.equal(plain, replay))

    async def test_gradients_match_finite_differences_with_cuda_graph_capture(
        self,
    ) -> None:
        """FD-vs-AD parity through replayed forward AND backward graphs (<1%)."""
        if not _cuda_available():
            self.skipTest("no CUDA device")
        bridge, commands = _make_bridge(device="cuda:0")
        generator = torch.Generator().manual_seed(3)
        weights = (
            torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5,
            0.1 * (torch.rand(1, _NUM_STEPS, _NUM_DOF, generator=generator) - 0.5),
        )
        theta0 = self._theta0()

        async def loss_of(theta_np, requires_grad=False):
            theta = torch.as_tensor(theta_np, dtype=torch.float32).reshape(1, -1)
            if requires_grad:
                theta.requires_grad_(True)
            result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
            return theta, (result.positions.cpu() * weights[0]).sum() + (result.velocities.cpu() * weights[1]).sum()

        # Warm-up + capture happen inside the first two calls; the FD probes
        # below all run through graph replay.
        theta, loss = await loss_of(theta0, requires_grad=True)
        loss.backward()
        theta, loss = await loss_of(theta0, requires_grad=True)
        loss.backward()
        self.assertEqual(len(bridge._ctx_cache[1].capture_cache), 1)
        grad_autodiff = theta.grad.reshape(-1).double().numpy()
        steps = _gradcheck_steps(_GRADCHECK_ENTRIES)
        for index in range(len(theta0)):
            plus, minus = theta0.copy(), theta0.copy()
            plus[index] += steps[index]
            minus[index] -= steps[index]
            _, loss_plus = await loss_of(plus)
            _, loss_minus = await loss_of(minus)
            grad_fd = (float(loss_plus) - float(loss_minus)) / (2.0 * steps[index])
            with self.subTest(param=_GRADCHECK_ENTRIES[index].param_type.value):
                self.assertLess(
                    abs(grad_autodiff[index] - grad_fd) / max(abs(grad_fd), 1e-4),
                    1e-2,
                    msg=f"autodiff={grad_autodiff[index]:.6g} fd={grad_fd:.6g}",
                )

    async def test_cuda_graph_capture_with_inverse_dynamics_feedforward(self) -> None:
        """Capture composes with the feedforward path (ff array is part of the key)."""
        if not _cuda_available():
            self.skipTest("no CUDA device")
        bridge, commands = _make_bridge(device="cuda:0", feedforward="inverse_dynamics")
        times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
        q = 0.3 * np.sin(2.0 * np.pi * 1.2 * times)[:, None] * np.array([[1.0, -0.6]])
        qd = np.gradient(q, times, axis=0)
        bridge.set_trajectory(TrajectoryDataset(times=times, positions=q.copy(), velocities=qd, commands=q.copy()))
        commands_t = torch.as_tensor(q, dtype=torch.float32)
        theta_np = self._theta0()
        results = []
        for _ in range(2):
            theta = torch.as_tensor(theta_np, dtype=torch.float32).reshape(1, -1).requires_grad_(True)
            rollout = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands_t, _NUM_STEPS)
            rollout.positions.sum().backward()
            results.append((rollout.positions.detach().cpu(), theta.grad.detach().cpu().clone()))
        ctx = bridge._ctx_cache[1]
        self.assertEqual(len(ctx.capture_cache), 1)
        self.assertTrue(ctx.gravity_ff_active)
        for plain, replay in zip(results[0], results[1]):
            self.assertTrue(torch.equal(plain, replay))
            self.assertTrue(torch.all(torch.isfinite(replay)))

    async def test_same_trajectory_reset_reuses_feedforward_and_buffer_identity(
        self,
    ) -> None:
        """Re-setting the same trajectory object must not recompute the feedforward.

        The optimizer calls `set_trajectory` on every training-chunk switch,
        every iteration. The feedforward must stay cached and `gravity_ff_wp`
        must keep its identity — the CUDA graph capture key includes
        `id(gravity_ff_wp)`, so a replaced array forces a full recapture.
        """
        bridge, _ = _make_bridge(feedforward="gravity")
        trajectory = _hold_trajectory(0.4)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        calls, restore = _count_feedforward_computes()
        try:
            bridge.set_trajectory(trajectory)
            first = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
            ctx = bridge._ctx_cache[1]
            self.assertTrue(ctx.gravity_ff_active)
            self.assertEqual(calls["count"], 1)
            buffer_id = id(ctx.gravity_ff_wp)

            bridge.set_trajectory(trajectory)
            second = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        finally:
            restore()
        self.assertEqual(calls["count"], 1, "feedforward was recomputed for an unchanged trajectory")
        self.assertEqual(
            id(ctx.gravity_ff_wp),
            buffer_id,
            "gravity_ff_wp identity (capture key) changed",
        )
        self.assertTrue(torch.allclose(first.positions, second.positions, atol=1e-6))
        self.assertTrue(torch.allclose(first.torques, second.torques, atol=1e-5))

    async def test_chunk_alternation_reuses_cached_feedforward_per_chunk(self) -> None:
        """Alternating training chunks must hit the per-trajectory feedforward cache.

        One computation per chunk (not per rollout), a stable `gravity_ff_wp`
        identity throughout (stable capture key), and bufferwise-correct
        contents: each chunk's repeat rollout reproduces its first one even
        though the other chunk's feedforward was loaded in between.
        """
        bridge, _ = _make_bridge(feedforward="gravity")
        chunk_a = _hold_trajectory(0.5)
        chunk_b = _hold_trajectory(-0.3)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        calls, restore = _count_feedforward_computes()
        results: dict[str, list] = {"a": [], "b": []}
        buffer_ids = set()
        try:
            for label, chunk in (
                ("a", chunk_a),
                ("b", chunk_b),
                ("a", chunk_a),
                ("b", chunk_b),
            ):
                bridge.set_trajectory(chunk)
                commands = torch.as_tensor(chunk.commands, dtype=torch.float32)
                result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
                results[label].append(result)
                buffer_ids.add(id(bridge._ctx_cache[1].gravity_ff_wp))
        finally:
            restore()
        self.assertEqual(calls["count"], 2, "expected one feedforward computation per chunk")
        self.assertEqual(
            len(buffer_ids),
            1,
            "gravity_ff_wp identity (capture key) changed during alternation",
        )
        for label in ("a", "b"):
            self.assertTrue(torch.allclose(results[label][0].positions, results[label][1].positions, atol=1e-6))
            self.assertTrue(torch.allclose(results[label][0].torques, results[label][1].torques, atol=1e-5))
        # The two chunks genuinely load different feedforwards/commands.
        self.assertFalse(torch.allclose(results["a"][0].positions, results["b"][0].positions, atol=1e-3))

    async def test_feedforward_recomputes_when_trajectory_mutated_in_place(
        self,
    ) -> None:
        """The identity-keyed cache must verify content: in-place edits invalidate it."""
        bridge, _ = _make_bridge(feedforward="gravity")
        chunk = _hold_trajectory(0.4)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        calls, restore = _count_feedforward_computes()
        try:
            bridge.set_trajectory(chunk)
            commands = torch.as_tensor(chunk.commands, dtype=torch.float32)
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
            self.assertEqual(calls["count"], 1)

            chunk.positions += 0.2  # same object identity, different content
            bridge.set_trajectory(chunk)
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        finally:
            restore()
        self.assertEqual(
            calls["count"],
            2,
            "mutated trajectory content must recompute the feedforward",
        )

    async def test_cuda_graph_capture_is_reused_across_chunk_alternation(self) -> None:
        """Chunk alternation must replay one captured graph, not recapture per rollout.

        Same rollout shape + stable `gravity_ff_wp` identity mean one capture
        key: the graph is captured once (on the second rollout) and every later
        rollout replays it with in-place-updated feedforward/command contents.
        Each chunk's replayed rollout must reproduce its uncaptured one.
        """
        if not _cuda_available():
            self.skipTest("no CUDA device")
        import isaacsim.robot_setup.sysid.newton_diff_sysid_bridge as diff_module

        bridge, _ = _make_bridge(device="cuda:0", feedforward="gravity")
        chunk_a = _hold_trajectory(0.5)
        chunk_b = _hold_trajectory(-0.3)
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)

        captures = {"count": 0}
        original = diff_module._capture_rollout_graphs

        def counting(*args, **kwargs):
            captures["count"] += 1
            return original(*args, **kwargs)

        diff_module._capture_rollout_graphs = counting
        results: dict[str, list] = {"a": [], "b": []}
        try:
            for label, chunk in (
                ("a", chunk_a),
                ("b", chunk_b),
                ("a", chunk_a),
                ("b", chunk_b),
                ("a", chunk_a),
            ):
                bridge.set_trajectory(chunk)
                commands = torch.as_tensor(chunk.commands, dtype=torch.float32)
                result = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
                results[label].append(result.positions.detach().cpu())
        finally:
            diff_module._capture_rollout_graphs = original
        ctx = bridge._ctx_cache[1]
        self.assertFalse(ctx.capture_failed)
        self.assertEqual(captures["count"], 1, "chunk alternation triggered a graph recapture")
        self.assertEqual(
            len(ctx.capture_cache),
            1,
            "expected exactly one capture record for one shape",
        )
        self.assertTrue(ctx.gravity_ff_active)
        for label in ("a", "b"):
            for later in results[label][1:]:
                self.assertTrue(
                    torch.allclose(results[label][0], later, atol=1e-6),
                    msg=f"replayed chunk {label} rollout deviates from its uncaptured rollout",
                )
        self.assertFalse(torch.allclose(results["a"][0], results["b"][0], atol=1e-3))

    async def test_release_rollout_memory_frees_buffers_and_disables_capture(
        self,
    ) -> None:
        """Post-solve release must drop rollout buffers and latch capture off bridge-wide.

        The run session calls this between the solve and the forward-only
        validation/confidence stages; a context built AFTER the release (the
        confidence batch's multi-world model) must not attempt capture either,
        or it would re-pin the multi-GiB graph memory the release just freed.
        """
        bridge, commands = _make_bridge()
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        baseline = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        ctx = bridge._ctx_cache[1]
        self.assertGreater(len(ctx.states), 0)

        bridge.release_rollout_memory()
        self.assertFalse(bridge._capture_enabled)
        self.assertEqual(len(ctx.states), 0)
        self.assertEqual(len(ctx.controls), 0)
        self.assertEqual(len(ctx.solvers), 0)
        self.assertIsNone(ctx.sim_q)
        self.assertIsNone(ctx.sim_qd)
        self.assertIsNone(ctx.sim_tau)
        self.assertIsNone(ctx.final_joint_f)
        self.assertEqual(len(ctx.capture_cache), 0)
        self.assertFalse(bridge._capture_supported(ctx))

        # Buffers rebuild lazily; the released bridge must reproduce the rollout,
        # and a context built after the release must also refuse capture.
        again = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertTrue(torch.allclose(baseline.positions, again.positions, atol=1e-6))
        theta_batch = theta.repeat(2, 1)
        await bridge.run_rollout_async(theta_batch, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertFalse(bridge._capture_supported(bridge._ctx_cache[2]))

    async def test_buffer_allocation_failure_reclaims_memory_and_retries(self) -> None:
        """An OOM growing the per-substep state lists must fall back, not abort.

        The confidence stage's full-trajectory nominal rollout is a NEW, longer
        shape: its state-list growth is where a 16 GB card dies. One failure
        must trigger the reclaim-and-retry path (capture caches dropped,
        growth resumed), producing the same rollout an unpressured bridge does.
        """
        bridge, commands = _make_bridge()
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, 10)
        ctx = bridge._ctx_cache[1]

        real_state = ctx.model.state
        failures = {"count": 0}

        def failing_state(*args, **kwargs):
            if failures["count"] == 0:
                failures["count"] += 1
                raise RuntimeError("Failed to allocate 4096 bytes on device 'cuda:0'")
            return real_state(*args, **kwargs)

        ctx.model.state = failing_state
        try:
            recovered = await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        finally:
            ctx.model.state = real_state
        self.assertEqual(failures["count"], 1)
        self.assertTrue(ctx.capture_failed, "memory pressure must latch capture off")
        self.assertTrue(torch.all(torch.isfinite(recovered.positions)))

        reference_bridge, _ = _make_bridge()
        reference = await reference_bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertTrue(torch.allclose(recovered.positions, reference.positions, atol=1e-6))

    async def test_buffer_allocation_failure_after_retry_raises_bridge_error(
        self,
    ) -> None:
        """A persistent allocation failure must surface as the bridge's error type.

        The run session's confidence stage catches exceptions and degrades to an
        empty payload with a logged reason instead of killing the whole run.
        """
        bridge, commands = _make_bridge()
        theta = torch.as_tensor(self._theta0(), dtype=torch.float32).reshape(1, -1)
        await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, 10)
        ctx = bridge._ctx_cache[1]

        def always_failing_state(*args, **kwargs):
            raise RuntimeError("Failed to allocate 4096 bytes on device 'cuda:0'")

        ctx.model.state = always_failing_state
        with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
            await bridge.run_rollout_async(theta, _GRADCHECK_ENTRIES, commands, _NUM_STEPS)
        self.assertIn("after releasing", str(raised.exception))

    async def test_stiff_held_joint_gains_are_capped_for_stable_gradients(self) -> None:
        """A held joint with real-robot drive gains (ke ~ 1e6) must not destabilize the tape.

        Imported gains on held joints routinely violate the explicit-integrator
        stability bound at sub_dt. The forward rollout can look bounded (joint
        limits saturate the unstable mode) while the adjoint follows the
        unsaturated linearization and grows exponentially per taped substep.
        The bridge caps held-joint gains into the stability region, so both the
        rollout and its gradient stay finite; mapped joints are untouched.
        """
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        links = [
            builder.add_link(
                mass=1.0,
                com=(0.0, 0.0, -0.1),
                inertia=np.diag([0.01, 0.012, 0.014]).tolist(),
            )
            for _ in range(3)
        ]
        joints = []
        parent = -1
        for index, link in enumerate(links):
            stiff_held = index == 1  # middle joint is held (not in the trajectory)
            joints.append(
                builder.add_joint_revolute(
                    parent=parent,
                    child=link,
                    axis=(0.0, 1.0, 0.0),
                    parent_xform=wp.transform((0.0, 0.0, 1.0 if index == 0 else -0.2), wp.quat_identity()),
                    child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                    target_ke=1.0e6 if stiff_held else 5.0,
                    target_kd=100.0 if stiff_held else 0.4,
                    label=f"/robot/joint{index}",
                )
            )
            parent = link
        builder.add_articulation(joints, label="chain")

        config = NewtonSimulationRunSpec(
            device="cpu",
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=2,
        )
        bridge = NewtonDifferentiableSysIdBridge(
            robot_prim_path="/robot",
            newton_config=config,
            joint_baselines=[
                SimpleNamespace(
                    stiffness=5.0,
                    damping=0.4,
                    friction=0.1,
                    dynamic_friction=0.1,
                    armature=0.1,
                )
            ]
            * 2,
            joint_paths=["/robot/joint0", "/robot/joint2"],
            num_joints=2,
            num_links=3,
            robot_builder=builder,
        )
        num_steps = 120
        times = np.arange(num_steps, dtype=np.float64) * _DT
        zeros = np.zeros((num_steps, 2), dtype=np.float64)
        commands = 0.3 * np.sin(2.0 * np.pi * 1.5 * times)[:, None] * np.array([[1.0, -1.0]])
        bridge.set_trajectory(
            TrajectoryDataset(
                times=times,
                positions=zeros.copy(),
                velocities=zeros.copy(),
                commands=commands,
            )
        )
        entries = [
            _entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=0),
            _entry(SysIdParameterType.JOINT_STIFFNESS, dof_index=1),
        ]
        theta = torch.tensor([[1.0, 1.0]], requires_grad=True)
        result = await bridge.run_rollout_async(
            theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps
        )
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        result.positions.sum().backward()
        self.assertTrue(torch.all(torch.isfinite(theta.grad)))
        # A stable adjoint keeps gradients at physical magnitudes; the unstable
        # held-joint mode used to inflate them by many orders of magnitude.
        self.assertLess(float(theta.grad.abs().max()), 1.0e6)

    @staticmethod
    def _resting_friction_fixture(
        damping: float = 0.4,
        substeps: int = 2,
        link_paths: list[str] | None = None,
        passive_damping: tuple[float, float] = (0.0, 0.0),
    ) -> tuple[NewtonDifferentiableSysIdBridge, list, np.ndarray, int]:
        """Build the high-friction two-link chain used by the friction tests.

        Args:
            damping: Authored drive damping on both joints.
            substeps: Configured ``featherstone_substeps``.
            link_paths: SysID link order; the Newton bodies are labelled
                ``/robot/link0`` and ``/robot/link1`` in body order.
            passive_damping: Model ``joint_damping`` per joint.

        Returns:
            Bridge, friction parameter entries, command array, and step count.
        """
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        links = [
            builder.add_link(
                mass=1.0,
                com=(0.0, 0.0, -0.1),
                inertia=np.diag([0.01, 0.012, 0.014]).tolist(),
                label=f"/robot/link{index}",
            )
            for index in range(2)
        ]
        joints = []
        parent = -1
        for index, link in enumerate(links):
            joints.append(
                builder.add_joint_revolute(
                    parent=parent,
                    child=link,
                    axis=(0.0, 1.0, 0.0),
                    parent_xform=wp.transform((0.0, 0.0, 1.0 if index == 0 else -0.2), wp.quat_identity()),
                    child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                    target_ke=5.0,
                    target_kd=damping,
                    label=f"/robot/joint{index}",
                )
            )
            parent = link
        builder.add_articulation(joints, label="chain")
        for dof, value in enumerate(passive_damping):
            builder.joint_damping[dof] = value

        config = NewtonSimulationRunSpec(
            device="cpu",
            solver=NEWTON_SOLVER_FEATHERSTONE_DIFF,
            featherstone_substeps=substeps,
        )
        bridge = NewtonDifferentiableSysIdBridge(
            robot_prim_path="/robot",
            newton_config=config,
            # High Coulomb friction with no armature: f * eps = 500 against an
            # effective inertia near 0.02 kg m^2 is far outside the explicit
            # integrator's stability region at this sub_dt.
            joint_baselines=[
                SimpleNamespace(stiffness=5.0, damping=damping, friction=5.0, dynamic_friction=5.0, armature=0.0)
            ]
            * 2,
            joint_paths=["/robot/joint0", "/robot/joint1"],
            link_paths=link_paths,
            num_joints=2,
            num_links=2,
            robot_builder=builder,
        )
        num_steps = 120
        times = np.arange(num_steps, dtype=np.float64) * _DT
        # Hold at rest for the first fifth of the chunk, then excite. The rest
        # segment is what parks qd inside the high-gain band of the tanh.
        ramp = np.clip((times - 0.2) / 0.2, 0.0, 1.0)
        commands = (0.3 * ramp * np.sin(2.0 * np.pi * 1.5 * times))[:, None] * np.array([[1.0, -1.0]])
        zeros = np.zeros((num_steps, 2), dtype=np.float64)
        bridge.set_trajectory(
            TrajectoryDataset(
                times=times,
                positions=zeros.copy(),
                velocities=zeros.copy(),
                commands=commands,
            )
        )
        entries = [
            _entry(SysIdParameterType.JOINT_FRICTION, dof_index=0),
            _entry(SysIdParameterType.JOINT_FRICTION, dof_index=1),
        ]
        return bridge, entries, commands, num_steps

    async def test_high_friction_keeps_gradients_finite_on_resting_joints(self) -> None:
        """Smooth Coulomb friction must not make the reverse pass diverge.

        ``-friction * tanh(qd * eps)`` saturates in the forward rollout but
        linearizes to an effective damping of ``friction * eps`` at ``qd = 0``.
        A joint that dwells near zero velocity (every excitation trajectory
        starts from rest) compounds that stiffness once per taped substep, so a
        fixed ``eps`` used to overflow the adjoint to NaN while the forward
        states stayed perfectly finite. Capping the slope removes that mode.
        """
        bridge, entries, commands, num_steps = self._resting_friction_fixture()
        # 10.0 is the largest friction scale the parameter registry allows the
        # optimizer to propose, so the adjoint has to survive the whole range.
        for scale in (1.0, 4.0, 10.0):
            theta = torch.tensor([[scale, scale]], requires_grad=True)
            result = await bridge.run_rollout_async(
                theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps
            )
            # The forward rollout was always finite; the regression is in the adjoint.
            self.assertTrue(torch.all(torch.isfinite(result.positions)), msg=f"scale={scale}")
            result.positions.sum().backward()
            self.assertTrue(torch.all(torch.isfinite(theta.grad)), msg=f"scale={scale}")
            self.assertLess(float(theta.grad.abs().max()), 1.0e6, msg=f"scale={scale}")

        # Pin the mechanism, not just the symptom: this model cannot carry the
        # requested velocity scale, so the kernel must be capping it.
        ctx = bridge._ctx_cache[1]
        budget = ctx.friction_slope_budget_np
        self.assertTrue(np.all(budget < FRICTION_INV_VELOCITY_EPS * 5.0), msg=f"budget={budget}")

    async def test_slope_budget_is_resized_when_the_sample_rate_changes(self) -> None:
        """A chunk recorded at another rate must not reuse the cached budget.

        The context is cached per world count, but the budget scales with
        `sub_dt`, so multiple-shooting over mixed-rate chunks would otherwise
        integrate friction against a budget sized for a different timestep.
        """
        bridge, entries, commands, num_steps = self._resting_friction_fixture()

        theta = torch.tensor([[1.0, 1.0]], requires_grad=True)
        result = await bridge.run_rollout_async(
            theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps
        )
        result.positions.sum().backward()
        fast = np.asarray(bridge._ctx_cache[1].friction_slope_budget_np, dtype=np.float64).copy()

        # Same joints and world count, half the sample period.
        fast_times = np.arange(num_steps, dtype=np.float64) * (_DT * 0.5)
        zeros = np.zeros((num_steps, 2), dtype=np.float64)
        bridge.set_trajectory(
            TrajectoryDataset(times=fast_times, positions=zeros.copy(), velocities=zeros.copy(), commands=commands)
        )
        theta = torch.tensor([[1.0, 1.0]], requires_grad=True)
        result = await bridge.run_rollout_async(
            theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps
        )
        result.positions.sum().backward()
        halved = np.asarray(bridge._ctx_cache[1].friction_slope_budget_np, dtype=np.float64)

        # Half the sub_dt carries twice the slope.
        np.testing.assert_allclose(halved, fast * 2.0, rtol=1e-6)

    async def test_inertia_probe_follows_newton_body_order(self) -> None:
        """A link parameter must floor the body the rollout will actually change.

        The rollout translates SysID link indices to Newton bodies before
        decoding, so a probe that skipped that translation would size the
        budget for the wrong link whenever the two orders differ.
        """
        bridge, entries, commands, num_steps = self._resting_friction_fixture(
            link_paths=["/robot/link1", "/robot/link0"]
        )
        # SysID link 0 is Newton body 1.
        entries = [*entries, _entry(SysIdParameterType.LINK_MASS, link_index=0)]
        theta = torch.tensor([[1.0, 1.0, 1.0]])
        await bridge.run_rollout_async(theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps)

        floor, _maximum = default_bounds_for_type(SysIdParameterType.LINK_MASS)
        scales = [tuple(corner.mass_scale.tolist()) for corner in bridge._ctx_cache[1].probed_corners]
        self.assertIn((1.0, floor), scales)
        self.assertNotIn((floor, 1.0), scales)

    async def test_passive_joint_damping_is_charged_against_the_budget(self) -> None:
        """The model's joint damping spends the same stability budget as the kernel.

        Featherstone applies ``joint_damping`` explicitly next to the kernel's
        drive and friction torque, so a budget that ignored it would let the
        total slope pass the integrator's limit.
        """
        passive = (0.0, 0.3)
        bridge, entries, commands, num_steps = self._resting_friction_fixture(passive_damping=passive)
        theta = torch.tensor([[1.0, 1.0]])
        await bridge.run_rollout_async(theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps)

        ctx = bridge._ctx_cache[1]
        np.testing.assert_allclose(ctx.passive_damping_np, passive, rtol=1e-6)
        total = _friction_slope_budget(ctx.mapped_inertia_np, ctx.mapped_coupling, ctx.substeps, _DT)
        np.testing.assert_allclose(ctx.friction_slope_budget_np, total - np.asarray(passive), rtol=1e-6)

    async def test_substeps_are_raised_until_damping_leaves_friction_its_share(self) -> None:
        """Damping the configured sub_dt cannot carry raises the substep count.

        Raising the inertia the budget is sized against instead would admit a
        total slope past the stability limit at the lowest reachable inertia,
        which is where the reverse pass diverges.
        """
        bridge, entries, commands, num_steps = self._resting_friction_fixture(damping=2.0, substeps=1)
        theta = torch.tensor([[1.0, 1.0]], requires_grad=True)
        with patch("carb.log_warn") as log_warn:
            result = await bridge.run_rollout_async(
                theta, entries, torch.as_tensor(commands, dtype=torch.float32), num_steps
            )
        result.positions.sum().backward()
        self.assertTrue(torch.all(torch.isfinite(theta.grad)))

        ctx = bridge._ctx_cache[1]
        self.assertGreater(ctx.substeps, 1)
        self.assertTrue(
            any(f"featherstone_substeps={ctx.substeps}" in str(call.args[0]) for call in log_warn.call_args_list)
        )
        self.assertEqual(bridge.actuator_metadata["featherstone_substeps_effective"], ctx.substeps)
        total = _friction_slope_budget(ctx.mapped_inertia_np, ctx.mapped_coupling, ctx.substeps, _DT)
        self.assertTrue(np.all(2.0 <= (1.0 - FRICTION_BUDGET_SHARE) * total * (1.0 + 1e-9)), msg=f"total={total}")
