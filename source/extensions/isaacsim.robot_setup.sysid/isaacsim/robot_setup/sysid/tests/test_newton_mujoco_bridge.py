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

# ruff: noqa: ANN001, ANN002, ANN003, ANN202, D101, D102, D401

"""Tests for the SolverMuJoCo rollout path of the Newton SysID bridge.

The genuine-physics tests skip when the optional Newton/MuJoCo-Warp modules are
unavailable (a ``SolverMuJoCo`` is constructed once on a probe model to verify
the backend actually runs in this environment).
"""

from __future__ import annotations

from importlib import metadata
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.env_bridge import SysIdEnvironmentBridgeError
from isaacsim.robot_setup.sysid.inertia_param import (
    inertia_matrix_to_log_cholesky,
    log_cholesky_to_inertia_matrix,
)
from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
    NEWTON_CLAMP_DC_MOTOR,
    NEWTON_CLAMP_MAX_EFFORT,
    NEWTON_CONTROLLER_NEURAL_MLP,
    NewtonActuatorConfig,
    NewtonSysIdBridge,
    _replace_invalid_rollout_worlds,
    _reported_net_joint_effort,
    classify_newton_neural_prim,
    newton_dependency_compatibility,
    newton_modules_available,
)
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.run_spec import (
    NEWTON_SOLVER_MUJOCO,
    NewtonSimulationRunSpec,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset

_NEWTON_AVAILABLE, _NEWTON_REASON = newton_modules_available()

_NUM_STEPS = 30
_DT = 0.01
# Scales the 5 N*m/rad baseline drive to 5e4 N*m/rad on a ~0.07 kg*m^2 link:
# omega*dt ~ 8, far past MuJoCo's explicit-stiffness bound of 2 at the 10 ms sample.
_DIVERGENT_STIFFNESS_SCALE = 1.0e4

_MUJOCO_PROBE: tuple[bool, str] | None = None


class NewtonDependencyCompatibilityTests(omni.kit.test.AsyncTestCase):
    async def test_kit_module_metadata_aliases_are_accepted(self) -> None:
        """Accept underscore metadata keys used by Kit's bundled pip importer."""
        versions = {
            "newton": "1.6.1rc1",
            "newton_usd_schemas": "0.5.0",
            "mujoco": "3.12.0",
            "mujoco_warp": "3.12.0",
        }

        def _version(name: str) -> str:
            if name in versions:
                return versions[name]
            raise metadata.PackageNotFoundError(name)

        with patch("isaacsim.robot_setup.sysid.newton_sysid_bridge.metadata.version", side_effect=_version):
            available, reason = newton_dependency_compatibility()

        self.assertTrue(available, msg=reason)
        self.assertEqual(reason, "")

    async def test_alias_version_mismatch_uses_canonical_distribution_name(self) -> None:
        """Keep diagnostics actionable even when the Kit metadata alias was queried."""
        versions = {
            "newton": "1.6.1rc1",
            "newton_usd_schemas": "0.5.0",
            "mujoco": "3.12.0",
            "mujoco_warp": "3.9.0",
        }

        def _version(name: str) -> str:
            if name in versions:
                return versions[name]
            raise metadata.PackageNotFoundError(name)

        with patch("isaacsim.robot_setup.sysid.newton_sysid_bridge.metadata.version", side_effect=_version):
            available, reason = newton_dependency_compatibility()

        self.assertFalse(available)
        self.assertIn("mujoco-warp==3.9.0", reason)


class ReportedNetJointEffortTests(omni.kit.test.AsyncTestCase):
    async def test_bridge_advertises_replicated_candidate_batches(self) -> None:
        self.assertTrue(NewtonSysIdBridge.supports_arbitrary_candidate_batch)

    async def test_friction_is_included_before_max_effort_saturation(self) -> None:
        actuator_effort = torch.tensor([[5.0]], dtype=torch.float32)
        velocity = torch.tensor([[0.1]], dtype=torch.float32)
        friction = torch.tensor([[0.5]], dtype=torch.float32)
        configs = [
            NewtonActuatorConfig(
                dof_index=0,
                clamp_kind=NEWTON_CLAMP_MAX_EFFORT,
                effort_limit=2.0,
            )
        ]
        reported = _reported_net_joint_effort(actuator_effort, velocity, friction, configs)
        self.assertAlmostEqual(float(reported[0, 0]), 2.0, places=6)

    async def test_dc_motor_clamp_uses_speed_dependent_effort_envelope(self) -> None:
        actuator_effort = torch.tensor([[9.0]], dtype=torch.float32)
        velocity = torch.tensor([[5.0]], dtype=torch.float32)
        friction = torch.zeros_like(velocity)
        configs = [
            NewtonActuatorConfig(
                dof_index=0,
                clamp_kind=NEWTON_CLAMP_DC_MOTOR,
                max_motor_effort=8.0,
                saturation_effort=10.0,
                velocity_limit=10.0,
            )
        ]
        reported = _reported_net_joint_effort(actuator_effort, velocity, friction, configs)
        self.assertAlmostEqual(float(reported[0, 0]), 5.0, places=6)


def _mujoco_solver_available() -> tuple[bool, str]:
    """Probe (once) that ``newton.solvers.SolverMuJoCo`` can be built and run here.

    Returns:
        Result produced by the operation.
    """
    global _MUJOCO_PROBE
    if _MUJOCO_PROBE is not None:
        return _MUJOCO_PROBE
    if not _NEWTON_AVAILABLE:
        _MUJOCO_PROBE = (False, _NEWTON_REASON)
        return _MUJOCO_PROBE
    try:
        import newton
        import warp as wp
        from newton.solvers import SolverMuJoCo

        builder = newton.ModelBuilder()
        link = builder.add_link(mass=1.0, com=(0.0, 0.0, -0.1), inertia=np.diag([0.01, 0.01, 0.01]).tolist())
        joint = builder.add_joint_revolute(
            parent=-1,
            child=link,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            target_ke=5.0,
            target_kd=0.4,
        )
        builder.add_articulation([joint], label="probe")
        model = builder.finalize()
        solver = SolverMuJoCo(model)
        solver.step(model.state(), model.state(), model.control(), None, _DT)
    except Exception as exc:  # missing mujoco/mujoco_warp, or backend cannot run here
        _MUJOCO_PROBE = (False, str(exc))
        return _MUJOCO_PROBE
    _MUJOCO_PROBE = (True, "")
    return _MUJOCO_PROBE


def _joint_snapshot() -> SimpleNamespace:
    return SimpleNamespace(
        stiffness=5.0,
        damping=0.4,
        friction=0.1,
        dynamic_friction=0.1,
        armature=0.05,
        max_force=float("inf"),
    )


def _build_chain(num_joints: int):
    """Serial revolute chain with USD-style joint labels ``/robot/joint{i}``.

    Args:
        num_joints: Value supplied for ``num_joints``.

    Returns:
        Result produced by the operation.
    """
    import newton
    import warp as wp

    builder = newton.ModelBuilder()
    joints = []
    parent = -1
    for index in range(num_joints):
        link = builder.add_link(
            mass=1.0,
            com=(0.0, 0.0, -0.1),
            inertia=np.diag([0.01, 0.012, 0.014]).tolist(),
            label=f"/robot/link{index}",
        )
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
        parent = link
    builder.add_articulation(joints, label="chain")
    return builder


def _build_floating_pendulum():
    """Build a free root followed by one actuated revolute joint.

    Returns:
        Newton model builder containing the floating-base articulation.
    """
    import newton
    import warp as wp

    builder = newton.ModelBuilder()
    base = builder.add_link(
        mass=2.0,
        com=(0.0, 0.0, 0.0),
        inertia=np.diag([0.1, 0.1, 0.1]).tolist(),
        label="/robot/base",
    )
    base_joint = builder.add_joint_free(child=base)
    arm = builder.add_link(
        mass=0.7,
        com=(0.0, 0.0, -0.1),
        inertia=np.diag([0.01, 0.015, 0.02]).tolist(),
        label="/robot/arm",
    )
    revolute_joint = builder.add_joint_revolute(
        parent=base,
        child=arm,
        axis=(0.0, 1.0, 0.0),
        parent_xform=wp.transform((0.0, 0.0, 0.2), wp.quat_identity()),
        child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
        target_ke=4.0,
        target_kd=0.3,
        label="/robot/joint0",
    )
    builder.add_articulation([base_joint, revolute_joint], label="floating")
    return builder


def _sine_trajectory(num_dof: int) -> TrajectoryDataset:
    times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
    zeros = np.zeros((_NUM_STEPS, num_dof), dtype=np.float64)
    signs = np.array([[1.0 if i % 2 == 0 else -1.0 for i in range(num_dof)]])
    commands = 0.3 * np.sin(2.0 * np.pi * 1.5 * times)[:, None] * signs
    return TrajectoryDataset(times=times, positions=zeros.copy(), velocities=zeros.copy(), commands=commands)


def _stiffness_entries(num_dof: int) -> list[SysIdParameterEntry]:
    return [
        SysIdParameterEntry(param_type=SysIdParameterType.JOINT_STIFFNESS, dof_index=index) for index in range(num_dof)
    ]


def _hold_trajectory(value: float, num_steps: int = _NUM_STEPS) -> TrajectoryDataset:
    """Constant-pose trajectory chunk; distinct `value`s give distinct feedforwards.

    Args:
        value: Value supplied for ``value``.
        num_steps: Value supplied for ``num_steps``.

    Returns:
        Result produced by the operation.
    """
    times = np.arange(num_steps, dtype=np.float64) * _DT
    hold = np.full((num_steps, 2), value, dtype=np.float64) * np.array([[1.0, -0.5]])
    return TrajectoryDataset(times=times, positions=hold.copy(), velocities=np.zeros_like(hold), commands=hold.copy())


def _count_feedforward_computes():
    """Patch the module-level feedforward computation with a counting wrapper.

    Returns:
        Result produced by the operation.
    """
    import isaacsim.robot_setup.sysid.newton_sysid_bridge as bridge_module

    calls = {"count": 0}
    original = bridge_module.compute_feedforward

    def counting(*args, **kwargs):
        calls["count"] += 1
        return original(*args, **kwargs)

    bridge_module.compute_feedforward = counting
    return calls, lambda: setattr(bridge_module, "compute_feedforward", original)


class NewtonMujocoBridgeRolloutTests(omni.kit.test.AsyncTestCase):
    """Genuine SolverMuJoCo rollouts on programmatic chains (no USD stage required)."""

    def setUp(self) -> None:
        available, reason = _mujoco_solver_available()
        if not available:
            self.skipTest(f"mujoco-warp unavailable: {reason}")

    def _make_bridge(
        self,
        builder,
        joint_paths: list[str],
        *,
        link_paths: list[str] | None = None,
        cuda_graph_capture: bool = True,
        mujoco_substeps: int = 1,
    ) -> NewtonSysIdBridge:
        num_dof = len(joint_paths)
        return NewtonSysIdBridge(
            robot_prim_path="/robot",
            newton_config=NewtonSimulationRunSpec(
                solver=NEWTON_SOLVER_MUJOCO,
                cuda_graph_capture=cuda_graph_capture,
                mujoco_substeps=mujoco_substeps,
            ),
            joint_baselines=[_joint_snapshot() for _ in range(num_dof)],
            joint_paths=joint_paths,
            link_paths=link_paths,
            num_joints=num_dof,
            robot_builder=builder,
        )

    async def _rollout(self, bridge: NewtonSysIdBridge, num_dof: int, world_count: int = 1):
        trajectory = _sine_trajectory(num_dof)
        bridge.set_trajectory(trajectory)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.ones((world_count, num_dof), dtype=torch.float32)
        return await bridge.run_rollout_async(theta, _stiffness_entries(num_dof), commands, _NUM_STEPS)

    async def test_genuine_rollout_runs_when_trajectory_covers_all_joints(self) -> None:
        bridge = self._make_bridge(_build_chain(2), ["/robot/joint0", "/robot/joint1"])
        result = await self._rollout(bridge, num_dof=2)
        self.assertEqual(tuple(result.positions.shape), (1, _NUM_STEPS, 2))
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        self.assertTrue(torch.all(torch.isfinite(result.velocities)))
        self.assertTrue(torch.all(torch.isfinite(result.torques)))

    async def test_trajectory_may_cover_subset_of_actuated_joints(self) -> None:
        """A 3-joint model with 2-DOF telemetry maps by joint path; the extra joint holds.

        This is the Franka-arm-telemetry shape (7 mapped joints, 2 held gripper
        DOFs): the MuJoCo path must map the requested joints directly.
        """
        bridge = self._make_bridge(_build_chain(3), ["/robot/joint0", "/robot/joint2"])
        result = await self._rollout(bridge, num_dof=2)
        self.assertEqual(tuple(result.positions.shape), (1, _NUM_STEPS, 2))
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        # Both mapped joints respond to their commands.
        self.assertGreater(float(result.positions[0, :, 0].abs().max()), 0.02)
        self.assertGreater(float(result.positions[0, :, 1].abs().max()), 0.02)
        # The mapping resolved by joint path, not by taking the first two joints.
        self.assertEqual(len(bridge._joint_mapping_summary), 2)
        self.assertIn("joint2", bridge._joint_mapping_summary[1])

    async def test_candidate_batch_maps_each_world_independently(self) -> None:
        """Per-world parameter scatter through the DOF map keeps candidates independent."""
        bridge = self._make_bridge(_build_chain(3), ["/robot/joint0", "/robot/joint2"])
        trajectory = _sine_trajectory(2)
        bridge.set_trajectory(trajectory)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.tensor([[1.0, 1.0], [4.0, 4.0], [1.0, 1.0]], dtype=torch.float32)
        result = await bridge.run_rollout_async(theta, _stiffness_entries(2), commands, _NUM_STEPS)
        self.assertEqual(tuple(result.positions.shape), (3, _NUM_STEPS, 2))
        self.assertTrue(torch.allclose(result.positions[0], result.positions[2], atol=1e-5))
        self.assertFalse(torch.allclose(result.positions[0], result.positions[1], atol=1e-4))

    async def test_floating_base_uses_coordinate_layout_for_position_targets(self) -> None:
        """Keep position targets separate from the velocity-DOF layout."""
        bridge = self._make_bridge(_build_floating_pendulum(), ["/robot/joint0"])

        result = await self._rollout(bridge, num_dof=1, world_count=2)

        ctx = bridge._mujoco_cache[2]
        self.assertEqual(tuple(result.positions.shape), (2, _NUM_STEPS, 1))
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        self.assertEqual(ctx.dof_map_np.tolist(), [6, 13])
        self.assertEqual(ctx.target_pos_map_np.tolist(), [7, 15])
        self.assertEqual(int(ctx.target_pos_stage.shape[1]), int(ctx.model.joint_target_q.shape[0]))
        expected = np.tile(_sine_trajectory(1).commands, (1, 2))
        np.testing.assert_allclose(
            ctx.target_pos_stage.numpy()[:_NUM_STEPS, ctx.target_pos_map_np],
            expected,
            atol=1e-6,
        )

    async def test_link_inertial_candidates_refresh_and_reuse_cuda_graph(self) -> None:
        """Mass, COM, and SPD inertia vary per world without graph recapture."""
        joint_paths = ["/robot/joint0", "/robot/joint1"]
        link_paths = ["/robot/link0", "/robot/link1"]
        bridge = self._make_bridge(
            _build_chain(2),
            joint_paths,
            link_paths=link_paths,
        )
        plain_bridge = self._make_bridge(
            _build_chain(2),
            joint_paths,
            link_paths=link_paths,
            cuda_graph_capture=False,
        )
        trajectory = _sine_trajectory(2)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        for candidate in (bridge, plain_bridge):
            candidate.set_trajectory(trajectory)

        entries = [
            SysIdParameterEntry(
                param_type=SysIdParameterType.LINK_MASS,
                dof_index=-1,
                link_index=0,
            ),
            SysIdParameterEntry(
                param_type=SysIdParameterType.LINK_COM_OFFSET_X,
                dof_index=-1,
                link_index=0,
            ),
        ]
        entries.extend(
            SysIdParameterEntry(
                param_type=SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
                dof_index=-1,
                link_index=1,
                component_index=component,
            )
            for component in range(6)
        )
        baseline_lc = inertia_matrix_to_log_cholesky(np.diag([0.01, 0.012, 0.014])).astype(np.float32)
        inertia_a = baseline_lc.copy()
        inertia_a[[0, 2, 5]] += np.float32(0.08)
        inertia_b = baseline_lc.copy()
        inertia_b[[0, 2, 5]] -= np.float32(0.05)
        theta_a = torch.as_tensor(
            np.stack(
                [
                    np.concatenate(([1.0, 0.0], baseline_lc)),
                    np.concatenate(([1.4, 0.02], inertia_a)),
                ]
            ),
            dtype=torch.float32,
        )
        theta_b = torch.as_tensor(
            np.stack(
                [
                    np.concatenate(([1.0, 0.0], baseline_lc)),
                    np.concatenate(([0.75, -0.015], inertia_b)),
                ]
            ),
            dtype=torch.float32,
        )

        await bridge.run_rollout_async(theta_a, entries, commands, _NUM_STEPS)
        ctx = bridge._mujoco_cache[2]
        if not bridge._capture_supported(ctx):
            self.skipTest(f"MuJoCo model device {ctx.model.device} is not CUDA")
        await bridge.run_rollout_async(theta_a, entries, commands, _NUM_STEPS)
        self.assertEqual(len(ctx.capture_cache), 1)

        replay = await bridge.run_rollout_async(theta_b, entries, commands, _NUM_STEPS)
        plain = await plain_bridge.run_rollout_async(theta_b, entries, commands, _NUM_STEPS)
        self.assertEqual(len(ctx.capture_cache), 1, "inertial values triggered recapture")
        self.assertTrue(torch.all(torch.isfinite(replay.positions)))
        self.assertFalse(torch.allclose(replay.positions[0], replay.positions[1], atol=1e-5))
        self.assertTrue(torch.allclose(replay.positions, plain.positions, atol=3e-5, rtol=3e-5))
        self.assertTrue(torch.allclose(replay.velocities, plain.velocities, atol=3e-4, rtol=3e-5))
        self.assertTrue(torch.allclose(replay.torques, plain.torques, atol=3e-3, rtol=3e-5))

        masses = ctx.model.body_mass.numpy().reshape(2, ctx.bodies_per_world)
        coms = ctx.model.body_com.numpy().reshape(2, ctx.bodies_per_world, 3)
        inertias = ctx.model.body_inertia.numpy().reshape(2, ctx.bodies_per_world, 3, 3)
        body0 = ctx.link_to_body[0]
        body1 = ctx.link_to_body[1]
        self.assertAlmostEqual(
            float(masses[1, body0]),
            0.75 * float(ctx.baseline_body_mass_world0[body0]),
            places=5,
        )
        self.assertAlmostEqual(
            float(coms[1, body0, 0]),
            float(ctx.baseline_body_com_world0[body0, 0]) - 0.015,
            places=5,
        )
        np.testing.assert_allclose(
            inertias[1, body1],
            log_cholesky_to_inertia_matrix(inertia_b),
            rtol=2e-5,
            atol=2e-7,
        )

    async def test_cuda_graph_capture_matches_warmup_rollout(self) -> None:
        """The second fixed-shape rollout captures/replays and preserves every signal."""
        bridge = self._make_bridge(_build_chain(2), ["/robot/joint0", "/robot/joint1"])
        trajectory = _sine_trajectory(2)
        bridge.set_trajectory(trajectory)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.tensor([[1.4, 0.7], [0.8, 1.8]], dtype=torch.float32)
        entries = _stiffness_entries(2)

        warmup = await bridge.run_rollout_async(theta, entries, commands, _NUM_STEPS)
        ctx = bridge._mujoco_cache[2]
        if not bridge._capture_supported(ctx):
            self.skipTest(f"MuJoCo model device {ctx.model.device} is not CUDA")
        self.assertEqual(len(ctx.capture_cache), 0)
        replay = await bridge.run_rollout_async(theta, entries, commands, _NUM_STEPS)

        self.assertFalse(ctx.capture_failed)
        self.assertEqual(len(ctx.capture_cache), 1)
        self.assertTrue(torch.allclose(replay.positions, warmup.positions, atol=1e-6, rtol=1e-6))
        self.assertTrue(torch.allclose(replay.velocities, warmup.velocities, atol=1e-6, rtol=1e-6))
        self.assertTrue(torch.allclose(replay.torques, warmup.torques, atol=1e-5, rtol=1e-6))

    async def test_cuda_graph_reuses_stable_buffers_for_new_inputs(self) -> None:
        """New parameter/command values update staging buffers without recapture."""
        paths = ["/robot/joint0", "/robot/joint1"]
        bridge = self._make_bridge(_build_chain(2), paths)
        plain_bridge = self._make_bridge(_build_chain(2), paths, cuda_graph_capture=False)
        trajectory = _sine_trajectory(2)
        entries = _stiffness_entries(2)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        for candidate in (bridge, plain_bridge):
            candidate.set_trajectory(trajectory)

        theta_a = torch.ones((2, 2), dtype=torch.float32)
        await bridge.run_rollout_async(theta_a, entries, commands, _NUM_STEPS)
        ctx = bridge._mujoco_cache[2]
        if not bridge._capture_supported(ctx):
            self.skipTest(f"MuJoCo model device {ctx.model.device} is not CUDA")
        await bridge.run_rollout_async(theta_a, entries, commands, _NUM_STEPS)
        self.assertEqual(len(ctx.capture_cache), 1)

        theta_b = torch.tensor([[2.0, 0.5], [0.6, 2.5]], dtype=torch.float32)
        commands_b = commands * 0.65
        replay = await bridge.run_rollout_async(theta_b, entries, commands_b, _NUM_STEPS)
        plain = await plain_bridge.run_rollout_async(theta_b, entries, commands_b, _NUM_STEPS)

        self.assertEqual(len(ctx.capture_cache), 1, "stable inputs triggered recapture")
        self.assertTrue(torch.allclose(replay.positions, plain.positions, atol=2e-5, rtol=2e-5))
        self.assertTrue(torch.allclose(replay.velocities, plain.velocities, atol=2e-4, rtol=2e-5))
        self.assertTrue(torch.allclose(replay.torques, plain.torques, atol=2e-3, rtol=2e-5))

    async def test_cuda_graph_opt_out_and_horizon_growth_invalidation(self) -> None:
        """Opt-out stays plain; staging-buffer growth discards stale graph addresses."""
        paths = ["/robot/joint0", "/robot/joint1"]
        trajectory = _sine_trajectory(2)
        entries = _stiffness_entries(2)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)

        disabled = self._make_bridge(_build_chain(2), paths, cuda_graph_capture=False)
        disabled.set_trajectory(trajectory)
        for _ in range(2):
            await disabled.run_rollout_async(torch.ones((1, 2)), entries, commands, 20)
        self.assertEqual(len(disabled._mujoco_cache[1].capture_cache), 0)

        bridge = self._make_bridge(_build_chain(2), paths)
        bridge.set_trajectory(trajectory)
        theta = torch.ones((1, 2), dtype=torch.float32)
        await bridge.run_rollout_async(theta, entries, commands, 20)
        ctx = bridge._mujoco_cache[1]
        if not bridge._capture_supported(ctx):
            self.skipTest(f"MuJoCo model device {ctx.model.device} is not CUDA")
        await bridge.run_rollout_async(theta, entries, commands, 20)
        self.assertEqual(len(ctx.capture_cache), 1)

        await bridge.run_rollout_async(theta, entries, commands, _NUM_STEPS)
        self.assertEqual(len(ctx.capture_cache), 0)
        self.assertEqual(len(ctx.capture_warm_keys), 1)

    async def _stiff_rollout(self, bridge: NewtonSysIdBridge, scales: list[float]):
        trajectory = _sine_trajectory(1)
        bridge.set_trajectory(trajectory)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.tensor([[scale] for scale in scales], dtype=torch.float32)
        # Run twice so the CUDA-graph replay path is exercised as well as the warmup.
        await bridge.run_rollout_async(theta, _stiffness_entries(1), commands, _NUM_STEPS)
        return await bridge.run_rollout_async(theta, _stiffness_entries(1), commands, _NUM_STEPS)

    async def test_divergent_stiff_drive_fails_with_substep_guidance(self) -> None:
        """A drive far past MuJoCo's explicit-stiffness bound fails closed instead of scoring 1e6."""
        bridge = self._make_bridge(_build_chain(1), ["/robot/joint0"])
        with self.assertRaisesRegex(
            SysIdEnvironmentBridgeError, r"diverged in 1/1 world.*mujoco_substeps to at least"
        ) as caught:
            await self._stiff_rollout(bridge, [_DIVERGENT_STIFFNESS_SCALE])
        self.assertIn("/robot/joint0", str(caught.exception))

    async def test_impractical_substep_count_points_at_the_asset(self) -> None:
        """A drive that would need hundreds of substeps is reported as an asset problem."""
        bridge = self._make_bridge(_build_chain(1), ["/robot/joint0"])
        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, r"so the asset itself needs correcting") as caught:
            await self._stiff_rollout(bridge, [1.0e7])
        self.assertNotIn("to at least", str(caught.exception))

    async def test_divergent_world_is_quarantined_from_stable_candidates(self) -> None:
        """Only the diverged world is invalidated, and the stable world matches a solo rollout."""
        bridge = self._make_bridge(_build_chain(1), ["/robot/joint0"])
        result = await self._stiff_rollout(bridge, [1.0, _DIVERGENT_STIFFNESS_SCALE])
        solo = await self._stiff_rollout(self._make_bridge(_build_chain(1), ["/robot/joint0"]), [1.0])

        self.assertTrue(torch.all(result.positions[1] == 1.0e6))
        self.assertTrue(torch.all(torch.isfinite(result.positions[0])))
        self.assertTrue(torch.allclose(result.positions[0], solo.positions[0], atol=1e-5))
        self.assertTrue(torch.allclose(result.velocities[0], solo.velocities[0], atol=1e-4))

    async def test_mujoco_substeps_stabilize_stiff_drive(self) -> None:
        """Substepping the same stiff drive keeps the rollout finite and bounded."""
        bridge = self._make_bridge(_build_chain(1), ["/robot/joint0"], mujoco_substeps=16)
        result = await self._stiff_rollout(bridge, [_DIVERGENT_STIFFNESS_SCALE])

        self.assertEqual(bridge.actuator_metadata["mujoco_substeps"], 16)
        self.assertTrue(torch.all(torch.isfinite(result.positions)))
        self.assertTrue(torch.all(torch.isfinite(result.velocities)))
        self.assertLess(float(result.positions.abs().max()), 1.0)

    async def test_mujoco_substeps_must_be_positive(self) -> None:
        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "mujoco_substeps must be at least 1"):
            self._make_bridge(_build_chain(1), ["/robot/joint0"], mujoco_substeps=0)


class NewtonMujocoBridgeFeedforwardTests(omni.kit.test.AsyncTestCase):
    """Feedforward modes on the genuine MuJoCo rollout (port of the diff-bridge tests)."""

    def setUp(self) -> None:
        available, reason = _mujoco_solver_available()
        if not available:
            self.skipTest(f"mujoco-warp unavailable: {reason}")

    def _make_bridge(self, builder, joint_paths: list[str], feedforward: str, snapshot=None) -> NewtonSysIdBridge:
        num_dof = len(joint_paths)
        return NewtonSysIdBridge(
            robot_prim_path="/robot",
            newton_config=NewtonSimulationRunSpec(solver=NEWTON_SOLVER_MUJOCO, feedforward=feedforward),
            joint_baselines=[snapshot or _joint_snapshot() for _ in range(num_dof)],
            joint_paths=joint_paths,
            num_joints=num_dof,
            robot_builder=builder,
        )

    async def test_gravity_feedforward_removes_static_sag(self) -> None:
        """The nominal-model feedforward holds a commanded pose against gravity.

        Mirrors the diff-bridge static-sag test: without compensation the chain
        sags below a constant target by roughly gravity_torque / ke; with it the
        steady-state error shrinks by an order of magnitude and the torque
        channel carries the feedforward (gravity-scale bias).
        """
        num_steps = 80
        times = np.arange(num_steps, dtype=np.float64) * _DT
        hold = np.full((num_steps, 2), 0.6, dtype=np.float64) * np.array([[1.0, -0.5]])
        errors: dict[str, float] = {}
        torque_means: dict[str, np.ndarray] = {}
        for mode in ("none", "gravity"):
            bridge = self._make_bridge(_build_chain(2), ["/robot/joint0", "/robot/joint1"], mode)
            bridge.set_trajectory(
                TrajectoryDataset(
                    times=times,
                    positions=hold.copy(),  # feedforward evaluated at the held pose
                    velocities=np.zeros_like(hold),
                    commands=hold.copy(),
                )
            )
            result = await bridge.run_rollout_async(
                torch.ones((1, 2), dtype=torch.float32),
                _stiffness_entries(2),
                torch.as_tensor(hold, dtype=torch.float32),
                num_steps,
            )
            self.assertTrue(torch.all(torch.isfinite(result.positions)))
            tail = result.positions[0, num_steps // 2 :, :]
            errors[mode] = float((tail - torch.as_tensor(hold[num_steps // 2 :], dtype=torch.float32)).abs().max())
            torque_means[mode] = result.torques[0].mean(dim=0).detach().cpu().numpy()
        self.assertLess(errors["gravity"], 0.15 * errors["none"])
        self.assertGreater(float(np.abs(torque_means["gravity"] - torque_means["none"]).max()), 0.1)

    async def test_inverse_dynamics_feedforward_tracks_dynamic_trajectory(self) -> None:
        """Velocity targets must act on the velocity ERROR for the feedforward to work.

        With the full inverse-dynamics feedforward supplying the motion torque,
        tracking error shrinks several-fold versus raw PD. If damping acted on
        absolute velocity (POSITION-mode actuators) it would cancel the
        feedforward and this assertion fails — the critical wiring this guards.
        """
        import newton
        import warp as wp

        num_steps = 60
        times = np.arange(num_steps, dtype=np.float64) * _DT
        omega_cmd = 2.0 * np.pi * 1.2
        commands = 0.5 * np.sin(omega_cmd * times)[:, None]
        velocities = 0.5 * omega_cmd * np.cos(omega_cmd * times)[:, None]
        errors: dict[str, float] = {}
        for mode in ("none", "inverse_dynamics"):
            builder = newton.ModelBuilder()
            link = builder.add_link(mass=1.7, com=(0.0, 0.0, -0.22), inertia=np.diag([0.03, 0.045, 0.05]).tolist())
            joint = builder.add_joint_revolute(
                parent=-1,
                child=link,
                axis=(0.0, 1.0, 0.0),
                parent_xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()),
                child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
                target_ke=5.0,
                target_kd=0.5,
                label="/robot/joint0",
            )
            builder.add_articulation([joint], label="pendulum1")
            # Zero friction/armature (as in the diff-bridge test): neither is part
            # of the rigid-body feedforward model, so they would leave a torque
            # residual for the weak PD and mask the ratio this test asserts.
            snapshot = SimpleNamespace(
                stiffness=5.0,
                damping=0.5,
                friction=0.0,
                dynamic_friction=0.0,
                armature=0.0,
                max_force=float("inf"),
            )
            bridge = self._make_bridge(builder, ["/robot/joint0"], mode, snapshot=snapshot)
            bridge.set_trajectory(
                TrajectoryDataset(
                    times=times, positions=commands.copy(), velocities=velocities.copy(), commands=commands.copy()
                )
            )
            result = await bridge.run_rollout_async(
                torch.ones((1, 1), dtype=torch.float32),
                _stiffness_entries(1),
                torch.as_tensor(commands, dtype=torch.float32),
                num_steps,
            )
            self.assertTrue(torch.all(torch.isfinite(result.positions)))
            tail = slice(num_steps // 3, None)
            errors[mode] = float(
                (result.positions[0, tail] - torch.as_tensor(commands, dtype=torch.float32)[tail]).abs().mean()
            )
        self.assertLess(errors["inverse_dynamics"], 0.25 * errors["none"])

    async def test_chunk_alternation_reuses_cached_feedforward_per_chunk(self) -> None:
        """Alternating training chunks must hit the per-trajectory feedforward cache.

        The optimizer re-sets the trajectory on every training-chunk switch,
        every iteration; the CPU Newton-Euler feedforward must be computed once
        per chunk (not per rollout), and each chunk's repeat rollout must
        reproduce its first one even though the other chunk's feedforward was
        loaded in between. Mirrors the diff-bridge chunk-alternation test.
        """
        bridge = self._make_bridge(_build_chain(2), ["/robot/joint0", "/robot/joint1"], "gravity")
        chunk_a = _hold_trajectory(0.5)
        chunk_b = _hold_trajectory(-0.3)
        theta = torch.ones((1, 2), dtype=torch.float32)
        calls, restore = _count_feedforward_computes()
        results: dict[str, list] = {"a": [], "b": []}
        try:
            for label, chunk in (("a", chunk_a), ("b", chunk_b), ("a", chunk_a), ("b", chunk_b)):
                bridge.set_trajectory(chunk)
                commands = torch.as_tensor(chunk.commands, dtype=torch.float32)
                result = await bridge.run_rollout_async(theta, _stiffness_entries(2), commands, _NUM_STEPS)
                results[label].append(result)
        finally:
            restore()
        self.assertEqual(calls["count"], 2, "expected one feedforward computation per chunk")
        for label in ("a", "b"):
            self.assertTrue(torch.allclose(results[label][0].positions, results[label][1].positions, atol=1e-5))
            self.assertTrue(torch.allclose(results[label][0].torques, results[label][1].torques, atol=1e-4))
        # The two chunks genuinely load different feedforwards/commands.
        self.assertFalse(torch.allclose(results["a"][0].positions, results["b"][0].positions, atol=1e-3))

    async def test_feedforward_recomputes_when_trajectory_mutated_in_place(self) -> None:
        """The identity-keyed cache must verify content: in-place edits invalidate it."""
        bridge = self._make_bridge(_build_chain(2), ["/robot/joint0", "/robot/joint1"], "gravity")
        chunk = _hold_trajectory(0.4)
        theta = torch.ones((1, 2), dtype=torch.float32)
        commands = torch.as_tensor(chunk.commands, dtype=torch.float32)
        calls, restore = _count_feedforward_computes()
        try:
            bridge.set_trajectory(chunk)
            await bridge.run_rollout_async(theta, _stiffness_entries(2), commands, _NUM_STEPS)
            self.assertEqual(calls["count"], 1)

            chunk.positions += 0.2  # same object identity, different content
            bridge.set_trajectory(chunk)
            await bridge.run_rollout_async(theta, _stiffness_entries(2), commands, _NUM_STEPS)
        finally:
            restore()
        self.assertEqual(calls["count"], 2, "mutated trajectory content must recompute the feedforward")

    async def test_feedforward_rejects_floating_base(self) -> None:
        """A FREE joint with a feedforward mode raises directly."""
        import newton
        import warp as wp

        builder = newton.ModelBuilder()
        base = builder.add_link(mass=2.0, com=(0.0, 0.0, 0.0), inertia=np.diag([0.1, 0.1, 0.1]).tolist())
        base_joint = builder.add_joint_free(child=base)
        arm = builder.add_link(mass=0.7, com=(0.0, 0.0, -0.1), inertia=np.diag([0.01, 0.015, 0.02]).tolist())
        rev = builder.add_joint_revolute(
            parent=base,
            child=arm,
            axis=(0.0, 1.0, 0.0),
            parent_xform=wp.transform((0.0, 0.0, 0.2), wp.quat_identity()),
            child_xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
            target_ke=4.0,
            target_kd=0.3,
            label="/robot/joint0",
        )
        builder.add_articulation([base_joint, rev], label="floating")
        bridge = self._make_bridge(builder, ["/robot/joint0"], "gravity")
        trajectory = _sine_trajectory(1)
        bridge.set_trajectory(trajectory)
        with self.assertRaises(SysIdEnvironmentBridgeError) as raised:
            await bridge.run_rollout_async(
                torch.ones((1, 1), dtype=torch.float32),
                _stiffness_entries(1),
                torch.as_tensor(trajectory.commands, dtype=torch.float32),
                _NUM_STEPS,
            )
        self.assertIn("fixed-base", str(raised.exception))


class NewtonMujocoBridgeGuardTests(omni.kit.test.AsyncTestCase):
    """Fail-closed guards that do not require a live Newton solver."""

    @staticmethod
    def _feedforward_bridge_and_context() -> tuple[NewtonSysIdBridge, SimpleNamespace]:
        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._feedforward_mode = "gravity"
        bridge._trajectory = _hold_trajectory(0.2)
        bridge._ff_cache = {}
        bridge._actuator_configs = [NewtonActuatorConfig(dof_index=index) for index in range(2)]
        bridge._modules = SimpleNamespace(
            newton=SimpleNamespace(JointType=SimpleNamespace(FREE=100, DISTANCE=101)),
        )
        ctx = SimpleNamespace(
            model=SimpleNamespace(joint_type=SimpleNamespace(numpy=lambda: np.asarray([0], dtype=np.int32))),
            joints_per_world=1,
            world_count=1,
            num_dof=2,
            coords_per_world=2,
            dofs_per_world=2,
            coord_map_np=np.asarray([0, 1], dtype=np.int64),
            dof_map_np=np.asarray([0, 1], dtype=np.int64),
            default_joint_q=np.zeros(2, dtype=np.float32),
            baseline_body_mass=np.ones(1, dtype=np.float32),
            baseline_body_com=np.zeros((1, 3), dtype=np.float32),
            baseline_body_inertia=np.eye(3, dtype=np.float32)[None, ...],
            explicit_columns=(),
            ff_active=False,
            ff_np=None,
            velocity_targets_np=None,
        )
        return bridge, ctx

    async def test_failed_feedforward_is_not_cached_and_is_retried(self) -> None:
        bridge, ctx = self._feedforward_bridge_and_context()
        commands = np.zeros((_NUM_STEPS, 2), dtype=np.float32)

        with patch(
            "isaacsim.robot_setup.sysid.newton_sysid_bridge.compute_feedforward",
            side_effect=RuntimeError("feedforward backend failed"),
        ) as compute:
            for _ in range(2):
                with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "Failed to compute configured"):
                    bridge._ensure_feedforward(ctx, _NUM_STEPS, commands)

        self.assertEqual(compute.call_count, 2)
        self.assertEqual(bridge._ff_cache, {})
        self.assertFalse(ctx.ff_active)

    async def test_nonfinite_feedforward_is_rejected_before_cache_activation(self) -> None:
        bridge, ctx = self._feedforward_bridge_and_context()
        commands = np.zeros((_NUM_STEPS, 2), dtype=np.float32)
        result = np.full((_NUM_STEPS, 2), np.nan, dtype=np.float64)

        with patch("isaacsim.robot_setup.sysid.newton_sysid_bridge.compute_feedforward", return_value=result):
            with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "non-finite"):
                bridge._ensure_feedforward(ctx, _NUM_STEPS, commands)

        self.assertEqual(bridge._ff_cache, {})
        self.assertFalse(ctx.ff_active)

    async def test_feedforward_rejects_implicit_neural_controller_dof(self) -> None:
        from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
            NEWTON_CONTROLLER_NEURAL_MLP,
        )

        bridge, ctx = self._feedforward_bridge_and_context()
        bridge._actuator_configs[0] = NewtonActuatorConfig(
            dof_index=0,
            controller_kind=NEWTON_CONTROLLER_NEURAL_MLP,
        )
        ctx.explicit_columns = ()

        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "neural"):
            bridge._ensure_feedforward(ctx, _NUM_STEPS, np.zeros((_NUM_STEPS, 2), dtype=np.float32))

    async def test_unusable_authored_neural_schema_fails_closed(self) -> None:
        class NeuralPrim:
            @staticmethod
            def GetAppliedSchemas():  # noqa: ANN205, N802
                return ["NewtonNeuralControlAPI"]

            @staticmethod
            def GetAttribute(_name):  # noqa: ANN205, N802
                return None

            @staticmethod
            def GetPath():  # noqa: ANN205, N802
                return "/robot/neural_actuator"

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._modules = SimpleNamespace(actuators=SimpleNamespace())
        bridge.newton_config = SimpleNamespace(controller="pd", effort_clamp="none")

        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "unusable NewtonNeuralControlAPI"):
            bridge._config_from_usd_prim(NeuralPrim(), NewtonActuatorConfig(dof_index=0))

    async def test_newton_16_neural_drive_is_classified(self) -> None:
        """Read Newton 1.6 drive metadata without consulting deprecated aliases."""

        class DriveNeuralMLP:
            pass

        class Parsed:
            drive_class = DriveNeuralMLP
            drive_kwargs = {"model_path": "models/actuator.onnx"}

            @property
            def controller_class(self):
                raise AssertionError("Deprecated controller metadata was accessed")

            @property
            def controller_kwargs(self):
                raise AssertionError("Deprecated controller metadata was accessed")

        class AssetPath:
            path = "models/actuator.onnx"
            resolvedPath = "/tmp/models/actuator.onnx"

        prim = SimpleNamespace(GetAttribute=lambda _name: SimpleNamespace(Get=lambda: AssetPath()))
        actuators = SimpleNamespace(parse_actuator_prim=lambda _prim: Parsed())

        classified = classify_newton_neural_prim(actuators, prim)

        self.assertEqual(classified["controller_kind"], NEWTON_CONTROLLER_NEURAL_MLP)
        self.assertEqual(classified["model_type"], "mlp")
        self.assertEqual(classified["controller_class_name"], "DriveNeuralMLP")
        self.assertEqual(classified["unsupported_schemas"], [])

    async def test_newton_16_neural_drive_class_is_resolved(self) -> None:
        """Resolve the Newton 1.6 drive class before any legacy alias."""

        class DriveNeuralMLP:
            pass

        class Actuators:
            def __getattr__(self, name):
                if name == "DriveNeuralMLP":
                    return DriveNeuralMLP
                raise AssertionError(f"Deprecated actuator alias was accessed: {name}")

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        config = NewtonActuatorConfig(
            dof_index=0,
            controller_kind=NEWTON_CONTROLLER_NEURAL_MLP,
            controller_class_name="DriveNeuralMLP",
        )

        resolved = bridge._resolve_neural_controller_class(Actuators(), config)

        self.assertIs(resolved, DriveNeuralMLP)

    async def test_nonfinite_physics_world_is_replaced_by_invalid_sentinel(self) -> None:
        positions = torch.zeros((2, 3, 1), dtype=torch.float32)
        velocities = torch.zeros_like(positions)
        torques = torch.zeros_like(positions)
        velocities[1, 1, 0] = float("nan")

        scrubbed = _replace_invalid_rollout_worlds(
            positions,
            velocities,
            torques,
            np.zeros(2, dtype=bool),
        )

        for values in scrubbed:
            self.assertTrue(torch.all(values[0] == 0.0))
            self.assertTrue(torch.all(values[1] == 1.0e6))


class NewtonMujocoBridgeFailureTests(omni.kit.test.AsyncTestCase):
    """Structural MuJoCo failures are reported without simulator substitution."""

    def setUp(self) -> None:
        if not _NEWTON_AVAILABLE:
            self.skipTest(_NEWTON_REASON)

    async def test_feedforward_rejects_neural_controller_dofs(self) -> None:
        """Feedforward mode with a neural-controller DOF must raise before physics runs.

        Neural controllers compute a self-contained torque signal; the extension's
        nominal-model Newton-Euler feedforward would stack additively on top via
        ``joint_control_feedforward`` and corrupt the identified dynamics.
        """
        from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
            NEWTON_CONTROLLER_NEURAL_MLP,
        )

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._feedforward_mode = "gravity"
        bridge._actuator_configs = [NewtonActuatorConfig(dof_index=0, controller_kind=NEWTON_CONTROLLER_NEURAL_MLP)]
        ctx = SimpleNamespace(explicit_columns=())
        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "neural"):
            bridge._ensure_feedforward(ctx, _NUM_STEPS, np.zeros((_NUM_STEPS, 1), dtype=np.float32))

    async def test_structural_failure_is_reported(self) -> None:
        # No stage, stage path, or robot builder means no MuJoCo model can be built.
        bridge = NewtonSysIdBridge(
            robot_prim_path="/robot",
            newton_config=NewtonSimulationRunSpec(solver=NEWTON_SOLVER_MUJOCO),
            joint_baselines=[_joint_snapshot() for _ in range(2)],
            num_joints=2,
        )
        trajectory = _sine_trajectory(2)
        bridge.set_trajectory(trajectory)
        commands = torch.as_tensor(trajectory.commands, dtype=torch.float32)
        theta = torch.ones((1, 2), dtype=torch.float32)
        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "ModelBuilder.add_usd"):
            await bridge.run_rollout_async(theta, _stiffness_entries(2), commands, _NUM_STEPS)
