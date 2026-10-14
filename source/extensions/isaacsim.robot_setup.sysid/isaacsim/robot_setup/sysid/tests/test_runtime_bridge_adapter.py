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

"""Focused coverage for Isaac Sim and Fabric runtime adapter contracts."""

from __future__ import annotations

import contextlib

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.backend_capabilities import (
    ISAAC_SIM_NEWTON_CAPABILITIES,
)
from isaacsim.robot_setup.sysid.env_bridge import SysIdEnvironmentBridgeError
from isaacsim.robot_setup.sysid.fabric_batch import (
    read_batched_joint_state,
    read_batched_link_pose,
)
from isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge import IsaacSimSysIdBridge
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)

# ruff: noqa: ANN001, ANN003, ANN202, D102


class _RecordingArticulation:
    def __init__(self) -> None:
        self.com_call: tuple[tuple[object, ...], dict[str, object]] | None = None
        self.friction_calls = 0
        self.gain_calls = 0

    def set_dof_friction_properties(self, **_kwargs) -> None:
        self.friction_calls += 1

    def set_dof_gains(self, **_kwargs) -> None:
        self.gain_calls += 1

    def set_link_coms(self, *args: object, **kwargs: object) -> None:
        self.com_call = (args, kwargs)


class _GainRecordingArticulation(_RecordingArticulation):
    def __init__(self) -> None:
        super().__init__()
        self.stiffness = np.asarray([[10.0]], dtype=np.float32)
        self.damping = np.asarray([[1.0]], dtype=np.float32)

    def set_dof_gains(self, **kwargs) -> None:
        self.gain_calls += 1
        self.stiffness = np.asarray(kwargs["stiffnesses"], dtype=np.float32)
        self.damping = np.asarray(kwargs["dampings"], dtype=np.float32)

    def get_dof_gains(self, **_kwargs) -> tuple[np.ndarray, np.ndarray]:
        return self.stiffness, self.damping


class _FullPropertyRecordingArticulation(_GainRecordingArticulation):
    def __init__(self) -> None:
        super().__init__()
        self.readbacks: set[str] = set()
        self.static_friction = np.asarray([[0.2]], dtype=np.float32)
        self.dynamic_friction = np.asarray([[0.1]], dtype=np.float32)
        self.armature = np.asarray([[0.0]], dtype=np.float32)
        self.masses = np.asarray([[2.0]], dtype=np.float32)
        self.com_positions = np.asarray([[[0.1, 0.2, 0.3]]], dtype=np.float32)
        self.com_orientations = np.asarray([[[1.0, 0.0, 0.0, 0.0]]], dtype=np.float32)
        self.inertias = np.asarray([[[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]]], dtype=np.float32)
        self.lower = np.asarray([[-1.0]], dtype=np.float32)
        self.upper = np.asarray([[1.0]], dtype=np.float32)

    def set_dof_friction_properties(self, **kwargs) -> None:
        self.static_friction = np.asarray(kwargs["static_frictions"], dtype=np.float32)
        self.dynamic_friction = np.asarray(kwargs["dynamic_frictions"], dtype=np.float32)

    def get_dof_friction_properties(self, **_kwargs) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self.readbacks.add("friction")
        return self.static_friction, self.dynamic_friction, np.zeros_like(self.dynamic_friction)

    def set_dof_armatures(self, **kwargs) -> None:
        self.armature = np.asarray(kwargs["armatures"], dtype=np.float32)

    def get_dof_armatures(self, **_kwargs) -> np.ndarray:
        self.readbacks.add("armature")
        return self.armature

    def set_link_masses(self, masses: np.ndarray, **_kwargs) -> None:
        self.masses = np.asarray(masses, dtype=np.float32)

    def get_link_masses(self, **_kwargs) -> np.ndarray:
        self.readbacks.add("mass")
        return self.masses

    def set_link_coms(self, positions: np.ndarray, orientations: np.ndarray, **_kwargs) -> None:
        self.com_positions = np.asarray(positions, dtype=np.float32)
        self.com_orientations = np.asarray(orientations, dtype=np.float32)

    def get_link_coms(self, **_kwargs) -> tuple[np.ndarray, np.ndarray]:
        self.readbacks.add("com")
        return self.com_positions, self.com_orientations

    def set_link_inertias(self, inertias: np.ndarray, **_kwargs) -> None:
        self.inertias = np.asarray(inertias, dtype=np.float32)

    def get_link_inertias(self, **_kwargs) -> np.ndarray:
        self.readbacks.add("inertia")
        return self.inertias

    def set_dof_limits(self, *, lower: np.ndarray, upper: np.ndarray, **_kwargs) -> None:
        self.lower = np.asarray(lower, dtype=np.float32)
        self.upper = np.asarray(upper, dtype=np.float32)

    def get_dof_limits(self, **_kwargs) -> tuple[np.ndarray, np.ndarray]:
        self.readbacks.add("limits")
        return self.lower, self.upper


class _FabricEffortFallback:
    def get_dof_positions(self, **_kwargs) -> np.ndarray:
        return np.asarray([[1.0]], dtype=np.float32)

    def get_dof_velocities(self, **_kwargs) -> np.ndarray:
        return np.asarray([[2.0]], dtype=np.float32)

    def get_dof_projected_joint_forces(self, **_kwargs) -> np.ndarray:
        raise NotImplementedError

    def get_dof_efforts(self, **_kwargs) -> np.ndarray:
        return np.asarray([[3.0]], dtype=np.float32)


class RuntimeBridgeAdapterTests(omni.kit.test.AsyncTestCase):
    """Lock runtime API conventions that otherwise fail silently."""

    async def test_explicit_actuators_ignore_non_trajectory_joints(self) -> None:
        """A gripper tail must not be treated as a trajectory actuator column."""
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge._selected_dof_paths = ["/robot/joint0", "/robot/joint1", "/robot/finger0", "/robot/finger1"]
        bridge._num_joints = 2

        self.assertEqual(
            bridge._active_trajectory_dof_paths(),
            ["/robot/joint0", "/robot/joint1"],
        )

    async def test_hosted_newton_rejects_friction_for_direct_bridge_callers(self) -> None:
        """Protect callers that construct a bridge without shared preflight."""
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge._backend_capabilities = ISAAC_SIM_NEWTON_CAPABILITIES
        bridge._pending_param_entries = []
        bridge._articulation = None

        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "joint_friction"):
            bridge.validate_parameter_entries([SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)])

    async def test_hosted_newton_rejects_placeholder_force_signals(self) -> None:
        """Never collect command-effort placeholders as torque/contact residuals."""
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge._backend_capabilities = ISAAC_SIM_NEWTON_CAPABILITIES

        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "contact_force, torque"):
            bridge.configure_rollout_signals(
                torque=True,
                end_effector_pose=False,
                contact_force=True,
            )

    async def test_explicit_actuators_reject_missing_trajectory_joint_paths(self) -> None:
        """Incomplete USD mappings should fail with a useful bridge error."""
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge._selected_dof_paths = ["/robot/joint0"]
        bridge._num_joints = 2

        with self.assertRaisesRegex(SysIdEnvironmentBridgeError, "trajectory has 2 DOFs"):
            bridge._active_trajectory_dof_paths()

    async def test_com_apply_updates_positions_without_overwriting_orientation(self) -> None:
        """COM optimization must preserve each link's authored inertia-frame orientation."""
        articulation = _RecordingArticulation()
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge._articulation = articulation
        bridge.use_parallel_clones = False
        bridge._cloned_for_count = 0
        bridge._logical_env_count = 3
        bridge._joint_dof_indices = [0]
        bridge._num_links = 1
        bridge._baseline_dynamic_friction = np.asarray([[0.1]], dtype=np.float32)
        bridge._baseline_static_friction = np.asarray([[0.2]], dtype=np.float32)
        bridge._baseline_stiffness = np.asarray([[10.0]], dtype=np.float32)
        bridge._baseline_damping = np.asarray([[1.0]], dtype=np.float32)
        bridge._baseline_armature = None
        bridge._baseline_link_masses = None
        bridge._baseline_link_com = np.asarray([[0.1, 0.2, 0.3]], dtype=np.float32)
        bridge._baseline_link_com_orientation = np.asarray(
            [[0.9238795, 0.0, 0.0, 0.3826834]],
            dtype=np.float32,
        )
        bridge._baseline_link_inertia = None
        bridge._baseline_link_inertia_lc = {}
        bridge._baseline_dof_lower = None
        bridge._baseline_dof_upper = None
        bridge._explicit_pd_compat = None
        bridge._articulation_backend_context = contextlib.nullcontext
        bridge._flush_physics_changes = lambda: None

        bridge.apply_parameter_vector(
            torch.tensor([[0.05]], dtype=torch.float32),
            [
                SysIdParameterEntry(
                    param_type=SysIdParameterType.LINK_COM_OFFSET_X,
                    dof_index=-1,
                    link_index=0,
                )
            ],
        )

        self.assertIsNotNone(articulation.com_call)
        args, kwargs = articulation.com_call
        self.assertEqual(len(args), 2)
        np.testing.assert_allclose(args[0], [[[0.15, 0.2, 0.3]]], rtol=0.0, atol=1e-7)
        np.testing.assert_allclose(
            args[1],
            [[[0.9238795, 0.0, 0.0, 0.3826834]]],
            rtol=0.0,
            atol=1e-7,
        )
        self.assertEqual(kwargs, {"indices": [0]})
        self.assertEqual(bridge._physics_env_count(), 1)
        self.assertEqual(articulation.friction_calls, 0)
        self.assertEqual(articulation.gain_calls, 0)

    async def test_fabric_effort_uses_supported_fallback(self) -> None:
        """An unsupported projected-force API may fall back to ordinary efforts."""
        _position, _velocity, effort = read_batched_joint_state(
            _FabricEffortFallback(),
            num_envs=1,
            dof_indices=[0],
            device=torch.device("cpu"),
        )
        self.assertIsNotNone(effort)
        self.assertEqual(float(effort[0, 0]), 3.0)

    async def test_accepted_physx_stiffness_is_applied_and_verified_live(self) -> None:
        articulation = _GainRecordingArticulation()
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge.physics_backend = "physx"
        bridge.actuator_runtime = "implicit_drive"
        bridge._device = torch.device("cpu")
        bridge._articulation = articulation
        bridge.use_parallel_clones = False
        bridge._cloned_for_count = 0
        bridge._logical_env_count = 1
        bridge._joint_dof_indices = [0]
        bridge._num_links = 1
        bridge._baseline_dynamic_friction = np.asarray([[0.1]], dtype=np.float32)
        bridge._baseline_static_friction = np.asarray([[0.2]], dtype=np.float32)
        bridge._baseline_stiffness = np.asarray([[10.0]], dtype=np.float32)
        bridge._baseline_damping = np.asarray([[1.0]], dtype=np.float32)
        bridge._baseline_armature = None
        bridge._baseline_link_masses = None
        bridge._baseline_link_com = None
        bridge._baseline_link_com_orientation = None
        bridge._baseline_link_inertia = None
        bridge._baseline_link_inertia_lc = {}
        bridge._baseline_dof_lower = None
        bridge._baseline_dof_upper = None
        bridge._explicit_pd_compat = None
        bridge._articulation_backend_context = contextlib.nullcontext
        bridge._flush_physics_changes = lambda: None
        entries = [SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0)]

        applied = bridge.apply_accepted_parameters_to_live_articulation([2.0], entries)

        self.assertTrue(applied)
        np.testing.assert_allclose(articulation.stiffness, [[20.0]])
        np.testing.assert_allclose(articulation.damping, [[1.0]])

    async def test_accepted_physx_non_gain_parameters_are_applied_and_verified_live(self) -> None:
        articulation = _FullPropertyRecordingArticulation()
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge.physics_backend = "physx"
        bridge.actuator_runtime = "implicit_drive"
        bridge._device = torch.device("cpu")
        bridge._articulation = articulation
        bridge.use_parallel_clones = False
        bridge._cloned_for_count = 0
        bridge._logical_env_count = 1
        bridge._joint_dof_indices = [0]
        bridge._num_links = 1
        bridge._baseline_dynamic_friction = np.asarray([[0.1]], dtype=np.float32)
        bridge._baseline_static_friction = np.asarray([[0.2]], dtype=np.float32)
        bridge._baseline_stiffness = np.asarray([[10.0]], dtype=np.float32)
        bridge._baseline_damping = np.asarray([[1.0]], dtype=np.float32)
        bridge._baseline_armature = np.asarray([[0.0]], dtype=np.float32)
        bridge._baseline_link_masses = np.asarray([[2.0]], dtype=np.float32)
        bridge._baseline_link_com = np.asarray([[0.1, 0.2, 0.3]], dtype=np.float32)
        bridge._baseline_link_com_orientation = np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        bridge._baseline_link_inertia = np.asarray(
            [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        bridge._baseline_link_inertia_lc = {0: np.zeros(6, dtype=np.float64)}
        bridge._baseline_dof_lower = np.asarray([-1.0], dtype=np.float32)
        bridge._baseline_dof_upper = np.asarray([1.0], dtype=np.float32)
        bridge._explicit_pd_compat = None
        bridge._articulation_backend_context = contextlib.nullcontext
        bridge._flush_physics_changes = lambda: None
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_ARMATURE, 0),
            SysIdParameterEntry(SysIdParameterType.LINK_MASS, -1, link_index=-1),
            SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_X, -1, link_index=0),
            SysIdParameterEntry(
                SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
                -1,
                link_index=0,
                component_index=0,
            ),
            SysIdParameterEntry(SysIdParameterType.JOINT_LIMIT_LOWER_SCALE, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_LIMIT_UPPER_SCALE, 0),
        ]

        applied = bridge.apply_accepted_parameters_to_live_articulation(
            [2.0, 0.3, 1.5, 0.05, 0.2, 0.9, 1.1],
            entries,
        )

        self.assertTrue(applied)
        self.assertEqual(articulation.readbacks, {"friction", "armature", "mass", "com", "inertia", "limits"})
        np.testing.assert_allclose(articulation.dynamic_friction, [[0.2]])
        np.testing.assert_allclose(articulation.static_friction, [[0.4]])
        np.testing.assert_allclose(articulation.armature, [[0.3]])
        np.testing.assert_allclose(articulation.masses, [[3.0]])
        np.testing.assert_allclose(articulation.com_positions, [[[0.15, 0.2, 0.3]]])
        np.testing.assert_allclose(articulation.lower, [[-0.9]])
        np.testing.assert_allclose(articulation.upper, [[1.1]])

    async def test_explicit_actuator_acceptance_is_not_reported_as_persistent_live_state(self) -> None:
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge.physics_backend = "physx"
        bridge.actuator_runtime = "newton_explicit"

        applied = bridge.apply_accepted_parameters_to_live_articulation([], [])

        self.assertFalse(applied)

    async def test_fabric_pose_propagates_unexpected_runtime_failure(self) -> None:
        """Real Fabric failures must not masquerade as an unavailable residual channel."""

        class BrokenArticulation:
            def get_world_poses(self, **_kwargs):
                raise RuntimeError("device read failed")

        with self.assertRaisesRegex(RuntimeError, "device read failed"):
            read_batched_link_pose(
                BrokenArticulation(),
                num_envs=1,
                link_index=0,
                device=torch.device("cpu"),
            )
