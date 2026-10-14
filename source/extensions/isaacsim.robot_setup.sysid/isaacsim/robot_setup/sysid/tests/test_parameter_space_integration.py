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

# ruff: noqa: ANN001, ANN003, ANN202, ANN204, ANN205, D101, D102, N802

"""Tests for parameter-space provenance and USD parameter IO integration."""

from __future__ import annotations

import math
import tempfile
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import ANY, Mock, patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.backend_capabilities import ISAAC_SIM_PHYSX_CAPABILITIES
from isaacsim.robot_setup.sysid.env_bridge import SysIdEnvironmentBridgeError
from isaacsim.robot_setup.sysid.fabric_batch import read_batched_link_pose
from isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge import IsaacSimSysIdBridge
from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
    NewtonActuatorConfig,
    NewtonSysIdBridge,
    _delayed_command_rows,
    _physical_inertia_is_valid,
)
from isaacsim.robot_setup.sysid.optimizer_base import (
    OptimizationIterationStatus,
    OptimizerConfig,
)
from isaacsim.robot_setup.sysid.optimizers.levenberg_marquardt import (
    _residual_is_torque_only,
)
from isaacsim.robot_setup.sysid.parameter_apply import (
    ParameterApplyState,
    decode_theta_to_apply_state,
)
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.provenance import (
    ProvenanceWriter,
    build_provenance_record,
    split_parameters_by_category,
)
from isaacsim.robot_setup.sysid.regressor import assemble_joint_regressor
from isaacsim.robot_setup.sysid.residual_config import ResidualWeightConfig
from isaacsim.robot_setup.sysid.run_controller import write_optimized_parameters_to_usd
from isaacsim.robot_setup.sysid.schema_validation import validate_sysid_run_spec_payload
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.tuning_assessment import (
    _is_holdout_metric,
    _training_normalized_cost,
)
from isaacsim.robot_setup.sysid.usd_parameter_io import (
    JointUsdSnapshot,
    LinkUsdSnapshot,
    read_joint_usd_snapshot,
    read_link_usd_snapshot,
    write_joint_usd_snapshot,
    write_link_usd_snapshot,
)
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics


class _FakeAttr:
    def __init__(self):
        self.value = None

    def IsValid(self):
        return True

    def Get(self):
        return self.value

    def Set(self, value):
        self.value = value

    def Clear(self):
        self.value = None


class _FakePrim:
    def __init__(self):
        self.attrs = {}

    def IsValid(self):
        return True

    def GetAttribute(self, name):
        return self.attrs.setdefault(name, _FakeAttr())

    def CreateAttribute(self, name, _type_name, custom=True):
        del custom
        return self.attrs.setdefault(name, _FakeAttr())

    def RemoveProperty(self, name):
        return self.attrs.pop(name, None) is not None


class _FakeStage:
    def __init__(self):
        self.prim = _FakePrim()

    def GetPrimAtPath(self, _path):
        return self.prim


class _Segment:
    def __init__(self, trajectory):
        self.trajectory = trajectory


class _FakeWpArray:
    def __init__(self, values, *, dtype=None, device=None):
        self.values = list(values)
        self.dtype = dtype
        self.device = device
        self.shape = (len(self.values),)


class _FakeWarp:
    float32 = "float32"
    int32 = "int32"
    uint32 = "uint32"

    @staticmethod
    def array(values, **kwargs):
        return _FakeWpArray(values, dtype=kwargs.get("dtype"), device=kwargs.get("device"))

    @staticmethod
    def zeros(count, **kwargs):
        return _FakeWpArray([0] * int(count), dtype=kwargs.get("dtype"), device=kwargs.get("device"))


def _assert_wp_array(value, dtype):
    assert isinstance(value, _FakeWpArray)
    assert value.shape == (1,)
    assert value.dtype == dtype


class _StrictDelay:
    def __init__(self, delay_steps, max_delay):
        _assert_wp_array(delay_steps, "int32")
        self.delay_steps = delay_steps
        self.max_delay = max_delay


class _StrictControllerPD:
    def __init__(self, kp, kd, const_effort=None):
        _assert_wp_array(kp, "float32")
        _assert_wp_array(kd, "float32")
        _assert_wp_array(const_effort, "float32")
        self.kp = kp
        self.kd = kd
        self.const_effort = const_effort


class _StrictControllerPID:
    def __init__(self, kp, kd, ki, integral_max=None, const_effort=None):
        _assert_wp_array(kp, "float32")
        _assert_wp_array(kd, "float32")
        _assert_wp_array(ki, "float32")
        _assert_wp_array(integral_max, "float32")
        _assert_wp_array(const_effort, "float32")
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral_max = integral_max
        self.const_effort = const_effort


class _StrictClampingMaxEffort:
    def __init__(self, max_effort):
        _assert_wp_array(max_effort, "float32")
        self.max_effort = max_effort


class _StrictClampingDCMotor:
    def __init__(self, max_motor_effort, saturation_effort, velocity_limit):
        _assert_wp_array(max_motor_effort, "float32")
        _assert_wp_array(saturation_effort, "float32")
        _assert_wp_array(velocity_limit, "float32")
        self.max_motor_effort = max_motor_effort
        self.saturation_effort = saturation_effort
        self.velocity_limit = velocity_limit


def _trajectory() -> TrajectoryDataset:
    values = np.zeros((2, 2), dtype=np.float64)
    return TrajectoryDataset(
        times=np.asarray([0.0, 1.0], dtype=np.float64),
        positions=values.copy(),
        velocities=values.copy(),
        commands=values.copy(),
    )


class ParameterSpaceIntegrationTests(omni.kit.test.AsyncTestCase):
    async def test_provenance_groups_and_sidecar_round_trip(self) -> None:
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 1),
        ]
        all_entries, sysid_entries, calibration_entries = split_parameters_by_category(
            entries,
            [1.25, 0.75],
            num_joints=2,
        )

        self.assertEqual(len(all_entries), 2)
        self.assertEqual([entry.param_type for entry in sysid_entries], ["joint_friction"])
        self.assertEqual([entry.param_type for entry in calibration_entries], ["joint_damping"])

        config = OptimizerConfig(
            trajectory=_trajectory(),
            param_entries=entries,
            theta_initial=torch.tensor([1.0, 1.0], dtype=torch.float32),
            theta_min=torch.tensor([0.0, 0.0], dtype=torch.float32),
            theta_max=torch.tensor([2.0, 2.0], dtype=torch.float32),
            epsilon=1e-4,
            damping_initial=1e-2,
            max_iterations=1,
        )
        status = OptimizationIterationStatus(
            iteration=3,
            cost=2.5,
            damping=0.0,
            accepted=True,
            theta=[1.25, 0.75],
        )
        record = build_provenance_record(
            config=config,
            final_status=status,
            optimizer_backend="lm",
            robot_prim_path="/World/Robot",
            validation_metrics=[],
            simulation_engine="isaac_sim",
            newton_config=None,
        )
        writer = ProvenanceWriter()
        stage = _FakeStage()
        writer.write_to_stage(stage, "/World/Robot", record)
        loaded = writer.read_from_stage(stage, "/World/Robot")

        self.assertEqual(loaded.optimizer_backend, "lm")
        self.assertEqual(len(loaded.parameters), 2)

        with tempfile.TemporaryDirectory() as tmp:
            out = writer.export_json_sidecar(str(Path(tmp) / "provenance.json"), record)
            self.assertTrue(Path(out).is_file())

    async def test_usd_joint_limits_preserve_zero_and_revolute_fallback_units(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        revolute = UsdPhysics.RevoluteJoint.Define(stage, "/World/revolute")
        revolute.GetLowerLimitAttr().Set(0.0)
        revolute.GetUpperLimitAttr().Set(0.0)
        snapshot = read_joint_usd_snapshot(stage, "/World/revolute")

        self.assertAlmostEqual(snapshot.lower_limit, 0.0)
        self.assertAlmostEqual(snapshot.upper_limit, 0.0)

        revolute.GetLowerLimitAttr().Clear()
        revolute.GetUpperLimitAttr().Clear()
        fallback = read_joint_usd_snapshot(stage, "/World/revolute")

        self.assertAlmostEqual(fallback.lower_limit, -math.pi)
        self.assertAlmostEqual(fallback.upper_limit, math.pi)

        prismatic = UsdPhysics.PrismaticJoint.Define(stage, "/World/prismatic")
        prismatic.GetLowerLimitAttr().Set(0.0)
        prismatic.GetUpperLimitAttr().Set(0.0)
        linear_snapshot = read_joint_usd_snapshot(stage, "/World/prismatic")

        self.assertAlmostEqual(linear_snapshot.lower_limit, 0.0)
        self.assertAlmostEqual(linear_snapshot.upper_limit, 0.0)

    async def test_segmented_training_assessment_uses_already_normalized_cost(self) -> None:
        trajectory = _trajectory()
        config = SimpleNamespace(
            trajectory=trajectory,
            training_segments=[_Segment(trajectory), _Segment(trajectory)],
            max_rollout_steps=2,
            residual_weight_config=None,
        )
        selected = SimpleNamespace(cost=0.25)

        self.assertAlmostEqual(_training_normalized_cost(config, selected), 0.25)

    async def test_legacy_training_assessment_still_normalizes_raw_residual_sum(self) -> None:
        trajectory = _trajectory()
        config = SimpleNamespace(
            trajectory=trajectory,
            training_segments=[],
            max_rollout_steps=2,
            residual_weight_config=None,
        )
        selected = SimpleNamespace(cost=8.0)

        self.assertAlmostEqual(_training_normalized_cost(config, selected), 1.0)

    async def test_link_full_inertia_writes_principal_axes(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/World/link", "Xform")
        UsdPhysics.MassAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)

        write_link_usd_snapshot(
            stage,
            LinkUsdSnapshot(
                path="/World/link",
                mass=2.0,
                com_offset=np.zeros(3, dtype=np.float64),
                diagonal_inertia=np.ones(3, dtype=np.float64),
                contact_offset=0.02,
                inertia_matrix=np.asarray(
                    [
                        [2.0, 0.25, 0.0],
                        [0.25, 3.0, 0.0],
                        [0.0, 0.0, 4.0],
                    ],
                    dtype=np.float64,
                ),
            ),
        )
        mass_api = UsdPhysics.MassAPI(prim)

        diag = mass_api.GetDiagonalInertiaAttr().Get()
        axes = mass_api.GetPrincipalAxesAttr().Get()
        self.assertIsNotNone(diag)
        self.assertIsNotNone(axes)
        self.assertNotAlmostEqual(float(diag[0]), 2.0)

    async def test_physical_inertia_rejects_spd_triangle_violation(self) -> None:
        self.assertTrue(_physical_inertia_is_valid(np.diag([2.0, 2.0, 2.0])))
        self.assertFalse(_physical_inertia_is_valid(np.diag([3.0, 1.0, 1.0])))
        self.assertFalse(_physical_inertia_is_valid(np.diag([np.nan, 1.0, 1.0])))

    async def test_joint_snapshot_captures_drive_max_force_for_newton(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        create_max_force = getattr(drive, "CreateMaxForceAttr", None)
        if callable(create_max_force):
            attr = create_max_force()
        else:
            attr = joint.GetPrim().CreateAttribute("drive:angular:physics:maxForce", Sdf.ValueTypeNames.Float)
        attr.Set(42.0)

        snapshot = read_joint_usd_snapshot(stage, "/World/joint")

        self.assertIsInstance(snapshot, JointUsdSnapshot)
        self.assertAlmostEqual(snapshot.max_force, 42.0)
        self.assertAlmostEqual(NewtonSysIdBridge._read_effort_limit(snapshot), 42.0)

    async def test_joint_snapshot_preserves_native_newton_properties(self) -> None:
        """Round-trip native Newton friction and armature while drive damping stays on the drive.

        Newton applies ``newton:damping`` as passive joint damping on top of the drive gain, so
        treating it as the drive gain would double the identified damping on reload.
        """
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        prim = joint.GetPrim()
        prim.CreateAttribute("newton:friction", Sdf.ValueTypeNames.Float, custom=False).Set(0.2)
        prim.CreateAttribute("newton:damping", Sdf.ValueTypeNames.Float, custom=False).Set(0.05)
        prim.CreateAttribute("newton:armature", Sdf.ValueTypeNames.Float, custom=False).Set(0.3)
        UsdPhysics.DriveAPI.Apply(prim, "angular").CreateDampingAttr().Set(0.001)

        snapshot = read_joint_usd_snapshot(stage, "/World/joint")

        self.assertTrue(snapshot.uses_newton_friction)
        self.assertFalse(snapshot.uses_newton_damping)
        self.assertTrue(snapshot.uses_newton_armature)
        self.assertAlmostEqual(snapshot.friction, 0.2)
        self.assertAlmostEqual(snapshot.damping, float(1.0 / np.deg2rad(1.0 / 0.001)))
        self.assertAlmostEqual(snapshot.armature, 0.3)

        write_joint_usd_snapshot(
            stage,
            replace(
                snapshot,
                friction=0.4,
                damping=snapshot.damping * 2.0,
                armature=0.6,
            ),
            physics_backend="newton",
            write_stiffness=False,
            write_limits=False,
        )

        self.assertAlmostEqual(float(prim.GetAttribute("newton:friction").Get()), 0.4)
        self.assertAlmostEqual(float(prim.GetAttribute("newton:armature").Get()), 0.6)
        self.assertAlmostEqual(float(prim.GetAttribute("drive:angular:physics:damping").Get()), 0.002)
        self.assertAlmostEqual(float(prim.GetAttribute("newton:damping").Get()), 0.05)
        self.assertFalse(prim.GetAttribute("physxJoint:jointFriction").HasAuthoredValueOpinion())
        self.assertFalse(prim.GetAttribute("physxJoint:armature").HasAuthoredValueOpinion())

    async def test_joint_writeback_uses_selected_physics_backend(self) -> None:
        """Choose the output schema from the solve backend rather than the source schema."""
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
        ]
        config = SimpleNamespace(param_entries=entries, trajectory=SimpleNamespace(num_joints=1))

        physx_stage = Usd.Stage.CreateInMemory()
        physx_joint = UsdPhysics.RevoluteJoint.Define(physx_stage, "/World/joint")
        physx_prim = physx_joint.GetPrim()
        physx_prim.ApplyAPI("NewtonJointAPI")
        physx_prim.GetAttribute("newton:friction").Set(0.2)
        UsdPhysics.DriveAPI.Apply(physx_prim, "angular").CreateDampingAttr().Set(0.001)
        physx_space = ParameterSpace.for_robot(1)
        physx_space.baseline.usd_joint_snapshots = [read_joint_usd_snapshot(physx_stage, "/World/joint")]

        self.assertTrue(
            write_optimized_parameters_to_usd(
                physx_stage,
                physx_space,
                config,
                [2.0, 2.0],
                1,
                physics_backend="physx",
            )
        )
        self.assertAlmostEqual(float(physx_prim.GetAttribute("physxJoint:jointFriction").Get()), 0.4)
        self.assertAlmostEqual(float(physx_prim.GetAttribute("drive:angular:physics:damping").Get()), 0.002)
        self.assertAlmostEqual(float(physx_prim.GetAttribute("newton:friction").Get()), 0.2)

        newton_stage = Usd.Stage.CreateInMemory()
        newton_joint = UsdPhysics.RevoluteJoint.Define(newton_stage, "/World/joint")
        newton_prim = newton_joint.GetPrim()
        PhysxSchema.PhysxJointAPI.Apply(newton_prim).CreateJointFrictionAttr().Set(0.2)
        UsdPhysics.DriveAPI.Apply(newton_prim, "angular").CreateDampingAttr().Set(0.001)
        newton_space = ParameterSpace.for_robot(1)
        newton_space.baseline.usd_joint_snapshots = [read_joint_usd_snapshot(newton_stage, "/World/joint")]

        self.assertTrue(
            write_optimized_parameters_to_usd(
                newton_stage,
                newton_space,
                config,
                [2.0, 2.0],
                1,
                physics_backend="newton",
            )
        )
        self.assertAlmostEqual(float(newton_prim.GetAttribute("newton:friction").Get()), 0.4)
        self.assertAlmostEqual(float(newton_prim.GetAttribute("physxJoint:jointFriction").Get()), 0.2)
        self.assertAlmostEqual(float(newton_prim.GetAttribute("drive:angular:physics:damping").Get()), 0.002)
        self.assertFalse(newton_prim.GetAttribute("newton:damping").HasAuthoredValueOpinion())

    async def test_joint_snapshot_reads_back_selected_backend_properties(self) -> None:
        """Read back Newton writeback values from a joint that also authors PhysX values."""
        stage = Usd.Stage.CreateInMemory()
        prim = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint").GetPrim()
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
        physx_joint.CreateJointFrictionAttr().Set(1.0)
        physx_joint.CreateArmatureAttr().Set(0.05)
        UsdPhysics.DriveAPI.Apply(prim, "angular").CreateDampingAttr().Set(0.001)
        prim.CreateAttribute(
            "physxJointAxis:angular:dynamicFrictionEffort", Sdf.ValueTypeNames.Float, custom=False
        ).Set(0.7)
        physx_snapshot = read_joint_usd_snapshot(stage, "/World/joint")
        self.assertAlmostEqual(physx_snapshot.dynamic_friction, 0.7)

        write_joint_usd_snapshot(
            stage,
            replace(physx_snapshot, friction=0.4, damping=physx_snapshot.damping * 2.0, armature=0.6),
            physics_backend="newton",
            write_stiffness=False,
            write_limits=False,
        )

        newton_snapshot = read_joint_usd_snapshot(stage, "/World/joint", physics_backend="newton")
        self.assertTrue(newton_snapshot.uses_newton_friction)
        self.assertFalse(newton_snapshot.uses_newton_damping)
        self.assertTrue(newton_snapshot.uses_newton_armature)
        self.assertAlmostEqual(newton_snapshot.friction, 0.4)
        self.assertAlmostEqual(newton_snapshot.dynamic_friction, 0.0)
        self.assertAlmostEqual(newton_snapshot.damping, physx_snapshot.damping * 2.0)
        self.assertAlmostEqual(newton_snapshot.armature, 0.6)
        self.assertFalse(prim.GetAttribute("newton:damping").HasAuthoredValueOpinion())

        for backend in ("physx", "", "auto"):
            snapshot = read_joint_usd_snapshot(stage, "/World/joint", physics_backend=backend)
            self.assertFalse(snapshot.uses_newton_friction)
            self.assertAlmostEqual(snapshot.friction, 1.0)
            self.assertAlmostEqual(snapshot.dynamic_friction, 0.7)
            self.assertAlmostEqual(snapshot.damping, physx_snapshot.damping * 2.0)
            self.assertAlmostEqual(snapshot.armature, 0.05)

        space = ParameterSpace.for_robot(1)
        space.read_usd_baselines(stage, [], ["/World/joint"], physics_backend="newton")
        self.assertAlmostEqual(float(space.baseline.joint_friction[0]), 0.4)
        self.assertAlmostEqual(float(space.baseline.joint_dynamic_friction[0]), 0.0)
        self.assertAlmostEqual(float(space.baseline.joint_damping[0]), physx_snapshot.damping * 2.0)
        self.assertAlmostEqual(float(space.baseline.joint_armature[0]), 0.6)

        # A calibrated zero must not fall back to the stale PhysX per-axis friction.
        prim.GetAttribute("newton:friction").Set(0.0)
        zero_snapshot = read_joint_usd_snapshot(stage, "/World/joint", physics_backend="newton")
        self.assertAlmostEqual(zero_snapshot.friction or zero_snapshot.dynamic_friction, 0.0)

        newton_only = UsdPhysics.RevoluteJoint.Define(stage, "/World/newton_only").GetPrim()
        newton_only.CreateAttribute("newton:friction", Sdf.ValueTypeNames.Float, custom=False).Set(0.3)
        unspecified = read_joint_usd_snapshot(stage, "/World/newton_only")
        self.assertTrue(unspecified.uses_newton_friction)
        self.assertAlmostEqual(unspecified.friction, 0.3)
        physx_only = read_joint_usd_snapshot(stage, "/World/newton_only", physics_backend="physx")
        self.assertFalse(physx_only.uses_newton_friction)
        self.assertAlmostEqual(physx_only.friction, 0.0)

        with self.assertRaises(ValueError):
            read_joint_usd_snapshot(stage, "/World/joint", physics_backend="bullet")

    async def test_build_parameter_space_reads_baselines_for_resolved_backend(self) -> None:
        """Read USD baselines with the backend resolved from the run specification."""
        from isaacsim.robot_setup.sysid import run_controller

        trajectory = SimpleNamespace(num_joints=1)
        cases = (
            ("newton", "auto", False, "newton"),
            ("isaac_sim", "auto", False, "physx"),
            ("isaac_sim", "physx", False, "physx"),
            ("isaac_sim", "newton", False, "newton"),
            ("newton", "auto", True, "newton"),
            ("isaac_sim", "auto", True, "physx"),
        )
        for engine, requested_backend, advanced, expected_backend in cases:
            flags = SimpleNamespace(
                include_basic=True,
                include_com_offsets=False,
                include_inertia_log_cholesky=False,
                include_joint_limit_scales=False,
                include_per_link_mass=advanced,
                include_command_delay=False,
            )
            simulation = SimpleNamespace(
                engine=engine, physics_backend=requested_backend, robot_prim_path="/World/robot"
            )
            spec = SimpleNamespace(simulation=simulation, parameters=SimpleNamespace(space=flags))
            with (
                patch.object(
                    run_controller,
                    "collect_robot_link_and_joint_paths",
                    return_value=(["/World/robot/link"], ["/World/robot/joint"]),
                ),
                patch.object(run_controller, "order_joint_paths_for_trajectory", side_effect=lambda paths, _: paths),
                patch.object(ParameterSpace, "read_usd_baselines") as read_baselines,
            ):
                run_controller.build_parameter_space(spec, Mock(), trajectory)
            read_baselines.assert_called_once_with(
                ANY,
                ["/World/robot/link"],
                ["/World/robot/joint"],
                physics_backend=expected_backend,
            )

    async def test_joint_writeback_preserves_original_dof_index_when_snapshot_missing(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint0")
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint2")
        space = ParameterSpace.for_robot(3)
        space.baseline.usd_joint_snapshots = [
            JointUsdSnapshot(
                dof_index=0,
                path="/World/joint0",
                lower_limit=-1.0,
                upper_limit=1.0,
                is_angular=True,
                friction=1.0,
                static_friction=1.1,
                dynamic_friction=1.0,
                viscous_friction=0.0,
                stiffness=1.0,
                damping=1.0,
                armature=0.0,
            ),
            None,
            JointUsdSnapshot(
                dof_index=2,
                path="/World/joint2",
                lower_limit=-1.0,
                upper_limit=1.0,
                is_angular=True,
                friction=10.0,
                static_friction=11.0,
                dynamic_friction=10.0,
                viscous_friction=0.0,
                stiffness=1.0,
                damping=1.0,
                armature=0.0,
            ),
        ]
        apply_state = ParameterApplyState(
            friction_scale=np.asarray([1.0, 2.0, 3.0], dtype=np.float32),
            stiffness_scale=np.ones(3, dtype=np.float32),
            damping_scale=np.ones(3, dtype=np.float32),
            joint_integral_gain=None,
            joint_armature=None,
            actuator_command_delay_seconds=None,
            link_mass_scale=np.ones(1, dtype=np.float32),
            link_com_delta=np.zeros((1, 3), dtype=np.float32),
        )

        space.write_usd_from_baselines(stage, apply_state)
        written = read_joint_usd_snapshot(stage, "/World/joint2")

        self.assertAlmostEqual(written.friction, 30.0)
        self.assertAlmostEqual(written.dynamic_friction, 30.0)

    async def test_selective_joint_writeback_does_not_author_link_properties(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        link_prim = stage.DefinePrim("/World/link", "Xform")
        mass_api = UsdPhysics.MassAPI.Apply(link_prim)
        mass_api.GetMassAttr().Set(2.0)
        mass_api.GetCenterOfMassAttr().Set((0.1, 0.2, 0.3))
        mass_api.GetDiagonalInertiaAttr().Set((0.02, 0.03, 0.04))
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")

        space = ParameterSpace.for_robot(1)
        space.baseline.usd_link_snapshots = [read_link_usd_snapshot(stage, "/World/link")]
        space.baseline.usd_joint_snapshots = [
            JointUsdSnapshot(
                dof_index=0,
                path="/World/joint",
                lower_limit=-1.0,
                upper_limit=1.0,
                is_angular=True,
                friction=1.0,
                static_friction=1.1,
                dynamic_friction=1.0,
                viscous_friction=0.0,
                stiffness=1.0,
                damping=1.0,
                armature=0.0,
            )
        ]
        apply_state = ParameterApplyState(
            friction_scale=np.asarray([2.0], dtype=np.float32),
            stiffness_scale=np.ones(1, dtype=np.float32),
            damping_scale=np.ones(1, dtype=np.float32),
            joint_integral_gain=None,
            joint_armature=None,
            actuator_command_delay_seconds=None,
            link_mass_scale=np.asarray([9.0], dtype=np.float32),
            link_com_delta=np.asarray([[4.0, 5.0, 6.0]], dtype=np.float32),
            link_inertia_flat={0: np.zeros(6, dtype=np.float32)},
        )

        space.write_usd_from_baselines(
            stage,
            apply_state,
            param_entries=[SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)],
        )

        link = read_link_usd_snapshot(stage, "/World/link")
        joint = read_joint_usd_snapshot(stage, "/World/joint")
        self.assertAlmostEqual(link.mass, 2.0)
        np.testing.assert_allclose(link.com_offset, [0.1, 0.2, 0.3])
        np.testing.assert_allclose(link.diagonal_inertia, [0.02, 0.03, 0.04])
        self.assertAlmostEqual(joint.friction, 2.0)

    async def test_newton_actuator_construction_uses_warp_arrays(self) -> None:
        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._modules = SimpleNamespace(
            warp=_FakeWarp(),
            actuators=SimpleNamespace(
                Delay=_StrictDelay,
                ControllerPD=_StrictControllerPD,
                ControllerPID=_StrictControllerPID,
                ClampingMaxEffort=_StrictClampingMaxEffort,
                ClampingDCMotor=_StrictClampingDCMotor,
            ),
        )
        bridge.newton_config = SimpleNamespace(device="cpu")
        bridge._physics_dt = 0.1
        bridge._trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 0.1, 0.2, 0.3], dtype=np.float64),
            positions=np.zeros((4, 2), dtype=np.float64),
            velocities=np.zeros((4, 2), dtype=np.float64),
            commands=np.zeros((4, 2), dtype=np.float64),
        )
        bridge._actuator_configs = [
            NewtonActuatorConfig(
                dof_index=0,
                controller_kind="pd",
                clamp_kind="max_effort",
                stiffness=10.0,
                damping=2.0,
                const_effort=0.5,
                effort_limit=20.0,
            ),
            NewtonActuatorConfig(
                dof_index=1,
                controller_kind="pid",
                clamp_kind="dc_motor",
                stiffness=11.0,
                damping=3.0,
                integral_gain=0.25,
                integral_max=4.0,
                max_motor_effort=30.0,
                saturation_effort=25.0,
                velocity_limit=12.0,
            ),
        ]

        bridge._build_actuators()

        self.assertEqual(len(bridge._actuator_specs), 2)
        self.assertEqual(bridge._actuator_specs[0].delay.max_delay, 3)
        self.assertEqual(bridge._actuator_specs[1].delay.max_delay, 3)
        self.assertIsInstance(bridge._actuator_specs[0].controller, _StrictControllerPD)
        self.assertIsInstance(bridge._actuator_specs[1].controller, _StrictControllerPID)
        self.assertIsInstance(bridge._actuator_specs[0].clamping, _StrictClampingMaxEffort)
        self.assertIsInstance(bridge._actuator_specs[1].clamping, _StrictClampingDCMotor)

    async def test_newton_command_delay_uses_trajectory_row_dt(self) -> None:
        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._physics_dt = 1.0 / 60.0
        bridge._trajectory = TrajectoryDataset(
            times=np.asarray([idx * 0.01 for idx in range(10)], dtype=np.float64),
            positions=np.zeros((10, 1), dtype=np.float64),
            velocities=np.zeros((10, 1), dtype=np.float64),
            commands=np.zeros((10, 1), dtype=np.float64),
        )
        commands = torch.arange(10, dtype=torch.float32).reshape(10, 1)

        delayed = _delayed_command_rows(
            commands,
            np.asarray([[0.05]], dtype=np.float32),
            bridge._rollout_dt(10),
        )

        self.assertEqual(float(delayed[0, 4, 0]), 0.0)
        self.assertEqual(float(delayed[0, 5, 0]), 0.0)
        self.assertEqual(float(delayed[0, 6, 0]), 1.0)

    async def test_isaac_bridge_deferred_validation_fails_after_articulation_resolution(self) -> None:
        bridge = IsaacSimSysIdBridge.__new__(IsaacSimSysIdBridge)
        bridge._backend_capabilities = ISAAC_SIM_PHYSX_CAPABILITIES
        bridge._articulation = None
        bridge._pending_param_entries = []
        entries = [SysIdParameterEntry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, -1, link_index=0)]

        bridge.validate_parameter_entries(entries)

        self.assertEqual(bridge._pending_param_entries, entries)

        bridge._articulation = SimpleNamespace()
        with self.assertRaises(SysIdEnvironmentBridgeError):
            bridge._validate_pending_parameter_entries()

    async def test_newton_command_delay_without_actuator_target_fails_writeback(self) -> None:
        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._actuator_configs = [SimpleNamespace(stiffness=1.0, damping=1.0)]

        written = bridge.write_parameters_to_usd(
            None,
            [SysIdParameterEntry(SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS, 0)],
            torch.tensor([0.02], dtype=torch.float32),
        )

        self.assertFalse(written)

    async def test_newton_actuator_delay_writeback_is_quantized_on_target_prim(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        actuator = stage.DefinePrim("/World/actuator", "Xform")
        actuator.CreateRelationship("newton:targets").AddTarget(Sdf.Path("/World/joint"))

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._physics_dt = 0.01
        bridge._actuator_promotions = []
        bridge._actuator_configs = [SimpleNamespace(stiffness=1.0, damping=1.0, source="usd")]

        written = bridge.write_parameters_to_usd(
            stage,
            [SysIdParameterEntry(SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS, 0)],
            torch.tensor([0.026], dtype=torch.float32),
        )

        self.assertTrue(written)
        self.assertIn("NewtonActuatorDelayAPI", set(actuator.GetAppliedSchemas()))
        self.assertEqual(actuator.GetAttribute("newton:delaySteps").Get(), 3)

    async def test_newton_promote_selected_creates_pid_delay_writeback_target(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Robot", "Xform")
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint")

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge.stage = stage
        bridge.robot_prim_path = "/World/Robot"
        bridge.explicit_actuator_policy = "promote_selected"
        bridge.newton_config = SimpleNamespace(
            actuator_source="drive_defaults",
            controller="pd",
            effort_clamp="max_effort",
        )
        bridge._joint_paths = ["/World/Robot/joint"]
        bridge._joint_baselines = []
        bridge._num_joints = 1
        bridge._physics_dt = 0.01
        bridge._actuator_configs = []
        bridge._actuator_specs = []
        bridge._actuator_promotions = []
        bridge._promoted_target_paths = set()
        bridge._mujoco_cache = {}
        bridge._build_actuators = lambda: None

        bridge._ensure_stateful_actuator_targets(
            [
                SysIdParameterEntry(SysIdParameterType.JOINT_INTEGRAL_GAIN, 0),
                SysIdParameterEntry(SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS, 0),
            ]
        )

        self.assertEqual(len(bridge._actuator_promotions), 1)
        actuator = stage.GetPrimAtPath(bridge._actuator_promotions[0].prim_path)
        self.assertIn("NewtonPIDControlAPI", set(actuator.GetAppliedSchemas()))
        self.assertIn("NewtonActuatorDelayAPI", set(actuator.GetAppliedSchemas()))
        self.assertEqual(bridge._actuator_configs[0].source, "usd")
        self.assertEqual(bridge._actuator_configs[0].controller_kind, "pid")
        actuator_path = bridge._actuator_promotions[0].prim_path

        written = bridge.write_parameters_to_usd(
            stage,
            [
                SysIdParameterEntry(SysIdParameterType.JOINT_INTEGRAL_GAIN, 0),
                SysIdParameterEntry(SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS, 0),
            ],
            torch.tensor([1.5, 0.024], dtype=torch.float32),
        )

        self.assertTrue(written)
        self.assertIsNone(stage.GetSessionLayer().GetPrimAtPath(actuator_path))
        self.assertIsNotNone(stage.GetRootLayer().GetPrimAtPath(actuator_path))
        self.assertAlmostEqual(stage.GetPrimAtPath(actuator_path).GetAttribute("newton:ki").Get(), 1.5)
        self.assertEqual(stage.GetPrimAtPath(actuator_path).GetAttribute("newton:delaySteps").Get(), 2)

    async def test_newton_mixed_mode_honors_authored_actuator_with_drive_fallback(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        actuator = stage.DefinePrim("/World/actuator", "Xform")
        actuator.AddAppliedSchema("NewtonPDControlAPI")
        actuator.CreateRelationship("newton:targets").AddTarget(Sdf.Path("/World/joint"))
        actuator.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(12.0)
        actuator.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(3.0)

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge.stage = stage
        bridge.actuator_runtime = "mixed"
        bridge.newton_config = SimpleNamespace(
            actuator_source="drive_defaults",
            controller="pd",
            effort_clamp="max_effort",
        )
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._joint_baselines = []
        bridge._promoted_target_paths = set()

        bridge._build_actuator_configs()

        self.assertEqual(bridge._actuator_configs[0].source, "usd")
        self.assertEqual(bridge._actuator_configs[0].stiffness, 12.0)
        self.assertEqual(bridge._actuator_configs[0].damping, 3.0)

    async def test_newton_actuator_gain_writeback_targets_matching_prim(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        actuator = stage.DefinePrim("/World/actuator", "Xform")
        actuator.CreateRelationship("newton:targets").AddTarget(Sdf.Path("/World/joint"))

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._actuator_configs = [SimpleNamespace(stiffness=10.0, damping=4.0, source="usd")]
        bridge.newton_config = SimpleNamespace(actuator_source="usd")
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_INTEGRAL_GAIN, 0),
        ]

        written = bridge.write_parameters_to_usd(
            stage,
            entries,
            torch.tensor([1.5, 0.25, 2.0], dtype=torch.float32),
        )

        self.assertTrue(written)
        self.assertAlmostEqual(actuator.GetAttribute("newton:kp").Get(), 15.0)
        self.assertAlmostEqual(actuator.GetAttribute("newton:kd").Get(), 1.0)
        self.assertAlmostEqual(actuator.GetAttribute("newton:ki").Get(), 2.0)

    async def test_newton_drive_default_gains_do_not_require_usd_actuator_target(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._actuator_configs = [SimpleNamespace(stiffness=10.0, damping=4.0, source="drive_defaults")]
        bridge.newton_config = SimpleNamespace(actuator_source="usd")

        written = bridge.write_parameters_to_usd(
            stage,
            [SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0)],
            torch.tensor([1.5], dtype=torch.float32),
        )

        self.assertTrue(written)

    async def test_newton_integral_gain_writeback_fails_without_target(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")

        bridge = NewtonSysIdBridge.__new__(NewtonSysIdBridge)
        bridge._num_joints = 1
        bridge._joint_paths = ["/World/joint"]
        bridge._actuator_configs = [SimpleNamespace(stiffness=1.0, damping=1.0)]

        written = bridge.write_parameters_to_usd(
            stage,
            [SysIdParameterEntry(SysIdParameterType.JOINT_INTEGRAL_GAIN, 0)],
            torch.tensor([2.0], dtype=torch.float32),
        )

        self.assertFalse(written)

    async def test_analytical_jacobian_stiffness_column_uses_command_reference(self) -> None:
        positions = np.array([[0.1], [0.2], [0.3]], dtype=np.float64)
        velocities = np.zeros((3, 1), dtype=np.float64)
        commands = np.full((3, 1), 0.5, dtype=np.float64)
        entries = [SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0)]

        with_command = assemble_joint_regressor(velocities, entries, positions=positions, commands=commands)
        command_equals_position = assemble_joint_regressor(velocities, entries, positions=positions, commands=positions)

        # d(tau)/d(stiffness) = command - position; nonzero with a real command,
        # but identically zero in the old commands==positions formulation.
        self.assertTrue(np.any(with_command[:, 0] != 0.0))
        self.assertTrue(np.allclose(command_equals_position[:, 0], 0.0))
        self.assertAlmostEqual(float(with_command[0, 0]), -(0.1 - 0.5))

    async def test_analytical_jacobian_gated_to_torque_only_residual(self) -> None:
        position_velocity = SimpleNamespace(residual_weight_config=ResidualWeightConfig())
        torque_only = SimpleNamespace(
            residual_weight_config=ResidualWeightConfig(position_weight=0.0, velocity_weight=0.0, torque_weight=1.0)
        )
        torque_plus_position = SimpleNamespace(
            residual_weight_config=ResidualWeightConfig(position_weight=1.0, torque_weight=1.0)
        )
        no_config = SimpleNamespace(residual_weight_config=None)

        self.assertTrue(_residual_is_torque_only(torque_only))
        self.assertFalse(_residual_is_torque_only(position_velocity))
        self.assertFalse(_residual_is_torque_only(torque_plus_position))
        self.assertFalse(_residual_is_torque_only(no_config))

    async def test_non_holdout_validation_metric_excluded_from_gap(self) -> None:
        holdout_metric = SimpleNamespace(normalized_cost=2.0, extra={})
        fallback_metric = SimpleNamespace(normalized_cost=1.0, extra={"holdout": False})

        self.assertTrue(_is_holdout_metric(holdout_metric))
        self.assertFalse(_is_holdout_metric(fallback_metric))

    async def test_decode_only_applies_inertia_to_optimized_links(self) -> None:
        baseline = {
            0: np.zeros(6, dtype=np.float64),
            1: np.zeros(6, dtype=np.float64),
        }

        no_inertia = decode_theta_to_apply_state(
            num_dof=2,
            num_links=2,
            param_entries=[SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)],
            theta_row=torch.tensor([1.0], dtype=torch.float32),
            baseline_link_inertia_lc=baseline,
        )
        self.assertEqual(no_inertia.link_inertia_flat, {})

        with_inertia = decode_theta_to_apply_state(
            num_dof=2,
            num_links=2,
            param_entries=[SysIdParameterEntry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, -1, link_index=1)],
            theta_row=torch.tensor([0.0], dtype=torch.float32),
            baseline_link_inertia_lc=baseline,
        )
        self.assertEqual(set(with_inertia.link_inertia_flat.keys()), {1})

    async def test_writeback_reports_failure_when_parameter_space_baseline_is_missing(self) -> None:
        """Do not report bridge success for a parameter the bridge did not claim."""
        space = ParameterSpace.for_robot(1)
        config = SimpleNamespace(
            param_entries=[SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)],
            trajectory=SimpleNamespace(num_joints=1),
        )
        bridge = SimpleNamespace(write_parameters_to_usd=lambda *args, **kwargs: True)

        result = write_optimized_parameters_to_usd(None, space, config, [1.0], 1, bridge)

        self.assertFalse(result)

    async def test_link_pose_fallback_returns_none_without_link_index_support(self) -> None:
        class _Articulation:
            def get_world_poses(self, indices=None, link_indices=None):
                if link_indices is not None:
                    raise TypeError("link_indices not supported by this view")
                count = len(list(indices))
                return np.zeros((count, 3), dtype=np.float32), np.zeros((count, 4), dtype=np.float32)

        # link_index 1 < num_envs 2: the old fallback returned environment 1's root
        # pose mislabeled as the link pose; the fix reports unavailable instead.
        result = read_batched_link_pose(_Articulation(), num_envs=2, link_index=1, device=torch.device("cpu"))
        self.assertIsNone(result)

    async def test_link_pose_converts_runtime_wxyz_to_public_xyzw(self) -> None:
        """Expose end-effector quaternions in the telemetry mapping convention."""

        class _Articulation:
            def get_world_poses(self, indices=None, link_indices=None):
                count = len(list(indices))
                positions = np.zeros((count, 3), dtype=np.float32)
                orientations = np.tile(
                    np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32),
                    (count, 1),
                )
                return positions, orientations

        result = read_batched_link_pose(
            _Articulation(),
            num_envs=2,
            link_index=1,
            device=torch.device("cpu"),
        )

        self.assertIsNotNone(result)
        torch.testing.assert_close(
            result[:, 3:7],
            torch.tensor([[2.0, 3.0, 4.0, 1.0], [2.0, 3.0, 4.0, 1.0]]),
        )

    async def test_clone_cleanup_restores_timeline_when_prim_removal_fails(self) -> None:
        """Keep Kit usable when cleanup encounters a secondary USD error."""
        bridge = object.__new__(IsaacSimSysIdBridge)
        bridge.fabric_clones = False
        bridge._physics_replication_registered = False
        bridge._created_clone_paths = ["/World/envs/env_1"]
        bridge._cloned_for_count = 2
        bridge._logical_env_count = 2
        bridge._last_clone_co_locate = True
        bridge._fabric_rebind_logged = False
        bridge._fabric_command_probe_logged = False
        bridge._fabric_state_probe_logged = False
        bridge._articulation = object()
        bridge._clone_paths_to_remove = Mock(return_value=["/World/envs/env_1"])
        bridge._reset_rollout_targets_to_initial_state = Mock()
        bridge._flush_physics_changes = Mock()
        bridge._kit_updates_sync = Mock()
        bridge._unregister_physics_replication = Mock()

        timeline = SimpleNamespace(is_playing=Mock(return_value=True), stop=Mock(), play=Mock())
        stage = SimpleNamespace(
            GetPrimAtPath=Mock(return_value=SimpleNamespace(IsValid=Mock(return_value=True))),
            RemovePrim=Mock(side_effect=RuntimeError("remove failed")),
        )

        with (
            patch(
                "isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge.runtime.get_current_stage",
                return_value=stage,
            ),
            patch(
                "isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge.omni.timeline.get_timeline_interface",
                return_value=timeline,
            ),
            patch(
                "isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge.SimulationManager.initialize_physics"
            ) as initialize,
            self.assertRaisesRegex(RuntimeError, "remove failed"),
        ):
            bridge._remove_cloned_envs_sync()

        timeline.stop.assert_called_once_with()
        timeline.play.assert_called_once_with()
        initialize.assert_called_once_with()
        self.assertEqual(bridge._created_clone_paths, [])
        self.assertEqual(bridge._cloned_for_count, 0)

    async def test_schema_validation_rejects_unordered_parameter_bounds(self) -> None:
        payload = {
            "parameters": {
                "selected": [{"param_type": "joint_friction", "dof_index": 0, "min": 1.0, "max": 0.5, "initial": 2.0}]
            }
        }

        issues = validate_sysid_run_spec_payload(payload)

        self.assertTrue(any(issue.severity == "error" and issue.path.endswith(".min") for issue in issues))
        self.assertTrue(any(issue.severity == "error" and issue.path.endswith(".initial") for issue in issues))

    async def test_schema_validation_accepts_ordered_parameter_bounds(self) -> None:
        payload = {
            "parameters": {
                "selected": [{"param_type": "joint_friction", "dof_index": 0, "min": 0.5, "max": 2.0, "initial": 1.0}]
            }
        }

        issues = validate_sysid_run_spec_payload(payload)

        self.assertFalse(any(issue.severity == "error" for issue in issues))

    async def test_schema_validation_rejects_invalid_confidence_outputs(self) -> None:
        """Reject confidence settings that would otherwise coerce or fail late."""
        invalid_payloads = (
            {"compute_parameter_confidence": "false"},
            {"parameter_confidence_rollout_budget": 0},
            {"parameter_confidence_rollout_budget": "many"},
            {"validation_relative_change_threshold": -0.1},
            {"validation_absolute_change_threshold": "small"},
        )

        for outputs in invalid_payloads:
            issues = validate_sysid_run_spec_payload({"schema_version": 2, "outputs": outputs})
            self.assertTrue(
                any(issue.severity == "error" and issue.path.startswith("$.outputs.") for issue in issues),
                msg=f"Expected an output validation error for {outputs!r}.",
            )
