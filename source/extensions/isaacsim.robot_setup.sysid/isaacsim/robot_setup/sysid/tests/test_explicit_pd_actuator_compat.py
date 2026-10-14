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

# ruff: noqa: ANN001, ANN003, ANN202, ANN204, D100, D101, D102

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.actuator_compatibility import ACTUATOR_RUNTIME_EXPLICIT
from isaacsim.robot_setup.sysid.explicit_pd_actuator_compat import (
    ExplicitPdActuatorCompat,
    ExplicitPdActuatorDescriptor,
    _actuators_by_dof_index,
    discard_session_actuator_promotions,
    discover_explicit_pd_actuators,
    promote_selected_dofs_to_session_actuators,
    validate_explicit_pd_stage,
)
from isaacsim.robot_setup.sysid.parameter_apply import ParameterApplyState
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import (
    GLOBAL_DOF_INDEX,
    SysIdParameterType,
    build_extended_parameter_specs,
)
from isaacsim.robot_setup.sysid.preflight import preflight_sysid_run_spec
from isaacsim.robot_setup.sysid.run_spec import ParameterRunSpec, SysIdRunSpec
from isaacsim.robot_setup.sysid.schema_validation import validate_sysid_run_spec_payload
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.usd_parameter_io import read_joint_usd_snapshot
from pxr import Sdf, Usd, UsdPhysics


class _Array:
    def __init__(self, values):
        self.values = np.asarray(values).copy()

    def assign(self, values):
        self.values = np.asarray(values).copy()

    def numpy(self):
        return self.values


class _DriveArticulation:
    def __init__(self):
        self.gain_writes = []

    def set_dof_gains(self, **kwargs):
        self.gain_writes.append(kwargs)


class _Manager:
    def __init__(self, articulation=None):
        self.reset_count = 0
        self.closed = False
        self.articulation = articulation or _DriveArticulation()

    def reset(self):
        self.reset_count += 1

    def close(self):
        self.closed = True


def _actuator():
    return SimpleNamespace(
        drive=SimpleNamespace(kp=_Array([0.0, 0.0]), kd=_Array([0.0, 0.0])),
        delay=SimpleNamespace(delay_steps=_Array([0, 0])),
    )


def _stage_with_pd(*, controller_schema: str = "NewtonPDControlAPI"):
    stage = Usd.Stage.CreateInMemory()
    root = stage.DefinePrim("/World/Robot", "Xform")
    UsdPhysics.ArticulationRootAPI.Apply(root)
    UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint1")
    actuator = stage.DefinePrim("/World/Robot/joint1_actuator", "Xform")
    actuator.AddAppliedSchema(controller_schema)
    actuator.AddAppliedSchema("NewtonActuatorDelayAPI")
    actuator.AddAppliedSchema("NewtonMaxEffortClampingAPI")
    actuator.CreateRelationship("newton:targets").AddTarget(Sdf.Path("/World/Robot/joint1"))
    actuator.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(12.0)
    actuator.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(3.0)
    actuator.CreateAttribute("newton:constEffort", Sdf.ValueTypeNames.Float).Set(0.5)
    actuator.CreateAttribute("newton:delaySteps", Sdf.ValueTypeNames.Int).Set(2)
    actuator.CreateAttribute("newton:maxEffort", Sdf.ValueTypeNames.Float).Set(40.0)
    return stage


class ExplicitPdActuatorCompatTests(omni.kit.test.AsyncTestCase):
    async def test_maps_actuators_by_their_remapped_dof_indices(self) -> None:
        shoulder_actuator = SimpleNamespace(indices=_Array([0, 3]))
        wrist_actuator = SimpleNamespace(indices=_Array([2, 5]))
        manager = SimpleNamespace(
            articulation=SimpleNamespace(num_dofs=3),
            actuators=[wrist_actuator, shoulder_actuator],
            actuated_dof_indices=[0, 2],
        )

        by_dof_index = _actuators_by_dof_index(manager)

        self.assertIs(by_dof_index[0], shoulder_actuator)
        self.assertIs(by_dof_index[2], wrist_actuator)

    async def test_maps_grouped_actuator_to_each_controlled_dof(self) -> None:
        grouped_actuator = SimpleNamespace(indices=_Array([0, 2, 3, 5]))
        manager = SimpleNamespace(
            articulation=SimpleNamespace(num_dofs=3),
            actuators=[grouped_actuator],
        )

        by_dof_index = _actuators_by_dof_index(manager)

        self.assertIs(by_dof_index[0], grouped_actuator)
        self.assertIs(by_dof_index[2], grouped_actuator)

    async def test_create_applies_candidates_in_selected_dof_order(self) -> None:
        stage = _stage_with_pd()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint2")
        prim = stage.DefinePrim("/World/Robot/joint2_actuator", "Xform")
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateRelationship("newton:targets").SetTargets([Sdf.Path("/World/Robot/joint2")])
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(20.0)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(5.0)
        dof_indices = {"joint1": 0, "joint2": 1}
        manager = _Manager(SimpleNamespace(num_dofs=2, get_dof_indices=lambda name: _Array([dof_indices[name]])))

        def from_actuators(paths, configs, **kwargs):
            manager.actuated_dof_indices = [0, 1]
            manager.actuators = [
                SimpleNamespace(
                    indices=_Array([dof_indices[name], 2 + dof_indices[name]]),
                    drive=config.controller,
                    delay=config.delay,
                )
                for config, name in configs
            ]
            return manager

        runtime = SimpleNamespace(
            wp=SimpleNamespace(array=lambda values, **kwargs: _Array(values), float32=np.float32, int32=np.int32),
            DrivePD=SimpleNamespace,
            DrivePID=SimpleNamespace,
            Delay=SimpleNamespace,
            ClampingMaxEffort=SimpleNamespace,
            ActuatorConfig=SimpleNamespace,
            ArticulationActuators=SimpleNamespace(from_actuators=from_actuators),
        )
        compat = ExplicitPdActuatorCompat.create(
            stage=stage,
            articulation_root="/World/Robot",
            articulation_paths="/World/Robot",
            selected_dof_paths=["/World/Robot/joint2", "/World/Robot/joint1"],
            env_count=2,
            physics_dt=0.01,
            max_delay_seconds=0.1,
            runtime=runtime,
        )
        try:
            compat.apply_candidate_batch(
                stiffness_scale=np.asarray([[2.0, 3.0], [4.0, 5.0]]),
                damping_scale=np.asarray([[3.0, 4.0], [5.0, 6.0]]),
                delay_seconds=None,
            )
            joint2_actuator, joint1_actuator = manager.actuators
            np.testing.assert_allclose(joint2_actuator.drive.kp.numpy(), [40.0, 80.0])
            np.testing.assert_allclose(joint1_actuator.drive.kp.numpy(), [36.0, 60.0])
            np.testing.assert_allclose(joint2_actuator.drive.kd.numpy(), [15.0, 25.0])
            np.testing.assert_allclose(joint1_actuator.drive.kd.numpy(), [12.0, 18.0])
        finally:
            compat.close()

    async def test_discovers_supported_pd_surface_in_selected_dof_order(self) -> None:
        stage = _stage_with_pd()

        descriptors = discover_explicit_pd_actuators(
            stage,
            "/World/Robot",
            selected_dof_paths=["/World/Robot/joint1"],
        )

        self.assertEqual(len(descriptors), 1)
        self.assertEqual(descriptors[0].dof_name, "joint1")
        self.assertEqual(descriptors[0].kp, 12.0)
        self.assertEqual(descriptors[0].kd, 3.0)
        self.assertEqual(descriptors[0].delay_steps, 2)
        self.assertEqual(descriptors[0].max_effort, 40.0)

    async def test_accepts_pid_controller_as_mutable_explicit_actuator(self) -> None:
        stage = _stage_with_pd(controller_schema="NewtonPIDControlAPI")

        descriptors = discover_explicit_pd_actuators(stage, "/World/Robot")

        self.assertEqual(descriptors[0].controller_kind, "pid")

    async def test_rejects_ambiguous_controller_and_clamp_schema_ownership(self) -> None:
        stage = _stage_with_pd()
        actuator = stage.GetPrimAtPath("/World/Robot/joint1_actuator")
        actuator.AddAppliedSchema("NewtonPIDControlAPI")
        actuator.AddAppliedSchema("NewtonDCMotorClampingAPI")

        with self.assertRaisesRegex(ValueError, "both PD and PID"):
            discover_explicit_pd_actuators(stage, "/World/Robot")

    async def test_rejects_invalid_pid_integral_gain(self) -> None:
        stage = _stage_with_pd(controller_schema="NewtonPIDControlAPI")
        actuator = stage.GetPrimAtPath("/World/Robot/joint1_actuator")
        actuator.CreateAttribute("newton:ki", Sdf.ValueTypeNames.Float).Set(-1.0)

        with self.assertRaisesRegex(ValueError, "newton:ki"):
            discover_explicit_pd_actuators(stage, "/World/Robot")

    async def test_unselected_non_pd_actuator_does_not_reject_selected_pd_joint(self) -> None:
        stage = _stage_with_pd()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint2")
        actuator = stage.DefinePrim("/World/Robot/joint2_actuator", "Xform")
        actuator.AddAppliedSchema("NewtonPIDControlAPI")
        actuator.CreateRelationship("newton:targets").AddTarget(Sdf.Path("/World/Robot/joint2"))

        descriptors = discover_explicit_pd_actuators(
            stage,
            "/World/Robot",
            selected_dof_paths=["/World/Robot/joint1"],
        )

        self.assertEqual([item.target_path for item in descriptors], ["/World/Robot/joint1"])

    async def test_selected_dof_validation_reports_missing_actuator(self) -> None:
        stage = _stage_with_pd()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint2")

        errors = validate_explicit_pd_stage(
            stage,
            "/World/Robot",
            selected_dof_paths=["/World/Robot/joint1", "/World/Robot/joint2"],
        )

        self.assertEqual(len(errors), 1)
        self.assertIn("joint2", errors[0])

    async def test_promotes_only_missing_selected_joint_in_session_layer(self) -> None:
        stage = _stage_with_pd()
        joint2 = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint2")
        drive = UsdPhysics.DriveAPI.Apply(joint2.GetPrim(), "angular")
        drive.CreateStiffnessAttr().Set(25.0)
        drive.CreateDampingAttr().Set(4.0)

        promotions = promote_selected_dofs_to_session_actuators(
            stage,
            "/World/Robot",
            ["/World/Robot/joint1", "/World/Robot/joint2"],
            include_delay=True,
        )

        self.assertEqual(len(promotions), 1)
        prim = stage.GetPrimAtPath(promotions[0].prim_path)
        self.assertTrue(prim.IsValid())
        self.assertAlmostEqual(prim.GetAttribute("newton:kp").Get(), 25.0 * 180.0 / np.pi, places=4)
        self.assertAlmostEqual(prim.GetAttribute("newton:kd").Get(), 4.0 * 180.0 / np.pi, places=4)
        self.assertEqual(prim.GetAttribute("newton:delaySteps").Get(), 0)
        self.assertIsNotNone(stage.GetSessionLayer().GetPrimAtPath(promotions[0].prim_path))

        discard_session_actuator_promotions(stage, promotions)
        self.assertFalse(stage.GetPrimAtPath(promotions[0].prim_path).IsValid())

    async def test_prismatic_promotion_keeps_linear_drive_gain_units(self) -> None:
        stage = _stage_with_pd()
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/Robot/slider")
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateStiffnessAttr().Set(25.0)
        drive.CreateDampingAttr().Set(4.0)

        promotions = promote_selected_dofs_to_session_actuators(
            stage,
            "/World/Robot",
            ["/World/Robot/slider"],
        )

        prim = stage.GetPrimAtPath(promotions[0].prim_path)
        self.assertEqual(prim.GetAttribute("newton:kp").Get(), 25.0)
        self.assertEqual(prim.GetAttribute("newton:kd").Get(), 4.0)

    async def test_preflight_accepts_explicit_isaac_runtime(self) -> None:
        stage = _stage_with_pd()
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint2")
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 0.1]),
            positions=np.zeros((2, 2), dtype=np.float32),
            velocities=np.zeros((2, 2), dtype=np.float32),
            commands=np.zeros((2, 2), dtype=np.float32),
        )
        spec = SysIdRunSpec()
        spec.simulation.robot_prim_path = "/World/Robot"
        spec.simulation.actuator_runtime = ACTUATOR_RUNTIME_EXPLICIT
        spec.parameters.selected = [
            ParameterRunSpec(param_type=SysIdParameterType.JOINT_STIFFNESS.value, min=0.5, max=2.0)
        ]

        result = preflight_sysid_run_spec(spec, stage=stage, trajectory=trajectory)

        errors = [issue.message for issue in result.errors if issue.code == "isaac_sim_actuator_runtime_unsupported"]
        self.assertEqual(errors, [])

    async def test_candidate_batch_updates_existing_arrays_and_quantizes_delay(self) -> None:
        descriptor = ExplicitPdActuatorDescriptor(
            prim_path="/World/Robot/joint1_actuator",
            target_path="/World/Robot/joint1",
            dof_name="joint1",
            kp=10.0,
            kd=2.0,
            const_effort=0.0,
            delay_steps=1,
            max_effort=40.0,
        )
        manager = _Manager()
        actuator = _actuator()
        compat = ExplicitPdActuatorCompat(
            manager=manager,
            descriptors=[descriptor],
            actuators=[actuator],
            env_count=2,
            physics_dt=0.01,
            max_delay_steps=20,
        )

        compat.apply_candidate_batch(
            stiffness_scale=np.asarray([[2.0], [0.5]], dtype=np.float32),
            damping_scale=np.asarray([[3.0], [0.25]], dtype=np.float32),
            delay_seconds=np.asarray([[0.05], [0.11]], dtype=np.float32),
        )

        np.testing.assert_allclose(actuator.drive.kp.values, [20.0, 5.0])
        np.testing.assert_allclose(actuator.drive.kd.values, [6.0, 0.5])
        np.testing.assert_array_equal(actuator.delay.delay_steps.values, [5, 11])
        self.assertEqual(manager.reset_count, 1)

    async def test_writeback_targets_newton_attributes_not_drive_api(self) -> None:
        stage = _stage_with_pd()
        descriptor = discover_explicit_pd_actuators(stage, "/World/Robot")[0]
        compat = ExplicitPdActuatorCompat(
            manager=_Manager(),
            descriptors=[descriptor],
            actuators=[_actuator()],
            env_count=2,
            physics_dt=0.01,
            max_delay_steps=20,
        )

        written = compat.write_parameters_to_usd(
            stage,
            kp=np.asarray([15.0]),
            kd=np.asarray([4.0]),
            delay_seconds=np.asarray([0.07]),
        )

        self.assertTrue(written)
        prim = stage.GetPrimAtPath(descriptor.prim_path)
        self.assertEqual(prim.GetAttribute("newton:kp").Get(), 15.0)
        self.assertEqual(prim.GetAttribute("newton:kd").Get(), 4.0)
        self.assertEqual(prim.GetAttribute("newton:delaySteps").Get(), 7)

    async def test_explicit_writeback_does_not_create_implicit_drive_api(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint1")
        space = ParameterSpace.for_robot(1)
        space.read_usd_baselines(stage, [], ["/World/Robot/joint1"])
        state = ParameterApplyState(
            friction_scale=np.asarray([1.5], dtype=np.float32),
            stiffness_scale=np.asarray([2.0], dtype=np.float32),
            damping_scale=np.asarray([3.0], dtype=np.float32),
            joint_integral_gain=None,
            joint_armature=None,
            actuator_command_delay_seconds=None,
            link_mass_scale=np.ones(1, dtype=np.float32),
            link_com_delta=np.zeros((1, 3), dtype=np.float32),
        )

        space.write_usd_from_baselines(
            stage,
            state,
            skip_parameter_types={
                SysIdParameterType.JOINT_STIFFNESS,
                SysIdParameterType.JOINT_DAMPING,
            },
        )

        self.assertNotIn("PhysicsDriveAPI:angular", [str(name) for name in joint.GetPrim().GetAppliedSchemas()])

    async def test_close_restores_live_drive_gains(self) -> None:
        articulation = _DriveArticulation()
        manager = _Manager(articulation=articulation)
        descriptor = ExplicitPdActuatorDescriptor(
            prim_path="/World/Robot/joint1_actuator",
            target_path="/World/Robot/joint1",
            dof_name="joint1",
            kp=10.0,
            kd=2.0,
            const_effort=0.0,
            delay_steps=0,
            max_effort=40.0,
        )
        compat = ExplicitPdActuatorCompat(
            manager=manager,
            descriptors=[descriptor],
            actuators=[_actuator()],
            env_count=2,
            physics_dt=0.01,
            max_delay_steps=20,
            restore_stiffness=np.asarray([[5.0], [6.0]], dtype=np.float32),
            restore_damping=np.asarray([[1.0], [1.5]], dtype=np.float32),
            restore_dof_indices=[0],
        )

        compat.close()

        self.assertTrue(manager.closed)
        self.assertEqual(len(articulation.gain_writes), 1)
        np.testing.assert_allclose(articulation.gain_writes[0]["stiffnesses"], [[5.0], [6.0]])
        np.testing.assert_allclose(articulation.gain_writes[0]["dampings"], [[1.0], [1.5]])
        self.assertEqual(articulation.gain_writes[0]["dof_indices"], [0])

    async def test_explicit_runtime_baselines_do_not_overwrite_drive_api(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint1")
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateStiffnessAttr().Set(5.0)
        drive.CreateDampingAttr().Set(2.0)
        space = ParameterSpace.for_robot(1)
        space.read_usd_baselines(stage, [], ["/World/Robot/joint1"])
        authored_snapshot = read_joint_usd_snapshot(stage, "/World/Robot/joint1")

        space.override_joint_runtime_baselines(
            stiffness=np.asarray([100.0]),
            damping=np.asarray([10.0]),
            preserve_usd_drive_gains=True,
        )
        self.assertEqual(space.baseline.joint_stiffness[0], 100.0)
        self.assertEqual(space.baseline.joint_damping[0], 10.0)

        state = ParameterApplyState(
            friction_scale=np.ones(1, dtype=np.float32),
            stiffness_scale=np.asarray([3.0], dtype=np.float32),
            damping_scale=np.asarray([4.0], dtype=np.float32),
            joint_integral_gain=None,
            joint_armature=None,
            actuator_command_delay_seconds=None,
            link_mass_scale=np.ones(1, dtype=np.float32),
            link_com_delta=np.zeros((1, 3), dtype=np.float32),
        )
        space.write_usd_from_baselines(
            stage,
            state,
            skip_parameter_types={
                SysIdParameterType.JOINT_STIFFNESS,
                SysIdParameterType.JOINT_DAMPING,
            },
        )

        snapshot = read_joint_usd_snapshot(stage, "/World/Robot/joint1")
        self.assertEqual(snapshot.stiffness, authored_snapshot.stiffness)
        self.assertEqual(snapshot.damping, authored_snapshot.damping)
        self.assertEqual(drive.GetStiffnessAttr().Get(), 5.0)
        self.assertEqual(drive.GetDampingAttr().Get(), 2.0)

    async def test_mixed_writeback_preserves_explicit_drive_and_updates_implicit_drive(self) -> None:
        stage = Usd.Stage.CreateInMemory()
        drives = []
        for index in range(2):
            joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Robot/joint{index + 1}")
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateStiffnessAttr().Set(5.0 + index)
            drive.CreateDampingAttr().Set(2.0 + index)
            drives.append(drive)
        space = ParameterSpace.for_robot(2)
        space.read_usd_baselines(
            stage,
            [],
            ["/World/Robot/joint1", "/World/Robot/joint2"],
        )
        state = ParameterApplyState(
            friction_scale=np.ones(2, dtype=np.float32),
            stiffness_scale=np.asarray([3.0, 4.0], dtype=np.float32),
            damping_scale=np.asarray([5.0, 6.0], dtype=np.float32),
            joint_integral_gain=None,
            joint_armature=None,
            actuator_command_delay_seconds=None,
            link_mass_scale=np.ones(1, dtype=np.float32),
            link_com_delta=np.zeros((1, 3), dtype=np.float32),
        )
        entries = [
            ParameterRunSpec(
                param_type=SysIdParameterType.JOINT_STIFFNESS.value,
                dof_index=GLOBAL_DOF_INDEX,
            ).to_entry(),
            ParameterRunSpec(
                param_type=SysIdParameterType.JOINT_DAMPING.value,
                dof_index=GLOBAL_DOF_INDEX,
            ).to_entry(),
        ]

        space.write_usd_from_baselines(
            stage,
            state,
            skip_joint_parameter_dofs={
                SysIdParameterType.JOINT_STIFFNESS: {0},
                SysIdParameterType.JOINT_DAMPING: {0},
            },
            param_entries=entries,
        )

        self.assertEqual(drives[0].GetStiffnessAttr().Get(), 5.0)
        self.assertEqual(drives[0].GetDampingAttr().Get(), 2.0)
        self.assertEqual(drives[1].GetStiffnessAttr().Get(), 24.0)
        self.assertEqual(drives[1].GetDampingAttr().Get(), 18.0)

    async def test_registry_and_run_spec_expose_opt_in_delay_and_actuator_runtime(self) -> None:
        specs = build_extended_parameter_specs(
            2,
            1,
            include_basic=False,
            include_command_delay=True,
        )
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].param_type, SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS)
        self.assertEqual(specs[0].dof_index, GLOBAL_DOF_INDEX)

        spec = SysIdRunSpec.from_dict({"simulation": {"actuator_runtime": ACTUATOR_RUNTIME_EXPLICIT}})
        self.assertEqual(spec.simulation.actuator_runtime, ACTUATOR_RUNTIME_EXPLICIT)
        self.assertEqual(spec.to_dict()["simulation"]["actuator_runtime"], ACTUATOR_RUNTIME_EXPLICIT)
        issues = validate_sysid_run_spec_payload(spec.to_dict())
        self.assertFalse([issue for issue in issues if issue.path == "$.simulation.actuator_runtime"])
