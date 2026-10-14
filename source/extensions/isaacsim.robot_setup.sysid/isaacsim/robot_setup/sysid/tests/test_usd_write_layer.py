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

"""Tests for routing accepted SysID values into composed physics layers."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import omni.kit.test
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.run_controller import write_optimized_parameters_to_usd
from isaacsim.robot_setup.sysid.usd_parameter_io import read_joint_usd_snapshot
from isaacsim.robot_setup.sysid.usd_write_layer import (
    create_physics_override_stage,
    is_multiphysics_stage,
    resolve_multiphysics_write_targets,
)
from pxr import Sdf, Usd, UsdPhysics


class TestUsdWriteLayer(omni.kit.test.AsyncTestCase):
    """Verify the narrow multi-physics output path."""

    @staticmethod
    def _author_joint(stage: Usd.Stage, path: str) -> None:
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateStiffnessAttr(4.0)
        drive.CreateDampingAttr(2.0)

    @staticmethod
    def _load_apply_tool() -> ModuleType:
        tools_dir = Path(__file__).resolve().parents[4] / "tools"
        solve_spec = importlib.util.spec_from_file_location(
            "_sysid_solve_for_apply_test", tools_dir / "headless_sysid_solve.py"
        )
        solve_module = importlib.util.module_from_spec(solve_spec)
        solve_spec.loader.exec_module(solve_module)
        apply_spec = importlib.util.spec_from_file_location(
            "_sysid_apply_variant_test", tools_dir / "headless_sysid_apply_theta_to_stage.py"
        )
        apply_module = importlib.util.module_from_spec(apply_spec)
        with patch.dict(sys.modules, {"headless_sysid_solve": solve_module}):
            apply_spec.loader.exec_module(apply_module)
        return apply_module

    async def test_copy_preserves_unselected_physics_variant_without_modifying_source(self) -> None:
        """Use the configured solver for copying while leaving both assets unselected."""
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            physics_dir = root / "payloads" / "Physics"
            physics_dir.mkdir(parents=True)
            physics_path = physics_dir / "physics.usda"
            physx_path = physics_dir / "physx.usda"
            source_path = root / "robot.usda"
            output_path = root / "identified.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            robot = physics_stage.DefinePrim("/Robot", "Xform")
            self._author_joint(physics_stage, "/Robot/joint")
            physics_stage.SetDefaultPrim(robot)
            physics_stage.Save()

            physx_layer = Sdf.Layer.CreateNew(str(physx_path))
            physx_layer.defaultPrim = "Robot"
            physx_layer.subLayerPaths = ["physics.usda"]
            physx_layer.Save()
            physx_stage = Usd.Stage.Open(str(physx_path))
            physx_stage.GetPrimAtPath("/Robot/joint").CreateAttribute(
                "physxJoint:jointFriction", Sdf.ValueTypeNames.Float
            ).Set(1.0)
            physx_stage.Save()

            source_layer = Sdf.Layer.CreateNew(str(source_path))
            robot_spec = Sdf.CreatePrimInLayer(source_layer, Sdf.Path("/Robot"))
            robot_spec.specifier = Sdf.SpecifierDef
            robot_spec.typeName = "Xform"
            source_layer.defaultPrim = "Robot"
            physics_variants = Sdf.VariantSetSpec(robot_spec, "Physics")
            robot_spec.variantSetNameList.Append("Physics")
            physx_variant = Sdf.VariantSpec(physics_variants, "physx")
            physx_variant.primSpec.payloadList.Prepend(Sdf.Payload("./payloads/Physics/physx.usda"))
            source_layer.Save()

            source_check_stage = Usd.Stage.Open(str(source_path))
            self.assertEqual(
                source_check_stage.GetPrimAtPath("/Robot").GetVariantSet("Physics").GetVariantSelection(), ""
            )
            self.assertFalse(is_multiphysics_stage(source_check_stage, "/Robot"))
            with Usd.EditContext(source_check_stage, source_check_stage.GetSessionLayer()):
                source_check_stage.GetPrimAtPath("/Robot").GetVariantSet("Physics").SetVariantSelection("physx")
            self.assertTrue(is_multiphysics_stage(source_check_stage, "/Robot"))

            apply_tool = self._load_apply_tool()
            output_physics_path, _ = apply_tool._copy_stage(
                source_path,
                output_path,
                overwrite=False,
                robot_prim_path="/Robot",
                solver_layer_name="physx",
            )
            self.assertIsNotNone(output_physics_path)

            output_stage = Usd.Stage.Open(str(output_path))
            self.assertEqual(output_stage.GetPrimAtPath("/Robot").GetVariantSet("Physics").GetVariantSelection(), "")
            self.assertTrue(apply_tool._select_physics_variant_for_session(output_stage, "/Robot", "physx"))
            self.assertTrue(is_multiphysics_stage(output_stage, "/Robot"))
            source_check_stage = Usd.Stage.Open(str(source_path))
            self.assertEqual(
                source_check_stage.GetPrimAtPath("/Robot").GetVariantSet("Physics").GetVariantSelection(), ""
            )

    async def test_unrelated_solver_layer_does_not_mark_robot_multiphysics(self) -> None:
        """A solver layer used only by another asset must not classify the robot."""
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            physics_path = root / "physics.usda"
            physx_path = root / "physx.usda"
            stage_path = root / "scene.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            physics_stage.DefinePrim("/Robot", "Xform")
            self._author_joint(physics_stage, "/Robot/joint")
            physics_stage.Save()

            physx_stage = Usd.Stage.CreateNew(str(physx_path))
            physx_stage.DefinePrim("/Unrelated", "Xform")
            physx_stage.Save()

            root_layer = Sdf.Layer.CreateNew(str(stage_path))
            root_layer.subLayerPaths = ["physx.usda", "physics.usda"]
            root_layer.Save()

            self.assertFalse(is_multiphysics_stage(Usd.Stage.Open(str(stage_path)), "/Robot"))

    async def test_live_stage_write_targets_map_into_payload_layers(self) -> None:
        """UI writeback must update payload layers through their composed namespace."""
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            physics_path = root / "physics.usda"
            physx_path = root / "physx.usda"
            stage_path = root / "robot.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            payload_robot = physics_stage.DefinePrim("/PayloadRobot", "Xform")
            self._author_joint(physics_stage, "/PayloadRobot/joint")
            physics_stage.SetDefaultPrim(payload_robot)
            physics_stage.Save()

            physx_layer = Sdf.Layer.CreateNew(str(physx_path))
            physx_layer.defaultPrim = "PayloadRobot"
            physx_layer.subLayerPaths = ["physics.usda"]
            physx_layer.Save()
            physx_stage = Usd.Stage.Open(str(physx_path))
            physx_stage.GetPrimAtPath("/PayloadRobot/joint").CreateAttribute(
                "physxJoint:jointFriction", Sdf.ValueTypeNames.Float
            ).Set(1.0)
            physx_stage.Save()

            root_layer = Sdf.Layer.CreateNew(str(stage_path))
            robot_spec = Sdf.CreatePrimInLayer(root_layer, Sdf.Path("/Robot"))
            robot_spec.specifier = Sdf.SpecifierDef
            robot_spec.typeName = "Xform"
            root_layer.defaultPrim = "Robot"
            physics_variants = Sdf.VariantSetSpec(robot_spec, "Physics")
            robot_spec.variantSetNameList.Append("Physics")
            physx_variant = Sdf.VariantSpec(physics_variants, "physx")
            physx_variant.primSpec.payloadList.Prepend(Sdf.Payload("./physx.usda"))
            root_layer.Save()

            stage = Usd.Stage.Open(str(stage_path))
            stage.GetPrimAtPath("/Robot").GetVariantSet("Physics").SetVariantSelection("physx")
            targets = resolve_multiphysics_write_targets(stage, "/Robot", solver_layer_name="physx")
            self.assertIsNotNone(targets)
            self.assertEqual(targets.neutral.GetLayer(), physics_stage.GetRootLayer())
            self.assertEqual(targets.solver.GetLayer(), physx_stage.GetRootLayer())
            self.assertEqual(
                targets.neutral.MapToSpecPath(Sdf.Path("/Robot/joint")),
                Sdf.Path("/PayloadRobot/joint"),
            )

            space = ParameterSpace.for_robot(1)
            space.baseline.usd_joint_snapshots = [read_joint_usd_snapshot(stage, "/Robot/joint")]
            entries = [
                SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
                SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            ]
            config = SimpleNamespace(param_entries=entries, trajectory=SimpleNamespace(num_joints=1))
            self.assertTrue(
                write_optimized_parameters_to_usd(
                    stage,
                    space,
                    config,
                    [3.0, 4.0],
                    1,
                    physics_backend="physx",
                    neutral_write_layer=targets.neutral,
                    solver_write_layer=targets.solver,
                )
            )

            damping = physics_stage.GetAttributeAtPath("/PayloadRobot/joint.drive:angular:physics:damping")
            friction = physx_stage.GetAttributeAtPath("/PayloadRobot/joint.physxJoint:jointFriction")
            self.assertAlmostEqual(float(damping.Get()), 6.0)
            self.assertAlmostEqual(float(friction.Get()), 4.0)
            self.assertIsNone(root_layer.GetAttributeAtPath("/Robot/joint.physxJoint:jointFriction"))

            targets.neutral.GetLayer().Save()
            targets.solver.GetLayer().Save()
            stage.GetRootLayer().Save()
            reopened = Usd.Stage.Open(str(stage_path))
            reopened.GetPrimAtPath("/Robot").GetVariantSet("Physics").SetVariantSelection("physx")
            self.assertAlmostEqual(
                float(reopened.GetAttributeAtPath("/Robot/joint.drive:angular:physics:damping").Get()),
                6.0,
            )
            self.assertAlmostEqual(
                float(reopened.GetAttributeAtPath("/Robot/joint.physxJoint:jointFriction").Get()),
                4.0,
            )

    async def test_masked_stiffness_writeback_fails_until_root_override_is_used(self) -> None:
        """Reject a weaker payload write and accept an effective instance-local override."""
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            physics_path = root / "physics.usda"
            stage_path = root / "robot.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            physics_stage.DefinePrim("/Robot", "Xform")
            self._author_joint(physics_stage, "/Robot/joint")
            physics_stage.Save()

            root_layer = Sdf.Layer.CreateNew(str(stage_path))
            root_layer.subLayerPaths = ["physics.usda"]
            root_layer.Save()
            stage = Usd.Stage.Open(str(stage_path))
            with Usd.EditContext(stage, root_layer):
                UsdPhysics.DriveAPI(stage.GetPrimAtPath("/Robot/joint"), "angular").GetStiffnessAttr().Set(5.0)

            space = ParameterSpace.for_robot(1)
            space.baseline.usd_joint_snapshots = [read_joint_usd_snapshot(stage, "/Robot/joint", dof_index=0)]
            entries = [SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0)]
            config = SimpleNamespace(param_entries=entries, trajectory=SimpleNamespace(num_joints=1))

            with self.assertRaisesRegex(RuntimeError, "authored but is not effective"):
                write_optimized_parameters_to_usd(
                    stage,
                    space,
                    config,
                    [2.0],
                    1,
                    physics_backend="physx",
                    neutral_write_layer=physics_stage.GetRootLayer(),
                )

            self.assertTrue(
                write_optimized_parameters_to_usd(
                    stage,
                    space,
                    config,
                    [2.0],
                    1,
                    physics_backend="physx",
                    neutral_write_layer=root_layer,
                )
            )
            self.assertAlmostEqual(
                float(stage.GetPrimAtPath("/Robot/joint").GetAttribute("drive:angular:physics:stiffness").Get()),
                10.0,
            )

    async def test_bridge_claims_are_routed_by_parameter_type(self) -> None:
        """Bridge-owned neutral and solver parameters must use their respective layers."""
        stage = Usd.Stage.CreateInMemory()
        neutral_layer = Sdf.Layer.CreateAnonymous("physics.usda")
        solver_layer = Sdf.Layer.CreateAnonymous("physx.usda")
        solver_layer.subLayerPaths = [neutral_layer.identifier]
        stage.GetRootLayer().subLayerPaths = [solver_layer.identifier]

        class _Bridge:
            usd_parameter_types = {
                SysIdParameterType.JOINT_DAMPING,
                SysIdParameterType.JOINT_FRICTION,
            }
            usd_parameter_dof_claims = {}

            def __init__(self) -> None:
                self.calls = []

            def write_parameters_to_usd(self, stage, entries, theta_row) -> bool:  # noqa: ANN001, D102
                self.calls.append(
                    (
                        stage.GetEditTarget().GetLayer(),
                        [entry.param_type for entry in entries],
                        theta_row.tolist(),
                    )
                )
                return True

        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
        ]
        config = SimpleNamespace(param_entries=entries, trajectory=SimpleNamespace(num_joints=1))
        bridge = _Bridge()

        self.assertTrue(
            write_optimized_parameters_to_usd(
                stage,
                ParameterSpace.for_robot(1),
                config,
                [3.0, 4.0],
                1,
                bridge,
                neutral_write_layer=neutral_layer,
                solver_write_layer=solver_layer,
            )
        )
        self.assertEqual(
            bridge.calls,
            [
                (neutral_layer, [SysIdParameterType.JOINT_DAMPING], [3.0]),
                (solver_layer, [SysIdParameterType.JOINT_FRICTION], [4.0]),
            ],
        )

    async def test_composed_output_routes_neutral_and_solver_parameters(self) -> None:
        """A relocatable output must preserve multi-physics layer ownership."""
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source_dir = root / "source"
            output_dir = root / "outputs"
            physics_dir = source_dir / "payloads" / "Physics"
            physics_dir.mkdir(parents=True)
            physics_path = physics_dir / "physics.usda"
            physx_path = physics_dir / "physx.usda"
            source_path = source_dir / "scene.usda"
            output_path = output_dir / "identified.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            robot = physics_stage.DefinePrim("/Robot", "Xform")
            self._author_joint(physics_stage, "/Robot/joint")
            physics_stage.SetDefaultPrim(robot)
            physics_stage.Save()

            physx_layer = Sdf.Layer.CreateNew(str(physx_path))
            physx_layer.subLayerPaths = ["physics.usda"]
            physx_layer.Save()
            physx_stage = Usd.Stage.Open(str(physx_path))
            physx_stage.GetPrimAtPath("/Robot/joint").CreateAttribute(
                "physxJoint:jointFriction",
                Sdf.ValueTypeNames.Float,
            ).Set(1.0)
            physx_stage.Save()

            source_layer = Sdf.Layer.CreateNew(str(source_path))
            source_layer.subLayerPaths = ["payloads/Physics/physx.usda"]
            source_layer.defaultPrim = "Robot"
            source_layer.Save()
            source_stage = Usd.Stage.Open(str(source_path))
            self.assertTrue(is_multiphysics_stage(source_stage, "/Robot"))

            create_physics_override_stage(source_path, output_path, solver_layer_name="physx")
            stage = Usd.Stage.Open(str(output_path))
            solver_layer = Sdf.Layer.FindOrOpenRelativeToLayer(
                stage.GetRootLayer(),
                stage.GetRootLayer().subLayerPaths[0],
            )
            self.assertIsNotNone(solver_layer)
            physics_layer = Sdf.Layer.FindOrOpenRelativeToLayer(
                solver_layer,
                solver_layer.subLayerPaths[0],
            )
            self.assertIsNotNone(physics_layer)

            space = ParameterSpace.for_robot(1)
            space.baseline.usd_joint_snapshots = [read_joint_usd_snapshot(stage, "/Robot/joint")]
            entries = [
                SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
                SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            ]
            config = SimpleNamespace(
                param_entries=entries,
                trajectory=SimpleNamespace(num_joints=1),
            )
            self.assertTrue(
                write_optimized_parameters_to_usd(
                    stage,
                    space,
                    config,
                    [3.0, 4.0],
                    1,
                    neutral_write_layer=physics_layer,
                    solver_write_layer=solver_layer,
                )
            )
            physics_layer.Save()
            solver_layer.Save()

            source_damping = physics_stage.GetAttributeAtPath("/Robot/joint.drive:angular:physics:damping")
            source_friction = physx_stage.GetAttributeAtPath("/Robot/joint.physxJoint:jointFriction")
            output_damping = physics_layer.GetAttributeAtPath("/Robot/joint.drive:angular:physics:damping")
            output_friction = solver_layer.GetAttributeAtPath("/Robot/joint.physxJoint:jointFriction")
            self.assertAlmostEqual(float(source_damping.Get()), 2.0)
            self.assertAlmostEqual(float(source_friction.Get()), 1.0)
            self.assertAlmostEqual(float(output_damping.default), 6.0)
            self.assertAlmostEqual(float(output_friction.default), 4.0)
            self.assertIsNone(physics_layer.GetAttributeAtPath("/Robot/joint.physxJoint:jointFriction"))
            self.assertIsNone(solver_layer.GetAttributeAtPath("/Robot/joint.drive:angular:physics:damping"))
            self.assertEqual(stage.GetRootLayer().subLayerPaths[0], "identified_layers/Physics/physx.usda")
            self.assertEqual(solver_layer.subLayerPaths, ["physics.usda"])
