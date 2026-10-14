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

"""Focused preflight validation tests."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import ModuleType
from unittest.mock import patch

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.errors import SysIdRuntimeUnavailableError
from isaacsim.robot_setup.sysid.preflight import (
    SysIdRunPreflight,
    _check_newton_usd_actuator_schemas,
    _check_outputs,
    _check_parameters,
    _check_residuals,
    _check_simulation,
    _check_solver,
    _check_telemetry,
    preflight_sysid_run_spec,
)
from isaacsim.robot_setup.sysid.run_spec import ParameterRunSpec, SysIdRunSpec
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset


class _FakeRelationship:
    def __init__(self, targets: list[str]) -> None:
        self._targets = targets

    def GetTargets(self) -> list[str]:  # noqa: N802
        return self._targets


class _FakeActuatorPrim:
    def __init__(self, path: str, schemas: list[str], targets: list[str]) -> None:
        self._path = path
        self._schemas = schemas
        self._targets = targets

    def GetAppliedSchemas(self) -> list[str]:  # noqa: N802
        return self._schemas

    def GetRelationship(self, name: str) -> _FakeRelationship | None:  # noqa: N802
        return _FakeRelationship(self._targets) if name == "newton:targets" else None

    def GetPath(self) -> str:  # noqa: N802
        return self._path


class _FakeActuatorStage:
    def __init__(self, prims: list[_FakeActuatorPrim]) -> None:
        self._prims = prims

    def Traverse(self) -> list[_FakeActuatorPrim]:  # noqa: N802
        return self._prims


def _bridge_module_with_load_error(error: Exception) -> ModuleType:
    """Build a fake bridge module whose runtime loader raises an error.

    Args:
        error: Exception raised by the fake runtime loader.

    Returns:
        Fake Newton bridge module.
    """
    bridge_module = ModuleType("isaacsim.robot_setup.sysid.newton_sysid_bridge")
    bridge_module._attr_asset_paths = lambda *_args: ("", "")
    bridge_module.classify_newton_neural_prim = lambda *_args: {}

    def _raise_load_error() -> None:
        raise error

    bridge_module._load_newton_modules = _raise_load_error
    return bridge_module


class PreflightValidationTests(omni.kit.test.AsyncTestCase):
    """Reject non-finite values even for programmatically constructed specs."""

    async def test_parallel_clones_with_empty_paths_reports_only_required_errors(self) -> None:
        """Do not claim two missing clone paths are also the same configured path."""
        spec = SysIdRunSpec()
        spec.simulation.parallel_clones = True
        spec.simulation.source_env_path = ""
        spec.simulation.env_paths_root = ""

        result = preflight_sysid_run_spec(
            spec,
            source_exists_fn=lambda *_args: True,
            allow_empty_parameters=True,
        )

        clone_errors = [
            issue.code for issue in result.errors if issue.code in {"source_env_path", "env_paths_root", "clone_paths"}
        ]
        self.assertEqual(clone_errors, ["source_env_path", "env_paths_root"])

    async def test_isaac_sim_newton_availability_check_fails_closed_on_runtime_error(self) -> None:
        """Reject Newton when the runtime cannot verify that its physics engine is registered."""
        spec = SysIdRunSpec()
        spec.simulation.physics_backend = "newton"

        with patch(
            "isaacsim.core.simulation_manager.SimulationManager.get_available_physics_engines",
            side_effect=RuntimeError("registry unavailable"),
        ):
            result = preflight_sysid_run_spec(spec, allow_empty_parameters=True)

        errors = [issue for issue in result.errors if issue.code == "isaac_sim_newton_unavailable"]
        self.assertEqual(len(errors), 1)
        self.assertIn("registry unavailable", errors[0].message)

    async def test_isaac_hosted_newton_rejects_friction_before_rollout(self) -> None:
        """Keep a tensor-backend limitation behind the shared UI/headless Check gate."""
        spec = SysIdRunSpec()
        spec.simulation.engine = "isaac_sim"
        spec.simulation.physics_backend = "newton"
        spec.parameters.selected = [
            ParameterRunSpec(param_type="joint_friction", dof_index=0, initial=1.0, min=0.5, max=1.5)
        ]
        result = SysIdRunPreflight()

        _check_parameters(spec, result)

        self.assertIn("isaac_sim_newton_joint_friction", {issue.code for issue in result.errors})

    async def test_standalone_newton_keeps_friction_available(self) -> None:
        """Do not apply the hosted tensor limitation to the standalone Newton bridge."""
        spec = SysIdRunSpec()
        spec.simulation.engine = "newton"
        spec.parameters.selected = [
            ParameterRunSpec(param_type="joint_friction", dof_index=0, initial=1.0, min=0.5, max=1.5)
        ]
        result = SysIdRunPreflight()

        _check_parameters(spec, result)

        self.assertNotIn("isaac_sim_newton_joint_friction", {issue.code for issue in result.errors})

    async def test_standalone_mujoco_rejects_joint_limit_scales_before_rollout(self) -> None:
        """Mirror the standalone bridge contract in shared preflight."""
        spec = SysIdRunSpec()
        spec.simulation.engine = "newton"
        spec.simulation.newton.solver = "mujoco"
        spec.parameters.selected = [
            ParameterRunSpec(param_type="joint_limit_lower_scale", dof_index=0, initial=1.0, min=0.5, max=1.5)
        ]
        result = SysIdRunPreflight()

        _check_parameters(spec, result)

        self.assertIn("parameter_backend_unsupported", {issue.code for issue in result.errors})

    async def test_programmatic_spec_rejects_unknown_newton_solver(self) -> None:
        """Do not route an unchecked solver typo into the MuJoCo bridge."""
        spec = SysIdRunSpec()
        spec.simulation.engine = "newton"
        spec.simulation.newton.solver = "mujco"
        result = SysIdRunPreflight()

        _check_simulation(spec, result, stage=None)

        errors = [issue for issue in result.errors if issue.code == "backend_capabilities"]
        self.assertEqual(len(errors), 1)
        self.assertIn("mujoco", errors[0].message)
        self.assertIn("featherstone_diff", errors[0].message)

    async def test_standalone_newton_rejects_unavailable_pose_and_contact_residuals(self) -> None:
        """Reject residuals that neither standalone bridge can simulate."""
        spec = SysIdRunSpec()
        spec.simulation.engine = "newton"
        spec.residuals.end_effector_pose_weight = 1.0
        spec.residuals.contact_force_weight = 1.0
        result = SysIdRunPreflight()

        _check_residuals(spec, result, trajectory=None)

        codes = {issue.code for issue in result.errors}
        self.assertIn("residual_backend_end_effector_pose", codes)
        self.assertIn("residual_backend_contact_force", codes)

    async def test_isaac_hosted_newton_rejects_unavailable_force_residuals(self) -> None:
        """Do not treat Newton's placeholder force tensors as measured state."""
        spec = SysIdRunSpec()
        spec.simulation.physics_backend = "newton"
        spec.residuals.torque_weight = 1.0
        spec.residuals.contact_force_weight = 1.0
        result = SysIdRunPreflight()

        _check_residuals(spec, result, trajectory=None)

        codes = {issue.code for issue in result.errors}
        self.assertIn("residual_backend_torque", codes)
        self.assertIn("residual_backend_contact_force", codes)

    async def test_isaac_physx_accepts_every_residual_channel(self) -> None:
        """Keep the full residual surface available on the complete backend."""
        spec = SysIdRunSpec()
        spec.simulation.physics_backend = "physx"
        spec.residuals.torque_weight = 1.0
        spec.residuals.end_effector_pose_weight = 1.0
        spec.residuals.contact_force_weight = 1.0
        result = SysIdRunPreflight()

        _check_residuals(spec, result, trajectory=None)

        self.assertFalse(any(issue.code.startswith("residual_backend_") for issue in result.errors))

    async def test_newton_schema_check_propagates_unexpected_bridge_failure(self) -> None:
        """Surface bridge defects while tolerating only known optional-runtime failures."""
        bridge_module = _bridge_module_with_load_error(RuntimeError("bridge defect"))

        with (
            patch.dict(sys.modules, {bridge_module.__name__: bridge_module}),
            self.assertRaisesRegex(RuntimeError, "bridge defect"),
        ):
            _check_newton_usd_actuator_schemas(_FakeActuatorStage([]), SysIdRunPreflight())

    async def test_newton_schema_check_tolerates_unavailable_optional_runtime(self) -> None:
        """Keep structural schema checks available when Newton is not installed."""
        bridge_module = _bridge_module_with_load_error(SysIdRuntimeUnavailableError("Newton unavailable"))
        stage = _FakeActuatorStage([_FakeActuatorPrim("/Robot/Actuator", [], ["/Robot/Joint"])])
        result = SysIdRunPreflight()

        with patch.dict(sys.modules, {bridge_module.__name__: bridge_module}):
            _check_newton_usd_actuator_schemas(stage, result)

        self.assertIn("newton_actuator_controller_schema", {issue.code for issue in result.errors})

    async def test_preflight_rejects_nonfinite_solver_and_residual_values(self) -> None:
        """Prevent NaN and infinity from bypassing schema validation."""
        spec = SysIdRunSpec()
        spec.solver.epsilon = float("nan")
        spec.solver.damping_initial = float("inf")
        spec.residuals.position_weight = 1.0
        spec.residuals.velocity_weight = float("nan")
        result = SysIdRunPreflight()

        _check_solver(spec, result)
        _check_residuals(spec, result, trajectory=None)

        self.assertIn("solver_epsilon", {issue.code for issue in result.errors})
        self.assertIn("solver_damping", {issue.code for issue in result.errors})
        self.assertIn("residual_weight", {issue.code for issue in result.errors})

    async def test_preflight_rejects_invalid_validation_thresholds(self) -> None:
        """Validate configurable comparison thresholds for programmatic callers."""
        spec = SysIdRunSpec()
        spec.outputs.validation_relative_change_threshold = float("nan")
        spec.outputs.validation_absolute_change_threshold = -1.0
        result = SysIdRunPreflight()

        _check_outputs(spec, result)

        codes = {issue.code for issue in result.errors}
        self.assertIn("validation_relative_change_threshold", codes)
        self.assertIn("validation_absolute_change_threshold", codes)

    async def test_preflight_reports_missing_robot_prim_path_once_with_stage(self) -> None:
        """Centralize the shared robot-path requirement for both simulation engines."""
        for engine in ("isaac_sim", "newton"):
            with self.subTest(engine=engine):
                spec = SysIdRunSpec()
                spec.simulation.engine = engine
                spec.simulation.robot_prim_path = ""

                result = preflight_sysid_run_spec(
                    spec,
                    stage=object(),
                    source_exists_fn=lambda *_args: True,
                    allow_empty_parameters=True,
                )

                robot_path_errors = [issue for issue in result.errors if issue.code == "robot_prim_path"]
                self.assertEqual(len(robot_path_errors), 1)

    async def test_preflight_without_stage_or_input_path_fails_closed(self) -> None:
        """Do not report a run as ready when no stage can validate the robot articulation."""
        spec = SysIdRunSpec()
        spec.telemetry.source_path = "telemetry.csv"
        spec.stage.input_path = ""

        result = preflight_sysid_run_spec(
            spec,
            stage=None,
            source_exists_fn=lambda *_args: True,
            allow_empty_parameters=True,
        )

        stage_issues = [issue for issue in result.issues if issue.code == "stage_input_path"]
        self.assertEqual([issue.severity for issue in stage_issues], ["error"])
        self.assertFalse(result.ok)

    async def test_preflight_rejects_nonfinite_trajectory_channels(self) -> None:
        """Protect callers that construct trajectory objects directly."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 0.1]),
            positions=np.asarray([[0.0], [np.nan]]),
            velocities=np.zeros((2, 1)),
            commands=np.zeros((2, 1)),
        )
        result = SysIdRunPreflight()

        _check_telemetry(SysIdRunSpec(), result, trajectory=trajectory, source_exists_fn=lambda *_args: True)

        self.assertIn("telemetry_nonfinite", {issue.code for issue in result.errors})

    async def test_preflight_rejects_mismatched_shapes_and_nonmonotonic_time(self) -> None:
        """Validate hand-built trajectories as strictly as ingestion-built trajectories."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 0.1, 0.05]),
            positions=np.zeros((3, 1)),
            velocities=np.zeros((2, 1)),
            commands=np.zeros((3, 1)),
            contact_forces=np.zeros((2, 3)),
        )
        result = SysIdRunPreflight()

        _check_telemetry(SysIdRunSpec(), result, trajectory=trajectory, source_exists_fn=lambda *_args: True)

        codes = {issue.code for issue in result.errors}
        self.assertIn("telemetry_shape", codes)
        self.assertIn("telemetry_time_order", codes)

    async def test_lerobot_preflight_reports_missing_pyarrow(self) -> None:
        """Surface the optional LeRobot reader dependency before loading data."""
        spec = SysIdRunSpec()
        spec.telemetry.source_type = "lerobot"
        spec.telemetry.source_path = "dataset"
        result = SysIdRunPreflight()

        with patch("isaacsim.robot_setup.sysid.preflight.importlib.util.find_spec", return_value=None):
            _check_telemetry(spec, result, trajectory=None, source_exists_fn=lambda *_args: True)

        self.assertIn("lerobot_pyarrow_missing", {issue.code for issue in result.errors})

    async def test_lerobot_structural_checks_run_despite_source_callback(self) -> None:
        """Do not let a host existence callback skip local LeRobot layout validation."""
        with tempfile.TemporaryDirectory(prefix="sysid_preflight_lerobot_") as tmp_dir:
            spec = SysIdRunSpec()
            spec.telemetry.source_type = "lerobot"
            spec.telemetry.source_path = tmp_dir
            result = SysIdRunPreflight()

            _check_telemetry(spec, result, trajectory=None, source_exists_fn=lambda *_args: True)

        codes = {issue.code for issue in result.errors}
        self.assertIn("lerobot_info_missing", codes)
        self.assertNotIn("telemetry_source_missing", codes)

    async def test_mcap_preflight_reports_ambiguous_companion_recordings(self) -> None:
        """Resolve the recording during preflight instead of failing inside the loader."""
        with tempfile.TemporaryDirectory(prefix="sysid_preflight_ambiguous_mcap_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "run_a.mcap").write_bytes(b"")
            (root / "run_b.mcap").write_bytes(b"")
            mapping = root / "topic_map.json"
            mapping.write_text(json.dumps({"position_topic": "/joint_states"}), encoding="utf-8")
            spec = SysIdRunSpec()
            spec.telemetry.source_type = "mcap"
            spec.telemetry.source_path = str(root)
            spec.telemetry.mapping_path = str(mapping)
            result = SysIdRunPreflight()

            _check_telemetry(spec, result, trajectory=None, source_exists_fn=lambda *_args: True)

        self.assertIn("telemetry_source_ambiguous", {issue.code for issue in result.errors})

    async def test_ros2_bag_preflight_reports_undeclared_split_recordings(self) -> None:
        """Report a folder of unrelated bags before ingestion merges them into one run."""
        with tempfile.TemporaryDirectory(prefix="sysid_preflight_undeclared_bag_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "run_a_0.mcap").write_bytes(b"")
            (root / "run_b_0.mcap").write_bytes(b"")
            mapping = root / "topic_map.json"
            mapping.write_text(json.dumps({"position_topic": "/joint_states"}), encoding="utf-8")
            spec = SysIdRunSpec()
            spec.telemetry.source_type = "ros2_bag"
            spec.telemetry.source_path = str(root)
            spec.telemetry.mapping_path = str(mapping)
            result = SysIdRunPreflight()

            _check_telemetry(spec, result, trajectory=None, source_exists_fn=lambda *_args: True)

        self.assertIn("telemetry_source_ambiguous", {issue.code for issue in result.errors})

    async def test_preflight_rejects_invalid_topic_mapping_file(self) -> None:
        """Validate topic mappings before the run reaches ingestion."""
        with tempfile.TemporaryDirectory(prefix="sysid_preflight_mapping_") as tmp_dir:
            root = Path(tmp_dir)
            source = root / "telemetry.mcap"
            source.write_bytes(b"")
            mapping = root / "topic_map.json"
            mapping.write_text(
                json.dumps({"position_topic": "/joint_states", "torque_sematic": "external"}),
                encoding="utf-8",
            )
            spec = SysIdRunSpec()
            spec.telemetry.source_type = "mcap"
            spec.telemetry.source_path = str(source)
            spec.telemetry.mapping_path = str(mapping)
            result = SysIdRunPreflight()

            _check_telemetry(spec, result, trajectory=None, source_exists_fn=None)

        self.assertIn("mapping_invalid", {issue.code for issue in result.errors})

    async def test_newton_usd_actuator_schema_validation_accepts_one_controller_and_target(self) -> None:
        """Accept one supported controller, one target, and at most one clamp schema."""
        stage = _FakeActuatorStage(
            [
                _FakeActuatorPrim(
                    "/Robot/Actuator",
                    ["NewtonPDControlAPI", "NewtonMaxEffortClampingAPI"],
                    ["/Robot/Joint"],
                )
            ]
        )
        result = SysIdRunPreflight()

        _check_newton_usd_actuator_schemas(stage, result)

        self.assertEqual(result.errors, [])

    async def test_newton_usd_actuator_schema_validation_reports_all_structural_conflicts(self) -> None:
        """Report controller, target, duplicate-target, clamp, and unsupported-schema conflicts."""
        stage = _FakeActuatorStage(
            [
                _FakeActuatorPrim(
                    "/Robot/MultipleControllers",
                    ["NewtonPDControlAPI", "NewtonPIDControlAPI"],
                    ["/Robot/JointA"],
                ),
                _FakeActuatorPrim("/Robot/MissingTarget", ["NewtonPDControlAPI"], []),
                _FakeActuatorPrim("/Robot/FirstClaim", ["NewtonPDControlAPI"], ["/Robot/JointB"]),
                _FakeActuatorPrim("/Robot/SecondClaim", ["NewtonPIDControlAPI"], ["/Robot/JointB"]),
                _FakeActuatorPrim(
                    "/Robot/MultipleClamps",
                    ["NewtonPDControlAPI", "NewtonMaxEffortClampingAPI", "NewtonDCMotorClampingAPI"],
                    ["/Robot/JointC"],
                ),
                _FakeActuatorPrim(
                    "/Robot/UnsupportedClamp",
                    ["NewtonPositionBasedClampingAPI"],
                    [],
                ),
            ]
        )
        result = SysIdRunPreflight()

        _check_newton_usd_actuator_schemas(stage, result)

        codes = {issue.code for issue in result.errors}
        self.assertIn("newton_actuator_controller_schema", codes)
        self.assertIn("newton_actuator_target", codes)
        self.assertIn("newton_actuator_duplicate_target", codes)
        self.assertIn("newton_actuator_clamp_schema", codes)
        self.assertIn("newton_unsupported_usd_actuator_schema", codes)
