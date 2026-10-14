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

# ruff: noqa: D101, D102

"""Tests for the public direct execution API."""

from __future__ import annotations

import tempfile
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import AsyncMock, Mock, patch

import isaacsim.robot_setup.sysid.execution as execution
import isaacsim.robot_setup.sysid.run_session as run_session
import isaacsim.robot_setup.sysid.runtime as runtime
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.optimizer_base import OptimizationIterationStatus
from isaacsim.robot_setup.sysid.run_spec import SIMULATION_ENGINE_NEWTON, SysIdRunSpec
from isaacsim.robot_setup.sysid.trajectory_segments import (
    ChunkValidationMetric,
    TelemetryChunkRunSpec,
    TrajectoryChunk,
)
from isaacsim.robot_setup.sysid.usd_write_layer import PhysicsWriteTargets


def _snapshot_capable_layer() -> SimpleNamespace:
    # Writeback refuses to run against a layer it cannot snapshot for rollback.
    return SimpleNamespace(
        ExportToString=Mock(return_value="authored"),
        ImportFromString=Mock(return_value=True),
    )


class TestExecution(omni.kit.test.AsyncTestCase):
    async def test_session_factory_forwards_validation_thresholds(self) -> None:
        spec = SysIdRunSpec()
        spec.outputs.validation_relative_change_threshold = 0.2
        spec.outputs.validation_absolute_change_threshold = 0.01
        prepared = object()

        with patch.object(run_session, "SysIdRunSession") as session_type:
            execution.create_sysid_session(spec, prepared)

        self.assertEqual(session_type.call_args.kwargs["validation_relative_change_threshold"], 0.2)
        self.assertEqual(session_type.call_args.kwargs["validation_absolute_change_threshold"], 0.01)

    async def test_session_solves_then_validates_authored_and_tuned_before_writeback(self) -> None:
        events: list[str] = []
        final = OptimizationIterationStatus(
            iteration=1,
            cost=0.25,
            damping=0.1,
            accepted=True,
            theta=[1.5],
        )

        async def evaluate(_config: Any, _chunks: Any, theta: Any, on_progress: Any = None) -> list[SimpleNamespace]:
            value = float(torch.as_tensor(theta).reshape(-1)[0])
            events.append(f"validate:{value:.1f}")
            metric = ChunkValidationMetric(
                "holdout",
                "validation",
                "step",
                0.0,
                1.0,
                10,
                value,
                value,
                value,
                value,
            )
            return [SimpleNamespace(metric=metric, rollout=None)]

        async def solve(_config: Any, on_iteration: Any = None, on_progress: Any = None) -> OptimizationIterationStatus:
            events.append("solve")
            return final

        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=solve,
            evaluate_validation_chunks_with_rollouts=evaluate,
            get_bridge=Mock(return_value=None),
            cleanup_bridge_async=AsyncMock(),
        )
        config = SimpleNamespace(
            trajectory=SimpleNamespace(times=[]),
            theta_initial=torch.tensor([1.2]),
            theta_authored_initial=torch.tensor([1.0]),
            param_entries=[],
        )
        prepared = SimpleNamespace(
            config=config,
            optimizer=optimizer,
            stage=object(),
            parameter_space=object(),
            link_paths=["/Robot/link"],
            validation_chunks=[object()],
            train_chunks=[],
            theta_authored_initial=torch.tensor([1.0]),
            physics_backend="physx",
            physics_solver="mujoco",
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=False,
            compute_parameter_confidence=False,
        )

        def write(*_args: Any, **_kwargs: Any) -> bool:
            events.append("write")
            return True

        with patch.object(run_session, "write_optimized_parameters_to_usd", side_effect=write):
            result = await session.run_async()

        self.assertEqual(events, ["solve", "validate:1.0", "validate:1.5", "write"])
        self.assertTrue(result.validation_comparison["available"])
        self.assertEqual(result.untuned_validation_metrics[0].extra["parameter_state"], "untuned")
        self.assertEqual(result.tuned_validation_metrics[0].extra["parameter_state"], "tuned")

    async def test_prepared_run_uses_public_runner_and_callbacks(self) -> None:
        spec = SysIdRunSpec()
        prepared = SimpleNamespace(simulation_engine=spec.simulation.engine)
        expected = object()
        session = SimpleNamespace(run_async=AsyncMock(return_value=expected))
        on_iteration = Mock()
        on_progress = Mock()

        with patch.object(execution, "create_sysid_session", return_value=session) as create:
            result = await execution.run_sysid(
                spec,
                on_iteration=on_iteration,
                on_progress=on_progress,
                prepared_run=prepared,
                provenance_sidecar_path="result.json",
            )

        self.assertIs(result, expected)
        create.assert_called_once_with(
            spec,
            prepared,
            provenance_sidecar_path="result.json",
            instance_local_writeback=False,
        )
        session.run_async.assert_awaited_once_with(on_iteration=on_iteration, on_progress=on_progress)

    async def test_direct_run_waits_for_optional_runtime_extensions_before_preparation(self) -> None:
        spec = SysIdRunSpec()
        stage = object()
        trajectory = object()
        prepared = SimpleNamespace(simulation_engine=spec.simulation.engine)
        controller = SimpleNamespace(
            load_trajectory=Mock(return_value=trajectory),
            prepare=Mock(return_value=prepared),
        )
        session = SimpleNamespace(run_async=AsyncMock(return_value=object()))

        with (
            patch.object(runtime, "ensure_required_runtime_extensions_async", AsyncMock()) as ensure_runtime,
            patch.object(execution, "_resolve_stage", AsyncMock(return_value=stage)) as resolve_stage,
            patch("isaacsim.robot_setup.sysid.run_controller.SysIdRunController", return_value=controller),
            patch.object(execution, "create_sysid_session", return_value=session),
        ):
            await execution.run_sysid(spec)

        ensure_runtime.assert_awaited_once_with(spec.simulation)
        resolve_stage.assert_awaited_once_with(spec, spec.simulation.engine)
        controller.prepare.assert_called_once_with(stage, trajectory=trajectory, timeline_playing=None)

    async def test_newton_input_path_wins_over_current_kit_stage(self) -> None:
        spec = SysIdRunSpec()
        spec.simulation.engine = SIMULATION_ENGINE_NEWTON
        spec.stage.input_path = "robot.usd"
        opened_stage = object()

        with (
            patch.object(runtime, "kit_runtime_available", return_value=True),
            patch.object(runtime, "open_stage_async", AsyncMock(return_value=opened_stage)) as opener,
            patch.object(runtime, "get_current_stage", return_value=object()) as current,
        ):
            stage = await execution._resolve_stage(spec, SIMULATION_ENGINE_NEWTON)

        self.assertIs(stage, opened_stage)
        opener.assert_awaited_once_with("robot.usd")
        current.assert_not_called()

    async def test_prepared_run_engine_must_match_spec(self) -> None:
        spec = SysIdRunSpec()
        prepared = SimpleNamespace(simulation_engine=SIMULATION_ENGINE_NEWTON)

        with self.assertRaisesRegex(ValueError, "simulation engine does not match"):
            await execution.run_sysid(spec, prepared_run=prepared)

    async def test_yield_control_uses_asyncio_without_bootstrapped_kit(self) -> None:
        sleeper = AsyncMock()
        with (
            patch.object(runtime, "kit_runtime_available", return_value=False),
            patch.object(runtime.asyncio, "sleep", sleeper),
        ):
            await runtime.yield_control()

        sleeper.assert_awaited_once_with(0)

    def test_kit_runtime_is_unavailable_when_iapp_is_not_bootstrapped(self) -> None:
        """An installed Kit module is not proof that a live application exists."""
        with patch("omni.kit.app.get_app", side_effect=RuntimeError("no IApp")):
            self.assertFalse(runtime.kit_runtime_available())

    async def test_requested_usd_writeback_fails_closed_when_nothing_is_written(self) -> None:
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.0],
        )
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=None),
            cleanup_bridge_async=AsyncMock(),
        )
        prepared = SimpleNamespace(
            config=SimpleNamespace(),
            optimizer=optimizer,
            stage=object(),
            parameter_space=object(),
            link_paths=["/World/robot/link"],
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=False,
            compute_parameter_confidence=False,
        )

        with (
            patch.object(run_session, "write_optimized_parameters_to_usd", return_value=False),
            self.assertRaisesRegex(RuntimeError, "no selected parameter was written"),
        ):
            await session.run_async()

        self.assertEqual(session.state, run_session.RunState.ERROR)
        optimizer.cleanup_bridge_async.assert_awaited_once_with(remove_clones=True)

    async def test_ui_writeback_passes_composition_targets_to_writer(self) -> None:
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.0],
        )
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=None),
            cleanup_bridge_async=AsyncMock(),
        )
        prepared = SimpleNamespace(
            config=SimpleNamespace(trajectory=SimpleNamespace(times=[]), param_entries=[]),
            optimizer=optimizer,
            stage=object(),
            parameter_space=object(),
            link_paths=["/Robot/link"],
            validation_chunks=[],
            train_chunks=[],
            physics_backend="physx",
            physics_solver="mujoco",
        )
        neutral_target = _snapshot_capable_layer()
        solver_target = _snapshot_capable_layer()
        targets = PhysicsWriteTargets(neutral=neutral_target, solver=solver_target)
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=False,
            robot_path="/Robot",
            compute_parameter_confidence=False,
        )

        with (
            patch.object(run_session, "resolve_multiphysics_write_targets", return_value=targets) as resolver,
            patch.object(run_session, "write_optimized_parameters_to_usd", return_value=True) as writer,
        ):
            result = await session.run_async()

        self.assertTrue(result.parameters_written)
        resolver.assert_called_once_with(prepared.stage, "/Robot", solver_layer_name="physx")
        self.assertIs(writer.call_args.kwargs["neutral_write_layer"], neutral_target)
        self.assertIs(writer.call_args.kwargs["solver_write_layer"], solver_target)

    async def test_instance_local_writeback_targets_root_and_refreshes_live_physx(self) -> None:
        root_layer = _snapshot_capable_layer()
        stage = SimpleNamespace(
            GetRootLayer=lambda: root_layer,
            GetSessionLayer=lambda: None,
            GetEditTarget=lambda: root_layer,
        )
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.5],
        )
        bridge = SimpleNamespace(apply_accepted_parameters_to_live_articulation=Mock(return_value=True))
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=bridge),
            cleanup_bridge_async=AsyncMock(),
        )
        entries = [object()]
        prepared = SimpleNamespace(
            config=SimpleNamespace(trajectory=SimpleNamespace(times=[]), param_entries=entries),
            optimizer=optimizer,
            stage=stage,
            parameter_space=object(),
            link_paths=["/Robot/link"],
            validation_chunks=[],
            train_chunks=[],
            physics_backend="physx",
            physics_solver="",
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=False,
            robot_path="/Robot",
            compute_parameter_confidence=False,
            instance_local_writeback=True,
        )

        with (
            patch.object(run_session, "resolve_multiphysics_write_targets") as resolver,
            patch.object(run_session, "write_optimized_parameters_to_usd", return_value=True) as writer,
        ):
            result = await session.run_async()

        resolver.assert_not_called()
        self.assertIs(writer.call_args.kwargs["neutral_write_layer"], root_layer)
        self.assertIs(writer.call_args.kwargs["solver_write_layer"], root_layer)
        bridge.apply_accepted_parameters_to_live_articulation.assert_called_once_with([1.5], entries)
        self.assertTrue(result.parameters_written)
        self.assertTrue(result.live_parameters_applied)

    async def test_late_failure_restores_root_and_multiphysics_layers(self) -> None:
        class Layer:
            def __init__(self, content: str) -> None:
                self.content = content

            def ExportToString(self) -> str:  # noqa: N802
                return self.content

            def ImportFromString(self, content: str) -> bool:  # noqa: N802
                self.content = content
                return True

        class EditTarget:
            def __init__(self, layer: Layer) -> None:
                self.layer = layer

            def GetLayer(self) -> Layer:  # noqa: N802
                return self.layer

        root_layer = Layer("root-authored")
        neutral_layer = Layer("neutral-authored")
        solver_layer = Layer("solver-authored")
        stage = SimpleNamespace(
            GetRootLayer=lambda: root_layer,
            GetSessionLayer=lambda: None,
            GetEditTarget=lambda: EditTarget(neutral_layer),
        )
        targets = PhysicsWriteTargets(neutral=EditTarget(neutral_layer), solver=EditTarget(solver_layer))
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.0],
        )
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=None),
            cleanup_bridge_async=AsyncMock(),
        )
        prepared = SimpleNamespace(
            config=SimpleNamespace(trajectory=SimpleNamespace(times=[]), param_entries=[]),
            optimizer=optimizer,
            stage=stage,
            parameter_space=object(),
            link_paths=["/Robot/link"],
            validation_chunks=[],
            train_chunks=[],
            physics_backend="physx",
            physics_solver="mujoco",
            backend_label="LM",
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=True,
            robot_path="/Robot",
            compute_parameter_confidence=False,
        )

        def mutate_layers(*_args: Any, **_kwargs: Any) -> bool:
            root_layer.content = "root-tuned"
            neutral_layer.content = "neutral-tuned"
            solver_layer.content = "solver-tuned"
            return True

        with (
            patch.object(run_session, "resolve_multiphysics_write_targets", return_value=targets),
            patch.object(run_session, "write_optimized_parameters_to_usd", side_effect=mutate_layers),
            patch.object(run_session, "_write_provenance", side_effect=RuntimeError("provenance failed")),
        ):
            with self.assertRaisesRegex(RuntimeError, "provenance failed"):
                await session.run_async()

        self.assertEqual(root_layer.content, "root-authored")
        self.assertEqual(neutral_layer.content, "neutral-authored")
        self.assertEqual(solver_layer.content, "solver-authored")
        self.assertEqual(session.state, run_session.RunState.ERROR)

    async def test_late_failure_restores_live_physx_parameters(self) -> None:
        root_layer = SimpleNamespace(
            ExportToString=Mock(return_value="authored"),
            ImportFromString=Mock(return_value=True),
        )
        stage = SimpleNamespace(
            GetRootLayer=Mock(return_value=root_layer),
            GetSessionLayer=Mock(return_value=None),
            GetEditTarget=Mock(return_value=root_layer),
        )
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.5],
        )
        bridge = SimpleNamespace(
            apply_accepted_parameters_to_live_articulation=Mock(return_value=True),
            restore_parameters_on_live_articulation=Mock(return_value=True),
        )
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=bridge),
            cleanup_bridge_async=AsyncMock(),
        )
        entries = [object()]
        authored = torch.tensor([1.0])
        prepared = SimpleNamespace(
            config=SimpleNamespace(trajectory=SimpleNamespace(times=[]), param_entries=entries),
            optimizer=optimizer,
            stage=stage,
            parameter_space=object(),
            link_paths=["/Robot/link"],
            joint_paths=[],
            validation_chunks=[],
            train_chunks=[],
            theta_authored_initial=authored,
            physics_backend="physx",
            physics_solver="",
            backend_label="LM",
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=True,
            write_usd_provenance=True,
            robot_path="/Robot",
            compute_parameter_confidence=False,
            instance_local_writeback=True,
        )

        with (
            patch.object(run_session, "write_optimized_parameters_to_usd", return_value=True),
            patch.object(run_session, "_write_provenance", side_effect=RuntimeError("provenance failed")),
            self.assertRaisesRegex(RuntimeError, "provenance failed"),
        ):
            await session.run_async()

        bridge.restore_parameters_on_live_articulation.assert_called_once_with(authored, entries)

    async def test_snapshot_rejects_write_target_without_rollback_support(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Cannot snapshot write-target layer"):
            run_session._snapshot_stage_layers(object(), ["/Robot/joint"], object())

    async def test_prim_scoped_snapshot_restores_changed_and_created_specs(self) -> None:
        from pxr import Sdf, Usd

        stage = Usd.Stage.CreateInMemory()
        root_layer = stage.GetRootLayer()
        joint = stage.DefinePrim("/World/Robot/joint")
        joint.CreateAttribute("test:value", Sdf.ValueTypeNames.Float, custom=True).Set(1.0)
        snapshots = run_session._snapshot_stage_layers(
            stage,
            ["/World/Robot/joint", "/World/Robot/created"],
            root_layer,
        )

        self.assertEqual(len(snapshots), 1)
        self.assertIsNone(snapshots[0].serialized_content)
        joint.GetAttribute("test:value").Set(9.0)
        stage.DefinePrim("/World/Robot/created")

        run_session._restore_stage_layers(snapshots)

        self.assertEqual(stage.GetPrimAtPath("/World/Robot/joint").GetAttribute("test:value").Get(), 1.0)
        self.assertFalse(stage.GetPrimAtPath("/World/Robot/created").IsValid())

    async def test_snapshot_maps_stage_paths_into_payload_layer_specs(self) -> None:
        from pxr import Sdf, Usd

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            physics_path = root / "physics.usda"
            physx_path = root / "physx.usda"
            stage_path = root / "robot.usda"

            physics_stage = Usd.Stage.CreateNew(str(physics_path))
            payload_robot = physics_stage.DefinePrim("/PayloadRobot", "Xform")
            joint = physics_stage.DefinePrim("/PayloadRobot/joint", "Xform")
            joint.CreateAttribute("test:value", Sdf.ValueTypeNames.Float, custom=True).Set(1.0)
            physics_stage.SetDefaultPrim(payload_robot)
            physics_stage.Save()

            physx_layer = Sdf.Layer.CreateNew(str(physx_path))
            physx_layer.defaultPrim = "PayloadRobot"
            physx_layer.subLayerPaths = ["physics.usda"]
            physx_layer.Save()

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
            targets = run_session.resolve_multiphysics_write_targets(stage, "/Robot", solver_layer_name="physx")
            self.assertIsNotNone(targets)
            snapshots = run_session._snapshot_stage_layers(stage, ["/Robot/joint"], targets.neutral)

            with Usd.EditContext(stage, targets.neutral):
                stage.GetPrimAtPath("/Robot/joint").GetAttribute("test:value").Set(9.0)
            self.assertEqual(physics_stage.GetAttributeAtPath("/PayloadRobot/joint.test:value").Get(), 9.0)

            run_session._restore_stage_layers(snapshots)

            self.assertEqual(physics_stage.GetAttributeAtPath("/PayloadRobot/joint.test:value").Get(), 1.0)

    async def test_training_replay_fallback_keeps_every_prepared_training_chunk(self) -> None:
        def chunk(name: str, start: float, end: float) -> TrajectoryChunk:
            trajectory = SimpleNamespace(times=[start, end])
            return TrajectoryChunk(
                spec=TelemetryChunkRunSpec(name=name, role="train", start=start, end=end),
                trajectory=trajectory,
                sample_count=2,
                duration_seconds=end - start,
            )

        training = [chunk("sweep", 0.0, 1.0), chunk("chirp", 2.0, 3.0)]

        replay = run_session._training_replay_validation_chunks(
            SimpleNamespace(train_chunks=training),
            SimpleNamespace(trajectory=training[0].trajectory),
        )

        self.assertEqual([item.spec.name for item in replay], ["Training replay: sweep", "Training replay: chirp"])
        self.assertEqual([item.spec.role for item in replay], ["validation", "validation"])
        self.assertEqual([item.trajectory for item in replay], [item.trajectory for item in training])

    async def test_provenance_only_failure_restores_its_edit_target(self) -> None:
        class Layer:
            def __init__(self) -> None:
                self.content = "authored"

            def ExportToString(self) -> str:  # noqa: N802
                return self.content

            def ImportFromString(self, content: str) -> bool:  # noqa: N802
                self.content = content
                return True

        layer = Layer()
        stage = SimpleNamespace(
            GetSessionLayer=Mock(return_value=None),
            GetEditTarget=Mock(return_value=layer),
        )
        final = OptimizationIterationStatus(
            iteration=1,
            cost=1.0,
            damping=0.1,
            accepted=True,
            theta=[1.0],
        )
        optimizer = SimpleNamespace(
            reset_cancel=Mock(),
            run_async=AsyncMock(return_value=final),
            get_bridge=Mock(return_value=None),
            cleanup_bridge_async=AsyncMock(),
        )
        prepared = SimpleNamespace(
            config=SimpleNamespace(trajectory=SimpleNamespace(times=[]), param_entries=[]),
            optimizer=optimizer,
            stage=stage,
            parameter_space=object(),
            link_paths=[],
            joint_paths=[],
            validation_chunks=[],
            train_chunks=[],
            physics_backend="physx",
            physics_solver="",
            backend_label="LM",
        )
        session = run_session.SysIdRunSession(
            prepared,
            apply_params_to_usd=False,
            write_usd_provenance=True,
            robot_path="/Robot",
            compute_parameter_confidence=False,
        )

        def fail_provenance(*_args: Any, **_kwargs: Any) -> str:
            layer.content = "partial-provenance"
            raise RuntimeError("provenance failed")

        with (
            patch.object(run_session, "_write_provenance", side_effect=fail_provenance),
            self.assertRaisesRegex(RuntimeError, "provenance failed"),
        ):
            await session.run_async()

        self.assertEqual(layer.content, "authored")
