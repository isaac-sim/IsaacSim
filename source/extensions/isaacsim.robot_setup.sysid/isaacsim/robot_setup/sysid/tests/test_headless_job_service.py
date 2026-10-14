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

# ruff: noqa: ANN001, ANN003, ANN202, D102

"""Tests for the reusable headless job API and persistent service queue."""

from __future__ import annotations

import ast
import asyncio
import importlib.util
import json
import sys
import tempfile
import types
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import AsyncMock, Mock, patch

import isaacsim.robot_setup.sysid.headless_job as headless_job
import isaacsim.robot_setup.sysid.job_service as job_service
import omni.kit.test


class TestHeadlessSysIdJob(omni.kit.test.AsyncTestCase):
    """Verify that the public job API owns one reusable solve lifecycle."""

    async def test_run_uses_existing_kit_without_simulation_app(self) -> None:
        spec = SimpleNamespace(
            stage=SimpleNamespace(input_path="robot.usd"),
            telemetry=SimpleNamespace(chunks=[]),
            simulation=SimpleNamespace(engine="isaac_sim"),
            to_dict=Mock(return_value={}),
        )
        stage = object()
        trajectory = object()
        solve_result = object()
        preflight = SimpleNamespace(raise_for_errors=Mock())
        # Preparation assesses telemetry quality while building the check report,
        # and hands the assessment back on the prepared run for everyone after it.
        quality = object()
        prepared = SimpleNamespace(telemetry_quality=quality)
        schema_issues = [object()]
        raw_spec_payload = {"solver": {"max_iteration": 3}}
        controller = SimpleNamespace(
            load_trajectory=Mock(return_value=trajectory),
            preflight=Mock(return_value=preflight),
            prepare=Mock(return_value=prepared),
        )
        on_prepared = Mock()
        progress: list[tuple[str, float]] = []

        async def _solve(*_args: Any, on_progress: Any = None, **_kwargs: Any) -> object:
            # The optimizer reports its own progress starting from zero.
            on_progress("Solving", 0.0)
            return solve_result

        execution_module = types.ModuleType("isaacsim.robot_setup.sysid.execution")
        execution_module.run_sysid = AsyncMock(side_effect=_solve)
        controller_module = types.ModuleType("isaacsim.robot_setup.sysid.run_controller")
        controller_module.SysIdRunController = Mock(return_value=controller)
        schema_module = types.ModuleType("isaacsim.robot_setup.sysid.schema_validation")
        schema_module.validate_sysid_run_spec_payload = Mock(return_value=schema_issues)
        schema_module.raise_for_schema_errors = Mock()

        with (
            patch.dict(
                sys.modules,
                {
                    execution_module.__name__: execution_module,
                    controller_module.__name__: controller_module,
                    schema_module.__name__: schema_module,
                },
            ),
            patch.object(headless_job, "_enable_backend_extension", AsyncMock()) as enable,
            patch(
                "isaacsim.robot_setup.sysid.runtime.ensure_required_runtime_extensions_async",
                new_callable=AsyncMock,
                return_value=[],
            ) as ensure_runtime,
            patch.object(headless_job, "_resolve_stage", AsyncMock(return_value=stage)) as resolve,
            patch.object(headless_job, "_start_timeline", AsyncMock()) as start,
            patch.object(headless_job, "_save_stage", AsyncMock(return_value=True)) as save,
        ):
            outcome = await headless_job.run_headless_sysid_job(
                headless_job.HeadlessSysIdJobRequest(
                    spec=spec,
                    raw_spec_payload=raw_spec_payload,
                    timeline_warmup_updates=3,
                    save_stage=True,
                ),
                on_prepared=on_prepared,
                on_progress=lambda message, fraction: progress.append((message, fraction)),
            )

        enable.assert_awaited_once_with()
        ensure_runtime.assert_awaited_once_with(spec.simulation)
        resolve.assert_awaited_once_with("robot.usd")
        self.assertEqual(
            [message for message, _ in progress],
            ["Opening stage", "Loading telemetry", "Running preflight", "Preparing run and pre-solve check", "Solving"],
        )
        fractions = [fraction for _, fraction in progress]
        self.assertEqual(fractions, sorted(fractions), "Reported progress must never move backward.")
        start.assert_awaited_once_with(3)
        save.assert_awaited_once_with(stage)
        preflight.raise_for_errors.assert_called_once_with()
        schema_module.validate_sysid_run_spec_payload.assert_called_once_with(raw_spec_payload)
        on_prepared.assert_called_once_with(prepared, preflight, quality, schema_issues)
        execution_module.run_sysid.assert_awaited_once()
        self.assertIs(outcome.result, solve_result)
        self.assertIs(outcome.telemetry_quality, quality)
        self.assertTrue(outcome.stage_saved)

    async def test_required_runtime_extension_enables_isaac_newton(self) -> None:
        """Enable Isaac-hosted Newton before a headless preflight uses it."""
        from isaacsim.robot_setup.sysid import runtime

        # Keyed by extension name rather than call order so an extra probe does
        # not exhaust a positional `side_effect` list.
        manager = Mock()
        enabled_extensions: set[str] = set()

        def _is_enabled(name: str) -> bool:
            return name in enabled_extensions

        def _enable(name: str, value: bool) -> bool:
            enabled_extensions.add(name) if value else enabled_extensions.discard(name)
            return True

        manager.is_extension_enabled.side_effect = _is_enabled
        manager.set_extension_enabled_immediate.side_effect = _enable
        app = SimpleNamespace(get_extension_manager=Mock(return_value=manager))
        simulation = SimpleNamespace(
            engine="isaac_sim",
            physics_backend="newton",
            actuator_runtime="newton_explicit",
            newton=SimpleNamespace(solver="mujoco"),
        )

        with (
            patch.object(runtime, "require_kit_runtime"),
            patch("omni.kit.app.get_app", return_value=app),
        ):
            enabled = runtime.ensure_required_runtime_extensions(simulation)

        self.assertEqual(enabled, ["isaacsim.physics.newton", "isaacsim.physics.newton.tensors"])
        self.assertEqual(manager.set_extension_enabled_immediate.call_count, 2)
        manager.set_extension_enabled_immediate.assert_any_call("isaacsim.physics.newton", True)
        manager.set_extension_enabled_immediate.assert_any_call("isaacsim.physics.newton.tensors", True)

    async def test_required_runtime_extension_ignores_standalone_newton(self) -> None:
        """Keep standalone Newton independent of the Isaac-hosted physics extension."""
        from isaacsim.robot_setup.sysid import runtime

        simulation = SimpleNamespace(
            engine="newton",
            physics_backend="newton",
            actuator_runtime="newton_explicit",
            newton=SimpleNamespace(solver="mujoco"),
        )
        with patch.object(runtime, "require_kit_runtime"):
            enabled = runtime.ensure_required_runtime_extensions(simulation)

        self.assertEqual(enabled, [])

    async def test_async_runtime_activation_waits_for_extension_registration(self) -> None:
        from isaacsim.robot_setup.sysid import runtime

        simulation = object()
        app = SimpleNamespace(next_update_async=AsyncMock())
        with (
            patch.object(runtime, "ensure_required_runtime_extensions", return_value=["optional.ext"]),
            patch("omni.kit.app.get_app", return_value=app),
        ):
            enabled = await runtime.ensure_required_runtime_extensions_async(simulation)

        self.assertEqual(enabled, ["optional.ext"])
        app.next_update_async.assert_awaited_once_with()

    async def test_save_stage_targets_explicit_non_current_stage(self) -> None:
        from isaacsim.core.experimental.utils import stage as stage_utils

        root_layer = SimpleNamespace(
            anonymous=False,
            realPath="C:/runs/robot.usd",
            Save=Mock(return_value=True),
        )
        stage = SimpleNamespace(GetRootLayer=Mock(return_value=root_layer))
        with (
            patch.object(stage_utils, "get_current_stage", return_value=object()),
            patch.object(stage_utils, "save_stage") as save_current,
        ):
            saved = await headless_job._save_stage(stage)

        self.assertTrue(saved)
        root_layer.Save.assert_called_once_with()
        save_current.assert_not_called()


class TestSysIdJobService(omni.kit.test.AsyncTestCase):
    """Verify serialized execution, snapshots, and cancellation."""

    async def tearDown(self) -> None:
        service = getattr(self, "service", None)
        if service is not None:
            await service.shutdown_async()

    async def test_jobs_run_serially_and_publish_json_safe_status(self) -> None:
        active = 0
        max_active = 0

        async def runner(_request, **callbacks):
            nonlocal active, max_active
            active += 1
            max_active = max(max_active, active)
            callbacks["on_prepared"](
                SimpleNamespace(backend_label="test", optimizer=object()),
                None,
                None,
                [],
            )
            callbacks["on_progress"]("Halfway", 0.5)
            callbacks["on_iteration"](SimpleNamespace(iteration=2, cost=1.25))
            await asyncio.sleep(0.01)
            active -= 1
            return object()

        resetter = AsyncMock()
        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=resetter,
            spec_loader=lambda _request: SimpleNamespace(),
        )
        payload = {"run_spec": {}}
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            first = self.service.submit(payload)
            second = self.service.submit(payload)
            await self._wait_for_state(first, "succeeded")
            await self._wait_for_state(second, "succeeded")

        self.assertEqual(max_active, 1)
        self.assertEqual(resetter.await_count, 2)
        snapshot = self.service.status(second)
        self.assertEqual(snapshot["backend"], "test")
        self.assertEqual(snapshot["iteration"], 2)
        self.assertEqual(snapshot["cost"], 1.25)
        self.assertEqual(snapshot["result"], {"ok": True})

    async def test_submit_rejects_work_when_pending_queue_is_full(self) -> None:
        runner_started = asyncio.Event()
        release = asyncio.Event()

        async def runner(_request, **_callbacks):
            runner_started.set()
            await release.wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
            max_pending_jobs=1,
        )
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            first = self.service.submit({"run_spec": {}})
            await runner_started.wait()
            second = self.service.submit({"run_spec": {}})
            with self.assertRaisesRegex(RuntimeError, "queue is full"):
                self.service.submit({"run_spec": {}})
            release.set()
            await self._wait_for_state(first, "succeeded")
            await self._wait_for_state(second, "succeeded")

    async def test_terminal_job_retention_prunes_oldest_snapshot(self) -> None:
        async def runner(_request, **_callbacks):
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
            max_retained_jobs=2,
        )
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            first = self.service.submit({"run_spec": {}})
            await self._wait_for_state(first, "succeeded")
            second = self.service.submit({"run_spec": {}})
            await self._wait_for_state(second, "succeeded")
            third = self.service.submit({"run_spec": {}})
            await self._wait_for_state(third, "succeeded")

        with self.assertRaises(KeyError):
            self.service.status(first)
        self.assertEqual([row["job_id"] for row in self.service.list_jobs()], [second, third])

    async def test_queued_job_can_be_cancelled_without_running(self) -> None:
        release = asyncio.Event()
        runner_calls = 0

        async def runner(_request, **callbacks):
            nonlocal runner_calls
            runner_calls += 1
            callbacks["on_prepared"](
                SimpleNamespace(backend_label="test", optimizer=object()),
                None,
                None,
                [],
            )
            await release.wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            first = self.service.submit({"run_spec": {}})
            second = self.service.submit({"run_spec": {}})
            await self._wait_for_state(first, "running")
            self.assertTrue(self.service.cancel(second))
            release.set()
            await self._wait_for_state(first, "succeeded")

        self.assertEqual(self.service.status(second)["state"], "cancelled")
        self.assertEqual(runner_calls, 1)

    async def test_cancelled_queued_job_immediately_frees_queue_capacity(self) -> None:
        runner_started = asyncio.Event()
        release = asyncio.Event()

        async def runner(_request, **_callbacks):
            runner_started.set()
            await release.wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
            max_pending_jobs=1,
        )
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            first = self.service.submit({"run_spec": {}})
            await runner_started.wait()
            cancelled = self.service.submit({"run_spec": {}})
            self.assertTrue(self.service.cancel(cancelled))
            replacement = self.service.submit({"run_spec": {}})
            release.set()
            await self._wait_for_state(first, "succeeded")
            await self._wait_for_state(replacement, "succeeded")

        self.assertEqual(self.service.status(cancelled)["state"], "cancelled")

    async def test_running_job_forwards_cancellation_to_optimizer(self) -> None:
        cancel_event = asyncio.Event()

        class Optimizer:
            def request_cancel(self) -> None:
                cancel_event.set()

        async def runner(_request, **callbacks):
            callbacks["on_prepared"](
                SimpleNamespace(backend_label="test", optimizer=Optimizer()),
                None,
                None,
                [],
            )
            await cancel_event.wait()
            raise asyncio.CancelledError()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        job_id = self.service.submit({"run_spec": {}})
        await self._wait_for_state(job_id, "running")
        self.assertTrue(self.service.cancel(job_id))
        await self._wait_for_state(job_id, "cancelled")
        self.assertTrue(cancel_event.is_set())

    async def test_cancellation_before_prepare_aborts_before_optimizer_starts(self) -> None:
        runner_started = asyncio.Event()
        allow_prepare = asyncio.Event()

        async def runner(_request, **callbacks):
            runner_started.set()
            await allow_prepare.wait()
            callbacks["on_prepared"](
                SimpleNamespace(backend_label="test", optimizer=object()),
                None,
                None,
                [],
            )
            raise AssertionError("Optimization must not start after cancellation.")

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        job_id = self.service.submit({"run_spec": {}})
        await runner_started.wait()

        self.assertTrue(self.service.cancel(job_id))
        allow_prepare.set()
        await self._wait_for_state(job_id, "cancelled")

    async def test_shutdown_async_drains_cancelled_worker(self) -> None:
        runner_started = asyncio.Event()

        async def runner(_request, **_callbacks):
            runner_started.set()
            await asyncio.Event().wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        job_id = self.service.submit({"run_spec": {}})
        await runner_started.wait()

        await self.service.shutdown_async()

        self.assertEqual(self.service.status(job_id)["state"], "cancelled")
        self.assertIsNone(self.service._worker_task)

    async def test_shutdown_marks_every_queued_job_cancelled(self) -> None:
        runner_started = asyncio.Event()

        async def runner(_request, **_callbacks):
            runner_started.set()
            await asyncio.Event().wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        active = self.service.submit({"run_spec": {}})
        await runner_started.wait()
        queued = [self.service.submit({"run_spec": {}}) for _ in range(2)]

        await self.service.shutdown_async()

        self.assertEqual(self.service.status(active)["state"], "cancelled")
        for job_id in queued:
            snapshot = self.service.status(job_id)
            self.assertEqual(snapshot["state"], "cancelled")
            self.assertIn("shutting down", snapshot["message"])
            self.assertTrue(snapshot["finished_at"])

    async def test_successful_runner_wins_over_late_cancel_request(self) -> None:
        optimizer = SimpleNamespace(request_cancel=Mock())

        async def runner(_request, **callbacks):
            callbacks["on_prepared"](
                SimpleNamespace(backend_label="test", optimizer=optimizer),
                None,
                None,
                [],
            )
            self.assertTrue(self.service.cancel(self.service._active_job_id))
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        with patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}):
            job_id = self.service.submit({"run_spec": {}})
            await self._wait_for_state(job_id, "succeeded")

        snapshot = self.service.status(job_id)
        self.assertTrue(snapshot["cancel_requested"])
        self.assertEqual(snapshot["result"], {"ok": True})
        optimizer.request_cancel.assert_called_once_with()

    async def test_service_rejects_paths_outside_explicit_allowed_roots(self) -> None:
        with tempfile.TemporaryDirectory(prefix="sysid_job_jail_") as tmp_dir:
            root = Path(tmp_dir)
            jail = root / "allowed"
            outside_dir = root / "outside"
            jail.mkdir()
            outside_dir.mkdir()
            outside = outside_dir / "robot.usd"
            self.service = job_service.SysIdJobService(
                runtime_resetter=AsyncMock(),
                spec_loader=lambda _request: SimpleNamespace(),
                allowed_roots=[jail],
            )

            with self.assertRaisesRegex(ValueError, "outside"):
                self.service.submit(
                    {
                        "run_spec": {"stage": {"input_path": str(outside)}},
                    }
                )
            with self.assertRaisesRegex(ValueError, "outside"):
                self.service.submit(
                    {
                        "run_spec": {},
                        "result_json": str(outside.with_suffix(".json")),
                    }
                )

    async def test_file_backed_service_loaders_resolve_paths_from_run_spec_directory(self) -> None:
        """Resolve file-backed service paths from the RunSpec directory and preserve the raw payload."""
        with tempfile.TemporaryDirectory(prefix="sysid_service_spec_paths_") as tmp_dir:
            root = Path(tmp_dir)
            spec_path = root / "run_spec.json"
            payload = {
                "stage": {"input_path": "assets/robot.usd"},
                "telemetry": {
                    "source_path": "data/bag_0.mcap",
                    "mapping_path": "data/topic_map.yaml",
                    "chunk_manifest_path": "data/collection_manifest.json",
                },
                "outputs": {
                    "provenance_path": "results/provenance.json",
                    "validation_animation_path": "results/validation.mp4",
                },
            }
            spec_path.write_text(json.dumps(payload), encoding="utf-8")
            request = job_service.SysIdServiceJobRequest(run_spec_path=str(spec_path))

            public_spec = request.load_spec(allowed_roots=[root])
            worker_spec, raw_payload = job_service._load_service_spec(request, allowed_roots=(root.resolve(),))

            expected_paths = {
                ("stage", "input_path"): root / "assets" / "robot.usd",
                ("telemetry", "source_path"): root / "data" / "bag_0.mcap",
                ("telemetry", "mapping_path"): root / "data" / "topic_map.yaml",
                ("telemetry", "chunk_manifest_path"): root / "data" / "collection_manifest.json",
                ("outputs", "provenance_path"): root / "results" / "provenance.json",
                ("outputs", "validation_animation_path"): root / "results" / "validation.mp4",
            }
            for (owner_name, field_name), expected in expected_paths.items():
                self.assertEqual(Path(getattr(getattr(public_spec, owner_name), field_name)), expected.resolve())
                self.assertEqual(Path(getattr(getattr(worker_spec, owner_name), field_name)), expected.resolve())
            self.assertEqual(raw_payload, payload)

    async def test_service_loader_preserves_non_local_paths_without_allowed_roots(self) -> None:
        """Allow URL-backed RunSpecs without granting local filesystem access."""
        payload = {
            "stage": {"input_path": "anon:sysid-stage"},
            "telemetry": {
                "source_path": "omniverse://server/Samples/SystemIdentification/bag_0.mcap",
                "mapping_path": "omniverse://server/Samples/SystemIdentification/topic_map.yaml",
                "chunk_manifest_path": "omniverse://server/Samples/SystemIdentification/manifest.json",
            },
            "outputs": {
                "provenance_path": "omniverse://server/Results/provenance.json",
                "validation_animation_path": "omniverse://server/Results/validation.mp4",
            },
        }
        request = job_service.SysIdServiceJobRequest(run_spec=payload)

        loaded = request.load_spec()

        self.assertEqual(loaded.stage.input_path, payload["stage"]["input_path"])
        for field_name, expected in payload["telemetry"].items():
            self.assertEqual(getattr(loaded.telemetry, field_name), expected)
        for field_name, expected in payload["outputs"].items():
            self.assertEqual(getattr(loaded.outputs, field_name), expected)

    async def test_service_loader_jails_local_file_urls(self) -> None:
        """Reject file URLs that resolve outside the configured filesystem roots."""
        with tempfile.TemporaryDirectory(prefix="sysid_job_file_url_jail_") as tmp_dir:
            root = Path(tmp_dir)
            jail = root / "allowed"
            inside = jail / "robot.usd"
            outside = root / "outside" / "robot.usd"
            jail.mkdir()
            outside.parent.mkdir()
            outside_request = job_service.SysIdServiceJobRequest(
                run_spec={"stage": {"input_path": outside.as_uri()}},
            )

            with self.assertRaisesRegex(ValueError, "outside"):
                outside_request.load_spec(allowed_roots=[jail])
            with self.assertRaisesRegex(ValueError, "outside"):
                job_service._validate_service_request_paths(outside_request, (jail.resolve(),))

            inside_request = job_service.SysIdServiceJobRequest(
                run_spec={"stage": {"input_path": inside.as_uri()}},
            )
            job_service._validate_service_request_paths(inside_request, (jail.resolve(),))
            loaded = inside_request.load_spec(allowed_roots=[jail])
            self.assertEqual(Path(loaded.stage.input_path), inside.resolve())

    async def test_service_rejects_symlink_escape_from_allowed_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="sysid_job_symlink_jail_") as tmp_dir:
            root = Path(tmp_dir)
            jail = root / "allowed"
            outside = root / "outside"
            jail.mkdir()
            outside.mkdir()
            link = jail / "escape"
            try:
                link.symlink_to(outside, target_is_directory=True)
            except OSError as exc:
                self.skipTest(f"Directory symlinks are unavailable: {exc}")
            self.service = job_service.SysIdJobService(
                runtime_resetter=AsyncMock(),
                spec_loader=lambda _request: SimpleNamespace(),
                allowed_roots=[jail],
            )

            with self.assertRaisesRegex(ValueError, "outside"):
                self.service.submit(
                    {
                        "run_spec": {
                            "stage": {
                                "input_path": str(link / "robot.usd"),
                            }
                        },
                    }
                )

    async def test_path_backed_request_is_disabled_without_allowed_roots(self) -> None:
        self.service = job_service.SysIdJobService(
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )

        with self.assertRaisesRegex(ValueError, "no allowed_roots"):
            self.service.submit({"run_spec": {}, "result_json": "result.json"})

    async def test_runtime_reset_failure_is_published_as_failed_result(self) -> None:
        async def runner(_request, **_callbacks):
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(side_effect=RuntimeError("reset boom")),
            spec_loader=lambda _request: SimpleNamespace(),
            allowed_roots=[Path.cwd()],
        )
        with (
            patch.object(
                job_service,
                "summarize_headless_sysid_job",
                return_value={"ok": True},
            ),
            patch.object(job_service, "_write_json") as write_json,
        ):
            job_id = self.service.submit({"run_spec": {}, "result_json": "result.json"})
            await self._wait_for_state(job_id, "failed")

        snapshot = self.service.status(job_id)
        self.assertEqual(snapshot["message"], "Runtime reset failed")
        self.assertEqual(snapshot["error"]["message"], "Runtime reset failed: reset boom")
        self.assertFalse(snapshot["result"]["ok"])
        self.assertEqual(snapshot["result"]["job_state"], "failed")
        self.assertEqual(write_json.call_args.args[1], snapshot["result"])

    async def test_runtime_reset_stops_timeline_before_replacing_stage(self) -> None:
        from isaacsim.core.experimental.utils import app as app_utils
        from isaacsim.core.experimental.utils import stage as stage_utils

        events: list[str] = []

        async def update(*, steps):
            events.append(f"update:{steps}")

        async def create(*, template):
            events.append(f"create:{template}")

        with (
            patch.object(app_utils, "is_playing", return_value=True),
            patch.object(app_utils, "stop", side_effect=lambda: events.append("stop")),
            patch.object(app_utils, "update_app_async", side_effect=update),
            patch.object(stage_utils, "create_new_stage_async", side_effect=create),
        ):
            await job_service._reset_runtime_stage()

        self.assertEqual(events, ["stop", "update:2", "create:empty", "update:5"])

    async def test_runner_failure_is_published_to_requested_result_json(self) -> None:
        """Write a terminal result artifact even when no solve outcome exists."""

        async def runner(_request, **_callbacks):
            raise RuntimeError("runner boom")

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
            allowed_roots=[Path.cwd()],
        )
        with patch.object(job_service, "_write_json") as write_json:
            job_id = self.service.submit({"run_spec": {}, "result_json": "result.json"})
            await self._wait_for_state(job_id, "failed")

        result = self.service.status(job_id)["result"]
        self.assertEqual(result["job_state"], "failed")
        self.assertEqual(result["message"], "runner boom")
        self.assertEqual(result["error"]["message"], "runner boom")
        self.assertEqual(write_json.call_args.args[1], result)

    async def test_queued_cancellation_is_published_to_requested_result_json(self) -> None:
        """Write the requested result artifact when cancellation precedes execution."""
        release = asyncio.Event()

        async def runner(_request, **_callbacks):
            await release.wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
            allowed_roots=[Path.cwd()],
        )
        with (
            patch.object(job_service, "_write_json") as write_json,
            patch.object(job_service, "summarize_headless_sysid_job", return_value={"ok": True}),
        ):
            first = self.service.submit({"run_spec": {}})
            second = self.service.submit({"run_spec": {}, "result_json": "cancelled.json"})
            await self._wait_for_state(first, "running")
            self.assertTrue(self.service.cancel(second))
            release.set()
            await self._wait_for_state(first, "succeeded")

        result = self.service.status(second)["result"]
        self.assertEqual(result["job_state"], "cancelled")
        self.assertEqual(result["message"], "Cancelled before execution")
        self.assertEqual(write_json.call_args.args[1], result)

    async def test_sync_singleton_shutdown_blocks_reentry_until_worker_drains(
        self,
    ) -> None:
        runner_started = asyncio.Event()

        async def runner(_request, **_callbacks):
            runner_started.set()
            await asyncio.Event().wait()
            return object()

        self.service = job_service.SysIdJobService(
            runner=runner,
            runtime_resetter=AsyncMock(),
            spec_loader=lambda _request: SimpleNamespace(),
        )
        self.service.submit({"run_spec": {}})
        await runner_started.wait()
        queued = self.service.submit({"run_spec": {}})

        with (
            patch.object(job_service, "_SERVICE", self.service),
            patch.object(job_service, "_RETIRED_SERVICE", None),
            patch.object(job_service, "_SERVICE_SHUTDOWN_TASK", None),
        ):
            job_service.shutdown_sysid_job_service()
            self.assertEqual(job_service.get_sysid_job_status(queued)["state"], "cancelled")
            with self.assertRaisesRegex(RuntimeError, "still shutting down"):
                job_service.get_sysid_job_service()
            await job_service.shutdown_sysid_job_service_async()
            replacement = job_service.get_sysid_job_service()
            self.assertIsNot(replacement, self.service)
            await replacement.shutdown_async()

        self.service = None

    async def _wait_for_state(self, job_id: str, expected: str) -> None:
        """Yield until a job reaches the expected state.

        Args:
            job_id: Value supplied for ``job_id``.
            expected: Value supplied for ``expected``.
        """
        for _ in range(200):
            if self.service.status(job_id)["state"] == expected:
                return
            await asyncio.sleep(0.001)
        self.fail(f"Job {job_id} did not reach state {expected}: {self.service.status(job_id)}")


class TestHeadlessToolHelpers(omni.kit.test.AsyncTestCase):
    """Verify reliability helpers used by the standalone headless tool."""

    async def test_simsim_generation_uses_controller_bridge_and_finally_cleanup(self) -> None:
        """Keep synthetic rollouts on the production bridge-selection contract."""
        tool_path = Path(__file__).resolve().parents[4] / "tools" / "headless_sysid_simsim.py"
        tree = ast.parse(tool_path.read_text(encoding="utf-8"), filename=str(tool_path))
        functions = {node.name: node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
        generate = functions["_generate"]
        run = functions["_run"]

        self.assertTrue(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "_create_bridge"
                for node in ast.walk(generate)
            )
        )
        self.assertFalse(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id in {"NewtonDifferentiableSysIdBridge", "NewtonSysIdBridge"}
                for node in ast.walk(generate)
            )
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Try)
                and any(
                    isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Name)
                    and child.func.id == "_cleanup_generation_bridges"
                    for statement in node.finalbody
                    for child in ast.walk(statement)
                )
                for node in ast.walk(run)
            )
        )

    async def test_apply_theta_provenance_includes_prepared_runtime_context(self) -> None:
        """Keep apply-theta provenance aligned with the main run session."""
        tool_path = Path(__file__).resolve().parents[4] / "tools" / "headless_sysid_apply_theta_to_stage.py"
        tree = ast.parse(tool_path.read_text(encoding="utf-8"), filename=str(tool_path))
        calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "build_provenance_record"
        ]

        self.assertEqual(len(calls), 1)
        keywords = {keyword.arg: keyword.value for keyword in calls[0].keywords}
        self.assertEqual(
            ast.unparse(keywords["simulation_engine"]),
            "str(getattr(prepared, 'simulation_engine', ''))",
        )
        self.assertEqual(
            ast.unparse(keywords["newton_config"]),
            "getattr(prepared, 'newton_config', None)",
        )

    async def test_load_spec_helpers_expose_single_spec_and_raw_payload_contracts(self) -> None:
        """Keep sibling CLI callers on the single-object loader contract."""
        from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

        tool_path = Path(__file__).resolve().parents[4] / "tools" / "headless_sysid_solve.py"
        spec = importlib.util.spec_from_file_location("_sysid_headless_solve_loader_test", tool_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory(prefix="sysid_load_spec_") as tmp_dir:
            path = Path(tmp_dir) / "run_spec.json"
            path.write_text(
                """{
  "stage": {"input_path": "assets/robot.usd"},
  "telemetry": {
    "source_path": "data/bag_0.mcap",
    "mapping_path": "data/topic_map.yaml",
    "chunk_manifest_path": "data/collection_manifest.json"
  },
  "outputs": {
    "provenance_path": "results/provenance.json",
    "validation_animation_path": "results/validation.mp4"
  }
}\n""",
                encoding="utf-8",
            )

            loaded = module._load_spec(path)
            loaded_with_payload, payload = module._load_spec_and_payload(path)

            self.assertIsInstance(loaded, SysIdRunSpec)
            self.assertIsInstance(loaded_with_payload, SysIdRunSpec)
            root = Path(tmp_dir)
            self.assertEqual(Path(loaded.stage.input_path), (root / "assets" / "robot.usd").resolve())
            self.assertEqual(Path(loaded.telemetry.source_path), (root / "data" / "bag_0.mcap").resolve())
            self.assertEqual(Path(loaded.telemetry.mapping_path), (root / "data" / "topic_map.yaml").resolve())
            self.assertEqual(
                Path(loaded.telemetry.chunk_manifest_path),
                (root / "data" / "collection_manifest.json").resolve(),
            )
            self.assertEqual(Path(loaded.outputs.provenance_path), (root / "results" / "provenance.json").resolve())
            self.assertEqual(
                Path(loaded.outputs.validation_animation_path),
                (root / "results" / "validation.mp4").resolve(),
            )
            self.assertEqual(loaded_with_payload.to_dict(), loaded.to_dict())
            self.assertEqual(payload["stage"]["input_path"], "assets/robot.usd")
            self.assertEqual(payload["telemetry"]["source_path"], "data/bag_0.mcap")

    async def test_load_spec_preserves_urls_and_absolute_paths(self) -> None:
        """Do not rebase absolute paths or non-local asset URLs."""
        tool_path = Path(__file__).resolve().parents[4] / "tools" / "headless_sysid_solve.py"
        spec = importlib.util.spec_from_file_location("_sysid_headless_solve_absolute_path_test", tool_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory(prefix="sysid_absolute_spec_") as tmp_dir:
            root = Path(tmp_dir)
            absolute_stage = root / "robot.usd"
            path = root / "run_spec.json"
            url_paths = {
                "source_path": "omniverse://server/Samples/SystemIdentification/bag_0.mcap",
                "mapping_path": "omniverse://server/Samples/SystemIdentification/topic_map.yaml",
                "chunk_manifest_path": "omniverse://server/Samples/SystemIdentification/manifest.json",
            }
            output_urls = {
                "provenance_path": "omniverse://server/Results/provenance.json",
                "validation_animation_path": "omniverse://server/Results/validation.mp4",
            }
            path.write_text(
                json.dumps(
                    {
                        "stage": {"input_path": str(absolute_stage)},
                        "telemetry": url_paths,
                        "outputs": output_urls,
                    }
                ),
                encoding="utf-8",
            )

            loaded = module._load_spec(path)

            self.assertEqual(Path(loaded.stage.input_path), absolute_stage.resolve())
            for field_name, expected in url_paths.items():
                self.assertEqual(getattr(loaded.telemetry, field_name), expected)
            for field_name, expected in output_urls.items():
                self.assertEqual(getattr(loaded.outputs, field_name), expected)

    async def test_json_write_preserves_previous_file_when_serialization_fails(self) -> None:
        """Keep the last complete checkpoint if a replacement write fails."""
        tool_path = Path(__file__).resolve().parents[4] / "tools" / "headless_sysid_solve.py"
        spec = importlib.util.spec_from_file_location("_sysid_headless_solve_test", tool_path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory(prefix="sysid_atomic_json_") as tmp_dir:
            destination = Path(tmp_dir) / "checkpoint.json"
            destination.write_text('{"state": "complete"}\n', encoding="utf-8")

            def fail_during_dump(_payload, stream, **_kwargs) -> None:
                stream.write('{"state": "partial"')
                raise RuntimeError("simulated serialization failure")

            with (
                patch.object(module.json, "dump", side_effect=fail_during_dump),
                self.assertRaisesRegex(RuntimeError, "serialization failure"),
            ):
                module._write_json(destination, {"state": "replacement"})

            self.assertEqual(destination.read_text(encoding="utf-8"), '{"state": "complete"}\n')
            self.assertEqual(list(destination.parent.glob(f".{destination.name}.*.tmp")), [])
