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

"""Test Actor SDG trust acknowledgement without launching Kit or loading assets."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch


def load_module(name: str, path: Path) -> ModuleType:
    """Load the local module under test without running its command-line entry point."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPO_ROOT = Path(__file__).resolve().parents[4]
ACTOR = load_module("actor_sdg", Path(__file__).resolve().parents[1] / "actor_sdg.py")
BATCH = load_module("batch_actor_sdg", REPO_ROOT / "skills/actor-sdg-sweep-config/scripts/batch_actor_sdg.py")


class TestScriptConsent(unittest.TestCase):
    """Verify the command-line and programmatic trust boundaries."""

    def test_cli_requires_consent(self) -> None:
        """Reject missing consent, abbreviated consent, and low-level guard overrides."""
        for extra in ([], ["--allow-trusted"], ["--/app/omni.graph.scriptnode/enable_opt_in=false"]):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    ACTOR.get_args(["-c", "config.yaml", *extra])
                self.assertEqual(raised.exception.code, 2)

    def test_cli_accepts_explicit_consent(self) -> None:
        """Accept the full trust flag while retaining support for other Kit arguments."""
        args = ACTOR.get_args(["-c", "config.yaml", "--allow-trusted-scripts", "--/log/level=warn"])
        self.assertTrue(args.allow_trusted_scripts)
        self.assertEqual(args.config_file, "config.yaml")

    def test_main_rejects_before_application_import(self) -> None:
        """Reject a normal invocation before importing or constructing SimulationApp."""
        with patch.object(sys, "argv", ["actor_sdg.py", "-c", "config.yaml"]):
            with patch.dict(sys.modules, {"isaacsim": None}), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    ACTOR.main()
        self.assertEqual(raised.exception.code, 2)

    def test_constructor_requires_consent(self) -> None:
        """Reject programmatic calls that omit the acknowledgement."""
        with self.assertRaisesRegex(ValueError, "trusted"):
            ACTOR.ActorSDG(Mock(), "config.yaml")

    def test_batch_rejects_before_discovery(self) -> None:
        """Reject execution before reading variants or launching a child process."""
        with patch.object(BATCH, "find_actor_sdg") as discover, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                BATCH.main(["--variants", "variants", "--python-sh", "python.sh"])
        self.assertEqual(raised.exception.code, 2)
        discover.assert_not_called()

    def test_batch_dry_run_needs_no_consent(self) -> None:
        """Permit previewing variants without executing any scripts."""
        variant = {"variant_id": 1, "config_path": Path(__file__)}
        with patch.object(BATCH, "discover_variants", return_value=[variant]):
            with patch.object(BATCH, "run_variant") as run, contextlib.redirect_stdout(io.StringIO()):
                result = BATCH.main(
                    [
                        "--variants",
                        str(Path(__file__).parent),
                        "--python-sh",
                        __file__,
                        "--actor-sdg",
                        str(Path(ACTOR.__file__)),
                        "--dry-run",
                    ]
                )
        self.assertEqual(result, 0)
        run.assert_not_called()

    def test_batch_programmatic_call_requires_consent(self) -> None:
        """Reject a batch helper call before writing logs or starting a process."""
        with patch.object(BATCH.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "trust"):
                BATCH.run_variant(Path("python.sh"), Path("actor_sdg.py"), {}, Path("logs"), None)
        run.assert_not_called()

    def test_batch_forwards_explicit_consent(self) -> None:
        """Pass the acknowledgement to the child only for an explicitly trusted run."""
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(BATCH.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    BATCH.run_variant(
                        Path("python.sh"),
                        Path("actor_sdg.py"),
                        {"variant_id": 1, "config_path": Path("config.yaml")},
                        Path(directory),
                        None,
                        allow_trusted_scripts=True,
                    )
        self.assertEqual(run.call_args.args[0][-1], "--allow-trusted-scripts")


class TestTrustedRun(unittest.IsolatedAsyncioTestCase):
    """Verify that trusted runs retain the guard and opt in before loading configuration."""

    async def test_trust_settings_precede_configuration_load(self) -> None:
        """Keep required scripting available after explicit consent."""
        settings: dict[str, object] = {}
        carb = Mock()
        carb.settings.get_settings.return_value.set.side_effect = settings.__setitem__
        omni = Mock()
        ira = Mock()

        def load_config(path: str) -> bool:
            self.assertEqual(path, "config.yaml")
            self.assertIs(settings["/app/omni.graph.scriptnode/enable_opt_in"], True)
            self.assertIs(settings["/app/omni.graph.scriptnode/opt_in"], True)
            self.assertIs(settings["/app/scripting/ignoreWarningDialog"], True)
            return True

        ira.load_config_file.side_effect = load_config
        ira.setup_simulation = AsyncMock()
        ira.start_data_generation_async = AsyncMock()
        modules = {
            "carb": carb,
            "omni": omni,
            "omni.replicator": omni.replicator,
            "omni.replicator.core": omni.replicator.core,
            "isaacsim.replicator.agent.core": SimpleNamespace(api=ira),
        }
        sim_app = Mock()
        sim_app.app.next_update_async = AsyncMock()
        actor = ACTOR.ActorSDG(sim_app, "config.yaml", allow_trusted_scripts=True)
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(await actor.run())
        ira.setup_simulation.assert_awaited_once()
        ira.start_data_generation_async.assert_awaited_once()


if __name__ == "__main__":
    unittest.main()
