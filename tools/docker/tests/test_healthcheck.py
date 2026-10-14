# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exercise the actual Docker and Compose health commands with isolated logs."""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path

DOCKER_ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = "/isaac-sim/.nvidia-omniverse/logs/Kit"


class HealthcheckTests(unittest.TestCase):
    """Require completed startup for both physics backends and deployment modes."""

    def setUp(self) -> None:
        """Extract production commands and redirect only their log directory."""
        self.temporary_directory = tempfile.TemporaryDirectory(prefix="stream health ")
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name) / "Kit"
        docker_line = next(
            line.strip()
            for line in (DOCKER_ROOT / "Dockerfile").read_text().splitlines()
            if line.strip().startswith("CMD bash -c ")
        )
        compose_line = next(
            line.strip()
            for line in (DOCKER_ROOT / "docker-compose.yml").read_text().splitlines()
            if line.strip().startswith('test: ["CMD-SHELL",')
        )
        self.commands = {
            "docker": shlex.split(docker_line)[3],
            "compose": json.loads(compose_line.removeprefix("test: "))[1].replace("$$", "$"),
        }

    def _write_log(self, text: str, app: str = "Isaac-Sim Streaming (Newton)", timestamp: int = 100) -> Path:
        path = self.root / app / "6.2" / f"kit_{timestamp}.log"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        os.utime(path, (timestamp, timestamp))
        return path

    def _assert_health(self, expected: int) -> None:
        for name, command in self.commands.items():
            with self.subTest(deployment=name):
                # Quote the replacement so paths with spaces work in both shell forms.
                command = command.replace(f'"{LOG_ROOT}"', shlex.quote(str(self.root)))
                command = command.replace(LOG_ROOT, shlex.quote(str(self.root)))
                result = subprocess.run(
                    ["/bin/bash" if name == "docker" else "/bin/sh", "-c", command],
                    capture_output=True,
                    text=True,
                    timeout=10,
                    env={"PATH": os.defpath},
                )
                self.assertEqual(result.returncode, expected, result.stderr)

    def test_missing_directory_and_logs_are_unhealthy(self) -> None:
        """A health check cannot pass before the application creates a log."""
        self._assert_health(1)
        self.root.mkdir()
        self._assert_health(1)

    def test_startup_delay_and_listening_are_not_ready(self) -> None:
        """Neither a delayed readiness event nor a listener proves viewport readiness."""
        self._write_log(
            "[Info] [omni.kit.app.plugin] kEventAppReady event was delayed by: IsaacSim\n"
            "[Info] Started primary stream server on signal port 49100 and stream port 47998\n"
        )
        self._assert_health(1)

    def test_both_backends_become_healthy_when_loaded(self) -> None:
        """Accept the real loaded message for PhysX and Newton, including log prefixes."""
        for title in ("Isaac Sim Full Streaming", "Isaac Sim Streaming (Newton Physics)"):
            with self.subTest(title=title):
                self._write_log(f"[Info] [omni.kit.app.plugin] [67.931s] {title} App is loaded.\n")
                self._assert_health(0)

    def test_incomplete_loaded_message_is_unhealthy(self) -> None:
        """Do not accept a partial line or a message describing a pending load."""
        self._write_log("Isaac Sim Streaming (Newton Physics) App is loaded\n App is loaded. pending\n")
        self._assert_health(1)

    def test_newest_log_must_be_ready(self) -> None:
        """An older successful log cannot mask a newer startup that is still loading."""
        self._write_log("Isaac Sim Full Streaming App is loaded.\n", app="Isaac-Sim Full", timestamp=100)
        latest = self._write_log("kEventAppReady event was delayed by: IsaacSim\n", timestamp=200)
        self._assert_health(1)
        latest.write_text("Isaac Sim Streaming (Newton Physics) App is loaded.\n")
        self._assert_health(0)


if __name__ == "__main__":
    unittest.main()
