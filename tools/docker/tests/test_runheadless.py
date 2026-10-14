# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exercise the container entrypoint with isolated license and launcher stubs."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ENTRYPOINT = Path(__file__).resolve().parents[1] / "data/runheadless.sh"


class RunHeadlessTests(unittest.TestCase):
    """Check backend selection, argument forwarding, and failure propagation."""

    def setUp(self) -> None:
        """Replace only the installation prefix with an isolated fixture tree."""
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.script = ENTRYPOINT.read_text().replace("/isaac-sim/", f"{self.root}/")
        for name in ("license.sh", "privacy.sh", "isaac-sim.streaming.sh", "isaac-sim.newton.streaming.sh"):
            self._write_stub(name)

    def _write_stub(self, name: str, exit_code: int = 0) -> None:
        path = self.root / name
        path.write_text(f"#!/bin/sh\nprintf '%s\\n' '{name}'\nprintf '<%s>\\n' \"$@\"\nexit {exit_code}\n")
        path.chmod(0o755)

    def _run(self, *args: str, **variables: str) -> subprocess.CompletedProcess[str]:
        # Keep inherited settings from influencing the entrypoint's defaults.
        env = {"PATH": os.defpath, **variables}
        return subprocess.run(
            ["/bin/sh", "-c", self.script, "runheadless.sh", *args],
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
        )

    def test_default_empty_and_explicit_physx(self) -> None:
        """Unset and empty selectors retain the existing PhysX launcher."""
        for variables in ({}, {"ISAACSIM_PHYSICS_ENGINE": ""}, {"ISAACSIM_PHYSICS_ENGINE": "physx"}):
            with self.subTest(variables=variables):
                result = self._run(**variables)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("isaac-sim.streaming.sh", result.stdout)
                self.assertNotIn("isaac-sim.newton.streaming.sh", result.stdout)

    def test_newton_preserves_checks_and_cli_overrides(self) -> None:
        """Run both gates and preserve arguments, including their override order."""
        override = "--/exts/omni.kit.livestream.app/primaryStream/signalPort=49200"
        result = self._run(
            override,
            "--/app/window/title=Test with spaces",
            ISAACSIM_PHYSICS_ENGINE="newton",
            ISAACSIM_HOST="192.0.2.10",
            ISAACSIM_SIGNAL_PORT="49100",
            ISAACSIM_STREAM_PORT="47998",
            OMNI_SERVER="omniverse://example.invalid/assets",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        self.assertLess(lines.index("license.sh"), lines.index("privacy.sh"))
        self.assertLess(lines.index("privacy.sh"), lines.index("isaac-sim.newton.streaming.sh"))
        self.assertIn(f"<--merge-config={self.root}/config/open_endpoint.toml>", lines)
        self.assertIn("<--/persistent/isaac/asset_root/default=omniverse://example.invalid/assets>", lines)
        self.assertIn("<--/exts/omni.kit.livestream.app/primaryStream/publicIp=192.0.2.10>", lines)
        self.assertIn("<--/exts/omni.kit.livestream.app/primaryStream/streamPort=47998>", lines)
        self.assertLess(
            lines.index("<--/exts/omni.kit.livestream.app/primaryStream/signalPort=49100>"),
            lines.index(f"<{override}>"),
        )
        self.assertIn("<--/app/window/title=Test with spaces>", lines)

    def test_invalid_selector_fails_before_launch(self) -> None:
        """Reject unsupported names without running license or launcher scripts."""
        for engine in ("Newton", "unknown", "../isaac-sim.streaming.sh", "newton; exit 0"):
            with self.subTest(engine=engine):
                result = self._run(ISAACSIM_PHYSICS_ENGINE=engine)
                self.assertEqual(result.returncode, 2)
                self.assertIn("Expected physx or newton", result.stderr)
                self.assertEqual(result.stdout, "")

    def test_unavailable_launcher_fails_without_fallback(self) -> None:
        """A missing or non-executable Newton launcher cannot silently run PhysX."""
        launcher = self.root / "isaac-sim.newton.streaming.sh"
        launcher.chmod(0o644)
        result = self._run(ISAACSIM_PHYSICS_ENGINE="newton")
        self.assertEqual(result.returncode, 1)
        self.assertIn("missing or not executable", result.stderr)
        self.assertEqual(result.stdout, "")
        launcher.unlink()
        result = self._run(ISAACSIM_PHYSICS_ENGINE="newton")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")

    def test_gate_failure_prevents_launch(self) -> None:
        """Both backend choices respect license and privacy failures."""
        for engine in ("physx", "newton"):
            for gate in ("license.sh", "privacy.sh"):
                with self.subTest(engine=engine, gate=gate):
                    self._write_stub(gate, 7)
                    result = self._run(ISAACSIM_PHYSICS_ENGINE=engine)
                    self.assertEqual(result.returncode, 7)
                    self.assertNotIn("isaac-sim.", result.stdout)
                    self._write_stub(gate)

    def test_launcher_exit_status_is_preserved(self) -> None:
        """Report application failure to the container supervisor."""
        self._write_stub("isaac-sim.newton.streaming.sh", 17)
        self.assertEqual(self._run(ISAACSIM_PHYSICS_ENGINE="newton").returncode, 17)


if __name__ == "__main__":
    unittest.main()
