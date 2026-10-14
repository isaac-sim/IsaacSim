# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exercise staging repository boundaries and offline dry runs."""

import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[4]
POLICY_PATH = ROOT / "tools/github/ci/internal_staging_policy.py"
SPEC = importlib.util.spec_from_file_location("staging_policy_public_test", POLICY_PATH)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"Unable to load policy module: {POLICY_PATH}")
POLICY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = POLICY
SPEC.loader.exec_module(POLICY)


def checks() -> list[dict[str, str]]:
    """Return successful required checks."""
    return [
        {"name": name, "status": "completed", "conclusion": "success"}
        for name in (*POLICY.REQUIRED_GITHUB_CHECKS, *POLICY.EXPECTED_GITHUB_CI_CHECKS)
    ]


class PolicyBoundaryTests(unittest.TestCase):
    """Check waiver boundaries using synthetic evidence only."""

    def test_waivers_block_on_public_forks_and_missing_repository(self) -> None:
        for repository in ("isaac-sim/IsaacSim", "example/IsaacSim-Internal", ""):
            with self.subTest(repository=repository):
                result = POLICY.evaluate_github_checks(checks(), POLICY.ALLOWLISTED_FORCE, repository)
                self.assertEqual(result.result, "block")
                self.assertFalse(result.waived)

    def test_clean_checks_pass_without_waivers(self) -> None:
        self.assertTrue(POLICY.evaluate_github_checks(checks(), "disabled", "isaac-sim/IsaacSim").allowed)

    def test_cli_dry_evaluation_has_no_network(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory) / "checks.json"
            evidence.write_text(json.dumps(checks()), encoding="utf-8")
            args = [str(POLICY_PATH), "github", str(evidence), "--mode", "allowlisted-force"]
            with mock.patch.object(sys, "argv", args), contextlib.redirect_stdout(io.StringIO()) as output:
                result = POLICY.main()
            self.assertEqual(result, 1)
            self.assertEqual(json.loads(output.getvalue())["result"], "block")

    def test_exported_helper_has_no_private_package_policy(self) -> None:
        source = POLICY_PATH.read_text(encoding="utf-8")
        for text in ("SIMREADY", "urm", "ct-omniverse", "evaluate_gitlab_findings"):
            self.assertNotIn(text, source)


class WorkflowBoundaryTests(unittest.TestCase):
    """Check workflow guards and execute the no-write merge path."""

    def setUp(self) -> None:
        self.workflow = yaml.safe_load((ROOT / ".github/workflows/internal-staging-auto-merge.yml").read_text())
        self.job = self.workflow["jobs"]["merge"]
        self.steps = {step["name"]: step for step in self.job["steps"]}

    def test_all_event_routes_require_exact_repository(self) -> None:
        condition = " ".join(self.job["if"].split())
        self.assertTrue(condition.startswith("github.repository == 'isaac-sim/IsaacSim-Internal' && (("))
        self.assertTrue(condition.endswith("github.event_name == 'workflow_dispatch')"))
        policy = self.steps["Apply narrow internal merge policy"]["run"]
        self.assertIn('--repository "$REPO"', policy)

    def test_dry_run_guards_merge_and_notification(self) -> None:
        self.assertIn("!inputs.dry_run", self.steps["Squash merge exact approved head"]["if"])
        self.assertIn("!inputs.dry_run", self.steps["Report terminal staging status to Slack"]["if"])

    def test_merge_shell_dry_run_never_invokes_gh(self) -> None:
        script = self.steps["Squash merge exact approved head"]["run"]
        # Fail immediately if any GitHub command is attempted.
        script = "gh() { echo UNEXPECTED_GITHUB_WRITE >&2; return 99; }; export -f gh\n" + script
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                ["bash", "-c", script],
                cwd=directory,
                env={"PATH": os.defpath, "DRY_RUN": "true"},
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Dry run: merge skipped.", result.stdout)
        self.assertNotIn("UNEXPECTED_GITHUB_WRITE", result.stderr)


if __name__ == "__main__":
    unittest.main()
