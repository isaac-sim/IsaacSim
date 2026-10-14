# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for terminal GitHub Internal staging reporting."""

import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

MODULE_PATH = Path(__file__).parents[1] / "report_internal_staging_status.py"
SPEC = importlib.util.spec_from_file_location("report_internal_staging_status", MODULE_PATH)
assert SPEC and SPEC.loader
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class LoadDecisionTests(unittest.TestCase):
    def test_no_post_never_calls_webhook(self) -> None:
        args = [
            "report",
            "--pr-number",
            "1",
            "--pr-url",
            "https://example.invalid/pr/1",
            "--head-sha",
            "a" * 40,
            "--merge-outcome",
            "skipped",
            "--run-url",
            "https://example.invalid/run/1",
            "--no-post",
        ]
        with mock.patch.object(sys, "argv", args), mock.patch.object(REPORT, "post") as post:
            with mock.patch.object(REPORT, "load_decision", return_value=None):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(REPORT.main(), 0)
            post.assert_not_called()

    def test_malformed_json_falls_back_to_step_outcomes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "decision.json"
            path.write_text("{", encoding="utf-8")
            self.assertIsNone(REPORT.load_decision(path))


def describe(
    *,
    merge_outcome: str,
    policy_outcome: str = "success",
    decision: dict[str, object] | None = None,
) -> str:
    return REPORT.describe(
        pr_number="118",
        pr_url="https://example.invalid/pr/118",
        head_sha="a" * 40,
        merge_outcome=merge_outcome,
        policy_outcome=policy_outcome,
        decision=decision,
        run_url="https://example.invalid/run/1",
    )


class DescribeTests(unittest.TestCase):
    def test_success_reports_merge_and_internal_waivers(self) -> None:
        text = describe(
            merge_outcome="success",
            decision={"waived": ["Format Check", "Build"]},
        )
        self.assertIn("PR #118", text)
        self.assertIn("(merged)", text)
        self.assertIn("Format Check", text)
        self.assertIn("not public-promotion evidence", text)

    def test_success_without_waivers_has_no_exception_warning(self) -> None:
        text = describe(merge_outcome="success", decision={"waived": []})
        self.assertIn("(merged)", text)
        self.assertNotIn("Internal-only checks waived", text)

    def test_policy_block_reports_reason_and_blocking_checks(self) -> None:
        text = describe(
            merge_outcome="skipped",
            policy_outcome="failure",
            decision={
                "result": "block",
                "reason": "required checks failed",
                "blocking": ["DCO"],
            },
        )
        self.assertIn("blocked: required checks failed: DCO", text)
        self.assertIn("Workflow run", text)

    def test_policy_block_without_details_is_not_reported_as_merge_failure(self) -> None:
        text = describe(
            merge_outcome="failure",
            policy_outcome="failure",
            decision={"result": "block"},
        )
        self.assertIn("merge policy blocked the PR", text)
        self.assertNotIn("GitHub merge step failed", text)

    def test_pre_policy_failure_remains_visible(self) -> None:
        text = describe(
            merge_outcome="skipped",
            policy_outcome="skipped",
        )
        self.assertIn("workflow stopped before merge (skipped)", text)

    def test_merge_failure_does_not_report_allow_reason_as_the_blocker(self) -> None:
        text = describe(
            merge_outcome="failure",
            decision={
                "result": "allow",
                "reason": "required checks passed",
            },
        )
        self.assertIn("GitHub merge step failed after policy evaluation", text)
        self.assertNotIn("required checks passed", text)


if __name__ == "__main__":
    unittest.main()
