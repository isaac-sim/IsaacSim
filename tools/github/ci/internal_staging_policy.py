#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Evaluate the narrow, internal-only staging merge policy.

The GitHub checks remain authoritative. This module
classifies their results without modifying or hiding them, so an accepted
internal QA exception can never be mistaken for public-release evidence.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

ALLOWLISTED_FORCE = "allowlisted-force"
REQUIRED_GITHUB_CHECKS = ("DCO", "Path Filter")
EXPECTED_GITHUB_CI_CHECKS = (
    "checks / Format Check (repo format -v)",
    "build (linux-x86_64, release) / Build (linux-x86_64 / release)",
    "build-cov (linux-x86_64, release) / Build (linux-x86_64 / release / cov)",
)
PACKMAN_CHECK_PREFIXES = (
    "checks / Format Check",
    "build (linux-x86_64",
    "build-cov (linux-x86_64",
)
PACKMAN_FAILURE_PATTERNS = (
    re.compile(r"packman\.errors\.PackmanErrorFileNotFound"),
    re.compile(r"Package not found on specified remote(?: servers|[ \t]*\r?\n[^\r\n]*\bservers\b)"),
)


@dataclass(frozen=True)
class Decision:
    """A merge-policy result with all evidence retained."""

    result: str
    reason: str
    blocking: tuple[str, ...] = ()
    waived: tuple[str, ...] = ()
    waiting: tuple[str, ...] = ()

    @property
    def allowed(self) -> bool:
        return self.result == "allow"


def _failure(check: dict[str, Any]) -> bool:
    return str(check.get("conclusion") or "").lower() in {
        "action_required",
        "cancelled",
        "failure",
        "stale",
        "timed_out",
    }


def _pending(check: dict[str, Any]) -> bool:
    conclusion = check.get("conclusion")
    status = str(check.get("status") or "").lower()
    return conclusion in (None, "") and status not in {"completed"}


def _packman_failure(check: dict[str, Any]) -> bool:
    name = str(check.get("name") or "")
    evidence = str(check.get("failure_text") or "")
    return name.startswith(PACKMAN_CHECK_PREFIXES) and any(
        pattern.search(evidence) for pattern in PACKMAN_FAILURE_PATTERNS
    )


def evaluate_github_checks(checks: list[dict[str, Any]], mode: str, repository: str = "") -> Decision:
    """Require checks and scope opt-in Packman waivers to the staging repository.

    Args:
        checks: Check results and their failure evidence.
        mode: Explicit waiver mode or a value that disables waivers.
        repository: Full repository name supplied by the trusted caller.

    Returns:
        Decision with blocking, waiting, and waived checks.
    """

    if mode == ALLOWLISTED_FORCE and repository != "isaac-sim/IsaacSim-Internal":
        return Decision("block", "check waivers are unavailable for this repository")

    by_name: dict[str, dict[str, Any]] = {}
    for check in checks:
        name = str(check.get("name") or "")
        if name:
            by_name[name] = check

    expected = REQUIRED_GITHUB_CHECKS + EXPECTED_GITHUB_CI_CHECKS
    missing = [name for name in expected if name not in by_name]
    waiting = [name for name in expected if name in by_name and _pending(by_name[name])]
    if missing or waiting:
        names = tuple(missing + waiting)
        return Decision("wait", "required GitHub checks have not completed", waiting=names)

    required_failures = [
        name for name in REQUIRED_GITHUB_CHECKS if str(by_name[name].get("conclusion") or "").lower() != "success"
    ]
    if required_failures:
        return Decision(
            "block",
            "required GitHub checks failed",
            blocking=tuple(required_failures),
        )

    failed = [check for check in checks if _failure(check)]
    if not failed:
        return Decision("allow", "all completed GitHub checks passed")

    if mode != ALLOWLISTED_FORCE:
        return Decision(
            "block",
            "GitHub checks failed and the internal override is disabled",
            blocking=tuple(str(check.get("name")) for check in failed),
        )

    waived = [str(check.get("name")) for check in failed if _packman_failure(check)]
    blocking = [str(check.get("name")) for check in failed if not _packman_failure(check)]
    if blocking:
        return Decision(
            "block",
            "a GitHub failure was not an evidenced Packman dependency failure",
            blocking=tuple(blocking),
            waived=tuple(waived),
        )
    return Decision(
        "allow",
        "required checks passed; Packman-dependent failures waived for internal QA",
        waived=tuple(waived),
    )


def _load(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("surface", choices=("github",))
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--mode", default="disabled")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--repository", default="")
    args = parser.parse_args()

    payload = _load(args.evidence)
    if not isinstance(payload, list):
        raise SystemExit("GitHub evidence must be a JSON list")
    decision = evaluate_github_checks(payload, args.mode, args.repository)

    rendered = json.dumps(asdict(decision), indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    if decision.result == "allow":
        return 0
    if decision.result == "wait":
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
