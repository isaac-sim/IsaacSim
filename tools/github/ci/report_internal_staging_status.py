#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Report the terminal GitHub Internal staging outcome to Slack."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def link(url: object, label: str) -> str:
    return f"<{url}|{label}>" if url else label


def load_decision(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def describe(
    *,
    pr_number: str,
    pr_url: str,
    head_sha: str,
    merge_outcome: str,
    policy_outcome: str,
    decision: dict[str, Any] | None,
    run_url: str,
) -> str:
    short_head = head_sha[:10] or "unknown"
    pr = link(pr_url, f"PR #{pr_number}")
    run = link(run_url, "Workflow run")
    if merge_outcome == "success":
        lines = [
            f":white_check_mark: GitHub Internal staging completed at `{short_head}`.",
            f"GitHub Internal: {pr} (merged).",
        ]
        waived = (decision or {}).get("waived") or []
        if waived:
            lines.append(
                ":warning: Internal-only checks waived: "
                + ", ".join(f"`{item}`" for item in waived)
                + ". This is not public-promotion evidence."
            )
        lines.append(run)
        return "\n".join(lines)

    policy_blocked = (decision or {}).get("result") == "block"
    reason = str((decision or {}).get("reason") or "").strip() if policy_blocked else ""
    blocking = ((decision or {}).get("blocking") or []) if policy_blocked else []
    if blocking:
        reason += (": " if reason else "") + ", ".join(str(item) for item in blocking)
    if not reason:
        if policy_blocked:
            reason = "merge policy blocked the PR"
        elif merge_outcome == "failure":
            reason = "GitHub merge step failed after policy evaluation"
        elif policy_outcome == "failure":
            reason = "merge policy blocked the PR"
        else:
            reason = f"workflow stopped before merge ({merge_outcome or 'unknown'})"
    return f":rotating_light: GitHub Internal staging at `{short_head}` blocked: " f"{reason}. {pr}. {run}"


def post(webhook: str, text: str) -> None:
    request = urllib.request.Request(
        webhook,
        data=json.dumps({"text": text}).encode("utf-8"),
        headers={"Content-Type": "application/json; charset=utf-8"},
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            body = response.read().decode("utf-8").strip()
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace").strip()
        raise SystemExit(f"Slack rejected staging report: HTTP {error.code} {detail}")
    except (urllib.error.URLError, TimeoutError) as error:
        raise SystemExit(f"could not reach Slack for staging report: {error}")
    if body != "ok":
        raise SystemExit(f"Slack returned an unexpected response: {body[:200]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pr-number", required=True)
    parser.add_argument("--pr-url", required=True)
    parser.add_argument("--head-sha", required=True)
    parser.add_argument("--merge-outcome", required=True)
    parser.add_argument("--policy-outcome", default="")
    parser.add_argument("--policy", type=Path, default=Path("_staging/decision.json"))
    parser.add_argument("--run-url", required=True)
    parser.add_argument("--no-post", action="store_true")
    args = parser.parse_args()

    text = describe(
        pr_number=args.pr_number,
        pr_url=args.pr_url,
        head_sha=args.head_sha,
        merge_outcome=args.merge_outcome,
        policy_outcome=args.policy_outcome,
        decision=load_decision(args.policy),
        run_url=args.run_url,
    )
    print(text)
    if not args.no_post:
        webhook = os.environ.get("ISAAC_SIM_NOTIFIER_SLACK_WEBHOOK")
        if not webhook:
            raise SystemExit("ISAAC_SIM_NOTIFIER_SLACK_WEBHOOK is unset")
        post(webhook, text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
