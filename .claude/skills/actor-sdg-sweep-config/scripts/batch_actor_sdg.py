#!/usr/bin/env python3
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

"""Preview or run Actor SDG sequentially for generated configuration variants."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ACTOR_SDG_SEARCH_PATHS = (
    "source/tools/actor_sdg/actor_sdg.py",
    "tools/actor_sdg/actor_sdg.py",
)


def find_actor_sdg(explicit: Path | None = None) -> Path | None:
    """Find the Actor SDG entry point.

    Args:
        explicit: User-provided path, if any.

    Returns:
        Resolved script path when found, otherwise None.
    """
    if explicit is not None:
        path = explicit.expanduser().resolve()
        return path if path.is_file() else None

    roots = [Path.cwd()]
    isaac_sim_dir = os.environ.get("ISAAC_SIM_DIR")
    if isaac_sim_dir:
        roots.append(Path(isaac_sim_dir).expanduser())
    for root in roots:
        for relative in ACTOR_SDG_SEARCH_PATHS:
            candidate = (root / relative).resolve()
            if candidate.is_file():
                return candidate
    return None


def discover_variants(variants_dir: Path) -> list[dict[str, Any]]:
    """Find generated variant configuration files.

    Args:
        variants_dir: Directory produced by the config variation generator.

    Returns:
        Variant identifiers and resolved configuration paths.
    """
    manifest_path = variants_dir / "manifest.csv"
    variants: list[dict[str, Any]] = []
    if manifest_path.is_file():
        with manifest_path.open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                config_path = Path(row["config_path"])
                if not config_path.is_absolute():
                    config_path = variants_dir / config_path
                variants.append(
                    {
                        "variant_id": int(row["variant_id"]),
                        "config_path": config_path.resolve(),
                    }
                )
        return sorted(variants, key=lambda item: item["variant_id"])

    for variant_dir in sorted(variants_dir.glob("variant_*")):
        suffix = variant_dir.name.removeprefix("variant_")
        config_path = variant_dir / "config.yaml"
        if variant_dir.is_dir() and suffix.isdigit() and config_path.is_file():
            variants.append(
                {
                    "variant_id": int(suffix),
                    "config_path": config_path.resolve(),
                }
            )
    return variants


def select_variants(
    variants: list[dict[str, Any]],
    start_from: int,
    max_count: int | None,
) -> list[dict[str, Any]]:
    """Select a consecutive subset of discovered variants."""
    selected = [variant for variant in variants if variant["variant_id"] >= start_from]
    return selected[:max_count] if max_count is not None else selected


def _read_error_summary(log_path: Path) -> str:
    """Read a concise error summary from an Actor SDG log."""
    try:
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return ""
    return "; ".join(line.strip() for line in lines[-3:] if line.strip())[:300]


def run_variant(
    python_launcher: Path,
    actor_sdg_path: Path,
    variant: dict[str, Any],
    log_dir: Path,
    timeout: int | None,
    *,
    allow_trusted_scripts: bool = False,
) -> dict[str, Any]:
    """Run Actor SDG for one variant and return its result row."""
    if not allow_trusted_scripts:
        raise ValueError("Explicit trust is required to execute Actor SDG scripts.")
    variant_id = variant["variant_id"]
    config_path = variant["config_path"]
    log_path = log_dir / f"variant_{variant_id:04d}.log"
    command = [str(python_launcher), str(actor_sdg_path), "-c", str(config_path), "--allow-trusted-scripts"]
    started = time.monotonic()

    try:
        with log_path.open("w", encoding="utf-8") as log_stream:
            result = subprocess.run(
                command,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )
        exit_code = result.returncode
        error_summary = "" if exit_code == 0 else _read_error_summary(log_path)
    except subprocess.TimeoutExpired:
        exit_code = -1
        error_summary = f"timed out after {timeout} seconds"
    except OSError as exc:
        exit_code = -1
        error_summary = str(exc)

    elapsed = time.monotonic() - started
    success = exit_code == 0
    status = "passed" if success else "failed"
    print(f"variant_{variant_id:04d}: {status} in {elapsed:.1f}s; log={log_path}")
    return {
        "variant_id": variant_id,
        "config_path": str(config_path),
        "success": success,
        "exit_code": exit_code,
        "elapsed_seconds": f"{elapsed:.1f}",
        "error_summary": error_summary,
        "log_path": str(log_path),
    }


def write_results(results: list[dict[str, Any]], output_path: Path) -> None:
    """Write batch results to CSV."""
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--variants", required=True, type=Path, help="Generated variants directory.")
    parser.add_argument("--python-sh", required=True, type=Path, help="Isaac Sim Python launcher.")
    parser.add_argument("--actor-sdg", type=Path, help="Path to actor_sdg.py.")
    parser.add_argument("--start-from", type=int, default=0, help="First variant identifier. Default: 0.")
    parser.add_argument("--max-count", type=int, help="Maximum number of variants to run.")
    parser.add_argument("--timeout", type=int, help="Timeout for each variant in seconds.")
    parser.add_argument("--dry-run", action="store_true", help="List selected variants without launching Actor SDG.")
    parser.add_argument(
        "--allow-trusted-scripts",
        action="store_true",
        help="Allow Python execution from trusted configurations and all referenced assets with your user permissions.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the Actor SDG batch command."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.dry_run and not args.allow_trusted_scripts:
        parser.error(
            "Execution requires --allow-trusted-scripts; use only trusted configurations and referenced assets."
        )
    variants_dir = args.variants.expanduser().resolve()
    python_launcher = args.python_sh.expanduser().resolve()
    actor_sdg_path = find_actor_sdg(args.actor_sdg)

    if not variants_dir.is_dir():
        parser.error(f"variants directory not found: {variants_dir}")
    if not python_launcher.is_file():
        parser.error(f"Isaac Sim Python launcher not found: {python_launcher}")
    if actor_sdg_path is None:
        parser.error("actor_sdg.py was not found; pass its path through --actor-sdg")
    if args.start_from < 0:
        parser.error("--start-from must be non-negative")
    if args.max_count is not None and args.max_count <= 0:
        parser.error("--max-count must be positive")
    if args.timeout is not None and args.timeout <= 0:
        parser.error("--timeout must be positive")

    variants = select_variants(discover_variants(variants_dir), args.start_from, args.max_count)
    if not variants:
        parser.error("no matching variant configs were found")
    missing_configs = [variant["config_path"] for variant in variants if not variant["config_path"].is_file()]
    if missing_configs:
        parser.error(f"{len(missing_configs)} selected config file(s) are missing; first missing: {missing_configs[0]}")

    print(f"Actor SDG script: {actor_sdg_path}")
    print(f"Isaac Sim Python: {python_launcher}")
    print(f"Selected variants: {len(variants)}")
    for variant in variants:
        print(f"  variant_{variant['variant_id']:04d}: {variant['config_path']}")
    if args.dry_run:
        print("Dry-run complete. Actor SDG was not launched.")
        return 0

    log_dir = variants_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    results = [
        run_variant(
            python_launcher,
            actor_sdg_path,
            variant,
            log_dir,
            args.timeout,
            allow_trusted_scripts=args.allow_trusted_scripts,
        )
        for variant in variants
    ]
    results_path = variants_dir / "run_results.csv"
    write_results(results, results_path)
    failed = [result for result in results if not result["success"]]
    print(f"Results: {results_path}")
    print(f"Completed: {len(results) - len(failed)} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
