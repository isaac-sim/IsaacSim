# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Write and summarize a benchmark result artifact."""

from __future__ import annotations

import json
import os
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from isaacsim.performance.benchmarking.impl.model import _BenchmarkResult
from isaacsim.performance.benchmarking.impl.schema import result_document

_OUTPUT_DIRECTORY_ENVIRONMENT_VARIABLE = "ISAACSIM_BENCHMARK_OUTPUT_DIR"


def resolve_output_path(result: _BenchmarkResult, output: str | Path | None = None) -> Path:
    """Resolve an explicit or unique default result path.

    Args:
        result: Benchmark result being written.
        output: Explicit output path, if requested.

    Returns:
        Absolute result path.

    """
    if output is not None:
        return Path(output).expanduser().resolve()

    configured_root = os.environ.get(_OUTPUT_DIRECTORY_ENVIRONMENT_VARIABLE)
    root = (
        Path(configured_root).expanduser() if configured_root else Path(tempfile.gettempdir()) / "isaacsim-benchmarks"
    )
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    filename = f"{timestamp}-{result.run_id}.json"
    return (root / result.definition.identifier / filename).resolve()


def _write_document(document: dict[str, Any], path: Path) -> None:
    """Publish a JSON document only after its complete contents reach disk.

    Args:
        document: JSON-compatible document to serialize.
        path: Destination JSON path.

    """
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, indent=2, sort_keys=True, allow_nan=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def write_result(result: _BenchmarkResult, path: Path) -> dict[str, Any]:
    """Write a result and, when requested, its referenced collector observations.

    Args:
        result: Benchmark result to serialize.
        path: Destination result JSON path.

    Returns:
        Structured result document written to disk.

    """
    document = result_document(result)
    sidecar = None
    if result.collector_capture is not None:
        sidecar = path.with_name(f"{path.stem}.{uuid.uuid4().hex}.collector.json")
        document["environment"] = {
            **document["environment"],
            "collector": {**document["environment"].get("collector", {}), "artifact": sidecar.name},
        }
        _write_document(result.collector_capture, sidecar)
    try:
        _write_document(document, path)
    except BaseException:
        if sidecar is not None:
            try:
                sidecar.unlink()
            except OSError:
                pass
        raise
    return document


def format_summary(document: dict[str, Any]) -> str:
    """Return one concise human-readable result summary.

    Args:
        document: Structured benchmark result document.

    Returns:
        Single-line benchmark status and duration summary.

    """
    identifier = document["benchmark"]["id"]
    status = "valid" if document["status"]["valid"] else "invalid"
    duration = document["metrics"].get("duration_ms", {}).get("statistics", {}).get("mean")
    if duration is None:
        return f"{identifier}: {status}"
    return f"{identifier}: {status}, mean {duration:.6g} ms"
