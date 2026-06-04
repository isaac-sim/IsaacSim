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

"""Shared utilities for Isaac Sim benchmark profiling scripts.

Provide benchmark discovery, environment configuration, subprocess
management, and common CLI argument handling used by both the Tracy
(``tracy_capture.py``) and Nsight Systems (``nsys_capture.py``) capture
wrappers.
"""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

__all__ = [
    "BENCHMARK_PHASE_TRIGGER",
    "KIT_LOG_FILE_SETTING",
    "METRICS_FOLDER_SETTING",
    "add_common_arguments",
    "build_env",
    "configure_logging",
    "discover_benchmarks",
    "format_benchmark_list",
    "format_kit_log_file_arg",
    "parse_common_args",
    "resolve_benchmark",
    "resolve_output_paths",
    "run_benchmark",
]

# ---------------------------------------------------------------------------
# Path defaults
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
# Walk up from `source/tools/benchmark/` to the repository root.
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_RELEASE_DIR = REPO_ROOT / "_build" / "linux-x86_64" / "release"
BENCHMARKS_DIR = REPO_ROOT / "source" / "standalone_examples" / "benchmarks"
_BENCHMARK_PREFIX = "benchmark_"

# Trigger message emitted by `BaseIsaacBenchmark.set_phase("benchmark")`.
BENCHMARK_PHASE_TRIGGER = "Starting phase: benchmark"

# Kit setting controlling where benchmark JSON metrics are written.
METRICS_FOLDER_SETTING = "/exts/isaacsim.benchmark.services/metrics/metrics_output_folder"

# Carb / Kit log file path (see Kit logging guide: runtime path is ``/log/file``).
KIT_LOG_FILE_SETTING = "/log/file"

# ROS 2 environment defaults (respected only when not already set).
_ROS2_DISTRO_DEFAULT = "humble"
_ROS2_RMW_DEFAULT = "rmw_fastrtps_cpp"
# Bundled libs live under ``<release>/exts/isaacsim.ros2.core/<ROS_DISTRO>/lib``.
_ROS2_CORE_EXT_REL = Path("exts") / "isaacsim.ros2.core"


# ---------------------------------------------------------------------------
# Benchmark discovery
# ---------------------------------------------------------------------------
def discover_benchmarks(benchmarks_dir: Path | None = None) -> dict[str, Path]:
    """Scan the benchmarks directory and return a mapping of short names to paths.

    Each file matching ``benchmark_*.py`` is registered under a short name
    formed by stripping the ``benchmark_`` prefix and ``.py`` suffix.  For
    example ``benchmark_camera.py`` becomes ``camera``.

    Args:
        benchmarks_dir: Directory to scan.

    Returns:
        Dict mapping short benchmark names to their absolute paths.
    """
    search_dir = benchmarks_dir or BENCHMARKS_DIR
    if not search_dir.is_dir():
        return {}
    return {
        p.stem.removeprefix(_BENCHMARK_PREFIX): p.resolve() for p in sorted(search_dir.glob(f"{_BENCHMARK_PREFIX}*.py"))
    }


def resolve_benchmark(name_or_path: str, benchmarks_dir: Path | None = None) -> Path:
    """Resolve a benchmark given a short name or an explicit file path.

    If *name_or_path* looks like a file system path (contains ``/`` or ``.py``
    suffix) it is treated as a literal path.  Otherwise it is looked up as a
    short name inside *benchmarks_dir*.

    Args:
        name_or_path: Short name (e.g. ``camera``) or full path.
        benchmarks_dir: Directory to search for short-name resolution.

    Returns:
        Resolved absolute path to the benchmark script.

    Raises:
        FileNotFoundError: If the benchmark cannot be resolved.
    """
    # Treat as a literal path when it contains a separator or ends with `.py`.
    if "/" in name_or_path or name_or_path.endswith(".py"):
        p = Path(name_or_path).resolve()
        if p.is_file():
            return p
        raise FileNotFoundError(f"Benchmark script not found: {p}")

    # Short-name lookup.
    available = discover_benchmarks(benchmarks_dir)
    if name_or_path in available:
        return available[name_or_path]

    # Provide a helpful error with close matches.
    close = [n for n in available if name_or_path in n or n in name_or_path]
    hint = f"  Did you mean one of: {', '.join(close)}" if close else ""
    names_list = ", ".join(sorted(available)) if available else "(none found)"
    raise FileNotFoundError(f"Unknown benchmark '{name_or_path}'. Available benchmarks: {names_list}.{hint}")


def format_benchmark_list(benchmarks_dir: Path | None = None) -> str:
    """Return a human-readable table of available benchmarks.

    Args:
        benchmarks_dir: Directory to scan.

    Returns:
        Formatted multi-line string listing every benchmark short name and its path.
    """
    available = discover_benchmarks(benchmarks_dir)
    if not available:
        return "No benchmark scripts found."
    lines = ["Available benchmarks (use the short name with --benchmark):", ""]
    max_name = max(len(n) for n in available)
    for name, path in available.items():
        lines.append(f"  {name:<{max_name}}  {path}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
def build_env(
    release_dir: Path,
    *,
    extra_env: dict[str, str] | None = None,
    enable_python_profiling: bool = False,
) -> dict[str, str]:
    """Build the environment dict for benchmark and capture subprocesses.

    Set ROS 2 variables (``ROS_DISTRO`` selects which bundled
    ``isaacsim.ros2.core`` libs are prepended to ``LD_LIBRARY_PATH``) and
    optionally enable Python function-scope profiling.
    Callers can inject tool-specific variables (e.g. Tracy defaults) via
    *extra_env*.

    Args:
        release_dir: Path to the Isaac Sim release build directory.
        extra_env: Additional env vars merged into the result.
        enable_python_profiling: If True, set ``CARB_PROFILING_PYTHON=1``.

    Returns:
        A copy of ``os.environ`` with the required additions.
    """
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)

    # ROS 2 setup — honour existing env vars, fall back to defaults.
    env.setdefault("ROS_DISTRO", _ROS2_DISTRO_DEFAULT)
    env.setdefault("RMW_IMPLEMENTATION", _ROS2_RMW_DEFAULT)
    ros2_lib_dir = str(release_dir / _ROS2_CORE_EXT_REL / env["ROS_DISTRO"] / "lib")
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    if ros2_lib_dir not in existing_ld:
        env["LD_LIBRARY_PATH"] = f"{existing_ld}:{ros2_lib_dir}" if existing_ld else ros2_lib_dir
    logger.info("ROS_DISTRO=%s  RMW_IMPLEMENTATION=%s", env["ROS_DISTRO"], env["RMW_IMPLEMENTATION"])
    logger.info("LD_LIBRARY_PATH includes %s", ros2_lib_dir)

    if enable_python_profiling:
        env["CARB_PROFILING_PYTHON"] = "1"
        logger.info("Python function scope profiling ENABLED (expect slower performance)")

    return env


# ---------------------------------------------------------------------------
# Benchmark execution
# ---------------------------------------------------------------------------
def run_benchmark(
    cmd: list[str],
    env: dict[str, str],
    *,
    on_trigger: tuple[str, callable] | None = None,
    log_path: Path | None = None,
) -> int:
    """Run the benchmark subprocess and return its exit code.

    When *on_trigger* is provided **or** *log_path* is set, the benchmark's
    combined stdout/stderr is monitored line-by-line.  The first time a line
    containing the trigger string is seen the supplied callback is invoked
    (once).  All output is forwarded to the parent process's stdout in real
    time and, when *log_path* is given, also written to a log file.

    Args:
        cmd: Full command list.
        env: Environment variables.
        on_trigger: Optional ``(trigger_string, callback)`` pair.  When
            the trigger appears in any output line, *callback()* is called.
        log_path: If set, benchmark stdout/stderr is tee'd to this file.

    Returns:
        Process exit code.
    """
    logger.info("Launching benchmark: %s", " ".join(cmd))

    needs_monitoring = on_trigger is not None or log_path is not None

    if not needs_monitoring:
        # Simple mode — no output monitoring needed.
        result = subprocess.run(cmd, env=env)
        logger.info("Benchmark exited with code %d", result.returncode)
        return result.returncode

    trigger_text, trigger_callback = on_trigger if on_trigger is not None else (None, None)
    triggered = trigger_text is None  # No trigger → treat as already triggered.

    proc = subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    log_file = open(log_path, "w") if log_path is not None else None  # noqa: SIM115
    try:
        for raw_line in iter(proc.stdout.readline, b""):
            line = raw_line.decode(errors="replace")
            sys.stdout.write(line)
            sys.stdout.flush()
            if log_file is not None:
                log_file.write(line)
                log_file.flush()
            if not triggered and trigger_text in line:
                logger.info("Trigger detected: %s", trigger_text)
                trigger_callback()
                triggered = True
    finally:
        proc.stdout.close()
        proc.wait()
        if log_file is not None:
            log_file.close()

    if on_trigger is not None and not triggered:
        logger.warning(
            "Benchmark finished without emitting trigger '%s'. "
            "Profiling capture may not have started — output may be empty or missing.",
            trigger_text,
        )

    logger.info("Benchmark exited with code %d", proc.returncode)
    if log_path is not None:
        logger.info("Benchmark log saved to %s", log_path)
    return proc.returncode


def format_kit_log_file_arg(absolute_path: Path) -> str:
    """Return a ``python.sh`` CLI token that sets Kit's carb log output file.

    Sets the ``/log/file`` Carbonite setting so Kit writes its process log to
    *absolute_path* directly (no post-run copy). Parent directories should exist.

    Args:
        absolute_path: Destination log file path.

    Returns:
        A string of the form ``--/log/file=<path>``.
    """
    return f"--{KIT_LOG_FILE_SETTING}={absolute_path.resolve()}"


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
def configure_logging() -> None:
    """Set up consistent logging for benchmark profiling scripts."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


# ---------------------------------------------------------------------------
# Output paths
# ---------------------------------------------------------------------------
def resolve_output_paths(
    output_dir: str | None,
    output_name: str | None,
    benchmark_script: Path,
    extension: str,
) -> tuple[Path, str, Path]:
    """Resolve and create the output directory, base name, and output file path.

    Args:
        output_dir: User-specified output directory, or None for cwd.
        output_name: User-specified base name, or None for auto-generated.
        benchmark_script: Benchmark script path (used to derive the default name).
        extension: File extension for the primary output (e.g. ``.tracy``, ``.nsys-rep``).

    Returns:
        Tuple of (resolved output directory, base name, full output file path).
    """
    resolved_dir = Path(output_dir).resolve() if output_dir else Path.cwd()
    resolved_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = output_name or f"{benchmark_script.stem}_{timestamp}"
    output_file = resolved_dir / f"{base_name}{extension}"
    return resolved_dir, base_name, output_file


# ---------------------------------------------------------------------------
# Common CLI arguments
# ---------------------------------------------------------------------------
def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    """Register CLI arguments shared across all profiling capture scripts.

    Args:
        parser: The argument parser to augment.
    """
    parser.add_argument(
        "--benchmark",
        help=(
            "Benchmark to run — either a short name (e.g. 'camera', 'rtx_lidar', "
            "'robots_nova_carter') or a full path to a .py script.  "
            "Use --list-benchmarks to see available short names."
        ),
    )
    parser.add_argument(
        "--list-benchmarks",
        action="store_true",
        help="Print all available benchmark short names and exit.",
    )
    parser.add_argument(
        "--release-dir",
        default=str(DEFAULT_RELEASE_DIR),
        help="Path to the Isaac Sim release build directory.",
    )
    parser.add_argument(
        "--output-dir",
        help=(
            "Directory to write profiling output, benchmark metrics JSON, "
            "and log file. Defaults to current working directory."
        ),
    )
    parser.add_argument(
        "--output-name",
        help="Base name for output files (without extension).",
    )
    parser.add_argument(
        "--capture-loading",
        action="store_true",
        help=(
            "Start profiling capture immediately, including the loading phase. "
            "By default capture only begins when the benchmark phase starts "
            "(i.e. when 'Starting phase: benchmark' appears in the output)."
        ),
    )
    parser.add_argument(
        "--capture-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the capture process/session to finish after the benchmark exits.",
    )
    parser.add_argument(
        "--enable-python-profiling",
        action="store_true",
        help="Set CARB_PROFILING_PYTHON=1 to capture Python function scopes (significantly slows performance).",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Do not save benchmark stdout/stderr to a log file alongside the profiling output.",
    )
    parser.add_argument(
        "--no-kit-log-file",
        action="store_true",
        help=(
            "Do not pass Kit's --/log/file setting; carb uses its default log location "
            "(timestamped path under omni logs). By default this script sets --/log/file so the "
            "Kit log is written directly to <output-dir>/<output-name>_kit.log."
        ),
    )
    parser.add_argument(
        "--kit-log-path",
        type=Path,
        default=None,
        help=(
            "Absolute destination for Kit's carb log file via --/log/file (full path including "
            "filename). Default when omitted: <output-dir>/<output-name>_kit.log."
        ),
    )


def parse_common_args(parser: argparse.ArgumentParser, argv: list[str] | None = None) -> argparse.Namespace:
    """Parse arguments, separating benchmark pass-through args.

    Everything after ``--`` is stored as ``extra_benchmark_args``.

    Args:
        parser: The configured argument parser.
        argv: Argument list.

    Returns:
        Parsed namespace with ``extra_benchmark_args`` attached.
    """
    args, extra = parser.parse_known_args(argv)
    # Strip the -- separator from the extra arguments so downstream scripts get clean arguments.
    args.extra_benchmark_args = [arg for arg in extra if arg != "--"]
    return args
