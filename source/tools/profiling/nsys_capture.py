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

"""Nsight Systems profiling wrapper for Isaac Sim benchmark scripts.

Launch a benchmark script under ``nsys launch`` with NVIDIA tracing enabled
and perform a timed capture session during the benchmark phase.

Typical usage from the build directory::

    # Select a benchmark by short name with a 10-second capture
    ./python.sh tools/profiling/nsys_capture.py \\
        --benchmark camera \\
        --capture-duration 10 \\
        --output-dir /tmp/nsys_output \\
        -- --num-cameras 2 --num-frames 300

    # Capture from app start (including loading phase)
    ./python.sh tools/profiling/nsys_capture.py \\
        --benchmark camera --capture-loading --capture-duration 30

    # List all available benchmarks
    ./python.sh tools/profiling/nsys_capture.py \\
        --list-benchmarks

The script:

1. Resolves the benchmark — either a short name (e.g. ``camera``,
   ``rtx_lidar``) or an explicit file path.
2. Verifies that ``nsys`` is available on the system PATH.
3. Wraps the benchmark in ``nsys launch`` with the configured trace types
   and creates a named session for capture control.
4. Monitors benchmark output for the trigger message (``Starting phase:
   benchmark``) and starts a timed ``nsys start`` → ``nsys stop`` capture
   session.
5. Saves the full benchmark stdout/stderr to a ``.log`` file alongside
   the ``.nsys-rep`` capture (disable with ``--no-log``).
6. Sets Kit's ``--/log/file`` so the carb process log is written directly under
   ``--output-dir`` (same defaults as ``tracy_capture.py``).
7. After the benchmark exits, waits for the capture session to complete
   and reports the output file location.

.. note::

   Some nsys tracing capabilities (e.g. OS runtime) may require
   elevated privileges.  Use ``--sudo`` to prefix nsys commands with
   ``sudo -E``.
"""

from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from _common import (
    BENCHMARK_PHASE_TRIGGER,
    METRICS_FOLDER_SETTING,
    add_common_arguments,
    build_env,
    configure_logging,
    format_benchmark_list,
    format_kit_log_file_arg,
    parse_common_args,
    resolve_benchmark,
    resolve_output_paths,
    run_benchmark,
)

configure_logging()
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Nsys-specific defaults
# ---------------------------------------------------------------------------
# Kit arguments that enable NVTX annotations and profiling for nsys.
NSYS_KIT_ARGS: list[str] = [
    "--/app/profilerBackend=nvtx",
    "--/app/profileFromStart=true",
    "--/profiler/enabled=true",
]

# Default nsys trace types matching the profiling repo's configuration.
NSYS_DEFAULT_TRACES = "nvtx,osrt,cuda"

# Default capture duration in seconds.
NSYS_DEFAULT_CAPTURE_DURATION = 10.0

# Default session name for ``nsys launch`` / ``nsys start`` / ``nsys stop``.
NSYS_SESSION_NAME = "isaacsim_benchmark"

# Seconds to wait after ``nsys launch`` before starting capture in
# ``--capture-loading`` mode, allowing the session to initialise.
_NSYS_INIT_DELAY = 3.0


# ---------------------------------------------------------------------------
# nsys binary discovery
# ---------------------------------------------------------------------------
def find_nsys_binary() -> Path:
    """Locate the ``nsys`` binary on the system PATH.

    Returns:
        Path to the ``nsys`` executable.

    Raises:
        FileNotFoundError: If ``nsys`` is not found.
    """
    nsys_path = shutil.which("nsys")
    if nsys_path is not None:
        return Path(nsys_path)
    raise FileNotFoundError(
        "nsys not found on PATH. Install NVIDIA Nsight Systems "
        "(https://developer.nvidia.com/nsight-systems) or ensure it is on your PATH."
    )


# ---------------------------------------------------------------------------
# Capture session
# ---------------------------------------------------------------------------
class NsysCaptureSession:
    """Manage a timed nsys capture session via ``nsys start`` / ``nsys stop``.

    The session is started by calling ``start_capture``, which launches a
    background thread that runs ``nsys start``, sleeps for the configured
    duration, then runs ``nsys stop``.

    Args:
        session_name: nsys session name (must match the session created
            by ``nsys launch --session-new``).
        output_path: Output file path prefix (nsys appends ``.nsys-rep``).
        duration: Capture duration in seconds.
        env: Environment variables for nsys subprocesses.
        sudo: If True, prefix nsys commands with ``sudo -E``.
    """

    def __init__(
        self,
        session_name: str,
        output_path: Path,
        duration: float,
        env: dict[str, str],
        *,
        sudo: bool = False,
    ) -> None:
        self._session = session_name
        self._output = output_path
        self._duration = duration
        self._env = env
        self._sudo = sudo
        self._thread: threading.Thread | None = None
        self._completed: bool = False
        self._error: str | None = None

    def _build_cmd(self, *args: str) -> list[str]:
        """Build an nsys command, optionally prefixed with ``sudo -E``."""
        cmd: list[str] = ["nsys", *args]
        if self._sudo:
            cmd = ["sudo", "-E", *cmd]
        return cmd

    def start_capture(self) -> None:
        """Begin a timed capture in a background thread."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("nsys capture session started (duration=%.0fs)", self._duration)

    def _run(self) -> None:
        """Execute ``nsys start`` → sleep → ``nsys stop``."""
        start_cmd = self._build_cmd(
            "start",
            "--session",
            self._session,
            "-o",
            str(self._output),
            "--sample=process-tree",
            "--backtrace=lbr",
        )
        logger.info("Running: %s", " ".join(start_cmd))
        try:
            result = subprocess.run(start_cmd, env=self._env, capture_output=True, text=True)
            if result.returncode != 0:
                self._error = f"nsys start failed (rc={result.returncode}): {result.stderr.strip()}"
                logger.error("%s", self._error)
                return

            logger.info("Capturing for %.0f seconds...", self._duration)
            time.sleep(self._duration)

            stop_cmd = self._build_cmd("stop", "--session", self._session)
            logger.info("Stopping capture: %s", " ".join(stop_cmd))
            result = subprocess.run(stop_cmd, env=self._env, capture_output=True, text=True)
            if result.returncode != 0:
                # Non-fatal — session may have ended (benchmark exited before capture finished).
                logger.warning("nsys stop returned rc=%d: %s", result.returncode, result.stderr.strip())

            self._completed = True
            logger.info("nsys capture completed")
        except Exception:
            logger.exception("nsys capture session failed")

    def wait(self, timeout: float = 30.0) -> None:
        """Wait for the capture thread to finish.

        Args:
            timeout: Maximum seconds to wait.
        """
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("nsys capture thread still alive after %.0fs", timeout)

    @property
    def completed(self) -> bool:
        """Return whether the capture session finished successfully."""
        return self._completed

    @property
    def error(self) -> str | None:
        """Return the error message if the capture session failed."""
        return self._error


# ---------------------------------------------------------------------------
# Command building
# ---------------------------------------------------------------------------
def build_nsys_launch_command(
    release_dir: Path,
    benchmark_script: Path,
    extra_args: list[str],
    *,
    session_name: str = NSYS_SESSION_NAME,
    traces: str = NSYS_DEFAULT_TRACES,
    metrics_output_dir: Path | None = None,
    kit_log_file: Path | None = None,
    env: dict[str, str] | None = None,
    sudo: bool = False,
) -> list[str]:
    """Build the ``nsys launch`` command that wraps the benchmark.

    Args:
        release_dir: Path to the Isaac Sim release build directory.
        benchmark_script: Path to the benchmark Python script.
        extra_args: Additional CLI arguments forwarded to the benchmark.
        session_name: nsys session name.
        traces: Comma-separated list of nsys trace types.
        metrics_output_dir: If provided, inject the Kit metrics folder setting.
        kit_log_file: If provided, set ``/log/file`` so carb writes its log here.
        env: Environment dict — critical vars are forwarded via ``--env-var``.
        sudo: If True, prefix with ``sudo -E``.

    Returns:
        Full command list.

    Raises:
        FileNotFoundError: If ``python.sh`` is missing.
    """
    python_sh = release_dir / "python.sh"
    if not python_sh.is_file():
        raise FileNotFoundError(f"python.sh not found at {python_sh}")

    cmd: list[str] = []
    if sudo:
        cmd.extend(["sudo", "-E"])

    cmd.extend(
        [
            "nsys",
            "launch",
            f"--session-new={session_name}",
            f"--trace={traces}",
            "--cuda-memory-usage=true",
        ]
    )

    # Forward critical env vars through nsys to the launched process.
    if env:
        forward_keys = ("ROS_DISTRO", "RMW_IMPLEMENTATION", "LD_LIBRARY_PATH")
        pairs = [f"{k}={env[k]}" for k in forward_keys if k in env]
        if pairs:
            cmd.append(f"--env-var={','.join(pairs)}")

    cmd.extend([str(python_sh), str(benchmark_script)])
    cmd.extend(NSYS_KIT_ARGS)
    if metrics_output_dir is not None:
        cmd.append(f"--{METRICS_FOLDER_SETTING}={metrics_output_dir}")
    if kit_log_file is not None:
        cmd.append(format_kit_log_file_arg(kit_log_file))
    cmd.extend(extra_args)
    return cmd


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments.

    Args:
        argv: Argument list.

    Returns:
        Parsed namespace.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Run an Isaac Sim benchmark script under Nsight Systems and " "capture a timed profiling session."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples (run from the build directory):\n"
            "  # Basic capture (10s default)\n"
            "  ./python.sh tools/benchmark/nsys_capture.py \\\n"
            "      --benchmark camera -- --num-cameras 2\n"
            "\n"
            "  # Custom capture duration and output\n"
            "  ./python.sh tools/benchmark/nsys_capture.py \\\n"
            "      --benchmark camera --capture-duration 20 \\\n"
            "      --output-dir /tmp/nsys_output -- --num-cameras 2\n"
            "\n"
            "  # Capture from app start (including loading)\n"
            "  ./python.sh tools/benchmark/nsys_capture.py \\\n"
            "      --benchmark camera --capture-loading --capture-duration 30\n"
            "\n"
            "  # With sudo for system-level tracing\n"
            "  ./python.sh tools/benchmark/nsys_capture.py \\\n"
            "      --benchmark camera --sudo\n"
            "\n"
            "  # Custom trace types\n"
            "  ./python.sh tools/benchmark/nsys_capture.py \\\n"
            "      --benchmark camera --trace nvtx,cuda\n"
            "\n"
            "Any arguments after '--' are forwarded directly to the benchmark script.\n"
        ),
    )
    add_common_arguments(parser)

    # Nsys-specific arguments.
    parser.add_argument(
        "--capture-duration",
        type=float,
        default=NSYS_DEFAULT_CAPTURE_DURATION,
        help="Duration in seconds for the nsys capture session.",
    )
    parser.add_argument(
        "--trace",
        default=NSYS_DEFAULT_TRACES,
        help="Comma-separated nsys trace types.",
    )
    parser.add_argument(
        "--sudo",
        action="store_true",
        help="Prefix nsys commands with 'sudo -E' for system-level tracing.",
    )

    return parse_common_args(parser, argv)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    """Entry point for the nsys capture wrapper.

    Args:
        argv: CLI arguments.

    Returns:
        Exit code (0 on success).
    """
    args = parse_args(argv)

    # Handle `--list-benchmarks` early exit.
    if args.list_benchmarks:
        print(format_benchmark_list())
        return 0

    if args.benchmark is None:
        logger.error("--benchmark is required (or use --list-benchmarks to see available options)")
        return 1

    release_dir = Path(args.release_dir).resolve()
    if not release_dir.is_dir():
        logger.error("Release directory does not exist: %s", release_dir)
        return 1

    # Resolve benchmark — short name or explicit path.
    try:
        benchmark_script = resolve_benchmark(args.benchmark)
    except FileNotFoundError as exc:
        logger.error("%s", exc)
        return 1
    logger.info("Resolved benchmark: %s", benchmark_script)

    # Verify nsys is available.
    try:
        nsys_bin = find_nsys_binary()
    except FileNotFoundError as exc:
        logger.error("%s", exc)
        return 1
    logger.info("Using nsys at %s", nsys_bin)

    # Output paths — nsys appends `.nsys-rep` to the base, so pass empty extension.
    output_dir, base_name, nsys_output = resolve_output_paths(args.output_dir, args.output_name, benchmark_script, "")

    kit_log_file: Path | None = None
    if not args.no_kit_log_file:
        kit_log_file = (
            args.kit_log_path.resolve()
            if args.kit_log_path is not None
            else (output_dir / f"{base_name}_kit.log").resolve()
        )
        kit_log_file.parent.mkdir(parents=True, exist_ok=True)
        logger.info("Kit carb log (--/log/file): %s", kit_log_file)

    # Build environment and command.
    env = build_env(release_dir, enable_python_profiling=args.enable_python_profiling)

    try:
        cmd = build_nsys_launch_command(
            release_dir,
            benchmark_script,
            args.extra_benchmark_args,
            traces=args.trace,
            metrics_output_dir=output_dir,
            kit_log_file=kit_log_file,
            env=env,
            sudo=args.sudo,
        )
    except FileNotFoundError as exc:
        logger.error("%s", exc)
        return 1

    # Log file.
    log_path = None if args.no_log else output_dir / f"{base_name}.log"

    # Capture session.
    capture_session = NsysCaptureSession(
        session_name=NSYS_SESSION_NAME,
        output_path=nsys_output,
        duration=args.capture_duration,
        env=env,
        sudo=args.sudo,
    )

    if args.capture_loading:
        logger.info("Capture mode: full (including loading phase, duration=%.0fs)", args.capture_duration)
        # Start capture after a brief delay to let the nsys session initialise.
        threading.Timer(_NSYS_INIT_DELAY, capture_session.start_capture).start()
        trigger = None
    else:
        logger.info(
            "Capture mode: benchmark-phase only (waiting for '%s', duration=%.0fs)",
            BENCHMARK_PHASE_TRIGGER,
            args.capture_duration,
        )
        trigger = (BENCHMARK_PHASE_TRIGGER, capture_session.start_capture)

    exit_code = run_benchmark(cmd, env, on_trigger=trigger, log_path=log_path)

    # Wait for capture to complete.
    logger.info("Waiting for nsys capture to complete (timeout=%.0fs)...", args.capture_timeout)
    capture_session.wait(timeout=args.capture_timeout)

    # Report output.
    nsys_output_file = output_dir / f"{base_name}.nsys-rep"
    if nsys_output_file.is_file():
        file_size_mb = nsys_output_file.stat().st_size / (1024 * 1024)
        logger.info("nsys capture file: %s (%.1f MB)", nsys_output_file, file_size_mb)
    elif capture_session.completed:
        # nsys may write the file elsewhere or with a different naming scheme.
        logger.warning("nsys reported success but output file not found at %s", nsys_output_file)
    else:
        logger.warning("nsys capture did not complete successfully")
        if capture_session.error:
            logger.error("Error: %s", capture_session.error)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
