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

"""Tracy profiling wrapper for Isaac Sim benchmark scripts.

Launch a benchmark script with the Tracy profiler backend enabled and
simultaneously capture the Tracy profile via the ``capture`` binary that
ships with ``omni.kit.profiler.tracy``.

Typical usage from the build directory::

    # Select a benchmark by short name and forward benchmark-specific args
    ./python.sh tools/benchmark/tracy_capture.py \\
        --benchmark camera \\
        --output-dir /tmp/tracy_output \\
        -- --num-cameras 2 --num-frames 300

    # Or provide a full path
    ./python.sh tools/benchmark/tracy_capture.py \\
        --benchmark standalone_examples/benchmarks/benchmark_camera.py \\
        -- --num-cameras 2

    # List all available benchmarks
    ./python.sh tools/benchmark/tracy_capture.py \\
        --list-benchmarks

The script:

1. Resolves the benchmark — either a short name (e.g. ``camera``,
   ``rtx_lidar``) looked up in ``standalone_examples/benchmarks/``
   or an explicit file path.
2. Locates the Tracy ``capture`` binary inside the Kit ``extscache``.
3. Starts ``capture -f -o <output>`` in a background thread so it is
   ready to accept a connection from the profiled process.
4. Launches ``python.sh <benchmark> <tracy_args> <user_args>`` as a
   subprocess, injecting the necessary Tracy backend flags and directing
   benchmark metrics JSON output into ``--output-dir``.
5. Saves the full benchmark stdout/stderr to a ``.log`` file alongside
   the ``.tracy`` capture (disable with ``--no-log``).
6. Sets Kit's ``--/log/file`` so the carb process log is written directly to
   ``<output-dir>/<output-name>_kit.log`` (override with ``--kit-log-path``, or
   pass ``--no-kit-log-file`` to keep Kit's default log location).
7. After the benchmark exits, stops the capture thread and optionally
   compresses the resulting ``.tracy`` file.
"""

from __future__ import annotations

import argparse
import logging
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
# Tracy-specific defaults
# ---------------------------------------------------------------------------
# Tracy backend Kit arguments recommended by the omniperf benchmark CI doc.
TRACY_KIT_ARGS: list[str] = [
    "--/app/profilerBackend=tracy",
    "--/app/profileFromStart=true",
    "--/profiler/gpu/tracyInject/enabled=true",
    "--/plugins/carb.profiler-tracy.plugin/fibersAsThreads=false",
    "--/profiler/gpu/tracyInject/msBetweenClockCalibration=0",
    "--/rtx/addTileGpuAnnotations=true",
    "--/profiler/channels/carb.events/enabled=false",
    "--/profiler/channels/carb.tasking/enabled=false",
    "--/app/profilerMask=1",
    "--/plugins/carb.profiler-tracy.plugin/instantEventsAsMessages=true",
]

# Environment variables for Tracy.
TRACY_ENV_DEFAULTS: dict[str, str] = {
    # Prevents large file sizes and slow capturing from system tracing.
    "TRACY_NO_SYS_TRACE": "1",
}


# ---------------------------------------------------------------------------
# Tracy tool discovery
# ---------------------------------------------------------------------------
def find_tracy_extension(release_dir: Path) -> Path:
    """Locate the ``omni.kit.profiler.tracy`` extension inside *release_dir*.

    Args:
        release_dir: Path to the Isaac Sim release build directory.

    Returns:
        Path to the Tracy extension root.

    Raises:
        FileNotFoundError: If no matching extension directory is found.
    """
    extscache = release_dir / "extscache"
    matches = sorted(extscache.glob("omni.kit.profiler.tracy*"))
    if not matches:
        raise FileNotFoundError(
            f"Could not find omni.kit.profiler.tracy extension in {extscache}. " "Make sure the project is built."
        )
    # Take the last (highest version) match.
    ext_path = matches[-1]
    logger.info("Found Tracy extension at %s", ext_path)
    return ext_path


def find_tracy_binary(tracy_ext_path: Path, name: str, *, required: bool = True) -> Path | None:
    """Return the path to a binary inside the Tracy extension's ``bin/`` directory.

    Args:
        tracy_ext_path: Root of the Tracy extension.
        name: Binary name (e.g. ``capture``, ``update``).
        required: If True, raise on missing binary; otherwise return None.

    Returns:
        Path to the executable, or None if not found and not required.

    Raises:
        FileNotFoundError: If *required* is True and the binary does not exist.
    """
    binary = tracy_ext_path / "bin" / name
    if binary.is_file():
        return binary
    if required:
        raise FileNotFoundError(f"Tracy '{name}' binary not found at {binary}")
    return None


# ---------------------------------------------------------------------------
# Capture thread
# ---------------------------------------------------------------------------
class TracyCaptureThread:
    """Manage a background ``capture`` process that records a ``.tracy`` file.

    The capture binary blocks until a Tracy-instrumented process connects.
    It continues recording until the profiled process disconnects or this
    wrapper terminates the capture process.

    Args:
        capture_bin: Path to the ``capture`` executable.
        output_path: Destination path for the ``.tracy`` file.
        env: Environment dict to pass to the subprocess.
    """

    def __init__(self, capture_bin: Path, output_path: Path, env: dict[str, str]) -> None:
        self._capture_bin = capture_bin
        self._output_path = output_path
        self._env = env
        self._process: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Launch the capture process in a daemon thread."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        # Give capture a moment to start listening.
        time.sleep(1)
        logger.info("Tracy capture thread started")

    def _run(self) -> None:
        """Execute the capture command."""
        cmd = [str(self._capture_bin), "-o", str(self._output_path), "-f"]
        logger.info("Running: %s", " ".join(cmd))
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=self._env,
            )
            stdout, _ = self._process.communicate()
            if stdout:
                logger.info("Tracy capture output:\n%s", stdout.decode(errors="replace"))
        except Exception:
            logger.exception("Tracy capture process failed")

    def wait(self, timeout: float = 30.0) -> None:
        """Wait for the capture thread to complete.

        Args:
            timeout: Maximum seconds to wait before force-killing the process.
        """
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("Capture thread still alive after %.0fs — terminating capture process", timeout)
                self.terminate()

    def terminate(self) -> None:
        """Send SIGTERM to the capture process if it is still running."""
        if self._process is not None and self._process.poll() is None:
            logger.info("Terminating capture process (pid=%d)", self._process.pid)
            self._process.terminate()
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.warning("Capture process did not exit — killing")
                self._process.kill()

    @property
    def output_exists(self) -> bool:
        """Check whether the ``.tracy`` output file was created."""
        return self._output_path.is_file()


# ---------------------------------------------------------------------------
# Benchmark launcher
# ---------------------------------------------------------------------------
def build_benchmark_command(
    release_dir: Path,
    benchmark_script: Path,
    extra_args: list[str],
    *,
    metrics_output_dir: Path | None = None,
    kit_log_file: Path | None = None,
) -> list[str]:
    """Assemble the full command line for the Tracy-profiled benchmark.

    Args:
        release_dir: Path to the Isaac Sim release build directory.
        benchmark_script: Path to the benchmark Python script.
        extra_args: Additional CLI arguments forwarded to the benchmark.
        metrics_output_dir: If provided, inject the Kit setting that tells
            ``isaacsim.benchmark.services`` to write JSON metrics here.
        kit_log_file: If provided, set ``/log/file`` so carb writes its log here.

    Returns:
        List of command tokens ready for ``subprocess.run``.
    """
    python_sh = release_dir / "python.sh"
    if not python_sh.is_file():
        raise FileNotFoundError(f"python.sh not found at {python_sh}")

    cmd = [str(python_sh), str(benchmark_script), *TRACY_KIT_ARGS]
    if metrics_output_dir is not None:
        cmd.append(f"--{METRICS_FOLDER_SETTING}={metrics_output_dir}")
    if kit_log_file is not None:
        cmd.append(format_kit_log_file_arg(kit_log_file))
    cmd.extend(extra_args)
    return cmd


# ---------------------------------------------------------------------------
# Compression
# ---------------------------------------------------------------------------
def compress_tracy_file(update_bin: Path, raw_path: Path, compressed_path: Path, env: dict[str, str]) -> bool:
    """Compress a ``.tracy`` file using the Tracy ``update`` tool.

    Args:
        update_bin: Path to the ``update`` executable.
        raw_path: Uncompressed ``.tracy`` file.
        compressed_path: Destination for the compressed file.
        env: Environment variables.

    Returns:
        True if compression succeeded and the compressed file exists.
    """
    cmd = [str(update_bin), "-z", "1", str(raw_path), str(compressed_path)]
    logger.info("Compressing Tracy file: %s", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if result.stdout:
        logger.info("Compression output: %s", result.stdout.strip())
    if result.stderr:
        logger.warning("Compression stderr: %s", result.stderr.strip())

    if compressed_path.is_file():
        logger.info("Compressed Tracy file written to %s", compressed_path)
        raw_path.unlink(missing_ok=True)
        return True
    logger.warning("Compression did not produce %s", compressed_path)
    return False


# ---------------------------------------------------------------------------
# CSV export
# ---------------------------------------------------------------------------
def export_tracy_to_csv(
    csvexport_bin: Path,
    tracy_path: Path,
    csv_path: Path,
    env: dict[str, str],
    *,
    filter_name: str = "",
    self_times: bool = False,
    unwrap: bool = False,
    messages: bool = False,
) -> bool:
    """Export a ``.tracy`` profile to CSV using the Tracy ``csvexport`` tool.

    Args:
        csvexport_bin: Path to the ``csvexport`` executable.
        tracy_path: Path to the ``.tracy`` file to export.
        csv_path: Destination path for the CSV output.
        env: Environment variables.
        filter_name: Zone name filter (passed via ``-f``).
        self_times: If True, report self times (``-e``).
        unwrap: If True, report each zone event (``-u``).
        messages: If True, report only messages (``-m``).

    Returns:
        True if the CSV file was written successfully.
    """
    cmd: list[str] = [str(csvexport_bin)]
    if filter_name:
        cmd.extend(["-f", filter_name])
    if self_times:
        cmd.append("-e")
    if unwrap:
        cmd.append("-u")
    if messages:
        cmd.append("-m")
    cmd.append(str(tracy_path))

    logger.info("Exporting Tracy profile to CSV: %s", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if result.stderr:
        logger.warning("csvexport stderr: %s", result.stderr.strip())

    if result.returncode != 0:
        logger.error("csvexport exited with code %d", result.returncode)
        return False

    csv_path.write_text(result.stdout)
    logger.info("CSV export written to %s", csv_path)
    return True


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
            "Run an Isaac Sim benchmark script with the Tracy profiler backend "
            "and simultaneously capture a .tracy profile."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples (run from the build directory):\n"
            "  # Select a benchmark by short name and forward its CLI args\n"
            "  ./python.sh tools/benchmark/tracy_capture.py \\\n"
            "      --benchmark camera \\\n"
            "      --output-dir /tmp/tracy_output \\\n"
            "      -- --num-cameras 2 --num-frames 300\n"
            "\n"
            "  # Full path also works\n"
            "  ./python.sh tools/benchmark/tracy_capture.py \\\n"
            "      --benchmark standalone_examples/benchmarks/benchmark_camera.py \\\n"
            "      -- --num-cameras 2\n"
            "\n"
            "  # List available benchmark short names\n"
            "  ./python.sh tools/benchmark/tracy_capture.py --list-benchmarks\n"
            "\n"
            "  # Also capture the loading phase\n"
            "  ./python.sh tools/benchmark/tracy_capture.py \\\n"
            "      --benchmark camera --capture-loading -- --num-cameras 2\n"
            "\n"
            "  # With Python-scope profiling (slower)\n"
            "  ./python.sh tools/benchmark/tracy_capture.py \\\n"
            "      --benchmark camera --enable-python-profiling\n"
            "\n"
            "Any arguments after '--' are forwarded directly to the benchmark script.\n"
        ),
    )
    add_common_arguments(parser)

    # Tracy-specific arguments.
    parser.add_argument(
        "--no-compress",
        action="store_true",
        help="Skip compressing the .tracy file (compression requires the 'update' binary).",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Export the captured .tracy profile to a CSV file using the 'csvexport' binary.",
    )
    parser.add_argument(
        "--csv-filter",
        default="",
        help="Zone name filter passed to csvexport (default: no filter).",
    )
    parser.add_argument(
        "--csv-self",
        action="store_true",
        help="Report self times instead of inclusive times in the CSV export.",
    )
    parser.add_argument(
        "--csv-unwrap",
        action="store_true",
        help="Report each individual zone event in the CSV export.",
    )
    parser.add_argument(
        "--csv-messages",
        action="store_true",
        help="Report only messages in the CSV export.",
    )

    return parse_common_args(parser, argv)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    """Entry point for the Tracy capture wrapper.

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

    # Output paths.
    output_dir, base_name, tracy_output = resolve_output_paths(
        args.output_dir, args.output_name, benchmark_script, ".tracy"
    )

    kit_log_file: Path | None = None
    if not args.no_kit_log_file:
        kit_log_file = (
            args.kit_log_path.resolve()
            if args.kit_log_path is not None
            else (output_dir / f"{base_name}_kit.log").resolve()
        )
        kit_log_file.parent.mkdir(parents=True, exist_ok=True)
        logger.info("Kit carb log (--/log/file): %s", kit_log_file)

    # Discover Tracy tools.
    try:
        tracy_ext = find_tracy_extension(release_dir)
        capture_bin = find_tracy_binary(tracy_ext, "capture")
    except FileNotFoundError as exc:
        logger.error("%s", exc)
        return 1

    update_bin = find_tracy_binary(tracy_ext, "update", required=False)
    csvexport_bin = find_tracy_binary(tracy_ext, "csvexport", required=False) if args.csv else None
    if args.csv and csvexport_bin is None:
        logger.error("--csv was requested but 'csvexport' binary not found in %s/bin", tracy_ext)
        return 1
    env = build_env(release_dir, extra_env=TRACY_ENV_DEFAULTS, enable_python_profiling=args.enable_python_profiling)

    # Build benchmark command.
    try:
        cmd = build_benchmark_command(
            release_dir,
            benchmark_script,
            args.extra_benchmark_args,
            metrics_output_dir=output_dir,
            kit_log_file=kit_log_file,
        )
    except FileNotFoundError as exc:
        logger.error("%s", exc)
        return 1

    # Log file.
    log_path = None if args.no_log else output_dir / f"{base_name}.log"

    # Capture + benchmark.
    capture_thread = TracyCaptureThread(capture_bin, tracy_output, env)

    if args.capture_loading:
        logger.info("Capture mode: full (including loading phase)")
        capture_thread.start()
        trigger = None
    else:
        logger.info("Capture mode: benchmark-phase only (waiting for '%s')", BENCHMARK_PHASE_TRIGGER)
        trigger = (BENCHMARK_PHASE_TRIGGER, capture_thread.start)

    exit_code = run_benchmark(cmd, env, on_trigger=trigger, log_path=log_path)

    # Wait for capture to finish.
    logger.info("Waiting for Tracy capture to complete (timeout=%.0fs)...", args.capture_timeout)
    capture_thread.wait(timeout=args.capture_timeout)

    if not capture_thread.output_exists:
        logger.warning("Tracy capture file was NOT created at %s", tracy_output)
        return exit_code or 1

    file_size_mb = tracy_output.stat().st_size / (1024 * 1024)
    logger.info("Tracy capture file: %s (%.1f MB)", tracy_output, file_size_mb)

    # Optional compression.
    final_tracy = tracy_output
    if not args.no_compress:
        if update_bin is not None:
            compressed_output = output_dir / f"{base_name}.compressed.tracy"
            if compress_tracy_file(update_bin, tracy_output, compressed_output, env):
                final_tracy = compressed_output
        else:
            logger.info(
                "Skipping compression — 'update' binary not found in %s/bin. "
                "The uncompressed .tracy file is still available.",
                tracy_ext,
            )

    # Optional CSV export.
    if csvexport_bin is not None:
        csv_output = output_dir / f"{base_name}.csv"
        export_tracy_to_csv(
            csvexport_bin,
            final_tracy,
            csv_output,
            env,
            filter_name=args.csv_filter,
            self_times=args.csv_self,
            unwrap=args.csv_unwrap,
            messages=args.csv_messages,
        )

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
