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

"""Validate Isaac Sim binary, Python, or container installation."""

from __future__ import annotations

import argparse
import os
import platform
import queue
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Callable

# Only inherit env vars needed to validate Isaac Sim. Other host
# secrets are deliberately not forwarded to the subprocess.
_SAFE_ENV_KEYS = frozenset(
    {
        "PATH",
        "HOME",
        "USER",
        "LOGNAME",
        "LANG",
        "LC_ALL",
        "LC_CTYPE",
        "TMPDIR",
        "TEMP",
        "TMP",
        "SYSTEMROOT",
        "SYSTEMDRIVE",
        "WINDIR",
        "COMSPEC",
        "ALLUSERSPROFILE",
        "COMMONPROGRAMFILES",
        "COMMONPROGRAMFILES(X86)",
        "COMMONPROGRAMW6432",
        "COMPUTERNAME",
        "DRIVERDATA",
        "HOMEDRIVE",
        "HOMEPATH",
        "NUMBER_OF_PROCESSORS",
        "OS",
        "PATHEXT",
        "PROCESSOR_ARCHITECTURE",
        "PROCESSOR_IDENTIFIER",
        "PROCESSOR_LEVEL",
        "PROCESSOR_REVISION",
        "PROGRAMDATA",
        "PROGRAMFILES",
        "PROGRAMFILES(X86)",
        "PROGRAMW6432",
        "PUBLIC",
        "USERDOMAIN",
        "USERNAME",
        "USERPROFILE",
        "APPDATA",
        "LOCALAPPDATA",
        "DISPLAY",
        "WAYLAND_DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "LD_LIBRARY_PATH",
        "VIRTUAL_ENV",
        "SSL_CERT_FILE",
        "SSL_CERT_DIR",
        "REQUESTS_CA_BUNDLE",
        "CURL_CA_BUNDLE",
        "HTTP_PROXY",
        "HTTPS_PROXY",
        "NO_PROXY",
        "http_proxy",
        "https_proxy",
        "no_proxy",
    }
)


def _safe_env(extra: dict[str, str] | None = None) -> dict[str, str]:
    # Build the subprocess environment by pulling each allowlisted variable
    # via os.getenv individually. This deliberately avoids enumerating the
    # parent environment table so unrelated host secrets are never touched.
    filtered: dict[str, str] = {}
    for key in _SAFE_ENV_KEYS:
        value = os.getenv(key)
        if value is not None:
            filtered[key] = value
    if extra:
        filtered.update(extra)
    return filtered


def print_command(command: list[str]) -> None:
    if platform.system().lower().startswith("win"):
        print(subprocess.list2cmdline(command))
    else:
        print(shlex.join(command))


def run(command: list[str], env: dict[str, str] | None = None) -> int:
    print_command(command)
    return subprocess.run(command, env=_safe_env(env), check=False).returncode


_COMPATIBILITY_PASS_MARKER = "System checking result: PASSED"
_COMPATIBILITY_FAIL_MARKER = "System checking result: FAILED"
_COMPATIBILITY_FATAL_PATTERNS = (
    re.compile(r"ModuleNotFoundError", re.IGNORECASE),
    re.compile(r"Failed to startup python extension", re.IGNORECASE),
    re.compile(r"Failed to import python module", re.IGNORECASE),
    re.compile(r"Segmentation fault", re.IGNORECASE),
    re.compile(r"Fatal Python error", re.IGNORECASE),
    re.compile(r"Aborted(?:\s+\(core dumped\))?", re.IGNORECASE),
)
DEFAULT_COMPATIBILITY_TIMEOUT_SECONDS = 600


def stop_process_tree(process: subprocess.Popen[str]) -> None:
    """Stop a checker that stays open after printing its semantic result."""
    if process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            env=_safe_env(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


def wait_for_process_stop(process: subprocess.Popen[str], timeout: int = 10) -> int:
    """Wait for a stopped process, escalating to a forced stop if needed."""
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        if os.name == "nt":
            process.kill()
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        return process.wait()


def run_compatibility(
    command: list[str],
    env: dict[str, str] | None = None,
    *,
    stop_after_result: bool = False,
    timeout_seconds: float = DEFAULT_COMPATIBILITY_TIMEOUT_SECONDS,
    cleanup: Callable[[], None] | None = None,
) -> int:
    """Run the checker and require its semantic pass marker, not just exit code 0."""
    if timeout_seconds <= 0:
        raise ValueError("Compatibility Checker timeout must be greater than zero.")
    print_command(command)
    saw_pass = False
    saw_result = False
    stopped_after_result = False
    timed_out = False
    first_fatal: str | None = None
    output_queue: queue.Queue[str | None] = queue.Queue()

    with subprocess.Popen(
        command,
        env=_safe_env(env),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        bufsize=1,
        start_new_session=os.name != "nt",
    ) as process:
        assert process.stdout is not None
        stdout = process.stdout

        def read_output() -> None:
            try:
                for output_line in stdout:
                    output_queue.put(output_line)
            finally:
                output_queue.put(None)

        reader = threading.Thread(target=read_output, name="isaac-compatibility-output", daemon=True)
        reader.start()
        deadline = time.monotonic() + timeout_seconds

        def run_cleanup() -> None:
            if cleanup is None:
                return
            try:
                cleanup()
            except OSError as exc:
                print(f"Compatibility Checker cleanup warning: {exc}")

        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    timed_out = True
                    stop_process_tree(process)
                    break
                try:
                    line = output_queue.get(timeout=min(0.25, remaining))
                except queue.Empty:
                    if process.poll() is not None and not reader.is_alive():
                        break
                    continue
                if line is None:
                    break
                print(line, end="")
                if _COMPATIBILITY_PASS_MARKER in line:
                    saw_pass = True
                    saw_result = True
                elif _COMPATIBILITY_FAIL_MARKER in line:
                    saw_result = True
                if first_fatal is None and any(pattern.search(line) for pattern in _COMPATIBILITY_FATAL_PATTERNS):
                    first_fatal = line.strip()
                if stop_after_result and saw_result:
                    stop_process_tree(process)
                    stopped_after_result = True
                    break
        except KeyboardInterrupt:
            stop_process_tree(process)
            run_cleanup()
            wait_for_process_stop(process)
            raise

        if timed_out or stopped_after_result:
            run_cleanup()
        returncode = wait_for_process_stop(process)
        reader.join(timeout=1)
        stdout.close()
        reader.join(timeout=1)

    if timed_out:
        print("Compatibility Checker failed: exceeded the wall-clock timeout " f"of {timeout_seconds:g} seconds.")
        return 124
    if returncode != 0 and not stopped_after_result:
        print(f"Compatibility Checker failed: process exited with status {returncode}.")
        return returncode
    if first_fatal is not None:
        print(f"Compatibility Checker failed: {first_fatal}")
        return 1
    if not saw_pass:
        print(
            "Compatibility Checker failed: process exited successfully but did not report "
            f"'{_COMPATIBILITY_PASS_MARKER}'."
        )
        return 1

    print("Compatibility Checker result verified: PASSED")
    return 0


def require_container_host() -> None:
    system = platform.system().lower()
    release = platform.release().lower()
    if system != "linux" or "microsoft" in release or "wsl" in release:
        raise RuntimeError(
            "Isaac Sim container validation is supported only on native Linux hosts "
            "(x86_64 or aarch64); Windows and WSL are not supported."
        )
    machine = platform.machine().lower()
    if machine not in {"amd64", "x86_64", "aarch64", "arm64"}:
        raise RuntimeError(
            f"Unsupported Isaac Sim container host architecture: {machine}. "
            "Only Linux x86_64 and Linux aarch64 are supported."
        )


def workstation_checker_path(install_dir: Path) -> Path:
    system = platform.system().lower()
    if system.startswith("win"):
        return install_dir / "isaac-sim.compatibility_check.bat"
    if system == "linux":
        return install_dir / "isaac-sim.compatibility_check.sh"
    raise RuntimeError(
        f"Isaac Sim workstation Compatibility Checker is unsupported on {platform.system()}. "
        "Use Windows x86_64, Linux x86_64, or supported Linux aarch64."
    )


def workstation_checker_command(install_dir: Path) -> list[str]:
    system = platform.system().lower()
    checker = workstation_checker_path(install_dir)
    if system.startswith("win"):
        return [
            "cmd",
            "/c",
            str(checker),
            "--no-window",
        ]
    if system == "linux":
        return ["bash", str(checker)]
    raise AssertionError("workstation_checker_path accepted an unsupported platform")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Isaac Sim install.")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    binary_parser = subparsers.add_parser("binary")
    binary_parser.add_argument("--install-dir", default="~/isaacsim")
    binary_parser.add_argument("--execute", action="store_true")

    python_parser = subparsers.add_parser("python")
    python_parser.add_argument("--python", default="python")
    python_parser.add_argument("--execute", action="store_true")

    workstation_parser = subparsers.add_parser("workstation-compatibility")
    workstation_parser.add_argument("--install-dir", default="~/isaacsim")
    workstation_parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_COMPATIBILITY_TIMEOUT_SECONDS,
    )
    workstation_parser.add_argument("--execute", action="store_true")

    python_compatibility_parser = subparsers.add_parser("python-compatibility")
    python_compatibility_parser.add_argument(
        "--isaacsim",
        default="isaacsim",
        help="Path to the isaacsim console command in the Python 3.12 environment.",
    )
    python_compatibility_parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_COMPATIBILITY_TIMEOUT_SECONDS,
    )
    python_compatibility_parser.add_argument("--execute", action="store_true")

    container_parser = subparsers.add_parser("container")
    container_parser.add_argument("--image", default="nvcr.io/nvidia/isaac-sim:6.2.0")
    container_parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_COMPATIBILITY_TIMEOUT_SECONDS,
    )
    container_parser.add_argument("--execute", action="store_true")

    args = parser.parse_args()

    if args.mode == "binary":
        install_dir = Path(args.install_dir).expanduser()
        launcher = install_dir / ("isaac-sim.bat" if os.name == "nt" else "isaac-sim.sh")
        print(f"# Inspect installation directory without launching: {install_dir}")
        print(f"# Expected packaged launcher file: {launcher}")
        if not args.execute:
            print("Dry-run only. Re-run with --execute to inspect files without launching Isaac Sim.")
            return 0
        if not install_dir.is_dir():
            raise FileNotFoundError(f"Install directory not found: {install_dir}")
        if not launcher.exists():
            raise FileNotFoundError(f"Launcher not found: {launcher}")
        print(f"Isaac Sim standalone files found at: {install_dir.resolve()}")
        print("Isaac Sim was not launched.")
        return 0

    if args.mode == "python":
        python_code = (
            "import importlib.metadata as m; " "print('isaacsim distribution version:', m.version('isaacsim'))"
        )
        command = [args.python, "-c", python_code]
        print_command(command)
        if not args.execute:
            print("Dry-run only. Re-run with --execute to inspect package metadata without importing Isaac Sim.")
            return 0
        result = run(command)
        if result == 0:
            print("Isaac Sim was not imported or launched.")
        return result

    if args.mode == "workstation-compatibility":
        install_dir = Path(args.install_dir).expanduser()
        command = workstation_checker_command(install_dir)
        checker = workstation_checker_path(install_dir)
        print("# Workstation Compatibility Checker; supported on Windows and Linux.")
        print_command(command)
        if not args.execute:
            print("Dry-run only. Re-run with --execute to run the Compatibility Checker.")
            return 0
        if not checker.is_file():
            raise FileNotFoundError(f"Compatibility Checker script not found: {checker}")
        return run_compatibility(
            command,
            {"OMNI_KIT_ACCEPT_EULA": "YES"},
            stop_after_result=True,
            timeout_seconds=args.timeout_seconds,
        )

    if args.mode == "python-compatibility":
        command = [
            args.isaacsim,
            "isaacsim.exp.compatibility_check",
        ]
        is_windows = platform.system().lower().startswith("win")
        if is_windows:
            command.append("--no-window")
        else:
            command.extend(["--/app/quitAfter=10", "--no-window"])
        print("# Python-package Compatibility Checker; supported on Windows and Linux.")
        print_command(command)
        if not args.execute:
            print("Dry-run only. Re-run with --execute to run the Compatibility Checker.")
            return 0
        return run_compatibility(
            command,
            {"OMNI_KIT_ACCEPT_EULA": "YES"},
            stop_after_result=is_windows,
            timeout_seconds=args.timeout_seconds,
        )

    try:
        require_container_host()
    except RuntimeError as exc:
        parser.error(str(exc))

    allowed_registries = ("nvcr.io/nvidia/", "nvcr.io/nvidia-omniverse/")
    if not any(args.image.startswith(prefix) for prefix in allowed_registries):
        print(
            f"# SECURITY WARNING: image '{args.image}' is not from an approved NVIDIA "
            f"registry ({', '.join(allowed_registries)}). Verify the image source "
            "before running with --gpus all."
        )

    container_name = f"isaac-sim-compatibility-{uuid.uuid4().hex[:12]}"
    command = [
        "docker",
        "run",
        "--name",
        container_name,
        "--entrypoint",
        "bash",
        "-i",
        "--gpus",
        "all",
        "-e",
        "ACCEPT_EULA=Y",
        "--rm",
    ]
    command += ["--network=bridge"]
    print("# Networking: bridge. Preserves Docker isolation for the compatibility check.")
    command += [
        args.image,
        "./isaac-sim.compatibility_check.sh",
        "--/app/quitAfter=10",
        "--no-window",
    ]
    print_command(command)
    if not args.execute:
        print("Dry-run only. Re-run with --execute to run the container compatibility checker.")
        return 0

    def cleanup_container() -> None:
        subprocess.run(
            ["docker", "rm", "--force", container_name],
            env=_safe_env(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    return run_compatibility(
        command,
        stop_after_result=True,
        timeout_seconds=args.timeout_seconds,
        cleanup=cleanup_container,
    )


if __name__ == "__main__":
    raise SystemExit(main())
