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

"""Plan or execute an installation-only Isaac Sim container image pull."""

from __future__ import annotations

import argparse
import platform
import shlex
import subprocess

_ALLOWED_IMAGE_REGISTRIES = ("nvcr.io/nvidia/", "nvcr.io/nvidia-omniverse/")


def print_command(command: list[str]) -> None:
    print(shlex.join(command))


def run(command: list[str], *, capture_output: bool = False) -> subprocess.CompletedProcess[str]:
    print_command(command)
    return subprocess.run(
        command,
        check=True,
        text=True,
        capture_output=capture_output,
    )


def require_supported_host() -> None:
    system = platform.system().lower()
    release = platform.release().lower()
    if system != "linux" or "microsoft" in release or "wsl" in release:
        raise RuntimeError(
            "Isaac Sim container installation is supported only on native Linux hosts "
            "(x86_64 or aarch64); Windows and WSL are not supported."
        )
    machine = platform.machine().lower()
    if machine not in {"amd64", "x86_64", "aarch64", "arm64"}:
        raise RuntimeError(
            f"Unsupported Isaac Sim container host architecture: {machine}. "
            "Only Linux x86_64 and Linux aarch64 are supported."
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Pull an Isaac Sim container image without creating or starting a container. "
            "This installer never invokes runheadless.sh."
        )
    )
    parser.add_argument("--image", default="nvcr.io/nvidia/isaac-sim:6.2.0")
    parser.add_argument("--execute", action="store_true", help="Pull the image. Default is dry-run.")
    args = parser.parse_args()

    # Reject unsupported hosts before printing even a dry-run plan. A Docker
    # Desktop plan on Windows/WSL would look actionable although Isaac Sim
    # containers are not supported there.
    try:
        require_supported_host()
    except RuntimeError as exc:
        parser.error(str(exc))

    print("Isaac Sim container installation plan:")
    print("# Installation-only boundary: pull and inspect the image; do not start a container.")
    print("# No Isaac Sim launcher, runheadless.sh, or warmup.sh will be invoked.")
    print("# Precondition: run preflight and the Compatibility Checker before installation.")
    if not any(args.image.startswith(prefix) for prefix in _ALLOWED_IMAGE_REGISTRIES):
        print(
            f"# SECURITY WARNING: image '{args.image}' is not from an approved NVIDIA "
            f"registry ({', '.join(_ALLOWED_IMAGE_REGISTRIES)}). Verify the image "
            "source before pulling."
        )
    print_command(["docker", "pull", args.image])
    print_command(["docker", "image", "inspect", "--format", "{{.Id}}", args.image])
    print_command(["docker", "info", "--format", "{{.DockerRootDir}}"])

    if not args.execute:
        print("\nDry-run only. Re-run with --execute after confirming the image and EULA.")
        return 0

    run(["docker", "pull", args.image])
    image_id = run(
        ["docker", "image", "inspect", "--format", "{{.Id}}", args.image],
        capture_output=True,
    ).stdout.strip()
    docker_root = run(
        ["docker", "info", "--format", "{{.DockerRootDir}}"],
        capture_output=True,
    ).stdout.strip()
    print("Isaac Sim container image installed successfully.")
    print(f"Image reference: {args.image}")
    print(f"Image ID: {image_id}")
    print(f"Docker-managed storage root: {docker_root}")
    print("No container was created or started.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
