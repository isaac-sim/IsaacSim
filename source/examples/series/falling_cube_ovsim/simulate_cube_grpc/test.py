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

"""Run the gRPC falling-cube example against a temporary loopback server."""

from __future__ import annotations

import queue
import shutil
import subprocess
import threading
import time
from typing import TextIO

import main as example

_READY_MARKER = "OV SIM gRPC server listening on "
_START_TIMEOUT_SECONDS = 30.0
_SHUTDOWN_TIMEOUT_SECONDS = 10.0


def _forward_server_output(stream: TextIO, messages: queue.Queue[str], output: list[str]) -> None:
    """Forward server output while making readiness messages available to the test.

    Args:
        stream: Captured server output stream.
        messages: Queue receiving complete output lines.
        output: Accumulated output used in startup failure messages.
    """
    for line in stream:
        output.append(line)
        messages.put(line)
        print(line, end="")


def _wait_for_endpoint(server: subprocess.Popen[str], messages: queue.Queue[str], output: list[str]) -> str:
    """Wait for the server to report its dynamically selected loopback endpoint.

    Args:
        server: Running server process.
        messages: Queue populated by the output reader.
        output: Accumulated server output used when startup fails.

    Returns:
        Bound loopback endpoint reported by the server.
    """
    deadline = time.monotonic() + _START_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        try:
            line = messages.get(timeout=0.1)
        except queue.Empty:
            if server.poll() is not None:
                break
            continue
        marker_index = line.find(_READY_MARKER)
        if marker_index >= 0:
            return line[marker_index + len(_READY_MARKER) :].strip()
    details = "".join(output).strip() or "no server output"
    raise RuntimeError(f"The OV SIM gRPC server did not become ready: {details}")


def main() -> None:
    """Verify that a loopback OV SIM server advances the cube simulation."""
    server_executable = shutil.which("isaacsim-ovsim-grpc-server")
    if server_executable is None:
        raise RuntimeError("isaacsim-ovsim-grpc-server is not available on PATH.")

    server = subprocess.Popen(
        [server_executable, "--listen-address", "127.0.0.1:0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if server.stdout is None:
        raise RuntimeError("Could not capture OV SIM gRPC server output.")
    messages: queue.Queue[str] = queue.Queue()
    output: list[str] = []
    reader = threading.Thread(
        target=_forward_server_output,
        args=(server.stdout, messages, output),
        name="ovsim-grpc-server-output",
        daemon=True,
    )
    reader.start()

    try:
        endpoint = _wait_for_endpoint(server, messages, output)
        initial_height, final_height = example.main(endpoint)
        if final_height >= initial_height:
            raise RuntimeError("The remotely simulated cube did not fall.")
        print("Loopback gRPC falling cube test passed.")
    finally:
        if server.poll() is None:
            server.terminate()
        try:
            server.wait(timeout=_SHUTDOWN_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait(timeout=_SHUTDOWN_TIMEOUT_SECONDS)
        reader.join(timeout=_SHUTDOWN_TIMEOUT_SECONDS)


if __name__ == "__main__":
    main()
