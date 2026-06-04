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

"""TCP bridge for the isaacsim.examples.ipc tutorial with **On Playback Tick** pacing.

Use with a graph wired like::

    On Playback Tick → Receive External Step → Send Simulation Clock

where **Isaac Read Simulation Time** ``simulationTime`` feeds **Send Simulation Clock**
``simulationTime``, and the node **.ogn** defaults match this script (Receive listens on
``127.0.0.1:9001``, Send connects to ``127.0.0.1:9000``).

This process:

1. Listens for **Send Simulation Clock** (TCP client) on ``--clock-host`` / ``--clock-port``.
2. Connects to **Receive External Step** (TCP server) on ``--step-host`` / ``--step-port``.
3. Sends an initial 4-byte **step** (uint32 LE) so the first playback tick can complete
   Receive → Send; **Send** then connects to the clock listener.
4. Each iteration: reads one **8-byte clock** (int64 LE nanoseconds), then sends the next
   **step** so the following playback tick can proceed.

**Startup order:** start **playback** in Isaac Sim first so **Receive External Step** is
listening, then run this script, then let the timeline advance (playback ticks).

Matches the wire formats in the Isaac Sim user guide (IPC OmniGraph nodes).
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys


class ClockPeerDisconnected(Exception):
    """Clock TCP peer closed the connection or reset (typical when simulation stops)."""

    __slots__ = ("bytes_received", "expected")

    def __init__(self, bytes_received: int, expected: int) -> None:
        self.bytes_received = bytes_received
        self.expected = expected


def _recv_exact(conn: socket.socket, n: int) -> bytes:
    chunks: list[bytes] = []
    got = 0
    while got < n:
        try:
            part = conn.recv(n - got)
        except (BrokenPipeError, ConnectionResetError):
            raise ClockPeerDisconnected(got, n) from None
        if not part:
            raise ClockPeerDisconnected(got, n)
        chunks.append(part)
        got += len(part)
    return b"".join(chunks)


def main(argv: list[str] | None = None) -> int:
    """Run the tutorial TCP playback bridge."""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--clock-host",
        default="127.0.0.1",
        help="Bind address for the clock listener (Send Simulation Clock connects here).",
    )
    p.add_argument(
        "--clock-port",
        type=int,
        default=9000,
        help="Listen port for the 8-byte clock (default: 9000, matches Send node default uri).",
    )
    p.add_argument(
        "--step-host",
        default="127.0.0.1",
        help="Host where Receive External Step listens.",
    )
    p.add_argument(
        "--step-port",
        type=int,
        default=9001,
        help="Receive External Step listen port (default: 9001).",
    )
    p.add_argument(
        "--initial-step",
        type=int,
        default=1,
        help="First uint32 step sent before the first clock (default: 1).",
    )
    p.add_argument(
        "--step-delta",
        type=int,
        default=1,
        help="Added to the step value after each clock (mod 2**32). Default: 1.",
    )
    p.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Exit after this many clock messages (default: run until Ctrl+C).",
    )
    args = p.parse_args(argv)

    if args.initial_step < 0 or args.initial_step > 0xFFFFFFFF:
        print("--initial-step must fit in uint32", file=sys.stderr)
        return 1
    if args.step_delta < 0 or args.step_delta > 0xFFFFFFFF:
        print("--step-delta must fit in uint32", file=sys.stderr)
        return 1

    clock_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clock_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    clock_srv.bind((args.clock_host, args.clock_port))
    clock_srv.listen(1)

    step_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clock_conn: socket.socket | None = None
    try:
        print(
            f"Listening for Send Simulation Clock on {args.clock_host}:{args.clock_port}…",
            flush=True,
        )
        print(
            f"Connecting to Receive External Step at {args.step_host}:{args.step_port}…",
            flush=True,
        )
        try:
            step_sock.connect((args.step_host, args.step_port))
        except ConnectionRefusedError:
            print(
                "No connection: Receive External Step is not listening yet "
                f"({args.step_host}:{args.step_port}). "
                "Start simulation / playback in Isaac Sim first, then run this script again.",
                flush=True,
            )
            return 0

        step_val = args.initial_step & 0xFFFFFFFF
        step_sock.sendall(struct.pack("<I", step_val))
        print(
            f"Primed step={step_val}; waiting for Send Simulation Clock to connect…",
            flush=True,
        )

        clock_conn, peer = clock_srv.accept()
        print(f"Clock TCP peer {peer}", flush=True)

        frame = 0
        while args.max_frames is None or frame < args.max_frames:
            try:
                data = _recv_exact(clock_conn, 8)
            except ClockPeerDisconnected as e:
                if e.bytes_received == 0:
                    print(
                        "No clock connection: peer closed (simulation stopped or Send Simulation Clock disconnected).",
                        flush=True,
                    )
                else:
                    print(
                        "No clock connection: peer closed mid-message "
                        f"({e.bytes_received} of {e.expected} bytes; simulation stopped or graph unloaded).",
                        flush=True,
                    )
                return 0
            (t_ns,) = struct.unpack("<q", data)
            print(f"frame={frame} time_ns={t_ns}", flush=True)
            frame += 1
            if args.max_frames is not None and frame >= args.max_frames:
                break
            step_val = (step_val + args.step_delta) & 0xFFFFFFFF
            try:
                step_sock.sendall(struct.pack("<I", step_val))
            except BrokenPipeError:
                print(
                    "No step connection: peer closed (simulation stopped or Receive External Step disconnected).",
                    flush=True,
                )
                return 0

        return 0
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except OSError as e:
        print(e, file=sys.stderr)
        return 1
    finally:
        if clock_conn is not None:
            try:
                clock_conn.close()
            except OSError:
                pass
        try:
            step_sock.close()
        except OSError:
            pass
        try:
            clock_srv.close()
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
