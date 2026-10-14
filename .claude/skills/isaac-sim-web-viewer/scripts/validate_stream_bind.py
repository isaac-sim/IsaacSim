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

"""Validate ISAACSIM_HOST and WebRTC ports before they reach Docker ARG/ENV."""

from __future__ import annotations

import argparse
import ipaddress
import json
import re
import sys

_HOSTNAME_RE = re.compile(
    r"^(?=.{1,253}$)"
    r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?"
    r"(?:\.[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?)*$"
)
_UNSAFE_HOST_CHARS = re.compile(r"""[\s"'\\;`$|&<>(){}\[\]#]""")


def is_valid_bind_host(host: str) -> bool:
    """Return True if host is a single IPv4 or DNS name safe to bind and bake."""
    if not host or _UNSAFE_HOST_CHARS.search(host) or "\n" in host or "\r" in host:
        return False
    if host in {"0.0.0.0", "*", "::", "[::]", "localhost"}:
        return False
    try:
        addr = ipaddress.IPv4Address(host)
    except ValueError:
        return bool(_HOSTNAME_RE.fullmatch(host))
    return not addr.is_unspecified and not addr.is_multicast


def is_valid_host_port(value: str) -> bool:
    """Return True if value is an integer TCP/UDP port in 1..65535."""
    if not re.fullmatch(r"[1-9][0-9]{0,4}", value or ""):
        return False
    return 1 <= int(value) <= 65535


def signaling_replacement(host: str, signal_port: int, stream_port: int) -> str:
    """Return the TS object fields to replace `signalingServer: '127.0.0.1',`."""
    host_lit = json.dumps(host)
    return (
        f"signalingServer: {host_lit},\n"
        f"    signalingPort: {signal_port},\n"
        f"    mediaServer: {host_lit},\n"
        f"    mediaPort: {stream_port},\n"
        f"    forceWSS: false,"
    )


def main(argv: list[str] | None = None) -> int:
    """CLI: exit 0 if --host / ports are bind-safe."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="", help="ISAACSIM_HOST after normalize")
    parser.add_argument("--signal-port", default="", help="ISAACSIM_SIGNAL_PORT")
    parser.add_argument("--stream-port", default="", help="ISAACSIM_STREAM_PORT")
    args = parser.parse_args(argv)
    if args.host and not is_valid_bind_host(args.host):
        print("ASK_USER_HOST: ISAACSIM_HOST must be a single IPv4 or DNS name (not 0.0.0.0).", file=sys.stderr)
        return 2
    if args.signal_port and not is_valid_host_port(args.signal_port):
        print("ASK_USER_PORT: ISAACSIM_SIGNAL_PORT is not a TCP port in 1..65535.", file=sys.stderr)
        return 2
    if args.stream_port and not is_valid_host_port(args.stream_port):
        print("ASK_USER_PORT: ISAACSIM_STREAM_PORT is not a UDP port in 1..65535.", file=sys.stderr)
        return 2
    if not (args.host or args.signal_port or args.stream_port):
        print("FAIL: pass --host and/or --signal-port / --stream-port", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
