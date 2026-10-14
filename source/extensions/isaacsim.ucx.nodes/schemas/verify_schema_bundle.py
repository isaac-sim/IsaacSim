# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0
"""Verify the versioned UCX schema bundle against its recorded source hashes."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


def schema_tokens(source: str) -> list[str]:
    """Tokenize a FlatBuffers schema for declaration-level comparison.

    Splits ``source`` into identifiers, integer literals, quoted string
    literals, and single-character punctuation. Line (``//``) and block
    (``/* */``) comments are matched by the regex so any comment characters
    inside a quoted string are preserved with the string; the matched
    comment tokens themselves are then dropped. Whitespace is discarded.
    The resulting list can be compared byte-for-byte to detect any
    declaration difference between two schemas while tolerating comment
    and formatting divergence.

    Args:
        source: Raw text of a ``.fbs`` schema file.

    Returns:
        List of significant tokens in the order they appear in ``source``,
        excluding whitespace and comments.
    """
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*[\s\S]*?\*/|[A-Za-z_][A-Za-z_0-9]*|[0-9]+|[^\s]', source)
    return [token for token in tokens if not token.startswith(("//", "/*"))]


def main() -> None:
    """Verify a UCX schema bundle against its recorded ``manifest.json``.

    Reads ``<schemas>/manifest.json``, which pins the expected SHA-256 of
    every schema file the extension ships plus (when ``sdk_sha256`` is
    populated) the SHA-256 of the corresponding upstream Holoscan SDK
    source. For every entry in ``manifest["sha256"]``:

    * The local file under ``--schemas`` (or its ``isaac_``-prefixed name
      for ``joint_*.fbs``) must hash to the recorded value.
    * When ``--sdk-schemas`` is provided, the SDK-installed file must hash
      to the recorded ``sdk_sha256``, and its declaration tokens (via
      :func:`schema_tokens`) must match the local copy.
    * When ``--peer-schemas`` is provided, the peer copy of each schema
      must hash to the same recorded value, and the peer's own
      ``manifest.json`` must be byte-identical to this one.

    Exits with a non-zero status by raising ``ValueError`` on the first
    mismatch, with a message identifying the offending file and the check
    that failed. On success, prints a one-line confirmation naming the
    verified protocol version and the SDK revision recorded in the
    manifest.

    Command-line arguments (all read from ``argparse``):
        --schemas (Path, required): Directory containing the extension's
            schema bundle. Must contain ``manifest.json``.
        --sdk-schemas (Path, optional): Directory containing the installed
            Holoscan SDK schemas to cross-check against ``sdk_sha256``.
        --peer-schemas (Path, optional): Directory containing the peer
            project's copy of the same schema bundle.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schemas", type=Path, required=True)
    parser.add_argument("--sdk-schemas", type=Path)
    parser.add_argument("--peer-schemas", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.schemas / "manifest.json").read_text())
    if args.peer_schemas:
        peer_manifest = json.loads((args.peer_schemas / "manifest.json").read_text())
        if peer_manifest != manifest:
            raise ValueError("Peer schema manifest differs from the UCX protocol manifest.")
    for name, expected in manifest["sha256"].items():
        local = args.schemas / name
        if not local.exists() and name.startswith("joint_"):
            local = args.schemas / ("isaac_" + name)
        if args.sdk_schemas and not name.startswith("joint_"):
            sdk = args.sdk_schemas / name
            if not sdk.exists():
                sdk = args.sdk_schemas / "sensor_io" / name
            if hashlib.sha256(sdk.read_bytes()).hexdigest() != manifest["sdk_sha256"][name]:
                raise ValueError(f"Installed SDK schema {name} differs from the pinned upstream source.")
            if schema_tokens(sdk.read_text()) != schema_tokens(local.read_text()):
                raise ValueError(f"Schema {name} declarations differ from the installed SDK.")
        if hashlib.sha256(local.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Schema {name} differs from the UCX protocol manifest.")
        if args.peer_schemas:
            peer = args.peer_schemas / name
            if not peer.exists() and name.startswith("joint_"):
                peer = args.peer_schemas / ("isaac_" + name)
            # The OS build independently verifies SDK-provided files against this manifest.
            if not peer.exists() or hashlib.sha256(peer.read_bytes()).hexdigest() != expected:
                raise ValueError(f"Peer schema {name} differs from the UCX protocol manifest.")
    print(f"Verified UCX protocol {manifest['protocol_version']} schemas (SDK {manifest['sdk_revision']}).")


if __name__ == "__main__":
    main()
