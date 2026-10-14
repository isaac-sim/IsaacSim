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

"""Resolve current and legacy extension wheel staging manifests."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any


def read_manifest(path: Path) -> dict[str, Any]:
    """Read a staging manifest, accepting the legacy carrier table.

    Args:
        path: Current or legacy manifest path; a missing current path falls back to its legacy sibling.

    Returns:
        Canonical document containing the wheel table.
    """
    current = path.parent / "wheel-staging.toml"
    legacy = path.parent / "module-carrier.toml"
    documents = []
    for candidate, table in ((current, "wheel"), (legacy, "carrier")):
        if candidate.is_file():
            with candidate.open("rb") as stream:
                document = tomllib.load(stream)
            documents.append((candidate, document, document.get(table)))
    if not documents:
        raise RuntimeError(f"Wheel staging manifest is missing: {current}")
    if len(documents) == 2 and documents[0][2] != documents[1][2]:
        raise RuntimeError(f"Conflicting staging manifests: {current} and {legacy}")
    _, document, wheel = documents[0]
    return {**document, "wheel": wheel}


def find_manifests(extension_root: Path) -> tuple[Path, ...]:
    """Find one staging manifest per extension, preferring the current spelling.

    Args:
        extension_root: Parent directory containing extension source directories.

    Returns:
        Sorted manifest paths, without duplicate extensions.
    """
    roots = {
        path.parent
        for pattern in ("*/wheel-staging.toml", "*/module-carrier.toml")
        for path in extension_root.glob(pattern)
    }
    return tuple(
        root / ("wheel-staging.toml" if (root / "wheel-staging.toml").is_file() else "module-carrier.toml")
        for root in sorted(roots)
    )
