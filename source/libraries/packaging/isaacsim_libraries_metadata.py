# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Provide shared-version internal requirements for library package metadata."""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

_DISTRIBUTION_NAME_PATTERN = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9_.-]*[A-Za-z0-9])?$")
_VERSION_PATTERN = re.compile(
    r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)"
    r"((a|b|rc)(0|[1-9][0-9]*))?(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
    r"(\+([0-9]*[a-z][a-z0-9]*|(0|[1-9][0-9]*))(\.([0-9]*[a-z][a-z0-9]*|(0|[1-9][0-9]*)))*)?$"
)
_VERSION_FILE = Path(__file__).resolve().parents[1] / "VERSION"


def _normalize_distribution_name(name: str) -> str:
    """Normalize a Python distribution name for identity comparisons.

    Args:
        name: Distribution name to normalize.

    Returns:
        Normalized distribution name.
    """
    return re.sub(r"[-_.]+", "-", name).lower()


def _read_shared_version() -> str:
    """Read the canonical shared library version.

    Returns:
        Canonical shared library version.

    Raises:
        RuntimeError: If the version file cannot be read or contains an invalid version.
    """
    try:
        version = _VERSION_FILE.read_text(encoding="utf-8").strip()
    except OSError as error:
        raise RuntimeError(f"Cannot read shared library version from {_VERSION_FILE}: {error}") from error
    if _VERSION_PATTERN.fullmatch(version) is None:
        raise RuntimeError(f"Invalid canonical PEP 440 library version: {version}")
    return version


def dynamic_metadata(settings: Mapping[str, Any], _project: Mapping[str, Any]) -> dict[str, list[str]]:
    """Generate exact internal requirements from the shared library version.

    Args:
        settings: Dynamic metadata settings declared in ``pyproject.toml``.
        _project: Project metadata resolved before this provider runs.

    Returns:
        A dynamic ``dependencies`` fragment for scikit-build-core.

    Raises:
        RuntimeError: If the provider settings or shared version are invalid.
    """
    expected_keys = {"field", "names"}
    unknown_keys = settings.keys() - expected_keys
    if unknown_keys:
        raise RuntimeError(f"Unknown shared-version metadata settings: {', '.join(sorted(unknown_keys))}")
    if settings.get("field") != "dependencies":
        raise RuntimeError("Shared-version metadata provider field must be 'dependencies'")
    names = settings.get("names")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) for name in names):
        raise RuntimeError("Shared-version metadata provider names must be a nonempty list of strings")
    invalid_names = [name for name in names if _DISTRIBUTION_NAME_PATTERN.fullmatch(name) is None]
    if invalid_names:
        raise RuntimeError(f"Invalid internal distribution names: {', '.join(invalid_names)}")
    normalized_names = [_normalize_distribution_name(name) for name in names]
    if len(normalized_names) != len(set(normalized_names)):
        raise RuntimeError("Shared-version metadata provider names must be unique")
    version = _read_shared_version()
    return {"dependencies": [f"{name}=={version}" for name in names]}
