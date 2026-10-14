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

"""Resolve portable asset and grasp-map locations in floating-gripper definitions."""

from __future__ import annotations

import re
from pathlib import Path

_URI_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.\-]*://")


def _resolve_definition_relative_asset_path(asset_path: str, base_dir: Path | None) -> tuple[str, str | None]:
    """Resolve an existing definition-relative file, retaining the attempted location."""
    from isaacsim.storage.native import is_absolute_path

    if base_dir is None or _URI_SCHEME.match(asset_path) or is_absolute_path(asset_path):
        return asset_path, None
    candidate = base_dir.expanduser().joinpath(asset_path).resolve()
    return (str(candidate) if candidate.is_file() else asset_path), str(candidate)


def _resolve_definition_grasp_config_path(config_path: str, base_dir: Path) -> str:
    """Resolve a local joint map beside its definition, preserving built-in and absolute paths."""
    from isaacsim.storage.native import is_absolute_path

    if config_path.startswith("builtin://") or is_absolute_path(config_path):
        return config_path
    candidate = base_dir.expanduser().joinpath(config_path).resolve()
    if not candidate.is_file():
        raise ValueError(f"Gripper grasp_controller.config_path '{config_path}' does not exist at '{candidate}'")
    return str(candidate)


async def resolve_floating_gripper_asset_path_async(
    asset_path: str,
    *,
    assets_root: str | None = None,
    base_dir: str | Path | None = None,
    verify: bool = True,
) -> str:
    """Resolve a gripper asset URI, local file, definition-relative file, or assets-root fragment.

    Resolution prefers a URI with a scheme, then an existing absolute local file,
    then an existing file relative to ``base_dir``. Anything else is appended to
    the configured assets root. This preserves fragments such as
    ``/Isaac/Robots/Ufactory/xarm_gripper/xarm_gripper.usd``, which also look like
    absolute POSIX paths. Missing local files therefore fall through to the root;
    use a ``file://`` URI to explicitly require a local location.

    The assets root is queried only for fragments. Relative paths use the
    definition directory, never the process working directory. Without
    ``base_dir``, a relative path is an assets-root fragment. Verification checks
    the resulting location, not its transitive USD references or physics setup.

    Args:
        asset_path: Location authored in the definition.
        assets_root: Explicit asset root, or None to resolve the configured root on demand.
        base_dir: Definition's local directory, or None when it has no source directory.
        verify: Check accessibility of the resolved location. Local existence checks
            used to distinguish the four forms still run when this is False.

    Returns:
        Location suitable for a USD reference.

    Raises:
        ValueError: If the authored path is empty or verification fails.
        RuntimeError: If an assets-root fragment needs an unavailable root.

    Example:

    .. code-block:: python

        >>> from isaacsim.replicator.teleop import resolve_floating_gripper_asset_path_async

        >>> print(await resolve_floating_gripper_asset_path_async(  # doctest: +NO_CHECK
        ...     "overlay.usda", base_dir="/data/my_gripper"))
        /data/my_gripper/overlay.usda
    """
    from isaacsim.storage.native import get_assets_root_path_async, is_absolute_path, path_exists, path_join

    authored = asset_path.strip()
    if not authored:
        raise ValueError("Gripper 'asset_path' is empty")
    candidate, definition_relative = _resolve_definition_relative_asset_path(
        authored, None if base_dir is None else Path(base_dir)
    )
    if _URI_SCHEME.match(candidate) or (is_absolute_path(candidate) and Path(candidate).is_file()):
        resolved = candidate
    else:
        if assets_root is None:
            assets_root = await get_assets_root_path_async()
        if not assets_root:
            detail = f"; no local file at '{definition_relative}'" if definition_relative is not None else ""
            raise RuntimeError(f"Cannot resolve the Isaac assets root for gripper asset_path '{authored}'{detail}")
        resolved = path_join(assets_root, candidate.lstrip("/"))
    if verify and not await path_exists(resolved):
        tried = [resolved] if definition_relative in (None, resolved) else [definition_relative, resolved]
        message = (
            f"Gripper asset_path '{authored}' cannot be accessed; tried {', '.join(f"'{path}'" for path in tried)}. "
            "Check that the asset exists and is accessible. Use an assets-root fragment, a URI, "
            "an absolute local file, or a file relative to its definition directory."
        )
        if base_dir is None and not _URI_SCHEME.match(authored) and not is_absolute_path(authored):
            message += " No definition directory was supplied; pass base_dir to resolve a definition-relative file."
        raise ValueError(message)
    return resolved
