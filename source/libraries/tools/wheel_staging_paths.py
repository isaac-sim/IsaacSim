# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compatibility paths for generated extension wheel output."""

from __future__ import annotations

import os
import shutil
import sys
import uuid
from pathlib import Path


def remove_generated_path(path: Path) -> None:
    """Remove a generated directory or directory link without visiting a link target."""
    is_junction = getattr(path, "is_junction", None)
    if is_junction is not None and is_junction():
        path.rmdir()
    elif path.is_symlink():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def ensure_directory_alias(alias: Path, target: Path, *, preserve_directory: bool = False) -> None:
    """Point a legacy generated path at its current payload.

    A generate-only build retains ordinary legacy directories. Publication may
    replace the selected legacy output, but creates its new link first.
    """
    is_junction = getattr(alias, "is_junction", None)
    linked = alias.is_symlink() or (is_junction is not None and is_junction())
    if linked and alias.resolve() == target.resolve():
        return
    if preserve_directory and alias.is_dir() and not linked:
        return

    alias.parent.mkdir(parents=True, exist_ok=True)
    temporary = alias.with_name(f".{alias.name}-{uuid.uuid4().hex}")
    try:
        if sys.platform == "win32":
            import _winapi

            _winapi.CreateJunction(str(target.resolve()), str(temporary))
        else:
            temporary.symlink_to(Path(os.path.relpath(target, alias.parent)), target_is_directory=True)
        remove_generated_path(alias)
        temporary.rename(alias)
    finally:
        remove_generated_path(temporary)
