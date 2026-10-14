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

"""Fail-closed mount checks: canonical roots, no symlink escapes, ACL-only widening."""

from __future__ import annotations

import argparse
import errno
import os
import stat
import subprocess
import sys
from pathlib import Path

CONTAINER_UID = 1234


def home_dir() -> Path:
    """Return the operator home used as the mount prefix root."""
    return Path(os.environ.get("HOME") or Path.home()).expanduser().resolve()


def allowed_mount_roots(home: Path | None = None) -> tuple[Path, Path]:
    """Return the only trees this skill may chmod/setfacl."""
    home = home or home_dir()
    return (home / "docker" / "isaac-sim", home / ".cache" / "isaac-sim-web-viewer")


def _existing_ancestors(path: Path) -> list[Path]:
    """Return path and each parent up to the filesystem root."""
    cur = path
    rows = []
    while True:
        rows.append(cur)
        if cur.parent == cur:
            break
        cur = cur.parent
    return rows


def path_has_symlink(path: Path, stop_at: Path) -> bool:
    """Return True if path or an ancestor below stop_at is a symlink."""
    stop_at = stop_at.resolve()
    for cur in _existing_ancestors(path):
        try:
            resolved_cur = cur.resolve()
        except OSError:
            return True
        # $HOME itself may be a symlink. That is not an escape.
        if resolved_cur == stop_at:
            break
        try:
            if cur.is_symlink():
                return True
        except OSError:
            return True
    return False


def is_safe_skill_mount(path: str, home: Path | None = None) -> bool:
    """Return True if path is under an allowed root and has no symlink components."""
    home = home or home_dir()
    raw = Path(path).expanduser()
    if path_has_symlink(raw if raw.exists() else raw.parent, home):
        return False
    try:
        resolved = raw.resolve(strict=False)
    except OSError:
        return False
    for root in allowed_mount_roots(home):
        try:
            resolved.relative_to(root.resolve() if root.exists() else root)
            return True
        except ValueError:
            continue
    return False


def _nofollow_flags(*, directory: bool) -> int:
    """Return open flags that refuse a symlink in the final path component."""
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
    if directory:
        flags |= os.O_DIRECTORY
    return flags


def _setfacl_fd(fd: int, host_uid: int, *, is_dir: bool, label: str) -> None:
    """Apply ACLs to the inode referenced by fd. Never pass a traversable pathname."""
    # /proc/<pid>/fd/<n> opens the existing file description, so a swapped
    # directory entry cannot redirect setfacl onto a symlink target.
    target = f"/proc/{os.getpid()}/fd/{fd}"
    cmd = [
        "setfacl",
        "-m",
        "u:1234:rwX",
        "-m",
        f"u:{host_uid}:rwX",
        "-m",
        "g::---",
        "-m",
        "o::---",
        "-m",
        "m::rwx",
    ]
    if is_dir:
        cmd.extend(
            [
                "-d",
                "-m",
                "u:1234:rwX",
                "-d",
                "-m",
                f"u:{host_uid}:rwX",
                "-d",
                "-m",
                "g::---",
                "-d",
                "-m",
                "o::---",
                "-d",
                "-m",
                "m::rwx",
            ]
        )
    cmd.append(target)
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise SystemExit(f"FAIL: setfacl failed on {label}") from exc


def _maybe_setfacl_fd(fd: int, st: os.stat_result, host_uid: int, root_dev: int, *, is_dir: bool, label: str) -> None:
    """setfacl host-owned inodes on the same device as the mount root. Skip uid 1234 leftovers."""
    if st.st_dev != root_dev:
        raise SystemExit(f"FAIL: refusing to change permissions across filesystems: {label}")
    if stat.S_ISLNK(st.st_mode):
        raise SystemExit(f"FAIL: refusing symlink in mount tree: {label}")
    if st.st_uid == CONTAINER_UID:
        return
    if st.st_uid != host_uid:
        raise SystemExit(f"FAIL: unexpected owner uid {st.st_uid} on {label}")
    _setfacl_fd(fd, host_uid, is_dir=is_dir, label=label)


def _apply_acl_dirfd(dir_fd: int, label: str, host_uid: int, root_dev: int, seen: set[tuple[int, int]]) -> None:
    """Walk a directory by fd and openat(O_NOFOLLOW) so intermediate swaps cannot be followed."""
    st = os.fstat(dir_fd)
    key = (st.st_dev, st.st_ino)
    if key in seen:
        return
    seen.add(key)
    _maybe_setfacl_fd(dir_fd, st, host_uid, root_dev, is_dir=True, label=label)
    with os.scandir(dir_fd) as iterator:
        children = list(iterator)
    for entry in children:
        name = entry.name
        if name in {".", ".."}:
            continue
        child_label = f"{label}/{name}"
        try:
            if entry.is_symlink():
                raise SystemExit(f"FAIL: refusing symlink in mount tree: {child_label}")
            is_dir = entry.is_dir(follow_symlinks=False)
        except OSError as exc:
            raise SystemExit(f"FAIL: cannot inspect {child_label}: {exc}") from exc
        flags = _nofollow_flags(directory=is_dir)
        try:
            child_fd = os.open(name, flags, dir_fd=dir_fd)
        except OSError as exc:
            if exc.errno == errno.ENOENT:
                continue
            if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
                raise SystemExit(f"FAIL: refusing symlink or replaced entry in mount tree: {child_label}") from exc
            if exc.errno == errno.EACCES:
                # Container leftovers are often 0700 uid 1234. Inspect via O_PATH
                # (no read) and skip them; do not follow a pathname.
                inspect_flags = os.O_PATH | os.O_NOFOLLOW | os.O_CLOEXEC
                if is_dir:
                    inspect_flags |= os.O_DIRECTORY
                try:
                    inspect_fd = os.open(name, inspect_flags, dir_fd=dir_fd)
                except OSError as inspect_exc:
                    if inspect_exc.errno == errno.ENOENT:
                        continue
                    raise SystemExit(f"FAIL: cannot open {child_label}: {inspect_exc}") from inspect_exc
                try:
                    inspect_st = os.fstat(inspect_fd)
                finally:
                    os.close(inspect_fd)
                if inspect_st.st_uid == CONTAINER_UID:
                    continue
                raise SystemExit(f"FAIL: cannot open {child_label}: {exc}") from exc
            raise SystemExit(f"FAIL: cannot open {child_label}: {exc}") from exc
        try:
            child_st = os.fstat(child_fd)
            if is_dir:
                _apply_acl_dirfd(child_fd, child_label, host_uid, root_dev, seen)
            else:
                child_key = (child_st.st_dev, child_st.st_ino)
                if child_key in seen:
                    continue
                seen.add(child_key)
                _maybe_setfacl_fd(child_fd, child_st, host_uid, root_dev, is_dir=False, label=child_label)
        finally:
            os.close(child_fd)


def apply_acl_tree(path: str, host_uid: int) -> None:
    """Grant uid 1234 and host_uid via setfacl. Skip container-owned leftovers. Never chmod."""
    root = Path(path)
    if not is_safe_skill_mount(str(root)):
        raise SystemExit(f"FAIL: refusing to change permissions on unsafe mount {root}")
    if path_has_symlink(root, home_dir()):
        raise SystemExit(f"FAIL: refusing symlink in mount tree {root}")
    try:
        root_fd = os.open(root, _nofollow_flags(directory=True))
    except OSError as exc:
        if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
            raise SystemExit(f"FAIL: refusing symlink in mount tree {root}") from exc
        raise SystemExit(f"FAIL: cannot open mount {root}: {exc}") from exc
    try:
        root_dev = os.fstat(root_fd).st_dev
        _apply_acl_dirfd(root_fd, str(root), host_uid, root_dev, set())
    finally:
        os.close(root_fd)


def main(argv: list[str] | None = None) -> int:
    """CLI for assert-safe and apply-acl."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    assert_p = sub.add_parser("assert-safe")
    assert_p.add_argument("path")
    acl_p = sub.add_parser("apply-acl")
    acl_p.add_argument("path")
    acl_p.add_argument("host_uid", type=int)
    args = parser.parse_args(argv)
    if args.cmd == "assert-safe":
        if not is_safe_skill_mount(args.path):
            print(
                f"FAIL: mount {args.path} must be a real path under "
                f"$HOME/docker/isaac-sim or $HOME/.cache/isaac-sim-web-viewer (no symlinks).",
                file=sys.stderr,
            )
            return 1
        return 0
    apply_acl_tree(args.path, args.host_uid)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
