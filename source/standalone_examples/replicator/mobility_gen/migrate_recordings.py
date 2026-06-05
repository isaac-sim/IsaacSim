# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Migrate MobilityGen recordings from the legacy pickled .npy format to .npz.

Old recordings store ``state/common/*.npy`` as pickled Python dicts, which
require ``allow_pickle=True`` and can execute arbitrary code on load.  New
recordings use ``state/common/*.npz`` (one named array per buffer key), which
load safely with the default ``allow_pickle=False``.

Run this script once on any recording directory (or a parent directory
containing multiple recordings) before using it with the current reader.

Usage
-----
Single recording::

    python migrate_recordings.py /path/to/recording

Directory tree (migrates all recordings found recursively)::

    python migrate_recordings.py /path/to/recordings/root --recursive
"""

import argparse
import glob
import os
import sys

import numpy as np


def _migrate_recording(recording_path: str) -> int:
    """Migrate all legacy .npy files in one recording directory.

    Args:
        recording_path: Path to a single recording directory.

    Returns:
        Number of files migrated.
    """
    pattern = os.path.join(recording_path, "state", "common", "*.npy")
    npy_paths = sorted(glob.glob(pattern))
    if not npy_paths:
        return 0

    count = 0
    for npy_path in npy_paths:
        data = np.load(npy_path, allow_pickle=True).item()
        npz_path = npy_path[:-4] + ".npz"
        np.savez(npz_path, **{k: v for k, v in data.items() if v is not None})
        os.remove(npy_path)
        count += 1

    return count


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Migrate MobilityGen recordings from .npy to .npz format.\n\n"
            "WARNING: This tool loads pickled .npy files using allow_pickle=True, which can execute\n"
            "arbitrary code. Only run this on recording data you trust."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("path", help="Recording directory (or root directory when --recursive is set).")
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search PATH recursively for recording directories containing state/common/*.npy files.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.path):
        print(f"error: not a directory: {args.path}", file=sys.stderr)
        sys.exit(1)

    if args.recursive:
        npy_files = glob.glob(os.path.join(args.path, "**/state/common/*.npy"), recursive=True)
        recording_dirs = sorted({os.path.dirname(os.path.dirname(os.path.dirname(p))) for p in npy_files})
    else:
        recording_dirs = [args.path]

    if not recording_dirs:
        print("No legacy .npy recordings found.")
        return

    total = 0
    for recording_dir in recording_dirs:
        count = _migrate_recording(recording_dir)
        if count:
            print(f"Migrated {count} file(s): {recording_dir}")
            total += count

    print(f"\nDone. {total} file(s) migrated across {len(recording_dirs)} recording(s).")


if __name__ == "__main__":
    main()
