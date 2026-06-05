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

"""Compare two tab-separated Tracy CSV exports and print a summary.

Usage:
    python compare_tracy_csvs.py <reference.csv> <new.csv> [--top N] [--sep SEP]
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def load_csv(path: str, sep: str = "\t") -> dict[str, dict]:
    zones: dict[str, dict] = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f, delimiter=sep)
        for row in reader:
            try:
                name = row["name"]
                zones[name] = {
                    "total_ns": int(row["total_ns"]),
                    "counts": int(row["counts"]),
                    "mean_ns": float(row["mean_ns"]),
                    "min_ns": int(row["min_ns"]),
                    "max_ns": int(row["max_ns"]),
                }
            except (KeyError, ValueError):
                continue
    return zones


def fmt_delta(ref_val: float, new_val: float) -> str:
    if ref_val == 0:
        return "N/A"
    pct = (new_val - ref_val) / ref_val * 100
    return f"{pct:+.1f}%"


def print_comparison(ref: dict, new: dict, top_n: int) -> None:
    top_ref = sorted(ref.items(), key=lambda x: x[1]["total_ns"], reverse=True)[:top_n]

    header = f"{'Zone':<55} {'Ref mean(ms)':>12} {'New mean(ms)':>12} {'Delta':>8} {'Ref calls':>10} {'New calls':>10}"
    print(header)
    print("-" * len(header))

    for name, rdata in top_ref:
        r_mean = rdata["mean_ns"] / 1e6
        ndata = new.get(name)
        if ndata:
            n_mean = ndata["mean_ns"] / 1e6
            delta = fmt_delta(r_mean, n_mean)
            print(
                f"{name[:55]:<55} {r_mean:>12.3f} {n_mean:>12.3f} {delta:>8} {rdata['counts']:>10} {ndata['counts']:>10}"
            )
        else:
            print(f"{name[:55]:<55} {r_mean:>12.3f} {'N/A':>12} {'':>8} {rdata['counts']:>10} {'':>10}")

    new_only = {k: v for k, v in new.items() if k not in ref}
    if new_only:
        top_new = sorted(new_only.items(), key=lambda x: x[1]["total_ns"], reverse=True)[:top_n]
        print()
        print(f"=== Top {len(top_new)} zones ONLY in new run (by total_ns) ===")
        header2 = f"{'Zone':<55} {'mean(ms)':>12} {'total(ms)':>12} {'calls':>10}"
        print(header2)
        print("-" * len(header2))
        for name, ndata in top_new:
            n_mean = ndata["mean_ns"] / 1e6
            n_total = ndata["total_ns"] / 1e6
            print(f"{name[:55]:<55} {n_mean:>12.3f} {n_total:>12.1f} {ndata['counts']:>10}")

    ref_only = {k: v for k, v in ref.items() if k not in new}
    if ref_only:
        top_gone = sorted(ref_only.items(), key=lambda x: x[1]["total_ns"], reverse=True)[:top_n]
        print()
        print(f"=== Top {len(top_gone)} zones ONLY in reference (disappeared) ===")
        header3 = f"{'Zone':<55} {'mean(ms)':>12} {'total(ms)':>12} {'calls':>10}"
        print(header3)
        print("-" * len(header3))
        for name, rdata in top_gone:
            r_mean = rdata["mean_ns"] / 1e6
            r_total = rdata["total_ns"] / 1e6
            print(f"{name[:55]:<55} {r_mean:>12.3f} {r_total:>12.1f} {rdata['counts']:>10}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare two Tracy CSV exports.")
    parser.add_argument("reference", help="Path to the reference CSV.")
    parser.add_argument("new", help="Path to the new CSV.")
    parser.add_argument("--top", type=int, default=20, help="Number of top zones to show (default: 20).")
    parser.add_argument("--sep", default="\t", help="Column separator (default: tab).")
    args = parser.parse_args()

    for p in (args.reference, args.new):
        if not Path(p).is_file():
            print(f"Error: file not found: {p}", file=sys.stderr)
            return 1

    ref = load_csv(args.reference, args.sep)
    new = load_csv(args.new, args.sep)

    print(f"Reference: {args.reference} ({len(ref)} zones)")
    print(f"New:       {args.new} ({len(new)} zones)")
    print()

    print_comparison(ref, new, args.top)
    return 0


if __name__ == "__main__":
    sys.exit(main())
