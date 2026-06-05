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

"""Read the current Kit session log or recent log entries.

Useful for debugging errors after an operation fails silently.
Works in both windowed and --no-window headless modes.

Injected globals (via isaacsim_send.py --arg):
    action: str — "tail" (default), "errors", "search", or "path".
    num_lines: int — Number of lines to return for "tail" (default: 50).
    query: str — Search string for "search" action (case-insensitive).
    level: str — Filter by log level for "errors": "error" (default), "warn", "all".
"""

if "action" not in dir():
    action = "tail"
if "num_lines" not in dir():
    num_lines = 50
if "query" not in dir():
    query = None
if "level" not in dir():
    level = "error"

import carb.settings

settings = carb.settings.get_settings()
log_file = settings.get("/log/file")

if not log_file:
    print("ERROR: Cannot determine log file path from settings")
else:
    import os

    if not os.path.isfile(log_file):
        print(f"ERROR: Log file not found: {log_file}")
    else:
        if action == "path":
            size = os.path.getsize(log_file)
            print(f"Log file: {log_file}")
            print(f"Size: {size:,} bytes")
            with open(log_file) as f:
                line_count = sum(1 for _ in f)
            print(f"Lines: {line_count:,}")

        elif action == "tail":
            n = int(num_lines)
            with open(log_file) as f:
                lines = f.readlines()
            for line in lines[-n:]:
                print(line.rstrip())

        elif action == "errors":
            level_filters = {"error": ["[Error]", "[Fatal]"], "warn": ["[Warning]", "[Error]", "[Fatal]"], "all": []}
            filters = level_filters.get(level, level_filters["error"])

            with open(log_file) as f:
                lines = f.readlines()

            if filters:
                matched = [l for l in lines if any(f in l for f in filters)]
            else:
                matched = lines

            n = int(num_lines)
            for line in matched[-n:]:
                print(line.rstrip())
            print(f"\n--- {len(matched)} matching lines total (showing last {min(n, len(matched))}) ---")

        elif action == "search":
            if not query:
                raise ValueError("query required for 'search' (e.g. --arg query=PhysX)")

            with open(log_file) as f:
                lines = f.readlines()

            matched = [l for l in lines if query.lower() in l.lower()]
            n = int(num_lines)
            for line in matched[-n:]:
                print(line.rstrip())
            print(f"\n--- {len(matched)} matches for '{query}' (showing last {min(n, len(matched))}) ---")

        else:
            print(f"ERROR: Unknown action '{action}'. Use: tail, errors, search, path")
