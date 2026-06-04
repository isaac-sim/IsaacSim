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

"""Select or query prim selection in the USD stage.

Selection is needed before many UI operations (Property panel, context menus).
Works in both windowed and --no-window headless modes.

Injected globals (via isaacsim_send.py --arg):
    action: str — "get" (default), "set", "clear", or "all".
    prim_path: str — Prim path to select (for "set" action). Comma-separated for multiple.
"""

if "action" not in dir():
    action = "get"
if "prim_path" not in dir():
    prim_path = None

import omni.usd

ctx = omni.usd.get_context()
sel = ctx.get_selection()

if action == "get":
    paths = sel.get_selected_prim_paths()
    if paths:
        print(f"Selected ({len(paths)}):")
        for p in paths:
            print(f"  {p}")
    else:
        print("No prims selected")

elif action == "set":
    if not prim_path:
        print("ERROR: prim_path required for 'set' (e.g. --arg prim_path=/World/Cube)")
    else:
        paths = [p.strip() for p in prim_path.split(",")]
        sel.set_selected_prim_paths(paths, True)
        print(f"Selected {len(paths)} prim(s):")
        for p in paths:
            print(f"  {p}")

elif action == "clear":
    sel.clear_selected_prim_paths()
    print("Selection cleared")

elif action == "all":
    sel.select_all_prims(ctx.get_stage_url())
    paths = sel.get_selected_prim_paths()
    print(f"Selected all prims ({len(paths)}):")
    for p in paths[:20]:
        print(f"  {p}")
    if len(paths) > 20:
        print(f"  ... and {len(paths) - 20} more")

else:
    print(f"ERROR: Unknown action '{action}'. Use: get, set, clear, all")
