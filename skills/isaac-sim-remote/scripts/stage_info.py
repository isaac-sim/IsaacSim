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

"""Print the current stage hierarchy and optionally detailed attributes for specific prims.

Works in both windowed and --no-window headless modes.

Injected args (via isaacsim_send.py --arg):
    mode: str — "tree" (default) or "list". Tree shows indented hierarchy, list shows flat paths.
    prim_path: str — Optional prim path to show detailed attributes for (default: None).
    filter_type: str — Optional type name filter (e.g. "Mesh", "Xform", "Camera"). Only show prims of this type.
    max_depth: int — Maximum traversal depth (default: unlimited). 0 = root only, 1 = direct children, etc.
    show_attrs: bool — If True, show attribute names and values for each prim (default: False).
    exclude_render: bool — If True, exclude /Render subtree from output (default: True).

Examples:
    isaacsim_send.py --file stage_info.py
    isaacsim_send.py --file stage_info.py --arg prim_path=/World/Cube
    isaacsim_send.py --file stage_info.py --arg filter_type=Camera --arg show_attrs=True
    isaacsim_send.py --file stage_info.py --arg mode=list --arg exclude_render=False
"""

# Defaults (overridden by --arg injection)
if "mode" not in dir():
    mode = "tree"
if "prim_path" not in dir():
    prim_path = None
if "filter_type" not in dir():
    filter_type = None
if "max_depth" not in dir():
    max_depth = None
if "show_attrs" not in dir():
    show_attrs = False
if "exclude_render" not in dir():
    exclude_render = True

from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils

stage = stage_utils.get_current_stage()
if not stage:
    print("ERROR: No stage open")
elif prim_path:
    prim = prim_utils.get_prim_at_path(prim_path)
    if not prim or not prim.IsValid():
        print(f"ERROR: Prim not found at '{prim_path}'")
    else:
        print(f"Path: {prim.GetPath()}")
        print(f"Type: {prim.GetTypeName()}")
        print(f"Active: {prim.IsActive()}")
        schemas = prim.GetAppliedSchemas()
        if schemas:
            print(f"Applied schemas: {list(schemas)}")
        children = prim.GetChildren()
        if children:
            print(f"Children ({len(children)}):")
            for child in children:
                print(f"  {child.GetPath()} ({child.GetTypeName()})")
        attrs = prim_utils.get_prim_attribute_names(prim_path)
        if attrs:
            print(f"\nAttributes ({len(attrs)}):")
            for attr_name in sorted(attrs):
                try:
                    val = prim_utils.get_prim_attribute_value(prim_path, attr_name)
                    val_str = repr(val)
                    if len(val_str) > 120:
                        val_str = val_str[:117] + "..."
                    print(f"  {attr_name} = {val_str}")
                except Exception:
                    print(f"  {attr_name} = <unreadable>")
else:
    if mode in ("tree", "list"):
        output = stage_utils.generate_stage_representation(mode=mode)
        if exclude_render:
            lines = output.split("\n")
            filtered = []
            skip_indent = -1
            for line in lines:
                if mode == "list":
                    if line.startswith("/Render"):
                        continue
                else:
                    stripped = line.lstrip("│ ")
                    indent = len(line) - len(stripped)
                    if "Render (" in stripped or "Render ()" in stripped:
                        skip_indent = indent
                        continue
                    if skip_indent >= 0:
                        if indent > skip_indent:
                            continue
                        else:
                            skip_indent = -1
                filtered.append(line)
            output = "\n".join(filtered)
        print(output)
    else:
        print(f"ERROR: Unknown mode '{mode}'. Use 'tree' or 'list'.")

    if filter_type or show_attrs:
        print(f"\n--- Filtered prims (type={filter_type or 'all'}) ---")
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if exclude_render and "/Render" in path:
                continue
            if max_depth is not None:
                depth = path.count("/") - 1
                if depth > int(max_depth):
                    continue
            if filter_type and prim.GetTypeName() != filter_type:
                continue
            print(f"{path} ({prim.GetTypeName()})")
            if show_attrs:
                attr_names = prim_utils.get_prim_attribute_names(path)
                for attr_name in sorted(attr_names):
                    try:
                        val = prim_utils.get_prim_attribute_value(path, attr_name)
                        val_str = repr(val)
                        if len(val_str) > 100:
                            val_str = val_str[:97] + "..."
                        print(f"    {attr_name} = {val_str}")
                    except Exception:
                        print(f"    {attr_name} = <unreadable>")
