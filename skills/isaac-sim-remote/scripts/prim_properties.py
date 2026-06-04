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

"""Read or write arbitrary USD prim attributes.

Uses isaacsim.core.experimental.utils.prim for attribute access.
Works in both windowed and --no-window headless modes.

Injected globals (via isaacsim_send.py --arg):
    prim_path: str — USD prim path (required).
    action: str — "get" (default), "set", or "list".
    attr_name: str — Attribute name for get/set (required for get/set).
    attr_value: str — Value to set (required for "set"). Parsed as Python literal.
    attr_type: str — Optional USD type name for create-on-set (e.g. "float", "double3", "string").
"""

if "prim_path" not in dir():
    raise ValueError("prim_path is required (e.g. --arg prim_path=/World/Cube)")
if "action" not in dir():
    action = "get"
if "attr_name" not in dir():
    attr_name = None
if "attr_value" not in dir():
    attr_value = None
if "attr_type" not in dir():
    attr_type = None

from isaacsim.core.experimental.utils import prim as prim_utils

prim = prim_utils.get_prim_at_path(prim_path)
if not prim or not prim.IsValid():
    print(f"ERROR: Prim not found at '{prim_path}'")
elif action == "list":
    attrs = prim_utils.get_prim_attribute_names(prim_path)
    print(f"Attributes on '{prim_path}' ({len(attrs)}):")
    for name in sorted(attrs):
        try:
            val = prim_utils.get_prim_attribute_value(prim_path, name)
            val_str = repr(val)
            if len(val_str) > 100:
                val_str = val_str[:97] + "..."
            print(f"  {name} = {val_str}")
        except Exception:
            print(f"  {name} = <unreadable>")

elif action == "get":
    if not attr_name:
        raise ValueError("attr_name required for 'get' (e.g. --arg attr_name=xformOp:translate)")
    try:
        val = prim_utils.get_prim_attribute_value(prim_path, attr_name)
        print(f"{prim_path}.{attr_name} = {repr(val)}")
    except Exception as e:
        print(f"ERROR reading '{attr_name}': {e}")

elif action == "set":
    if not attr_name:
        raise ValueError("attr_name required for 'set'")
    if attr_value is None:
        raise ValueError("attr_value required for 'set'")

    # Parse the value
    import ast

    try:
        parsed = ast.literal_eval(attr_value)
    except (ValueError, SyntaxError):
        parsed = attr_value  # treat as string

    # If the attribute doesn't exist, create it
    attr = prim.GetAttribute(attr_name)
    if attr and attr.IsValid():
        attr.Set(parsed)
        print(f"Set {prim_path}.{attr_name} = {repr(parsed)}")
    else:
        if attr_type:
            from pxr import Sdf

            type_map = {
                "bool": Sdf.ValueTypeNames.Bool,
                "int": Sdf.ValueTypeNames.Int,
                "float": Sdf.ValueTypeNames.Float,
                "double": Sdf.ValueTypeNames.Double,
                "string": Sdf.ValueTypeNames.String,
                "token": Sdf.ValueTypeNames.Token,
                "float3": Sdf.ValueTypeNames.Float3,
                "double3": Sdf.ValueTypeNames.Double3,
                "color3f": Sdf.ValueTypeNames.Color3f,
                "point3f": Sdf.ValueTypeNames.Point3f,
                "normal3f": Sdf.ValueTypeNames.Normal3f,
                "vector3f": Sdf.ValueTypeNames.Vector3f,
                "quatf": Sdf.ValueTypeNames.Quatf,
                "matrix4d": Sdf.ValueTypeNames.Matrix4d,
                "asset": Sdf.ValueTypeNames.Asset,
            }
            sdf_type = type_map.get(attr_type.lower())
            if sdf_type:
                new_attr = prim.CreateAttribute(attr_name, sdf_type)
                new_attr.Set(parsed)
                print(f"Created and set {prim_path}.{attr_name} ({attr_type}) = {repr(parsed)}")
            else:
                print(f"ERROR: Unknown attr_type '{attr_type}'. Available: {list(type_map.keys())}")
        else:
            print(f"ERROR: Attribute '{attr_name}' does not exist. Use --arg attr_type=<type> to create it.")

else:
    print(f"ERROR: Unknown action '{action}'. Use: list, get, set")
