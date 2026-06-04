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

"""Execute any registered omni.kit.commands command by name.

Isaac Sim has 370+ registered commands for creating physics joints, materials,
meshes, references, and more. This script provides access to all of them.

Injected globals (via isaacsim_send.py --arg):
    action: str — "run" (default) or "list".
    command_name: str — Command name for "run" (e.g. "CreateMeshPrimWithDefaultXform").
    kwargs: str — JSON string of keyword arguments (e.g. '{"prim_type":"Cube"}').
    filter: str — Filter string for "list" action (case-insensitive substring match).
    undo_last: str — If "true", undo the last command instead of running one.
"""

if "action" not in dir():
    action = "run"
if "command_name" not in dir():
    command_name = None
if "kwargs" not in dir():
    kwargs = None
if "filter" not in dir():
    filter = None  # noqa: A001
if "undo_last" not in dir():
    undo_last = "false"

import omni.kit.commands

if action == "list":
    cmds = sorted(omni.kit.commands.get_commands().keys())
    # Deduplicate (many commands register with and without "Command" suffix)
    unique = sorted(set(c.removesuffix("Command") for c in cmds))
    if filter:
        unique = [c for c in unique if filter.lower() in c.lower()]
    print(f"Commands ({len(unique)}):")
    for c in unique:
        print(f"  {c}")

elif action == "run":
    if not command_name:
        raise ValueError("command_name required (e.g. --arg command_name=CreateMeshPrimWithDefaultXform)")

    cmd_kwargs = {}
    if kwargs:
        if isinstance(kwargs, dict):
            cmd_kwargs = kwargs
        else:
            import json

            cmd_kwargs = json.loads(kwargs)

    import isaacsim.core.experimental.utils.app as app_utils

    result = omni.kit.commands.execute(command_name, **cmd_kwargs)
    app_utils.update_app(steps=5)

    print(f"Executed: {command_name}")
    if cmd_kwargs:
        print(f"Args: {cmd_kwargs}")
    if result is not None:
        print(f"Result: {result}")

elif undo_last.lower() == "true":
    omni.kit.commands.undo()
    print("Undo: last command undone")

else:
    print(f"ERROR: Unknown action '{action}'. Use: run, list")
