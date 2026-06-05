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

"""Open, create, or save a USD stage.

Uses isaacsim.core.experimental.utils.stage.
Works in both windowed and --no-window headless modes.

Injected globals (via isaacsim_send.py --arg):
    action: str — "open" (default), "new", "save", or "info".
    usd_path: str — Path to USD file for open/save (required for open, optional for save).
    template: str — Stage template for "new" action (default: "empty"). Options: "empty", "default".
"""

if "action" not in dir():
    action = "open"
if "usd_path" not in dir():
    usd_path = None
if "template" not in dir():
    template = "empty"

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils


async def _run():
    if action == "open":
        if not usd_path:
            raise ValueError("usd_path required for 'open' (e.g. --arg usd_path=/path/to/scene.usd)")
        result = await stage_utils.open_stage_async(usd_path)
        app_utils.update_app(steps=10)
        if result:
            stage = stage_utils.get_current_stage()
            print(f"Opened: {usd_path}")
            print(f"Stage prims: {len(list(stage.Traverse()))}")
        else:
            print(f"ERROR: Failed to open '{usd_path}'")

    elif action == "new":
        await stage_utils.create_new_stage_async(template=template)
        app_utils.update_app(steps=10)
        print(f"Created new stage (template: {template})")

    elif action == "save":
        if usd_path:
            import omni.usd

            result = omni.usd.get_context().save_as_stage(usd_path)
            if result:
                print(f"Saved as: {usd_path}")
            else:
                print(f"ERROR: Failed to save as '{usd_path}'")
        else:
            stage_utils.save_stage()
            print("Stage saved")

    elif action == "info":
        stage = stage_utils.get_current_stage()
        if not stage:
            print("No stage open")
        else:
            import omni.usd

            url = omni.usd.get_context().get_stage_url()
            up_axis = stage_utils.get_stage_up_axis()
            units = stage_utils.get_stage_units()
            prim_count = len(list(stage.Traverse()))
            print(f"Stage URL: {url}")
            print(f"Up axis: {up_axis}")
            print(f"Units (meters per unit): {units}")
            print(f"Prim count: {prim_count}")
            print(f"Stage loading: {stage_utils.is_stage_loading()}")
    else:
        print(f"ERROR: Unknown action '{action}'. Use: open, new, save, info")


await _run()
