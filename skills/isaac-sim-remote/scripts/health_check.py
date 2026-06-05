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

"""Health check: verify Isaac Sim python server is responsive and report environment.

No injected globals required.
"""

import os

import carb.settings
import omni.kit.app
import omni.usd

app = omni.kit.app.get_app()
s = carb.settings.get_settings()

# Basic info
version = s.get("/app/version") or "unknown"
print(f"Isaac Sim version: {version}")

# Asset root
asset_root = s.get("/persistent/isaac/asset_root/default")
print(f"Asset root: {asset_root}")

# Stage info
ctx = omni.usd.get_context()
stage = ctx.get_stage()
if stage:
    prims = list(stage.Traverse())
    up_axis = stage.GetMetadata("upAxis") or "?"
    meters = stage.GetMetadata("metersPerUnit") or "?"
    print(f"Stage: {len(prims)} prims, up={up_axis}, meters/unit={meters}")
else:
    print("Stage: None (no stage open)")

# Timeline
import isaacsim.core.experimental.utils.app as app_utils

playing = app_utils.is_playing()
print(f"Timeline: {'playing' if playing else 'stopped'}")

# Display
display = os.environ.get("DISPLAY", "none")
print(f"Display: {display}")

# Loaded extensions count
ext_manager = omni.kit.app.get_app().get_extension_manager()
enabled = [e for e in ext_manager.get_extensions() if e["enabled"]]
print(f"Extensions: {len(enabled)} enabled")

print()
print("Health: OK")
