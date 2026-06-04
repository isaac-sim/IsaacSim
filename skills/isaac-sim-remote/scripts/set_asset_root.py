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

"""Configure the Isaac Sim asset root path.

Use when Nucleus server is unavailable and you need cloud assets as fallback.

Injected globals (via isaacsim_send.py --arg):
    asset_root: str — Asset root URL. Special values:
        "production" -> S3 production assets (Isaac 5.0)
        "staging" -> S3 staging assets (Isaac 6.0, recommended for 6.x builds)
        "nucleus" -> Reset to default Nucleus server
        Any URL -> Set directly
    check: str — "true" to verify the path works (default: "true").
"""

import carb.settings

# Defaults
if "asset_root" not in dir():
    # Show current setting
    s = carb.settings.get_settings()
    current = s.get("/persistent/isaac/asset_root/default")
    print(f"Current asset root: {current}")
    print()
    print("Usage: --arg asset_root=<value>")
    print("  staging    -> S3 staging assets (Isaac 6.0, recommended for 6.x builds)")
    print("  production -> S3 production assets (Isaac 5.0)")
    print("  nucleus    -> Reset to Nucleus default")
    asset_root = None  # signal to skip the rest

if asset_root is not None:
    if "check" not in dir():
        check = "true"

    S3_PROD = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac"
    S3_STAGING = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac"
    PRESETS = {
        "production": f"{S3_PROD}/5.0",
        "staging": f"{S3_STAGING}/6.0",
    }

    url = PRESETS.get(asset_root, asset_root)

    s = carb.settings.get_settings()
    old = s.get("/persistent/isaac/asset_root/default")

    if asset_root == "nucleus":
        s.destroy_item("/persistent/isaac/asset_root/default")
        url = s.get("/persistent/isaac/asset_root/default") or "(unset)"
    else:
        s.set("/persistent/isaac/asset_root/default", url)

    print(f"Asset root changed: {old} -> {url}")

    if check.lower() == "true" and not url.startswith("omniverse://"):
        try:
            from isaacsim.storage.native import get_assets_root_path

            result = get_assets_root_path()
            print(f"Verification: OK ({result})")
        except Exception as e:
            print(f"Verification: FAILED ({e})")
            print("Assets may not be accessible at this URL")
