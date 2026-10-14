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

"""Verify that a concrete Isaac asset URL is reachable and can be referenced.

Injected globals via ``isaacsim_send.py --arg``:
    asset_path: Asset path under the Isaac asset root, or a full URL.
    asset_root: Optional preset or URL. ``production`` and ``staging`` map to
        Isaac 6.2 S3 assets. Use staging only for explicit testing. If omitted,
        preserve the configured application asset root.
    prim_path: Prim path for a reference-open check, default ``/World/Asset``.
    open_stage: Whether to reference the asset into a new stage, default True.
"""

if "asset_path" not in dir():
    asset_path = None
if "asset_root" not in dir():
    asset_root = None
if "prim_path" not in dir():
    prim_path = "/World/Asset"
if "open_stage" not in dir():
    open_stage = True

import json

import carb.settings
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.client
from isaacsim.storage.native import get_assets_root_path

PRESETS = {
    "staging": "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2",
    "production": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2",
}


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in {"1", "true", "yes", "on"}


def _set_asset_root(value):
    if not value:
        return None
    url = PRESETS.get(str(value), str(value))
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", url)
    return url


def _configured_asset_root():
    return carb.settings.get_settings().get("/persistent/isaac/asset_root/default")


def _asset_url(path, requested_root=None):
    if not path:
        raise RuntimeError("asset_path is required.")
    text = str(path)
    if "://" in text:
        return text
    try:
        root = get_assets_root_path()
    except Exception as exc:
        root = requested_root or _configured_asset_root()
        if not root:
            raise RuntimeError(f"Could not resolve Isaac asset root: {exc}") from exc
    return root.rstrip("/") + "/" + text.lstrip("/")


def _resolved_root():
    try:
        return get_assets_root_path()
    except Exception as exc:
        return f"ERROR: {type(exc).__name__}: {exc}"


def _stat(url):
    result, entry = omni.client.stat(url)
    return {
        "url": url,
        "result": str(result),
        "exists": result == omni.client.Result.OK,
        "size": getattr(entry, "size", None) if entry else None,
    }


def _run():
    requested_root = _set_asset_root(asset_root)
    url = _asset_url(asset_path, requested_root=requested_root)
    stat = _stat(url)

    opened = None
    prim_valid = None
    if _as_bool(open_stage):
        stage_utils.create_new_stage(template="default stage")
        app_utils.update_app(steps=5)
        stage_utils.add_reference_to_stage(usd_path=url, path=prim_path)
        while stage_utils.is_stage_loading():
            app_utils.update_app(steps=1)
        app_utils.update_app(steps=30)
        stage = stage_utils.get_current_stage()
        prim_valid = stage.GetPrimAtPath(prim_path).IsValid()
        opened = bool(prim_valid)

    payload = {
        "requested_asset_root": requested_root,
        "configured_asset_root": _configured_asset_root(),
        "resolved_asset_root": _resolved_root(),
        "asset_path": asset_path,
        "asset_url": url,
        "stat": stat,
        "opened_stage": opened,
        "prim_path": prim_path,
        "prim_valid": prim_valid,
    }
    print("ASSET_VERIFY_RESULT=" + json.dumps(payload, indent=2, sort_keys=True))
    if not stat["exists"]:
        raise RuntimeError(f"Asset is not reachable: {url} ({stat['result']})")
    if opened is False:
        raise RuntimeError(f"Asset did not open as a valid prim at {prim_path}: {url}")


_run()
