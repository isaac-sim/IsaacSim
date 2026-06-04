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

"""Control the viewport camera position, orientation, and properties.

Uses isaacsim.core.experimental for camera transforms and objects.Camera for properties.
Works in both windowed and --no-window headless modes.

Injected args (via isaacsim_send.py --arg):
    action: str — "get" (default), "set", "look_at", or "list_cameras".
    camera_path: str — Camera prim path (default: active viewport camera).
    position: str/tuple — x,y,z for set (e.g. "5,5,5").
    orientation: str/tuple — Quaternion w,x,y,z for set (e.g. "1,0,0,0").
    target: str/tuple — x,y,z look-at target for "look_at" action.
    focal_length: float — Focal length in mm (optional, for "set").

Examples:
    isaacsim_send.py --file camera_control.py
    isaacsim_send.py --file camera_control.py --arg action=set --arg "position=10,10,10"
    isaacsim_send.py --file camera_control.py --arg action=look_at --arg "target=0,0,0" --arg "position=5,5,5"
    isaacsim_send.py --file camera_control.py --arg action=list_cameras
"""

import numpy as np

if "action" not in dir():
    action = "get"
if "camera_path" not in dir():
    camera_path = None
if "position" not in dir():
    position = None
if "orientation" not in dir():
    orientation = None
if "target" not in dir():
    target = None
if "focal_length" not in dir():
    focal_length = None


def _parse_vec(val):
    """Parse a comma-separated string or tuple/list into a numpy array."""
    if val is None:
        return None
    if isinstance(val, (list, tuple)):
        return np.array([float(x) for x in val])
    return np.array([float(x.strip()) for x in str(val).split(",")])


import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils import xform


def _get_active_camera_path():
    from omni.kit.viewport.utility import get_active_viewport

    return get_active_viewport().camera_path


def _resolve_camera():
    if camera_path:
        return camera_path
    return _get_active_camera_path()


if action == "get":
    cam = _resolve_camera()
    cam_str = str(cam)
    pos, rot = xform.get_world_pose(cam_str)
    print(f"Camera: {cam_str}")
    print(f"Position (x, y, z): {pos}")
    print(f"Orientation (w, x, y, z): {rot}")
    try:
        from isaacsim.core.experimental.objects import Camera

        cam_obj = Camera(paths=cam_str)
        print(f"Focal length: {cam_obj.get_focal_lengths()[0]} mm")
        print(f"Clipping range: {cam_obj.get_clipping_ranges()[0]}")
    except Exception:
        fl = prim_utils.get_prim_attribute_value(cam_str, "focalLength")
        if fl is not None:
            print(f"Focal length: {fl} mm")

elif action == "set":
    from isaacsim.core.rendering_manager import ViewportManager

    cam = _resolve_camera()
    cam_str = str(cam)

    new_pos = _parse_vec(position)
    new_target = _parse_vec(target)

    if new_pos is not None or new_target is not None:
        kwargs = {}
        if new_pos is not None:
            kwargs["eye"] = new_pos.tolist()
        if new_target is not None:
            kwargs["target"] = new_target.tolist()
        ViewportManager.set_camera_view(cam_str, **kwargs)
        app_utils.update_app(steps=30)

    print(f"Camera: {cam_str}")
    if new_pos is not None:
        print(f"Position set to: {new_pos}")
    if new_target is not None:
        print(f"Looking at: {new_target}")

    if focal_length is not None:
        from isaacsim.core.experimental.objects import Camera

        cam_obj = Camera(paths=cam_str)
        cam_obj.set_focal_lengths(np.array([float(focal_length)]))
        print(f"Focal length set to: {focal_length} mm")

elif action == "look_at":
    if not target:
        raise ValueError("target required for 'look_at' (e.g. --arg target=0,0,0)")

    from isaacsim.core.rendering_manager import ViewportManager

    cam = _resolve_camera()
    cam_str = str(cam)
    target_pos = _parse_vec(target)
    eye_pos = _parse_vec(position)

    kwargs = {"target": target_pos.tolist()}
    if eye_pos is not None:
        kwargs["eye"] = eye_pos.tolist()
    ViewportManager.set_camera_view(cam_str, **kwargs)
    app_utils.update_app(steps=30)

    print(f"Camera: {cam_str}")
    if eye_pos is not None:
        print(f"Position: {eye_pos}")
    print(f"Looking at: {target_pos}")

elif action == "list_cameras":
    stage = stage_utils.get_current_stage()
    cameras = [p for p in stage.Traverse() if p.GetTypeName() == "Camera"]
    active = str(_get_active_camera_path())
    print(f"Cameras ({len(cameras)}):")
    for cam_prim in cameras:
        path = str(cam_prim.GetPath())
        marker = " (active)" if path == active else ""
        pos, _ = xform.get_world_pose(path)
        print(f"  {path}{marker} — pos={pos}")

else:
    print(f"ERROR: Unknown action '{action}'. Use: get, set, look_at, list_cameras")
