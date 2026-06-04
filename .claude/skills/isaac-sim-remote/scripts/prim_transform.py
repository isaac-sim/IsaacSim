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

"""Get or set the world-space transform (position, orientation, scale) of a prim.

Uses isaacsim.core.experimental.utils.xform for pose and XformPrim for scale.
Works in both windowed and --no-window headless modes.

Injected args (via isaacsim_send.py --arg):
    prim_path: str — USD prim path (required).
    action: str — "get" (default) or "set".
    position: str/tuple — x,y,z for set (e.g. "1.0,2.0,3.0" or 1.0,2.0,3.0).
    orientation: str/tuple — Quaternion w,x,y,z for set (e.g. "1,0,0,0").
    scale: str/tuple — x,y,z scale for set (e.g. "1,1,1").

Examples:
    isaacsim_send.py --file prim_transform.py --arg prim_path=/World/Cube
    isaacsim_send.py --file prim_transform.py --arg prim_path=/World/Cube --arg action=set --arg "position=3,0,1"
"""

import numpy as np

if "prim_path" not in dir():
    raise ValueError("prim_path is required (e.g. --arg prim_path=/World/Cube)")
if "action" not in dir():
    action = "get"
if "position" not in dir():
    position = None
if "orientation" not in dir():
    orientation = None
if "scale" not in dir():
    scale = None


def _parse_vec(val):
    """Parse a comma-separated string or tuple/list into a numpy array."""
    if val is None:
        return None
    if isinstance(val, (list, tuple)):
        return np.array([float(x) for x in val])
    return np.array([float(x.strip()) for x in str(val).split(",")])


from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import xform

prim = prim_utils.get_prim_at_path(prim_path)
if not prim or not prim.IsValid():
    print(f"ERROR: Prim not found at '{prim_path}'")
elif action == "get":
    from isaacsim.core.experimental.prims import XformPrim

    xp = XformPrim(paths=prim_path)
    poses = xp.get_world_poses()
    pos = np.array(poses[0].numpy())[0]
    rot = np.array(poses[1].numpy())[0]
    print(f"Path: {prim_path}")
    print(f"Position (x, y, z): {pos}")
    print(f"Orientation (w, x, y, z): {rot}")
    try:
        scales = xp.get_local_scales()
        print(f"Scale (x, y, z): {np.array(scales.numpy())[0]}")
    except Exception:
        print("Scale: <unable to read>")
elif action == "set":
    from isaacsim.core.experimental.prims import XformPrim

    pos_vec = _parse_vec(position)
    rot_vec = _parse_vec(orientation)
    scale_vec = _parse_vec(scale)
    xp = XformPrim(paths=prim_path)

    if pos_vec is not None or rot_vec is not None:
        cur_poses = xp.get_world_poses()
        cur_pos = np.array(cur_poses[0].numpy())[0]  # warp array -> numpy
        cur_rot = np.array(cur_poses[1].numpy())[0]
        new_pos = pos_vec if pos_vec is not None else cur_pos
        new_rot = rot_vec if rot_vec is not None else cur_rot
        xp.set_world_poses(
            positions=np.array(new_pos, dtype=np.float32).reshape(1, 3),
            orientations=np.array(new_rot, dtype=np.float32).reshape(1, 4),
        )
        print(f"Set pose for '{prim_path}':")
        print(f"  Position: {new_pos}")
        print(f"  Orientation: {new_rot}")

    if scale_vec is not None:
        xp.set_local_scales(np.array(scale_vec, dtype=np.float32).reshape(1, 3))
        print(f"  Scale: {scale_vec}")

    if pos_vec is None and rot_vec is None and scale_vec is None:
        print("WARNING: action='set' but no position, orientation, or scale provided")
else:
    print(f"ERROR: Unknown action '{action}'. Use 'get' or 'set'.")
