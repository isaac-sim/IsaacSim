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

import argparse
import math
import os

import numpy as np
import yaml
from isaacsim import SimulationApp

DEFAULT_CONFIG = {
    "headless": True,
    "renderer": "RealTimePathTracing",
    "resolution": [1280, 720],
    "rt_subframes": 32,
    "num_frames": 100,
    "seed": 42,
    "env_url": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "output_dir": "_out_warehouse_sdg",
    "annotations": {
        "rgb": True,
        "bounding_box_2d_tight": True,
        "semantic_segmentation": True,
        "distance_to_image_plane": True,
        "bounding_box_3d": True,
        "occlusion": True,
    },
    "objects": [
        {"url": "/Isaac/Props/Forklift/forklift.usd", "label": "forklift", "count": 1},
        {"url": "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd", "label": "pallet", "count": 2},
        {"url": "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd", "label": "cardbox", "count": 8},
        {"url": "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd", "label": "traffic_cone", "count": 3},
    ],
    "camera": {
        "focal_length": 24.0,
        "focus_distance": 400.0,
        "clipping_range": [0.1, 10000000.0],
        "position_min": [-15, -5, 1.5],
        "position_max": [5, 10, 4.0],
    },
}

parser = argparse.ArgumentParser(description="Warehouse data collection with domain randomization")
parser.add_argument("--config", type=str, default=None, help="YAML config file (overrides defaults)")
parser.add_argument("--num-frames", type=int, default=None)
parser.add_argument("--output-dir", type=str, default=None)
parser.add_argument("--seed", type=int, default=None)
args, _ = parser.parse_known_args()

config = dict(DEFAULT_CONFIG)
if args.config and os.path.isfile(args.config):
    with open(args.config) as f:
        config.update(yaml.safe_load(f))
if args.num_frames is not None:
    config["num_frames"] = args.num_frames
if args.output_dir is not None:
    config["output_dir"] = args.output_dir
if args.seed is not None:
    config["seed"] = args.seed

simulation_app = SimulationApp({"renderer": config["renderer"], "headless": config["headless"]})

import carb
import carb.settings
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.storage.native import get_assets_root_path

# Setup
rep.orchestrator.set_capture_on_play(False)
carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
rep.set_global_seed(config["seed"])
rng = np.random.default_rng(config["seed"])

# Load environment
assets_root = get_assets_root_path()
if not assets_root:
    carb.log_error("Cannot resolve assets root path")
    simulation_app.close()
    exit(1)

env_url = assets_root + config["env_url"]
print(f"[SDG] Loading stage: {env_url}")
stage_utils.open_stage(env_url)
stage = stage_utils.get_current_stage()

# Create SDG scope
stage.DefinePrim("/SDG", "Scope")

# Spawn objects (modern API: add_reference_to_stage + functional.modify.semantics)
spawned = {}
for obj_cfg in config["objects"]:
    label = obj_cfg["label"]
    spawned[label] = []
    for i in range(obj_cfg["count"]):
        prim_path = f"/SDG/{label}_{i}"
        stage_utils.add_reference_to_stage(
            usd_path=assets_root + obj_cfg["url"],
            path=prim_path,
        )
        p = stage.GetPrimAtPath(prim_path)
        # Pose at spawn
        from pxr import Gf, UsdGeom

        xf = UsdGeom.Xformable(p)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(rng.uniform(-15, 5), rng.uniform(-5, 10), 0))
        quat = euler_angles_to_quaternion([0, 0, rng.uniform(0, 2 * math.pi)])
        xf.AddOrientOp().Set(Gf.Quatf(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
        # Semantic label for annotators
        rep.functional.modify.semantics(p, {"class": label}, mode="add")
        spawned[label].append(p)

# Camera
cam_cfg = config["camera"]
cam = rep.functional.create.camera(
    focal_length=cam_cfg["focal_length"],
    focus_distance=cam_cfg["focus_distance"],
    clipping_range=tuple(cam_cfg["clipping_range"]),
    name="DataCam",
    parent="/SDG",
)

resolution = tuple(config["resolution"])
rp = rep.create.render_product(cam, resolution, name="main_view")

# Writer
output_dir = config["output_dir"]
if not os.path.isabs(output_dir):
    output_dir = os.path.join(os.getcwd(), output_dir)
print(f"[SDG] Output: {output_dir}")

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=output_dir, **config["annotations"])
writer.attach(rp)

# Capture loop
num_frames = config["num_frames"]
rt_subframes = config["rt_subframes"]
pos_min = np.array(cam_cfg["position_min"])
pos_max = np.array(cam_cfg["position_max"])

# Pick a random look-at target from all spawned objects
all_prims = [p for group in spawned.values() for p in group]

print(f"[SDG] Generating {num_frames} frames (rt_subframes={rt_subframes})")
for i in range(num_frames):
    # Randomize camera
    target = all_prims[rng.integers(0, len(all_prims))]
    rep.functional.modify.pose(
        cam,
        position_value=rng.uniform(pos_min, pos_max).tolist(),
        look_at_value=target,
        look_at_up_axis=(0, 0, 1),
    )

    # Randomize object poses every 5 frames
    if i % 5 == 0:
        for label, obj_prims in spawned.items():
            for op in obj_prims:
                rep.functional.modify.pose(
                    op,
                    position_value=(rng.uniform(-15, 5), rng.uniform(-5, 10), 0),
                    rotation_value=list(euler_angles_to_quaternion([0, 0, rng.uniform(0, 2 * math.pi)])),
                )

    rep.orchestrator.step(delta_time=0.0, rt_subframes=rt_subframes)
    if (i + 1) % 10 == 0:
        print(f"[SDG] Frame {i + 1}/{num_frames}")

# Cleanup
print("[SDG] Waiting for writes to complete...")
rep.orchestrator.wait_until_complete()
writer.detach()
rp.destroy()
print(f"[SDG] Done. {num_frames} frames written to {output_dir}")
simulation_app.close()
