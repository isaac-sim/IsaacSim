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

"""Set omni:sensor:tickRate to match omni:sensor:Core:scanRateBaseHz for all supported lidar configs.

Opens each lidar sensor USD asset, iterates over its variants (if any), locates the
actual layer where the OmniLidar prim is authored (the referenced variant USDA for
variant configs, or the root layer for non-variant configs), reads scanRateBaseHz,
authors tickRate in that same layer, and saves it.

Usage:
    ./python.sh source/tools/isaacsim.sensors.rtx/set_lidar_tick_rates.py
    ./python.sh source/tools/isaacsim.sensors.rtx/set_lidar_tick_rates.py --dry-run
"""

import argparse

parser = argparse.ArgumentParser(description="Set tickRate to scanRateBaseHz for all supported lidar configs")
parser.add_argument("--dry-run", action="store_true", help="Print what would change without saving")

args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.client
from isaacsim.core.utils.extensions import enable_extension
from pxr import Sdf, Usd

enable_extension("isaacsim.sensors.rtx")

from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS, SUPPORTED_LIDAR_VARIANT_SET_NAME
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()


def find_omni_lidar_prims(stage):
    prims = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "OmniLidar":
            prims.append(prim)
    return prims


def find_layer_for_attribute(prim, attr_name, root_layer):
    """Find the layer where an attribute is authored.

    Prefers a non-root layer (i.e. the referenced variant USDA) so that the
    tickRate opinion lands in the same file as scanRateBaseHz.  Falls back to
    the root layer for non-variant configs where everything lives in one file.
    """
    attr = prim.GetAttribute(attr_name)
    if attr:
        for prop_spec in attr.GetPropertyStack(Usd.TimeCode.Default()):
            if prop_spec.layer != root_layer:
                return prop_spec.layer, prop_spec.path.GetPrimPath()
    return root_layer, prim.GetPath()


def set_tick_rate_in_layer(layer, prim_path, scan_rate):
    """Author omni:sensor:tickRate in the given layer at the given prim path."""
    prim_spec = layer.GetPrimAtPath(prim_path)
    if not prim_spec:
        return False
    tick_attr = prim_spec.attributes.get("omni:sensor:tickRate")
    if tick_attr:
        tick_attr.default = float(scan_rate)
    else:
        tick_attr = Sdf.AttributeSpec(prim_spec, "omni:sensor:tickRate", Sdf.ValueTypeNames.Float)
        tick_attr.default = float(scan_rate)
    return True


def save_layer(layer):
    """Save a layer, using omni.client for Nucleus paths."""
    layer_id = layer.identifier
    content = layer.ExportToString()
    result = omni.client.write_file(layer_id, bytes(content, "utf-8"))
    if result != omni.client.Result.OK:
        print(f"  ERROR: omni.client.write_file returned {result} for {layer_id}")
        return False
    print(f"  Saved: {layer_id}")
    return True


total_updated = 0
total_skipped = 0

for config_path, variants in SUPPORTED_LIDAR_CONFIGS.items():
    full_path = assets_root + config_path
    print(f"\n{'='*60}")
    print(f"Config: {config_path}")

    stage = Usd.Stage.Open(full_path)
    if not stage:
        print(f"  ERROR: Could not open stage at {full_path}")
        total_skipped += 1
        continue

    root_layer = stage.GetRootLayer()
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        print(f"  WARNING: No default prim, skipping")
        total_skipped += 1
        continue

    if variants:
        vset = default_prim.GetVariantSets().GetVariantSet(SUPPORTED_LIDAR_VARIANT_SET_NAME)
        available = vset.GetVariantNames() if vset else []
        if not available:
            print(f"  ERROR: No '{SUPPORTED_LIDAR_VARIANT_SET_NAME}' variant set or it is empty")
            total_skipped += 1
            continue

        for variant_name in sorted(variants):
            if variant_name not in available:
                print(f"  {variant_name}: WARNING - variant not found in USD, skipping")
                total_skipped += 1
                continue

            vset.SetVariantSelection(variant_name)
            lidar_prims = find_omni_lidar_prims(stage)
            if not lidar_prims:
                print(f"  {variant_name}: WARNING - no OmniLidar prim found")
                total_skipped += 1
                continue

            for lidar_prim in lidar_prims:
                scan_rate = lidar_prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get()
                if scan_rate is None:
                    print(f"  {variant_name}: {lidar_prim.GetPath()} - no scanRateBaseHz, skipping")
                    total_skipped += 1
                    continue

                target_layer, target_path = find_layer_for_attribute(
                    lidar_prim, "omni:sensor:Core:scanRateBaseHz", root_layer
                )
                layer_id = target_layer.identifier
                if set_tick_rate_in_layer(target_layer, target_path, scan_rate):
                    print(f"  {variant_name}: tickRate = {scan_rate}  [{layer_id}]")
                    if not args.dry_run:
                        save_layer(target_layer)
                    total_updated += 1
                else:
                    print(f"  {variant_name}: {lidar_prim.GetPath()} - could not author tickRate")
                    total_skipped += 1
    else:
        lidar_prims = find_omni_lidar_prims(stage)
        if not lidar_prims:
            print(f"  WARNING: No OmniLidar prim found, skipping")
            total_skipped += 1
            continue

        for lidar_prim in lidar_prims:
            scan_rate = lidar_prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get()
            if scan_rate is None:
                print(f"  {lidar_prim.GetPath()}: no scanRateBaseHz, skipping")
                total_skipped += 1
                continue

            target_layer, target_path = find_layer_for_attribute(
                lidar_prim, "omni:sensor:Core:scanRateBaseHz", root_layer
            )
            layer_id = target_layer.identifier
            if set_tick_rate_in_layer(target_layer, target_path, scan_rate):
                print(f"  tickRate = {scan_rate}  [{layer_id}]")
                if not args.dry_run:
                    save_layer(target_layer)
                total_updated += 1
            else:
                print(f"  {lidar_prim.GetPath()}: could not author tickRate")
                total_skipped += 1

print(f"\n{'='*60}")
print(f"Done. Updated: {total_updated}, Skipped/Warnings: {total_skipped}")
if args.dry_run:
    print("(no files were modified)")

simulation_app.close()
