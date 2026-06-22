# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Export USD robot assets to URDF format."""

import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.app
import omni.usd
from isaacsim.asset.exporter.urdf.converter import UsdToUrdfConverter
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.storage.native import get_assets_root_path

UR10E_ASSET = "Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"

parser = argparse.ArgumentParser(description="Export a USD robot to URDF using Isaac Sim.")
parser.add_argument("--usd-path", required=False, default=None, help="Path or Nucleus URI to the USD file to export.")
parser.add_argument(
    "--output-dir",
    required=False,
    default=None,
    help="Directory to write the exported URDF and mesh files. Defaults to the same directory as the input USD.",
)
parser.add_argument(
    "--root-prim",
    required=False,
    default=None,
    help="Prim path of the articulation root. Uses the stage default prim if omitted.",
)
parser.add_argument(
    "--mesh-prefix",
    required=False,
    default="./",
    help="Path prefix for mesh references in the URDF (default: './').",
)
parser.add_argument(
    "--variant",
    action="append",
    metavar="SET=SELECTION",
    default=None,
    help="Variant selection to apply on the root prim before export (repeatable). Example: --variant Physics=PhysX --variant LOD=high",
)
parser.add_argument(
    "--test",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Run in test mode: exports the UR10e from the Isaac Sim asset library into a temp directory.",
)
args, unknown = parser.parse_known_args()


def _open_stage(usd_path: str) -> None:
    """Open a USD stage and block until fully loaded."""
    omni.usd.get_context().open_stage(usd_path)
    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()


def _validate_urdf(output_path: str) -> bool:
    """Run basic structural validation on the exported URDF.

    Returns:
        True when validation passes.

    Raises:
        RuntimeError: On any validation failure.

    """
    if not os.path.exists(output_path):
        raise RuntimeError(f"URDF file was not created: {output_path}")

    tree = ET.parse(output_path)
    root = tree.getroot()

    if root.tag != "robot":
        raise RuntimeError(f"Root element should be <robot>, got <{root.tag}>")

    if not root.get("name"):
        raise RuntimeError("Robot name attribute is empty")

    links = root.findall("link")
    joints = root.findall("joint")

    if len(links) < 2:
        raise RuntimeError(f"Expected at least 2 links, got {len(links)}")
    if len(joints) < 1:
        raise RuntimeError(f"Expected at least 1 joint, got {len(joints)}")

    for link in links:
        if not link.get("name"):
            raise RuntimeError("Link has empty or missing name attribute")

    for joint in joints:
        if not joint.get("name"):
            raise RuntimeError("Joint has empty or missing name attribute")
        if not joint.get("type"):
            raise RuntimeError(f"Joint '{joint.get('name')}' has no type attribute")
        if joint.find("parent") is None:
            raise RuntimeError(f"Joint '{joint.get('name')}' missing <parent>")
        if joint.find("child") is None:
            raise RuntimeError(f"Joint '{joint.get('name')}' missing <child>")

    mesh_elems = list(root.iter("mesh"))
    urdf_dir = os.path.dirname(output_path)
    for mesh in mesh_elems:
        fn = mesh.get("filename", "")
        if not fn:
            raise RuntimeError("Mesh element has empty filename")
        resolved = os.path.normpath(os.path.join(urdf_dir, fn))
        if not os.path.isfile(resolved):
            raise RuntimeError(f"Mesh file '{fn}' does not exist at '{resolved}'")

    return True


def main() -> int:
    """Run the URDF export workflow with CLI configuration.

    Returns:
        Exit code integer.

    """
    try:
        if args.test:
            assets_root = get_assets_root_path()
            if assets_root is None:
                raise RuntimeError("Could not find Isaac Sim assets folder")
            usd_path = assets_root + "/" + UR10E_ASSET
            if args.output_dir is None:
                args.output_dir = tempfile.mkdtemp(prefix="urdf_export_test_")
        else:
            if args.usd_path is None:
                raise RuntimeError(
                    "USD file path is required. Use --usd-path to specify a file or --test to use a test asset."
                )
            usd_path = args.usd_path
            if not usd_path.startswith(("omniverse://", "http://", "https://")):
                usd_path = os.path.abspath(usd_path)

        usd_basename = os.path.splitext(os.path.basename(usd_path))[0]

        if args.output_dir is not None:
            output_dir = os.path.abspath(args.output_dir)
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, f"{usd_basename}.urdf")
        else:
            output_path = None

        print(f"Opening USD stage: {usd_path}")
        _open_stage(usd_path)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError(f"Failed to open USD stage: {usd_path}")

        variant_selections = None
        if args.variant:
            variant_selections = {}
            for entry in args.variant:
                if "=" not in entry:
                    raise RuntimeError(f"Invalid --variant format '{entry}'. Expected SET=SELECTION.")
                k, v = entry.split("=", 1)
                variant_selections[k] = v

        print("Exporting to URDF...")
        converter = UsdToUrdfConverter(
            stage=stage,
            root_prim_path=args.root_prim,
            mesh_dir_name="meshes",
            mesh_path_prefix=args.mesh_prefix,
            variant_selections=variant_selections,
        )
        result_path = converter.convert(output_path)

        print(f"URDF export successful. Output: {result_path}")

        if args.test:
            print("Running validation...")
            _validate_urdf(result_path)
            print("Validation passed.")

        simulation_app.close()
        return 0
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        simulation_app.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
