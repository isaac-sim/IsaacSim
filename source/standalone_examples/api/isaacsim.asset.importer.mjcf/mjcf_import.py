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

"""Import and convert MJCF files to USD format."""

import argparse
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.app


def _enable_scene_optimizer_extension():
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)


def _enable_robot_schema_extension():
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)


_enable_scene_optimizer_extension()
_enable_robot_schema_extension()

from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
from usd.schema.isaac.robot_schema import Attributes as RobotSchemaAttributes
from usd.schema.isaac.robot_schema import get_allowed_tokens

parser = argparse.ArgumentParser(description="Import an MJCF file using Isaac Sim.")
parser.add_argument(
    "--mjcf",
    required=False,
    default=None,
    help="Path to an MJCF file (.xml) or a directory containing MJCF files to import.",
)
parser.add_argument(
    "--usd-path",
    required=False,
    default=None,
    help="Directory to write converted USD assets.",
)
parser.add_argument(
    "--import-scene",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Import the MJCF simulation settings along with the model.",
)
parser.add_argument(
    "--merge-mesh",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Merge meshes after conversion.",
)
parser.add_argument(
    "--debug-mode",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Enable debug mode and keep intermediate outputs.",
)
parser.add_argument(
    "--collision-from-visuals",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Generate collision geometry from visuals.",
)
parser.add_argument(
    "--collision-type",
    default=None,
    choices=["Convex Hull", "Convex Decomposition", "Bounding Sphere", "Bounding Cube"],
    help="Collision geometry type when --collision-from-visuals is enabled.",
)
parser.add_argument(
    "--allow-self-collision",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Allow self-collision for the imported asset.",
)
parser.add_argument(
    "--test",
    action=argparse.BooleanOptionalAction,
    default=False,
    help="Run in test mode: uses nv_ant.xml test asset into a temp directory",
)
parser.add_argument(
    "--robot-type",
    default="Default",
    choices=list(get_allowed_tokens(RobotSchemaAttributes.ROBOT_TYPE)),
    help="Robot type for the Isaac robot schema.",
)
parser.add_argument(
    "--fix-base",
    action=argparse.BooleanOptionalAction,
    default=None,
    help=(
        "Tri-state base-type toggle. --fix-base anchors the robot to the world via a fixed joint; "
        "--no-fix-base strips any existing world-to-root fixed joint so the robot is floating-base; "
        "omitting the flag leaves the source asset's base authoring untouched."
    ),
)
parser.add_argument(
    "--link-density",
    type=float,
    default=None,
    help="Default density (kg/m^3) for rigid body links with no explicit mass.",
)
parser.add_argument(
    "--override-gain-type",
    default=None,
    help='MuJoCo actuator gain type (e.g. "fixed").',
)
parser.add_argument(
    "--override-bias-type",
    default=None,
    help='MuJoCo actuator bias type (e.g. "affine").',
)
parser.add_argument(
    "--override-gain-prm",
    type=float,
    nargs="+",
    default=None,
    help="MuJoCo actuator gain parameter array (up to 10 floats). Example: --override-gain-prm 100 0 0 0 0 0 0 0 0 0",
)
parser.add_argument(
    "--override-bias-prm",
    type=float,
    nargs="+",
    default=None,
    help="MuJoCo actuator bias parameter array (up to 10 floats). Example: --override-bias-prm 0 -100 -10 0 0 0 0 0 0 0",
)
parser.add_argument(
    "--run-asset-transformer",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Run asset transformer after conversion.",
)
parser.add_argument(
    "--run-multi-physics-conversion",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Run multi-physics conversion after conversion.",
)
args, unknown = parser.parse_known_args()


def _collect_mjcf_files(path):
    """Return a list of .xml MJCF file paths from *path*.

    If *path* is a single file it is returned as-is.  If it is a directory the
    directory tree is walked and every file ending with ``.xml`` is collected.
    """
    if os.path.isfile(path):
        return [path]

    mjcf_files = []
    for root, _dirs, files in os.walk(path):
        for fname in sorted(files):
            if fname.lower().endswith(".xml"):
                mjcf_files.append(os.path.join(root, fname))
    return sorted(mjcf_files)


def _resolve_usd_paths(source_files, base_usd_path, input_dir=None):
    """Return a dict mapping each source file to a unique ``usd_path``.

    When *input_dir* is given (batch / directory import) the relative directory
    structure of each source file is mirrored inside *base_usd_path* so that
    files with the same basename in different subdirectories never collide.

    For a single-file import or when *base_usd_path* is ``None`` the returned
    value for every file is simply *base_usd_path*.
    """
    if base_usd_path is None or input_dir is None or len(source_files) <= 1:
        return {f: base_usd_path for f in source_files}

    mapping = {}
    for src in source_files:
        rel = os.path.relpath(os.path.dirname(src), input_dir)
        mapping[src] = os.path.normpath(os.path.join(base_usd_path, rel))
    return mapping


def _apply_cli_overrides(import_config, usd_path_override=None):
    """Apply CLI flag overrides onto *import_config* (mutates in-place)."""
    usd_path = usd_path_override if usd_path_override is not None else args.usd_path
    if usd_path is not None:
        import_config.usd_path = os.path.abspath(usd_path)
    if args.import_scene is not None:
        import_config.import_scene = args.import_scene
    if args.merge_mesh is not None:
        import_config.merge_mesh = args.merge_mesh
    if args.debug_mode is not None:
        import_config.debug_mode = args.debug_mode
    if args.collision_from_visuals is not None:
        import_config.collision_from_visuals = args.collision_from_visuals
    if args.collision_type is not None:
        import_config.collision_type = args.collision_type
    if args.allow_self_collision is not None:
        import_config.allow_self_collision = args.allow_self_collision
    if args.robot_type is not None:
        import_config.robot_type = args.robot_type
    if args.fix_base is not None:
        import_config.fix_base = args.fix_base
    if args.link_density is not None:
        import_config.link_density = args.link_density
    if args.override_gain_type is not None:
        import_config.override_gain_type = args.override_gain_type
    if args.override_bias_type is not None:
        import_config.override_bias_type = args.override_bias_type
    if args.override_gain_prm is not None:
        import_config.override_gain_prm = args.override_gain_prm
    if args.override_bias_prm is not None:
        import_config.override_bias_prm = args.override_bias_prm
    if args.run_asset_transformer is not None:
        import_config.run_asset_transformer = args.run_asset_transformer
    if args.run_multi_physics_conversion is not None:
        import_config.run_multi_physics_conversion = args.run_multi_physics_conversion


def main():
    """Run the MJCF import workflow with CLI configuration."""
    try:
        mjcf_path = None
        if args.test:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf")
            extension_path = ext_manager.get_extension_path(ext_id)
            mjcf_files = [os.path.join(extension_path, "data", "mjcf", "nv_ant.xml")]
        else:
            if args.mjcf is None:
                raise RuntimeError(
                    "MJCF path is required. Use --mjcf to specify a file/directory or --test to use a test asset."
                )
            mjcf_path = os.path.abspath(args.mjcf) if not os.path.isabs(args.mjcf) else args.mjcf

            if not os.path.exists(mjcf_path):
                raise RuntimeError(f"MJCF path not found: {mjcf_path}")

            mjcf_files = _collect_mjcf_files(mjcf_path)
            if not mjcf_files:
                raise RuntimeError(f"No .xml files found in directory: {mjcf_path}")

        input_dir = mjcf_path if mjcf_path is not None and os.path.isdir(mjcf_path) else None
        base_usd_path = os.path.abspath(args.usd_path) if args.usd_path else None
        usd_paths = _resolve_usd_paths(mjcf_files, base_usd_path, input_dir)

        succeeded, failed = [], []
        for mjcf_file in mjcf_files:
            print(f"\nImporting: {mjcf_file}")
            try:
                import_config = MJCFImporterConfig()
                import_config.mjcf_path = mjcf_file
                _apply_cli_overrides(import_config, usd_path_override=usd_paths[mjcf_file])

                importer = MJCFImporter(import_config)
                output_usd = importer.import_mjcf()
            except Exception as exc:
                print(f"Failed to import {mjcf_file}: {exc!r}")
                failed.append((mjcf_file, repr(exc)))
                continue

            if output_usd:
                print(f"Success: {output_usd}")
                succeeded.append(output_usd)
            else:
                print(f"Failed to import: {mjcf_file}")
                failed.append((mjcf_file, "importer returned no output path"))

        print(f"\nImport complete - {len(succeeded)} succeeded, {len(failed)} failed.")
        if failed:
            print("Failed files:")
            for path, reason in failed:
                print(f"  - {path}: {reason}")

        simulation_app.update()
        simulation_app.update()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            simulation_app.update()

        simulation_app.close()
    except Exception as e:
        print(f"Error: {e}")
        simulation_app.close()


if __name__ == "__main__":
    main()
