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

"""Import and convert URDF files to USD format."""

import argparse
import os
import tempfile

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.app


def _enable_scene_optimizer_extension() -> None:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)


def _enable_robot_schema_extension() -> None:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)


_enable_scene_optimizer_extension()
_enable_robot_schema_extension()

from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from usd.schema.isaac.robot_schema import Attributes as RobotSchemaAttributes
from usd.schema.isaac.robot_schema import get_allowed_tokens

parser = argparse.ArgumentParser(description="Import a URDF file using Isaac Sim.")
parser.add_argument(
    "--urdf",
    required=False,
    default=None,
    help="Path to a URDF file (.urdf) or a directory containing URDF files to import.",
)
parser.add_argument(
    "--usd-path",
    required=False,
    default=None,
    help="Directory to write converted USD assets.",
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
    help="Run in test mode: uses carter.urdf test asset into a temp directory",
)
parser.add_argument(
    "--ros-package",
    action="append",
    metavar="NAME:PATH",
    help="ROS package mapping in format 'name:path'. Can be specified multiple times for multiple packages.",
)
parser.add_argument(
    "--robot-type",
    default="Default",
    choices=list(get_allowed_tokens(RobotSchemaAttributes.ROBOT_TYPE)),
    help="Robot type for the Isaac robot schema.",
)
parser.add_argument(
    "--merge-fixed-joints",
    action=argparse.BooleanOptionalAction,
    default=None,
    help="Merge fixed joints where possible to optimize the model.",
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
    "--joint-drive-type",
    default=None,
    choices=["force", "acceleration"],
    help="Joint drive type applied to all joints.",
)
parser.add_argument(
    "--joint-target-type",
    default=None,
    choices=["none", "position", "velocity"],
    help="Joint target type applied to all joints.",
)
parser.add_argument(
    "--override-joint-stiffness",
    type=float,
    default=None,
    help="Joint stiffness in Nm/rad (revolute) or N/m (prismatic) applied to all joints.",
)
parser.add_argument(
    "--override-joint-damping",
    type=float,
    default=None,
    help="Joint damping in Nm*s/rad (revolute) or N*s/m (prismatic) applied to all joints.",
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


def _collect_urdf_files(path: str) -> list[str]:
    """Return a list of .urdf file paths from *path*.

    If *path* is a single file it is returned as-is.  If it is a directory the
    directory tree is walked and every file ending with ``.urdf`` is collected.
    """
    if os.path.isfile(path):
        return [path]

    urdf_files = []
    for root, _dirs, files in os.walk(path):
        for fname in sorted(files):
            if fname.lower().endswith(".urdf"):
                urdf_files.append(os.path.join(root, fname))
    return sorted(urdf_files)


def _resolve_usd_paths(
    source_files: list[str], base_usd_path: str | None, input_dir: str | None = None
) -> dict[str, str | None]:
    """Return a dict mapping each source file to a unique ``usd_path``.

    When *input_dir* is given (batch / directory import) the relative directory
    structure of each source file is mirrored inside *base_usd_path* so that
    files with the same basename in different subdirectories never collide.

    For a single-file import or when *base_usd_path* is ``None`` the returned
    value for every file is simply *base_usd_path*.
    """
    if base_usd_path is None or input_dir is None or len(source_files) <= 1:
        return dict.fromkeys(source_files, base_usd_path)

    mapping = {}
    for src in source_files:
        rel = os.path.relpath(os.path.dirname(src), input_dir)
        mapping[src] = os.path.normpath(os.path.join(base_usd_path, rel))
    return mapping


def _apply_cli_overrides(import_config: URDFImporterConfig, usd_path_override: str | None = None) -> None:
    """Apply CLI flag overrides onto *import_config* (mutates in-place)."""
    usd_path = usd_path_override if usd_path_override is not None else args.usd_path
    if usd_path is not None:
        import_config.usd_path = os.path.abspath(usd_path)
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
    if args.merge_fixed_joints is not None:
        import_config.merge_fixed_joints = args.merge_fixed_joints
    if args.fix_base is not None:
        import_config.fix_base = args.fix_base
    if args.link_density is not None:
        import_config.link_density = args.link_density
    if args.joint_drive_type is not None:
        import_config.joint_drive_type = args.joint_drive_type
    if args.joint_target_type is not None:
        import_config.joint_target_type = args.joint_target_type
    if args.override_joint_stiffness is not None:
        import_config.override_joint_stiffness = args.override_joint_stiffness
    if args.override_joint_damping is not None:
        import_config.override_joint_damping = args.override_joint_damping
    if args.run_asset_transformer is not None:
        import_config.run_asset_transformer = args.run_asset_transformer
    if args.run_multi_physics_conversion is not None:
        import_config.run_multi_physics_conversion = args.run_multi_physics_conversion
    if args.ros_package:
        ros_packages = []
        for package_spec in args.ros_package:
            if ":" not in package_spec:
                raise ValueError(f"Invalid ROS package format: {package_spec}. Expected format: 'name:path'")
            name, path = package_spec.split(":", 1)
            ros_packages.append({"name": name.strip(), "path": path.strip()})
        import_config.ros_package_paths = ros_packages


def main() -> None:
    """Run the URDF import workflow with CLI configuration."""
    try:
        urdf_path = None
        if args.test:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
            extension_path = ext_manager.get_extension_path(ext_id)
            urdf_files = [os.path.join(extension_path, "data", "urdf", "robots", "carter", "urdf", "carter.urdf")]
            if args.usd_path is None:
                args.usd_path = tempfile.mkdtemp(prefix="urdf_import_test_")
        else:
            if args.urdf is None:
                raise RuntimeError(
                    "URDF path is required. Use --urdf to specify a file/directory or --test to use a test asset."
                )
            urdf_path = os.path.abspath(args.urdf) if not os.path.isabs(args.urdf) else args.urdf

            if not os.path.exists(urdf_path):
                raise RuntimeError(f"URDF path not found: {urdf_path}")

            urdf_files = _collect_urdf_files(urdf_path)
            if not urdf_files:
                raise RuntimeError(f"No .urdf files found in directory: {urdf_path}")

        input_dir = urdf_path if urdf_path is not None and os.path.isdir(urdf_path) else None
        base_usd_path = os.path.abspath(args.usd_path) if args.usd_path else None
        usd_paths = _resolve_usd_paths(urdf_files, base_usd_path, input_dir)

        succeeded, failed = [], []
        for urdf_file in urdf_files:
            print(f"\nImporting: {urdf_file}")
            try:
                import_config = URDFImporterConfig()
                import_config.urdf_path = urdf_file
                _apply_cli_overrides(import_config, usd_path_override=usd_paths[urdf_file])

                importer = URDFImporter(import_config)
                output_usd = importer.import_urdf()
            except Exception as exc:
                print(f"Failed to import {urdf_file}: {exc!r}")
                failed.append((urdf_file, repr(exc)))
                continue

            if output_usd:
                print(f"Success: {output_usd}")
                succeeded.append(output_usd)
            else:
                print(f"Failed to import: {urdf_file}")
                failed.append((urdf_file, "importer returned no output path"))

        print("=" * 60)
        print(f"Import complete - {len(succeeded)} succeeded, {len(failed)} failed.")
        if failed:
            print("Failed files:")
            for path, reason in failed:
                print(f"  - {path}: {reason}")

        simulation_app.close()
    except Exception as e:
        print(f"Error: {e}")
        simulation_app.close()


if __name__ == "__main__":
    main()
