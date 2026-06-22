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

"""Validates Isaac Sim USD assets for missing, absolute, external, deleted, and out-of-scope references, deprecated graph nodes, and non-anonymous render deltas. Optionally runs the Omniverse ValidationEngine and writes text and CSV reports."""

import argparse
import csv

# Initialize simulation app first
from isaacsim import SimulationApp

# Parse command-line arguments
parser = argparse.ArgumentParser(description="USD Asset Validator Script")
parser.add_argument("--use-validation-engine", action="store_true", help="Run Omniverse ValidationEngine")
parser.add_argument("--result-file", default="asset_validation_results.txt", help="Path to the text result file")
parser.add_argument("--csv-file", default="asset_validation_results.csv", help="Path to the CSV result file")
parser.add_argument("--no-window", action="store_true", help="Run without a window")
args, _ = parser.parse_known_args()

# Launch the simulator
launch_config = {"disable_viewport_updates": True}
if args.no_window:
    launch_config["headless"] = True

kit = SimulationApp(launch_config=launch_config)

# Enable the asset_validator extension for the Validation Engine
from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.asset_validator.core")

# Isaac Sim imports
import carb
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage
from isaacsim.storage.native import (
    find_files_recursive,
    get_stage_references,
    is_absolute_path,
    is_path_external,
    is_valid_usd_file,
    prim_has_missing_references,
)

# Omniverse Validation Engine
from omni.asset_validator.core import IssueSeverity, ValidationEngine
from omni.physx import get_physx_interface
from pxr import PhysxSchema, Sdf, Usd, UsdGeom


class AssetValidator:
    """Class to handle USD asset validation logic."""

    def __init__(self, root_path: str, use_validation_engine: bool = False) -> None:
        """Initialize the AssetValidator.

        Args:
            root_path: Root path for asset validation.
            use_validation_engine: Whether to run the Omniverse ValidationEngine.
        """
        self._root_path = root_path
        self._use_validation_engine = use_validation_engine

    def validate(self, usd_path: str) -> list[str]:
        """Validate a single USD file against all validation checks.

        Args:
            usd_path: Path to the USD file to validate.

        Returns:
            List of validation errors found.
        """
        file_results = []

        # Open stage and wait until loaded
        try:
            open_stage(usd_path)
            # Wait for stage to load
            for _ in range(10):  # Basic wait
                kit.update()
            while is_stage_loading():
                kit.update()
        except Exception as e:
            return [f"Failed to open stage: {e}"]

        stage = get_current_stage()
        if not stage:
            return ["Failed to retrieve stage after opening"]

        # Check stage-level issues
        # file_results.extend(self.check_stage_units(stage))
        file_results.extend(self.check_abs_refs(usd_path))
        file_results.extend(self.check_external_refs(usd_path))
        file_results.extend(self.check_rel_refs_scope(usd_path))
        file_results.extend(self.check_incorrect_delta(stage))

        # Check prim-level issues
        for prim in stage.Traverse():
            file_results.extend(self.check_missing_ref(prim))
            file_results.extend(self.check_deleted_ref(prim))
            file_results.extend(self.check_deleted_payload(prim))
            file_results.extend(self.check_properties(usd_path, prim))
            # file_results.extend(self.check_physics_scene(prim))
            file_results.extend(self.check_deprecated_og(prim))

        return file_results

    def run_validation_engine(self, usd_path: str) -> list[str]:
        """Run Omniverse ValidationEngine on a USD file.

        Args:
            usd_path: Path to the USD file.

        Returns:
            List of errors found by the ValidationEngine.
        """
        if not self._use_validation_engine:
            return []

        engine = ValidationEngine(initRules=False)
        # engine.enable_rule(OmniDefaultPrimChecker)

        issues = engine.validate(usd_path)

        errors = []
        for issue in issues:
            if issue.severity == IssueSeverity.FAILURE:
                msg = f"[{issue.severity.name}] {issue.message}"
                errors.append(msg)

        return errors

    def check_stage_units(self, stage: Usd.Stage) -> list[str]:
        """Check if stage units are in meters.

        Args:
            stage: The USD stage to check.

        Returns:
            List of error messages if stage units are not in meters.
        """
        units = UsdGeom.GetStageMetersPerUnit(stage)
        if units != 1.0:
            return ["stage has units which are not in meters"]
        return []

    def check_physics_schema(self) -> list[str]:
        """Check if physics schema is up to date.

        Returns:
            List of error messages if physics schema is outdated.
        """
        if get_physx_interface().check_backwards_compatibility() is True:
            return ["stage has an old physics schema"]
        return []

    def check_missing_ref(self, prim: Usd.Prim) -> list[str]:
        """Check if prim has missing references.

        Args:
            prim: The USD prim to check.

        Returns:
            List of error messages if prim has missing references.
        """
        if prim_has_missing_references(prim) is True:
            return [f"stage has missing references for {prim.GetPath()}"]
        return []

    def check_external_refs(self, usd_path: str) -> list[str]:
        """Check if USD file has external references.

        Args:
            usd_path: Path to the USD file.

        Returns:
            List of error messages if USD file has external references.
        """
        ext_refs = [
            i for i in get_stage_references(usd_path, resolve_relatives=False) if is_path_external(i, self._root_path)
        ]
        if len(ext_refs) != 0:
            return [f"stage has external references {ext_refs}"]
        return []

    def check_abs_refs(self, usd_path: str) -> list[str]:
        """Check if USD file has absolute references.

        Args:
            usd_path: Path to the USD file.

        Returns:
            List of error messages if USD file has absolute references.
        """
        abs_refs = [i for i in get_stage_references(usd_path) if is_absolute_path(i)]
        if len(abs_refs) != 0:
            return [f"stage has absolute references {abs_refs}"]
        return []

    def check_rel_refs_scope(self, usd_path: str) -> list[str]:
        """Check if USD file has relative references that are outside of our asset server.

        Args:
            usd_path: Path to the USD file.

        Returns:
            List of error messages if USD file has relative references.
        """
        rel_refs = [i for i in get_stage_references(usd_path) if not is_absolute_path(i)]
        rel_refs_outside_asset_server = []
        if len(rel_refs) != 0:
            for ref in rel_refs:
                if "../Isaac/" in ref:
                    rel_refs_outside_asset_server.append(ref)
            if len(rel_refs_outside_asset_server) != 0:
                return [f"stage: has relative references outside of asset server {rel_refs_outside_asset_server}"]
        return []

    def check_properties(self, usd_path: str, prim: Usd.Prim) -> list[str]:
        """Check if prim properties contain absolute references.

        Args:
            usd_path: Path to the USD file.
            prim: The USD prim to check.

        Returns:
            List of error messages if prim properties contain absolute references.
        """
        abs_refs = []
        try:
            if prim.GetAttributes():
                for attr in prim.GetAttributes():
                    if attr.GetTypeName() == Sdf.ValueTypeNames.String:
                        val = attr.Get()
                        if val is not None and "omniverse://" in str(val):
                            abs_refs.append(val)

            if len(abs_refs) != 0:
                return [f"Prim {prim.GetPath()} contains absolute reference {abs_refs}"]
            return []
        except Exception as e:
            carb.log_error(f"{e} fail to check {usd_path}, {prim.GetPath()}")
            return []

    def check_deleted_ref(self, prim: Usd.Prim) -> list[str]:
        """Check if prim has deleted references.

        Args:
            prim: The USD prim to check.

        Returns:
            List of error messages if prim has deleted references.
        """
        stage = prim.GetStage()
        ref_prim_spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath())
        if ref_prim_spec:
            references_info = ref_prim_spec.GetInfo("references")
            if len(references_info.deletedItems) > 0:
                return [f"stage has deleted references {references_info.deletedItems}"]
        return []

    def check_deleted_payload(self, prim: Usd.Prim) -> list[str]:
        """Check if prim has deleted payloads.

        Args:
            prim: The USD prim to check.

        Returns:
            List of error messages if prim has deleted payloads.
        """
        stage = prim.GetStage()
        ref_prim_spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath())
        if ref_prim_spec:
            if ref_prim_spec.HasInfo("payload"):
                payload_info = ref_prim_spec.GetInfo("payload")
                if len(payload_info.deletedItems) > 0:
                    return [f"stage has deleted payload {payload_info.deletedItems}"]
        return []

    def check_physics_scene(self, prim: Usd.Prim) -> list[str]:
        """Check if physics scene has valid configuration.

        Args:
            prim: The USD prim to check.

        Returns:
            List of error messages if physics scene has invalid configuration.
        """
        errors = []
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            physics_api = PhysxSchema.PhysxSceneAPI(prim)
            if physics_api.GetEnableGPUDynamicsAttr().Get():
                errors.append(f"Physics scene: {prim.GetPath()} has gpu dynamics enabled")
            if physics_api.GetBroadphaseTypeAttr().Get() != "MBP":
                errors.append(
                    f"Physics scene: {prim.GetPath()} has {physics_api.GetBroadphaseTypeAttr().Get()} broadphase"
                )
        return errors

    def check_deprecated_og(self, prim: Usd.Prim) -> list[str]:
        """Check if prim uses deprecated Omniverse Graph nodes.

        Args:
            prim: The USD prim to check.

        Returns:
            List of error messages if prim uses deprecated Omniverse Graph nodes.
        """
        if prim.HasAttribute("node:type"):
            type_attr = prim.GetAttribute("node:type")
            value = type_attr.Get()
            if value and "omni.graph.nodes.MakeArray" in str(value):
                return [f"stage has node {prim.GetPath()} of type omni.graph.nodes.MakeArray"]
        return []

    def check_incorrect_delta(self, stage: Usd.Stage) -> list[str]:
        """Check if prim uses incorrect delta.

        Args:
            stage: The USD stage to check.

        Returns:
            List of error messages if prim has delta on non-anonymous layer.
        """
        results = []
        paths = ["/Render/PostProcess"]
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if prim is not None and prim.IsValid():
                stack = prim.GetPrimStack()

                if not all(s.layer.anonymous for s in stack):
                    results.append(f"stage has delta for {path} on non-anonymous layer")
        return results


def main() -> None:
    """Main entry point for the asset validation script."""
    # Wait for simulator to initialize
    for _ in range(10):
        kit.update()

    try:
        # Setup paths and filters
        settings = carb.settings.get_settings()
        root_path = settings.get("/persistent/isaac/asset_root/default")
        if not root_path:
            carb.log_warn("Could not find default asset root path, defaulting to empty string.")
            root_path = ""

        search_paths = [
            root_path + "/Isaac",
        ]
        exclude_paths = ["Environments/Outdoor/Rivermark", ".thumbs"]

        # Find all USD files to validate
        all_files = find_files_recursive(search_paths)
        usd_files = [file for file in all_files if is_valid_usd_file(file, exclude_paths)]
        print(f"Found a total of {len(all_files)} files and {len(usd_files)} USD files")

        # Initialize validator
        validator = AssetValidator(root_path=root_path, use_validation_engine=args.use_validation_engine)

        # Create/clear results files
        with open(args.result_file, "w") as f:
            f.write("USD Asset Validation Results\n")
            f.write("==========================\n\n")

        with open(args.csv_file, "w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["USD File", "Error", "Source"])

        # Process each USD file and collect validation results
        all_errors = []
        total_files = len(usd_files)

        with open(args.result_file, "a") as results_file, open(args.csv_file, "a", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)

            for i, usd_path in enumerate(usd_files):
                print(f"[{i + 1}/{total_files}] Validating: {usd_path}")

                try:
                    print(f"Opening stage: {usd_path}")

                    # Validate the file (custom checks)
                    file_errors = validator.validate(usd_path)
                    for error in file_errors:
                        results_file.write(f"\n--- {usd_path} ---\n  • {error}\n")
                        csv_writer.writerow([usd_path, error, "Custom"])

                    # Optionally run ValidationEngine
                    engine_errors = validator.run_validation_engine(usd_path)
                    for error in engine_errors:
                        results_file.write(f"\n--- {usd_path} ---\n  • {error}\n")
                        csv_writer.writerow([usd_path, error, "ValidationEngine"])

                    # Flush to ensure data is written even if crash occurs
                    results_file.flush()
                    csvfile.flush()

                    all_errors.extend(file_errors)
                    all_errors.extend(engine_errors)

                except Exception as e:
                    err_msg = f"Unexpected error validating {usd_path}: {e}"
                    # traceback.print_exc()
                    carb.log_error(err_msg)
                    csv_writer.writerow([usd_path, err_msg, "Internal"])
                    results_file.write(f"\n--- {usd_path} ---\n  • {err_msg}\n")
                    results_file.flush()
                    csvfile.flush()

        # Report overall validation results
        print(f"\nValidation complete. Found {len(all_errors)} errors.")
        print(f"Results saved to: {args.result_file} and {args.csv_file}")

    finally:
        # Ensure clean application shutdown
        kit.close()


if __name__ == "__main__":
    main()
