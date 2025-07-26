# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Asset validation test script that checks USD assets for common issues."""

import argparse
import csv

# Standard library imports
import os

# Initialize simulation app first
from isaacsim import SimulationApp

# Parse command-line arguments
parser = argparse.ArgumentParser(description="USD Asset Validator Script")
parser.add_argument("--use-validation-engine", action="store_true", help="Run Omniverse ValidationEngine")
args, _ = parser.parse_known_args()

kit = SimulationApp(launch_config={"disable_viewport_updates": True})

# Enable the asset_validator extension for the Validation Engine
from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.asset_validator.core")

# Isaac Sim imports
import carb
import omni
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
from omni.asset_validator.core import IssueSeverity, OmniDefaultPrimChecker, ValidationEngine
from omni.physx import get_physx_interface
from pxr import PhysxSchema, Sdf, UsdGeom

# Configuration
RESULTS_FILE = "asset_validation_results.txt"
CSV_RESULTS_FILE = "asset_validation_results.csv"


def run_validation_engine_on_usd(usd_path):
    """Run Omniverse ValidationEngine on a USD file."""
    engine = ValidationEngine(initRules=False)

    # Enable only the desired custom rule (optional)
    # engine.enable_rule(OmniDefaultPrimChecker)

    issues = engine.validate(usd_path)

    errors = []
    for issue in issues:
        if issue.severity == IssueSeverity.FAILURE:
            msg = f"[{issue.severity.name}] {issue.message}"
            errors.append(msg)

    return errors


def check_stage_units(stage, usd_path):
    """Check if stage units are in meters.

    Args:
        stage: The USD stage to check.
        usd_path: Path to the USD file.

    Returns:
        List of error messages if stage units are not in meters.
    """
    units = UsdGeom.GetStageMetersPerUnit(stage)
    if units != 1.0:
        return [f"stage has units which are not in meters"]
    return []


def check_physics_schema(usd_path):
    """Check if physics schema is up to date.

    Args:
        usd_path: Path to the USD file.

    Returns:
        List of error messages if physics schema is outdated.
    """
    if get_physx_interface().check_backwards_compatibility() is True:
        return [f"stage has an old physics schema"]
    return []


def check_missing_ref(usd_path, prim):
    """Check if prim has missing references.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim has missing references.
    """
    if prim_has_missing_references(prim) is True:
        return [f"stage has missing references for {prim}"]
    return []


def check_external_refs(root_path, usd_path):
    """Check if USD file has external references.

    Args:
        root_path: Root path for asset validation.
        usd_path: Path to the USD file.

    Returns:
        List of error messages if USD file has external references.
    """
    ext_refs = [i for i in get_stage_references(usd_path, resolve_relatives=False) if is_path_external(i, root_path)]
    if len(ext_refs) != 0:
        return [f"stage has external references {ext_refs}"]
    return []


def check_abs_refs(usd_path):
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


def check_rel_refs_scope(usd_path):
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


def check_properties(usd_path, prim):
    """Check if prim properties contain absolute references.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim properties contain absolute references.
    """
    abs_refs = []
    try:
        if prim.GetAttributes() is not None:
            for attr in prim.GetAttributes():
                if attr.GetTypeName() == Sdf.ValueTypeNames.String:
                    if attr.Get() is not None:
                        if "omniverse://" in attr.Get():
                            abs_refs.append(attr.Get())

        if len(abs_refs) != 0:
            return [f"Prim {prim} contains absolute reference {abs_refs}"]
        return []
    except Exception as e:
        carb.log_error(f"{e} fail to check {usd_path}, {prim}")
        return []


def check_deleted_ref(usd_path, prim):
    """Check if prim has deleted references.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim has deleted references.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
    ref_prim_spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath())
    if ref_prim_spec:
        references_info = ref_prim_spec.GetInfo("references")
        if len(references_info.deletedItems) > 0:
            return [f"stage has deleted references {references_info.deletedItems}"]
    return []


def check_deleted_payload(usd_path, prim):
    """Check if prim has deleted payloads.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim has deleted payloads.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
    ref_prim_spec = stage.GetRootLayer().GetPrimAtPath(prim.GetPath())
    if ref_prim_spec:
        payload_info = ref_prim_spec.GetInfo("payload")
        if len(payload_info.deletedItems) > 0:
            return [f"stage has deleted payload {payload_info.deletedItems}"]
    return []


def check_physics_scene(usd_path, prim):
    """Check if physics scene has valid configuration.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if physics scene has invalid configuration.
    """
    errors = []
    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        physics_api = PhysxSchema.PhysxSceneAPI(prim)
        if physics_api.GetEnableGPUDynamicsAttr().Get():
            errors.append(f"Physics scene: {prim} has gpu dynamics enabled")
        if physics_api.GetBroadphaseTypeAttr().Get() != "MBP":
            errors.append(f"Physics scene: {prim} has {physics_api.GetBroadphaseTypeAttr().Get()} broadphase")
    return errors


def check_deprecated_og(usd_path, prim):
    """Check if prim uses deprecated Omniverse Graph nodes.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim uses deprecated Omniverse Graph nodes.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
    if prim.HasAttribute("node:type"):
        type_attr = prim.GetAttribute("node:type")
        value = type_attr.Get()
        if "omni.graph.nodes.MakeArray" in value:
            return [f"stage has node {prim.GetPath()} of type omni.graph.nodes.MakeArray"]
    return []


def check_incorrect_delta(usd_path):
    """Check if prim uses incorrect delta.

    Args:
        usd_path: Path to the USD file.
        prim: The USD prim to check.

    Returns:
        List of error messages if prim has delta on non-anonymous layer.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
    results = []
    paths = ["/Render/PostProcess"]
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if prim is not None and prim.IsValid():
            stack = prim.GetPrimStack()

            if not all([s.layer.anonymous for s in stack]):
                results.append(f"stage has delta for {path} on non-anonymous layer")
    return results


def validate_usd_file(usd_path, root_path):
    """Validate a single USD file against all validation checks.

    Args:
        usd_path: Path to the USD file to validate.
        root_path: Root path for asset validation.

    Returns:
        List of validation errors found.
    """
    file_results = []

    # Open stage and wait until loaded
    open_stage(usd_path)
    kit.update()
    while is_stage_loading():
        kit.update()

    stage = get_current_stage()
    if not stage:
        return ["Failed to open stage"]

    # Check stage-level issues
    # file_results.extend(check_stage_units(stage, usd_path))
    file_results.extend(check_abs_refs(usd_path))
    file_results.extend(check_external_refs(root_path, usd_path))
    file_results.extend(check_rel_refs_scope(usd_path))
    file_results.extend(check_incorrect_delta(usd_path))
    # Check prim-level issues
    for prim in stage.Traverse():
        file_results.extend(check_missing_ref(usd_path, prim))
        file_results.extend(check_deleted_ref(usd_path, prim))
        file_results.extend(check_deleted_payload(usd_path, prim))
        file_results.extend(check_properties(usd_path, prim))
        # file_results.extend(check_physics_scene(usd_path, prim))
        file_results.extend(check_deprecated_og(usd_path, prim))

    return file_results


# Wait for simulator to initialize
for i in range(10):
    kit.update()

# Main validation process
try:
    # Setup paths and filters
    root_path = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
    search_paths = [
        root_path + "/Isaac",
    ]
    exclude_paths = ["Environments/Outdoor/Rivermark", ".thumbs"]

    # Find all USD files to validate
    all_files = find_files_recursive(search_paths)
    usd_files = [file for file in all_files if is_valid_usd_file(file, exclude_paths)]
    print(f"Found a total of {len(all_files)} files and {len(usd_files)} USD files")

    # Create/clear results file
    with open(RESULTS_FILE, "w") as f:
        f.write(f"USD Asset Validation Results\n")
        f.write(f"==========================\n\n")

    # Create/clear CSV results file
    with open(CSV_RESULTS_FILE, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["USD File", "Error", "Source"])

    # Process each USD file and collect validation results
    all_errors = []
    total_files = len(usd_files)

    with open(RESULTS_FILE, "a") as results_file, open(CSV_RESULTS_FILE, "a", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)

        for i, usd_path in enumerate(usd_files):
            print(f"[{i+1}/{total_files}] Validating: {usd_path}")

            try:
                print(f"Opening stage: {usd_path}")

                # Validate the file (custom checks)
                file_errors = validate_usd_file(usd_path, root_path)
                for error in file_errors:
                    results_file.write(f"\n--- {usd_path} ---\n  • {error}\n")
                    csv_writer.writerow([usd_path, error, "Custom"])
                results_file.flush()
                csvfile.flush()

                # Optionally run ValidationEngine
                if args.use_validation_engine:
                    print(f"Running ValidationEngine on: {usd_path}")
                    engine_errors = run_validation_engine_on_usd(usd_path)
                    for error in engine_errors:
                        results_file.write(f"\n--- {usd_path} ---\n  • {error}\n")
                        csv_writer.writerow([usd_path, error, "ValidationEngine"])
                    results_file.flush()
                    csvfile.flush()

                # Add to overall results
                all_errors.extend(file_errors)
                if args.use_validation_engine:
                    all_errors.extend(engine_errors)

            except Exception as e:
                err_msg = f"{e}"
                carb.log_error(err_msg)
                csv_writer.writerow([usd_path, err_msg, "Internal"])
                results_file.write(f"\n--- {usd_path} ---\n  • {err_msg}\n")
                results_file.flush()
                csvfile.flush()

    # Report overall validation results
    print(f"\nValidation complete. Found {len(all_errors)} errors.")
    print(f"Results saved to: {RESULTS_FILE} and {CSV_RESULTS_FILE}")

finally:
    # Ensure clean application shutdown
    kit.close()
