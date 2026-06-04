# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from pathlib import Path
from typing import Any, Dict, List, Union

USD_EXTENSION = ".usd"  # switch to .usda for debugging


def convert_json_to_usd(json_path: Union[str, Path], usd_path: Union[str, Path], model_config: Dict[str, Any]) -> None:
    """Convert a LiDAR configuration JSON file to a USD file.

    This function reads a LiDAR configuration JSON file and creates a USD file containing
    an OmniLidar prim with the specified configuration. It handles emitter states, attributes,
    and model configuration.

    Args:
        param json_path (Union[str, Path]): Path to the input JSON configuration file.
        param usd_path (Union[str, Path]): Path where the output USD file will be saved.
        param model_config (Dict[str, Any]): Dictionary containing model configuration parameters.

    Raises:
        FileNotFoundError: If the JSON file cannot be found.
        Exception: If there are issues setting attributes or creating the USD prim.
    """
    import json
    from itertools import cycle, islice

    import carb
    import omni.replicator.core as rep
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage

    with open(json_path, "r") as f:
        config = json.load(f)

    profile = config["profile"]
    if "numberOfEmitters" not in profile:
        raise KeyError("numberOfEmitters not found in profile. This field is required.")

    # Test number of emitter states
    emitter_state_count = profile["emitterStateCount"]
    num_emitter_states = len(profile["emitterStates"])
    if emitter_state_count != num_emitter_states:
        carb.log_warn(
            f"Emitter state count {emitter_state_count} does not match the number of emitter states {num_emitter_states}. Using minimum of the two."
        )
        emitter_state_count = min(emitter_state_count, num_emitter_states)

    prim_creation_kwargs = {}
    for i in range(emitter_state_count):
        if i < 2:
            continue
        # Get first provided field in emitter state
        emitter_state_key = next(iter(profile["emitterStates"][i]))
        # Specify a new object of the same type as the value for the keyword argument
        # This value will be populated later
        prim_creation_kwargs[f"omni:sensor:Core:emitterState:s{i+1:03}:{emitter_state_key}"] = type(
            profile["emitterStates"][i][emitter_state_key]
        )()

    # Create a new stage
    create_new_stage()

    # Create the lidar prim
    prim = rep.functional.create.omni_lidar(**prim_creation_kwargs)

    def set_attribute(prim, attribute, value):
        if prim.HasAttribute(attribute):
            try:
                prim.GetAttribute(attribute).Set(value)
                return True
            except Exception as e:
                carb.log_warn(f"Error setting attribute {attribute} to {value}: {e}. Skipping.")
        return False

    # Set attributes which are emitter states
    # azimuthDeg, channelId, elevationDeg, fireTimeNs must all be specifieid
    required_emitter_state_fields = {"azimuthDeg", "channelId", "elevationDeg", "fireTimeNs"}
    for i in range(emitter_state_count):
        # Determine which of the required fields are missing from the JSON profile,
        # then populate the missing fields with a list of zeros of the appropriate length
        emitter_state_fields_as_list = list(profile["emitterStates"][i].keys())
        emitter_state_fields = set(emitter_state_fields_as_list)
        missing_fields = required_emitter_state_fields - emitter_state_fields
        num_emitters = profile["numberOfEmitters"]
        for field in missing_fields:
            if field == "channelId":
                if "numberOfChannels" in profile:
                    carb.log_warn(
                        f"numberOfChannels found in profile, but channelId not found in emitterStates[{i}]. Autogenerating channelId by repeating the range 1 to numberOfChannels."
                    )
                    profile["emitterStates"][i][field] = list(
                        islice(cycle(range(1, profile["numberOfChannels"] + 1)), num_emitters)
                    )
                else:
                    carb.log_warn(
                        f"numberOfChannels not found in profile, and channelId not found in emitterStates[{i}]. Autogenerating channelId by repeating the range 1 to numberOfEmitters."
                    )
                    profile["emitterStates"][i][field] = list(range(1, num_emitters + 1))
                    carb.log_warn(f"Setting numberOfChannels to numberOfEmitters {num_emitters}.")
                    profile["numberOfChannels"] = num_emitters
            else:
                profile["emitterStates"][i][field] = [0] * num_emitters
        # Now iterate over the specified fields and set the attributes
        for field in profile["emitterStates"][i]:
            attribute = f"omni:sensor:Core:emitterState:s{i+1:03}:{field}"
            value = get_usd_value(profile["emitterStates"][i][field])
            if set_attribute(prim, attribute, value):
                continue
            carb.log_warn(
                f"Field emitterState[{i}].{field} found in profile, without corresponding prim attribute {attribute}. Skipping."
            )

    # Set attributes which are not emitter states
    # Note we do this second, to catch any fields that may have been added or changed when iterating over emitter states
    for field in profile:
        if field == "emitterStates":
            continue
        attribute = f"omni:sensor:Core:{field}"
        value = get_usd_value(profile[field])
        if set_attribute(prim, attribute, value):
            continue
        carb.log_warn(f"Field {field} found in profile, without corresponding prim attribute {attribute}. Skipping.")

    # Set model config
    for key, value in model_config.items():
        attribute = f"omni:sensor:{key}"
        if set_attribute(prim, attribute, value):
            continue
        carb.log_warn(f"Field {key} found in model config, without corresponding prim attribute {attribute}. Skipping.")

    # Save the stage as a USD file
    stage = get_current_stage()
    stage.SetDefaultPrim(prim)
    stage.Export(str(usd_path))


def get_usd_value(value: Any) -> Any:
    """Convert a value to its USD-compatible format.

    This function handles special cases for string values and ensures values are in the
    correct format for USD attributes.

    Args:
        param value (Any): The value to convert to USD format.

    Returns:
        Any: The converted value in USD-compatible format.
    """
    if isinstance(value, str):
        if value == "solidState":
            value = "solid_state"
        return value.upper()
    return value


def convert_single_json(json_path: Union[str, Path], model_config: Dict[str, Any]) -> Path:
    """Convert a single JSON file to USD.

    This function converts a single LiDAR configuration JSON file to a USD file,
    maintaining the same base name but changing the extension.

    Args:
        param json_path (Union[str, Path]): Path to the input JSON configuration file.
        param model_config (Dict[str, Any]): Dictionary containing model configuration parameters.

    Returns:
        Path: Path object pointing to the generated USD file.

    Raises:
        FileNotFoundError: If the JSON file cannot be found.
    """
    json_path = Path(json_path)
    usd_path = json_path.with_suffix(USD_EXTENSION)
    convert_json_to_usd(json_path, usd_path, model_config)
    return usd_path


def process_lidar_configs(config_dir: Union[str, Path], model_config: Dict[str, Any]) -> List[Path]:
    """Process all JSON files in a directory and convert them to USD.

    This function recursively searches for JSON files in the specified directory
    and converts each one to a USD file.

    Args:
        param config_dir (Union[str, Path]): Directory containing JSON configuration files.
        param model_config (Dict[str, Any]): Dictionary containing model configuration parameters.

    Returns:
        List[Path]: List of paths to the generated USD files.

    Raises:
        FileNotFoundError: If the config directory cannot be found.
    """
    config_path = Path(config_dir)
    converted_files = []
    for json_file in config_path.glob("**/*.json"):
        usd_path = json_file.with_suffix(USD_EXTENSION)
        convert_json_to_usd(json_file, usd_path, model_config)
        converted_files.append(usd_path)
        print(f"Generated: {usd_path}")
    return converted_files


# Read variant to JSON file map
def build_variant_to_usd_map(variant_to_json_map: Union[str, Path], converted_files: List[Path]) -> Dict[str, Path]:
    """Build a mapping from variant names to USD file paths.

    This function reads a mapping file that associates variant names with JSON files
    and converts it to a mapping of variant names to USD files.

    Args:
        param variant_to_json_map (Union[str, Path]): Path to the variant-to-JSON mapping file.
        param converted_files (List[Path]): List of paths to converted USD files.

    Returns:
        Dict[str, Path]: Dictionary mapping variant names to USD file paths.

    Raises:
        FileNotFoundError: If a referenced JSON file is not found in the converted files.
    """
    variant_to_usd_map = {}
    with open(variant_to_json_map, "r") as f:
        for line in f.readlines():
            variant_name, json_file = line.strip().split(",")
            if variant_name[0].isdigit():
                carb.log_warn(f"Variant name {variant_name} starts with a digit. This is not allowed. Skipping.")
                continue
            if " " in variant_name:
                carb.log_warn(f"Variant name {variant_name} contains space(s). Converting to underscore(s).")
                variant_name = variant_name.replace(" ", "_")

            # Find matching JSON file in converted files
            matching_file = [file for file in converted_files if Path(file).stem == Path(json_file).stem]
            if len(matching_file) == 0:
                raise FileNotFoundError(f"JSON file {json_file} not found in converted files.")
            else:
                matching_file = matching_file[0]
            variant_to_usd_map[variant_name] = matching_file.with_suffix(USD_EXTENSION)
    return variant_to_usd_map


def build_lidar_usd_with_variants(
    case_usd: Path, variant_to_usd_map: Dict[str, Path], model_config: Dict[str, Any]
) -> Path:
    """Build a USD file with variants from a case USD file and variant mappings.

    This function creates a new USD file that includes a case reference and sets up
    variants for different LiDAR configurations.

    Args:
        param case_usd (Path): Path to the case USD file.
        param variant_to_usd_map (Dict[str, Path]): Dictionary mapping variant names to USD file paths.
        param model_config (Dict[str, Any]): Dictionary containing model configuration parameters.

    Returns:
        Path: Path object pointing to the generated USD file with variants.

    Raises:
        Exception: If there are issues creating the USD prim or setting up variants.
    """

    import numpy as np
    from isaacsim.core.utils.rotations import euler_angles_to_quat
    from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
    from pxr import Gf, Sdf, UsdGeom

    # Create a new stage
    create_new_stage()
    stage = get_current_stage()

    root_prim_path = f"/{model_config['modelVendor']}_{model_config['modelName']}"
    root_prim = stage.DefinePrim(root_prim_path, "Xform")

    # Add reference to case USD, assuming it is in the same directory as the USD we're building
    add_reference_to_stage(usd_path=str(case_usd), prim_path=f"{root_prim_path}/case")

    # Add an OmniLidar prim
    sensor = stage.DefinePrim(f"{root_prim_path}/sensor", "OmniLidar")
    UsdGeom.Xformable(sensor).AddTranslateOp()
    sensor.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*model_config["sensorTranslation"]))
    quat = euler_angles_to_quat(np.array([*model_config["sensorOrientation"]]))
    UsdGeom.Xformable(sensor).AddOrientOp()
    sensor.GetAttribute("xformOp:orient").Set(Gf.Quatf(*quat))

    # Add a variant set
    variant_set = root_prim.GetVariantSets().AddVariantSet("sensor")

    # Iterate over variant names, adding referenced as absolute paths for each one.
    from omni.kit.variant.editor.commands import EditVariantCommand

    for variant, usd_file in variant_to_usd_map.items():
        variant_set.AddVariant(variant)
        cmd = EditVariantCommand(
            prim_path=root_prim.GetPath(),
            variant_set_name="sensor",
            cmd_name="AddReference",
            cmd_args={"stage": stage, "prim_path": sensor.GetPath(), "reference": Sdf.Reference(str(usd_file))},
            target_variant=variant,
        )
        variant_set.SetVariantSelection(variant)
        cmd.do()

    # Save the USD file
    stage.SetDefaultPrim(root_prim)
    usd_path = case_usd.parent / Path(f"{model_config['modelVendor']}_{model_config['modelName']}{USD_EXTENSION}")
    stage.GetRootLayer().Export(str(usd_path))
    return usd_path


def make_references_relative(usd_path: Union[str, Path]) -> None:
    """Convert absolute references in a USD file to relative references.

    This function processes a USD file and converts any absolute references to
    relative references, making the file more portable.

    Args:
        param usd_path (Union[str, Path]): Path to the USD file to process.

    Raises:
        Exception: If there are issues opening or modifying the USD file.
    """
    # Reopen the USD file and make all the references relative
    from isaacsim.storage.native import get_stage_references, is_absolute_path, path_dirname, path_relative
    from pxr import Usd, UsdUtils

    stage = Usd.Stage.Open(str(usd_path))
    stage_path = stage.GetRootLayer().realPath

    abs_refs = [i for i in get_stage_references(stage_path) if is_absolute_path(i)]
    if not abs_refs:
        return

    base_dir = path_dirname(stage_path)

    all_layers, _, _ = UsdUtils.ComputeAllDependencies(stage_path)

    def make_relative(asset_path):
        if is_absolute_path(asset_path):
            new_path = path_relative(asset_path, base_dir)
            print(f"Updating absolute path {asset_path} to relative path {new_path}")
            return new_path
        # Also check that relative paths stay in scope
        if "../Isaac/" in asset_path:
            new_path = asset_path.replace("../Isaac/", "", 1)
            print(f"Updating relative path {asset_path} to relative path {new_path}")
            return new_path
        return asset_path

    for layer in all_layers:
        UsdUtils.ModifyAssetPaths(layer, make_relative)

    stage.GetRootLayer().Export(str(usd_path))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert LiDAR configuration JSON files to USD files containing OmniLidar prim."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--dir", help="Directory containing JSON files to convert")
    group.add_argument("--files", "-f", nargs="+", help="Specific JSON files to convert")
    parser.add_argument("--market-name", "-m", help="Market name", type=str, default="Generic")
    parser.add_argument("--model-name", "-n", help="Model name", type=str, default="LidarCore")
    parser.add_argument("--model-vendor", "-v", help="Model vendor", type=str, default="NVIDIA")
    parser.add_argument("--model-version", "-r", help="Model version", type=str, default="0.0.0")
    parser.add_argument("--case-usd", help="Case USD file", type=str, default=None)
    parser.add_argument("--variant-to-json-map", help="Variant to JSON file map", type=str, default=None)
    parser.add_argument(
        "--sensor-translation",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        help="Sensor position in stage, in world coordinates",
    )
    parser.add_argument(
        "--sensor-orientation",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        help="Sensor orientation in stage as Euler angles, in degrees",
    )
    args, _ = parser.parse_known_args()

    from isaacsim import SimulationApp

    kit = SimulationApp()
    import carb
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.kit.variant.editor")

    model_config = {
        "marketName": args.market_name,
        "modelName": args.model_name,
        "modelVendor": args.model_vendor,
        "modelVersion": args.model_version,
        "sensorTranslation": args.sensor_translation,
        "sensorOrientation": args.sensor_orientation,
    }

    if args.dir:
        # Process all files in directory
        converted_files = process_lidar_configs(args.dir, model_config)
    else:
        # Process specific files
        converted_files = []
        for json_file in args.files:
            usd_path = convert_single_json(json_file, model_config)
            converted_files.append(usd_path)
            print(f"Generated: {usd_path}")

    if args.case_usd and args.variant_to_json_map:
        if not Path(args.variant_to_json_map).exists():
            raise FileNotFoundError(f"Variant to JSON file map {args.variant_to_json_map} does not exist.")
        if not Path(args.case_usd).exists():
            raise FileNotFoundError(f"Case USD file {args.case_usd} does not exist.")

        variant_to_usd_map = build_variant_to_usd_map(args.variant_to_json_map, converted_files)
        usd_with_variants = build_lidar_usd_with_variants(Path(args.case_usd), variant_to_usd_map, model_config)
        make_references_relative(usd_with_variants)
        print(f"Generated: {usd_with_variants}")

    kit.close()
