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

"""Standalone Asset Transformer Runner.

This script runs the Asset Transformer on a USD file using a rule profile JSON.
It can be used to organize and restructure USD assets according to predefined rules.

Usage:
    ./python.sh source/standalone_examples/api/isaacsim.asset.transformer/run_asset_transformer.py \


        --input /path/to/input.usd \
        --profile /path/to/profile.json \
        --output /path/to/output_package

    # Run built-in test with UR10e asset and Isaac Sim Structure profile:
    ./python.sh source/standalone_examples/api/isaacsim.asset.transformer/run_asset_transformer.py --test
"""

import argparse
import json
import os
import sys
import tempfile

from isaacsim import SimulationApp

# Parse arguments before starting SimulationApp (required for headless mode)
parser = argparse.ArgumentParser(
    description="Run Asset Transformer on a USD file using a rule profile.",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Examples:
  # Basic usage with default profile:
  ./python.sh run_asset_transformer.py --input asset.usd --output /tmp/output

  # With custom profile:
  ./python.sh run_asset_transformer.py --input asset.usd --profile custom_rules.json --output /tmp/output

  # Save execution log to file:
  ./python.sh run_asset_transformer.py --input asset.usd --output /tmp/output --log /tmp/report.json

  # Run built-in test:
  ./python.sh run_asset_transformer.py --test
""",
)
parser.add_argument(
    "--input",
    "-i",
    type=str,
    default=None,
    help="Path to the input USD file to transform",
)
parser.add_argument(
    "--profile",
    "-p",
    type=str,
    default=None,
    help="Path to the rule profile JSON file (optional, uses default organize_routing.json if not specified)",
)
parser.add_argument(
    "--output",
    "-o",
    type=str,
    default=None,
    help="Output package root directory where transformed assets will be written",
)
parser.add_argument(
    "--log",
    "-l",
    type=str,
    default=None,
    help="Path to save the execution report JSON (optional)",
)
parser.add_argument(
    "--headless",
    action="store_true",
    default=True,
    help="Run in headless mode (default: True)",
)
parser.add_argument(
    "--test",
    default=False,
    action="store_true",
    help="Run in test mode: uses UR10e test asset with Isaac Sim Structure profile into a temp directory",
)
args, unknown = parser.parse_known_args()

# In test mode, defer path resolution until after SimulationApp is initialized
# (extension paths are only available after kit startup)
if not args.test:
    if not args.input:
        parser.error("--input is required unless --test is specified")
    if not args.output:
        parser.error("--output is required unless --test is specified")

# Validate input file exists (non-test mode)
input_path = None
profile_path = None
output_path = None

if not args.test:
    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)

    # Validate profile file if specified
    if args.profile:
        profile_path = os.path.abspath(args.profile)
        if not os.path.isfile(profile_path):
            print(f"Error: Profile file not found: {profile_path}")
            sys.exit(1)

    output_path = os.path.abspath(args.output)

if not args.test:
    print("Starting Isaac Sim for Asset Transformer...")
    print(f"  Input:   {input_path}")
    print(f"  Profile: {profile_path or '(default organize_routing.json)'}")
    print(f"  Output:  {output_path}")
else:
    print("Starting Isaac Sim for Asset Transformer (test mode)...")

# Start SimulationApp in headless mode
kit = SimulationApp({"headless": args.headless})

# Now import the rest after SimulationApp is initialized
import carb
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.app import get_extension_path as get_extension_path_from_name


def get_rules_extension_path() -> str:
    """Get the root path of the isaacsim.asset.transformer.rules extension.

    Returns:
        Absolute path to the extension root.

    Raises:
        RuntimeError: If the rules extension is not found.
    """
    extension_path = get_extension_path_from_name("isaacsim.asset.transformer.rules")
    if not extension_path:
        raise RuntimeError("Could not find isaacsim.asset.transformer.rules extension")
    return extension_path


def get_default_profile_path() -> str:
    """Get path to the default .json profile.

    Returns:
            Absolute path to the default rule profile.

    Raises:
            RuntimeError: If the rules extension is not found.
    """
    return os.path.join(get_rules_extension_path(), "data", "isaacsim_structure.json")


def get_test_config() -> tuple[str, str, str]:
    """Get pre-configured test paths: UR10e input, Isaac Sim Structure profile, temp output dir.

    Returns:
        Tuple of (input_path, profile_path, output_dir).

    Raises:
        RuntimeError: If test asset or profile is not found.
    """
    ext_path = get_rules_extension_path()
    test_input = os.path.join(ext_path, "data", "tests", "ur10e", "ur10e.usd")
    test_profile = os.path.join(ext_path, "data", "isaacsim_structure.json")

    if not os.path.isfile(test_input):
        raise RuntimeError(f"Test asset not found: {test_input}")
    if not os.path.isfile(test_profile):
        raise RuntimeError(f"Test profile not found: {test_profile}")

    test_output = tempfile.mkdtemp(prefix="asset_transformer_test_")
    return test_input, test_profile, test_output


def run_asset_transformer(
    input_stage_path: str,
    profile_json_path: str,
    output_package_root: str,
    log_path: str | None = None,
) -> bool:
    """Run the asset transformer with the specified profile.

    Args:
        input_stage_path: Path to the source USD stage.
        profile_json_path: Path to the rule profile JSON file.
        output_package_root: Destination directory for transformed assets.
        log_path: Optional path to save the execution report JSON.

    Returns:
        True if transformation was successful, False otherwise.
    """
    print("Running Asset Transformer")
    print(f"  Input stage: {input_stage_path}")
    print(f"  Profile: {profile_json_path}")
    print(f"  Output root: {output_package_root}")

    # Load the rule profile
    with open(profile_json_path, encoding="utf-8") as f:
        profile = RuleProfile.from_json(f.read())

    print(f"\nProfile: {profile.profile_name} (v{profile.version or 'N/A'})")
    print(f"  Rules: {len(profile.rules)} total, {sum(1 for r in profile.rules if r.enabled)} enabled")

    # Create output directory if it doesn't exist
    os.makedirs(output_package_root, exist_ok=True)

    # Run the transformer
    manager = AssetTransformerManager()
    report = manager.run(
        input_stage=input_stage_path,
        profile=profile,
        package_root=output_package_root,
    )

    # Print summary
    print("\n" + "=" * 60)
    print("Execution Report")
    print("=" * 60)
    print(f"  Started:  {report.started_at}")
    print(f"  Finished: {report.finished_at}")
    print(f"  Results:  {len(report.results)} rules executed")

    all_success = True
    for result in report.results:
        status = "[PASS]" if result.success else "[FAIL]"
        print(f"\n  {status} {result.rule.name} ({result.rule.type})")
        if result.error:
            print(f"    Error: {result.error}")
            all_success = False
        if result.affected_stages:
            print(f"    Affected stages: {', '.join(result.affected_stages)}")

    print("\n" + "=" * 60)

    # Save log if requested
    if log_path:
        log_dir = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(log_path, "w", encoding="utf-8") as log_file:
            json.dump(json.loads(report.to_json()), log_file, indent=2)
        print(f"Report saved to: {log_path}")

    if all_success:
        print(f"\nAsset transformation completed successfully.")
        print(f"  Output location: {output_package_root}")
    else:
        print(f"\nAsset transformation completed with errors.")

    return all_success


def main() -> int:
    """Main entry point for the standalone asset transformer."""
    global input_path, profile_path, output_path

    test_tmpdir = None
    try:
        # Enable required extensions
        print("\nEnabling required extensions...")
        enable_extension("isaacsim.asset.transformer")
        enable_extension("isaacsim.asset.transformer.rules")

        # Allow extensions to initialize
        kit.update()

        # Resolve test mode paths now that extensions are available
        if args.test:
            input_path, profile_path, test_tmpdir = get_test_config()
            output_path = test_tmpdir
            print(f"  Test input:   {input_path}")
            print(f"  Test profile: {profile_path}")
            print(f"  Test output:  {output_path}")

        # Get profile path (use default if not specified)
        actual_profile_path = profile_path
        if not actual_profile_path:
            actual_profile_path = get_default_profile_path()
            print(f"  Using default profile: {actual_profile_path}")

        # Run the transformer
        success = run_asset_transformer(
            input_stage_path=input_path,
            profile_json_path=actual_profile_path,
            output_package_root=output_path,
            log_path=args.log,
        )

        return 0 if success else 1

    except Exception as e:
        carb.log_error(f"Asset transformation failed: {e}")
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()
        return 1

    finally:
        # Flush pending async operations before shutdown
        kit.update()
        kit.update()

        if test_tmpdir:
            import shutil

            shutil.rmtree(test_tmpdir, ignore_errors=True)
            print(f"Cleaned up test output: {test_tmpdir}")


if __name__ == "__main__":
    exit_code = main()
    kit.close()
