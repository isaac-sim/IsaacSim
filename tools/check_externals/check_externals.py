# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import fnmatch
import json
import os
import platform
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from glob import glob

# Add the root directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import packmanapi

from tools.check_externals.download_all_kits import get_package_combinations
from tools.check_externals.license_finder import find_license_file


def detect_platform():
    """Detect the current platform and return appropriate platform strings.

    Returns:
        tuple: (platform_abi, platform_target) strings for the current system.

    Raises:
        RuntimeError: If running on an unsupported platform.
    """
    system = platform.system().lower()
    if system == "linux":
        return "manylinux_2_35_x86_64", "linux-x86_64"
    elif system == "windows":
        return "windows-x86_64", "windows-x86_64"
    else:
        raise RuntimeError(f"Unsupported platform: {system}")


def should_filter_package(platform_filter, pkg):
    """Check if a package should be filtered out based on platform filter.

    The package is filtered out only if the platform_filter matches ALL platforms
    listed in the package's platforms attribute.

    Args:
        platform_filter: String to filter out platforms
        pkg: Package element with platforms and version attributes

    Returns:
        bool: True if the package should be filtered out, False otherwise
    """
    if not platform_filter:
        return False

    # Check version string for platform filter
    version = pkg.get("version", "")
    if platform_filter.lower() in version.lower():
        return True

    # Check platforms attribute
    platforms = pkg.get("platforms", "")
    if not platforms:
        return False

    # Split platforms by whitespace and check if platform_filter matches ALL of them
    platform_list = platforms.split()
    if not platform_list:
        return False

    # Only filter out if the platform_filter matches ALL platforms
    return all(platform_filter.lower() in platform.lower() for platform in platform_list)


def gather_package_data(
    files,
    config,
    platform_abi,
    platform_target,
    package=None,
    platform_filter=None,
    full_license_details=False,
    exclude_packages=None,
    allowed_external=None,
):
    """Gather package data from XML files.

    Args:
        files: List of XML files to process
        config: Config to check (i.e. release or debug)
        platform_abi: Platform ABI string
        platform_target: Platform target string
        package: Optional specific package to check
        platform_filter: Optional string to filter out platforms (e.g., 'linux' or 'aarch64')
        full_license_details: If True, include individual PACKAGE-LICENSES files in output
        exclude_packages: Optional list of package names to exclude from processing
        allowed_external: Optional list of package names that are allowed to be external/private

    Returns:
        tuple: (results_by_file, deps_count_by_file, json_output, full_package_listing)
    """
    results_by_file = defaultdict(lambda: defaultdict(list))
    deps_count_by_file = defaultdict(lambda: defaultdict(int))
    json_output = defaultdict(dict)
    full_package_listing = defaultdict(lambda: defaultdict(list))
    imported_packages = {}  # Store package info from imported files
    filtered_package_names = set()  # Track which package names were filtered

    if exclude_packages:
        print(f"Excluding packages: {exclude_packages}")

    def should_exclude_package(package_name):
        """Check if a package should be excluded based on the exclude_packages list.

        Args:
            package_name: Name of the package to check

        Returns:
            bool: True if the package should be excluded, False otherwise
        """
        if not exclude_packages:
            return False

        for exclude_name in exclude_packages:
            # Direct exact match
            if exclude_name == package_name:
                print(f"    Excluding package '{package_name}' (exact match with '{exclude_name}')")
                return True
            # Substring match (case insensitive)
            if exclude_name.lower() in package_name.lower():
                print(f"    Excluding package '{package_name}' (contains '{exclude_name}')")
                return True
        return False

    def substitute_variables(path, platforms_attr=None):
        """Substitute variables in a path string.

        Args:
            path: Path string that may contain variables
            platforms_attr: Optional platforms attribute to determine ABI vs target preference

        Returns:
            str: Path with variables substituted
        """
        path = path.replace("${config}", config)
        path = path.replace("${platform_target}", platform_target)
        path = path.replace("${platform_target_abi}", platform_abi)

        # For ${platform}, prefer ABI format if both target and ABI are supported
        platform_value = platform_target
        if platforms_attr and platform_abi.lower() in platforms_attr.lower():
            platform_value = platform_abi

        path = path.replace("${platform}", platform_value)
        return path

    def process_imported_file(import_path, filters):
        """Process an imported file and store its package information.

        Args:
            import_path: Path to the imported file
            filters: List of package names to include from the imported file
        """
        try:
            # Check if the file exists before trying to parse it
            if not os.path.exists(import_path):
                print(f"  Skipping imported file (not found): {import_path}")
                return

            print(f"  Processing imported file: {import_path}")
            if filters:
                print(f"    Filters: {filters}")

            import_tree = ET.parse(import_path)
            import_root = import_tree.getroot()

            # Store package info from imported file
            for dependency in import_root.findall(".//dependency"):
                name = dependency.get("name")
                if not name:
                    continue

                # Substitute variables in name for comparison
                substituted_name = substitute_variables(name)

                # Get all package elements within this dependency
                package_elements = dependency.findall("package")

                if package_elements:
                    # Process each package element separately
                    for package_elem in package_elements:
                        package_name = package_elem.get("name")
                        if not package_name:
                            continue

                        # Substitute variables in package name for comparison
                        substituted_package_name = substitute_variables(package_name)

                        # Only store packages that match the filters
                        if filters:

                            def normalize_name(name):
                                """Normalize a name for comparison by removing common separators and suffixes."""
                                # Remove platform suffixes
                                name = name.split(".")[0]  # Remove everything after first dot
                                # Replace common separators with underscores
                                name = name.replace("+", "_").replace("-", "_")
                                return name.lower()

                            def filter_matches_package(filter_name, package_name):
                                """Check if a filter matches a package name using various strategies."""
                                # Direct substring matching
                                if filter_name in package_name or package_name in filter_name:
                                    return True
                                # Exact match
                                if filter_name == package_name:
                                    return True
                                # Normalized match
                                if normalize_name(filter_name) == normalize_name(package_name):
                                    return True
                                # Split filter by separators and check if all parts appear in package
                                filter_parts = filter_name.replace("-", " ").replace("_", " ").split()
                                package_lower = package_name.lower()
                                if all(part.lower() in package_lower for part in filter_parts):
                                    return True
                                return False

                            filter_match = any(
                                filter_matches_package(substitute_variables(filter_name), substituted_package_name)
                                for filter_name in filters
                            )
                            if not filter_match:
                                continue

                        # Check if this package is for our platform
                        platforms = package_elem.get("platforms", "")
                        if platforms:
                            platform_list = platforms.split()
                            # Skip if none of the platforms match our target
                            if not any(
                                platform_abi.lower() in platform.lower() or platform_target.lower() in platform.lower()
                                for platform in platform_list
                            ):
                                continue

                        # Skip if this package should be excluded
                        if should_exclude_package(substituted_package_name):
                            continue

                        # Get version from the package element
                        version = package_elem.get("version", "")
                        # Replace variables in version string
                        version = substitute_variables(version, platforms)

                        # Skip packages without valid version information
                        if not version or version.strip() == "":
                            continue

                        # Use composite key
                        imported_packages[(substituted_name, package_name, version)] = {
                            "dependency_name": substituted_name,
                            "name": package_name,  # Use package name
                            "version": version,
                            "license_file": find_license_file(
                                dependency.get("linkPath", ""), substituted_name, None, full_license_details
                            ),
                        }

                        filtered_package_names.add((substituted_name, package_name, version))
                else:
                    # No package elements, check dependency-level platform info
                    platforms = dependency.get("platforms", "")
                    if platforms:
                        platform_list = platforms.split()
                        # Skip if none of the platforms match our target
                        if not any(
                            platform_target.lower() in platform.lower() or platform_abi.lower() in platform.lower()
                            for platform in platform_list
                        ):
                            print(
                                f"    Skipping {substituted_name} (platform mismatch: {platforms} vs {platform_target})"
                            )
                            continue

                    # Get version from dependency level
                    version = dependency.get("version", "")
                    # Replace variables in version string
                    version = substitute_variables(version, platforms)

                    # Skip dependencies without valid version information
                    if not version or version.strip() == "":
                        print(f"    Skipping {substituted_name} (no version)")
                        continue

                    # Skip if this package should be excluded
                    if should_exclude_package(substituted_name):
                        print(f"    Skipping {substituted_name} (excluded)")
                        continue

                    print(f"    Adding {substituted_name} version {version}")

                    imported_packages[(substituted_name, substituted_name, version)] = {
                        "dependency_name": substituted_name,
                        "name": substituted_name,  # Use dependency name
                        "version": version,
                        "license_file": find_license_file(
                            dependency.get("linkPath", ""), substituted_name, None, full_license_details
                        ),
                    }

                    filtered_package_names.add((substituted_name, substituted_name, version))
        except Exception as e:
            print(f"Warning: Failed to process imported file {import_path}: {e}")

    for file in files:
        tree = ET.parse(file)
        root = tree.getroot()
        file_dir = os.path.dirname(os.path.abspath(file))

        print(
            f"\nChecking {file} with config: {config}, platform_abi: {platform_abi}, platform_target: {platform_target}"
        )

        # Process imports first
        for import_elem in root.findall(".//import"):
            import_path = import_elem.get("path")
            if not import_path:
                continue

            # Substitute variables in import path
            import_path = substitute_variables(import_path)

            # Resolve import path relative to the file being scanned
            import_path = os.path.normpath(os.path.join(file_dir, import_path))

            # Get filters if any
            filters = []
            for filter_elem in import_elem.findall("filter"):
                include = filter_elem.get("include")
                if include:
                    # Substitute variables in filter include value
                    include = substitute_variables(include)
                    filters.append(include)

            # Process the imported file
            process_imported_file(import_path, filters)

        # Process dependencies
        total_deps = 0
        full_package_listing[file][config] = []

        # Process all dependencies from the main file
        for dependency in root.findall(".//dependency"):
            name = dependency.get("name")
            if not name:
                continue

            # Substitute variables in name
            name = substitute_variables(name)

            # Skip if this package matches our filter
            if should_filter_package(platform_filter, dependency) or should_exclude_package(name):
                continue

            total_deps += 1

            # Use imported package info if available, otherwise use local info
            imported_found = False
            for imported_key, imported_info in imported_packages.items():
                dependency_name, package_name, version = imported_key
                # Try to match by dependency name or package name
                if dependency_name == name or package_name == name:
                    imported_found = True
                    if not package or package_name == package:
                        full_package_listing[file][config].append(imported_info)

            if not imported_found:
                # Process all package elements within this dependency
                package_elements = dependency.findall("package")

                if package_elements:
                    # Process each package element separately
                    for package_elem in package_elements:
                        version = package_elem.get("version", "")
                        # Check platforms for version substitution
                        platforms = package_elem.get("platforms", "")
                        # Replace variables in version string
                        version = substitute_variables(version, platforms)

                        # Skip packages without valid version information
                        if not version or version.strip() == "":
                            continue

                        # Check if this package is for our platform
                        if platforms:
                            platform_list = platforms.split()
                            # Skip if none of the platforms match our target
                            if not any(
                                platform_abi.lower() in platform.lower() or platform_target.lower() in platform.lower()
                                for platform in platform_list
                            ):
                                continue

                        # Skip if this package should be excluded
                        package_name = package_elem.get("name") if package_elem is not None else name
                        if should_exclude_package(package_name):
                            continue

                        package_info = {
                            "name": package_name,
                            "version": version,
                            "license_file": find_license_file(
                                dependency.get("linkPath", ""), name, None, full_license_details
                            ),
                        }

                        if not package or name == package:
                            full_package_listing[file][config].append(package_info)
                else:
                    # No package elements, use dependency-level version
                    version = dependency.get("version", "")
                    # Get dependency-level platforms for version substitution
                    dep_platforms = dependency.get("platforms", "")
                    # Replace variables in version string
                    version = substitute_variables(version, dep_platforms)

                    # Skip dependencies without valid version information
                    if not version or version.strip() == "":
                        continue

                    package_info = {
                        "name": name,
                        "version": version,
                        "license_file": find_license_file(
                            dependency.get("linkPath", ""), name, None, full_license_details
                        ),
                    }

                    if not package or name == package:
                        full_package_listing[file][config].append(package_info)

        # Also include packages from imported_packages that don't have corresponding dependency elements
        for imported_key, imported_info in imported_packages.items():
            # Check if this imported package was already processed above
            already_processed = any(
                pkg["name"] == imported_info["name"] and pkg["version"] == imported_info["version"]
                for pkg in full_package_listing[file][config]
            )

            if not already_processed:
                if not package or imported_info["name"] == package:
                    full_package_listing[file][config].append(imported_info)
                    total_deps += 1

        deps_count_by_file[file][config] = total_deps

        _, results = packmanapi.verify(
            file,
            exclude_local=True,
            remotes=["cloudfront"],
            tokens={
                "config": config,
                "platform_target": platform_target,
                "platform_target_abi": platform_abi,
                "platform": platform_abi,
            },
            platform=platform_abi,
            tags={"public": "true"},
        )

        results_by_file[file][config] = results

        # Prepare JSON output for this file and config
        json_output[file][config] = {
            "problem_packages": [
                {
                    "name": result[1].name,
                    "version": result[1].version,
                }
                for result in results
            ],
        }

    return results_by_file, deps_count_by_file, json_output, full_package_listing


def write_output_files(
    results_by_file,
    deps_count_by_file,
    json_output,
    full_package_listing,
    configs,
    full_license_details=False,
    allowed_external=None,
):
    """Write gathered data to output files and print summary.

    Args:
        results_by_file: Dict of verification results by file and config
        deps_count_by_file: Dict of dependency counts by file and config
        json_output: Dict of problem packages by file and config
        full_package_listing: Dict of all packages by file and config
        configs: List of configs that were checked
        full_license_details: If True, include individual PACKAGE-LICENSES files in output
        allowed_external: Optional list of package names that are allowed to be external/private

    Returns:
        tuple: (total_issues, private_package_count)
    """
    # Print summary and count total issues
    total_issues = 0
    total_deps = 0
    print("\nSummary of issues:")
    print("-" * 80)
    for file in sorted(results_by_file.keys()):
        print(f"\n{file}:")
        for config in configs:
            issues = results_by_file[file][config]
            total_issues += len(issues)
            total_deps += deps_count_by_file[file][config]
            print(
                f"  {config}: {len(issues)} issue{'s' if len(issues) != 1 else ''} out of {deps_count_by_file[file][config]} dependencies"
            )

    print(f"\nTotal issues found: {total_issues} across {total_deps} total dependencies")

    # Write CSV output with alphabetically sorted entries
    with open("packman_full_results.csv", "w") as full_csv, open("packman_private_results.csv", "w") as private_csv:
        full_csv.write("Package Name,Version,Public/Private,Expected Public/Private,License Files,License Type\n")
        private_csv.write("Package Name,Version,Public/Private,License Files,License Type\n")

        # Create a dictionary to track unique package entries
        unique_packages = {}
        private_package_count = 0

        for file in full_package_listing:
            for config in configs:
                private_packages = {pkg["name"] for pkg in json_output[file][config]["problem_packages"]}

                for package in full_package_listing[file][config]:
                    name = package["name"]
                    version = package["version"]
                    is_public = "private" if name in private_packages else "public"

                    # Determine expected public/private based on allowed_external list
                    expected_public_private = "private" if allowed_external and name in allowed_external else "public"

                    # Create a key for the package based on its identifying information
                    package_key = (name, version)

                    # Store package info if we haven't seen it before
                    if package_key not in unique_packages:
                        license_info = package.get("license_file")
                        license_type = ""
                        license_files = ""
                        if license_info:
                            # Handle the new dictionary format from find_license_file
                            if isinstance(license_info, dict):
                                license_files_list = []
                                license_types_list = []

                                # Always include main license file first if present
                                if license_info.get("main_license"):
                                    license_files_list.append(license_info["main_license"])

                                # Handle PACKAGE-LICENSES
                                if license_info.get("package_licenses_count", 0) > 0:
                                    if full_license_details:
                                        # Include individual files
                                        license_files_list.extend(license_info.get("package_licenses_files", []))
                                    else:
                                        # Just show count
                                        license_files_list.append(
                                            f"PACKAGE-LICENSES found ({license_info['package_licenses_count']})"
                                        )

                                # Add other license files in priority order
                                license_files_list.extend(license_info.get("nvidia_proprietary", []))
                                license_files_list.extend(license_info.get("mit_licenses", []))
                                license_files_list.extend(license_info.get("spdx_licenses", []))
                                license_files_list.extend(license_info.get("other_licenses", []))

                                # Extract license types
                                if license_info.get("nvidia_proprietary"):
                                    license_types_list.append("NVIDIA Proprietary")
                                if license_info.get("mit_licenses"):
                                    license_types_list.append("MIT")
                                if license_info.get("spdx_license_types"):
                                    license_types_list.extend(license_info.get("spdx_license_types", []))

                                license_files = ";".join(license_files_list)
                                license_type = ";".join(license_types_list)
                            else:
                                # Fallback for old string format
                                license_files = str(license_info)
                        unique_packages[package_key] = {
                            "name": name,
                            "version": version,
                            "public_private": is_public,
                            "expected_public_private": expected_public_private,
                            "license_files": license_files,
                            "license_type": license_type,
                        }
                        # Count private packages
                        if is_public == "private":
                            private_package_count += 1

        # Sort packages with custom priority: private+expected_public first, public+expected_public second, private+expected_private last
        def sort_key(package_info):
            is_private = package_info["public_private"] == "private"
            expected_private = package_info["expected_public_private"] == "private"

            # Priority order: private+expected_public (0), public+expected_public (1), private+expected_private (2)
            if is_private and not expected_private:
                priority = 0
            elif not is_private and not expected_private:
                priority = 1
            else:  # private+expected_private
                priority = 2

            return (priority, package_info["name"].lower(), package_info["version"].lower())

        sorted_packages = sorted(unique_packages.values(), key=sort_key)

        # Write sorted packages to both CSVs
        for package_info in sorted_packages:
            # Generate the CSV line for full results
            full_csv_line = (
                f"{package_info['name']},{package_info['version']},"
                f"{package_info['public_private']},{package_info['expected_public_private']},"
                f"{package_info['license_files']},{package_info['license_type']}\n"
            )
            # Write to full CSV
            full_csv.write(full_csv_line)

            # Write to private CSV if it's a private package AND not in allowed_external
            if package_info["public_private"] == "private" and (
                not allowed_external or package_info["name"] not in allowed_external
            ):
                # Generate CSV line for private results (without expected public/private column)
                private_csv_line = (
                    f"{package_info['name']},{package_info['version']},"
                    f"{package_info['public_private']},{package_info['license_files']},"
                    f"{package_info['license_type']}\n"
                )
                private_csv.write(private_csv_line)
    print("\nFull package listing written to packman_full_results.csv")
    print("Private packages only written to packman_private_results.csv")
    print(f"Found {private_package_count} private package(s)")
    return total_issues, private_package_count


def merge_verification_results(*results_list):
    """Merge multiple verification results into one.

    Args:
        *results_list: List of (results_by_file, deps_count_by_file, json_output, full_package_listing) tuples

    Returns:
        tuple: Merged (results_by_file, deps_count_by_file, json_output, full_package_listing)
    """
    merged_results = defaultdict(lambda: defaultdict(list))
    merged_deps = defaultdict(lambda: defaultdict(int))
    merged_json = defaultdict(dict)
    merged_listing = defaultdict(lambda: defaultdict(list))

    for results in results_list:
        results_by_file, deps_count_by_file, json_output, full_package_listing = results

        # Merge results_by_file
        for file, configs in results_by_file.items():
            for config, results in configs.items():
                merged_results[file][config].extend(results)

        # Merge deps_count
        for file, configs in deps_count_by_file.items():
            for config, count in configs.items():
                merged_deps[file][config] += count

        # Merge JSON output
        for file, configs in json_output.items():
            for config, data in configs.items():
                if config not in merged_json[file]:
                    merged_json[file][config] = data
                else:
                    merged_json[file][config]["problem_packages"].extend(data["problem_packages"])

        # Merge package listings
        for file, configs in full_package_listing.items():
            for config, packages in configs.items():
                merged_listing[file][config].extend(packages)

    return merged_results, merged_deps, merged_json, merged_listing


def verify_externals(
    xml_files=None,
    package=None,
    platform_abi=None,
    platform_target=None,
    platform_filter=None,
    full_license_details=False,
    exclude_packages=None,
    allowed_external=None,
):
    """Verify external dependencies and their licenses.

    Args:
        xml_files: List of packman XML files to scan. If None, scans all files in ./deps/*.packman.xml
        package: Specific package name to check licenses for. If None, checks all packages.
        platform_abi: Platform ABI string (e.g., 'manylinux_2_35_x86_64'). If None, detects from system.
        platform_target: Platform target string (e.g., 'linux-x86_64'). If None, detects from system.
        platform_filter: String to filter out platforms (e.g., 'linux' will skip all linux platforms)
        full_license_details: If True, include individual PACKAGE-LICENSES files in output
        exclude_packages: Optional list of package names to exclude from processing
        allowed_external: Optional list of package names that are allowed to be external/private

    Returns:
        int: Number of issues found (non-zero indicates verification failure)
    """
    if platform_abi is None or platform_target is None:
        platform_abi, platform_target = detect_platform()

    # Get list of files to process
    if xml_files:
        files = [f for f in xml_files if os.path.exists(f)]
        if not files:
            raise FileNotFoundError(f"No specified files found")
    else:
        files = glob("./deps/*.packman.xml")

    # Get combinations from kit-sdk.packman.xml
    _, _, combinations = get_package_combinations("./deps/kit-sdk.packman.xml", platform_filter)

    # Filter combinations based on platform and config if specified
    if platform_filter or platform_target:
        filtered_combinations = []
        for combo in combinations:
            if (
                platform_filter
                and combo.get("platform_target")
                and platform_filter.lower() in combo.get("platform_target").lower()
            ):
                continue
            if platform_target and combo.get("platform_target") != platform_target:
                continue
            if platform_filter or platform_target:
                filtered_combinations.append(combo)
        combinations = filtered_combinations

    if not combinations:
        print("No matching combinations found for the specified platform/config")
        return 1

    # Gather results for each combination
    all_results = []
    for combo in combinations:
        print(f"\nVerifying with combination: {combo}")
        results = gather_package_data(
            files,
            combo["config"],
            combo.get("platform_target_abi", platform_abi),
            combo.get("platform_target", platform_target),
            package,
            platform_filter,
            full_license_details,
            exclude_packages,
            allowed_external,
        )
        all_results.append(results)

    # Merge results
    merged_results = merge_verification_results(*all_results)

    # Write output and get total issues
    configs = list(set(combo["config"] for combo in combinations))
    total_issues, private_package_count = write_output_files(
        *merged_results, configs, full_license_details, allowed_external
    )

    # Return 1 if there are private packages found
    if private_package_count > 0:
        return 1

    return total_issues


def main():
    """Main entry point with argument parsing."""
    parser = argparse.ArgumentParser(description="Check packman external dependencies and licenses.")
    parser.add_argument(
        "--file",
        "-f",
        help="Specific packman XML file to scan. If not provided, scans all files in ./deps/*.packman.xml",
    )
    parser.add_argument(
        "--package", "-p", help="Specific package name to check licenses for. If not provided, checks all packages."
    )
    parser.add_argument(
        "--skip-platform", "-s", help="Skip platforms containing this string (e.g., 'linux' or 'aarch64')"
    )
    parser.add_argument(
        "--platform", "-t", help="Explicitly set platform target (e.g., 'linux-x86_64' or 'windows-x86_64')"
    )
    parser.add_argument("--config", "-c", help="Explicitly set config (e.g., 'release' or 'debug')")
    parser.add_argument(
        "--full-package-license-details", "-l", help="Include full package license details", action="store_true"
    )
    parser.add_argument(
        "--exclude-package", "-e", help="Exclude packages with this name (can be used multiple times)", action="append"
    )
    parser.add_argument(
        "--allowed-external",
        "-a",
        help="Package names that are allowed to be external/private (can be used multiple times)",
        action="append",
    )
    args = parser.parse_args()

    # Get platform info
    platform_abi = None
    platform_target = args.platform
    if platform_target is None:
        platform_abi, platform_target = detect_platform()

    # Get list of files to process
    if args.file:
        files = [args.file]
        if not os.path.exists(args.file):
            raise FileNotFoundError(f"Specified file not found: {args.file}")
    else:
        files = glob("./deps/*.packman.xml")
        if not files:
            raise FileNotFoundError("No packman XML files found in ./deps/")

    # Get combinations from kit-sdk.packman.xml
    _, _, combinations = get_package_combinations("./deps/kit-sdk.packman.xml", args.skip_platform)

    # Filter combinations based on platform and config if specified
    if args.platform or args.config:
        filtered_combinations = []
        for combo in combinations:
            if args.platform and combo.get("platform_target") != args.platform:
                continue
            if args.config and combo.get("config") != args.config:
                continue
            filtered_combinations.append(combo)
        combinations = filtered_combinations

    if not combinations:
        print("No matching combinations found for the specified platform/config")
        return 1

    # Gather results for each combination
    all_results = []
    for combo in combinations:
        print(f"\nVerifying with combination: {combo}")
        results = gather_package_data(
            files,
            combo["config"],
            combo.get("platform_target_abi", platform_abi),
            combo.get("platform_target", platform_target),
            args.package,
            args.skip_platform,
            args.full_package_license_details,
            args.exclude_package,
            args.allowed_external,
        )
        all_results.append(results)

    # Merge results
    merged_results = merge_verification_results(*all_results)

    # Write output and get total issues
    configs = list(set(combo["config"] for combo in combinations))
    total_issues, private_package_count = write_output_files(
        *merged_results, configs, args.full_package_license_details, args.allowed_external
    )

    # Return 1 if there are private packages found
    if private_package_count > 0:
        exit(1)

    return total_issues


if __name__ == "__main__":
    main()
