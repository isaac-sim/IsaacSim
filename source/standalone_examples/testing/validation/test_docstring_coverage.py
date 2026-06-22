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

"""Inspects Isaac Sim extension modules for public functions and methods with missing or too-short docstrings after enabling their extensions. Produces human, JSON, or AI-oriented coverage reports with file and method details."""

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()
for i in range(10):
    kit.update()

import fnmatch
import importlib
import inspect
import json
import re
import sys
from pathlib import Path
from typing import Any

from isaacsim.core.utils.extensions import enable_extension


def is_public_method(name: str) -> bool:
    """Check if a method/function name is public.

    Args:
        name: Method or function name.

    Returns:
        True if the method is public (doesn't start with underscore).
    """
    return not name.startswith("_")


def is_extension_excluded(ext_name: str, exclusion_patterns: list[str]) -> bool:
    """Check if an extension matches any exclusion pattern.

    Args:
        ext_name: Extension name to check.
        exclusion_patterns: List of patterns (supports wildcards).

    Returns:
        True if the extension matches any exclusion pattern.
    """
    return any(fnmatch.fnmatch(ext_name, pattern) for pattern in exclusion_patterns)


def get_function_info(obj: Any, name: str) -> dict[str, Any]:
    """Get detailed information about a function/method.

    Args:
        obj: Function or method object.
        name: Name of the function/method.

    Returns:
        Dictionary with function details including signature, file, and line number.
    """
    info = {
        "name": name,
        "signature": None,
        "file": None,
        "line_number": None,
        "has_docstring": False,
        "docstring": None,
    }

    try:
        # Get signature
        sig = inspect.signature(obj)
        info["signature"] = str(sig)
    except Exception:
        pass

    try:
        # Get source file and line number
        source_file = inspect.getsourcefile(obj)
        if source_file:
            info["file"] = source_file
            lines, line_number = inspect.getsourcelines(obj)
            info["line_number"] = line_number
    except Exception:
        pass

    # Check docstring
    docstring = inspect.getdoc(obj)
    if docstring is not None:
        info["docstring"] = docstring
        if len(docstring.strip()) > 10:
            info["has_docstring"] = True

    return info


def check_docstring(obj: Any, name: str) -> tuple[bool, str]:
    """Check if an object has a valid docstring.

    Args:
        obj: Object to check.
        name: Name of the object.

    Returns:
        Tuple of (has_valid_docstring, docstring_or_none).
    """
    docstring = inspect.getdoc(obj)
    if docstring is None:
        return False, ""
    if len(docstring.strip()) <= 10:
        return False, docstring
    return True, docstring


def inspect_module(module_name: str, file_path: str) -> dict[str, Any]:
    """Inspect a module and check for docstrings on public methods.

    Args:
        module_name: Name of the module to inspect.
        file_path: Path to the module file.

    Returns:
        Dictionary containing inspection results.
    """
    result = {
        "module_name": module_name,
        "file_path": file_path,
        "imported": False,
        "error": None,
        "public_methods": [],
        "methods_without_docstrings": [],
        "methods_with_short_docstrings": [],
        "methods_needing_docstrings": [],  # Detailed info for AI processing
    }

    try:
        # Try to import the module
        module = importlib.import_module(module_name)
        result["imported"] = True

        # Inspect all members of the module
        for name, obj in inspect.getmembers(module):
            # Only check public methods/functions defined in this module
            if not is_public_method(name):
                continue

            # Check if it's a function or method
            if inspect.isfunction(obj) or inspect.ismethod(obj):
                # Check if it's defined in this module or a submodule
                if hasattr(obj, "__module__") and (
                    obj.__module__ == module_name or obj.__module__.startswith(module_name + ".")
                ):
                    result["public_methods"].append(name)
                    has_docstring, docstring = check_docstring(obj, name)
                    if not has_docstring:
                        func_info = get_function_info(obj, name)
                        func_info["module"] = module_name
                        func_info["type"] = "function"
                        result["methods_needing_docstrings"].append(func_info)

                        if docstring:
                            result["methods_with_short_docstrings"].append({"name": name, "docstring": docstring})
                        else:
                            result["methods_without_docstrings"].append(name)

            # Check classes and their methods
            elif inspect.isclass(obj):
                # Check if the class is defined in this module or a submodule
                if hasattr(obj, "__module__") and (
                    obj.__module__ == module_name or obj.__module__.startswith(module_name + ".")
                ):
                    for method_name, method_obj in inspect.getmembers(obj):
                        if not is_public_method(method_name):
                            continue
                        if inspect.isfunction(method_obj) or inspect.ismethod(method_obj):
                            full_name = f"{name}.{method_name}"
                            result["public_methods"].append(full_name)
                            has_docstring, docstring = check_docstring(method_obj, method_name)
                            if not has_docstring:
                                func_info = get_function_info(method_obj, method_name)
                                func_info["module"] = module_name
                                func_info["class"] = name
                                func_info["type"] = "method"
                                func_info["full_name"] = full_name
                                result["methods_needing_docstrings"].append(func_info)

                                if docstring:
                                    result["methods_with_short_docstrings"].append(
                                        {"name": full_name, "docstring": docstring}
                                    )
                                else:
                                    result["methods_without_docstrings"].append(full_name)

    except Exception as e:
        result["error"] = str(e)

    return result


def get_extensions_to_check(base_dir: str) -> dict[str, str]:
    """Get list of extensions to check.

    Args:
        base_dir: Base directory to search for extensions.

    Returns:
        Dictionary mapping extension names to their extension.toml paths.
    """
    extensions = {}
    base_path = Path(base_dir)

    if not base_path.exists():
        return extensions

    # Iterate through all extension directories
    for ext_dir in base_path.iterdir():
        if not ext_dir.is_dir():
            continue

        ext_name = ext_dir.name
        config_file = ext_dir / "config" / "extension.toml"

        if config_file.exists():
            extensions[ext_name] = str(config_file)

    return extensions


def get_python_modules_from_extension_config(config_path: str) -> list[str]:
    """Parse extension.toml to find Python module names.

    Args:
        config_path: Path to extension.toml file.

    Returns:
        List of Python module names defined in the extension.
    """
    modules = []
    try:
        with open(config_path) as f:
            content = f.read()

        # Simple parsing for [[python.module]] sections
        pattern = r'\[\[python\.module\]\]\s*name\s*=\s*["\']([^"\']+)["\']'
        matches = re.findall(pattern, content)

        for module_name in matches:
            # Skip test modules
            if ".tests" not in module_name and ".test" not in module_name:
                modules.append(module_name)

    except Exception as e:
        pass

    return modules


def main() -> None:
    """Main function to check docstrings across all extension modules."""
    # Check for output format argument
    output_format = "human"  # default
    if len(sys.argv) > 1:
        if sys.argv[1] in ["--json", "-j"]:
            output_format = "json"
        elif sys.argv[1] in ["--ai", "-a"]:
            output_format = "ai"

    # Define the base directories to search
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent.parent.parent.parent

    extension_dirs = [
        str(repo_root / "source" / "deprecated"),
        str(repo_root / "source" / "extensions"),
    ]

    # Extensions to exclude from checking
    excluded_extensions = {
        "isaacsim.xr.input_devices",
        "isaacsim.code_editor.jupyter",
    }

    if output_format == "human":
        print("=" * 80)
        print("DOCSTRING COVERAGE CHECK FOR ISAAC SIM EXTENSIONS")
        print("=" * 80)
        print()

        if excluded_extensions:
            print(f"Excluding {len(excluded_extensions)} extension(s):")
            for ext in sorted(excluded_extensions):
                print(f"  - {ext}")
            print()

    all_results = []
    total_modules = 0
    total_imported = 0
    total_failed = 0
    total_public_methods = 0
    total_missing_docstrings = 0
    total_short_docstrings = 0
    enabled_extensions = 0
    failed_to_enable = []
    skipped_extensions = 0

    # Find and inspect all modules, grouped by extension
    for ext_dir in extension_dirs:
        if output_format == "human":
            print(f"Scanning directory: {ext_dir}")
        extensions = get_extensions_to_check(ext_dir)
        if output_format == "human":
            print(f"  Found {len(extensions)} extensions")
            print()

        for ext_name, config_path in extensions.items():
            # Skip excluded extensions
            if ext_name in excluded_extensions:
                if output_format == "human":
                    print(f"Extension: {ext_name} (SKIPPED - excluded)")
                    print()
                skipped_extensions += 1
                continue

            # Get Python modules defined in this extension
            python_modules = get_python_modules_from_extension_config(config_path)

            if not python_modules:
                continue

            if output_format == "human":
                print(f"Extension: {ext_name} ({len(python_modules)} module(s))")

            # Try to enable the extension first
            if output_format == "human":
                print(f"  Enabling...", end=" ")
            try:
                enable_extension(ext_name)
                if output_format == "human":
                    print("✓")
                enabled_extensions += 1
            except Exception as e:
                if output_format == "human":
                    print(f"✗ ({str(e)[:60]}...)" if len(str(e)) > 60 else f"✗ ({e})")
                failed_to_enable.append((ext_name, str(e)))

            # Now inspect all modules defined by this extension
            for module_name in python_modules:
                total_modules += 1
                if output_format == "human":
                    print(f"  Checking module: {module_name}...", end=" ")
                result = inspect_module(module_name, "")
                all_results.append(result)

                if result["imported"]:
                    total_imported += 1
                    num_methods = len(result["public_methods"])
                    num_missing = len(result["methods_without_docstrings"])
                    num_short = len(result["methods_with_short_docstrings"])

                    total_public_methods += num_methods
                    total_missing_docstrings += num_missing
                    total_short_docstrings += num_short

                    if output_format == "human":
                        if num_missing > 0 or num_short > 0:
                            print(f"✓ ({num_methods} methods, {num_missing + num_short} issues)")
                        else:
                            print(f"✓ ({num_methods} methods)")
                else:
                    total_failed += 1
                    if output_format == "human":
                        print(f"✗ (Import failed)")

            if output_format == "human":
                print()

    # Output results based on format
    if output_format == "human":
        # Print summary
        print()
        print("=" * 80)
        print("SUMMARY")
        print("=" * 80)
        print(f"Extensions skipped (excluded): {skipped_extensions}")
        print(f"Extensions enabled: {enabled_extensions}")
        if failed_to_enable:
            print(f"Extensions failed to enable: {len(failed_to_enable)}")
        print(f"Total modules found: {total_modules}")
        print(f"Successfully imported: {total_imported}")
        print(f"Failed to import: {total_failed}")
        print(f"Total public methods/functions: {total_public_methods}")
        print(f"Methods without docstrings: {total_missing_docstrings}")
        print(f"Methods with short docstrings (<=10 chars): {total_short_docstrings}")
        print()

        if total_public_methods > 0:
            coverage = (
                (total_public_methods - total_missing_docstrings - total_short_docstrings) / total_public_methods
            ) * 100
            print(f"Docstring coverage: {coverage:.2f}%")
        print()

        # Print detailed results for modules with issues
        print("=" * 80)
        print("MODULES WITH MISSING OR SHORT DOCSTRINGS")
        print("=" * 80)
        print()

        for result in all_results:
            if result["imported"] and (result["methods_without_docstrings"] or result["methods_with_short_docstrings"]):
                print(f"Module: {result['module_name']}")
                if result["file_path"]:
                    print(f"  File: {result['file_path']}")

                if result["methods_without_docstrings"]:
                    print(f"  Missing docstrings ({len(result['methods_without_docstrings'])}):")
                    for method in result["methods_without_docstrings"]:
                        print(f"    - {method}")

                if result["methods_with_short_docstrings"]:
                    print(f"  Short docstrings ({len(result['methods_with_short_docstrings'])}):")
                    for method_info in result["methods_with_short_docstrings"]:
                        print(f"    - {method_info['name']}: '{method_info['docstring']}'")

                print()

        # Print extension enable errors
        if failed_to_enable:
            print("=" * 80)
            print("EXTENSIONS THAT FAILED TO ENABLE")
            print("=" * 80)
            print()

            for ext_name, error in failed_to_enable:
                print(f"Extension: {ext_name}")
                print(f"  Error: {error}")
                print()

        # Print import errors
        failed_modules = [r for r in all_results if not r["imported"]]
        if failed_modules:
            print("=" * 80)
            print("MODULES THAT FAILED TO IMPORT")
            print("=" * 80)
            print()

            for result in failed_modules:
                print(f"Module: {result['module_name']}")
                if result["file_path"]:
                    print(f"  File: {result['file_path']}")
                print(f"  Error: {result['error']}")
                print()

        # Print AI Summary for LLM processing
        print()
        print("=" * 80)
        print("SUMMARY FOR AI PROCESSING")
        print("=" * 80)
        print()

        # Calculate coverage
        if total_public_methods > 0:
            coverage = (
                (total_public_methods - total_missing_docstrings - total_short_docstrings) / total_public_methods
            ) * 100
            print(f"Overall Coverage: {coverage:.2f}%")
            print(f"Total Issues: {total_missing_docstrings + total_short_docstrings}")
        print()

        # Group all methods needing docstrings by file
        methods_by_file = {}
        for result in all_results:
            if result["imported"] and result["methods_needing_docstrings"]:
                for method_info in result["methods_needing_docstrings"]:
                    file_path = method_info.get("file")
                    if file_path:
                        if file_path not in methods_by_file:
                            methods_by_file[file_path] = []
                        methods_by_file[file_path].append(method_info)

        print(f"Found {len(methods_by_file)} files with missing docstrings")
        print()
        print("FILES AND METHODS TO FIX:")
        print()

        for file_path in sorted(methods_by_file.keys()):
            methods = methods_by_file[file_path]
            print(f"File: {file_path}")
            for method in sorted(methods, key=lambda x: x.get("line_number", 0)):
                print(f"  - Method: {method.get('full_name', method['name'])}")
                print(f"    Line: {method.get('line_number', 'Unknown')}")
                print(f"    Type: {method.get('type', 'unknown')}")
                if method.get("signature"):
                    print(f"    Signature: {method['name']}{method['signature']}")
                if method.get("docstring"):
                    print(f"    Current docstring (short): {repr(method['docstring'])}")
                print()
            print("-" * 40)
            print()

        print("=" * 80)
        print("DOCSTRING CHECK COMPLETE")
        print("=" * 80)

    elif output_format == "json":
        # Output complete JSON for programmatic processing
        output_data = {
            "summary": {
                "extensions_skipped": skipped_extensions,
                "extensions_enabled": enabled_extensions,
                "total_modules": total_modules,
                "successfully_imported": total_imported,
                "failed_to_import": total_failed,
                "total_public_methods": total_public_methods,
                "methods_without_docstrings": total_missing_docstrings,
                "methods_with_short_docstrings": total_short_docstrings,
                "coverage_percentage": (
                    (
                        (total_public_methods - total_missing_docstrings - total_short_docstrings)
                        / total_public_methods
                        * 100
                    )
                    if total_public_methods > 0
                    else 0
                ),
            },
            "results": all_results,
            "failed_to_enable": [{"extension": ext, "error": err} for ext, err in failed_to_enable],
        }
        print(json.dumps(output_data, indent=2))

    elif output_format == "ai":
        # Output structured format optimized for AI processing
        print("=" * 80)
        print("DOCSTRING COVERAGE REPORT FOR AI PROCESSING")
        print("=" * 80)
        print()

        # Calculate coverage
        if total_public_methods > 0:
            coverage = (
                (total_public_methods - total_missing_docstrings - total_short_docstrings) / total_public_methods
            ) * 100
            print(f"Overall Coverage: {coverage:.2f}%")
            print(f"Total Issues: {total_missing_docstrings + total_short_docstrings}")
        print()

        # Group all methods needing docstrings by file
        methods_by_file = {}
        for result in all_results:
            if result["imported"] and result["methods_needing_docstrings"]:
                for method_info in result["methods_needing_docstrings"]:
                    file_path = method_info.get("file")
                    if file_path:
                        if file_path not in methods_by_file:
                            methods_by_file[file_path] = []
                        methods_by_file[file_path].append(method_info)

        print(f"Found {len(methods_by_file)} files with missing docstrings")
        print()
        print("=" * 80)
        print("METHODS NEEDING DOCSTRINGS (grouped by file)")
        print("=" * 80)
        print()

        for file_path in sorted(methods_by_file.keys()):
            methods = methods_by_file[file_path]
            print(f"File: {file_path}")
            print(f"Methods needing docstrings: {len(methods)}")
            print()

            for method in sorted(methods, key=lambda x: x.get("line_number", 0)):
                print(f"  - {method.get('full_name', method['name'])}")
                print(f"    Line: {method.get('line_number', 'Unknown')}")
                print(f"    Type: {method.get('type', 'unknown')}")
                if method.get("signature"):
                    print(f"    Signature: {method['name']}{method['signature']}")
                if method.get("class"):
                    print(f"    Class: {method['class']}")
                if method.get("docstring"):
                    print(f"    Current (short): {repr(method['docstring'])}")
                print()

            print("-" * 80)
            print()

        # Export detailed JSON for programmatic access
        print()
        print("=" * 80)
        print("STRUCTURED DATA (JSON)")
        print("=" * 80)
        print()

        ai_data = {
            "summary": {
                "total_files_with_issues": len(methods_by_file),
                "total_methods_needing_docstrings": sum(len(methods) for methods in methods_by_file.values()),
                "coverage_percentage": (
                    (
                        (total_public_methods - total_missing_docstrings - total_short_docstrings)
                        / total_public_methods
                        * 100
                    )
                    if total_public_methods > 0
                    else 0
                ),
            },
            "files": {},
        }

        for file_path, methods in methods_by_file.items():
            ai_data["files"][file_path] = [
                {
                    "name": m["name"],
                    "full_name": m.get("full_name", m["name"]),
                    "line_number": m.get("line_number"),
                    "type": m.get("type"),
                    "signature": m.get("signature"),
                    "class": m.get("class"),
                    "module": m.get("module"),
                    "current_docstring": m.get("docstring"),
                }
                for m in sorted(methods, key=lambda x: x.get("line_number", 0))
            ]

        print(json.dumps(ai_data, indent=2))
        print()

        print("=" * 80)
        print("INSTRUCTIONS FOR AI ASSISTANT")
        print("=" * 80)
        print()
        print("To fix docstring issues:")
        print("1. Use the file paths above to read the source files")
        print("2. For each method, use the line number to locate it in the file")
        print("3. Add docstrings following the Python docstring guidelines:")
        print("   - Use reStructuredText (RST) syntax")
        print("   - Include brief summary, Args, Returns, and Example sections")
        print("   - Never include type annotations in docstrings")
        print("4. Use search_replace to add/update docstrings")
        print()
        print("=" * 80)


if __name__ == "__main__":
    main()
    kit.close()  # Cleanup application
