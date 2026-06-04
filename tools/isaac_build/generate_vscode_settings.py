# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import os
import sys
from typing import Callable, Dict, List

from omni.repo.build import load_settings_from_config
from omni.repo.build.vscode import setup_vscode_env
from omni.repo.man import get_and_validate_host_platform, get_repo_paths

_WINDOWS_MSVC_PRELOAD = """\
# Pre-load system MSVC runtimes to prevent conflicts with older copies
# bundled in Kit directories.  Windows reuses already-loaded DLLs by name,
# so loading them from System32 first ensures that DLLs such as torch's
# c10.dll always find a compatible runtime version.
if hasattr(os, "add_dll_directory"):
    import ctypes as _ctypes
    for _dll_name in ("vcruntime140.dll", "msvcp140.dll", "vcruntime140_1.dll"):
        try:
            _ctypes.CDLL(_dll_name)
        except OSError:
            pass
    del _ctypes, _dll_name
"""


def _validate_template_output_paths(template_paths: List[str], output_paths: List[str]) -> bool:
    """Validate that template and output paths have matching lengths.

    Args:
        template_paths: List of template file paths.
        output_paths: List of corresponding output paths.

    Returns:
        True if paths are valid, False otherwise.
    """
    if len(template_paths) != len(output_paths):
        print(f"Error: Length of template_paths {template_paths} must match output_paths {output_paths}")
        return False
    return True


def _setup_vscode_directory(output_path: str, config: str) -> str:
    """Create VSCode directory structure if it doesn't exist.

    Args:
        output_path: Base output path with potential config placeholder.
        config: Configuration name to replace placeholder.

    Returns:
        Path to the .vscode directory.
    """
    root_path = output_path.replace("${config}", config)
    vscode_folder = os.path.join(root_path, ".vscode")

    if not os.path.exists(vscode_folder):
        os.makedirs(vscode_folder)

    return vscode_folder


def _patch_sitecustomize_msvc_preload(repo_root: str) -> None:
    """Patch the generated sitecustomize.py to pre-load system MSVC runtimes on Windows.

    On Windows, repo_build's sitecustomize.py registers Kit directories via
    os.add_dll_directory().  Some of those directories ship older MSVC runtime
    DLLs that shadow the system copies.  PyTorch's LoadLibraryExW-based loader
    then picks up the stale runtimes, causing c10.dll to fail with WinError 1114.

    Injecting ctypes.CDLL() calls for the system runtimes *before* any
    add_dll_directory() calls ensures the correct versions are already in
    memory when torch (or any other native extension) loads.
    """
    if sys.platform != "win32":
        return

    sitecustomize_path = os.path.join(repo_root, "site", "sitecustomize.py")
    if not os.path.isfile(sitecustomize_path):
        return

    with open(sitecustomize_path, "r") as f:
        content = f.read()

    if "Pre-load system MSVC runtimes" in content:
        return

    # Insert the preload block right after the add_library_path definition
    # (before any os.add_dll_directory calls that register Kit paths).
    marker = "add_library_path = os.add_dll_directory"
    idx = content.find(marker)
    if idx == -1:
        return

    # Find the end of the if/else block that defines add_library_path
    # (ends after the else clause's body).
    end_of_block = content.find("\nif os.path.exists", idx)
    if end_of_block == -1:
        end_of_block = len(content)

    patched = content[:end_of_block] + "\n" + _WINDOWS_MSVC_PRELOAD + content[end_of_block:]

    with open(sitecustomize_path, "w") as f:
        f.write(patched)

    print(f">>> Patched {sitecustomize_path} with MSVC runtime pre-loading")


def _generate_python_environment(
    repo_folders: Dict, platform_target: str, config: str, settings, tool_config: Dict
) -> None:
    """Generate Python environment setup without VSCode settings.

    Args:
        repo_folders: Dictionary containing repository folder paths.
        platform_target: Target platform string.
        config: Build configuration name.
        settings: Build settings object.
        tool_config: Tool-specific configuration dictionary.
    """
    settings.generate_python_setup_shell_script = True
    tool_config["vscode"]["write_python_paths_in_settings_json"] = False
    settings.vscode_python_env_postprocess_fn = None

    setup_vscode_env(
        repo_folders=repo_folders,
        platform_target=platform_target,
        configs=[config],
        settings=settings,
        tool_config=tool_config,
    )

    _patch_sitecustomize_msvc_preload(repo_folders["root"])


def _generate_vscode_settings(
    repo_folders: Dict,
    platform_target: str,
    config: str,
    settings,
    tool_config: Dict,
    python_analysis_extra_mapping: List[str],
) -> None:
    """Generate VSCode settings with Python path configuration.

    Args:
        repo_folders: Dictionary containing repository folder paths.
        platform_target: Target platform string.
        config: Build configuration name.
        settings: Build settings object.
        tool_config: Tool-specific configuration dictionary.
        python_analysis_extra_mapping: Additional Python paths for analysis.
    """
    settings.generate_python_setup_shell_script = False
    tool_config["vscode"]["write_python_paths_in_settings_json"] = True

    # Create a closure to capture the python_analysis_extra_mapping
    def append_analysis_paths_closure(env_dict: Dict, platform_target: str, config: str) -> Dict:
        """Closure that captures python_analysis_extra_mapping for the postprocess function."""
        env_dict["PYTHONPATH"].extend(python_analysis_extra_mapping)

        # Remove python_packages entry to fix VSCode Python autocomplete for imports
        # This prevents incorrect aliasing of isaacsim imports
        for entry in env_dict["PYTHONPATH"]:
            if "$config/python_packages" in entry:
                env_dict["PYTHONPATH"].remove(entry)
                break

        return env_dict

    settings.vscode_python_env_postprocess_fn = append_analysis_paths_closure

    setup_vscode_env(
        repo_folders=repo_folders,
        platform_target=platform_target,
        configs=[config],
        settings=settings,
        tool_config=tool_config,
    )


def setup_repo_tool(parser: argparse.ArgumentParser, config: Dict) -> Callable:
    """Set up the repository tool for generating VSCode settings.

    Args:
        parser: Argument parser to configure with tool-specific options.
        config: Configuration dictionary containing tool settings.

    Returns:
        Callable function that executes the repository tool.
    """
    parser.description = "Generate VSCode settings for Isaac Sim development."
    parser.add_argument(
        "-c",
        "--config",
        dest="config",
        required=False,
        default="release",
        help="Build configuration to use (default: %(default)s)",
    )

    def run_repo_tool(options: Dict, config: Dict) -> bool:
        """Execute the VSCode settings generation tool.

        Args:
            options: Command-line options and arguments.
            config: Full configuration dictionary.

        Returns:
            True if successful, False otherwise.
        """
        # Configure build options
        options.build_verbose = False
        options.jobs = -1
        options.compilation_cores = 1
        options.mem_per_core = 4
        options.max_cores = 32
        options.msbuild_renice = False

        # Load build settings and get repository paths
        settings = load_settings_from_config(config, options)
        repo_folders = get_repo_paths()

        # Validate platform target
        supported_platforms = ["windows-x86_64", "linux-x86_64", "linux-aarch64", "macos-x86_64", "macos-aarch64"]
        platform_target = get_and_validate_host_platform(supported_platforms)

        # Get VSCode-specific configuration
        vscode_config = config.get("repo_generate_vscode_settings", {})
        template_paths = vscode_config["template_paths"]
        output_paths = vscode_config["output_paths"]
        python_analysis_extra_mapping = vscode_config["python_analysis_extra_mapping"]

        # Validate configuration
        if not _validate_template_output_paths(template_paths, output_paths):
            return False

        # Get tool configuration
        tool_config = config.get("repo_build", {})

        # Generate settings for each template/output pair
        for template_path, output_path in zip(template_paths, output_paths):
            # Configure template path
            tool_config["vscode"]["settings_template_file"] = template_path

            # Set up output directory
            repo_folders["root"] = output_path.replace("${config}", options.config)
            _setup_vscode_directory(output_path, options.config)

            # Generate Python environment setup
            _generate_python_environment(repo_folders, platform_target, options.config, settings, tool_config)

            # Generate VSCode settings
            _generate_vscode_settings(
                repo_folders, platform_target, options.config, settings, tool_config, python_analysis_extra_mapping
            )

        return True

    return run_repo_tool
