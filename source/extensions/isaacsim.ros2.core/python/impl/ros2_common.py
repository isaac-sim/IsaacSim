# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides common utilities for ROS 2 environment setup and configuration in Isaac Sim."""

import os
import platform
import sys

import carb
import omni

# Bridge constants
BRIDGE_NAME = "isaacsim.ros2.core"
BRIDGE_PREFIX = "ROS2"

# Supported Ubuntu to ROS distribution mapping
SUPPORTED_ROS_DISTROS = {
    "22": "humble",  # Ubuntu 22.04 LTS -> ROS 2 Humble
    "24": "jazzy",  # Ubuntu 24.04 LTS -> ROS 2 Jazzy
}


def get_ubuntu_version() -> str | None:
    """Detect Ubuntu version and return the appropriate ROS distribution.

    Returns:
        'humble' for Ubuntu 22.x, 'jazzy' for Ubuntu 24.x, None for other versions.
    """
    # Check if we're on Linux using sys.platform
    if not sys.platform.startswith("linux"):
        return None

    # Get Ubuntu version using platform.freedesktop_os_release().get("VERSION_ID")
    os_release = platform.freedesktop_os_release()
    version = os_release.get("VERSION_ID")
    major_version = version.split(".")[0]

    # Use the supported distros dictionary to get the ROS distribution
    ros_distro = SUPPORTED_ROS_DISTROS.get(major_version)
    if ros_distro:
        return ros_distro
    else:
        carb.log_error(f"Ubuntu version {version} is not supported for automatic ROS distribution selection")
        return None


def restore_ros2_python_paths() -> None:
    """Restore system ROS 2 Python paths that Isaac Sim removed at startup.

    Isaac Sim overrides PYTHONPATH with its own paths and saves the original in OLD_PYTHONPATH. This function
    cross-references OLD_PYTHONPATH with AMENT_PREFIX_PATH to find and re-add only the ROS 2 Python paths to
    sys.path.
    """
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH")
    old_pythonpath = os.environ.get("OLD_PYTHONPATH")
    if ament_prefix is None or old_pythonpath is None:
        return

    for python_path in old_pythonpath.split(os.pathsep):
        for ament_path in ament_prefix.split(os.pathsep):
            if python_path.startswith(os.path.abspath(ament_path) + os.sep):
                if python_path not in sys.path:
                    sys.path.append(python_path)
                break


def setup_ros2_environment(extension_path: str, ros_distro: str) -> None:
    """Set up ROS 2 environment variables and paths if required.

    Args:
        extension_path: Path to the extension directory.
        ros_distro: ROS distribution name (e.g., 'humble', 'jazzy').
    """
    if sys.platform == "win32":
        if os.environ.get("PATH"):
            os.environ["PATH"] = os.environ.get("PATH") + ";" + extension_path + f"/{ros_distro}/lib"
        else:
            os.environ["PATH"] = extension_path + f"/{ros_distro}/lib"
            os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    # For linux, manually setting up environment variables are not necessary as they are expected to be setup already in the terminal.


def print_environment_setup_instructions(extension_path: str, ros_distro: str) -> None:
    """Print instructions for setting up ROS 2 environment variables.

    Args:
        extension_path: Path to the extension directory.
        ros_distro: ROS distribution name (e.g., 'humble', 'jazzy').
    """
    if sys.platform == "linux":
        omni.kit.app.get_app().print_and_log(
            f"To use the internal libraries included with the extension please set the following environment variables to use with FastDDS (default) or CycloneDDS before starting Isaac Sim:\n\n"
            f"FastDDS (default):\n"
            f"export ROS_DISTRO={ros_distro}\n"
            f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
            f"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{extension_path}/{ros_distro}/lib\n\n"
            f"OR\n\n"
            f"CycloneDDS:\n"
            f"export ROS_DISTRO={ros_distro}\n"
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
            f"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{extension_path}/{ros_distro}/lib\n\n"
        )
    else:
        omni.kit.app.get_app().print_and_log(
            f"To use the internal libraries included with the extension, please set the environment variables using one of the following methods before starting Isaac Sim:\n\n"
            f"Command Prompt (CMD):\n"
            f"set ROS_DISTRO={ros_distro}\n"
            f"set RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
            f"set PATH=%PATH%;{extension_path}/{ros_distro}/lib\n\n"
            f"PowerShell:\n"
            f'$env:ROS_DISTRO = "{ros_distro}"\n'
            f'$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"\n'
            f'$env:PATH = "$env:PATH;{extension_path}/{ros_distro}/lib"\n\n'
        )
