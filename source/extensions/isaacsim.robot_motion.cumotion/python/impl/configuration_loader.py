# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides functionality for loading and configuring cuMotion robots from URDF and XRDF files."""

import pathlib
from dataclasses import dataclass

import cumotion
import isaacsim.core.experimental.utils.app as app_utils

from .urdf_normalize import normalize_urdf_for_urdfdom


@dataclass
class CumotionRobot:
    """Robot for cuMotion planning and control.

    This dataclass encapsulates all the necessary data for a robot
    to be used with cuMotion, including the robot description, kinematics solver,
    and controlled joint names.

    Args:
        directory: Path to the robot configuration directory containing URDF/XRDF files.
        robot_description: cuMotion robot description loaded from URDF/XRDF files.
        kinematics: cuMotion kinematics solver for the robot.
        controlled_joint_names: List of joint names that are controlled by the planner.
    """

    directory: pathlib.Path
    robot_description: cumotion.RobotDescription
    kinematics: cumotion.Kinematics
    controlled_joint_names: list[str]


def load_cumotion_robot(
    directory: pathlib.Path | str,
    urdf_filename: pathlib.Path | str = "robot.urdf",
    xrdf_filename: pathlib.Path | str = "robot.xrdf",
) -> CumotionRobot:
    """Load a cuMotion robot from URDF and XRDF files.

    Loads the robot description from the specified URDF and XRDF files and creates
    a robot object containing the robot description, kinematics solver, and
    controlled joint names.

    Args:
        directory: Path to the directory containing the robot configuration files.
        urdf_filename: Name of the URDF file. Defaults to "robot.urdf".
        xrdf_filename: Name of the XRDF file. Defaults to "robot.xrdf".

    Returns:
        Robot object containing all necessary data for cuMotion.

    Raises:
        FileNotFoundError: If the URDF/XRDF files cannot be found.
        Exception: If the URDF/XRDF files cannot be loaded or parsed.

    Example:

        .. code-block:: python

            robot = load_cumotion_robot(
                directory="/path/to/franka",
                urdf_filename="robot.urdf",
                xrdf_filename="robot.xrdf"
            )
    """
    if isinstance(directory, str):
        directory = pathlib.Path(directory)

    if isinstance(urdf_filename, str):
        urdf_filename = pathlib.Path(urdf_filename)

    if isinstance(xrdf_filename, str):
        xrdf_filename = pathlib.Path(xrdf_filename)

    full_xrdf_path = directory / xrdf_filename
    full_urdf_path = directory / urdf_filename

    if not full_xrdf_path.exists():
        raise FileNotFoundError(f"{full_xrdf_path} is not a valid file path.")

    if not full_urdf_path.exists():
        raise FileNotFoundError(f"{full_urdf_path} is not a valid file path.")

    # urdfdom (the parser statically linked into libcumotion.so) rejects valid
    # URDFs that omit attributes it considers mandatory - most commonly
    # ``effort`` / ``velocity`` on ``<limit>`` elements emitted by the Isaac
    # Sim USD->URDF exporter. Normalize the text in-memory and hand it to the
    # ``load_robot_from_memory`` binding so disk content is never mutated.
    xrdf_text = full_xrdf_path.read_text(encoding="utf-8")
    urdf_text = normalize_urdf_for_urdfdom(full_urdf_path.read_text(encoding="utf-8"))

    # if doesn't succeed, will throw. That is the desired behaviour.
    robot_description: cumotion.RobotDescription = cumotion.load_robot_from_memory(xrdf_text, urdf_text)
    kinematics: cumotion.Kinematics = robot_description.kinematics()

    return CumotionRobot(
        directory=directory,
        robot_description=robot_description,
        kinematics=kinematics,
        controlled_joint_names=[
            robot_description.cspace_coord_name(i) for i in range(robot_description.num_cspace_coords())
        ],
    )


def load_cumotion_supported_robot(robot_name: str) -> CumotionRobot:
    """Load a cuMotion robot for a supported robot.

    Loads a pre-configured robot from the extension's robot_configurations directory.
    This is a convenience function for commonly used robots that come packaged with
    the extension.

    Args:
        robot_name: Name of the robot (e.g., "franka", "ur10").

    Returns:
        Robot object for the specified robot.

    Raises:
        FileNotFoundError: If the robot cannot be found or loaded.

    Example:

        .. code-block:: python

            robot = load_cumotion_supported_robot("franka")
    """
    # Here, create the graph planner and also the cumotion robot configurations:
    extension_path = pathlib.Path(app_utils.get_extension_path("isaacsim.robot_motion.cumotion"))
    robot_directory = extension_path / "robot_configurations" / robot_name

    if not robot_directory.exists():
        raise FileNotFoundError(f"{robot_directory} is not a valid path. This may not be a cumotion-supported robot")

    # Create a cumotion robot from the URDF/XRDF files:
    cumotion_robot = load_cumotion_robot(
        directory=robot_directory,
    )

    return cumotion_robot
