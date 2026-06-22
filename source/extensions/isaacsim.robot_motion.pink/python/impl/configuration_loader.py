# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides functionality for loading and configuring PINK robots from URDF files via Pinocchio."""

from __future__ import annotations

import pathlib
from dataclasses import dataclass, field

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import pinocchio as pin


@dataclass
class PinkRobot:
    """Robot configuration for PINK inverse kinematics.

    Encapsulates a Pinocchio model and associated data needed for differential IK solving
    with the PINK library. The model is loaded from URDF and provides forward kinematics,
    Jacobian computation, and frame placement.

    Args:
        directory: Path to the robot configuration directory containing the URDF file.
        model: Pinocchio rigid-body model parsed from the URDF.
        data: Pinocchio model data (pre-allocated workspace for FK/Jacobians).
        controlled_joint_names: Ordered list of actuated joint names controlled by the IK solver.
        collision_model: Pinocchio geometry model for collision checking. None if not loaded.
        collision_data: Pinocchio geometry data for collision distance queries. None if not loaded.
        q0: Neutral (home) configuration vector. Defaults to the Pinocchio model neutral pose.
    """

    directory: pathlib.Path
    model: pin.Model
    data: pin.Data
    controlled_joint_names: list[str]
    collision_model: pin.GeometryModel | None = None
    collision_data: pin.GeometryData | None = None
    q0: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self) -> None:  # noqa: D105
        if self.q0.size == 0:
            self.q0 = pin.neutral(self.model)


def _get_controlled_joint_names(model: pin.Model) -> list[str]:
    """Extract actuated joint names from a Pinocchio model, excluding the universe joint.

    Args:
        model: Pinocchio model to inspect.

    Returns:
        Ordered names of joints with configuration variables.
    """
    names = []
    for i in range(1, model.njoints):
        joint = model.joints[i]
        if joint.nq > 0:
            names.append(model.names[i])
    return names


def load_pink_robot(
    urdf_path: pathlib.Path | str,
    package_dirs: list[str] | None = None,
    srdf_path: pathlib.Path | str | None = None,
    build_collision_model: bool = False,
) -> PinkRobot:
    """Load a PINK robot from a URDF file via Pinocchio.

    Parses the URDF into a Pinocchio model and optionally builds collision geometry
    for self-collision avoidance barriers.

    Args:
        urdf_path: Path to the URDF file.
        package_dirs: List of package directories for resolving mesh paths in the URDF.
            Defaults to the URDF's parent directory.
        srdf_path: Optional path to an SRDF file for collision pair exclusion.
        build_collision_model: If True, build the collision geometry model from the URDF.
            Required for SelfCollisionBarrier support.

    Returns:
        PinkRobot containing the Pinocchio model and controlled joint information.

    Raises:
        FileNotFoundError: If the URDF file does not exist.
        RuntimeError: If the URDF cannot be parsed by Pinocchio.

    Example:

        .. code-block:: python

            robot = load_pink_robot(
                urdf_path="/path/to/franka/robot.urdf",
                build_collision_model=True,
            )
    """
    urdf_path = pathlib.Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"{urdf_path} is not a valid file path.")

    if package_dirs is None:
        package_dirs = [str(urdf_path.parent)]

    collision_model = None
    collision_data = None

    if build_collision_model:
        model, collision_model, _ = pin.buildModelsFromUrdf(str(urdf_path), package_dirs)
    else:
        model = pin.buildModelFromUrdf(str(urdf_path))

    data = model.createData()
    controlled_joint_names = _get_controlled_joint_names(model)

    if collision_model is not None:
        collision_model.addAllCollisionPairs()
        if srdf_path is not None:
            srdf_path = pathlib.Path(srdf_path)
            if not srdf_path.exists():
                raise FileNotFoundError(f"{srdf_path} is not a valid file path.")
            pin.removeCollisionPairs(model, collision_model, str(srdf_path))
        collision_data = pin.GeometryData(collision_model)

    return PinkRobot(
        directory=urdf_path.parent,
        model=model,
        data=data,
        controlled_joint_names=controlled_joint_names,
        collision_model=collision_model,
        collision_data=collision_data,
        q0=pin.neutral(model),
    )


def load_pink_supported_robot(robot_name: str) -> PinkRobot:
    """Load a pre-configured robot bundled with the extension.

    Loads a robot from the extension's ``robot_configurations`` directory. Each supported
    robot has a subdirectory containing at minimum a ``robot.urdf`` file and optionally
    an SRDF for collision pair configuration.

    Args:
        robot_name: Name of the robot (e.g., "franka", "ur10"). Must match a subdirectory
            under ``robot_configurations/``.

    Returns:
        PinkRobot for the specified robot.

    Raises:
        FileNotFoundError: If the robot name does not correspond to a bundled configuration.

    Example:

        .. code-block:: python

            robot = load_pink_supported_robot("franka")
    """
    extension_path = pathlib.Path(app_utils.get_extension_path("isaacsim.robot_motion.pink"))
    robot_directory = extension_path / "robot_configurations" / robot_name

    if not robot_directory.exists():
        raise FileNotFoundError(
            f"{robot_directory} is not a valid path. '{robot_name}' may not be a supported robot. "
            f"Available robots are subdirectories of {extension_path / 'robot_configurations'}."
        )

    urdf_path = robot_directory / "robot.urdf"
    srdf_path = robot_directory / "robot.srdf"

    return load_pink_robot(
        urdf_path=urdf_path,
        package_dirs=[str(robot_directory)],
        srdf_path=srdf_path if srdf_path.exists() else None,
        build_collision_model=srdf_path.exists(),
    )
