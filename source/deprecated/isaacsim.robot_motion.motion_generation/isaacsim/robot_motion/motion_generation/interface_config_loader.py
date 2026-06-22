# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""This InterfaceLoader makes it trivial to load a valid config for supported interface implementations.

For example, RMPflow has a collection of robot-specific config files stored in the motion_generation extension.
This loader makes it simple to load RMPflow for the Franka robot using load_supported_motion_policy_config("Franka","RMPflow")
"""

from __future__ import annotations

import json
import os

import carb
from isaacsim.core.utils.extensions import get_extension_path_from_name


def get_supported_robot_policy_pairs() -> dict:
    """Get a dictionary of MotionPolicy names that are supported for each given robot name.

    Returns:
        Dictionary mapping robot names (keys) to a list of supported MotionPolicy config files (values).
    """
    mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    supported_policy_names_by_robot = {}
    for k, v in policy_map.items():
        supported_policy_names_by_robot[k] = list(v.keys())

    return supported_policy_names_by_robot


def get_supported_robots_with_lula_kinematics() -> list[str]:
    """Robot names that have supported lula kinematics configurations.

    Returns:
        List of robot names with supported lula kinematics.
    """
    # Currently just uses robots that have RmpFlow supported
    robots = []

    pairs = get_supported_robot_policy_pairs()
    for k, v in pairs.items():
        if "RMPflow" in v:
            robots.append(k)
    return robots


def get_supported_robot_path_planner_pairs() -> dict:
    """Get a dictionary of PathPlanner names that are supported for each given robot name.

    Returns:
        Dictionary mapping robot names (keys) to a list of supported PathPlanner config files (values).
    """
    mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    policy_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    with open(os.path.join(policy_config_dir, "path_planner_map.json")) as planner_map:
        planner_map = json.load(planner_map)

    supported_planner_names_by_robot = {}
    for k, v in planner_map.items():
        supported_planner_names_by_robot[k] = list(v.keys())

    return supported_planner_names_by_robot


def load_supported_lula_kinematics_solver_config(robot_name: str, policy_config_dir: str = None) -> dict:
    """Load lula kinematics solver for a supported robot.

    Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics.

    Args:
        robot_name: Name of robot.
        policy_config_dir: Path to directory where a policy_map.json file is stored.

    Returns:
        A dictionary whose keyword arguments are sufficient to load the lula kinematics solver.
            e.g. lula.LulaKinematicsSolver(**load_supported_lula_kinematics_solver_config("Franka"))
    """
    policy_name = "RMPflow"

    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics"
        )
        return None
    if policy_name not in policy_map[robot_name]:
        carb.log_error(
            robot_name
            + " does not have supported lula kinematics.  Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics"
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][policy_name])
    rmp_config = _process_policy_config(config_path)

    kinematics_config = {}
    kinematics_config["robot_description_path"] = rmp_config["robot_description_path"]
    kinematics_config["urdf_path"] = rmp_config["urdf_path"]
    return kinematics_config


def load_supported_motion_policy_config(robot_name: str, policy_name: str, policy_config_dir: str = None) -> dict:
    """Load a MotionPolicy object by specifying the robot name and policy name.

    For a dictionary mapping supported robots to supported policies on those robots,
    use get_supported_robot_policy_pairs().

    To use this loader for a new policy, a user may copy the config file structure found under /motion_policy_configs/
    in the motion_generation extension, passing in a path to a directory containing a "policy_map.json"

    Args:
        robot_name: Name of robot.
        policy_name: Name of MotionPolicy.
        policy_config_dir: Path to directory where a policy_map.json file is stored,
            defaults to ".../isaacsim.robot_motion.motion_generation/motion_policy_configs".

    Returns:
        A dictionary whose keyword arguments are sufficient to load the desired motion policy
            e.g. lula.motion_policies.RmpFlow(**load_supported_motion_policy_config("Franka","RMPflow"))
    """
    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies"
        )
        return None
    if policy_name not in policy_map[robot_name]:
        carb.log_error(
            'Unsupported policy name passed to InterfaceLoader for robot "'
            + robot_name
            + '".  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies'
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][policy_name])
    config = _process_policy_config(config_path)

    return config


def load_supported_path_planner_config(robot_name: str, planner_name: str, policy_config_dir: str = None) -> dict:
    """Load a PathPlanner configuration by specifying the robot name and planner name.

    Args:
        robot_name: Name of robot.
        planner_name: Name of PathPlanner.
        policy_config_dir: Path to directory where a path_planner_map.json file is stored.

    Returns:
        A dictionary whose keyword arguments are sufficient to load the desired path planner.
    """
    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    with open(os.path.join(policy_config_dir, "path_planner_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies"
        )
        return None
    if planner_name not in policy_map[robot_name]:
        carb.log_error(
            'Unsupported policy name passed to InterfaceLoader for robot "'
            + robot_name
            + '".  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies'
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][planner_name])
    config = _process_policy_config(config_path)

    return config


def _process_policy_config(mg_config_file: str) -> dict:
    """Process a motion generation config file by resolving relative asset paths.

    Args:
        mg_config_file: Path to the motion generation config file.

    Returns:
        The processed configuration dictionary with resolved asset paths.
    """
    mp_config_dir = os.path.dirname(mg_config_file)
    with open(mg_config_file) as config_file:
        config = json.load(config_file)
    rel_assets = config.get("relative_asset_paths", {})
    for k, v in rel_assets.items():
        config[k] = os.path.join(mp_config_dir, v)
    del config["relative_asset_paths"]
    return config
