# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utilities for loading and parsing robot policy environment configuration files."""

import fnmatch
import io
import sys
from typing import Any

import carb
import omni
import yaml


def parse_env_config(env_config_path: str = "env.yaml") -> dict[str, Any]:
    """Loads and parses an environment configuration YAML file with custom handling for Python tuples.

    and unknown tags. Uses a safe YAML loader that ignores unknown tags and properly constructs
    Python tuples from YAML sequences.

    Args:
        env_config_path: Path to the environment configuration file

    Returns:
        Parsed configuration dictionary containing environment settings including robot,
        scene, simulation, and physics parameters
    """

    class SafeLoaderIgnoreUnknown(yaml.SafeLoader):
        def ignore_unknown(self, node) -> None:
            return None

        def tuple_constructor(loader, node) -> tuple:
            # The node is expected to be a sequence node
            return tuple(loader.construct_sequence(node))

    SafeLoaderIgnoreUnknown.add_constructor("tag:yaml.org,2002:python/tuple", SafeLoaderIgnoreUnknown.tuple_constructor)
    SafeLoaderIgnoreUnknown.add_constructor(None, SafeLoaderIgnoreUnknown.ignore_unknown)

    file_content = omni.client.read_file(env_config_path)[2]
    file = io.BytesIO(memoryview(file_content).tobytes())
    data = yaml.load(file, Loader=SafeLoaderIgnoreUnknown)
    return data


def get_robot_joint_properties(
    data: dict[str, Any], joint_names: list[str]
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]:
    """Extracts and processes robot joint properties from environment configuration.

    Handles both scalar and per-joint property specifications, with pattern matching
    for joint names. Provides default values for missing properties.

    Args:
        data: Environment configuration dictionary containing robot actuator and state data
        joint_names: Ordered list of joint names to extract properties for

    Returns:
        A tuple containing ordered lists of joint properties:
        - effort_limits: Maximum torque/force limits for each joint
        - velocity_limits: Maximum velocity limits for each joint
        - stiffness: Position control stiffness gains
        - damping: Velocity control damping gains
        - armature: Joint armature (rotor inertia)
        - default_pos: Initial/default joint positions
        - default_vel: Initial/default joint velocities
    """
    actuator_data = data.get("scene").get("robot").get("actuators")
    stiffness = {}
    damping = {}
    armature = {}
    effort_limits = {}
    velocity_limits = {}
    default_pos = {}
    default_vel = {}
    joint_names_expr_list = []

    for actuator in actuator_data:
        actuator_config = actuator_data.get(actuator)
        joint_names_expr = actuator_config.get("joint_names_expr")
        joint_names_expr_list.extend(joint_names_expr)

        effort_limit = actuator_config.get("effort_limit")
        velocity_limit = actuator_config.get("velocity_limit")
        joint_stiffness = actuator_config.get("stiffness")
        joint_damping = actuator_config.get("damping")
        joint_armature = actuator_config.get("armature")

        if isinstance(effort_limit, (float, int)) or effort_limit is None:
            if effort_limit is None or effort_limit == float("inf"):
                effort_limit = float(sys.maxsize)
            for names in joint_names_expr:
                effort_limits[names] = float(effort_limit)
        elif isinstance(effort_limit, dict):
            effort_limits.update(effort_limit)
        else:
            carb.log_error(f"Failed to parse effort limit, expected float, int, or dict, got: {type(effort_limit)}")

        if isinstance(velocity_limit, (float, int)) or velocity_limit is None:
            if velocity_limit is None or velocity_limit == float("inf"):
                velocity_limit = float(sys.maxsize)
            for names in joint_names_expr:
                velocity_limits[names] = float(velocity_limit)
        elif isinstance(velocity_limit, dict):
            velocity_limits.update(velocity_limit)
        else:
            carb.log_error(f"Failed to parse velocity limit, expected float, int, or dict, got: {type(velocity_limit)}")

        if isinstance(joint_stiffness, (float, int)) or joint_stiffness is None:
            if joint_stiffness is None:
                joint_stiffness = 0
            for names in joint_names_expr:
                stiffness[names] = float(joint_stiffness)
        elif isinstance(joint_stiffness, dict):
            stiffness.update(joint_stiffness)
        else:
            carb.log_error(f"Failed to parse stiffness, expected float, int, or dict, got: {type(joint_stiffness)}")

        if isinstance(joint_damping, (float, int)) or joint_damping is None:
            if joint_damping is None:
                joint_damping = 0
            for names in joint_names_expr:
                damping[names] = float(joint_damping)
        elif isinstance(joint_damping, dict):
            damping.update(joint_damping)
        else:
            carb.log_error(f"Failed to parse damping, expected float, int, or dict, got: {type(joint_damping)}")

        if isinstance(joint_armature, (float, int)) or joint_armature is None:
            if joint_armature is None:
                joint_armature = 0
            for names in joint_names_expr:
                armature[names] = float(joint_armature)
        elif isinstance(joint_armature, dict):
            armature.update(joint_armature)
        else:
            carb.log_error(f"Failed to parse armature, expected float, int, or dict, got: {type(joint_armature)}")

    # parse default joint position
    init_joint_pos = data.get("scene").get("robot").get("init_state").get("joint_pos")
    if isinstance(init_joint_pos, (float, int)):
        for names in joint_names_expr:
            default_pos[names] = float(init_joint_pos)
    elif isinstance(init_joint_pos, dict):
        default_pos.update(init_joint_pos)
    else:
        carb.log_error(
            f"Failed to parse init state joint position, expected float, int, or dict, got: {type(init_joint_pos)}"
        )

    # parse default joint velocity
    init_joint_vel = data.get("scene").get("robot").get("init_state").get("joint_vel")
    if isinstance(init_joint_vel, (float, int)):
        for names in joint_names_expr:
            default_vel[names] = float(init_joint_vel)
    elif isinstance(init_joint_vel, dict):
        default_vel.update(init_joint_vel)
    else:
        carb.log_error(
            f"Failed to parse init state vel position, expected float, int, or dict, got: {type(init_joint_vel)}"
        )

    stiffness_inorder = []
    damping_inorder = []
    armature_inorder = []
    effort_limits_inorder = []
    velocity_limits_inorder = []
    default_pos_inorder = []
    default_vel_inorder = []

    for joint in joint_names:
        for pattern in joint_names_expr_list:
            if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                if pattern in stiffness:
                    stiffness_inorder.append(stiffness[pattern])
                else:
                    stiffness_inorder.append(0)
                    carb.log_warn(f"{joint} stiffness not found, setting to 0")
                if pattern in damping:
                    damping_inorder.append(damping[pattern])
                else:
                    damping_inorder.append(0)
                    carb.log_warn(f"{joint} damping not found, setting to 0")
                if pattern in armature:
                    armature_inorder.append(armature[pattern])
                else:
                    armature_inorder.append(0)
                    carb.log_warn(f"{joint} armature not found, setting to 0")
                if pattern in effort_limits:
                    effort_limits_inorder.append(effort_limits[pattern])
                else:
                    effort_limits_inorder.append(0)
                    carb.log_warn(f"{joint} effort limit not found, setting to 0")
                if pattern in velocity_limits:
                    velocity_limits_inorder.append(velocity_limits[pattern])
                else:
                    velocity_limits_inorder.append(0)
                    carb.log_warn(f"{joint} velocity limit not found, setting to 0")
                break

        default_position_found = False
        for pattern in default_pos:
            if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                default_pos_inorder.append(default_pos[pattern])
                default_position_found = True
                break
        if not default_position_found:
            default_pos_inorder.append(0)
            carb.log_warn(f"{joint} default position not found, setting to 0")

        default_velocity_found = False
        for pattern in default_vel:
            if fnmatch.fnmatch(joint, pattern.replace(".", "*") + "*"):
                default_vel_inorder.append(default_vel[pattern])
                default_velocity_found = True
                break
        if not default_velocity_found:
            default_vel_inorder.append(0)
            carb.log_warn(f"{joint} default velocity not found, setting to 0")

    return (
        effort_limits_inorder,
        velocity_limits_inorder,
        stiffness_inorder,
        damping_inorder,
        armature_inorder,
        default_pos_inorder,
        default_vel_inorder,
    )


def get_articulation_props(data: dict[str, Any]) -> dict[str, Any]:
    """Retrieves articulation properties from the robot spawn configuration.

    These properties define the physical characteristics and simulation
    parameters for the robot's articulated joints and links.

    Args:
        data: Environment configuration dictionary

    Returns:
        Articulation properties dictionary from scene.robot.spawn.articulation_props
    """
    return data.get("scene").get("robot").get("spawn").get("articulation_props")


def get_physics_properties(data: dict[str, Any]) -> tuple[int, float, int]:
    """Extracts simulation timing parameters from the environment configuration.

    These parameters control the simulation's temporal resolution and rendering.

    Args:
        data: Environment configuration dictionary

    Returns:
        A tuple containing (decimation, dt, render_interval) where decimation is the policy update decimation factor,
        dt is the physics simulation timestep in seconds, and render_interval is the number of physics steps between
        renders.
    """
    return data.get("decimation"), data.get("sim").get("dt"), data.get("sim").get("render_interval")


def get_observations(data: dict[str, Any]) -> dict[str, Any]:
    """Retrieves policy observation specifications from the configuration.

    These define what state information is provided to the policy
    for decision making.

    Args:
        data: Environment configuration dictionary

    Returns:
        Observation specification dictionary from observations.policy section
    """
    return data.get("observations").get("policy")


def get_action(data: dict[str, Any]) -> dict[str, Any]:
    """Retrieves policy action specifications from the configuration.

    These define the control interface and action space available
    to the policy for controlling the robot.

    Args:
        data: Environment configuration dictionary

    Returns:
        Action specification dictionary defining control parameters and limits
    """
    return data.get("actions")


def get_physx_settings(data: dict[str, Any]) -> dict[str, Any]:
    """Retrieves PhysX simulation engine configuration parameters.

    These settings control physics simulation quality, stability,
    and performance characteristics.

    Args:
        data: Environment configuration dictionary

    Returns:
        PhysX settings dictionary from sim.physx section, containing solver
        settings, collision parameters, and other physics simulation properties
    """
    return data.get("sim").get("physx")
