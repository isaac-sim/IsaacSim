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

"""Interfaces and implementations for robot motion generation including kinematics, motion policies, trajectory generation, and path planning for collision-aware robot control."""

from __future__ import annotations

from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.articulation_trajectory import ArticulationTrajectory
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    get_supported_robot_path_planner_pairs,
    get_supported_robot_policy_pairs,
    get_supported_robots_with_lula_kinematics,
    load_supported_lula_kinematics_solver_config,
    load_supported_motion_policy_config,
    load_supported_path_planner_config,
)
from isaacsim.robot_motion.motion_generation.kinematics_interface import KinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.motion_policies import RmpFlow, RmpFlowSmoothed
from isaacsim.robot_motion.motion_generation.lula.trajectory_generator import (
    LulaCSpaceTrajectoryGenerator,
    LulaTaskSpaceTrajectoryGenerator,
)
from isaacsim.robot_motion.motion_generation.motion_policy_controller import MotionPolicyController
from isaacsim.robot_motion.motion_generation.motion_policy_interface import MotionPolicy
from isaacsim.robot_motion.motion_generation.path_planner_visualizer import PathPlannerVisualizer
from isaacsim.robot_motion.motion_generation.path_planning_interface import PathPlanner
from isaacsim.robot_motion.motion_generation.trajectory import Trajectory
from isaacsim.robot_motion.motion_generation.world_interface import WorldInterface

__all__ = [
    "WorldInterface",
    "MotionPolicy",
    "RmpFlow",
    "RmpFlowSmoothed",
    "ArticulationMotionPolicy",
    "KinematicsSolver",
    "LulaKinematicsSolver",
    "ArticulationKinematicsSolver",
    "PathPlanner",
    "PathPlannerVisualizer",
    "Trajectory",
    "LulaCSpaceTrajectoryGenerator",
    "LulaTaskSpaceTrajectoryGenerator",
    "ArticulationTrajectory",
    "MotionPolicyController",
    "get_supported_robot_path_planner_pairs",
    "get_supported_robot_policy_pairs",
    "get_supported_robots_with_lula_kinematics",
    "load_supported_lula_kinematics_solver_config",
    "load_supported_motion_policy_config",
    "load_supported_path_planner_config",
]
