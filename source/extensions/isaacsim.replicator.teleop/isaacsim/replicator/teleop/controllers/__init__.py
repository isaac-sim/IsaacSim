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

"""End effector controllers for VR teleop.

This package provides different controller types for VR-driven end effector control:

Floating Controllers (unattached rigid bodies):
- FloatingRigidBodyController: Controls a free rigid body in velocity mode

Robot Controllers (attached to robot arms):
- RobotIKController: Uses inverse kinematics to compute joint targets from VR 6DOF
"""

from .base import EndEffectorValidationResult
from .floating_rigid_body import FloatingRigidBodyController
from .grasp import (
    BUILTIN_GRASP_CONFIG_SCHEME,
    GraspConfig,
    GraspController,
    GraspValidationResult,
    JointMapping,
    get_builtin_grasp_config_uri,
    get_builtin_grasp_configs,
    load_grasp_config,
    normalize_grasp_config_path,
    resolve_grasp_config_path,
)
from .lm_ik import LMIKController
from .locomotion import LocomotionController
from .pink_ik import PinkIKController
from .position_ik import PositionBasedIKController
from .robot_ik import (
    IKMethod,
    IKSolverType,
    IKValidationResult,
    RobotIKController,
)
from .velocity_ik import VelocityBasedIKController

__all__ = [
    "EndEffectorValidationResult",
    "FloatingRigidBodyController",
    "GraspConfig",
    "GraspController",
    "GraspValidationResult",
    "BUILTIN_GRASP_CONFIG_SCHEME",
    "JointMapping",
    "get_builtin_grasp_configs",
    "get_builtin_grasp_config_uri",
    "load_grasp_config",
    "normalize_grasp_config_path",
    "resolve_grasp_config_path",
    "IKMethod",
    "IKSolverType",
    "IKValidationResult",
    "LMIKController",
    "LocomotionController",
    "PinkIKController",
    "PositionBasedIKController",
    "RobotIKController",
    "VelocityBasedIKController",
]
