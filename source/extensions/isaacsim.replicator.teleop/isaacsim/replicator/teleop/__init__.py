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

from ._backend import get_teleop_backend, set_teleop_backend, teleop_backend_ctx
from .controllers import (
    BUILTIN_GRASP_CONFIG_SCHEME,
    EndEffectorValidationResult,
    FloatingRigidBodyController,
    GraspConfig,
    GraspController,
    GraspValidationResult,
    IKMethod,
    IKSolverType,
    IKValidationResult,
    JointMapping,
    LocomotionController,
    PositionBasedIKController,
    RobotIKController,
    VelocityBasedIKController,
    get_builtin_grasp_config_uri,
    get_builtin_grasp_configs,
    load_grasp_config,
    normalize_grasp_config_path,
)
from .coordinate_utils import (
    OXR_TO_ISS_QUAT,
    OXR_TO_ISS_ROTATION,
    CoordinateSystem,
    transform_pose,
    transform_pose_openxr_to_isaacsim,
)
from .markers_manager import MarkersManager
from .recordables import TeleopControllerRecordable, TeleopHeadRecordable
from .recorder_factory import build_teleop_recorder
from .teleop_manager import (
    TELEOP_CMD_EVENT,
    TELEOP_STATUS_EVENT,
    TeleopCommand,
    TeleopManager,
    dispatch_command,
)
from .teleop_profiles import (
    BimanualControllerProfile,
    ControllerSideProfile,
    GraspControllerProfile,
    GraspSideProfile,
    LocomotionProfile,
    TeleopProfile,
    TeleopSettingsProfile,
    get_builtin_teleop_profiles_dir,
    get_last_teleop_profile_path,
    load_teleop_profile,
    save_teleop_profile,
    scan_teleop_profiles,
)
from .teleop_resolver import (
    SEVERITY_ERROR,
    SEVERITY_WARNING,
    STAGE_STATE_LOADING,
    STAGE_STATE_NO_STAGE,
    STAGE_STATE_READY,
    TeleopResolutionReport,
    TeleopResolverIssue,
    resolve_teleop_profile,
)
from .teleop_session_injector import install_teleop_session_injector
from .validation import (
    ValidationResult,
    validate_floating_end_effector,
    validate_marker_path,
)
from .vr_recording_button import VRButton, VRRecordingButton
from .xr_anchor_manager import (
    AnchorRotationMode,
    XrAnchorManager,
    activate_pre_session_anchor,
    restore_pre_session_anchor,
)

__all__ = [
    "get_teleop_backend",
    "set_teleop_backend",
    "teleop_backend_ctx",
    "BUILTIN_GRASP_CONFIG_SCHEME",
    "EndEffectorValidationResult",
    "FloatingRigidBodyController",
    "GraspConfig",
    "GraspController",
    "GraspValidationResult",
    "IKMethod",
    "IKSolverType",
    "IKValidationResult",
    "JointMapping",
    "LocomotionController",
    "PositionBasedIKController",
    "RobotIKController",
    "VelocityBasedIKController",
    "get_builtin_grasp_config_uri",
    "get_builtin_grasp_configs",
    "load_grasp_config",
    "normalize_grasp_config_path",
    "OXR_TO_ISS_QUAT",
    "OXR_TO_ISS_ROTATION",
    "CoordinateSystem",
    "transform_pose",
    "transform_pose_openxr_to_isaacsim",
    "MarkersManager",
    "TeleopControllerRecordable",
    "TeleopHeadRecordable",
    "build_teleop_recorder",
    "TELEOP_CMD_EVENT",
    "TELEOP_STATUS_EVENT",
    "TeleopCommand",
    "TeleopManager",
    "dispatch_command",
    "install_teleop_session_injector",
    "BimanualControllerProfile",
    "ControllerSideProfile",
    "GraspControllerProfile",
    "GraspSideProfile",
    "LocomotionProfile",
    "TeleopProfile",
    "TeleopSettingsProfile",
    "get_builtin_teleop_profiles_dir",
    "get_last_teleop_profile_path",
    "load_teleop_profile",
    "save_teleop_profile",
    "scan_teleop_profiles",
    "SEVERITY_ERROR",
    "SEVERITY_WARNING",
    "STAGE_STATE_LOADING",
    "STAGE_STATE_NO_STAGE",
    "STAGE_STATE_READY",
    "TeleopResolutionReport",
    "TeleopResolverIssue",
    "resolve_teleop_profile",
    "ValidationResult",
    "validate_floating_end_effector",
    "validate_marker_path",
    "VRButton",
    "VRRecordingButton",
    "AnchorRotationMode",
    "XrAnchorManager",
    "activate_pre_session_anchor",
    "restore_pre_session_anchor",
]
