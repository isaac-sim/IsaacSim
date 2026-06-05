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

"""Validates physics joint drive configurations and properties for Isaac Sim assets."""

from __future__ import annotations

from omni.asset_validator.core import registerRule
from pxr import PhysxSchema, Usd, UsdPhysics

from .util import DedupBaseRuleChecker


def get_joint_drives_and_joint_states(
    joint: Usd.Prim,
) -> tuple[list[UsdPhysics.DriveAPI], list[PhysxSchema.JointStateAPI]]:
    """Get the drive APIs and joint state APIs for a joint.

    Args:
        joint: The joint to get drive and state APIs for.

    Returns:
        A tuple of (drive_apis, joint_state_apis) for the joint.
    """
    driveAPIs = []
    joint_states = []
    if joint.IsA(UsdPhysics.RevoluteJoint):
        if joint.HasAPI(UsdPhysics.DriveAPI, "angular"):
            driveAPIs.append(UsdPhysics.DriveAPI(joint, "angular"))
            joint_states.append(PhysxSchema.JointStateAPI(joint, "angular"))
    elif joint.IsA(UsdPhysics.PrismaticJoint):
        if joint.HasAPI(UsdPhysics.DriveAPI, "linear"):
            driveAPIs.append(UsdPhysics.DriveAPI(joint, "linear"))
            joint_states.append(PhysxSchema.JointStateAPI(joint, "linear"))
    else:
        for axis in (f"{prefix}{i}" for prefix in ("rot", "trans") for i in ("X", "Y", "Z")):
            if joint.HasAPI(UsdPhysics.DriveAPI, axis):
                driveAPIs.append(UsdPhysics.DriveAPI(joint, axis))
                joint_states.append(PhysxSchema.JointStateAPI(joint, axis))
    return driveAPIs, joint_states


@registerRule("IsaacSim.PhysicsRules")
class PhysicsJointHasDriveOrMimicAPI(DedupBaseRuleChecker):
    """Validates that joints have a drive or mimic API.

    This rule ensures that all joints (except fixed joints) have either a drive API
    or a mimic API configured. Joints with both APIs are checked to ensure proper
    configuration where drive stiffness and damping are set to 0.0 when mimic is used.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if a prim has proper drive or mimic API configuration.

        Args:
            prim: The USD prim to validate.
        """
        if not UsdPhysics.Joint(prim) or UsdPhysics.FixedJoint(prim):
            return
        drives, joint_states = get_joint_drives_and_joint_states(prim)
        has_mimic = prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
        exclude_from_articulation = UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Get()
        if not drives and not has_mimic and not exclude_from_articulation:
            self._AddError(message=f"Joint {prim.GetPath()} has no drive or mimic API", at=prim)
        if drives and has_mimic:
            # Check if stiffness and damping are set to 0.0
            for drive in drives:
                stiffness = drive.GetStiffnessAttr().Get()
                damping = drive.GetDampingAttr().Get()
                if (stiffness is not None and stiffness != 0.0) or (damping is not None and damping != 0.0):
                    self._AddError(message=f"Joint {prim.GetPath()} has both drive and mimic API", at=prim)


@registerRule("IsaacSim.PhysicsRules")
class PhysicsJointMaxVelocity(DedupBaseRuleChecker):
    """Validates that joints have a positive max velocity set.

    This rule checks that joints with the PhysxJointAPI have a defined and positive
    max joint velocity, which is required for proper joint simulation.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if a prim has proper max joint velocity configuration.

        Args:
            prim: The USD prim to validate.
        """
        if prim.HasAPI(PhysxSchema.PhysxJointAPI):
            joint = PhysxSchema.PhysxJointAPI(prim)
            attr = joint.GetMaxJointVelocityAttr()
            if not attr.IsDefined():
                self._AddError(
                    message=f"Max joint velocity is not set on <{prim.GetPath()}>",
                    at=prim,
                )
            else:
                max_joint_velocity = attr.Get()
                if max_joint_velocity <= 0:
                    self._AddError(
                        message=f"Max joint velocity is zero <{attr.GetPath()}>",
                        at=attr,
                    )


@registerRule("IsaacSim.PhysicsRules")
class PhysicsDriveAndJointState(DedupBaseRuleChecker):
    """Validates that joint drives have proper force limits and matching state values.

    This rule checks that joint drives have defined and reasonable max force values,
    and that drive target positions/velocities match joint state positions/velocities.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if a prim has proper drive and joint state configuration.

        Args:
            prim: The USD prim to validate.
        """
        drives, joint_states = get_joint_drives_and_joint_states(prim)
        if not drives:
            return
        is_mimic = prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
        stop = True
        if is_mimic:
            for drive, joint_state in zip(drives, joint_states):
                stiffness = drive.GetStiffnessAttr().Get()
                damping = drive.GetDampingAttr().Get()
                if (stiffness is not None and stiffness != 0.0) or (damping is not None and damping != 0.0):
                    stop = False
                    break
        if stop:
            return

        for drive, joint_state in zip(drives, joint_states):
            force_attr = drive.GetMaxForceAttr()
            if not force_attr.IsDefined():
                self._AddError(
                    message=f"Drive Max Force is not set on <{prim.GetPath()}>",
                    at=force_attr,
                )
            else:
                max_force = force_attr.Get()
                if max_force <= 0:
                    self._AddError(
                        message=f"Drive Max Force is zero <{force_attr.GetPath()}>",
                        at=force_attr,
                    )
                if max_force >= float("inf"):
                    self._AddError(
                        message=f"Drive Max Force is infinite <{force_attr.GetPath()}>",
                        at=force_attr,
                    )

                drive_target_position = drive.GetTargetPositionAttr()
                drive_target_velocity = drive.GetTargetVelocityAttr()

                joint_state_position = joint_state.GetPositionAttr()
                joint_state_velocity = joint_state.GetVelocityAttr()

                tolerance = 1e-2
                if drive_target_position and joint_state_position:
                    pos_diff = abs(drive_target_position.Get() - joint_state_position.Get())
                    if pos_diff > tolerance:
                        self._AddWarning(
                            message=f"Joint state position is very different from drive target position <{drive_target_position.GetPath()}>: difference is {pos_diff}",
                            at=drive_target_position,
                        )

                if drive_target_velocity and joint_state_velocity:
                    vel_diff = abs(drive_target_velocity.Get() - joint_state_velocity.Get())
                    if vel_diff > tolerance:
                        self._AddWarning(
                            message=f"Joint state velocity is very different from drive target velocity <{drive_target_velocity.GetPath()}>: difference is {vel_diff}",
                            at=drive_target_velocity,
                        )


@registerRule("IsaacSim.PhysicsRules")
class DriveJointValueReasonable(DedupBaseRuleChecker):
    """Validates that joint drive stiffness values are within reasonable ranges.

    This rule checks that joint drive stiffness values are within defined minimum and
    maximum limits to ensure stable simulation behavior.
    """

    DRIVE_STIFFNESS_MIN = 0.0
    """Minimum allowed drive stiffness value for joint validation."""
    DRIVE_STIFFNESS_MAX = 1000000.0  # 1e6 stiffness
    """Maximum allowed drive stiffness value for joint validation."""
    NATURAL_FREQUENCY_MIN = 0.0
    """Minimum allowed natural frequency value for joint validation."""
    NATURAL_FREQUENCY_MAX = 500.0  # 500 Hz - warning threshold.
    """Maximum allowed natural frequency value for joint validation."""

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if a prim has reasonable drive stiffness values.

        Args:
            prim: The USD prim to validate.
        """
        drives, joint_states = get_joint_drives_and_joint_states(prim)
        is_mimic = prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
        for drive in drives:
            stiffness = drive.GetStiffnessAttr().Get()
            if stiffness is None and not is_mimic:
                self._AddError(
                    message=f"Drive stiffness is not set on <{drive.GetPath()}>", at=drive.GetStiffnessAttr()
                )
                continue
            elif is_mimic:
                damping = drive.GetDampingAttr().Get()
                if damping is not None and damping != 0.0:
                    self._AddError(
                        message=f"joint is mimic but has damping set <{drive.GetPath()}>", at=drive.GetDampingAttr()
                    )
                if stiffness is not None and stiffness != 0.0:
                    self._AddError(
                        message=f"joint is mimic but has stiffness set <{drive.GetPath()}>",
                        at=drive.GetStiffnessAttr(),
                    )
            elif stiffness < self.DRIVE_STIFFNESS_MIN or stiffness > self.DRIVE_STIFFNESS_MAX:
                self._AddError(
                    message=f"Drive stiffness is out of range <{drive.GetPath()}>: {stiffness}",
                    at=prim,
                )
            continue
            # TODO: Work in progress for natural frequency
