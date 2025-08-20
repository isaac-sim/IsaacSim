# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import omni.asset_validator.core as av_core
from omni.asset_validator.core import AuthoringLayers, registerRule
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


# utils functions
def get_world_translation(prim: Usd.Prim) -> Gf.Vec3d:
    """Get the world space translation of a prim.

    Args:
        prim: The USD prim to get the world translation for.

    Returns:
        The world space translation as a Gf.Vec3d.
    """
    xform = UsdGeom.Xformable(prim)
    world_xform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return Gf.Vec3d(world_xform.ExtractTranslation())


@registerRule("IsaacSim.PhysicsRules")
class JointHasCorrectTransformAndState(av_core.BaseRuleChecker):
    """Validates that joint transforms and states are consistent with the connected bodies.

    This rule checks that the joint's transform and state values correctly define the
    relationship between the connected bodies. Inconsistencies can cause incorrect
    joint behavior during simulation.
    """

    joint_axis_map = {
        "X": Gf.Vec3d(1, 0, 0),
        "Y": Gf.Vec3d(0, 1, 0),
        "Z": Gf.Vec3d(0, 0, 1),
    }

    def CheckPrim(self, prim: Usd.Prim) -> None:
        # print(f"JointHasCorrectTransform: {prim.GetPath()}")

        joint = UsdPhysics.Joint(prim)

        if not joint:
            return

        if not (
            prim.IsA(UsdPhysics.RevoluteJoint)
            # or prim.IsA(UsdPhysics.SphericalJoint)
            or prim.IsA(UsdPhysics.PrismaticJoint)
            # or prim.IsA(UsdPhysics.FixedJoint)
        ):
            return

        stage = prim.GetStage()

        # Check if the bodies are valid
        b0paths = joint.GetBody0Rel().GetTargets()
        b1paths = joint.GetBody1Rel().GetTargets()

        if len(b0paths):
            if not stage.GetPrimAtPath(b0paths[0]).IsValid():
                return
        else:
            return

        if len(b1paths):
            if not stage.GetPrimAtPath(b1paths[0]).IsValid():
                return
        else:
            return

        # Get the expected transform
        cache = UsdGeom.XformCache()
        expected_tm_0 = get_world_body_transform(stage, cache, joint, False)
        expected_tm_1 = get_world_body_transform(stage, cache, joint, True)

        # Compute the joint state offset transform
        joint_state_transform = Gf.Transform()
        if prim.IsA(UsdPhysics.RevoluteJoint):
            jointState = PhysxSchema.JointStateAPI(prim, "angular")
            if jointState:
                revolute_joint = UsdPhysics.RevoluteJoint(prim)
                value = jointState.GetPositionAttr().Get()
                axis = revolute_joint.GetAxisAttr().Get()
                joint_state_transform.SetRotation(
                    Gf.Rotation(Gf.Vec3d(JointHasCorrectTransformAndState.joint_axis_map[str(axis)]), value)
                )
        if prim.IsA(UsdPhysics.PrismaticJoint):
            jointState = PhysxSchema.JointStateAPI(prim, "linear")
            if jointState:
                prismatic_joint = UsdPhysics.PrismaticJoint(prim)
                value = jointState.GetPositionAttr().Get()
                axis = prismatic_joint.GetAxisAttr().Get()
                joint_state_transform.SetTranslation(
                    Gf.Vec3d(JointHasCorrectTransformAndState.joint_axis_map[str(axis)]) * value
                )

        joint_state_pos_0 = joint_state_transform * expected_tm_0

        expected_state_pos_0 = joint_state_pos_0.GetTranslation()
        expected_pos_0 = expected_tm_0.GetTranslation()
        expected_pos_1 = expected_tm_1.GetTranslation()

        if not Gf.IsClose(expected_state_pos_0, expected_pos_1, 1e-5):
            if not Gf.IsClose(expected_pos_0, expected_pos_1, 1e-5):
                self._AddError(
                    message=f"Joint {prim.GetPath()} position not well-defined. From body 0: {expected_pos_0}, from body 1: {expected_pos_1}",
                    at=prim,
                )
            else:
                self._AddError(
                    message=f"Joint {prim.GetPath()} state not matching robot pose. From body 0: {expected_state_pos_0}, from body 1: {expected_pos_1}",
                    at=prim,
                )

        # Check if the orientation is as expected
        expected_state_rot_0 = joint_state_pos_0.GetRotation()
        expected_rot_0 = expected_tm_0.GetRotation()
        expected_rot_1 = expected_tm_1.GetRotation()
        expected_state_rot0_as_vec4d = GfQuatToVec4d(expected_state_rot_0.GetQuat())
        expected_rot0_as_vec4d = GfQuatToVec4d(expected_rot_0.GetQuat().GetNormalized())
        expected_rot1_as_vec4d = GfQuatToVec4d(expected_rot_1.GetQuat().GetNormalized())

        if not Gf.IsClose(expected_state_rot0_as_vec4d, expected_rot1_as_vec4d, 1e-5):
            if not Gf.IsClose(expected_rot0_as_vec4d, expected_rot1_as_vec4d, 1e-5):
                self._AddError(
                    message=f"Joint {prim.GetPath()} Rotation not well defined, From body 0: {expected_rot_0}, From body 1: {expected_rot_1}",
                    at=prim,
                )
            else:
                self._AddError(
                    message=f"Joint {prim.GetPath()} state not matching robot pose. From body 0: {expected_state_rot_0}, from body 1: {expected_rot_1}",
                    at=prim,
                )


def GfQuatToVec4d(quat: Gf.Quatd) -> Gf.Vec4d:
    """Convert a quaternion to a 4D vector.

    Args:
        quat: The quaternion to convert.

    Returns:
        A Vec4d with (real, imaginary_x, imaginary_y, imaginary_z) components.
    """
    return Gf.Vec4d(quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2])


def GfRotationToVec4d(rot: Gf.Rotation) -> Gf.Vec4d:
    """Convert a rotation to a 4D vector.

    Args:
        rot: The rotation to convert.

    Returns:
        A Vec4d representation of the rotation's quaternion.
    """
    return GfQuatToVec4d(rot.GetQuat())


@registerRule("IsaacSim.PhysicsRules")
class JointHasJointStateAPI(av_core.BaseRuleChecker):
    """Validates that joints have the JointStateAPI applied.

    This rule checks that all joints (except fixed joints) have the PhysxSchema.JointStateAPI
    applied. The JointStateAPI is required for proper joint state tracking during simulation.
    """

    @classmethod
    def apply_api(cls, _: Usd.Stage, joint_prim: Usd.Prim) -> None:
        """Apply the appropriate JointStateAPI to a joint prim.

        Args:
            stage: The USD stage containing the joint.
            joint_prim: The joint prim to apply the API to.
        """
        actuator_type = None
        spec_stack = joint_prim.GetPrimStack()
        if spec_stack:
            defining_spec = spec_stack[-1]
            layer = defining_spec.layer
            edit_stage = Usd.Stage.Open(layer.identifier)

            prim = edit_stage.GetPrimAtPath(defining_spec.path)

            if prim.IsA(UsdPhysics.PrismaticJoint):
                actuator_type = "linear"
            elif prim.IsA(UsdPhysics.RevoluteJoint):
                actuator_type = "angular"

            PhysxSchema.JointStateAPI.Apply(prim, actuator_type)
            edit_stage.Save()

    def CheckPrim(self, prim: Usd.Prim) -> None:
        """Check if a prim has the required JointStateAPI applied.

        Args:
            prim: The USD prim to validate.
        """
        if not UsdPhysics.Joint(prim) or UsdPhysics.FixedJoint(prim):
            return

        actuator_type = None
        if prim.IsA(UsdPhysics.PrismaticJoint):
            actuator_type = "linear"
        elif prim.IsA(UsdPhysics.RevoluteJoint):
            actuator_type = "angular"
        # check if the joint api has a joint state api
        if actuator_type is None:
            return
        if not PhysxSchema.JointStateAPI(prim, actuator_type):
            self._AddError(
                message=f"{prim.GetPath()} Has no Joint State API",
                at=prim,
                suggestion=av_core.Suggestion(message="Apply Joint State API", callable=self.apply_api),
            )
        else:
            return


def get_prismatic_or_revolute_limits(joint_prim: Usd.Prim) -> tuple[float, float]:
    """Get the lower and upper limits for prismatic or revolute joints.

    Args:
        joint_prim: The joint prim to get limits from.

    Returns:
        A tuple containing (lower_limit, upper_limit) or (None, None) if not applicable.
    """
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        joint = UsdPhysics.RevoluteJoint(joint_prim)
        return joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        joint = UsdPhysics.PrismaticJoint(joint_prim)
        return joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
    else:
        return None, None


@registerRule("IsaacSim.PhysicsRules")
class MimicAPICheck(av_core.BaseRuleChecker):
    """Validates proper configuration of mimic joint APIs.

    This rule checks that mimic joints have proper reference joints, gear ratios,
    natural frequencies, damping ratios, and compatible joint limits.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:
        """Check if a prim with mimic API is properly configured.

        Args:
            prim: The USD prim to validate.
        """
        if not prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
            return
        else:
            applied_schema = prim.GetAppliedSchemas()
            list_of_mimic_apis = []
            for schema in applied_schema:
                if schema.startswith("PhysxMimicJointAPI"):
                    list_of_mimic_apis.append(schema[19:])
            for axis in list_of_mimic_apis:
                match axis:
                    case "rotX":
                        mimic_api = PhysxSchema.PhysxMimicJointAPI(prim, UsdPhysics.Tokens.rotX)

                    case "rotY":
                        mimic_api = PhysxSchema.PhysxMimicJointAPI(prim, UsdPhysics.Tokens.rotY)

                    case "rotZ":
                        mimic_api = PhysxSchema.PhysxMimicJointAPI(prim, UsdPhysics.Tokens.rotZ)
                    case _:
                        self._AddWarning(
                            message=f"Joint {prim.GetPath()} has unknown mimic axis: {axis}, aborting checks", at=prim
                        )
                        return

                # For the mimic API, perform checks

                # reference joint check
                reference_joint = mimic_api.GetReferenceJointRel().GetTargets()
                if not reference_joint or len(reference_joint) > 1:
                    self._AddError(
                        message=f"Joint {prim.GetPath()} has incorrect number of reference joints, expected: 1, actual: {len(reference_joint)}",
                        at=prim,
                    )

                # self value checks
                gear_ratio = mimic_api.GetGearingAttr().Get()
                natural_frequency = prim.GetAttribute(f"physxMimicJoint:{axis}:naturalFrequency").Get()
                damping_ratio = prim.GetAttribute(f"physxMimicJoint:{axis}:dampingRatio").Get()

                if gear_ratio is None:
                    self._AddError(message=f"Joint {prim.GetPath()} has no gear ratio", at=prim)

                if natural_frequency is None:
                    self._AddError(message=f"Joint {prim.GetPath()} has no natural frequency", at=prim)

                if damping_ratio is None:
                    self._AddError(message=f"Joint {prim.GetPath()} has no damping ratio", at=prim)

                if gear_ratio == 0:
                    self._AddError(message=f"Joint {prim.GetPath()} has gear ratio == 0", at=prim)

                if natural_frequency == 0:
                    self._AddError(message=f"Joint {prim.GetPath()} has natural frequency == 0", at=prim)

                if damping_ratio == 0:
                    self._AddInfo(message=f"Joint {prim.GetPath()} has damping ratio == 0", at=prim)

                # limit checks
                self_joint_lower_limit, self_joint_upper_limit = get_prismatic_or_revolute_limits(prim)
                if self_joint_lower_limit is None or self_joint_upper_limit is None:
                    self._AddError(message=f"Joint {prim.GetPath()} has no limits", at=prim)
                    return

                # obtain the limits from the reference joint
                stage = prim.GetStage()
                reference_joint_prim = stage.GetPrimAtPath(reference_joint[0])
                reference_joint_lower_limit, reference_joint_upper_limit = get_prismatic_or_revolute_limits(
                    reference_joint_prim
                )
                if reference_joint_lower_limit is None or reference_joint_upper_limit is None:
                    self._AddError(message=f"Joint {prim.GetPath()} has no limits", at=prim)
                    return

                # check joint limits
                # if gear_ratio < 0:
                # We want:
                # - reference_lower * gear_ratio > self_lower
                # - self_upper > reference_upper * gear_ratio
                # else:
                # We want:
                # - reference_lower * gear_ratio < self_upper
                # - self_lower < reference_upper * gear_ratio

                if gear_ratio < 0:
                    if not reference_joint_lower_limit * gear_ratio > self_joint_lower_limit:
                        self._AddError(
                            message=f"Joint {prim.GetPath()}'s lower limit ({self_joint_lower_limit}) should be > reference joint limits * gear ratio({reference_joint_lower_limit * gear_ratio})",
                            at=prim,
                        )

                    if not self_joint_upper_limit > reference_joint_upper_limit * gear_ratio:
                        self._AddError(
                            message=f"Joint {prim.GetPath()}'s upper limit ({self_joint_upper_limit}) should be < reference joint limits * gear ratio({reference_joint_upper_limit * gear_ratio})",
                            at=prim,
                        )
                else:
                    if not reference_joint_lower_limit * gear_ratio < self_joint_upper_limit:
                        self._AddError(
                            message=f"Joint {prim.GetPath()}'s lower limit ({self_joint_upper_limit}) should be > reference joint limits * gear ratio({reference_joint_lower_limit * gear_ratio})",
                            at=prim,
                        )

                    if not self_joint_lower_limit < reference_joint_upper_limit * gear_ratio:
                        self._AddError(
                            message=f"Joint {prim.GetPath()}'s upper limit ({self_joint_lower_limit}) should be < reference joint limits * gear ratio({reference_joint_upper_limit * gear_ratio})",
                            at=prim,
                        )


# ------------------------------------------
# JW - this code is from omni\extensions\runtime\source\omni.physx\python\scripts\utils.py
# RG - Modified the code below to return the world transform of the joint computed from the body 0 or body 1


def get_world_body_transform(stage, cache, joint, body0base):
    """Get the world transform of a joint computed from either body 0 or body 1.

    Args:
        stage: The USD stage containing the joint.
        cache: XformCache for efficient transform computation.
        joint: The joint to compute the transform for.
        body0base: If True, compute transform from body 0, otherwise from body 1.

    Returns:
        The world transform of the joint.
    """
    # get both bodies if available
    b0paths = joint.GetBody0Rel().GetTargets()
    b1paths = joint.GetBody1Rel().GetTargets()

    b0prim = None
    b1prim = None

    if len(b0paths):
        b0prim = stage.GetPrimAtPath(b0paths[0])
        if not b0prim.IsValid():
            b0prim = None

    if len(b1paths):
        b1prim = stage.GetPrimAtPath(b1paths[0])
        if not b1prim.IsValid():
            b1prim = None

    b0locpos = joint.GetLocalPos0Attr().Get()
    b1locpos = joint.GetLocalPos1Attr().Get()
    b0locrot = joint.GetLocalRot0Attr().Get()
    b1locrot = joint.GetLocalRot1Attr().Get()

    # switch depending on which is the base
    if body0base:
        t0prim = b0prim
        t0locpos = b0locpos
        t0locrot = b0locrot
        t1prim = b1prim
    else:
        t0prim = b1prim
        t0locpos = b1locpos
        t0locrot = b1locrot
        t1prim = b0prim

    if t0prim:
        t0world = cache.GetLocalToWorldTransform(t0prim)
    else:
        t0world = Gf.Matrix4d()
        t0world.SetIdentity()

    if t1prim:
        t1world = cache.GetLocalToWorldTransform(t1prim)
    else:
        t1world = Gf.Matrix4d()
        t1world.SetIdentity()

    t0local = Gf.Transform()
    t0local.SetRotation(Gf.Rotation(Gf.Quatd(t0locrot)))
    t0local.SetTranslation(Gf.Vec3d(t0locpos))
    t0mult = t0local * Gf.Transform(t0world)

    return t0mult
