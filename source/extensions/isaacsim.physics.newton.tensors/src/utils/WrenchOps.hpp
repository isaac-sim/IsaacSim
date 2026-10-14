// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "WarpCompat.hpp"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{
namespace details
{

WP_CUDA_CALLABLE inline wp::vec3 makeVec3(float x, float y, float z)
{
    wp::vec3 result;
    result[0] = x;
    result[1] = y;
    result[2] = z;
    return result;
}

WP_CUDA_CALLABLE inline wp::vec3 addVec3(const wp::vec3& a, const wp::vec3& b)
{
    return makeVec3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

WP_CUDA_CALLABLE inline wp::vec3 subtractVec3(const wp::vec3& a, const wp::vec3& b)
{
    return makeVec3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

WP_CUDA_CALLABLE inline wp::vec3 crossVec3(const wp::vec3& a, const wp::vec3& b)
{
    return makeVec3(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

WP_CUDA_CALLABLE inline wp::vec3 rotateVec3(const wp::quat& rotation, const wp::vec3& vector)
{
    const wp::vec3 imaginary = makeVec3(rotation[0], rotation[1], rotation[2]);
    const wp::vec3 twiceCross = makeVec3(2.0f * (imaginary[1] * vector[2] - imaginary[2] * vector[1]),
                                         2.0f * (imaginary[2] * vector[0] - imaginary[0] * vector[2]),
                                         2.0f * (imaginary[0] * vector[1] - imaginary[1] * vector[0]));
    return addVec3(
        vector, addVec3(makeVec3(rotation[3] * twiceCross[0], rotation[3] * twiceCross[1], rotation[3] * twiceCross[2]),
                        crossVec3(imaginary, twiceCross)));
}

WP_CUDA_CALLABLE inline wp::quat multiplyQuaternion(const wp::quat& a, const wp::quat& b)
{
    wp::quat result;
    result[0] = a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1];
    result[1] = a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2];
    result[2] = a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0];
    result[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    return result;
}

WP_CUDA_CALLABLE inline wp::quat inverseUnitQuaternion(const wp::quat& rotation)
{
    wp::quat result;
    result[0] = -rotation[0];
    result[1] = -rotation[1];
    result[2] = -rotation[2];
    result[3] = rotation[3];
    return result;
}

WP_CUDA_CALLABLE inline void computeIncomingJointWrench(const wp::spatial_vector& bodyParentForce,
                                                        const wp::transform& bodyTransform,
                                                        const wp::vec3& bodyCenterOfMass,
                                                        const wp::transform& jointChildTransform,
                                                        float* output)
{
    const wp::vec3 forceWorld = makeVec3(bodyParentForce.w[0], bodyParentForce.w[1], bodyParentForce.w[2]);
    const wp::vec3 torqueAtCenterOfMassWorld = makeVec3(bodyParentForce.v[0], bodyParentForce.v[1], bodyParentForce.v[2]);
    const wp::vec3 centerOfMassWorld = addVec3(bodyTransform.p, rotateVec3(bodyTransform.q, bodyCenterOfMass));
    const wp::vec3 jointPositionWorld = addVec3(bodyTransform.p, rotateVec3(bodyTransform.q, jointChildTransform.p));
    const wp::vec3 torqueAtJointWorld =
        addVec3(torqueAtCenterOfMassWorld, crossVec3(subtractVec3(centerOfMassWorld, jointPositionWorld), forceWorld));
    const wp::quat jointOrientationWorld = multiplyQuaternion(bodyTransform.q, jointChildTransform.q);
    const wp::quat jointOrientationWorldInverse = inverseUnitQuaternion(jointOrientationWorld);
    const wp::vec3 forceJoint = rotateVec3(jointOrientationWorldInverse, forceWorld);
    const wp::vec3 torqueJoint = rotateVec3(jointOrientationWorldInverse, torqueAtJointWorld);

    output[0] = forceJoint[0];
    output[1] = forceJoint[1];
    output[2] = forceJoint[2];
    output[3] = torqueJoint[0];
    output[4] = torqueJoint[1];
    output[5] = torqueJoint[2];
}

WP_CUDA_CALLABLE inline float computeProjectedJointForce(const wp::spatial_vector& bodyParentForce,
                                                         const wp::transform& bodyTransform,
                                                         const wp::vec3& bodyCenterOfMass,
                                                         const wp::spatial_vector& motionSubspace)
{
    const wp::vec3 forceWorld = makeVec3(bodyParentForce.w[0], bodyParentForce.w[1], bodyParentForce.w[2]);
    const wp::vec3 torqueAtCenterOfMassWorld = makeVec3(bodyParentForce.v[0], bodyParentForce.v[1], bodyParentForce.v[2]);
    const wp::vec3 linearVelocityAtOrigin = makeVec3(motionSubspace.w[0], motionSubspace.w[1], motionSubspace.w[2]);
    const wp::vec3 angularVelocity = makeVec3(motionSubspace.v[0], motionSubspace.v[1], motionSubspace.v[2]);
    const wp::vec3 centerOfMassWorld = addVec3(bodyTransform.p, rotateVec3(bodyTransform.q, bodyCenterOfMass));
    const wp::vec3 linearVelocityAtCenterOfMass =
        addVec3(linearVelocityAtOrigin, crossVec3(angularVelocity, centerOfMassWorld));

    return linearVelocityAtCenterOfMass[0] * forceWorld[0] + linearVelocityAtCenterOfMass[1] * forceWorld[1] +
           linearVelocityAtCenterOfMass[2] * forceWorld[2] + angularVelocity[0] * torqueAtCenterOfMassWorld[0] +
           angularVelocity[1] * torqueAtCenterOfMassWorld[1] + angularVelocity[2] * torqueAtCenterOfMassWorld[2];
}

WP_CUDA_CALLABLE inline void computeWorldWrench(const float* force,
                                                const float* torque,
                                                const float* position,
                                                const wp::transform& bodyTransform,
                                                const wp::vec3& bodyCenterOfMass,
                                                bool isGlobal,
                                                bool hasForce,
                                                bool hasTorque,
                                                bool hasPosition,
                                                wp::vec3& forceWorld,
                                                wp::vec3& torqueWorld)
{
    forceWorld = makeVec3(0.0f, 0.0f, 0.0f);
    torqueWorld = makeVec3(0.0f, 0.0f, 0.0f);

    if (hasForce)
    {
        const wp::vec3 inputForce = makeVec3(force[0], force[1], force[2]);
        forceWorld = isGlobal ? inputForce : rotateVec3(bodyTransform.q, inputForce);

        if (hasPosition)
        {
            const wp::vec3 inputPosition = makeVec3(position[0], position[1], position[2]);
            const wp::vec3 positionWorld =
                isGlobal ? inputPosition : addVec3(bodyTransform.p, rotateVec3(bodyTransform.q, inputPosition));
            const wp::vec3 centerOfMassWorld = addVec3(bodyTransform.p, rotateVec3(bodyTransform.q, bodyCenterOfMass));
            torqueWorld = crossVec3(subtractVec3(positionWorld, centerOfMassWorld), forceWorld);
        }
    }

    if (hasTorque)
    {
        const wp::vec3 inputTorque = makeVec3(torque[0], torque[1], torque[2]);
        torqueWorld = addVec3(torqueWorld, isGlobal ? inputTorque : rotateVec3(bodyTransform.q, inputTorque));
    }
}

} // namespace details
} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
