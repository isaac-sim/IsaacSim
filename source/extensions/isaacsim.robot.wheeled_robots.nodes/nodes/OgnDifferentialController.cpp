// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include <isaacsim/core/includes/BaseResetNode.h>

#include <OgnDifferentialControllerDatabase.h>
#include <cmath>

namespace isaacsim
{
namespace robot
{
namespace wheeled_robots
{

class OgnDifferentialController : public isaacsim::core::includes::BaseResetNode
{

public:
    static bool compute(OgnDifferentialControllerDatabase& db)
    {
        auto& state = db.perInstanceState<OgnDifferentialController>();
        state.m_nodeObj = db.abi_node();

        if (!state.m_initialized)
        {
            if (db.inputs.wheelRadius() <= 0 || db.inputs.wheelDistance() <= 0)
            {
                db.logWarning("Invalid wheel radius and distance");
                return false;
            }

            state.m_wheelRadius = db.inputs.wheelRadius();
            state.m_wheelDistance = db.inputs.wheelDistance();

            if (std::fabs(db.inputs.maxLinearSpeed()) > 0)
            {
                state.m_maxLinearSpeed = std::fabs(db.inputs.maxLinearSpeed());
            }

            if (std::fabs(db.inputs.maxAcceleration()) > 0)
            {
                state.m_maxAcceleration = std::fabs(db.inputs.maxAcceleration());
            }

            if (std::fabs(db.inputs.maxDeceleration()) > 0)
            {
                state.m_maxDeceleration = std::fabs(db.inputs.maxDeceleration());
            }

            if (std::fabs(db.inputs.maxAngularAcceleration()) > 0)
            {
                state.m_maxAngularAcceleration = std::fabs(db.inputs.maxAngularAcceleration());
            }

            if (std::fabs(db.inputs.maxAngularSpeed()) > 0)
            {
                state.m_maxAngularSpeed = std::fabs(db.inputs.maxAngularSpeed());
            }

            if (std::fabs(db.inputs.maxWheelSpeed()) > 0)
            {
                state.m_maxWheelSpeed = std::fabs(db.inputs.maxWheelSpeed());
            }

            state.m_previousAngularSpeed = 0;
            state.m_previousLinearSpeed = 0;
            state.m_initialized = true;
        }

        double deltaTime = db.inputs.dt();

        // If deltaTime is invalid and acceleration checks are required, skip compute
        if (deltaTime <= 0.0 &&
            (state.m_maxAcceleration != 0.0 || state.m_maxDeceleration != 0.0 || state.m_maxAngularAcceleration != 0.0))
        {
            db.logWarning("Invalid deltaTime %f, cannot check for acceleration limits, skipping current step", deltaTime);
            return false;
        }

        // Clip command velocity to maximum velocity limits
        double linearVelocity =
            std::max(-state.m_maxLinearSpeed, std::min(state.m_maxLinearSpeed, db.inputs.linearVelocity()));
        double angularVelocity =
            std::max(-state.m_maxAngularSpeed, std::min(state.m_maxAngularSpeed, db.inputs.angularVelocity()));

        if (state.m_maxAcceleration != 0.0 && state.m_maxDeceleration != 0.0)
        {
            // If magnitude is increasing and sign is the same, robot is accelerating
            if (std::fabs(linearVelocity) > std::fabs(state.m_previousLinearSpeed) &&
                linearVelocity * state.m_previousLinearSpeed >= 0)
            {
                linearVelocity = std::max(
                    state.m_previousLinearSpeed - state.m_maxAcceleration * deltaTime,
                    std::min(linearVelocity, state.m_previousLinearSpeed + state.m_maxAcceleration * deltaTime));
            }
            // If magnitude is decreasing or sign is different, robot is decelerating
            else
            {
                linearVelocity = std::max(
                    state.m_previousLinearSpeed - state.m_maxDeceleration * deltaTime,
                    std::min(linearVelocity, state.m_previousLinearSpeed + state.m_maxDeceleration * deltaTime));
            }
        }

        if (state.m_maxAngularAcceleration != 0.0)
        {
            angularVelocity = std::max(
                state.m_previousAngularSpeed - state.m_maxAngularAcceleration * deltaTime,
                std::min(angularVelocity, state.m_previousAngularSpeed + state.m_maxAngularAcceleration * deltaTime));
        }

        state.m_previousAngularSpeed = angularVelocity;
        state.m_previousLinearSpeed = linearVelocity;

        // Calculate wheel speeds
        auto& jointVelocities = db.outputs.velocityCommand();
        jointVelocities.resize(2);
        jointVelocities[0] =
            ((2 * linearVelocity) - (angularVelocity * state.m_wheelDistance)) / (2 * state.m_wheelRadius);
        jointVelocities[1] =
            ((2 * linearVelocity) + (angularVelocity * state.m_wheelDistance)) / (2 * state.m_wheelRadius);

        jointVelocities[0] = std::max(-state.m_maxWheelSpeed, std::min(state.m_maxWheelSpeed, jointVelocities[0]));
        jointVelocities[1] = std::max(-state.m_maxWheelSpeed, std::min(state.m_maxWheelSpeed, jointVelocities[1]));

        return true;
    }

    void reset() override
    {
        if (!m_nodeObj.iNode)
        {
            return;
        }

        GraphObj graphObj{ m_nodeObj.iNode->getGraph(m_nodeObj) };
        GraphContextObj context{ graphObj.iGraph->getDefaultGraphContext(graphObj) };

        // Reset linear velocity
        AttributeObj linearAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "inputs:linearVelocity");
        auto linearHandle = linearAttr.iAttribute->getAttributeDataHandle(linearAttr, kAccordingToContextIndex);
        double* linearVelocity = getDataW<double>(context, linearHandle);
        if (linearVelocity)
        {
            *linearVelocity = 0;
        }

        // Reset angular velocity
        AttributeObj angularAttr = m_nodeObj.iNode->getAttribute(m_nodeObj, "inputs:angularVelocity");
        auto angularHandle = angularAttr.iAttribute->getAttributeDataHandle(angularAttr, kAccordingToContextIndex);
        double* angularVelocity = getDataW<double>(context, angularHandle);
        if (angularVelocity)
        {
            *angularVelocity = 0;
        }

        m_previousLinearSpeed = 0.0;
        m_previousAngularSpeed = 0.0;
        m_initialized = false;
    }

private:
    bool m_initialized = false;
    double m_maxAngularSpeed = 1.0e7;
    double m_maxWheelSpeed = 1.0e7;
    double m_maxLinearSpeed = 1.0e7;
    double m_maxAcceleration = 0.0;
    double m_maxDeceleration = 0.0;
    double m_maxAngularAcceleration = 0.0;
    double m_previousLinearSpeed = 0.0;
    double m_previousAngularSpeed = 0.0;
    double m_wheelDistance = 0;
    double m_wheelRadius = 0;
    NodeObj m_nodeObj;
};

REGISTER_OGN_NODE()
}
}
}
