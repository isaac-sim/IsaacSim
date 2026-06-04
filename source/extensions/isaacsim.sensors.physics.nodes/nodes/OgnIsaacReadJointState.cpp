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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/Defines.h>
#include <carb/Types.h>

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/sensors/experimental/physics/IJointStateSensor.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacReadJointStateDatabase.h>
#include <algorithm>
#include <string>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
namespace nodes
{

using experimental::physics::IJointStateSensor;
using experimental::physics::JointStateSensorReading;

class OgnIsaacReadJointState : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacReadJointState()
    {
        cleanup();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadJointStateDatabase::sPerInstanceState<OgnIsaacReadJointState>(nodeObj, instanceId);
        state.m_jointStateInterface = carb::getCachedInterface<IJointStateSensor>();
    }

    static bool compute(OgnIsaacReadJointStateDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadJointState>();

        if (!state.m_jointStateInterface)
        {
            state.m_jointStateInterface = carb::getCachedInterface<IJointStateSensor>();
            if (!state.m_jointStateInterface)
            {
                setDefaultOutputs(db);
                db.logError("Failed to acquire IJointStateSensor interface");
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return false;
            }
        }

        const auto& prim = db.inputs.prim();
        if (prim.empty())
        {
            setDefaultOutputs(db);
            db.logError("Failed to create joint state sensor, unable to find prim path");
            db.outputs.execOut() = kExecutionAttributeStateDisabled;
            return false;
        }

        std::string primPath = omni::fabric::toSdfPath(prim[0]).GetString();

        if (state.m_firstFrame || primPath != state.m_primPath || !state.m_sensorCreated)
        {
            if (state.m_sensorCreated)
            {
                state.m_jointStateInterface->removeSensor(state.m_primPath.c_str());
                state.m_sensorCreated = false;
            }

            state.m_primPath = primPath;
            state.m_sensorCreated = state.m_jointStateInterface->createSensor(primPath.c_str());
            state.m_firstFrame = false;

            if (!state.m_sensorCreated)
            {
                setDefaultOutputs(db);
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return true;
            }
        }

        JointStateSensorReading reading = state.m_jointStateInterface->getSensorReading(state.m_primPath.c_str());

        if (reading.isValid)
        {
            fillOutputsFromReading(db, reading);
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }

        if (state.m_sensorCreated)
        {
            state.m_jointStateInterface->removeSensor(state.m_primPath.c_str());
            state.m_sensorCreated = false;
        }

        state.m_sensorCreated = state.m_jointStateInterface->createSensor(state.m_primPath.c_str());

        if (state.m_sensorCreated)
        {
            reading = state.m_jointStateInterface->getSensorReading(state.m_primPath.c_str());
            if (reading.isValid)
            {
                fillOutputsFromReading(db, reading);
                db.outputs.execOut() = kExecutionAttributeStateEnabled;
                return true;
            }
        }

        setDefaultOutputs(db);
        db.outputs.execOut() = kExecutionAttributeStateDisabled;
        return true;
    }

    void reset() override
    {
        cleanup();
        m_firstFrame = true;
    }

private:
    void cleanup()
    {
        if (m_jointStateInterface && m_sensorCreated)
        {
            m_jointStateInterface->removeSensor(m_primPath.c_str());
            m_sensorCreated = false;
        }
        m_primPath.clear();
    }

    static void fillOutputsFromReading(OgnIsaacReadJointStateDatabase& db, const JointStateSensorReading& reading)
    {
        const int32_t n = reading.dofCount;
        db.outputs.sensorTime() = reading.time;
        db.outputs.stageMetersPerUnit() = reading.stageMetersPerUnit;

        db.outputs.jointNames().resize(n);
        for (int32_t i = 0; i < n; i++)
        {
            db.outputs.jointNames().at(i) = db.stringToToken(reading.dofNames[i]);
        }

        db.outputs.jointPositions().resize(n);
        db.outputs.jointVelocities().resize(n);
        db.outputs.jointEfforts().resize(n);
        db.outputs.jointDofTypes().resize(n);

        for (int32_t i = 0; i < n; i++)
        {
            db.outputs.jointPositions().at(i) = static_cast<double>(reading.positions[i]);
            db.outputs.jointVelocities().at(i) = static_cast<double>(reading.velocities[i]);
            db.outputs.jointEfforts().at(i) = static_cast<double>(reading.efforts[i]);
            db.outputs.jointDofTypes().at(i) = reading.dofTypes ? reading.dofTypes[i] : static_cast<uint8_t>(0);
        }
    }

    static void setDefaultOutputs(OgnIsaacReadJointStateDatabase& db)
    {
        db.outputs.sensorTime() = 0.0f;
        db.outputs.stageMetersPerUnit() = 0.0f;
        db.outputs.jointNames().resize(0);
        db.outputs.jointPositions().resize(0);
        db.outputs.jointVelocities().resize(0);
        db.outputs.jointEfforts().resize(0);
        db.outputs.jointDofTypes().resize(0);
    }

    IJointStateSensor* m_jointStateInterface = nullptr;
    bool m_sensorCreated = false;
    std::string m_primPath;
    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()

} // namespace nodes
} // namespace physics
} // namespace sensors
} // namespace isaacsim
