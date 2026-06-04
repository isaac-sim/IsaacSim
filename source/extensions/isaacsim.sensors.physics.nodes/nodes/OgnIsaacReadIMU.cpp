// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <isaacsim/sensors/experimental/physics/IImuSensor.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdContextIncludes.h>
#include <pxr/base/gf/quatd.h>

#include <OgnIsaacReadIMUDatabase.h>
#include <string>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
namespace nodes
{

using experimental::physics::IImuSensor;
using experimental::physics::ImuSensorReading;

class OgnIsaacReadIMU : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacReadIMU()
    {
        cleanup();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadIMUDatabase::sPerInstanceState<OgnIsaacReadIMU>(nodeObj, instanceId);
        state.m_imuInterface = carb::getCachedInterface<IImuSensor>();
    }

    static bool compute(OgnIsaacReadIMUDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadIMU>();

        if (!state.m_imuInterface)
        {
            state.m_imuInterface = carb::getCachedInterface<IImuSensor>();
            if (!state.m_imuInterface)
            {
                setDefaultOutputs(db);
                db.logError("Failed to acquire IImuSensor interface");
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return false;
            }
        }

        const auto& imuPrim = db.inputs.imuPrim();
        if (imuPrim.empty())
        {
            setDefaultOutputs(db);
            db.logError("Invalid Imu sensor prim");
            db.outputs.execOut() = kExecutionAttributeStateDisabled;
            return false;
        }

        std::string primPath = omni::fabric::toSdfPath(imuPrim[0]).GetString();

        if (state.m_firstFrame || primPath != state.m_primPath || !state.m_sensorCreated)
        {
            if (state.m_sensorCreated)
            {
                state.m_imuInterface->removeSensor(state.m_primPath.c_str());
                state.m_sensorCreated = false;
            }

            state.m_primPath = primPath;
            state.m_sensorCreated = state.m_imuInterface->createSensor(primPath.c_str());
            state.m_firstFrame = false;

            if (!state.m_sensorCreated)
            {
                setDefaultOutputs(db);
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return true;
            }
        }

        bool readGravity = db.inputs.readGravity();
        ImuSensorReading reading = state.m_imuInterface->getSensorReading(state.m_primPath.c_str(), readGravity);

        if (reading.isValid)
        {
            db.outputs.linAcc() = { reading.linearAccelerationX, reading.linearAccelerationY,
                                    reading.linearAccelerationZ };
            db.outputs.angVel() = { reading.angularVelocityX, reading.angularVelocityY, reading.angularVelocityZ };
            db.outputs.orientation() =
                GfQuatd(reading.orientationW, reading.orientationX, reading.orientationY, reading.orientationZ);
            db.outputs.sensorTime() = reading.time;
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }

        if (state.m_sensorCreated)
        {
            state.m_imuInterface->removeSensor(state.m_primPath.c_str());
            state.m_sensorCreated = false;
        }
        state.m_sensorCreated = state.m_imuInterface->createSensor(state.m_primPath.c_str());
        if (state.m_sensorCreated)
        {
            reading = state.m_imuInterface->getSensorReading(state.m_primPath.c_str(), readGravity);
            if (reading.isValid)
            {
                db.outputs.linAcc() = { reading.linearAccelerationX, reading.linearAccelerationY,
                                        reading.linearAccelerationZ };
                db.outputs.angVel() = { reading.angularVelocityX, reading.angularVelocityY, reading.angularVelocityZ };
                db.outputs.orientation() =
                    GfQuatd(reading.orientationW, reading.orientationX, reading.orientationY, reading.orientationZ);
                db.outputs.sensorTime() = reading.time;
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
        if (m_imuInterface && m_sensorCreated)
        {
            m_imuInterface->removeSensor(m_primPath.c_str());
            m_sensorCreated = false;
        }
        m_primPath.clear();
    }

    static void setDefaultOutputs(OgnIsaacReadIMUDatabase& db)
    {
        db.outputs.linAcc() = { 0.0, 0.0, 0.0 };
        db.outputs.angVel() = { 0.0, 0.0, 0.0 };
        db.outputs.orientation() = GfQuatd(1.0, 0.0, 0.0, 0.0);
        db.outputs.sensorTime() = 0.0f;
    }

    IImuSensor* m_imuInterface = nullptr;
    bool m_sensorCreated = false;
    std::string m_primPath;
    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()

} // namespace nodes
} // namespace physics
} // namespace sensors
} // namespace isaacsim
