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
#include <isaacsim/sensors/experimental/physics/IContactSensor.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacReadContactSensorDatabase.h>
#include <string>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
namespace nodes
{

using experimental::physics::ContactSensorReading;
using experimental::physics::IContactSensor;

class OgnIsaacReadContactSensor : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacReadContactSensor()
    {
        cleanup();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacReadContactSensorDatabase::sPerInstanceState<OgnIsaacReadContactSensor>(nodeObj, instanceId);
        state.m_contactInterface = carb::getCachedInterface<IContactSensor>();
    }

    static bool compute(OgnIsaacReadContactSensorDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadContactSensor>();

        if (!state.m_contactInterface)
        {
            state.m_contactInterface = carb::getCachedInterface<IContactSensor>();
            if (!state.m_contactInterface)
            {
                setDefaultOutputs(db);
                db.logError("Failed to acquire IContactSensor interface");
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return false;
            }
        }

        const auto& csPrim = db.inputs.csPrim();
        if (csPrim.empty())
        {
            setDefaultOutputs(db);
            db.logError("Invalid contact sensor prim");
            db.outputs.execOut() = kExecutionAttributeStateDisabled;
            return false;
        }

        std::string primPath = omni::fabric::toSdfPath(csPrim[0]).GetString();

        if (state.m_firstFrame || primPath != state.m_primPath || !state.m_sensorCreated)
        {
            if (state.m_sensorCreated)
            {
                state.m_contactInterface->removeSensor(state.m_primPath.c_str());
                state.m_sensorCreated = false;
            }

            state.m_primPath = primPath;
            state.m_sensorCreated = state.m_contactInterface->createSensor(primPath.c_str());
            state.m_firstFrame = false;

            if (!state.m_sensorCreated)
            {
                setDefaultOutputs(db);
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return true;
            }
        }

        ContactSensorReading reading = state.m_contactInterface->getSensorReading(state.m_primPath.c_str());

        if (reading.isValid)
        {
            db.outputs.inContact() = reading.inContact;
            db.outputs.value() = reading.value;
            db.outputs.sensorTime() = reading.time;
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }

        if (state.m_sensorCreated)
        {
            state.m_contactInterface->removeSensor(state.m_primPath.c_str());
            state.m_sensorCreated = false;
        }
        state.m_sensorCreated = state.m_contactInterface->createSensor(state.m_primPath.c_str());
        if (state.m_sensorCreated)
        {
            reading = state.m_contactInterface->getSensorReading(state.m_primPath.c_str());
            if (reading.isValid)
            {
                db.outputs.inContact() = reading.inContact;
                db.outputs.value() = reading.value;
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
        if (m_contactInterface && m_sensorCreated)
        {
            m_contactInterface->removeSensor(m_primPath.c_str());
            m_sensorCreated = false;
        }
        m_primPath.clear();
    }

    static void setDefaultOutputs(OgnIsaacReadContactSensorDatabase& db)
    {
        db.outputs.inContact() = false;
        db.outputs.value() = 0.0f;
        db.outputs.sensorTime() = 0.0f;
    }

    IContactSensor* m_contactInterface = nullptr;
    bool m_sensorCreated = false;
    std::string m_primPath;
    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()

} // namespace nodes
} // namespace physics
} // namespace sensors
} // namespace isaacsim
