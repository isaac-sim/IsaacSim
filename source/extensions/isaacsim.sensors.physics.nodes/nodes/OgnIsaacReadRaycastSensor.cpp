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
#include <isaacsim/sensors/experimental/physics/IRaycastSensor.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacReadRaycastSensorDatabase.h>
#include <string>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
namespace nodes
{

using experimental::physics::IRaycastSensor;
using experimental::physics::RaycastSensorReading;

class OgnIsaacReadRaycastSensor : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacReadRaycastSensor()
    {
        cleanup();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacReadRaycastSensorDatabase::sPerInstanceState<OgnIsaacReadRaycastSensor>(nodeObj, instanceId);
        state.m_raycastInterface = carb::getCachedInterface<IRaycastSensor>();
    }

    static bool compute(OgnIsaacReadRaycastSensorDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadRaycastSensor>();

        if (!state.m_raycastInterface)
        {
            state.m_raycastInterface = carb::getCachedInterface<IRaycastSensor>();
            if (!state.m_raycastInterface)
            {
                setDefaultOutputs(db);
                db.logError("Failed to acquire IRaycastSensor interface");
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return false;
            }
        }

        const auto& sensorPrim = db.inputs.raycastSensorPrim();
        if (sensorPrim.empty())
        {
            setDefaultOutputs(db);
            db.logError("Invalid physics raycast sensor prim");
            db.outputs.execOut() = kExecutionAttributeStateDisabled;
            return false;
        }

        std::string primPath = omni::fabric::toSdfPath(sensorPrim[0]).GetString();

        if (state.m_firstFrame || primPath != state.m_primPath || !state.m_sensorCreated)
        {
            if (state.m_sensorCreated)
            {
                state.m_raycastInterface->removeSensor(state.m_primPath.c_str());
                state.m_sensorCreated = false;
            }

            state.m_primPath = primPath;
            state.m_sensorCreated = state.m_raycastInterface->createSensor(primPath.c_str());
            state.m_firstFrame = false;

            if (!state.m_sensorCreated)
            {
                setDefaultOutputs(db);
                db.outputs.execOut() = kExecutionAttributeStateDisabled;
                return true;
            }
        }

        RaycastSensorReading reading = state.m_raycastInterface->getSensorReading(state.m_primPath.c_str());

        if (reading.isValid)
        {
            fillOutputsFromReading(db, reading);
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
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
        if (m_raycastInterface && m_sensorCreated)
        {
            m_raycastInterface->removeSensor(m_primPath.c_str());
            m_sensorCreated = false;
        }
        m_primPath.clear();
    }

    static void fillOutputsFromReading(OgnIsaacReadRaycastSensorDatabase& db, const RaycastSensorReading& reading)
    {
        const uint32_t n = reading.rayCount;
        db.outputs.sensorTime() = reading.time;
        db.outputs.numRays() = n;

        db.outputs.depths().resize(n);
        for (uint32_t i = 0; i < n; i++)
        {
            db.outputs.depths().at(i) = reading.depths[i];
        }

        const uint32_t n3 = n * 3;
        db.outputs.hitPositions().resize(n3);
        db.outputs.hitNormals().resize(n3);
        for (uint32_t i = 0; i < n3; i++)
        {
            db.outputs.hitPositions().at(i) = reading.hitPositions[i];
            db.outputs.hitNormals().at(i) = reading.hitNormals[i];
        }

        db.outputs.hitPrimPaths().resize(n);
        if (reading.hitPrimPaths)
        {
            for (uint32_t i = 0; i < n; i++)
            {
                db.outputs.hitPrimPaths().at(i) =
                    db.stringToToken(reading.hitPrimPaths[i] ? reading.hitPrimPaths[i] : "");
            }
        }
        else
        {
            for (uint32_t i = 0; i < n; i++)
            {
                db.outputs.hitPrimPaths().at(i) = db.stringToToken("");
            }
        }

        db.outputs.beamOrigins().resize(n);
        db.outputs.beamEndPoints().resize(n);
        if (n > 0 && reading.rayOriginsWorld && reading.rayEndPointsWorld)
        {
            for (uint32_t i = 0; i < n; i++)
            {
                const float* o = reading.rayOriginsWorld + i * 3;
                const float* e = reading.rayEndPointsWorld + i * 3;
                db.outputs.beamOrigins()[i] = pxr::GfVec3f(o[0], o[1], o[2]);
                db.outputs.beamEndPoints()[i] = pxr::GfVec3f(e[0], e[1], e[2]);
            }
        }
    }

    static void setDefaultOutputs(OgnIsaacReadRaycastSensorDatabase& db)
    {
        db.outputs.sensorTime() = 0.0f;
        db.outputs.numRays() = 0;
        db.outputs.depths().resize(0);
        db.outputs.hitPositions().resize(0);
        db.outputs.hitNormals().resize(0);
        db.outputs.hitPrimPaths().resize(0);
        db.outputs.beamOrigins().resize(0);
        db.outputs.beamEndPoints().resize(0);
    }

    IRaycastSensor* m_raycastInterface = nullptr;
    bool m_sensorCreated = false;
    std::string m_primPath;
    bool m_firstFrame = true;
};

REGISTER_OGN_NODE()

} // namespace nodes
} // namespace physics
} // namespace sensors
} // namespace isaacsim
