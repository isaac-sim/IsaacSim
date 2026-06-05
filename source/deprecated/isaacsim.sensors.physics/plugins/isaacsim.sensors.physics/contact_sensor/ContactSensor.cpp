// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifdef _WIN32
#    pragma warning(push)
#    pragma warning(disable : 4996)
#endif
// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <isaacsim/core/includes/Pose.h>
#include <isaacsim/sensors/physics/ContactManager.h>
#include <isaacsim/sensors/physics/ContactSensor.h>
#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <isaacsim/sensors/physics/IsaacSensorComponent.h>
#include <isaacsim/sensors/physics/IsaacSensorManager.h>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
ContactSensor::~ContactSensor()
{
    reset();
}

void ContactSensor::reset()
{
    m_contactManagerPtr = nullptr;
    m_currentTime = 0.0f;
    m_sensorTime = 0.0f;
    m_timeSeconds = 0.0f;
    m_timeDelta = 0.0f;
    m_readingPair[0] = m_readingPair[1] = CsReading();
    m_contactsRawData = nullptr;
}

CsRawData* ContactSensor::getRawData(size_t& size)
{
    CARB_LOG_WARN_ONCE("*** warning: get_contact_sensor_raw_data is deprecated and will replaced in the next release");

    if (m_contactsRawData == nullptr)
    {
        size = 0;
    }
    else
    {
        size = m_size; // Return the actual number of contacts processed
    }
    return m_contactsRawData;
}

CsReading ContactSensor::getSensorReading(const bool& getLatestValue)
{
    if (m_props.sensorPeriod > 0 && m_props.sensorPeriod < m_timeDelta && m_timeDelta - m_props.sensorPeriod > 0.001)
    {
        CARB_LOG_WARN_ONCE(
            "*** warning: contact sensor frequency is higher than physics frequency, returning the latest physics value");
    }

    CsReading sensorReading = CsReading();

    if (m_enabled)
    {
        // if sensor period is shorter than physics downtime, or user choose latest value return current value
        // or internal time + sensor period time is behind the last step time (something went wrong
        // i.e. sensor was disabled for a long time and then re-enabled)
        // get the latest time and measurement
        if (m_props.sensorPeriod <= m_timeDelta ||
            m_sensorTime + m_props.sensorPeriod < m_readingPair[!m_current].time || getLatestValue)
        {
            sensorReading = m_readingPair[m_current];
            sensorReading.isValid = true;
            if (m_props.sensorPeriod > 0 && m_sensorTime + m_props.sensorPeriod < m_readingPair[!m_current].time)
            {
                CARB_LOG_WARN("*** warning Contact sensor time out of sync, using latest measurements");
            }
        }
        else
        {
            sensorReading = m_sensorReading;
            sensorReading.isValid = true;
        }
    }
    return sensorReading;
}

void ContactSensor::processRawContacts(CsRawData* rawContact, const size_t& size, const size_t& index, const double& time)
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] Contact Sensor::processRawContacts");
    m_readingPair[index].value = 0.0f;
    m_readingPair[index].inContact = false;
    m_readingPair[index].time = static_cast<float>(time);
    if (rawContact == nullptr || rawContact->time == 0)
    {
        // CARB_LOG_INFO("Failed to process data, raw contact is null");
        return;
    }

    if (size > static_cast<size_t>(0))
    {
        usdrt::GfMatrix4d usdTransform =
            isaacsim::core::includes::pose::computeWorldXformNoCache(m_stage, m_usdrtStage, m_prim.GetPath());
        const usdrt::GfVec3d sensorPos = usdTransform.ExtractTranslation();
        pxr::GfVec3d pos(sensorPos[0], sensorPos[1], sensorPos[2]);
        pxr::GfVec3d totalImpulse(0.0, 0.0, 0.0);
        for (size_t i = 0; i < size; ++i)
        {
            pxr::GfVec3d contactPoint(rawContact[i].position.x, rawContact[i].position.y, rawContact[i].position.z);
            // CARB_LOG_WARN("contact Pos: %f %f %f", contactPoint[0], contactPoint[1], contactPoint[2]);
            // CARB_LOG_WARN("sensor Pos: %f %f %f", pos[0],pos[1], pos[2]);
            auto d = pxr::GfVec3d(0.0f); // dp*rawContact->dt; Pending update on physics contact position being delayed
                                         // a few frames dp is the linear vel of the parent
            auto distance = pos - contactPoint - d;
            // pos.GetLength(), m_props.radius);

            // Check if the distance from sensor to contact position is within sensor radius
            if (m_props.radius <= 0.0f || distance.GetLength() < static_cast<double>(m_props.radius))
            {
                pxr::GfVec3d contactImpulse(rawContact[i].impulse.x, rawContact[i].impulse.y, rawContact[i].impulse.z);
                m_readingPair[index].inContact = true;

                // Determine if impulse should be inverted based on which body the sensor belongs to
                // Following RigidContactView logic: invert if sensor's parent is body1
                uint64_t sensorParentToken = asInt(m_parentPrim.GetPath());
                bool shouldInvert = (sensorParentToken == rawContact[i].body1);

                // Apply impulse along normal direction, with proper sign
                if (shouldInvert)
                {
                    contactImpulse = -contactImpulse;
                }
                totalImpulse += contactImpulse;
            }
        }
        // Calculate final force value
        float forceValue = static_cast<float>((totalImpulse.GetLength()) / rawContact[0].dt);
        m_readingPair[index].value = std::min(forceValue, m_props.maxThreshold);

        // if force reading is lower than the min threshold, override to no contact
        if (m_readingPair[index].value < m_props.minThreshold)
        {
            m_readingPair[index].value = 0;
            m_readingPair[index].inContact = false;
        }
    }
}

void ContactSensor::onPhysicsStep()
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] ContactSensor::physics step");
    if (m_contactManagerPtr == nullptr)
    {
        CARB_LOG_ERROR("*** error: ContactManager not found");
        return;
    }

    m_contactsRawData = m_contactManagerPtr->getCsRawData(asInt(m_parentPrim.GetPath()), m_size);

    m_current = !m_current;
    processRawContacts(m_contactsRawData, m_size, m_current, m_timeSeconds);

    // clear raw data if not in contact
    if (m_readingPair[m_current].inContact == false)
    {
        m_contactsRawData = nullptr;
    }

    if (m_props.sensorPeriod <= m_timeDelta)
    {
        m_sensorTime = m_readingPair[m_current].time;
    }
    else if (m_sensorTime + m_props.sensorPeriod <= m_readingPair[m_current].time)
    {
        m_sensorTime += m_props.sensorPeriod;
        // the sensor measurement is closer to current reading than the last reading
        if (abs(m_readingPair[m_current].time - m_sensorTime) <= abs(m_readingPair[!m_current].time - m_sensorTime))
        {
            m_sensorReading = m_readingPair[m_current];
        }
        else
        {
            m_sensorReading = m_readingPair[!m_current];
        }
        m_sensorReading.time = m_sensorTime;
    }
}

void ContactSensor::setContactReportApi()
{
    if (!m_parentPrim.GetPrim().IsValid())
    {
        CARB_LOG_ERROR("*** error: failed to set Contact Report API, parent prim is invalid or not found");
        return;
    }

    pxr::PhysxSchemaPhysxContactReportAPI contactReportAPI =
        pxr::PhysxSchemaPhysxContactReportAPI::Get(m_stage, m_parentPrim.GetPath());

    if (!contactReportAPI)
    {
        CARB_LOG_ERROR(
            "*** error: %s parent prim is missing contact report API, automatically adding contact report API, stop and play the simulation for this change to take effect",
            this->m_prim.GetPath().GetString().c_str());
        contactReportAPI = pxr::PhysxSchemaPhysxContactReportAPI::Apply(m_parentPrim.GetPrim());
    }
    if (!contactReportAPI.GetReportPairsRel())
    {
        contactReportAPI.CreateReportPairsRel();
    }
    contactReportAPI.GetThresholdAttr().Set(0.0f);

    pxr::PhysxSchemaPhysxRigidBodyAPI rigidBodyAPI =
        pxr::PhysxSchemaPhysxRigidBodyAPI::Get(m_stage, m_parentPrim.GetPath());

    if (rigidBodyAPI)
    {
        pxr::VtValue vtFloatValue(0.0f);
        rigidBodyAPI.CreateSleepThresholdAttr(vtFloatValue);
    }
}

bool ContactSensor::findValidParent()
{
    pxr::UsdPrim tempPrim = this->m_stage->GetPrimAtPath(this->m_prim.GetPath()).GetParent();
    while (tempPrim.IsValid() && tempPrim.GetName().GetString() != "/")
    {
        // check if it's a rigid body
        bool rigidBodyEnabled = false;
        tempPrim.GetAttribute(pxr::TfToken("physics:rigidBodyEnabled")).Get(&rigidBodyEnabled);
        if (rigidBodyEnabled)
        {
            m_parentPrim = tempPrim;
            // findParentScale();
            setContactReportApi();
            return true;
        }
        // go to parent
        tempPrim = tempPrim.GetParent();
    }
    CARB_LOG_WARN("No valid parent for %s with a rigid body api was found, sensor will not be created",
                  this->m_prim.GetPath().GetString().c_str());
    return false;
}

void ContactSensor::onComponentChange()
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] Contact Sensor - component change");
    IsaacSensorComponentBase::onComponentChange();
    float sensorPeriod = 0.0f;
    float radius = 0.0f;
    pxr::GfVec2f thresholdAttr = pxr::GfVec2f(0.01f, 100000.0f);

    using namespace isaacsim::robot::schema::sensors;
    // contact sensor onComponentChange
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kThresholdAttr), thresholdAttr);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kRadiusAttr), radius);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kColorAttr), m_color);
    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kSensorPeriodAttr), sensorPeriod);

    setContactReportApi();
    const float* thresholds = thresholdAttr.GetArray();

    // contact sensor props
    m_props.maxThreshold = thresholds[1];
    m_props.minThreshold = thresholds[0];
    m_props.radius = radius;
    m_props.sensorPeriod = sensorPeriod;

    if (m_previousEnabled != this->m_enabled)
    {
        if (m_enabled)
        {
            this->onPhysicsStep(); // force on physics step to run to get up to date value
            m_readingPair[!m_current] = m_readingPair[m_current]; // first step, copy latest values for both
                                                                  // readingpairs
            m_sensorTime = m_readingPair[m_current].time;
            m_sensorReading = m_readingPair[m_current];
        }
        else
        {
            this->onStop();
        }
        m_previousEnabled = this->m_enabled;
    }
}

void ContactSensor::onStop()
{
    m_currentTime = 0.0f;
    m_sensorTime = 0.0f;
    m_timeSeconds = 0.0f;
    m_timeDelta = 0.0f;
    m_readingPair[0] = m_readingPair[1] = CsReading();


    m_contactsRawData = nullptr;
}

void ContactSensor::printRawData(CsRawData* data)
{
    if (data == nullptr)
    {
        CARB_LOG_WARN("Raw Data is NULL");
        return;
    }
    float time = data->time;

    uint64_t body0 = data->body0;
    uint64_t body1 = data->body1;

    float posX = data->position.x;
    float posY = data->position.y;
    float posZ = data->position.z;

    float normalX = data->normal.x;
    float normalY = data->normal.y;
    float normalZ = data->normal.z;

    float impulseX = data->impulse.x;
    float impulseY = data->impulse.y;
    float impulseZ = data->impulse.z;

    CARB_LOG_INFO("Raw Data \n");
    CARB_LOG_INFO("Time: %f\n", time);
    CARB_LOG_INFO("Body 0: %s Body 1: %s \n", isaacsim::core::includes::getSdfPathFromUint64(body0).GetString().c_str(),
                  isaacsim::core::includes::getSdfPathFromUint64(body1).GetString().c_str());
    CARB_LOG_INFO("Position: %f, %f, %f \n", posX, posY, posZ);
    CARB_LOG_INFO("Normal: %f, %f, %f \n", normalX, normalY, normalZ);
    CARB_LOG_INFO("Impulse: %f, %f, %f \n", impulseX, impulseY, impulseZ);
}

void ContactSensor::printReadingPair()
{
    CsReading reading0 = m_readingPair[0];
    float value0 = reading0.value;
    float time0 = reading0.time;

    CsReading reading1 = m_readingPair[1];
    float value1 = reading1.value;
    float time1 = reading1.time;

    CARB_LOG_INFO("Reading Pair\n");
    CARB_LOG_INFO("Reading 0: value: %f, time: %f \n", value0, time0);
    CARB_LOG_INFO("Reading 1: value: %f, time: %f \n", value1, time1);
}


}
}
}
#ifdef _WIN32
#    pragma warning(pop)
#endif
