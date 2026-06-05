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

// clang-format off
#include <pch/UsdPCH.h>
#include <omni/physx/ContactEvent.h>
// clang-format on

#include <carb/profiler/Profile.h>

#include <isaacsim/sensors/physics/ContactManager.h>


namespace isaacsim
{
namespace sensors
{
namespace physics
{
ContactManager::ContactManager() = default;

ContactManager::~ContactManager()
{
    m_contactCallbackPtr = nullptr;
}

void ContactManager::resetSensors()
{
    for (auto& it : m_contactRawMap)
    {
        it.second.resize(0);
    }
    m_contactRaw.clear();
    m_currentTime = 0.0f;
}

void ContactManager::processContact(const omni::physx::ContactEventHeader c,
                                    const omni::physx::ContactData* contactDataBuffer,
                                    uint32_t& dataIdx)
{
    // CARB_LOG_INFO("onContactReport");
    switch (c.type)
    {
    case omni::physx::ContactEventType::Enum::eCONTACT_FOUND:
    case omni::physx::ContactEventType::Enum::eCONTACT_PERSIST:
    {
        // CARB_LOG_INFO("Contact Header");

        // pxr::SdfPath body0 = reinterpret_cast<const pxr::SdfPath&>(c.actor0);
        // pxr::SdfPath body1 = reinterpret_cast<const pxr::SdfPath&>(c.actor1);

        // CARB_LOG_INFO("Collision between: Body 0: %s Body 1: %s \n", (char*)body0.GetText(),
        // (char*)body1.GetText()); CARB_LOG_INFO("%s, %s", body0.GetText(), body1.GetText());
        CsRawData contact;
        contact.time = m_currentTime;
        contact.dt = m_currentDt;
        contact.body0 = c.actor0;
        contact.body1 = c.actor1;
        removeRawData(ContactPair(c.actor0, c.actor1));
        // CARB_LOG_INFO("Adding to contact Raw");
        for (uint32_t i = 0; i < c.numContactData; i++)
        {
            auto data = contactDataBuffer[i + dataIdx];
            // CARB_LOG_INFO("Contact Data");
            contact.normal = data.normal;
            contact.position = data.position;
            contact.impulse = data.impulse;
            m_contactRaw.push_back(contact);
        }
        dataIdx += c.numContactData;
        break;
    }
    case omni::physx::ContactEventType::Enum::eCONTACT_LOST:
    {
        // search for contact on persistent data
        // CARB_LOG_INFO("Contact Lost");

        removeRawData(ContactPair(c.actor0, c.actor1));
        break;
    }
    }
}

CsRawData* ContactManager::getCsRawData(const char* usdPath, size_t& size)
{
    pxr::SdfPath path(usdPath);
    return getCsRawData(asInt(path), size);
}

CsRawData* ContactManager::getCsRawData(uint64_t token, size_t& size)
{
    // If filtered list was not generated, create it now
    if (m_contactRawMap.find(token) == m_contactRawMap.end() || m_contactRawMap[token].empty())
    {
        m_contactRawMap[token].resize(m_contactRaw.size());
        auto it = std::copy_if(m_contactRaw.begin(), m_contactRaw.end(), m_contactRawMap[token].begin(),
                               [token](const CsRawData& i) { return i.body0 == token || i.body1 == token; });
        m_contactRawMap[token].resize(std::distance(m_contactRawMap[token].begin(), it));
    }
    size = m_contactRawMap[token].size();
    return m_contactRawMap[token].data();
}

void ContactManager::removeRawData(const ContactPair& p)
{
    // CARB_LOG_INFO("Remove Raw Data %s %s", std::string(p.body0).c_str(), std::string(p.body1).c_str());
    if (m_contactRawMap.find(p.body0) != m_contactRawMap.end())
    {
        m_contactRawMap[p.body0].resize(0);
    }
    if (m_contactRawMap.find(p.body1) != m_contactRawMap.end())
    {
        m_contactRawMap[p.body1].resize(0);
    }
    if (!m_contactRaw.empty())
    {
        auto it = std::remove_if(
            m_contactRaw.begin(), m_contactRaw.end(), [p](const CsRawData& d) { return p == ContactPair(d); });
        m_contactRaw.erase(it, m_contactRaw.end());
    }
}

void ContactManager::onPhysicsStep(const float& currentTime, const float& timeElapsed)
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] Contact Sensor manager - physics step");
    m_currentTime = currentTime;
    m_currentDt = timeElapsed;

    // Clear previous contact data at start of physics step
    m_contactRaw.clear();
    for (auto& it : m_contactRawMap)
    {
        it.second.clear();
    }

    const omni::physx::ContactEventHeader* contactEventHeadersBuffer = nullptr;
    const omni::physx::ContactData* contactDataBuffer = nullptr;
    const ::omni::physx::FrictionAnchor* frictionAnchorData = nullptr;
    uint32_t numContactHeaders = 0;
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Contact Sensor manager - Get Data");
        auto* physxSim = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        if (!physxSim)
        {
            CARB_LOG_WARN("IPhysxSimulation interface not available, skipping contact report");
            return;
        }
        uint32_t numContactData = 0;
        uint32_t numFrictionAnchorData = 0;
        numContactHeaders = physxSim->getFullContactReport(
            &contactEventHeadersBuffer, &contactDataBuffer, numContactData, &frictionAnchorData, numFrictionAnchorData);
    }
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Contact Sensor manager - update lists");
        uint32_t dataIdx = 0;
        for (uint32_t i = 0; i < numContactHeaders; i++)
        {
            const omni::physx::ContactEventHeader c = contactEventHeadersBuffer[i];
            processContact(c, contactDataBuffer, dataIdx);
        }
    }
}

float ContactManager::getCurrentTime()
{
    return m_currentTime;
}

}
}
}
