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

#include "ContactSensorImpl.h"

#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/experimental/prims/SdfPathToken.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#if defined(_WIN32)
#    include <usdrt/scenegraph/usd/usd/stage.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    include <usdrt/scenegraph/usd/usd/stage.h>
#    pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace experimental
{
namespace physics
{
namespace
{

using core::experimental::prims::IPrimDataReader;
using core::experimental::prims::IPrimDataReaderManager;
using core::experimental::prims::IXformDataView;
using core::simulation_manager::ISimulationManager;

static std::string findParentRigidBody(pxr::UsdStageRefPtr stage, const pxr::SdfPath& sensorPath)
{
    pxr::UsdPrim prim = stage->GetPrimAtPath(sensorPath);
    if (!prim.IsValid())
    {
        return {};
    }

    prim = prim.GetParent();
    while (prim.IsValid() && prim.GetPath() != pxr::SdfPath::AbsoluteRootPath())
    {
        bool enabled = false;
        pxr::UsdAttribute attr = prim.GetAttribute(pxr::TfToken("physics:rigidBodyEnabled"));
        bool hasRigidBodyAPI = prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();
        bool attrValid = attr.IsValid();
        if (attrValid)
        {
            attr.Get(&enabled);
        }

        if (enabled)
        {
            return prim.GetPath().GetString();
        }

        if (hasRigidBodyAPI && !attrValid)
        {
            return prim.GetPath().GetString();
        }

        prim = prim.GetParent();
    }
    return {};
}

using isaacsim::core::experimental::prims::sdfPathToToken;

class ContactDataStore
{
public:
    std::vector<ContactRawData> rawContacts;
    std::map<uint64_t, std::vector<ContactRawData>> perBodyMap;

    void clear()
    {
        rawContacts.clear();
        perBodyMap.clear();
    }

    void removeContactPair(uint64_t body0, uint64_t body1)
    {
        uint64_t lower = std::min(body0, body1);
        uint64_t higher = std::max(body0, body1);

        rawContacts.erase(std::remove_if(rawContacts.begin(), rawContacts.end(),
                                         [lower, higher](const ContactRawData& e)
                                         {
                                             uint64_t entryLower = std::min(e.body0, e.body1);
                                             uint64_t entryHigher = std::max(e.body0, e.body1);
                                             return entryLower == lower && entryHigher == higher;
                                         }),
                          rawContacts.end());

        perBodyMap.erase(body0);
        perBodyMap.erase(body1);
    }

    const std::vector<ContactRawData>& getForBody(uint64_t token)
    {
        auto it = perBodyMap.find(token);
        if (it != perBodyMap.end() && !it->second.empty())
        {
            return it->second;
        }

        auto& vec = perBodyMap[token];
        vec.clear();
        for (const auto& e : rawContacts)
        {
            if (e.body0 == token || e.body1 == token)
            {
                vec.push_back(e);
            }
        }
        return vec;
    }
};

class SensorData
{
public:
    std::string sensorPrimPath;
    std::string parentRigidBodyPath;
    std::string viewId;
    IXformDataView* xformView = nullptr;
    uint64_t parentToken = 0;

    float radius = -1.0f;
    float minThreshold = 0.0f;
    float maxThreshold = 100000.0f;
    bool enabled = true;
    bool previousEnabled = true;

    ContactSensorReading latestReading;
    std::vector<ContactRawData> latestRawContacts;

    void refreshConfig(pxr::UsdStageRefPtr stage)
    {
        using namespace isaacsim::robot::schema::sensors;

        pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(sensorPrimPath));
        if (!prim.IsValid())
        {
            return;
        }

        pxr::UsdAttribute enabledAttr = prim.GetAttribute(kEnabledAttr);
        if (enabledAttr.IsValid())
        {
            bool val = true;
            enabledAttr.Get(&val);
            enabled = val;
        }
        else
        {
            enabled = true;
        }

        pxr::GfVec2f thresholdAttr(0.0f, 100000.0f);
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kThresholdAttr), thresholdAttr);
        const float* t = thresholdAttr.GetArray();
        minThreshold = t[0];
        maxThreshold = t[1];

        float r = -1.0f;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kRadiusAttr), r);
        radius = r;
    }
};

} // namespace

struct ContactSensorImpl::ImplData
{
    long stageId = 0;
    float lastDt = 0.0f;
    int stepCount = 0;
    uint64_t readerGeneration = 0;

    ISimulationManager* simManager = nullptr;
    IPrimDataReaderManager* readerManager = nullptr;
    IPrimDataReader* reader = nullptr;
    omni::physics::IPhysicsSimulation* physicsSimulation = nullptr;
    omni::physics::SubscriptionId physicsStepSub = omni::physics::kInvalidSubscriptionId;
    carb::events::ISubscriptionPtr physicsEventSub;

    pxr::UsdStageRefPtr usdStage;
    usdrt::UsdStageRefPtr usdrtStage;
    std::unordered_map<std::string, SensorData> sensors;
    ContactDataStore contactStore;
};

ContactSensorImpl::ContactSensorImpl() : m_impl(std::make_unique<ImplData>())
{
    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    m_impl->reader = m_impl->readerManager ? m_impl->readerManager->getReader() : nullptr;
    _subscribeToPhysicsStepEvents();
    _subscribeToPhysicsEvents();
}

ContactSensorImpl::~ContactSensorImpl()
{
    shutdown();
}

void ContactSensorImpl::shutdown()
{
    _unsubscribeFromPhysicsStepEvents();
    m_impl->physicsEventSub.reset();
    _clearSensors();
    m_impl->readerManager = nullptr;
    m_impl->reader = nullptr;
    m_impl->simManager = nullptr;
    m_impl->physicsSimulation = nullptr;
    m_impl->usdStage = nullptr;
    m_impl->usdrtStage = nullptr;
    m_impl->stageId = 0;
    m_impl->stepCount = 0;
    m_impl->lastDt = 0.0f;
    m_impl->readerGeneration = 0;
    m_impl->contactStore.clear();
}

void ContactSensorImpl::_initializeFromContext()
{
    auto* usdContext = omni::usd::UsdContext::getContext();
    if (!usdContext)
    {
        return;
    }

    pxr::UsdStageRefPtr stage = usdContext->getStage();
    if (!stage)
    {
        return;
    }

    pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    const long stageId = cache.GetId(stage).ToLongInt();
    if (stageId == 0)
    {
        return;
    }

    _initializeStage(stageId);
    _discoverSensorsFromStage();
}

void ContactSensorImpl::_initializeStage(long stageId)
{
    if (m_impl->stageId == stageId && m_impl->usdStage && m_impl->readerManager && m_impl->reader)
    {
        return;
    }

    if (m_impl->stageId != 0 && m_impl->stageId != stageId)
    {
        _clearSensors();
    }

    m_impl->stageId = stageId;
    m_impl->stepCount = 0;
    m_impl->lastDt = 0.0f;
    m_impl->contactStore.clear();

    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    if (m_impl->readerManager)
    {
        m_impl->readerManager->ensureInitialized(stageId, -1);
        m_impl->reader = m_impl->readerManager->getReader();
    }
    else
    {
        m_impl->reader = nullptr;
    }

    pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    m_impl->usdStage = cache.Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

    if (m_impl->usdStage)
    {
        omni::fabric::UsdStageId fabricStageId = { static_cast<uint64_t>(stageId) };
        omni::fabric::IStageReaderWriter* iStageReaderWriter =
            carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        if (iStageReaderWriter)
        {
            omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(fabricStageId);
            m_impl->usdrtStage = usdrt::UsdStage::Attach(fabricStageId, stageInProgress);
        }
    }
}

bool ContactSensorImpl::createSensor(const char* primPath)
{
    if (!m_impl->usdStage)
    {
        return false;
    }

    std::string key(primPath);
    pxr::SdfPath sdfPath(primPath);
    pxr::UsdPrim prim = m_impl->usdStage->GetPrimAtPath(sdfPath);

    auto existing = m_impl->sensors.find(key);
    if (existing != m_impl->sensors.end())
    {
        // Tear down the cached entry when the prim has been deleted, when its
        // type is no longer IsaacContactSensor, or when its parent rigid body
        // has changed (delete/recreate at the same path can land under a
        // different rigid body). Otherwise reuse the view and refresh config
        // so attribute updates on a recreated prim are picked up.
        if (prim.IsValid() && prim.GetTypeName() == "IsaacContactSensor")
        {
            std::string currentParent = findParentRigidBody(m_impl->usdStage, sdfPath);
            if (currentParent == existing->second.parentRigidBodyPath)
            {
                existing->second.refreshConfig(m_impl->usdStage);
                return true;
            }
        }
        if (m_impl->reader && !existing->second.viewId.empty())
        {
            m_impl->reader->removeView(existing->second.viewId.c_str());
        }
        m_impl->sensors.erase(existing);
    }

    if (!prim.IsValid())
    {
        return false;
    }

    if (prim.GetTypeName() != "IsaacContactSensor")
    {
        return false;
    }

    std::string parentPath = findParentRigidBody(m_impl->usdStage, sdfPath);
    if (parentPath.empty())
    {
        return false;
    }

    // SIDE EFFECT: enableContactReporting() modifies the USD stage on the parent rigid body.
    //
    // PhysX's getFullContactReport() only returns data for bodies that have
    // PhysxContactReportAPI applied. The reader applies this schema (along with
    // threshold and sleep settings) so that contacts are actually reported.
    // These changes persist on the USD stage for the lifetime of the session.
    if (m_impl->reader)
    {
        if (!m_impl->reader->enableContactReporting(parentPath.c_str()))
        {
            CARB_LOG_WARN("ContactSensorImpl: failed to enable contact reporting for '%s'", parentPath.c_str());
        }
    }

    SensorData& sensor = m_impl->sensors[key];
    sensor.sensorPrimPath = primPath;
    sensor.parentRigidBodyPath = parentPath;
    sensor.parentToken = sdfPathToToken(pxr::SdfPath(parentPath));
    sensor.viewId = "contact_xform_" + key;

    if (m_impl->reader)
    {
        const char* sensorPathPtr = primPath;
        sensor.xformView = m_impl->reader->createXformView(sensor.viewId.c_str(), &sensorPathPtr, 1, "physx");
        m_impl->readerGeneration = m_impl->reader->getGeneration();
    }

    sensor.refreshConfig(m_impl->usdStage);

    return true;
}

void ContactSensorImpl::removeSensor(const char* primPath)
{
    auto it = m_impl->sensors.find(std::string(primPath));
    if (it == m_impl->sensors.end())
    {
        return;
    }
    if (m_impl->reader && !it->second.viewId.empty())
    {
        m_impl->reader->removeView(it->second.viewId.c_str());
    }
    m_impl->sensors.erase(it);
}

ContactSensorReading ContactSensorImpl::getSensorReading(const char* primPath)
{
    auto it = m_impl->sensors.find(std::string(primPath));
    if (it == m_impl->sensors.end())
    {
        return ContactSensorReading();
    }

    // Tear down the cached sensor when the underlying USD prim has been removed
    // since the last update. Without this we'd return the last cached reading
    // for a deleted prim, mirroring the gates on IMU and Raycast.
    if (m_impl->usdStage)
    {
        pxr::UsdPrim prim = m_impl->usdStage->GetPrimAtPath(pxr::SdfPath(primPath));
        if (!prim.IsValid())
        {
            if (m_impl->reader && !it->second.viewId.empty())
            {
                m_impl->reader->removeView(it->second.viewId.c_str());
            }
            m_impl->sensors.erase(it);
            return ContactSensorReading();
        }
    }

    return it->second.latestReading;
}

void ContactSensorImpl::getRawContacts(const char* primPath, const ContactRawData** outData, int32_t* outCount)
{
    if (!outData || !outCount)
    {
        return;
    }

    *outData = nullptr;
    *outCount = 0;

    auto it = m_impl->sensors.find(std::string(primPath));
    if (it == m_impl->sensors.end())
    {
        return;
    }

    // Mirror the prim-deletion gate in getSensorReading so raw-data callers
    // don't see contacts attributed to a deleted sensor.
    if (m_impl->usdStage)
    {
        pxr::UsdPrim prim = m_impl->usdStage->GetPrimAtPath(pxr::SdfPath(primPath));
        if (!prim.IsValid())
        {
            if (m_impl->reader && !it->second.viewId.empty())
            {
                m_impl->reader->removeView(it->second.viewId.c_str());
            }
            m_impl->sensors.erase(it);
            return;
        }
    }

    const auto& contacts = it->second.latestRawContacts;
    if (!contacts.empty())
    {
        *outData = contacts.data();
        *outCount = static_cast<int32_t>(contacts.size());
    }
}

void ContactSensorImpl::_discoverSensorsFromStage()
{
    if (!m_impl->usdStage)
    {
        return;
    }

    int found = 0;
    for (auto prim : m_impl->usdStage->Traverse())
    {
        if (prim.GetTypeName() == "IsaacContactSensor")
        {
            found++;
            (void)createSensor(prim.GetPath().GetString().c_str());
        }
    }
}

void ContactSensorImpl::_clearSensors()
{
    for (auto& [id, sensor] : m_impl->sensors)
    {
        (void)id;
        if (m_impl->reader && !sensor.viewId.empty())
        {
            m_impl->reader->removeView(sensor.viewId.c_str());
        }
    }
    m_impl->sensors.clear();
    m_impl->contactStore.clear();
}

void ContactSensorImpl::_recreateSensorViews()
{
    if (!m_impl->reader)
    {
        return;
    }

    for (auto& [id, sensor] : m_impl->sensors)
    {
        sensor.xformView = nullptr;
        if (sensor.viewId.empty() || sensor.sensorPrimPath.empty())
        {
            continue;
        }

        const char* sensorPathPtr = sensor.sensorPrimPath.c_str();
        sensor.xformView = m_impl->reader->createXformView(sensor.viewId.c_str(), &sensorPathPtr, 1, "physx");
    }
    m_impl->readerGeneration = m_impl->reader->getGeneration();
}

void ContactSensorImpl::_subscribeToPhysicsEvents()
{
    if (m_impl->physicsEventSub)
    {
        return;
    }

    auto* physicsStageUpdate = carb::getCachedInterface<omni::physics::IPhysicsStageUpdate>();
    if (!physicsStageUpdate)
    {
        return;
    }

    m_impl->physicsEventSub = carb::events::createSubscriptionToPop(
        physicsStageUpdate->getSimulationEventStream().get(),
        [this](carb::events::IEvent* e)
        {
            if (e->type == omni::physics::SimulationEvent::eStopped)
            {
                _clearSensors();
                m_impl->usdStage = nullptr;
                m_impl->usdrtStage = nullptr;
                m_impl->stageId = 0;
                m_impl->stepCount = 0;
                m_impl->lastDt = 0.0f;
            }
            else if (e->type == omni::physics::SimulationEvent::eResumed)
            {
                _initializeFromContext();
            }
        },
        0, "IsaacSim.Sensors.Experimental.Physics.ContactSensor.SimulationEvent");
}

void ContactSensorImpl::_subscribeToPhysicsStepEvents()
{
    if (m_impl->physicsStepSub != omni::physics::kInvalidSubscriptionId)
    {
        return;
    }

    m_impl->physicsSimulation = carb::getCachedInterface<omni::physics::IPhysicsSimulation>();
    if (!m_impl->physicsSimulation)
    {
        return;
    }

    m_impl->physicsStepSub = m_impl->physicsSimulation->subscribePhysicsOnStepEvents(
        false, 1,
        [this](float elapsedTime, const omni::physics::PhysicsStepContext& /*context*/) { _stepSensors(elapsedTime); });
}

void ContactSensorImpl::_unsubscribeFromPhysicsStepEvents()
{
    if (m_impl->physicsSimulation && m_impl->physicsStepSub != omni::physics::kInvalidSubscriptionId)
    {
        m_impl->physicsSimulation->unsubscribePhysicsOnStepEvents(m_impl->physicsStepSub);
        m_impl->physicsStepSub = omni::physics::kInvalidSubscriptionId;
    }
}

void ContactSensorImpl::_pullContactData(float dt)
{
    m_impl->contactStore.rawContacts.clear();
    for (auto& it : m_impl->contactStore.perBodyMap)
    {
        it.second.clear();
    }

    if (!m_impl->reader)
    {
        CARB_LOG_WARN("ContactSensorImpl: no IPrimDataReader available");
        return;
    }

    std::vector<const char*> bodyPaths;
    bodyPaths.reserve(m_impl->sensors.size());
    for (const auto& [id, sensor] : m_impl->sensors)
    {
        (void)id;
        bodyPaths.push_back(sensor.parentRigidBodyPath.c_str());
    }

    if (bodyPaths.empty())
        return;

    isaacsim::core::experimental::prims::ContactReportData report;
    if (!m_impl->reader->getContactReport(bodyPaths.data(), bodyPaths.size(), &report))
        return;

    using isaacsim::core::experimental::prims::kContactEventFound;
    using isaacsim::core::experimental::prims::kContactEventLost;
    using isaacsim::core::experimental::prims::kContactEventPersist;

    float simTime = report.simTime;
    // Use reader-reported dt when available; fall back to caller's physics step dt
    float contactDt = report.dt > 0.0f ? report.dt : dt;

    for (uint32_t ei = 0; ei < report.numEvents; ei++)
    {
        const auto& event = report.events[ei];

        if (event.eventType == kContactEventFound || event.eventType == kContactEventPersist)
        {
            m_impl->contactStore.removeContactPair(event.body0, event.body1);

            for (uint32_t ci = 0; ci < event.numContacts; ci++)
            {
                const auto& cp = event.contacts[ci];
                ContactRawData entry;
                entry.body0 = event.body0;
                entry.body1 = event.body1;
                entry.positionX = cp.positionX;
                entry.positionY = cp.positionY;
                entry.positionZ = cp.positionZ;
                entry.normalX = cp.normalX;
                entry.normalY = cp.normalY;
                entry.normalZ = cp.normalZ;
                entry.impulseX = cp.impulseX;
                entry.impulseY = cp.impulseY;
                entry.impulseZ = cp.impulseZ;
                entry.time = simTime;
                entry.dt = contactDt;
                m_impl->contactStore.rawContacts.push_back(entry);
            }
        }
        else if (event.eventType == kContactEventLost)
        {
            m_impl->contactStore.removeContactPair(event.body0, event.body1);
        }
    }
}

void ContactSensorImpl::_stepSensors(float dt)
{
    m_impl->lastDt = dt;
    m_impl->stepCount++;

    if (!m_impl->simManager || !m_impl->usdStage)
    {
        return;
    }

    if (m_impl->reader && m_impl->reader->getGeneration() != m_impl->readerGeneration)
    {
        _recreateSensorViews();
    }

    _pullContactData(dt);

    if (m_impl->sensors.empty())
    {
        return;
    }

    const double simTime = m_impl->simManager->getSimulationTime();
    for (auto& [id, sensor] : m_impl->sensors)
    {
        (void)sensor;
        _processSensor(*m_impl, id, dt, simTime);
    }
}

void ContactSensorImpl::_processSensor(ImplData& impl, const std::string& primPath, float dt, double simTime)
{
    auto it = impl.sensors.find(primPath);
    if (it == impl.sensors.end())
    {
        return;
    }
    SensorData& sensor = it->second;

    sensor.refreshConfig(impl.usdStage);

    if (sensor.previousEnabled != sensor.enabled)
    {
        if (!sensor.enabled)
        {
            sensor.latestReading = ContactSensorReading();
            sensor.latestRawContacts.clear();
        }
        sensor.previousEnabled = sensor.enabled;
    }

    if (!sensor.enabled)
    {
        return;
    }

    ContactSensorReading reading;
    reading.time = static_cast<float>(simTime);

    const auto& contacts = impl.contactStore.getForBody(sensor.parentToken);

    // Snapshot raw contacts for this sensor so they persist for Python access
    sensor.latestRawContacts.assign(contacts.begin(), contacts.end());

    if (contacts.empty())
    {
        reading.isValid = true;
        sensor.latestReading = reading;
        return;
    }

    float sensorPos[3] = {};
    float sensorOri[4] = {};
    if (sensor.xformView)
    {
        sensor.xformView->getPrimWorldTransform(sensor.sensorPrimPath.c_str(), sensorPos, sensorOri);
    }

    double totalImpulseX = 0.0, totalImpulseY = 0.0, totalImpulseZ = 0.0;
    float contactDt = dt;

    for (const auto& c : contacts)
    {
        if (sensor.radius > 0.0f)
        {
            double dx = sensorPos[0] - c.positionX;
            double dy = sensorPos[1] - c.positionY;
            double dz = sensorPos[2] - c.positionZ;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance >= static_cast<double>(sensor.radius))
            {
                continue;
            }
        }

        double impulseX = static_cast<double>(c.impulseX);
        double impulseY = static_cast<double>(c.impulseY);
        double impulseZ = static_cast<double>(c.impulseZ);

        if (c.body1 == sensor.parentToken)
        {
            impulseX = -impulseX;
            impulseY = -impulseY;
            impulseZ = -impulseZ;
        }

        totalImpulseX += impulseX;
        totalImpulseY += impulseY;
        totalImpulseZ += impulseZ;

        if (c.dt > 0.0f)
        {
            contactDt = c.dt;
        }
    }

    double impulseMagnitude =
        std::sqrt(totalImpulseX * totalImpulseX + totalImpulseY * totalImpulseY + totalImpulseZ * totalImpulseZ);

    if (impulseMagnitude <= 0.0)
    {
        reading.isValid = true;
        sensor.latestReading = reading;
        return;
    }

    if (contactDt <= 0.0f)
    {
        contactDt = dt > 0.0f ? dt : 1.0f / 60.0f;
    }

    float forceValue = static_cast<float>(impulseMagnitude / static_cast<double>(contactDt));

    forceValue = std::min(forceValue, sensor.maxThreshold);
    if (forceValue < sensor.minThreshold)
    {
        reading.isValid = true;
        sensor.latestReading = reading;
        return;
    }

    reading.value = forceValue;
    reading.inContact = true;
    reading.isValid = true;
    sensor.latestReading = reading;
}

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim
