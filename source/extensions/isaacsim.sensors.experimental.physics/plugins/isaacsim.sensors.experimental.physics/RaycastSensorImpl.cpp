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

#include "RaycastSensorImpl.h"

#include "SensorImplUtils.h"

#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/simulation/IPhysicsSceneQuery.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#if defined(_WIN32)
#    include <usdrt/scenegraph/usd/rt/xformable.h>
#    include <usdrt/scenegraph/usd/usd/stage.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    include <usdrt/scenegraph/usd/rt/xformable.h>
#    include <usdrt/scenegraph/usd/usd/stage.h>
#    pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cmath>
#include <cstring>
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
using core::experimental::prims::IRigidBodyDataView;
using core::simulation_manager::ISimulationManager;

constexpr float kDirectionNormEpsilon = 1e-8f;
constexpr float kTimeOffsetEpsilon = 1e-8f;
constexpr double kVelocityEpsilon = 1e-10;
constexpr double kQuaternionNormEpsilon = 1e-10;

inline void writeVec3(float* buf, size_t i, float x, float y, float z)
{
    buf[i * 3 + 0] = x;
    buf[i * 3 + 1] = y;
    buf[i * 3 + 2] = z;
}

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
        bool hasRigidBodyAPI = prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();
        pxr::UsdAttribute attr = prim.GetAttribute(pxr::TfToken("physics:rigidBodyEnabled"));
        if (attr.IsValid())
        {
            bool enabled = false;
            attr.Get(&enabled);
            if (enabled)
            {
                return prim.GetPath().GetString();
            }
        }
        else if (hasRigidBodyAPI)
        {
            return prim.GetPath().GetString();
        }

        prim = prim.GetParent();
    }
    return {};
}

class SensorData
{
public:
    std::string sensorPrimPath;
    std::string parentRigidBodyPath;
    std::string viewId;
    IRigidBodyDataView* rigidBodyView = nullptr;

    // Cached schema attributes (read once on first step, frozen for run)
    uint32_t numRays = 0;
    std::vector<pxr::GfVec3f> rayOrigins;
    std::vector<pxr::GfVec3f> rayDirections; // normalized
    std::vector<float> rayTimeOffsets;
    float minRange = 0.4f;
    float maxRange = 100.0f;
    bool enabled = true;
    bool previousEnabled = true;
    bool sensorFrame = true; // outputFrameOfReference == "SENSOR"
    bool reportHitPrimPaths = false;
    float sweepPeriod = 0.0f;
    uint64_t sweepStepCount = 0;
    bool configCached = false;
    bool configError = false;

    // Per-ray output buffers
    std::vector<float> depths;
    std::vector<float> hitPositions; // flat [x,y,z, x,y,z, ...]
    std::vector<float> hitNormals; // flat [x,y,z, ...]
    std::vector<float> rayOriginsWorld; // flat [x,y,z, ...], always world frame
    std::vector<float> rayEndPointsWorld; // flat [x,y,z, ...], always world frame
    std::vector<std::string> hitPrimPathStrings;
    std::vector<const char*> hitPrimPathPtrs;
    float readingTime = 0.0f;
    bool readingValid = false;

    void cacheConfig(pxr::UsdStageRefPtr stage)
    {
        using namespace isaacsim::robot::schema::sensors;

        pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(sensorPrimPath));
        if (!prim.IsValid())
        {
            configError = true;
            return;
        }

        // enabled, numRays, minRange, maxRange
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kEnabledAttr), enabled);
        unsigned int numRaysAttr = 1;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kNumRaysAttr), numRaysAttr);
        numRays = numRaysAttr;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kMinRangeAttr), minRange);
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kMaxRangeAttr), maxRange);

        if (numRays == 0)
        {
            CARB_LOG_ERROR(
                "IsaacRaycastSensor '%s': numRays (%u) must be > 0. Sensor disabled.", sensorPrimPath.c_str(), numRays);
            configError = true;
            return;
        }

        if (minRange >= maxRange)
        {
            CARB_LOG_ERROR("IsaacRaycastSensor '%s': minRange (%f) >= maxRange (%f). Sensor disabled.",
                           sensorPrimPath.c_str(), minRange, maxRange);
            configError = true;
            return;
        }

        // rayOrigins
        pxr::VtArray<pxr::GfVec3f> originsVt;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kRayOriginsAttr), originsVt);

        if (originsVt.size() != numRays)
        {
            CARB_LOG_ERROR("IsaacRaycastSensor '%s': rayOrigins length (%zu) != numRays (%u). Sensor disabled.",
                           sensorPrimPath.c_str(), originsVt.size(), numRays);
            configError = true;
            return;
        }
        rayOrigins.assign(originsVt.begin(), originsVt.end());

        // rayDirections
        pxr::VtArray<pxr::GfVec3f> directionsVt;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kRayDirectionsAttr), directionsVt);

        if (directionsVt.size() != numRays)
        {
            CARB_LOG_ERROR("IsaacRaycastSensor '%s': rayDirections length (%zu) != numRays (%u). Sensor disabled.",
                           sensorPrimPath.c_str(), directionsVt.size(), numRays);
            configError = true;
            return;
        }

        // Normalize directions
        rayDirections.resize(directionsVt.size());
        for (size_t i = 0; i < directionsVt.size(); i++)
        {
            pxr::GfVec3f d = directionsVt[i];
            float len = d.GetLength();
            if (len > kDirectionNormEpsilon)
            {
                rayDirections[i] = d / len;
            }
            else
            {
                rayDirections[i] = pxr::GfVec3f(1.0f, 0.0f, 0.0f);
            }
        }

        // rayTimeOffsets (optional; when non-empty must have numRays elements)
        pxr::VtArray<float> offsetsVt;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kRayTimeOffsetsAttr), offsetsVt);
        if (!offsetsVt.empty() && offsetsVt.size() != numRays)
        {
            CARB_LOG_ERROR("IsaacRaycastSensor '%s': rayTimeOffsets length (%zu) != numRays (%u). Sensor disabled.",
                           sensorPrimPath.c_str(), offsetsVt.size(), numRays);
            configError = true;
            return;
        }
        rayTimeOffsets.assign(offsetsVt.begin(), offsetsVt.end());

        // outputFrameOfReference
        pxr::TfToken frameToken;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kOutputFrameOfReferenceAttr), frameToken);
        sensorFrame = (frameToken != kOutputFrameWorld);

        // reportHitPrimPaths
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kReportHitPrimPathsAttr), reportHitPrimPaths);

        // Rotation period is max(rayTimeOffsets); cycle repeats after the last ray fires.
        if (!rayTimeOffsets.empty())
        {
            sweepPeriod = *std::max_element(rayTimeOffsets.begin(), rayTimeOffsets.end());
        }

        // Allocate output buffers
        size_t n = numRays;
        depths.resize(n, maxRange);
        hitPositions.resize(n * 3, 0.0f);
        hitNormals.resize(n * 3, 0.0f);
        rayOriginsWorld.resize(n * 3, 0.0f);
        rayEndPointsWorld.resize(n * 3, 0.0f);
        if (reportHitPrimPaths)
        {
            hitPrimPathStrings.resize(n);
            hitPrimPathPtrs.resize(n, nullptr);
        }

        configCached = true;
    }

    void clearReading()
    {
        readingValid = false;
        readingTime = 0.0f;
        std::fill(depths.begin(), depths.end(), maxRange);
        std::fill(hitPositions.begin(), hitPositions.end(), 0.0f);
        std::fill(hitNormals.begin(), hitNormals.end(), 0.0f);
        std::fill(rayOriginsWorld.begin(), rayOriginsWorld.end(), 0.0f);
        std::fill(rayEndPointsWorld.begin(), rayEndPointsWorld.end(), 0.0f);
        for (auto& s : hitPrimPathStrings)
        {
            s.clear();
        }
        for (auto& p : hitPrimPathPtrs)
        {
            p = nullptr;
        }
    }
};

} // namespace

struct RaycastSensorImpl::ImplData
{
    long stageId = 0;
    std::string engineType = "physx";
    float lastDt = 0.0f;
    int stepCount = 0;
    uint64_t readerGeneration = 0;

    ISimulationManager* simManager = nullptr;
    IPrimDataReaderManager* readerManager = nullptr;
    IPrimDataReader* reader = nullptr;
    omni::physics::IPhysicsSceneQuery* sceneQuery = nullptr;
    omni::physics::IPhysicsSimulation* physicsSimulation = nullptr;
    omni::physics::SubscriptionId physicsStepSub = omni::physics::kInvalidSubscriptionId;
    carb::events::ISubscriptionPtr physicsEventSub;

    pxr::UsdStageRefPtr usdStage;
    usdrt::UsdStageRefPtr usdrtStage;
    std::unordered_map<std::string, SensorData> sensors;
};

RaycastSensorImpl::RaycastSensorImpl() : m_impl(std::make_unique<ImplData>())
{
    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    m_impl->reader = m_impl->readerManager ? m_impl->readerManager->getReader() : nullptr;
    _subscribeToPhysicsEvents();
}

RaycastSensorImpl::~RaycastSensorImpl()
{
    shutdown();
}

void RaycastSensorImpl::shutdown()
{
    _unsubscribeFromPhysicsStepEvents();
    m_impl->physicsEventSub.reset();
    _clearSensors();
    m_impl->readerManager = nullptr;
    m_impl->reader = nullptr;
    m_impl->simManager = nullptr;
    m_impl->sceneQuery = nullptr;
    m_impl->physicsSimulation = nullptr;
    m_impl->usdStage = nullptr;
    m_impl->usdrtStage = nullptr;
    m_impl->stageId = 0;
    m_impl->stepCount = 0;
    m_impl->lastDt = 0.0f;
    m_impl->readerGeneration = 0;
}

void RaycastSensorImpl::_initializeFromContext()
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

    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (settings)
    {
        const char* engineSetting = settings->getStringBuffer("/exts/isaacsim.core.simulation_manager/default_engine");
        if (engineSetting && engineSetting[0] != '\0')
        {
            m_impl->engineType = engineSetting;
        }
    }

    _initializeStage(stageId);
    _discoverSensorsFromStage();
}

void RaycastSensorImpl::_initializeStage(long stageId)
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

    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    if (m_impl->readerManager)
    {
        if (!m_impl->readerManager->ensureInitialized(stageId, -1))
        {
            return;
        }
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

    _subscribeToPhysicsStepEvents();
}

bool RaycastSensorImpl::createSensor(const char* primPath)
{
    if (!m_impl->usdStage || !m_impl->reader)
    {
        return false;
    }

    std::string key(primPath);
    pxr::SdfPath sdfPath(primPath);
    pxr::UsdPrim prim = m_impl->usdStage->GetPrimAtPath(sdfPath);

    auto existing = m_impl->sensors.find(key);
    if (existing != m_impl->sensors.end())
    {
        // Tear down the cached entry when the prim has been deleted, or when
        // its parent rigid body has changed (delete/recreate at the same path
        // can land under a different rigid body). Otherwise reuse the view
        // and re-read config so attribute updates on a recreated prim are
        // picked up.
        if (prim.IsValid())
        {
            std::string currentParent = findParentRigidBody(m_impl->usdStage, sdfPath);
            if (currentParent == existing->second.parentRigidBodyPath)
            {
                // Re-read attributes so a delete-and-recreate-at-same-path
                // picks up the new prim's config.
                existing->second.configCached = false;
                existing->second.configError = false;
                existing->second.cacheConfig(m_impl->usdStage);
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

    std::string parentPath = findParentRigidBody(m_impl->usdStage, sdfPath);

    SensorData& sensor = m_impl->sensors[key];
    sensor.sensorPrimPath = primPath;
    sensor.parentRigidBodyPath = parentPath;

    if (!parentPath.empty())
    {
        sensor.viewId = "raycast_rb_" + key;
        const char* pathStr = parentPath.c_str();
        sensor.rigidBodyView =
            m_impl->reader->createRigidBodyView(sensor.viewId.c_str(), &pathStr, 1, m_impl->engineType.c_str());
        if (!sensor.rigidBodyView)
        {
            m_impl->sensors.erase(key);
            return false;
        }
    }

    if (m_impl->reader)
    {
        m_impl->readerGeneration = m_impl->reader->getGeneration();
    }

    return true;
}

void RaycastSensorImpl::removeSensor(const char* primPath)
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

RaycastSensorReading RaycastSensorImpl::getSensorReading(const char* primPath)
{
    std::string key(primPath);
    auto it = m_impl->sensors.find(key);
    if (it == m_impl->sensors.end())
    {
        return RaycastSensorReading();
    }

    // Tear down the cached sensor when the underlying USD prim has been removed
    // since the last update. Without this we'd return the last cached reading
    // for a deleted prim, and a subsequent createSensor() would early-out on
    // the stale map entry without rebuilding the rigid-body view.
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
            return RaycastSensorReading();
        }
    }

    SensorData& sensor = it->second;
    if (!sensor.readingValid || sensor.configError)
    {
        return RaycastSensorReading();
    }

    RaycastSensorReading reading;
    reading.rayCount = sensor.numRays;
    reading.depths = sensor.depths.data();
    reading.hitPositions = sensor.hitPositions.data();
    reading.hitNormals = sensor.hitNormals.data();
    reading.rayOriginsWorld = sensor.rayOriginsWorld.data();
    reading.rayEndPointsWorld = sensor.rayEndPointsWorld.data();
    if (sensor.reportHitPrimPaths && !sensor.hitPrimPathPtrs.empty())
    {
        reading.hitPrimPaths = sensor.hitPrimPathPtrs.data();
    }
    reading.time = sensor.readingTime;
    reading.isValid = true;
    return reading;
}

void RaycastSensorImpl::_discoverSensorsFromStage()
{
    if (!m_impl->usdStage || !m_impl->reader)
    {
        return;
    }

    for (auto prim : m_impl->usdStage->Traverse())
    {
        if (prim.GetTypeName() == isaacsim::robot::schema::sensors::kIsaacRaycastSensorType)
        {
            (void)createSensor(prim.GetPath().GetString().c_str());
        }
    }
}

void RaycastSensorImpl::_clearSensors()
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
}

void RaycastSensorImpl::_subscribeToPhysicsEvents()
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
                m_impl->sceneQuery = nullptr;
                m_impl->stageId = 0;
                m_impl->stepCount = 0;
                m_impl->lastDt = 0.0f;
            }
            else if (e->type == omni::physics::SimulationEvent::eResumed)
            {
                _initializeFromContext();
            }
        },
        0, "IsaacSim.Sensors.Experimental.Physics.RaycastSensor.SimulationEvent");
}

void RaycastSensorImpl::_subscribeToPhysicsStepEvents()
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

void RaycastSensorImpl::_unsubscribeFromPhysicsStepEvents()
{
    if (m_impl->physicsSimulation && m_impl->physicsStepSub != omni::physics::kInvalidSubscriptionId)
    {
        m_impl->physicsSimulation->unsubscribePhysicsOnStepEvents(m_impl->physicsStepSub);
        m_impl->physicsStepSub = omni::physics::kInvalidSubscriptionId;
    }
}

void RaycastSensorImpl::_stepSensors(float dt)
{
    m_impl->lastDt = dt;
    m_impl->stepCount++;

    if (!m_impl->simManager || !m_impl->usdStage || m_impl->sensors.empty())
    {
        return;
    }

    if (!m_impl->sceneQuery)
    {
        m_impl->sceneQuery = carb::getCachedInterface<omni::physics::IPhysicsSceneQuery>();
    }

    const double simTime = m_impl->simManager->getSimulationTime();
    std::vector<std::string> sensorIds;
    sensorIds.reserve(m_impl->sensors.size());
    for (const auto& [id, sensor] : m_impl->sensors)
    {
        (void)sensor;
        sensorIds.push_back(id);
    }
    for (const auto& id : sensorIds)
    {
        _processSensor(*m_impl, id, dt, simTime);
    }
}

void RaycastSensorImpl::_processSensor(ImplData& impl, const std::string& primPath, float dt, double simTime)
{
    auto it = impl.sensors.find(primPath);
    if (it == impl.sensors.end())
    {
        return;
    }
    SensorData& sensor = it->second;

    // Cache config once on first step (frozen for duration of run)
    if (!sensor.configCached && !sensor.configError && impl.usdStage)
    {
        sensor.cacheConfig(impl.usdStage);
    }

    if (sensor.configError)
    {
        sensor.readingValid = false;
        return;
    }

    if (sensor.previousEnabled != sensor.enabled)
    {
        if (!sensor.enabled)
        {
            sensor.clearReading();
        }
        sensor.previousEnabled = sensor.enabled;
    }

    if (!sensor.enabled || sensor.rayOrigins.empty())
    {
        sensor.readingValid = false;
        return;
    }

    // Get sensor world transform
    float sensorPos[3] = {};
    float sensorOri[4] = {}; // [qw, qx, qy, qz]
    bool hasTransform = false;
    if (sensor.rigidBodyView)
    {
        hasTransform = sensor.rigidBodyView->getPrimWorldTransform(sensor.sensorPrimPath.c_str(), sensorPos, sensorOri);
    }
    if (!hasTransform)
    {
        // Fallback: try to read transform from USD directly
        pxr::UsdPrim prim = impl.usdStage->GetPrimAtPath(pxr::SdfPath(sensor.sensorPrimPath));
        if (prim.IsValid())
        {
            pxr::UsdGeomXformable xformable(prim);
            pxr::GfMatrix4d worldXform = xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            pxr::GfVec3d translation = worldXform.ExtractTranslation();
            sensorPos[0] = static_cast<float>(translation[0]);
            sensorPos[1] = static_cast<float>(translation[1]);
            sensorPos[2] = static_cast<float>(translation[2]);
            pxr::GfRotation rotation = worldXform.ExtractRotation();
            pxr::GfQuatd quat = rotation.GetQuat();
            sensorOri[0] = static_cast<float>(quat.GetReal());
            pxr::GfVec3d imag = quat.GetImaginary();
            sensorOri[1] = static_cast<float>(imag[0]);
            sensorOri[2] = static_cast<float>(imag[1]);
            sensorOri[3] = static_cast<float>(imag[2]);
            hasTransform = true;
        }
    }
    if (!hasTransform)
    {
        sensor.readingValid = false;
        return;
    }

    // Build rotation matrix from sensor quaternion
    usdrt::GfMatrix4d sensorToWorld(1.0);
    sensorToWorld.SetRotate(usdrt::GfQuatd(static_cast<double>(sensorOri[0]), static_cast<double>(sensorOri[1]),
                                           static_cast<double>(sensorOri[2]), static_cast<double>(sensorOri[3])));
    usdrt::GfMatrix4d worldToSensor = sensorToWorld.GetInverse();

    omni::math::linalg::vec3d sensorPosW(sensorPos[0], sensorPos[1], sensorPos[2]);

    // Get rigid body velocity for time offset extrapolation
    omni::math::linalg::vec3d linVelW(0.0, 0.0, 0.0);
    omni::math::linalg::vec3d angVelW(0.0, 0.0, 0.0);
    if (sensor.rigidBodyView && !sensor.rayTimeOffsets.empty())
    {
        int linearCount = 0, angularCount = 0;
        const float* linearVelocityPointer = sensor.rigidBodyView->getLinearVelocitiesHost(&linearCount);
        const float* angularVelocityPointer = sensor.rigidBodyView->getAngularVelocitiesHost(&angularCount);
        if (linearVelocityPointer && linearCount >= 3)
        {
            linVelW.Set(linearVelocityPointer[0], linearVelocityPointer[1], linearVelocityPointer[2]);
        }
        if (angularVelocityPointer && angularCount >= 3)
        {
            angVelW.Set(angularVelocityPointer[0], angularVelocityPointer[1], angularVelocityPointer[2]);
        }
    }

    if (!impl.sceneQuery)
    {
        sensor.readingValid = false;
        return;
    }

    size_t rayCount = sensor.numRays;
    float maxDepth = sensor.maxRange - sensor.minRange;

    float windowStart = 0.0f;
    float windowEnd = 0.0f;
    bool windowWrapped = false;
    if (sensor.sweepPeriod > 0.0f)
    {
        sensor.sweepStepCount++;
        float sweepTime = std::fmod(static_cast<float>(sensor.sweepStepCount) * dt, sensor.sweepPeriod);
        windowEnd = sweepTime;
        windowStart = windowEnd - dt;
        if (windowStart < 0.0f)
        {
            windowStart += sensor.sweepPeriod;
            windowWrapped = true;
        }
    }

    for (size_t i = 0; i < rayCount; i++)
    {
        float timeOffset = 0.0f;
        if (i < sensor.rayTimeOffsets.size())
        {
            timeOffset = sensor.rayTimeOffsets[i];
        }

        if (sensor.sweepPeriod > 0.0f)
        {
            bool active = windowWrapped ? (timeOffset >= windowStart || timeOffset < windowEnd) :
                                          (timeOffset >= windowStart && timeOffset < windowEnd);
            if (!active)
            {
                sensor.depths[i] = sensor.maxRange;
                writeVec3(sensor.hitPositions.data(), i, 0.0f, 0.0f, 0.0f);
                writeVec3(sensor.hitNormals.data(), i, 0.0f, 0.0f, 0.0f);
                writeVec3(sensor.rayOriginsWorld.data(), i, sensorPos[0], sensorPos[1], sensorPos[2]);
                writeVec3(sensor.rayEndPointsWorld.data(), i, sensorPos[0], sensorPos[1], sensorPos[2]);
                if (sensor.reportHitPrimPaths)
                {
                    sensor.hitPrimPathStrings[i].clear();
                    sensor.hitPrimPathPtrs[i] = sensor.hitPrimPathStrings[i].c_str();
                }
                continue;
            }
        }

        omni::math::linalg::vec3d rayPosW = sensorPosW;
        usdrt::GfMatrix4d rayS2W = sensorToWorld;
        usdrt::GfMatrix4d rayW2S = worldToSensor;

        if (std::abs(timeOffset) > kTimeOffsetEpsilon &&
            (linVelW.GetLength() > kVelocityEpsilon || angVelW.GetLength() > kVelocityEpsilon))
        {
            double dtOffset = static_cast<double>(timeOffset);
            rayPosW = sensorPosW + linVelW * dtOffset;

            double wx = angVelW[0] * dtOffset * 0.5;
            double wy = angVelW[1] * dtOffset * 0.5;
            double wz = angVelW[2] * dtOffset * 0.5;
            double qw = sensorOri[0], qx = sensorOri[1], qy = sensorOri[2], qz = sensorOri[3];
            double nw = qw - wx * qx - wy * qy - wz * qz;
            double nx = qx + wx * qw + wy * qz - wz * qy;
            double ny = qy - wx * qz + wy * qw + wz * qx;
            double nz = qz + wx * qy - wy * qx + wz * qw;
            double norm = std::sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
            if (norm > kQuaternionNormEpsilon)
            {
                nw /= norm;
                nx /= norm;
                ny /= norm;
                nz /= norm;
            }
            rayS2W = usdrt::GfMatrix4d(1.0);
            rayS2W.SetRotate(usdrt::GfQuatd(nw, nx, ny, nz));
            rayW2S = rayS2W.GetInverse();
        }

        const pxr::GfVec3f& localOrigin = sensor.rayOrigins[i];
        const pxr::GfVec3f& localDir = sensor.rayDirections[i];

        omni::math::linalg::vec3d worldOrigin =
            rayPosW + rayS2W.TransformDir(omni::math::linalg::vec3d(localOrigin[0], localOrigin[1], localOrigin[2]));
        omni::math::linalg::vec3d worldDir =
            rayS2W.TransformDir(omni::math::linalg::vec3d(localDir[0], localDir[1], localDir[2]));

        double dirLen = worldDir.GetLength();
        if (dirLen > kVelocityEpsilon)
        {
            worldDir = worldDir / dirLen;
        }

        writeVec3(sensor.rayOriginsWorld.data(), i, static_cast<float>(worldOrigin[0]),
                  static_cast<float>(worldOrigin[1]), static_cast<float>(worldOrigin[2]));

        omni::math::linalg::vec3d rayStart = worldOrigin + worldDir * static_cast<double>(sensor.minRange);

        carb::Float3 origin{ static_cast<float>(rayStart[0]), static_cast<float>(rayStart[1]),
                             static_cast<float>(rayStart[2]) };
        carb::Float3 unitDir{ static_cast<float>(worldDir[0]), static_cast<float>(worldDir[1]),
                              static_cast<float>(worldDir[2]) };

        omni::physics::RaycastHit hit{};
        bool hitFound = impl.sceneQuery->raycastClosest(origin, unitDir, maxDepth, hit, false);

        if (hitFound)
        {
            sensor.depths[i] = hit.distance + sensor.minRange;

            writeVec3(sensor.rayEndPointsWorld.data(), i, hit.position.x, hit.position.y, hit.position.z);

            omni::math::linalg::vec3d hitPos(hit.position.x, hit.position.y, hit.position.z);
            omni::math::linalg::vec3d hitNormal(hit.normal.x, hit.normal.y, hit.normal.z);

            if (sensor.sensorFrame)
            {
                auto diff = hitPos - rayPosW;
                omni::math::linalg::vec3d localHitPos =
                    rayW2S.TransformDir(omni::math::linalg::vec3d(diff[0], diff[1], diff[2]));
                omni::math::linalg::vec3d localHitNormal = rayW2S.TransformDir(hitNormal);
                writeVec3(sensor.hitPositions.data(), i, static_cast<float>(localHitPos[0]),
                          static_cast<float>(localHitPos[1]), static_cast<float>(localHitPos[2]));
                writeVec3(sensor.hitNormals.data(), i, static_cast<float>(localHitNormal[0]),
                          static_cast<float>(localHitNormal[1]), static_cast<float>(localHitNormal[2]));
            }
            else
            {
                writeVec3(sensor.hitPositions.data(), i, hit.position.x, hit.position.y, hit.position.z);
                writeVec3(sensor.hitNormals.data(), i, hit.normal.x, hit.normal.y, hit.normal.z);
            }

            if (sensor.reportHitPrimPaths && hit.rigidBody != 0)
            {
                pxr::SdfPath hitPath = isaacsim::core::includes::getSdfPathFromUint64(hit.rigidBody);
                sensor.hitPrimPathStrings[i] = hitPath.GetString();
                sensor.hitPrimPathPtrs[i] = sensor.hitPrimPathStrings[i].c_str();
            }
            else if (sensor.reportHitPrimPaths)
            {
                sensor.hitPrimPathStrings[i].clear();
                sensor.hitPrimPathPtrs[i] = sensor.hitPrimPathStrings[i].c_str();
            }
        }
        else
        {
            sensor.depths[i] = sensor.maxRange;
            writeVec3(sensor.rayEndPointsWorld.data(), i,
                      static_cast<float>(worldOrigin[0] + worldDir[0] * static_cast<double>(sensor.maxRange)),
                      static_cast<float>(worldOrigin[1] + worldDir[1] * static_cast<double>(sensor.maxRange)),
                      static_cast<float>(worldOrigin[2] + worldDir[2] * static_cast<double>(sensor.maxRange)));
            writeVec3(sensor.hitPositions.data(), i, 0.0f, 0.0f, 0.0f);
            writeVec3(sensor.hitNormals.data(), i, 0.0f, 0.0f, 0.0f);
            if (sensor.reportHitPrimPaths)
            {
                sensor.hitPrimPathStrings[i].clear();
                sensor.hitPrimPathPtrs[i] = sensor.hitPrimPathStrings[i].c_str();
            }
        }
    }

    sensor.readingTime = static_cast<float>(simTime);
    sensor.readingValid = true;
}

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim
