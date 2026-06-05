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

#include "ImuSensorImpl.h"

#include <carb/events/EventsUtils.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/usd/UsdContext.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
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

static omni::math::linalg::vec3d readGravityFromStage(pxr::UsdStageRefPtr stage, pxr::SdfPath& cachedScenePath)
{
    double unitScale = UsdGeomGetStageMetersPerUnit(stage);
    if (!std::isfinite(unitScale) || unitScale <= 0.0)
    {
        unitScale = 1.0;
    }

    omni::math::linalg::vec3d dir(0.0, 0.0, -1.0);
    double mag = 9.80665;

    pxr::UsdPrim scenePrim;
    if (!cachedScenePath.IsEmpty())
    {
        scenePrim = stage->GetPrimAtPath(cachedScenePath);
    }

    if (!scenePrim.IsValid() || !scenePrim.IsA<pxr::UsdPhysicsScene>())
    {
        cachedScenePath = pxr::SdfPath();
        for (auto prim : stage->Traverse())
        {
            if (prim.IsA<pxr::UsdPhysicsScene>())
            {
                scenePrim = prim;
                cachedScenePath = prim.GetPath();
                break;
            }
        }
    }

    if (scenePrim.IsValid())
    {
        pxr::UsdPhysicsScene scene(scenePrim);
        float magAttr = 0.0f;
        isaacsim::core::includes::safeGetAttribute(scene.GetGravityMagnitudeAttr(), magAttr);
        if (std::isfinite(magAttr) && magAttr != 0.0f)
        {
            mag = static_cast<double>(std::abs(magAttr));
        }

        pxr::GfVec3f dirAttr(0.0f, 0.0f, 0.0f);
        isaacsim::core::includes::safeGetAttribute(scene.GetGravityDirectionAttr(), dirAttr);
        double dirLen = std::sqrt(dirAttr[0] * dirAttr[0] + dirAttr[1] * dirAttr[1] + dirAttr[2] * dirAttr[2]);
        if (std::isfinite(dirLen) && dirLen > 1e-10)
        {
            dir.Set(static_cast<double>(dirAttr[0]), static_cast<double>(dirAttr[1]), static_cast<double>(dirAttr[2]));
        }
    }

    return mag / unitScale * -dir;
}

class SensorData
{
public:
    std::string sensorPrimPath;
    std::string parentRigidBodyPath;
    std::string viewId;
    IRigidBodyDataView* rigidBodyView = nullptr;

    int rawBufferSize = 20;
    int rawBufferHead = 0;
    int readingsHead = 0;
    std::vector<ImuRawData> rawBuffer;
    std::vector<ImuSensorReading> readings;

    int linearAccelerationFilterSize = 1;
    int angularVelocityFilterSize = 1;
    int orientationFilterSize = 1;

    omni::math::linalg::vec3d gravity{ 0.0, 0.0, 0.0 };
    omni::math::linalg::vec3d gravitySensorFrame{ 0.0, 0.0, 0.0 };

    double timeSeconds = 0.0;
    double timeDelta = 0.0;
    bool enabled = true;
    bool previousEnabled = true;
    int64_t lastProcessedStep = -1;
    bool configLockedForRun = false;

    ImuRawData& rawAt(int i)
    {
        return rawBuffer[(rawBufferHead - i + rawBufferSize) % rawBufferSize];
    }
    ImuSensorReading& readingAt(int i)
    {
        return readings[(readingsHead - i + rawBufferSize) % rawBufferSize];
    }

    void pushRaw()
    {
        rawBufferHead = (rawBufferHead + 1) % rawBufferSize;
        rawBuffer[rawBufferHead] = ImuRawData();
    }

    void pushReading()
    {
        readingsHead = (readingsHead + 1) % rawBufferSize;
        readings[readingsHead] = ImuSensorReading();
    }

    void resetBuffers()
    {
        rawBuffer.assign(rawBufferSize, ImuRawData());
        readings.assign(rawBufferSize, ImuSensorReading());
        rawBufferHead = rawBufferSize - 1;
        readingsHead = rawBufferSize - 1;
    }

    void refreshEnabled(pxr::UsdStageRefPtr stage)
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
            if (enabledAttr.Get(&val))
            {
                enabled = val;
            }
        }
    }

    void refreshConfig(pxr::UsdStageRefPtr stage, pxr::SdfPath& cachedScenePath)
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

        int linearFilter = 1, angularFilter = 1, orientationFilter = 1;
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kLinearAccelerationFilterWidthAttr), linearFilter);
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kAngularVelocityFilterWidthAttr), angularFilter);
        isaacsim::core::includes::safeGetAttribute(prim.GetAttribute(kOrientationFilterWidthAttr), orientationFilter);

        linearAccelerationFilterSize = std::max(linearFilter, 1);
        angularVelocityFilterSize = std::max(angularFilter, 1);
        orientationFilterSize = std::max(orientationFilter, 1);

        int maxRolling = std::max({ linearAccelerationFilterSize, angularVelocityFilterSize, orientationFilterSize });
        int desiredSize = std::max(2 * maxRolling, 20);
        if (desiredSize != rawBufferSize)
        {
            rawBufferSize = desiredSize;
            resetBuffers();
        }

        gravity = readGravityFromStage(stage, cachedScenePath);
    }
};

} // namespace

struct ImuSensorImpl::ImplData
{
    long stageId = 0;
    std::string engineType = "physx";
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
    pxr::SdfPath cachedPhysicsScenePath;
    std::unordered_map<std::string, SensorData> sensors;
};

ImuSensorImpl::ImuSensorImpl() : m_impl(std::make_unique<ImplData>())
{
    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    m_impl->reader = m_impl->readerManager ? m_impl->readerManager->getReader() : nullptr;
    _subscribeToPhysicsEvents();
}

ImuSensorImpl::~ImuSensorImpl()
{
    shutdown();
}

void ImuSensorImpl::shutdown()
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
}

void ImuSensorImpl::_initializeFromContext()
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

void ImuSensorImpl::_initializeStage(long stageId)
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

bool ImuSensorImpl::createSensor(const char* primPath)
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
        // the prim's parent rigid body has changed (delete/recreate at the
        // same path can land under a different rigid body). Otherwise reuse
        // the view and refresh config so attribute updates on a recreated
        // prim are picked up.
        if (prim.IsValid())
        {
            std::string currentParent = findParentRigidBody(m_impl->usdStage, sdfPath);
            if (currentParent == existing->second.parentRigidBodyPath)
            {
                existing->second.refreshConfig(m_impl->usdStage, m_impl->cachedPhysicsScenePath);
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
    if (parentPath.empty())
    {
        return false;
    }

    SensorData& sensor = m_impl->sensors[key];
    sensor.sensorPrimPath = primPath;
    sensor.parentRigidBodyPath = parentPath;
    sensor.viewId = "imu_rb_" + key;

    const char* pathStr = parentPath.c_str();
    sensor.rigidBodyView =
        m_impl->reader->createRigidBodyView(sensor.viewId.c_str(), &pathStr, 1, m_impl->engineType.c_str());
    if (!sensor.rigidBodyView)
    {
        m_impl->sensors.erase(key);
        return false;
    }

    m_impl->readerGeneration = m_impl->reader->getGeneration();
    sensor.refreshConfig(m_impl->usdStage, m_impl->cachedPhysicsScenePath);
    sensor.resetBuffers();
    return true;
}

void ImuSensorImpl::removeSensor(const char* primPath)
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

ImuSensorReading ImuSensorImpl::getSensorReading(const char* primPath, bool readGravity)
{
    std::string key(primPath);
    auto it = m_impl->sensors.find(key);
    if (it == m_impl->sensors.end())
    {
        return ImuSensorReading();
    }

    // Tear down the cached sensor when the underlying USD prim has been removed
    // since the last update. Without this, the sensor would keep returning the
    // last valid reading after the prim is deleted, AND a subsequent
    // createSensor() call at the same path would early-out on the stale map
    // entry without rebuilding the rigid-body view.
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
            return ImuSensorReading();
        }
    }

    if (m_impl->readerManager && m_impl->stageId != 0)
    {
        if (m_impl->readerManager->ensureInitialized(m_impl->stageId, -1))
        {
            m_impl->reader = m_impl->readerManager->getReader();
        }
    }

    if (m_impl->reader && m_impl->reader->getGeneration() != m_impl->readerGeneration)
    {
        _recreateSensorViews();
    }

    SensorData& sensor = it->second;
    if (sensor.enabled && !sensor.readings.empty() && !sensor.readingAt(0).isValid && sensor.rigidBodyView &&
        m_impl->simManager && m_impl->usdStage && m_impl->lastDt > 0.0f)
    {
        double simTime = m_impl->simManager->getSimulationTime();
        _processSensor(*m_impl, key, m_impl->lastDt, simTime, m_impl->stepCount);
    }

    if (!sensor.enabled || sensor.readings.empty() || !sensor.readingAt(0).isValid)
    {
        return ImuSensorReading();
    }

    ImuSensorReading reading = sensor.readingAt(0);
    if (readGravity)
    {
        reading.linearAccelerationX += static_cast<float>(sensor.gravitySensorFrame[0]);
        reading.linearAccelerationY += static_cast<float>(sensor.gravitySensorFrame[1]);
        reading.linearAccelerationZ += static_cast<float>(sensor.gravitySensorFrame[2]);
    }

    _sanitizeReading(reading);
    reading.isValid = true;
    return reading;
}

void ImuSensorImpl::_discoverSensorsFromStage()
{
    if (!m_impl->usdStage || !m_impl->reader)
    {
        return;
    }

    for (auto prim : m_impl->usdStage->Traverse())
    {
        if (prim.GetTypeName() == "IsaacImuSensor")
        {
            (void)createSensor(prim.GetPath().GetString().c_str());
        }
    }
}

void ImuSensorImpl::_clearSensors()
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
    m_impl->cachedPhysicsScenePath = pxr::SdfPath();
}

void ImuSensorImpl::_recreateSensorViews()
{
    if (!m_impl->reader)
    {
        return;
    }

    for (auto& [id, sensor] : m_impl->sensors)
    {
        sensor.rigidBodyView = nullptr;
        if (sensor.viewId.empty() || sensor.parentRigidBodyPath.empty())
        {
            continue;
        }

        const char* pathStr = sensor.parentRigidBodyPath.c_str();
        sensor.rigidBodyView =
            m_impl->reader->createRigidBodyView(sensor.viewId.c_str(), &pathStr, 1, m_impl->engineType.c_str());
    }
    m_impl->readerGeneration = m_impl->reader->getGeneration();
}

void ImuSensorImpl::_subscribeToPhysicsEvents()
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
        0, "IsaacSim.Sensors.Experimental.Physics.SimulationEvent");
}

void ImuSensorImpl::_subscribeToPhysicsStepEvents()
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

void ImuSensorImpl::_unsubscribeFromPhysicsStepEvents()
{
    if (m_impl->physicsSimulation && m_impl->physicsStepSub != omni::physics::kInvalidSubscriptionId)
    {
        m_impl->physicsSimulation->unsubscribePhysicsOnStepEvents(m_impl->physicsStepSub);
        m_impl->physicsStepSub = omni::physics::kInvalidSubscriptionId;
    }
}

void ImuSensorImpl::_stepSensors(float dt)
{
    m_impl->lastDt = dt;
    m_impl->stepCount++;

    if (!m_impl->simManager || !m_impl->usdStage || m_impl->sensors.empty())
    {
        return;
    }

    if (m_impl->readerManager && m_impl->stageId != 0)
    {
        if (!m_impl->readerManager->ensureInitialized(m_impl->stageId, -1))
        {
            return;
        }
        m_impl->reader = m_impl->readerManager->getReader();
    }

    if (m_impl->reader && m_impl->reader->getGeneration() != m_impl->readerGeneration)
    {
        _recreateSensorViews();
    }

    const double simTime = m_impl->simManager->getSimulationTime();
    for (auto& [id, sensor] : m_impl->sensors)
    {
        (void)sensor;
        _processSensor(*m_impl, id, dt, simTime, m_impl->stepCount);
    }
}

void ImuSensorImpl::_processSensor(ImplData& impl, const std::string& primPath, float dt, double simTime, int64_t stepIndex)
{
    auto it = impl.sensors.find(primPath);
    if (it == impl.sensors.end())
    {
        return;
    }
    SensorData& sensor = it->second;

    if (sensor.lastProcessedStep == stepIndex)
    {
        return;
    }

    sensor.timeDelta = dt;
    sensor.timeSeconds = simTime;

    // Freeze USD-driven config updates after the first processed sim step.
    // On stop/play, sensors are recreated so config can be re-read at startup.
    if (impl.usdStage && !sensor.configLockedForRun)
    {
        sensor.refreshConfig(impl.usdStage, impl.cachedPhysicsScenePath);
        sensor.configLockedForRun = true;
    }

    if (sensor.previousEnabled != sensor.enabled)
    {
        if (sensor.enabled)
        {
            sensor.resetBuffers();
        }
        sensor.previousEnabled = sensor.enabled;
    }

    if (!sensor.enabled || !sensor.rigidBodyView)
    {
        sensor.lastProcessedStep = stepIndex;
        return;
    }

    sensor.rigidBodyView->update();

    int linearCount = 0, angularCount = 0;
    const float* linearVelocityPointer = sensor.rigidBodyView->getLinearVelocitiesHost(&linearCount);
    const float* angularVelocityPointer = sensor.rigidBodyView->getAngularVelocitiesHost(&angularCount);

    omni::math::linalg::vec3d vW(0.0, 0.0, 0.0);
    omni::math::linalg::vec3d wW(0.0, 0.0, 0.0);
    if (linearVelocityPointer && linearCount >= 3)
    {
        vW.Set(linearVelocityPointer[0], linearVelocityPointer[1], linearVelocityPointer[2]);
    }
    if (angularVelocityPointer && angularCount >= 3)
    {
        wW.Set(angularVelocityPointer[0], angularVelocityPointer[1], angularVelocityPointer[2]);
    }

    float sensorPos[3] = {};
    float sensorOri[4] = {}; // [qw, qx, qy, qz]
    if (!sensor.rigidBodyView->getPrimWorldTransform(sensor.sensorPrimPath.c_str(), sensorPos, sensorOri))
    {
        sensor.lastProcessedStep = stepIndex;
        return;
    }
    // Build rotation matrix from quaternion (already orthonormal, no scale needed).
    usdrt::GfMatrix4d rBw(1.0);
    rBw.SetRotate(usdrt::GfQuatd(static_cast<double>(sensorOri[0]), static_cast<double>(sensorOri[1]),
                                 static_cast<double>(sensorOri[2]), static_cast<double>(sensorOri[3])));
    usdrt::GfMatrix4d rWb = rBw.GetInverse();
    usdrt::GfMatrix3d rotMatrix = rBw.ExtractRotationMatrix();
    omni::math::linalg::quatd qWb = rotMatrix.ExtractRotation();
    const omni::math::linalg::vec3d imaginary = qWb.GetImaginary();

    omni::math::linalg::vec3d vB = rWb.TransformDir(vW);
    omni::math::linalg::vec3d wB = rWb.TransformDir(wW);
    sensor.gravitySensorFrame = rWb.TransformDir(sensor.gravity);

    sensor.pushRaw();
    ImuRawData& raw = sensor.rawAt(0);
    raw.time = static_cast<float>(sensor.timeSeconds);
    raw.dt = static_cast<float>(sensor.timeDelta);
    raw.linearVelocityX = static_cast<float>(vB[0]);
    raw.linearVelocityY = static_cast<float>(vB[1]);
    raw.linearVelocityZ = static_cast<float>(vB[2]);
    raw.angularVelocityX = static_cast<float>(wB[0]);
    raw.angularVelocityY = static_cast<float>(wB[1]);
    raw.angularVelocityZ = static_cast<float>(wB[2]);
    raw.orientationW = static_cast<float>(qWb.GetReal());
    raw.orientationX = static_cast<float>(imaginary[0]);
    raw.orientationY = static_cast<float>(imaginary[1]);
    raw.orientationZ = static_cast<float>(imaginary[2]);

    sensor.pushReading();
    ImuSensorReading& reading = sensor.readingAt(0);
    reading.time = static_cast<float>(sensor.timeSeconds);
    reading.isValid = true;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (int i = 0; i < sensor.angularVelocityFilterSize; i++)
    {
        sumX += sensor.rawAt(i).angularVelocityX;
        sumY += sensor.rawAt(i).angularVelocityY;
        sumZ += sensor.rawAt(i).angularVelocityZ;
    }
    reading.angularVelocityX = sumX / sensor.angularVelocityFilterSize;
    reading.angularVelocityY = sumY / sensor.angularVelocityFilterSize;
    reading.angularVelocityZ = sumZ / sensor.angularVelocityFilterSize;

    sumX = 0.0f;
    sumY = 0.0f;
    sumZ = 0.0f;
    for (int i = 0; i < sensor.linearAccelerationFilterSize; i++)
    {
        float timeDiff = sensor.rawAt(i).time - sensor.rawAt(i + sensor.linearAccelerationFilterSize).time;
        if (timeDiff > 1e-10f)
        {
            sumX += (sensor.rawAt(i).linearVelocityX -
                     sensor.rawAt(i + sensor.linearAccelerationFilterSize).linearVelocityX) /
                    timeDiff;
            sumY += (sensor.rawAt(i).linearVelocityY -
                     sensor.rawAt(i + sensor.linearAccelerationFilterSize).linearVelocityY) /
                    timeDiff;
            sumZ += (sensor.rawAt(i).linearVelocityZ -
                     sensor.rawAt(i + sensor.linearAccelerationFilterSize).linearVelocityZ) /
                    timeDiff;
        }
    }
    reading.linearAccelerationX = sumX / sensor.linearAccelerationFilterSize;
    reading.linearAccelerationY = sumY / sensor.linearAccelerationFilterSize;
    reading.linearAccelerationZ = sumZ / sensor.linearAccelerationFilterSize;

    float sumW = 0.0f;
    sumX = 0.0f;
    sumY = 0.0f;
    sumZ = 0.0f;
    for (int i = 0; i < sensor.orientationFilterSize; i++)
    {
        sumW += sensor.rawAt(i).orientationW;
        sumX += sensor.rawAt(i).orientationX;
        sumY += sensor.rawAt(i).orientationY;
        sumZ += sensor.rawAt(i).orientationZ;
    }
    float averageW = sumW / sensor.orientationFilterSize;
    float averageX = sumX / sensor.orientationFilterSize;
    float averageY = sumY / sensor.orientationFilterSize;
    float averageZ = sumZ / sensor.orientationFilterSize;

    float norm = std::sqrt(averageW * averageW + averageX * averageX + averageY * averageY + averageZ * averageZ);
    if (norm > 0.0f && std::isfinite(norm))
    {
        reading.orientationW = averageW / norm;
        reading.orientationX = averageX / norm;
        reading.orientationY = averageY / norm;
        reading.orientationZ = averageZ / norm;
    }
    else
    {
        reading.orientationW = raw.orientationW;
        reading.orientationX = raw.orientationX;
        reading.orientationY = raw.orientationY;
        reading.orientationZ = raw.orientationZ;
    }

    sensor.lastProcessedStep = stepIndex;
}

void ImuSensorImpl::_sanitizeReading(ImuSensorReading& r)
{
    if (!std::isfinite(r.linearAccelerationX))
    {
        r.linearAccelerationX = 0.0f;
    }
    if (!std::isfinite(r.linearAccelerationY))
    {
        r.linearAccelerationY = 0.0f;
    }
    if (!std::isfinite(r.linearAccelerationZ))
    {
        r.linearAccelerationZ = 0.0f;
    }
    if (!std::isfinite(r.angularVelocityX))
    {
        r.angularVelocityX = 0.0f;
    }
    if (!std::isfinite(r.angularVelocityY))
    {
        r.angularVelocityY = 0.0f;
    }
    if (!std::isfinite(r.angularVelocityZ))
    {
        r.angularVelocityZ = 0.0f;
    }

    float norm = std::sqrt(r.orientationW * r.orientationW + r.orientationX * r.orientationX +
                           r.orientationY * r.orientationY + r.orientationZ * r.orientationZ);
    if (norm > 0.0f && std::isfinite(norm))
    {
        r.orientationW /= norm;
        r.orientationX /= norm;
        r.orientationY /= norm;
        r.orientationZ /= norm;
    }
    else
    {
        r.orientationW = 1.0f;
        r.orientationX = 0.0f;
        r.orientationY = 0.0f;
        r.orientationZ = 0.0f;
    }
}

} // namespace physics
} // namespace experimental
} // namespace sensors
} // namespace isaacsim
