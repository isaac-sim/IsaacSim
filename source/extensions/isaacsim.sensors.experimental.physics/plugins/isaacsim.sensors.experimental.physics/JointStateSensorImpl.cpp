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

#include "JointStateSensorImpl.h"

#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/usd/UsdContext.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim::sensors::experimental::physics
{

namespace
{

using core::experimental::prims::IArticulationDataView;
using core::experimental::prims::IPrimDataReader;
using core::experimental::prims::IPrimDataReaderManager;
using core::simulation_manager::ISimulationManager;

// Per-sensor data: one articulation view and cached DOF state updated each physics step.
struct JointStateSensorData
{
    std::string articulationRootPath;
    std::string viewId;
    IArticulationDataView* articulationView = nullptr;

    // DOF metadata (indexed by articulation DOF index)
    std::vector<std::string> dofNames;
    std::vector<const char*> dofNamePtrs; // C-string pointers into dofNames; valid until next update
    std::vector<uint8_t> dofTypes; // 0 = rotation (revolute), 1 = translation (prismatic)

    // Latest sensor values (updated each physics step)
    std::vector<float> positions;
    std::vector<float> velocities;
    std::vector<float> efforts;

    JointStateSensorReading latestReading;
    bool enabled = true;
};

} // namespace

// PIMPL: shared stage/reader, subscriptions, and the map of prim path -> JointStateSensorData.
struct JointStateSensorImpl::ImplData
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
    std::unordered_map<std::string, JointStateSensorData> sensors;
};

JointStateSensorImpl::JointStateSensorImpl() : m_impl(std::make_unique<ImplData>())
{
    m_impl->simManager = carb::getCachedInterface<ISimulationManager>();
    m_impl->readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    m_impl->reader = m_impl->readerManager ? m_impl->readerManager->getReader() : nullptr;
    _subscribeToPhysicsEvents(); // Listen for eResumed/eStopped to init/clear
}

JointStateSensorImpl::~JointStateSensorImpl()
{
    shutdown();
}

void JointStateSensorImpl::shutdown()
{
    _unsubscribeFromPhysicsStepEvents();
    m_impl->physicsEventSub.reset();
    _clearSensors();
    m_impl->readerManager = nullptr;
    m_impl->reader = nullptr;
    m_impl->simManager = nullptr;
    m_impl->physicsSimulation = nullptr;
    m_impl->usdStage = nullptr;
    m_impl->stageId = 0;
    m_impl->stepCount = 0;
    m_impl->lastDt = 0.0f;
    m_impl->readerGeneration = 0;
}

void JointStateSensorImpl::_initializeFromContext()
{
    const omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
    if (!usdContext)
    {
        return;
    }
    const pxr::UsdStageRefPtr stage = usdContext->getStage();
    if (!stage)
    {
        return;
    }

    const pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    const long stageId = cache.GetId(stage).ToLongInt();
    if (stageId == 0)
    {
        return;
    }

    const carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (settings)
    {
        const char* engineSetting = settings->getStringBuffer("/exts/isaacsim.core.simulation_manager/default_engine");
        if (engineSetting && engineSetting[0] != '\0')
        {
            m_impl->engineType = engineSetting;
        }
    }

    _initializeStage(stageId);
}

void JointStateSensorImpl::_initializeStage(const long stageId)
{
    if (m_impl->stageId == stageId && m_impl->usdStage && m_impl->readerManager && m_impl->reader)
    {
        return;
    }

    // Stage changed: clear existing sensors before re-binding.
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

    const pxr::UsdStageCache& cache = pxr::UsdUtilsStageCache::Get();
    m_impl->usdStage = cache.Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

    _subscribeToPhysicsStepEvents();
}

bool JointStateSensorImpl::createSensor(const char* articulationRootPath)
{
    if (!articulationRootPath || articulationRootPath[0] == '\0')
    {
        return false;
    }
    if (!m_impl->usdStage || !m_impl->reader)
    {
        return false;
    }

    std::string key(articulationRootPath);
    const pxr::SdfPath sdfPath(articulationRootPath);
    const pxr::UsdPrim rootPrim = m_impl->usdStage->GetPrimAtPath(sdfPath);

    auto existing = m_impl->sensors.find(key);
    if (existing != m_impl->sensors.end())
    {
        // Reuse the cached entry only when the articulation root prim is still
        // valid; otherwise tear down and rebuild so a delete/recreate at the
        // same path picks up the new DOF metadata.
        if (rootPrim.IsValid())
        {
            return true;
        }
        if (m_impl->reader && !existing->second.viewId.empty())
        {
            m_impl->reader->removeView(existing->second.viewId.c_str());
        }
        m_impl->sensors.erase(existing);
    }

    if (!rootPrim.IsValid())
    {
        return false;
    }

    JointStateSensorData& sensor = m_impl->sensors[key];
    sensor.articulationRootPath = articulationRootPath;
    sensor.viewId = "joint_state_art_" + key;

    const char* pathStr = articulationRootPath;
    sensor.articulationView =
        m_impl->reader->createArticulationView(sensor.viewId.c_str(), &pathStr, 1, m_impl->engineType.c_str());
    if (!sensor.articulationView)
    {
        m_impl->sensors.erase(key);
        return false;
    }

    int nameCount = 0;
    const char* const* names = sensor.articulationView->getDofNames(&nameCount);
    int typeCount = 0;
    const uint8_t* types = sensor.articulationView->getDofTypes(&typeCount);

    if (!names || nameCount == 0)
    {
        CARB_LOG_WARN("JointStateSensor: no DOFs found under articulation root '%s'", articulationRootPath);
        m_impl->reader->removeView(sensor.viewId.c_str());
        m_impl->sensors.erase(key);
        return false;
    }

    const int dofCount = nameCount;
    sensor.dofNames.clear();
    sensor.dofNames.reserve(dofCount);
    for (int i = 0; i < dofCount; ++i)
    {
        sensor.dofNames.push_back(names[i] ? names[i] : std::string());
    }
    sensor.dofTypes.assign(types, types + (typeCount >= dofCount ? dofCount : typeCount));
    if (static_cast<int>(sensor.dofTypes.size()) < dofCount)
    {
        sensor.dofTypes.resize(dofCount, 0);
    }

    sensor.dofNamePtrs.resize(dofCount);
    for (int i = 0; i < dofCount; i++)
    {
        sensor.dofNamePtrs[i] = sensor.dofNames[i].c_str();
    }

    sensor.positions.resize(dofCount, 0.0f);
    sensor.velocities.resize(dofCount, 0.0f);
    sensor.efforts.resize(dofCount, 0.0f);

    m_impl->readerGeneration = m_impl->reader->getGeneration();
    return true;
}

void JointStateSensorImpl::removeSensor(const char* articulationRootPath)
{
    const auto it = m_impl->sensors.find(std::string(articulationRootPath));
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

JointStateSensorReading JointStateSensorImpl::getSensorReading(const char* articulationRootPath)
{
    std::string key(articulationRootPath);
    const auto it = m_impl->sensors.find(key);
    if (it == m_impl->sensors.end())
    {
        return JointStateSensorReading();
    }

    if (m_impl->reader && m_impl->reader->getGeneration() != m_impl->readerGeneration)
    {
        _recreateSensorViews();
    }

    JointStateSensorData& sensor = it->second;

    if (sensor.enabled && !sensor.latestReading.isValid && sensor.articulationView && m_impl->simManager &&
        m_impl->usdStage && m_impl->lastDt > 0.0f)
    {
        double simTime = m_impl->simManager->getSimulationTime();
        _processSensor(*m_impl, key, simTime);
    }

    if (!sensor.enabled || !sensor.latestReading.isValid)
    {
        return JointStateSensorReading();
    }

    // Fill reading with pointers into sensor-owned vectors (valid until next step or removal).
    JointStateSensorReading reading;
    reading.time = sensor.latestReading.time;
    reading.isValid = true;
    reading.dofCount = static_cast<int32_t>(sensor.dofNames.size());
    reading.dofNames = sensor.dofNamePtrs.empty() ? nullptr : sensor.dofNamePtrs.data();
    reading.positions = sensor.positions.empty() ? nullptr : sensor.positions.data();
    reading.velocities = sensor.velocities.empty() ? nullptr : sensor.velocities.data();
    reading.efforts = sensor.efforts.empty() ? nullptr : sensor.efforts.data();
    reading.dofTypes = sensor.dofTypes.empty() ? nullptr : sensor.dofTypes.data();
    reading.stageMetersPerUnit = 0.0f;
    if (m_impl->usdStage)
    {
        const double metersPerUnit = UsdGeomGetStageMetersPerUnit(m_impl->usdStage);
        reading.stageMetersPerUnit =
            static_cast<float>(std::isfinite(metersPerUnit) && metersPerUnit > 0.0 ? metersPerUnit : 1.0);
    }
    return reading;
}

void JointStateSensorImpl::_clearSensors()
{
    for (const auto& [id, sensor] : m_impl->sensors)
    {
        (void)id;
        if (m_impl->reader && !sensor.viewId.empty())
        {
            m_impl->reader->removeView(sensor.viewId.c_str());
        }
    }
    m_impl->sensors.clear();
}

void JointStateSensorImpl::_recreateSensorViews()
{
    if (!m_impl->reader)
    {
        return;
    }

    for (auto& [id, sensor] : m_impl->sensors)
    {
        sensor.articulationView = nullptr;
        if (sensor.viewId.empty() || sensor.articulationRootPath.empty())
        {
            continue;
        }

        const char* pathStr = sensor.articulationRootPath.c_str();
        sensor.articulationView =
            m_impl->reader->createArticulationView(sensor.viewId.c_str(), &pathStr, 1, m_impl->engineType.c_str());
        // DOF names/count remain unchanged across view recreation
    }
    m_impl->readerGeneration = m_impl->reader->getGeneration();
}

void JointStateSensorImpl::_subscribeToPhysicsEvents()
{
    if (m_impl->physicsEventSub)
    {
        return;
    }

    const omni::physics::IPhysicsStageUpdate* physicsStageUpdate =
        carb::getCachedInterface<omni::physics::IPhysicsStageUpdate>();
    if (!physicsStageUpdate)
    {
        return;
    }

    // On eStopped: clear state; on eResumed: reinit stage/reader and step subscription.
    m_impl->physicsEventSub = carb::events::createSubscriptionToPop(
        physicsStageUpdate->getSimulationEventStream().get(),
        [this](carb::events::IEvent* e)
        {
            if (e->type == omni::physics::SimulationEvent::eStopped)
            {
                _clearSensors();
                m_impl->usdStage = nullptr;
                m_impl->stageId = 0;
                m_impl->stepCount = 0;
                m_impl->lastDt = 0.0f;
            }
            else if (e->type == omni::physics::SimulationEvent::eResumed)
            {
                _initializeFromContext();
            }
        },
        0, "IsaacSim.Sensors.Experimental.Physics.JointStateSensor.SimulationEvent");
}

void JointStateSensorImpl::_subscribeToPhysicsStepEvents()
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

void JointStateSensorImpl::_unsubscribeFromPhysicsStepEvents()
{
    if (m_impl->physicsSimulation && m_impl->physicsStepSub != omni::physics::kInvalidSubscriptionId)
    {
        m_impl->physicsSimulation->unsubscribePhysicsOnStepEvents(m_impl->physicsStepSub);
        m_impl->physicsStepSub = omni::physics::kInvalidSubscriptionId;
    }
}

void JointStateSensorImpl::_stepSensors(const float dt)
{
    m_impl->lastDt = dt;
    m_impl->stepCount++;

    if (!m_impl->simManager || !m_impl->usdStage || m_impl->sensors.empty())
    {
        return;
    }

    if (m_impl->reader && m_impl->reader->getGeneration() != m_impl->readerGeneration)
    {
        _recreateSensorViews();
    }

    const double simTime = m_impl->simManager->getSimulationTime();
    for (const auto& [id, sensor] : m_impl->sensors)
    {
        (void)sensor;
        _processSensor(*m_impl, id, simTime);
    }
}

void JointStateSensorImpl::_processSensor(ImplData& impl, const std::string& articulationRootPath, const double simTime)
{
    const auto it = impl.sensors.find(articulationRootPath);
    if (it == impl.sensors.end())
    {
        return;
    }
    JointStateSensorData& sensor = it->second;

    if (!sensor.enabled || !sensor.articulationView)
    {
        return;
    }

    int posCount = 0;
    const float* const positions = sensor.articulationView->getDofPositionsHost(&posCount);
    if (!positions || posCount == 0)
    {
        sensor.latestReading.isValid = false;
        return;
    }

    int velCount = 0;
    const float* const velocities = sensor.articulationView->getDofVelocitiesHost(&velCount);

    int effCount = 0;
    const float* const efforts = sensor.articulationView->getDofEffortsHost(&effCount);

    const int dofCount = static_cast<int>(sensor.dofNames.size());
    const int n = std::min(posCount, dofCount);
    if (n <= 0)
    {
        sensor.latestReading.isValid = false;
        return;
    }

    // Keep arrays sized to dofCount so getSensorReading() can safely expose dofCount elements
    // (bindings copy r.dofCount elements; shrinking would cause buffer over-read).
    sensor.positions.resize(dofCount, 0.0f);
    std::copy(positions, positions + n, sensor.positions.begin());

    if (velocities && velCount >= n)
    {
        sensor.velocities.resize(dofCount, 0.0f);
        std::copy(velocities, velocities + n, sensor.velocities.begin());
    }
    else
    {
        sensor.velocities.assign(dofCount, 0.0f);
    }
    if (efforts && effCount >= n)
    {
        sensor.efforts.resize(dofCount, 0.0f);
        std::copy(efforts, efforts + n, sensor.efforts.begin());
    }
    else
    {
        sensor.efforts.assign(dofCount, 0.0f);
    }

    sensor.latestReading.time = static_cast<float>(simTime);
    sensor.latestReading.isValid = true;
}

} // namespace isaacsim::sensors::experimental::physics
