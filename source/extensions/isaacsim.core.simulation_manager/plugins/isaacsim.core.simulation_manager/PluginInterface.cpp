// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/PluginUtils.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/events/EventsUtils.h>

#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/core/simulation_manager/TimeSampleStorage.h>
#include <isaacsim/core/simulation_manager/UsdNoticeListener.h>
#include <omni/ext/IExt.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IToken.h>
#include <omni/fabric/SimStageWithHistory.h>
#include <omni/fabric/core/Type.h>
#include <omni/graph/core/Type.h>
#include <omni/kit/IMinimal.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/usd/UsdContext.h>

#include <RunLoopRunner.h>

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
#include <chrono>
#include <memory>

/**
 * @brief Plugin descriptor for the simulation manager plugin.
 * @details Defines the plugin's name, description, author, hot reload capability, and version.
 */
const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.core.simulation_manager.plugin",
                                                    "Helpful text describing the plugin", "Author",
                                                    carb::PluginHotReload::eEnabled, "dev" };

namespace
{
omni::physics::IPhysics* g_physicsInterface = nullptr;
omni::physics::IPhysicsStageUpdate* g_physicsStageUpdateInterface = nullptr;
omni::physics::IPhysicsSimulation* g_physicsSimulationInterface = nullptr;
omni::physics::SubscriptionId g_physicsOnStepSubscription = omni::physics::kInvalidSubscriptionId;
omni::physics::SubscriptionId g_simulationRegistrySubscription = omni::physics::kInvalidSubscriptionId;
carb::events::ISubscriptionPtr g_physicsEventSubscription;
omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::IRunLoopRunnerImpl* g_runLoopRunnerInterface = nullptr;

omni::fabric::UsdStageId g_stageId;
double g_simulationTime = 0.0;
double g_simulationTimeMonotonic = 0.0;
double g_systemTime = 0.0;
size_t g_numPhysicsSteps = 0;
bool g_simulating = false;
bool g_paused = false;

/// Monotonic time tracking across simulation stop/start cycles and rate changes.
/// g_monotonicTimeAccumulated stores the monotonic time from previous segments
/// (prior simulation runs or prior rate periods). g_monotonicStepCount tracks
/// steps within the current segment. g_lastStepsPerSecond detects rate changes.
double g_monotonicTimeAccumulated = 0.0;
uint64_t g_monotonicStepCount = 0;
uint32_t g_lastStepsPerSecond = 0;

void updateMultiTickExternalSimulationTime()
{
    // Write the current simulation time to the Fabric prim
    // /ExternalSimulationTime (attribute omni:time). Writing here from onPhysicsStep
    // (eUsdContextUpdate, order -10) ensures the multitick rendering codepath
    // reads the up-to-date value at eHydraRendering (order 30),
    // within the same app update.
    auto iSRW = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    if (iSRW && g_stageId.id)
    {
        auto srwId = iSRW->get(g_stageId);
        if (srwId != omni::fabric::kInvalidStageReaderWriterId)
        {
            static const omni::fabric::Path externalTimePrim =
                omni::fabric::Path::createImmortal("/ExternalSimulationTime");
            static const omni::fabric::Token timeAttrToken = omni::fabric::Token::createImmortal("omni:time");

            iSRW->createPrim(srwId, externalTimePrim);
            static constexpr omni::fabric::Type kDoubleType = { omni::fabric::BaseDataType::eDouble, 1, 0,
                                                                omni::fabric::AttributeRole::eNone };
            iSRW->createAttribute(srwId, externalTimePrim, timeAttrToken, omni::fabric::TypeC(kDoubleType));

            auto span = iSRW->getAttributeWr(srwId, externalTimePrim, timeAttrToken);
            if (auto* p = span.getTypedPointer<double>())
            {
                *p = g_simulationTime;
            }
            else
            {
                CARB_LOG_ERROR("Failed to write external simulation time to Fabric");
            }
        }
    }
}

/** @brief Global time storage instance for simulation time data */
std::unique_ptr<isaacsim::core::simulation_manager::TimeSampleStorage> g_timeStorage = nullptr;
}

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

/**
 * @class SimulationManagerImpl
 * @brief Implementation of the ISimulationManager interface.
 * @details
 * Provides functionality for managing simulation-related events and callbacks.
 * Handles USD notices and maintains callback registrations for physics scene additions
 * and deletion events.
 */
class SimulationManagerImpl : public ISimulationManager
{
public:
    /**
     * @brief Constructor for SimulationManagerImpl.
     * @details
     * Initializes the USD notice listener and registers it to handle USD notices.
     */
    SimulationManagerImpl() : m_usdNoticeListener(std::make_unique<UsdNoticeListener>())
    {
        m_usdNoticeListenerKey =
            pxr::TfNotice::Register(pxr::TfCreateWeakPtr(m_usdNoticeListener.get()), &UsdNoticeListener::handle);
        _initializeStageEventSubscription();
    }

    /**
     * @brief Destructor for SimulationManagerImpl.
     * @details
     * Cleans up the USD notice listener.
     */
    ~SimulationManagerImpl()
    {
        m_usdNoticeListener.reset();
        m_stageEventSubscription.reset();
    }

    /**
     * @brief Registers a callback function to be called when an object is deleted.
     * @details
     * The callback will be invoked with the path of the deleted object as a parameter.
     *
     * @param[in] callback Function to be called when an object is deleted.
     * @return Unique identifier for the registered callback.
     */
    int registerDeletionCallback(const std::function<void(std::string)>& callback) override
    {
        _initializeStageEventSubscription();
        int& callbackIter = m_usdNoticeListener->getCallbackIter();
        m_usdNoticeListener->getDeletionCallbacks().emplace(callbackIter, callback);
        callbackIter += 1;
        return callbackIter - 1;
    }

    /**
     * @brief Registers a callback function to be called when a physics scene is added.
     * @details
     * The callback will be invoked with the path of the added physics scene as a parameter.
     *
     * @param[in] callback Function to be called when a physics scene is added.
     * @return Unique identifier for the registered callback.
     */
    int registerPhysicsSceneAdditionCallback(const std::function<void(std::string)>& callback) override
    {
        _initializeStageEventSubscription();
        int& callbackIter = m_usdNoticeListener->getCallbackIter();
        m_usdNoticeListener->getPhysicsSceneAdditionCallbacks().emplace(callbackIter, callback);
        callbackIter += 1;
        return callbackIter - 1;
    }

    /**
     * @brief Deregisters a previously registered callback.
     * @details
     * Removes a callback from either the physics scene addition callbacks or deletion callbacks
     * based on the provided callback ID.
     *
     * @param[in] callbackId The unique identifier of the callback to deregister.
     * @return True if the callback was successfully deregistered, false otherwise.
     */
    bool deregisterCallback(const int& callbackId) override
    {
        std::map<int, std::function<void(const std::string&)>>& physicsSceneCallbacks =
            m_usdNoticeListener->getPhysicsSceneAdditionCallbacks();
        std::map<int, std::function<void(const std::string&)>>& deletiomCallbacks =
            m_usdNoticeListener->getDeletionCallbacks();
        if (physicsSceneCallbacks.count(callbackId) > 0)
        {
            physicsSceneCallbacks.erase(callbackId);
            return true;
        }
        else if (deletiomCallbacks.count(callbackId) > 0)
        {
            deletiomCallbacks.erase(callbackId);
            return true;
        }
        return false;
    }

    /**
     * @brief Gets the current callback iterator value.
     * @details
     * This value is used to generate unique identifiers for callbacks.
     *
     * @return Reference to the current callback iterator.
     */
    int& getCallbackIter() override
    {
        return m_usdNoticeListener->getCallbackIter();
    }

    /**
     * @brief Sets the callback iterator to a specific value.
     * @details
     * Allows manual control over the callback identifier generation.
     *
     * @param[in] val The value to set the callback iterator to.
     */
    void setCallbackIter(int const& val) override
    {
        int& callbackIter = m_usdNoticeListener->getCallbackIter();
        callbackIter = val;
    }

    /**
     * @brief Gets the current simulation time.
     * @details
     * Returns the current simulation time.
     *
     * @return The current simulation time.
     */
    double getSimulationTime() override
    {
        return g_simulationTime;
    }

    /**
     * @brief Gets the current simulation time.
     * @details
     * Returns the current simulation time which does not reset when the simulation is stopped.
     *
     * @return The current simulation time.
     */
    double getSimulationTimeMonotonic() override
    {
        return g_simulationTimeMonotonic;
    }

    /**
     * @brief Gets the current system time.
     * @details
     * Returns the current system time.
     *
     * @return The current system time.
     */
    double getSystemTime() override
    {
        return g_systemTime;
    }

    /**
     * @brief Gets the current frame time from best available source.
     * @details
     * Returns the current frame time using the same priority order as TimeSampleStorage.
     * This is useful for testing to track exact frame times being written to storage.
     *
     * @return Current rational time or kInvalidRationalTime if unavailable.
     */
    omni::fabric::RationalTime getCurrentTime() override
    {
        if (!g_timeStorage)
        {
            CARB_LOG_WARN("getCurrentTime: time storage not initialized");
            return omni::fabric::kInvalidRationalTime;
        }

        return g_timeStorage->getCurrentTime();
    }

    /**
     * @brief Enables or disables the USD notice handler.
     * @details
     * Controls whether USD notices are processed by the notice listener.
     *
     * @param[in] flag True to enable the handler, false to disable it.
     */
    void enableUsdNoticeHandler(bool const& flag) override
    {
        m_usdNoticeListener->enable(flag);
    }

    /**
     * @brief Enables or disables the Fabric USD notice handler for a specific stage.
     * @details
     * Controls whether Fabric USD notices are processed for the specified stage.
     * If enabled, forces a minimal populate of the Fabric USD.
     *
     * @param[in] stageId The ID of the stage to configure.
     * @param[in] flag True to enable the handler, false to disable it.
     */
    void enableFabricUsdNoticeHandler(long stageId, bool const& flag) override
    {
        auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
        auto iStageReadWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        if (iFabricUsd && iStageReadWriter)
        {
            omni::fabric::StageReaderWriterId stageRwId = iStageReadWriter->get(stageId);
            if (stageRwId.id)
            {
                auto fabricId = iStageReadWriter->getFabricId(stageRwId);
                iFabricUsd->setEnableChangeNotifies(fabricId, flag);
                if (flag)
                {
                    CARB_PROFILE_ZONE(0, "[IsaacSim] EnableFabricUsdNoticeHandler::forceMinulaPopulate");
                    iFabricUsd->forceMinimalPopulate(fabricId);
                }
            }
        }
    }

    /**
     * @brief Checks if the Fabric USD notice handler is enabled for a specific stage.
     * @details
     * Determines whether Fabric USD notices are being processed for the specified stage.
     *
     * @param[in] stageId The ID of the stage to check.
     * @return True if the notice handler is enabled, false otherwise.
     */
    bool isFabricUsdNoticeHandlerEnabled(long stageId) override
    {
        auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
        auto iStageReadWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        if (iFabricUsd && iStageReadWriter)
        {
            omni::fabric::StageReaderWriterId stageRwId = iStageReadWriter->get(stageId);
            if (stageRwId.id)
            {
                auto fabricId = iStageReadWriter->getFabricId(stageRwId);
                return iFabricUsd->getEnableChangeNotifies(fabricId);
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Gets the current physics step count.
     * @details
     * Returns the current physics step count.
     *
     * @return The current physics step count.
     */
    size_t getNumPhysicsSteps() override
    {
        return g_numPhysicsSteps;
    }

    /**
     * @brief Gets the current simulation pause state.
     * @details
     * Returns the current simulation pause state.
     *
     * @return The current simulation pause state.
     */
    bool isSimulating() override
    {
        return g_simulating;
    }

    /**
     * @brief Gets the current simulation pause state.
     * @details
     * Returns the current simulation pause state.
     *
     * @return The current simulation pause state.
     */
    bool isPaused() override
    {
        return g_paused;
    }


    /**
     * @brief Resets the simulation manager.
     * @details
     * Calls all registered deletion callbacks with a root path ("/"),
     * clears all registered callbacks, clears the physics scenes list,
     * and resets the callback iterator to 0.
     */
    void reset() override
    {
        std::vector<int> deletionKeys;
        auto deletionCallbacksMap = m_usdNoticeListener->getDeletionCallbacks();
        std::transform(deletionCallbacksMap.begin(), deletionCallbacksMap.end(), std::back_inserter(deletionKeys),
                       [](const auto& p) { return p.first; });
        for (auto const& key : deletionKeys)
        {
            deletionCallbacksMap[key]("/");
        }
        m_usdNoticeListener->getDeletionCallbacks().clear();
        m_usdNoticeListener->getPhysicsSceneAdditionCallbacks().clear();
        m_usdNoticeListener->getPhysicsScenes().clear();
        int& callbackIter = m_usdNoticeListener->getCallbackIter();
        callbackIter = 0;
    }

    /**
     * @brief Removes any tracked physics scenes with invalid prims.
     * @details
     * Iterates through the internally tracked physics scenes and removes any
     * whose underlying USD prim is no longer valid. This handles cases where
     * physics scene prims become invalid without triggering USD notices
     * (e.g., layer removal operations). Also triggers deletion callbacks for
     * any removed physics scenes.
     *
     * @return The paths of physics scenes that were removed.
     */
    std::vector<std::string> cleanupInvalidPhysicsScenes() override
    {
        std::vector<std::string> removedPaths;
        auto& physicsScenes = m_usdNoticeListener->getPhysicsScenes();
        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();

        // Collect paths of invalid physics scenes
        std::vector<pxr::SdfPath> invalidPaths;
        for (const auto& [path, sceneApi] : physicsScenes)
        {
            pxr::UsdPrim prim = stage ? stage->GetPrimAtPath(path) : pxr::UsdPrim();
            if (!prim.IsValid() || !prim.IsActive())
            {
                invalidPaths.push_back(path);
                removedPaths.push_back(path.GetString());
            }
        }

        // Remove invalid entries and trigger deletion callbacks
        for (const auto& path : invalidPaths)
        {
            physicsScenes.erase(path);
            // Trigger deletion callbacks for the removed physics scene
            for (const auto& [key, callback] : m_usdNoticeListener->getDeletionCallbacks())
            {
                callback(path.GetString());
            }
            CARB_LOG_WARN("Removed stale physics scene at '%s' (prim is no longer valid)", path.GetString().c_str());
        }

        return removedPaths;
    }


    double getSimulationTimeAtTime(const omni::fabric::RationalTime& rtime) override
    {
        // If no samples are stored, return the current simulation time
        if (!g_timeStorage || g_timeStorage->getSampleCount() == 0)
        {
            return g_simulationTime;
        }

        auto result = g_timeStorage->getSimulationTimeAt(rtime);
        if (result.has_value())
        {
            return result.value();
        }
        else
        {
            CARB_LOG_INFO("getSimulationTimeAtTime: no data found for time %s, returning current sim time",
                          rtime.toString().c_str());
            return g_simulationTime;
        }
    }


    double getSimulationTimeMonotonicAtTime(const omni::fabric::RationalTime& rtime) override
    {
        // If no samples are stored, return the current simulation time
        if (!g_timeStorage || g_timeStorage->getSampleCount() == 0)
        {
            return g_simulationTimeMonotonic;
        }

        auto result = g_timeStorage->getMonotonicSimulationTimeAt(rtime);
        if (result.has_value())
        {
            return result.value();
        }
        else
        {
            CARB_LOG_WARN("getSimulationTimeMonotonicAtTime: no data found for time %s, returning current sim time",
                          rtime.toString().c_str());
            return g_simulationTimeMonotonic;
        }
    }

    double getSystemTimeAtTime(const omni::fabric::RationalTime& rtime) override
    {
        // If no samples are stored, return the current system time
        if (!g_timeStorage || g_timeStorage->getSampleCount() == 0)
        {
            return g_systemTime;
        }

        auto result = g_timeStorage->getSystemTimeAt(rtime);
        if (result.has_value())
        {
            return result.value();
        }
        else
        {
            CARB_LOG_WARN("getSystemTimeAtTime: no data found for time %s, returning current system time",
                          rtime.toString().c_str());
            return g_systemTime;
        }
    }

    std::vector<TimeSampleStorage::Entry> getAllSamples() override
    {
        if (!g_timeStorage)
        {
            return std::vector<TimeSampleStorage::Entry>();
        }

        return g_timeStorage->getAllSamples();
    }

    size_t getSampleCount() override
    {
        return g_timeStorage ? g_timeStorage->getSampleCount() : 0;
    }

    void logStatistics() override
    {
        if (g_timeStorage)
        {
            g_timeStorage->logStatistics();
        }
    }

    std::optional<std::pair<omni::fabric::RationalTime, omni::fabric::RationalTime>> getSampleRange() override
    {
        if (!g_timeStorage || g_timeStorage->getSampleCount() == 0)
        {
            return std::nullopt;
        }

        return g_timeStorage->getSampleRange();
    }

    size_t getBufferCapacity() override
    {
        return TimeSampleStorage::getBufferCapacity();
    }

private:
    void _initializeStageEventSubscription()
    {
        if (m_stageEventSubscription)
        {
            return;
        }

        auto usdContext = omni::usd::UsdContext::getContext();
        if (!usdContext)
        {
            CARB_LOG_INFO("USD context is not available; stage-opened subscription is deferred");
            return;
        }

        auto stage = usdContext->getStage();
        if (!stage)
        {
            CARB_LOG_INFO("USD stage is not available; stage-opened subscription is deferred");
            return;
        }

        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        static const carb::RStringKey s_kEventName("IsaacSimStageOpenedUsdNoticeListener");
        m_stageEventSubscription = ed->observeEvent(
            s_kEventName, 1000, usdContext->stageEventName(omni::usd::StageEventType::eOpened),
            [this, usdContext](const auto&)
            {
                auto stage = usdContext->getStage();
                PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
                omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(stage).ToLongInt()) };
                omni::fabric::IStageReaderWriter* iStageReaderWriter =
                    carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
                omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
                usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);
                for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsScene")))
                {
                    const omni::fabric::Path path(usdrtPath);
                    const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                    pxr::UsdPrim prim = stage->GetPrimAtPath(primPath);
                    if (m_usdNoticeListener->getPhysicsScenes().count(primPath) == 0)
                    {
                        m_usdNoticeListener->getPhysicsScenes().emplace(primPath, pxr::PhysxSchemaPhysxSceneAPI(prim));
                        for (auto const& [key, AdditionFunc] : m_usdNoticeListener->getPhysicsSceneAdditionCallbacks())
                        {
                            (void)key;
                            AdditionFunc(primPath.GetString());
                        }
                    }
                }
            });
    }

    /**
     * @brief USD notice listener object that handles USD notices.
     */
    std::unique_ptr<UsdNoticeListener> m_usdNoticeListener;

    /**
     * @brief Key for the registered USD notice listener.
     */
    pxr::TfNotice::Key m_usdNoticeListenerKey;

    /**
     * @brief Subscription for stage opened events.
     */
    carb::eventdispatcher::ObserverGuard m_stageEventSubscription;
};

/**
 * @brief Callback function for resume events
 * @details
 * Writes initial time data when simulation resumes.
 *
 * @param[in] currentTime The current time
 * @param[in] userData User data pointer
 */
void onResume(float currentTime, void* userData)
{
    // Only write initial time data if storage exists
    if (g_timeStorage)
    {
        g_timeStorage->storeSample(g_simulationTime, g_simulationTimeMonotonic, g_systemTime);
        CARB_LOG_INFO("onResume: Stored initial time data");
    }
    else
    {
        CARB_LOG_WARN("onResume: Time storage not available");
    }
}

/**
 * @brief Callback function for physics step events
 * @details
 * Updates simulation time values and writes them to time storage on each physics step.
 *
 * @param[in] timeElapsed The elapsed time since the last physics step
 * @param[in] context Physics step context information
 */
void onPhysicsStep(float timeElapsed, const omni::physics::PhysicsStepContext& context)
{
    // Derive simulation time from integer step count and steps-per-second to avoid
    // accumulated floating-point drift from repeated addition of float dt.
    // All physics backends must implement getSimulationStepCount() and
    // getSimulationTimeStepsPerSecond() on IPhysicsSimulation.
    uint32_t stepsPerSecond = g_physicsSimulationInterface->getSimulationTimeStepsPerSecond(
        context.simulationId, g_stageId.id, context.scenePath);
    uint64_t stepCount = g_physicsSimulationInterface->getSimulationStepCount(context.simulationId);

    g_simulationTime = static_cast<double>(stepCount) / static_cast<double>(stepsPerSecond);

    // Monotonic time: accumulate across simulation restarts and rate changes.
    if (stepsPerSecond != g_lastStepsPerSecond && g_lastStepsPerSecond != 0)
    {
        g_monotonicTimeAccumulated = g_simulationTimeMonotonic;
        g_monotonicStepCount = 0;
    }
    g_lastStepsPerSecond = stepsPerSecond;
    g_monotonicStepCount++;
    g_simulationTimeMonotonic =
        g_monotonicTimeAccumulated + static_cast<double>(g_monotonicStepCount) / static_cast<double>(stepsPerSecond);

    g_numPhysicsSteps += 1;
    g_systemTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    g_simulating = true;
    updateMultiTickExternalSimulationTime();

    if (!g_timeStorage)
    {
        CARB_LOG_WARN("onPhysicsStep: time storage not initialized, initializing");
        g_timeStorage = std::make_unique<isaacsim::core::simulation_manager::TimeSampleStorage>(g_stageId);
        return;
    }

    // Write time data to storage
    bool success = g_timeStorage->storeSample(g_simulationTime, g_simulationTimeMonotonic, g_systemTime);
    if (!success)
    {
        CARB_LOG_ERROR("Failed to write time data to storage");
    }
}

/**
 * @brief Callback function for simulation registry events
 * @details
 * Re-subscribes to physics step events when a simulation is registered.
 * This ensures newly registered simulations (like Newton) receive the step callback.
 *
 * @param[in] eventType The type of simulation registry event
 * @param[in] simulationId The ID of the simulation
 * @param[in] simulationName The name of the simulation
 * @param[in] userData User data pointer
 */
void onSimulationRegistryEvent(omni::physics::SimulationRegistryEventType::Enum eventType,
                               omni::physics::SimulationId simulationId,
                               const char* simulationName,
                               void* userData)
{
    if (eventType == omni::physics::SimulationRegistryEventType::eSIMULATION_REGISTERED)
    {
        // Re-subscribe to step events so the newly registered simulation receives the callback
        if (g_physicsOnStepSubscription != omni::physics::kInvalidSubscriptionId)
        {
            g_physicsSimulationInterface->unsubscribePhysicsOnStepEvents(g_physicsOnStepSubscription);
        }
        g_physicsOnStepSubscription = g_physicsSimulationInterface->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep);
    }
}

/**
 * @brief Callback function for stop events
 * @details
 * Clears stored time samples but keeps the storage object alive when simulation stops.
 *
 * @param[in] userData User data pointer
 */
void onStop(void* userData)
{
    // Clear time samples but keep storage object alive
    if (g_timeStorage)
    {
        g_timeStorage->clear();
        CARB_LOG_INFO("onStop: Cleared time samples");
    }

    // Reset simulation state
    g_simulationTime = 0;
    g_numPhysicsSteps = 0;
    // Preserve monotonic time across stop/start; reset segment counter
    g_monotonicTimeAccumulated = g_simulationTimeMonotonic;
    g_monotonicStepCount = 0;
    updateMultiTickExternalSimulationTime();
}

/**
 * @brief Callback function for stage attach events
 * @details
 * Initializes time storage and resets simulation state when a new stage is attached.
 *
 * @param[in] stageId The ID of the stage
 * @param[in] metersPerUnit The meters per unit scale of the stage
 * @param[in] userData User data pointer
 */
void onAttach(long int stageId, double metersPerUnit, void* userData)
{
    // Reset simulation state
    g_simulationTime = 0;
    g_numPhysicsSteps = 0;
    // New stage: reset all monotonic time tracking
    g_simulationTimeMonotonic = 0.0;
    g_monotonicTimeAccumulated = 0.0;
    g_monotonicStepCount = 0;
    g_lastStepsPerSecond = 0;

    // Find the USD stage to validate it exists
    pxr::UsdStageWeakPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    if (!stage)
    {
        CARB_LOG_ERROR("Isaac Core Simulation Manager could not find USD stage");
        return;
    }
    g_stageId.id = stageId;

    updateMultiTickExternalSimulationTime();

    // Initialize time storage for this stage
    g_timeStorage = std::make_unique<isaacsim::core::simulation_manager::TimeSampleStorage>(g_stageId);
}

/**
 * @brief Callback function for stage detach events
 * @details
 * Cleans up time storage when a stage is detached.
 *
 * @param[in] stageId The ID of the stage being detached
 * @param[in] metersPerUnit The meters per unit scale of the stage
 * @param[in] userData User data pointer
 */
void onDetach(void* userData)
{
    // Clean up time storage
    if (g_timeStorage)
    {
        g_timeStorage.reset();
    }

    // Clear stage reference
    g_stageId = omni::fabric::UsdStageId();
}


/**
 * @class Extension
 * @brief Implementation of the IExt interface for the simulation manager extension.
 * @details
 * Provides lifecycle management for the simulation manager extension.
 */
class Extension : public omni::ext::IExt
{
public:
    /**
     * @brief Method called when the extension is loaded/enabled.
     * @details
     * Initializes the extension when it is loaded.
     *
     * @param[in] extId The ID of the extension being loaded.
     */
    void onStartup(const char* extId) override
    {
        g_physicsInterface = carb::getCachedInterface<omni::physics::IPhysics>();
        g_physicsStageUpdateInterface = carb::getCachedInterface<omni::physics::IPhysicsStageUpdate>();
        g_physicsSimulationInterface = carb::getCachedInterface<omni::physics::IPhysicsSimulation>();
        g_runLoopRunnerInterface = carb::getCachedInterface<omni::kit::IRunLoopRunnerImpl>();

        g_physicsOnStepSubscription = g_physicsSimulationInterface->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep);

        // Subscribe to simulation registry events to re-subscribe when new simulations are registered
        g_simulationRegistrySubscription =
            g_physicsInterface->subscribeSimulationRegistryEvents(onSimulationRegistryEvent, nullptr);

        g_systemTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        g_simulationTime = 0;
        g_simulationTimeMonotonic = 0;
        g_numPhysicsSteps = 0;
        g_monotonicTimeAccumulated = 0.0;
        g_monotonicStepCount = 0;
        g_lastStepsPerSecond = 0;

        // Set the initial simulation time to zero
        updateMultiTickExternalSimulationTime();

        g_physicsEventSubscription = carb::events::createSubscriptionToPop(
            g_physicsStageUpdateInterface->getSimulationEventStream().get(),
            [](carb::events::IEvent* e)
            {
                switch (e->type)
                {
                case omni::physics::SimulationEvent::eStopped:
                    g_simulating = false;
                    g_paused = false;
                    break;
                case omni::physics::SimulationEvent::ePaused:
                    g_paused = true;
                    break;
                case omni::physics::SimulationEvent::eResumed:
                    g_simulating = true;
                    g_paused = false;
                    break;
                default:
                    break;
                }
            },
            0, "IsaacSim.Core.SimulationManager.SimulationEvent");

        g_stageUpdate = carb::getCachedInterface<omni::kit::IStageUpdate>()->getStageUpdate();

        omni::kit::StageUpdateNodeDesc desc = { nullptr };
        desc.displayName = "Isaac Simulation Manager";
        desc.onStop = onStop;
        desc.onAttach = onAttach;
        desc.onDetach = onDetach;
        desc.onResume = onResume;
        g_stageUpdate->createStageUpdateNode(desc);

        g_runLoopRunnerInterface = carb::getCachedInterface<omni::kit::IRunLoopRunnerImpl>();
    }

    /**
     * @brief Method called when the extension is disabled.
     * @details
     * Cleans up the extension when it is disabled.
     */
    void onShutdown() override
    {
        // Unsubscribe from simulation registry events
        if (g_simulationRegistrySubscription != omni::physics::kInvalidSubscriptionId)
        {
            g_physicsInterface->unsubscribeSimulationRegistryEvents(g_simulationRegistrySubscription);
            g_simulationRegistrySubscription = omni::physics::kInvalidSubscriptionId;
        }

        // Unsubscribe from physics step events
        if (g_physicsOnStepSubscription != omni::physics::kInvalidSubscriptionId)
        {
            g_physicsSimulationInterface->unsubscribePhysicsOnStepEvents(g_physicsOnStepSubscription);
            g_physicsOnStepSubscription = omni::physics::kInvalidSubscriptionId;
        }
    }
};

} // namespace isaacsim
} // namespace core
} // namespace simulation_manager

/**
 * @brief Optional function called the first time an interface is acquired from the plugin library.
 * @details
 * This function is invoked by the Carbonite framework when the plugin is first loaded.
 */
CARB_EXPORT void carbOnPluginStartup()
{
}

/**
 * @brief Optional function called right before the OS releases the plugin library.
 * @details
 * This function is invoked by the Carbonite framework when the plugin is about to be unloaded.
 */
CARB_EXPORT void carbOnPluginShutdown()
{
}

/**
 * @brief Implements the plugin with the specified simulation manager and extension implementations.
 * @details
 * Registers the plugin with the Carbonite framework.
 */
CARB_PLUGIN_IMPL(g_kPluginDesc,
                 isaacsim::core::simulation_manager::SimulationManagerImpl,
                 isaacsim::core::simulation_manager::Extension)

/**
 * @brief Fills the interface for the simulation manager implementation.
 * @details
 * This function is called by the Carbonite framework to initialize the interface.
 *
 * @param[in,out] iface The interface to fill.
 */
void fillInterface(isaacsim::core::simulation_manager::SimulationManagerImpl& iface)
{
}

/**
 * @brief Fills the interface for the extension implementation.
 * @details
 * This function is called by the Carbonite framework to initialize the interface.
 *
 * @param[in,out] iface The interface to fill.
 */
void fillInterface(isaacsim::core::simulation_manager::Extension& iface)
{
}
