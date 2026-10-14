// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.hpp>
#include <omni/usd/UtilsIncludes.h>

#include "isaacsim/robot/surface_gripper/SurfaceGripperComponent.hpp"
#include "isaacsim/robot/surface_gripper/SurfaceGripperManager.hpp"
// Threading utilities
#include "isaacsim/robot/surface_gripper/ThreadUtils.hpp"
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
// <carb/settings/ISettings.h> include removed: settings interface not used

#include <isaacsim/robot/surface_gripper/ISurfaceGripper.hpp>
#include <omni/fabric/usd/PathConversion.h>
#include <omni/graph/core/ogn/Registration.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/usd/UsdContext.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.robot.surface_gripper.plugin", "Surface Gripper",
                                                    "NVIDIA", carb::PluginHotReload::eDisabled, "dev" };


CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::robot::surface_gripper::SurfaceGripperInterface)

CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::physx::IPhysxSceneQuery,
                      omni::physx::IPhysxSimulation,
                      omni::kit::IStageUpdate,
                      omni::graph::core::IGraphRegistry)

DECLARE_OGN_NODES()


// private stuff
namespace
{
omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
omni::physx::IPhysx* g_physx = nullptr;
omni::physx::IPhysxSimulation* g_physxSimulation = nullptr;
std::thread::id g_stageUpdateThread;
pxr::UsdStageWeakPtr g_stage = nullptr;
omni::physics::tensors::TensorApi* g_tensorApi = nullptr;
omni::physics::tensors::ISimulationView* g_simulationView = nullptr;
std::unique_ptr<isaacsim::robot::surface_gripper::SurfaceGripperManager> g_surfaceGripperManager;
omni::physx::SubscriptionId g_stepSubscription;
omni::physx::SubscriptionId g_postStepBoundarySubscription;
omni::physx::SubscriptionId g_simulationSubscription;
bool g_firstFrame = true;
long int g_stageID;

// One guard per public operation, not per worker: distinct grippers stay parallel.
std::recursive_mutex g_operationMutex;
std::condition_variable_any g_simulationComplete;
bool g_simulationRunning = false;
uint64_t g_simulationGeneration = 1;
thread_local uint64_t g_postStepGeneration = 0;
size_t g_operationDepth = 0;
struct PendingPrimNotice
{
    pxr::SdfPath path;
    bool removed;
};
std::vector<PendingPrimNotice> g_pendingPrimNotices;

/** @brief Protect a complete API or callback phase and defer reentrant prim notices. */
class OperationScope
{
public:
    explicit OperationScope(bool engineCallback = false) : m_lock(g_operationMutex)
    {
        if (!engineCallback && g_operationDepth == 0)
        {
            const auto canAccessSimulation = []
            { return !g_simulationRunning || g_postStepGeneration == g_simulationGeneration; };
            if (std::this_thread::get_id() == g_stageUpdateThread)
            {
                // Async physics finishes on a worker, but its completion notification is delivered
                // by this thread on the next Kit update. Waiting only for that notification deadlocks.
                // Check all scenes' actual completion without fetching results or pumping Kit events.
                // Keep the operation guard when proceeding; release it between polls so physics
                // callbacks can finish. The next Kit simulation launch cannot run on this thread yet.
                while (!canAccessSimulation() && !g_physxSimulation->checkResults())
                {
                    g_simulationComplete.wait_for(m_lock, std::chrono::milliseconds(1));
                }
            }
            else
            {
                g_simulationComplete.wait(m_lock, canAccessSimulation);
            }
        }
        ++g_operationDepth;
    }
    ~OperationScope()
    {
        // Keep depth nonzero while draining: notices generated by processing a notice
        // must join the next pass instead of invalidating the current iteration.
        if (g_operationDepth == 1)
        {
            while (!g_pendingPrimNotices.empty())
            {
                std::vector<PendingPrimNotice> notices;
                notices.swap(g_pendingPrimNotices);
                for (const auto& notice : notices)
                {
                    if (!g_stage || !g_surfaceGripperManager)
                    {
                        continue;
                    }
                    auto prim = g_stage->GetPrimAtPath(notice.path);
                    if (notice.removed && !prim)
                    {
                        g_surfaceGripperManager->onComponentRemove(notice.path);
                    }
                    else if (prim)
                    {
                        g_surfaceGripperManager->onComponentChange(prim);
                    }
                }
            }
        }
        --g_operationDepth;
    }
    OperationScope(const OperationScope&) = delete;
    OperationScope& operator=(const OperationScope&) = delete;

private:
    std::unique_lock<std::recursive_mutex> m_lock;
};

/**
 * @brief Run distinct gripper paths in parallel, keeping repeated paths in input order.
 * @param primPaths Paths whose character storage stays alive until this call returns.
 * @param count Number of requests.
 * @param func Operation accepting the original request index.
 */
template <typename Func>
void forEachGripperPath(const char* const* primPaths, size_t count, Func func)
{
    std::unordered_map<std::string_view, size_t> lastIndices;
    lastIndices.reserve(count);
    std::vector<size_t> firstIndices;
    firstIndices.reserve(count);
    std::vector<size_t> nextIndices(count, count);
    for (size_t i = 0; i < count; ++i)
    {
        auto entry = lastIndices.emplace(primPaths[i], i);
        if (entry.second)
        {
            firstIndices.push_back(i);
        }
        else
        {
            nextIndices[entry.first->second] = i;
            entry.first->second = i;
        }
    }
    isaacsim::robot::surface_gripper::parallelForIndex(firstIndices.size(),
                                                       [&](size_t group)
                                                       {
                                                           for (size_t i = firstIndices[group]; i != count;
                                                                i = nextIndices[i])
                                                           {
                                                               func(i);
                                                           }
                                                       });
}

} // end of anonymous namespace

// Parallel utilities are provided by ThreadUtils.hpp

namespace surface_gripper
{

/**
 * @brief Checks if the given prim path corresponds to a surface gripper
 * @param[in] primPath USD path to check
 * @return True if the prim is a surface gripper, false otherwise
 */
static bool isSurfaceGripperImpl(const char* primPath)
{
    if (g_stage && g_surfaceGripperManager)
    {
        isaacsim::robot::surface_gripper::SurfaceGripperComponent* gripper =
            g_surfaceGripperManager->getGripper(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (gripper)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("Surface Gripper does not exist");
        }
    }
    return false;
}

/**
 * @brief Gets the status of a surface gripper
 * @param[in] primPath USD path of the gripper
 * @return Status string ("Open" or "Closed"), or empty string if not found
 */
static int getGripperStatusImpl(const char* primPath)
{
    if (g_stage && g_surfaceGripperManager)
    {
        return static_cast<int>(g_surfaceGripperManager->getGripperStatus(primPath));
    }
    return -1;
}

/**
 * @brief Opens a surface gripper
 * @param[in] primPath USD path of the gripper to open
 * @return True if successful, false otherwise
 */
static bool openGripperImpl(const char* primPath)
{
    if (g_stage && g_surfaceGripperManager)
    {
        return g_surfaceGripperManager->setGripperStatus(primPath, isaacsim::robot::surface_gripper::GripperStatus::Open);
    }
    return false;
}

/**
 * @brief Closes a surface gripper
 * @param[in] primPath USD path of the gripper to close
 * @return True if successful, false otherwise
 */
static bool closeGripperImpl(const char* primPath)
{
    if (g_stage && g_surfaceGripperManager)
    {
        return g_surfaceGripperManager->setGripperStatus(
            primPath, isaacsim::robot::surface_gripper::GripperStatus::Closed);
    }
    return false;
}

/**
 * @brief Sets the action of a surface gripper
 * @param[in] primPath USD path of the gripper
 * @param[in] action Action value (-1.0 to 1.0)
 * @return True if successful, false otherwise
 */
static bool setGripperActionImpl(const char* primPath, const float action)
{
    if (action < -0.3f)
    {
        return openGripperImpl(primPath);
    }
    else if (action > 0.3f)
    {
        return closeGripperImpl(primPath);
    }
    return false;
}


/**
 * @brief Gets a list of objects currently gripped by a specific gripper
 * @param[in] primPath USD path of the gripper
 * @return Vector of prim paths for gripped objects
 */
static std::vector<std::string> getGrippedObjectsImpl(const char* primPath)
{
    if (g_stage && g_surfaceGripperManager)
    {
        isaacsim::robot::surface_gripper::SurfaceGripperComponent* gripper =
            g_surfaceGripperManager->getGripper(primPath);
        if (gripper)
        {
            std::vector<std::string> grippedObjects;
            for (const auto& object : gripper->getGrippedObjects())
            {
                grippedObjects.push_back(object);
            }
            return grippedObjects;
        }
    }
    return std::vector<std::string>();
}

/**
 * @brief Batched: get status strings for multiple surface grippers in input order.
 */
std::vector<int> CARB_ABI getGripperStatusBatch(const char* const* primPaths, size_t count)
{
    OperationScope operation;
    std::vector<int> results(count);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    forEachGripperPath(
        primPaths, count, [&](size_t i) { results[i] = static_cast<int>(getGripperStatusImpl(primPaths[i])); });
    return results;
}

/**
 * @brief Batched: open multiple surface grippers in input order.
 */
std::vector<bool> CARB_ABI openGripperBatch(const char* const* primPaths, size_t count)
{
    OperationScope operation;
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    // vector<bool> packs adjacent results into shared words; workers write separate bytes instead.
    std::vector<unsigned char> flags(count);
    forEachGripperPath(primPaths, count, [&](size_t i) { flags[i] = openGripperImpl(primPaths[i]); });
    for (size_t i = 0; i < count; ++i)
    {
        results[i] = flags[i] != 0;
    }
    return results;
}

/**
 * @brief Batched: close multiple surface grippers in input order.
 */
std::vector<bool> CARB_ABI closeGripperBatch(const char* const* primPaths, size_t count)
{
    OperationScope operation;
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    // vector<bool> packs adjacent results into shared words; workers write separate bytes instead.
    std::vector<unsigned char> flags(count);
    forEachGripperPath(primPaths, count, [&](size_t i) { flags[i] = closeGripperImpl(primPaths[i]); });
    for (size_t i = 0; i < count; ++i)
    {
        results[i] = flags[i] != 0;
    }
    return results;
}

/**
 * @brief Batched: set actions for multiple surface grippers in input order.
 */
std::vector<bool> CARB_ABI setGripperActionBatch(const char* const* primPaths, const float* actions, size_t count)
{
    OperationScope operation;
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || actions == nullptr || count == 0)
    {
        return results;
    }
    // vector<bool> packs adjacent results into shared words; workers write separate bytes instead.
    std::vector<unsigned char> flags(count);
    forEachGripperPath(primPaths, count, [&](size_t i) { flags[i] = setGripperActionImpl(primPaths[i], actions[i]); });
    for (size_t i = 0; i < count; ++i)
    {
        results[i] = flags[i] != 0;
    }
    return results;
}

/**
 * @brief Batched: get gripped objects arrays for multiple surface grippers in input order.
 */
std::vector<std::vector<std::string>> CARB_ABI getGrippedObjectsBatch(const char* const* primPaths, size_t count)
{
    OperationScope operation;
    std::vector<std::vector<std::string>> results(count);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    forEachGripperPath(primPaths, count, [&](size_t i) { results[i] = getGrippedObjectsImpl(primPaths[i]); });
    return results;
}

/**
 * @brief Gets a list of all surface grippers in the scene
 * @return Vector of prim paths for all surface grippers
 */
static std::vector<std::string> getAllGrippersImpl()
{
    if (g_surfaceGripperManager)
    {
        return g_surfaceGripperManager->getAllGrippers();
    }
    return std::vector<std::string>();
}

static bool setWriteToUsdImpl(const bool writeToUsd)
{
    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->setWriteToUsd(writeToUsd);
        return true;
    }
    return false;
}


/** @brief Serialize the complete isSurfaceGripper operation against physics and lifecycle changes. */
bool CARB_ABI isSurfaceGripper(const char* primPath)
{
    OperationScope operation;
    return isSurfaceGripperImpl(primPath);
}

/** @brief Serialize the complete getGripperStatus operation against physics and lifecycle changes. */
int CARB_ABI getGripperStatus(const char* primPath)
{
    OperationScope operation;
    return getGripperStatusImpl(primPath);
}

/** @brief Serialize the complete openGripper operation against physics and lifecycle changes. */
bool CARB_ABI openGripper(const char* primPath)
{
    OperationScope operation;
    return openGripperImpl(primPath);
}

/** @brief Serialize the complete closeGripper operation against physics and lifecycle changes. */
bool CARB_ABI closeGripper(const char* primPath)
{
    OperationScope operation;
    return closeGripperImpl(primPath);
}

/** @brief Serialize the complete setGripperAction operation against physics and lifecycle changes. */
bool CARB_ABI setGripperAction(const char* primPath, const float action)
{
    OperationScope operation;
    return setGripperActionImpl(primPath, action);
}

/** @brief Serialize the complete getGrippedObjects operation against physics and lifecycle changes. */
std::vector<std::string> CARB_ABI getGrippedObjects(const char* primPath)
{
    OperationScope operation;
    return getGrippedObjectsImpl(primPath);
}

/** @brief Serialize the complete getAllGrippers operation against physics and lifecycle changes. */
std::vector<std::string> CARB_ABI getAllGrippers()
{
    OperationScope operation;
    return getAllGrippersImpl();
}

/** @brief Serialize the complete setWriteToUsd operation against physics and lifecycle changes. */
bool CARB_ABI setWriteToUsd(const bool writeToUsd)
{
    OperationScope operation;
    return setWriteToUsdImpl(writeToUsd);
}


} // namespace surface_gripper

/**
 * @brief Called when simulation starts playing
 */
void onPlay()
{
    OperationScope operation(true);
    // CARB_LOG_WARN("On Play");
    if (!g_stage)
    {
        return;
    }
    // CARB_LOG_WARN("Stage exists");
    g_simulationView = g_tensorApi->createSimulationView(g_stageID, nullptr);

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(g_stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    // CARB_LOG_WARN("Getting elements with type: %s",
    // isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::SURFACE_GRIPPER).GetString().c_str());
    const std::vector<usdrt::SdfPath> surfaceGripperPaths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken(
        isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::SURFACE_GRIPPER).GetString()));


    for (const usdrt::SdfPath& usdrtPath : surfaceGripperPaths)
    {
        // For any already existing grippers, add them to the manager
        const omni::fabric::Path path(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
        // CARB_LOG_WARN("Surface Gripper found: %s", usdPath.GetString().c_str());

        if (g_surfaceGripperManager)
        {
            pxr::UsdPrim prim = g_stage->GetPrimAtPath(usdPath);
            if (prim)
            {
                // CARB_LOG_WARN("Surface Gripper found: %s", prim.GetPath().GetString().c_str());
                g_surfaceGripperManager->onComponentAdd(prim);
            }
        }
    }
    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->onStart();
    }
}


/**
 * @brief Called when a stage is attached
 * @param[in] stageId ID of the stage being attached
 * @param[in] metersPerUnit Conversion factor from stage units to meters
 * @param[in] userData User data pointer
 */
static void onAttach(long int stageId, double metersPerUnit, void* userData)
{
    OperationScope operation(true);
    g_simulationRunning = false;
    g_simulationComplete.notify_all();
    // Stage lifecycle callbacks run on the Kit thread that drives simulation completion.
    g_stageUpdateThread = std::this_thread::get_id();
    g_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    g_stageID = stageId;


    if (!g_stage)
    {
        CARB_LOG_ERROR("Stage not available!");
        return;
    }

    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->initialize(g_stage);
        g_surfaceGripperManager->initComponents();
    }
}

/**
 * @brief Called when simulation stops
 * @param[in] userData User data pointer
 */
static void onStop(void* userData)
{
    OperationScope operation(true);
    g_simulationRunning = false;
    g_simulationComplete.notify_all();
    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->onStop();
        g_surfaceGripperManager->deleteAllComponents();
    }
    g_firstFrame = true;
    if (g_simulationView != nullptr)
    {
        g_simulationView->release(true);
        g_simulationView = nullptr;
    }
}

static void onDetach(void* userData)
{
    OperationScope operation(true);
    g_simulationRunning = false;
    g_simulationComplete.notify_all();
    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->deleteAllComponents();
    }
    if (g_simulationView != nullptr)
    {
        g_simulationView->release(true);
        g_simulationView = nullptr;
    }
}

/**
 * @brief Called when a component changes
 * @param[in] primOrPropertyPath Path of the prim or property that changed
 * @param[in] userData User data pointer
 */
void onComponentChange(const pxr::SdfPath& primOrPropertyPath, void* userData)
{
    OperationScope operation(true);
    g_pendingPrimNotices.push_back({ primOrPropertyPath, false });
}

/**
 * @brief Called for each physics step
 * @param[in] dt Time step in seconds
 * @param[in] userData User data pointer
 */
void onPhysicsStep(float dt, void* userData)
{
    OperationScope operation(true);
    if (g_surfaceGripperManager)
    {
        if (g_firstFrame)
        {
            CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripper::firstFramePlay");
            onPlay();
            g_firstFrame = false;
        }
        g_surfaceGripperManager->onPhysicsStep(dt);
    }
}

/**
 * @brief Called when a prim is removed
 * @param[in] primPath Path of the prim being removed
 * @param[in] userData User data pointer
 */
static void onPrimRemove(const pxr::SdfPath& primPath, void* userData)
{
    OperationScope operation(true);
    g_pendingPrimNotices.push_back({ primPath, true });
}

/** @brief Mark the engine thread's safe post-step phase without changing gripper update ordering. */
void onPostStepBoundary(float dt, void* userData)
{
    OperationScope operation(true);
    // Later subscribers on this thread may use the API without waiting for their own
    // completion event. The next simulation start invalidates this permission everywhere.
    g_postStepGeneration = g_simulationGeneration;
}

/** @brief Exclude public API access from simulate/fetchResults, including asynchronous simulation. */
void onSimulationEvent(omni::physx::SimulationStatusEvent event, void* userData)
{
    OperationScope operation(true);
    if (event == omni::physx::eSimulationStarting)
    {
        // Acquiring the guard above first waits for any already-running API batch.
        ++g_simulationGeneration;
        g_simulationRunning = true;
    }
    else if (event == omni::physx::eSimulationComplete)
    {
        // Unlike a substep end, this event explicitly permits simulation reads/writes.
        g_simulationRunning = false;
        g_simulationComplete.notify_all();
    }
}

/**
 * @brief Called when the plugin is initialized
 */
CARB_EXPORT void carbOnPluginStartup()
{
    // CARB_LOG_WARN("SURFACE GRIPPER On Plugin Startup");
    g_stageUpdate = carb::getCachedInterface<omni::kit::IStageUpdate>()->getStageUpdate();

    if (!g_stageUpdate)
    {
        CARB_LOG_ERROR("*** Failed to acquire Stage Update interface\n");
        return;
    }

    g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
    if (!g_physx)
    {
        CARB_LOG_ERROR("*** Failed to acquire PhysX interface\n");
        return;
    }

    g_physxSimulation = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
    if (!g_physxSimulation)
    {
        CARB_LOG_ERROR("*** Failed to acquire PhysX simulation interface\n");
        return;
    }

    g_tensorApi = carb::getCachedInterface<omni::physics::tensors::TensorApi>();
    if (!g_tensorApi)
    {
        CARB_LOG_ERROR("*** Failed to acquire Tensor Api interface\n");
        return;
    }


    // Carb settings interface removed (unused)

    g_surfaceGripperManager = std::make_unique<isaacsim::robot::surface_gripper::SurfaceGripperManager>(g_physx);
    // CARB_LOG_WARN("Surface Gripper Manager created");
    omni::kit::StageUpdateNodeDesc desc = { nullptr };
    desc.displayName = "Surface Gripper Interface";
    desc.onAttach = onAttach;
    desc.onDetach = onDetach;
    desc.onPrimRemove = onPrimRemove;
    desc.onStop = onStop;
    desc.onPrimOrPropertyChange = onComponentChange;
    desc.order = 60; // Happens after physx and sensor plugins

    g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);
    g_postStepBoundarySubscription =
        g_physx->subscribePhysicsOnStepEvents(false, std::numeric_limits<int>::min(), onPostStepBoundary, nullptr);
    g_stepSubscription = g_physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep, nullptr);
    g_simulationSubscription = g_physx->subscribePhysicsSimulationEvents(onSimulationEvent, nullptr);
    if (!g_stageUpdateNode)
    {
        CARB_LOG_ERROR("*** Failed to create stage update node\n");
        return;
    }
    INITIALIZE_OGN_NODES()
}

/**
 * @brief Called when the plugin is shut down
 */
CARB_EXPORT void carbOnPluginShutdown()
{
    RELEASE_OGN_NODES()
    g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
    g_physx->unsubscribePhysicsOnStepEvents(g_postStepBoundarySubscription);
    g_physx->unsubscribePhysicsOnStepEvents(g_stepSubscription);
    g_physx->unsubscribePhysicsSimulationEvents(g_simulationSubscription);
    OperationScope operation(true);
    g_simulationRunning = false;
    g_simulationComplete.notify_all();
    g_pendingPrimNotices.clear();
    g_surfaceGripperManager.reset();
    g_physx = nullptr;
    g_physxSimulation = nullptr;
    g_stageUpdateThread = {};
    g_stage = nullptr;
}

/**
 * @brief Fills the interface with function pointers
 * @param[out] iface The interface to fill
 */
void fillInterface(isaacsim::robot::surface_gripper::SurfaceGripperInterface& iface)
{
    using namespace isaacsim::robot::surface_gripper;
    memset(&iface, 0, sizeof(iface));

    iface.getGripperStatus = surface_gripper::getGripperStatus;
    iface.openGripper = surface_gripper::openGripper;
    iface.closeGripper = surface_gripper::closeGripper;
    iface.setGripperAction = surface_gripper::setGripperAction;
    iface.getGrippedObjects = surface_gripper::getGrippedObjects;
    iface.setWriteToUsd = surface_gripper::setWriteToUsd;

    // Batched functions
    iface.getGripperStatusBatch = surface_gripper::getGripperStatusBatch;
    iface.openGripperBatch = surface_gripper::openGripperBatch;
    iface.closeGripperBatch = surface_gripper::closeGripperBatch;
    iface.setGripperActionBatch = surface_gripper::setGripperActionBatch;
    iface.getGrippedObjectsBatch = surface_gripper::getGrippedObjectsBatch;
}

#ifdef _WIN32
#    pragma warning(pop)
#endif
