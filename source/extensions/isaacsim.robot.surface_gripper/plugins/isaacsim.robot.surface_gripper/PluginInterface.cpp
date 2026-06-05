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
#include <pch/UsdPCH.h>
#include <omni/usd/UtilsIncludes.h>

#include "isaacsim/robot/surface_gripper/SurfaceGripperComponent.h"
#include "isaacsim/robot/surface_gripper/SurfaceGripperManager.h"
// Threading utilities
#include "isaacsim/robot/surface_gripper/ThreadUtils.h"
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
// <carb/settings/ISettings.h> include removed: settings interface not used

#include <isaacsim/robot/surface_gripper/ISurfaceGripper.h>
#include <omni/fabric/usd/PathConversion.h>
#include <omni/graph/core/ogn/Registration.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <omni/usd/UsdContext.h>

#include <algorithm>
#include <vector>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.robot.surface_gripper.plugin", "Surface Gripper",
                                                    "NVIDIA", carb::PluginHotReload::eDisabled, "dev" };


CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::robot::surface_gripper::SurfaceGripperInterface)

CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::physx::IPhysxSceneQuery,
                      omni::kit::IStageUpdate,
                      omni::graph::core::IGraphRegistry)

DECLARE_OGN_NODES()


// private stuff
namespace
{
omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
omni::physx::IPhysx* g_physx = nullptr;
pxr::UsdStageWeakPtr g_stage = nullptr;
omni::physics::tensors::TensorApi* g_tensorApi = nullptr;
omni::physics::tensors::ISimulationView* g_simulationView = nullptr;
std::unique_ptr<isaacsim::robot::surface_gripper::SurfaceGripperManager> g_surfaceGripperManager;
omni::physx::SubscriptionId g_stepSubscription;
bool g_firstFrame = true;
long int g_stageID;
} // end of anonymous namespace

// Parallel utilities are provided by ThreadUtils.h

namespace surface_gripper
{

/**
 * @brief Checks if the given prim path corresponds to a surface gripper
 * @param[in] primPath USD path to check
 * @return True if the prim is a surface gripper, false otherwise
 */
bool CARB_ABI isSurfaceGripper(const char* primPath)
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
int CARB_ABI getGripperStatus(const char* primPath)
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
bool CARB_ABI openGripper(const char* primPath)
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
bool CARB_ABI closeGripper(const char* primPath)
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
bool CARB_ABI setGripperAction(const char* primPath, const float action)
{
    if (action < -0.3f)
    {
        return openGripper(primPath);
    }
    else if (action > 0.3f)
    {
        return closeGripper(primPath);
    }
    return false;
}


/**
 * @brief Gets a list of objects currently gripped by a specific gripper
 * @param[in] primPath USD path of the gripper
 * @return Vector of prim paths for gripped objects
 */
std::vector<std::string> CARB_ABI getGrippedObjects(const char* primPath)
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
 * @brief Batched: get status strings for multiple surface grippers in parallel.
 */
std::vector<int> CARB_ABI getGripperStatusBatch(const char* const* primPaths, size_t count)
{
    std::vector<int> results(count);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    isaacsim::robot::surface_gripper::parallelForIndex(
        count, [&](size_t i) { results[i] = static_cast<int>(getGripperStatus(primPaths[i])); });
    return results;
}

/**
 * @brief Batched: open multiple surface grippers in parallel.
 */
std::vector<bool> CARB_ABI openGripperBatch(const char* const* primPaths, size_t count)
{
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    isaacsim::robot::surface_gripper::parallelForIndex(count, [&](size_t i) { results[i] = openGripper(primPaths[i]); });
    return results;
}

/**
 * @brief Batched: close multiple surface grippers in parallel.
 */
std::vector<bool> CARB_ABI closeGripperBatch(const char* const* primPaths, size_t count)
{
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    isaacsim::robot::surface_gripper::parallelForIndex(count, [&](size_t i) { results[i] = closeGripper(primPaths[i]); });
    return results;
}

/**
 * @brief Batched: set actions for multiple surface grippers in parallel.
 */
std::vector<bool> CARB_ABI setGripperActionBatch(const char* const* primPaths, const float* actions, size_t count)
{
    std::vector<bool> results(count, false);
    if (primPaths == nullptr || actions == nullptr || count == 0)
    {
        return results;
    }
    isaacsim::robot::surface_gripper::parallelForIndex(
        count, [&](size_t i) { results[i] = setGripperAction(primPaths[i], actions[i]); });
    return results;
}

/**
 * @brief Batched: get gripped objects arrays for multiple surface grippers in parallel.
 */
std::vector<std::vector<std::string>> CARB_ABI getGrippedObjectsBatch(const char* const* primPaths, size_t count)
{
    std::vector<std::vector<std::string>> results(count);
    if (primPaths == nullptr || count == 0)
    {
        return results;
    }
    isaacsim::robot::surface_gripper::parallelForIndex(
        count, [&](size_t i) { results[i] = getGrippedObjects(primPaths[i]); });
    return results;
}

/**
 * @brief Gets a list of all surface grippers in the scene
 * @return Vector of prim paths for all surface grippers
 */
std::vector<std::string> CARB_ABI getAllGrippers()
{
    if (g_surfaceGripperManager)
    {
        return g_surfaceGripperManager->getAllGrippers();
    }
    return std::vector<std::string>();
}

bool setWriteToUsd(const bool writeToUsd)
{
    if (g_surfaceGripperManager)
    {
        g_surfaceGripperManager->setWriteToUsd(writeToUsd);
        return true;
    }
    return false;
}

} // namespace surface_gripper

/**
 * @brief Called when simulation starts playing
 */
void onPlay()
{
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
    if (g_stage && g_surfaceGripperManager)
    {
        pxr::UsdPrim prim = g_stage->GetPrimAtPath(primOrPropertyPath);
        if (prim)
        {
            g_surfaceGripperManager->onComponentChange(prim);
        }
    }
}

/**
 * @brief Called for each physics step
 * @param[in] dt Time step in seconds
 * @param[in] userData User data pointer
 */
void onPhysicsStep(float dt, void* userData)
{
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
    if (g_surfaceGripperManager)
    {
        auto prim = g_stage->GetPrimAtPath(primPath);
        if (!prim) // Was it really removed?
        {
            g_surfaceGripperManager->onComponentRemove(primPath);
        }
        else
        {
            g_surfaceGripperManager->onComponentChange(prim);
        }
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
    g_stepSubscription = g_physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep, nullptr);
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
    g_surfaceGripperManager.reset();
    g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
    g_physx->unsubscribePhysicsOnStepEvents(g_stepSubscription);
    g_physx = nullptr;
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
