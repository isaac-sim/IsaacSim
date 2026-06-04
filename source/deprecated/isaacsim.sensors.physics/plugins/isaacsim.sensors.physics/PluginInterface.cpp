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

#include <isaacsim/sensors/physics/IsaacSensorComponent.h>
#include <isaacsim/sensors/physics/IsaacSensorManager.h>
#include <isaacsim/sensors/physics/ImuSensor.h>
#include <isaacsim/sensors/physics/ContactManager.h>
#include <isaacsim/sensors/physics/ContactSensor.h>
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/core/includes/Buffer.h>
#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <omni/fabric/usd/PathConversion.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physics/tensors/TensorDesc.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>


const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.sensors.physics.plugin", "Isaac Sensor", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };


CARB_PLUGIN_IMPL(g_kPluginDesc,
                 isaacsim::sensors::physics::ContactSensorInterface,
                 isaacsim::sensors::physics::ImuSensorInterface)

CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx, omni::physx::IPhysxSceneQuery, omni::kit::IStageUpdate)


// private stuff
namespace
{
omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
omni::physx::IPhysx* g_physx = nullptr;
pxr::UsdStageWeakPtr g_stage = nullptr;
omni::physics::tensors::TensorApi* g_tensorApi = nullptr;
omni::physics::tensors::ISimulationView* g_simulationView = nullptr;
omni::physics::tensors::IRigidBodyView* g_rigidBodyView = nullptr;
omni::physics::tensors::TensorDesc g_rigidBodyVelocitiesTensor;
isaacsim::core::includes::GenericBufferBase<float> g_rigidBodyVelocitiesBuffer;
float* g_rigidBodyVelocitiesData = nullptr;
size_t g_rigidBodyVelocitiesDataSize = 0;
cudaStream_t g_velocityCopyStream = nullptr;
std::vector<std::string> g_rigidBodyPaths;
carb::settings::ISettings* g_settings = nullptr;
std::unique_ptr<isaacsim::sensors::physics::IsaacSensorManager> g_isaacSensorManager;
omni::physx::SubscriptionId g_stepSubscription;
bool g_firstFrame = true;
long int g_stageID;
std::unordered_map<std::string, size_t> g_rigidBodyToDataBufferMap;
} // end of anonymous namespace

namespace contact_sensor
{

bool CARB_ABI isContactSensor(const char* primPath)
{
    if (g_stage && g_isaacSensorManager)
    {
        isaacsim::sensors::physics::ContactSensor* sensor =
            g_isaacSensorManager->getContactSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("Contact Sensor does not exist");
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return false;
    }
}

const char* CARB_ABI csDecodeBodyName(uint64_t pathToken)
{
    return isaacsim::core::includes::getSdfPathFromUint64(pathToken).GetString().c_str();
}

isaacsim::sensors::physics::CsRawData* CARB_ABI csGetBodyRawData(const char* primPath, size_t& numContacts)
{
    if (g_stage != nullptr)
    {
        isaacsim::sensors::physics::CsRawData* data = nullptr;
        if (g_isaacSensorManager != nullptr && g_isaacSensorManager->getContactManager() != nullptr)
        {
            data = g_isaacSensorManager->getContactManager()->getCsRawData(primPath, numContacts);
        }
        else
        {
            CARB_LOG_ERROR("Sensor Manager or Contact Manager not found");
        }
        return data;
    }
    else
    {
        CARB_LOG_ERROR("Stage not found");
    }
    return nullptr;
}


isaacsim::sensors::physics::CsRawData* CARB_ABI csGetSensorRawData(const char* primPath, size_t& numContacts)
{
    isaacsim::sensors::physics::ContactSensor* sensor =
        g_isaacSensorManager->getContactSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
    isaacsim::sensors::physics::CsRawData* data = nullptr;

    if (sensor)
    {
        data = sensor->getRawData(numContacts);
    }
    return data;
}

isaacsim::sensors::physics::CsReading CARB_ABI csGetSensorReading(const char* primPath, const bool& getLatestValue = false)
{
    isaacsim::sensors::physics::ContactSensor* sensor =
        g_isaacSensorManager->getContactSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
    isaacsim::sensors::physics::CsReading data;
    if (sensor)
    {
        data = sensor->getSensorReading(getLatestValue);
    }
    return data;
}
}

namespace imu_sensor
{

bool CARB_ABI isImuSensor(const char* primPath)
{
    if (g_stage && g_isaacSensorManager)
    {
        isaacsim::sensors::physics::ImuSensor* sensor =
            g_isaacSensorManager->getImuSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("Imu Sensor does not exist");
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return false;
    }
}

isaacsim::sensors::physics::IsReading CARB_ABI isGetSensorReading(
    const char* primPath,
    const std::function<isaacsim::sensors::physics::IsReading(std::vector<isaacsim::sensors::physics::IsReading>, float)>&
        interpolateFunction = {},
    const bool& getLatestValue = false,
    const bool& readGravity = true)
{
    isaacsim::sensors::physics::ImuSensor* sensor =
        g_isaacSensorManager->getImuSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
    isaacsim::sensors::physics::IsReading data = isaacsim::sensors::physics::IsReading();
    if (sensor)
    {
        data = sensor->getSensorReading(interpolateFunction, getLatestValue, readGravity);
    }
    return data;
}

}

void onPlay()
{
    g_simulationView = g_tensorApi->createSimulationView(g_stageID, nullptr);

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(g_stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    const std::vector<usdrt::SdfPath> imuSensorPaths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("IsaacImuSensor"));

    // First create the sensors and find the sensor parents to create physics views
    for (const usdrt::SdfPath& usdrtPath : imuSensorPaths)
    {
        const omni::fabric::Path path(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
        pxr::UsdPrim prim = g_stage->GetPrimAtPath(usdPath);
        if (prim)
        {
            // Add the root prim
            g_isaacSensorManager->onComponentAdd(prim);
            isaacsim::sensors::physics::ImuSensor* imuSensor = dynamic_cast<isaacsim::sensors::physics::ImuSensor*>(
                g_isaacSensorManager->getComponent(prim.GetPath().GetString()));

            //  if the imu has no valid parent, return
            if (imuSensor != nullptr)
            {
                std::string parentPath = imuSensor->getParentPrim().GetPath().GetString();

                if (std::find(g_rigidBodyPaths.begin(), g_rigidBodyPaths.end(), parentPath) == g_rigidBodyPaths.end())
                {
                    g_rigidBodyPaths.push_back(parentPath);
                }
            }
        }
    }

    {
        const std::vector<usdrt::SdfPath> contactSensorPaths =
            usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("IsaacContactSensor"));

        // First create the sensors and find the sensor parents to create physics views
        for (const usdrt::SdfPath& usdrtPath : contactSensorPaths)
        {
            const omni::fabric::Path path(usdrtPath);
            const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
            pxr::UsdPrim prim = g_stage->GetPrimAtPath(usdPath);

            if (prim)
            {
                // Add the root prim
                g_isaacSensorManager->onComponentAdd(prim);
            }
        }
    }

    // Then create physics views
    if (!g_rigidBodyPaths.empty())
    {
        g_rigidBodyView = g_simulationView->createRigidBodyView(g_rigidBodyPaths);
    }
    else
    {
        g_rigidBodyView = nullptr;
    }

    for (size_t i = 0; i < g_rigidBodyPaths.size(); i++)
    {
        g_rigidBodyToDataBufferMap[g_rigidBodyPaths[i]] = i * 6;
    }

    size_t newSize = 6 * g_rigidBodyPaths.size();
    int deviceOrdinal = g_simulationView->getDeviceOrdinal();
    if (g_rigidBodyVelocitiesData && g_rigidBodyVelocitiesDataSize != newSize)
    {
        if (deviceOrdinal >= 0)
        {
            CUDA_CHECK(cudaFreeHost(g_rigidBodyVelocitiesData));
        }
        else
        {
            free(g_rigidBodyVelocitiesData);
        }
        g_rigidBodyVelocitiesData = nullptr;
    }
    if (g_rigidBodyVelocitiesData == nullptr && newSize > 0)
    {
        if (deviceOrdinal >= 0)
        {
            CUDA_CHECK(cudaHostAlloc(&g_rigidBodyVelocitiesData, newSize * sizeof(float), cudaHostAllocDefault));
        }
        else
        {
            g_rigidBodyVelocitiesData = static_cast<float*>(malloc(newSize * sizeof(float)));
            if (!g_rigidBodyVelocitiesData)
            {
                CARB_LOG_ERROR("Failed to allocate CPU memory for rigid body velocities");
                return;
            }
        }
        g_rigidBodyVelocitiesDataSize = newSize;
        memset(g_rigidBodyVelocitiesData, 0, newSize * sizeof(float));
    }
    g_rigidBodyVelocitiesBuffer.resize(newSize);
    g_rigidBodyVelocitiesBuffer.setDevice(deviceOrdinal);
    g_rigidBodyVelocitiesTensor.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    g_rigidBodyVelocitiesTensor.numDims = 2;
    g_rigidBodyVelocitiesTensor.dims[0] = static_cast<int>(g_rigidBodyPaths.size());
    g_rigidBodyVelocitiesTensor.dims[1] = 6;
    g_rigidBodyVelocitiesTensor.data = g_rigidBodyVelocitiesBuffer.data();
    g_rigidBodyVelocitiesTensor.ownData = true;
    g_rigidBodyVelocitiesTensor.device = deviceOrdinal;

    if (g_velocityCopyStream == nullptr && deviceOrdinal >= 0)
    {
        CUDA_CHECK(cudaStreamCreate(&g_velocityCopyStream));
    }

    // pass in the view data and index to the sensor
    for (const usdrt::SdfPath& usdrtPath : imuSensorPaths)
    {
        const omni::fabric::Path path(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
        pxr::UsdPrim prim = g_stage->GetPrimAtPath(usdPath);

        isaacsim::sensors::physics::ImuSensor* imuSensor = g_isaacSensorManager->getImuSensor(prim);
        if (imuSensor != nullptr)
        {
            size_t sensorDataIndex = g_rigidBodyToDataBufferMap[imuSensor->getParentPrim().GetPath().GetString()];
            imuSensor->initialize(g_rigidBodyVelocitiesData, sensorDataIndex);
        }
    }
}

static void onAttach(long int stageId, double metersPerUnit, void* userData)
{
    g_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    g_stageID = stageId;
    if (!g_stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    if (g_isaacSensorManager)
    {
        g_isaacSensorManager->initialize(g_stage);
        g_isaacSensorManager->initComponents();
    }
}

static void onStop(void* userData)
{
    if (g_isaacSensorManager)
    {
        g_isaacSensorManager->onStop();
        g_isaacSensorManager->deleteAllComponents();
    }
    g_firstFrame = true;
    if (g_simulationView != nullptr)
    {
        g_simulationView->release(true);
        g_simulationView = nullptr;
        g_rigidBodyView = nullptr;
    }
    g_rigidBodyPaths.clear();
    if (g_rigidBodyVelocitiesData)
    {
        int deviceOrdinal = g_simulationView ? g_simulationView->getDeviceOrdinal() : -1;
        if (deviceOrdinal >= 0)
        {
            CUDA_CHECK(cudaFreeHost(g_rigidBodyVelocitiesData));
        }
        else
        {
            free(g_rigidBodyVelocitiesData);
        }
        g_rigidBodyVelocitiesData = nullptr;
    }
    g_rigidBodyVelocitiesDataSize = 0;
    if (g_velocityCopyStream)
    {
        CUDA_CHECK(cudaStreamDestroy(g_velocityCopyStream));
        g_velocityCopyStream = nullptr;
    }
    g_rigidBodyToDataBufferMap.clear();
}

void onComponentChange(const pxr::SdfPath& primOrPropertyPath, void* userData)
{
    if (g_stage && g_isaacSensorManager)
    {
        g_isaacSensorManager->onComponentChange(g_stage->GetPrimAtPath(primOrPropertyPath));
    }
}

void onPhysicsStep(float dt, void* userData)
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacSensor::onPhysicsStep");
    if (g_isaacSensorManager)
    {
        if (g_firstFrame)
        {
            CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacSensor::firstFramePlay");
            onPlay();
            g_firstFrame = false;
        }

        if (g_rigidBodyView != nullptr)
        {
            g_rigidBodyView->getVelocities(&g_rigidBodyVelocitiesTensor);
            int deviceOrdinal = g_simulationView->getDeviceOrdinal();
            if (deviceOrdinal >= 0)
            {
                g_rigidBodyVelocitiesBuffer.copyToAsync(
                    g_rigidBodyVelocitiesData, g_rigidBodyVelocitiesBuffer.size(), g_velocityCopyStream);
                CUDA_CHECK(cudaStreamSynchronize(g_velocityCopyStream));
            }
            else
            {
                memcpy(g_rigidBodyVelocitiesData, g_rigidBodyVelocitiesBuffer.data(),
                       g_rigidBodyVelocitiesBuffer.size() * sizeof(float));
            }
        }

        g_isaacSensorManager->onPhysicsStep(static_cast<double>(dt));
    }
}


static void onPrimRemove(const pxr::SdfPath& primPath, void* userData)
{
    if (g_isaacSensorManager)
    {
        g_isaacSensorManager->onComponentRemove(primPath);
    }
}

CARB_EXPORT void carbOnPluginStartup()
{
    g_stageUpdate = carb::getCachedInterface<omni::kit::IStageUpdate>()->getStageUpdate();
    if (!g_stageUpdate)
    {
        CARB_LOG_ERROR("*** Failed to acquire stage update interface\n");
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

    g_settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (!g_settings)
    {
        CARB_LOG_ERROR("*** Failed to acquire Carb Setting interface\n");
        return;
    }
    static constexpr char s_kSetting[] = "/physics/suppressReadback";
    g_settings->setBool(s_kSetting, false);

    g_isaacSensorManager = std::make_unique<isaacsim::sensors::physics::IsaacSensorManager>(g_physx);

    omni::kit::StageUpdateNodeDesc desc = { nullptr };
    desc.displayName = "Isaac Sensor Interface";
    desc.onAttach = onAttach;
    desc.onDetach = onStop;
    desc.onPrimRemove = onPrimRemove;
    desc.onStop = onStop;
    desc.onPrimOrPropertyChange = onComponentChange;
    desc.order = 50; // happens after physx, dc, but before robot engine bridge

    g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);
    g_stepSubscription = g_physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep, nullptr);
    if (!g_stageUpdateNode)
    {
        CARB_LOG_ERROR("*** Failed to create stage update node\n");
        return;
    }
}


CARB_EXPORT void carbOnPluginShutdown()
{
    g_isaacSensorManager.reset();
    g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
    g_physx->unsubscribePhysicsOnStepEvents(g_stepSubscription);
    g_physx = nullptr;
    g_stage = nullptr;
}

void fillInterface(isaacsim::sensors::physics::ContactSensorInterface& iface)
{
    using namespace isaacsim::sensors::physics;

    memset(&iface, 0, sizeof(iface));


    iface.getSensorRawData = contact_sensor::csGetSensorRawData;
    iface.getBodyRawData = contact_sensor::csGetBodyRawData;
    iface.getSensorReading = contact_sensor::csGetSensorReading;
    iface.decodeBodyName = contact_sensor::csDecodeBodyName;
    iface.isContactSensor = contact_sensor::isContactSensor; // Checks if the path is a contact sensor
}

void fillInterface(isaacsim::sensors::physics::ImuSensorInterface& iface)
{
    using namespace isaacsim::sensors::physics;

    memset(&iface, 0, sizeof(iface));

    iface.getSensorReading = imu_sensor::isGetSensorReading;
    iface.isImuSensor = imu_sensor::isImuSensor;
}

#ifdef _WIN32
#    pragma warning(pop)
#endif
