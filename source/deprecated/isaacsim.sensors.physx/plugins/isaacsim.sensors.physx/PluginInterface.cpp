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

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
#include <pxr/usd/usd/inherits.h>
// clang-format on

#include "core/RangeSensorManager.h"
#include "generic/GenericSensor.h"
#include "lidar/LidarSensor.h"
#include "lightbeam_sensor/LightBeamSensor.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>

#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/graph/core/ogn/Registration.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>

#include <vector>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.sensors.physx.plugin", "Isaac Range Sensor", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(g_kPluginDesc,
                 isaacsim::sensors::physx::LidarSensorInterface,
                 isaacsim::sensors::physx::GenericSensorInterface,
                 isaacsim::sensors::physx::LightBeamSensorInterface)


CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::kit::IStageUpdate,
                      omni::fabric::IStageReaderWriter,
                      carb::tasking::ITasking,
                      omni::graph::core::IGraphRegistry)

DECLARE_OGN_NODES()


// private stuff
namespace
{


omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
omni::physx::IPhysx* g_physx = nullptr;
pxr::UsdStageWeakPtr g_stage = nullptr;
carb::tasking::ITasking* g_tasking = nullptr;


std::unique_ptr<isaacsim::sensors::physx::RangeSensorManager> g_rangeSensorManager;
omni::physx::SubscriptionId g_stepSubscription;

} // end of anonymous namespace

namespace lidar
{

int CARB_ABI getNumCols(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->getNumCols();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return 0;
    }
}

int CARB_ABI getNumRows(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getNumRows();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return 0;
    }
}

int CARB_ABI getNumColsTicked(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getNumColsTicked();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return 0;
    }
}

uint16_t* CARB_ABI getDepthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getDepthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getLinearDepthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getLinearDepthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getBeamTimeData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getBeamTimeData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}
uint8_t* CARB_ABI getIntensityData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getIntensityData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getZenithData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getZenithData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getAzimuthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getAzimuthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

carb::Float3* CARB_ABI getPointCloud(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getPointCloud().data();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return nullptr;
    }
}

std::vector<std::string> CARB_ABI getPrimData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getPrimData();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return std::vector<std::string>();
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return std::vector<std::string>();
    }
}


bool CARB_ABI isLidarSensor(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return false;
    }
}

uint64_t CARB_ABI getSequenceNumber(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->getSequenceNumber();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return 0;
    }
}

carb::Float2 CARB_ABI getAzimuthRange(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->getAzimuthRange();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return carb::Float2();
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return carb::Float2();
    }
}

carb::Float2 CARB_ABI getZenithRange(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LidarSensor* sensor =
            g_rangeSensorManager->getLidarSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->getZenithRange();
        }
        else
        {
            CARB_LOG_ERROR("Lidar Sensor does not exist");
            return carb::Float2();
        }
    }
    else
    {
        CARB_LOG_ERROR("Lidar Sensor Manager does not exist");
        return carb::Float2();
    }
}
}

namespace lightbeam_sensor
{
bool CARB_ABI isLightBeamSensor(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("isLightBeamSensor: Light Beam Sensor does not exist (%s)", primPath);
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return false;
    }
}

float* CARB_ABI getLinearDepthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getLinearDepthData().data();
        }
        else
        {
            CARB_LOG_ERROR("getLinearDepthData: Light Beam Sensor does not exist (%s)", primPath);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return nullptr;
    }
}

int CARB_ABI getNumRays(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getNumRays();
        }
        else
        {
            CARB_LOG_ERROR("getNumRays: Light Beam Sensor does not exist (%s)", primPath);
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return 0;
    }
}

carb::Float3* CARB_ABI getHitPosData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getHitPosData().data();
        }
        else
        {
            CARB_LOG_ERROR("getHitPosData: Light Beam Sensor does not exist (%s)", primPath);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return nullptr;
    }
}

uint8_t* CARB_ABI getBeamHitData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getBeamHitData().data();
        }
        else
        {
            CARB_LOG_ERROR("getBeamHitData: Light Beam Sensor does not exist (%s)", primPath);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return nullptr;
    }
}

void CARB_ABI getTransformData(const char* primPath, omni::math::linalg::matrix4d& matrixOutput)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            sensor->getTransformData(matrixOutput);
        }
        else
        {
            CARB_LOG_ERROR("getTransformData: Light Beam Sensor does not exist (%s)", primPath);
            matrixOutput.SetIdentity();
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        matrixOutput.SetIdentity();
    }
}

carb::Float3* CARB_ABI getBeamOrigins(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getBeamOrigins().data();
        }
        else
        {
            CARB_LOG_ERROR("getBeamOrigins: Light Beam Sensor does not exist (%s)", primPath);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return nullptr;
    }
}

carb::Float3* CARB_ABI getBeamEndPoints(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::LightBeamSensor* sensor =
            g_rangeSensorManager->getLightBeamSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));

        if (sensor)
        {
            return sensor->getBeamEndPoints().data();
        }
        else
        {
            CARB_LOG_ERROR("getBeamEndPoints: Light Beam Sensor does not exist (%s)", primPath);
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Isaac Sensor Manager does not exist");
        return nullptr;
    }
}

} // lightbeam_sensor
namespace generic
{
bool CARB_ABI isGenericSensor(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return true;
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return false;
    }
}

bool CARB_ABI sendNextBatch(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->sendNextBatch();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return false;
    }
}


void CARB_ABI setNextBatchRays(const char* primPath,
                               const float* azimuthAngles,
                               const float* zenithAngles,
                               const int sampleLength)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            sensor->setNextBatchRays(azimuthAngles, zenithAngles, sampleLength);
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
    }
}

void CARB_ABI setNextBatchOffsets(const char* primPath, const float* originOffsets, const int sampleLength)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            sensor->setNextBatchOffsets(originOffsets, sampleLength);
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
    }
}


int CARB_ABI getNumSamplesTicked(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getNumSamplesTicked();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return 0;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return 0;
    }
}

uint16_t* CARB_ABI getDepthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getDepthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getLinearDepthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getLinearDepthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}
uint8_t* CARB_ABI getIntensityData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getIntensityData().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getZenithData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getZenithData().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}

float* CARB_ABI getAzimuthData(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {

            return sensor->getAzimuthData().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}

carb::Float3* CARB_ABI getPointCloud(const char* primPath)
{
    if (g_stage && g_rangeSensorManager)
    {
        isaacsim::sensors::physx::GenericSensor* sensor =
            g_rangeSensorManager->getGenericSensor(g_stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        if (sensor)
        {
            return sensor->getPointCloud().data();
        }
        else
        {
            CARB_LOG_ERROR("Generic Sensor does not exist");
            return nullptr;
        }
    }
    else
    {
        CARB_LOG_ERROR("Generic Sensor Manager does not exist");
        return nullptr;
    }
}

}


void onAttach(long stageId, double metersPerUnit, void* data)
{
    g_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    if (!g_stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->initialize(g_stage);
        g_rangeSensorManager->initComponents();
    }
}

void onDetach(void* data)
{
    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->deleteAllComponents();
    }
}

void onUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings* settings, void* userData)
{
    if (!settings->isPlaying)
    {
        return;
    }

    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->tick(elapsedSecs);
    }
}

void onStop(void* userData)
{
    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->onStop();
    }
}

void onPrimAdd(const pxr::SdfPath& primPath, void* userData)
{
    // CARB_LOG_INFO("++ Lidar: Prim Add: %s of type %s\n", primPath,
    //    g_stage->GetPrimAtPath(pxr::SdfPath(primPath)).GetTypeName().GetString().c_str());

    if (g_stage && g_rangeSensorManager)
    {
        pxr::UsdPrim addedPrim = g_stage->GetPrimAtPath(primPath);
        if (!addedPrim)
        {
            return;
        }
        // Add the root prim
        g_rangeSensorManager->onComponentAdd(addedPrim);

        // Check if it has any descendants that need to be added
        pxr::UsdPrimSubtreeRange range = addedPrim.GetDescendants();
        for (pxr::UsdPrimSubtreeRange::iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            pxr::UsdPrim prim = *iter;
            g_rangeSensorManager->onComponentAdd(prim);
        }
    }
}
void onComponentChange(const pxr::SdfPath& primOrPropertyPath, void* userData)
{
    // CARB_LOG_INFO("++ Lidar: Prim Change: %s of type %s\n", primPath,
    //    g_stage->GetPrimAtPath(pxr::SdfPath(primPath)).GetTypeName().GetString().c_str());
    if (g_stage && g_rangeSensorManager)
    {
        g_rangeSensorManager->onComponentChange(g_stage->GetPrimAtPath(primOrPropertyPath));
    }
}

void onPhysicsStep(float dt, void* userData)
{
    CARB_PROFILE_ZONE(0, "[IsaacSim] RangeSensor::onPhysicsStep");
    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->onPhysicsStep(static_cast<double>(dt));
    }
}

void onPrimRemove(const pxr::SdfPath& primPath, void* userData)
{
    // CARB_LOG_INFO("++ Lidar: Prim Remove: %s\n", primPath);
    if (g_rangeSensorManager)
    {
        g_rangeSensorManager->onComponentRemove(primPath);
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

    g_tasking = carb::getCachedInterface<carb::tasking::ITasking>();


    g_rangeSensorManager = std::make_unique<isaacsim::sensors::physx::RangeSensorManager>(g_physx, g_tasking);

    omni::kit::StageUpdateNodeDesc desc = { nullptr };
    desc.displayName = "Range Sensor Interface";
    desc.onAttach = onAttach;
    desc.onDetach = onDetach;
    desc.onUpdate = onUpdate;
    desc.onStop = onStop;
    desc.onPrimAdd = onPrimAdd;
    desc.onPrimOrPropertyChange = onComponentChange;
    desc.onPrimRemove = onPrimRemove;
    desc.order = 50; // happens after physx, dc, but before robot engine bridge

    g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);
    g_stepSubscription = g_physx->subscribePhysicsOnStepEvents(false, 0, onPhysicsStep, nullptr);
    if (!g_stageUpdateNode)
    {
        CARB_LOG_ERROR("*** Failed to create stage update node\n");
        return;
    }

    INITIALIZE_OGN_NODES()
}


CARB_EXPORT void carbOnPluginShutdown()
{
    RELEASE_OGN_NODES()

    g_rangeSensorManager.reset();
    g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
    g_physx->unsubscribePhysicsOnStepEvents(g_stepSubscription);

    g_physx = nullptr;
    g_stage = nullptr;
}


void fillInterface(isaacsim::sensors::physx::LidarSensorInterface& iface)
{
    using namespace isaacsim::sensors::physx;

    memset(&iface, 0, sizeof(iface));
    iface.getNumCols = lidar::getNumCols;
    iface.getNumRows = lidar::getNumRows;
    iface.getNumColsTicked = lidar::getNumColsTicked;
    iface.getDepthData = lidar::getDepthData;
    iface.getLinearDepthData = lidar::getLinearDepthData;
    iface.getIntensityData = lidar::getIntensityData;
    iface.getZenithData = lidar::getZenithData;
    iface.getAzimuthData = lidar::getAzimuthData;
    iface.getBeamTimeData = lidar::getBeamTimeData;
    iface.getPointCloud = lidar::getPointCloud;
    iface.getPrimData = lidar::getPrimData;
    iface.isLidarSensor = lidar::isLidarSensor;
    iface.getSequenceNumber = lidar::getSequenceNumber;
    iface.getAzimuthRange = lidar::getAzimuthRange;
    iface.getZenithRange = lidar::getZenithRange;
}

void fillInterface(isaacsim::sensors::physx::GenericSensorInterface& iface)
{
    using namespace isaacsim::sensors::physx;
    memset(&iface, 0, sizeof(iface));
    iface.getNumSamplesTicked = generic::getNumSamplesTicked;
    iface.getDepthData = generic::getDepthData;
    iface.getLinearDepthData = generic::getLinearDepthData;
    iface.getIntensityData = generic::getIntensityData;
    iface.getZenithData = generic::getZenithData;
    iface.getAzimuthData = generic::getAzimuthData;
    iface.getPointCloud = generic::getPointCloud;
    iface.isGenericSensor = generic::isGenericSensor;
    iface.sendNextBatch = generic::sendNextBatch;
    iface.setNextBatchRays = generic::setNextBatchRays;
    iface.setNextBatchOffsets = generic::setNextBatchOffsets;
}

void fillInterface(isaacsim::sensors::physx::LightBeamSensorInterface& iface)
{
    using namespace isaacsim::sensors::physx;

    memset(&iface, 0, sizeof(iface));

    iface.isLightBeamSensor = lightbeam_sensor::isLightBeamSensor;
    iface.getBeamHitData = lightbeam_sensor::getBeamHitData;
    iface.getNumRays = lightbeam_sensor::getNumRays;
    iface.getLinearDepthData = lightbeam_sensor::getLinearDepthData;
    iface.getHitPosData = lightbeam_sensor::getHitPosData;
    iface.getTransformData = lightbeam_sensor::getTransformData;
    iface.getBeamOrigins = lightbeam_sensor::getBeamOrigins;
    iface.getBeamEndPoints = lightbeam_sensor::getBeamEndPoints;
}
