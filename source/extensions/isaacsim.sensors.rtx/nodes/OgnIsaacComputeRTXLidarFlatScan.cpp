// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <pch/UsdPCH.h>
// clang-format on

#include "GenericModelOutput.h"
#include "LidarConfigHelper.h"
#include "OgnIsaacComputeRTXLidarFlatScanDatabase.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/ScopedCudaDevice.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <math.h>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacComputeRTXLidarFlatScan : public isaacsim::core::includes::BaseResetNode
{
private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };
    float m_rotationRate{ 0.0f };
    float m_horizontalFov{ 0.0f };
    float m_horizontalResolution{ 0.0f };
    float m_azimuthRangeStart{ 0.0f };
    float m_azimuthRangeEnd{ 0.0f };
    float m_nearRangeM{ 0.0f };
    float m_farRangeM{ 0.0f };

public:
    void reset()
    {
        m_firstFrame = true;
        m_isInitialized = false;
    }
    bool initialize(OgnIsaacComputeRTXLidarFlatScanDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "IsaacComputeRTXLidarFlatScan initialize");
        auto& state = db.perInstanceState<OgnIsaacComputeRTXLidarFlatScan>();

        // Retrieve lidar prim from render product path, then validate its attributes
        const std::string renderProductPath = std::string(db.tokenToString(db.inputs.renderProductPath()));
        if (renderProductPath.length() == 0)
        {
            CARB_LOG_ERROR("IsaacComputeRTXLidarFlatScan: renderProductPath input is empty. Skipping execution.");
            return false;
        }
        pxr::UsdPrim lidarPrim = isaacsim::core::includes::getCameraPrimFromRenderProduct(renderProductPath);
        if (lidarPrim.IsA<pxr::UsdGeomCamera>())
        {
            CARB_LOG_WARN(
                "RTX sensors as camera prims are deprecated as of Isaac Sim 5.0, and support will be removed in a future release. Please use an OmniLidar prim with the new OmniSensorGenericLidarCoreAPI schema.");
            LidarConfigHelper configHelper;
            configHelper.updateLidarConfig(renderProductPath.c_str());
            if (configHelper.scanType == LidarScanType::kUnknown)
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim scanType is Unknown, and node will not execute. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            if (!configHelper.is2D)
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim is not a 2D Lidar, and node will not execute. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            if (configHelper.scanType == LidarScanType::kSolidState)
            {
                state.m_azimuthRangeStart = configHelper.azimuthStartDeg;
                state.m_azimuthRangeEnd = configHelper.azimuthEndDeg;
                state.m_horizontalFov = state.m_azimuthRangeStart - state.m_azimuthRangeEnd;
                state.m_horizontalResolution = state.m_horizontalFov / static_cast<float>(configHelper.numberOfEmitters);
            }
            else
            {
                state.m_horizontalResolution = 360.0f * configHelper.scanRateBaseHz / configHelper.reportRateBaseHz;
                state.m_azimuthRangeStart = -180.0f;
                state.m_azimuthRangeEnd = 180.0f - state.m_horizontalResolution;
                state.m_horizontalFov = 360.0f;
            }
            state.m_nearRangeM = configHelper.nearRangeM;
            state.m_farRangeM = configHelper.farRangeM;
            state.m_rotationRate = static_cast<float>(configHelper.scanRateBaseHz);
        }
        else
        {
            pxr::TfToken elementsCoordsType;
            if (lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:elementsCoordsType")).Get(&elementsCoordsType) &&
                elementsCoordsType != pxr::TfToken("SPHERICAL"))
            {
                CARB_LOG_ERROR(
                    "IsaacComputeRTXLidarFlatScan: Lidar prim elementsCoordsType is not set to SPHERICAL, and node will not execute. Stop the simulation, correct the issue, and restart.");
                return false;
            }
            pxr::VtFloatArray elevationDeg;
            if (lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:emitterState:s001:elevationDeg")).Get(&elevationDeg))
            {
                const float epsilon = 1e-3f; // Tolerance of 0.001 degrees
                for (const float elev : elevationDeg)
                {
                    if (::fabs(elev) > epsilon)
                    {
                        CARB_LOG_ERROR(
                            "IsaacComputeRTXLidarFlatScan: Lidar prim elevationDeg contains nonzero value %f.", elev);
                        CARB_LOG_ERROR(
                            "IsaacComputeRTXLidarFlatScan: Lidar prim is not a 2D Lidar, and node will not execute. Stop the simulation, correct the issue, and restart.");
                        return false;
                    }
                }
            }
            // Populate any prim-specific outputs
            uint32_t rotationRateAsInt;
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanRateBaseHz")).Get(&rotationRateAsInt);
            state.m_rotationRate = static_cast<float>(rotationRateAsInt);
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:nearRangeM")).Get(&state.m_nearRangeM);
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:farRangeM")).Get(&state.m_farRangeM);

            pxr::TfToken outputType;
            lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanType")).Get(&outputType);
            if (outputType == pxr::TfToken("SOLID_STATE"))
            {
                pxr::VtFloatArray azimuthDeg;
                lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:emitterState:s001:azimuthDeg")).Get(&azimuthDeg);

                state.m_azimuthRangeStart = *std::min_element(azimuthDeg.begin(), azimuthDeg.end());
                state.m_azimuthRangeEnd = *std::max_element(azimuthDeg.begin(), azimuthDeg.end());
                state.m_horizontalFov = state.m_azimuthRangeEnd - state.m_azimuthRangeStart;
                state.m_horizontalResolution = state.m_horizontalFov / static_cast<float>(azimuthDeg.size());
            }
            else
            {
                // Set useful state variables
                uint32_t reportRateBaseHzAsInt;
                lidarPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:reportRateBaseHz")).Get(&reportRateBaseHzAsInt);
                float reportRateBaseHz = static_cast<float>(reportRateBaseHzAsInt);

                state.m_horizontalResolution = 360.0f * state.m_rotationRate / reportRateBaseHz;
                state.m_azimuthRangeStart = -180.0f;
                state.m_azimuthRangeEnd = 180.0f - state.m_horizontalResolution;
                state.m_horizontalFov = 360.0;
            }
        }
        return true;
    }

    static bool compute(OgnIsaacComputeRTXLidarFlatScanDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "IsaacComputeRTXLidarFlatScan compute");
        auto& state = db.perInstanceState<OgnIsaacComputeRTXLidarFlatScan>();
        // Enable downstream execution by default
        db.outputs.exec() = kExecutionAttributeStateEnabled;

        if (state.m_firstFrame)
        {
            state.m_isInitialized = state.initialize(db);
            state.m_firstFrame = false;
        }

        if (!state.m_isInitialized)
        {
            CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: Failed to initialize correctly. Skipping execution.");
            return false;
        }

        // Store CUDA stream on which data pointer will be provided
        cudaStream_t cudaStream = (cudaStream_t)db.inputs.cudaStream();
        // Get CUDA context corresponding to incoming CUDA stream
        CUcontext cudaContext;
        cuStreamGetCtx(cudaStream, &cudaContext);
        // Set thread running this node's CUDA context to this one
        cuCtxPushCurrent(cudaContext);
        // Get CUDA device corresponding to incoming CUDA stream from current CUDA context
        int cudaDevice;
        cuCtxGetDevice(&cudaDevice);

        isaacsim::core::includes::ScopedDevice scopedDevice(cudaDevice);

        // Synchronize the stream to ensure the GMO is populated
        CUDA_CHECK(cudaStreamSynchronize(cudaStream));
        if (!db.inputs.newData())
        {
            CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: newData input is false. Skipping execution.");
            return false;
        }
        // Retrieve and test input data pointer
        void* dataPtr = reinterpret_cast<void*>(db.inputs.dataPtr());
        if (!dataPtr)
        {
            CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: dataPtr input is empty. Skipping execution.");
            return false;
        }

        // Determine if input data pointer is on device or host, based on associated CUDA stream
        // cudaStream_t cudaStream = (cudaStream_t)db.inputs.cudaStream();
        cudaPointerAttributes attributes;
        CUDA_CHECK(cudaPointerGetAttributes(&attributes, dataPtr));
        const bool isDevicePtr{ attributes.type == cudaMemoryTypeDevice };

        if (isDevicePtr)
        {
            CARB_LOG_ERROR("IsaacComputeRTXLidarFlatScan.inputs:dataPtr unexpectedly on device. Expecting input on host.");
            return false;
        }
        auto hostGMO = omni::sensors::getModelOutputPtrFromBuffer(dataPtr);

        if (hostGMO->numElements == 0)
        {
            CARB_LOG_INFO("IsaacComputeRTXLidarFlatScan: hostGMO->numElements is 0. Skipping execution.");
            return false;
        }

        // Verify that we have a supported modality
        if (hostGMO->modality != omni::sensors::Modality::LIDAR)
        {
            CARB_LOG_WARN("IsaacComputeRTXLidarFlatScan: Unsupported sensor modality: %d. Only LIDAR is supported.",
                          static_cast<int>(hostGMO->modality));
            return false;
        }

        db.outputs.horizontalFov() = state.m_horizontalFov;
        db.outputs.horizontalResolution() = state.m_horizontalResolution;
        db.outputs.azimuthRange() = { state.m_azimuthRangeStart, state.m_azimuthRangeEnd - state.m_horizontalResolution };
        db.outputs.rotationRate() = state.m_rotationRate;
        db.outputs.depthRange() = { state.m_nearRangeM, state.m_farRangeM };
        db.outputs.numRows() = 1;

        // Size output buffers as number of elements in horizontal FOV, based on horizontal FOV and resolution
        size_t numElements = static_cast<size_t>(state.m_horizontalFov / state.m_horizontalResolution);
        db.outputs.numCols() = static_cast<int>(numElements);
        db.outputs.linearDepthData().resize(numElements);
        db.outputs.intensitiesData().resize(numElements);
        for (size_t i = 0; i < numElements; i++)
        {
            db.outputs.linearDepthData()[i] = 0.0f;
            db.outputs.intensitiesData()[i] = 0;
        }

        // Iterate over the point cloud, storing valid returns in the appropriate index of the output buffers
        for (size_t i = 0; i < hostGMO->numElements; i++)
        {
            if ((hostGMO->elements.flags[i] & omni::sensors::ElementFlags::VALID) != omni::sensors::ElementFlags::VALID)
            {
                continue;
            }
            float azimuth = hostGMO->elements.x[i];
            float depth = hostGMO->elements.z[i];
            uint8_t intensity = static_cast<uint8_t>(hostGMO->elements.scalar[i] * 255.0f);
            size_t index = static_cast<size_t>((azimuth - state.m_azimuthRangeStart) / state.m_horizontalResolution);
            if (index >= numElements)
            {
                CARB_LOG_INFO(
                    "IsaacComputeRTXLidarFlatScan: Unexpected index: %zu, azimuth: %f, depth: %f, numElements: %zu, horizontalResolution: %f",
                    index, azimuth, depth, numElements, state.m_horizontalResolution);
                continue;
            }
            db.outputs.linearDepthData()[index] = depth;
            db.outputs.intensitiesData()[index] = intensity;
        }

        return true;
    }
};

REGISTER_OGN_NODE()
} // rtx
} // sensors
} // isaacsim
