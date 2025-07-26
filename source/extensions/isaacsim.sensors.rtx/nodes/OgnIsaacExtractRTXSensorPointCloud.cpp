// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "IsaacSimSensorsRTXCuda.cuh"
#include "LidarConfigHelper.h"
#include "OgnIsaacExtractRTXSensorPointCloudDatabase.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/Buffer.h"


namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacExtractRTXSensorPointCloud : public isaacsim::core::includes::BaseResetNode
{
public:
    void reset()
    {
        m_firstFrame = true;
        m_isInitialized = false;
    }

    bool initialize(OgnIsaacExtractRTXSensorPointCloudDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "IsaacExtractRTXSensorPointCloud initialize");

        m_deviceBuffers.initialize(reinterpret_cast<void*>(db.inputs.dataPtr()), (cudaStream_t)db.inputs.cudaStream());
        if (!m_deviceBuffers.gmoOnDevice)
        {
            // Preallocate host buffer to store point cloud
            hostPcBuffer.resize(m_deviceBuffers.bufferSize);
        }
        return true;
    }

    static bool compute(OgnIsaacExtractRTXSensorPointCloudDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "IsaacExtractRTXSensorPointCloud compute");
        auto& state = db.perInstanceState<OgnIsaacExtractRTXSensorPointCloud>();
        // Enable downstream execution by default
        db.outputs.exec() = kExecutionAttributeStateEnabled;

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

        // Synchronize the stream to ensure the GMO is populated and newData is set, if necessary
        if (db.inputs.accumulateLidarScan())
        {
            isaacsim::core::includes::ScopedDevice scopedDevice(cudaDevice);
            CUDA_CHECK(cudaStreamSynchronize((cudaStream_t)db.inputs.cudaStream()));
            if (!db.inputs.newData())
            {
                CARB_LOG_INFO("IsaacExtractRTXSensorPointCloud: newData input is false. Skipping execution.");
                return false;
            }
        }

        // Test if input data pointer is nullptr before we initialize the node, to avoid incorrect CUDA memory
        // allocation
        void* dataPtr = reinterpret_cast<void*>(db.inputs.dataPtr());
        if (!dataPtr)
        {
            CARB_LOG_INFO("IsaacExtractRTXSensorPointCloud: dataPtr input is empty. Skipping execution.");
            return false;
        }

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

        // Determine if input data pointer is on device or host
        if ((cudaStream_t)db.inputs.cudaStream() != state.m_deviceBuffers.cudaStream)
        {
            CARB_LOG_ERROR(
                "IsaacExtractRTXSensorPointCloud: dataPtr input stream changed mid-execution. This is not supported. Node will not execute.");
            return false;
        }

        // Fill the point cloud buffer with the valid points
        state.m_deviceBuffers.fillPointCloudBuffer(dataPtr, state.m_numValidPointsHost, state.m_frameAtEnd);

        // Populate outputs
        if (state.m_deviceBuffers.gmoOnDevice)
        {
            // Simply set the output buffer pointer to the point cloud buffer address
            db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.m_deviceBuffers.pointCloudBuffer.data());
            db.outputs.cudaDeviceIndex() = state.m_deviceBuffers.cudaDevice;
        }
        else
        {
            // Synchronize the stream to ensure the point cloud buffer is populated
            CUDA_CHECK(cudaStreamSynchronize(state.m_deviceBuffers.cudaStream));
            // Copy the point cloud buffer to the host
            if (state.hostPcBuffer.size() < state.m_deviceBuffers.bufferSize)
            {
                state.hostPcBuffer.resize(state.m_deviceBuffers.bufferSize);
            }
            CUDA_CHECK(cudaMemcpyAsync(state.hostPcBuffer.data(), state.m_deviceBuffers.pointCloudBuffer.data(),
                                       state.m_numValidPointsHost * sizeof(float3), cudaMemcpyDeviceToHost,
                                       state.m_deviceBuffers.cudaStream));
            db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.hostPcBuffer.data());
            db.outputs.cudaDeviceIndex() = -1;
        }

        // Resolve transform from sensor pose at end of scan as 4x4 matrix
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        getTransformFromSensorPose(state.m_frameAtEnd, matrixOutput);
        db.outputs.cudaStream() = db.inputs.cudaStream();
        db.outputs.bufferSize() = state.m_numValidPointsHost * sizeof(float3);
        db.outputs.width() = static_cast<uint32_t>(state.m_numValidPointsHost);
        db.outputs.height() = 1;
        db.outputs.accumulatedData() = !db.inputs.accumulateLidarScan() || db.inputs.newData();
        return true;
    }

private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };
    IsaacExtractRTXSensorPointCloudDeviceBuffers m_deviceBuffers;
    isaacsim::core::includes::HostBufferBase<float3> hostPcBuffer;
    size_t m_numValidPointsHost{ 0 };
    omni::sensors::FrameAtTime m_frameAtEnd;
};

REGISTER_OGN_NODE()
} // rtx
} // sensors
} // isaacsim
