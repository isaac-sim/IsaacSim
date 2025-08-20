// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "GenericModelOutput.h"
#include "IsaacSimSensorsRTXCuda.cuh"
#include "LidarConfigHelper.h"
#include "LidarMetaData.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/Buffer.h"
#include "isaacsim/core/includes/ScopedCudaDevice.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/tasking/ITasking.h>

#include <OgnIsaacCreateRTXLidarScanBufferDatabase.h>
#include <cstdint>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacCreateRTXLidarScanBuffer : public isaacsim::core::includes::BaseResetNode
{
private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };

    size_t m_maxPoints{ 0 };
    size_t m_currentReturnCount{ 0 };
    size_t m_currentBuffer{ 0 };
    size_t m_nextBuffer{ 1 };
    size_t m_totalElements{ 0 };

    size_t m_numStreams{ 25 }; // 2 buffers for each output type, plus 1 for the point cloud

    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> distanceBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> intensityBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> azimuthBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> elevationBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<int32_t>, 2> deltaTimesBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<int32_t>, 2> timestampBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> emitterIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> materialIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> objectIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float3>, 2> normalBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float3>, 2> velocityBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint8_t>, 2> flagsBuffers;

    bool m_outputAzimuth{ false };
    bool m_outputElevation{ false };
    bool m_outputDistance{ false };
    bool m_outputIntensity{ false };
    bool m_outputTimestamp{ false };
    bool m_outputEmitterId{ false };
    bool m_outputMaterialId{ false };
    bool m_outputObjectId{ false };
    bool m_outputNormal{ false };
    bool m_outputVelocity{ false };

    isaacsim::core::includes::DeviceBufferBase<size_t> indicesBuffer;
    isaacsim::core::includes::DeviceBufferBase<size_t> indicesValidBuffer;

    // Dedicated output buffers for the valid points
    isaacsim::core::includes::DeviceBufferBase<float3> pcBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> distanceBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> intensityBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> azimuthBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> elevationBufferValid;
    isaacsim::core::includes::DeviceBufferBase<int32_t> deltaTimesBufferValid;
    isaacsim::core::includes::DeviceBufferBase<int32_t> timestampBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> emitterIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> materialIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> objectIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float3> normalBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float3> velocityBufferValid;

    omni::sensors::GenericModelOutput* hostGMO{ nullptr };
    omni::sensors::LidarAuxiliaryData* hostAuxPoints{ nullptr };

    int* numValidPointsHost{ nullptr };
    int* numValidPointsDevice{ nullptr };

public:
    void reset()
    {
        m_firstFrame = true;
        m_isInitialized = false;
        m_maxPoints = 0;
        m_currentReturnCount = 0;
        m_currentBuffer = 0;
        m_nextBuffer = 1;
        m_totalElements = 0;
        CUDA_CHECK(cudaFreeHost(hostGMO));
        CUDA_CHECK(cudaFreeHost(hostAuxPoints));
    }

    bool initialize(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "IsaacCreateRTXLidarScanBuffer initialize");

        omni::sensors::lidar::LidarMetaData* lidarMetaData =
            reinterpret_cast<omni::sensors::lidar::LidarMetaData*>(db.inputs.metadataPtr());
        m_maxPoints = lidarMetaData->maxPoints;

        int cudaDeviceIndex = db.inputs.cudaDeviceIndex() == -1 ? 0 : db.inputs.cudaDeviceIndex();
        isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);

        // Allocate pinned host memory
        CUDA_CHECK(cudaMallocHost(&hostGMO, sizeof(omni::sensors::GenericModelOutput)));
        CUDA_CHECK(cudaMallocHost(&hostAuxPoints, sizeof(omni::sensors::LidarAuxiliaryData)));
        CUDA_CHECK(cudaMallocHost(&numValidPointsHost, sizeof(int)));
        numValidPointsHost[0] = 0;

        // Get auxiliary data structure to set output flags
        CUDA_CHECK(cudaMemcpy(hostGMO, reinterpret_cast<void*>(db.inputs.dataPtr()),
                              sizeof(omni::sensors::GenericModelOutput), cudaMemcpyDeviceToHost));
        auto auxType = hostGMO->auxType;
        if (auxType > omni::sensors::AuxType::NONE)
        {
            CUDA_CHECK(cudaMemcpy(hostAuxPoints, hostGMO->auxiliaryData, sizeof(omni::sensors::LidarAuxiliaryData),
                                  cudaMemcpyDeviceToHost));
        }

        m_outputAzimuth = db.inputs.outputAzimuth();
        m_outputElevation = db.inputs.outputElevation();
        m_outputDistance = db.inputs.outputDistance();
        m_outputIntensity = db.inputs.outputIntensity();
        m_outputTimestamp = db.inputs.outputTimestamp();
        m_outputEmitterId = db.inputs.outputEmitterId() && auxType >= omni::sensors::AuxType::BASIC &&
                            (hostAuxPoints->filledAuxMembers & omni::sensors::LidarAuxHas::EMITTER_ID) ==
                                omni::sensors::LidarAuxHas::EMITTER_ID;
        m_outputMaterialId = db.inputs.outputMaterialId() && auxType >= omni::sensors::AuxType::EXTRA &&
                             (hostAuxPoints->filledAuxMembers & omni::sensors::LidarAuxHas::MAT_ID) ==
                                 omni::sensors::LidarAuxHas::MAT_ID;
        m_outputObjectId = db.inputs.outputObjectId() && auxType >= omni::sensors::AuxType::EXTRA &&
                           (hostAuxPoints->filledAuxMembers & omni::sensors::LidarAuxHas::OBJ_ID) ==
                               omni::sensors::LidarAuxHas::OBJ_ID;
        m_outputNormal = db.inputs.outputNormal() && auxType >= omni::sensors::AuxType::FULL &&
                         (hostAuxPoints->filledAuxMembers & omni::sensors::LidarAuxHas::HIT_NORMALS) ==
                             omni::sensors::LidarAuxHas::HIT_NORMALS;
        m_outputVelocity = db.inputs.outputVelocity() && auxType >= omni::sensors::AuxType::FULL &&
                           (hostAuxPoints->filledAuxMembers & omni::sensors::LidarAuxHas::VELOCITIES) ==
                               omni::sensors::LidarAuxHas::VELOCITIES;

        // Allocate additional device memory for the number of valid points
        CUDA_CHECK(cudaMalloc(&numValidPointsDevice, sizeof(int)));

        // Allocate device buffers necessary for the point cloud kernel, and any other output buffers that are requested
        for (size_t i = 0; i < 2; ++i)
        {
            azimuthBuffers[i].setDevice(cudaDeviceIndex);
            azimuthBuffers[i].resize(m_maxPoints);
            elevationBuffers[i].setDevice(cudaDeviceIndex);
            elevationBuffers[i].resize(m_maxPoints);
            distanceBuffers[i].setDevice(cudaDeviceIndex);
            distanceBuffers[i].resize(m_maxPoints);
            flagsBuffers[i].setDevice(cudaDeviceIndex);
            flagsBuffers[i].resize(m_maxPoints);
            if (m_outputIntensity)
            {
                intensityBuffers[i].setDevice(cudaDeviceIndex);
                intensityBuffers[i].resize(m_maxPoints);
            }
            if (m_outputTimestamp)
            {
                timestampBuffers[i].setDevice(cudaDeviceIndex);
                timestampBuffers[i].resize(m_maxPoints);
            }
            if (m_outputEmitterId)
            {
                emitterIdBuffers[i].setDevice(cudaDeviceIndex);
                emitterIdBuffers[i].resize(m_maxPoints);
            }
            if (m_outputMaterialId)
            {
                materialIdBuffers[i].setDevice(cudaDeviceIndex);
                materialIdBuffers[i].resize(m_maxPoints);
            }
            if (m_outputObjectId)
            {
                objectIdBuffers[i].setDevice(cudaDeviceIndex);
                objectIdBuffers[i].resize(m_maxPoints);
            }
            if (m_outputNormal)
            {
                normalBuffers[i].setDevice(cudaDeviceIndex);
                normalBuffers[i].resize(m_maxPoints);
            }
            if (m_outputVelocity)
            {
                velocityBuffers[i].setDevice(cudaDeviceIndex);
                velocityBuffers[i].resize(m_maxPoints);
            }
        }

        // Allocate any necessary output buffers
        pcBufferValid.setDevice(cudaDeviceIndex);
        pcBufferValid.resize(m_maxPoints);
        if (m_outputDistance)
        {
            distanceBufferValid.setDevice(cudaDeviceIndex);
            distanceBufferValid.resize(m_maxPoints);
        }
        if (m_outputIntensity)
        {
            intensityBufferValid.setDevice(cudaDeviceIndex);
            intensityBufferValid.resize(m_maxPoints);
        }
        if (m_outputAzimuth)
        {
            azimuthBufferValid.setDevice(cudaDeviceIndex);
            azimuthBufferValid.resize(m_maxPoints);
        }
        if (m_outputElevation)
        {
            elevationBufferValid.setDevice(cudaDeviceIndex);
            elevationBufferValid.resize(m_maxPoints);
        }
        if (m_outputTimestamp)
        {
            timestampBufferValid.setDevice(cudaDeviceIndex);
            timestampBufferValid.resize(m_maxPoints);
        }
        if (m_outputEmitterId)
        {
            emitterIdBufferValid.setDevice(cudaDeviceIndex);
            emitterIdBufferValid.resize(m_maxPoints);
        }
        if (m_outputMaterialId)
        {
            materialIdBufferValid.setDevice(cudaDeviceIndex);
            materialIdBufferValid.resize(m_maxPoints);
        }
        if (m_outputObjectId)
        {
            objectIdBufferValid.setDevice(cudaDeviceIndex);
            objectIdBufferValid.resize(m_maxPoints);
        }
        if (m_outputNormal)
        {
            normalBufferValid.setDevice(cudaDeviceIndex);
            normalBufferValid.resize(m_maxPoints);
        }
        if (m_outputVelocity)
        {
            velocityBufferValid.setDevice(cudaDeviceIndex);
            velocityBufferValid.resize(m_maxPoints);
        }

        indicesBuffer.setDevice(cudaDeviceIndex);
        indicesBuffer.resize(m_maxPoints);
        indicesValidBuffer.setDevice(cudaDeviceIndex);
        indicesValidBuffer.resize(m_maxPoints);

        fillIndices(indicesBuffer.data(), m_maxPoints, cudaDeviceIndex);
        return true;
    }


    static bool compute(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "Create RTX Lidar Scan Buffer");
        // Enable downstream execution by default, so that downstream nodes can do any initialization
        db.outputs.exec() = kExecutionAttributeStateEnabled;
        auto& state = db.perInstanceState<OgnIsaacCreateRTXLidarScanBuffer>();

        // Set default output values
        db.outputs.dataPtr() = 0;
        db.outputs.cudaDeviceIndex() = db.inputs.cudaDeviceIndex();
        db.outputs.bufferSize() = 0;
        db.outputs.width() = 0;
        db.outputs.height() = 1;
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        matrixOutput.SetIdentity();
        db.outputs.indexPtr() = 0; // index only if keepOnlyPositiveDistance
        db.outputs.indexBufferSize() = 0;
        db.outputs.intensityPtr() = 0;
        db.outputs.intensityBufferSize() = 0;
        db.outputs.distancePtr() = 0;
        db.outputs.distanceBufferSize() = 0;
        db.outputs.azimuthPtr() = 0;
        db.outputs.azimuthBufferSize() = 0;
        db.outputs.elevationPtr() = 0;
        db.outputs.elevationBufferSize() = 0;
        db.outputs.objectIdPtr() = 0;
        db.outputs.objectIdBufferSize() = 0;
        db.outputs.velocityPtr() = 0;
        db.outputs.velocityBufferSize() = 0;
        db.outputs.normalPtr() = 0;
        db.outputs.normalBufferSize() = 0;
        db.outputs.timestampPtr() = 0;
        db.outputs.timestampBufferSize() = 0;
        db.outputs.emitterIdPtr() = 0;
        db.outputs.emitterIdBufferSize() = 0;
        db.outputs.materialIdPtr() = 0;
        db.outputs.materialIdBufferSize() = 0;

        db.outputs.numReturnsPerScan() = 0;
        db.outputs.ticksPerScan() = 0;
        db.outputs.numChannels() = 0;
        db.outputs.numEchos() = 0;
        db.outputs.renderProductPath() = db.inputs.renderProductPath();

        if (!db.inputs.dataPtr())
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: dataPtr input is empty. Skipping execution.");
            return false;
        }

        if (!db.inputs.metadataPtr())
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: metadataPtr input is empty. Skipping execution.");
            return false;
        }

        // Get the CUDA device index
        int cudaDeviceIndex = db.inputs.cudaDeviceIndex() == -1 ? 0 : db.inputs.cudaDeviceIndex();
        isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);

        if (state.m_firstFrame)
        {
            state.m_isInitialized = state.initialize(db);
            state.m_firstFrame = false;
        }

        if (!state.m_isInitialized)
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: Failed to initialize correctly. Skipping execution.");
            return false;
        }

        // Copy just the GMO basic structure to the host
        CUDA_CHECK(cudaMemcpy(state.hostGMO, reinterpret_cast<void*>(db.inputs.dataPtr()),
                              sizeof(omni::sensors::GenericModelOutput), cudaMemcpyDeviceToHost));
        size_t numElements = static_cast<size_t>(state.hostGMO->numElements);
        auto auxType = state.hostGMO->auxType;

        if (numElements == 0)
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: No returns in the input buffer. Skipping execution.");
            return false;
        }

        // Copy the auxiliary data structure to the host
        if (auxType > omni::sensors::AuxType::NONE)
        {
            CUDA_CHECK(cudaMemcpy(state.hostAuxPoints, state.hostGMO->auxiliaryData,
                                  sizeof(omni::sensors::LidarAuxiliaryData), cudaMemcpyDeviceToHost));
        }

        // Create streams for all the copy operations
        std::vector<cudaStream_t> cudaStreams(state.m_numStreams);
        for (size_t i = 0; i < state.m_numStreams; ++i)
        {
            CUDA_CHECK(cudaStreamCreate(&cudaStreams[i]));
        }

        // Get indices and element counts for the current and next buffer
        size_t startIndex = state.m_totalElements % state.m_maxPoints;
        size_t numElementsToCopyToCurrentBuffer = std::min(numElements, state.m_maxPoints - startIndex);
        size_t numElementsToCopyToNextBuffer = numElements - numElementsToCopyToCurrentBuffer;

        // Create events to track all the copy operations
        std::vector<cudaEvent_t> copyEvents(state.m_numStreams);
        for (size_t i = 0; i < state.m_numStreams; ++i)
        {
            CUDA_CHECK(cudaEventCreate(&copyEvents[i]));
        }

        // Kick off streams to copy elements into the device buffer
        CUDA_CHECK(cudaMemcpyAsync(state.azimuthBuffers[state.m_currentBuffer].data() + startIndex,
                                   state.hostGMO->elements.x, numElementsToCopyToCurrentBuffer * sizeof(float),
                                   cudaMemcpyDeviceToDevice, cudaStreams[0]));
        CUDA_CHECK(cudaEventRecord(copyEvents[0], cudaStreams[0]));
        CUDA_CHECK(cudaMemcpyAsync(
            state.azimuthBuffers[state.m_nextBuffer].data(), state.hostGMO->elements.x + numElementsToCopyToCurrentBuffer,
            numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice, cudaStreams[1]));
        CUDA_CHECK(cudaEventRecord(copyEvents[1], cudaStreams[1]));

        CUDA_CHECK(cudaMemcpyAsync(state.elevationBuffers[state.m_currentBuffer].data() + startIndex,
                                   state.hostGMO->elements.y, numElementsToCopyToCurrentBuffer * sizeof(float),
                                   cudaMemcpyDeviceToDevice, cudaStreams[2]));
        CUDA_CHECK(cudaEventRecord(copyEvents[2], cudaStreams[2]));
        CUDA_CHECK(cudaMemcpyAsync(state.elevationBuffers[state.m_nextBuffer].data(),
                                   state.hostGMO->elements.y + numElementsToCopyToCurrentBuffer,
                                   numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                   cudaStreams[3]));
        CUDA_CHECK(cudaEventRecord(copyEvents[3], cudaStreams[3]));

        CUDA_CHECK(cudaMemcpyAsync(state.distanceBuffers[state.m_currentBuffer].data() + startIndex,
                                   state.hostGMO->elements.z, numElementsToCopyToCurrentBuffer * sizeof(float),
                                   cudaMemcpyDeviceToDevice, cudaStreams[4]));
        CUDA_CHECK(cudaEventRecord(copyEvents[4], cudaStreams[4]));
        CUDA_CHECK(cudaMemcpyAsync(state.distanceBuffers[state.m_nextBuffer].data(),
                                   state.hostGMO->elements.z + numElementsToCopyToCurrentBuffer,
                                   numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                   cudaStreams[5]));
        CUDA_CHECK(cudaEventRecord(copyEvents[5], cudaStreams[5]));

        CUDA_CHECK(cudaMemcpyAsync(state.flagsBuffers[state.m_currentBuffer].data() + startIndex,
                                   state.hostGMO->elements.flags, numElementsToCopyToCurrentBuffer * sizeof(uint8_t),
                                   cudaMemcpyDeviceToDevice, cudaStreams[6]));
        CUDA_CHECK(cudaEventRecord(copyEvents[6], cudaStreams[6]));
        CUDA_CHECK(cudaMemcpyAsync(state.flagsBuffers[state.m_nextBuffer].data(),
                                   state.hostGMO->elements.flags + numElementsToCopyToCurrentBuffer,
                                   numElementsToCopyToNextBuffer * sizeof(uint8_t), cudaMemcpyDeviceToDevice,
                                   cudaStreams[7]));
        CUDA_CHECK(cudaEventRecord(copyEvents[7], cudaStreams[7]));

        // Optionally copy other data if the user has requested it
        if (state.m_outputIntensity)
        {
            CUDA_CHECK(cudaMemcpyAsync(state.intensityBuffers[state.m_currentBuffer].data() + startIndex,
                                       state.hostGMO->elements.scalar, numElementsToCopyToCurrentBuffer * sizeof(float),
                                       cudaMemcpyDeviceToDevice, cudaStreams[8]));
            CUDA_CHECK(cudaEventRecord(copyEvents[8], cudaStreams[8]));
            CUDA_CHECK(cudaMemcpyAsync(state.intensityBuffers[state.m_nextBuffer].data(),
                                       state.hostGMO->elements.scalar + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                       cudaStreams[9]));
            CUDA_CHECK(cudaEventRecord(copyEvents[9], cudaStreams[9]));
        }
        if (state.m_outputTimestamp)
        {
            CUDA_CHECK(cudaMemcpyAsync(
                state.timestampBuffers[state.m_currentBuffer].data() + startIndex, state.hostGMO->elements.timeOffsetNs,
                numElementsToCopyToCurrentBuffer * sizeof(int32_t), cudaMemcpyDeviceToDevice, cudaStreams[10]));
            CUDA_CHECK(cudaEventRecord(copyEvents[10], cudaStreams[10]));
            CUDA_CHECK(cudaMemcpyAsync(state.timestampBuffers[state.m_nextBuffer].data(),
                                       state.hostGMO->elements.timeOffsetNs + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(int32_t), cudaMemcpyDeviceToDevice,
                                       cudaStreams[11]));
            CUDA_CHECK(cudaEventRecord(copyEvents[11], cudaStreams[11]));
        }
        if (state.m_outputEmitterId)
        {
            CUDA_CHECK(cudaMemcpyAsync(
                state.emitterIdBuffers[state.m_currentBuffer].data() + startIndex, state.hostAuxPoints->emitterId,
                numElementsToCopyToCurrentBuffer * sizeof(uint32_t), cudaMemcpyHostToDevice, cudaStreams[12]));
            CUDA_CHECK(cudaEventRecord(copyEvents[12], cudaStreams[12]));
            CUDA_CHECK(cudaMemcpyAsync(state.emitterIdBuffers[state.m_nextBuffer].data(),
                                       state.hostAuxPoints->emitterId + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(uint32_t), cudaMemcpyHostToDevice,
                                       cudaStreams[13]));
            CUDA_CHECK(cudaEventRecord(copyEvents[13], cudaStreams[13]));
        }
        if (state.m_outputMaterialId)
        {
            CUDA_CHECK(cudaMemcpyAsync(state.materialIdBuffers[state.m_currentBuffer].data() + startIndex,
                                       state.hostAuxPoints->matId, numElementsToCopyToCurrentBuffer * sizeof(uint32_t),
                                       cudaMemcpyHostToDevice, cudaStreams[14]));
            CUDA_CHECK(cudaEventRecord(copyEvents[14], cudaStreams[14]));
            CUDA_CHECK(cudaMemcpyAsync(state.materialIdBuffers[state.m_nextBuffer].data(),
                                       state.hostAuxPoints->matId + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(uint32_t), cudaMemcpyHostToDevice,
                                       cudaStreams[15]));
            CUDA_CHECK(cudaEventRecord(copyEvents[15], cudaStreams[15]));
        }
        if (state.m_outputObjectId)
        {
            CUDA_CHECK(cudaMemcpyAsync(state.objectIdBuffers[state.m_currentBuffer].data() + startIndex,
                                       state.hostAuxPoints->objId, numElementsToCopyToCurrentBuffer * sizeof(uint32_t),
                                       cudaMemcpyHostToDevice, cudaStreams[16]));
            CUDA_CHECK(cudaEventRecord(copyEvents[16], cudaStreams[16]));
            CUDA_CHECK(cudaMemcpyAsync(state.objectIdBuffers[state.m_nextBuffer].data(),
                                       state.hostAuxPoints->objId + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(uint32_t), cudaMemcpyHostToDevice,
                                       cudaStreams[17]));
            CUDA_CHECK(cudaEventRecord(copyEvents[17], cudaStreams[17]));
        }
        if (state.m_outputNormal)
        {
            CUDA_CHECK(cudaMemcpyAsync(
                state.normalBuffers[state.m_currentBuffer].data() + startIndex, state.hostAuxPoints->hitNormals,
                numElementsToCopyToCurrentBuffer * sizeof(float3), cudaMemcpyHostToDevice, cudaStreams[18]));
            CUDA_CHECK(cudaEventRecord(copyEvents[18], cudaStreams[18]));
            CUDA_CHECK(cudaMemcpyAsync(state.normalBuffers[state.m_nextBuffer].data(),
                                       state.hostAuxPoints->hitNormals + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(float3), cudaMemcpyHostToDevice,
                                       cudaStreams[19]));
            CUDA_CHECK(cudaEventRecord(copyEvents[19], cudaStreams[19]));
        }
        if (state.m_outputVelocity)
        {
            CUDA_CHECK(cudaMemcpyAsync(
                state.velocityBuffers[state.m_currentBuffer].data() + startIndex, state.hostAuxPoints->velocities,
                numElementsToCopyToCurrentBuffer * sizeof(float3), cudaMemcpyHostToDevice, cudaStreams[18]));
            CUDA_CHECK(cudaEventRecord(copyEvents[20], cudaStreams[20]));
            CUDA_CHECK(cudaMemcpyAsync(state.velocityBuffers[state.m_nextBuffer].data(),
                                       state.hostAuxPoints->velocities + numElementsToCopyToCurrentBuffer,
                                       numElementsToCopyToNextBuffer * sizeof(float3), cudaMemcpyHostToDevice,
                                       cudaStreams[21]));
            CUDA_CHECK(cudaEventRecord(copyEvents[21], cudaStreams[21]));
        }

        // Sync on the current buffer copy events
        // TODO - move event syncs to individual output buffer checks to avoid waiting for all the data to be copied
        // before the point cloud kernel is called.
        for (size_t i = 0; i < state.m_numStreams; i += 2)
        {
            CUDA_CHECK(cudaStreamSynchronize(cudaStreams[i]));
        }

        if (startIndex + numElementsToCopyToCurrentBuffer == state.m_maxPoints)
        {
            // We've reached the end of the current buffer, and the buffers are guaranteed to be filled.
            // Kick off the point cloud kernel on its own stream

            auto outputEvent = copyEvents[state.m_numStreams - 1];
            auto outputStream = cudaStreams[state.m_numStreams - 1];

            // Select only valid indices
            cudaEvent_t findValidIndicesEvent;
            CUDA_CHECK(cudaEventCreate(&findValidIndicesEvent));
            findValidIndices(state.indicesBuffer.data(), state.indicesValidBuffer.data(), state.numValidPointsDevice,
                             static_cast<int>(state.m_maxPoints), state.flagsBuffers[state.m_currentBuffer].data(),
                             cudaDeviceIndex, outputStream);
            // Copy the number of valid points to the host
            CUDA_CHECK(cudaMemcpyAsync(state.numValidPointsHost, state.numValidPointsDevice, sizeof(int),
                                       cudaMemcpyDeviceToHost, outputStream));
            CUDA_CHECK(cudaEventRecord(findValidIndicesEvent, outputStream));
            CUDA_CHECK(cudaStreamSynchronize(outputStream));

            // Fill the valid cartesian points
            fillValidCartesianPoints(state.azimuthBuffers[state.m_currentBuffer].data(),
                                     state.elevationBuffers[state.m_currentBuffer].data(),
                                     state.distanceBuffers[state.m_currentBuffer].data(), state.pcBufferValid.data(),
                                     state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                     static_cast<int>(state.m_maxPoints), cudaDeviceIndex, outputStream);
            CUDA_CHECK(cudaEventRecord(outputEvent, outputStream));

            // Set requested outputs to the valid point buffers, then asynchronously copy the valid points to the output
            // buffers
            if (state.m_outputAzimuth)
            {
                db.outputs.azimuthPtr() = reinterpret_cast<uint64_t>(state.azimuthBufferValid.data());
                db.outputs.azimuthBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                selectValidPoints(state.azimuthBuffers[state.m_currentBuffer].data(), state.azimuthBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[0]);
            }
            if (state.m_outputElevation)
            {
                db.outputs.elevationPtr() = reinterpret_cast<uint64_t>(state.elevationBufferValid.data());
                db.outputs.elevationBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                selectValidPoints(state.elevationBuffers[state.m_currentBuffer].data(), state.elevationBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[2]);
            }
            if (state.m_outputDistance)
            {
                db.outputs.distancePtr() = reinterpret_cast<uint64_t>(state.distanceBufferValid.data());
                db.outputs.distanceBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                selectValidPoints(state.distanceBuffers[state.m_currentBuffer].data(), state.distanceBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[4]);
            }
            if (state.m_outputIntensity)
            {
                db.outputs.intensityPtr() = reinterpret_cast<uint64_t>(state.intensityBufferValid.data());
                db.outputs.intensityBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                selectValidPoints(state.intensityBuffers[state.m_currentBuffer].data(), state.intensityBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[8]);
            }
            if (state.m_outputTimestamp)
            {
                db.outputs.timestampPtr() = reinterpret_cast<uint64_t>(state.timestampBufferValid.data());
                db.outputs.timestampBufferSize() = state.numValidPointsHost[0] * sizeof(int32_t);
                selectValidPoints(state.timestampBuffers[state.m_currentBuffer].data(), state.timestampBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[10]);
            }
            if (state.m_outputEmitterId)
            {
                db.outputs.emitterIdPtr() = reinterpret_cast<uint64_t>(state.emitterIdBufferValid.data());
                db.outputs.emitterIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                selectValidPoints(state.emitterIdBuffers[state.m_currentBuffer].data(), state.emitterIdBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[12]);
            }
            if (state.m_outputMaterialId)
            {
                db.outputs.materialIdPtr() = reinterpret_cast<uint64_t>(state.materialIdBufferValid.data());
                db.outputs.materialIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                selectValidPoints(state.materialIdBuffers[state.m_currentBuffer].data(),
                                  state.materialIdBufferValid.data(), state.indicesValidBuffer.data(),
                                  state.numValidPointsDevice, static_cast<int>(state.m_maxPoints), cudaDeviceIndex,
                                  cudaStreams[14]);
            }
            if (state.m_outputObjectId)
            {
                db.outputs.objectIdPtr() = reinterpret_cast<uint64_t>(state.objectIdBufferValid.data());
                db.outputs.objectIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                selectValidPoints(state.objectIdBuffers[state.m_currentBuffer].data(), state.objectIdBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[16]);
            }
            if (state.m_outputNormal)
            {
                db.outputs.normalPtr() = reinterpret_cast<uint64_t>(state.normalBufferValid.data());
                db.outputs.normalBufferSize() = state.numValidPointsHost[0] * sizeof(float3);
                selectValidPoints(state.normalBuffers[state.m_currentBuffer].data(), state.normalBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[18]);
            }
            if (state.m_outputVelocity)
            {
                db.outputs.velocityPtr() = reinterpret_cast<uint64_t>(state.velocityBufferValid.data());
                db.outputs.velocityBufferSize() = state.numValidPointsHost[0] * sizeof(float3);
                selectValidPoints(state.velocityBuffers[state.m_currentBuffer].data(), state.velocityBufferValid.data(),
                                  state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                  static_cast<int>(state.m_maxPoints), cudaDeviceIndex, cudaStreams[20]);
            }

            // Synchronize on the output/current streams
            for (size_t i = 0; i < state.m_numStreams; i += 2)
            {
                CUDA_CHECK(cudaStreamSynchronize(cudaStreams[i]));
            }
            CUDA_CHECK(cudaStreamSynchronize(outputStream));

            // Set output buffers
            db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.pcBufferValid.data());
            db.outputs.bufferSize() = state.numValidPointsHost[0] * sizeof(float3);
            db.outputs.width() = state.numValidPointsHost[0];
            db.outputs.height() = 1;

            auto frameEnd = state.hostGMO->frameEnd;
            getTransformFromSensorPose(frameEnd, matrixOutput);

            // Swap the current and next buffers
            std::swap(state.m_currentBuffer, state.m_nextBuffer);
        }

        // Increment the total number of elements written to the buffers
        state.m_totalElements += numElements;


        // Sync on the copy next events
        for (size_t i = 1; i < state.m_numStreams; i += 2)
        {
            CUDA_CHECK(cudaEventSynchronize(copyEvents[i]));
        }

        // Destroy all the copy events
        for (size_t i = 0; i < state.m_numStreams; ++i)
        {
            CUDA_CHECK(cudaEventDestroy(copyEvents[i]));
        }

        // Synchronize on all the streams
        for (size_t i = 0; i < state.m_numStreams; ++i)
        {
            CUDA_CHECK(cudaStreamSynchronize(cudaStreams[i]));
        }

        // Destroy all the streams
        for (size_t i = 0; i < state.m_numStreams; ++i)
        {
            CUDA_CHECK(cudaStreamDestroy(cudaStreams[i]));
        }

        return true;
    }
};


REGISTER_OGN_NODE()
} // namespace rtx
} // namespace sensors
} // namespace isaacsim
