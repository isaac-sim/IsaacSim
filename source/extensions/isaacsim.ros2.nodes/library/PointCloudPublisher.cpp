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


#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <isaacsim/ros2/nodes/PointCloudPublisher.h>

#include <cstring>

using namespace isaacsim::ros2::core;

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

PointCloudPublisher::~PointCloudPublisher()
{
    PointCloudPublisher::reset();
}

const void* PointCloudPublisher::onCreateMessage()
{
    m_message = m_factory->createPointCloudMessage();
    return m_message ? m_message->getTypeSupportHandle() : nullptr;
}

bool PointCloudPublisher::prepareFromHost(const void* data,
                                          size_t dataSize,
                                          double timestamp,
                                          const PointCloudMetadata& metadata)
{
    CARB_PROFILE_ZONE(1, "[IsaacSim] PointCloudPublisher::prepareFromHost");

    if (!m_publisher)
    {
        return false;
    }

    m_tasks.wait();

    if (!shouldPublish())
    {
        return false;
    }

    m_message->setUsePinnedBuffer(false);
    m_message->generateBuffer(timestamp, m_frameId, dataSize, metadata.intensityPtr, metadata.timestampPtr,
                              metadata.emitterIdPtr, metadata.channelIdPtr, metadata.materialIdPtr, metadata.tickIdPtr,
                              metadata.hitNormalPtr, metadata.velocityPtr, metadata.objectIdPtr, metadata.echoIdPtr,
                              metadata.tickStatePtr, metadata.radialVelocityMSPtr);

    std::memcpy(m_message->getBufferPtr(), data, m_message->getTotalBytes());
    return true;
}

bool PointCloudPublisher::publishFromHost(const void* data,
                                          size_t dataSize,
                                          double timestamp,
                                          const PointCloudMetadata& metadata)
{
    if (!prepareFromHost(data, dataSize, timestamp, metadata))
    {
        return false;
    }
    send();
    return true;
}

bool PointCloudPublisher::prepareFromDevice(
    const void* devicePtr, size_t bufferSize, double timestamp, int cudaDeviceIndex, const PointCloudMetadata& metadata)
{
    CARB_PROFILE_ZONE(1, "[IsaacSim] PointCloudPublisher::prepareFromDevice");

    if (!m_publisher)
    {
        return false;
    }

    m_tasks.wait();

    if (!shouldPublish())
    {
        return false;
    }

    if (!initializeCudaProperties(cudaDeviceIndex))
    {
        return false;
    }

    isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);
    ensureCudaStream(cudaDeviceIndex);

    m_message->setUsePinnedBuffer(true);
    m_message->generateBuffer(timestamp, m_frameId, bufferSize, metadata.intensityPtr, metadata.timestampPtr,
                              metadata.emitterIdPtr, metadata.channelIdPtr, metadata.materialIdPtr, metadata.tickIdPtr,
                              metadata.hitNormalPtr, metadata.velocityPtr, metadata.objectIdPtr, metadata.echoIdPtr,
                              metadata.tickStatePtr, metadata.radialVelocityMSPtr);

    void* outputPtr = m_message->getBufferPtr();

    if (m_message->getOrderedFields().empty())
    {
        CUDA_CHECK(cudaMemcpyAsync(outputPtr, devicePtr, bufferSize, cudaMemcpyDeviceToHost, m_stream));
    }
    else
    {
        if (m_deviceBufferSize < m_message->getTotalBytes())
        {
            CUDA_CHECK(cudaFree(m_deviceBuffer));
            m_deviceBuffer = nullptr;
            m_deviceBufferSize = 0;
        }
        if (m_deviceBuffer == nullptr)
        {
            m_deviceBufferSize = m_message->getTotalBytes();
            CUDA_CHECK(cudaMalloc(&m_deviceBuffer, m_deviceBufferSize));
        }
        fillPointCloudBuffer(m_deviceBuffer, reinterpret_cast<const float3*>(devicePtr), m_message->getOrderedFields(),
                             m_message->getPointStep(), m_message->getNumPoints(), m_maxThreadsPerBlock,
                             m_multiProcessorCount, cudaDeviceIndex, m_stream);
        CUDA_CHECK(
            cudaMemcpyAsync(outputPtr, m_deviceBuffer, m_message->getTotalBytes(), cudaMemcpyDeviceToHost, m_stream));
    }
    CUDA_CHECK(cudaStreamSynchronize(m_stream));

    return true;
}

bool PointCloudPublisher::publishFromDevice(
    const void* devicePtr, size_t bufferSize, double timestamp, int cudaDeviceIndex, const PointCloudMetadata& metadata)
{
    if (!prepareFromDevice(devicePtr, bufferSize, timestamp, cudaDeviceIndex, metadata))
    {
        return false;
    }
    send();
    return true;
}

bool PointCloudPublisher::initializeCudaProperties(int cudaDeviceIndex)
{
    if (m_maxThreadsPerBlock > 0)
    {
        return true;
    }

    if (cudaDeviceIndex == -1)
    {
        return true;
    }

    isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);
    try
    {
        cudaDeviceProp prop;
        CUDA_CHECK(cudaGetDeviceProperties(&prop, cudaDeviceIndex));
        m_maxThreadsPerBlock = prop.maxThreadsPerBlock;
        m_multiProcessorCount = prop.multiProcessorCount;
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR(
            "PointCloudPublisher: failed to get CUDA device properties for GPU %d: %s", cudaDeviceIndex, e.what());
        return false;
    }
    return true;
}

void PointCloudPublisher::ensureCudaStream(int cudaDeviceIndex)
{
    if (m_streamDevice != cudaDeviceIndex && !m_streamNotCreated)
    {
        CUDA_CHECK(cudaStreamDestroy(m_stream));
        m_streamNotCreated = true;
        m_streamDevice = -1;
    }
    if (m_streamNotCreated)
    {
        CUDA_CHECK(cudaStreamCreate(&m_stream));
        m_streamNotCreated = false;
        m_streamDevice = cudaDeviceIndex;
    }
}

void PointCloudPublisher::destroyCudaStream()
{
    if (!m_streamNotCreated)
    {
        isaacsim::core::includes::ScopedDevice scopedDev(m_streamDevice);
        CUDA_CHECK(cudaStreamDestroy(m_stream));
        m_streamDevice = -1;
        m_streamNotCreated = true;
    }
}

void PointCloudPublisher::reset()
{
    m_tasks.wait();
    destroyCudaStream();

    if (m_deviceBuffer)
    {
        CUDA_CHECK(cudaFree(m_deviceBuffer));
        m_deviceBuffer = nullptr;
        m_deviceBufferSize = 0;
    }

    m_maxThreadsPerBlock = 0;
    m_multiProcessorCount = 0;

    m_message.reset();
    PublisherBase::reset();
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
