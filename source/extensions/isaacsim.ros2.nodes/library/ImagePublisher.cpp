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

#include <isaacsim/ros2/nodes/ImagePublisher.h>

#include <cstring>

using namespace isaacsim::ros2::core;

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

ImagePublisher::~ImagePublisher()
{
    ImagePublisher::reset();
}

const void* ImagePublisher::onCreateMessage()
{
    m_message = m_factory->createImageMessage();
    return m_message ? m_message->getTypeSupportHandle() : nullptr;
}

bool ImagePublisher::prepareMessage(
    uint32_t width, uint32_t height, const std::string& encoding, double timestamp, bool usePinnedMemory)
{
    if (width == 0 || height == 0)
    {
        CARB_LOG_ERROR("ImagePublisher: width %u or height %u is not valid", width, height);
        return false;
    }

    m_message->writeHeader(timestamp, m_frameId);
    m_message->generateBuffer(height, width, encoding, usePinnedMemory);
    return true;
}

bool ImagePublisher::prepareFromHost(
    const void* data, size_t dataSize, uint32_t width, uint32_t height, const std::string& encoding, double timestamp)
{
    CARB_PROFILE_ZONE(1, "[IsaacSim] ImagePublisher::prepareFromHost");

    if (!m_publisher)
    {
        return false;
    }

    m_tasks.wait();

    if (!shouldPublish())
    {
        return false;
    }

    if (!prepareMessage(width, height, encoding, timestamp, false))
    {
        return false;
    }

    size_t totalBytes = m_message->getTotalBytes();
    void* bufPtr = m_message->getBufferPtr();

    if (dataSize != totalBytes)
    {
        CARB_LOG_ERROR("ImagePublisher: buffer size %zu does not match expected %zu", dataSize, totalBytes);
        return false;
    }

    std::memcpy(bufPtr, data, totalBytes);
    return true;
}

bool ImagePublisher::publishFromHost(
    const void* data, size_t dataSize, uint32_t width, uint32_t height, const std::string& encoding, double timestamp)
{
    if (!prepareFromHost(data, dataSize, width, height, encoding, timestamp))
    {
        return false;
    }
    send();
    return true;
}

bool ImagePublisher::prepareFromDevice(const void* devicePtr,
                                       size_t bufferSize,
                                       uint32_t width,
                                       uint32_t height,
                                       const std::string& encoding,
                                       double timestamp,
                                       int cudaDeviceIndex,
                                       carb::Format format)
{
    CARB_PROFILE_ZONE(1, "[IsaacSim] ImagePublisher::prepareFromDevice");

    if (!m_publisher)
    {
        return false;
    }

    m_tasks.wait();

    if (!shouldPublish())
    {
        return false;
    }

    if (!prepareMessage(width, height, encoding, timestamp, true))
    {
        return false;
    }

    size_t totalBytes = m_message->getTotalBytes();
    void* bufPtr = m_message->getBufferPtr();

    isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);
    ensureCudaStream(cudaDeviceIndex);

    if (bufferSize == 0)
    {
        cudaArray_t levelArray = nullptr;
        CUDA_CHECK(cudaGetMipmappedArrayLevel(
            &levelArray, reinterpret_cast<cudaMipmappedArray_t>(const_cast<void*>(devicePtr)), 0));

        switch (format)
        {
        case carb::Format::eR32_SFLOAT:
            if (static_cast<size_t>(width) * height * sizeof(float) != totalBytes)
            {
                CARB_LOG_ERROR("ImagePublisher: totalBytes mismatch for eR32_SFLOAT");
                return false;
            }
            CUDA_CHECK(cudaMemcpy2DFromArrayAsync(bufPtr, width * sizeof(float), levelArray, 0, 0,
                                                  width * sizeof(float), height, cudaMemcpyDeviceToHost, m_stream));
            CUDA_CHECK(cudaStreamSynchronize(m_stream));
            break;
        default:
            CARB_LOG_ERROR("ImagePublisher: unsupported texture format %d", static_cast<int>(format));
            return false;
        }
    }
    else
    {
        CUDA_CHECK(cudaMemcpyAsync(bufPtr, devicePtr, bufferSize, cudaMemcpyDeviceToHost, m_stream));
        CUDA_CHECK(cudaStreamSynchronize(m_stream));
    }

    return true;
}

bool ImagePublisher::publishFromDevice(const void* devicePtr,
                                       size_t bufferSize,
                                       uint32_t width,
                                       uint32_t height,
                                       const std::string& encoding,
                                       double timestamp,
                                       int cudaDeviceIndex,
                                       carb::Format format)
{
    if (!prepareFromDevice(devicePtr, bufferSize, width, height, encoding, timestamp, cudaDeviceIndex, format))
    {
        return false;
    }
    send();
    return true;
}

void ImagePublisher::ensureCudaStream(int cudaDeviceIndex)
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

void ImagePublisher::destroyCudaStream()
{
    if (!m_streamNotCreated)
    {
        isaacsim::core::includes::ScopedDevice scopedDev(m_streamDevice);
        CUDA_CHECK(cudaStreamDestroy(m_stream));
        m_streamDevice = -1;
        m_streamNotCreated = true;
    }
}

void ImagePublisher::reset()
{
    m_tasks.wait();
    destroyCudaStream();
    m_message.reset();
    PublisherBase::reset();
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
