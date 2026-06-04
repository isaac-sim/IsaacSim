// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Ros2SrtxLidarPublisher.h"

#include <carb/profiler/Profile.h>

#include <GenericModelOutputTypes.h>
#include <cmath>
#include <cstring>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

namespace
{

static constexpr size_t g_kGmoHeaderSize = sizeof(omni::sensors::GenericModelOutput);
static constexpr size_t g_kGmoMinSize = g_kGmoHeaderSize + sizeof(int32_t) + sizeof(float) * 4 + sizeof(uint8_t);
static constexpr float g_kDegToRad = static_cast<float>(M_PI) / 180.0f;

} // namespace

bool Ros2SrtxLidarPublisher::initialize(const std::string& topicName,
                                        const std::string& frameId,
                                        const std::string& nodeNamespace,
                                        uint64_t queueSize,
                                        const std::string& qosProfile)
{
    m_frameId = frameId;

    m_ros2Bridge = carb::getCachedInterface<isaacsim::ros2::core::Ros2Bridge>();
    if (!m_ros2Bridge || !m_ros2Bridge->getStartupStatus())
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to get Ros2Bridge interface");
        return false;
    }

    m_factory = m_ros2Bridge->getFactory();
    if (!m_factory)
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to get Ros2Factory");
        return false;
    }

    auto* contextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle>*>(
        m_ros2Bridge->getDefaultContextHandleAddr());
    if (!contextHandlePtr || !*contextHandlePtr)
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to get default context handle");
        return false;
    }

    std::string ns = nodeNamespace;
    while (!ns.empty() && !std::isalnum(static_cast<unsigned char>(ns.front())))
    {
        ns.erase(ns.begin());
    }
    while (!ns.empty() && !std::isalnum(static_cast<unsigned char>(ns.back())))
    {
        ns.pop_back();
    }
    if (!ns.empty())
    {
        ns = "/" + ns;
    }

    std::string nodeName = "srtx_lidar_publisher" + topicName;
    for (auto& c : nodeName)
    {
        if (c == '/' || c == '-')
        {
            c = '_';
        }
    }

    m_nodeHandle = m_factory->createNodeHandle(nodeName.c_str(), ns.c_str(), contextHandlePtr->get());
    if (!m_nodeHandle || !m_nodeHandle->getNode())
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to create ROS2 node handle");
        return false;
    }

    m_message = m_factory->createPointCloudMessage();
    if (!m_message)
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to create point cloud message");
        return false;
    }

    isaacsim::ros2::core::Ros2QoSProfile qos;
    if (!qosProfile.empty())
    {
        if (!isaacsim::ros2::core::jsonToRos2QoSProfile(qos, qosProfile))
        {
            CARB_LOG_WARN("Ros2SrtxLidarPublisher: Invalid QoS profile, using defaults");
        }
    }
    else
    {
        qos.depth = queueSize;
    }

    std::string trimmedTopic = topicName;
    while (!trimmedTopic.empty() && !std::isalnum(static_cast<unsigned char>(trimmedTopic.front())))
    {
        trimmedTopic.erase(trimmedTopic.begin());
    }
    while (!trimmedTopic.empty() && !std::isalnum(static_cast<unsigned char>(trimmedTopic.back())))
    {
        trimmedTopic.pop_back();
    }
    std::string fullTopicName = ns + "/" + trimmedTopic;

    m_publisher =
        m_factory->createPublisher(m_nodeHandle.get(), fullTopicName.c_str(), m_message->getTypeSupportHandle(), qos);
    if (!m_publisher || !m_publisher->isValid())
    {
        CARB_LOG_ERROR("Ros2SrtxLidarPublisher: Failed to create publisher for topic '%s'", fullTopicName.c_str());
        return false;
    }

    m_initialized = true;
    CARB_LOG_INFO(
        "Ros2SrtxLidarPublisher: Initialized for topic '%s' frame_id '%s'", fullTopicName.c_str(), frameId.c_str());
    return true;
}

void Ros2SrtxLidarPublisher::publishData(const uint8_t* data, size_t dataSize, double timestamp)
{
    CARB_PROFILE_ZONE(0, "SRTX ROS2 LidarData Publish");
    if (!m_initialized || !m_publisher || !data)
    {
        return;
    }

    if (dataSize < g_kGmoMinSize)
    {
        CARB_LOG_WARN("Ros2SrtxLidarPublisher: Buffer too small (%zu bytes)", dataSize);
        return;
    }

    const auto* gmo = reinterpret_cast<const omni::sensors::GenericModelOutput*>(data);

    if (gmo->magicNumber != omni::sensors::MAGIC_NUMBER_GMO)
    {
        CARB_LOG_WARN("Ros2SrtxLidarPublisher: Invalid GMO magic number: 0x%08X", gmo->magicNumber);
        return;
    }

    const uint32_t numElements = gmo->numElements;
    if (numElements == 0)
    {
        return;
    }

    if (dataSize < g_kGmoMinSize)
    {
        CARB_LOG_WARN(
            "Ros2SrtxLidarPublisher: Buffer has %u elements but data too small (%zu bytes)", numElements, dataSize);
        return;
    }

    size_t offset = g_kGmoHeaderSize;
    offset += sizeof(int32_t) * numElements; // timeOffsetNs
    const float* azimuth = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const float* elevation = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const float* distance = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const float* intensity = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const uint8_t* flags = reinterpret_cast<const uint8_t*>(data + offset);

    size_t requiredSize = offset + sizeof(uint8_t) * numElements;
    if (dataSize < requiredSize)
    {
        CARB_LOG_WARN("Ros2SrtxLidarPublisher: Buffer %zu too small for %u elements (need %zu)", dataSize, numElements,
                      requiredSize);
        return;
    }

    const bool isSpherical = (gmo->elementsCoordsType == omni::sensors::CoordsType::SPHERICAL);

    m_cartesianXYZ.resize(numElements * 3);
    m_intensity.resize(numElements);
    size_t numValid = 0;

    {
        CARB_PROFILE_ZONE(0, "SRTX LidarData Convert");
        for (uint32_t i = 0; i < numElements; ++i)
        {
            if (!(flags[i] & omni::sensors::ElementFlags::VALID))
            {
                continue;
            }

            float x, y, z;
            if (isSpherical)
            {
                float azRad = azimuth[i] * g_kDegToRad;
                float elRad = elevation[i] * g_kDegToRad;
                float cosEl = std::cos(elRad);
                float rangeXY = distance[i] * cosEl;
                x = rangeXY * std::cos(azRad);
                y = rangeXY * std::sin(azRad);
                z = distance[i] * std::sin(elRad);
            }
            else
            {
                x = azimuth[i];
                y = elevation[i];
                z = distance[i];
            }

            size_t base = numValid * 3;
            m_cartesianXYZ[base + 0] = x;
            m_cartesianXYZ[base + 1] = y;
            m_cartesianXYZ[base + 2] = z;
            m_intensity[numValid] = intensity[i];
            ++numValid;
        }
    }

    if (numValid == 0)
    {
        return;
    }

    size_t xyzBufferSize = numValid * 3 * sizeof(float);

    m_message->setUsePinnedBuffer(false);
    m_message->generateBuffer(timestamp, m_frameId, xyzBufferSize, m_intensity.data(), nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    size_t pointStep = m_message->getPointStep();
    size_t totalBytes = m_message->getTotalBytes();
    uint8_t* bufPtr = reinterpret_cast<uint8_t*>(m_message->getBufferPtr());

    if (pointStep == 3 * sizeof(float))
    {
        size_t copyBytes = std::min(totalBytes, xyzBufferSize);
        std::memcpy(bufPtr, m_cartesianXYZ.data(), copyBytes);
    }
    else
    {
        for (size_t i = 0; i < numValid; ++i)
        {
            uint8_t* dst = bufPtr + i * pointStep;
            std::memcpy(dst, &m_cartesianXYZ[i * 3], 3 * sizeof(float));
            std::memcpy(dst + 3 * sizeof(float), &m_intensity[i], sizeof(float));
        }
    }

    m_publisher->publish(m_message->getPtr());
}

void Ros2SrtxLidarPublisher::trampoline(void* userData,
                                        const uint8_t* data,
                                        size_t dataSize,
                                        const uint32_t* shape,
                                        size_t shapeLen,
                                        const char* format,
                                        const char* outputPath,
                                        double timestamp)
{
    auto* pub = static_cast<Ros2SrtxLidarPublisher*>(userData);
    if (!pub || !data)
    {
        return;
    }
    pub->publishData(data, dataSize, timestamp);
}

void Ros2SrtxLidarPublisher::destroy(void* userData)
{
    delete static_cast<Ros2SrtxLidarPublisher*>(userData);
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
