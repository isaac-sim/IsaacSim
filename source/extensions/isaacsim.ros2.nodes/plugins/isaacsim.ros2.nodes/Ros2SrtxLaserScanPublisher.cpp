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

#include "Ros2SrtxLaserScanPublisher.h"

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

bool Ros2SrtxLaserScanPublisher::initialize(const std::string& topicName,
                                            const std::string& frameId,
                                            const std::string& nodeNamespace,
                                            uint64_t queueSize,
                                            const std::string& qosProfile)
{
    CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Use the overload with scan metadata parameters");
    return false;
}

bool Ros2SrtxLaserScanPublisher::initialize(const std::string& topicName,
                                            const std::string& frameId,
                                            const std::string& nodeNamespace,
                                            uint64_t queueSize,
                                            const std::string& qosProfile,
                                            float azimuthRangeStart,
                                            float azimuthRangeEnd,
                                            float depthRangeMin,
                                            float depthRangeMax,
                                            float rotationRate,
                                            float horizontalResolution,
                                            float horizontalFov)
{
    m_frameId = frameId;
    m_azimuthRangeStart = azimuthRangeStart;
    m_azimuthRangeEnd = azimuthRangeEnd;
    m_depthRangeMin = depthRangeMin;
    m_depthRangeMax = depthRangeMax;
    m_rotationRate = rotationRate;
    m_horizontalResolution = horizontalResolution;
    m_horizontalFov = horizontalFov;

    if (m_horizontalResolution <= 0.0f)
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: horizontalResolution must be > 0 (got %f)", m_horizontalResolution);
        return false;
    }
    m_numOutputElements = static_cast<size_t>(m_horizontalFov / m_horizontalResolution);
    if (m_numOutputElements == 0)
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Computed 0 output elements from fov=%f res=%f", m_horizontalFov,
                       m_horizontalResolution);
        return false;
    }

    m_ros2Bridge = carb::getCachedInterface<isaacsim::ros2::core::Ros2Bridge>();
    if (!m_ros2Bridge || !m_ros2Bridge->getStartupStatus())
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to get Ros2Bridge interface");
        return false;
    }

    m_factory = m_ros2Bridge->getFactory();
    if (!m_factory)
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to get Ros2Factory");
        return false;
    }

    auto* contextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle>*>(
        m_ros2Bridge->getDefaultContextHandleAddr());
    if (!contextHandlePtr || !*contextHandlePtr)
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to get default context handle");
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

    std::string nodeName = "srtx_laser_scan_publisher" + topicName;
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
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to create ROS2 node handle");
        return false;
    }

    m_message = m_factory->createLaserScanMessage();
    if (!m_message)
    {
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to create laser scan message");
        return false;
    }

    isaacsim::ros2::core::Ros2QoSProfile qos;
    if (!qosProfile.empty())
    {
        if (!isaacsim::ros2::core::jsonToRos2QoSProfile(qos, qosProfile))
        {
            CARB_LOG_WARN("Ros2SrtxLaserScanPublisher: Invalid QoS profile, using defaults");
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
        CARB_LOG_ERROR("Ros2SrtxLaserScanPublisher: Failed to create publisher for topic '%s'", fullTopicName.c_str());
        return false;
    }

    m_initialized = true;
    CARB_LOG_INFO(
        "Ros2SrtxLaserScanPublisher: Initialized for topic '%s' frame_id '%s'", fullTopicName.c_str(), frameId.c_str());
    return true;
}

void Ros2SrtxLaserScanPublisher::publishData(const uint8_t* data, size_t dataSize, double timestamp)
{
    CARB_PROFILE_ZONE(0, "SRTX ROS2 LaserScan Publish");
    if (!m_initialized || !m_publisher || !data)
    {
        return;
    }

    if (dataSize < g_kGmoMinSize)
    {
        CARB_LOG_WARN("Ros2SrtxLaserScanPublisher: Buffer too small (%zu bytes)", dataSize);
        return;
    }

    const auto* gmo = reinterpret_cast<const omni::sensors::GenericModelOutput*>(data);

    if (gmo->magicNumber != omni::sensors::MAGIC_NUMBER_GMO)
    {
        CARB_LOG_WARN("Ros2SrtxLaserScanPublisher: Invalid GMO magic number: 0x%08X", gmo->magicNumber);
        return;
    }

    const uint32_t numElements = gmo->numElements;
    if (numElements == 0)
    {
        return;
    }

    size_t offset = g_kGmoHeaderSize;
    offset += sizeof(int32_t) * numElements; // timeOffsetNs
    const float* azimuth = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    // skip elevation
    offset += sizeof(float) * numElements;
    const float* distance = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const float* intensity = reinterpret_cast<const float*>(data + offset);
    offset += sizeof(float) * numElements;
    const uint8_t* flags = reinterpret_cast<const uint8_t*>(data + offset);

    size_t requiredSize = offset + sizeof(uint8_t) * numElements;
    if (dataSize < requiredSize)
    {
        CARB_LOG_WARN("Ros2SrtxLaserScanPublisher: Buffer %zu too small for %u elements (need %zu)", dataSize,
                      numElements, requiredSize);
        return;
    }

    const bool isSpherical = (gmo->elementsCoordsType == omni::sensors::CoordsType::SPHERICAL);

    pxr::GfVec2f azimuthRange(m_azimuthRangeStart, m_azimuthRangeEnd - m_horizontalResolution);
    pxr::GfVec2f depthRange(m_depthRangeMin, m_depthRangeMax);

    m_message->writeHeader(timestamp, m_frameId);
    m_message->writeData(azimuthRange, m_rotationRate, depthRange, m_horizontalResolution, m_horizontalFov);
    m_message->generateBuffers(m_numOutputElements);

    std::vector<float>& rangeData = m_message->getRangeData();
    std::vector<float>& intensitiesData = m_message->getIntensitiesData();

    std::fill(rangeData.begin(), rangeData.end(), -1.0f);
    std::fill(intensitiesData.begin(), intensitiesData.end(), 0.0f);

    for (uint32_t i = 0; i < numElements; ++i)
    {
        if (!(flags[i] & omni::sensors::ElementFlags::VALID))
        {
            continue;
        }

        float az = isSpherical ? azimuth[i] : 0.0f;
        float dist = distance[i];
        float inten = intensity[i];

        size_t outIdx = static_cast<size_t>((az - m_azimuthRangeStart) / m_horizontalResolution);
        if (outIdx >= m_numOutputElements)
        {
            outIdx = m_numOutputElements - 1;
        }

        rangeData[outIdx] = dist;
        intensitiesData[outIdx] = inten;
    }

    m_publisher->publish(m_message->getPtr());
}

void Ros2SrtxLaserScanPublisher::trampoline(void* userData,
                                            const uint8_t* data,
                                            size_t dataSize,
                                            const uint32_t* shape,
                                            size_t shapeLen,
                                            const char* format,
                                            const char* outputPath,
                                            double timestamp)
{
    auto* pub = static_cast<Ros2SrtxLaserScanPublisher*>(userData);
    if (!pub || !data)
    {
        return;
    }
    pub->publishData(data, dataSize, timestamp);
}

void Ros2SrtxLaserScanPublisher::destroy(void* userData)
{
    delete static_cast<Ros2SrtxLaserScanPublisher*>(userData);
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
