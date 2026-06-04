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

#include "Ros2SrtxImagePublisher.h"

#include <carb/profiler/Profile.h>
#include <carb/settings/ISettings.h>

#include <algorithm>
#include <cstring>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

namespace
{

/// Map an SRTX format string to a ROS image encoding.
std::string srtxFormatToEncoding(const char* format)
{
    if (!format)
    {
        return "rgba8";
    }

    const std::string fmt(format);
    if (fmt == "RGBA8_UNORM" || fmt == "RGBA8")
    {
        return "rgba8";
    }
    if (fmt == "RGB8_UNORM" || fmt == "RGB8")
    {
        return "rgb8";
    }
    if (fmt == "R32_SFLOAT")
    {
        return "32FC1";
    }
    if (fmt == "R16_SFLOAT" || fmt == "R16_UNORM")
    {
        return "16UC1";
    }
    if (fmt == "BGRA8_UNORM" || fmt == "BGRA8")
    {
        return "bgra8";
    }
    if (fmt == "R8_UNORM" || fmt == "R8")
    {
        return "mono8";
    }

    CARB_LOG_WARN("Ros2SrtxImagePublisher: unknown SRTX format '%s', defaulting to rgba8", format);
    return "rgba8";
}

} // namespace

bool Ros2SrtxImagePublisher::initialize(const std::string& topicName,
                                        const std::string& frameId,
                                        const std::string& nodeNamespace,
                                        uint64_t queueSize,
                                        const std::string& qosProfile)
{
    m_frameId = frameId;

    m_ros2Bridge = carb::getCachedInterface<isaacsim::ros2::core::Ros2Bridge>();
    if (!m_ros2Bridge || !m_ros2Bridge->getStartupStatus())
    {
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to get Ros2Bridge interface");
        return false;
    }

    m_factory = m_ros2Bridge->getFactory();
    if (!m_factory)
    {
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to get Ros2Factory");
        return false;
    }

    auto* contextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle>*>(
        m_ros2Bridge->getDefaultContextHandleAddr());
    if (!contextHandlePtr || !*contextHandlePtr)
    {
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to get default context handle");
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

    std::string nodeName = "srtx_publisher" + topicName;
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
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to create ROS2 node handle");
        return false;
    }

    m_message = m_factory->createImageMessage();
    if (!m_message)
    {
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to create image message");
        return false;
    }

    isaacsim::ros2::core::Ros2QoSProfile qos;
    if (!qosProfile.empty())
    {
        if (!isaacsim::ros2::core::jsonToRos2QoSProfile(qos, qosProfile))
        {
            CARB_LOG_WARN("Ros2SrtxImagePublisher: Invalid QoS profile, using defaults");
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
        CARB_LOG_ERROR("Ros2SrtxImagePublisher: Failed to create publisher for topic '%s'", fullTopicName.c_str());
        return false;
    }

    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    static constexpr char s_kNitrosBridgeEnabled[] = "/exts/isaacsim.ros2.bridge/enable_nitros_bridge";
    m_nitrosBridgeEnabled = settings && settings->getAsBool(s_kNitrosBridgeEnabled);

    if (m_nitrosBridgeEnabled)
    {
        m_nitrosBridgeMessage = m_factory->createNitrosBridgeImageMessage();
        if (m_nitrosBridgeMessage && m_nitrosBridgeMessage->getPtr())
        {
            m_nitrosBridgePublisher =
                m_factory->createPublisher(m_nodeHandle.get(), (fullTopicName + "/nitros_bridge").c_str(),
                                           m_nitrosBridgeMessage->getTypeSupportHandle(), qos);

            if (!m_nitrosBridgePublisher || !m_nitrosBridgePublisher->isValid())
            {
                CARB_LOG_WARN("Ros2SrtxImagePublisher: Failed to create NITROS bridge publisher");
                m_nitrosBridgePublisher.reset();
            }
        }
        else
        {
            m_nitrosBridgeMessage.reset();
        }
    }

    m_initialized = true;
    CARB_LOG_INFO(
        "Ros2SrtxImagePublisher: Initialized for topic '%s' frame_id '%s'", fullTopicName.c_str(), frameId.c_str());
    return true;
}

void Ros2SrtxImagePublisher::publishImage(
    const uint8_t* data, uint32_t width, uint32_t height, const std::string& encoding, size_t bufferSize, double timestamp)
{
    CARB_PROFILE_ZONE(0, "SRTX ROS2 Publish Image");
    if (!m_initialized || !m_publisher || !data)
    {
        return;
    }

    m_message->writeHeader(timestamp, m_frameId);
    m_message->generateBuffer(height, width, encoding, false);

    size_t totalBytes = m_message->getTotalBytes();
    size_t copyBytes = std::min(totalBytes, bufferSize);
    void* bufPtr = m_message->getBufferPtr();
    std::memcpy(bufPtr, data, copyBytes);

    m_publisher->publish(m_message->getPtr());

#if !defined(_WIN32)
    if (m_nitrosBridgePublisher)
    {
        CARB_PROFILE_ZONE(0, "SRTX ROS2 Publish NITROS Bridge Image");

        m_nitrosBridgeMessage->writeHeader(timestamp, m_frameId);
        m_nitrosBridgeMessage->generateBuffer(height, width, encoding);
        size_t nitrosTotalBytes = m_nitrosBridgeMessage->getTotalBytes();

        if (!m_ipcBufferManager)
        {
            if (nitrosTotalBytes == 0)
            {
                CARB_LOG_WARN("Ros2SrtxImagePublisher: NITROS Bridge: zero totalBytes, disabling");
                m_nitrosBridgePublisher.reset();
                return;
            }
            try
            {
                m_ipcBufferManager = std::make_shared<IPCBufferManager>(40, nitrosTotalBytes);
            }
            catch (const std::runtime_error&)
            {
                CARB_LOG_WARN("Ros2SrtxImagePublisher: NITROS Bridge: IPCBufferManager init failed, disabling");
                m_nitrosBridgePublisher.reset();
                return;
            }
        }

        void* ipcBufPtr = reinterpret_cast<void*>(m_ipcBufferManager->getCurBufferPtr());
        size_t nitrosCopyBytes = std::min(nitrosTotalBytes, bufferSize);
        cudaMemcpy(ipcBufPtr, data, nitrosCopyBytes, cudaMemcpyHostToDevice);

        m_nitrosBridgeMessage->writeData(m_ipcBufferManager->getCurIpcMemHandle());
        m_ipcBufferManager->next();

        m_nitrosBridgePublisher->publish(m_nitrosBridgeMessage->getPtr());
    }
#endif
}

void Ros2SrtxImagePublisher::trampoline(void* userData,
                                        const uint8_t* data,
                                        size_t dataSize,
                                        const uint32_t* shape,
                                        size_t shapeLen,
                                        const char* format,
                                        const char* outputPath,
                                        double timestamp)
{
    auto* pub = static_cast<Ros2SrtxImagePublisher*>(userData);
    if (!pub || !data || shapeLen < 2)
    {
        return;
    }

    uint32_t height = shape[0];
    uint32_t width = shape[1];
    std::string encoding = srtxFormatToEncoding(format);

    pub->publishImage(data, width, height, encoding, dataSize, timestamp);
}

void Ros2SrtxImagePublisher::destroy(void* userData)
{
    delete static_cast<Ros2SrtxImagePublisher*>(userData);
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
