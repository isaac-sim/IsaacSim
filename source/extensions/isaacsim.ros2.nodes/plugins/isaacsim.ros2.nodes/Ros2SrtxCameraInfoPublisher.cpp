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

#include "Ros2SrtxCameraInfoPublisher.h"

#include <carb/profiler/Profile.h>

#include <cctype>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

namespace
{

std::string sanitizeNamespace(std::string ns)
{
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
    return ns;
}

std::string sanitizeTopic(std::string topicName)
{
    while (!topicName.empty() && !std::isalnum(static_cast<unsigned char>(topicName.front())))
    {
        topicName.erase(topicName.begin());
    }
    while (!topicName.empty() && !std::isalnum(static_cast<unsigned char>(topicName.back())))
    {
        topicName.pop_back();
    }
    return topicName;
}

std::string makeNodeName(const std::string& topicName)
{
    std::string nodeName = "srtx_camera_info_publisher" + topicName;
    for (auto& c : nodeName)
    {
        if (c == '/' || c == '-')
        {
            c = '_';
        }
    }
    return nodeName;
}

} // namespace

bool Ros2SrtxCameraInfoPublisher::initialize(const std::string& topicName,
                                             const std::string& frameId,
                                             const std::string& nodeNamespace,
                                             uint64_t queueSize,
                                             const std::string& qosProfile)
{
    CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Use the overload with camera calibration parameters");
    return false;
}

bool Ros2SrtxCameraInfoPublisher::initialize(const std::string& topicName,
                                             const std::string& frameId,
                                             const std::string& nodeNamespace,
                                             uint64_t queueSize,
                                             const std::string& qosProfile,
                                             uint32_t width,
                                             uint32_t height,
                                             const std::string& distortionModel,
                                             const std::vector<double>& k,
                                             const std::vector<double>& r,
                                             const std::vector<double>& p,
                                             const std::vector<double>& d)
{
    m_frameId = frameId;
    m_width = width;
    m_height = height;
    m_distortionModel = distortionModel;
    m_k = k;
    m_r = r;
    m_p = p;
    m_d = d;

    m_ros2Bridge = carb::getCachedInterface<isaacsim::ros2::core::Ros2Bridge>();
    if (!m_ros2Bridge || !m_ros2Bridge->getStartupStatus())
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to get Ros2Bridge interface");
        return false;
    }

    m_factory = m_ros2Bridge->getFactory();
    if (!m_factory)
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to get Ros2Factory");
        return false;
    }

    auto* contextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle>*>(
        m_ros2Bridge->getDefaultContextHandleAddr());
    if (!contextHandlePtr || !*contextHandlePtr)
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to get default context handle");
        return false;
    }

    const std::string ns = sanitizeNamespace(nodeNamespace);
    const std::string nodeName = makeNodeName(topicName);

    m_nodeHandle = m_factory->createNodeHandle(nodeName.c_str(), ns.c_str(), contextHandlePtr->get());
    if (!m_nodeHandle || !m_nodeHandle->getNode())
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to create ROS2 node handle");
        return false;
    }

    m_message = m_factory->createCameraInfoMessage();
    if (!m_message)
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to create camera info message");
        return false;
    }

    isaacsim::ros2::core::Ros2QoSProfile qos;
    if (!qosProfile.empty())
    {
        if (!isaacsim::ros2::core::jsonToRos2QoSProfile(qos, qosProfile))
        {
            CARB_LOG_WARN("Ros2SrtxCameraInfoPublisher: Invalid QoS profile, using defaults");
        }
    }
    else
    {
        qos.depth = queueSize;
    }

    const std::string fullTopicName = ns + "/" + sanitizeTopic(topicName);
    m_publisher =
        m_factory->createPublisher(m_nodeHandle.get(), fullTopicName.c_str(), m_message->getTypeSupportHandle(), qos);
    if (!m_publisher || !m_publisher->isValid())
    {
        CARB_LOG_ERROR("Ros2SrtxCameraInfoPublisher: Failed to create publisher for topic '%s'", fullTopicName.c_str());
        return false;
    }

    m_initialized = true;
    CARB_LOG_INFO("Ros2SrtxCameraInfoPublisher: Initialized for topic '%s' frame_id '%s'", fullTopicName.c_str(),
                  frameId.c_str());
    return true;
}

void Ros2SrtxCameraInfoPublisher::publishCameraInfo(double timestamp)
{
    CARB_PROFILE_ZONE(0, "SRTX ROS2 Publish CameraInfo");
    if (!m_initialized || !m_publisher)
    {
        return;
    }

    m_message->writeHeader(timestamp, m_frameId);
    m_message->writeResolution(m_height, m_width);
    if (!m_k.empty())
    {
        m_message->writeIntrinsicMatrix(m_k.data(), m_k.size());
    }
    if (!m_r.empty())
    {
        m_message->writeRectificationMatrix(m_r.data(), m_r.size());
    }
    if (!m_p.empty())
    {
        m_message->writeProjectionMatrix(m_p.data(), m_p.size());
    }
    m_message->writeDistortionParameters(m_d, m_distortionModel);

    m_publisher->publish(m_message->getPtr());
}

void Ros2SrtxCameraInfoPublisher::trampoline(void* userData,
                                             const uint8_t* data,
                                             size_t dataSize,
                                             const uint32_t* shape,
                                             size_t shapeLen,
                                             const char* format,
                                             const char* outputPath,
                                             double timestamp)
{
    auto* pub = static_cast<Ros2SrtxCameraInfoPublisher*>(userData);
    if (!pub)
    {
        return;
    }
    pub->publishCameraInfo(timestamp);
}

void Ros2SrtxCameraInfoPublisher::destroy(void* userData)
{
    delete static_cast<Ros2SrtxCameraInfoPublisher*>(userData);
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
