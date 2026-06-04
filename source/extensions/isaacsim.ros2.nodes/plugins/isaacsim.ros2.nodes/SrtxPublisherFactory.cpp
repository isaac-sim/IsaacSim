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
#include "Ros2SrtxImagePublisher.h"
#include "Ros2SrtxLaserScanPublisher.h"
#include "Ros2SrtxLidarPublisher.h"

#include <isaacsim/ros2/nodes/SrtxPublisherFactory.h>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

SrtxFrameCallbackDescriptor* createImagePublisherDescriptor(const std::string& topicName,
                                                            const std::string& frameId,
                                                            const std::string& nodeNamespace,
                                                            uint64_t queueSize,
                                                            const std::string& qosProfile)
{
    auto* pub = new Ros2SrtxImagePublisher();
    if (!pub->initialize(topicName, frameId, nodeNamespace, queueSize, qosProfile))
    {
        delete pub;
        return nullptr;
    }

    return new SrtxFrameCallbackDescriptor{ &Ros2SrtxImagePublisher::trampoline, pub, &Ros2SrtxImagePublisher::destroy };
}

SrtxFrameCallbackDescriptor* createCameraInfoPublisherDescriptor(const std::string& topicName,
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
    auto* pub = new Ros2SrtxCameraInfoPublisher();
    if (!pub->initialize(
            topicName, frameId, nodeNamespace, queueSize, qosProfile, width, height, distortionModel, k, r, p, d))
    {
        delete pub;
        return nullptr;
    }

    return new SrtxFrameCallbackDescriptor{ &Ros2SrtxCameraInfoPublisher::trampoline, pub,
                                            &Ros2SrtxCameraInfoPublisher::destroy };
}

SrtxFrameCallbackDescriptor* createLidarPublisherDescriptor(const std::string& topicName,
                                                            const std::string& frameId,
                                                            const std::string& nodeNamespace,
                                                            uint64_t queueSize,
                                                            const std::string& qosProfile)
{
    auto* pub = new Ros2SrtxLidarPublisher();
    if (!pub->initialize(topicName, frameId, nodeNamespace, queueSize, qosProfile))
    {
        delete pub;
        return nullptr;
    }

    return new SrtxFrameCallbackDescriptor{ &Ros2SrtxLidarPublisher::trampoline, pub, &Ros2SrtxLidarPublisher::destroy };
}

SrtxFrameCallbackDescriptor* createLaserScanPublisherDescriptor(const std::string& topicName,
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
    auto* pub = new Ros2SrtxLaserScanPublisher();
    if (!pub->initialize(topicName, frameId, nodeNamespace, queueSize, qosProfile, azimuthRangeStart, azimuthRangeEnd,
                         depthRangeMin, depthRangeMax, rotationRate, horizontalResolution, horizontalFov))
    {
        delete pub;
        return nullptr;
    }

    return new SrtxFrameCallbackDescriptor{ &Ros2SrtxLaserScanPublisher::trampoline, pub,
                                            &Ros2SrtxLaserScanPublisher::destroy };
}

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
