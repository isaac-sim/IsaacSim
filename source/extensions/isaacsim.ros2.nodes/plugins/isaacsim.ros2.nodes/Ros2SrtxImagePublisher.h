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

#pragma once

#include "Ros2SrtxPublisher.h"

#if !defined(_WIN32)
#    include <isaacsim/ros2/core/IpcBufferManager.h>
#endif

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Publishes SRTX image frames (kImage2d) to a ROS 2 topic.
class Ros2SrtxImagePublisher : public Ros2SrtxPublisher
{
public:
    Ros2SrtxImagePublisher() = default;
    ~Ros2SrtxImagePublisher() override = default;

    Ros2SrtxImagePublisher(const Ros2SrtxImagePublisher&) = delete;
    Ros2SrtxImagePublisher& operator=(const Ros2SrtxImagePublisher&) = delete;

    bool initialize(const std::string& topicName,
                    const std::string& frameId,
                    const std::string& nodeNamespace,
                    uint64_t queueSize,
                    const std::string& qosProfile) override;

    void publishImage(const uint8_t* data,
                      uint32_t width,
                      uint32_t height,
                      const std::string& encoding,
                      size_t bufferSize,
                      double timestamp) override;

    /// C-ABI trampoline matching SrtxFrameCallbackFn.  userData is a
    /// Ros2SrtxImagePublisher*.  Extracts width/height from shape and
    /// derives the ROS encoding from the SRTX format string.
    static void trampoline(void* userData,
                           const uint8_t* data,
                           size_t dataSize,
                           const uint32_t* shape,
                           size_t shapeLen,
                           const char* format,
                           const char* outputPath,
                           double timestamp);

    static void destroy(void* userData);

private:
    std::shared_ptr<isaacsim::ros2::core::Ros2ImageMessage> m_message;

    bool m_nitrosBridgeEnabled = false;
    std::shared_ptr<isaacsim::ros2::core::Ros2Publisher> m_nitrosBridgePublisher;
    std::shared_ptr<isaacsim::ros2::core::Ros2NitrosBridgeImageMessage> m_nitrosBridgeMessage;
#if !defined(_WIN32)
    std::shared_ptr<IPCBufferManager> m_ipcBufferManager;
#endif
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
