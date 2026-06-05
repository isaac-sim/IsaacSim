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

#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Publishes USD-derived camera calibration each time SRTX delivers a frame callback.
class Ros2SrtxCameraInfoPublisher : public Ros2SrtxPublisher
{
public:
    Ros2SrtxCameraInfoPublisher() = default;
    ~Ros2SrtxCameraInfoPublisher() override = default;

    Ros2SrtxCameraInfoPublisher(const Ros2SrtxCameraInfoPublisher&) = delete;
    Ros2SrtxCameraInfoPublisher& operator=(const Ros2SrtxCameraInfoPublisher&) = delete;

    /// Not used — call the overload with camera calibration instead.
    bool initialize(const std::string& topicName,
                    const std::string& frameId,
                    const std::string& nodeNamespace,
                    uint64_t queueSize,
                    const std::string& qosProfile) override;

    bool initialize(const std::string& topicName,
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
                    const std::vector<double>& d);

    void publishCameraInfo(double timestamp);

    /// C-ABI trampoline matching SrtxFrameCallbackFn.
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
    std::shared_ptr<isaacsim::ros2::core::Ros2CameraInfoMessage> m_message;

    uint32_t m_width = 0;
    uint32_t m_height = 0;
    std::string m_distortionModel;
    std::vector<double> m_k;
    std::vector<double> m_r;
    std::vector<double> m_p;
    std::vector<double> m_d;
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
