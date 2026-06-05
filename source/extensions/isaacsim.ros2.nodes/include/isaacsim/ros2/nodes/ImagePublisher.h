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

#pragma once

#include <carb/RenderingTypes.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/core/includes/ScopedCudaDevice.h>
#include <isaacsim/ros2/nodes/PublisherBase.h>

#include <memory>
#include <string>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// Standalone ROS 2 image publisher that can be used from OmniGraph nodes
/// or registered as a callback for SRTX frame delivery.
///
/// Lifecycle:
///   1. Construct
///   2. Call initialize() once
///   3. Call publishFromHost() / publishFromDevice() per frame
///   4. Call reset() or destroy
class ImagePublisher : public PublisherBase
{
public:
    ~ImagePublisher() override;

    /// Fill the message from host (CPU) data without publishing.
    bool prepareFromHost(
        const void* data, size_t dataSize, uint32_t width, uint32_t height, const std::string& encoding, double timestamp);

    /// Convenience: prepareFromHost() + send().
    bool publishFromHost(
        const void* data, size_t dataSize, uint32_t width, uint32_t height, const std::string& encoding, double timestamp);

    /// Fill the message from CUDA device memory without publishing.
    bool prepareFromDevice(const void* devicePtr,
                           size_t bufferSize,
                           uint32_t width,
                           uint32_t height,
                           const std::string& encoding,
                           double timestamp,
                           int cudaDeviceIndex,
                           carb::Format format = carb::Format::eUnknown);

    /// Convenience: prepareFromDevice() + send().
    bool publishFromDevice(const void* devicePtr,
                           size_t bufferSize,
                           uint32_t width,
                           uint32_t height,
                           const std::string& encoding,
                           double timestamp,
                           int cudaDeviceIndex,
                           carb::Format format = carb::Format::eUnknown);

    /// @copydoc PublisherBase::reset()
    void reset() override;

protected:
    const void* onCreateMessage() override;
    isaacsim::ros2::core::Ros2Message* getMessagePtr() override
    {
        return m_message.get();
    }

private:
    bool prepareMessage(uint32_t width, uint32_t height, const std::string& encoding, double timestamp, bool usePinnedMemory);

    void ensureCudaStream(int cudaDeviceIndex);
    void destroyCudaStream();

    std::shared_ptr<isaacsim::ros2::core::Ros2ImageMessage> m_message;

    cudaStream_t m_stream{};
    int m_streamDevice = -1;
    bool m_streamNotCreated = true;

    carb::tasking::TaskGroup m_tasks;
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
