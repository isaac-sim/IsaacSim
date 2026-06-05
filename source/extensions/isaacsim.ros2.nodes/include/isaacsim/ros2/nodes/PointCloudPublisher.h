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

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/core/includes/ScopedCudaDevice.h>
#include <isaacsim/ros2/nodes/PublisherBase.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/// CUDA kernel that interleaves XYZ point data with metadata fields into a
/// single contiguous buffer on the device. Defined in FillPointCloudBuffer.cu
/// (or ROS2PublishPointCloud.cu).
void fillPointCloudBuffer(uint8_t* buffer,
                          const float3* pointCloudData,
                          const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields,
                          size_t pointWidth,
                          size_t numPoints,
                          int maxThreadsPerBlock,
                          int multiProcessorCount,
                          int cudaDeviceIndex,
                          cudaStream_t stream);

/// Host-side version that interleaves xyz + metadata into a contiguous buffer.
void fillPointCloudBufferHost(uint8_t* buffer,
                              const float3* pointCloudData,
                              const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields,
                              size_t pointWidth,
                              size_t numPoints);

/// Metadata pointers for optional per-point fields in a lidar point cloud.
/// Any pointer that is null will be omitted from the published message.
struct PointCloudMetadata
{
    float* intensityPtr = nullptr; ///< Per-point intensity values.
    uint64_t* timestampPtr = nullptr; ///< Per-point timestamps (ns).
    uint32_t* emitterIdPtr = nullptr; ///< Per-point emitter IDs.
    uint32_t* channelIdPtr = nullptr; ///< Per-point channel IDs.
    uint32_t* materialIdPtr = nullptr; ///< Per-point material IDs.
    uint32_t* tickIdPtr = nullptr; ///< Per-point tick IDs.
    pxr::GfVec3f* hitNormalPtr = nullptr; ///< Per-point hit normals.
    pxr::GfVec3f* velocityPtr = nullptr; ///< Per-point velocity vectors.
    uint32_t* objectIdPtr = nullptr; ///< Per-point object IDs.
    uint8_t* echoIdPtr = nullptr; ///< Per-point echo IDs.
    uint8_t* tickStatePtr = nullptr; ///< Per-point tick states.
    float* radialVelocityMSPtr = nullptr; ///< Per-point radial velocity (m/s).
};

/// Standalone ROS 2 point cloud publisher that can be used from OmniGraph
/// nodes or registered as a callback for SRTX frame delivery.
///
/// Lifecycle:
///   1. Construct
///   2. Call initialize() once
///   3. Call publishFromHost() / publishFromDevice() per frame
///   4. Call reset() or destroy
class PointCloudPublisher : public PublisherBase
{
public:
    ~PointCloudPublisher() override;

    /// Fill the message from host (CPU) data without publishing.
    bool prepareFromHost(const void* data, size_t dataSize, double timestamp, const PointCloudMetadata& metadata = {});

    /// Convenience: prepareFromHost() + send().
    bool publishFromHost(const void* data, size_t dataSize, double timestamp, const PointCloudMetadata& metadata = {});

    /// Fill the message from CUDA device memory without publishing.
    bool prepareFromDevice(const void* devicePtr,
                           size_t bufferSize,
                           double timestamp,
                           int cudaDeviceIndex,
                           const PointCloudMetadata& metadata = {});

    /// Convenience: prepareFromDevice() + send().
    bool publishFromDevice(const void* devicePtr,
                           size_t bufferSize,
                           double timestamp,
                           int cudaDeviceIndex,
                           const PointCloudMetadata& metadata = {});

    /// @copydoc PublisherBase::reset()
    void reset() override;

    /// Access to the underlying message for fill-from-host flows that need
    /// generateBuffer + fillPointCloudBufferHost. Used by OgnROS2PublishPointCloud.
    isaacsim::ros2::core::Ros2PointCloudMessage* getPointCloudMessage()
    {
        return m_message.get();
    }

    /// Get the TF frame ID stamped into published messages.
    const std::string& getFrameId() const
    {
        return m_frameId;
    }

protected:
    const void* onCreateMessage() override;
    isaacsim::ros2::core::Ros2Message* getMessagePtr() override
    {
        return m_message.get();
    }

private:
    void ensureCudaStream(int cudaDeviceIndex);
    void destroyCudaStream();
    bool initializeCudaProperties(int cudaDeviceIndex);

    std::shared_ptr<isaacsim::ros2::core::Ros2PointCloudMessage> m_message;

    cudaStream_t m_stream{};
    int m_streamDevice = -1;
    bool m_streamNotCreated = true;

    int m_maxThreadsPerBlock = 0;
    int m_multiProcessorCount = 0;

    uint8_t* m_deviceBuffer = nullptr;
    size_t m_deviceBufferSize = 0;

    carb::tasking::TaskGroup m_tasks;
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
