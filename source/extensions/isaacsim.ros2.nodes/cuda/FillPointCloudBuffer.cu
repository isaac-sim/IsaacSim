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

#include <tuple>
#include <vector>

#include "isaacsim/core/includes/ScopedCudaDevice.h"
#include "FillPointCloudBuffer.cuh"

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

__global__ void fillBufferKernel(uint8_t* __restrict__ buffer, const uint8_t* __restrict__ data, const size_t dataSize, const size_t dataOffset, const size_t numPoints, const size_t pointWidth) {
    int pointIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (pointIdx >= numPoints) {
        return;
    }
    const size_t destIdx = pointIdx*pointWidth + dataOffset;
    const size_t srcIdx = pointIdx*dataSize;
    for (int byteIdx = 0; byteIdx < dataSize; byteIdx++) {
        buffer[destIdx + byteIdx] = data[srcIdx + byteIdx];
    }
}

void fillPointCloudBuffer(uint8_t* __restrict__ buffer, const float3* __restrict__ pointCloudData, const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields, const size_t pointWidth, const size_t numPoints, const int maxThreadsPerBlock, const int multiProcessorCount, const int cudaDeviceIndex, cudaStream_t stream)
{
    isaacsim::core::includes::ScopedDevice scopedDevice(cudaDeviceIndex);

    if (numPoints == 0) {
        return;
    }
    // Select appropriate number of threads and blocks, optimizing for occupancy. Select all possible threads by default.
    int nt = maxThreadsPerBlock;
    int nb = (numPoints + nt - 1) / nt;
    if (numPoints < 1024) {
        // vectorized approach - high occupancy
        nt = 256;
        nb = (multiProcessorCount * 4); // Ensure high occupancy
    }
    fillBufferKernel<<<nb, nt, 0, stream>>>(buffer, reinterpret_cast<const uint8_t*>(pointCloudData), sizeof(float3), 0, numPoints, pointWidth);

    for (const auto& [data, size, offset] : orderedFields) {
        fillBufferKernel<<<nb, nt, 0, stream>>>(buffer, reinterpret_cast<const uint8_t*>(data), size, offset, numPoints, pointWidth);
    }
}

}
}
}
