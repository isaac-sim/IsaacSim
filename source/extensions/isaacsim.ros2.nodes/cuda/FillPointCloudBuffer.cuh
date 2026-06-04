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

#include <vector>
#include <tuple>
#include <cuda.h>
#include <cuda_runtime.h>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/**
* @brief Fill the point cloud buffer with the given point cloud data and ordered fields.
* @param buffer The devicebuffer to fill.
* @param pointCloudData The device pointer to the point cloud data, arranged as x, y, z.
* @param orderedFields The ordered fields. Tuple of (data pointer, size, offset).
* @param pointWidth The width of the point in bytes.
* @param numPoints The number of points.
* @param maxThreadsPerBlock The maximum number of threads per block.
* @param multiProcessorCount The number of multi-processors.
* @param cudaDeviceIndex The index of the device.
* @param stream The stream to use.
*/
void fillPointCloudBuffer(uint8_t* buffer, const float3* pointCloudData, const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields, const size_t pointWidth, const size_t numPoints, const int maxThreadsPerBlock, const int multiProcessorCount, const int cudaDeviceIndex, cudaStream_t stream);

/**
* @brief Host-side version of fillPointCloudBuffer. Interleaves xyz + metadata fields into a
*        contiguous buffer using carb::tasking for parallelism.
* @param buffer The host buffer to fill.
* @param pointCloudData The host pointer to the point cloud data, arranged as x, y, z.
* @param orderedFields The ordered fields. Tuple of (data pointer, size, offset).
* @param pointWidth The width of the point in bytes.
* @param numPoints The number of points.
*/
void fillPointCloudBufferHost(uint8_t* buffer, const float3* pointCloudData, const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields, const size_t pointWidth, const size_t numPoints);
}
}
}
