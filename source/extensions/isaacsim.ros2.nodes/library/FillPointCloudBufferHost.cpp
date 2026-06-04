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

#include "../cuda/FillPointCloudBuffer.cuh"

#include <carb/tasking/ITasking.h>

#include <cstring>
#include <tuple>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

void fillPointCloudBufferHost(uint8_t* buffer,
                              const float3* pointCloudData,
                              const std::vector<std::tuple<void*, size_t, size_t>>& orderedFields,
                              const size_t pointWidth,
                              const size_t numPoints)
{
    if (numPoints == 0)
    {
        return;
    }

    auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

    // Single parallel pass: each iteration handles one point's full copy (xyz + all fields)
    // Cache-friendly for writes since each iteration writes a contiguous pointWidth block
    tasking->parallelFor(size_t(0), numPoints,
                         [buffer, pointCloudData, &orderedFields, pointWidth](size_t i)
                         {
                             uint8_t* dst = buffer + i * pointWidth;
                             // Copy xyz
                             memcpy(dst, reinterpret_cast<const uint8_t*>(&pointCloudData[i]), sizeof(float3));
                             // Copy each metadata field for this point
                             for (const auto& [data, dataSize, offset] : orderedFields)
                             {
                                 memcpy(dst + offset, reinterpret_cast<const uint8_t*>(data) + i * dataSize, dataSize);
                             }
                         });
}

}
}
}
