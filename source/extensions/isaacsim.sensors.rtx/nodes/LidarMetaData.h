// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdint>

namespace omni
{
namespace sensors
{
namespace lidar
{

// #pragma pack(push, 1) // Make sure we have consistent structure packing

struct LidarMetaData
{
    uint64_t dataSize{ 0 };
    uint64_t numPoints{ 0 };
    uint64_t maxPoints{ 0 };
    uint64_t scanStartTimeNs{ 0 };
    uint64_t startTimeNs{ 0 };
    uint64_t endTimeNs{ 0 };
    bool scanComplete{ false };
    void* syncHandler{ nullptr }; // passing pointer to memhandler of model -- normally, not used except with frames in
                                  // flight = 1
};

// #pragma pack(pop)


} // namespace lidar
} // namespace sensors
} // namespace omni
