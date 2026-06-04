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
#include <cstdint>
#include <cuda_runtime.h>

namespace isaacsim::hsb::core
{

static constexpr uint32_t kVB1940EmbeddedLines = 3; // 1 leading + 2 trailing

inline uint32_t lineBytesRaw10(uint32_t width)
{
    uint32_t lb = (width + 3) / 4 * 5;
    return ((lb + 7) >> 3) << 3; // 8-byte aligned
}

inline uint32_t lineBytesRaw10Coe(uint32_t width)
{
    uint32_t lb = (width + 2) / 3 * 4;
    return ((lb + 63) >> 6) << 6; // 64-byte aligned
}

inline uint32_t vb1940CsiLinuxFrameSize(uint32_t w, uint32_t h)
{
    return lineBytesRaw10(w) * (h + kVB1940EmbeddedLines);
}

inline uint32_t vb1940CsiCoeFrameSize(uint32_t w, uint32_t h)
{
    return lineBytesRaw10Coe(w) * (h + kVB1940EmbeddedLines);
}

// Launch wrappers (defined in RGBToVB1940Kernels.cu)
void launchRGBToGBRG(
    const uint8_t* src, uint8_t* dest, uint32_t width, uint32_t height, uint8_t channels, cudaStream_t stream = nullptr);

void launchBayer8pTo10p(
    const uint8_t* src, uint8_t* dest, uint32_t width, uint32_t height, uint32_t lineBytes, cudaStream_t stream = nullptr);

void launchBayer8pToTX2Rc10Rb10Ra10(
    const uint8_t* src, uint8_t* dest, uint32_t width, uint32_t height, uint32_t lineBytes, cudaStream_t stream = nullptr);
} // namespace isaacsim::hsb::core
