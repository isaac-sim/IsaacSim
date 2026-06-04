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

#include <isaacsim/hsb/core/RGBToVB1940Kernels.h>
#include <cstdint>

namespace isaacsim::hsb::core {

#define MAX_8BIT_COLOR  255
#define MAX_10BIT_COLOR 1023

// Statistical round and clamp to range (identical to emulator_kernels.cu)
__device__ static uint16_t bankers_round(float value, uint16_t max_value, uint16_t min_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    uint16_t trim = (uint16_t)value;
    float diff = value - trim;
    if (diff > 0.5f) {
        return trim + 1;
    } else if (diff < 0.5f) {
        return trim;
    }
    return trim + (trim & 1);
}

// Convert packed RGB/RGBA input (H x W x channels) to GBRG Bayer 8-bit (H x W).
// GBRG Bayer pattern:
//   even row, even col -> G (channel 1)
//   even row, odd  col -> B (channel 2)
//   odd  row, even col -> R (channel 0)
//   odd  row, odd  col -> G (channel 1)
__global__ static void rgb_to_gbrg_kernel(const uint8_t* __restrict__ src, uint8_t* __restrict__ dest,
                                          uint32_t width, uint32_t height, uint8_t channels)
{
    uint32_t col = blockIdx.x * blockDim.x + threadIdx.x;
    uint32_t row = blockIdx.y * blockDim.y + threadIdx.y;

    if (col >= width || row >= height) {
        return;
    }

    uint8_t pixel;
    if (row & 1u) {
        // odd row: R G row of GBRG
        pixel = (col & 1u) ? src[(row * width + col) * channels + 1]  // G
                           : src[(row * width + col) * channels + 0]; // R
    } else {
        // even row: G B row of GBRG
        pixel = (col & 1u) ? src[(row * width + col) * channels + 2]  // B
                           : src[(row * width + col) * channels + 1]; // G
    }

    dest[row * width + col] = pixel;
}

// Convert 8-bit Bayer to RAW10-packed (4 pixels per 5 bytes).
// Matches bayer8p_to_10p_kernel from emulator_kernels.cu.
__global__ static void bayer8p_to_10p_kernel(uint8_t* __restrict__ dest, uint32_t line_bytes,
                                              const uint8_t* __restrict__ src,
                                              uint32_t pixel_height, uint32_t pixel_width)
{
    int32_t ix = 4 * (blockIdx.x * blockDim.x + threadIdx.x);
    int32_t iy = blockIdx.y * blockDim.y + threadIdx.y;

    if (ix >= pixel_width || iy >= pixel_height) {
        return;
    }

    int32_t src_offset  = iy * pixel_width + ix;
    int32_t dest_offset = iy * line_bytes + (ix / 4) * 5;
    const float factor = MAX_10BIT_COLOR * 1.0f / MAX_8BIT_COLOR;

    memset(&dest[dest_offset], 0, 5);

    uint16_t color = bankers_round(src[src_offset + 0] * factor, MAX_10BIT_COLOR, 0);
    dest[dest_offset]     = (color >> 2) & 0xFF;
    dest[dest_offset + 4] = (color & 0x3) << 0;
    if (ix + 1 < pixel_width) {
        color = bankers_round(src[src_offset + 1] * factor, MAX_10BIT_COLOR, 0);
        dest[dest_offset + 1]  = (color >> 2) & 0xFF;
        dest[dest_offset + 4] |= (color & 0x3) << 2;
    }
    if (ix + 2 < pixel_width) {
        color = bankers_round(src[src_offset + 2] * factor, MAX_10BIT_COLOR, 0);
        dest[dest_offset + 2]  = (color >> 2) & 0xFF;
        dest[dest_offset + 4] |= (color & 0x3) << 4;
    }
    if (ix + 3 < pixel_width) {
        color = bankers_round(src[src_offset + 3] * factor, MAX_10BIT_COLOR, 0);
        dest[dest_offset + 3]  = (color >> 2) & 0xFF;
        dest[dest_offset + 4] |= (color & 0x3) << 6;
    }
}

// Convert 8-bit Bayer to T_X2Rc10Rb10Ra10 (3 pixels per 4 bytes, 64-byte line alignment).
// Matches bayer8p_to_T_X2Rc10Rb10Ra10_kernel from emulator_kernels.cu.
__global__ static void bayer8p_to_T_X2Rc10Rb10Ra10_kernel(uint8_t* __restrict__ dest, uint32_t line_bytes,
                                                            const uint8_t* __restrict__ src,
                                                            uint32_t pixel_height, uint32_t pixel_width)
{
    int32_t ix = 3 * (blockIdx.x * blockDim.x + threadIdx.x);
    int32_t iy = blockIdx.y * blockDim.y + threadIdx.y;

    if (ix >= pixel_width || iy >= pixel_height) {
        return;
    }

    int32_t src_offset  = iy * pixel_width + ix;
    int32_t dest_offset = iy * line_bytes + (ix / 3) * 4;
    const float factor = MAX_10BIT_COLOR * 1.0f / MAX_8BIT_COLOR;

    uint16_t color = bankers_round(src[src_offset + 0] * factor, MAX_10BIT_COLOR, 0);
    dest[dest_offset]     =  color & 0xFF;
    dest[dest_offset + 1] = (color >> 8) & 0x03;
    if (ix + 1 < pixel_width) {
        color = bankers_round(src[src_offset + 1] * factor, MAX_10BIT_COLOR, 0);
        dest[dest_offset + 1] |= (color & 0x3F) << 2;
        dest[dest_offset + 2]  = (color >> 6) & 0x0F;
    }
    if (ix + 2 < pixel_width) {
        color = bankers_round(src[src_offset + 2] * factor, MAX_10BIT_COLOR, 0);
        dest[dest_offset + 2] |= (color & 0xF) << 4;
        dest[dest_offset + 3]  = (color >> 4) & 0x3F;
    }
}

// ---- C++ launch wrappers ----

void launchRGBToGBRG(const uint8_t* src, uint8_t* dest,
                     uint32_t width, uint32_t height, uint8_t channels,
                     cudaStream_t stream)
{
    dim3 block(32, 32);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);
    rgb_to_gbrg_kernel<<<grid, block, 0, stream>>>(src, dest, width, height, channels);
}

void launchBayer8pTo10p(const uint8_t* src, uint8_t* dest,
                        uint32_t width, uint32_t height, uint32_t lineBytes,
                        cudaStream_t stream)
{
    // Each thread handles 4 pixels in x
    dim3 block(32, 8);
    dim3 grid(((width + 3) / 4 + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);
    bayer8p_to_10p_kernel<<<grid, block, 0, stream>>>(dest, lineBytes, src, height, width);
}

void launchBayer8pToTX2Rc10Rb10Ra10(const uint8_t* src, uint8_t* dest,
                                     uint32_t width, uint32_t height, uint32_t lineBytes,
                                     cudaStream_t stream)
{
    // Each thread handles 3 pixels in x
    dim3 block(32, 8);
    dim3 grid(((width + 2) / 3 + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);
    bayer8p_to_T_X2Rc10Rb10Ra10_kernel<<<grid, block, 0, stream>>>(dest, lineBytes, src, height, width);
}

} // namespace isaacsim::hsb::core
