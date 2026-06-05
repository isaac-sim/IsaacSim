// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// CUDA error checking macros and context validation for Newton tensor GPU operations.

#include <carb/logging/Log.h>

#include <cuda_runtime.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

/// Checks a CUDA runtime call, logs the error, and returns false on failure.
#define CHECK_CUDA(call)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        cudaError_t err = call;                                                                                        \
        if (err != cudaSuccess)                                                                                        \
        {                                                                                                              \
            (void)cudaGetLastError();                                                                                  \
            CARB_LOG_ERROR("CUDA error at %s:%d: %s", __FILE__, __LINE__, cudaGetErrorString(err));                    \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

/// Checks for errors after a kernel launch (cudaGetLastError), logs and returns false on failure.
#define CHECK_CUDA_LAUNCH()                                                                                            \
    do                                                                                                                 \
    {                                                                                                                  \
        cudaError_t err = cudaGetLastError();                                                                          \
        if (err != cudaSuccess)                                                                                        \
        {                                                                                                              \
            CARB_LOG_ERROR("CUDA kernel launch error at %s:%d: %s", __FILE__, __LINE__, cudaGetErrorString(err));      \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

/// Validates that a CUDA context exists for the given device ordinal, creating one if needed.
bool validateCudaContext(int deviceOrdinal);

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
