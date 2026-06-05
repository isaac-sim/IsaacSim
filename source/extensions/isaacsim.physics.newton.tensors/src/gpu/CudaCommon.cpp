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

#include "CudaCommon.h"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

bool validateCudaContext(int deviceOrdinal)
{
    if (deviceOrdinal < 0)
    {
        return true;
    }

    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        CARB_LOG_ERROR("Failed to get CUDA device count: %s", cudaGetErrorString(err));
        return false;
    }

    if (deviceOrdinal >= deviceCount)
    {
        CARB_LOG_ERROR("Invalid device ordinal %d (available: %d)", deviceOrdinal, deviceCount);
        return false;
    }

    err = cudaSetDevice(deviceOrdinal);
    if (err != cudaSuccess)
    {
        (void)cudaGetLastError();
        CARB_LOG_ERROR("Failed to set CUDA device %d: %s", deviceOrdinal, cudaGetErrorString(err));
        return false;
    }

    return true;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
