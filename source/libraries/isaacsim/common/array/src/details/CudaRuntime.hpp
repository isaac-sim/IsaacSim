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

#include "isaacsim/common/array/Export.h"

#include <cstddef>
#include <cuda_runtime_api.h>
#include <string>

namespace isaacsim
{
namespace common
{
namespace array
{

/** @brief Runtime-loaded access to the CUDA runtime functions used by Array. */
class ISAACSIM_COMMON_ARRAY_API CudaRuntime
{
public:
    /** @brief Returns the process-wide CUDA runtime loader. */
    static CudaRuntime& getInstance(bool throwIfInvalid = true);

    /** @brief Returns whether the CUDA runtime library was loaded successfully. */
    bool isLoaded() const;
    /** @brief Calls `cudaGetDeviceCount` when the CUDA runtime is available. */
    size_t cudaGetDeviceCount() const;
    /** @brief Calls `cudaMalloc` when the CUDA runtime is available. */
    bool cudaMalloc(void** devicePointer, size_t size, bool throwIfInvalid = true) const;
    /** @brief Calls `cudaFree` when the CUDA runtime is available. */
    bool cudaFree(void* devicePointer, bool throwIfInvalid = true) const;
    /** @brief Calls `cudaMemcpy` when the CUDA runtime is available. */
    bool cudaMemcpy(void* destination, const void* source, size_t size, cudaMemcpyKind kind, bool throwIfInvalid = true) const;
    /** @brief Calls `cudaMemset` when the CUDA runtime is available. */
    bool cudaMemset(void* devicePointer, int value, size_t size, bool throwIfInvalid = true) const;
    /** @brief Calls `cudaGetDevice` when the CUDA runtime is available. */
    bool cudaGetDevice(int* device, bool throwIfInvalid = true) const;
    /** @brief Calls `cudaSetDevice` when the CUDA runtime is available. */
    bool cudaSetDevice(int device, bool throwIfInvalid = true) const;

    CudaRuntime(const CudaRuntime&) = delete;
    CudaRuntime& operator=(const CudaRuntime&) = delete;

private:
    using CudaGetDeviceCountFunction = cudaError_t (*)(int*);
    using CudaMallocFunction = cudaError_t (*)(void**, size_t);
    using CudaFreeFunction = cudaError_t (*)(void*);
    using CudaMemcpyFunction = cudaError_t (*)(void*, const void*, size_t, cudaMemcpyKind);
    using CudaMemsetFunction = cudaError_t (*)(void*, int, size_t);
    using CudaGetDeviceFunction = cudaError_t (*)(int*);
    using CudaSetDeviceFunction = cudaError_t (*)(int);
    using CudaGetErrorStringFunction = const char* (*)(cudaError_t);

    CudaRuntime();
    ~CudaRuntime();

    bool _checkResult(cudaError_t result, const std::string& callerName, bool throwIfInvalid) const;
    bool _reportUnavailable(const std::string& callerName, bool throwIfInvalid) const;

    void* m_handle{ nullptr };
    CudaGetDeviceCountFunction m_cudaGetDeviceCount{ nullptr };
    CudaMallocFunction m_cudaMalloc{ nullptr };
    CudaFreeFunction m_cudaFree{ nullptr };
    CudaMemcpyFunction m_cudaMemcpy{ nullptr };
    CudaMemsetFunction m_cudaMemset{ nullptr };
    CudaGetDeviceFunction m_cudaGetDevice{ nullptr };
    CudaSetDeviceFunction m_cudaSetDevice{ nullptr };
    CudaGetErrorStringFunction m_cudaGetErrorString{ nullptr };
};

} // namespace array
} // namespace common
} // namespace isaacsim
