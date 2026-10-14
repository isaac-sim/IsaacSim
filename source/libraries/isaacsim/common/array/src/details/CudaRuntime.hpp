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
#include <cuda.h>
#include <mutex>
#include <string>
#include <unordered_map>

namespace isaacsim
{
namespace common
{
namespace array
{

/**
 * @brief Runtime-loaded access to the CUDA functions used by Array.
 * @details
 * This binds the CUDA *driver* library rather than the CUDA runtime. The driver is installed by the
 * NVIDIA display driver into the loader's default search path, so it resolves without any vendored
 * redistributable, RPATH entry, or `LD_LIBRARY_PATH`/`PATH` contribution; the runtime's shared
 * library has none of those guarantees and is not shipped with this project. The method names still
 * read `cudaXxx` after the API they replace, so call sites did not have to churn; each one documents
 * the driver entry point it actually calls.
 *
 * Absence stays a supported state: when the driver cannot be loaded or `cuInit()` fails -- a machine
 * with no GPU, or no driver installed -- `isLoaded()` reports false and every call either throws or
 * returns false according to its `throwIfInvalid` argument, leaving CPU paths fully usable.
 *
 * Contexts are the one place the driver API is not a drop-in. The runtime API keeps an implicit
 * per-thread current device and creates that device's primary context on demand; the driver API
 * requires a context to already be current. Every entry point that needs one therefore routes
 * through `_ensureContext()`, which binds the primary context -- the same context the CUDA runtime
 * itself uses, so streams and events remain interoperable with the statically linked runtime inside
 * the kernel library, and with any other component in the process that uses CUDA.
 */
class ISAACSIM_COMMON_ARRAY_API CudaRuntime
{
public:
    /**
     * @brief Where an allocation lives, as reported by the driver.
     * @details Replaces `cudaPointerAttributes`, of which only these two fields were ever read.
     */
    struct PointerAttributes
    {
        /** @brief Whether the pointer addresses device or managed memory. */
        bool isDevice{ false };
        /** @brief Owning device ordinal, or -1 when the driver does not report one. */
        int device{ -1 };
    };

    /** @brief Returns the process-wide CUDA driver loader. */
    static CudaRuntime& getInstance(bool throwIfInvalid = true);

    /** @brief Returns whether the CUDA driver library was loaded and initialized successfully. */
    bool isLoaded() const;
    /** @brief Calls `cuDeviceGetCount`; needs no context. */
    size_t cudaGetDeviceCount() const;
    /** @brief Calls `cuMemAlloc`. */
    bool cudaMalloc(void** devicePointer, size_t size, bool throwIfInvalid = true) const;
    /** @brief Calls `cuMemFree`. */
    bool cudaFree(void* devicePointer, bool throwIfInvalid = true) const;
    /**
     * @brief Calls `cuMemcpy`, which infers each side's location from unified addressing.
     * @details The direction argument the runtime API required is gone: `cuMemcpy` resolves both
     *          pointers itself, exactly as `cudaMemcpyDefault` did. Unified addressing is always on
     *          for the 64-bit platforms this library builds for.
     */
    bool cudaMemcpy(void* destination, const void* source, size_t size, bool throwIfInvalid = true) const;
    /** @brief Calls `cuMemcpyAsync`; direction is inferred as in @ref cudaMemcpy. */
    bool cudaMemcpyAsync(
        void* destination, const void* source, size_t size, CUstream stream, bool throwIfInvalid = true) const;
    /** @brief Calls `cuMemsetD8`. */
    bool cudaMemset(void* devicePointer, int value, size_t size, bool throwIfInvalid = true) const;
    /** @brief Reports the current context's device, binding the default one if no context is current. */
    bool cudaGetDevice(int* device, bool throwIfInvalid = true) const;
    /** @brief Binds the primary context of @p device to the calling thread. */
    bool cudaSetDevice(int device, bool throwIfInvalid = true) const;
    /** @brief Calls `cuEventCreate`. */
    bool cudaEventCreateWithFlags(CUevent* event, unsigned int flags, bool throwIfInvalid = true) const;
    /** @brief Calls `cuEventDestroy`. */
    bool cudaEventDestroy(CUevent event, bool throwIfInvalid = true) const;
    /** @brief Calls `cuEventRecord`. */
    bool cudaEventRecord(CUevent event, CUstream stream, bool throwIfInvalid = true) const;
    /** @brief Calls `cuEventQuery`; returns `CUDA_SUCCESS`, `CUDA_ERROR_NOT_READY`, or an error. */
    CUresult cudaEventQuery(CUevent event) const;
    /** @brief Calls `cuEventSynchronize`. */
    bool cudaEventSynchronize(CUevent event, bool throwIfInvalid = true) const;
    /** @brief Calls `cuStreamWaitEvent`. */
    bool cudaStreamWaitEvent(CUstream stream, CUevent event, unsigned int flags, bool throwIfInvalid = true) const;
    /** @brief Calls `cuStreamSynchronize`. */
    bool cudaStreamSynchronize(CUstream stream, bool throwIfInvalid = true) const;
    /** @brief Calls `cuStreamGetDevice`. */
    bool cudaStreamGetDevice(CUstream stream, int* device, bool throwIfInvalid = true) const;
    /** @brief Calls `cuStreamCreate`. */
    bool cudaStreamCreateWithFlags(CUstream* stream, unsigned int flags, bool throwIfInvalid = true) const;
    /** @brief Calls `cuStreamDestroy`. */
    bool cudaStreamDestroy(CUstream stream, bool throwIfInvalid = true) const;
    /** @brief Calls `cuLaunchHostFunc`. */
    bool cudaLaunchHostFunc(CUstream stream, CUhostFn function, void* userData, bool throwIfInvalid = true) const;
    /** @brief Fills @p attributes from `cuPointerGetAttribute`. */
    bool cudaPointerGetAttributes(PointerAttributes* attributes, const void* pointer, bool throwIfInvalid = true) const;

    CudaRuntime(const CudaRuntime&) = delete;
    CudaRuntime& operator=(const CudaRuntime&) = delete;

private:
    using CuInitFunction = CUresult (*)(unsigned int);
    using CuDeviceGetCountFunction = CUresult (*)(int*);
    using CuDeviceGetFunction = CUresult (*)(CUdevice*, int);
    using CuDevicePrimaryCtxRetainFunction = CUresult (*)(CUcontext*, CUdevice);
    using CuCtxGetCurrentFunction = CUresult (*)(CUcontext*);
    using CuCtxSetCurrentFunction = CUresult (*)(CUcontext);
    using CuCtxGetDeviceFunction = CUresult (*)(CUdevice*);
    using CuMemAllocFunction = CUresult (*)(CUdeviceptr*, size_t);
    using CuMemFreeFunction = CUresult (*)(CUdeviceptr);
    using CuMemcpyFunction = CUresult (*)(CUdeviceptr, CUdeviceptr, size_t);
    using CuMemcpyAsyncFunction = CUresult (*)(CUdeviceptr, CUdeviceptr, size_t, CUstream);
    using CuMemsetD8Function = CUresult (*)(CUdeviceptr, unsigned char, size_t);
    using CuEventCreateFunction = CUresult (*)(CUevent*, unsigned int);
    using CuEventDestroyFunction = CUresult (*)(CUevent);
    using CuEventRecordFunction = CUresult (*)(CUevent, CUstream);
    using CuEventQueryFunction = CUresult (*)(CUevent);
    using CuEventSynchronizeFunction = CUresult (*)(CUevent);
    using CuStreamWaitEventFunction = CUresult (*)(CUstream, CUevent, unsigned int);
    using CuStreamSynchronizeFunction = CUresult (*)(CUstream);
    using CuStreamGetDeviceFunction = CUresult (*)(CUstream, CUdevice*);
    using CuStreamCreateFunction = CUresult (*)(CUstream*, unsigned int);
    using CuStreamDestroyFunction = CUresult (*)(CUstream);
    using CuLaunchHostFuncFunction = CUresult (*)(CUstream, CUhostFn, void*);
    using CuPointerGetAttributeFunction = CUresult (*)(void*, CUpointer_attribute, CUdeviceptr);
    using CuGetErrorStringFunction = CUresult (*)(CUresult, const char**);

    CudaRuntime();
    ~CudaRuntime();

    bool _checkResult(CUresult result, const char* callerName, bool throwIfInvalid) const;
    bool _reportUnavailable(const char* callerName, bool throwIfInvalid) const;
    /**
     * @brief Binds a primary context to the calling thread when none is current yet.
     * @details The runtime API creates and binds a device's primary context implicitly on first use,
     *          defaulting to device 0; driver entry points instead fail with
     *          `CUDA_ERROR_INVALID_CONTEXT`. Calling this first reproduces the implicit behavior, so
     *          callers that never went through @ref cudaSetDevice (or a DeviceGuard) still work.
     */
    bool _ensureContext(const char* callerName, bool throwIfInvalid) const;
    /** @brief Retains and caches @p device's primary context, returning null on failure. */
    CUcontext _primaryContext(int device, CUresult* status) const;

    // Primary contexts are retained once per device and never released: the retain count is what
    // keeps the context alive, and dropping it during static destruction would race teardown in any
    // other component still holding CUDA objects. They are cached because cuDevicePrimaryCtxRetain
    // sits on the allocation path, which runs per array.
    mutable std::mutex m_contextMutex;
    mutable std::unordered_map<int, CUcontext> m_primaryContexts;

    void* m_handle{ nullptr };
    CuInitFunction m_cuInit{ nullptr };
    CuDeviceGetCountFunction m_cuDeviceGetCount{ nullptr };
    CuDeviceGetFunction m_cuDeviceGet{ nullptr };
    CuDevicePrimaryCtxRetainFunction m_cuDevicePrimaryCtxRetain{ nullptr };
    CuCtxGetCurrentFunction m_cuCtxGetCurrent{ nullptr };
    CuCtxSetCurrentFunction m_cuCtxSetCurrent{ nullptr };
    CuCtxGetDeviceFunction m_cuCtxGetDevice{ nullptr };
    CuMemAllocFunction m_cuMemAlloc{ nullptr };
    CuMemFreeFunction m_cuMemFree{ nullptr };
    CuMemcpyFunction m_cuMemcpy{ nullptr };
    CuMemcpyAsyncFunction m_cuMemcpyAsync{ nullptr };
    CuMemsetD8Function m_cuMemsetD8{ nullptr };
    CuEventCreateFunction m_cuEventCreate{ nullptr };
    CuEventDestroyFunction m_cuEventDestroy{ nullptr };
    CuEventRecordFunction m_cuEventRecord{ nullptr };
    CuEventQueryFunction m_cuEventQuery{ nullptr };
    CuEventSynchronizeFunction m_cuEventSynchronize{ nullptr };
    CuStreamWaitEventFunction m_cuStreamWaitEvent{ nullptr };
    CuStreamSynchronizeFunction m_cuStreamSynchronize{ nullptr };
    CuStreamGetDeviceFunction m_cuStreamGetDevice{ nullptr };
    CuStreamCreateFunction m_cuStreamCreate{ nullptr };
    CuStreamDestroyFunction m_cuStreamDestroy{ nullptr };
    CuLaunchHostFuncFunction m_cuLaunchHostFunc{ nullptr };
    CuPointerGetAttributeFunction m_cuPointerGetAttribute{ nullptr };
    CuGetErrorStringFunction m_cuGetErrorString{ nullptr };
};

} // namespace array
} // namespace common
} // namespace isaacsim
