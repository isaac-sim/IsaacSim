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

#include "CudaRuntime.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>

#if defined(_WIN32)
#    if !defined(NOMINMAX)
#        define NOMINMAX
#    endif
#    if !defined(WIN32_LEAN_AND_MEAN)
#        define WIN32_LEAN_AND_MEAN
#    endif
#    include <windows.h>
#else
#    include <dlfcn.h>
#endif

namespace
{

#if defined(_WIN32)
constexpr const char* g_kCudaRuntimeLibraries[] = { "cudart64_12.dll" };

void* loadLibrary(const char* name)
{
    return reinterpret_cast<void*>(::LoadLibraryA(name));
}

void* loadSymbol(void* library, const char* name)
{
    return reinterpret_cast<void*>(::GetProcAddress(reinterpret_cast<HMODULE>(library), name));
}

void unloadLibrary(void* library)
{
    if (library)
    {
        ::FreeLibrary(reinterpret_cast<HMODULE>(library));
    }
}
#else
constexpr const char* g_kCudaRuntimeLibraries[] = { "libcudart.so" };

void* loadLibrary(const char* name)
{
    return ::dlopen(name, RTLD_LAZY | RTLD_LOCAL);
}

void* loadSymbol(void* library, const char* name)
{
    return ::dlsym(library, name);
}

void unloadLibrary(void* library)
{
    if (library)
    {
        ::dlclose(library);
    }
}
#endif

} // namespace

namespace isaacsim
{
namespace common
{
namespace array
{

CudaRuntime& CudaRuntime::getInstance(bool throwIfInvalid)
{
    static CudaRuntime s_instance;
    if (!s_instance.isLoaded() && throwIfInvalid)
    {
        throw isaacsim::common::exceptions::CudaRuntimeError(
            "CudaRuntime::getInstance", "the CUDA runtime was not loaded");
    }
    return s_instance;
}

bool CudaRuntime::isLoaded() const
{
    return m_handle != nullptr;
}

size_t CudaRuntime::cudaGetDeviceCount() const
{
    if (!m_cudaGetDeviceCount)
    {
        return 0;
    }
    int count = 0;
    m_cudaGetDeviceCount(&count);
    return count;
}

bool CudaRuntime::cudaMalloc(void** devicePointer, size_t size, bool throwIfInvalid) const
{
    if (!m_cudaMalloc)
    {
        return _reportUnavailable("cudaMalloc", throwIfInvalid);
    }
    return _checkResult(m_cudaMalloc(devicePointer, size), "cudaMalloc", throwIfInvalid);
}

bool CudaRuntime::cudaFree(void* devicePointer, bool throwIfInvalid) const
{
    if (!m_cudaFree)
    {
        return _reportUnavailable("cudaFree", throwIfInvalid);
    }
    return _checkResult(m_cudaFree(devicePointer), "cudaFree", throwIfInvalid);
}

bool CudaRuntime::cudaMemcpy(void* destination, const void* source, size_t size, cudaMemcpyKind kind, bool throwIfInvalid) const
{
    if (!m_cudaMemcpy)
    {
        return _reportUnavailable("cudaMemcpy", throwIfInvalid);
    }
    return _checkResult(m_cudaMemcpy(destination, source, size, kind), "cudaMemcpy", throwIfInvalid);
}

bool CudaRuntime::cudaMemset(void* devicePointer, int value, size_t size, bool throwIfInvalid) const
{
    if (!m_cudaMemset)
    {
        return _reportUnavailable("cudaMemset", throwIfInvalid);
    }
    return _checkResult(m_cudaMemset(devicePointer, value, size), "cudaMemset", throwIfInvalid);
}

bool CudaRuntime::cudaGetDevice(int* device, bool throwIfInvalid) const
{
    if (!m_cudaGetDevice)
    {
        return _reportUnavailable("cudaGetDevice", throwIfInvalid);
    }
    return _checkResult(m_cudaGetDevice(device), "cudaGetDevice", throwIfInvalid);
}

bool CudaRuntime::cudaSetDevice(int device, bool throwIfInvalid) const
{
    if (!m_cudaSetDevice)
    {
        return _reportUnavailable("cudaSetDevice", throwIfInvalid);
    }
    return _checkResult(m_cudaSetDevice(device), "cudaSetDevice", throwIfInvalid);
}

CudaRuntime::CudaRuntime()
{
    for (const char* library : g_kCudaRuntimeLibraries)
    {
        m_handle = loadLibrary(library);
        if (m_handle)
        {
            break;
        }
    }
    if (!m_handle)
    {
        return;
    }

    m_cudaGetDeviceCount = reinterpret_cast<CudaGetDeviceCountFunction>(loadSymbol(m_handle, "cudaGetDeviceCount"));
    m_cudaMalloc = reinterpret_cast<CudaMallocFunction>(loadSymbol(m_handle, "cudaMalloc"));
    m_cudaFree = reinterpret_cast<CudaFreeFunction>(loadSymbol(m_handle, "cudaFree"));
    m_cudaMemcpy = reinterpret_cast<CudaMemcpyFunction>(loadSymbol(m_handle, "cudaMemcpy"));
    m_cudaMemset = reinterpret_cast<CudaMemsetFunction>(loadSymbol(m_handle, "cudaMemset"));
    m_cudaGetDevice = reinterpret_cast<CudaGetDeviceFunction>(loadSymbol(m_handle, "cudaGetDevice"));
    m_cudaSetDevice = reinterpret_cast<CudaSetDeviceFunction>(loadSymbol(m_handle, "cudaSetDevice"));
    m_cudaGetErrorString = reinterpret_cast<CudaGetErrorStringFunction>(loadSymbol(m_handle, "cudaGetErrorString"));

    // Every entry point is mandatory: a runtime missing any of them is not one this library can
    // work against, so it is treated as absent rather than left partially usable.
    if (!m_cudaGetDeviceCount || !m_cudaMalloc || !m_cudaFree || !m_cudaMemcpy || !m_cudaMemset || !m_cudaGetDevice ||
        !m_cudaSetDevice || !m_cudaGetErrorString)
    {
        unloadLibrary(m_handle);
        m_handle = nullptr;
        m_cudaGetDeviceCount = nullptr;
        m_cudaMalloc = nullptr;
        m_cudaFree = nullptr;
        m_cudaMemcpy = nullptr;
        m_cudaMemset = nullptr;
        m_cudaGetDevice = nullptr;
        m_cudaSetDevice = nullptr;
        m_cudaGetErrorString = nullptr;
    }
}

CudaRuntime::~CudaRuntime()
{
    unloadLibrary(m_handle);
}

bool CudaRuntime::_reportUnavailable(const std::string& callerName, bool throwIfInvalid) const
{
    if (throwIfInvalid)
    {
        throw isaacsim::common::exceptions::CudaRuntimeError("CudaRuntime::" + callerName, "CUDA runtime was not loaded");
    }
    return false;
}

bool CudaRuntime::_checkResult(cudaError_t result, const std::string& callerName, bool throwIfInvalid) const
{
    if (result == cudaSuccess)
    {
        return true;
    }
    if (throwIfInvalid)
    {
        const char* errorString = m_cudaGetErrorString ? m_cudaGetErrorString(result) : "unknown error";
        throw isaacsim::common::exceptions::CudaRuntimeError(
            "CudaRuntime::" + callerName, errorString, static_cast<int>(result));
    }
    return false;
}

} // namespace array
} // namespace common
} // namespace isaacsim
