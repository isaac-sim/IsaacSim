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

#include "CudaKernel.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>

#include <filesystem>
#include <string>

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
constexpr const char* g_kKernelLibraryFileName = "isaacsim-common-array-cuda-kernels.dll";

// Resolves the sibling kernel library next to the DLL this code itself lives in, so the lookup
// does not depend on the process' PATH or working directory.
std::filesystem::path siblingLibraryPath()
{
    HMODULE ownModule = nullptr;
    if (!::GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                              reinterpret_cast<LPCSTR>(&siblingLibraryPath), &ownModule))
    {
        return g_kKernelLibraryFileName;
    }
    char path[MAX_PATH]{};
    if (::GetModuleFileNameA(ownModule, path, MAX_PATH) == 0)
    {
        return g_kKernelLibraryFileName;
    }
    return std::filesystem::path(path).parent_path() / g_kKernelLibraryFileName;
}

void* loadLibrary(const char* name)
{
    return reinterpret_cast<void*>(::LoadLibraryA(name));
}

std::string lastLoadError()
{
    return "Windows error " + std::to_string(::GetLastError());
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
#    if defined(__APPLE__)
constexpr const char* g_kKernelLibraryFileName = "libisaacsim-common-array-cuda-kernels.dylib";
#    else
constexpr const char* g_kKernelLibraryFileName = "libisaacsim-common-array-cuda-kernels.so";
#    endif

// Resolves the sibling kernel library next to the shared object this code itself lives in, so
// the lookup does not depend on RPATH/RUNPATH propagation for bare-name dlopen() calls.
std::filesystem::path siblingLibraryPath()
{
    Dl_info info{};
    if (::dladdr(reinterpret_cast<const void*>(&siblingLibraryPath), &info) == 0 || !info.dli_fname)
    {
        return g_kKernelLibraryFileName;
    }
    return std::filesystem::path(info.dli_fname).parent_path() / g_kKernelLibraryFileName;
}

void* loadLibrary(const char* name)
{
    return ::dlopen(name, RTLD_LAZY | RTLD_LOCAL);
}

std::string lastLoadError()
{
    const char* error = ::dlerror();
    return error ? error : "unknown error";
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
namespace details
{

CudaKernel& CudaKernel::getInstance(bool throwIfInvalid)
{
    static CudaKernel s_instance;
    if (!s_instance.isLoaded() && throwIfInvalid)
    {
        throw isaacsim::common::exceptions::CudaRuntimeError(
            "CudaKernel::getInstance", "the CUDA kernel library was not loaded: " + s_instance.m_loadError);
    }
    return s_instance;
}

bool CudaKernel::isLoaded() const
{
    return m_handle != nullptr;
}

bool CudaKernel::cast(const void* source,
                      void* destination,
                      size_t count,
                      Dtype::Kind sourceKind,
                      Dtype::Kind destinationKind,
                      cudaStream_t stream,
                      bool throwIfInvalid) const
{
    if (!m_cast)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::cast", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    cudaError_t result = m_cast(
        source, destination, count, static_cast<int32_t>(sourceKind), static_cast<int32_t>(destinationKind), stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::cast", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

bool CudaKernel::broadcastTo(const void* source,
                             void* destination,
                             const std::vector<int64_t>& sourceStrides,
                             const std::vector<int64_t>& destinationShape,
                             size_t elementSize,
                             size_t totalElements,
                             cudaStream_t stream,
                             bool throwIfInvalid) const
{
    if (!m_broadcastTo)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::broadcastTo", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    if (sourceStrides.size() != destinationShape.size())
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError("CudaKernel::broadcastTo",
                                                                 "strides and shape have different lengths",
                                                                 static_cast<int>(cudaErrorInvalidValue));
        }
        return false;
    }
    cudaError_t result = m_broadcastTo(source, destination, sourceStrides.data(), destinationShape.data(),
                                       destinationShape.size(), elementSize, totalElements, stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::broadcastTo", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

bool CudaKernel::take(const void* source,
                      void* destination,
                      const std::vector<int64_t>& indices,
                      size_t axisSize,
                      size_t inner,
                      size_t elementSize,
                      size_t totalElements,
                      cudaStream_t stream,
                      bool throwIfInvalid) const
{
    if (!m_take)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::take", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    // takeKernel divides by `inner` and `indices.size()`, and only a zero `totalElements` keeps it
    // from running at all, so a non-empty destination whose extents do not multiply out is rejected
    // here rather than reaching a modulo by zero on the device.
    if (totalElements != 0 && (inner == 0 || indices.empty() || totalElements % (inner * indices.size()) != 0))
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::take", "the destination element count is not a multiple of indices.size() * inner",
                static_cast<int>(cudaErrorInvalidValue));
        }
        return false;
    }
    cudaError_t result =
        m_take(source, destination, indices.data(), axisSize, inner, indices.size(), elementSize, totalElements, stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::take", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

bool CudaKernel::binaryOp(const void* source1,
                          const void* source2,
                          void* destination,
                          size_t count,
                          Dtype::Kind kind,
                          BinaryOp operation,
                          cudaStream_t stream,
                          bool throwIfInvalid) const
{
    if (!m_binaryOp)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::binaryOp", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    cudaError_t result = m_binaryOp(
        source1, source2, destination, count, static_cast<int32_t>(kind), static_cast<int32_t>(operation), stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::binaryOp", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

bool CudaKernel::unaryOp(const void* source,
                         void* destination,
                         size_t count,
                         Dtype::Kind kind,
                         UnaryOp operation,
                         cudaStream_t stream,
                         bool throwIfInvalid) const
{
    if (!m_unaryOp)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::unaryOp", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    cudaError_t result =
        m_unaryOp(source, destination, count, static_cast<int32_t>(kind), static_cast<int32_t>(operation), stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::unaryOp", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

bool CudaKernel::reduceOp(const void* source,
                          void* destination,
                          size_t count,
                          Dtype::Kind kind,
                          ReduceOp operation,
                          cudaStream_t stream,
                          bool throwIfInvalid) const
{
    if (!m_reduceOp)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::reduceOp", "the CUDA kernel library was not loaded: " + m_loadError);
        }
        return false;
    }
    cudaError_t result =
        m_reduceOp(source, destination, count, static_cast<int32_t>(kind), static_cast<int32_t>(operation), stream);
    if (result != cudaSuccess)
    {
        if (throwIfInvalid)
        {
            throw isaacsim::common::exceptions::CudaRuntimeError(
                "CudaKernel::reduceOp", "kernel launch failed", static_cast<int>(result));
        }
        return false;
    }
    return true;
}

CudaKernel::CudaKernel()
{
    const std::string libraryPath = siblingLibraryPath().string();
    m_handle = loadLibrary(libraryPath.c_str());
    if (!m_handle)
    {
        m_loadError = "could not load '" + libraryPath + "': " + lastLoadError();
        return;
    }
    // A library that loads but exports nothing usually means a stale copy, or entry points built
    // without the export macro, so collect every name that failed rather than just the first.
    std::string missingSymbols;
    auto resolveSymbol = [&](const char* name)
    {
        void* symbol = loadSymbol(m_handle, name);
        if (!symbol)
        {
            if (!missingSymbols.empty())
            {
                missingSymbols += ", ";
            }
            missingSymbols += name;
        }
        return symbol;
    };

    m_cast = reinterpret_cast<CastFunction>(resolveSymbol("castFunction"));
    m_broadcastTo = reinterpret_cast<BroadcastToFunction>(resolveSymbol("broadcastToFunction"));
    m_take = reinterpret_cast<TakeFunction>(resolveSymbol("takeFunction"));
    m_binaryOp = reinterpret_cast<BinaryOpFunction>(resolveSymbol("binaryOpFunction"));
    m_unaryOp = reinterpret_cast<UnaryOpFunction>(resolveSymbol("unaryOpFunction"));
    m_reduceOp = reinterpret_cast<ReduceOpFunction>(resolveSymbol("reduceOpFunction"));

    if (!missingSymbols.empty())
    {
        // Every entry point is cleared, not just the ones that failed: the library is unloaded
        // below, so a pointer that did resolve would dangle into unmapped memory.
        m_loadError = "loaded '" + libraryPath + "' but could not resolve: " + missingSymbols;
        unloadLibrary(m_handle);
        m_handle = nullptr;
        m_cast = nullptr;
        m_broadcastTo = nullptr;
        m_take = nullptr;
        m_binaryOp = nullptr;
        m_unaryOp = nullptr;
        m_reduceOp = nullptr;
    }
}

CudaKernel::~CudaKernel()
{
    unloadLibrary(m_handle);
}

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim
