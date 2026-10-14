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
// The driver's own module name. Unlike the CUDA runtime redistributable, this is installed by the
// display driver into a directory the loader already searches.
constexpr const char* g_kCudaDriverLibraries[] = { "nvcuda.dll" };

void* loadLibrary(const char* name)
{
    return reinterpret_cast<void*>(::LoadLibraryA(name));
}

void* loadSymbol(void* library, const char* name)
{
    return reinterpret_cast<void*>(::GetProcAddress(reinterpret_cast<HMODULE>(library), name));
}
#else
// The soname the NVIDIA driver installs is `libcuda.so.1`; the unsuffixed name is a development
// symlink that only a full toolkit install provides. Trying the soname first also means that when
// another component in the process (Warp, for one) has already loaded the driver, the loader
// resolves this to the object it already mapped instead of searching the filesystem at all.
constexpr const char* g_kCudaDriverLibraries[] = { "libcuda.so.1", "libcuda.so" };

void* loadLibrary(const char* name)
{
    return ::dlopen(name, RTLD_LAZY | RTLD_LOCAL);
}

void* loadSymbol(void* library, const char* name)
{
    return ::dlsym(library, name);
}
#endif

// CUdeviceptr is an integer handle rather than a pointer. Under unified addressing it holds the same
// numeric value as the host-side pointer, which is what makes the casts below sound.
CUdeviceptr toDevicePointer(const void* pointer)
{
    return reinterpret_cast<CUdeviceptr>(pointer);
}

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
            "CudaRuntime::getInstance", "the CUDA driver was not loaded");
    }
    return s_instance;
}

bool CudaRuntime::isLoaded() const
{
    return m_handle != nullptr;
}

CUcontext CudaRuntime::_primaryContext(int device, CUresult* status) const
{
    *status = CUDA_SUCCESS;
    const std::lock_guard<std::mutex> guard(m_contextMutex);
    const auto cached = m_primaryContexts.find(device);
    if (cached != m_primaryContexts.end())
    {
        return cached->second;
    }

    CUdevice handle{};
    *status = m_cuDeviceGet(&handle, device);
    if (*status != CUDA_SUCCESS)
    {
        return nullptr;
    }
    CUcontext context{};
    *status = m_cuDevicePrimaryCtxRetain(&context, handle);
    if (*status != CUDA_SUCCESS)
    {
        return nullptr;
    }
    m_primaryContexts.emplace(device, context);
    return context;
}

bool CudaRuntime::_ensureContext(const char* callerName, bool throwIfInvalid) const
{
    if (!isLoaded())
    {
        return _reportUnavailable(callerName, throwIfInvalid);
    }

    CUcontext current{};
    if (m_cuCtxGetCurrent(&current) == CUDA_SUCCESS && current != nullptr)
    {
        return true;
    }

    // No context bound yet on this thread. The runtime API would have created device 0's primary
    // context here, so do the same rather than failing a first call that never named a device.
    CUresult status = CUDA_SUCCESS;
    CUcontext context = _primaryContext(0, &status);
    if (!context)
    {
        return _checkResult(status, callerName, throwIfInvalid);
    }
    return _checkResult(m_cuCtxSetCurrent(context), callerName, throwIfInvalid);
}

size_t CudaRuntime::cudaGetDeviceCount() const
{
    if (!isLoaded())
    {
        return 0;
    }
    int count = 0;
    if (m_cuDeviceGetCount(&count) != CUDA_SUCCESS || count < 0)
    {
        return 0;
    }
    return static_cast<size_t>(count);
}

bool CudaRuntime::cudaMalloc(void** devicePointer, size_t size, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaMalloc", throwIfInvalid))
    {
        return false;
    }
    CUdeviceptr allocation{};
    if (!_checkResult(m_cuMemAlloc(&allocation, size), "cudaMalloc", throwIfInvalid))
    {
        return false;
    }
    *devicePointer = reinterpret_cast<void*>(allocation);
    return true;
}

bool CudaRuntime::cudaFree(void* devicePointer, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaFree", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuMemFree(toDevicePointer(devicePointer)), "cudaFree", throwIfInvalid);
}

bool CudaRuntime::cudaMemcpy(void* destination, const void* source, size_t size, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaMemcpy", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(
        m_cuMemcpy(toDevicePointer(destination), toDevicePointer(source), size), "cudaMemcpy", throwIfInvalid);
}

bool CudaRuntime::cudaMemcpyAsync(void* destination, const void* source, size_t size, CUstream stream, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaMemcpyAsync", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuMemcpyAsync(toDevicePointer(destination), toDevicePointer(source), size, stream),
                        "cudaMemcpyAsync", throwIfInvalid);
}

bool CudaRuntime::cudaMemset(void* devicePointer, int value, size_t size, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaMemset", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuMemsetD8(toDevicePointer(devicePointer), static_cast<unsigned char>(value), size),
                        "cudaMemset", throwIfInvalid);
}

bool CudaRuntime::cudaGetDevice(int* device, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaGetDevice", throwIfInvalid))
    {
        return false;
    }
    CUdevice handle{};
    if (!_checkResult(m_cuCtxGetDevice(&handle), "cudaGetDevice", throwIfInvalid))
    {
        return false;
    }
    *device = static_cast<int>(handle);
    return true;
}

bool CudaRuntime::cudaSetDevice(int device, bool throwIfInvalid) const
{
    if (!isLoaded())
    {
        return _reportUnavailable("cudaSetDevice", throwIfInvalid);
    }
    CUresult status = CUDA_SUCCESS;
    CUcontext context = _primaryContext(device, &status);
    if (!context)
    {
        // Reported through _checkResult so an out-of-range ordinal still surfaces as an invalid-device
        // error with its code, rather than being flattened into "the driver was not loaded".
        return _checkResult(status, "cudaSetDevice", throwIfInvalid);
    }
    return _checkResult(m_cuCtxSetCurrent(context), "cudaSetDevice", throwIfInvalid);
}

bool CudaRuntime::cudaEventCreateWithFlags(CUevent* event, unsigned int flags, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaEventCreateWithFlags", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuEventCreate(event, flags), "cudaEventCreateWithFlags", throwIfInvalid);
}

bool CudaRuntime::cudaEventDestroy(CUevent event, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaEventDestroy", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuEventDestroy(event), "cudaEventDestroy", throwIfInvalid);
}

bool CudaRuntime::cudaEventRecord(CUevent event, CUstream stream, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaEventRecord", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuEventRecord(event, stream), "cudaEventRecord", throwIfInvalid);
}

CUresult CudaRuntime::cudaEventQuery(CUevent event) const
{
    if (!_ensureContext("cudaEventQuery", /*throwIfInvalid=*/false))
    {
        return CUDA_ERROR_NOT_INITIALIZED;
    }
    return m_cuEventQuery(event);
}

bool CudaRuntime::cudaEventSynchronize(CUevent event, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaEventSynchronize", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuEventSynchronize(event), "cudaEventSynchronize", throwIfInvalid);
}

bool CudaRuntime::cudaStreamWaitEvent(CUstream stream, CUevent event, unsigned int flags, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaStreamWaitEvent", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuStreamWaitEvent(stream, event, flags), "cudaStreamWaitEvent", throwIfInvalid);
}

bool CudaRuntime::cudaStreamSynchronize(CUstream stream, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaStreamSynchronize", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuStreamSynchronize(stream), "cudaStreamSynchronize", throwIfInvalid);
}

bool CudaRuntime::cudaStreamGetDevice(CUstream stream, int* device, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaStreamGetDevice", throwIfInvalid))
    {
        return false;
    }
    CUdevice handle{};
    if (!_checkResult(m_cuStreamGetDevice(stream, &handle), "cudaStreamGetDevice", throwIfInvalid))
    {
        return false;
    }
    *device = static_cast<int>(handle);
    return true;
}

bool CudaRuntime::cudaStreamCreateWithFlags(CUstream* stream, unsigned int flags, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaStreamCreateWithFlags", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuStreamCreate(stream, flags), "cudaStreamCreateWithFlags", throwIfInvalid);
}

bool CudaRuntime::cudaStreamDestroy(CUstream stream, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaStreamDestroy", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuStreamDestroy(stream), "cudaStreamDestroy", throwIfInvalid);
}

bool CudaRuntime::cudaLaunchHostFunc(CUstream stream, CUhostFn function, void* userData, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaLaunchHostFunc", throwIfInvalid))
    {
        return false;
    }
    return _checkResult(m_cuLaunchHostFunc(stream, function, userData), "cudaLaunchHostFunc", throwIfInvalid);
}

bool CudaRuntime::cudaPointerGetAttributes(PointerAttributes* attributes, const void* pointer, bool throwIfInvalid) const
{
    if (!_ensureContext("cudaPointerGetAttributes", throwIfInvalid))
    {
        return false;
    }

    // An ordinary host allocation is not registered with the driver, so this reports
    // CUDA_ERROR_INVALID_VALUE rather than describing it as host memory the way the runtime API did.
    // `attributes` is cleared either way, so a caller passing throwIfInvalid=false reads the answer
    // it wanted -- not device memory -- while the default still surfaces the driver's error.
    unsigned int memoryType = 0;
    const CUresult result =
        m_cuPointerGetAttribute(&memoryType, CU_POINTER_ATTRIBUTE_MEMORY_TYPE, toDevicePointer(pointer));
    if (result != CUDA_SUCCESS)
    {
        *attributes = PointerAttributes{};
        return _checkResult(result, "cudaPointerGetAttributes", throwIfInvalid);
    }

    attributes->isDevice = memoryType == CU_MEMORYTYPE_DEVICE || memoryType == CU_MEMORYTYPE_UNIFIED;

    int ordinal = -1;
    if (m_cuPointerGetAttribute(&ordinal, CU_POINTER_ATTRIBUTE_DEVICE_ORDINAL, toDevicePointer(pointer)) != CUDA_SUCCESS)
    {
        ordinal = -1;
    }
    attributes->device = ordinal;
    return true;
}

CudaRuntime::CudaRuntime()
{
    for (const char* library : g_kCudaDriverLibraries)
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

    m_cuInit = reinterpret_cast<CuInitFunction>(loadSymbol(m_handle, "cuInit"));
    m_cuDeviceGetCount = reinterpret_cast<CuDeviceGetCountFunction>(loadSymbol(m_handle, "cuDeviceGetCount"));
    m_cuDeviceGet = reinterpret_cast<CuDeviceGetFunction>(loadSymbol(m_handle, "cuDeviceGet"));
    m_cuDevicePrimaryCtxRetain =
        reinterpret_cast<CuDevicePrimaryCtxRetainFunction>(loadSymbol(m_handle, "cuDevicePrimaryCtxRetain"));
    m_cuCtxGetCurrent = reinterpret_cast<CuCtxGetCurrentFunction>(loadSymbol(m_handle, "cuCtxGetCurrent"));
    m_cuCtxSetCurrent = reinterpret_cast<CuCtxSetCurrentFunction>(loadSymbol(m_handle, "cuCtxSetCurrent"));
    m_cuCtxGetDevice = reinterpret_cast<CuCtxGetDeviceFunction>(loadSymbol(m_handle, "cuCtxGetDevice"));
    // The `_v2` names below are mandatory, never a preference with the plain name as a fallback.
    // `cuda.h` redirects the plain name to the revised one with a macro, which resolving by string
    // bypasses, and the superseded entry points are still exported under their original ABI: v1
    // `cuMemAlloc` takes a 32-bit `CUdeviceptr_v1*` and a 32-bit size, so reaching it through the
    // 64-bit signature declared here would fill only half of the returned pointer. Treating a
    // missing `_v2` as unresolved -- and therefore the whole driver as unusable -- is the safe
    // reading; every driver since CUDA 4.0 exports them.
    m_cuMemAlloc = reinterpret_cast<CuMemAllocFunction>(loadSymbol(m_handle, "cuMemAlloc_v2"));
    m_cuMemFree = reinterpret_cast<CuMemFreeFunction>(loadSymbol(m_handle, "cuMemFree_v2"));
    m_cuMemcpy = reinterpret_cast<CuMemcpyFunction>(loadSymbol(m_handle, "cuMemcpy"));
    m_cuMemcpyAsync = reinterpret_cast<CuMemcpyAsyncFunction>(loadSymbol(m_handle, "cuMemcpyAsync"));
    m_cuMemsetD8 = reinterpret_cast<CuMemsetD8Function>(loadSymbol(m_handle, "cuMemsetD8_v2"));
    m_cuEventCreate = reinterpret_cast<CuEventCreateFunction>(loadSymbol(m_handle, "cuEventCreate"));
    m_cuEventDestroy = reinterpret_cast<CuEventDestroyFunction>(loadSymbol(m_handle, "cuEventDestroy_v2"));
    m_cuEventRecord = reinterpret_cast<CuEventRecordFunction>(loadSymbol(m_handle, "cuEventRecord"));
    m_cuEventQuery = reinterpret_cast<CuEventQueryFunction>(loadSymbol(m_handle, "cuEventQuery"));
    m_cuEventSynchronize = reinterpret_cast<CuEventSynchronizeFunction>(loadSymbol(m_handle, "cuEventSynchronize"));
    m_cuStreamWaitEvent = reinterpret_cast<CuStreamWaitEventFunction>(loadSymbol(m_handle, "cuStreamWaitEvent"));
    m_cuStreamSynchronize = reinterpret_cast<CuStreamSynchronizeFunction>(loadSymbol(m_handle, "cuStreamSynchronize"));
    m_cuStreamGetDevice = reinterpret_cast<CuStreamGetDeviceFunction>(loadSymbol(m_handle, "cuStreamGetDevice"));
    m_cuStreamCreate = reinterpret_cast<CuStreamCreateFunction>(loadSymbol(m_handle, "cuStreamCreate"));
    m_cuStreamDestroy = reinterpret_cast<CuStreamDestroyFunction>(loadSymbol(m_handle, "cuStreamDestroy_v2"));
    m_cuLaunchHostFunc = reinterpret_cast<CuLaunchHostFuncFunction>(loadSymbol(m_handle, "cuLaunchHostFunc"));
    m_cuPointerGetAttribute =
        reinterpret_cast<CuPointerGetAttributeFunction>(loadSymbol(m_handle, "cuPointerGetAttribute"));
    m_cuGetErrorString = reinterpret_cast<CuGetErrorStringFunction>(loadSymbol(m_handle, "cuGetErrorString"));

    // Every entry point is mandatory: a driver missing any of them is not one this library can work
    // against, so it is treated as absent rather than left partially usable. cuInit() is part of that
    // check -- a driver that loads but cannot initialize (no GPU present, or a version mismatch) is
    // equally unusable, and reporting it here keeps every later call on the unavailable path.
    const bool resolved = m_cuInit && m_cuDeviceGetCount && m_cuDeviceGet && m_cuDevicePrimaryCtxRetain &&
                          m_cuCtxGetCurrent && m_cuCtxSetCurrent && m_cuCtxGetDevice && m_cuMemAlloc && m_cuMemFree &&
                          m_cuMemcpy && m_cuMemcpyAsync && m_cuMemsetD8 && m_cuEventCreate && m_cuEventDestroy &&
                          m_cuEventRecord && m_cuEventQuery && m_cuEventSynchronize && m_cuStreamWaitEvent &&
                          m_cuStreamSynchronize && m_cuStreamGetDevice && m_cuStreamCreate && m_cuStreamDestroy &&
                          m_cuLaunchHostFunc && m_cuPointerGetAttribute && m_cuGetErrorString;
    if (!resolved || m_cuInit(0) != CUDA_SUCCESS)
    {
        // Clearing the handle is all that is needed to disable the loader: isLoaded() reads it, and
        // every entry point gates on that before touching a function pointer. The module is left
        // mapped rather than unloaded, for the same reason the destructor does not unload it.
        m_handle = nullptr;
    }
}

CudaRuntime::~CudaRuntime()
{
    // The driver module is deliberately left loaded. This singleton is destroyed during static
    // teardown, by which point the retained primary contexts, the never-destroyed per-device
    // operation streams, and any events still held by quarantined storage are all live and owned by
    // the very module an unload would evict. Releasing it here is the larger version of the hazard
    // that already stops the primary contexts from being released, and the mapping costs nothing for
    // the remainder of a process that is exiting anyway.
}

bool CudaRuntime::_reportUnavailable(const char* callerName, bool throwIfInvalid) const
{
    if (throwIfInvalid)
    {
        throw isaacsim::common::exceptions::CudaRuntimeError(
            std::string("CudaRuntime::") + callerName, "CUDA driver was not loaded");
    }
    return false;
}

bool CudaRuntime::_checkResult(CUresult result, const char* callerName, bool throwIfInvalid) const
{
    if (result == CUDA_SUCCESS)
    {
        return true;
    }
    if (throwIfInvalid)
    {
        const char* errorString = nullptr;
        if (!m_cuGetErrorString || m_cuGetErrorString(result, &errorString) != CUDA_SUCCESS || !errorString)
        {
            errorString = "unknown error";
        }
        throw isaacsim::common::exceptions::CudaRuntimeError(
            std::string("CudaRuntime::") + callerName, errorString, static_cast<int>(result));
    }
    return false;
}

} // namespace array
} // namespace common
} // namespace isaacsim
