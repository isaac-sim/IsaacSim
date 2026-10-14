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

// Shared utility functions for Newton tensor views.
// Provides index resolution for CPU (resolveViewIndices) and GPU (resolveGpuViewIndices)
// paths, source data staging (ensureGpuSrc), and CUDA memory helpers.

#include "gpu/CudaKernels.hpp"

#include <carb/logging/Log.h>

#include <omni/physics/tensors/TensorDesc.h>
#include <omni/physics/tensors/TensorUtils.h>

#include <cstdint>
#include <cstring>
#include <cuda_runtime.h>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::getTensorDtypeCstr;
using omni::physics::tensors::getTensorTotalSize;
using omni::physics::tensors::TensorDataType;
using omni::physics::tensors::TensorDesc;

/// Validates that a tensor is non-null, resides on the expected device, is float32, and has
/// an exact total element count. Logs a descriptive error and returns false on any mismatch.
inline bool validateFloat32Tensor(
    const TensorDesc* tensor, int expectedDevice, size_t expectedElements, const char* tensorName, const char* funcName)
{
    if (!tensor || !tensor->data)
    {
        CARB_LOG_ERROR("%s tensor is null or has no data in %s", tensorName, funcName);
        return false;
    }
    if (!checkTensorDevice(*tensor, expectedDevice, tensorName, funcName) ||
        !checkTensorFloat32(*tensor, tensorName, funcName) ||
        !checkTensorSizeExact(*tensor, expectedElements, tensorName, funcName))
    {
        return false;
    }
    return true;
}

/// Validates that a tensor is non-null, resides on the expected device, and has an exact
/// total element count. The data type check is skipped (caller should pick the appropriate
/// variant above when a dtype constraint applies).
inline bool validateUint8Tensor(
    const TensorDesc* tensor, int expectedDevice, size_t expectedElements, const char* tensorName, const char* funcName)
{
    if (!tensor || !tensor->data)
    {
        CARB_LOG_ERROR("%s tensor is null or has no data in %s", tensorName, funcName);
        return false;
    }
    if (!checkTensorDevice(*tensor, expectedDevice, tensorName, funcName) ||
        !checkTensorSizeExact(*tensor, expectedElements, tensorName, funcName))
    {
        return false;
    }
    if (tensor->dtype != TensorDataType::eUint8 && tensor->dtype != TensorDataType::eInt8)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 8-bit integer, received %s",
                       tensorName, funcName, getTensorDtypeCstr(*tensor));
        return false;
    }
    return true;
}

/// Validates a uint8/int8 tensor's dtype and size but allows the tensor to reside on any device.
inline bool validateUint8TensorAnyDevice(const TensorDesc* tensor,
                                         size_t expectedElements,
                                         const char* tensorName,
                                         const char* funcName)
{
    if (!tensor || !tensor->data)
    {
        CARB_LOG_ERROR("%s tensor is null or has no data in %s", tensorName, funcName);
        return false;
    }
    if (!checkTensorSizeExact(*tensor, expectedElements, tensorName, funcName))
    {
        return false;
    }
    if (tensor->dtype != TensorDataType::eUint8 && tensor->dtype != TensorDataType::eInt8)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 8-bit integer, received %s",
                       tensorName, funcName, getTensorDtypeCstr(*tensor));
        return false;
    }
    return true;
}

/// Converts a boolean mask to view indices and invokes an indexed setter.
/// GPU masks are converted in place on the device to avoid host synchronization.
template <typename Setter>
bool applyMaskToSetter(const TensorDesc* maskTensor,
                       uint32_t viewCount,
                       std::vector<uint32_t>& scratchMaskIndices,
                       int modelDeviceOrdinal,
                       int* deviceIndexScratch,
                       const char* funcName,
                       Setter&& setter)
{
    if (!validateUint8TensorAnyDevice(maskTensor, viewCount, "mask", funcName))
    {
        return false;
    }

    if (maskTensor->device >= 0)
    {
        if (modelDeviceOrdinal < 0 || !deviceIndexScratch)
        {
            CARB_LOG_ERROR("GPU mask tensor is not supported by the CPU view in %s", funcName);
            return false;
        }
        if (maskTensor->device != modelDeviceOrdinal)
        {
            CARB_LOG_ERROR("Incompatible device of mask tensor in function %s: expected %d, received %d", funcName,
                           modelDeviceOrdinal, maskTensor->device);
            return false;
        }
        if (!launchMaskToIndices(
                static_cast<const uint8_t*>(maskTensor->data), deviceIndexScratch, static_cast<int>(viewCount)))
        {
            return false;
        }

        // Keep one slot per view row. Masked rows contain the -1 sentinel and are
        // skipped by GPU scatter kernels, avoiding a device-to-host selected-count read.
        TensorDesc indexTensor{};
        indexTensor.device = modelDeviceOrdinal;
        indexTensor.dtype = TensorDataType::eInt32;
        indexTensor.numDims = 1;
        indexTensor.dims[0] = static_cast<int>(viewCount);
        indexTensor.data = deviceIndexScratch;
        indexTensor.ownData = false;
        return std::forward<Setter>(setter)(&indexTensor);
    }

    const auto* maskValues = static_cast<const uint8_t*>(maskTensor->data);
    scratchMaskIndices.clear();
    scratchMaskIndices.reserve(viewCount);
    for (uint32_t i = 0; i < viewCount; ++i)
    {
        if (maskValues[i] != 0)
        {
            scratchMaskIndices.push_back(i);
        }
    }

    if (scratchMaskIndices.empty())
    {
        return true;
    }
    if (scratchMaskIndices.size() == viewCount)
    {
        return std::forward<Setter>(setter)(nullptr);
    }

    TensorDesc indexTensor{};
    indexTensor.device = -1;
    indexTensor.dtype = TensorDataType::eInt32;
    indexTensor.numDims = 1;
    indexTensor.dims[0] = static_cast<int>(scratchMaskIndices.size());
    indexTensor.data = scratchMaskIndices.data();
    indexTensor.ownData = false;
    return std::forward<Setter>(setter)(&indexTensor);
}

/// Validates an optional index tensor. If provided, it must reside on the expected device
/// and be int32/uint32. A null/empty index tensor is accepted (identity mapping).
inline bool validateOptionalIndexTensor(const TensorDesc* indexTensor, int expectedDevice, const char* funcName)
{
    if (!indexTensor || !indexTensor->data)
        return true;
    if (!checkTensorDevice(*indexTensor, expectedDevice, "index", funcName) ||
        !checkTensorInt32(*indexTensor, "index", funcName))
    {
        return false;
    }
    return true;
}

/// Validates a float32 tensor's dtype and size but allows the tensor to reside on any device.
/// Used by GPU views that internally stage CPU tensors via H2D copy (GC configuration).
inline bool validateFloat32TensorAnyDevice(const TensorDesc* tensor,
                                           size_t expectedElements,
                                           const char* tensorName,
                                           const char* funcName)
{
    if (!tensor || !tensor->data)
    {
        CARB_LOG_ERROR("%s tensor is null or has no data in %s", tensorName, funcName);
        return false;
    }
    if (!checkTensorFloat32(*tensor, tensorName, funcName) ||
        !checkTensorSizeExact(*tensor, expectedElements, tensorName, funcName))
    {
        return false;
    }
    return true;
}

/// Validates an optional index tensor's dtype (int32/uint32) but allows any device.
/// Used by GPU views that internally stage CPU index tensors via H2D copy (GC configuration).
inline bool validateOptionalIndexTensorAnyDevice(const TensorDesc* indexTensor, const char* funcName)
{
    if (!indexTensor || !indexTensor->data)
        return true;
    if (!checkTensorInt32(*indexTensor, "index", funcName))
        return false;
    return true;
}

/// Resolves an index tensor into a list of uint32_t view indices.
/// If indexTensor is null, fills out with [0, 1, ..., defaultCount-1].
/// Writes into the caller-provided vector to avoid per-call allocation.
/// Handles both CPU and GPU source tensors (D2H copy for GPU).
inline void resolveViewIndices(const TensorDesc* indexTensor, uint32_t defaultCount, std::vector<uint32_t>& out)
{
    out.clear();
    if (indexTensor && indexTensor->data)
    {
        uint32_t numIndices = static_cast<uint32_t>(getTensorTotalSize(*indexTensor));
        out.resize(numIndices);
        static_assert(sizeof(int32_t) == sizeof(uint32_t), "index size mismatch");
        if (indexTensor->device >= 0)
        {
            cudaMemcpy(out.data(), indexTensor->data, numIndices * sizeof(uint32_t), cudaMemcpyDeviceToHost);
        }
        else
        {
            std::memcpy(out.data(), indexTensor->data, numIndices * sizeof(uint32_t));
        }
    }
    else
    {
        out.resize(defaultCount);
        for (uint32_t i = 0; i < defaultCount; ++i)
            out[i] = i;
    }
}

/// Holds a device-side index pointer and count for GPU kernel dispatch.
struct GpuIndexGuard
{
    const int* ptr = nullptr; ///< Device pointer to indices, or nullptr for identity (all bodies).
    int count = 0; ///< Number of indices.
};

/// Resolves an index tensor for GPU kernel use. If the tensor is on GPU, returns its
/// pointer directly. If on CPU and devScratchBuf is provided, performs H2D copy into
/// the scratch buffer. If indexTensor is null, returns count = defaultCount with ptr = nullptr
/// (signaling identity/all-elements to the kernel).
inline GpuIndexGuard resolveGpuViewIndices(const TensorDesc* indexTensor,
                                           uint32_t defaultCount,
                                           int* devScratchBuf = nullptr)
{
    GpuIndexGuard g;
    if (!indexTensor || !indexTensor->data)
    {
        g.count = static_cast<int>(defaultCount);
        return g;
    }
    g.count = 1;
    for (int i = 0; i < indexTensor->numDims; ++i)
        g.count *= indexTensor->dims[i];
    if (indexTensor->device >= 0)
    {
        g.ptr = static_cast<const int*>(indexTensor->data);
    }
    else if (devScratchBuf)
    {
        cudaError_t err = cudaMemcpy(
            devScratchBuf, indexTensor->data, static_cast<size_t>(g.count) * sizeof(int), cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            CARB_LOG_ERROR("Failed to upload CPU index tensor to GPU: %s", cudaGetErrorString(err));
            (void)cudaGetLastError();
            g.ptr = nullptr;
            g.count = 0;
        }
        else
        {
            g.ptr = devScratchBuf;
        }
    }
    else
    {
        CARB_LOG_ERROR("GPU view received a CPU index tensor but no device scratch buffer is available");
        g.ptr = nullptr;
        g.count = 0;
    }
    return g;
}

/// Ensures source data resides on the GPU. If srcTensor is already on device, returns
/// its pointer. If on CPU, performs H2D copy into devStagingBuf. Returns nullptr on error.
inline const float* ensureGpuSrc(const TensorDesc* srcTensor,
                                 float* devStagingBuf,
                                 size_t maxFloats,
                                 size_t stagingOffset = 0)
{
    if (!srcTensor || !srcTensor->data)
        return nullptr;
    if (srcTensor->device >= 0)
        return static_cast<const float*>(srcTensor->data);
    if (!devStagingBuf)
    {
        CARB_LOG_ERROR("GPU view received CPU source tensor but no staging buffer is available");
        return nullptr;
    }
    uint32_t totalFloats = 1;
    for (int i = 0; i < srcTensor->numDims; ++i)
        totalFloats *= srcTensor->dims[i];
    if (stagingOffset > maxFloats || totalFloats > maxFloats - stagingOffset)
    {
        CARB_LOG_ERROR("CPU source tensor (%u floats at offset %zu) exceeds staging buffer (%zu floats)", totalFloats,
                       stagingOffset, maxFloats);
        return nullptr;
    }
    float* stagingDestination = devStagingBuf + stagingOffset;
    cudaError_t err = cudaMemcpy(
        stagingDestination, srcTensor->data, static_cast<size_t>(totalFloats) * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        CARB_LOG_ERROR("Failed to upload CPU source tensor to GPU: %s", cudaGetErrorString(err));
        (void)cudaGetLastError();
        return nullptr;
    }
    return stagingDestination;
}

/// Safely frees a CUDA pointer, clearing any pending error state.
inline void safeCudaFree(void* p)
{
    if (p)
    {
        cudaFree(p);
        (void)cudaGetLastError();
    }
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
