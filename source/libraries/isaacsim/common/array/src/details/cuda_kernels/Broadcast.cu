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
//
// Compiled by nvcc into the standalone `isaacsim-common-array-cuda-kernels` shared library.
// Never linked into `isaacsim-common-array` itself -- see CudaKernel.hpp.

#include "details/KernelExport.hpp"

#include <cstddef>
#include <cstdint>
#include <cuda_runtime_api.h>

namespace
{

// `sourceStrides` and `destinationStrides` are device-resident arrays of length `ndim`, aligned to
// the shared logical shape; a source stride of 0 marks a broadcast axis (source size 1, or an axis
// absent from the source that was implicitly prepended), so every position along that axis reads
// the same source element. Contiguous destination strides make this a compacting gather.
__global__ void copyStridedKernel(const unsigned char* source,
                                  unsigned char* destination,
                                  const int64_t* sourceStrides,
                                  const int64_t* destinationStrides,
                                  const int64_t* shape,
                                  size_t ndim,
                                  size_t elementSize,
                                  size_t totalElements)
{
    const size_t linear = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (linear >= totalElements)
    {
        return;
    }
    size_t remaining = linear;
    int64_t sourceOffset = 0;
    int64_t destinationOffset = 0;
    for (size_t axis = ndim; axis-- > 0;)
    {
        const int64_t index = static_cast<int64_t>(remaining % static_cast<size_t>(shape[axis]));
        remaining /= static_cast<size_t>(shape[axis]);
        sourceOffset += index * sourceStrides[axis];
        destinationOffset += index * destinationStrides[axis];
    }
    const unsigned char* sourceElement = source + static_cast<size_t>(sourceOffset) * elementSize;
    unsigned char* destinationElement = destination + static_cast<size_t>(destinationOffset) * elementSize;
    for (size_t byte = 0; byte < elementSize; ++byte)
    {
        destinationElement[byte] = sourceElement[byte];
    }
}

} // namespace

// The three host arrays share one device scratch buffer for the lifetime of the launch, since every
// thread has to recompute its own multi-index from `shape` and both stride vectors. Nothing is
// allocated on the host: this is an `extern "C"` entry point, so an escaping exception would cross
// a module boundary.
ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t copyStridedFunction(const void* source,
                                                                    void* destination,
                                                                    const int64_t* sourceStrides,
                                                                    const int64_t* destinationStrides,
                                                                    const int64_t* shape,
                                                                    size_t ndim,
                                                                    size_t elementSize,
                                                                    size_t totalElements,
                                                                    cudaStream_t stream)
{
    if (totalElements == 0)
    {
        return cudaSuccess;
    }
    if (ndim == 0)
    {
        // 0-D: the single element always sits at offset 0 on both sides.
        return cudaMemcpyAsync(destination, source, elementSize, cudaMemcpyDeviceToDevice, stream);
    }

    const size_t axisBytes = ndim * sizeof(int64_t);
    int64_t* deviceMetadata = nullptr;
    cudaError_t result = cudaMalloc(&deviceMetadata, 3 * axisBytes);
    if (result != cudaSuccess)
    {
        return result;
    }
    const int64_t* const hostMetadata[] = { sourceStrides, destinationStrides, shape };
    for (size_t part = 0; part < 3 && result == cudaSuccess; ++part)
    {
        result = cudaMemcpy(deviceMetadata + part * ndim, hostMetadata[part], axisBytes, cudaMemcpyHostToDevice);
    }
    if (result == cudaSuccess)
    {
        constexpr int threadsPerBlock = 256;
        const int blocks = static_cast<int>((totalElements + threadsPerBlock - 1) / threadsPerBlock);
        copyStridedKernel<<<blocks, threadsPerBlock, 0, stream>>>(
            static_cast<const unsigned char*>(source), static_cast<unsigned char*>(destination), deviceMetadata,
            deviceMetadata + ndim, deviceMetadata + 2 * ndim, ndim, elementSize, totalElements);
        result = cudaGetLastError();
    }
    // The scratch buffer must outlive the (asynchronous) kernel launch; block here so it can be
    // freed immediately after instead of leaking its lifetime management into the caller. The
    // synchronization status is folded in because cudaGetLastError() above only reports launch
    // failures -- a fault during execution surfaces here, and dropping it would hand the caller a
    // destination full of uninitialized memory alongside cudaSuccess.
    const cudaError_t synchronizeResult = cudaStreamSynchronize(stream);
    if (result == cudaSuccess)
    {
        result = synchronizeResult;
    }
    const cudaError_t freeResult = cudaFree(deviceMetadata);
    return result == cudaSuccess ? freeResult : result;
}
