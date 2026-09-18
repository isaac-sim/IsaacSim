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

// `sourceStrides` and `destinationShape` are device-resident arrays of length `ndim`, aligned to
// the destination axes; a stride of 0 marks a broadcast axis (source size 1 or an added leading
// axis), mirroring the CPU gather in Array::broadcastTo().
__global__ void broadcastToKernel(const unsigned char* source,
                                  unsigned char* destination,
                                  const int64_t* sourceStrides,
                                  const int64_t* destinationShape,
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
    for (size_t i = ndim; i-- > 0;)
    {
        const int64_t dimSize = destinationShape[i];
        const int64_t index = static_cast<int64_t>(remaining % static_cast<size_t>(dimSize));
        remaining /= static_cast<size_t>(dimSize);
        sourceOffset += index * sourceStrides[i];
    }
    const unsigned char* sourceElement = source + static_cast<size_t>(sourceOffset) * elementSize;
    unsigned char* destinationElement = destination + linear * elementSize;
    for (size_t b = 0; b < elementSize; ++b)
    {
        destinationElement[b] = sourceElement[b];
    }
}

} // namespace

// `sourceStrides` and `destinationShape` are host arrays of length `ndim`; this uploads them to
// small device scratch buffers for the lifetime of the launch since the destination gather index
// must be recomputed by every thread.
ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t broadcastToFunction(const void* source,
                                                                    void* destination,
                                                                    const int64_t* sourceStrides,
                                                                    const int64_t* destinationShape,
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
        // 0-D destination: the single element always reads from source offset 0.
        return cudaMemcpyAsync(destination, source, elementSize, cudaMemcpyDeviceToDevice, stream);
    }

    int64_t* deviceSourceStrides = nullptr;
    int64_t* deviceDestinationShape = nullptr;
    cudaError_t result = cudaMalloc(&deviceSourceStrides, ndim * sizeof(int64_t));
    if (result != cudaSuccess)
    {
        return result;
    }
    result = cudaMalloc(&deviceDestinationShape, ndim * sizeof(int64_t));
    if (result != cudaSuccess)
    {
        cudaFree(deviceSourceStrides);
        return result;
    }
    result = cudaMemcpy(deviceSourceStrides, sourceStrides, ndim * sizeof(int64_t), cudaMemcpyHostToDevice);
    if (result == cudaSuccess)
    {
        result = cudaMemcpy(deviceDestinationShape, destinationShape, ndim * sizeof(int64_t), cudaMemcpyHostToDevice);
    }
    if (result == cudaSuccess)
    {
        constexpr int threadsPerBlock = 256;
        const int blocks = static_cast<int>((totalElements + threadsPerBlock - 1) / threadsPerBlock);
        broadcastToKernel<<<blocks, threadsPerBlock, 0, stream>>>(
            static_cast<const unsigned char*>(source), static_cast<unsigned char*>(destination), deviceSourceStrides,
            deviceDestinationShape, ndim, elementSize, totalElements);
        result = cudaGetLastError();
    }
    // The scratch buffers must outlive the (asynchronous) kernel launch; block here so they can be
    // freed immediately after instead of leaking their lifetime management into the caller. The
    // synchronization status is folded in because cudaGetLastError() above only reports launch
    // failures -- a fault during execution surfaces here, and dropping it would hand the caller a
    // destination full of uninitialized memory alongside cudaSuccess.
    const cudaError_t synchronizeResult = cudaStreamSynchronize(stream);
    if (result == cudaSuccess)
    {
        result = synchronizeResult;
    }
    const cudaError_t firstFreeResult = cudaFree(deviceSourceStrides);
    const cudaError_t secondFreeResult = cudaFree(deviceDestinationShape);
    if (result == cudaSuccess)
    {
        result = firstFreeResult != cudaSuccess ? firstFreeResult : secondFreeResult;
    }
    return result;
}
