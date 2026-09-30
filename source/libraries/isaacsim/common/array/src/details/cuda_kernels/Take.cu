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

// Index lists up to this length travel as a by-value kernel argument instead of a device
// allocation. The motivating cases are small -- a quaternion component reorder is 4 entries -- and
// the scratch buffer that a device-resident list needs costs a cudaMalloc, a host-to-device copy,
// a blocking stream synchronization, and a cudaFree, all of which dwarf the copy itself. 64 entries
// is 512 bytes, comfortably inside the 4 KB kernel parameter limit alongside the other arguments.
constexpr size_t kMaxInlineIndices = 64;

struct InlineIndices
{
    int64_t values[kMaxInlineIndices];
};

// Both operands are viewed as three-dimensional: the source as `(outer, axisSize, inner)` and the
// destination as `(outer, indexCount, inner)`, so that
// `destination[o][j][i] == source[o][indices[j]][i]`. `outer` never appears explicitly because it
// is recoverable from `totalElements`, which is `outer * indexCount * inner`.
//
// `indices` is already resolved to `[0, axisSize)` by the host, and is read either from
// `inlineIndices` or, when the list is too long to pass by value, from the device-resident
// `deviceIndices`.
__global__ void takeKernel(const unsigned char* source,
                           unsigned char* destination,
                           InlineIndices inlineIndices,
                           const int64_t* deviceIndices,
                           size_t axisSize,
                           size_t inner,
                           size_t indexCount,
                           size_t elementSize,
                           size_t totalElements)
{
    const size_t linear = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (linear >= totalElements)
    {
        return;
    }
    const size_t innerIndex = linear % inner;
    const size_t remaining = linear / inner;
    const size_t position = remaining % indexCount;
    const size_t outerIndex = remaining / indexCount;

    const int64_t index = deviceIndices ? deviceIndices[position] : inlineIndices.values[position];
    const size_t sourceIndex = (outerIndex * axisSize + static_cast<size_t>(index)) * inner + innerIndex;
    const unsigned char* sourceElement = source + sourceIndex * elementSize;
    unsigned char* destinationElement = destination + linear * elementSize;
    for (size_t byteIndex = 0; byteIndex < elementSize; ++byteIndex)
    {
        destinationElement[byteIndex] = sourceElement[byteIndex];
    }
}

} // namespace

// `indices` is a host array of length `indexCount`. Short lists ride along as a by-value kernel
// argument; longer ones are uploaded to a device scratch buffer for the lifetime of the launch,
// since every thread has to look its own index up.
ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t takeFunction(const void* source,
                                                             void* destination,
                                                             const int64_t* indices,
                                                             size_t axisSize,
                                                             size_t inner,
                                                             size_t indexCount,
                                                             size_t elementSize,
                                                             size_t totalElements,
                                                             cudaStream_t stream)
{
    // `inner` and `indexCount` are the kernel's divisors, and both are non-zero whenever the
    // destination holds at least one element, so the empty case must short-circuit here.
    if (totalElements == 0)
    {
        return cudaSuccess;
    }

    constexpr int threadsPerBlock = 256;
    const int blocks = static_cast<int>((totalElements + threadsPerBlock - 1) / threadsPerBlock);

    // Short list: no allocation, no upload, and no synchronization, so the launch stays purely
    // stream-ordered and the caller's later work on the same stream sees the result without this
    // function ever blocking.
    if (indexCount <= kMaxInlineIndices)
    {
        InlineIndices inlineIndices{};
        for (size_t i = 0; i < indexCount; ++i)
        {
            inlineIndices.values[i] = indices[i];
        }
        takeKernel<<<blocks, threadsPerBlock, 0, stream>>>(
            static_cast<const unsigned char*>(source), static_cast<unsigned char*>(destination), inlineIndices, nullptr,
            axisSize, inner, indexCount, elementSize, totalElements);
        return cudaGetLastError();
    }

    int64_t* deviceIndices = nullptr;
    cudaError_t result = cudaMalloc(&deviceIndices, indexCount * sizeof(int64_t));
    if (result != cudaSuccess)
    {
        return result;
    }
    // Ordered on `stream` rather than the default stream, so the kernel below is guaranteed to see
    // the uploaded indices even when the caller passes a non-blocking stream.
    result = cudaMemcpyAsync(deviceIndices, indices, indexCount * sizeof(int64_t), cudaMemcpyHostToDevice, stream);
    if (result == cudaSuccess)
    {
        takeKernel<<<blocks, threadsPerBlock, 0, stream>>>(
            static_cast<const unsigned char*>(source), static_cast<unsigned char*>(destination), InlineIndices{},
            deviceIndices, axisSize, inner, indexCount, elementSize, totalElements);
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
    const cudaError_t freeResult = cudaFree(deviceIndices);
    if (result == cudaSuccess)
    {
        result = freeResult;
    }
    return result;
}
