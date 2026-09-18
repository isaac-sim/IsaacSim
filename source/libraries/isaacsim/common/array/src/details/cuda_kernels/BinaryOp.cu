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

#include "details/CudaKernel.hpp"
#include "details/KernelExport.hpp"

#include <isaacsim/common/array/Dtype.hpp>

#include <cstddef>
#include <cstdint>
#include <cuda_runtime_api.h>

using isaacsim::common::array::Dtype;
using isaacsim::common::array::details::BinaryOp;

namespace
{

// Boolean addition is a logical OR and boolean multiplication a logical AND, as in NumPy;
// subtraction and division of booleans are rejected on the host side and never reach a kernel.
template <typename T>
__device__ T applyOperation(T source1, T source2, BinaryOp operation)
{
    switch (operation)
    {
    case BinaryOp::eAdd:
        return static_cast<T>(source1 + source2);
    case BinaryOp::eSubtract:
        return static_cast<T>(source1 - source2);
    case BinaryOp::eMultiply:
        return static_cast<T>(source1 * source2);
    case BinaryOp::eDivide:
        return static_cast<T>(source1 / source2);
    }
    return T{};
}

template <>
__device__ bool applyOperation<bool>(bool source1, bool source2, BinaryOp operation)
{
    return operation == BinaryOp::eAdd ? (source1 || source2) : (source1 && source2);
}

template <typename T>
__global__ void binaryOpKernel(const T* source1, const T* source2, T* destination, size_t count, BinaryOp operation)
{
    const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i < count)
    {
        destination[i] = applyOperation<T>(source1[i], source2[i], operation);
    }
}

template <typename T>
cudaError_t launchKernel(
    const void* source1, const void* source2, void* destination, size_t count, BinaryOp operation, cudaStream_t stream)
{
    if (count == 0)
    {
        return cudaSuccess;
    }
    constexpr int threadsPerBlock = 256;
    const int blocks = static_cast<int>((count + threadsPerBlock - 1) / threadsPerBlock);
    binaryOpKernel<T><<<blocks, threadsPerBlock, 0, stream>>>(
        static_cast<const T*>(source1), static_cast<const T*>(source2), static_cast<T*>(destination), count, operation);
    return cudaGetLastError();
}

} // namespace


ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t binaryOpFunction(const void* source1,
                                                                 const void* source2,
                                                                 void* destination,
                                                                 size_t count,
                                                                 int32_t kind,
                                                                 int32_t operation,
                                                                 cudaStream_t stream)
{
    return Dtype::dispatchByKind(
        static_cast<Dtype::Kind>(kind),
        [&](auto sample) -> cudaError_t
        {
            using T = decltype(sample);
            return launchKernel<T>(source1, source2, destination, count, static_cast<BinaryOp>(operation), stream);
        });
}
