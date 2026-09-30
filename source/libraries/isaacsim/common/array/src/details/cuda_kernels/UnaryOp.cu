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
#include <type_traits>

using isaacsim::common::array::Dtype;
using isaacsim::common::array::details::UnaryOp;

namespace
{

// Negation of a boolean is rejected on the host side and never reaches a kernel; absolute value of
// a boolean is the identity, as in NumPy.
template <typename T>
__device__ T applyOperation(T source, UnaryOp operation)
{
    if constexpr (std::is_same_v<T, bool>)
    {
        return source;
    }
    else if constexpr (std::is_unsigned_v<T>)
    {
        // Unsigned values are never negative, so the absolute value is the identity; negation wraps
        // around modulo 2^N, matching NumPy.
        return operation == UnaryOp::eNegative ? static_cast<T>(T{} - source) : source;
    }
    else if constexpr (std::is_floating_point_v<T>)
    {
        // fabs() rather than a comparison, so that the absolute value of -0.0 is +0.0.
        return operation == UnaryOp::eNegative ? static_cast<T>(-source) : static_cast<T>(::fabs(source));
    }
    else
    {
        return operation == UnaryOp::eNegative ? static_cast<T>(-source) :
                                                 static_cast<T>(source < T{} ? -source : source);
    }
}

// `destination` is untyped because eLogicalNot narrows every input kind to a boolean result, while
// the arithmetic operations keep the source kind.
template <typename T>
__global__ void unaryOpKernel(const T* source, void* destination, size_t count, UnaryOp operation)
{
    const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= count)
    {
        return;
    }
    if (operation == UnaryOp::eLogicalNot)
    {
        static_cast<bool*>(destination)[i] = !static_cast<bool>(source[i]);
    }
    else
    {
        static_cast<T*>(destination)[i] = applyOperation<T>(source[i], operation);
    }
}

template <typename T>
cudaError_t launchKernel(const void* source, void* destination, size_t count, UnaryOp operation, cudaStream_t stream)
{
    if (count == 0)
    {
        return cudaSuccess;
    }
    constexpr int threadsPerBlock = 256;
    const int blocks = static_cast<int>((count + threadsPerBlock - 1) / threadsPerBlock);
    unaryOpKernel<T>
        <<<blocks, threadsPerBlock, 0, stream>>>(static_cast<const T*>(source), destination, count, operation);
    return cudaGetLastError();
}

} // namespace


ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t unaryOpFunction(
    const void* source, void* destination, size_t count, int32_t kind, int32_t operation, cudaStream_t stream)
{
    return Dtype::dispatchByKind(static_cast<Dtype::Kind>(kind),
                                 [&](auto sample) -> cudaError_t
                                 {
                                     using T = decltype(sample);
                                     return launchKernel<T>(
                                         source, destination, count, static_cast<UnaryOp>(operation), stream);
                                 });
}
