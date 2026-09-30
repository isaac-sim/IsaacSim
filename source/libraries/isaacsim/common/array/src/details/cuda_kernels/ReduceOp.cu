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
using isaacsim::common::array::details::ReduceOp;

namespace
{

constexpr int g_kThreadsPerBlock = 256;

// Every kernel below reduces the whole array from a single block: every thread walks the input
// with a block-wide stride and folds its share through `Op::combine`, then the per-thread results
// are combined through shared memory, so the result is produced by one launch with no cross-block
// atomics and no scratch allocation. Each sibling pair (all/any, min/max, sum/prod) shares one
// kernel body, templated on a compile-time bool that picks the operation within the pair, so the
// per-element loop carries no runtime branch on `operation`.

// Shared block-wide tree reduction, common to every kernel below: fold `value` into `shared`,
// synchronize, then halve the active range until a single combined result remains in `shared[0]`.
template <typename T, typename Op>
__device__ __forceinline__ T blockReduce(T value, T* shared)
{
    const int threadId = static_cast<int>(threadIdx.x);
    shared[threadId] = value;
    __syncthreads();

    for (int stride = g_kThreadsPerBlock / 2; stride > 0; stride >>= 1)
    {
        if (threadId < stride)
        {
            shared[threadId] = Op::combine(shared[threadId], shared[threadId + stride]);
        }
        __syncthreads();
    }
    return shared[0];
}

// eAll and eAny: the result is a boolean for every operand kind. Folds as int rather than bool:
// shared arrays of bool are not guaranteed to be free of bank-conflict padding surprises, and the
// tree reduction only needs a 0/1 flag.
template <bool IsAll>
struct AllAnyOp
{
    static __device__ __forceinline__ int identity()
    {
        return IsAll ? 1 : 0;
    }
    static __device__ __forceinline__ int combine(int a, int b)
    {
        if constexpr (IsAll)
        {
            return a && b;
        }
        else
        {
            return a || b;
        }
    }
};

// eMin and eMax: the result is an element of T, seeded from element 0 since the extremum
// reductions have no representable identity. The host rejects an empty operand, so element 0
// always exists, and folding in an element the array already contains cannot change the result.
// This is the only pair booleans reach: sum and product promote a boolean operand to an integer
// accumulator on the host side, as in NumPy, so `T` is never `bool` for the other two kernels.
template <typename T, bool IsMin>
struct MinMaxOp
{
    static __device__ __forceinline__ T identity(const T* source)
    {
        return source[0];
    }
    static __device__ __forceinline__ T combine(T a, T b)
    {
        // NumPy propagates NaN through the extremum reductions. Every comparison against NaN is
        // false, so it would otherwise be silently skipped, and it has to be tested for explicitly.
        if constexpr (std::is_floating_point_v<T>)
        {
            if (a != a)
            {
                return a;
            }
            if (b != b)
            {
                return b;
            }
        }
        if constexpr (IsMin)
        {
            return b < a ? b : a;
        }
        else
        {
            return a < b ? b : a;
        }
    }
};

// eSum and eProd: the result is an element of T, which the host has already promoted to the
// accumulator kind.
template <typename T, bool IsSum>
struct SumProdOp
{
    static __device__ __forceinline__ T identity()
    {
        if constexpr (IsSum)
        {
            return T{};
        }
        else
        {
            return static_cast<T>(1);
        }
    }
    static __device__ __forceinline__ T combine(T a, T b)
    {
        if constexpr (IsSum)
        {
            return static_cast<T>(a + b);
        }
        else
        {
            return static_cast<T>(a * b);
        }
    }
};

template <typename T, bool IsAll>
__global__ void allAnyReduceKernel(const T* source, bool* destination, size_t count)
{
    using Op = AllAnyOp<IsAll>;
    __shared__ int sharedFlags[g_kThreadsPerBlock];

    const int threadId = static_cast<int>(threadIdx.x);
    int result = Op::identity();
    for (size_t i = threadId; i < count; i += blockDim.x)
    {
        result = Op::combine(result, static_cast<bool>(source[i]) ? 1 : 0);
    }
    result = blockReduce<int, Op>(result, sharedFlags);

    if (threadId == 0)
    {
        *destination = result != 0;
    }
}

template <typename T, bool IsMin>
__global__ void minMaxReduceKernel(const T* source, T* destination, size_t count)
{
    using Op = MinMaxOp<T, IsMin>;
    __shared__ T sharedValues[g_kThreadsPerBlock];

    const int threadId = static_cast<int>(threadIdx.x);
    T result = Op::identity(source);
    for (size_t i = threadId; i < count; i += blockDim.x)
    {
        result = Op::combine(result, source[i]);
    }
    result = blockReduce<T, Op>(result, sharedValues);

    if (threadId == 0)
    {
        *destination = result;
    }
}

template <typename T, bool IsSum>
__global__ void sumProdReduceKernel(const T* source, T* destination, size_t count)
{
    using Op = SumProdOp<T, IsSum>;
    __shared__ T sharedValues[g_kThreadsPerBlock];

    const int threadId = static_cast<int>(threadIdx.x);
    T result = Op::identity();
    for (size_t i = threadId; i < count; i += blockDim.x)
    {
        result = Op::combine(result, source[i]);
    }
    result = blockReduce<T, Op>(result, sharedValues);

    if (threadId == 0)
    {
        *destination = result;
    }
}

template <typename T>
cudaError_t launchKernel(const void* source, void* destination, size_t count, ReduceOp operation, cudaStream_t stream)
{
    const T* typedSource = static_cast<const T*>(source);
    // Launched even for an empty array, unlike the element-wise kernels: the destination always
    // holds one element, which the kernel seeds with the reduction's identity.
    switch (operation)
    {
    case ReduceOp::eAll:
        allAnyReduceKernel<T, true>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<bool*>(destination), count);
        break;
    case ReduceOp::eAny:
        allAnyReduceKernel<T, false>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<bool*>(destination), count);
        break;
    case ReduceOp::eMin:
        minMaxReduceKernel<T, true>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<T*>(destination), count);
        break;
    case ReduceOp::eMax:
        minMaxReduceKernel<T, false>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<T*>(destination), count);
        break;
    case ReduceOp::eSum:
        sumProdReduceKernel<T, true>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<T*>(destination), count);
        break;
    case ReduceOp::eProd:
        sumProdReduceKernel<T, false>
            <<<1, g_kThreadsPerBlock, 0, stream>>>(typedSource, static_cast<T*>(destination), count);
        break;
    default:
        // operation crosses the shared-library boundary as an int32_t (see reduceOpFunction below), so an
        // out-of-range value is representable. Report it rather than leaving destination
        // uninitialized: no kernel below would otherwise run, and cudaGetLastError() would report
        // success.
        return cudaErrorInvalidValue;
    }
    return cudaGetLastError();
}

} // namespace


ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t reduceOpFunction(
    const void* source, void* destination, size_t count, int32_t kind, int32_t operation, cudaStream_t stream)
{
    return Dtype::dispatchByKind(static_cast<Dtype::Kind>(kind),
                                 [&](auto sample) -> cudaError_t
                                 {
                                     using T = decltype(sample);
                                     return launchKernel<T>(
                                         source, destination, count, static_cast<ReduceOp>(operation), stream);
                                 });
}
