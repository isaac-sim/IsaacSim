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

#include <isaacsim/common/array/Dtype.hpp>

#include <cstddef>
#include <cstdint>
#include <cuda_runtime_api.h>

using isaacsim::common::array::Dtype;

namespace
{

template <typename SrcT, typename DstT>
__global__ void castKernel(const SrcT* source, DstT* destination, size_t count)
{
    const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i < count)
    {
        destination[i] = static_cast<DstT>(source[i]);
    }
}

template <typename SrcT, typename DstT>
cudaError_t launchKernel(const void* source, void* destination, size_t count, cudaStream_t stream)
{
    if (count == 0)
    {
        return cudaSuccess;
    }
    constexpr int threadsPerBlock = 256;
    const int blocks = static_cast<int>((count + threadsPerBlock - 1) / threadsPerBlock);
    castKernel<SrcT, DstT><<<blocks, threadsPerBlock, 0, stream>>>(
        static_cast<const SrcT*>(source), static_cast<DstT*>(destination), count);
    return cudaGetLastError();
}

} // namespace


ISAACSIM_COMMON_ARRAY_KERNEL_EXPORT cudaError_t castFunction(
    const void* source, void* destination, size_t count, int32_t sourceKind, int32_t destinationKind, cudaStream_t stream)
{
    return Dtype::dispatchByKind(static_cast<Dtype::Kind>(sourceKind),
                                 [&](auto sourceSample) -> cudaError_t
                                 {
                                     using SrcT = decltype(sourceSample);
                                     return Dtype::dispatchByKind(static_cast<Dtype::Kind>(destinationKind),
                                                                  [&](auto destinationSample) -> cudaError_t
                                                                  {
                                                                      using DstT = decltype(destinationSample);
                                                                      return launchKernel<SrcT, DstT>(
                                                                          source, destination, count, stream);
                                                                  });
                                 });
}
