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

#pragma once

#include "isaacsim/common/array/Dtype.hpp"
#include "isaacsim/common/array/Export.h"

#include <cstddef>
#include <cstdint>
#include <cuda_runtime_api.h>
#include <string>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{
namespace details
{

/** @brief Element-wise unary operation selected at runtime. */
enum class UnaryOp : int32_t
{
    /** @brief Element-wise logical negation. Produces a boolean result for every input kind. */
    eLogicalNot,

    /** @brief Element-wise arithmetic negation. */
    eNegative,

    /** @brief Element-wise absolute value. */
    eAbsolute,
};

/** @brief Element-wise binary operation selected at runtime. */
enum class BinaryOp : int32_t
{
    /** @brief Element-wise sum. */
    eAdd,

    /** @brief Element-wise difference. */
    eSubtract,

    /** @brief Element-wise product. */
    eMultiply,

    /** @brief Element-wise quotient. */
    eDivide,
};

/**
 * @brief Whole-array reduction selected at runtime.
 * @details
 * `eAll` and `eAny` reduce to a boolean regardless of the operand kind; the remaining
 * operations reduce to a value
 * of the operand's own kind.
 */
enum class ReduceOp : int32_t
{
    /** @brief Whether every element evaluates to `true`. */
    eAll,

    /** @brief Whether any element evaluates to `true`. */
    eAny,

    /** @brief Sum of every element. */
    eSum,

    /** @brief Product of every element. */
    eProd,

    /** @brief Smallest element. Undefined for an empty operand, which the host rejects. */
    eMin,

    /** @brief Largest element. Undefined for an empty operand, which the host rejects. */
    eMax,
};

/**
 * @brief Lazily loads the optional, separately-compiled CUDA kernel library and exposes its kernels.
 * @details
 * The kernels are compiled by nvcc, at configure time, into a sibling shared library
 * (`isaacsim-common-array-cuda-kernels`) that is statically linked against the CUDA runtime and
 * never linked into `isaacsim-common-array` itself. This class `dlopen`s that sibling library
 * next to `isaacsim-common-array`, on first use, mirroring the lazy-loading pattern already used
 * in @ref CudaRuntime. If the sibling library was not built (no vendored `nvcc` at configure time)
 * or fails to load, @ref isLoaded() returns `false` and every kernel call reports failure either by
 * throwing @ref CudaRuntimeError or by returning `false`, depending on its `throwIfInvalid` argument
 * -- `isaacsim-common-array` itself never fails to load or crashes as a result, on a machine with no
 * CUDA toolkit or driver installed. Call `getInstance(false)` to probe availability via
 * @ref isLoaded() without throwing.
 */
class ISAACSIM_COMMON_ARRAY_API CudaKernel
{
public:
    /** @brief Returns the process-wide, lazily-loaded kernel library. */
    static CudaKernel& getInstance(bool throwIfInvalid = true);

    /** @brief Returns whether the sibling kernel library was found and successfully loaded. */
    bool isLoaded() const;

    /**
     * @brief Casts a device-resident buffer from one supported dtype to another.
     * @details
     * Supports every @ref Dtype::Kind pair (all 11 scalar kinds), dispatched at runtime to a
     * `static_cast`-equivalent element-wise kernel templated on the concrete source/destination
     * C++ types.
     * @param[in] source Device pointer to `count` elements of @p sourceKind.
     * @param[out] destination Device pointer to `count` elements of @p destinationKind.
     * @param[in] count Number of elements to cast.
     * @param[in] sourceKind Element kind of @p source.
     * @param[in] destinationKind Element kind of @p destination.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool cast(const void* source,
              void* destination,
              size_t count,
              Dtype::Kind sourceKind,
              Dtype::Kind destinationKind,
              cudaStream_t stream = nullptr,
              bool throwIfInvalid = true) const;

    /**
     * @brief Gathers a device-resident buffer into a broadcast destination shape.
     * @details
     * Element-wise gather: for every element of the destination (in row-major order), reads from
     * `source` at the offset given by the dot product of the destination multi-index with
     * @p sourceStrides. A stride of 0 marks a broadcast axis (source size 1, or an axis absent
     * from the source that was implicitly prepended), so every element along that axis reads the
     * same source element -- mirroring the CPU gather in `Array::broadcastTo()`.
     * @param[in] source Device pointer to the source buffer.
     * @param[out] destination Device pointer to `totalElements` elements of @p elementSize bytes.
     * @param[in] sourceStrides Per-axis source stride, in elements, aligned to the destination
     * axes (length equal to the destination's number of dimensions).
     * @param[in] destinationShape Destination shape, in elements per axis (same length as
     * @p sourceStrides).
     * @param[in] elementSize Size, in bytes, of one element.
     * @param[in] totalElements Total number of elements in the destination.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool broadcastTo(const void* source,
                     void* destination,
                     const std::vector<int64_t>& sourceStrides,
                     const std::vector<int64_t>& destinationShape,
                     size_t elementSize,
                     size_t totalElements,
                     cudaStream_t stream = nullptr,
                     bool throwIfInvalid = true) const;

    /**
     * @brief Gathers slices of a device-resident buffer along a single axis.
     * @details
     * Views the source as the three-dimensional shape `(outer, axisSize, inner)` and the
     * destination as `(outer, indexCount, inner)`, so that
     * `destination[o][j][i] == source[o][indices[j]][i]` -- the layout-independent form of NumPy's
     * `take()` along one axis. `outer` is not passed separately because it is recoverable from
     * @p totalElements, which is `outer * indices.size() * inner`.
     * @param[in] source Device pointer to the source buffer.
     * @param[out] destination Device pointer to @p totalElements elements of @p elementSize bytes.
     * @param[in] indices Host-side source position along the indexed axis for each destination
     * position, already resolved to the range `[0, axisSize)`.
     * @param[in] axisSize Size of the indexed axis in the source.
     * @param[in] inner Product of the axis sizes following the indexed axis.
     * @param[in] elementSize Size, in bytes, of one element.
     * @param[in] totalElements Total number of elements in the destination.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool take(const void* source,
              void* destination,
              const std::vector<int64_t>& indices,
              size_t axisSize,
              size_t inner,
              size_t elementSize,
              size_t totalElements,
              cudaStream_t stream = nullptr,
              bool throwIfInvalid = true) const;

    /**
     * @brief Applies an element-wise binary operation to two device-resident buffers.
     * @details
     * Both operands and the destination hold @p count elements of @p kind: the caller is expected
     * to have cast and broadcast them to a common dtype and shape beforehand.
     * @param[in] source1 Device pointer to the first operand.
     * @param[in] source2 Device pointer to the second operand.
     * @param[out] destination Device pointer to `count` elements of @p kind.
     * @param[in] count Number of elements to process.
     * @param[in] kind Element kind shared by both operands and the destination.
     * @param[in] operation Operation to apply.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool binaryOp(const void* source1,
                  const void* source2,
                  void* destination,
                  size_t count,
                  Dtype::Kind kind,
                  BinaryOp operation,
                  cudaStream_t stream = nullptr,
                  bool throwIfInvalid = true) const;

    /**
     * @brief Applies an element-wise unary operation to a device-resident buffer.
     * @details
     * The source holds @p count elements of @p kind. The destination holds @p count elements of
     * @p kind as well, except for @ref UnaryOp::eLogicalNot, whose destination is always boolean.
     * @param[in] source Device pointer to the operand.
     * @param[out] destination Device pointer to `count` elements of the destination kind.
     * @param[in] count Number of elements to process.
     * @param[in] kind Element kind of @p source.
     * @param[in] operation Operation to apply.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool unaryOp(const void* source,
                 void* destination,
                 size_t count,
                 Dtype::Kind kind,
                 UnaryOp operation,
                 cudaStream_t stream = nullptr,
                 bool throwIfInvalid = true) const;

    /**
     * @brief Reduces a device-resident buffer to a single element via a @ref ReduceOp.
     * @details
     * The source holds @p count elements of @p kind. The destination holds exactly one element
     * regardless of @p count: a boolean for @ref ReduceOp::eAll and @ref ReduceOp::eAny, and an
     * element of @p kind for every other operation, so the caller is expected to have cast the
     * operand to the accumulator kind beforehand.
     *
     * An empty operand yields the reduction's identity (`true` for @ref ReduceOp::eAll, `false`
     * for @ref ReduceOp::eAny, 0 for @ref ReduceOp::eSum, and 1 for @ref ReduceOp::eProd), as in
     * NumPy. @ref ReduceOp::eMin and @ref ReduceOp::eMax have no identity and read element 0, so
     * the caller must reject an empty operand before calling.
     * @param[in] source Device pointer to the operand.
     * @param[out] destination Device pointer to a single element of the destination kind.
     * @param[in] count Number of elements to reduce.
     * @param[in] kind Element kind of @p source.
     * @param[in] operation Reduction to apply.
     * @param[in] stream CUDA stream to launch on, or `nullptr` for the default stream.
     * @param[in] throwIfInvalid If `true`, throws @ref CudaRuntimeError on failure; otherwise returns `false`.
     * @return `true` on success, `false` on failure when @p throwIfInvalid is `false`.
     */
    bool reduceOp(const void* source,
                  void* destination,
                  size_t count,
                  Dtype::Kind kind,
                  ReduceOp operation,
                  cudaStream_t stream = nullptr,
                  bool throwIfInvalid = true) const;

    CudaKernel(const CudaKernel&) = delete;
    CudaKernel& operator=(const CudaKernel&) = delete;

private:
    using CastFunction = cudaError_t (*)(const void*, void*, size_t, int32_t, int32_t, cudaStream_t);
    using BroadcastToFunction =
        cudaError_t (*)(const void*, void*, const int64_t*, const int64_t*, size_t, size_t, size_t, cudaStream_t);
    using TakeFunction =
        cudaError_t (*)(const void*, void*, const int64_t*, size_t, size_t, size_t, size_t, size_t, cudaStream_t);
    using BinaryOpFunction = cudaError_t (*)(const void*, const void*, void*, size_t, int32_t, int32_t, cudaStream_t);
    using UnaryOpFunction = cudaError_t (*)(const void*, void*, size_t, int32_t, int32_t, cudaStream_t);
    using ReduceOpFunction = cudaError_t (*)(const void*, void*, size_t, int32_t, int32_t, cudaStream_t);

    CudaKernel();
    ~CudaKernel();

    std::string m_loadError;
    void* m_handle{ nullptr };
    CastFunction m_cast{ nullptr };
    BroadcastToFunction m_broadcastTo{ nullptr };
    TakeFunction m_take{ nullptr };
    BinaryOpFunction m_binaryOp{ nullptr };
    UnaryOpFunction m_unaryOp{ nullptr };
    ReduceOpFunction m_reduceOp{ nullptr };
};

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim
