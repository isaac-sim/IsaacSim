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

#include "isaacsim/common/array/Array.hpp"
#include "isaacsim/common/array/Export.h"

#include <dlpack/dlpack.h>

namespace isaacsim
{
namespace common
{
namespace array
{

/**
 * @brief Borrows a C-contiguous tensor, canonicalizing strides on singleton and empty axes.
 * @details Ownership and synchronization follow `fromDLPackBorrowed()`.
 * @param[in] tensor Tensor to view; empty tensors may have arbitrary strides.
 * @return A contiguous Array borrowing the tensor allocation.
 * @throws std::invalid_argument if the tensor is not C-contiguous or has unsupported metadata.
 * @throws std::overflow_error if the element count or byte offset cannot be represented.
 */
ISAACSIM_COMMON_ARRAY_API Array fromDLPack(const DLTensor& tensor);

/**
 * @brief Borrows Array data and metadata with the lifetime contract of `toDLPackBorrowed()`.
 * @param[in] array Array that must outlive the returned tensor.
 * @return A borrowed tensor with data addressing element zero.
 */
ISAACSIM_COMMON_ARRAY_API DLTensor toDLPack(const Array& array);

/**
 * @brief Creates an Array view of a DLPack tensor without copying its data.
 * @details
 * The tensor must use a supported scalar dtype, reside on a CPU or CUDA device, and use non-negative element strides.
 * A null strides pointer selects a C-contiguous layout.
 *
 * The returned Array borrows the tensor's data allocation. The owner of @p tensor must keep the allocation alive for
 * the lifetime of the returned Array and every view derived from it. Shape and dtype metadata are copied.
 *
 * This function does not perform stream synchronization.
 *
 * @param[in] tensor DLPack tensor to view.
 * @return An Array that aliases the tensor's data.
 * @throws std::invalid_argument if the tensor metadata is invalid or unsupported.
 * @throws std::overflow_error if the tensor rank, element count, or byte offset cannot be represented.
 */
ISAACSIM_COMMON_ARRAY_API Array fromDLPackBorrowed(const DLTensor& tensor);

/**
 * @brief Consumes an owned unversioned DLPack tensor and returns a zero-copy Array.
 * @details The managed tensor's deleter is called after the returned Array and all of its views are
 *          released. For CUDA, @p producerStream is the native stream on which the producer made
 *          the tensor ready; value 0 selects the legacy default stream.
 *
 * The unversioned `DLManagedTensor` carries no flags field, so a producer cannot advertise
 * read-only status and the imported Array is writeable. Prefer `fromDLPackVersioned()` instead.
 *
 * @param[in] tensor Managed tensor whose ownership is transferred to the returned Array.
 * @param[in] producerStream Native CUDA producer stream, or 0 for CPU and the CUDA legacy default stream.
 * @return An Array that retains the managed tensor allocation.
 * @throws std::invalid_argument if @p tensor or its deleter is null, or if a CPU tensor is given a
 *         non-zero @p producerStream.
 */
ISAACSIM_COMMON_ARRAY_API Array fromDLPackManaged(DLManagedTensor* tensor, std::uintptr_t producerStream = 0);

/**
 * @brief Consumes an owned versioned DLPack tensor and returns a zero-copy Array.
 * @details Ownership and CUDA producer readiness follow `fromDLPackManaged()`.
 *          The DLPack read-only flag is set and propagated to the returned Array.
 * @param[in] tensor Versioned managed tensor whose ownership is transferred to the returned Array.
 * @param[in] producerStream Native CUDA producer stream, or 0 for CPU and the CUDA legacy default stream.
 * @return An Array that retains the managed tensor allocation and honors its flags.
 * @throws std::invalid_argument if @p tensor or its deleter is null, if the tensor declares an
 *         incompatible DLPack major version, or if a CPU tensor is given a non-zero @p producerStream.
 */
ISAACSIM_COMMON_ARRAY_API Array fromDLPackVersioned(DLManagedTensorVersioned* tensor, std::uintptr_t producerStream = 0);

/**
 * @brief Creates a borrowed DLPack tensor view of an Array without copying its data.
 * @details
 * The returned tensor aliases @p array and points into its shape metadata. It remains valid only while @p array is
 * alive and is neither moved from nor assigned. The strides pointer is null for a C-contiguous layout and otherwise
 * points into the Array's stride metadata.
 *
 * This function does not perform stream synchronization.
 *
 * @warning Never pass a temporary. The result points into @p array's own shape and stride vectors, which
 *          every Array owns individually, so `toDLPackBorrowed(a.contiguous())` leaves the tensor dangling
 *          as soon as the full expression ends. This holds however @p array is produced -- @ref
 *          Array::contiguous() returns by value even when it forwards an already-contiguous array, so the
 *          temporary carries its own metadata either way. Bind the operand to a named variable that
 *          outlives the tensor.
 *
 * @param[in] array Array to view.
 * @return A DLPack tensor that aliases the Array's data and metadata, with `data` addressing element 0
 *         and `byte_offset` set to 0.
 * @throws std::invalid_argument if the Array has invalid shape metadata.
 * @throws std::overflow_error if the Array rank or element count cannot be represented by DLPack.
 */
ISAACSIM_COMMON_ARRAY_API DLTensor toDLPackBorrowed(const Array& array);

/**
 * @brief Exports an owned DLPack tensor retaining the Array allocation and metadata.
 * @details The caller owns the result and must eventually call its deleter. For CUDA,
 *          @p consumerStream is ordered after the Array's last recorded write, and allocation
 *          retirement is fenced when the managed tensor is deleted. The published view must be
 *          treated as read-only; this legacy structure has no field in which to advertise that flag.
 *          `data` addresses element 0 and `byte_offset` is 0, so a consumer that ignores
 *          `byte_offset` still reads the right elements from an offset view.
 * @param[in] array Array to export.
 * @param[in] consumerStream Native CUDA consumer stream, or 0 for CPU and the CUDA legacy default stream.
 * @return Heap-allocated managed tensor owned by the caller.
 */
ISAACSIM_COMMON_ARRAY_API DLManagedTensor* toDLPackManaged(const Array& array, std::uintptr_t consumerStream = 0);

/**
 * @brief Exports an owned, read-only versioned DLPack tensor.
 * @details The exported tensor retains scoped read access and advertises
 *          `DLPACK_FLAG_BITMASK_READ_ONLY`. The caller must eventually call its deleter.
 *          `data` addresses element 0 and `byte_offset` is 0, so a consumer that ignores
 *          `byte_offset` still reads the right elements from an offset view.
 * @param[in] array Array to export.
 * @param[in] consumerStream Native CUDA consumer stream, or 0 for CPU and the CUDA legacy default stream.
 * @return Heap-allocated versioned managed tensor owned by the caller.
 */
ISAACSIM_COMMON_ARRAY_API DLManagedTensorVersioned* toDLPackVersioned(const Array& array,
                                                                      std::uintptr_t consumerStream = 0);

} // namespace array
} // namespace common
} // namespace isaacsim
