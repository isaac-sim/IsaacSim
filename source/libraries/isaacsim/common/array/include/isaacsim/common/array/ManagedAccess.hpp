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

#include "isaacsim/common/array/Device.hpp"
#include "isaacsim/common/array/Export.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

/**
 * @file ManagedAccess.hpp
 * @brief Retained, stream-aware read and write access to the allocation behind an
 *        @ref isaacsim::common::array::Array.
 * @details
 * @ref isaacsim::common::array::Array itself covers allocation, layout, and computation, and its
 * raw @ref isaacsim::common::array::Array::data() accessor neither retains the allocation nor
 * orders CUDA work. The scopes and entry points here add both: acquiring one keeps the allocation
 * alive for as long as the scope is held, serializes it against other scopes over the same
 * allocation, and performs the CUDA stream handshake in both directions.
 *
 * Use them whenever a buffer crosses a lifetime or stream boundary -- an asynchronous CUDA
 * consumer, a DLPack export, a provider publishing into a staging buffer -- and the plain
 * accessors elsewhere for ordinary synchronous work.
 *
 * @note A scope may be moved to and released on a thread other than the one that acquired it, so
 *       `CU_STREAM_PER_THREAD` is rejected by @ref isaacsim::common::array::acquireRead(),
 *       @ref isaacsim::common::array::acquireWrite() and
 *       @ref isaacsim::common::array::tryAcquireWrite(). That handle names a different physical
 *       stream on every calling thread: completion would be recorded on the releasing thread's
 *       stream, and the allocation could retire while the acquiring thread's work is still
 *       pending. Pass an explicit stream, or 0 for the legacy default stream.
 *       @ref isaacsim::common::array::recordCudaWrite() records on the calling thread and accepts
 *       it.
 *
 * @note `Array.hpp` includes this header and its own templates call into it, so this header only
 *       forward-declares @ref isaacsim::common::array::Array rather than including `Array.hpp`
 *       back. Include `Array.hpp`; it pulls this file in.
 */

namespace isaacsim
{
namespace common
{
namespace array
{

class Array;

namespace details
{
class Storage;
}

/**
 * @class ArrayReadAccess
 * @brief Move-only scoped read access that retains an Array allocation while it is in use.
 * @details
 * CPU access only retains storage. CUDA access first orders the selected stream after the Array's most
 * recently recorded write; releasing the access records completion on that stream and keeps the
 * allocation alive until the queued reads finish.
 *
 * Queue every CUDA read before this object is released. The default stream value of 0 denotes the
 * CUDA legacy default stream, matching a null `cudaStream_t`.
 *
 * Instances are returned by `%acquireRead()`.
 */
class ISAACSIM_COMMON_ARRAY_API ArrayReadAccess
{
public:
    ArrayReadAccess() = default;
    ~ArrayReadAccess();

    /**
     * @brief Transfers ownership of an active read access.
     * @param[in] other Read access to move from. It is empty after the operation.
     */
    ArrayReadAccess(ArrayReadAccess&& other) noexcept;

    /**
     * @brief Releases the current access and takes ownership of another read access.
     * @param[in] other Read access to move from. It is empty after the operation.
     * @return Reference to this object.
     */
    ArrayReadAccess& operator=(ArrayReadAccess&& other) noexcept;
    ArrayReadAccess(const ArrayReadAccess&) = delete;
    ArrayReadAccess& operator=(const ArrayReadAccess&) = delete;

    /** @brief Returns whether this object owns an active access. */
    explicit operator bool() const noexcept;
    /** @brief Returns the offset-adjusted read-only pointer for this access. */
    const void* data() const noexcept;
    /** @brief Returns the logical number of visible element bytes. */
    size_t nbytes() const noexcept;
    /** @brief Releases this access and records CUDA consumer completion when applicable. */
    void release();

private:
    ArrayReadAccess(std::shared_ptr<details::Storage> storage,
                    const void* data,
                    size_t nbytes,
                    Device device,
                    std::uintptr_t cudaStream);
    void _release(bool throwOnError);

    std::shared_ptr<details::Storage> m_storage;
    const void* m_data{ nullptr };
    size_t m_nbytes{ 0 };
    Device m_device{ Device::Cpu() };
    std::uintptr_t m_cudaStream{ 0 };
    // Thread that acquired this scope. Release may happen on another thread, so the owner travels
    // with the object rather than being read back from whichever thread ends up releasing it.
    std::thread::id m_owner;

    friend ISAACSIM_COMMON_ARRAY_API ArrayReadAccess acquireRead(const Array& array, std::uintptr_t cudaStream);
};

/**
 * @class ArrayWriteAccess
 * @brief Move-only scoped write access that serializes mutation of an Array allocation.
 * @details
 * Acquisition waits for all CPU access scopes over the shared allocation to close. For CUDA, it
 * also orders the selected stream after the previous producer and every completed reader. Releasing
 * the scope records the new producer event, allowing later readers on other streams to proceed.
 *
 * Queue every CUDA write before this object is released. The default stream value of 0 denotes the
 * CUDA legacy default stream, matching a null `cudaStream_t`.
 *
 * Instances are returned by `%acquireWrite()` and `%tryAcquireWrite()`.
 */
class ISAACSIM_COMMON_ARRAY_API ArrayWriteAccess
{
public:
    ArrayWriteAccess() = default;
    ~ArrayWriteAccess();

    /**
     * @brief Transfers ownership of an active write access.
     * @param[in] other Write access to move from. It is empty after the operation.
     */
    ArrayWriteAccess(ArrayWriteAccess&& other) noexcept;

    /**
     * @brief Releases the current access and takes ownership of another write access.
     * @param[in] other Write access to move from. It is empty after the operation.
     * @return Reference to this object.
     */
    ArrayWriteAccess& operator=(ArrayWriteAccess&& other) noexcept;
    ArrayWriteAccess(const ArrayWriteAccess&) = delete;
    ArrayWriteAccess& operator=(const ArrayWriteAccess&) = delete;

    /** @brief Returns whether this object owns an active access. */
    explicit operator bool() const noexcept;
    /** @brief Returns the offset-adjusted mutable pointer for this access. */
    void* data() const noexcept;
    /** @brief Returns the logical number of visible element bytes. */
    size_t nbytes() const noexcept;
    /** @brief Releases this access and publishes CUDA producer completion when applicable. */
    void release();

private:
    ArrayWriteAccess(
        std::shared_ptr<details::Storage> storage, void* data, size_t nbytes, Device device, std::uintptr_t cudaStream);
    void _release(bool throwOnError);

    std::shared_ptr<details::Storage> m_storage;
    void* m_data{ nullptr };
    size_t m_nbytes{ 0 };
    Device m_device{ Device::Cpu() };
    std::uintptr_t m_cudaStream{ 0 };
    // Thread that acquired this scope. Release may happen on another thread, so the owner travels
    // with the object rather than being read back from whichever thread ends up releasing it.
    std::thread::id m_owner;

    friend ISAACSIM_COMMON_ARRAY_API ArrayWriteAccess acquireWrite(Array& array, std::uintptr_t cudaStream);
    friend ISAACSIM_COMMON_ARRAY_API std::optional<ArrayWriteAccess> tryAcquireWrite(Array& array,
                                                                                     std::uintptr_t cudaStream);
};

/**
 * @brief Acquires retained read access to an array's visible data.
 * @details For a CUDA array, @p cudaStream is the native `cudaStream_t` value reinterpreted as
 *          an integer. Value 0 selects the legacy default stream. Producer readiness is queued
 *          before this function returns, and consumer completion is recorded when the returned
 *          scope is released.
 * @param[in] array      Array whose allocation is retained for the lifetime of the scope.
 * @param[in] cudaStream Consumer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
 * @return A move-only scoped access retaining the underlying allocation.
 * @throws std::invalid_argument if @p cudaStream does not belong to the array's device, or is
 *         `CU_STREAM_PER_THREAD`, which scoped access does not support.
 * @throws std::runtime_error if the calling thread already holds a scope that would block the wait.
 */
ISAACSIM_COMMON_ARRAY_API ArrayReadAccess acquireRead(const Array& array, std::uintptr_t cudaStream = 0);

/**
 * @brief Acquires exclusive retained write access to an array's visible data.
 * @details Access waits for other admitted readers and writers over any view of the same
 *          allocation. For CUDA, prior GPU dependencies are queued on @p cudaStream and the
 *          new producer completion is recorded when the returned scope is released.
 * @param[in] array      Array whose allocation is retained and serialized for the lifetime of the scope.
 * @param[in] cudaStream Writer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
 * @return A move-only scoped access retaining and serializing the underlying allocation.
 * @throws std::runtime_error if @p array is read-only, or if the calling thread already holds a
 *         scope that would block the wait.
 * @throws std::invalid_argument if @p cudaStream does not belong to the array's device, or is
 *         `CU_STREAM_PER_THREAD`, which scoped access does not support.
 */
ISAACSIM_COMMON_ARRAY_API ArrayWriteAccess acquireWrite(Array& array, std::uintptr_t cudaStream = 0);

/**
 * @brief Acquires exclusive retained write access, or reports contention without waiting.
 * @details Identical to @ref acquireWrite() when the allocation is idle. When another access
 *          scope over any view of the same allocation is still open, this returns an empty
 *          optional instead of waiting for it. Use it when the caller can publish a fresh
 *          allocation rather than block -- for example a reusable staging buffer whose previous
 *          contents are still held by an exported DLPack tensor.
 * @param[in] array      Array whose allocation is retained and serialized for the lifetime of the scope.
 * @param[in] cudaStream Writer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
 * @return A scoped access, or `std::nullopt` if the allocation is currently in use.
 * @throws std::runtime_error if @p array is read-only.
 * @throws std::invalid_argument if @p cudaStream does not belong to the array's device, or is
 *         `CU_STREAM_PER_THREAD`, which scoped access does not support.
 */
ISAACSIM_COMMON_ARRAY_API std::optional<ArrayWriteAccess> tryAcquireWrite(Array& array, std::uintptr_t cudaStream = 0);

/**
 * @brief Records that externally managed CUDA writes were enqueued on @p cudaStream.
 * @details This low-level compatibility API does not admit or serialize the write. Prefer
 *          @ref acquireWrite(). Call it only after externally synchronizing all aliases and
 *          submitting the final write. Value 0 selects the legacy default CUDA stream.
 * @param[in] array      CUDA array whose producer readiness is republished.
 * @param[in] cudaStream Stream on which the external writes were enqueued.
 * @throws std::invalid_argument if @p array is not a CUDA array, or if @p cudaStream does not
 *         belong to its device.
 * @throws std::runtime_error if a scoped access over the allocation is currently open.
 */
ISAACSIM_COMMON_ARRAY_API void recordCudaWrite(Array& array, std::uintptr_t cudaStream = 0);

} // namespace array
} // namespace common
} // namespace isaacsim
