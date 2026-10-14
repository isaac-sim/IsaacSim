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

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace isaacsim
{
namespace common
{
namespace array
{
namespace details
{

class CudaEvent;

/**
 * @class Storage
 * @brief Reference-counted allocation shared by every Array view over it, and the serialization
 *        point for the retained access scopes declared in `ManagedAccess.hpp`.
 * @details
 * Storage owns the base pointer, the allocation capacity, and the device, and tracks the CUDA
 * producer-readiness event plus one completion event per consumer stream. Array holds the
 * layout -- shape, strides, offset, dtype, writeability -- and never the allocation itself.
 *
 * Every member is thread-safe. The begin/end pairs admit and retire access scopes: readers may
 * overlap, a writer is exclusive, and the CUDA handshake orders the caller's stream behind the
 * work it must not race.
 */
class Storage
{
public:
    /**
     * @brief Adopts @p data as an allocation of @p capacity bytes residing on @p device.
     * @param[in] data   Shared ownership of the allocation; its deleter is preserved.
     * @param[in] capacity Capacity in bytes of the allocation.
     * @param[in] device Device on which the allocation resides.
     */
    Storage(std::shared_ptr<std::byte[]> data, size_t capacity, Device device);

    /**
     * @brief Releases the allocation, deferring it behind any outstanding CUDA event.
     * @details The allocation is freed inline only when no producer or consumer event is pending;
     *          otherwise it is handed to the retirement worker and freed once the device work
     *          completes.
     */
    ~Storage() noexcept;

    /** @brief Returns shared ownership of the allocation, or null once quarantined. */
    std::shared_ptr<std::byte[]> data() const;
    /** @brief Returns the base pointer of the allocation. */
    std::byte* pointer() const;
    /** @brief Returns the capacity in bytes of the allocation. */
    size_t capacity() const;
    /** @brief Returns the device on which the allocation resides. */
    Device device() const;

    /**
     * @brief Admits a reader and orders @p stream after the recorded producer work.
     * @param[in] stream Consumer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
     * @throws std::runtime_error if the calling thread already holds a scope that would block the
     *         wait, or if the storage was quarantined.
     */
    void beginRead(std::uintptr_t stream);

    /**
     * @brief Retires a reader and records its completion event for @p stream.
     * @param[in] stream       Stream the reader was admitted on.
     * @param[in] throwOnError Whether a CUDA failure is rethrown; false during destruction.
     * @param[in] owner        Thread that acquired the scope, which may differ from the releasing one.
     */
    void endRead(std::uintptr_t stream, bool throwOnError, std::thread::id owner);

    /**
     * @brief Admits the exclusive writer, waiting for other scopes to close.
     * @param[in] stream Writer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
     * @throws std::runtime_error if the calling thread already holds a scope that would block the
     *         wait, or if the storage was quarantined.
     */
    void beginWrite(std::uintptr_t stream);

    /**
     * @brief Non-blocking counterpart of @ref beginWrite() that reports contention instead of waiting.
     * @param[in] stream Writer CUDA stream, or 0 for CPU access and the CUDA legacy default stream.
     * @return `true` if the writer was admitted, `false` if another scope is open.
     * @throws std::runtime_error if the storage was quarantined.
     */
    bool tryBeginWrite(std::uintptr_t stream);

    /**
     * @brief Retires the writer and publishes its completion as the new producer readiness.
     * @param[in] stream       Stream the writer was admitted on.
     * @param[in] throwOnError Whether a CUDA failure is rethrown; false during destruction.
     * @param[in] owner        Thread that acquired the scope, which may differ from the releasing one.
     */
    void endWrite(std::uintptr_t stream, bool throwOnError, std::thread::id owner);

    /**
     * @brief Publishes producer readiness for a write this library did not admit.
     * @param[in] stream Stream on which the external write was enqueued.
     * @throws std::runtime_error if a scope is currently open, or if the storage was quarantined.
     */
    void recordExternalWrite(std::uintptr_t stream);

    /**
     * @brief Blocks the calling thread until the recorded producer work has completed.
     * @throws std::runtime_error if the CUDA synchronization fails.
     */
    void synchronizeReady() const;

private:
    struct Quarantine;

    void _orderWriteAfter(std::uintptr_t stream,
                          const std::shared_ptr<CudaEvent>& ready,
                          const std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>>& readers);
    void _requireWaitCanFinish(bool wouldBlock, const char* callerName) const;
    void _addHolder();
    void _removeHolder(std::thread::id owner);
    void cancelRead();
    void cancelWrite();
    bool synchronizeStream(std::uintptr_t stream) const noexcept;
    bool synchronizeEvent(const std::shared_ptr<CudaEvent>& event) const noexcept;
    void quarantine() noexcept;

    std::shared_ptr<std::byte[]> m_data;
    std::byte* m_pointer;
    size_t m_capacity;
    Device m_device;
    mutable std::mutex m_mutex;
    std::condition_variable m_accessChanged;
    size_t m_activeReaders{ 0 };
    bool m_writerActive{ false };
    bool m_quarantined{ false };
    // Open access scopes per acquiring thread, used only to reject a wait that could never finish.
    std::unordered_map<std::thread::id, size_t> m_holders;
    std::shared_ptr<CudaEvent> m_ready;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>> m_readerCompletions;
    std::unique_ptr<Quarantine> m_quarantineAnchor;
};

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim
