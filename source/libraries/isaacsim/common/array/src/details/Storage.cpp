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

#include "Storage.hpp"

#include "CudaRuntime.hpp"

#include <atomic>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace array
{
namespace details
{

/**
 * @brief Owning wrapper around a `CUevent` that can outlive its Storage.
 * @details Quarantining an event parks the allocation and a self-reference on the event itself, so a
 *          CUDA failure leaks memory rather than freeing something the device may still touch.
 */
class CudaEvent
{
public:
    CudaEvent(Device device, CUevent event) : m_device(device), m_event(event)
    {
    }

    ~CudaEvent() noexcept
    {
        try
        {
            DeviceGuard deviceGuard(m_device);
            CudaRuntime::getInstance(false).cudaEventDestroy(m_event, false);
        }
        catch (...)
        {
        }
    }

    CudaEvent(const CudaEvent&) = delete;
    CudaEvent& operator=(const CudaEvent&) = delete;

    CUevent get() const
    {
        return m_event;
    }

    Device device() const
    {
        return m_device;
    }

    void quarantine(std::shared_ptr<std::byte[]> allocation, const std::shared_ptr<CudaEvent>& self) noexcept
    {
        m_quarantinedAllocation = std::move(allocation);
        m_quarantinedSelf = self;
    }

private:
    Device m_device;
    CUevent m_event;
    std::shared_ptr<std::byte[]> m_quarantinedAllocation;
    std::shared_ptr<CudaEvent> m_quarantinedSelf;
};

namespace
{

std::shared_ptr<CudaEvent> recordCudaEvent(const Device& device, std::uintptr_t stream)
{
    DeviceGuard deviceGuard(device);
    auto& cudaRuntime = CudaRuntime::getInstance();
    CUevent event = nullptr;
    cudaRuntime.cudaEventCreateWithFlags(&event, CU_EVENT_DISABLE_TIMING);
    std::shared_ptr<CudaEvent> result;
    try
    {
        result = std::make_shared<CudaEvent>(device, event);
    }
    catch (...)
    {
        cudaRuntime.cudaEventDestroy(event, false);
        throw;
    }
    cudaRuntime.cudaEventRecord(event, reinterpret_cast<CUstream>(stream));
    return result;
}

class RetirementService
{
public:
    struct Job
    {
        Job(std::shared_ptr<std::byte[]> selectedAllocation, std::shared_ptr<CudaEvent> selectedCompletion) noexcept
            : allocation(std::move(selectedAllocation)), completion(std::move(selectedCompletion))
        {
        }

        Job(Job&&) noexcept = default;
        Job& operator=(Job&&) noexcept = default;
        Job(const Job&) = delete;
        Job& operator=(const Job&) = delete;

        std::shared_ptr<std::byte[]> allocation;
        std::shared_ptr<CudaEvent> completion;
    };

    static void defer(std::shared_ptr<std::byte[]> allocation, std::shared_ptr<CudaEvent> completion) noexcept
    {
        Job job(std::move(allocation), std::move(completion));
        if (shutdownStarted().load())
        {
            quarantine(std::move(job));
            return;
        }
        try
        {
            instance().enqueue(job);
        }
        catch (...)
        {
            quarantine(std::move(job));
        }
    }

private:
    RetirementService() : m_worker([this] { run(); })
    {
    }

    ~RetirementService()
    {
        shutdownStarted().store(true);
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stopping = true;
        }
        m_changed.notify_one();
        if (m_worker.joinable())
        {
            m_worker.join();
        }
        for (Job& job : m_jobs)
        {
            quarantine(std::move(job));
        }
    }

    static RetirementService& instance()
    {
        static RetirementService service;
        return service;
    }

    static std::atomic<bool>& shutdownStarted()
    {
        static std::atomic<bool> started{ false };
        return started;
    }

    static void quarantine(Job job) noexcept
    {
        if (!job.completion)
        {
            return;
        }
        // A self-cycle on the already allocated event retains both the event and allocation. This
        // fallback never waits for device work, so process shutdown cannot hang on a stalled event.
        std::shared_ptr<CudaEvent> completion = std::move(job.completion);
        completion->quarantine(std::move(job.allocation), completion);
    }

    void enqueue(Job& job)
    {
        bool accepted = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_stopping)
            {
                m_jobs.push_back(std::move(job));
                accepted = true;
            }
        }
        if (!accepted)
        {
            quarantine(std::move(job));
            return;
        }
        m_changed.notify_one();
    }

    void run() noexcept
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (!m_stopping)
        {
            m_changed.wait_for(lock, std::chrono::milliseconds(1));
            if (m_stopping)
            {
                break;
            }

            for (auto iterator = m_jobs.begin(); iterator != m_jobs.end();)
            {
                CUresult result = CUDA_ERROR_NOT_INITIALIZED;
                try
                {
                    DeviceGuard deviceGuard(iterator->completion->device());
                    result = CudaRuntime::getInstance(false).cudaEventQuery(iterator->completion->get());
                }
                catch (...)
                {
                }

                if (result == CUDA_SUCCESS)
                {
                    Job completed = std::move(*iterator);
                    iterator = m_jobs.erase(iterator);
                    lock.unlock();
                    // A provider deleter may itself release another access. Run it without the
                    // queue mutex so recursive retirement cannot deadlock this worker.
                    completed.allocation.reset();
                    completed.completion.reset();
                    lock.lock();
                    iterator = m_jobs.begin();
                }
                else if (result == CUDA_ERROR_NOT_READY)
                {
                    ++iterator;
                }
                else
                {
                    Job failed = std::move(*iterator);
                    iterator = m_jobs.erase(iterator);
                    lock.unlock();
                    quarantine(std::move(failed));
                    lock.lock();
                    iterator = m_jobs.begin();
                }
            }
        }
    }

    std::mutex m_mutex;
    std::condition_variable m_changed;
    std::vector<Job> m_jobs;
    bool m_stopping{ false };
    std::thread m_worker;
};

} // namespace

struct Storage::Quarantine
{
    std::shared_ptr<std::byte[]> allocation;
    std::shared_ptr<CudaEvent> producer;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>> readers;
};

Storage::Storage(std::shared_ptr<std::byte[]> data, size_t capacity, Device device)
    : m_data(std::move(data)),
      m_pointer(m_data.get()),
      m_capacity(capacity),
      m_device(device),
      m_quarantineAnchor(std::make_unique<Quarantine>())
{
}

Storage::~Storage() noexcept
{
    std::shared_ptr<CudaEvent> ready;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>> readers;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        ready = std::move(m_ready);
        readers = std::move(m_readerCompletions);
    }
    if (m_data)
    {
        if (ready)
        {
            RetirementService::defer(m_data, std::move(ready));
        }
        for (auto& reader : readers)
        {
            RetirementService::defer(m_data, std::move(reader.second));
        }
    }
}

std::shared_ptr<std::byte[]> Storage::data() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data;
}

std::byte* Storage::pointer() const
{
    return m_pointer;
}

size_t Storage::capacity() const
{
    return m_capacity;
}

Device Storage::device() const
{
    return m_device;
}

void Storage::beginRead(std::uintptr_t stream)
{
    std::shared_ptr<CudaEvent> ready;
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        _requireWaitCanFinish(m_writerActive, "acquireRead()");
        m_accessChanged.wait(lock, [this] { return !m_writerActive || m_quarantined; });
        if (m_quarantined)
        {
            throw std::runtime_error("Array access: storage was quarantined after a CUDA synchronization failure");
        }
        _addHolder();
        ++m_activeReaders;
        ready = m_ready;
    }
    if (!m_device.isCuda() || !ready)
    {
        return;
    }
    try
    {
        DeviceGuard deviceGuard(m_device);
        CudaRuntime::getInstance().cudaStreamWaitEvent(reinterpret_cast<CUstream>(stream), ready->get(), 0);
    }
    catch (...)
    {
        cancelRead();
        throw;
    }
}

void Storage::endRead(std::uintptr_t stream, bool throwOnError, std::thread::id owner)
{
    std::shared_ptr<CudaEvent> completion;
    std::shared_ptr<std::byte[]> allocation;
    std::exception_ptr error;
    if (m_device.isCuda())
    {
        try
        {
            completion = recordCudaEvent(m_device, stream);
        }
        catch (...)
        {
            error = std::current_exception();
            if (!synchronizeStream(stream))
            {
                quarantine();
            }
        }
    }

    if (completion)
    {
        try
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_readerCompletions.insert_or_assign(stream, completion);
            allocation = m_data;
        }
        catch (...)
        {
            error = std::current_exception();
            if (!synchronizeEvent(completion))
            {
                std::shared_ptr<std::byte[]> allocation;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    allocation = m_data;
                }
                completion->quarantine(std::move(allocation), completion);
                quarantine();
            }
            completion.reset();
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        --m_activeReaders;
        _removeHolder(owner);
    }
    m_accessChanged.notify_all();

    if (completion)
    {
        RetirementService::defer(std::move(allocation), std::move(completion));
    }
    if (error && throwOnError)
    {
        std::rethrow_exception(error);
    }
}

void Storage::beginWrite(std::uintptr_t stream)
{
    std::shared_ptr<CudaEvent> ready;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>> readers;
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        _requireWaitCanFinish(m_writerActive || m_activeReaders != 0, "acquireWrite()");
        m_accessChanged.wait(lock, [this] { return (!m_writerActive && m_activeReaders == 0) || m_quarantined; });
        if (m_quarantined)
        {
            throw std::runtime_error("Array access: storage was quarantined after a CUDA synchronization failure");
        }
        ready = m_ready;
        readers = m_readerCompletions;
        _addHolder();
        m_writerActive = true;
    }
    _orderWriteAfter(stream, ready, readers);
}

// Non-blocking counterpart of beginWrite(): reports contention rather than waiting for it, so a
// caller holding its own retained access can fall back to a fresh allocation.
bool Storage::tryBeginWrite(std::uintptr_t stream)
{
    std::shared_ptr<CudaEvent> ready;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>> readers;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_quarantined)
        {
            throw std::runtime_error("Array access: storage was quarantined after a CUDA synchronization failure");
        }
        if (m_writerActive || m_activeReaders != 0)
        {
            return false;
        }
        ready = m_ready;
        readers = m_readerCompletions;
        _addHolder();
        m_writerActive = true;
    }
    _orderWriteAfter(stream, ready, readers);
    return true;
}

void Storage::endWrite(std::uintptr_t stream, bool throwOnError, std::thread::id owner)
{
    std::shared_ptr<CudaEvent> ready;
    std::exception_ptr error;
    if (m_device.isCuda())
    {
        try
        {
            ready = recordCudaEvent(m_device, stream);
        }
        catch (...)
        {
            error = std::current_exception();
            if (!synchronizeStream(stream))
            {
                quarantine();
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_quarantined)
        {
            m_ready = std::move(ready);
            m_readerCompletions.clear();
        }
        m_writerActive = false;
        _removeHolder(owner);
    }
    m_accessChanged.notify_all();

    if (error && throwOnError)
    {
        std::rethrow_exception(error);
    }
}

void Storage::recordExternalWrite(std::uintptr_t stream)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_writerActive || m_activeReaders != 0)
        {
            throw std::runtime_error("recordCudaWrite(): scoped access is active for this allocation");
        }
        if (m_quarantined)
        {
            throw std::runtime_error("recordCudaWrite(): storage is quarantined");
        }
        _addHolder();
        m_writerActive = true;
    }
    endWrite(stream, true, std::this_thread::get_id());
}

void Storage::synchronizeReady() const
{
    std::shared_ptr<CudaEvent> ready;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        ready = m_ready;
    }
    if (ready && !synchronizeEvent(ready))
    {
        throw std::runtime_error("Array: failed to synchronize CUDA producer readiness");
    }
}

// Queues the admitted writer's stream behind the previous producer and every completed reader.
// Releases the admission if the CUDA handshake fails, so a throw never strands the storage.
void Storage::_orderWriteAfter(std::uintptr_t stream,
                               const std::shared_ptr<CudaEvent>& ready,
                               const std::unordered_map<std::uintptr_t, std::shared_ptr<CudaEvent>>& readers)
{
    if (!m_device.isCuda())
    {
        return;
    }
    try
    {
        DeviceGuard deviceGuard(m_device);
        auto& cudaRuntime = CudaRuntime::getInstance();
        if (ready)
        {
            cudaRuntime.cudaStreamWaitEvent(reinterpret_cast<CUstream>(stream), ready->get(), 0);
        }
        for (const auto& reader : readers)
        {
            cudaRuntime.cudaStreamWaitEvent(reinterpret_cast<CUstream>(stream), reader.second->get(), 0);
        }
    }
    catch (...)
    {
        cancelWrite();
        throw;
    }
}

// Requires m_mutex. A thread blocked here can never release the scope it already holds, so the
// wait would never end. Reject it with a diagnostic instead: a silent hang is far harder to attribute, and this
// is reachable from ordinary code -- a retained DLPack export holds read access for as long as
// its consumer keeps the tensor, and the next write to that allocation runs on the same thread.
// Cross-thread contention is left to wait, since another thread can still make progress.
void Storage::_requireWaitCanFinish(bool wouldBlock, const char* callerName) const
{
    if (!wouldBlock || m_quarantined || m_holders.count(std::this_thread::get_id()) == 0)
    {
        return;
    }
    throw std::runtime_error(std::string(callerName) +
                             ": the calling thread already holds an access scope over this allocation, so waiting "
                             "for it would deadlock; release that scope first, or use tryAcquireWrite()");
}

// Both helpers require m_mutex. The holder is keyed by the acquiring thread rather than the
// releasing one, so moving an access to another thread and releasing it there stays balanced.
//
// _addHolder() allocates a node and may rehash, so it can throw. Every acquire path must therefore
// order it *before* it sets m_activeReaders/m_writerActive, and must leave nothing that throws
// after that point: a throw once the admission state is set escapes the acquire without ever
// constructing a scope to release it, stranding the storage so every later writer waits on a
// predicate that can no longer come true. Throwing ahead of the admission needs no unwind, since
// no state has changed yet.
void Storage::_addHolder()
{
    ++m_holders[std::this_thread::get_id()];
}

void Storage::_removeHolder(std::thread::id owner)
{
    const auto holder = m_holders.find(owner);
    if (holder != m_holders.end() && --holder->second == 0)
    {
        m_holders.erase(holder);
    }
}

void Storage::cancelRead()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        --m_activeReaders;
        _removeHolder(std::this_thread::get_id());
    }
    m_accessChanged.notify_all();
}

void Storage::cancelWrite()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_writerActive = false;
        _removeHolder(std::this_thread::get_id());
    }
    m_accessChanged.notify_all();
}

bool Storage::synchronizeStream(std::uintptr_t stream) const noexcept
{
    try
    {
        DeviceGuard deviceGuard(m_device);
        return CudaRuntime::getInstance(false).cudaStreamSynchronize(reinterpret_cast<CUstream>(stream), false);
    }
    catch (...)
    {
        return false;
    }
}

bool Storage::synchronizeEvent(const std::shared_ptr<CudaEvent>& event) const noexcept
{
    try
    {
        DeviceGuard deviceGuard(m_device);
        return CudaRuntime::getInstance(false).cudaEventSynchronize(event->get(), false);
    }
    catch (...)
    {
        return false;
    }
}

void Storage::quarantine() noexcept
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_quarantined)
    {
        return;
    }
    m_quarantineAnchor->allocation = std::move(m_data);
    m_quarantineAnchor->producer = std::move(m_ready);
    m_quarantineAnchor->readers = std::move(m_readerCompletions);
    (void)m_quarantineAnchor.release();
    m_quarantined = true;
}

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim
