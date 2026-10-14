// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <ucxx/api.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace ucx
{
namespace nodes
{

/**
 * @class UcxPendingRequest
 * @brief An in-flight UCX request together with the buffer it was posted against.
 * @details
 * A cancelled request is retired with its buffer and released once it reports completion;
 * requests that outlive their owner move to a process-wide graveyard, so a buffer is never freed
 * while UCX may still be writing into it. abandon() never blocks and is the only variant safe to
 * call from a node's compute(); cancelAndDrain() blocks briefly and is teardown only.
 */
class UcxPendingRequest
{
public:
    /** @brief Time cancelled requests are given to drain on teardown, in milliseconds. */
    static constexpr int64_t s_kCancelDrainMs = 50;

    /** @brief Constructs an idle pending request. */
    UcxPendingRequest() = default;

    UcxPendingRequest(const UcxPendingRequest&) = delete;
    UcxPendingRequest& operator=(const UcxPendingRequest&) = delete;

    /** @brief Destroys the object, retiring every outstanding request before its buffer is freed. */
    ~UcxPendingRequest()
    {
        cancelAndDrain();
    }

    /**
     * @brief Pre-allocates the buffer so posting does not have to.
     *
     * @param[in] size Capacity to reserve, in bytes
     */
    void reserveBuffer(size_t size)
    {
        m_buffer.reserve(size);
    }

    /**
     * @brief Checks whether a request is currently posted.
     *
     * @return bool True if a request handle is held, false otherwise
     */
    bool isOutstanding() const
    {
        return m_request != nullptr;
    }

    /**
     * @brief Checks whether the posted request has finished.
     *
     * @return bool True if a request is posted and reports completion, false otherwise
     */
    bool isCompleted() const
    {
        return m_request && m_request->isCompleted();
    }

    /**
     * @brief Accesses the buffer the current request was posted against.
     *
     * @return const std::vector<uint8_t>& The buffer
     */
    const std::vector<uint8_t>& buffer() const
    {
        return m_buffer;
    }

    /**
     * @brief Posts a request that receives into a freshly sized buffer.
     *
     * @tparam PostFn Callable `bool(void*, size_t, std::shared_ptr<ucxx::Request>&)` that
     *                posts the request and reports whether it succeeded
     * @param[in] bufferSize Size of the buffer to receive into, in bytes
     * @param[in] postRequest Callable that performs the actual UCX call
     * @return bool True if a request is now outstanding, false otherwise
     * @pre No request is outstanding; the resize must not touch a live request's buffer.
     */
    template <typename PostFn>
    bool post(size_t bufferSize, PostFn&& postRequest)
    {
        m_buffer.resize(bufferSize);
        return _postInternal(std::forward<PostFn>(postRequest));
    }

    /**
     * @brief Posts a request that sends an already serialized payload, taking ownership of it.
     *
     * @tparam PostFn Callable `bool(void*, size_t, std::shared_ptr<ucxx::Request>&)` that
     *                posts the request and reports whether it succeeded
     * @param[in] payload Message bytes to send
     * @param[in] postRequest Callable that performs the actual UCX call
     * @return bool True if a request is now outstanding, false otherwise
     * @pre No request is outstanding; the previous payload must not be replaced in flight.
     */
    template <typename PostFn>
    bool post(std::vector<uint8_t>&& payload, PostFn&& postRequest)
    {
        m_buffer = std::move(payload);
        return _postInternal(std::forward<PostFn>(postRequest));
    }

    /**
     * @brief Hands the request handle back, leaving the buffer intact so it can still be parsed.
     *
     * @return std::shared_ptr<ucxx::Request> The request handle, or nullptr if none was posted
     */
    std::shared_ptr<ucxx::Request> releaseRequest()
    {
        return std::move(m_request);
    }

    /**
     * @brief Cancels the outstanding request and retires it without waiting.
     * @details
     * Never blocks. Also releases previously abandoned requests that have since settled.
     */
    void abandon()
    {
        pruneCompleted();

        if (!m_request)
        {
            return;
        }

        try
        {
            if (!m_request->isCompleted())
            {
                m_request->cancel();
            }
        }
        catch (const std::exception&)
        {
            // Nothing useful to do; the request is retired below either way.
        }

        // Dropping the handle does not cancel: the endpoint holds its own reference. Retire request
        // and buffer together, which also forces the next post() to allocate memory it cannot touch.
        m_abandoned.push_back(AbandonedRequest{ std::move(m_buffer), std::move(m_request) });
        m_buffer.clear();
        m_request.reset();
    }

    /**
     * @brief Releases every abandoned request whose transfer has completed, and its buffer.
     * @details
     * Also prunes the process-wide graveyard. Never blocks; call once per compute() so retired
     * requests cannot accumulate.
     */
    void pruneCompleted()
    {
        m_abandoned.erase(
            std::remove_if(m_abandoned.begin(), m_abandoned.end(), &UcxPendingRequest::_isSettled), m_abandoned.end());
        _pruneGraveyard();
    }

    /**
     * @brief Cancels every outstanding request and waits a bounded time for it to drain.
     * @details
     * Teardown only: blocks for up to s_kCancelDrainMs, so use abandon() from compute(). Requests
     * that have not settled within that bound are moved to the process-wide graveyard, not freed.
     */
    void cancelAndDrain()
    {
        abandon();

        const auto drainStart = std::chrono::steady_clock::now();
        pruneCompleted();
        while (
            !m_abandoned.empty() &&
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - drainStart).count() <
                s_kCancelDrainMs)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            pruneCompleted();
        }

        // Anything still unsettled goes to the graveyard rather than being freed: releasing the
        // handle does not stop UCX from writing into the buffer. Entries are released once their
        // request completes, which needs the listener's progress thread; a node torn down after
        // its listener is gone leaks its entries instead - still better than a use-after-free.
        _retireToGraveyard();
    }

    /**
     * @brief Returns the object to its idle state, releasing the buffer as well.
     * @details Teardown only; blocks like cancelAndDrain().
     */
    void reset()
    {
        cancelAndDrain();
        m_buffer.clear();
    }

private:
    /**
     * @struct AbandonedRequest
     * @brief A cancelled request together with the buffer it was posted against.
     */
    struct AbandonedRequest
    {
        //! Buffer the request was posted against. Declared first so it outlives `request`.
        std::vector<uint8_t> buffer;
        //! Cancelled request, retired once it completes.
        std::shared_ptr<ucxx::Request> request;
    };

    /**
     * @brief Reports whether a retired request can be released together with its buffer.
     *
     * @param[in] entry Retired request to test
     * @return bool True if the entry holds no request or its transfer has finished
     */
    static bool _isSettled(const AbandonedRequest& entry)
    {
        try
        {
            return !entry.request || entry.request->isCompleted();
        }
        catch (const std::exception&)
        {
            return true;
        }
    }

    /**
     * @brief Accesses the process-wide store of requests that outlived their owner.
     * @details
     * Function-local so no static initialization order applies, and deliberately leaked: its
     * destructor would free buffers UCX may still be writing into at process exit.
     *
     * @return std::vector<AbandonedRequest>& The graveyard; callers must hold _graveyardMutex()
     */
    static std::vector<AbandonedRequest>& _graveyard()
    {
        static std::vector<AbandonedRequest>* graveyard = new std::vector<AbandonedRequest>();
        return *graveyard;
    }

    /**
     * @brief Accesses the mutex guarding the graveyard.
     * @details Leaked for the same reason as the graveyard: nodes can tear down during exit.
     *
     * @return std::mutex& The graveyard mutex
     */
    static std::mutex& _graveyardMutex()
    {
        static std::mutex* mutex = new std::mutex();
        return *mutex;
    }

    /**
     * @brief Accesses the published graveyard entry count.
     * @details Lets the common empty case skip the mutex entirely.
     *
     * @return std::atomic<size_t>& Number of entries currently in the graveyard
     */
    static std::atomic<size_t>& _graveyardSize()
    {
        static std::atomic<size_t> size{ 0 };
        return size;
    }

    /** @brief Moves every request that has not settled to the graveyard, with its buffer. */
    void _retireToGraveyard()
    {
        if (m_abandoned.empty())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(_graveyardMutex());
        std::vector<AbandonedRequest>& graveyard = _graveyard();
        for (AbandonedRequest& entry : m_abandoned)
        {
            graveyard.push_back(std::move(entry));
        }
        m_abandoned.clear();
        _graveyardSize().store(graveyard.size(), std::memory_order_relaxed);
    }

    /**
     * @brief Releases graveyard entries whose transfers have finished.
     * @details Never blocks on UCX; called opportunistically from post() and pruneCompleted().
     */
    static void _pruneGraveyard()
    {
        if (_graveyardSize().load(std::memory_order_relaxed) == 0)
        {
            return;
        }

        // Settled entries are moved out under the lock and destroyed after it is released:
        // ~ucxx::Request can join a progress thread, and doing that under a process-wide lock
        // would stall every other node's compute() behind it.
        std::vector<AbandonedRequest> settled;
        {
            std::lock_guard<std::mutex> lock(_graveyardMutex());
            std::vector<AbandonedRequest>& graveyard = _graveyard();
            const auto firstSettled = std::stable_partition(graveyard.begin(), graveyard.end(),
                                                            [](const AbandonedRequest& entry)
                                                            { return !UcxPendingRequest::_isSettled(entry); });
            settled.insert(
                settled.end(), std::make_move_iterator(firstSettled), std::make_move_iterator(graveyard.end()));
            graveyard.erase(firstSettled, graveyard.end());
            _graveyardSize().store(graveyard.size(), std::memory_order_relaxed);
        }
    }

    /**
     * @brief Posts the prepared buffer and keeps the handle on success.
     *
     * @tparam PostFn Callable `bool(void*, size_t, std::shared_ptr<ucxx::Request>&)`
     * @param[in] postRequest Callable that performs the actual UCX call
     * @return bool True if a request is now outstanding, false otherwise
     */
    template <typename PostFn>
    bool _postInternal(PostFn&& postRequest)
    {
        _pruneGraveyard();

        if (!postRequest(static_cast<void*>(m_buffer.data()), m_buffer.size(), m_request))
        {
            m_request.reset();
            return false;
        }

        return true;
    }

    // A posted buffer must stay alive and un-resized until its request completes; members are
    // destroyed in reverse, so every ucxx::Request handle is declared *after* its buffer.
    std::vector<uint8_t> m_buffer; //!< Buffer the outstanding request is posted against
    std::vector<AbandonedRequest> m_abandoned; //!< Cancelled requests awaiting completion
    std::shared_ptr<ucxx::Request> m_request; //!< Outstanding non-blocking request
};

} // namespace nodes
} // namespace ucx
} // namespace isaacsim
