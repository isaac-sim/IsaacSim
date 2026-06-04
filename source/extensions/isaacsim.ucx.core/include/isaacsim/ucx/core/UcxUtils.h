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

#include <chrono>
#include <memory>
#include <optional>
#include <thread>

namespace isaacsim::ucx::core
{

/**
 * @brief Special timeout value for infinite wait.
 */
constexpr uint32_t g_kUcxInfiniteTimeout = UINT32_MAX;

/**
 * @brief Result status of a UCX request wait operation.
 */
enum class UcxRequestWaitResult
{
    eCompleted, ///< Request completed successfully
    eTimedOut, ///< Request timed out before completion
    eFailed ///< Request failed with an error (null request, UCS error status, or exception)
};

/**
 * @brief Result status of a UCX send operation.
 */
enum class UcxSendResult
{
    eSuccess, ///< Send completed successfully (or initiated for async)
    eEmptyMessage, ///< Message data is empty
    eNullRequest, ///< tagSend returned null request
    eTimedOut, ///< Send request timed out before completion
    eFailed, ///< Send request failed with UCX error status
    eException ///< Exception occurred during send
};

/**
 * @brief Result status of a UCX receive operation.
 */
enum class UcxReceiveResult
{
    eSuccess, ///< Receive completed successfully (or initiated for async)
    eNullRequest, ///< tagReceive returned null request
    eTimedOut, ///< Receive request timed out before completion
    eFailed, ///< Receive request failed with UCX error status
    eException ///< Exception occurred during receive
};

/**
 * @brief Wait for a UCX request to complete with timeout.
 * @details
 * Polls the request status until it completes, fails, or the timeout expires.
 * If timeoutMs is g_kUcxInfiniteTimeout (UINT32_MAX), waits indefinitely until completion or failure.
 *
 * The function uses a configurable polling interval which provides a balance
 * between responsiveness and CPU usage. The default interval (1ms) is suitable
 * for most real-time simulation scenarios.
 *
 * @param[in] request UCX request to wait for (must not be null)
 * @param[in] timeoutMs Timeout in milliseconds (g_kUcxInfiniteTimeout = infinite wait)
 * @param[out] errorMessage String for storing error messages from checkError()
 * @param[in] pollIntervalUs Polling interval in microseconds (default: 1000)
 * @return UcxRequestWaitResult indicating the outcome:
 *         - eCompleted: Request finished successfully (UCS_OK)
 *         - eTimedOut: Timeout expired before completion (request is cancelled)
 *         - eFailed: Null request or UCX error status (not UCS_OK or UCS_INPROGRESS)
 *
 * @note This function is thread-safe if the UCX request object is thread-safe.
 * @warning Polling with very short intervals may consume significant CPU when
 *          multiple nodes are polling simultaneously. Consider using intervals
 *          >= 1000μs (1ms) for non-critical operations.
 */
UcxRequestWaitResult waitForRequestWithTimeout(std::shared_ptr<ucxx::Request> request,
                                               uint32_t timeoutMs,
                                               std::string& errorMessage,
                                               uint32_t pollIntervalUs = 1000);

} // namespace isaacsim::ucx::core
