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

#include <isaacsim/ucx/core/UcxUtils.h>

#include <chrono>
#include <thread>

namespace isaacsim::ucx::core
{

UcxRequestWaitResult waitForRequestWithTimeout(std::shared_ptr<ucxx::Request> request,
                                               uint32_t timeoutMs,
                                               std::string& errorMessage,
                                               uint32_t pollIntervalUs)
{
    if (!request)
    {
        errorMessage = "Request is null";
        return UcxRequestWaitResult::eFailed;
    }

    const auto startTime = std::chrono::steady_clock::now();
    const bool hasTimeout = (timeoutMs != g_kUcxInfiniteTimeout);

    // Loop until timeout (if specified) or indefinitely if timeout is g_kUcxInfiniteTimeout
    while (!hasTimeout ||
           std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() <
               timeoutMs)
    {
        if (request->isCompleted())
        {
            // Check if there was an error during completion
            try
            {
                request->checkError();
                errorMessage.clear();
                return UcxRequestWaitResult::eCompleted;
            }
            catch (const std::exception& e)
            {
                errorMessage = e.what();
                return UcxRequestWaitResult::eFailed;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(pollIntervalUs));
    }

    // If we exit the loop, the timeout was exceeded
    request->cancel();
    errorMessage = "Request timed out";
    return UcxRequestWaitResult::eTimedOut;
}

} // namespace isaacsim::ucx::core
