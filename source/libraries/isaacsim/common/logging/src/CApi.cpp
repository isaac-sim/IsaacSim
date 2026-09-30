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

#include "details/CarboniteBackend.hpp"
#include "details/LoggingHostInternal.hpp"
#include "isaacsim/common/logging/Logging.h"
#include "isaacsim/common/logging/LoggingHost.h"

namespace logging = isaacsim::common::logging;

extern "C"
{

    IsaacSimCommonLoggingConfigureResult isaacsimCommonLoggingConfigureGlobal(
        const IsaacSimCommonLoggingGlobalConfig* configuration)
    {
        if (configuration == nullptr)
        {
            return ISAACSIM_COMMON_LOGGING_CONFIGURE_INVALID_ARGUMENT;
        }
        return logging::details::configureCarboniteGlobal(*configuration);
    }

    void isaacsimCommonLoggingWrite(IsaacSimCommonLoggingLogLevel severity,
                                    const char* channel,
                                    const char* message,
                                    const char* file,
                                    const char* function,
                                    unsigned int line)
    {
        if (severity < ISAACSIM_COMMON_LOGGING_LOG_VERBOSE || severity > ISAACSIM_COMMON_LOGGING_LOG_ERROR ||
            channel == nullptr || channel[0] == '\0' || message == nullptr)
        {
            return;
        }

        logging::details::emitToCarbonite(static_cast<logging::LogLevel>(severity), message, channel,
                                          logging::SourceLocation{ file, function, line });
    }

    void isaacsimCommonLoggingReport(
        const char* channel, const char* message, const char* file, const char* function, unsigned int line)
    {
        if (channel == nullptr || channel[0] == '\0' || message == nullptr)
        {
            return;
        }

        logging::details::reportToStandardOutputAndCarbonite(
            message, channel, logging::SourceLocation{ file, function, line });
    }

    void isaacsimCommonLoggingFlush(void)
    {
        logging::details::flushCarboniteBackend();
    }

    IsaacSimCommonLoggingHostResult isaacsimCommonLoggingAttachCarboniteHost(void* carboniteLogging,
                                                                             IsaacSimCommonLoggingHostToken* token)
    {
        if (carboniteLogging == nullptr || token == nullptr)
        {
            return ISAACSIM_COMMON_LOGGING_HOST_INVALID_ARGUMENT;
        }
        return logging::details::attachCarboniteHost(carboniteLogging, *token);
    }

    IsaacSimCommonLoggingHostResult isaacsimCommonLoggingDetachCarboniteHost(IsaacSimCommonLoggingHostToken token)
    {
        return logging::details::detachCarboniteHost(token);
    }
}
