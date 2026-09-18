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

#include "isaacsim/common/logging/Logging.hpp"

#include <atomic>

#define ISAACSIM_COMMON_LOGGING_DETAIL_LOG(logger, severity, ...)                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        auto&& isaacsimLoggingMacroLogger = (logger);                                                                  \
        if (isaacsimLoggingMacroLogger.isEnabled(severity))                                                            \
        {                                                                                                              \
            isaacsimLoggingMacroLogger.logFormatted(                                                                   \
                severity, ::isaacsim::common::logging::SourceLocation{ __FILE__, __func__, __LINE__ }, __VA_ARGS__);   \
        }                                                                                                              \
    } while (false)

/** @brief Format and submit a VERBOSE record while preserving the call-site source location. */
#define ISAACSIM_LOG_VERBOSE(logger, ...)                                                                              \
    ISAACSIM_COMMON_LOGGING_DETAIL_LOG(logger, ::isaacsim::common::logging::LogLevel::eVerbose, __VA_ARGS__)
/** @brief Format and submit an INFO record while preserving the call-site source location. */
#define ISAACSIM_LOG_INFO(logger, ...)                                                                                 \
    ISAACSIM_COMMON_LOGGING_DETAIL_LOG(logger, ::isaacsim::common::logging::LogLevel::eInfo, __VA_ARGS__)
/** @brief Format and submit a WARNING record while preserving the call-site source location. */
#define ISAACSIM_LOG_WARNING(logger, ...)                                                                              \
    ISAACSIM_COMMON_LOGGING_DETAIL_LOG(logger, ::isaacsim::common::logging::LogLevel::eWarning, __VA_ARGS__)
/** @brief Alias for `ISAACSIM_LOG_WARNING`. */
#define ISAACSIM_LOG_WARN(logger, ...) ISAACSIM_LOG_WARNING(logger, __VA_ARGS__)
/** @brief Format and submit a WARNING record once for each call site. */
#define ISAACSIM_LOG_WARNING_ONCE(logger, ...)                                                                         \
    do                                                                                                                 \
    {                                                                                                                  \
        auto&& isaacsimLoggingLogger = (logger);                                                                       \
        if (isaacsimLoggingLogger.isEnabled(::isaacsim::common::logging::LogLevel::eWarning))                          \
        {                                                                                                              \
            static std::atomic_flag s_isaacsimLoggingSubmitted = ATOMIC_FLAG_INIT;                                     \
            if (!s_isaacsimLoggingSubmitted.test_and_set(std::memory_order_relaxed))                                   \
            {                                                                                                          \
                ISAACSIM_LOG_WARNING(isaacsimLoggingLogger, __VA_ARGS__);                                              \
            }                                                                                                          \
        }                                                                                                              \
    } while (false)
/** @brief Alias for `ISAACSIM_LOG_WARNING_ONCE`. */
#define ISAACSIM_LOG_WARN_ONCE(logger, ...) ISAACSIM_LOG_WARNING_ONCE(logger, __VA_ARGS__)
/** @brief Submit one WARNING deprecation record for each call site. */
#define ISAACSIM_LOG_DEPRECATION_ONCE(logger, ...) ISAACSIM_LOG_WARNING_ONCE(logger, __VA_ARGS__)
/** @brief Format and submit an ERROR record while preserving the call-site source location. */
#define ISAACSIM_LOG_ERROR(logger, ...)                                                                                \
    ISAACSIM_COMMON_LOGGING_DETAIL_LOG(logger, ::isaacsim::common::logging::LogLevel::eError, __VA_ARGS__)
/** @brief Format, print, and submit a report while preserving the call-site source location. */
#define ISAACSIM_REPORT(logger, ...)                                                                                   \
    (logger).reportFormatted(::isaacsim::common::logging::SourceLocation{ __FILE__, __func__, __LINE__ }, __VA_ARGS__)
