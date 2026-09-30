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

#include "isaacsim/common/logging/Logging.h"

#if !defined(FMT_HEADER_ONLY)
#    define FMT_HEADER_ONLY 1
#    define ISAACSIM_COMMON_LOGGING_UNDEFINE_FMT_HEADER_ONLY 1
#endif
#include <fmt/format.h>
#if defined(ISAACSIM_COMMON_LOGGING_UNDEFINE_FMT_HEADER_ONLY)
#    undef ISAACSIM_COMMON_LOGGING_UNDEFINE_FMT_HEADER_ONLY
#    undef FMT_HEADER_ONLY
#endif

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace logging
{

namespace details
{
/** @brief Opaque process-wide state for a registered Carbonite channel. */
class CarboniteChannel;
}

/** @brief Severity of a submitted log record. */
enum class LogLevel
{
    /** @brief Detailed diagnostic information. */
    eVerbose = ISAACSIM_COMMON_LOGGING_LOG_VERBOSE,
    /** @brief Informational messages. */
    eInfo = ISAACSIM_COMMON_LOGGING_LOG_INFO,
    /** @brief Potential problems that allow continued operation. */
    eWarning = ISAACSIM_COMMON_LOGGING_LOG_WARNING,
    /** @brief Failures that prevent an operation from completing. */
    eError = ISAACSIM_COMMON_LOGGING_LOG_ERROR,
};

/** @brief Result of applying process-wide logging configuration. */
enum class ConfigureResult
{
    /** @brief The backend accepted the selected settings. */
    eSuccess = ISAACSIM_COMMON_LOGGING_CONFIGURE_SUCCESS,
    /** @brief A selected setting was invalid. */
    eInvalidArgument = ISAACSIM_COMMON_LOGGING_CONFIGURE_INVALID_ARGUMENT,
    /** @brief The native logging backend was unavailable. */
    eBackendUnavailable = ISAACSIM_COMMON_LOGGING_CONFIGURE_BACKEND_UNAVAILABLE,
};

/** @brief Selection of the standard stream used by the Carbonite standard logger. */
enum class OutputStream
{
    /** @brief Route lower severities to standard output and errors to standard error. */
    eDefault = ISAACSIM_COMMON_LOGGING_OUTPUT_STREAM_DEFAULT,
    /** @brief Route all standard-stream records to standard error. */
    eStderr = ISAACSIM_COMMON_LOGGING_OUTPUT_STREAM_STDERR,
};

/** @brief Unit used for the elapsed-time prefix. */
enum class ElapsedTimeUnit
{
    /** @brief Do not include an elapsed-time prefix. */
    eDisabled = ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_DISABLED,
    /** @brief Display elapsed time in milliseconds. */
    eMilliseconds = ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_MILLISECONDS,
    /** @brief Display elapsed time in microseconds. */
    eMicroseconds = ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_MICROSECONDS,
    /** @brief Display elapsed time in nanoseconds. */
    eNanoseconds = ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_NANOSECONDS,
};

/** @brief Action to apply to one channel setting in a configuration patch. */
enum class ChannelSettingBehavior
{
    /** @brief Leave the existing Carbonite per-channel setting unchanged. */
    eUnchanged = ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_UNCHANGED,
    /** @brief Clear the per-channel override and inherit the process-wide setting. */
    eInherit = ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_INHERIT,
    /** @brief Override the process-wide setting with the supplied channel value. */
    eOverride = ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_OVERRIDE,
};

/** @brief Patch-style Carbonite settings for one logging channel. */
struct ChannelLoggingConfiguration
{
    /** Non-empty Carbonite channel name. */
    std::string channel;
    /** Action to apply to `enabled`. */
    ChannelSettingBehavior enabledBehavior{ ChannelSettingBehavior::eUnchanged };
    /** Channel enablement used when `enabledBehavior` is `eOverride`. */
    bool enabled{ true };
    /** Action to apply to `minimumLevel`. */
    ChannelSettingBehavior minimumLevelBehavior{ ChannelSettingBehavior::eUnchanged };
    /** Channel threshold used when `minimumLevelBehavior` is `eOverride`. */
    LogLevel minimumLevel{ LogLevel::eWarning };
};

/**
 * @brief Patch-style process-wide logging configuration.
 *
 * An empty optional leaves the corresponding backend setting unchanged. An empty `filePath` string
 * disables file logging. Parent directories are not created.
 */
struct GlobalLoggingConfiguration
{
    /** Enable or disable process-wide logging. */
    std::optional<bool> enabled;
    /** Set the process-wide admission threshold. */
    std::optional<LogLevel> minimumLevel;
    /** Enable or disable asynchronous backend delivery. */
    std::optional<bool> asynchronous;
    /** Enable or disable stdout and stderr output. */
    std::optional<bool> standardStreamEnabled;
    /** Set the standard-stream threshold. */
    std::optional<LogLevel> standardStreamLevel;
    /** Flush stdout after every standard-stream record. */
    std::optional<bool> standardStreamFlush;
    /** Select default stdout/stderr routing or force stderr. */
    std::optional<OutputStream> outputStream;
    /** Enable or disable the Windows debug console destination. */
    std::optional<bool> debugConsoleEnabled;
    /** Set the Windows debug console threshold. */
    std::optional<LogLevel> debugConsoleLevel;
    /** Set the UTF-8 log file path; an empty string disables file logging. */
    std::optional<std::string> filePath;
    /** Append to the configured log file instead of replacing it. */
    std::optional<bool> fileAppend;
    /** Set the file destination threshold. */
    std::optional<LogLevel> fileLevel;
    /** Flush the file for records at or above this level. */
    std::optional<LogLevel> fileFlushLevel;
    /** Include the source filename. */
    std::optional<bool> filenameIncluded;
    /** Include the source line number. */
    std::optional<bool> lineNumberIncluded;
    /** Include the source function name. */
    std::optional<bool> functionNameIncluded;
    /** Include an absolute timestamp. */
    std::optional<bool> timestampIncluded;
    /** Render absolute timestamps in UTC rather than local time. */
    std::optional<bool> utcTimestamps;
    /** Use microsecond rather than millisecond absolute-timestamp precision. */
    std::optional<bool> microsecondTimestamps;
    /** Select or disable the elapsed-time prefix. */
    std::optional<ElapsedTimeUnit> elapsedTime;
    /** Include the emitting thread identifier. */
    std::optional<bool> threadIdIncluded;
    /** Include the logger channel. */
    std::optional<bool> sourceIncluded;
    /** Include the process identifier. */
    std::optional<bool> processIdIncluded;
    /** Include the emitting thread's trace identifier when present. */
    std::optional<bool> traceIdIncluded;
    /** Include terminal color control codes. */
    std::optional<bool> colorIncluded;
    /** Force ANSI color control codes when color output is enabled. */
    std::optional<bool> forceAnsiColor;
    /** Set a nonzero identifier to serialize output across cooperating processes. */
    std::optional<std::int32_t> multiprocessGroupId;
    /** Apply Carbonite enablement and threshold patches to the named channels. */
    std::vector<ChannelLoggingConfiguration> channels;
};

/** @brief Borrowed source metadata forwarded to the logging backend. */
struct SourceLocation
{
    /** @brief Source filename, or `nullptr` when unavailable. */
    const char* file{ nullptr };
    /** @brief Source function name, or `nullptr` when unavailable. */
    const char* function{ nullptr };
    /** @brief One-based source line number, or zero when unavailable. */
    unsigned int line{ 0 };
};

/**
 * @brief Apply selected process-wide logging settings.
 *
 * @param[in] configuration Configuration patch to apply.
 * @return Configuration result.
 */
ISAACSIM_COMMON_LOGGING_API ConfigureResult configureGlobalLogging(const GlobalLoggingConfiguration& configuration) noexcept;

/**
 * @brief Lightweight logger with one immutable Carbonite channel.
 *
 * Construction caches channel state without selecting or initializing a logging backend.
 */
class ISAACSIM_COMMON_LOGGING_API Logger
{
public:
    /**
     * @brief Construct a logger for a non-empty channel.
     *
     * @param[in] channel Channel attached to every record submitted by this logger.
     * @throws std::invalid_argument If `channel` is empty.
     */
    explicit Logger(std::string_view channel);

    /** @brief Destroy the logger without flushing process-wide logging state. */
    ~Logger();

    /**
     * @brief Prevent copying a logger and its channel registration.
     *
     * @param[in] other Logger that would be copied.
     */
    Logger(const Logger& other) = delete;
    /**
     * @brief Prevent assigning a copied logger.
     *
     * @param[in] other Logger that would be copied.
     * @return Reference to this logger.
     */
    Logger& operator=(const Logger& other) = delete;
    /**
     * @brief Prevent moving a logger and its channel registration.
     *
     * @param[in] other Logger that would be moved.
     */
    Logger(Logger&& other) = delete;
    /**
     * @brief Prevent assigning a moved logger.
     *
     * @param[in] other Logger that would be moved.
     * @return Reference to this logger.
     */
    Logger& operator=(Logger&& other) = delete;

    /**
     * @brief Return this logger's immutable channel.
     *
     * @return Channel attached to the logger.
     */
    std::string_view getChannel() const noexcept;

    /**
     * @brief Return whether the backend currently admits a severity for this logger's channel.
     *
     * @param[in] severity Severity to query.
     * @return `true` when a record at the severity is currently admitted.
     */
    bool isEnabled(LogLevel severity) const noexcept;

    /**
     * @brief Submit an already formatted message.
     *
     * @param[in] severity Severity assigned to the record.
     * @param[in] message Message to submit.
     * @param[in] location Borrowed source metadata.
     */
    void log(LogLevel severity, std::string_view message, SourceLocation location = {}) const noexcept;

    /**
     * @brief Format a message with {fmt} and submit it without a second message copy.
     *
     * @tparam Format Format-string type accepted by {fmt}.
     * @tparam Args Types of values referenced by the format string.
     * @param[in] severity Severity assigned to the record.
     * @param[in] location Borrowed source metadata.
     * @param[in] format Format string.
     * @param[in] args Values referenced by `format`.
     */
    template <typename Format, typename... Args>
    void logFormatted(LogLevel severity, SourceLocation location, const Format& format, Args&&... args) const noexcept
    {
        if (!isEnabled(severity))
        {
            return;
        }
        try
        {
            fmt::memory_buffer message;
            fmt::format_to(message, format, std::forward<Args>(args)...);
            log(severity, std::string_view(message.data(), message.size()), location);
        }
        catch (...)
        {
        }
    }

    /**
     * @brief Print one channel-prefixed line to standard output and submit the message at INFO severity.
     *
     * The standard-output line is `[channel] message`.
     *
     * @param[in] message Message to print and submit.
     * @param[in] location Borrowed source metadata.
     */
    void report(std::string_view message, SourceLocation location = {}) const noexcept;

    /**
     * @brief Format a report with {fmt}, print it after the channel prefix, and submit it at INFO severity.
     *
     * @tparam Format Format-string type accepted by {fmt}.
     * @tparam Args Types of values referenced by the format string.
     * @param[in] location Borrowed source metadata.
     * @param[in] format Format string.
     * @param[in] args Values referenced by `format`.
     */
    template <typename Format, typename... Args>
    void reportFormatted(SourceLocation location, const Format& format, Args&&... args) const noexcept
    {
        try
        {
            fmt::memory_buffer message;
            fmt::format_to(message, format, std::forward<Args>(args)...);
            report(std::string_view(message.data(), message.size()), location);
        }
        catch (...)
        {
        }
    }

private:
    const std::shared_ptr<details::CarboniteChannel> m_channel;
};

/** @brief Flush all records currently pending in the process-wide Carbonite backend. */
ISAACSIM_COMMON_LOGGING_API void flush() noexcept;

} // namespace logging
} // namespace common
} // namespace isaacsim

#include "isaacsim/common/logging/details/LogMacros.hpp"
