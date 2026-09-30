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

#include "isaacsim/common/logging/Export.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Severity of a submitted log record. */
    typedef enum IsaacSimCommonLoggingLogLevel
    {
        /** @brief Detailed diagnostic information. */
        ISAACSIM_COMMON_LOGGING_LOG_VERBOSE = 0,
        /** @brief Informational messages. */
        ISAACSIM_COMMON_LOGGING_LOG_INFO = 1,
        /** @brief Potential problems that allow continued operation. */
        ISAACSIM_COMMON_LOGGING_LOG_WARNING = 2,
        /** @brief Failures that prevent an operation from completing. */
        ISAACSIM_COMMON_LOGGING_LOG_ERROR = 3,
    } IsaacSimCommonLoggingLogLevel;

    /** @brief Result of applying process-wide logging configuration. */
    typedef enum IsaacSimCommonLoggingConfigureResult
    {
        /** @brief The backend accepted the selected settings. */
        ISAACSIM_COMMON_LOGGING_CONFIGURE_SUCCESS = 0,
        /** @brief A selected setting was invalid. */
        ISAACSIM_COMMON_LOGGING_CONFIGURE_INVALID_ARGUMENT = 1,
        /** @brief The native logging backend was unavailable. */
        ISAACSIM_COMMON_LOGGING_CONFIGURE_BACKEND_UNAVAILABLE = 2,
    } IsaacSimCommonLoggingConfigureResult;

    /** @brief Selection of the standard stream used by the Carbonite standard logger. */
    typedef enum IsaacSimCommonLoggingOutputStream
    {
        /** @brief Route lower severities to standard output and errors to standard error. */
        ISAACSIM_COMMON_LOGGING_OUTPUT_STREAM_DEFAULT = 0,
        /** @brief Route all standard-stream records to standard error. */
        ISAACSIM_COMMON_LOGGING_OUTPUT_STREAM_STDERR = 1,
    } IsaacSimCommonLoggingOutputStream;

    /** @brief Unit used for the elapsed-time prefix. */
    typedef enum IsaacSimCommonLoggingElapsedTimeUnit
    {
        /** @brief Do not include an elapsed-time prefix. */
        ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_DISABLED = 0,
        /** @brief Display elapsed time in milliseconds. */
        ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_MILLISECONDS = 1,
        /** @brief Display elapsed time in microseconds. */
        ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_MICROSECONDS = 2,
        /** @brief Display elapsed time in nanoseconds. */
        ISAACSIM_COMMON_LOGGING_ELAPSED_TIME_NANOSECONDS = 3,
    } IsaacSimCommonLoggingElapsedTimeUnit;

    /** @brief Action to apply to one channel setting in a configuration patch. */
    typedef enum IsaacSimCommonLoggingChannelSettingBehavior
    {
        /** @brief Leave the existing Carbonite per-channel setting unchanged. */
        ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_UNCHANGED = 0,
        /** @brief Clear the per-channel override and inherit the process-wide setting. */
        ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_INHERIT = 1,
        /** @brief Override the process-wide setting with the supplied channel value. */
        ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_OVERRIDE = 2,
    } IsaacSimCommonLoggingChannelSettingBehavior;

    /** @brief Patch-style Carbonite settings for one logging channel. */
    typedef struct IsaacSimCommonLoggingChannelConfig
    {
        size_t structSize; /**< Size of this structure for ABI versioning. */
        const char* channel; /**< Non-empty null-terminated channel name. */
        int32_t enabledBehavior; /**< Action to apply to `enabled`. */
        uint32_t enabled; /**< Channel enablement used by `OVERRIDE`. */
        int32_t minimumLevelBehavior; /**< Action to apply to `minimumLevel`. */
        int32_t minimumLevel; /**< Channel threshold used by `OVERRIDE`. */
    } IsaacSimCommonLoggingChannelConfig;

    /**
     * @brief Return an unchanged channel patch with the ABI size initialized.
     *
     * @return Initialized channel configuration patch.
     */
    static inline IsaacSimCommonLoggingChannelConfig isaacsimCommonLoggingMakeChannelConfig(void)
    {
#ifdef __cplusplus
        IsaacSimCommonLoggingChannelConfig configuration{};
#else
    IsaacSimCommonLoggingChannelConfig configuration = { 0 };
#endif
        configuration.structSize = sizeof(configuration);
        return configuration;
    }

#define ISAACSIM_COMMON_LOGGING_CHANNEL_CONFIG_INIT isaacsimCommonLoggingMakeChannelConfig()

    /** @brief Bit mask that selects members of a process-wide configuration patch. */
    typedef uint64_t IsaacSimCommonLoggingGlobalConfigFields;

#define ISAACSIM_COMMON_LOGGING_CONFIG_ENABLED (UINT64_C(1) << 0)
#define ISAACSIM_COMMON_LOGGING_CONFIG_MINIMUM_LEVEL (UINT64_C(1) << 1)
#define ISAACSIM_COMMON_LOGGING_CONFIG_ASYNCHRONOUS (UINT64_C(1) << 2)
#define ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_ENABLED (UINT64_C(1) << 3)
#define ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_LEVEL (UINT64_C(1) << 4)
#define ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_FLUSH (UINT64_C(1) << 5)
#define ISAACSIM_COMMON_LOGGING_CONFIG_OUTPUT_STREAM (UINT64_C(1) << 6)
#define ISAACSIM_COMMON_LOGGING_CONFIG_DEBUG_CONSOLE_ENABLED (UINT64_C(1) << 7)
#define ISAACSIM_COMMON_LOGGING_CONFIG_DEBUG_CONSOLE_LEVEL (UINT64_C(1) << 8)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FILE_PATH (UINT64_C(1) << 9)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FILE_APPEND (UINT64_C(1) << 10)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FILE_LEVEL (UINT64_C(1) << 11)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FILE_FLUSH_LEVEL (UINT64_C(1) << 12)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FILENAME_INCLUDED (UINT64_C(1) << 13)
#define ISAACSIM_COMMON_LOGGING_CONFIG_LINE_NUMBER_INCLUDED (UINT64_C(1) << 14)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FUNCTION_NAME_INCLUDED (UINT64_C(1) << 15)
#define ISAACSIM_COMMON_LOGGING_CONFIG_TIMESTAMP_INCLUDED (UINT64_C(1) << 16)
#define ISAACSIM_COMMON_LOGGING_CONFIG_UTC_TIMESTAMPS (UINT64_C(1) << 17)
#define ISAACSIM_COMMON_LOGGING_CONFIG_MICROSECOND_TIMESTAMPS (UINT64_C(1) << 18)
#define ISAACSIM_COMMON_LOGGING_CONFIG_ELAPSED_TIME (UINT64_C(1) << 19)
#define ISAACSIM_COMMON_LOGGING_CONFIG_THREAD_ID_INCLUDED (UINT64_C(1) << 20)
#define ISAACSIM_COMMON_LOGGING_CONFIG_SOURCE_INCLUDED (UINT64_C(1) << 21)
#define ISAACSIM_COMMON_LOGGING_CONFIG_PROCESS_ID_INCLUDED (UINT64_C(1) << 22)
#define ISAACSIM_COMMON_LOGGING_CONFIG_TRACE_ID_INCLUDED (UINT64_C(1) << 23)
#define ISAACSIM_COMMON_LOGGING_CONFIG_COLOR_INCLUDED (UINT64_C(1) << 24)
#define ISAACSIM_COMMON_LOGGING_CONFIG_FORCE_ANSI_COLOR (UINT64_C(1) << 25)
#define ISAACSIM_COMMON_LOGGING_CONFIG_MULTIPROCESS_GROUP_ID (UINT64_C(1) << 26)
#define ISAACSIM_COMMON_LOGGING_CONFIG_CHANNELS (UINT64_C(1) << 27)

    /**
     * @brief Patch-style process-wide logging configuration.
     *
     * Only members selected by `fields` are applied. Other backend settings are left unchanged.
     * Set `structSize` to `sizeof(IsaacSimCommonLoggingGlobalConfig)`. When `FILE_PATH`
     * is selected, a NULL or empty path disables file logging. Parent directories are not created.
     * When `CHANNELS` is selected, each entry independently patches a Carbonite per-source
     * enablement setting and threshold. `INHERIT` clears the corresponding override, and
     * `UNCHANGED` leaves it untouched.
     */
    typedef struct IsaacSimCommonLoggingGlobalConfig
    {
        size_t structSize; /**< Size of this structure for ABI versioning. */
        /** Bit mask selecting members to apply. */
        IsaacSimCommonLoggingGlobalConfigFields fields;
        uint32_t enabled; /**< Whether process-wide logging is enabled. */
        int32_t minimumLevel; /**< Process-wide admission threshold. */
        uint32_t asynchronous; /**< Whether backend delivery is asynchronous. */
        uint32_t standardStreamEnabled; /**< Whether stdout and stderr output is enabled. */
        int32_t standardStreamLevel; /**< Standard-stream threshold. */
        uint32_t standardStreamFlush; /**< Whether stdout is flushed after every record. */
        int32_t outputStream; /**< Standard-stream selection. */
        uint32_t debugConsoleEnabled; /**< Whether the Windows debug console is enabled. */
        int32_t debugConsoleLevel; /**< Windows debug-console threshold. */
        uint32_t fileAppend; /**< Whether file output appends instead of replacing. */
        int32_t fileLevel; /**< File-destination threshold. */
        int32_t fileFlushLevel; /**< Minimum severity that flushes the file. */
        uint32_t filenameIncluded; /**< Whether source filenames are included. */
        uint32_t lineNumberIncluded; /**< Whether source line numbers are included. */
        uint32_t functionNameIncluded; /**< Whether source function names are included. */
        uint32_t timestampIncluded; /**< Whether absolute timestamps are included. */
        uint32_t utcTimestamps; /**< Whether absolute timestamps use UTC. */
        /** Whether absolute timestamps use microsecond precision. */
        uint32_t microsecondTimestamps;
        int32_t elapsedTime; /**< Elapsed-time prefix unit. */
        uint32_t threadIdIncluded; /**< Whether thread identifiers are included. */
        uint32_t sourceIncluded; /**< Whether logger channels are included. */
        uint32_t processIdIncluded; /**< Whether process identifiers are included. */
        uint32_t traceIdIncluded; /**< Whether active trace identifiers are included. */
        uint32_t colorIncluded; /**< Whether terminal color codes are included. */
        uint32_t forceAnsiColor; /**< Whether ANSI color codes are forced. */
        /** Nonzero identifier for cross-process output serialization. */
        int32_t multiprocessGroupId;
        const char* filePath; /**< UTF-8 file path, or NULL or empty to disable file output. */
        /** Borrowed channel patch array. */
        const IsaacSimCommonLoggingChannelConfig* channelConfigs;
        size_t channelConfigCount; /**< Number of entries in `channelConfigs`. */
    } IsaacSimCommonLoggingGlobalConfig;

    /**
     * @brief Return a zero-valued configuration patch with the ABI size initialized.
     *
     * @return Initialized process-wide configuration patch.
     */
    static inline IsaacSimCommonLoggingGlobalConfig isaacsimCommonLoggingMakeGlobalConfig(void)
    {
#ifdef __cplusplus
        IsaacSimCommonLoggingGlobalConfig configuration{};
#else
    IsaacSimCommonLoggingGlobalConfig configuration = { 0 };
#endif
        configuration.structSize = sizeof(configuration);
        return configuration;
    }

#define ISAACSIM_COMMON_LOGGING_GLOBAL_CONFIG_INIT isaacsimCommonLoggingMakeGlobalConfig()

    /**
     * @brief Apply selected process-wide logging settings.
     *
     * Configuration calls are serialized with each other. Log emission may continue concurrently
     * and can observe the settings as Carbonite applies them. Success means that the backend
     * accepted the settings; Carbonite does not report file-open failures through its
     * configuration API.
     * A synchronous reentrant call from a backend log callback returns
     * `ISAACSIM_COMMON_LOGGING_CONFIGURE_BACKEND_UNAVAILABLE` without changing configuration.
     *
     * @param[in] configuration Configuration patch to apply. The pointed-to data is not retained.
     * @return Configuration result.
     */
    ISAACSIM_COMMON_LOGGING_API IsaacSimCommonLoggingConfigureResult
    isaacsimCommonLoggingConfigureGlobal(const IsaacSimCommonLoggingGlobalConfig* configuration);

    /**
     * @brief Submit an already formatted message to Carbonite.
     *
     * `channel` must be a non-empty null-terminated string, and `message` must be a
     * null-terminated string. Invalid input is ignored because logging is supplemental. `file`
     * and `function` may be NULL; `line` may be zero. Carbonite owns filtering, timestamps,
     * destinations, callbacks, and asynchronous delivery. Input strings are not retained.
     *
     * @param[in] severity Severity assigned to the record.
     * @param[in] channel Non-empty null-terminated channel name.
     * @param[in] message Null-terminated message to submit.
     * @param[in] file Source filename, or NULL when unavailable.
     * @param[in] function Source function name, or NULL when unavailable.
     * @param[in] line One-based source line number, or zero when unavailable.
     */
    ISAACSIM_COMMON_LOGGING_API void isaacsimCommonLoggingWrite(IsaacSimCommonLoggingLogLevel severity,
                                                                const char* channel,
                                                                const char* message,
                                                                const char* file,
                                                                const char* function,
                                                                unsigned int line);

    /**
     * @brief Print one channel-prefixed line to standard output and submit the message to Carbonite at INFO.
     *
     * The standard-output write is independent of Carbonite's level and enabled settings. Carbonite
     * delivery remains subject to its configured policy. Input and source-location semantics match
     * `isaacsimCommonLoggingWrite`. The standard-output line is `[channel] message`.
     *
     * @param[in] channel Non-empty null-terminated channel name.
     * @param[in] message Null-terminated message to print after the channel prefix and submit.
     * @param[in] file Source filename, or NULL when unavailable.
     * @param[in] function Source function name, or NULL when unavailable.
     * @param[in] line One-based source line number, or zero when unavailable.
     */
    ISAACSIM_COMMON_LOGGING_API void isaacsimCommonLoggingReport(
        const char* channel, const char* message, const char* file, const char* function, unsigned int line);

    /** @brief Flush records currently pending in the process-wide Carbonite backend. */
    ISAACSIM_COMMON_LOGGING_API void isaacsimCommonLoggingFlush(void);

#ifdef __cplusplus
}
#endif

/** @brief Submit a VERBOSE record while preserving the call-site source location. */
#define ISAACSIM_COMMON_LOGGING_VERBOSE(channel, message)                                                              \
    isaacsimCommonLoggingWrite(ISAACSIM_COMMON_LOGGING_LOG_VERBOSE, channel, message, __FILE__, __func__, __LINE__)
/** @brief Submit an INFO record while preserving the call-site source location. */
#define ISAACSIM_COMMON_LOGGING_INFO(channel, message)                                                                 \
    isaacsimCommonLoggingWrite(ISAACSIM_COMMON_LOGGING_LOG_INFO, channel, message, __FILE__, __func__, __LINE__)
/** @brief Submit a WARNING record while preserving the call-site source location. */
#define ISAACSIM_COMMON_LOGGING_WARNING(channel, message)                                                              \
    isaacsimCommonLoggingWrite(ISAACSIM_COMMON_LOGGING_LOG_WARNING, channel, message, __FILE__, __func__, __LINE__)
/** @brief Submit an ERROR record while preserving the call-site source location. */
#define ISAACSIM_COMMON_LOGGING_ERROR(channel, message)                                                                \
    isaacsimCommonLoggingWrite(ISAACSIM_COMMON_LOGGING_LOG_ERROR, channel, message, __FILE__, __func__, __LINE__)
/** @brief Print and submit a report while preserving the call-site source location. */
#define ISAACSIM_COMMON_LOGGING_REPORT(channel, message)                                                               \
    isaacsimCommonLoggingReport(channel, message, __FILE__, __func__, __LINE__)
