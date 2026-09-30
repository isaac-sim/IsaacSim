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

#include "isaacsim/common/logging/Logging.hpp"

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct FormatInvocationProbe
{
    std::atomic<size_t>* invocationCount;
};

} // namespace

namespace fmt
{

template <>
struct formatter<FormatInvocationProbe> : formatter<std::string_view>
{
    template <typename FormatContext>
    auto format(const FormatInvocationProbe& probe, FormatContext& context)
    {
        probe.invocationCount->fetch_add(1, std::memory_order_relaxed);
        return formatter<std::string_view>::format("probe", context);
    }
};

} // namespace fmt

namespace isaacsim
{
namespace common
{
namespace logging
{

namespace
{

void submitWarningOnce(Logger& logger, int threadIndex)
{
    ISAACSIM_LOG_WARN_ONCE(logger, "isaacsim-warning-once-probe {}", threadIndex);
}

void submitDeprecationOnce(Logger& logger, int threadIndex)
{
    ISAACSIM_LOG_DEPRECATION_ONCE(logger, "isaacsim-deprecation-once-probe {}", threadIndex);
}

size_t countOccurrences(const std::string& text, const std::string& value)
{
    size_t count = 0;
    size_t position = 0;
    while ((position = text.find(value, position)) != std::string::npos)
    {
        ++count;
        position += value.size();
    }
    return count;
}

} // namespace

TEST_SUITE("isaacsim.common.logging")
{
    TEST_CASE("Logger owns one immutable channel")
    {
        Logger logger("isaacsim.test");
        CHECK(logger.getChannel() == "isaacsim.test");
        CHECK_THROWS_AS(Logger(""), std::invalid_argument);
    }

    TEST_CASE("First submission initializes the standalone Carbonite backend")
    {
        Logger logger("isaacsim.test.backend");
        logger.log(LogLevel::eWarning, "isaacsim-carbonite-backend-probe");
        flush();
    }

    TEST_CASE("Formatting macros preserve the explicit logger")
    {
        Logger logger("isaacsim.test.formatting");
        ISAACSIM_LOG_VERBOSE(logger, "verbose {}", 1);
        ISAACSIM_LOG_INFO(logger, "info {}", "message");
        ISAACSIM_LOG_WARNING(logger, "warning {}", size_t{ 3 });
        ISAACSIM_LOG_WARN(logger, "warning alias");
        ISAACSIM_LOG_ERROR(logger, "error {:.1f}", 4.0);
        ISAACSIM_LOG_INFO(logger, FMT_STRING("compile-time checked {}"), 5);
        CHECK_NOTHROW(logger.logFormatted(LogLevel::eInfo, {}, std::string("{"), 6));
        flush();
    }

    TEST_CASE("Filtered formatted records do not format arguments")
    {
        const std::string channelName = "isaacsim.test.filtered_formatting";
        ChannelLoggingConfiguration channel;
        channel.channel = channelName;
        channel.minimumLevelBehavior = ChannelSettingBehavior::eOverride;
        channel.minimumLevel = LogLevel::eError;
        GlobalLoggingConfiguration filterConfiguration;
        filterConfiguration.channels.push_back(channel);
        CHECK(configureGlobalLogging(filterConfiguration) == ConfigureResult::eSuccess);

        Logger logger(channelName);
        std::atomic<size_t> invocationCount{ 0 };
        size_t argumentEvaluationCount = 0;
        ISAACSIM_LOG_INFO(logger, "{}", ++argumentEvaluationCount);
        CHECK(argumentEvaluationCount == 0);
        ISAACSIM_LOG_INFO(logger, "{}", FormatInvocationProbe{ &invocationCount });
        CHECK(invocationCount.load(std::memory_order_relaxed) == 0);
        ISAACSIM_LOG_ERROR(logger, "{}", ++argumentEvaluationCount);
        CHECK(argumentEvaluationCount == 1);
        ISAACSIM_LOG_ERROR(logger, "{}", FormatInvocationProbe{ &invocationCount });
        CHECK(invocationCount.load(std::memory_order_relaxed) == 1);

        channel.minimumLevelBehavior = ChannelSettingBehavior::eInherit;
        GlobalLoggingConfiguration restoreConfiguration;
        restoreConfiguration.channels.push_back(channel);
        CHECK(configureGlobalLogging(restoreConfiguration) == ConfigureResult::eSuccess);
    }

    TEST_CASE("Once macros submit one admitted record per call site under contention")
    {
        const std::filesystem::path logPath =
            std::filesystem::temp_directory_path() /
            ("isaacsim-common-logging-once-" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".log");

        GlobalLoggingConfiguration configuration;
        configuration.minimumLevel = LogLevel::eVerbose;
        configuration.standardStreamEnabled = false;
        configuration.filePath = logPath.string();
        configuration.fileAppend = false;
        configuration.fileLevel = LogLevel::eVerbose;
        configuration.fileFlushLevel = LogLevel::eVerbose;
        configuration.colorIncluded = false;
        ChannelLoggingConfiguration channel;
        channel.channel = "isaacsim.test.warning_once";
        channel.minimumLevelBehavior = ChannelSettingBehavior::eOverride;
        channel.minimumLevel = LogLevel::eError;
        configuration.channels.push_back(channel);
        CHECK(configureGlobalLogging(configuration) == ConfigureResult::eSuccess);

        Logger logger(channel.channel);
        CHECK_FALSE(logger.isEnabled(LogLevel::eWarning));
        submitWarningOnce(logger, -1);
        submitDeprecationOnce(logger, -1);

        GlobalLoggingConfiguration enableWarning;
        channel.minimumLevelBehavior = ChannelSettingBehavior::eInherit;
        enableWarning.channels.push_back(channel);
        CHECK(configureGlobalLogging(enableWarning) == ConfigureResult::eSuccess);
        CHECK(logger.isEnabled(LogLevel::eWarning));

        std::vector<std::thread> threads;
        for (int threadIndex = 0; threadIndex < 16; ++threadIndex)
        {
            threads.emplace_back(
                [&logger, threadIndex]
                {
                    submitWarningOnce(logger, threadIndex);
                    submitDeprecationOnce(logger, threadIndex);
                });
        }
        for (std::thread& thread : threads)
        {
            thread.join();
        }
        flush();

        GlobalLoggingConfiguration restore;
        restore.minimumLevel = LogLevel::eWarning;
        restore.standardStreamEnabled = true;
        restore.filePath = "";
        CHECK(configureGlobalLogging(restore) == ConfigureResult::eSuccess);

        std::ifstream file(logPath);
        const std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        CHECK(countOccurrences(contents, "isaacsim-warning-once-probe") == 1);
        CHECK(countOccurrences(contents, "isaacsim-deprecation-once-probe") == 1);
        file.close();
        std::filesystem::remove(logPath);
    }

    TEST_CASE("Report writes to standard output and submits to Carbonite")
    {
        Logger logger("isaacsim.test.report");
        logger.report("isaacsim-cpp-report-probe");
        ISAACSIM_REPORT(logger, "isaacsim-cpp-formatted-result-{}", 42);
        flush();
    }

    TEST_CASE("Process and channel configuration controls file logging")
    {
        const std::filesystem::path logPath =
            std::filesystem::temp_directory_path() /
            ("isaacsim-common-logging-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
             ".log");

        GlobalLoggingConfiguration globalConfiguration;
        globalConfiguration.minimumLevel = LogLevel::eVerbose;
        globalConfiguration.asynchronous = true;
        globalConfiguration.filePath = logPath.string();
        globalConfiguration.fileAppend = false;
        globalConfiguration.fileLevel = LogLevel::eVerbose;
        globalConfiguration.fileFlushLevel = LogLevel::eVerbose;
        globalConfiguration.colorIncluded = false;

        ChannelLoggingConfiguration thresholdChannel;
        thresholdChannel.channel = "isaacsim.test.file.threshold";
        thresholdChannel.minimumLevelBehavior = ChannelSettingBehavior::eOverride;
        thresholdChannel.minimumLevel = LogLevel::eError;
        globalConfiguration.channels.push_back(thresholdChannel);

        ChannelLoggingConfiguration disabledChannel;
        disabledChannel.channel = "isaacsim.test.file.disabled";
        disabledChannel.enabledBehavior = ChannelSettingBehavior::eOverride;
        disabledChannel.enabled = false;
        globalConfiguration.channels.push_back(disabledChannel);
        CHECK(configureGlobalLogging(globalConfiguration) == ConfigureResult::eSuccess);

        Logger thresholdLogger(thresholdChannel.channel);
        thresholdLogger.log(LogLevel::eWarning, "filtered-by-channel-threshold");
        thresholdLogger.log(LogLevel::eError, "included-by-channel-threshold");
        Logger disabledLogger(disabledChannel.channel);
        CHECK_FALSE(disabledLogger.isEnabled(LogLevel::eError));
        disabledLogger.log(LogLevel::eError, "filtered-by-disabled-channel");
        Logger inheritedLogger("isaacsim.test.file.inherited");
        inheritedLogger.log(LogLevel::eInfo, "included-by-global-policy");
        flush();

        GlobalLoggingConfiguration restoreChannels;
        thresholdChannel.minimumLevelBehavior = ChannelSettingBehavior::eInherit;
        disabledChannel.enabledBehavior = ChannelSettingBehavior::eInherit;
        restoreChannels.channels = { thresholdChannel, disabledChannel };
        CHECK(configureGlobalLogging(restoreChannels) == ConfigureResult::eSuccess);
        thresholdLogger.log(LogLevel::eWarning, "included-after-threshold-inherit");
        disabledLogger.log(LogLevel::eError, "included-after-enabled-inherit");
        flush();

        GlobalLoggingConfiguration disableFile;
        disableFile.filePath = "";
        disableFile.minimumLevel = LogLevel::eWarning;
        disableFile.asynchronous = false;
        CHECK(configureGlobalLogging(disableFile) == ConfigureResult::eSuccess);

        std::ifstream file(logPath);
        const std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        CHECK(contents.find("included-by-channel-threshold") != std::string::npos);
        CHECK(contents.find("included-by-global-policy") != std::string::npos);
        CHECK(contents.find("included-after-threshold-inherit") != std::string::npos);
        CHECK(contents.find("included-after-enabled-inherit") != std::string::npos);
        CHECK(contents.find("filtered-by-channel-threshold") == std::string::npos);
        CHECK(contents.find("filtered-by-disabled-channel") == std::string::npos);
        file.close();
        std::filesystem::remove(logPath);
    }

    TEST_CASE("Invalid channel configuration is rejected")
    {
        GlobalLoggingConfiguration configuration;
        ChannelLoggingConfiguration channel;
        channel.enabledBehavior = ChannelSettingBehavior::eOverride;
        channel.enabled = false;
        configuration.channels.push_back(channel);
        CHECK(configureGlobalLogging(configuration) == ConfigureResult::eInvalidArgument);
    }

    TEST_CASE("Logger submission is safe from multiple threads")
    {
        Logger logger("isaacsim.test.threading");
        GlobalLoggingConfiguration configuration;
        configuration.elapsedTime = ElapsedTimeUnit::eMilliseconds;
        std::atomic<bool> configurationSucceeded{ true };
        std::vector<std::thread> threads;
        for (int threadIndex = 0; threadIndex < 4; ++threadIndex)
        {
            threads.emplace_back(
                [&logger, &configuration, &configurationSucceeded, threadIndex]
                {
                    for (int recordIndex = 0; recordIndex < 100; ++recordIndex)
                    {
                        if (configureGlobalLogging(configuration) != ConfigureResult::eSuccess)
                        {
                            configurationSucceeded.store(false);
                        }
                        logger.log(LogLevel::eInfo,
                                   "thread " + std::to_string(threadIndex) + " record " + std::to_string(recordIndex));
                    }
                });
        }
        for (std::thread& thread : threads)
        {
            thread.join();
        }
        CHECK(configurationSucceeded.load());
        flush();
    }
}

} // namespace logging
} // namespace common
} // namespace isaacsim
