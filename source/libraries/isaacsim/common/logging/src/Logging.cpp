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

#include "details/CarboniteBackend.hpp"

#include <stdexcept>

namespace isaacsim
{
namespace common
{
namespace logging
{
namespace
{

template <typename Source, typename Destination>
void assignOptional(const std::optional<Source>& source, uint64_t field, uint64_t& fields, Destination& destination)
{
    if (source.has_value())
    {
        fields |= field;
        destination = static_cast<Destination>(*source);
    }
}

std::shared_ptr<details::CarboniteChannel> acquireLoggerChannel(std::string_view channel)
{
    if (channel.empty())
    {
        throw std::invalid_argument("A logging channel must not be empty.");
    }
    return details::acquireCarboniteChannel(channel);
}

} // namespace

ConfigureResult configureGlobalLogging(const GlobalLoggingConfiguration& configuration) noexcept
{
    IsaacSimCommonLoggingGlobalConfig nativeConfiguration{};
    nativeConfiguration.structSize = sizeof(nativeConfiguration);
    std::vector<IsaacSimCommonLoggingChannelConfig> nativeChannels;
    assignOptional(configuration.enabled, ISAACSIM_COMMON_LOGGING_CONFIG_ENABLED, nativeConfiguration.fields,
                   nativeConfiguration.enabled);
    assignOptional(configuration.minimumLevel, ISAACSIM_COMMON_LOGGING_CONFIG_MINIMUM_LEVEL, nativeConfiguration.fields,
                   nativeConfiguration.minimumLevel);
    assignOptional(configuration.asynchronous, ISAACSIM_COMMON_LOGGING_CONFIG_ASYNCHRONOUS, nativeConfiguration.fields,
                   nativeConfiguration.asynchronous);
    assignOptional(configuration.standardStreamEnabled, ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_ENABLED,
                   nativeConfiguration.fields, nativeConfiguration.standardStreamEnabled);
    assignOptional(configuration.standardStreamLevel, ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_LEVEL,
                   nativeConfiguration.fields, nativeConfiguration.standardStreamLevel);
    assignOptional(configuration.standardStreamFlush, ISAACSIM_COMMON_LOGGING_CONFIG_STANDARD_STREAM_FLUSH,
                   nativeConfiguration.fields, nativeConfiguration.standardStreamFlush);
    assignOptional(configuration.outputStream, ISAACSIM_COMMON_LOGGING_CONFIG_OUTPUT_STREAM, nativeConfiguration.fields,
                   nativeConfiguration.outputStream);
    assignOptional(configuration.debugConsoleEnabled, ISAACSIM_COMMON_LOGGING_CONFIG_DEBUG_CONSOLE_ENABLED,
                   nativeConfiguration.fields, nativeConfiguration.debugConsoleEnabled);
    assignOptional(configuration.debugConsoleLevel, ISAACSIM_COMMON_LOGGING_CONFIG_DEBUG_CONSOLE_LEVEL,
                   nativeConfiguration.fields, nativeConfiguration.debugConsoleLevel);
    if (configuration.filePath.has_value())
    {
        nativeConfiguration.fields |= ISAACSIM_COMMON_LOGGING_CONFIG_FILE_PATH;
        nativeConfiguration.filePath = configuration.filePath->empty() ? nullptr : configuration.filePath->c_str();
    }
    assignOptional(configuration.fileAppend, ISAACSIM_COMMON_LOGGING_CONFIG_FILE_APPEND, nativeConfiguration.fields,
                   nativeConfiguration.fileAppend);
    assignOptional(configuration.fileLevel, ISAACSIM_COMMON_LOGGING_CONFIG_FILE_LEVEL, nativeConfiguration.fields,
                   nativeConfiguration.fileLevel);
    assignOptional(configuration.fileFlushLevel, ISAACSIM_COMMON_LOGGING_CONFIG_FILE_FLUSH_LEVEL,
                   nativeConfiguration.fields, nativeConfiguration.fileFlushLevel);
    assignOptional(configuration.filenameIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_FILENAME_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.filenameIncluded);
    assignOptional(configuration.lineNumberIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_LINE_NUMBER_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.lineNumberIncluded);
    assignOptional(configuration.functionNameIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_FUNCTION_NAME_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.functionNameIncluded);
    assignOptional(configuration.timestampIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_TIMESTAMP_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.timestampIncluded);
    assignOptional(configuration.utcTimestamps, ISAACSIM_COMMON_LOGGING_CONFIG_UTC_TIMESTAMPS,
                   nativeConfiguration.fields, nativeConfiguration.utcTimestamps);
    assignOptional(configuration.microsecondTimestamps, ISAACSIM_COMMON_LOGGING_CONFIG_MICROSECOND_TIMESTAMPS,
                   nativeConfiguration.fields, nativeConfiguration.microsecondTimestamps);
    assignOptional(configuration.elapsedTime, ISAACSIM_COMMON_LOGGING_CONFIG_ELAPSED_TIME, nativeConfiguration.fields,
                   nativeConfiguration.elapsedTime);
    assignOptional(configuration.threadIdIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_THREAD_ID_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.threadIdIncluded);
    assignOptional(configuration.sourceIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_SOURCE_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.sourceIncluded);
    assignOptional(configuration.processIdIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_PROCESS_ID_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.processIdIncluded);
    assignOptional(configuration.traceIdIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_TRACE_ID_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.traceIdIncluded);
    assignOptional(configuration.colorIncluded, ISAACSIM_COMMON_LOGGING_CONFIG_COLOR_INCLUDED,
                   nativeConfiguration.fields, nativeConfiguration.colorIncluded);
    assignOptional(configuration.forceAnsiColor, ISAACSIM_COMMON_LOGGING_CONFIG_FORCE_ANSI_COLOR,
                   nativeConfiguration.fields, nativeConfiguration.forceAnsiColor);
    assignOptional(configuration.multiprocessGroupId, ISAACSIM_COMMON_LOGGING_CONFIG_MULTIPROCESS_GROUP_ID,
                   nativeConfiguration.fields, nativeConfiguration.multiprocessGroupId);
    if (!configuration.channels.empty())
    {
        nativeChannels.reserve(configuration.channels.size());
        for (const ChannelLoggingConfiguration& channel : configuration.channels)
        {
            IsaacSimCommonLoggingChannelConfig nativeChannel = ISAACSIM_COMMON_LOGGING_CHANNEL_CONFIG_INIT;
            nativeChannel.channel = channel.channel.c_str();
            nativeChannel.enabledBehavior = static_cast<int32_t>(channel.enabledBehavior);
            nativeChannel.enabled = channel.enabled ? 1u : 0u;
            nativeChannel.minimumLevelBehavior = static_cast<int32_t>(channel.minimumLevelBehavior);
            nativeChannel.minimumLevel = static_cast<int32_t>(channel.minimumLevel);
            nativeChannels.push_back(nativeChannel);
        }
        nativeConfiguration.fields |= ISAACSIM_COMMON_LOGGING_CONFIG_CHANNELS;
        nativeConfiguration.channelConfigs = nativeChannels.data();
        nativeConfiguration.channelConfigCount = nativeChannels.size();
    }

    return static_cast<ConfigureResult>(isaacsimCommonLoggingConfigureGlobal(&nativeConfiguration));
}

Logger::Logger(std::string_view channel) : m_channel(acquireLoggerChannel(channel))
{
}

Logger::~Logger() = default;

std::string_view Logger::getChannel() const noexcept
{
    return details::getCarboniteChannelName(m_channel);
}

bool Logger::isEnabled(LogLevel severity) const noexcept
{
    return details::isCarboniteLevelEnabled(severity, m_channel);
}

void Logger::log(LogLevel severity, std::string_view message, SourceLocation location) const noexcept
{
    details::emitToCarbonite(severity, message, m_channel, location);
}

void Logger::report(std::string_view message, SourceLocation location) const noexcept
{
    details::reportToStandardOutputAndCarbonite(message, m_channel, location);
}

void flush() noexcept
{
    details::flushCarboniteBackend();
}

} // namespace logging
} // namespace common
} // namespace isaacsim
