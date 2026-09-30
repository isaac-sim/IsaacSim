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

#include <memory>
#include <string_view>

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

/**
 * @brief Acquire the process-wide Carbonite channel state for a name.
 *
 * @param[in] channel Non-empty channel name.
 * @return Shared channel state.
 */
std::shared_ptr<CarboniteChannel> acquireCarboniteChannel(std::string_view channel);

/**
 * @brief Get the name stored in a Carbonite channel state object.
 *
 * @param[in] channel Shared channel state, or `nullptr`.
 * @return Channel name, or an empty view when `channel` is `nullptr`.
 */
std::string_view getCarboniteChannelName(const std::shared_ptr<CarboniteChannel>& channel) noexcept;

/**
 * @brief Return whether a Carbonite channel currently admits a severity.
 *
 * @param[in] severity Severity to query.
 * @param[in] channel Shared channel state.
 * @return `true` when the channel currently admits the severity.
 */
bool isCarboniteLevelEnabled(LogLevel severity, const std::shared_ptr<CarboniteChannel>& channel) noexcept;

/**
 * @brief Submit a record through an acquired Carbonite channel.
 *
 * @param[in] severity Severity assigned to the record.
 * @param[in] message Message to submit.
 * @param[in] channel Shared channel state.
 * @param[in] location Borrowed source metadata.
 */
void emitToCarbonite(LogLevel severity,
                     std::string_view message,
                     const std::shared_ptr<CarboniteChannel>& channel,
                     SourceLocation location) noexcept;

/**
 * @brief Print a channel-prefixed report and submit it through an acquired Carbonite channel.
 *
 * @param[in] message Message to print and submit.
 * @param[in] channel Shared channel state.
 * @param[in] location Borrowed source metadata.
 */
void reportToStandardOutputAndCarbonite(std::string_view message,
                                        const std::shared_ptr<CarboniteChannel>& channel,
                                        SourceLocation location) noexcept;

/**
 * @brief Submit a record through a named Carbonite channel.
 *
 * @param[in] severity Severity assigned to the record.
 * @param[in] message Message to submit.
 * @param[in] channel Non-empty null-terminated channel name.
 * @param[in] location Borrowed source metadata.
 */
void emitToCarbonite(LogLevel severity, std::string_view message, const char* channel, SourceLocation location) noexcept;

/**
 * @brief Print a channel-prefixed report and submit it through a named Carbonite channel.
 *
 * @param[in] message Message to print and submit.
 * @param[in] channel Non-empty null-terminated channel name.
 * @param[in] location Borrowed source metadata.
 */
void reportToStandardOutputAndCarbonite(std::string_view message, const char* channel, SourceLocation location) noexcept;

/**
 * @brief Apply a validated process-wide Carbonite configuration patch.
 *
 * @param[in] configuration Configuration patch to apply.
 * @return Configuration result.
 */
IsaacSimCommonLoggingConfigureResult configureCarboniteGlobal(const IsaacSimCommonLoggingGlobalConfig& configuration) noexcept;

/** @brief Flush records pending in the process-wide Carbonite backend. */
void flushCarboniteBackend() noexcept;

} // namespace details
} // namespace logging
} // namespace common
} // namespace isaacsim
