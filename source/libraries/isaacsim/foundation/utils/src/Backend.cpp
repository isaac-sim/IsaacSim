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

#include <isaacsim/foundation/utils/Backend.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace utils
{

namespace details
{

thread_local BackendContext g_context;

} // namespace details

namespace
{

std::string resolveCurrentBackend(const std::string* data, std::size_t count, std::optional<bool> raiseOnUnsupported)
{
    if (count == 0)
    {
        throw std::invalid_argument("The list of supported backends must not be empty");
    }

    const std::string& defaultBackend = data[0];
    const std::string& activeBackend =
        details::g_context.backend.has_value() ? *details::g_context.backend : defaultBackend;

    for (std::size_t i = 0; i < count; ++i)
    {
        if (data[i] == activeBackend)
        {
            return activeBackend;
        }
    }

    if (raiseOnUnsupported.value_or(details::g_context.raiseOnUnsupported))
    {
        std::ostringstream messageStream;
        messageStream << "Unsupported backend '" << activeBackend << "'. Supported backends: ";
        for (std::size_t i = 0; i < count; ++i)
        {
            if (i > 0)
            {
                messageStream << ", ";
            }
            messageStream << '\'' << data[i] << '\'';
        }
        throw std::runtime_error(messageStream.str());
    }
    return defaultBackend;
}

} // namespace


BackendGuard::BackendGuard(const std::string& backend, bool raiseOnUnsupported, bool raiseOnFallback)
    : m_previous(details::g_context)
{
    details::g_context.backend = backend;
    details::g_context.raiseOnUnsupported = raiseOnUnsupported;
    details::g_context.raiseOnFallback = raiseOnFallback;
}

BackendGuard::~BackendGuard()
{
    details::g_context = m_previous;
}

std::string getCurrentBackend(const std::initializer_list<std::string>& supportedBackends,
                              std::optional<bool> raiseOnUnsupported)
{
    return resolveCurrentBackend(supportedBackends.begin(), supportedBackends.size(), raiseOnUnsupported);
}

std::string getCurrentBackend(const std::vector<std::string>& supportedBackends, std::optional<bool> raiseOnUnsupported)
{
    return resolveCurrentBackend(supportedBackends.data(), supportedBackends.size(), raiseOnUnsupported);
}

bool isBackendSet()
{
    return details::g_context.backend.has_value();
}

bool shouldRaiseOnUnsupported()
{
    return details::g_context.raiseOnUnsupported;
}

bool shouldRaiseOnFallback()
{
    return details::g_context.raiseOnFallback;
}

} // namespace utils
} // namespace foundation
} // namespace isaacsim
