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

#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace isaacsim
{
namespace ovsim
{
namespace servers
{
namespace grpc
{
namespace details
{

/**
 * @brief Parse the server address, honoring the last address option or printing help.
 * @param[in] argc Number of command-line arguments, including the program name.
 * @param[in] argv Command-line arguments, including the program name.
 * @return The listen address, or no value when help was requested.
 * @throws std::invalid_argument If an option is invalid or the address is missing.
 */
inline std::optional<std::string> parseListenAddress(int argc, const char* const* argv)
{
    std::optional<std::string> listenAddress;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument(argv[index]);
        if (argument == "--listen-address" && index + 1 < argc)
        {
            listenAddress = argv[++index];
            continue;
        }
        const std::string prefix = "--listen-address=";
        if (argument.rfind(prefix, 0) == 0)
        {
            listenAddress = argument.substr(prefix.size());
            continue;
        }
        if (argument == "--help" || argument == "-h")
        {
            std::cout << "Usage: isaacsim-ovsim-grpc-server --listen-address HOST:PORT\n";
            return std::nullopt;
        }
        throw std::invalid_argument("Unknown or incomplete command-line argument: " + argument);
    }
    if (!listenAddress || listenAddress->empty())
    {
        throw std::invalid_argument("--listen-address HOST:PORT is required.");
    }
    return listenAddress;
}

} // namespace details
} // namespace grpc
} // namespace servers
} // namespace ovsim
} // namespace isaacsim
