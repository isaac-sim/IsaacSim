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

#include <doctest/doctest.h>

#include <CommandLine.hpp>
#include <iterator>

namespace
{

using isaacsim::ovsim::servers::grpc::details::parseListenAddress;

TEST_SUITE("ovsim.grpc.CommandLine")
{
    TEST_CASE("The last listen address overrides defaults")
    {
        const char* arguments[] = { "server",           "--listen-address",
                                    "127.0.0.1:50051",  "--listen-address=127.0.0.1:50052",
                                    "--listen-address", "127.0.0.1:50053" };
        CHECK(parseListenAddress(static_cast<int>(std::size(arguments)), arguments) == "127.0.0.1:50053");
    }

    TEST_CASE("Help after the default address does not launch the server")
    {
        const char* arguments[] = { "server", "--listen-address", "127.0.0.1:50051", "--help" };
        CHECK_FALSE(parseListenAddress(static_cast<int>(std::size(arguments)), arguments).has_value());
    }

    TEST_CASE("A listen address is required")
    {
        const char* arguments[] = { "server" };
        CHECK_THROWS_AS(parseListenAddress(static_cast<int>(std::size(arguments)), arguments), std::invalid_argument);
    }

    TEST_CASE("Trailing unknown and incomplete options are rejected")
    {
        const char* unknown[] = { "server", "--listen-address", "127.0.0.1:50051", "--unknown" };
        CHECK_THROWS_AS(parseListenAddress(static_cast<int>(std::size(unknown)), unknown), std::invalid_argument);
        const char* incomplete[] = { "server", "--listen-address", "127.0.0.1:50051", "--listen-address" };
        CHECK_THROWS_AS(parseListenAddress(static_cast<int>(std::size(incomplete)), incomplete), std::invalid_argument);
    }
}

} // namespace
