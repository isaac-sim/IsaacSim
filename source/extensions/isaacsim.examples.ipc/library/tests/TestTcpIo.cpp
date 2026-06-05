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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <TcpIo.h>

using namespace isaacsim::examples::ipc::tcp;

TEST_CASE("parseHostPort accepts ipv4 host:port")
{
    std::string host;
    uint16_t port = 0;
    REQUIRE(parseHostPort("127.0.0.1:9000", host, port));
    CHECK(host == "127.0.0.1");
    CHECK(port == 9000);
}

TEST_CASE("parseHostPort rejects invalid uri")
{
    std::string host;
    uint16_t port = 0;
    CHECK(!parseHostPort("nocolon", host, port));
    CHECK(!parseHostPort(":1234", host, port));
    CHECK(!parseHostPort("host:", host, port));
}

TEST_CASE("writeLe64 and readLe64 roundtrip")
{
    uint8_t buf[8];
    const int64_t v = -123456789012LL;
    writeLe64(buf, v);
    CHECK(readLe64(buf) == v);
}

TEST_CASE("writeLe32 and readLe32 roundtrip")
{
    uint8_t buf[4];
    const uint32_t v = 4294967290u;
    writeLe32(buf, v);
    CHECK(readLe32(buf) == v);
}
