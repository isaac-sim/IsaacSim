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

#include "TcpIo.h"

#include <cstdint>
#include <string>

namespace isaacsim
{
namespace examples
{
namespace ipc
{

class TcpClockClient
{
public:
    explicit TcpClockClient(std::string uri) : m_uri(std::move(uri)), m_socketFd(kInvalidSocket)
    {
    }

    ~TcpClockClient()
    {
        disconnect();
    }

    bool connect()
    {
        disconnect();
        std::string host;
        uint16_t port = 0;
        if (!tcp::parseHostPort(m_uri, host, port))
        {
            return false;
        }
        m_socketFd = tcp::connectTcpClient(host, port);
        return m_socketFd != kInvalidSocket;
    }

    void disconnect()
    {
        tcp::closeFd(m_socketFd);
    }

    bool isConnected() const
    {
        return m_socketFd != kInvalidSocket;
    }

    bool sendClock(int64_t timeNanoseconds)
    {
        if (m_socketFd == kInvalidSocket)
        {
            return false;
        }
        uint8_t wire[8];
        tcp::writeLe64(wire, timeNanoseconds);
        return tcp::sendAll(m_socketFd, wire, sizeof(wire));
    }

    const std::string& getUri() const
    {
        return m_uri;
    }

private:
    std::string m_uri;
    socket_t m_socketFd;
};

} // namespace ipc
} // namespace examples
} // namespace isaacsim
