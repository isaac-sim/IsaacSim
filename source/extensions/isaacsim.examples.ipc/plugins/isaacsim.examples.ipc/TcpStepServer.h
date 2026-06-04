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
#include <cstring>
#include <optional>
#include <string>

namespace isaacsim
{
namespace examples
{
namespace ipc
{

class TcpStepServer
{
public:
    explicit TcpStepServer(std::string uri)
        : m_uri(std::move(uri)), m_listenFd(kInvalidSocket), m_clientFd(kInvalidSocket), m_filled(0)
    {
        std::memset(m_buffer, 0, sizeof(m_buffer));
    }

    ~TcpStepServer()
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
        m_listenFd = tcp::listenTcpServer(host, port);
        return m_listenFd != kInvalidSocket;
    }

    void disconnect()
    {
        tcp::closeFd(m_clientFd);
        tcp::closeFd(m_listenFd);
        m_filled = 0;
    }

    bool isConnected() const
    {
        return m_listenFd != kInvalidSocket;
    }

    std::optional<uint32_t> tryReceiveStep()
    {
        if (m_listenFd == kInvalidSocket)
        {
            return std::nullopt;
        }

        if (m_clientFd == kInvalidSocket)
        {
            const socket_t c = ::accept(m_listenFd, nullptr, nullptr);
            if (c == kInvalidSocket)
            {
                return std::nullopt;
            }
            if (!tcp::setNonBlocking(c))
            {
                tcp::closeSocket(c);
                return std::nullopt;
            }
            m_clientFd = c;
            m_filled = 0;
        }

        while (m_filled < 4)
        {
            // ::recv() takes char* on Windows and void* on POSIX; char* is accepted by both.
            // Length is int on Windows and size_t on POSIX; int covers both since the request never exceeds 4 bytes.
            const int n =
                ::recv(m_clientFd, reinterpret_cast<char*>(m_buffer + m_filled), static_cast<int>(4 - m_filled), 0);
            if (n > 0)
            {
                m_filled += static_cast<size_t>(n);
                continue;
            }
            if (n == 0)
            {
                tcp::closeFd(m_clientFd);
                m_filled = 0;
                return std::nullopt;
            }
#ifndef _WIN32
            if (errno == EINTR)
            {
                continue;
            }
#endif
            if (tcp::wouldBlock())
            {
                return std::nullopt;
            }
            tcp::closeFd(m_clientFd);
            m_filled = 0;
            return std::nullopt;
        }

        const uint32_t step = tcp::readLe32(m_buffer);
        m_filled = 0;
        return step;
    }

    const std::string& getUri() const
    {
        return m_uri;
    }

private:
    std::string m_uri;
    socket_t m_listenFd;
    socket_t m_clientFd;
    uint8_t m_buffer[4];
    size_t m_filled;
};

} // namespace ipc
} // namespace examples
} // namespace isaacsim
