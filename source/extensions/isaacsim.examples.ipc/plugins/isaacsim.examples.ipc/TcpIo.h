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

// Platform socket abstraction.
// Kit initialises Winsock2 (WSAStartup) before loading extensions, so plugins
// do not need to call it themselves.
#ifdef _WIN32
#    ifndef WIN32_LEAN_AND_MEAN
#        define WIN32_LEAN_AND_MEAN
#    endif
#    include <winsock2.h>
#    include <ws2tcpip.h>
// socket handle type — unsigned UINT_PTR on Windows, int on POSIX
using socket_t = SOCKET;
inline const socket_t kInvalidSocket = INVALID_SOCKET;
#    ifndef MSG_NOSIGNAL
#        define MSG_NOSIGNAL 0 // no SIGPIPE on Windows; silently map send flag to 0
#    endif
#else
#    include <arpa/inet.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <sys/types.h>

#    include <fcntl.h>
#    include <unistd.h>
using socket_t = int;
inline constexpr socket_t kInvalidSocket = -1;
#endif

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace isaacsim
{
namespace examples
{
namespace ipc
{
namespace tcp
{

inline void writeLe64(uint8_t* out, int64_t value)
{
    for (int i = 0; i < 8; ++i)
    {
        out[i] = static_cast<uint8_t>((static_cast<uint64_t>(value) >> (8 * i)) & 0xFFu);
    }
}

inline int64_t readLe64(const uint8_t* data)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
    {
        v |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return static_cast<int64_t>(v);
}

inline void writeLe32(uint8_t* out, uint32_t value)
{
    for (int i = 0; i < 4; ++i)
    {
        out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFFu);
    }
}

inline uint32_t readLe32(const uint8_t* data)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
    {
        v |= static_cast<uint32_t>(data[i]) << (8 * i);
    }
    return v;
}

inline bool parseHostPort(const std::string& uri, std::string& outHost, uint16_t& outPort)
{
    const size_t colon = uri.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= uri.size())
    {
        return false;
    }
    outHost = uri.substr(0, colon);
    try
    {
        const unsigned long p = std::stoul(uri.substr(colon + 1));
        if (p > 65535u)
        {
            return false;
        }
        outPort = static_cast<uint16_t>(p);
    }
    catch (...)
    {
        return false;
    }
    return true;
}

// Returns true when the last socket operation failed because no data was ready
// (non-blocking would-block condition). Abstracts errno/WSAGetLastError.
inline bool wouldBlock()
{
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

inline bool setNonBlocking(socket_t fd)
{
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(fd, FIONBIO, &mode) == 0;
#else
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
    {
        return false;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

inline void closeSocket(socket_t fd)
{
#ifdef _WIN32
    if (fd != INVALID_SOCKET)
    {
        closesocket(fd);
    }
#else
    if (fd >= 0)
    {
        ::close(fd);
    }
#endif
}

inline void closeFd(socket_t& fd)
{
#ifdef _WIN32
    if (fd != INVALID_SOCKET)
    {
        closesocket(fd);
        fd = INVALID_SOCKET;
    }
#else
    if (fd >= 0)
    {
        ::close(fd);
        fd = kInvalidSocket;
    }
#endif
}

inline bool sendAll(socket_t fd, const void* data, size_t length)
{
    const char* bytes = static_cast<const char*>(data);
    size_t sent = 0;
    while (sent < length)
    {
        // ::send() returns int on Windows and ssize_t on POSIX; int covers both
        // since a single send never exceeds INT_MAX bytes.
        const int n = ::send(fd, bytes + sent, static_cast<int>(length - sent), MSG_NOSIGNAL);
        if (n < 0)
        {
#ifndef _WIN32
            if (errno == EINTR)
            {
                continue;
            }
#endif
            return false;
        }
        if (n == 0)
        {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

inline socket_t connectTcpClient(const std::string& host, uint16_t port)
{
    const socket_t fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd == kInvalidSocket)
    {
        return kInvalidSocket;
    }

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }
    return fd;
}

inline socket_t listenTcpServer(const std::string& bindHost, uint16_t port)
{
    const socket_t fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd == kInvalidSocket)
    {
        return kInvalidSocket;
    }

    int one = 1;
    // setsockopt takes const char* on Windows and const void* on POSIX; const char* is accepted by both.
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one)) != 0)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, bindHost.c_str(), &addr.sin_addr) != 1)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    if (::listen(fd, 1) != 0)
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    if (!setNonBlocking(fd))
    {
        closeSocket(fd);
        return kInvalidSocket;
    }

    return fd;
}

} // namespace tcp
} // namespace ipc
} // namespace examples
} // namespace isaacsim
