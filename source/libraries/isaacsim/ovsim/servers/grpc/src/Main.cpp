// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <isaacsim/ovsim/servers/grpc/Server.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#if !defined(_WIN32)
#    include <csignal>
#    include <pthread.h>
#endif

namespace
{

std::optional<std::string> parseListenAddress(int argc, char** argv)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument(argv[index]);
        if (argument == "--listen-address" && index + 1 < argc)
        {
            return argv[++index];
        }
        const std::string prefix = "--listen-address=";
        if (argument.rfind(prefix, 0) == 0)
        {
            return argument.substr(prefix.size());
        }
        if (argument == "--help" || argument == "-h")
        {
            std::cout << "Usage: isaacsim-ovsim-grpc-server --listen-address HOST:PORT\n";
            return std::nullopt;
        }
        throw std::invalid_argument("Unknown or incomplete command-line argument: " + argument);
    }
    throw std::invalid_argument("--listen-address HOST:PORT is required.");
}

} // namespace

int main(int argc, char** argv)
{
    try
    {
        const auto listenAddress = parseListenAddress(argc, argv);
        if (!listenAddress)
        {
            return 0;
        }

#if !defined(_WIN32)
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0)
        {
            throw std::runtime_error("Could not install the server shutdown signal mask.");
        }
#endif

        isaacsim::ovsim::servers::grpc::Server server(*listenAddress);
        std::cout << "OV SIM gRPC server listening on " << server.getEndpoint() << std::endl;

#if !defined(_WIN32)
        int signal = 0;
        if (sigwait(&signals, &signal) != 0)
        {
            throw std::runtime_error("Could not wait for a server shutdown signal.");
        }
        server.shutdown();
#else
        server.wait();
#endif
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "OV SIM gRPC server failed: " << error.what() << std::endl;
        // Some loaded simulation runtimes install process-exit cleanup that can replace a returned nonzero status.
        // Stack unwinding has already destroyed local server state here, so terminate with the required failure code.
        std::_Exit(EXIT_FAILURE);
    }
}
