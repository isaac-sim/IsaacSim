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

#include "CommandLine.hpp"

#include <isaacsim/ovsim/servers/grpc/Server.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>

#if !defined(_WIN32)
#    include <csignal>
#    include <pthread.h>
#endif

int main(int argc, char** argv)
{
    try
    {
        const auto listenAddress = isaacsim::ovsim::servers::grpc::details::parseListenAddress(argc, argv);
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
