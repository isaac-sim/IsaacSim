// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <isaacsim/ovsim/servers/grpc/Export.h>

#include <memory>
#include <string>

namespace isaacsim
{
namespace ovsim
{
namespace servers
{
namespace grpc
{

/** @brief Synchronous OV SIM gRPC service host backed by the local OV SIM implementation. */
class ISAACSIM_OVSIM_SERVERS_GRPC_API Server
{
public:
    /** @brief Construct and start an insecure server on the supplied gRPC listen address.
     * @param[in] listenAddress gRPC listen address. A terminal port of zero requests an available port.
     * @throws std::invalid_argument If `listenAddress` is empty.
     * @throws std::runtime_error If the physics backend cannot be activated or the server cannot bind.
     */
    explicit Server(std::string listenAddress);

    /** @brief Destroy the server, stopping it first if needed. */
    ~Server();

    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;
    Server(Server&&) = delete;
    Server& operator=(Server&&) = delete;

    /** @brief Get the bound endpoint.
     * @return Bound endpoint. A requested port of zero is replaced with the selected port.
     * @note The returned reference remains valid until this server is destroyed.
     */
    const std::string& getEndpoint() const;

    /** @brief Block until shutdown completes. */
    void wait();

    /**
     * @brief Stop accepting RPCs and clean up the hosted simulation.
     *
     * This operation is safe to call repeatedly.
     *
     * @throws std::runtime_error If simulation or backend cleanup does not complete.
     */
    void shutdown();

private:
    class Implementation;
    std::unique_ptr<Implementation> m_implementation;
};

} // namespace grpc
} // namespace servers
} // namespace ovsim
} // namespace isaacsim
