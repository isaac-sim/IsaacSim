// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "UcxUtils.h"

#include <ucxx/api.h>
#include <ucxx/utils/sockaddr.h>
#include <ucxx/utils/ucx.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace isaacsim::ucx::core
{

/**
 * @class UCXListener
 * @brief Server-side listener for incoming UCX connections.
 * @details
 * Manages a listening socket on a specified port and handles incoming connection
 * requests from clients. Provides endpoint creation for accepted connections.
 * The listener operates asynchronously and can wait for client connections with
 * configurable timeouts.
 *
 * @note The listener automatically accepts incoming client connections as endpoints.
 */
class UCXListener
{
public:
    /**
     * @brief Constructor for UCX listener.
     * @details
     * Creates a new UCX listener that will bind to the specified port and wait
     * for incoming client connections. The listener uses the provided context
     * to create workers and manage connections.
     *
     * @param[in] context UCX context for creating workers and managing connections
     * @param[in] port Port number to listen on for incoming connections
     *
     * @throws std::runtime_error If listener creation or port binding fails
     */
    UCXListener(std::shared_ptr<ucxx::Context> context, uint16_t port);

    /**
     * @brief Destructor - ensures proper cleanup of listener resources.
     * @details
     * Automatically shuts down the listener and cleans up all resources.
     * Any active connections will be properly closed.
     */
    ~UCXListener();

    /**
     * @brief Wait for a connection with optional timeout.
     * @details
     * Blocks until a connection is established or the timeout expires. When a connection is established,
     * a new UCXConnection object is created and stored internally.
     *
     * @param[in] timeoutMs Timeout in milliseconds (-1 for infinite timeout)
     *
     * @return true if a connection was established within the timeout, false if timeout expired
     */
    bool waitForConnection(int timeoutMs = -1);

    /**
     * @brief Check if the listener currently has an active client connection.
     * @details
     * Determines whether an endpoint representing a client connection exists
     * and is currently active. This can be used to verify if a client is connected to the server.
     *
     * @return true if a client connection exists and is active; false otherwise.
     */
    bool isConnected() const;

    /**
     * @brief Get the port number the listener is bound to.
     * @details
     * Returns the actual port number that the listener is using. This may differ
     * from the requested port if port 0 was specified (automatic port selection).
     *
     * @return Port number the listener is bound to
     */
    inline uint16_t getPort() const
    {
        if (!m_listener)
        {
            throw std::runtime_error("UCXListener: listener not initialized");
        }
        return m_listener->getPort();
    }

    /**
     * @brief Get the UCX worker used by this listener.
     * @details
     * Returns the worker that manages the listener's operations and progress thread.
     * This can be used to create client endpoints on the same worker for testing
     * or for applications that need to share a worker between listener and client.
     *
     * @return Shared pointer to the UCX worker
     */
    inline std::shared_ptr<ucxx::Worker> getWorker() const
    {
        return m_worker;
    }

    /**
     * @brief Start the progress thread for the worker.
     * @details
     * Starts the background progress thread that handles UCX communication operations.
     * This should be called after all endpoints are created to follow the official UCXX
     * pattern and avoid race conditions during endpoint creation.
     *
     * @note This function is idempotent - calling it multiple times has no effect if
     *       the progress thread is already running.
     */
    void startProgressThread();

    /**
     * @brief Stop the progress thread for the worker.
     * @details
     * Stops the background progress thread that handles UCX communication operations.
     * This should be called before destroying the listener to ensure proper cleanup.
     */
    void stopProgressThread();

    /**
     * @brief Shutdown the listener and close all connections.
     * @details
     * Stops the listener from accepting new connections and closes any existing
     * client connections. This method should be called before destroying the
     * listener to ensure proper cleanup.
     *
     * @note After shutdown, the listener cannot be restarted
     */
    void shutdown();

    // Tag communication methods - forward to the connection
    /**
     * @brief Send data using tagged communication with optional timeout.
     * @details
     * Sends message data using UCX tagged send.
     * - If timeout is not specified (std::nullopt), returns immediately without waiting (async).
     * - If timeout is g_kUcxInfiniteTimeout (UINT32_MAX), waits indefinitely until completion or failure.
     * - Otherwise, waits up to the specified timeout in milliseconds.
     *
     * @param[in] buffer Memory address of the data buffer to send
     * @param[in] length Size of the data buffer in bytes
     * @param[in] tag UCX tag for message identification
     * @param[out] errorMessage String for storing error messages
     * @param[in] timeout Optional timeout in milliseconds (nullopt = async, g_kUcxInfiniteTimeout = infinite wait)
     * @return UcxSendResult indicating the outcome (eSuccess, eEmptyMessage, eNullRequest, eTimedOut, eFailed, or
     * eException)
     */
    UcxSendResult tagSend(const void* buffer,
                          size_t length,
                          uint64_t tag,
                          std::string& errorMessage,
                          std::optional<uint32_t> timeout = std::nullopt);

    /**
     * @brief Send data using tagged communication and return request handle for async completion tracking.
     * @details
     * Sends message data using UCX tagged send without waiting (async).
     * Returns the request handle so caller can check completion before reusing buffer.
     * Intended for large message sends where buffer lifetime management is critical.
     *
     * @param[in] buffer Memory address of the data buffer to send
     * @param[in] length Size of the data buffer in bytes
     * @param[in] tag UCX tag for message identification
     * @param[out] errorMessage String for storing error messages
     * @param[out] outRequest Reference to receive the request handle
     * @return UcxSendResult indicating the outcome (eSuccess if initiated, error otherwise)
     */
    UcxSendResult tagSendWithRequest(const void* buffer,
                                     size_t length,
                                     uint64_t tag,
                                     std::string& errorMessage,
                                     std::shared_ptr<ucxx::Request>& outRequest);

    /**
     * @brief Send multiple buffers using tagged communication with optional timeout.
     * @details
     * Sends multiple buffers using UCX tagged multi-send.
     * - If timeout is not specified (std::nullopt), returns immediately without waiting (async).
     * - If timeout is g_kUcxInfiniteTimeout (UINT32_MAX), waits indefinitely until completion or failure.
     * - Otherwise, waits up to the specified timeout in milliseconds.
     *
     * @param[in] buffer List of memory addresses pointing to data buffers
     * @param[in] size List of buffer sizes in bytes
     * @param[in] isCuda List indicating if each buffer is CUDA memory
     * @param[in] tag UCX tag for message identification
     * @param[out] errorMessage String for storing error messages
     * @param[in] timeout Optional timeout in milliseconds (nullopt = async, g_kUcxInfiniteTimeout = infinite wait)
     * @return UcxSendResult indicating the outcome
     */
    UcxSendResult tagMultiSend(const std::vector<const void*>& buffer,
                               const std::vector<size_t>& size,
                               const std::vector<int>& isCuda,
                               const uint64_t tag,
                               std::string& errorMessage,
                               std::optional<uint32_t> timeout = std::nullopt);

    /**
     * @brief Receive data using tagged communication with optional timeout.
     * @details
     * Receives message data using UCX tagged receive and waits for completion.
     * Defaults to infinite wait if timeout is not specified.
     *
     * @param[in] buffer Memory address where received data will be stored
     * @param[in] length Size of the receive buffer in bytes
     * @param[in] tag UCX tag for message identification
     * @param[in] mask Tag mask for selective message matching
     * @param[out] errorMessage String for storing error messages
     * @param[in] timeout Timeout in milliseconds (g_kUcxInfiniteTimeout = infinite wait)
     * @return UcxReceiveResult indicating the outcome
     */
    UcxReceiveResult tagReceive(void* buffer,
                                size_t length,
                                uint64_t tag,
                                uint64_t mask,
                                std::string& errorMessage,
                                uint32_t timeout = g_kUcxInfiniteTimeout);

    /**
     * @brief Receive multiple buffers using tagged communication with optional timeout.
     * @details
     * Receives multiple buffers using UCX tagged multi-receive and waits for completion.
     * Defaults to infinite wait if timeout is not specified.
     *
     * @param[in] tag UCX tag for message identification
     * @param[in] tagMask Tag mask for selective message matching
     * @param[out] errorMessage String for storing error messages
     * @param[in] timeout Timeout in milliseconds (g_kUcxInfiniteTimeout = infinite wait)
     * @return UcxReceiveResult indicating the outcome
     */
    UcxReceiveResult tagMultiReceive(const uint64_t tag,
                                     const uint64_t tagMask,
                                     std::string& errorMessage,
                                     uint32_t timeout = g_kUcxInfiniteTimeout);

    // TODO(rhua): handle disconnects and reconnects

private:
    /** @brief UCX context for creating workers and managing connections. */
    std::shared_ptr<ucxx::Context> m_context;

    /** @brief Port number the listener is bound to. */
    uint16_t m_port;

    /** @brief Mutex for protecting access to the endpoint object. */
    mutable std::mutex m_endpointMutex;

    /** @brief Current client endpoint, if any. */
    std::shared_ptr<ucxx::Endpoint> m_endpoint;

    /** @brief UCX worker for handling listener operations. */
    std::shared_ptr<ucxx::Worker> m_worker;

    /** @brief UCX listener for accepting incoming connections. */
    std::shared_ptr<ucxx::Listener> m_listener;

    /** @brief Condition variable for waiting for a connection. */
    std::condition_variable m_connectionCondition;

    /** @brief Flag indicating if the listener has been shut down. */
    std::atomic<bool> m_shutdown;

    /**
     * @brief Static callback function for handling incoming connection requests.
     * @details
     * This callback is invoked by UCX when a client attempts to connect.
     * It handles the connection acceptance process.
     *
     * @param[in] connRequest UCX connection request handle
     * @param[in] arg User-defined argument (pointer to UCXListener instance)
     */
    static void onConnectionRequest(ucp_conn_request_h connRequest, void* arg);

    /**
     * @brief Initialize the listener and worker.
     * @details
     * Sets up the UCX worker and listener for accepting connections.
     */
    void initialize();

    /**
     * @brief Create a connection from an incoming connection request.
     * @details
     * Accepts a connection request and creates a UCXConnection object.
     *
     * @param[in] connRequest UCX connection request handle to accept
     */
    void createConnection(ucp_conn_request_h connRequest);

    /**
     * @brief Handle endpoint closure events.
     * @details
     * Callback invoked when the remote endpoint is closed.
     *
     * @param[in] status Status code indicating the closure reason
     */
    void onEndpointClosed(ucs_status_t status);
};

} // namespace isaacsim::ucx::core
