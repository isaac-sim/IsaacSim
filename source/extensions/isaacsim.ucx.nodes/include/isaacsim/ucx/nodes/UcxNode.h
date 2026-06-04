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

#include "isaacsim/core/includes/BaseResetNode.h"

#include <carb/Defines.h>
#include <carb/Types.h>

#include <isaacsim/ucx/core/UcxListener.h>
#include <isaacsim/ucx/core/UcxListenerRegistry.h>
#include <isaacsim/ucx/core/UcxUtils.h>

#include <memory>
#include <string>
#include <vector>

namespace isaacsim::ucx::nodes
{

/**
 * @class UcxNode
 * @brief Base class for UCX omnigraph nodes.
 * @details
 * This class provides the foundation for UCX nodes in Isaac Sim.
 * It handles listener management and provides common functionality for
 * UCX-based communication nodes.
 */
class UcxNode : public isaacsim::core::includes::BaseResetNode
{
public:
    /**
     * @brief Constructor for the UCX node.
     * @details
     * Initializes the node with default settings.
     */
    UcxNode() = default;

    /**
     * @brief Destructor.
     * @details
     * Ensures proper cleanup by calling reset().
     */
    virtual ~UcxNode()
    {
        reset();
    }

    /**
     * @brief Resets the node state.
     * @details
     * Cleans up the listener reference. Derived classes should call this method
     * after cleaning up their own resources.
     *
     * @note This is a virtual method that can be overridden by derived classes.
     */
    virtual void reset() override
    {
        if (m_listener)
        {
            const uint16_t port = m_listener->getPort();
            m_listener.reset();
            isaacsim::ucx::core::UCXListenerRegistry::tryRemoveListener(port);
        }
    }

protected:
    /**
     * @brief Validates port number is in acceptable range for OmniGraph nodes.
     * @details
     * Ports 0-1023 are system reserved and should not be used by OmniGraph nodes.
     * Port 0 (auto-assign) is also not allowed to ensure deterministic behavior.
     *
     * @param[in] port Port number to validate
     * @return bool True if port is valid (>= 1024), false otherwise
     */
    static bool isValidPort(uint16_t port)
    {
        return port >= 1024;
    }

    /**
     * @brief Validates UCX tag value.
     * @details
     * UCX uses the full 64-bit tag space. This function can be extended
     * to restrict tag ranges if needed for application-specific protocols.
     *
     * @param[in] tag Tag value to validate
     * @return bool True if tag is valid, false otherwise
     */
    static bool isValidTag(uint64_t tag)
    {
        // Currently all tag values are valid
        // Future: could restrict reserved ranges if needed
        return true;
    }

    /**
     * @brief Gets or creates a listener for the specified port.
     * @details
     * Returns an existing listener if one is already registered for the port,
     * otherwise creates a new listener on the specified port.
     *
     * The port is validated before attempting to create a listener. Invalid
     * ports (0-1023) are rejected.
     *
     * @param[in] port Port number for the listener (must be >= 1024)
     * @return bool True if listener was successfully obtained or created, false otherwise.
     */
    bool initializeListener(uint16_t port)
    {
        if (!isValidPort(port))
        {
            return false;
        }

        if (m_listener && m_listener->getPort() == port)
        {
            return true;
        }

        m_listener = isaacsim::ucx::core::UCXListenerRegistry::addListener(port);
        return m_listener != nullptr;
    }

    /**
     * @brief Checks if the node has a valid listener.
     * @details
     * Verifies if the listener has been properly created and initialized.
     *
     * @return bool True if the listener exists, false otherwise.
     */
    bool isInitialized() const
    {
        return m_listener != nullptr;
    }

    /**
     * @brief Ensures the listener is initialized and started.
     * @details
     * Initializes the listener if not already done or if the port has changed,
     * and starts the progress thread. Validates the port number before initialization.
     *
     * @tparam DatabaseT The database type for logging
     * @param[in] db Database accessor for logging
     * @param[in] port Port number for the listener (must be >= 1024)
     * @return bool True if listener is ready, false on error
     */
    template <typename DatabaseT>
    bool ensureListenerReady(DatabaseT& db, uint16_t port)
    {
        if (!isValidPort(port))
        {
            db.logError("Invalid port %u. Port must be >= 1024 to avoid system reserved ports.", port);
            return false;
        }

        if (!isInitialized() || (m_listener && m_listener->getPort() != port))
        {
            try
            {
                if (!initializeListener(port))
                {
                    db.logError(
                        "Failed to create or get UCX listener on port %u. Port may be in use or unavailable.", port);
                    return false;
                }
            }
            catch (const std::exception& e)
            {
                db.logError("Exception during listener initialization: %s", e.what());
                return false;
            }

            m_listener->startProgressThread();
        }

        return true;
    }

    /**
     * @brief Waits for a client connection.
     * @details
     * Checks if a connection exists, and if not, performs a non-blocking wait.
     * Returns true if connected and ready to communicate, false if not yet connected.
     *
     * @return bool True if connected, false if no connection available
     */
    bool waitForConnection()
    {
        bool isConnected = m_listener->isConnected();

        if (!isConnected)
        {
            if (!m_listener->waitForConnection(0))
            {
                return false;
            }
            isConnected = m_listener->isConnected();
        }

        return true;
    }

    /**
     * @brief Publishes a message over UCX with validation and timeout.
     * @details
     * Validates that message data is not empty, then sends using UCX tagged send.
     * Wrapper for UCXListener::tagSend() that adds logging for OmniGraph nodes.
     *
     * @tparam DatabaseT The database type for logging
     * @param[in] db Database accessor for logging
     * @param[in] messageData Serialized message data to send
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for send request (0 = infinite)
     * @return bool True if send completed successfully, false on error or timeout
     */
    template <typename DatabaseT>
    bool publishMessage(DatabaseT& db, const std::vector<uint8_t>& messageData, uint64_t tag, uint32_t timeoutMs)
    {
        if (messageData.empty())
        {
            db.logError("Failed to generate message");
            return false;
        }

        std::string errorMessage;
        auto result = m_listener->tagSend(messageData.data(), messageData.size(), tag, errorMessage, timeoutMs);

        switch (result)
        {
        case isaacsim::ucx::core::UcxSendResult::eSuccess:
            return true;
        case isaacsim::ucx::core::UcxSendResult::eEmptyMessage:
            db.logError("Cannot send empty message: %s", errorMessage.c_str());
            return false;
        case isaacsim::ucx::core::UcxSendResult::eNullRequest:
            db.logError("Failed to send message - tagSend returned null: %s", errorMessage.c_str());
            return false;
        case isaacsim::ucx::core::UcxSendResult::eTimedOut:
            db.logError("Send request timed out after %u ms: %s", timeoutMs, errorMessage.c_str());
            return false;
        case isaacsim::ucx::core::UcxSendResult::eFailed:
            db.logError("Send request failed with UCX error: %s", errorMessage.c_str());
            return false;
        case isaacsim::ucx::core::UcxSendResult::eException:
            db.logError("Exception during message send: %s", errorMessage.c_str());
            return false;
        default:
            db.logError("Unexpected send result");
            return false;
        }
    }

    std::shared_ptr<isaacsim::ucx::core::UCXListener> m_listener = nullptr; //!< UCX listener instance.
};

} // namespace isaacsim::ucx::nodes
