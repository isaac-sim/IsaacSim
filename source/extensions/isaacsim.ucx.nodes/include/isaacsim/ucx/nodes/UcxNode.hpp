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

#include "isaacsim/core/includes/BaseResetNode.hpp"

#include <carb/Defines.h>
#include <carb/Types.h>

#include <isaacsim/ucx/core/UcxListener.hpp>
#include <isaacsim/ucx/core/UcxListenerRegistry.hpp>
#include <isaacsim/ucx/core/UcxUtils.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace isaacsim::ucx::nodes
{

/**
 * @enum UcxWarningCategory
 * @brief Categories of warning a UCX node latches independently.
 * @details
 * Each category is reported once, so a condition already being reported cannot hide an
 * unrelated one that starts later.
 */
enum class UcxWarningCategory : size_t
{
    eUnboundedSendTimeout = 0, //!< An unbounded send timeout was clamped to a finite wait
    ePostFailed, //!< A request could not be posted
    eRequestFailed, //!< A posted request completed with an error
    eDeadlineExpired, //!< A posted request was cancelled because its deadline passed
    eCount //!< Number of categories; not a category itself
};

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
        _clearWarningLatches();
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
     * @brief Emits a warning the first time its category reports a problem.
     * @details
     * The category stays latched until _clearWarningLatch() or _clearWarningLatches() re-arms it.
     *
     * @tparam DatabaseT The database type for logging
     * @tparam Args Format argument types
     * @param[in,out] db Database accessor for logging
     * @param[in] category Warning category to latch
     * @param[in] format printf-style format string
     * @param[in] args Format arguments
     */
    template <typename DatabaseT, typename... Args>
    void _logWarningOnce(DatabaseT& db, UcxWarningCategory category, const char* format, Args&&... args)
    {
        bool& latched = m_warningLatched[static_cast<size_t>(category)];
        if (latched)
        {
            return;
        }

        latched = true;
        db.logWarning(format, std::forward<Args>(args)...);
    }

    /**
     * @brief Re-arms a single warning category.
     *
     * @param[in] category Warning category to clear
     */
    void _clearWarningLatch(UcxWarningCategory category)
    {
        m_warningLatched[static_cast<size_t>(category)] = false;
    }

    /** @brief Re-arms every warning category. */
    void _clearWarningLatches()
    {
        m_warningLatched.fill(false);
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
     * @param[in] timeoutMs Timeout in milliseconds for send request (0 is clamped to
     *                      s_kDefaultSendTimeoutMs, see below)
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

        // compute() runs on the application update thread, so a send that never gives up freezes
        // the application as soon as a peer stops draining its receive queue.
        uint32_t effectiveTimeoutMs = timeoutMs;
        if (effectiveTimeoutMs == 0)
        {
            effectiveTimeoutMs = s_kDefaultSendTimeoutMs;
            _logWarningOnce(db, UcxWarningCategory::eUnboundedSendTimeout,
                            "inputs:timeoutMs is 0, which would block the application update thread until the peer "
                            "consumes the message. Waiting %u ms instead; set inputs:timeoutMs to a positive value to "
                            "choose the bound explicitly.",
                            effectiveTimeoutMs);
        }

        std::string errorMessage;
        auto result = m_listener->tagSend(messageData.data(), messageData.size(), tag, errorMessage, effectiveTimeoutMs);

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
            db.logError("Send request timed out after %u ms: %s", effectiveTimeoutMs, errorMessage.c_str());
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

    /** @brief Send timeout substituted when a node is configured with an unbounded (0) timeout. */
    static constexpr uint32_t s_kDefaultSendTimeoutMs = 100;

    std::shared_ptr<isaacsim::ucx::core::UCXListener> m_listener = nullptr; //!< UCX listener instance.
    //! One latch per UcxWarningCategory; true once that category has been reported.
    std::array<bool, static_cast<size_t>(UcxWarningCategory::eCount)> m_warningLatched{};
};

} // namespace isaacsim::ucx::nodes
