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

#include <isaacsim/ucx/core/UcxListener.h>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace isaacsim::ucx::core
{

/**
 * @class UCXListenerRegistry
 * @brief Singleton registry for managing UCX listeners by port.
 * @details
 * Provides centralized management of UCX listeners across the application.
 * Ensures only one listener per port and handles thread-safe access to listeners.
 *
 * @note This is a singleton class that cannot be instantiated.
 */
class UCXListenerRegistry
{
public:
    /**
     * @brief Add a listener to the registry.
     * @details
     * When port is 0 (default), always creates a new listener on an automatically
     * assigned ephemeral port. The caller can use listener->getPort() to retrieve
     * the actual port number.
     *
     * When port is non-zero, returns an existing listener if one is already
     * registered for the port, otherwise creates a new listener on the specified port.
     *
     * @param[in] port Port number for the listener (0 for ephemeral port, default = 0)
     *
     * @return Shared pointer to the listener
     *
     * @throws std::runtime_error If listener creation fails
     *
     * @note When port is 0, this always creates a new listener, even if other listeners exist
     * @note When port is non-zero, this returns the existing listener if one is registered
     */
    static std::shared_ptr<UCXListener> addListener(uint16_t port = 0);

    /**
     * @brief Remove and shutdown listener for specified port.
     * @details
     * Removes the listener associated with the specified port from the registry.
     * The listener will be shut down if it's the last reference.
     *
     * @param[in] port Port number of listener to remove
     */
    static void removeListener(uint16_t port);

    /**
     * @brief Check if a listener is registered for the specified port.
     * @details
     * Returns true if a listener is currently registered for the given port.
     *
     * @param[in] port Port number to check
     *
     * @return true if listener exists for port, false otherwise
     */
    static bool isListenerRegistered(uint16_t port);

    /**
     * @brief Shutdown all registered listeners and clear registry.
     * @details
     * Shuts down all listeners in the registry and clears the registry.
     * This method should be called during application shutdown.
     */
    static void shutdown();

    /**
     * @brief Remove listener only if no other references exist.
     * @details
     * Removes and shuts down the listener if the registry holds the only reference
     * (use_count == 1). If other shared_ptr holders exist, the listener remains active.
     *
     * This should be called after releasing your own shared_ptr:
     * @code
     * uint16_t port = m_listener->getPort();
     * m_listener.reset();  // Release our reference first
     * UCXListenerRegistry::tryRemoveListener(port);  // Remove if we were the last
     * @endcode
     *
     * @param[in] port Port number of listener to potentially remove
     *
     * @return true if listener was removed, false if still in use or not found
     */
    static bool tryRemoveListener(uint16_t port);

private:
    /** @brief Map of port numbers to listener instances. */
    static std::unordered_map<uint16_t, std::shared_ptr<UCXListener>> g_listeners;

    /** @brief Mutex for protecting access to the listeners map. */
    static std::mutex g_registryMutex;

    /** @brief Deleted constructor to prevent instantiation. */
    UCXListenerRegistry() = delete;
};

} // namespace isaacsim::ucx::core
