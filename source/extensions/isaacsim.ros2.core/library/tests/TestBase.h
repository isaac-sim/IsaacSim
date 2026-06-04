// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/BindingsUtils.h>
#include <carb/tokens/ITokens.h>
#include <carb/tokens/TokensUtils.h>

#include <doctest/doctest.h>
#if defined(_WIN32)
#    include <filesystem>
#else
#    include <experimental/filesystem>
#endif
#include <isaacsim/core/includes/LibraryLoader.h>
#include <isaacsim/ros2/core/IRos2Core.h>
#include <isaacsim/ros2/core/Ros2Factory.h>
#include <isaacsim/ros2/core/Ros2QoS.h>
#include <isaacsim/ros2/core/Ros2Types.h>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

// Platform-specific includes for memory tracking
#ifdef _WIN32
#    include <psapi.h>
#    include <windows.h>
#elif defined(__linux__)
#    include <unistd.h>
#endif

/**
 * @brief Base test fixture class that provides common setup and utilities for ROS2 bridge tests
 *
 * This class reduces boilerplate code by providing:
 * - Factory loading with error handling
 * - Context and node creation helpers
 * - Default QoS profile creation
 * - Standard skip-if-no-factory pattern
 */
class Ros2TestBase
{
public:
    /**
     * @brief Constructor that loads the ROS2 factory for the specified distribution
     * @param distro ROS2 distribution name (default: value of ROS_DISTRO environment variable,
     *               falling back to "humble" when unset).  Using the sourced distro avoids
     *               loading an ABI-incompatible backend on top of the ROS 2 runtime libraries
     *               already loaded by the core plugin.
     */
    explicit Ros2TestBase(const std::string& distro = "")
        : m_distro(distro), m_factory(nullptr), m_context(nullptr), m_node(nullptr)
    {
        if (m_distro.empty())
        {
            const char* envDistro = std::getenv("ROS_DISTRO");
            m_distro = (envDistro && *envDistro) ? std::string(envDistro) : std::string("humble");
        }
        m_factory = loadRos2Factory(m_distro);
    }

    /**
     * @brief Helper function to load a ROS2 bridge factory for a specific distro
     *
     * @param distro The ROS2 distribution to load (e.g., "humble")
     * @return std::shared_ptr<isaacsim::ros2::core::Ros2Factory> Pointer to the factory or nullptr if loading failed
     */
    static std::shared_ptr<isaacsim::ros2::core::Ros2Factory> loadRos2Factory(const std::string& distro)
    {
        // Get the extension path
        carb::tokens::ITokens* tokens = carb::getCachedInterface<carb::tokens::ITokens>();
#if defined(_WIN32)
        std::filesystem::path p = carb::tokens::resolveString(tokens, "${app}");
#else
        std::experimental::filesystem::path p = carb::tokens::resolveString(tokens, "${app}");
#endif

        // Create a library loader for the factory
        auto factoryLoader = std::make_shared<isaacsim::core::includes::LibraryLoader>("isaacsim.ros2.core." + distro);

        if (!factoryLoader->isValid())
        {
            printf("Factory loader is not valid for distro: %s\n", distro.c_str());
            return nullptr;
        }

        // Get the createFactory function from the library
        using CreateFactoryBinding = isaacsim::ros2::core::Ros2Factory* (*)(void);
        CreateFactoryBinding createFactory = factoryLoader->getSymbol<CreateFactoryBinding>("createFactoryC");

        if (!createFactory)
        {
            printf("Could not find createFactory symbol in library\n");
            return nullptr;
        }

        // Create the factory and return it as a shared pointer
        isaacsim::ros2::core::Ros2Factory* factory = createFactory();
        if (!factory)
        {
            printf("createFactory returned nullptr\n");
            return nullptr;
        }

        return std::shared_ptr<isaacsim::ros2::core::Ros2Factory>(factory);
    }

    /**
     * @brief Destructor that cleans up resources
     */
    virtual ~Ros2TestBase()
    {
        shutdown();
    }

    /**
     * @brief Check if the factory was loaded successfully
     * @return true if factory is available, false otherwise
     */
    bool isFactoryAvailable() const
    {
        return m_factory != nullptr;
    }

    /**
     * @brief Get the ROS2 factory instance
     * @return Shared pointer to the factory, or nullptr if not available
     */
    std::shared_ptr<isaacsim::ros2::core::Ros2Factory> getFactory() const
    {
        return m_factory;
    }

    /**
     * @brief Skip test if factory is not available (use this in test cases)
     * @return true if test should be skipped, false if it can proceed
     */
    bool skipIfNoFactory() const
    {
        if (!m_factory)
        {
            MESSAGE("Could not load ROS2 factory for " << m_distro << " distro, skipping test");
            return true;
        }
        return false;
    }

    /**
     * @brief Create and initialize a ROS2 context
     * @return true if successful, false otherwise
     */
    bool createContext()
    {
        if (!m_factory)
        {
            return false;
        }

        m_context = m_factory->createContextHandle();
        if (!m_context)
        {
            return false;
        }

        m_context->init(0, nullptr);
        return m_context->isValid();
    }

    /**
     * @brief Create a ROS2 node handle
     * @param nodeName Name of the node
     * @param nodeNamespace Namespace for the node (default: "test")
     * @return true if successful, false otherwise
     */
    bool createNode(const std::string& nodeName, const std::string& nodeNamespace = "test")
    {
        if (!m_factory || !m_context)
        {
            return false;
        }

        m_node = m_factory->createNodeHandle(nodeName.c_str(), nodeNamespace.c_str(), m_context.get());
        return m_node != nullptr;
    }

    /**
     * @brief Create context and node in one call for convenience
     * @param nodeName Name of the node
     * @param nodeNamespace Namespace for the node (default: "test")
     * @return true if both context and node were created successfully
     */
    bool setupContextAndNode(const std::string& nodeName, const std::string& nodeNamespace = "test")
    {
        return createContext() && createNode(nodeName, nodeNamespace);
    }

    /**
     * @brief Get the current context handle
     * @return Shared pointer to context, or nullptr if not created
     */
    std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle> getContext() const
    {
        return m_context;
    }

    /**
     * @brief Get the current node handle
     * @return Shared pointer to node, or nullptr if not created
     */
    std::shared_ptr<isaacsim::ros2::core::Ros2NodeHandle> getNode() const
    {
        return m_node;
    }

    /**
     * @brief Create a default QoS profile for testing
     * @return Default QoS profile with reliable delivery, volatile durability, depth 10
     */
    static isaacsim::ros2::core::Ros2QoSProfile createDefaultQoS()
    {
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 10;
        return qos;
    }

    /**
     * @brief Create a best-effort QoS profile for testing
     * @return Best-effort QoS profile suitable for sensor data
     */
    static isaacsim::ros2::core::Ros2QoSProfile createBestEffortQoS()
    {
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eBestEffort;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eVolatile;
        qos.depth = 5;
        return qos;
    }

    /**
     * @brief Create a transient local QoS profile for testing
     * @return Transient local QoS profile suitable for configuration data
     */
    static isaacsim::ros2::core::Ros2QoSProfile createTransientLocalQoS()
    {
        isaacsim::ros2::core::Ros2QoSProfile qos;
        qos.reliability = isaacsim::ros2::core::Ros2QoSReliabilityPolicy::eReliable;
        qos.durability = isaacsim::ros2::core::Ros2QoSDurabilityPolicy::eTransientLocal;
        qos.depth = 10;
        return qos;
    }

    /**
     * @brief Shutdown the ROS2 context if active
     * @param reason Reason for shutdown (default: "test-cleanup")
     */
    void shutdown(const std::string& reason = "test-cleanup")
    {
        if (m_context && m_context->isValid())
        {
            m_context->shutdown(reason.c_str());
        }
        m_node.reset();
        m_context.reset();
    }

protected:
    std::string m_distro; ///< Active ROS 2 distribution name.
    std::shared_ptr<isaacsim::ros2::core::Ros2Factory> m_factory; ///< ROS 2 factory for creating contexts and nodes.
    std::shared_ptr<isaacsim::ros2::core::Ros2ContextHandle> m_context; ///< ROS 2 context handle for the test.
    std::shared_ptr<isaacsim::ros2::core::Ros2NodeHandle> m_node; ///< ROS 2 node handle for the test.
};

/**
 * @brief Macro to reduce boilerplate in test cases
 *
 * Usage in test cases:
 * TEST_CASE("MyTest") {
 *     ROS2_TEST_SETUP();
 *     // Your test code here using testBase.getFactory(), etc.
 * }
 */
#define ROS2_TEST_SETUP()                                                                                              \
    Ros2TestBase testBase;                                                                                             \
    if (testBase.skipIfNoFactory())                                                                                    \
    return

/**
 * @brief Macro for tests that need context and node
 *
 * Usage:
 * TEST_CASE("MyTestWithNode") {
 *     ROS2_TEST_SETUP_WITH_NODE("my_test_node");
 *     // Your test code here using testBase.getNode(), etc.
 * }
 */
#define ROS2_TEST_SETUP_WITH_NODE(nodeName)                                                                            \
    Ros2TestBase testBase;                                                                                             \
    if (testBase.skipIfNoFactory())                                                                                    \
        return;                                                                                                        \
    REQUIRE(testBase.setupContextAndNode(nodeName))
