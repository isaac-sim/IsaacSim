// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief ROS 2 Node definition and implementation
 * @details
 * This file contains the base class definition for ROS 2 nodes in the Isaac Sim core.
 * It provides core functionality for node initialization, namespace management,
 * and node lifecycle handling.
 */
#pragma once

#include "isaacsim/core/includes/BaseResetNode.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/EventsUtils.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/ros2/core/IRos2Core.h>
#include <isaacsim/ros2/core/Ros2Factory.h>
#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/usd/UsdContext.h>

namespace isaacsim
{
namespace ros2
{
namespace core
{

/**
 * @class Ros2Node
 * @brief Base class for ROS 2 core nodes
 * @details
 * This class provides the foundation for ROS 2 nodes in the Isaac Sim core.
 * It handles node lifecycle management, initialization, and automatic cleanup
 * of the internal ROS 2 node handle. Derived classes should implement specific
 * node functionality while relying on this base class for core ROS 2 operations.
 */
class Ros2Node : public isaacsim::core::includes::BaseResetNode
{

public:
    /**
     * @brief Constructor for the ROS 2 node
     * @details
     * Initializes the node by retrieving necessary interfaces:
     * - Core node framework interface
     * - ROS 2 core interface
     * - Settings interface for configuration
     * - ROS 2 factory for node creation
     *
     * The constructor also configures publishing verification settings
     * based on the extension configuration.
     */
    Ros2Node()
    {
        m_ros2Bridge = carb::getCachedInterface<isaacsim::ros2::core::Ros2Bridge>();

        m_settings = carb::getCachedInterface<carb::settings::ISettings>();
        static constexpr char s_kSetting[] = "/exts/isaacsim.ros2.bridge/publish_without_verification";
        m_publishWithoutVerification = m_settings->getAsBool(s_kSetting);

        // Here m_factory should be set according to the env var for ROS Distro..?
        m_factory = m_ros2Bridge->getFactory();
    }

    /**
     * @brief Destructor
     * @details Ensures proper cleanup by calling reset()
     */
    ~Ros2Node()
    {
        reset();
    }

    /**
     * @brief Resets the node handle
     * @details
     * Cleans up the ROS 2 node handle. Derived classes should call this method
     * after cleaning up their own publishers/subscribers.
     *
     * @note This is a virtual method that can be overridden by derived classes
     */
    virtual void reset()
    {
        if (m_nodeHandle)
        {
            // std::cout << "Calling RESET for this ROS2 Node..." << std::endl;
            // m_nodeHandle->getContextHandle()->shutdown();
            m_nodeHandle.reset();
        }
    }

    /**
     * @brief Initializes the ROS 2 node handle
     * @details
     * Creates and initializes the ROS 2 node handle with the specified name and namespace.
     * This method should be called before performing any operations with the node.
     * Node name changes are only applied after a Stop and Play cycle.
     *
     * @param[in] nodeName Name for the ROS 2 node
     * @param[in] namespaceName Namespace for the ROS 2 node
     * @param[in] contextHandleAddr Memory address of the context handler
     * @return bool True if initialization succeeded, false otherwise
     *
     * @note The method sanitizes node names and validates them before initialization
     */
    bool initializeNodeHandle(const std::string& nodeName, const std::string& namespaceName, uint64_t contextHandleAddr)
    {
        // Handle is initialized, so we should be ok, return true
        if (m_nodeHandle)
        {
            return true;
        }

        // Handle is not valid, try to initialize handle.
        // Make sure the ROS node name is valid
        std::string sanitizedNodeName = sanitizeName(nodeName);
        if (!m_factory->validateNodeName(sanitizedNodeName))
        {
            return false;
        }

        m_namespaceName = trimNonAlnum(namespaceName);
        if (!m_namespaceName.empty())
        {
            m_namespaceName = std::string("/") + m_namespaceName;
        }

        // Set the ROS 2 context if its available, if this is zero we use the default context
        if (contextHandleAddr)
        {
            // CARB_LOG_WARN("GET HANDLE %" PRIu64 "\n", contextHandleAddr);
            void* contextHandlePtr = m_ros2Bridge->getHandle(contextHandleAddr);
            if (contextHandlePtr == nullptr)
            {
                // CARB_LOG_WARN("CONTEXT DOES NOT EXIST");
                return false;
            }
            m_contextHandle = reinterpret_cast<std::shared_ptr<Ros2ContextHandle>*>(contextHandlePtr);
        }
        else
        {
            m_contextHandle =
                reinterpret_cast<std::shared_ptr<Ros2ContextHandle>*>(m_ros2Bridge->getDefaultContextHandleAddr());
        }

        // Create the ROS 2 node handler
        if (m_namespaceName.empty())
        {
            m_nodeHandle = m_factory->createNodeHandle(sanitizedNodeName.c_str(), "", m_contextHandle->get());
        }
        else if (m_factory->validateNamespaceName(m_namespaceName))
        {
            m_nodeHandle =
                m_factory->createNodeHandle(sanitizedNodeName.c_str(), m_namespaceName.c_str(), m_contextHandle->get());
        }
        else
        {
            return false;
        }
        return m_nodeHandle->getNode() != nullptr;
    }

    /**
     * @brief Checks if the node is initialized
     * @details Verifies if the node handle has been properly created and initialized
     *
     * @return bool True if the node handle exists, false otherwise
     */
    bool isInitialized() const
    {
        return m_nodeHandle != nullptr;
    }

    /**
     * @brief Adds a prefix to a topic name
     * @details
     * Prepends the specified prefix to a topic name, ensuring proper
     * separator character (/) placement between prefix and topic name.
     *
     * @param[in] prefix Prefix to add to the topic name
     * @param[in] topicName Original topic name
     * @return std::string The prefixed topic name
     *
     * @note Returns empty string if input topic name is empty
     */
    static inline std::string addTopicPrefix(const std::string& prefix, const std::string& topicName)
    {
        if (topicName.empty())
        {
            return std::string("");
        }
        return prefix + std::string("/") + trimNonAlnum(topicName);
    }

    /**
     * @brief Collects namespace information from USD stage
     * @details
     * Traverses up the USD stage hierarchy from the start prim, collecting
     * namespace information from isaac:namespace attributes. For non-TF nodes,
     * builds a hierarchical namespace. For TF nodes, only uses the highest-level
     * namespace.
     *
     * @param[in] namespaceInput Initial namespace (if not empty, skips collection)
     * @param[in] startPrim USD Prim to start namespace collection from
     * @param[in] tfNode Whether collecting namespace for a TF node
     * @return std::string Collected namespace string
     *
     * @note Returns input namespace if not empty
     */
    static inline std::string collectNamespace(const std::string& namespaceInput,
                                               const PXR_NS::UsdPrim& startPrim,
                                               const bool tfNode = false)
    {
        if (!namespaceInput.empty())
        {
            return namespaceInput;
        }
        std::string namespaceString = "";
        PXR_NS::UsdPrim currentPrim = startPrim;
        static const PXR_NS::TfToken s_kIsaacNamespace("isaac:namespace");

        std::string namespaceValue = "";

        // Traverse upwards until there are no more parents
        while (currentPrim.IsValid())
        {
            const pxr::UsdAttribute attr = currentPrim.GetAttribute(s_kIsaacNamespace);
            if (currentPrim.GetAttribute(s_kIsaacNamespace).HasValue())
            {
                if (attr.Get(&namespaceValue))
                {
                    if (!tfNode)
                    {
                        // Prepend the value to the accumulated string
                        if (!namespaceString.empty())
                        {
                            namespaceString = namespaceValue + "/" + namespaceString;
                        }
                        else
                        {
                            namespaceString = namespaceValue;
                        }
                    }
                    else
                    { // If collecting namespace for a TF node, only retrieve the highest level prim namespace
                        if (!namespaceValue.empty())
                        {
                            namespaceString = namespaceValue;
                        }
                    }
                }
            }

            // Move to the parent prim
            currentPrim = currentPrim.GetParent();
        }

        return namespaceString;
    }

private:
    /**
     * @brief Sanitizes a name for ROS 2 use
     * @details
     * Replaces all non-alphanumeric characters (except underscore)
     * with underscores to ensure ROS 2 naming compliance.
     *
     * @param[in] input String to sanitize
     * @return std::string Sanitized string
     */
    static inline std::string sanitizeName(std::string input)
    {
        std::replace_if(
            input.begin(), input.end(), [](auto ch) { return !(::isalnum(ch) || ch == '_'); }, '_');
        return input;
    }

    /**
     * @brief Trims non-alphanumeric characters from string
     * @details
     * Removes leading and trailing non-alphanumeric characters
     * from the input string.
     *
     * @param[in] s String to trim
     * @return std::string Trimmed string
     */
    static inline std::string trimNonAlnum(const std::string& s)
    {
        if (s.empty())
        {
            return "";
        }

        size_t startIdx = 0;
        size_t endIdx = s.size() - 1;

        while (startIdx < s.size() && !std::isalnum(s[startIdx]))
        {
            startIdx++;
        }

        while (endIdx > startIdx && !std::isalnum(s[endIdx]))
        {
            endIdx--;
        }

        return s.substr(startIdx, endIdx - startIdx + 1);
    }

protected:
    isaacsim::ros2::core::Ros2Bridge* m_ros2Bridge = nullptr; //!< \ref Ros2Bridge (carb) interface.
    std::shared_ptr<Ros2NodeHandle> m_nodeHandle = nullptr; //!< Node handler.
    carb::settings::ISettings* m_settings = nullptr; //!< Settings (carb) interface.
    bool m_publishWithoutVerification; //!< Whether to publish in a topic even if there are no subscription to it.
    std::shared_ptr<Ros2ContextHandle>* m_contextHandle; //!< Context handler.
    Ros2Factory* m_factory = nullptr; //!< Factory instance for creating ROS 2 related objects according to the sourced
                                      //!< ROS 2 distribution.
    std::string m_namespaceName; //!< Namespace name.
};

} // namespace core
} // namespace ros2
} // namespace isaacsim
