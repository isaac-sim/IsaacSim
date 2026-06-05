// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/**
 * @file
 * @brief Interface to access the ROS 2 factory and context handler for the sourced ROS 2 distribution.
 * @details
 * This header file defines the main interface for the ROS 2 core, providing access to factory
 * and context handling functionality for ROS 2 integration within Isaac Sim.
 */
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace core
{

class Ros2Factory;

/**
 * @class Ros2Bridge
 * @brief Main interface class for the ROS 2 core functionality.
 * @details
 * The Ros2Bridge class provides the core interface for interacting with ROS 2 functionality
 * within Isaac Sim. It manages the lifecycle of ROS 2 context handlers and provides factory
 * access for creating ROS 2 related objects.
 */
struct Ros2Bridge
{
    /**
     * @brief Plugin interface definition for the ROS 2 core.
     * @details Defines the plugin interface with version information.
     */
    CARB_PLUGIN_INTERFACE("isaacsim::ros2::core::Ros2Bridge", 1, 0);

    /**
     * @brief Retrieves the memory address of the ROS 2 context handler.
     * @details
     * The Ros2ContextHandle object encapsulates a `rcl_context_t` (non-global state of an init/shutdown cycle)
     * instance used in the creation of ROS 2 nodes and other entities.
     *
     * @return Memory address of the context handler as a 64-bit unsigned integer.
     *
     * @note This address points to a shared pointer object of type Ros2ContextHandle.
     * @warning Do not attempt to dereference this address directly without proper casting to the correct type.
     */
    uint64_t const(CARB_ABI* getDefaultContextHandleAddr)();

    /**
     * @brief Retrieves the factory instance for creating ROS 2 objects.
     * @details
     * Returns a factory instance that can create various ROS 2 related functions and objects
     * according to the sourced ROS 2 distribution. The factory provides a centralized way
     * to instantiate ROS 2 components.
     *
     * @return Pointer to the factory instance.
     *
     * @note The returned pointer is owned by the bridge implementation and should not be deleted.
     * @see Ros2Factory for the complete list of available factory methods.
     */
    Ros2Factory* const(CARB_ABI* getFactory)();

    /**
     * @brief Checks the initialization status of the bridge.
     * @details
     * Verifies if both the factory (Ros2Factory) and the handler (Ros2ContextHandle) objects
     * have been properly instantiated. These objects are created when the Ros2Bridge interface
     * is first acquired after the plugin is loaded by the isaacsim.ros2.core extension.
     *
     * @return True if both factory and context handler are successfully instantiated,
     *         false otherwise.
     *
     * @note This method should be called before attempting to use any other methods of this interface.
     * @warning Using other methods when this returns false may result in undefined behavior.
     */
    bool const(CARB_ABI* getStartupStatus)();

    /**
     * @brief Registers a handle within the ROS 2 handle registry.
     * @details Stores the provided handle pointer and returns an identifier that can be used to retrieve it later.
     *
     * @param[in] handle Pointer to the handle that needs to be tracked.
     * @return Unique identifier for the stored handle.
     */
    uint64_t const(CARB_ABI* addHandle)(void* handle);

    /**
     * @brief Retrieves a registered handle from the registry.
     * @details Looks up the handle associated with the provided identifier.
     *
     * @param[in] handleId Identifier returned by addHandle.
     * @return Pointer to the registered handle, or nullptr if it does not exist.
     */
    void* const(CARB_ABI* getHandle)(const uint64_t handleId);

    /**
     * @brief Removes a registered handle from the registry.
     * @details Erases the handle tracked by the given identifier.
     *
     * @param[in] handleId Identifier of the handle to remove.
     * @return True if the handle existed and was removed, false otherwise.
     */
    bool const(CARB_ABI* removeHandle)(const uint64_t handleId);
};

} // namespace core
} // namespace ros2
} // namespace isaacsim
