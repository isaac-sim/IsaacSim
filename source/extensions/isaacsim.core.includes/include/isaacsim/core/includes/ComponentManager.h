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

#pragma once

#include "Component.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class ComponentManager
 * @brief Base class for managing USD-based components in an application
 * @details
 * ComponentManager provides the core interface for managing components within a USD stage.
 * It handles component lifecycle (creation, updates, deletion), stage management,
 * and application-level events. This class serves as the foundation for building
 * component-based applications that interact with USD stages.
 *
 * @note All pure virtual functions must be implemented by derived classes
 * @warning This class is not thread-safe by default; derived classes must implement
 *          their own thread safety mechanisms if needed
 */
class ComponentManager
{
public:
    /**
     * @brief Constructs a new ComponentManager instance
     */
    ComponentManager() = default;

    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    ~ComponentManager() = default;

    /**
     * @brief Initializes the manager with a USD stage
     * @details Sets up the stage reference for component management
     *
     * @param[in] stage Weak pointer to the USD stage to be managed
     * @post The manager is initialized with the provided stage
     */
    virtual void initialize(pxr::UsdStageWeakPtr stage)
    {
        m_stage = stage;
    }

    /**
     * @brief Updates the manager and all managed components
     * @details Pure virtual function that must be implemented by derived classes
     *          to define the update behavior of components
     *
     * @param[in] dt Time step in seconds since the last tick
     */
    virtual void tick(double dt) = 0;

    /**
     * @brief Initializes components from the current stage
     * @details Pure virtual function that must be implemented by derived classes
     *          to define how components are discovered and initialized from the stage
     */
    virtual void initComponents() = 0;

    /**
     * @brief Handles application start event
     * @details Optional callback that runs when the application starts
     *          Override this to implement custom start behavior
     */
    virtual void onStart()
    {
    }

    /**
     * @brief Handles application stop event
     * @details Optional callback that runs when the application stops
     *          Override this to implement custom stop behavior
     */
    virtual void onStop()
    {
    }

    /**
     * @brief Creates a new component for the given prim
     * @details Pure virtual function that must be implemented by derived classes
     *          to define component creation behavior
     *
     * @param[in] prim The USD prim to create a component for
     */
    virtual void onComponentAdd(const pxr::UsdPrim& prim) = 0;

    /**
     * @brief Updates a component when its corresponding prim changes
     * @details Pure virtual function that must be implemented by derived classes
     *          to define how components react to prim changes
     *
     * @param[in] prim The USD prim that changed
     */
    virtual void onComponentChange(const pxr::UsdPrim& prim) = 0;

    /**
     * @brief Removes a component and its associated resources
     * @details Pure virtual function that must be implemented by derived classes
     *          to define component cleanup behavior
     *
     * @param[in] primPath Path to the prim whose component should be removed
     */
    virtual void onComponentRemove(const pxr::SdfPath& primPath) = 0;

    /**
     * @brief Removes all components and performs cleanup
     * @details Pure virtual function that must be implemented by derived classes
     *          to define complete cleanup behavior
     */
    virtual void deleteAllComponents() = 0;

    /**
     * @brief Retrieves the managed USD stage
     * @return Weak pointer to the current USD stage
     */
    pxr::UsdStageWeakPtr getStage()
    {
        return m_stage;
    }

protected:
    /** @brief Weak pointer to the managed USD stage */
    pxr::UsdStageWeakPtr m_stage = nullptr;

    /** @brief Current simulation time in seconds */
    double m_timeSeconds = 0;

    /** @brief Current simulation time in nanoseconds */
    int64_t m_timeNanoSeconds = 0;

    /** @brief Time delta for current tick in seconds */
    double m_timeDelta = 0;
};
}
}
}
