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

#pragma once

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/eventdispatcher/IEventDispatcher.h>

#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/timeline/TimelineTypes.h>
#include <omni/usd/UsdContext.h>

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class BaseResetNode
 * @brief Base class for nodes that automatically reset their state when simulation is stopped.
 * @details
 * This class provides automatic reset functionality for nodes in the simulation graph.
 * It subscribes to timeline events via IEventDispatcher and triggers a reset when a stop event is received.
 * Derived classes must implement the reset() function to define their specific reset behavior.
 *
 * The class handles:
 * - Timeline event subscription management
 * - Automatic cleanup of subscriptions
 * - Reset triggering on simulation stop
 *
 * @note This class uses RAII principles to manage timeline event subscriptions
 * @warning Derived classes must implement reset() or a compilation error will occur
 */
class BaseResetNode
{
public:
    /**
     * @brief Constructs a new BaseResetNode instance.
     * @details
     * Sets up the timeline event subscription to automatically trigger reset on simulation stop.
     * The subscription is created with the following characteristics:
     * - Listens for stop events via IEventDispatcher
     * - Automatically triggers reset() when stop occurs
     * - Uses a unique handler name for identification
     *
     * @post Timeline event subscription is created and active
     */
    BaseResetNode()
    {
        // When the node is created, we create a stage event subscription
        // The idea is that node is reset whenever stop is pressed
        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        m_timelineEventSub = ed->observeEvent(carb::RStringKey("IsaacSimOGNTimelineEventHandler"),
                                              carb::eventdispatcher::kDefaultOrder, omni::timeline::kGlobalEventStop,
                                              [this](const carb::eventdispatcher::Event&) { reset(); });
    }

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     * @details
     * Cleans up the timeline event subscription using RAII principles.
     * The subscription is automatically released when the object is destroyed.
     */
    virtual ~BaseResetNode() = default;

    /**
     * @brief Pure virtual function to reset the node's state.
     * @details
     * This function is called automatically when the simulation is stopped.
     * Derived classes must implement this function to define their specific reset behavior.
     *
     * @note This function is called from the timeline event handler thread
     * @warning Implementation must be thread-safe as it may be called asynchronously
     */
    virtual void reset() = 0;

private:
    /** @brief Subscription handle for timeline stop events */
    carb::eventdispatcher::ObserverGuard m_timelineEventSub;
};

} // namespace includes
} // namespace core
} // namespace isaacsim
