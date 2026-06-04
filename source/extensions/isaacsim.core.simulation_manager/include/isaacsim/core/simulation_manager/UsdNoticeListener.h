// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <physxSchema/physxSceneAPI.h>
#include <pxr/base/tf/notice.h>
#include <pxr/pxr.h>

#include <functional>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

/**
 * @class UsdNoticeListener
 * @brief Listener class for USD object change notifications.
 * @details
 * This class listens for changes to USD objects and manages callbacks for deletion
 * and physics scene addition events. It inherits from pxr::TfWeakBase to support
 * the USD notification system.
 */
class UsdNoticeListener : public pxr::TfWeakBase
{
public:
    /**
     * @brief Default constructor.
     * @details
     * Initializes the callback iterator to 0 and enables the listener by default.
     */
    UsdNoticeListener();

    /**
     * @brief Handles USD object change notifications.
     * @param[in] objectsChanged The notification containing information about changed objects.
     */
    void handle(const pxr::UsdNotice::ObjectsChanged& objectsChanged);

    /**
     * @brief Enables or disables the listener.
     * @param[in] flag True to enable the listener, false to disable.
     */
    void enable(const bool& flag);

    /**
     * @brief Checks if the listener is enabled.
     * @return True if the listener is enabled, false otherwise.
     */
    bool isEnabled();

    /**
     * @brief Gets the map of deletion callbacks.
     * @return Reference to the map of deletion callbacks, keyed by callback ID.
     */
    std::map<int, std::function<void(const std::string&)>>& getDeletionCallbacks();

    /**
     * @brief Gets the map of physics scene addition callbacks.
     * @return Reference to the map of physics scene addition callbacks, keyed by callback ID.
     */
    std::map<int, std::function<void(const std::string&)>>& getPhysicsSceneAdditionCallbacks();

    /**
     * @brief Gets the map of physics scenes.
     * @return Reference to the map of physics scenes, keyed by their USD paths.
     */
    std::map<pxr::SdfPath, pxr::PhysxSchemaPhysxSceneAPI>& getPhysicsScenes();

    /**
     * @brief Gets the callback iteration counter.
     * @return Reference to the current callback iteration counter.
     */
    int& getCallbackIter();

private:
    /** @brief Map of physics scenes keyed by their USD paths */
    std::map<pxr::SdfPath, pxr::PhysxSchemaPhysxSceneAPI> m_physicsScenes;

    /** @brief Map of deletion callbacks keyed by callback ID */
    std::map<int, std::function<void(const std::string&)>> m_deletionCallbacks;

    /** @brief Map of physics scene addition callbacks keyed by callback ID */
    std::map<int, std::function<void(const std::string&)>> m_physicsSceneAdditionCallbacks;

    /** @brief Counter for generating unique callback IDs */
    int m_callbackIter;

    /** @brief Flag indicating whether the listener is enabled */
    bool m_enableFlag;
};

} // namespace simulation_manager
} // namespace core
} // namespace isaacsim
