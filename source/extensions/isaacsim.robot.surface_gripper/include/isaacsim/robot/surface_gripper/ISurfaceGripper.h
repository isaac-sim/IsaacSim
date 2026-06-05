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

#include <carb/Defines.h>
#include <carb/Types.h>

#include <usdrt/gf/matrix.h>

#include <string>
#include <vector>

namespace isaacsim
{
namespace robot
{
namespace surface_gripper
{

/**
 * @struct SurfaceGripperInterface
 * @brief Interface for controlling surface gripper functionality.
 * @details
 * Provides function pointers for controlling surface grippers in the simulation.
 * Surface grippers can attach to and manipulate objects through surface contact
 * rather than traditional mechanical gripping.
 *
 * All functions operate on grippers identified by their USD prim path.
 */
struct SurfaceGripperInterface
{
    CARB_PLUGIN_INTERFACE("isaacsim::robot::surface_gripper::ISurfaceGripper", 0, 1);

    /**
     * @brief Gets the current status of a surface gripper.
     * @details
     * Returns the status code indicating whether the gripper is open, closed, or closing.
     *
     * @param[in] primPath USD path of the gripper to query
     * @return Status code: 0 for Open, 1 for Closed, 2 for Closing, -1 if not found
     */
    int(CARB_ABI* getGripperStatus)(const char* primPath);

    /**
     * @brief Opens/releases a surface gripper.
     * @details
     * Commands the specified gripper to release any held objects and return to the open state.
     *
     * @param[in] primPath USD path of the gripper to open
     * @return True if the command was successful, false otherwise
     */
    bool(CARB_ABI* openGripper)(const char* primPath);

    /**
     * @brief Closes/activates a surface gripper.
     * @details
     * Commands the specified gripper to attempt to grip objects in contact with its surface.
     *
     * @param[in] primPath USD path of the gripper to close
     * @return True if the command was successful, false otherwise
     */
    bool(CARB_ABI* closeGripper)(const char* primPath);

    /**
     * @brief Sets a specific gripper action value.
     * @details
     * Sets the gripper action based on a continuous value. Values less than -0.3
     * will open the gripper, values greater than 0.3 will close the gripper,
     * and values in between have no effect.
     *
     * @param[in] primPath USD path of the gripper to control
     * @param[in] action Action value, typically in range [-1.0, 1.0]
     * @return True if the command was successful, false otherwise
     */
    bool(CARB_ABI* setGripperAction)(const char* primPath, const float action);

    /**
     * @brief Gets the list of objects currently gripped.
     * @details
     * Returns the USD paths of all objects that are currently held by the specified gripper.
     *
     * @param[in] primPath USD path of the gripper to query
     * @return Vector of USD paths for all gripped objects, empty vector if none or gripper not found
     */
    std::vector<std::string>(CARB_ABI* getGrippedObjects)(const char* primPath);

    /**
     * @brief Sets whether to write gripper state to USD.
     * @details
     * Controls whether gripper state changes are persisted to the USD stage
     * or maintained only in memory for improved performance.
     *
     * @param[in] writeToUsd True to write state to USD, false to keep in memory only
     * @return True if the setting was applied successfully, false otherwise
     */
    bool(CARB_ABI* setWriteToUsd)(const bool writeToUsd);

    /**
     * @brief Gets statuses for multiple surface grippers in parallel.
     * @details
     * Batch operation that queries the status of multiple grippers efficiently
     * by processing them in parallel.
     *
     * @param[in] primPaths Array of USD paths for grippers to query
     * @param[in] count Number of grippers in the array
     * @return Vector of status codes corresponding to each gripper path
     */
    std::vector<int>(CARB_ABI* getGripperStatusBatch)(const char* const* primPaths, size_t count);

    /**
     * @brief Opens multiple surface grippers in parallel.
     * @details
     * Batch operation that opens multiple grippers efficiently by processing them in parallel.
     *
     * @param[in] primPaths Array of USD paths for grippers to open
     * @param[in] count Number of grippers in the array
     * @return Vector of success flags corresponding to each gripper path
     */
    std::vector<bool>(CARB_ABI* openGripperBatch)(const char* const* primPaths, size_t count);

    /**
     * @brief Closes multiple surface grippers in parallel.
     * @details
     * Batch operation that closes multiple grippers efficiently by processing them in parallel.
     *
     * @param[in] primPaths Array of USD paths for grippers to close
     * @param[in] count Number of grippers in the array
     * @return Vector of success flags corresponding to each gripper path
     */
    std::vector<bool>(CARB_ABI* closeGripperBatch)(const char* const* primPaths, size_t count);

    /**
     * @brief Sets actions for multiple surface grippers in parallel.
     * @details
     * Batch operation that sets gripper actions for multiple grippers efficiently
     * by processing them in parallel.
     *
     * @param[in] primPaths Array of USD paths for grippers to control
     * @param[in] actions Array of action values corresponding to each gripper
     * @param[in] count Number of grippers in the arrays
     * @return Vector of success flags corresponding to each gripper path
     */
    std::vector<bool>(CARB_ABI* setGripperActionBatch)(const char* const* primPaths, const float* actions, size_t count);

    /**
     * @brief Gets gripped objects for multiple surface grippers in parallel.
     * @details
     * Batch operation that retrieves the list of gripped objects for multiple grippers
     * efficiently by processing them in parallel.
     *
     * @param[in] primPaths Array of USD paths for grippers to query
     * @param[in] count Number of grippers in the array
     * @return Vector of vectors containing USD paths of gripped objects for each gripper
     */
    std::vector<std::vector<std::string>>(CARB_ABI* getGrippedObjectsBatch)(const char* const* primPaths, size_t count);
};

} // namespace surface_gripper
} // namespace robot
} // namespace isaacsim
