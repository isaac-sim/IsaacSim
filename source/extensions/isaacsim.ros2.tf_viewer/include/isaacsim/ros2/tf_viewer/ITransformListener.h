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
#include <carb/Interface.h>

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @class ITransformListener
 * @brief ROS 2 transform listener interface
 * @details
 * Interface for a listener that subscribes to ROS 2 transform messages (tf)
 * and provides methods to retrieve and manipulate transformation data.
 * This interface allows for integration with the ROS 2 tf2 system.
 */
class ITransformListener
{
public:
    CARB_PLUGIN_INTERFACE("isaacsim::ros2::tf_viewer::ITransformListener", 1, 0);

    /**
     * @brief Initializes the transform listener by loading the adequate ROS 2 plugin and creating the tf2 buffer.
     * @details
     * This method loads the appropriate ROS 2 plugin based on the specified distribution
     * and sets up the necessary tf2 buffer for transform handling.
     *
     * @param[in] rosDistro The ROS 2 distribution name to use for initialization.
     * @return Whether the transform listener was initialized successfully.
     */
    virtual bool initialize(const std::string& rosDistro) = 0;

    /**
     * @brief Finalizes subscriptions to the tf (if any) and resets ROS 2 node.
     * @details
     * Cleans up resources and disconnects from the ROS 2 system.
     */
    virtual void finalize() = 0;

    /**
     * @brief Processes transform listener to record incoming tf transforms.
     * @details
     * This method should be called regularly to process incoming transform messages
     * and update the internal buffer.
     *
     * @return True if the tf messages have been processed without error, false otherwise.
     */
    virtual bool spin() = 0;

    /**
     * @brief Resets the internal buffer by clearing its data.
     * @details
     * Removes all stored transforms from the internal buffer.
     */
    virtual void reset() = 0;

    /**
     * @brief Computes all transforms with respect to the specified (root) frame.
     * @details
     * Calculates the transformation data for all available frames relative to
     * the specified root frame. This data can then be retrieved using other methods.
     *
     * @param[in] rootFrame Frame ID on which to compute transforms.
     */
    virtual void computeTransforms(const std::string& rootFrame) = 0;

    /**
     * @brief Gets the available frame IDs.
     * @details
     * Retrieves the list of all available frame IDs in the tf tree.
     * It is necessary to call the computeTransforms method first before invoking
     * this one to obtain updated data.
     *
     * @return A list of available frame IDs.
     */
    virtual const std::vector<std::string>& getFrames() = 0;

    /**
     * @brief Gets the relations between available frame IDs.
     * @details
     * Retrieves the parent-child relationships between frames in the tf tree.
     * It is necessary to call the computeTransforms method first before invoking
     * this one to obtain updated data.
     *
     * @return A list of relation-pairs between frame IDs (parent-child tuples).
     */
    virtual const std::vector<std::tuple<std::string, std::string>>& getRelations() = 0;

    /**
     * @brief Gets the transforms of the available frame IDs with respect to the specified (root) frame.
     * @details
     * Retrieves the transformation data (translation and rotation) for each frame
     * relative to the root frame specified in the last call to computeTransforms.
     * It is necessary to call the computeTransforms method first before invoking
     * this one to obtain updated data.
     *
     * @return A map of transforms (translation: xyz, rotation: xyzw) for each frame ID.
     */
    virtual const std::unordered_map<std::string,
                                     std::tuple<std::tuple<double, double, double>, std::tuple<double, double, double, double>>>&
    getTransforms() = 0;
};

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim
