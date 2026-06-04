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

#include <memory>
#include <string>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @class Ros2BufferCore
 * @brief Class that partially implements a tf2 `BufferCore`.
 * @details
 * This abstract class provides an interface for tf2 buffer operations such as
 * setting transforms, retrieving transforms, and querying frame information.
 * It serves as a compatibility layer for working with ROS 2 tf2 functionality.
 */
class Ros2BufferCore
{
public:
    /**
     * @brief Adds a transform information to the buffer.
     * @details
     * This function takes a ROS TFMessage and stores the transform information
     * in the buffer for later retrieval.
     *
     * @param[in] msg `TFMessage` message pointer.
     * @param[in] authority The source of the information for the transform.
     * @param[in] isStatic Whether to record the transform as static (the transform will be good across all time).
     * @return Whether the transform has been added successfully.
     */
    virtual bool setTransform(void* msg, const std::string& authority, bool isStatic) = 0;

    /**
     * @brief Gets the transform between two frames at the ROS zero time.
     * @details
     * Calculates and retrieves the coordinate transformation between two frames
     * in the tf tree at the ROS zero time.
     *
     * @param[in] targetFrame Frame ID to which data should be transformed.
     * @param[in] sourceFrame Frame ID where the data originated.
     * @param[out] translation Buffer to store the Cartesian translation. Buffer length should be 3.
     * @param[out] rotation Buffer to store the rotation (as quaternion: xyzw). Buffer length should be 4.
     * @return Whether the transform has been computed without exceptions.
     */
    virtual bool getTransform(const std::string& targetFrame,
                              const std::string& sourceFrame,
                              double translation[],
                              double rotation[]) = 0;

    /**
     * @brief Gets the parent frame of the specified frame at the ROS zero time.
     * @details
     * Retrieves the parent frame ID for a given frame in the tf tree.
     *
     * @param[in] frame Frame ID to search for.
     * @param[out] parentFrame Reference in which the parent frame ID will be stored.
     * @return True if a parent exists, false otherwise.
     */
    virtual bool getParentFrame(const std::string& frame, std::string& parentFrame) = 0;

    /**
     * @brief Gets the available frame IDs.
     * @details
     * Retrieves a list of all frame IDs currently available in the tf tree.
     *
     * @return A list of available frame IDs.
     */
    virtual std::vector<std::string> getFrames() = 0;

    /**
     * @brief Clears all buffer data.
     * @details
     * Removes all transformations and frames from the buffer.
     */
    virtual void clear() = 0;
};


/**
 * @class Tf2Factory
 * @brief Base class for creating ROS 2 tf2 related functions/objects according to the sourced ROS 2 distribution.
 * @details
 * This abstract factory class provides an interface for creating tf2 related objects
 * that are compatible with the currently loaded ROS 2 distribution.
 */
class Tf2Factory
{
public:
    /**
     * @brief Virtual destructor with default implementation.
     */
    virtual ~Tf2Factory() = default;

    /**
     * @brief Creates a ROS 2 tf2 `BufferCore`.
     * @details
     * Factory method to instantiate a new buffer appropriate for the ROS 2 distribution.
     *
     * @return Pointer to the newly created buffer.
     */
    virtual std::shared_ptr<Ros2BufferCore> createBuffer() = 0;
};

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim
