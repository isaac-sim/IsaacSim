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

#include <carb/PluginUtils.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <isaacsim/ros2/tf_viewer/Tf2FactoryImpl.h>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.h>

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @class Ros2BufferCoreImpl
 * @brief Implementation of ROS 2 Transform Buffer Core.
 * @details
 * Manages a transform buffer that stores and maintains the transform tree,
 * providing functionality to set and query transforms between different coordinate frames.
 */
class Ros2BufferCoreImpl : public Ros2BufferCore
{
public:
    /**
     * @brief Constructor for ROS 2 Buffer Core implementation.
     */
    Ros2BufferCoreImpl();

    /**
     * @brief Virtual destructor.
     */
    virtual ~Ros2BufferCoreImpl();

    /**
     * @brief Sets a transform in the buffer.
     * @param[in] msg Pointer to the transform message.
     * @param[in] authority Name of the authority setting the transform.
     * @param[in] isStatic Whether the transform is static (unchanging).
     * @return True if transform was successfully set.
     */
    virtual bool setTransform(void* msg, const std::string& authority, bool isStatic);

    /**
     * @brief Gets the transform between two frames.
     * @param[in] targetFrame Name of the target frame.
     * @param[in] sourceFrame Name of the source frame.
     * @param[out] translation Array to store the translation components [x, y, z].
     * @param[out] rotation Array to store the rotation components [x, y, z, w].
     * @return True if transform was successfully retrieved.
     */
    virtual bool getTransform(const std::string& targetFrame,
                              const std::string& sourceFrame,
                              double translation[],
                              double rotation[]);

    /**
     * @brief Gets the parent frame of a given frame.
     * @param[in] frame Name of the frame to query.
     * @param[out] parentFrame Name of the parent frame.
     * @return True if parent frame was successfully retrieved.
     */
    virtual bool getParentFrame(const std::string& frame, std::string& parentFrame);

    /**
     * @brief Gets a list of all frames in the buffer.
     * @return Vector of frame names.
     */
    virtual std::vector<std::string> getFrames();

    /**
     * @brief Clears all transforms from the buffer.
     */
    virtual void clear();

private:
    /** @brief The underlying TF2 buffer core */
    tf2::BufferCore m_buffer;

    /** @brief Cache of frame names in the buffer */
    std::vector<std::string> m_frames;
};

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim
